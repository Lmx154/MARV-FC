use core::sync::atomic::{AtomicBool, Ordering};

use common::messages::runtime::FlightPhase;
use common::policies::modes::evaluate_init_hil_command;
use common::protocol::mavlink::{DecodeError, MavlinkHilEgressEncoder, MavlinkHilMessagePump};
use common::services::health::LivenessUpdate;
use common::services::hil::{
    HilByteWriter, HilEgressMessage, HilIngressMessage, HilIngressRoutes, HilRuntime,
    send_hil_egress_message,
};
use defmt::{info, warn};
use embassy_executor::{Spawner, task};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Instant, TimeoutError, with_timeout};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use crate::channels::{
    BAROMETER_CHANNEL, HIL_BOOT_SIGNAL, HIL_CONTROL_COMMAND_CHANNEL,
    PayloadFlightPhaseSubscriber, PayloadHilEgressReceiver, PayloadHilEgressSender,
    PayloadWatchdogLivenessSender, TIME_CHANNEL,
};
use crate::config::HilConfig;
use crate::watchdog;

const USB_PACKET_SIZE: u16 = 64;
const MAVLINK_STREAM_CAPACITY: usize = 4096;
const INIT_PROBE_POLL_MS: u64 = 100;
const USB_BANNER: &[u8] = b"payload-controller-rp2350\r\n";

type RpUsbDriver = Driver<'static, USB>;
type RpUsbDevice = UsbDevice<'static, RpUsbDriver>;
type RpUsbSender = Sender<'static, RpUsbDriver>;
type RpUsbReceiver = Receiver<'static, RpUsbDriver>;

struct UsbHilWriter {
    class: RpUsbSender,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
enum UsbHilMode {
    InitProbe,
    HilActive,
    Passive,
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR: StaticCell<[u8; 0]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_ACM_STATE: StaticCell<State<'static>> = StaticCell::new();
static USB_CDC_HOST_CONNECTED: AtomicBool = AtomicBool::new(false);

#[task]
async fn usb_device_task(mut usb: RpUsbDevice) -> ! {
    usb.run().await
}

impl HilByteWriter for UsbHilWriter {
    type Error = EndpointError;

    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.class.write_packet(bytes).await
    }
}

pub fn reset_host_connected() {
    USB_CDC_HOST_CONNECTED.store(false, Ordering::Relaxed);
}

pub fn host_connected() -> bool {
    USB_CDC_HOST_CONNECTED.load(Ordering::Relaxed)
}

fn set_host_connected(connected: bool) {
    USB_CDC_HOST_CONNECTED.store(connected, Ordering::Relaxed);
}

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

fn mode_for_phase(phase: FlightPhase) -> UsbHilMode {
    match phase {
        FlightPhase::Init => UsbHilMode::InitProbe,
        FlightPhase::Hil => UsbHilMode::HilActive,
        FlightPhase::Ready | FlightPhase::Active | FlightPhase::Fault => UsbHilMode::Passive,
    }
}

fn apply_phase_updates(
    phase_subscriber: &mut PayloadFlightPhaseSubscriber,
    mode: &mut UsbHilMode,
    protocol: &mut MavlinkHilMessagePump<MAVLINK_STREAM_CAPACITY>,
    runtime: &mut HilRuntime,
) {
    while let Some(phase) = phase_subscriber.try_next_message_pure() {
        let next_mode = mode_for_phase(phase);
        if next_mode == *mode {
            continue;
        }

        *mode = next_mode;
        protocol.reset();
        runtime.reset();
        info!("usb cdc mode -> {:?}", next_mode);
    }
}

fn report_init_probe_liveness(sender: &PayloadWatchdogLivenessSender, mode: UsbHilMode) {
    if matches!(mode, UsbHilMode::InitProbe) {
        let _ = sender.try_send(LivenessUpdate::new(watchdog::SOURCE_USB_PROBE, now_ms()));
    }
}

fn handle_control_command(
    command: common::services::hil::HilControlCommand,
    hil_config: HilConfig,
    mode: UsbHilMode,
    boot_started_at: Instant,
    egress: &PayloadHilEgressSender,
) {
    let elapsed_ms = match mode {
        UsbHilMode::InitProbe if host_connected() => 0,
        UsbHilMode::InitProbe => boot_started_at.elapsed().as_millis() as u32,
        UsbHilMode::HilActive | UsbHilMode::Passive => hil_config.boot_window_ms.saturating_add(1),
    };
    let decision = evaluate_init_hil_command(
        command,
        hil_config.system_id,
        hil_config.component_id,
        elapsed_ms,
        hil_config.boot_window_ms,
    );

    if let Some(ack) = decision.into_ack(command) {
        let _ = egress.try_send(HilEgressMessage::CommandAck(ack));
    }
    if decision.enter_hil {
        HIL_BOOT_SIGNAL.signal(());
    }
}

#[task]
async fn usb_cdc_ingest_task(
    mut class: RpUsbReceiver,
    hil_config: HilConfig,
    mut phase_subscriber: PayloadFlightPhaseSubscriber,
    egress: PayloadHilEgressSender,
    liveness: PayloadWatchdogLivenessSender,
) -> ! {
    let mut packet = [0u8; USB_PACKET_SIZE as usize];
    let routes = HilIngressRoutes::new(
        &TIME_CHANNEL,
        &(),
        &BAROMETER_CHANNEL,
        &(),
        &(),
        &HIL_CONTROL_COMMAND_CHANNEL,
    );
    let mut protocol = MavlinkHilMessagePump::<MAVLINK_STREAM_CAPACITY>::new();
    let mut runtime = HilRuntime::new();
    let boot_started_at = Instant::now();
    let mut mode = UsbHilMode::InitProbe;

    loop {
        apply_phase_updates(&mut phase_subscriber, &mut mode, &mut protocol, &mut runtime);
        match with_timeout(
            Duration::from_millis(INIT_PROBE_POLL_MS),
            class.wait_connection(),
        )
        .await
        {
            Ok(()) => {
                set_host_connected(true);
                info!("usb cdc host connected");
                break;
            }
            Err(TimeoutError) => report_init_probe_liveness(&liveness, mode),
        }
    }

    loop {
        apply_phase_updates(&mut phase_subscriber, &mut mode, &mut protocol, &mut runtime);

        match with_timeout(
            Duration::from_millis(INIT_PROBE_POLL_MS),
            class.read_packet(&mut packet),
        )
        .await
        {
            Ok(Ok(len)) => {
                if len == 0 {
                    continue;
                }

                protocol.ingest_bytes(&packet[..len]);
                while let Some(message) = protocol.try_next_message() {
                    match message {
                        Ok(HilIngressMessage::ControlCommand(command)) => {
                            if matches!(mode, UsbHilMode::HilActive) {
                                let _ = HIL_CONTROL_COMMAND_CHANNEL.try_send(command);
                            } else {
                                handle_control_command(
                                    command,
                                    hil_config,
                                    mode,
                                    boot_started_at,
                                    &egress,
                                );
                            }
                        }
                        Ok(message) if matches!(mode, UsbHilMode::HilActive) => {
                            let dispatch = runtime.accept(message, &routes);
                            if dispatch.tick.is_some() {
                                let _ = liveness.try_send(LivenessUpdate::new(
                                    watchdog::SOURCE_HIL_TIME,
                                    now_ms(),
                                ));
                            }
                        }
                        Ok(_) => {}
                        Err(error) => match error {
                            DecodeError::UnsupportedMessage { .. } => {}
                            _ => warn!("usb cdc decode error"),
                        },
                    }
                }
            }
            Ok(Err(_)) => {
                set_host_connected(false);
                warn!("usb cdc host disconnected");
                protocol.reset();
                runtime.reset();

                loop {
                    apply_phase_updates(&mut phase_subscriber, &mut mode, &mut protocol, &mut runtime);
                    match with_timeout(
                        Duration::from_millis(INIT_PROBE_POLL_MS),
                        class.wait_connection(),
                        )
                        .await
                    {
                        Ok(()) => {
                            set_host_connected(true);
                            info!("usb cdc host connected");
                            break;
                        }
                        Err(TimeoutError) => report_init_probe_liveness(&liveness, mode),
                    }
                }
            }
            Err(TimeoutError) => report_init_probe_liveness(&liveness, mode),
        }
    }
}

#[task]
async fn usb_cdc_egress_task(
    class: RpUsbSender,
    hil_config: HilConfig,
    receiver: PayloadHilEgressReceiver,
) -> ! {
    let mut writer = UsbHilWriter { class };
    let receiver = receiver;
    let mut encoder = MavlinkHilEgressEncoder::new(hil_config.system_id, hil_config.component_id);

    loop {
        writer.class.wait_connection().await;
        let _ = writer.class.write_packet(USB_BANNER).await;

        loop {
            let message = receiver.receive().await;
            if send_hil_egress_message(&mut writer, &mut encoder, message)
                .await
                .is_err()
            {
                warn!("usb cdc HIL egress write failed");
                break;
            }
        }
    }
}

pub fn spawn(
    spawner: &Spawner,
    usb: embassy_rp::Peri<'static, USB>,
    hil_config: HilConfig,
    receiver: PayloadHilEgressReceiver,
    phase_subscriber: PayloadFlightPhaseSubscriber,
    liveness: PayloadWatchdogLivenessSender,
) {
    let driver = Driver::new(usb, Irqs);

    let mut usb_config = Config::new(0x1209, 0x0001);
    usb_config.manufacturer = Some("MARV");
    usb_config.product = Some("Payload Controller");
    usb_config.serial_number = Some("PAYLOAD-CONTROLLER-RP2350");
    usb_config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        usb_config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        MSOS_DESCRIPTOR.init([0; 0]),
        CONTROL_BUF.init([0; 64]),
    );
    let state = CDC_ACM_STATE.init(State::new());
    let class = CdcAcmClass::new(&mut builder, state, USB_PACKET_SIZE);
    let (sender, receiver_class) = class.split();
    let usb = builder.build();

    spawner.spawn(usb_device_task(usb)).unwrap();
    spawner
        .spawn(usb_cdc_ingest_task(
            receiver_class,
            hil_config,
            phase_subscriber,
            crate::channels::HIL_EGRESS_CHANNEL.sender(),
            liveness,
        ))
        .unwrap();
    spawner
        .spawn(usb_cdc_egress_task(sender, hil_config, receiver))
        .unwrap();
}
