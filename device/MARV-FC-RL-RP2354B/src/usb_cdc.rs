use core::sync::atomic::{AtomicBool, Ordering};

use common::protocol::mavlink::{DecodeError, MavlinkHilEgressEncoder, MavlinkHilMessagePump};
use common::services::health::LivenessUpdate;
use common::services::hil::{
    HilByteWriter, HilEgressMessage, HilEgressTrySender, HilIngressMessage, HilIngressRoutes,
    HilRuntime, UsbHilMode, evaluate_usb_init_control_command, send_hil_egress_message,
    should_report_init_probe_liveness, usb_hil_mode_for_phase,
};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::{Duration, Instant, TimeoutError, with_timeout};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use crate::channels::{
    BAROMETER_CHANNEL, FcFlightPhaseSubscriber, FcHilControlCommandSender, FcHilEgressReceiver,
    FcHilEgressSender, FcHilSessionStateSubscriber, FcWatchdogLivenessSender, GPS_CHANNEL,
    HIL_BOOT_SIGNAL, IMU_CHANNEL, TIME_CHANNEL,
};
use crate::config::HilConfig;
use crate::watchdog;

const USB_PACKET_SIZE: u16 = 64;
const MAVLINK_STREAM_CAPACITY: usize = 4096;
const INIT_PROBE_POLL_MS: u64 = 100;

type RpUsbDriver = Driver<'static, USB>;
type RpUsbDevice = UsbDevice<'static, RpUsbDriver>;
type RpUsbSender = Sender<'static, RpUsbDriver>;
type RpUsbReceiver = Receiver<'static, RpUsbDriver>;

struct UsbHilWriter {
    class: RpUsbSender,
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

#[embassy_executor::task]
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

fn apply_phase_updates(
    phase_subscriber: &mut FcFlightPhaseSubscriber,
    mode: &mut UsbHilMode,
    protocol: &mut MavlinkHilMessagePump<MAVLINK_STREAM_CAPACITY>,
    runtime: &mut HilRuntime,
) {
    while let Some(phase) = phase_subscriber.try_next_message_pure() {
        let next_mode = usb_hil_mode_for_phase(phase);
        if next_mode == *mode {
            continue;
        }

        *mode = next_mode;
        protocol.reset();
        runtime.reconcile_usb_hil_mode(next_mode);
        info!("usb cdc mode -> {:?}", next_mode);
    }
}

fn apply_session_state_updates(
    state_subscriber: &mut FcHilSessionStateSubscriber,
    runtime: &mut HilRuntime,
) {
    while let Some(state) = state_subscriber.try_next_message_pure() {
        runtime.set_session_state(state);
    }
}

fn report_init_probe_liveness(sender: &FcWatchdogLivenessSender, mode: UsbHilMode) {
    if should_report_init_probe_liveness(mode) {
        let _ = sender.try_send(LivenessUpdate::new(watchdog::SOURCE_USB_PROBE, now_ms()));
    }
}

async fn handle_control_command<E>(
    command: common::services::hil::HilControlCommand,
    hil_config: HilConfig,
    mode: UsbHilMode,
    boot_started_at: Instant,
    control_sender: &FcHilControlCommandSender,
    egress: &E,
) where
    E: HilEgressTrySender,
{
    let outcome = evaluate_usb_init_control_command(
        command,
        hil_config.system_id,
        hil_config.component_id,
        mode,
        host_connected(),
        boot_started_at.elapsed().as_millis() as u32,
        hil_config.boot_window_ms,
    );

    if let Some(ack) = outcome.ack {
        let _ = egress.try_send_hil_egress(HilEgressMessage::CommandAck(ack));
    }
    if outcome.enter_hil {
        if outcome.relay_to_control_runtime {
            control_sender.send(command).await;
        }
        HIL_BOOT_SIGNAL.signal(());
    }
}

#[embassy_executor::task]
async fn usb_cdc_ingest_task(
    mut class: RpUsbReceiver,
    hil_config: HilConfig,
    mut phase_subscriber: FcFlightPhaseSubscriber,
    mut state_subscriber: FcHilSessionStateSubscriber,
    control_sender: FcHilControlCommandSender,
    egress: FcHilEgressSender,
    liveness: FcWatchdogLivenessSender,
) -> ! {
    let mut packet = [0u8; USB_PACKET_SIZE as usize];
    let routes = HilIngressRoutes::new(
        &TIME_CHANNEL,
        &IMU_CHANNEL,
        &BAROMETER_CHANNEL,
        &GPS_CHANNEL,
        &(),
        &(),
    );
    let mut protocol = MavlinkHilMessagePump::<MAVLINK_STREAM_CAPACITY>::new();
    let mut runtime = HilRuntime::new();
    let boot_started_at = Instant::now();
    let mut mode = UsbHilMode::InitProbe;

    loop {
        apply_phase_updates(
            &mut phase_subscriber,
            &mut mode,
            &mut protocol,
            &mut runtime,
        );
        apply_session_state_updates(&mut state_subscriber, &mut runtime);
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
            Err(TimeoutError) => {
                report_init_probe_liveness(&liveness, mode);
            }
        }
    }

    loop {
        apply_phase_updates(
            &mut phase_subscriber,
            &mut mode,
            &mut protocol,
            &mut runtime,
        );
        apply_session_state_updates(&mut state_subscriber, &mut runtime);

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
                            if mode.is_hil_active() {
                                control_sender.send(command).await;
                            } else {
                                handle_control_command(
                                    command,
                                    hil_config,
                                    mode,
                                    boot_started_at,
                                    &control_sender,
                                    &egress,
                                )
                                .await;
                            }
                        }
                        Ok(message) if mode.is_hil_active() => {
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
                    apply_phase_updates(
                        &mut phase_subscriber,
                        &mut mode,
                        &mut protocol,
                        &mut runtime,
                    );
                    apply_session_state_updates(&mut state_subscriber, &mut runtime);
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
                        Err(TimeoutError) => {
                            report_init_probe_liveness(&liveness, mode);
                        }
                    }
                }
            }
            Err(TimeoutError) => {
                report_init_probe_liveness(&liveness, mode);
            }
        }
    }
}

#[embassy_executor::task]
async fn usb_cdc_egress_task(
    class: RpUsbSender,
    hil_config: HilConfig,
    receiver: FcHilEgressReceiver,
) -> ! {
    let mut writer = UsbHilWriter { class };
    let receiver = receiver;
    let mut encoder = MavlinkHilEgressEncoder::new(hil_config.system_id, hil_config.component_id);

    loop {
        writer.class.wait_connection().await;

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
    receiver: FcHilEgressReceiver,
    phase_subscriber: FcFlightPhaseSubscriber,
    state_subscriber: FcHilSessionStateSubscriber,
    control_sender: FcHilControlCommandSender,
    liveness: FcWatchdogLivenessSender,
) {
    let driver = Driver::new(usb, Irqs);

    let mut config = Config::new(0x1209, 0x0001);
    config.manufacturer = Some("MARV");
    config.product = Some("MARV FC");
    config.serial_number = Some("MARV-FC-RP2354B");
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        config,
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
            state_subscriber,
            control_sender,
            crate::channels::HIL_EGRESS_CHANNEL.sender(),
            liveness,
        ))
        .unwrap();
    spawner
        .spawn(usb_cdc_egress_task(sender, hil_config, receiver))
        .unwrap();
}
