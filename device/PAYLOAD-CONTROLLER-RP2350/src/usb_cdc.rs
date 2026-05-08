use core::sync::atomic::{AtomicBool, Ordering};

use common::protocol::mavlink::{DecodeError, MavlinkHilEgressEncoder, MavlinkHilMessagePump};
use common::services::health::LivenessUpdate;
use common::services::hil::{
    HilEgressMessage, HilEgressTrySender, HilIngressMessage, HilIngressRoutes, HilRuntime,
    UsbHilMode, evaluate_usb_init_control_command, send_hil_egress_message,
    should_report_init_probe_liveness, usb_hil_mode_for_phase,
};
use defmt::{info, warn};
use embassy_executor::{Spawner, task};
use embassy_rp::peripherals::USB;
use embassy_time::{Duration, Instant, TimeoutError, with_timeout};
use rp235x_base::usb_cdc::{
    RpUsbReceiver, RpUsbSender, USB_PACKET_SIZE, UsbCdcConfig, UsbHilWriter, spawn_cdc_acm,
};

use crate::channels::{
    BAROMETER_CHANNEL, HIL_BOOT_SIGNAL, PayloadFlightPhaseSubscriber,
    PayloadHilControlCommandSender, PayloadHilEgressReceiver, PayloadHilEgressSender,
    PayloadHilSessionStateSubscriber, PayloadWatchdogLivenessSender, TIME_CHANNEL,
};
use crate::config::HilConfig;
use crate::watchdog;

const MAVLINK_STREAM_CAPACITY: usize = 4096;
const INIT_PROBE_POLL_MS: u64 = 100;
const USB_BANNER: &[u8] = b"payload-controller-rp2350\r\n";

static USB_CDC_HOST_CONNECTED: AtomicBool = AtomicBool::new(false);

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
    phase_subscriber: &mut PayloadFlightPhaseSubscriber,
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
    state_subscriber: &mut PayloadHilSessionStateSubscriber,
    runtime: &mut HilRuntime,
) {
    while let Some(state) = state_subscriber.try_next_message_pure() {
        runtime.set_session_state(state);
    }
}

fn report_init_probe_liveness(sender: &PayloadWatchdogLivenessSender, mode: UsbHilMode) {
    if should_report_init_probe_liveness(mode) {
        let _ = sender.try_send(LivenessUpdate::new(watchdog::SOURCE_USB_PROBE, now_ms()));
    }
}

async fn handle_control_command<E>(
    command: common::services::hil::HilControlCommand,
    hil_config: HilConfig,
    mode: UsbHilMode,
    boot_started_at: Instant,
    control_sender: &PayloadHilControlCommandSender,
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

#[task]
async fn usb_cdc_ingest_task(
    mut class: RpUsbReceiver,
    hil_config: HilConfig,
    mut phase_subscriber: PayloadFlightPhaseSubscriber,
    mut state_subscriber: PayloadHilSessionStateSubscriber,
    control_sender: PayloadHilControlCommandSender,
    egress: PayloadHilEgressSender,
    liveness: PayloadWatchdogLivenessSender,
) -> ! {
    let mut packet = [0u8; USB_PACKET_SIZE as usize];
    let routes = HilIngressRoutes::new(&TIME_CHANNEL, &(), &BAROMETER_CHANNEL, &(), &(), &());
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
            Err(TimeoutError) => report_init_probe_liveness(&liveness, mode),
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
    state_subscriber: PayloadHilSessionStateSubscriber,
    control_sender: PayloadHilControlCommandSender,
    liveness: PayloadWatchdogLivenessSender,
) {
    let parts = spawn_cdc_acm(
        spawner,
        usb,
        UsbCdcConfig::new(
            0x1209,
            0x0001,
            "MARV",
            "Payload Controller",
            "PAYLOAD-CONTROLLER-RP2350",
        ),
    );

    spawner
        .spawn(usb_cdc_ingest_task(
            parts.receiver,
            hil_config,
            phase_subscriber,
            state_subscriber,
            control_sender,
            crate::channels::HIL_EGRESS_CHANNEL.sender(),
            liveness,
        ))
        .unwrap();
    spawner
        .spawn(usb_cdc_egress_task(parts.sender, hil_config, receiver))
        .unwrap();
}
