use common::protocol::mavlink::{DecodeError, MavlinkHilEgressEncoder, MavlinkHilMessagePump};
use common::services::health::LivenessUpdate;
use common::services::hil::{
    HilIngressMessage, HilIngressRoutes, HilRuntime, send_hil_egress_message,
};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::peripherals::USB;
use embassy_time::Instant;
use rp235x_base::usb_cdc::{
    RpUsbReceiver, RpUsbSender, USB_PACKET_SIZE, UsbCdcConfig, UsbHilWriter, spawn_cdc_acm,
};

use crate::channels::{
    BAROMETER_CHANNEL, FcHilEgressReceiver, FcWatchdogLivenessSender, GPS_CHANNEL, IMU_CHANNEL,
    TIME_CHANNEL,
};
use crate::config::HilConfig;
use crate::watchdog;

const MAVLINK_STREAM_CAPACITY: usize = 4096;

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

#[cfg(feature = "hil-sensor-backend")]
const fn hil_ingress_enabled() -> bool {
    true
}

#[cfg(not(feature = "hil-sensor-backend"))]
const fn hil_ingress_enabled() -> bool {
    false
}

#[embassy_executor::task]
async fn usb_cdc_ingest_task(mut class: RpUsbReceiver, liveness: FcWatchdogLivenessSender) -> ! {
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

    if hil_ingress_enabled() {
        runtime.enable_virtual_sensor_streaming();
        info!("usb cdc virtual sensor ingress enabled");
    }

    loop {
        class.wait_connection().await;
        info!("usb cdc host connected");

        loop {
            match class.read_packet(&mut packet).await {
                Ok(len) => {
                    if len == 0 {
                        continue;
                    }

                    protocol.ingest_bytes(&packet[..len]);
                    while let Some(message) = protocol.try_next_message() {
                        match message {
                            Ok(HilIngressMessage::ControlCommand(_)) => {}
                            Ok(message) if hil_ingress_enabled() => {
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
                Err(_) => {
                    warn!("usb cdc host disconnected");
                    protocol.reset();
                    runtime.reset();
                    break;
                }
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
    liveness: FcWatchdogLivenessSender,
) {
    let parts = spawn_cdc_acm(
        spawner,
        usb,
        UsbCdcConfig::new(0x1209, 0x0001, "MARV", "MARV FC", "MARV-FC-RP2354B"),
    );

    spawner
        .spawn(usb_cdc_ingest_task(parts.receiver, liveness))
        .unwrap();
    spawner
        .spawn(usb_cdc_egress_task(parts.sender, hil_config, receiver))
        .unwrap();
}
