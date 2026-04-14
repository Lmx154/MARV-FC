use common::protocol::mavlink::{DecodeError, MavlinkHilEgressEncoder, MavlinkHilMessagePump};
use common::services::health::LivenessUpdate;
use common::services::hil::{
    HilByteWriter, HilIngressMessage, HilIngressRoutes, HilRuntime, send_hil_egress_message,
};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_time::Instant;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use crate::channels::{
    BAROMETER_CHANNEL, FcHilEgressReceiver, FcWatchdogLivenessSender, GPS_CHANNEL, IMU_CHANNEL,
    TIME_CHANNEL,
};
use crate::config::HilConfig;
use crate::watchdog;

const USB_PACKET_SIZE: u16 = 64;
const MAVLINK_STREAM_CAPACITY: usize = 4096;

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
        .spawn(usb_cdc_ingest_task(receiver_class, liveness))
        .unwrap();
    spawner
        .spawn(usb_cdc_egress_task(sender, hil_config, receiver))
        .unwrap();
}
