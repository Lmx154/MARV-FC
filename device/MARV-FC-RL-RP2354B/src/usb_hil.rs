use common::protocol::mavlink::DecodeError;
use common::services::acquisition::MavlinkHilSensorBridge;
use common::services::telemetry::MavlinkStreamPump;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use crate::channels::{
    HIL_BAROMETER_CHANNEL, HIL_GPS_CHANNEL, HIL_IMU_CHANNEL, HIL_TIME_CHANNEL,
};

const USB_PACKET_SIZE: u16 = 64;
const MAVLINK_STREAM_CAPACITY: usize = 4096;

type RpUsbDriver = Driver<'static, USB>;
type RpUsbDevice = UsbDevice<'static, RpUsbDriver>;
type RpUsbClass = CdcAcmClass<'static, RpUsbDriver>;

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

#[embassy_executor::task]
async fn usb_hil_ingest_task(mut class: RpUsbClass) -> ! {
    let mut packet = [0u8; USB_PACKET_SIZE as usize];
    let mut stream = MavlinkStreamPump::<MAVLINK_STREAM_CAPACITY>::new();
    let mut bridge = MavlinkHilSensorBridge::default();

    loop {
        class.wait_connection().await;
        info!("usb cdc HIL host connected");

        loop {
            let len = match class.read_packet(&mut packet).await {
                Ok(len) => len,
                Err(_) => {
                    warn!("usb cdc HIL host disconnected");
                    break;
                }
            };

            stream.ingest_bytes(&packet[..len]);

            while let Some(frame) = stream.try_next_frame() {
                let frame = match frame {
                    Ok(frame) => frame,
                    Err(DecodeError::UnsupportedMessage { .. }) => continue,
                    Err(_) => {
                        warn!("usb cdc HIL decode error");
                        continue;
                    }
                };

                let _ = bridge.handle_frame(
                    frame,
                    &HIL_TIME_CHANNEL,
                    &HIL_IMU_CHANNEL,
                    &HIL_BAROMETER_CHANNEL,
                    &HIL_GPS_CHANNEL,
                );
            }
        }
    }
}

pub fn spawn(spawner: &Spawner, usb: embassy_rp::Peri<'static, USB>) {
    let driver = Driver::new(usb, Irqs);

    let mut config = Config::new(0x1209, 0x0001);
    config.manufacturer = Some("MARV");
    config.product = Some("MARV HIL");
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
    let usb = builder.build();

    spawner.spawn(usb_device_task(usb)).unwrap();
    spawner.spawn(usb_hil_ingest_task(class)).unwrap();
}
