use common::protocol::mavlink::{DecodeError, MavlinkHilMessagePump};
use common::services::hil::{
    HilByteReader, HilIngressLoopError, HilIngressRoutes, HilRuntime, run_hil_ingress_loop,
};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use crate::channels::{BAROMETER_CHANNEL, GPS_CHANNEL, IMU_CHANNEL, TIME_CHANNEL};

const USB_PACKET_SIZE: u16 = 64;
const MAVLINK_STREAM_CAPACITY: usize = 4096;

type RpUsbDriver = Driver<'static, USB>;
type RpUsbDevice = UsbDevice<'static, RpUsbDriver>;
type RpUsbClass = CdcAcmClass<'static, RpUsbDriver>;

struct UsbHilReader {
    class: RpUsbClass,
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

impl HilByteReader for UsbHilReader {
    type Error = EndpointError;

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        self.class.read_packet(buffer).await
    }
}

#[embassy_executor::task]
async fn usb_cdc_ingest_task(class: RpUsbClass) -> ! {
    let mut packet = [0u8; USB_PACKET_SIZE as usize];
    let routes = HilIngressRoutes::new(
        &TIME_CHANNEL,
        &IMU_CHANNEL,
        &BAROMETER_CHANNEL,
        &GPS_CHANNEL,
        &(),
    );
    let mut protocol = MavlinkHilMessagePump::<MAVLINK_STREAM_CAPACITY>::new();
    let mut runtime = HilRuntime::new();
    let mut reader = UsbHilReader { class };

    loop {
        reader.class.wait_connection().await;
        info!("usb cdc host connected");

        if let Err(HilIngressLoopError::Transport(_)) = run_hil_ingress_loop(
            &mut reader,
            &mut protocol,
            &mut runtime,
            &routes,
            &mut packet,
            |_| Ok::<(), ()>(()),
            |error| match error {
                DecodeError::UnsupportedMessage { .. } => {}
                _ => warn!("usb cdc decode error"),
            },
        )
        .await
        {
            warn!("usb cdc host disconnected");
            protocol.reset();
            runtime.reset();
        }
    }
}

pub fn spawn(spawner: &Spawner, usb: embassy_rp::Peri<'static, USB>) {
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
    let usb = builder.build();

    spawner.spawn(usb_device_task(usb)).unwrap();
    spawner.spawn(usb_cdc_ingest_task(class)).unwrap();
}
