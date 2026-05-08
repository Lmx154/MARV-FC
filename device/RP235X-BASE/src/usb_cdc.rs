use common::services::hil::HilByteWriter;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
pub use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

pub const USB_PACKET_SIZE: u16 = 64;

pub type RpUsbDriver = Driver<'static, USB>;
pub type RpUsbDevice = UsbDevice<'static, RpUsbDriver>;
pub type RpUsbSender = Sender<'static, RpUsbDriver>;
pub type RpUsbReceiver = Receiver<'static, RpUsbDriver>;

pub struct UsbCdcConfig {
    pub vendor_id: u16,
    pub product_id: u16,
    pub manufacturer: Option<&'static str>,
    pub product: Option<&'static str>,
    pub serial_number: Option<&'static str>,
}

impl UsbCdcConfig {
    pub const fn new(
        vendor_id: u16,
        product_id: u16,
        manufacturer: &'static str,
        product: &'static str,
        serial_number: &'static str,
    ) -> Self {
        Self {
            vendor_id,
            product_id,
            manufacturer: Some(manufacturer),
            product: Some(product),
            serial_number: Some(serial_number),
        }
    }
}

pub struct UsbCdcParts {
    pub sender: RpUsbSender,
    pub receiver: RpUsbReceiver,
}

pub struct UsbHilWriter {
    pub class: RpUsbSender,
}

bind_interrupts!(struct UsbIrqs {
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

pub async fn write_packet_chunks(
    sender: &mut RpUsbSender,
    bytes: &[u8],
) -> Result<(), EndpointError> {
    for chunk in bytes.chunks(USB_PACKET_SIZE as usize) {
        sender.write_packet(chunk).await?;
    }

    Ok(())
}

pub fn spawn_cdc_acm(
    spawner: &Spawner,
    usb: embassy_rp::Peri<'static, USB>,
    cdc_config: UsbCdcConfig,
) -> UsbCdcParts {
    let driver = Driver::new(usb, UsbIrqs);

    let mut config = Config::new(cdc_config.vendor_id, cdc_config.product_id);
    config.manufacturer = cdc_config.manufacturer;
    config.product = cdc_config.product;
    config.serial_number = cdc_config.serial_number;
    config.max_packet_size_0 = USB_PACKET_SIZE as u8;

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
    let (sender, receiver) = class.split();
    let usb = builder.build();

    spawner.spawn(usb_device_task(usb)).unwrap();

    UsbCdcParts { sender, receiver }
}
