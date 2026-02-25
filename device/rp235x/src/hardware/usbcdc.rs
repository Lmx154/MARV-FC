//! USB CDC hardware setup for RP2350
//!
//! Keeps USB descriptor/state allocation and class wiring out of `main.rs`.

use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver as UsbDriver;
use embassy_usb::{
    Builder, Config as UsbConfig, UsbDevice,
    class::cdc_acm::{CdcAcmClass, State as CdcAcmState},
    driver::EndpointError,
};
use static_cell::StaticCell;

static USB_CDC_STATE: StaticCell<CdcAcmState<'static>> = StaticCell::new();
static USB_CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

// USB-CDC wrapper for MAVLink framing.
pub struct UsbCdc<'d> {
    class: CdcAcmClass<'d, UsbDriver<'d, USB>>,
    max_packet: usize,
}

impl<'d> UsbCdc<'d> {
    pub async fn wait_connection(&mut self) {
        self.class.wait_connection().await;
    }

    pub async fn read_packet(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
        self.class.read_packet(buf).await
    }

    pub async fn write(&mut self, bytes: &[u8]) -> Result<(), EndpointError> {
        let mut remaining = bytes;
        while !remaining.is_empty() {
            let n = core::cmp::min(remaining.len(), self.max_packet);
            self.class.write_packet(&remaining[..n]).await?;
            remaining = &remaining[n..];
        }
        // Short packet to flush if we ended on a packet boundary.
        if !bytes.is_empty() && bytes.len() % self.max_packet == 0 {
            self.class.write_packet(&[]).await?;
        }
        Ok(())
    }
}

pub type RpUsbDevice = UsbDevice<'static, UsbDriver<'static, USB>>;

pub fn build_usb_cdc(usb_driver: UsbDriver<'static, USB>) -> (RpUsbDevice, UsbCdc<'static>) {
    let mut usb_config = UsbConfig::new(0xC0DE, 0x4001);
    usb_config.manufacturer = Some("MARV-FC");
    usb_config.product = Some("Flight Computer");
    usb_config.serial_number = Some("FC001");
    usb_config.max_packet_size_0 = 64;
    usb_config.max_power = 100;

    let cdc_state = USB_CDC_STATE.init(CdcAcmState::new());
    let cfg_desc = USB_CONFIG_DESCRIPTOR.init([0u8; 256]);
    let bos_desc = USB_BOS_DESCRIPTOR.init([0u8; 256]);
    let msos_desc = USB_MSOS_DESCRIPTOR.init([0u8; 256]);
    let control_buf = USB_CONTROL_BUF.init([0u8; 64]);

    let mut usb_builder = Builder::new(
        usb_driver,
        usb_config,
        cfg_desc,
        bos_desc,
        msos_desc,
        control_buf,
    );
    let cdc = CdcAcmClass::new(&mut usb_builder, cdc_state, 64);
    let max_packet = cdc.max_packet_size() as usize;
    let usb_dev = usb_builder.build();
    let usb_cdc = UsbCdc {
        class: cdc,
        max_packet,
    };

    (usb_dev, usb_cdc)
}
