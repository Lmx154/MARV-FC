use defmt::{info, warn};
use embassy_executor::{Spawner, task};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

const USB_PACKET_SIZE: u16 = 64;
const USB_BANNER: &[u8] = b"payload-controller-rp2350\r\n";

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

#[task]
async fn usb_device_task(mut usb: RpUsbDevice) -> ! {
    usb.run().await
}

#[task]
async fn usb_cdc_session_task(mut class: RpUsbClass) -> ! {
    let mut packet = [0u8; USB_PACKET_SIZE as usize];

    loop {
        class.wait_connection().await;
        info!("usb cdc host connected");
        let _ = class.write_packet(USB_BANNER).await;

        loop {
            let len = match class.read_packet(&mut packet).await {
                Ok(len) => len,
                Err(_) => {
                    warn!("usb cdc host disconnected");
                    break;
                }
            };

            if len == 0 {
                continue;
            }

            if class.write_packet(&packet[..len]).await.is_err() {
                warn!("usb cdc echo failed");
                break;
            }
        }
    }
}

pub fn spawn(spawner: &Spawner, usb: embassy_rp::Peri<'static, USB>) {
    let driver = Driver::new(usb, Irqs);

    let mut config = Config::new(0x1209, 0x0001);
    config.manufacturer = Some("MARV");
    config.product = Some("Payload Controller");
    config.serial_number = Some("PAYLOAD-CONTROLLER-RP2350");
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
    spawner.spawn(usb_cdc_session_task(class)).unwrap();
}
