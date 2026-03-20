use common::protocol::mavlink::{DecodeError, MavlinkHilEgressEncoder, MavlinkHilMessagePump};
use common::services::hil::{
    HilByteReader, HilByteWriter, HilEgressMessage, HilIngressLoopError, HilIngressRoutes,
    HilRuntime, run_hil_ingress_loop, send_hil_egress_message,
};
use defmt::{info, warn};
use embassy_executor::{Spawner, task};
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use crate::channels::{BAROMETER_CHANNEL, PayloadHilMissionEventReceiver, TIME_CHANNEL};
use crate::config::HilConfig;

const USB_PACKET_SIZE: u16 = 64;
const MAVLINK_STREAM_CAPACITY: usize = 4096;
const USB_BANNER: &[u8] = b"payload-controller-rp2350\r\n";

type RpUsbDriver = Driver<'static, USB>;
type RpUsbDevice = UsbDevice<'static, RpUsbDriver>;
type RpUsbSender = Sender<'static, RpUsbDriver>;
type RpUsbReceiver = Receiver<'static, RpUsbDriver>;

struct UsbHilReader {
    class: RpUsbReceiver,
}

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

#[task]
async fn usb_device_task(mut usb: RpUsbDevice) -> ! {
    usb.run().await
}

impl HilByteReader for UsbHilReader {
    type Error = EndpointError;

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        self.class.read_packet(buffer).await
    }
}

impl HilByteWriter for UsbHilWriter {
    type Error = EndpointError;

    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.class.write_packet(bytes).await
    }
}

#[task]
async fn usb_cdc_ingest_task(class: RpUsbReceiver) -> ! {
    let mut packet = [0u8; USB_PACKET_SIZE as usize];
    let routes = HilIngressRoutes::new(&TIME_CHANNEL, &(), &BAROMETER_CHANNEL, &(), &());
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

#[task]
async fn usb_cdc_egress_task(
    class: RpUsbSender,
    hil_config: HilConfig,
    receiver: PayloadHilMissionEventReceiver,
) -> ! {
    let mut writer = UsbHilWriter { class };
    let receiver = receiver;
    let mut encoder = MavlinkHilEgressEncoder::new(hil_config.system_id, hil_config.component_id);

    loop {
        writer.class.wait_connection().await;
        let _ = writer.class.write_packet(USB_BANNER).await;

        loop {
            let event = receiver.receive().await;
            if send_hil_egress_message(
                &mut writer,
                &mut encoder,
                HilEgressMessage::MissionEvent(event),
            )
            .await
            .is_err()
            {
                warn!("usb cdc mission event write failed");
                break;
            }
        }
    }
}

pub fn spawn(
    spawner: &Spawner,
    usb: embassy_rp::Peri<'static, USB>,
    hil_config: HilConfig,
    receiver: PayloadHilMissionEventReceiver,
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
    spawner.spawn(usb_cdc_ingest_task(receiver_class)).unwrap();
    spawner
        .spawn(usb_cdc_egress_task(sender, hil_config, receiver))
        .unwrap();
}
