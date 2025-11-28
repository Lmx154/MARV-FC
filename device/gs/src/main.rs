#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::usb::{Driver as UsbDriver, InterruptHandler as UsbInterruptHandler};
use embassy_usb::{
    class::cdc_acm::{CdcAcmClass, State as CdcAcmState},
    driver::EndpointError,
    Builder, Config, UsbDevice,
};
use embassy_time::{Duration, Timer};
use static_cell::StaticCell;
use core::cmp::min;

use common::drivers::sx1262::*;
use common::lora::lora_config::LoRaConfig;
use common::lora::link::LoRaLink;
use common::utils::delay::DelayMs;
use common::tasks::radio::{run_mavlink_text_demo, Role};
use common::tasks::coms::MavEndpointConfig;
use common::coms::usb_cdc::AsyncUsbCdc;
use common::coms::usb_cdc;
use common::coms::uart_coms::MAVLINK_MAX_FRAME;
use common::tasks::radio::RawFrameSink;
use embassy_sync::channel::Channel;
use embassy_sync::channel::Receiver;
use embassy_sync::channel::Sender;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;

embassy_rp::bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<embassy_rp::peripherals::USB>;
});

static USB_CDC_STATE: StaticCell<CdcAcmState<'static>> = StaticCell::new();
static USB_CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
type FrameMsg = heapless::Vec<u8, MAVLINK_MAX_FRAME>;
static FRAME_CH: Channel<RawMutex, FrameMsg, 4> = Channel::new();

struct RawUsbFrameSink {
    tx: Sender<'static, RawMutex, FrameMsg, 4>,
}

impl RawFrameSink for RawUsbFrameSink {
    fn handle(&mut self, frame_bytes: &[u8]) {
        let mut msg: FrameMsg = FrameMsg::new();
        if msg.extend_from_slice(frame_bytes).is_err() {
            warn!("Frame too large for USB channel");
            return;
        }
        if let Err(_e) = self.tx.try_send(msg) {
            warn!("USB frame channel full");
        }
    }
}

struct UsbCdc<'d> {
    class: CdcAcmClass<'d, UsbDriver<'d, embassy_rp::peripherals::USB>>,
    max_packet: usize,
}

impl<'d> UsbCdc<'d> {
    async fn wait_connection(&mut self) {
        self.class.wait_connection().await;
    }

    async fn read_packet(&mut self, buf: &mut [u8]) -> core::result::Result<usize, EndpointError> {
        self.class.read_packet(buf).await
    }
}

impl<'d> AsyncUsbCdc for UsbCdc<'d> {
    type Error = EndpointError;

    async fn write(&mut self, bytes: &[u8]) -> core::result::Result<(), Self::Error> {
        let mut remaining = bytes;
        while !remaining.is_empty() {
            let n = min(remaining.len(), self.max_packet);
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

#[embassy_executor::task]
async fn usb_task(
    mut device: UsbDevice<'static, UsbDriver<'static, embassy_rp::peripherals::USB>>,
) {
    device.run().await;
}

#[embassy_executor::task]
async fn usb_bridge_task(
    mut serial: UsbCdc<'static>,
    frame_rx: Receiver<'static, RawMutex, FrameMsg, 4>,
) {
    let mut rx_buf = [0u8; 64];
    let mut scratch = [0u8; 128];
    let mut frame_rx = frame_rx;

    let _ = usb_cdc::write_line(&mut serial, "USB CDC bridge ready", &mut scratch).await;

    loop {
        // Race incoming frames vs host commands so neither starves.
        let tel_fut = frame_rx.receive();
        let usb_fut = serial.read_packet(&mut rx_buf);
        match embassy_futures::select::select(tel_fut, usb_fut).await {
            embassy_futures::select::Either::First(ev) => {
                let _ = serial.write(&ev).await;
            }
            embassy_futures::select::Either::Second(res) => match res {
                Ok(n) if n > 0 => {
                    let msg = core::str::from_utf8(&rx_buf[..n]).unwrap_or("<non-utf8>");
                    let reply = if msg.trim() == "ping" { "pong" } else { "ack" };

                    let _ = usb_cdc::write_line(&mut serial, reply, &mut scratch).await;
                }
                Ok(_) => {}
                Err(e) => {
                    let _ = usb_cdc::write_line(&mut serial, "usb read err", &mut scratch).await;
                    warn!("USB CDC read error: {:?}", e);
                }
            },
        }
    }
}

// Simple delay
struct EmbassyDelay;
impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms.into())).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("GS Boot");

    let p = embassy_rp::init(Default::default());

    let mav_cfg = MavEndpointConfig {
        sys_id: 1,
        comp_id: 1,
    };

    // USB CDC (for forwarding telemetry to host).
    let usb_driver = UsbDriver::new(p.USB, UsbIrqs);
    let mut usb_config = Config::new(0xC0DE, 0x4001);
    usb_config.manufacturer = Some("MARV-FC");
    usb_config.product = Some("GS USB CDC");
    usb_config.serial_number = Some("0001");
    usb_config.max_packet_size_0 = 64;
    usb_config.max_power = 100;

    let cdc_state = USB_CDC_STATE.init(CdcAcmState::new());
    let cfg_desc = USB_CONFIG_DESCRIPTOR.init([0u8; 256]);
    let bos_desc = USB_BOS_DESCRIPTOR.init([0u8; 256]);
    let msos_desc = USB_MSOS_DESCRIPTOR.init([0u8; 256]);
    let control_buf = USB_CONTROL_BUF.init([0u8; 64]);

    let mut usb_builder = Builder::new(usb_driver, usb_config, cfg_desc, bos_desc, msos_desc, control_buf);
    let cdc = CdcAcmClass::new(&mut usb_builder, cdc_state, 64);
    let max_packet = cdc.max_packet_size() as usize;
    let usb_dev = usb_builder.build();
    let mut usb_serial = UsbCdc {
        class: cdc,
        max_packet,
    };

    spawner.spawn(usb_task(usb_dev)).unwrap();
    info!("GS USB: waiting for host connection...");
    usb_serial.wait_connection().await;
    info!("GS USB: host connected");
    let frame_rx = FRAME_CH.receiver();
    spawner.spawn(usb_bridge_task(usb_serial, frame_rx)).unwrap();

    let mut raw_sink = RawUsbFrameSink {
        tx: FRAME_CH.sender(),
    };

    // SPI
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 10_000_000;

    let spi = Spi::new(
        p.SPI0,
        p.PIN_2,  // SCK
        p.PIN_3,  // MOSI
        p.PIN_4,  // MISO
        p.DMA_CH0,
        p.DMA_CH1,
        spi_cfg,
    );

    let nss   = Output::new(p.PIN_5, Level::High);
    let reset = Output::new(p.PIN_1, Level::High);
    let busy  = Input::new(p.PIN_0, Pull::None);
    let dio1  = Input::new(p.PIN_6, Pull::None);

    // RF Switch (external TXEN/RXEN)
    struct ExtSw {
        tx: Output<'static>,
        rx: Output<'static>,
    }
    impl RfSwitch for ExtSw {
        fn set(&mut self, s: RfState) {
            match s {
                RfState::Rx => {
                    self.rx.set_low();
                    self.tx.set_high();
                }
                RfState::Tx => {
                    self.rx.set_high();
                    self.tx.set_low();
                }
                RfState::Off => {
                    self.rx.set_low();
                    self.tx.set_low();
                }
            }
        }
    }

    let rf_sw = ExtSw {
        tx: Output::new(p.PIN_8, Level::Low),
        rx: Output::new(p.PIN_9, Level::Low),
    };

    let cfg = LoRaConfig::preset_default();
    info!(
        "GS LoRa cfg: f={} Hz sf={} bw_code={} cr_code={} sw=0x{:04X}",
        cfg.freq_hz, cfg.sf, cfg.bw, cfg.cr, cfg.sync_word
    );

    let mut radio = Sx1262::new(spi, nss, reset, busy, dio1, rf_sw, cfg);
    let mut delay = EmbassyDelay;

    radio.init(&mut delay).await.unwrap();

    let mut link = LoRaLink::new(&mut radio);
    link.ack_timeout_ms = cfg.link_ack_timeout_ms;
    link.retries        = cfg.link_retries;

    info!(
        "GS: LoRaLink MAVLink text demo (ack_timeout_ms={} retries={})",
        link.ack_timeout_ms,
        link.retries
    );

    run_mavlink_text_demo(
        &mut link,
        &mut delay,
        Role::Ground,
        mav_cfg,
        None,
        Some(&mut raw_sink),
    )
    .await;
}
