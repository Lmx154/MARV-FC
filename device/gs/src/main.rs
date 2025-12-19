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
use heapless::Vec;

use common::drivers::sx1262::*;
use common::coms::transport::lora::lora_config::LoRaConfig;
use common::coms::transport::lora::link::LoRaLink;
use common::utils::delay::DelayMs;
use common::tasks::coms::MavEndpointConfig;
use common::coms::usb_cdc::AsyncUsbCdc;
use common::coms::usb_cdc;
use common::coms::transport::uart::MAVLINK_MAX_FRAME;
use embassy_sync::channel::Channel;
use embassy_sync::channel::Receiver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;

embassy_rp::bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => UsbInterruptHandler<embassy_rp::peripherals::USB>;
});

static USB_CDC_STATE: StaticCell<CdcAcmState<'static>> = StaticCell::new();
static USB_CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static USB_CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
type FrameMsg = Vec<u8, MAVLINK_MAX_FRAME>;
static FRAME_CH: Channel<RawMutex, FrameMsg, 4> = Channel::new();

// Host -> LoRa (commands/params/etc)
static USB_TO_LORA_CH: Channel<RawMutex, FrameMsg, 8> = Channel::new();

// (LoRa -> USB forwarding uses FRAME_CH directly.)

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
    let frame_rx = frame_rx;

    // Accumulate raw bytes from USB until we can extract whole MAVLink2 frames.
    // Capacity is intentionally bounded; on overflow we drop oldest bytes.
    let mut stream_buf: Vec<u8, 512> = Vec::new();

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
                    // Binary bridge: do NOT emit ASCII replies (they corrupt MAVLink streams).
                    if push_bytes_with_drop_oldest(&mut stream_buf, &rx_buf[..n]).is_err() {
                        warn!("USB RX: stream buffer overflow");
                    }

                    while let Some(frame) = try_pop_mavlink2_frame(&mut stream_buf) {
                        if USB_TO_LORA_CH.sender().try_send(frame).is_err() {
                            warn!("USB->LoRa queue full (dropping)");
                        }
                    }
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

fn push_bytes_with_drop_oldest<const N: usize>(
    buf: &mut Vec<u8, N>,
    data: &[u8],
) -> core::result::Result<(), ()> {
    for &b in data {
        if buf.push(b).is_err() {
            // Drop oldest byte to make room.
            if !buf.is_empty() {
                buf.remove(0);
            }
            if buf.push(b).is_err() {
                return Err(());
            }
        }
    }
    Ok(())
}

fn try_pop_mavlink2_frame<const N: usize>(buf: &mut Vec<u8, N>) -> Option<FrameMsg> {
    const MAGIC: u8 = 0xFD;
    const HEADER_LEN: usize = 10;
    const CRC_LEN: usize = 2;
    const SIG_LEN: usize = 13;

    // Find magic.
    let start = buf.iter().position(|&b| b == MAGIC)?;
    if start != 0 {
        // Discard leading noise bytes.
        for _ in 0..start {
            buf.remove(0);
        }
    }

    if buf.len() < HEADER_LEN {
        return None;
    }

    let payload_len = buf[1] as usize;
    let incompat_flags = buf[2];
    let sig_len = if (incompat_flags & 0x01) != 0 { SIG_LEN } else { 0 };
    let frame_len = HEADER_LEN + payload_len + CRC_LEN + sig_len;

    if buf.len() < frame_len {
        return None;
    }

    let mut out: FrameMsg = FrameMsg::new();
    if out.extend_from_slice(&buf[..frame_len]).is_err() {
        // Frame too big for MAVLINK_MAX_FRAME (should not happen).
        for _ in 0..frame_len {
            buf.remove(0);
        }
        return None;
    }

    for _ in 0..frame_len {
        buf.remove(0);
    }
    Some(out)
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
        "GS: LoRaLink MAVLink bridge (ack_timeout_ms={} retries={})",
        link.ack_timeout_ms,
        link.retries
    );

    if let Err(e) = link.start_rx(&mut delay).await {
        warn!("GS: start_rx failed: {:?}", e);
    }

    gs_lora_loop(&mut link, &mut delay, mav_cfg).await;
}

async fn gs_lora_loop<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut EmbassyDelay,
    _cfg: MavEndpointConfig,
) where
    RADIO: common::coms::transport::lora::link::Sx1262Interface,
{
    let mut rx_buf = [0u8; 255];
    let mut tx_gate = common::coms::scheduler::TxGate::new();

    let usb_to_lora_rx = USB_TO_LORA_CH.receiver();

    loop {
        // Poll LoRa RX frequently, but allow outbound USB traffic to preempt.
        let tx_fut = usb_to_lora_rx.receive();
        let tick_fut = Timer::after(Duration::from_millis(2));

        match embassy_futures::select::select(tx_fut, tick_fut).await {
            embassy_futures::select::Either::First(msg) => {
                let now_ms = embassy_time::Instant::now().as_millis() as u32;
                let min_gap_ms = link.recommended_tx_gap_ms();
                let wait_ms = tx_gate.time_until_allowed_ms(now_ms, min_gap_ms);
                if wait_ms != 0 {
                    delay.delay_ms(wait_ms).await;
                }

                if let Err(e) = link.send(delay, &msg).await {
                    warn!("GS LoRa TX error: {:?}", e);
                } else {
                    let now_ms = embassy_time::Instant::now().as_millis() as u32;
                    tx_gate.on_tx(now_ms);
                }
            }
            embassy_futures::select::Either::Second(()) => {
                match link.try_recv(delay, &mut rx_buf).await {
                    Ok(Some(len)) => {
                        let mut out: FrameMsg = FrameMsg::new();
                        if out.extend_from_slice(&rx_buf[..len]).is_ok() {
                            if FRAME_CH.sender().try_send(out).is_err() {
                                warn!("LoRa->USB queue full (dropping)");
                            }
                        }
                    }
                    Ok(None) => {}
                    Err(e) => warn!("GS LoRa RX error: {:?}", e),
                }
            }
        }
    }
}
