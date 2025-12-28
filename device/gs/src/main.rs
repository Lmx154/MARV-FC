#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use static_cell::StaticCell;

use common::coms::transport::lora::lora_config::LoRaConfig;
use common::coms::transport::lora::phy::{
    PhyChannels, PhyError, PhyService, PhyServiceConfig, TimeSource,
};
use common::drivers::sx1262::Sx1262;

const TX_INTERVAL_MS: u64 = 200;
const RX_TIMEOUT_SYMBOLS: u16 = 16;
const TX_QUEUE_LEN: usize = 4;
const RX_QUEUE_LEN: usize = 4;

type SpiBus = Mutex<RawMutex, Spi<'static, SPI0, Async>>;
static SPI_BUS: StaticCell<SpiBus> = StaticCell::new();
static PHY_CHANNELS: StaticCell<PhyChannels<TX_QUEUE_LEN, RX_QUEUE_LEN>> = StaticCell::new();

type SpiDev = SpiDevice<'static, RawMutex, Spi<'static, SPI0, Async>, Output<'static>>;
type PhyServiceImpl =
    PhyService<'static, SpiDev, Output<'static>, Input<'static>, Delay, EmbassyTimeSource, TX_QUEUE_LEN, RX_QUEUE_LEN>;

#[derive(Clone, Copy)]
struct EmbassyTimeSource;

impl TimeSource for EmbassyTimeSource {
    fn now_ms(&self) -> u64 {
        Instant::now().as_millis()
    }
}

#[embassy_executor::task]
async fn phy_service_task(mut service: PhyServiceImpl) -> ! {
    service.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("GS L2 ping-pong (SX1262)");

    let p = embassy_rp::init(Default::default());

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

    let spi_bus = SPI_BUS.init(Mutex::new(spi));
    let nss = Output::new(p.PIN_5, Level::High);
    let spi_dev = SpiDevice::new(spi_bus, nss);

    let reset = Output::new(p.PIN_1, Level::High);
    let busy = Input::new(p.PIN_0, Pull::None);
    let dio1 = Input::new(p.PIN_6, Pull::None);

    let rf_tx = Output::new(p.PIN_8, Level::Low);
    let rf_rx = Output::new(p.PIN_9, Level::Low);

    let cfg = LoRaConfig::preset_default();
    info!(
        "LoRa cfg: f={} Hz sf={} bw_code={} cr_code={} sw=0x{:04X}",
        cfg.freq_hz, cfg.sf, cfg.bw, cfg.cr, cfg.sync_word
    );

    let radio = Sx1262::new(
        spi_dev,
        reset,
        busy,
        dio1,
        rf_tx,
        rf_rx,
        cfg,
        Delay,
    )
    .await
    .unwrap();

    let channels = PHY_CHANNELS.init(PhyChannels::new());
    let phy = channels.phy();
    let queues = channels.service_queues();
    let service = PhyService::new(
        radio,
        EmbassyTimeSource,
        queues,
        PhyServiceConfig {
            rx_timeout_symbols: RX_TIMEOUT_SYMBOLS,
        },
    );

    spawner.spawn(phy_service_task(service)).unwrap();

    let mut seq: u32 = 0;
    let mut pong_ok: u32 = 0;
    let mut last_tx_ms: u64 = 0;

    loop {
        seq = seq.wrapping_add(1);
        last_tx_ms = Instant::now().as_millis();
        match phy.tx(b"PING").await {
            Ok(()) => {}
            Err(PhyError::PayloadTooLarge) => {
                warn!("TX payload too large");
            }
        }
        info!("TX PING seq={}", seq);

        let rx = phy.rx().await;
        if rx.bytes.as_slice() == b"PONG" {
            pong_ok = pong_ok.wrapping_add(1);
            let rtt_ms = rx.timestamp_ms.saturating_sub(last_tx_ms);
            info!(
                "RX PONG seq={} ok={} rtt_ms={} rssi={} snr={}",
                seq, pong_ok, rtt_ms, rx.rssi, rx.snr
            );
        } else if rx.bytes.as_slice() == b"PING" {
            warn!("RX unexpected PING len={}", rx.bytes.len());
        } else {
            let b0 = *rx.bytes.get(0).unwrap_or(&0);
            let b1 = *rx.bytes.get(1).unwrap_or(&0);
            let b2 = *rx.bytes.get(2).unwrap_or(&0);
            let b3 = *rx.bytes.get(3).unwrap_or(&0);
            warn!(
                "RX unexpected len={} bytes={:02X} {:02X} {:02X} {:02X}",
                rx.bytes.len(),
                b0,
                b1,
                b2,
                b3
            );
        }

        Timer::after(Duration::from_millis(TX_INTERVAL_MS)).await;
    }
}
