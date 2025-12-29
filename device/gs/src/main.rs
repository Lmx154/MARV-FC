#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_futures::select;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use static_cell::StaticCell;

use common::coms::transport::lora::mac_codec::{
    decode_frame, encode_frame, FrameHeader, FrameType, HEADER_LEN,
};
use common::coms::transport::lora::lora_config::LoRaConfig;
use common::coms::transport::lora::phy::{
    PhyChannels, PhyError, PhyService, PhyServiceConfig, TimeSource,
};
use common::drivers::sx1262::Sx1262;

const TICK_HZ: u32 = 50;
const TICK_PERIOD_MS: u64 = 1000 / TICK_HZ as u64;
const TICK_PULSE_US: u64 = 50;
const RX_PULSE_US: u64 = 50;
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
    info!("GS L3 tick uplink (SX1262)");

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
    let mut tick_pin = Output::new(p.PIN_16, Level::Low);
    let mut rx_led = Output::new(p.PIN_25, Level::Low);

    let cfg = LoRaConfig::preset_fast();
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

    let mut tick_seq: u16 = 0;
    let mut next_tick_ms: u64 = Instant::now().as_millis();

    loop {
        let now_ms = Instant::now().as_millis();
        let wait_ms = next_tick_ms.saturating_sub(now_ms);
        let rx_fut = phy.rx();
        let wait_fut = Timer::after(Duration::from_millis(wait_ms));

        match select::select(rx_fut, wait_fut).await {
            select::Either::First(rx) => {
                rx_led.set_high();
                Timer::after_micros(RX_PULSE_US).await;
                rx_led.set_low();
                match decode_frame(rx.bytes.as_slice()) {
                    Ok((header, payload)) => {
                        warn!(
                            "RX unexpected frame type={} tick={} len={} rssi={} snr={}",
                            header.frame_type.name(),
                            header.tick_seq,
                            payload.len(),
                            rx.rssi,
                            rx.snr
                        );
                    }
                    Err(err) => {
                        warn!("RX L3 decode error: {:?}", defmt::Debug2Format(&err));
                    }
                }
            }
            select::Either::Second(_) => {
                tick_seq = tick_seq.wrapping_add(1);
                let header = FrameHeader::new(FrameType::ControlUp, tick_seq);
                let tx = encode_frame(&header, &[]);
                if tx.len() < HEADER_LEN {
                    warn!("TX encode error");
                } else {
                    tick_pin.set_high();
                    Timer::after_micros(TICK_PULSE_US).await;
                    tick_pin.set_low();
                    match phy.tx(tx.as_slice()).await {
                        Ok(()) => {}
                        Err(PhyError::PayloadTooLarge) => {
                            warn!("TX payload too large");
                        }
                    }
                    info!("TX CONTROL_UP tick={}", tick_seq);
                }
                next_tick_ms = Instant::now().as_millis().wrapping_add(TICK_PERIOD_MS);
            }
        }
    }
}
