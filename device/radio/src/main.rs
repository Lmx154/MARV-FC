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
use embassy_time::{Delay, Instant, Timer};
use static_cell::StaticCell;

use common::coms::transport::lora::mac_codec::{decode_frame, FrameType};
use common::coms::transport::lora::mac_scheduler::{
    TickTracker, LinkEvent, DEFAULT_LOCK_TIMEOUT_MS,
};
use common::coms::transport::lora::lora_config::LoRaConfig;
use common::coms::transport::lora::phy::{
    PhyChannels, PhyService, PhyServiceConfig, TimeSource,
};
use common::drivers::sx1262::Sx1262;

const RX_TIMEOUT_SYMBOLS: u16 = 16;
const TX_QUEUE_LEN: usize = 4;
const RX_QUEUE_LEN: usize = 4;
const LOCK_TIMEOUT_MS: u64 = DEFAULT_LOCK_TIMEOUT_MS;
const SYNC_POLL_MS: u64 = 100;
const TICK_PULSE_US: u64 = 50;
const RX_PULSE_US: u64 = 50;
const TICK_LOG_INTERVAL: u32 = 50;

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
    info!("Radio L3 tick RX (SX1262)");

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

    let mut tracker = TickTracker::new(LOCK_TIMEOUT_MS);
    let mut tick_logs: u32 = 0;
    loop {
        let rx_fut = phy.rx();
        let poll_fut = Timer::after_millis(SYNC_POLL_MS);
        match select::select(rx_fut, poll_fut).await {
            select::Either::First(rx) => {
                rx_led.set_high();
                Timer::after_micros(RX_PULSE_US).await;
                rx_led.set_low();
                match decode_frame(rx.bytes.as_slice()) {
                    Ok((header, _payload)) => {
                        if header.frame_type != FrameType::ControlUp {
                            warn!(
                                "RX unexpected frame type={} tick={}",
                                header.frame_type.name(),
                                header.tick_seq
                            );
                            continue;
                        }
                        tick_pin.set_high();
                        Timer::after_micros(TICK_PULSE_US).await;
                        tick_pin.set_low();

                        let update = tracker.on_uplink(rx.timestamp_ms, header.tick_seq);
                        if update.event == LinkEvent::LockAcquired {
                            info!("SYNC LOCKED tick={}", header.tick_seq);
                        }
                        if update.missed != 0 {
                            warn!(
                                "TICK gap missed={} total_missed={}",
                                update.missed,
                                tracker.missed_ticks()
                            );
                        }
                        tick_logs = tick_logs.wrapping_add(1);
                        if tick_logs % TICK_LOG_INTERVAL == 0 {
                            info!(
                                "TICK rx seq={} missed_total={} rssi={} snr={}",
                                header.tick_seq,
                                tracker.missed_ticks(),
                                rx.rssi,
                                rx.snr
                            );
                        }
                    }
                    Err(err) => {
                        warn!("RX L3 decode error: {:?}", defmt::Debug2Format(&err));
                    }
                }
            }
            select::Either::Second(_) => {
                if tracker.poll(Instant::now().as_millis()) == LinkEvent::LockLost {
                    info!("SYNC LOST: timeout -> SEARCH");
                }
            }
        }
    }
}
