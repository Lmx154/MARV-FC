#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_futures::select;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Instant, Timer};
use smart_leds::RGB8;
use static_cell::StaticCell;

use common::coms::transport::lora::mac_codec::{
    decode_frame, encode_frame, FrameHeader, FrameType, HEADER_LEN,
};
use common::coms::transport::lora::link_test_config::{
    slot_rx_symbols, ACTIVE as LINK_TEST,
};
use common::coms::transport::lora::mac_scheduler::{
    TickClock, TickTracker, LinkEvent, DEFAULT_LOCK_TIMEOUT_MS,
};
use common::coms::transport::lora::phy::{
    PhyChannels, PhyError, PhyService, PhyServiceConfig, TimeSource,
};
use common::drivers::sx1262::{set_irq_timestamp_fn, Sx1262};

const TX_QUEUE_LEN: usize = 4;
const RX_QUEUE_LEN: usize = 4;
const LOCK_TIMEOUT_MS: u64 = DEFAULT_LOCK_TIMEOUT_MS;
const SYNC_POLL_MS: u64 = 100;
const TICK_HZ: u32 = LINK_TEST.tick_hz;
const TICK_PERIOD_US: u64 = LINK_TEST.tick_period_us();
const TICK_PERIOD_MS: u64 = LINK_TEST.tick_period_ms();
const TICK_PULSE_US: u64 = 50;
const TICK_LOG_INTERVAL: u32 = 50;
const TX_GUARD_US: u64 = LINK_TEST.tx_guard_us;
const SLOT_RATIO_R: u16 = LINK_TEST.slot_ratio_r;
const DL_TX_OFFSET_US: u64 = LINK_TEST.dl_tx_offset_us;
const DOWNLINK_PAYLOAD_LEN: usize = LINK_TEST.downlink_payload_len;
const LED_BRIGHTNESS: u8 = 50;
const LED_TICK_COLOR: RGB8 = RGB8 { r: 0, g: LED_BRIGHTNESS, b: LED_BRIGHTNESS };
const LED_RX_OK_COLOR: RGB8 = RGB8 { r: 0, g: LED_BRIGHTNESS, b: 0 };
const LED_ERROR_COLOR: RGB8 = RGB8 { r: LED_BRIGHTNESS, g: 0, b: 0 };
const LED_OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const LED_QUEUE_LEN: usize = 16;
const LED_PULSE_US: u64 = 2000;

type SpiBus = Mutex<RawMutex, Spi<'static, SPI0, Async>>;
static SPI_BUS: StaticCell<SpiBus> = StaticCell::new();
static PHY_CHANNELS: StaticCell<PhyChannels<TX_QUEUE_LEN, RX_QUEUE_LEN>> = StaticCell::new();
static LED_EVENTS: StaticCell<Channel<RawMutex, LedEvent, LED_QUEUE_LEN>> = StaticCell::new();

type SpiDev = SpiDevice<'static, RawMutex, Spi<'static, SPI0, Async>, Output<'static>>;
type PhyServiceImpl =
    PhyService<'static, SpiDev, Output<'static>, Input<'static>, Delay, EmbassyTimeSource, TX_QUEUE_LEN, RX_QUEUE_LEN>;
type Ws2812 = PioWs2812<'static, embassy_rp::peripherals::PIO0, 0, 1>;

#[derive(Clone, Copy)]
enum LedEvent {
    Tick,
    RxOk,
    Error,
}

#[derive(Clone, Copy)]
struct EmbassyTimeSource;

impl TimeSource for EmbassyTimeSource {
    fn now_us(&self) -> u64 {
        Instant::now().as_micros()
    }
}

fn irq_timestamp_us() -> u64 {
    Instant::now().as_micros()
}

embassy_rp::bind_interrupts!(struct PioIrqs {
    PIO0_IRQ_0 => PioInterruptHandler<embassy_rp::peripherals::PIO0>;
});

#[embassy_executor::task]
async fn phy_service_task(mut service: PhyServiceImpl) -> ! {
    service.run().await
}

#[embassy_executor::task]
async fn led_task(
    mut led: Ws2812,
    mut events: Receiver<'static, RawMutex, LedEvent, LED_QUEUE_LEN>,
) -> ! {
    let mut colors = [LED_OFF; 1];
    loop {
        let event = events.receive().await;
        let next = match event {
            LedEvent::Tick => LED_TICK_COLOR,
            LedEvent::RxOk => LED_RX_OK_COLOR,
            LedEvent::Error => LED_ERROR_COLOR,
        };
        colors[0] = next;
        led.write(&colors).await;
        Timer::after_micros(LED_PULSE_US).await;
        colors[0] = LED_OFF;
        led.write(&colors).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Radio L3 tick RX (SX1262)");

    let p = embassy_rp::init(Default::default());
    set_irq_timestamp_fn(irq_timestamp_us);

    let mut pio0 = Pio::new(p.PIO0, PioIrqs);
    let ws2812_program = PioWs2812Program::new(&mut pio0.common);
    let mut led = PioWs2812::<_, 0, 1>::new(
        &mut pio0.common,
        pio0.sm0,
        p.DMA_CH2,
        p.PIN_15,
        &ws2812_program,
    );
    let led_events = LED_EVENTS.init(Channel::new());
    let led_tx = led_events.sender();
    let led_rx = led_events.receiver();
    spawner.spawn(led_task(led, led_rx)).unwrap();

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

    let cfg = LINK_TEST.lora;
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
    let rx_timeout_symbols = slot_rx_symbols(LINK_TEST);
    let service = PhyService::new(
        radio,
        EmbassyTimeSource,
        queues,
        PhyServiceConfig {
            rx_timeout_symbols,
        },
    );

    spawner.spawn(phy_service_task(service)).unwrap();

    let mut tracker = TickTracker::new(LOCK_TIMEOUT_MS, TICK_PERIOD_MS);
    let mut tick_clock = TickClock::new(Instant::now().as_micros(), TICK_PERIOD_US);
    let mut tick_logs: u32 = 0;
    let mut next_tick_seq: Option<u16> = None;
    let mut constraint_violations: u32 = 0;
    loop {
        let now_us = Instant::now().as_micros();
        let wait_us = tick_clock.next_tick_boundary_us().saturating_sub(now_us);
        let rx_fut = phy.rx();
        let tick_fut = Timer::after_micros(wait_us);
        let poll_fut = Timer::after_millis(SYNC_POLL_MS);
        match select::select3(rx_fut, tick_fut, poll_fut).await {
            select::Either3::First(rx) => {
                match decode_frame(rx.bytes.as_slice()) {
                    Ok((header, _payload)) => {
                        led_tx.send(LedEvent::RxOk).await;
                        if header.frame_type != FrameType::ControlUp {
                            warn!(
                                "RX unexpected frame type={} tick={}",
                                header.frame_type.name(),
                                header.tick_seq
                            );
                            continue;
                        }
                        let rx_toa_us = cfg.toa_us(rx.bytes.len());
                        let tick_start_us =
                            rx.rx_done_instant_us.saturating_sub(rx_toa_us);
                        tick_clock.align(tick_start_us);
                        next_tick_seq = Some(header.tick_seq.wrapping_add(1));

                        let update = tracker.on_uplink_tdma(
                            rx.rx_done_instant_us / 1000,
                            header.tick_seq,
                            SLOT_RATIO_R,
                        );
                        if update.event == LinkEvent::LockAcquired {
                            info!("SYNC LOCKED tick={}", header.tick_seq);
                        }
                        if update.missed != 0 {
                            warn!(
                                "UL slot gap missed={} total_missed={}",
                                update.missed,
                                tracker.missed_ticks()
                            );
                        }
                        if update.seq_anomaly != 0 {
                            warn!(
                                "TICK seq anomaly delta={} total_seq_anom={}",
                                update.seq_anomaly,
                                tracker.seq_anomaly_count()
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
                        led_tx.send(LedEvent::Error).await;
                        warn!("RX L3 decode error: {:?}", defmt::Debug2Format(&err));
                    }
                }
            }
            select::Either3::Second(_) => {
                let now_us = Instant::now().as_micros();
                let Some(tick_start_us) = tick_clock.poll(now_us) else {
                    continue;
                };
                led_tx.send(LedEvent::Tick).await;
                tick_pin.set_high();
                Timer::after_micros(TICK_PULSE_US).await;
                tick_pin.set_low();

                let Some(tick_seq) = next_tick_seq else {
                    continue;
                };
                next_tick_seq = Some(tick_seq.wrapping_add(1));

                if tick_seq % SLOT_RATIO_R != 0 {
                    continue;
                }

                let payload = [0u8; DOWNLINK_PAYLOAD_LEN];
                let header = FrameHeader::new(FrameType::ControlDown, tick_seq);
                let tx = encode_frame(&header, &payload);
                if tx.len() < HEADER_LEN {
                    warn!("TX encode error");
                    continue;
                }

                let window = tick_clock.window(tick_start_us, TX_GUARD_US);
                let planned_tx_start_us =
                    tick_start_us.saturating_add(DL_TX_OFFSET_US);
                let now_us = Instant::now().as_micros();
                let tx_start_us = planned_tx_start_us.max(now_us);
                let duration_us = cfg.toa_us(tx.len());
                if !window.fits_tx(tx_start_us, duration_us) {
                    constraint_violations = constraint_violations.wrapping_add(1);
                    led_tx.send(LedEvent::Error).await;
                    warn!(
                        "Refusing DL TX: ToA={}ms > SlotBudget={}ms (violations={})",
                        duration_us / 1000,
                        window.budget_us_from(tx_start_us) / 1000,
                        constraint_violations
                    );
                    continue;
                }

                if tx_start_us > now_us {
                    Timer::after_micros(tx_start_us - now_us).await;
                }
                match phy.tx(tx.as_slice()).await {
                    Ok(()) => {}
                    Err(PhyError::PayloadTooLarge) => {
                        warn!("TX payload too large");
                    }
                }
                info!("TX CONTROL_DOWN tick={}", tick_seq);
            }
            select::Either3::Third(_) => {
                if tracker.poll(Instant::now().as_millis()) == LinkEvent::LockLost {
                    info!("SYNC LOST: timeout -> SEARCH");
                }
            }
        }
    }
}
