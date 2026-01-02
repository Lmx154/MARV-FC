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
use embassy_time::{Delay, Duration, Instant, Timer};
use smart_leds::RGB8;
use static_cell::StaticCell;

use common::coms::transport::lora::mac_codec::{
    decode_frame, encode_frame, FrameHeader, FrameType, HEADER_LEN,
};
use common::coms::transport::lora::link_test_config::{
    rx_timeout_symbols, slot_rx_symbols, ACTIVE as LINK_TEST,
};
use common::coms::transport::lora::mac_scheduler::TickClock;
use common::coms::transport::lora::phy::{
    PhyChannels, PhyError, PhyService, PhyServiceConfig, TimeSource,
};
use common::drivers::sx1262::{set_irq_timestamp_fn, Sx1262};

const TICK_HZ: u32 = LINK_TEST.tick_hz;
const TICK_PERIOD_US: u64 = LINK_TEST.tick_period_us();
const TICK_PULSE_US: u64 = 50;
const TX_QUEUE_LEN: usize = 4;
const RX_QUEUE_LEN: usize = 4;
const LED_BRIGHTNESS: u8 = 50;
const LED_TICK_COLOR: RGB8 = RGB8 { r: 0, g: LED_BRIGHTNESS, b: LED_BRIGHTNESS };
const LED_RX_OK_COLOR: RGB8 = RGB8 { r: 0, g: LED_BRIGHTNESS, b: 0 };
const LED_ERROR_COLOR: RGB8 = RGB8 { r: LED_BRIGHTNESS, g: 0, b: 0 };
const LED_OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const LED_QUEUE_LEN: usize = 16;
const LED_PULSE_US: u64 = 2000;
const TX_GUARD_US: u64 = LINK_TEST.tx_guard_us;
const SLOT_RATIO_R: u16 = LINK_TEST.slot_ratio_r;
const DL_TX_OFFSET_US: u64 = LINK_TEST.dl_tx_offset_us;
const RX_READY_GUARD_US: u64 = LINK_TEST.rx_ready_guard_us;
const UPLINK_PAYLOAD_LEN: usize = LINK_TEST.uplink_payload_len;

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
    info!("GS L3 tick uplink (SX1262)");

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
    let rx_timeout_symbols_cfg = rx_timeout_symbols(LINK_TEST);
    let service = PhyService::new(
        radio,
        EmbassyTimeSource,
        queues,
        PhyServiceConfig {
            rx_timeout_symbols: rx_timeout_symbols_cfg,
        },
    );

    spawner.spawn(phy_service_task(service)).unwrap();

    let slot_rx_symbols = slot_rx_symbols(LINK_TEST);
    let mut tick_seq: u16 = 0;
    let mut constraint_violations: u32 = 0;
    let mut tick_clock = TickClock::new(Instant::now().as_micros(), TICK_PERIOD_US);

    loop {
        let now_us = Instant::now().as_micros();
        let wait_us = tick_clock.next_tick_boundary_us().saturating_sub(now_us);
        let rx_fut = phy.rx();
        let wait_fut = Timer::after(Duration::from_micros(wait_us));

        match select::select(rx_fut, wait_fut).await {
            select::Either::First(rx) => {
                match decode_frame(rx.bytes.as_slice()) {
                    Ok((header, payload)) => {
                        led_tx.send(LedEvent::RxOk).await;
                        match header.frame_type {
                            FrameType::ControlDown => {
                                let expected = header.tick_seq % SLOT_RATIO_R == 0;
                                if !expected {
                                    warn!(
                                        "RX CONTROL_DOWN in uplink slot tick={} len={} rssi={} snr={}",
                                        header.tick_seq,
                                        payload.len(),
                                        rx.rssi,
                                        rx.snr
                                    );
                                } else {
                                    info!(
                                        "RX CONTROL_DOWN tick={} len={} rssi={} snr={}",
                                        header.tick_seq,
                                        payload.len(),
                                        rx.rssi,
                                        rx.snr
                                    );
                                }
                            }
                            _ => {
                                warn!(
                                    "RX unexpected frame type={} tick={} len={} rssi={} snr={}",
                                    header.frame_type.name(),
                                    header.tick_seq,
                                    payload.len(),
                                    rx.rssi,
                                    rx.snr
                                );
                            }
                        }
                    }
                    Err(err) => {
                        led_tx.send(LedEvent::Error).await;
                        warn!("RX L3 decode error: {:?}", defmt::Debug2Format(&err));
                    }
                }
            }
            select::Either::Second(_) => {
                let now_us = Instant::now().as_micros();
                let Some(tick_start_us) = tick_clock.poll(now_us) else {
                    continue;
                };
                led_tx.send(LedEvent::Tick).await;
                tick_pin.set_high();
                Timer::after_micros(TICK_PULSE_US).await;
                tick_pin.set_low();

                tick_seq = tick_seq.wrapping_add(1);
                if tick_seq % SLOT_RATIO_R == 0 {
                    phy.arm_rx(slot_rx_symbols);
                    let rx_ready_deadline_us = tick_start_us
                        .saturating_add(DL_TX_OFFSET_US)
                        .saturating_sub(RX_READY_GUARD_US);
                    let now_us = Instant::now().as_micros();
                    if now_us > rx_ready_deadline_us {
                        warn!(
                            "RX arm late for DL slot tick={} now={}us deadline={}us",
                            tick_seq,
                            now_us,
                            rx_ready_deadline_us
                        );
                    }
                    continue;
                }

                let header = FrameHeader::new(FrameType::ControlUp, tick_seq);
                let payload = [0xA5u8; UPLINK_PAYLOAD_LEN];
                let tx = encode_frame(&header, &payload);
                if tx.len() < HEADER_LEN {
                    warn!("TX encode error");
                    continue;
                }

                let window = tick_clock.window(tick_start_us, TX_GUARD_US);
                let tx_start_us = Instant::now().as_micros();
                let duration_us = cfg.toa_us(tx.len());
                if !window.fits_tx(tx_start_us, duration_us) {
                    constraint_violations = constraint_violations.wrapping_add(1);
                    led_tx.send(LedEvent::Error).await;
                    warn!(
                        "Refusing TX: ToA={}ms > SlotBudget={}ms (violations={})",
                        duration_us / 1000,
                        window.budget_us_from(tx_start_us) / 1000,
                        constraint_violations
                    );
                    continue;
                }
                match phy.tx(tx.as_slice()).await {
                    Ok(()) => {}
                    Err(PhyError::PayloadTooLarge) => {
                        warn!("TX payload too large");
                    }
                }
                info!("TX CONTROL_UP tick={} len={}", tick_seq, payload.len());
            }
        }
    }
}
