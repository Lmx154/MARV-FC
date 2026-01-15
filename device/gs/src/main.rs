#![no_std]
#![no_main]

mod mac_engine;

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_rp::uart::{
    Async as UartAsync, Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::{Channel, Receiver};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Instant, Timer};
use smart_leds::RGB8;
use static_cell::StaticCell;

use common::coms::transport::lora::link_config::ACTIVE as LINK_CONFIG;
use common::coms::transport::lora::phy_service::{
    PhyChannels, PhyService, PhyServiceConfig, TimeSource,
};
use common::coms::transport::lora::link_transport::LoraTransport;
use common::coms::transport::uart::AsyncUartBus;
use common::coms::transport::uart_packet::{send_packet_over_uart, UART_PACKET_MAX_FRAME};
use common::drivers::sx1262::{set_irq_timestamp_fn, Sx1262};
use common::protocol::packet::Packet;

use crate::mac_engine::{
    LedEvent, MacEngine, MacEngineConfig, LED_QUEUE_LEN, UART_PACKET_QUEUE_LEN,
};

const TX_QUEUE_LEN: usize = 4;
const RX_QUEUE_LEN: usize = 4;
const LED_BRIGHTNESS: u8 = 50;
const LED_RX_OK_COLOR: RGB8 = RGB8 { r: 0, g: LED_BRIGHTNESS, b: 0 };
const LED_TX_OK_COLOR: RGB8 = RGB8 { r: LED_BRIGHTNESS, g: 0, b: 0 };
const LED_ERROR_COLOR: RGB8 = RGB8 { r: LED_BRIGHTNESS, g: 0, b: 0 };
const LED_OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
const LED_PULSE_US: u64 = 2000;
const UART_TX_LOG_EVERY: u32 = 1;
const UART_TX_LOG_PAYLOAD_MAX: usize = 32;

type SpiBus = Mutex<RawMutex, Spi<'static, SPI0, Async>>;
static SPI_BUS: StaticCell<SpiBus> = StaticCell::new();
static PHY_CHANNELS: StaticCell<PhyChannels<TX_QUEUE_LEN, RX_QUEUE_LEN>> = StaticCell::new();
static LED_EVENTS: StaticCell<Channel<RawMutex, LedEvent, LED_QUEUE_LEN>> = StaticCell::new();
static UART_PACKETS: StaticCell<Channel<RawMutex, Packet, UART_PACKET_QUEUE_LEN>> =
    StaticCell::new();

type SpiDev = SpiDevice<'static, RawMutex, Spi<'static, SPI0, Async>, Output<'static>>;
type PhyServiceImpl =
    PhyService<'static, SpiDev, Output<'static>, Input<'static>, Delay, EmbassyTimeSource, TX_QUEUE_LEN, RX_QUEUE_LEN>;
type Ws2812 = PioWs2812<'static, embassy_rp::peripherals::PIO0, 0, 1>;
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

embassy_rp::bind_interrupts!(struct UartIrqs {
    UART0_IRQ => UartInterruptHandler<embassy_rp::peripherals::UART0>;
});

struct RpUart<'d>(Uart<'d, UartAsync>);

impl<'d> AsyncUartBus for RpUart<'d> {
    type Error = embassy_rp::uart::Error;

    async fn write(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        Uart::write(&mut self.0, bytes).await
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        Uart::read(&mut self.0, buf).await
    }
}

#[embassy_executor::task]
async fn phy_service_task(mut service: PhyServiceImpl) -> ! {
    service.run().await
}

#[embassy_executor::task]
async fn led_task(
    mut heartbeat: Output<'static>,
    mut rgb: Ws2812,
    mut events: Receiver<'static, RawMutex, LedEvent, LED_QUEUE_LEN>,
) -> ! {
    let mut colors = [LED_OFF; 1];
    loop {
        let event = events.receive().await;
        match event {
            LedEvent::Tick => {
                heartbeat.set_high();
                Timer::after_micros(LED_PULSE_US).await;
                heartbeat.set_low();
            }
            LedEvent::RxOk => {
                colors[0] = LED_RX_OK_COLOR;
                rgb.write(&colors).await;
                Timer::after_micros(LED_PULSE_US).await;
                colors[0] = LED_OFF;
                rgb.write(&colors).await;
            }
            LedEvent::TxOk => {
                colors[0] = LED_TX_OK_COLOR;
                rgb.write(&colors).await;
                Timer::after_micros(LED_PULSE_US).await;
                colors[0] = LED_OFF;
                rgb.write(&colors).await;
            }
            LedEvent::Error => {
                colors[0] = LED_ERROR_COLOR;
                rgb.write(&colors).await;
                Timer::after_micros(LED_PULSE_US).await;
                colors[0] = LED_OFF;
                rgb.write(&colors).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn uart_tx_task(
    mut uart: RpUart<'static>,
    mut rx: Receiver<'static, RawMutex, Packet, UART_PACKET_QUEUE_LEN>,
) -> ! {
    let mut scratch = [0u8; UART_PACKET_MAX_FRAME];
    let mut sent: u32 = 0;
    let mut errors: u32 = 0;
    loop {
        let packet = rx.receive().await;
        match send_packet_over_uart(&mut uart, &packet, &mut scratch).await {
            Ok(()) => {
                sent = sent.wrapping_add(1);
                if UART_TX_LOG_EVERY > 0 && (sent == 1 || sent % UART_TX_LOG_EVERY == 0) {
                    let payload = packet.payload.as_slice();
                    let show_len = payload.len().min(UART_TX_LOG_PAYLOAD_MAX);
                    if show_len < payload.len() {
                        info!(
                            "GS UART TX frame={} type={} payload_len={} payload_head={=[u8]} (trunc)",
                            sent,
                            packet.packet_type.name(),
                            payload.len(),
                            &payload[..show_len]
                        );
                    } else {
                        info!(
                            "GS UART TX frame={} type={} payload_len={} payload={=[u8]}",
                            sent,
                            packet.packet_type.name(),
                            payload.len(),
                            payload
                        );
                    }
                } else if sent == 1 || sent % 50 == 0 {
                    info!(
                        "UART TX packet type={} count={}",
                        packet.packet_type.name(),
                        sent
                    );
                }
            }
            Err(err) => {
                errors = errors.wrapping_add(1);
                if errors == 1 || errors % 50 == 0 {
                    warn!("UART TX error count={} last={}", errors, err);
                }
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("GS L3 tick uplink (SX1262)");

    let p = embassy_rp::init(Default::default());
    set_irq_timestamp_fn(irq_timestamp_us);

    let mut pio0 = Pio::new(p.PIO0, PioIrqs);
    let ws2812_program = PioWs2812Program::new(&mut pio0.common);
    let rgb = PioWs2812::<_, 0, 1>::new(
        &mut pio0.common,
        pio0.sm0,
        p.DMA_CH2,
        p.PIN_11,
        &ws2812_program,
    );
    let heartbeat = Output::new(p.PIN_25, Level::Low);
    let led_events = LED_EVENTS.init(Channel::new());
    let led_tx = led_events.sender();
    let led_rx = led_events.receiver();
    spawner.spawn(led_task(heartbeat, rgb, led_rx)).unwrap();

    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 10_000_000;

    let spi = Spi::new(
        p.SPI0,
        p.PIN_2,  // SCK
        p.PIN_3,  // MOSI
        p.PIN_0,  // MISO
        p.DMA_CH0,
        p.DMA_CH1,
        spi_cfg,
    );

    let spi_bus = SPI_BUS.init(Mutex::new(spi));
    let nss = Output::new(p.PIN_1, Level::High);
    let spi_dev = SpiDevice::new(spi_bus, nss);

    // ----- UART0 (GS -> CP2102) -----
    // TX: GP12, RX: GP13
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let uart = RpUart(Uart::new(
        p.UART0,
        p.PIN_12,
        p.PIN_13,
        UartIrqs,
        p.DMA_CH3,
        p.DMA_CH4,
        uart_cfg,
    ));

    let reset = Output::new(p.PIN_5, Level::High);
    let busy = Input::new(p.PIN_4, Pull::None);
    let dio1 = Input::new(p.PIN_9, Pull::None);

    let rf_tx = Output::new(p.PIN_7, Level::Low);
    let rf_rx = Output::new(p.PIN_6, Level::Low);
    let tick_pin = Output::new(p.PIN_16, Level::Low);

    let link_cfg = LINK_CONFIG;
    let rf_cfg = link_cfg.rf;
    info!(
        "LoRa cfg: f={} Hz sf={} bw_code={} cr_code={} sw=0x{:04X}",
        rf_cfg.freq_hz, rf_cfg.sf, rf_cfg.bw, rf_cfg.cr, rf_cfg.sync_word
    );
    let radio = Sx1262::new(
        spi_dev,
        reset,
        busy,
        dio1,
        rf_tx,
        rf_rx,
        rf_cfg,
        Delay,
    )
    .await
    .unwrap();

    let channels = PHY_CHANNELS.init(PhyChannels::new());
    let phy = channels.phy();
    let queues = channels.service_queues();
    let rx_timeout_symbols_cfg = link_cfg.rx_timeout_symbols();
    let service = PhyService::new(
        radio,
        EmbassyTimeSource,
        queues,
        PhyServiceConfig {
            rx_timeout_symbols: rx_timeout_symbols_cfg,
        },
    );

    spawner.spawn(phy_service_task(service)).unwrap();

    let uart_packets = UART_PACKETS.init(Channel::new());
    let uart_tx = uart_packets.sender();
    let uart_rx = uart_packets.receiver();
    spawner.spawn(uart_tx_task(uart, uart_rx)).unwrap();

    let slot_rx_symbols = link_cfg.slot_rx_symbols();
    let profile = link_cfg.profile();
    let transport = LoraTransport::default();
    let mut engine = MacEngine::new(
        phy,
        transport,
        profile,
        Some(tick_pin),
        Some(led_tx),
        Some(uart_tx),
        slot_rx_symbols,
        MacEngineConfig::default(),
    );
    engine.run().await;
}
