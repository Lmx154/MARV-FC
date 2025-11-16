#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Duration, Timer};

use common::drivers::sx1262::*;
use common::lora::lora_config::LoRaConfig;
use common::utils::delay::DelayMs;

// Simple delay
struct EmbassyDelay;
impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms.into())).await;
    }
}

// Frame types
const KIND_PING: u8 = 0x01;
const KIND_PONG: u8 = 0x02;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("GS Boot");

    let p = embassy_rp::init(Default::default());

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

    let nss = Output::new(p.PIN_5, Level::High);
    let reset = Output::new(p.PIN_1, Level::High);
    let busy = Input::new(p.PIN_0, Pull::None);
    let dio1 = Input::new(p.PIN_6, Pull::None);

    // LED for visual indication (adjust pin as needed)
    let mut led = Output::new(p.PIN_25, Level::Low);

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

    info!("GS: PONG responder mode");
    radio.start_rx_continuous(&mut delay).await.unwrap();

    let mut rx_buf = [0u8; 255];

    loop {
        if let Some(rx) = radio.poll_raw(&mut delay, &mut rx_buf).await.unwrap() {
            if rx.len >= 2 {
                let kind = rx_buf[0];
                let seq = rx_buf[1];
                info!(
                    "GS RX kind=0x{:02X} seq={} len={} rssi={} snr_x4={}",
                    kind, seq, rx.len, rx.rssi, rx.snr_x4
                );

                if kind == KIND_PING {
                    // Blink LED: full RX+TX cycle on GS
                    led.toggle();

                    // Build PONG frame [kind, seq]
                    let tx_payload = [KIND_PONG, seq];
                    info!("GS: replying PONG seq={}", seq);
                    if let Err(e) = radio.tx_raw(&mut delay, &tx_payload).await {
                        warn!("GS tx_raw error: {:?}", e);
                    }

                    // Back to continuous RX
                    if let Err(e) = radio.start_rx_continuous(&mut delay).await {
                        warn!("GS start_rx_continuous error: {:?}", e);
                    }
                } else {
                    warn!("GS: unexpected frame kind=0x{:02X}", kind);
                }
            }
        }

        delay.delay_ms(20).await;
    }
}
