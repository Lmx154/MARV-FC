// device/radio/src/main.rs
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
use common::lora::link::LoRaLink;
use common::utils::delay::DelayMs;
use common::tasks::radio::{run_bidir_test, Role};

// Simple embassy-based delay
struct EmbassyDelay;
impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms.into())).await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Radio Boot");

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

    // Shared LoRa config (PHY + link policy)
    let cfg = LoRaConfig::preset_default();
    info!(
        "Radio LoRa cfg: f={} Hz sf={} bw_code={} cr_code={} sw=0x{:04X}",
        cfg.freq_hz, cfg.sf, cfg.bw, cfg.cr, cfg.sync_word
    );

    let mut radio = Sx1262::new(spi, nss, reset, busy, dio1, rf_sw, cfg);
    let mut delay = EmbassyDelay;

    radio.init(&mut delay).await.unwrap();

    // Build link layer on top of Layer 0
    let mut link = LoRaLink::new(&mut radio);

    // Use link policy from LoRaConfig (shared between layers)
    link.ack_timeout_ms = cfg.link_ack_timeout_ms;
    link.retries        = cfg.link_retries;

    info!(
        "Radio: LoRaLink bidirectional test mode (ack_timeout_ms={} retries={})",
        link.ack_timeout_ms,
        link.retries
    );

    run_bidir_test(&mut link, &mut delay, Role::Radio).await;
}
