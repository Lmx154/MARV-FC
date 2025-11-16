//radio/src/main.rs
#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::spi::{Spi, Config as SpiConfig};
use embassy_time::{Timer, Duration};

use common::drivers::sx1262::*;
use common::lora::lora_config::*;
use common::utils::delay::DelayMs;

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
        p.PIN_2, p.PIN_3, p.PIN_4,
        p.DMA_CH0, p.DMA_CH1,
        spi_cfg,
    );

    let nss   = Output::new(p.PIN_5, Level::High);
    let reset = Output::new(p.PIN_1, Level::High);
    let busy  = Input::new( p.PIN_0, Pull::None );
    let dio1  = Input::new( p.PIN_6, Pull::None );

    struct ExtSw { tx: Output<'static>, rx: Output<'static> }
    impl RfSwitch for ExtSw {
        fn set(&mut self, s: RfState) {
            match s {
                RfState::Rx => { self.rx.set_low(); self.tx.set_high(); }
                RfState::Tx => { self.rx.set_high(); self.tx.set_low(); }
                RfState::Off => { self.rx.set_low(); self.tx.set_low(); }
            }
        }
    }

    let rf_sw = ExtSw {
        tx: Output::new(p.PIN_8, Level::Low),
        rx: Output::new(p.PIN_9, Level::Low),
    };

    let mut radio = Sx1262::new(spi, nss, reset, busy, dio1, rf_sw);
    let mut delay = EmbassyDelay;

    let cfg = LoRaConfig::preset_default();
    radio.init(&mut delay, &cfg).await.unwrap();

    info!("Radio: starting PING loop");

    loop {
        info!("Sending PING…");
        radio.tx_raw(&mut delay, b"PING").await.unwrap();
        Timer::after_secs(1).await;
    }
}