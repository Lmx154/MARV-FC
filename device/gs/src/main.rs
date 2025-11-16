// gs/src/main.rs

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

// Simple delay
struct EmbassyDelay;
impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms.into())).await;
    }
}

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
        p.DMA_CH0, p.DMA_CH1,
        spi_cfg,
    );

    let nss   = Output::new(p.PIN_5, Level::High);
    let reset = Output::new(p.PIN_1, Level::High);
    let busy  = Input::new( p.PIN_0, Pull::None );
    let dio1  = Input::new( p.PIN_6, Pull::None );

    // RF Switch
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

    // Continuous RX
    radio.start_rx_continuous(&mut delay).await.unwrap();

    let mut buf = [0u8; 255];

    info!("GS ready, listening…");

    loop {
        if let Ok(Some(rx)) = radio.poll_raw(&mut delay, &mut buf).await {
            info!("GS RX len={} RSSI={} SNR/4={}", rx.len, rx.rssi, rx.snr_x4);

            if rx.len == 4 && &buf[..4] == b"PING" {
                info!("Received PING → sending PONG");
                radio.tx_raw(&mut delay, b"PONG").await.unwrap();
                radio.start_rx_continuous(&mut delay).await.unwrap();
            }
        }

        Timer::after_millis(50).await;
    }
}
