#![no_std]
#![no_main]

// Minimal radio firmware entrypoint for RP2350A Pico 2 using SX1262 only.

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output, Input, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_time::{Timer, Duration};

use common::drivers::sx1262::{Sx1262, Sx1262Config, RfSwitchMode, RfState, RfSwitch};
use common::utils::delay::DelayMs;

// Simple Embassy-based async delay bridge
struct EmbassyDelay;
impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms as u64)).await;
    }
}

// Simple external RF switch stub (no-op by default)
struct NoopRfSwitch;
impl RfSwitch for NoopRfSwitch {
    fn set(&mut self, _state: RfState) { }
}

// Program metadata for `picotool info`.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"MARV-FC Radio"),
    embassy_rp::binary_info::rp_program_description!(
        c"MARV-FC dedicated radio firmware using SX1262 on RP2350A Pico 2"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("MARV-FC Radio: boot");

    // LED for heartbeat
    let mut led = Output::new(p.PIN_16, Level::Low);

    // SPI wiring for SX1262 (assumes same pins as Core1262-HF-like module).
    // Adjust pins to match actual hardware once confirmed.
    // Per docs/hardware.md: SPI0 CLK=GP2, MOSI=GP3, MISO=GP4
    let sck = p.PIN_2;
    let mosi = p.PIN_3;
    let miso = p.PIN_4;

    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 10_000_000; // 10 MHz typical for SX1262

    let spi = Spi::new(
        p.SPI0,
        sck,
        mosi,
        miso,
        p.DMA_CH0,
        p.DMA_CH1,
        spi_cfg,
    );

    // Control pins per docs/hardware.md:
    // CS/NSS=GP5, RESET=GP1, BUSY=GP0, DIO1=GP6, DIO2=GP7
    let nss = Output::new(p.PIN_5, Level::High);
    let reset = Output::new(p.PIN_1, Level::High);
    let busy = Input::new(p.PIN_0, Pull::Down);
    let dio1 = Input::new(p.PIN_6, Pull::None);
    let dio2 = Input::new(p.PIN_7, Pull::None);

    let rf_switch = NoopRfSwitch;

    let cfg = Sx1262Config::default();

    let mut radio = Sx1262::new(
        spi,
        nss,
        reset,
        busy,
        dio1,
        dio2,
        rf_switch,
        RfSwitchMode::Dio2,
        cfg,
    );

    let mut delay = EmbassyDelay;

    // Example LoRa center frequency (e.g., 868 MHz); adjust per region.
    let center_freq_hz: u32 = 868_000_000;

    match radio.init_lora(&mut delay, center_freq_hz).await {
        Ok(_) => info!("SX1262: init OK at {} Hz", center_freq_hz),
        Err(e) => warn!("SX1262: init failed: {:?}", e),
    }

    // Simple heartbeat loop; radio is left in configured state.
    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(900).await;
    }
}
