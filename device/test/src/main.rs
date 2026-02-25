#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use common::sd::{HalSdSpi, SdStore};
use common::tasks::sd::run_sd_hello_world_task;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Config as SpiConfig, Spi};

#[path = "../../rp235x/src/hardware/pinout.rs"]
mod rp235x_pinout;

use rp235x_pinout::fc;

const _: () = {
    core::assert!(fc::sd_spi0::MOSI == 19);
    core::assert!(fc::sd_spi0::MISO == 20);
    core::assert!(fc::sd_spi0::SCK == 18);
    core::assert!(fc::sd_spi0::CS == 21);
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("test: sd spi trait abstraction");

    let p = embassy_rp::init(Default::default());

    let mut cfg = SpiConfig::default();
    cfg.frequency = 1_000_000;

    let spi = Spi::new_blocking(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_20, cfg);
    let cs = Output::new(p.PIN_21, Level::High);
    let sd = HalSdSpi::new(spi, cs);
    let mut store = SdStore::new(sd);

    if let Err(e) = store.init() {
        defmt::panic!("sd init failed: {:?}", e);
    }

    if let Err(e) = run_sd_hello_world_task(&mut store).await {
        defmt::panic!("sd hello task failed: {:?}", e);
    }
    info!("sd hello task complete");

    loop {
        cortex_m::asm::wfi();
    }
}
