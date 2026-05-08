#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

// RP235x uses an IMAGE_DEF block in flash instead of an RP2040-style boot2 blob.
#[unsafe(link_section = ".start_block")]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());
    let _core1 = peripherals.CORE1;

    info!("RP2354B cross-core test harness booted");
    info!("CORE1 handle reserved for upcoming experiments");

    loop {
        Timer::after(Duration::from_secs(1)).await;
        info!("cross-core test harness heartbeat");
    }
}
