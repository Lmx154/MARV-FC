#![no_std]
#![no_main]

mod buses;
mod channels;
mod clocks;
mod config;
mod core0;
mod core1;
mod pinmap;
mod resources;
mod storage;
mod usb_cdc;
mod watchdog;

use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use {defmt_rtt as _, panic_probe as _};

// RP235x uses an IMAGE_DEF block in flash instead of an RP2040-style boot2 blob.
#[unsafe(link_section = ".start_block")]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(clocks::hal_config());
    let resources = resources::split(peripherals);
    core0::run(spawner, resources).await;
}
