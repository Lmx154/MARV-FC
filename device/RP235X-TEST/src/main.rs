#![no_std]
#![no_main]

mod buses;
mod dshot;
mod gps;
mod pinmap;
mod protocol;
mod resources;

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use {defmt_rtt as _, panic_probe as _};

// RP235x uses an IMAGE_DEF block in flash instead of an RP2040-style boot2 blob.
#[unsafe(link_section = ".start_block")]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());
    let resources = resources::split(peripherals);

    info!("rp235x-test firmware booted");

    protocol::spawn(&spawner, resources.system.usb);
    dshot::spawn(&spawner, resources.buses.dshot, resources.pins.actuators);
    gps::spawn(
        &spawner,
        resources.buses.gps_pio_uart,
        resources.pins.gps_pio_uart,
    );
}
