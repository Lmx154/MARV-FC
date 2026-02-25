#![no_std]
#![no_main]

mod app;

use embassy_executor::Spawner;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    app::spawn_all(spawner).await;
}
