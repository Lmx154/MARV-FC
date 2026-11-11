//! Skeleton firmware: proves that every device driver crate is linked, touches no bus.
#![no_std]
#![no_main]

use core::any::type_name;
use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;

/// Logs the verdict and ends the `probe-rs run` session through semihosting SYS_EXIT.
/// The host exit status is 0 when `failures == 0`, otherwise 1.
fn exit(failures: u32) -> ! {
    if failures == 0 {
        defmt::info!("RESULT: PASS");
    } else {
        defmt::error!("RESULT: FAIL ({} failed checks)", failures);
    }
    semihosting::process::exit(if failures == 0 { 0 } else { 1 })
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_rp::init(Default::default());
    defmt::info!("[INFO] icm45686: {}", type_name::<icm45686::Info>());
    defmt::info!("[INFO] lsm6dsk320x: {}", type_name::<lsm6dsk320x::Info>());
    defmt::info!("[INFO] bmp581: {}", type_name::<bmp581::Info>());
    defmt::info!("[INFO] bmm350: {}", type_name::<bmm350::Info>());
    defmt::info!("[INFO] lc76g: {}", type_name::<lc76g::Config>());
    defmt::info!("[INFO] pio-sdmmc: {}", type_name::<pio_sdmmc::Config>());
    defmt::info!("[INFO] qmi-psram: {}", type_name::<qmi_psram::Config>());
    defmt::info!("[INFO] ws2812c: {}", type_name::<ws2812c::Rgb>());
    defmt::info!("[INFO] alive");
    exit(0)
}
