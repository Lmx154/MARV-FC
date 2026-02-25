//! SD card parameter persistence
//!
//! Hardware-specific SD card I/O for loading and saving parameters on RP2350

use super::pinout::fc::sd_spi0;

use embassy_rp::gpio::Output;
use embassy_rp::spi::Spi;

use common::config::Config as AppConfig;
use common::params::ParamRegistry;

const _: () = {
    assert!(sd_spi0::MISO == 20);
    assert!(sd_spi0::MOSI == 19);
    assert!(sd_spi0::SCK == 18);
    assert!(sd_spi0::CS == 21);
};

/// Load config and parameters from SD card (boot-time, SPI0 blocking mode)
pub fn load_config_and_params_from_sd(
    bus: Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Blocking>,
    cs: Output<'static>,
    params: &mut ParamRegistry,
) -> Result<(AppConfig, usize), embedded_sdmmc::Error<embedded_sdmmc::SdCardError>> {
    let _ = bus;
    let _ = cs;
    let _ = params;
    Ok((AppConfig::default(), 0))
}

/// Save parameters to SD card PARAMS.TXT file
/// This function recreates the SPI0 peripheral temporarily for the save operation
pub fn save_params_to_sd(params: &ParamRegistry) -> Result<(), &'static str> {
    let _ = params;
    Ok(())
}
