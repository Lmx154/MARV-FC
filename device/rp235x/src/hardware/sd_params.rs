//! SD card parameter persistence
//! 
//! Hardware-specific SD card I/O for loading and saving parameters on RP2350

use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{
    sdcard::DummyCsPin, Mode as FsMode, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};

use common::config::Config as AppConfig;
use common::params::ParamRegistry;

/// SD card time source (dummy for now)
pub struct SdTime;

impl TimeSource for SdTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 54,
            zero_indexed_month: 11,
            zero_indexed_day: 17,
            hours: 12,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// SD card delay implementation
pub struct SdDelay;

impl embedded_hal::delay::DelayNs for SdDelay {
    fn delay_ns(&mut self, ns: u32) {
        embassy_time::block_for(embassy_time::Duration::from_micros((ns / 1000).max(1) as u64));
    }
}

/// Load config and parameters from SD card (boot-time, SPI0 blocking mode)
pub fn load_config_and_params_from_sd(
    bus: Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Blocking>,
    cs: Output<'static>,
    params: &mut ParamRegistry,
) -> Result<(AppConfig, usize), embedded_sdmmc::Error<embedded_sdmmc::SdCardError>> {
    let spi_dev = ExclusiveDevice::new(bus, DummyCsPin, SdDelay).unwrap();
    let sdcard = embedded_sdmmc::sdcard::SdCard::new(spi_dev, cs, SdDelay);
    let mut volume_mgr: VolumeManager<_, _> = VolumeManager::new(sdcard, SdTime);

    // Load config first
    let config = {
        let mut volume0 = volume_mgr.open_volume(VolumeIdx(0))?;
        let mut root_dir = volume0.open_root_dir()?;

        let config_result = root_dir.open_file_in_dir("CONFIG.TXT", FsMode::ReadOnly);
        let config = match config_result {
            Ok(mut file) => {
                let mut buf = [0u8; 256];
                let n = file.read(&mut buf)?;
                drop(file);
                AppConfig::from_bytes(&buf[..n])
            }
            Err(_) => {
                drop(config_result);
                // Create default config file
                let defaults = AppConfig::default();
                let mut file =
                    root_dir.open_file_in_dir("CONFIG.TXT", FsMode::ReadWriteCreateOrTruncate)?;
                let bytes = defaults.to_bytes();
                file.write(bytes.as_bytes())?;
                drop(file);
                defaults
            }
        };
        config
    };

    // Load parameters second
    let param_count = load_params_from_sd(&mut volume_mgr, params)?;

    Ok((config, param_count))
}

/// Load parameters from SD card PARAMS.TXT file
fn load_params_from_sd(
    volume_mgr: &mut VolumeManager<
        embedded_sdmmc::sdcard::SdCard<
            ExclusiveDevice<
                Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Blocking>,
                DummyCsPin,
                SdDelay,
            >,
            Output<'static>,
            SdDelay,
        >,
        SdTime,
    >,
    params: &mut ParamRegistry,
) -> Result<usize, embedded_sdmmc::Error<embedded_sdmmc::SdCardError>> {
    let mut volume0 = volume_mgr.open_volume(VolumeIdx(0))?;
    let mut root_dir = volume0.open_root_dir()?;

    let res = root_dir.open_file_in_dir("PARAMS.TXT", FsMode::ReadOnly);
    if let Ok(mut file) = res {
        let mut buf = [0u8; 2048]; // Enough for ~40 parameters
        let n = file.read(&mut buf)?;
        if let Ok(count) = params.deserialize_from_text(&buf[..n]) {
            return Ok(count);
        }
    }

    Ok(0) // No file found or parse error - use defaults
}

/// Save parameters to SD card PARAMS.TXT file
/// This function recreates the SPI0 peripheral temporarily for the save operation
pub fn save_params_to_sd(params: &ParamRegistry) -> Result<(), &'static str> {
    // Safety: We're temporarily taking the SPI0 peripheral
    // This only works because we know the SPI0 is not being used elsewhere
    // The peripheral will be consumed and dropped after this function
    let p = unsafe { embassy_rp::Peripherals::steal() };

    // Reconfigure SPI0 pins for SD card access
    let sd_miso = p.PIN_20;
    let sd_mosi = p.PIN_19;
    let sd_sck = p.PIN_18;
    let sd_cs = Output::new(p.PIN_21, Level::High);

    let mut sd_spi_cfg = SpiConfig::default();
    sd_spi_cfg.frequency = 1_000_000;

    let sd_bus = Spi::new_blocking(p.SPI0, sd_sck, sd_mosi, sd_miso, sd_spi_cfg);

    let spi_dev = ExclusiveDevice::new(sd_bus, DummyCsPin, SdDelay).unwrap();
    let sdcard = embedded_sdmmc::sdcard::SdCard::new(spi_dev, sd_cs, SdDelay);
    let mut volume_mgr: VolumeManager<_, _> = VolumeManager::new(sdcard, SdTime);
    let mut volume0 = volume_mgr
        .open_volume(VolumeIdx(0))
        .map_err(|_| "Failed to open volume")?;
    let mut root_dir = volume0
        .open_root_dir()
        .map_err(|_| "Failed to open root dir")?;

    let mut buf = [0u8; 2048];
    let len = params
        .serialize_to_text(&mut buf)
        .map_err(|_| "Failed to serialize params")?;

    let mut file = root_dir
        .open_file_in_dir("PARAMS.TXT", FsMode::ReadWriteCreateOrTruncate)
        .map_err(|_| "Failed to open PARAMS.TXT")?;
    file.write(&buf[..len])
        .map_err(|_| "Failed to write params")?;

    Ok(())
}
