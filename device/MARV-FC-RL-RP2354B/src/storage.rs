use common::drivers::storage::{MicrosdLogger, MicrosdLoggerConfig};
use common::interfaces::storage::{LogError, LogPath};
use common::tasks::slow_loop::run_sd_card_smoke_test;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::sdcard::{DummyCsPin, SdCard};
use embedded_sdmmc::{TimeSource, Timestamp};

use crate::buses::StorageSpiBus;
use crate::resources::StoragePins;

pub fn run_startup_smoke_test(
    pins: StoragePins,
    bus: StorageSpiBus,
) -> Result<LogPath, LogError> {
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = 1_000_000;

    let spi = Spi::new_blocking(bus.spi, pins.sck, pins.mosi, pins.miso, spi_config);
    let cs = Output::new(pins.cs, Level::High);
    let spi_device =
        ExclusiveDevice::new(spi, DummyCsPin, SdDelay).map_err(|_| LogError::Device)?;
    let sdcard = SdCard::new(spi_device, cs, SdDelay);

    let logger_config = MicrosdLoggerConfig::default();
    let mut logger = MicrosdLogger::<_, _, 8, 4, 1>::new(
        sdcard,
        FixedTimeSource,
        logger_config,
    )?;
    run_sd_card_smoke_test(&mut logger)
}

struct FixedTimeSource;

impl TimeSource for FixedTimeSource {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

struct SdDelay;

impl embedded_hal::delay::DelayNs for SdDelay {
    fn delay_ns(&mut self, ns: u32) {
        let micros = (ns as u64).div_ceil(1_000);
        embassy_time::block_for(embassy_time::Duration::from_micros(micros.max(1)));
    }
}
