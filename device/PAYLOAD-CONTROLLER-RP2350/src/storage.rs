use common::interfaces::storage::LogError;
pub use rp235x_base::storage::StorageLoggerEngine;
use rp235x_base::storage::{
    MicrosdStorageConfig, StoragePins as BaseStoragePins,
    build_logger_engine as build_base_logger_engine,
};

use crate::buses::StorageSpiBus;
use crate::config::LoggingConfig;
use crate::resources::StoragePins;

pub fn build_logger_engine(
    pins: StoragePins,
    bus: StorageSpiBus,
    config: LoggingConfig,
) -> Result<StorageLoggerEngine, LogError> {
    build_base_logger_engine(
        BaseStoragePins {
            sck: pins.sck,
            mosi: pins.mosi,
            miso: pins.miso,
            cs: pins.cs,
        },
        bus.spi,
        MicrosdStorageConfig {
            sd_spi_frequency_hz: config.sd_spi_frequency_hz,
            sd_flush_every_lines: config.sd_flush_every_lines,
        },
    )
}
