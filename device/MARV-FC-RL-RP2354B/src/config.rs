#![allow(dead_code)]

use common::services::logging::{
    SensorSnapshotLogFlags, SensorSnapshotLoggerConfig, SensorSnapshotSensorConfig,
};

pub const XOSC_HZ: u32 = 12_000_000;
pub const FAST_LOOP_HZ: u32 = 1_000;
pub const WATCHDOG_TIMEOUT_MS: u32 = 250;
pub const STATUS_HEARTBEAT_PERIOD_MS: u64 = 1_000;
pub const LOG_FILE_PREFIX: &str = "FLGT";
pub const LOG_RECORD_PERIOD_MS: u32 = 10;
pub const LOG_SD_SPI_FREQUENCY_HZ: u32 = 12_000_000;
pub const LOG_SD_FLUSH_EVERY_LINES: usize = 64;

#[derive(Clone, Copy, Debug)]
pub struct LoggingConfig {
    pub enabled: bool,
    pub file_prefix: &'static str,
    pub sd_spi_frequency_hz: u32,
    pub sd_flush_every_lines: usize,
    pub sensor_snapshot: SensorSnapshotLoggerConfig,
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            file_prefix: LOG_FILE_PREFIX,
            sd_spi_frequency_hz: LOG_SD_SPI_FREQUENCY_HZ,
            sd_flush_every_lines: LOG_SD_FLUSH_EVERY_LINES,
            sensor_snapshot: SensorSnapshotLoggerConfig {
                period_ms: LOG_RECORD_PERIOD_MS,
                emit_header: true,
                sensors: SensorSnapshotSensorConfig {
                    imu: true,
                    aux_imu: true,
                    barometer: true,
                    pressure_transducer: false,
                    magnetometer: true,
                    gps: true,
                },
                flags: SensorSnapshotLogFlags {
                    include_sink_state: true,
                    include_sensor_state: true,
                    include_sample_timestamps: true,
                    include_lag_counters: true,
                    mark_stale_data: true,
                },
            },
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DeviceConfig {
    pub fast_loop_hz: u32,
    pub watchdog_timeout_ms: u32,
    pub status_heartbeat_period_ms: u64,
    pub logging: LoggingConfig,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            fast_loop_hz: FAST_LOOP_HZ,
            watchdog_timeout_ms: WATCHDOG_TIMEOUT_MS,
            status_heartbeat_period_ms: STATUS_HEARTBEAT_PERIOD_MS,
            logging: LoggingConfig::default(),
        }
    }
}
