#![allow(dead_code)]

use common::services::logging::{
    SensorSnapshotLogFlags, SensorSnapshotLoggerConfig, SensorSnapshotSensorConfig,
};

pub const XOSC_HZ: u32 = 12_000_000;
pub const FAST_LOOP_HZ: u32 = 1_000;
pub const WATCHDOG_TIMEOUT_MS: u32 = 250;
pub const STATUS_HEARTBEAT_PERIOD_MS: u64 = 1_000;
pub const LOG_PATH: &str = "flight.csv";
pub const LOG_RECORD_PERIOD_MS: u32 = 10;

#[derive(Clone, Copy, Debug)]
pub struct LoggingConfig {
    pub enabled: bool,
    pub path: &'static str,
    pub sensor_snapshot: SensorSnapshotLoggerConfig,
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            path: LOG_PATH,
            sensor_snapshot: SensorSnapshotLoggerConfig {
                period_ms: LOG_RECORD_PERIOD_MS,
                emit_header: true,
                sensors: SensorSnapshotSensorConfig {
                    imu: true,
                    barometer: true,
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
