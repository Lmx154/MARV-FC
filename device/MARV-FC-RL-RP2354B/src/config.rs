#![allow(dead_code)]

use common::drivers::bmp581::{BMP581_ADDR_PRIMARY, Bmp581Config};
use common::messages::control::RgbLedCommand;
use common::policies::mission::BarometerRgbLedMissionConfig;
use common::services::logging::{
    SensorSnapshotLogFlags, SensorSnapshotLoggerConfig, SensorSnapshotSensorConfig,
};

pub const XOSC_HZ: u32 = 12_000_000;
pub const FAST_LOOP_HZ: u32 = 1_000;
pub const WATCHDOG_TIMEOUT_MS: u32 = 250;
pub const WATCHDOG_ENABLED_IN_HIL: bool = false;
pub const STATUS_HEARTBEAT_PERIOD_MS: u64 = 1_000;
pub const LOG_FILE_PREFIX: &str = "FLGT";
pub const LOG_RECORD_PERIOD_MS: u32 = 10;
pub const LOG_SD_SPI_FREQUENCY_HZ: u32 = 100_000;
pub const LOG_SD_FLUSH_EVERY_LINES: usize = 64;
pub const LOG_SD_STARTUP_DELAY_MS: u64 = 5_000;
pub const LOG_SD_INIT_ATTEMPTS: u32 = 4;
pub const LOG_SD_INIT_RETRY_BACKOFF_MS: u64 = 250;
pub const HIL_SYSTEM_ID: u8 = 42;
pub const HIL_COMPONENT_ID: u8 = 1;
pub const HIL_BOOT_WINDOW_MS: u32 = 1_500;
pub const BMP581_I2C_FREQUENCY_HZ: u32 = 400_000;
pub const BMP581_PERIOD_MS: u32 = 20;

#[derive(Clone, Copy, Debug)]
pub struct LoggingConfig {
    pub enabled: bool,
    pub file_prefix: &'static str,
    pub sd_spi_frequency_hz: u32,
    pub sd_flush_every_lines: usize,
    pub sd_startup_delay_ms: u64,
    pub sd_init_attempts: u32,
    pub sd_init_retry_backoff_ms: u64,
    pub sensor_snapshot: SensorSnapshotLoggerConfig,
}

impl Default for LoggingConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            file_prefix: LOG_FILE_PREFIX,
            sd_spi_frequency_hz: LOG_SD_SPI_FREQUENCY_HZ,
            sd_flush_every_lines: LOG_SD_FLUSH_EVERY_LINES,
            sd_startup_delay_ms: LOG_SD_STARTUP_DELAY_MS,
            sd_init_attempts: LOG_SD_INIT_ATTEMPTS,
            sd_init_retry_backoff_ms: LOG_SD_INIT_RETRY_BACKOFF_MS,
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
    pub bmp581: Bmp581RuntimeConfig,
    pub hil: HilConfig,
    pub mission: MissionConfig,
    pub fast_loop_hz: u32,
    pub hil_boot_window_ms: u32,
    pub watchdog_enabled_in_hil: bool,
    pub watchdog_timeout_ms: u32,
    pub status_heartbeat_period_ms: u64,
    pub logging: LoggingConfig,
}

#[derive(Clone, Copy, Debug)]
pub struct HilConfig {
    pub system_id: u8,
    pub component_id: u8,
    pub boot_window_ms: u32,
}

impl Default for HilConfig {
    fn default() -> Self {
        Self {
            system_id: HIL_SYSTEM_ID,
            component_id: HIL_COMPONENT_ID,
            boot_window_ms: HIL_BOOT_WINDOW_MS,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Bmp581RuntimeConfig {
    pub enabled: bool,
    pub address: u8,
    pub i2c_frequency_hz: u32,
    pub period_ms: u32,
    pub driver_config: Bmp581Config,
}

impl Default for Bmp581RuntimeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            address: BMP581_ADDR_PRIMARY,
            i2c_frequency_hz: BMP581_I2C_FREQUENCY_HZ,
            period_ms: BMP581_PERIOD_MS,
            driver_config: Bmp581Config::default(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct MissionConfig {
    pub altitude_led_latch: AltitudeLedLatchMissionConfig,
}

impl Default for MissionConfig {
    fn default() -> Self {
        Self {
            altitude_led_latch: AltitudeLedLatchMissionConfig::default(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AltitudeLedLatchMissionConfig {
    pub enabled: bool,
    pub mission: BarometerRgbLedMissionConfig,
}

impl Default for AltitudeLedLatchMissionConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            mission: BarometerRgbLedMissionConfig::new(
                5_000.0,
                common::utilities::units::STANDARD_SEA_LEVEL_PRESSURE_PA,
                RgbLedCommand::new(24, 24, 24),
            ),
        }
    }
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            bmp581: Bmp581RuntimeConfig::default(),
            hil: HilConfig::default(),
            mission: MissionConfig::default(),
            fast_loop_hz: FAST_LOOP_HZ,
            hil_boot_window_ms: HIL_BOOT_WINDOW_MS,
            watchdog_enabled_in_hil: WATCHDOG_ENABLED_IN_HIL,
            watchdog_timeout_ms: WATCHDOG_TIMEOUT_MS,
            status_heartbeat_period_ms: STATUS_HEARTBEAT_PERIOD_MS,
            logging: LoggingConfig::default(),
        }
    }
}
