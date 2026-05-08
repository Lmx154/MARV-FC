#![allow(dead_code)]

use common::drivers::bmp388::BMP388_ADDR_SDO_LOW;
use common::drivers::pressure_transducer::PressureTransducerConfig;
use common::services::logging::{
    SensorSnapshotLogFlags, SensorSnapshotLoggerConfig, SensorSnapshotSensorConfig,
};
use common::utilities::units::STANDARD_SEA_LEVEL_PRESSURE_PA;

pub const STATUS_HEARTBEAT_PERIOD_MS: u64 = 1_000;
pub const WATCHDOG_TIMEOUT_MS: u32 = 250;
pub const WATCHDOG_ENABLED_IN_HIL: bool = false;
pub const BMP388_I2C_FREQUENCY_HZ: u32 = 400_000;
pub const BMP388_SAMPLE_PERIOD_MS: u32 = 50;
pub const PRESSURE_TRANSDUCER_SAMPLE_PERIOD_MS: u32 = 50;
pub const SERVO_PWM_FRAME_PERIOD_US: u32 = 20_000;
pub const SERVO_PWM_DIVIDER: u8 = 64;
pub const SERVO_CLOSED_ANGLE_DEG: u16 = 0;
pub const SERVO_OPEN_ANGLE_DEG: u16 = 90;
pub const SERVO_TRIGGER_ALTITUDE_FT: u32 = 8_000;
pub const SERVO_SEA_LEVEL_PRESSURE_PA: u32 = STANDARD_SEA_LEVEL_PRESSURE_PA as u32;
pub const LOG_FILE_PREFIX: &str = "PAYL";
pub const LOG_RECORD_PERIOD_MS: u32 = 10;
pub const LOG_SD_SPI_FREQUENCY_HZ: u32 = 12_000_000;
pub const LOG_SD_FLUSH_EVERY_LINES: usize = 32;
pub const PAYLOAD_SERVO_TEST_COMMAND_ID: u16 = 31_000;
pub const HIL_SYSTEM_ID: u8 = 42;
pub const HIL_COMPONENT_ID: u8 = 1;
pub const HIL_BOOT_WINDOW_MS: u32 = 1_500;

#[derive(Clone, Copy, Debug)]
pub struct Bmp388RuntimeConfig {
    pub address: u8,
    pub i2c_frequency_hz: u32,
    pub period_ms: u32,
}

impl Default for Bmp388RuntimeConfig {
    fn default() -> Self {
        Self {
            address: BMP388_ADDR_SDO_LOW,
            i2c_frequency_hz: BMP388_I2C_FREQUENCY_HZ,
            period_ms: BMP388_SAMPLE_PERIOD_MS,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PressureTransducerRuntimeConfig {
    pub enabled: bool,
    pub period_ms: u32,
    pub sensor: PressureTransducerConfig,
}

impl Default for PressureTransducerRuntimeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            period_ms: PRESSURE_TRANSDUCER_SAMPLE_PERIOD_MS,
            sensor: PressureTransducerConfig::default(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ServoRuntimeConfig {
    pub enabled: bool,
    pub frame_period_us: u32,
    pub pwm_divider: u8,
    pub closed_angle_deg: u16,
    pub open_angle_deg: u16,
    pub trigger_altitude_ft: u32,
    pub sea_level_pressure_pa: u32,
}

impl Default for ServoRuntimeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            frame_period_us: SERVO_PWM_FRAME_PERIOD_US,
            pwm_divider: SERVO_PWM_DIVIDER,
            closed_angle_deg: SERVO_CLOSED_ANGLE_DEG,
            open_angle_deg: SERVO_OPEN_ANGLE_DEG,
            trigger_altitude_ft: SERVO_TRIGGER_ALTITUDE_FT,
            sea_level_pressure_pa: SERVO_SEA_LEVEL_PRESSURE_PA,
        }
    }
}

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
                    imu: false,
                    aux_imu: false,
                    barometer: true,
                    pressure_transducer: true,
                    magnetometer: false,
                    gps: false,
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
    pub status_heartbeat_period_ms: u64,
    pub bmp388: Bmp388RuntimeConfig,
    pub hil: HilConfig,
    pub hil_boot_window_ms: u32,
    pub watchdog_enabled_in_hil: bool,
    pub watchdog_timeout_ms: u32,
    pub pressure_transducer: PressureTransducerRuntimeConfig,
    pub servo: ServoRuntimeConfig,
    pub logging: LoggingConfig,
}

#[derive(Clone, Copy, Debug)]
pub struct HilConfig {
    pub system_id: u8,
    pub component_id: u8,
    pub boot_window_ms: u32,
    pub payload_servo_test_command_id: u16,
}

impl Default for HilConfig {
    fn default() -> Self {
        Self {
            system_id: HIL_SYSTEM_ID,
            component_id: HIL_COMPONENT_ID,
            boot_window_ms: HIL_BOOT_WINDOW_MS,
            payload_servo_test_command_id: PAYLOAD_SERVO_TEST_COMMAND_ID,
        }
    }
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            status_heartbeat_period_ms: STATUS_HEARTBEAT_PERIOD_MS,
            bmp388: Bmp388RuntimeConfig::default(),
            hil: HilConfig::default(),
            hil_boot_window_ms: HIL_BOOT_WINDOW_MS,
            watchdog_enabled_in_hil: WATCHDOG_ENABLED_IN_HIL,
            watchdog_timeout_ms: WATCHDOG_TIMEOUT_MS,
            pressure_transducer: PressureTransducerRuntimeConfig::default(),
            servo: ServoRuntimeConfig::default(),
            logging: LoggingConfig::default(),
        }
    }
}
