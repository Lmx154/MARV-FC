#![allow(dead_code)]

use common::drivers::bmp390::BMP390_ADDR_SDO_LOW;
use common::drivers::mpu6050::{AccelRange, GyroRange, MPU6050_ADDR_AD0_LOW};
use common::services::logging::{
    SensorSnapshotLogFlags, SensorSnapshotLoggerConfig, SensorSnapshotSensorConfig,
};

pub const STATUS_HEARTBEAT_PERIOD_MS: u64 = 1_000;
pub const BMP390_I2C_FREQUENCY_HZ: u32 = 400_000;
pub const BMP390_SAMPLE_PERIOD_MS: u32 = 50;
pub const MPU6050_SAMPLE_PERIOD_MS: u32 = 10;
pub const LOG_FILE_PREFIX: &str = "ALTI";
pub const LOG_RECORD_PERIOD_MS: u32 = 100;
pub const LOG_SD_SPI_FREQUENCY_HZ: u32 = 12_000_000;
pub const LOG_SD_FLUSH_EVERY_LINES: usize = 32;

#[derive(Clone, Copy, Debug)]
pub struct Bmp390RuntimeConfig {
    pub address: u8,
    pub i2c_frequency_hz: u32,
    pub period_ms: u32,
}

impl Default for Bmp390RuntimeConfig {
    fn default() -> Self {
        Self {
            address: BMP390_ADDR_SDO_LOW,
            i2c_frequency_hz: BMP390_I2C_FREQUENCY_HZ,
            period_ms: BMP390_SAMPLE_PERIOD_MS,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Mpu6050RuntimeConfig {
    pub address: u8,
    pub accel_range: AccelRange,
    pub gyro_range: GyroRange,
    pub period_ms: u32,
}

impl Default for Mpu6050RuntimeConfig {
    fn default() -> Self {
        Self {
            address: MPU6050_ADDR_AD0_LOW,
            accel_range: AccelRange::G16,
            gyro_range: GyroRange::Dps2000,
            period_ms: MPU6050_SAMPLE_PERIOD_MS,
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
                    imu: true,
                    aux_imu: false,
                    barometer: true,
                    pressure_transducer: false,
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
    pub bmp390: Bmp390RuntimeConfig,
    pub mpu6050: Mpu6050RuntimeConfig,
    pub logging: LoggingConfig,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            status_heartbeat_period_ms: STATUS_HEARTBEAT_PERIOD_MS,
            bmp390: Bmp390RuntimeConfig::default(),
            mpu6050: Mpu6050RuntimeConfig::default(),
            logging: LoggingConfig::default(),
        }
    }
}
