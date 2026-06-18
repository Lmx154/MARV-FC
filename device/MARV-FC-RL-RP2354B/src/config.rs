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
// SD-card data clock used AFTER the card is initialised at 400 kHz (see the
// two-stage clocking in `rp235x_base::storage::build_logger_engine`). 4 MHz is
// ~12x the bandwidth a 100 Hz all-sensor CSV needs (~40 KB/s) while staying
// conservative for the PCB's SD signal integrity. Raise toward 12-25 MHz if the
// board's routing is clean and you want more headroom for flush stalls.
pub const LOG_SD_SPI_FREQUENCY_HZ: u32 = 4_000_000;
// How many CSV rows to buffer between durable flushes. In embedded-sdmmc 0.7.0 a
// "flush" is a full file close+reopen (no `flush_file` API exists), which is an
// expensive multi-block blocking SD op. At 8 it fired every ~80 ms and stalled
// the sink long enough to back-pressure the LOG_CHANNEL and drop ~1/3 of rows
// (effective ~66 Hz vs the 100 Hz target). 32 amortizes that cost over ~4x more
// rows — ~2x throughput headroom, no back-pressure — while bounding power-loss
// data loss to ~320 ms (32 rows @ 100 Hz) of uncommitted tail. Lower it for
// tighter crash-durability (at the cost of throughput); raise toward 64 for more
// headroom on slow cards.
pub const LOG_SD_FLUSH_EVERY_LINES: usize = 32;
// Async settle delay before touching the SD card, giving its supply rail time to
// come up after boot. Runs on `Timer::after` (yields), so the watchdog and
// feed-critical sensor tasks keep running during it.
pub const LOG_SD_STARTUP_DELAY_MS: u64 = 1_000;
pub const HIL_SYSTEM_ID: u8 = 42;
pub const HIL_COMPONENT_ID: u8 = 1;
pub const BMP581_I2C_FREQUENCY_HZ: u32 = 400_000;
pub const BMP581_PERIOD_MS: u32 = 20;
pub const BMM350_I2C_FREQUENCY_HZ: u32 = 400_000;
pub const BMM350_PERIOD_MS: u32 = 40;
pub const RADIO_LINK_UART_BAUD: u32 = 460_800;
pub const RADIO_LINK_UART_BUFFER_BYTES: usize = 512;
pub const GPS_UART_BUFFER_BYTES: usize = 256;
pub const RADIO_LINK_RX_LED_PULSE_MS: u64 = 120;
pub const RADIO_LINK_STATUS_PERIOD_MS: u32 = 1_000;
// FC→radio UART emit cadences. The radio caches the latest of each class and re-paces the
// air link by its own airtime budget, so these only need to keep the cache fresh without
// flooding the UART/priority queues — emitting far faster than the air rate just gets dropped.
pub const RADIO_LINK_TELEMETRY_SNAPSHOT_PERIOD_MS: u32 = 50;
pub const RADIO_LINK_GPS_PERIOD_MS: u32 = 1_000;
pub const RADIO_LINK_IMU_PERIOD_MS: u32 = 50;
pub const RADIO_LINK_MAG_PERIOD_MS: u32 = 100;
pub const RADIO_LINK_BARO_PERIOD_MS: u32 = 100;

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
    pub bmp581: Bmp581RuntimeConfig,
    pub bmm350: Bmm350RuntimeConfig,
    pub hil: HilConfig,
    pub mission: MissionConfig,
    pub fast_loop_hz: u32,
    pub watchdog_enabled_in_hil: bool,
    pub watchdog_timeout_ms: u32,
    pub status_heartbeat_period_ms: u64,
    pub logging: LoggingConfig,
}

#[derive(Clone, Copy, Debug)]
pub struct HilConfig {
    pub system_id: u8,
    pub component_id: u8,
}

impl Default for HilConfig {
    fn default() -> Self {
        Self {
            system_id: HIL_SYSTEM_ID,
            component_id: HIL_COMPONENT_ID,
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

#[derive(Clone, Copy, Debug)]
pub struct Bmm350RuntimeConfig {
    pub enabled: bool,
    pub i2c_frequency_hz: u32,
    pub period_ms: u32,
}

impl Default for Bmm350RuntimeConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            i2c_frequency_hz: BMM350_I2C_FREQUENCY_HZ,
            period_ms: BMM350_PERIOD_MS,
        }
    }
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
            bmm350: Bmm350RuntimeConfig::default(),
            hil: HilConfig::default(),
            mission: MissionConfig::default(),
            fast_loop_hz: FAST_LOOP_HZ,
            watchdog_enabled_in_hil: WATCHDOG_ENABLED_IN_HIL,
            watchdog_timeout_ms: WATCHDOG_TIMEOUT_MS,
            status_heartbeat_period_ms: STATUS_HEARTBEAT_PERIOD_MS,
            logging: LoggingConfig::default(),
        }
    }
}
