use std::fmt;
use std::fs::{self, File, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

use embassy_sync::blocking_mutex::raw::RawMutex;

use common::interfaces::storage::{
    LogEntryList, LogError, LogLine, LogLineList, LogPath, LoggerEngine, MAX_LOG_PREFIX_LEN,
};
use common::messages::sensor::TimeSample;
use common::services::acquisition::{
    BarometerSampleChannel, BarometerSampleSubscriber, GpsFixSampleChannel, GpsFixSampleSubscriber,
    ImuSampleChannel, ImuSampleSubscriber,
};
use common::services::logging::{
    SensorSnapshotLogFlags, SensorSnapshotLogger, SensorSnapshotLoggerConfig,
    SensorSnapshotLoggerError, SensorSnapshotSensorConfig,
};

const DEFAULT_LOG_PREFIX: &str = "FLGT";

pub struct FsLogger<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>
where
    M: RawMutex + 'static,
{
    engine: HostFileLoggerEngine,
    sensor_snapshot: SensorSnapshotLogger,
    imu_subscriber: ImuSampleSubscriber<'static, M, DEPTH, SUBS, PUBS>,
    barometer_subscriber: BarometerSampleSubscriber<'static, M, DEPTH, SUBS, PUBS>,
    gps_subscriber: GpsFixSampleSubscriber<'static, M, DEPTH, SUBS, PUBS>,
    last_emit_us: Option<u64>,
}

impl<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> FsLogger<M, DEPTH, SUBS, PUBS>
where
    M: RawMutex + 'static,
{
    pub fn new(
        log_dir: &Path,
        imu_channel: &'static ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
        barometer_channel: &'static BarometerSampleChannel<M, DEPTH, SUBS, PUBS>,
        gps_channel: &'static GpsFixSampleChannel<M, DEPTH, SUBS, PUBS>,
    ) -> Result<Self, FsLoggerError> {
        let mut engine = HostFileLoggerEngine::new(log_dir);
        let path = engine.create_new_csv(DEFAULT_LOG_PREFIX)?;
        let sensor_snapshot = SensorSnapshotLogger::new(
            path.as_str(),
            SensorSnapshotLoggerConfig {
                period_ms: 10,
                emit_header: true,
                sensors: SensorSnapshotSensorConfig {
                    imu: true,
                    aux_imu: false,
                    barometer: true,
                    pressure_transducer: false,
                    magnetometer: false,
                    gps: true,
                },
                flags: SensorSnapshotLogFlags::default(),
            },
        )?;

        Ok(Self {
            engine,
            sensor_snapshot,
            imu_subscriber: imu_channel
                .subscriber()
                .map_err(|_| FsLoggerError::Subscriber)?,
            barometer_subscriber: barometer_channel
                .subscriber()
                .map_err(|_| FsLoggerError::Subscriber)?,
            gps_subscriber: gps_channel
                .subscriber()
                .map_err(|_| FsLoggerError::Subscriber)?,
            last_emit_us: None,
        })
    }

    pub fn log_tick(&mut self, tick: TimeSample) -> Result<(), FsLoggerError> {
        self.sensor_snapshot.drain_imu(&mut self.imu_subscriber);
        self.sensor_snapshot
            .drain_barometer(&mut self.barometer_subscriber);
        self.sensor_snapshot.drain_gps(&mut self.gps_subscriber);

        let now_us = tick.timestamp.as_micros();
        let period_us = u64::from(self.sensor_snapshot.period_ms()) * 1_000;
        if self
            .last_emit_us
            .is_some_and(|last_emit_us| now_us.saturating_sub(last_emit_us) < period_us)
        {
            return Ok(());
        }

        self.sensor_snapshot
            .append_snapshot(&mut self.engine, tick.timestamp)?;
        self.last_emit_us = Some(now_us);
        Ok(())
    }
}

#[derive(Debug)]
pub enum FsLoggerError {
    Path(LogError),
    Snapshot(SensorSnapshotLoggerError),
    Subscriber,
}

impl fmt::Display for FsLoggerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Path(error) => write!(f, "log path error: {error:?}"),
            Self::Snapshot(error) => write!(f, "snapshot logger error: {error:?}"),
            Self::Subscriber => write!(f, "failed to allocate sample channel subscriber"),
        }
    }
}

impl std::error::Error for FsLoggerError {}

impl From<LogError> for FsLoggerError {
    fn from(value: LogError) -> Self {
        Self::Path(value)
    }
}

impl From<SensorSnapshotLoggerError> for FsLoggerError {
    fn from(value: SensorSnapshotLoggerError) -> Self {
        Self::Snapshot(value)
    }
}

struct HostFileLoggerEngine {
    root: PathBuf,
}

impl HostFileLoggerEngine {
    fn new(root: &Path) -> Self {
        Self {
            root: root.to_path_buf(),
        }
    }

    fn resolve(&self, path: &str) -> PathBuf {
        self.root.join(path.trim_start_matches('/'))
    }
}

impl LoggerEngine for HostFileLoggerEngine {
    fn create_new_csv(&mut self, prefix: &str) -> Result<LogPath, LogError> {
        if prefix.is_empty() || prefix.len() > MAX_LOG_PREFIX_LEN {
            return Err(LogError::PrefixTooLong);
        }

        fs::create_dir_all(&self.root).map_err(map_io_error)?;

        let prefix = prefix.to_ascii_uppercase();
        let mut highest = 0u16;
        for entry in fs::read_dir(&self.root).map_err(map_io_error)? {
            let entry = entry.map_err(map_io_error)?;
            if !entry.file_type().map_err(map_io_error)?.is_file() {
                continue;
            }

            let Some(name) = entry.file_name().to_str().map(str::to_owned) else {
                continue;
            };
            if let Some(sequence) = parse_csv_sequence(&name, &prefix) {
                highest = highest.max(sequence);
            }
        }

        let next_sequence = highest.saturating_add(1);
        if next_sequence == 0 || next_sequence > 9999 {
            return Err(LogError::SequenceOverflow);
        }

        let file_name = format!("{prefix}{next_sequence:04}.CSV");
        OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(self.resolve(&file_name))
            .map_err(map_io_error)?;

        to_log_path(&file_name)
    }

    fn list_files(&mut self, path: &str) -> Result<LogEntryList, LogError> {
        let mut entries = LogEntryList::new();
        for entry in fs::read_dir(self.resolve(path)).map_err(map_io_error)? {
            let entry = entry.map_err(map_io_error)?;
            if !entry.file_type().map_err(map_io_error)?.is_file() {
                continue;
            }
            let name = entry.file_name();
            let Some(name) = name.to_str() else {
                return Err(LogError::Utf8);
            };
            entries
                .push(to_log_path(name)?)
                .map_err(|_| LogError::TooManyEntries)?;
        }
        Ok(entries)
    }

    fn list_directories(&mut self, path: &str) -> Result<LogEntryList, LogError> {
        let mut entries = LogEntryList::new();
        for entry in fs::read_dir(self.resolve(path)).map_err(map_io_error)? {
            let entry = entry.map_err(map_io_error)?;
            if !entry.file_type().map_err(map_io_error)?.is_dir() {
                continue;
            }
            let name = entry.file_name();
            let Some(name) = name.to_str() else {
                return Err(LogError::Utf8);
            };
            entries
                .push(to_log_path(name)?)
                .map_err(|_| LogError::TooManyEntries)?;
        }
        Ok(entries)
    }

    fn read_lines(
        &mut self,
        path: &str,
        start_line: usize,
        count: usize,
    ) -> Result<LogLineList, LogError> {
        let file = File::open(self.resolve(path)).map_err(map_io_error)?;
        let reader = BufReader::new(file);
        let mut lines = LogLineList::new();

        for line in reader.lines().skip(start_line).take(count) {
            let line = line.map_err(map_io_error)?;
            let mut owned = LogLine::new();
            owned.push_str(&line).map_err(|_| LogError::LineTooLong)?;
            lines
                .push(owned)
                .map_err(|_| LogError::TooManyLinesRequested)?;
        }

        Ok(lines)
    }

    fn append_line(&mut self, filename: &str, data: &str) -> Result<(), LogError> {
        let mut file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(self.resolve(filename))
            .map_err(map_io_error)?;
        file.write_all(data.as_bytes()).map_err(map_io_error)?;
        file.write_all(b"\n").map_err(map_io_error)?;
        file.flush().map_err(map_io_error)?;
        Ok(())
    }
}

fn parse_csv_sequence(name: &str, prefix: &str) -> Option<u16> {
    let upper = name.to_ascii_uppercase();
    if !upper.starts_with(prefix) || !upper.ends_with(".CSV") {
        return None;
    }

    let digits = &upper[prefix.len()..upper.len() - 4];
    if digits.len() != 4 || !digits.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }

    digits.parse().ok()
}

fn to_log_path(path: &str) -> Result<LogPath, LogError> {
    let mut owned = LogPath::new();
    owned.push_str(path).map_err(|_| LogError::PathTooLong)?;
    Ok(owned)
}

fn map_io_error(error: std::io::Error) -> LogError {
    use std::io::ErrorKind;

    match error.kind() {
        ErrorKind::NotFound => LogError::NotFound,
        ErrorKind::AlreadyExists => LogError::AlreadyExists,
        ErrorKind::PermissionDenied => LogError::ReadOnly,
        _ => LogError::Filesystem,
    }
}

#[cfg(test)]
mod tests {
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;

    use super::{FsLogger, HostFileLoggerEngine, LoggerEngine};
    use common::messages::sensor::{
        BarometerSample, BarometerSampleStamped, GpsFixSample, GpsFixSampleStamped, ImuSample,
        ImuSampleStamped, TimeSample,
    };
    use common::services::acquisition::{
        BarometerSampleChannel, GpsFixSampleChannel, ImuSampleChannel,
    };
    use common::utilities::time::MeasurementTimestamp;

    #[test]
    fn host_engine_uses_flight_style_csv_names() {
        let temp = std::env::temp_dir().join(format!("marv-fc-sitl-test-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&temp);
        let mut engine = HostFileLoggerEngine::new(&temp);

        let first = engine.create_new_csv("FLGT").unwrap();
        let second = engine.create_new_csv("FLGT").unwrap();

        assert_eq!(first.as_str(), "FLGT0001.CSV");
        assert_eq!(second.as_str(), "FLGT0002.CSV");

        let _ = std::fs::remove_dir_all(&temp);
    }

    #[test]
    fn fs_logger_uses_common_sensor_snapshot_schema() {
        let temp = std::env::temp_dir().join(format!("marv-fc-sitl-log-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&temp);
        let imu = Box::leak(Box::new(ImuSampleChannel::<NoopRawMutex, 8, 2, 1>::new()));
        let baro = Box::leak(Box::new(
            BarometerSampleChannel::<NoopRawMutex, 8, 2, 1>::new(),
        ));
        let gps = Box::leak(Box::new(GpsFixSampleChannel::<NoopRawMutex, 8, 2, 1>::new()));
        let mut logger = FsLogger::new(&temp, imu, baro, gps).unwrap();

        imu.immediate_publisher()
            .publish_immediate(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(10_000),
                ImuSample {
                    accel_mps2: [1.0, 2.0, 3.0],
                    gyro_rad_s: [0.1, 0.2, 0.3],
                },
            ));
        baro.immediate_publisher()
            .publish_immediate(BarometerSampleStamped {
                timestamp: MeasurementTimestamp::from_micros(10_000),
                sample: BarometerSample {
                    pressure_pa: 101_325.0,
                    temperature_c: 22.0,
                },
            });
        gps.immediate_publisher()
            .publish_immediate(GpsFixSampleStamped {
                timestamp: MeasurementTimestamp::from_micros(10_000),
                sample: GpsFixSample {
                    lat_deg: 30.0,
                    lon_deg: -97.0,
                    alt_m: 100.0,
                    vel_ned_mps: [1.0, 2.0, 3.0],
                    sats: 10,
                },
            });

        logger
            .log_tick(TimeSample {
                timestamp: MeasurementTimestamp::from_micros(10_000),
                time_boot_ms: 10,
            })
            .unwrap();

        let contents = std::fs::read_to_string(temp.join("FLGT0001.CSV")).unwrap();
        assert!(contents.contains("log_us,sink_state"));
        assert!(contents.contains("imu_ax_mps2"));
        assert!(contents.contains("baro_pressure_pa"));
        assert!(contents.contains("gps_lat_deg"));

        let _ = std::fs::remove_dir_all(&temp);
    }
}
