//! Typed runtime logging snapshots and status flags.

use crate::interfaces::storage::LogError;
use crate::messages::sensor::{BarometerSample, GpsFixSample, ImuSample, MagnetometerSample};
use crate::utilities::time::MeasurementTimestamp;

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum LoggedSensor {
    Imu,
    AuxImu,
    Barometer,
    Magnetometer,
    Gps,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum SensorLogState {
    Missing,
    Fresh,
    Stale,
    Fault,
}

impl SensorLogState {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Missing => "missing",
            Self::Fresh => "fresh",
            Self::Stale => "stale",
            Self::Fault => "fault",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum LogSinkState {
    Healthy,
    Backpressured,
    DeviceError,
    FilesystemError,
    NotFound,
    Busy,
    ReadOnly,
    PathError,
    Corrupt,
}

impl LogSinkState {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Healthy => "healthy",
            Self::Backpressured => "backpressured",
            Self::DeviceError => "device_error",
            Self::FilesystemError => "filesystem_error",
            Self::NotFound => "not_found",
            Self::Busy => "busy",
            Self::ReadOnly => "read_only",
            Self::PathError => "path_error",
            Self::Corrupt => "corrupt",
        }
    }
}

impl From<LogError> for LogSinkState {
    fn from(error: LogError) -> Self {
        match error {
            LogError::Device => Self::DeviceError,
            LogError::Filesystem => Self::FilesystemError,
            LogError::NotFound => Self::NotFound,
            LogError::Busy => Self::Busy,
            LogError::ReadOnly => Self::ReadOnly,
            LogError::InvalidName
            | LogError::InvalidPath
            | LogError::PrefixTooLong
            | LogError::PathTooLong
            | LogError::LineTooLong
            | LogError::TooManyEntries
            | LogError::TooManyLinesRequested
            | LogError::SequenceOverflow => Self::PathError,
            LogError::AlreadyExists => Self::FilesystemError,
            LogError::Utf8 => Self::Corrupt,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorLogField<T> {
    pub state: SensorLogState,
    pub sample_timestamp: Option<MeasurementTimestamp>,
    pub lagged_samples: u32,
    pub sample: Option<T>,
}

impl<T> Default for SensorLogField<T> {
    fn default() -> Self {
        Self {
            state: SensorLogState::Missing,
            sample_timestamp: None,
            lagged_samples: 0,
            sample: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SensorLogSnapshot {
    pub timestamp: MeasurementTimestamp,
    pub sink_state: LogSinkState,
    pub imu: SensorLogField<ImuSample>,
    pub aux_imu: SensorLogField<ImuSample>,
    pub barometer: SensorLogField<BarometerSample>,
    pub magnetometer: SensorLogField<MagnetometerSample>,
    pub gps: SensorLogField<GpsFixSample>,
}
