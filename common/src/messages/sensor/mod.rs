//! Sensor sample messages published by acquisition producers.

use crate::utilities::time::{MeasurementDelta, MeasurementTimestamp};

/// IMU sample in SI units.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSample {
    pub accel_mps2: [f32; 3],
    pub gyro_rad_s: [f32; 3],
}

/// IMU sample plus acquisition timestamp.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuSampleStamped {
    pub timestamp: MeasurementTimestamp,
    pub sample: ImuSample,
}

impl ImuSampleStamped {
    pub const fn new(timestamp: MeasurementTimestamp, sample: ImuSample) -> Self {
        Self { timestamp, sample }
    }

    pub fn dt_since(self, previous: Self) -> Option<MeasurementDelta> {
        self.timestamp.checked_delta_since(previous.timestamp)
    }
}

/// Barometer sample using SI-friendly units.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BarometerSample {
    pub pressure_pa: f32,
    pub temperature_c: f32,
}

/// Barometer sample plus acquisition timestamp.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BarometerSampleStamped {
    pub timestamp: MeasurementTimestamp,
    pub sample: BarometerSample,
}

/// Magnetometer sample in microtesla.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MagnetometerSample {
    pub field_ut: [f32; 3],
}

/// Magnetometer sample plus acquisition timestamp.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MagnetometerSampleStamped {
    pub timestamp: MeasurementTimestamp,
    pub sample: MagnetometerSample,
}

/// GPS fix sample.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GpsFixSample {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f32,
    pub vel_ned_mps: [f32; 3],
    pub sats: u8,
}

/// GPS fix plus acquisition timestamp.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GpsFixSampleStamped {
    pub timestamp: MeasurementTimestamp,
    pub sample: GpsFixSample,
}
