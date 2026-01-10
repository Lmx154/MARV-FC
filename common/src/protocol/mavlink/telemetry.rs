//! MAVLink telemetry bundle helpers.

#![allow(async_fn_in_trait)]

use mavio::dialects::common::messages::{
    Attitude, GpsRawInt, RawImu, ScaledPressure, SysStatus,
};

/// Bundle of standard MAVLink telemetry messages.
#[derive(Clone, Debug)]
pub struct MavlinkTelemetryBundle {
    pub sys_status: SysStatus,
    pub raw_imu: RawImu,
    pub scaled_pressure: ScaledPressure,
    pub gps_raw_int: GpsRawInt,
    pub attitude: Attitude,
}

/// Source of standard MAVLink telemetry bundles.
pub trait MavlinkTelemetrySource {
    async fn bundle(&self, now_ms: u32, now_us: u64) -> MavlinkTelemetryBundle;
}
