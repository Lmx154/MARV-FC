//! MAVLink telemetry abstraction.
//!
//! Mission Planner (and many other GCS tools) drive their built-in sensor/status
//! panels from a standard set of MAVLink messages (RAW_IMU, GPS_RAW_INT, etc).
//!
//! To keep the architecture maintainable, the transport/task layer should not
//! depend on *where* the data came from (raw sensors vs EKF). Instead, it should
//! depend on a source that can produce the required MAVLink messages.

use mavio::dialects::common::messages;

/// Bundle of the common MAVLink messages Mission Planner expects for its built-in UI.
///
/// This bundle is intentionally small and stable. When your estimator comes online,
/// implement `MavlinkTelemetrySource` against estimator state and keep the message
/// schedule unchanged.
#[derive(Clone, Debug, Default)]
pub struct MavlinkTelemetryBundle {
    pub sys_status: messages::SysStatus,
    pub raw_imu: messages::RawImu,
    pub scaled_pressure: messages::ScaledPressure,
    pub gps_raw_int: messages::GpsRawInt,
    pub attitude: messages::Attitude,
}

/// A provider of MAVLink telemetry messages.
///
/// Implementations may be backed by:
/// - raw sensor readings (bring-up)
/// - an EKF/state estimator (flight)
/// - simulation feeds
pub trait MavlinkTelemetrySource {
    async fn bundle(&self, now_ms: u32, now_us: u64) -> MavlinkTelemetryBundle;
}
