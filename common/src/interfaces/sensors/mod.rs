//! Sensor capability traits belong here.

use core::future::Future;

use crate::messages::sensor::ImuSample;

/// Portable IMU source abstraction used by acquisition producers.
///
/// Implementations can wrap real hardware drivers (embedded) or virtual
/// backends (SITL/replay) while exposing the same sample API.
pub trait ImuSource {
    type Error;

    fn read_imu_sample(&mut self) -> impl Future<Output = Result<ImuSample, Self::Error>> + '_;
}
