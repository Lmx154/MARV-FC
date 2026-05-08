//! Sensor capability traits belong here.

use core::future::Future;

use crate::messages::sensor::{
    BarometerSample, ImuSample, MagnetometerSample, PressureTransducerSample,
};

/// Portable IMU source abstraction used by acquisition producers.
///
/// Implementations can wrap real hardware drivers (embedded) or virtual
/// backends (SITL/replay) while exposing the same sample API.
pub trait ImuSource {
    type Error;

    fn read_imu_sample(&mut self) -> impl Future<Output = Result<ImuSample, Self::Error>> + '_;
}

/// Portable barometer source abstraction used by acquisition producers.
pub trait BarometerSource {
    type Error;

    fn read_barometer_sample(
        &mut self,
    ) -> impl Future<Output = Result<BarometerSample, Self::Error>> + '_;
}

/// Portable magnetometer source abstraction used by acquisition producers.
pub trait MagnetometerSource {
    type Error;

    fn read_magnetometer_sample(
        &mut self,
    ) -> impl Future<Output = Result<MagnetometerSample, Self::Error>> + '_;
}

/// Portable analog pressure transducer abstraction used by acquisition producers.
pub trait PressureTransducerSource {
    type Error;

    fn read_pressure_transducer_sample(
        &mut self,
    ) -> impl Future<Output = Result<PressureTransducerSample, Self::Error>> + '_;
}
