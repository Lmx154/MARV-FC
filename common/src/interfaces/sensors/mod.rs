//! Sensor capability traits belong here.

use core::future::Future;

use crate::messages::sensor::{
    BarometerSample, ImuSample, MagnetometerSample, PressureTransducerSample,
};
use crate::utils::delay::DelayMs;

/// Portable IMU source abstraction used by acquisition producers.
///
/// Implementations can wrap real hardware drivers (embedded) or virtual
/// backends (SITL/replay) while exposing the same sample API.
pub trait ImuSource {
    type Error;

    fn read_imu_sample(&mut self) -> impl Future<Output = Result<ImuSample, Self::Error>> + '_;

    /// Re-run the source's hardware bring-up after repeated read failures.
    ///
    /// Default is a no-op, which is correct for virtual sources (HIL/replay).
    /// Real driver-backed adapters override this to replay their init sequence
    /// so a parked or wedged bus transaction can recover instead of leaving the
    /// sensor permanently dead.
    fn reinitialize<D: DelayMs>(
        &mut self,
        _delay: &mut D,
    ) -> impl Future<Output = Result<(), Self::Error>> {
        async { Ok(()) }
    }
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

    /// Re-run the source's hardware bring-up after repeated read failures.
    /// Default no-op for virtual sources; real adapters override it.
    fn reinitialize<D: DelayMs>(
        &mut self,
        _delay: &mut D,
    ) -> impl Future<Output = Result<(), Self::Error>> {
        async { Ok(()) }
    }
}

/// Portable analog pressure transducer abstraction used by acquisition producers.
pub trait PressureTransducerSource {
    type Error;

    fn read_pressure_transducer_sample(
        &mut self,
    ) -> impl Future<Output = Result<PressureTransducerSample, Self::Error>> + '_;
}
