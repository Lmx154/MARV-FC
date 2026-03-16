//! Monotonic time and scheduler-facing traits belong here.

use crate::utilities::time::MeasurementTimestamp;

/// Portable monotonic clock used at producer boundaries for sample stamping.
pub trait MonotonicClock {
    fn now(&self) -> MeasurementTimestamp;
}
