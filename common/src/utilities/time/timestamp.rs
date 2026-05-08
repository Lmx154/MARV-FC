//! Portable measurement timestamp primitives for embedded, SITL, and replay.

/// Monotonic measurement timestamp represented in microseconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct MeasurementTimestamp {
    micros: u64,
}

impl MeasurementTimestamp {
    /// Creates a timestamp from a monotonic microsecond counter.
    pub const fn from_micros(micros: u64) -> Self {
        Self { micros }
    }

    /// Returns the underlying monotonic microsecond count.
    pub const fn as_micros(self) -> u64 {
        self.micros
    }

    /// Returns the elapsed delta from `earlier` to `self`.
    ///
    /// Returns `None` if `self` is older than `earlier`.
    pub fn checked_delta_since(self, earlier: Self) -> Option<MeasurementDelta> {
        self.micros
            .checked_sub(earlier.micros)
            .map(MeasurementDelta::from_micros)
    }

    /// Returns a saturating elapsed delta from `earlier` to `self`.
    pub fn saturating_delta_since(self, earlier: Self) -> MeasurementDelta {
        MeasurementDelta::from_micros(self.micros.saturating_sub(earlier.micros))
    }
}

/// Elapsed time between two measurement timestamps.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct MeasurementDelta {
    micros: u64,
}

impl MeasurementDelta {
    /// Creates a delta from microseconds.
    pub const fn from_micros(micros: u64) -> Self {
        Self { micros }
    }

    /// Returns the elapsed microseconds.
    pub const fn as_micros(self) -> u64 {
        self.micros
    }

    /// Converts microseconds to seconds for estimator/control `dt` math.
    pub fn as_seconds_f32(self) -> f32 {
        (self.micros as f32) * 1.0e-6
    }
}

#[cfg(test)]
mod tests {
    use super::{MeasurementDelta, MeasurementTimestamp};

    #[test]
    fn checked_delta_handles_ordering() {
        let older = MeasurementTimestamp::from_micros(100);
        let newer = MeasurementTimestamp::from_micros(260);

        assert_eq!(
            newer.checked_delta_since(older),
            Some(MeasurementDelta::from_micros(160))
        );
        assert_eq!(older.checked_delta_since(newer), None);
    }

    #[test]
    fn saturating_delta_never_panics_on_reverse_order() {
        let older = MeasurementTimestamp::from_micros(100);
        let newer = MeasurementTimestamp::from_micros(260);

        assert_eq!(newer.saturating_delta_since(older).as_micros(), 160);
        assert_eq!(older.saturating_delta_since(newer).as_micros(), 0);
    }
}
