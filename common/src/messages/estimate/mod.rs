//! Estimator output messages.

use crate::utilities::time::MeasurementTimestamp;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StateEstimateSample {
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub attitude_quat: [f32; 4],
    pub valid: bool,
}

impl StateEstimateSample {
    pub const NEUTRAL_INVALID: Self = Self {
        position_ned_m: [0.0, 0.0, 0.0],
        velocity_ned_mps: [0.0, 0.0, 0.0],
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        valid: false,
    };
}

impl Default for StateEstimateSample {
    fn default() -> Self {
        Self::NEUTRAL_INVALID
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StateEstimateStamped {
    pub timestamp: MeasurementTimestamp,
    pub sample: StateEstimateSample,
}
