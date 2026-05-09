//! Control-loop configuration surfaces.
//!
//! This is the shared shape future runtime/profile storage can update before
//! constructing the pure controllers.

use crate::control::altitude::AltitudeControllerConfig;
use crate::control::attitude::AttitudeControllerConfig;
use crate::control::mixing::MixerLimits;
use crate::control::position::PositionControllerConfig;
use crate::control::rate::RateControllerConfig;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlLoopConfig {
    pub position: PositionControllerConfig,
    pub altitude: AltitudeControllerConfig,
    pub attitude: AttitudeControllerConfig,
    pub rate: RateControllerConfig,
    pub mixer_limits: MixerLimits,
}

impl ControlLoopConfig {
    pub const fn new(
        position: PositionControllerConfig,
        altitude: AltitudeControllerConfig,
        attitude: AttitudeControllerConfig,
        rate: RateControllerConfig,
        mixer_limits: MixerLimits,
    ) -> Self {
        Self {
            position,
            altitude,
            attitude,
            rate,
            mixer_limits,
        }
    }
}

impl Default for ControlLoopConfig {
    fn default() -> Self {
        Self::new(
            PositionControllerConfig::default(),
            AltitudeControllerConfig::default(),
            AttitudeControllerConfig::default(),
            RateControllerConfig::default(),
            MixerLimits::default(),
        )
    }
}
