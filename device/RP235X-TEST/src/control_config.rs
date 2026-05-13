//! RP235X-TEST control tuning profile.
//!
//! Keep HIL bring-up gains in one place so a later software-editable profile
//! store can populate this same shared config shape.

use common::control::altitude::AltitudeControllerConfig;
use common::control::attitude::AttitudeControllerConfig;
use common::control::config::ControlLoopConfig;
use common::control::mixing::{MixerLimits, MotorGeometry};
use common::control::position::PositionControllerConfig;
use common::control::rate::RateControllerConfig;

pub const HIL_CONTROL_CONFIG: ControlLoopConfig = ControlLoopConfig::new(
    PositionControllerConfig {
        position_gain: 0.12,
        velocity_gain: 0.35,
        max_horizontal_velocity_mps: 0.5,
        horizontal_integral_gain: 0.0,
        horizontal_integral_leak: 0.999,
        max_horizontal_integral_accel_mps2: 0.45,
        max_horizontal_accel_mps2: 1.5,
        max_tilt_rad: 10.0 * core::f32::consts::PI / 180.0,
    },
    AltitudeControllerConfig {
        hover_throttle: 0.58,
        altitude_gain: 0.08,
        vertical_velocity_gain: 0.04,
        max_altitude_error_m: 3.0,
        max_vertical_velocity_mps: 2.0,
        max_throttle_correction: 0.35,
    },
    AttitudeControllerConfig {
        roll_rate_gain: 4.0,
        pitch_rate_gain: 4.0,
        yaw_rate_gain: 2.0,
        max_rate_rps: 2.0,
        max_yaw_rate_rps: 1.0,
    },
    RateControllerConfig {
        roll_gain: 0.08,
        pitch_gain: 0.08,
        yaw_gain: 0.04,
        max_axis_command: 0.25,
        measured_rate_deadband_rps: 0.0,
    },
    MixerLimits::NORMALIZED,
);

pub const HIL_MOTOR_GEOMETRY: MotorGeometry = MotorGeometry::f450_xing2_2809_1045_4s_v0();
