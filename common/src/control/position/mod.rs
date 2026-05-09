//! Conservative position-to-attitude control for multicopter waypoint hold.

use crate::control::attitude::AttitudeSetpoint;

const STANDARD_GRAVITY_MPS2: f32 = 9.806_65;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionControllerConfig {
    pub position_gain: f32,
    pub velocity_gain: f32,
    pub max_horizontal_accel_mps2: f32,
    pub max_tilt_rad: f32,
}

impl Default for PositionControllerConfig {
    fn default() -> Self {
        Self {
            position_gain: 0.12,
            velocity_gain: 0.35,
            max_horizontal_accel_mps2: 1.5,
            max_tilt_rad: 10.0 * core::f32::consts::PI / 180.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionControllerSetpoint {
    pub position_ned_m: [f32; 3],
    pub yaw_rad: f32,
}

impl PositionControllerSetpoint {
    pub const ORIGIN_HOLD: Self = Self {
        position_ned_m: [0.0, 0.0, 0.0],
        yaw_rad: 0.0,
    };

    pub const fn new(position_ned_m: [f32; 3], yaw_rad: f32) -> Self {
        Self {
            position_ned_m,
            yaw_rad,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionControllerInput {
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
}

impl PositionControllerInput {
    pub const fn new(position_ned_m: [f32; 3], velocity_ned_mps: [f32; 3]) -> Self {
        Self {
            position_ned_m,
            velocity_ned_mps,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionController {
    config: PositionControllerConfig,
}

impl PositionController {
    pub const fn new(config: PositionControllerConfig) -> Self {
        Self { config }
    }

    pub fn update(
        &self,
        setpoint: PositionControllerSetpoint,
        input: PositionControllerInput,
    ) -> Option<AttitudeSetpoint> {
        if !finite_f32x3(setpoint.position_ned_m)
            || !setpoint.yaw_rad.is_finite()
            || !finite_f32x3(input.position_ned_m)
            || !finite_f32x3(input.velocity_ned_mps)
        {
            return None;
        }

        let north_error_m = setpoint.position_ned_m[0] - input.position_ned_m[0];
        let east_error_m = setpoint.position_ned_m[1] - input.position_ned_m[1];
        let accel_ned_mps2 = clamp_horizontal_accel(
            [
                self.config.position_gain * north_error_m
                    - self.config.velocity_gain * input.velocity_ned_mps[0],
                self.config.position_gain * east_error_m
                    - self.config.velocity_gain * input.velocity_ned_mps[1],
            ],
            self.config.max_horizontal_accel_mps2,
        );
        let (roll_rad, pitch_rad) =
            accel_to_tilt(accel_ned_mps2, setpoint.yaw_rad, self.config.max_tilt_rad);

        AttitudeSetpoint::from_euler_rad(roll_rad, pitch_rad, setpoint.yaw_rad)
    }
}

impl Default for PositionController {
    fn default() -> Self {
        Self::new(PositionControllerConfig::default())
    }
}

fn accel_to_tilt(accel_ned_mps2: [f32; 2], yaw_rad: f32, max_tilt_rad: f32) -> (f32, f32) {
    let yaw_cos = micromath::F32Ext::cos(yaw_rad);
    let yaw_sin = micromath::F32Ext::sin(yaw_rad);
    let max_tilt_rad = positive_finite_or_zero(max_tilt_rad);
    let roll_rad =
        (-yaw_sin * accel_ned_mps2[0] + yaw_cos * accel_ned_mps2[1]) / STANDARD_GRAVITY_MPS2;
    let pitch_rad =
        -(yaw_cos * accel_ned_mps2[0] + yaw_sin * accel_ned_mps2[1]) / STANDARD_GRAVITY_MPS2;

    (
        roll_rad.clamp(-max_tilt_rad, max_tilt_rad),
        pitch_rad.clamp(-max_tilt_rad, max_tilt_rad),
    )
}

fn clamp_horizontal_accel(accel_mps2: [f32; 2], limit_mps2: f32) -> [f32; 2] {
    let limit_mps2 = positive_finite_or_zero(limit_mps2);
    let magnitude =
        micromath::F32Ext::sqrt(accel_mps2[0] * accel_mps2[0] + accel_mps2[1] * accel_mps2[1]);

    if magnitude <= limit_mps2 || magnitude <= f32::EPSILON {
        return accel_mps2;
    }

    let scale = limit_mps2 / magnitude;
    [accel_mps2[0] * scale, accel_mps2[1] * scale]
}

fn positive_finite_or_zero(value: f32) -> f32 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        0.0
    }
}

fn finite_f32x3(values: [f32; 3]) -> bool {
    values.iter().all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
    use super::{
        PositionController, PositionControllerConfig, PositionControllerInput,
        PositionControllerSetpoint,
    };

    #[test]
    fn zero_error_holds_level_attitude() {
        let controller = PositionController::default();

        let attitude = controller
            .update(
                PositionControllerSetpoint::ORIGIN_HOLD,
                PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            )
            .expect("finite setpoint and input should produce attitude");

        assert!((attitude.quaternion[0] - 1.0).abs() < 0.000_001);
        assert_eq!(attitude.quaternion[1], 0.0);
        assert_eq!(attitude.quaternion[2], 0.0);
        assert_eq!(attitude.quaternion[3], 0.0);
    }

    #[test]
    fn north_position_error_commands_nose_down_pitch() {
        let controller = PositionController::default();

        let attitude = controller
            .update(
                PositionControllerSetpoint::new([10.0, 0.0, 0.0], 0.0),
                PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            )
            .expect("finite setpoint and input should produce attitude");

        assert!(attitude.quaternion[2] < 0.0);
    }

    #[test]
    fn east_position_error_commands_right_roll() {
        let controller = PositionController::default();

        let attitude = controller
            .update(
                PositionControllerSetpoint::new([0.0, 10.0, 0.0], 0.0),
                PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            )
            .expect("finite setpoint and input should produce attitude");

        assert!(attitude.quaternion[1] > 0.0);
    }

    #[test]
    fn horizontal_acceleration_is_limited_before_tilt_mapping() {
        let controller = PositionController::new(PositionControllerConfig {
            position_gain: 100.0,
            velocity_gain: 0.0,
            max_horizontal_accel_mps2: 0.5,
            max_tilt_rad: 1.0,
        });

        let attitude = controller
            .update(
                PositionControllerSetpoint::new([1_000.0, 0.0, 0.0], 0.0),
                PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            )
            .expect("finite setpoint and input should produce attitude");

        assert!(
            (attitude.quaternion[2] + 0.025_485).abs() < 0.002,
            "qy={}",
            attitude.quaternion[2]
        );
    }

    #[test]
    fn invalid_input_is_rejected() {
        let controller = PositionController::default();

        assert!(
            controller
                .update(
                    PositionControllerSetpoint::new([f32::NAN, 0.0, 0.0], 0.0),
                    PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                )
                .is_none()
        );
    }
}
