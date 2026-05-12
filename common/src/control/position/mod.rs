//! Conservative position-to-attitude control for multicopter waypoint hold.

use core::cell::Cell;

use crate::control::attitude::AttitudeSetpoint;

const STANDARD_GRAVITY_MPS2: f32 = 9.806_65;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionControllerConfig {
    pub position_gain: f32,
    pub velocity_gain: f32,
    pub max_horizontal_velocity_mps: f32,
    pub horizontal_integral_gain: f32,
    pub horizontal_integral_leak: f32,
    pub max_horizontal_integral_accel_mps2: f32,
    pub max_horizontal_accel_mps2: f32,
    pub max_tilt_rad: f32,
}

impl Default for PositionControllerConfig {
    fn default() -> Self {
        Self {
            position_gain: 0.12,
            velocity_gain: 0.35,
            max_horizontal_velocity_mps: 0.5,
            horizontal_integral_gain: 0.0,
            horizontal_integral_leak: 0.999,
            max_horizontal_integral_accel_mps2: 0.45,
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

#[derive(Debug)]
pub struct PositionController {
    config: PositionControllerConfig,
    horizontal_integral_accel_ned_mps2: [Cell<f32>; 2],
}

impl PositionController {
    pub const fn new(config: PositionControllerConfig) -> Self {
        Self {
            config,
            horizontal_integral_accel_ned_mps2: [Cell::new(0.0), Cell::new(0.0)],
        }
    }

    pub fn reset(&self) {
        self.horizontal_integral_accel_ned_mps2[0].set(0.0);
        self.horizontal_integral_accel_ned_mps2[1].set(0.0);
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
            self.reset();
            return None;
        }

        let north_error_m = setpoint.position_ned_m[0] - input.position_ned_m[0];
        let east_error_m = setpoint.position_ned_m[1] - input.position_ned_m[1];
        let integral_accel_ned_mps2 = self.integral_accel_ned_mps2();
        let velocity_error_ned_mps = [
            velocity_error_from_position(
                north_error_m,
                input.velocity_ned_mps[0],
                self.config.position_gain,
                self.config.velocity_gain,
                self.config.max_horizontal_velocity_mps,
            ),
            velocity_error_from_position(
                east_error_m,
                input.velocity_ned_mps[1],
                self.config.position_gain,
                self.config.velocity_gain,
                self.config.max_horizontal_velocity_mps,
            ),
        ];
        let requested_accel_ned_mps2 = if self.config.velocity_gain > f32::EPSILON {
            [
                self.config.velocity_gain * velocity_error_ned_mps[0] + integral_accel_ned_mps2[0],
                self.config.velocity_gain * velocity_error_ned_mps[1] + integral_accel_ned_mps2[1],
            ]
        } else {
            [
                self.config.position_gain * north_error_m + integral_accel_ned_mps2[0],
                self.config.position_gain * east_error_m + integral_accel_ned_mps2[1],
            ]
        };
        let accel_ned_mps2 =
            clamp_horizontal_accel(requested_accel_ned_mps2, self.config.max_horizontal_accel_mps2);
        self.update_horizontal_integral(
            velocity_error_ned_mps,
            integral_accel_ned_mps2,
            requested_accel_ned_mps2,
            accel_ned_mps2,
        );

        let (roll_rad, pitch_rad) =
            accel_to_tilt(accel_ned_mps2, setpoint.yaw_rad, self.config.max_tilt_rad);

        AttitudeSetpoint::from_euler_rad(roll_rad, pitch_rad, setpoint.yaw_rad)
    }

    fn integral_accel_ned_mps2(&self) -> [f32; 2] {
        [
            self.horizontal_integral_accel_ned_mps2[0].get(),
            self.horizontal_integral_accel_ned_mps2[1].get(),
        ]
    }

    fn update_horizontal_integral(
        &self,
        velocity_error_ned_mps: [f32; 2],
        previous_integral_accel_ned_mps2: [f32; 2],
        requested_accel_ned_mps2: [f32; 2],
        applied_accel_ned_mps2: [f32; 2],
    ) {
        let max_integral_accel =
            positive_finite_or_zero(self.config.max_horizontal_integral_accel_mps2)
                .min(positive_finite_or_zero(self.config.max_horizontal_accel_mps2));
        if max_integral_accel <= f32::EPSILON {
            self.reset();
            return;
        }

        let leak = if self.config.horizontal_integral_leak.is_finite() {
            self.config.horizontal_integral_leak.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let gain = positive_finite_or_zero(self.config.horizontal_integral_gain);
        let saturated = horizontal_norm(requested_accel_ned_mps2)
            > horizontal_norm(applied_accel_ned_mps2) + 0.000_1;
        let increment = [
            gain * velocity_error_ned_mps[0],
            gain * velocity_error_ned_mps[1],
        ];
        let relieves_saturation = dot2(increment, requested_accel_ned_mps2) < 0.0 || !saturated;

        let next = if relieves_saturation {
            [
                previous_integral_accel_ned_mps2[0] * leak + increment[0],
                previous_integral_accel_ned_mps2[1] * leak + increment[1],
            ]
        } else {
            [
                previous_integral_accel_ned_mps2[0] * leak,
                previous_integral_accel_ned_mps2[1] * leak,
            ]
        };
        let next = clamp_horizontal_accel(next, max_integral_accel);
        self.horizontal_integral_accel_ned_mps2[0].set(next[0]);
        self.horizontal_integral_accel_ned_mps2[1].set(next[1]);
    }
}

impl Default for PositionController {
    fn default() -> Self {
        Self::new(PositionControllerConfig::default())
    }
}

impl Clone for PositionController {
    fn clone(&self) -> Self {
        Self {
            config: self.config,
            horizontal_integral_accel_ned_mps2: [
                Cell::new(self.horizontal_integral_accel_ned_mps2[0].get()),
                Cell::new(self.horizontal_integral_accel_ned_mps2[1].get()),
            ],
        }
    }
}

impl PartialEq for PositionController {
    fn eq(&self, other: &Self) -> bool {
        self.config == other.config
            && self.horizontal_integral_accel_ned_mps2[0].get()
                == other.horizontal_integral_accel_ned_mps2[0].get()
            && self.horizontal_integral_accel_ned_mps2[1].get()
                == other.horizontal_integral_accel_ned_mps2[1].get()
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
    let magnitude = horizontal_norm(accel_mps2);

    if magnitude <= limit_mps2 || magnitude <= f32::EPSILON {
        return accel_mps2;
    }

    let scale = limit_mps2 / magnitude;
    [accel_mps2[0] * scale, accel_mps2[1] * scale]
}

fn velocity_error_from_position(
    position_error_m: f32,
    velocity_mps: f32,
    position_gain: f32,
    velocity_gain: f32,
    max_velocity_mps: f32,
) -> f32 {
    let velocity_gain = positive_finite_or_zero(velocity_gain);
    if velocity_gain <= f32::EPSILON {
        return 0.0;
    }

    let velocity_setpoint_mps =
        clamp_symmetric(position_gain * position_error_m / velocity_gain, max_velocity_mps);
    velocity_setpoint_mps - velocity_mps
}

fn horizontal_norm(values: [f32; 2]) -> f32 {
    micromath::F32Ext::sqrt(values[0] * values[0] + values[1] * values[1])
}

fn dot2(a: [f32; 2], b: [f32; 2]) -> f32 {
    a[0] * b[0] + a[1] * b[1]
}

fn positive_finite_or_zero(value: f32) -> f32 {
    if value.is_finite() && value > 0.0 {
        value
    } else {
        0.0
    }
}

fn clamp_symmetric(value: f32, limit: f32) -> f32 {
    let limit = positive_finite_or_zero(limit);
    if limit <= f32::EPSILON {
        0.0
    } else {
        value.clamp(-limit, limit)
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
    use crate::test_helpers::assert_quaternion_near;

    #[test]
    fn zero_error_holds_level_attitude() {
        let controller = PositionController::default();

        let attitude = controller
            .update(
                PositionControllerSetpoint::ORIGIN_HOLD,
                PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            )
            .expect("finite setpoint and input should produce attitude");

        assert_quaternion_near(attitude.quaternion, [1.0, 0.0, 0.0, 0.0], 0.000_001);
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
    fn south_position_error_commands_nose_up_pitch() {
        let controller = PositionController::default();

        let attitude = controller
            .update(
                PositionControllerSetpoint::new([-10.0, 0.0, 0.0], 0.0),
                PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            )
            .expect("finite setpoint and input should produce attitude");

        assert!(attitude.quaternion[2] > 0.0);
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
    fn west_position_error_commands_left_roll() {
        let controller = PositionController::default();

        let attitude = controller
            .update(
                PositionControllerSetpoint::new([0.0, -10.0, 0.0], 0.0),
                PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            )
            .expect("finite setpoint and input should produce attitude");

        assert!(attitude.quaternion[1] < 0.0);
    }

    #[test]
    fn east_velocity_commands_left_roll_to_brake() {
        let controller = PositionController::default();

        let attitude = controller
            .update(
                PositionControllerSetpoint::ORIGIN_HOLD,
                PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 2.0, 0.0]),
            )
            .expect("finite setpoint and input should produce attitude");

        assert!(attitude.quaternion[1] < 0.0);
    }

    #[test]
    fn horizontal_acceleration_is_limited_before_tilt_mapping() {
        let controller = PositionController::new(PositionControllerConfig {
            position_gain: 100.0,
            velocity_gain: 0.0,
            max_horizontal_velocity_mps: 100.0,
            horizontal_integral_gain: 0.0,
            horizontal_integral_leak: 0.0,
            max_horizontal_integral_accel_mps2: 0.0,
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
