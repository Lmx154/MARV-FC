//! Conservative position-to-attitude control for multicopter waypoint hold.

use core::cell::Cell;

use crate::control::attitude::AttitudeSetpoint;

const STANDARD_GRAVITY_MPS2: f32 = 9.806_65;
const DEFAULT_CONTROL_DT_S: f32 = 0.01;

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
    pub velocity_ned_mps: [f32; 3],
    pub acceleration_ned_mps2: [f32; 3],
    pub yaw_rad: f32,
}

impl PositionControllerSetpoint {
    pub const ORIGIN_HOLD: Self = Self {
        position_ned_m: [0.0, 0.0, 0.0],
        velocity_ned_mps: [0.0, 0.0, 0.0],
        acceleration_ned_mps2: [0.0, 0.0, 0.0],
        yaw_rad: 0.0,
    };

    pub const fn new(position_ned_m: [f32; 3], yaw_rad: f32) -> Self {
        Self {
            position_ned_m,
            velocity_ned_mps: [0.0, 0.0, 0.0],
            acceleration_ned_mps2: [0.0, 0.0, 0.0],
            yaw_rad,
        }
    }

    pub const fn with_velocity_acceleration(
        position_ned_m: [f32; 3],
        velocity_ned_mps: [f32; 3],
        acceleration_ned_mps2: [f32; 3],
        yaw_rad: f32,
    ) -> Self {
        Self {
            position_ned_m,
            velocity_ned_mps,
            acceleration_ned_mps2,
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
pub struct PositionControllerDebug {
    pub position_error_ned_m: [f32; 3],
    pub velocity_setpoint_ned_mps: [f32; 2],
    pub velocity_error_ned_mps: [f32; 2],
    pub feed_forward_accel_ned_mps2: [f32; 2],
    pub requested_accel_ned_mps2: [f32; 2],
    pub applied_accel_ned_mps2: [f32; 2],
    pub integral_accel_ned_mps2: [f32; 2],
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub yaw_rad: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PositionControllerOutput {
    pub attitude_setpoint: AttitudeSetpoint,
    pub debug: PositionControllerDebug,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrustVectorConfig {
    pub hover_throttle: f32,
    pub min_collective_throttle: f32,
    pub max_collective_throttle: f32,
    pub min_tilt_cosine: f32,
}

impl Default for ThrustVectorConfig {
    fn default() -> Self {
        Self {
            hover_throttle: 0.5,
            min_collective_throttle: 0.0,
            max_collective_throttle: 1.0,
            min_tilt_cosine: 0.25,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrustVectorSetpoint {
    pub attitude_setpoint: AttitudeSetpoint,
    pub requested_horizontal_accel_ned_mps2: [f32; 2],
    pub applied_horizontal_accel_ned_mps2: [f32; 2],
    pub requested_collective_throttle: f32,
    pub applied_collective_throttle: f32,
    pub tilt_compensation: f32,
    pub tilt_rad: f32,
    pub vertical_throttle_margin: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThrustVectorController {
    config: ThrustVectorConfig,
}

impl ThrustVectorController {
    pub const fn new(config: ThrustVectorConfig) -> Self {
        Self { config }
    }

    pub fn update(
        &self,
        attitude_setpoint: AttitudeSetpoint,
        position_debug: PositionControllerDebug,
        vertical_collective_throttle: f32,
    ) -> Option<ThrustVectorSetpoint> {
        if !vertical_collective_throttle.is_finite()
            || !position_debug.roll_rad.is_finite()
            || !position_debug.pitch_rad.is_finite()
            || !finite_f32x2(position_debug.requested_accel_ned_mps2)
            || !finite_f32x2(position_debug.applied_accel_ned_mps2)
        {
            return None;
        }

        let roll_cos = micromath::F32Ext::cos(position_debug.roll_rad);
        let pitch_cos = micromath::F32Ext::cos(position_debug.pitch_rad);
        let min_tilt_cosine = positive_finite_or_zero(self.config.min_tilt_cosine).clamp(0.01, 1.0);
        let vertical_thrust_fraction = (roll_cos * pitch_cos).abs().max(min_tilt_cosine);
        let tilt_compensation = 1.0 / vertical_thrust_fraction;
        let requested_collective_throttle = vertical_collective_throttle * tilt_compensation;
        let applied_collective_throttle = requested_collective_throttle.clamp(
            self.config.min_collective_throttle.clamp(0.0, 1.0),
            self.config.max_collective_throttle.clamp(0.0, 1.0),
        );
        let tilt_rad = micromath::F32Ext::sqrt(
            position_debug.roll_rad * position_debug.roll_rad
                + position_debug.pitch_rad * position_debug.pitch_rad,
        );

        Some(ThrustVectorSetpoint {
            attitude_setpoint,
            requested_horizontal_accel_ned_mps2: position_debug.requested_accel_ned_mps2,
            applied_horizontal_accel_ned_mps2: position_debug.applied_accel_ned_mps2,
            requested_collective_throttle,
            applied_collective_throttle,
            tilt_compensation,
            tilt_rad,
            vertical_throttle_margin: self.config.max_collective_throttle
                - applied_collective_throttle,
        })
    }
}

impl Default for ThrustVectorController {
    fn default() -> Self {
        Self::new(ThrustVectorConfig::default())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XYPositionVelocityOutput {
    pub velocity_setpoint_ned_mps: [f32; 2],
    pub velocity_error_ned_mps: [f32; 2],
    pub feed_forward_accel_ned_mps2: [f32; 2],
    pub requested_accel_ned_mps2: [f32; 2],
    pub applied_accel_ned_mps2: [f32; 2],
    pub integral_accel_ned_mps2: [f32; 2],
}

#[derive(Debug)]
pub struct XYPositionVelocityController {
    config: PositionControllerConfig,
    horizontal_integral_accel_ned_mps2: [Cell<f32>; 2],
}

impl XYPositionVelocityController {
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
        dt_s: f32,
    ) -> Option<XYPositionVelocityOutput> {
        if !valid_dt(dt_s)
            || !finite_f32x3(setpoint.position_ned_m)
            || !finite_f32x3(setpoint.velocity_ned_mps)
            || !finite_f32x3(setpoint.acceleration_ned_mps2)
            || !finite_f32x3(input.position_ned_m)
            || !finite_f32x3(input.velocity_ned_mps)
        {
            self.reset();
            return None;
        }

        let position_error_ned_m = [
            setpoint.position_ned_m[0] - input.position_ned_m[0],
            setpoint.position_ned_m[1] - input.position_ned_m[1],
        ];
        let velocity_gain = positive_finite_or_zero(self.config.velocity_gain);
        let position_velocity_ned_mps = if velocity_gain > f32::EPSILON {
            [
                self.config.position_gain * position_error_ned_m[0] / velocity_gain,
                self.config.position_gain * position_error_ned_m[1] / velocity_gain,
            ]
        } else {
            [0.0, 0.0]
        };
        let velocity_setpoint_ned_mps = clamp_horizontal_vector(
            [
                setpoint.velocity_ned_mps[0] + position_velocity_ned_mps[0],
                setpoint.velocity_ned_mps[1] + position_velocity_ned_mps[1],
            ],
            self.config.max_horizontal_velocity_mps,
        );
        let velocity_error_ned_mps = [
            velocity_setpoint_ned_mps[0] - input.velocity_ned_mps[0],
            velocity_setpoint_ned_mps[1] - input.velocity_ned_mps[1],
        ];
        let feed_forward_accel_ned_mps2 = [
            setpoint.acceleration_ned_mps2[0],
            setpoint.acceleration_ned_mps2[1],
        ];
        let integral_accel_ned_mps2 = self.integral_accel_ned_mps2();
        let requested_accel_ned_mps2 = [
            feed_forward_accel_ned_mps2[0]
                + self.config.velocity_gain * velocity_error_ned_mps[0]
                + integral_accel_ned_mps2[0],
            feed_forward_accel_ned_mps2[1]
                + self.config.velocity_gain * velocity_error_ned_mps[1]
                + integral_accel_ned_mps2[1],
        ];
        let applied_accel_ned_mps2 = clamp_horizontal_vector(
            requested_accel_ned_mps2,
            self.config.max_horizontal_accel_mps2,
        );

        self.update_horizontal_integral(
            velocity_error_ned_mps,
            integral_accel_ned_mps2,
            requested_accel_ned_mps2,
            applied_accel_ned_mps2,
            dt_s,
        );

        Some(XYPositionVelocityOutput {
            velocity_setpoint_ned_mps,
            velocity_error_ned_mps,
            feed_forward_accel_ned_mps2,
            requested_accel_ned_mps2,
            applied_accel_ned_mps2,
            integral_accel_ned_mps2,
        })
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
        dt_s: f32,
    ) {
        let max_integral_accel =
            positive_finite_or_zero(self.config.max_horizontal_integral_accel_mps2).min(
                positive_finite_or_zero(self.config.max_horizontal_accel_mps2),
            );
        if max_integral_accel <= f32::EPSILON {
            self.reset();
            return;
        }

        let nominal_dt_s = DEFAULT_CONTROL_DT_S;
        let dt_scale = dt_s / nominal_dt_s;
        let leak = if self.config.horizontal_integral_leak.is_finite() {
            self.config.horizontal_integral_leak.clamp(0.0, 1.0)
        } else {
            0.0
        };
        let leak = micromath::F32Ext::powf(leak, dt_scale);
        let gain = positive_finite_or_zero(self.config.horizontal_integral_gain);
        let saturated = horizontal_norm(requested_accel_ned_mps2)
            > horizontal_norm(applied_accel_ned_mps2) + 0.000_1;
        let increment = [
            gain * velocity_error_ned_mps[0] * dt_scale,
            gain * velocity_error_ned_mps[1] * dt_scale,
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
        let next = clamp_horizontal_vector(next, max_integral_accel);
        self.horizontal_integral_accel_ned_mps2[0].set(next[0]);
        self.horizontal_integral_accel_ned_mps2[1].set(next[1]);
    }
}

impl Clone for XYPositionVelocityController {
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

impl PartialEq for XYPositionVelocityController {
    fn eq(&self, other: &Self) -> bool {
        self.config == other.config
            && self.horizontal_integral_accel_ned_mps2[0].get()
                == other.horizontal_integral_accel_ned_mps2[0].get()
            && self.horizontal_integral_accel_ned_mps2[1].get()
                == other.horizontal_integral_accel_ned_mps2[1].get()
    }
}

#[derive(Debug)]
pub struct PositionController {
    config: PositionControllerConfig,
    xy: XYPositionVelocityController,
}

impl PositionController {
    pub const fn new(config: PositionControllerConfig) -> Self {
        Self {
            config,
            xy: XYPositionVelocityController::new(config),
        }
    }

    pub fn reset(&self) {
        self.xy.reset();
    }

    pub fn update(
        &self,
        setpoint: PositionControllerSetpoint,
        input: PositionControllerInput,
    ) -> Option<AttitudeSetpoint> {
        self.update_with_debug(setpoint, input)
            .map(|output| output.attitude_setpoint)
    }

    pub fn update_with_debug(
        &self,
        setpoint: PositionControllerSetpoint,
        input: PositionControllerInput,
    ) -> Option<PositionControllerOutput> {
        self.update_with_debug_dt(setpoint, input, DEFAULT_CONTROL_DT_S)
    }

    pub fn update_with_debug_dt(
        &self,
        setpoint: PositionControllerSetpoint,
        input: PositionControllerInput,
        dt_s: f32,
    ) -> Option<PositionControllerOutput> {
        if !finite_f32x3(setpoint.position_ned_m)
            || !finite_f32x3(setpoint.velocity_ned_mps)
            || !finite_f32x3(setpoint.acceleration_ned_mps2)
            || !setpoint.yaw_rad.is_finite()
            || !finite_f32x3(input.position_ned_m)
            || !finite_f32x3(input.velocity_ned_mps)
            || !valid_dt(dt_s)
        {
            self.reset();
            return None;
        }

        let north_error_m = setpoint.position_ned_m[0] - input.position_ned_m[0];
        let east_error_m = setpoint.position_ned_m[1] - input.position_ned_m[1];
        let down_error_m = setpoint.position_ned_m[2] - input.position_ned_m[2];
        let xy_output = self.xy.update(setpoint, input, dt_s)?;

        let (roll_rad, pitch_rad) = accel_to_tilt(
            xy_output.applied_accel_ned_mps2,
            setpoint.yaw_rad,
            self.config.max_tilt_rad,
        );

        let attitude_setpoint =
            AttitudeSetpoint::from_euler_rad(roll_rad, pitch_rad, setpoint.yaw_rad)?;

        Some(PositionControllerOutput {
            attitude_setpoint,
            debug: PositionControllerDebug {
                position_error_ned_m: [north_error_m, east_error_m, down_error_m],
                velocity_setpoint_ned_mps: xy_output.velocity_setpoint_ned_mps,
                velocity_error_ned_mps: xy_output.velocity_error_ned_mps,
                feed_forward_accel_ned_mps2: xy_output.feed_forward_accel_ned_mps2,
                requested_accel_ned_mps2: xy_output.requested_accel_ned_mps2,
                applied_accel_ned_mps2: xy_output.applied_accel_ned_mps2,
                integral_accel_ned_mps2: xy_output.integral_accel_ned_mps2,
                roll_rad,
                pitch_rad,
                yaw_rad: setpoint.yaw_rad,
            },
        })
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
            xy: self.xy.clone(),
        }
    }
}

impl PartialEq for PositionController {
    fn eq(&self, other: &Self) -> bool {
        self.config == other.config && self.xy == other.xy
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

fn clamp_horizontal_vector(values: [f32; 2], limit: f32) -> [f32; 2] {
    let limit = positive_finite_or_zero(limit);
    let magnitude = horizontal_norm(values);

    if magnitude <= limit || magnitude <= f32::EPSILON {
        return values;
    }

    let scale = limit / magnitude;
    [values[0] * scale, values[1] * scale]
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

fn valid_dt(dt_s: f32) -> bool {
    dt_s.is_finite() && dt_s > 0.0
}

fn finite_f32x3(values: [f32; 3]) -> bool {
    values.iter().all(|value| value.is_finite())
}

fn finite_f32x2(values: [f32; 2]) -> bool {
    values.iter().all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
    use super::{
        PositionController, PositionControllerConfig, PositionControllerDebug,
        PositionControllerInput, PositionControllerSetpoint, ThrustVectorConfig,
        ThrustVectorController, XYPositionVelocityController,
    };
    use crate::control::attitude::AttitudeSetpoint;
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
            velocity_gain: 1.0,
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
    fn xy_integral_is_loop_rate_invariant_for_same_elapsed_time() {
        let accel_50hz = run_xy_integrator_for_elapsed_time(0.02, 51);
        let accel_100hz = run_xy_integrator_for_elapsed_time(0.01, 101);
        let accel_200hz = run_xy_integrator_for_elapsed_time(0.005, 201);

        assert!(
            (accel_50hz - accel_100hz).abs() < 0.000_1,
            "50 Hz integral drifted from 100 Hz: {accel_50hz} vs {accel_100hz}"
        );
        assert!(
            (accel_200hz - accel_100hz).abs() < 0.000_1,
            "200 Hz integral drifted from 100 Hz: {accel_200hz} vs {accel_100hz}"
        );
    }

    #[test]
    fn xy_integral_freezes_while_saturated_and_recovers_when_error_relieves_limit() {
        let controller = XYPositionVelocityController::new(PositionControllerConfig {
            position_gain: 10.0,
            velocity_gain: 1.0,
            max_horizontal_velocity_mps: 5.0,
            horizontal_integral_gain: 0.05,
            horizontal_integral_leak: 1.0,
            max_horizontal_integral_accel_mps2: 2.0,
            max_horizontal_accel_mps2: 0.2,
            max_tilt_rad: 1.0,
        });
        let saturated_setpoint = PositionControllerSetpoint::new([100.0, 0.0, 0.0], 0.0);
        let stationary = PositionControllerInput::new([0.0; 3], [0.0; 3]);

        let mut output = None;
        for _ in 0..50 {
            output = Some(
                controller
                    .update(saturated_setpoint, stationary, 0.01)
                    .expect("finite saturated update should remain valid"),
            );
        }
        let output = output.unwrap();
        assert_eq!(output.integral_accel_ned_mps2, [0.0, 0.0]);
        assert!(
            output.requested_accel_ned_mps2[0] > output.applied_accel_ned_mps2[0],
            "test setup should be saturated: {output:?}"
        );

        let relieving_setpoint = PositionControllerSetpoint::with_velocity_acceleration(
            [0.0; 3],
            [0.0; 3],
            [1.0, 0.0, 0.0],
            0.0,
        );
        let braking_input = PositionControllerInput::new([0.0; 3], [0.5, 0.0, 0.0]);
        controller
            .update(relieving_setpoint, braking_input, 0.01)
            .expect("finite relieving update should remain valid");
        let relieved = controller
            .update(relieving_setpoint, braking_input, 0.01)
            .expect("finite relieving update should remain valid");

        assert!(
            relieved.integral_accel_ned_mps2[0] < 0.0,
            "integral should move only when it relieves saturation: {relieved:?}"
        );
    }

    #[test]
    fn thrust_vector_compensates_collective_for_tilt() {
        let controller = ThrustVectorController::new(ThrustVectorConfig {
            hover_throttle: 0.5,
            min_collective_throttle: 0.0,
            max_collective_throttle: 1.0,
            min_tilt_cosine: 0.25,
        });
        let debug = position_debug_with_tilt(20.0_f32.to_radians(), 0.0);
        let attitude = AttitudeSetpoint::from_euler_rad(debug.roll_rad, debug.pitch_rad, 0.0)
            .expect("finite attitude setpoint should be valid");

        let thrust = controller
            .update(attitude, debug, 0.5)
            .expect("finite thrust vector should be valid");

        assert!(thrust.applied_collective_throttle > 0.5);
        assert!(thrust.tilt_compensation > 1.0);
        assert_eq!(
            thrust.requested_horizontal_accel_ned_mps2,
            debug.requested_accel_ned_mps2
        );
        assert_eq!(
            thrust.applied_horizontal_accel_ned_mps2,
            debug.applied_accel_ned_mps2
        );
    }

    #[test]
    fn thrust_vector_reports_lost_vertical_margin_when_collective_clamps() {
        let controller = ThrustVectorController::new(ThrustVectorConfig {
            hover_throttle: 0.5,
            min_collective_throttle: 0.0,
            max_collective_throttle: 0.55,
            min_tilt_cosine: 0.25,
        });
        let debug = position_debug_with_tilt(35.0_f32.to_radians(), 0.0);
        let attitude = AttitudeSetpoint::from_euler_rad(debug.roll_rad, debug.pitch_rad, 0.0)
            .expect("finite attitude setpoint should be valid");

        let thrust = controller
            .update(attitude, debug, 0.55)
            .expect("finite thrust vector should be valid");

        assert!(thrust.requested_collective_throttle > thrust.applied_collective_throttle);
        assert_eq!(thrust.applied_collective_throttle, 0.55);
        assert!(thrust.vertical_throttle_margin <= f32::EPSILON);
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

    fn run_xy_integrator_for_elapsed_time(dt_s: f32, steps: usize) -> f32 {
        let controller = XYPositionVelocityController::new(PositionControllerConfig {
            position_gain: 0.0,
            velocity_gain: 0.0,
            max_horizontal_velocity_mps: 10.0,
            horizontal_integral_gain: 0.01,
            horizontal_integral_leak: 1.0,
            max_horizontal_integral_accel_mps2: 10.0,
            max_horizontal_accel_mps2: 10.0,
            max_tilt_rad: 1.0,
        });
        let setpoint = PositionControllerSetpoint::with_velocity_acceleration(
            [0.0; 3],
            [1.0, 0.0, 0.0],
            [0.0; 3],
            0.0,
        );
        let input = PositionControllerInput::new([0.0; 3], [0.0; 3]);
        let mut output = None;

        for _ in 0..steps {
            output = Some(
                controller
                    .update(setpoint, input, dt_s)
                    .expect("finite integrator update should remain valid"),
            );
        }

        output.unwrap().integral_accel_ned_mps2[0]
    }

    fn position_debug_with_tilt(roll_rad: f32, pitch_rad: f32) -> PositionControllerDebug {
        PositionControllerDebug {
            position_error_ned_m: [0.0; 3],
            velocity_setpoint_ned_mps: [0.0; 2],
            velocity_error_ned_mps: [0.0; 2],
            feed_forward_accel_ned_mps2: [0.0; 2],
            requested_accel_ned_mps2: [0.3, 0.0],
            applied_accel_ned_mps2: [0.25, 0.0],
            integral_accel_ned_mps2: [0.0; 2],
            roll_rad,
            pitch_rad,
            yaw_rad: 0.0,
        }
    }
}
