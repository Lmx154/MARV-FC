//! Portable estimate-to-actuator control step.
//!
//! This module owns the controller composition used by host tests and firmware.
//! Callers are responsible for gathering fresh estimates, IMU samples, setpoints,
//! and arming state; this step validates those inputs and produces normalized
//! quad-X motor commands.

use crate::control::altitude::AltitudeController;
use crate::control::attitude::{AttitudeController, AttitudeSetpoint, BodyRateSetpoint};
use crate::control::config::ControlLoopConfig;
use crate::control::mixing::{
    DesaturationReport, MixerLimitFlags, MotorGeometry, MotorOrder, MotorOutputs,
    PhysicalControlAllocator, QUAD_MOTOR_COUNT, TorqueCommand,
};
use crate::control::position::{
    PositionController, PositionControllerDebug, PositionControllerInput,
    PositionControllerSetpoint, ThrustVectorConfig, ThrustVectorController, ThrustVectorSetpoint,
};
use crate::control::rate::{BodyRates, RateController, RateControllerOutput, RateLimitFlags};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightControlConfig {
    pub loop_config: ControlLoopConfig,
    pub motor_geometry: MotorGeometry,
}

impl FlightControlConfig {
    pub const fn new(loop_config: ControlLoopConfig, motor_geometry: MotorGeometry) -> Self {
        Self {
            loop_config,
            motor_geometry,
        }
    }

    pub const fn with_loop_config(loop_config: ControlLoopConfig) -> Self {
        Self::new(loop_config, MotorGeometry::quad_x())
    }
}

impl Default for FlightControlConfig {
    fn default() -> Self {
        Self::with_loop_config(ControlLoopConfig::default())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimateSnapshot {
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub quaternion: [f32; 4],
    pub valid: bool,
}

impl EstimateSnapshot {
    pub const LEVEL_ORIGIN: Self = Self {
        position_ned_m: [0.0, 0.0, 0.0],
        velocity_ned_mps: [0.0, 0.0, 0.0],
        quaternion: [1.0, 0.0, 0.0, 0.0],
        valid: true,
    };

    pub fn finite(self) -> bool {
        finite_f32x3(self.position_ned_m)
            && finite_f32x3(self.velocity_ned_mps)
            && finite_f32x4(self.quaternion)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TruthEvidence {
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub yaw_rad: f32,
}

impl TruthEvidence {
    pub const fn new(position_ned_m: [f32; 3], velocity_ned_mps: [f32; 3], yaw_rad: f32) -> Self {
        Self {
            position_ned_m,
            velocity_ned_mps,
            yaw_rad,
        }
    }

    pub fn horizontal_error_to(self, target_ned_m: [f32; 3]) -> Option<f32> {
        if !finite_f32x3(self.position_ned_m) || !finite_f32x3(target_ned_m) {
            return None;
        }

        let north_error_m = target_ned_m[0] - self.position_ned_m[0];
        let east_error_m = target_ned_m[1] - self.position_ned_m[1];
        Some(micromath::F32Ext::sqrt(
            north_error_m * north_error_m + east_error_m * east_error_m,
        ))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatorLocalState {
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub yaw_rad: f32,
    pub valid: bool,
}

impl EstimatorLocalState {
    pub const fn new(
        position_ned_m: [f32; 3],
        velocity_ned_mps: [f32; 3],
        yaw_rad: f32,
        valid: bool,
    ) -> Self {
        Self {
            position_ned_m,
            velocity_ned_mps,
            yaw_rad,
            valid,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EstimatorResetDelta {
    pub xy_reset_counter: u32,
    pub z_reset_counter: u32,
    pub velocity_reset_counter: u32,
    pub heading_reset_counter: u32,
    pub position_delta_ned_m: [f32; 3],
    pub velocity_delta_ned_mps: [f32; 3],
    pub heading_delta_rad: f32,
}

impl EstimatorResetDelta {
    pub const NONE: Self = Self {
        xy_reset_counter: 0,
        z_reset_counter: 0,
        velocity_reset_counter: 0,
        heading_reset_counter: 0,
        position_delta_ned_m: [0.0; 3],
        velocity_delta_ned_mps: [0.0; 3],
        heading_delta_rad: 0.0,
    };

    pub fn finite(self) -> bool {
        finite_f32x3(self.position_delta_ned_m)
            && finite_f32x3(self.velocity_delta_ned_mps)
            && self.heading_delta_rad.is_finite()
    }

    pub const fn has_reset(self) -> bool {
        self.xy_reset_counter != 0
            || self.z_reset_counter != 0
            || self.velocity_reset_counter != 0
            || self.heading_reset_counter != 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MissionWaypoint {
    pub position_ned_m: [f32; 3],
    pub yaw_rad: f32,
}

impl MissionWaypoint {
    pub const fn new(position_ned_m: [f32; 3], yaw_rad: f32) -> Self {
        Self {
            position_ned_m,
            yaw_rad,
        }
    }

    pub const fn into_local_trajectory(self) -> LocalTrajectorySetpoint {
        LocalTrajectorySetpoint::position_hold(self.position_ned_m, self.yaw_rad)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LocalTrajectorySetpoint {
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub acceleration_ned_mps2: [f32; 3],
    pub yaw_rad: f32,
    pub yaw_rate_rad_s: f32,
}

impl LocalTrajectorySetpoint {
    pub const ORIGIN_HOLD: Self = Self::position_hold([0.0, 0.0, 0.0], 0.0);

    pub const fn position_hold(position_ned_m: [f32; 3], yaw_rad: f32) -> Self {
        Self {
            position_ned_m,
            velocity_ned_mps: [0.0, 0.0, 0.0],
            acceleration_ned_mps2: [0.0, 0.0, 0.0],
            yaw_rad,
            yaw_rate_rad_s: 0.0,
        }
    }

    pub fn finite(self) -> bool {
        finite_f32x3(self.position_ned_m)
            && finite_f32x3(self.velocity_ned_mps)
            && finite_f32x3(self.acceleration_ned_mps2)
            && self.yaw_rad.is_finite()
            && self.yaw_rate_rad_s.is_finite()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuControlInput {
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
}

impl ImuControlInput {
    pub fn finite(self) -> bool {
        finite_f32x3(self.accel_mps2) && finite_f32x3(self.gyro_rps)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ControlSetpointSource {
    EstimatorLocalTrajectory,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlSetpoint {
    position_ned_m: [f32; 3],
    velocity_ned_mps: [f32; 3],
    acceleration_ned_mps2: [f32; 3],
    yaw_rad: f32,
    armed: bool,
    source: ControlSetpointSource,
}

impl ControlSetpoint {
    pub const ORIGIN_HOLD_ARMED: Self = Self {
        position_ned_m: [0.0, 0.0, 0.0],
        velocity_ned_mps: [0.0, 0.0, 0.0],
        acceleration_ned_mps2: [0.0, 0.0, 0.0],
        yaw_rad: 0.0,
        armed: true,
        source: ControlSetpointSource::EstimatorLocalTrajectory,
    };

    pub const fn local_position_ned(position_ned_m: [f32; 3], yaw_rad: f32, armed: bool) -> Self {
        Self::from_local_trajectory(
            LocalTrajectorySetpoint::position_hold(position_ned_m, yaw_rad),
            armed,
        )
    }

    pub const fn from_local_trajectory(setpoint: LocalTrajectorySetpoint, armed: bool) -> Self {
        Self {
            position_ned_m: setpoint.position_ned_m,
            velocity_ned_mps: setpoint.velocity_ned_mps,
            acceleration_ned_mps2: setpoint.acceleration_ned_mps2,
            yaw_rad: setpoint.yaw_rad,
            armed,
            source: ControlSetpointSource::EstimatorLocalTrajectory,
        }
    }

    pub const fn from_position_setpoint(setpoint: PositionControllerSetpoint, armed: bool) -> Self {
        Self {
            position_ned_m: setpoint.position_ned_m,
            velocity_ned_mps: setpoint.velocity_ned_mps,
            acceleration_ned_mps2: setpoint.acceleration_ned_mps2,
            yaw_rad: setpoint.yaw_rad,
            armed,
            source: ControlSetpointSource::EstimatorLocalTrajectory,
        }
    }

    pub const fn position_ned_m(self) -> [f32; 3] {
        self.position_ned_m
    }

    pub const fn yaw_rad(self) -> f32 {
        self.yaw_rad
    }

    pub const fn velocity_ned_mps(self) -> [f32; 3] {
        self.velocity_ned_mps
    }

    pub const fn acceleration_ned_mps2(self) -> [f32; 3] {
        self.acceleration_ned_mps2
    }

    pub const fn armed(self) -> bool {
        self.armed
    }

    pub const fn source(self) -> ControlSetpointSource {
        self.source
    }

    pub const fn position_setpoint(self) -> PositionControllerSetpoint {
        PositionControllerSetpoint::with_velocity_acceleration(
            self.position_ned_m,
            self.velocity_ned_mps,
            self.acceleration_ned_mps2,
            self.yaw_rad,
        )
    }

    pub fn finite(self) -> bool {
        finite_f32x3(self.position_ned_m)
            && finite_f32x3(self.velocity_ned_mps)
            && finite_f32x3(self.acceleration_ned_mps2)
            && self.yaw_rad.is_finite()
    }

    pub fn adjusted_for_estimator_reset(self, delta: EstimatorResetDelta) -> Option<Self> {
        if !self.finite() || !delta.finite() {
            return None;
        }

        Some(Self {
            position_ned_m: [
                self.position_ned_m[0] + delta.position_delta_ned_m[0],
                self.position_ned_m[1] + delta.position_delta_ned_m[1],
                self.position_ned_m[2] + delta.position_delta_ned_m[2],
            ],
            velocity_ned_mps: [
                self.velocity_ned_mps[0] + delta.velocity_delta_ned_mps[0],
                self.velocity_ned_mps[1] + delta.velocity_delta_ned_mps[1],
                self.velocity_ned_mps[2] + delta.velocity_delta_ned_mps[2],
            ],
            acceleration_ned_mps2: self.acceleration_ned_mps2,
            yaw_rad: wrap_pi(self.yaw_rad + delta.heading_delta_rad),
            armed: self.armed,
            source: self.source,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlInput {
    pub estimate: EstimateSnapshot,
    pub imu: Option<ImuControlInput>,
    pub setpoint: ControlSetpoint,
}

impl ControlInput {
    pub const fn new(
        estimate: EstimateSnapshot,
        imu: ImuControlInput,
        setpoint: ControlSetpoint,
    ) -> Self {
        Self {
            estimate,
            imu: Some(imu),
            setpoint,
        }
    }

    pub const fn without_imu(estimate: EstimateSnapshot, setpoint: ControlSetpoint) -> Self {
        Self {
            estimate,
            imu: None,
            setpoint,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ControlFrame {
    #[default]
    Unknown,
    EstimatorLocalNed,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ControlNavPhase {
    #[default]
    Unknown,
    DirectSetpoint,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ClampSource {
    #[default]
    None,
    MotorOutputLimit,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlDebug {
    pub estimator_frame: ControlFrame,
    pub setpoint_frame: ControlFrame,
    pub nav_phase: ControlNavPhase,
    pub takeoff_origin_ned_m: Option<[f32; 3]>,
    pub control_setpoint_ned_m: Option<[f32; 3]>,
    pub position_error_ned_m: Option<[f32; 3]>,
    pub velocity_setpoint_ned_mps: Option<[f32; 2]>,
    pub velocity_error_ned_mps: Option<[f32; 2]>,
    pub feed_forward_accel_ned_mps2: Option<[f32; 2]>,
    pub requested_accel_ned_mps2: Option<[f32; 2]>,
    pub applied_accel_ned_mps2: Option<[f32; 2]>,
    pub integral_accel_ned_mps2: Option<[f32; 2]>,
    pub thrust_vector_setpoint: Option<ThrustVectorSetpoint>,
    pub requested_collective_throttle: Option<f32>,
    pub applied_collective_throttle: Option<f32>,
    pub tilt_compensation: Option<f32>,
    pub vertical_throttle_margin: Option<f32>,
    pub attitude_setpoint: Option<AttitudeSetpoint>,
    pub rate_setpoint: Option<BodyRateSetpoint>,
    pub throttle: Option<f32>,
    pub rate_controller_output: Option<RateControllerOutput>,
    pub rate_limit_flags: RateLimitFlags,
    pub torque_command: Option<TorqueCommand>,
    pub motor_outputs: Option<MotorOutputs<QUAD_MOTOR_COUNT>>,
    pub mixer_limit_flags: MixerLimitFlags,
    pub desaturation: DesaturationReport,
    pub clamp_source: ClampSource,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlOutput {
    pub estimate: EstimateSnapshot,
    pub imu: ImuControlInput,
    pub attitude_setpoint: Option<AttitudeSetpoint>,
    pub rate_setpoint: Option<BodyRateSetpoint>,
    pub throttle: f32,
    pub torque_command: TorqueCommand,
    pub motors: [f32; QUAD_MOTOR_COUNT],
    pub clamped: bool,
    pub control_valid: bool,
    pub armed: bool,
    pub debug: ControlDebug,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FlightControlPipeline {
    position: PositionController,
    altitude: AltitudeController,
    thrust_vector: ThrustVectorController,
    attitude: AttitudeController,
    rate: RateController,
    allocator: PhysicalControlAllocator,
}

impl FlightControlPipeline {
    pub fn new(config: FlightControlConfig) -> Self {
        let loop_config = config.loop_config;
        Self {
            position: PositionController::new(loop_config.position),
            altitude: AltitudeController::new(loop_config.altitude),
            thrust_vector: ThrustVectorController::new(ThrustVectorConfig {
                hover_throttle: loop_config.altitude.hover_throttle,
                min_collective_throttle: 0.0,
                max_collective_throttle: 1.0,
                min_tilt_cosine: 0.25,
            }),
            attitude: AttitudeController::new(loop_config.attitude),
            rate: RateController::new(loop_config.rate),
            allocator: PhysicalControlAllocator::new(
                config.motor_geometry,
                loop_config.mixer_limits,
            ),
        }
    }

    pub fn set_motor_order(&mut self, motor_order: MotorOrder) {
        self.allocator.set_motor_order(motor_order);
    }

    pub fn motor_order(&self) -> MotorOrder {
        self.allocator.motor_order().unwrap_or(MotorOrder::IDENTITY)
    }

    pub fn reset_position_integrators(&self) {
        self.position.reset();
    }

    pub fn step(
        &self,
        estimate: EstimateSnapshot,
        imu: ImuControlInput,
        setpoint: ControlSetpoint,
    ) -> ControlOutput {
        self.step_input(ControlInput::new(estimate, imu, setpoint))
    }

    pub fn step_input(&self, input: ControlInput) -> ControlOutput {
        if !input.setpoint.armed() {
            self.position.reset();
            return ControlOutput::invalid_zero(
                input.estimate,
                input.imu.unwrap_or_default(),
                false,
                ControlDebug::default(),
            );
        }

        let Some(imu) = input.imu else {
            self.position.reset();
            return ControlOutput::invalid_zero(
                input.estimate,
                ImuControlInput::default(),
                true,
                ControlDebug::default(),
            );
        };

        if !input.estimate.valid
            || !input.estimate.finite()
            || !imu.finite()
            || !input.setpoint.finite()
        {
            self.position.reset();
            return ControlOutput::invalid_zero(input.estimate, imu, true, ControlDebug::default());
        }

        let Some(position_output) = self.position.update_with_debug(
            input.setpoint.position_setpoint(),
            PositionControllerInput::new(
                input.estimate.position_ned_m,
                input.estimate.velocity_ned_mps,
            ),
        ) else {
            return ControlOutput::invalid_zero(input.estimate, imu, true, ControlDebug::default());
        };
        let attitude_setpoint = position_output.attitude_setpoint;
        let debug = ControlDebug {
            estimator_frame: ControlFrame::EstimatorLocalNed,
            setpoint_frame: ControlFrame::EstimatorLocalNed,
            nav_phase: ControlNavPhase::DirectSetpoint,
            takeoff_origin_ned_m: None,
            control_setpoint_ned_m: Some(input.setpoint.position_ned_m()),
            position_error_ned_m: Some(position_output.debug.position_error_ned_m),
            velocity_setpoint_ned_mps: Some(position_output.debug.velocity_setpoint_ned_mps),
            velocity_error_ned_mps: Some(position_output.debug.velocity_error_ned_mps),
            feed_forward_accel_ned_mps2: Some(position_output.debug.feed_forward_accel_ned_mps2),
            requested_accel_ned_mps2: Some(position_output.debug.requested_accel_ned_mps2),
            applied_accel_ned_mps2: Some(position_output.debug.applied_accel_ned_mps2),
            integral_accel_ned_mps2: Some(position_output.debug.integral_accel_ned_mps2),
            attitude_setpoint: Some(attitude_setpoint),
            ..ControlDebug::default()
        };

        let Some(vertical_throttle) = self.altitude.update(
            input.setpoint.position_ned_m()[2],
            input.estimate.position_ned_m[2],
            input.estimate.velocity_ned_mps[2],
        ) else {
            return ControlOutput::invalid_zero(input.estimate, imu, true, debug);
        };

        let Some(thrust_vector_setpoint) =
            self.thrust_vector
                .update(attitude_setpoint, position_output.debug, vertical_throttle)
        else {
            return ControlOutput::invalid_zero(input.estimate, imu, true, debug);
        };
        let attitude_setpoint = thrust_vector_setpoint.attitude_setpoint;
        let throttle = thrust_vector_setpoint.applied_collective_throttle;
        let debug = ControlDebug {
            throttle: Some(throttle),
            thrust_vector_setpoint: Some(thrust_vector_setpoint),
            requested_collective_throttle: Some(
                thrust_vector_setpoint.requested_collective_throttle,
            ),
            applied_collective_throttle: Some(thrust_vector_setpoint.applied_collective_throttle),
            tilt_compensation: Some(thrust_vector_setpoint.tilt_compensation),
            vertical_throttle_margin: Some(thrust_vector_setpoint.vertical_throttle_margin),
            attitude_setpoint: Some(attitude_setpoint),
            ..debug
        };

        let Some(rate_setpoint) = self
            .attitude
            .update(attitude_setpoint, input.estimate.quaternion)
        else {
            return ControlOutput::invalid_zero(input.estimate, imu, true, debug);
        };
        let debug = ControlDebug {
            rate_setpoint: Some(rate_setpoint),
            ..debug
        };

        let rate_output = self.rate.update_with_debug(
            rate_setpoint,
            BodyRates::from_gyro_rad_s(imu.gyro_rps),
            throttle,
        );
        let torque_command = rate_output.command;
        let motor_outputs = self.allocator.allocate(torque_command);
        let debug = ControlDebug {
            rate_controller_output: Some(rate_output),
            rate_limit_flags: rate_output.limit_flags,
            torque_command: Some(torque_command),
            motor_outputs: Some(motor_outputs),
            mixer_limit_flags: motor_outputs.limit_flags,
            desaturation: motor_outputs.desaturation,
            clamp_source: if motor_outputs.clamped {
                ClampSource::MotorOutputLimit
            } else {
                ClampSource::None
            },
            ..debug
        };

        ControlOutput {
            estimate: input.estimate,
            imu,
            attitude_setpoint: Some(attitude_setpoint),
            rate_setpoint: Some(rate_setpoint),
            throttle,
            torque_command,
            motors: motor_outputs.commands,
            clamped: motor_outputs.clamped,
            control_valid: true,
            armed: true,
            debug,
        }
    }
}

impl From<PositionControllerDebug> for ControlDebug {
    fn from(position: PositionControllerDebug) -> Self {
        Self {
            position_error_ned_m: Some(position.position_error_ned_m),
            velocity_setpoint_ned_mps: Some(position.velocity_setpoint_ned_mps),
            velocity_error_ned_mps: Some(position.velocity_error_ned_mps),
            feed_forward_accel_ned_mps2: Some(position.feed_forward_accel_ned_mps2),
            requested_accel_ned_mps2: Some(position.requested_accel_ned_mps2),
            applied_accel_ned_mps2: Some(position.applied_accel_ned_mps2),
            integral_accel_ned_mps2: Some(position.integral_accel_ned_mps2),
            ..Self::default()
        }
    }
}

impl Default for FlightControlPipeline {
    fn default() -> Self {
        Self::new(FlightControlConfig::default())
    }
}

impl ControlOutput {
    fn invalid_zero(
        estimate: EstimateSnapshot,
        imu: ImuControlInput,
        armed: bool,
        debug: ControlDebug,
    ) -> Self {
        Self {
            estimate,
            imu,
            attitude_setpoint: debug.attitude_setpoint,
            rate_setpoint: debug.rate_setpoint,
            throttle: debug.throttle.unwrap_or(0.0),
            torque_command: debug.torque_command.unwrap_or_default(),
            motors: [0.0; QUAD_MOTOR_COUNT],
            clamped: false,
            control_valid: false,
            armed,
            debug,
        }
    }
}

fn finite_f32x3(values: [f32; 3]) -> bool {
    values.iter().all(|value| value.is_finite())
}

fn finite_f32x4(values: [f32; 4]) -> bool {
    values.iter().all(|value| value.is_finite())
}

fn wrap_pi(mut angle: f32) -> f32 {
    let two_pi = 2.0 * core::f32::consts::PI;
    while angle > core::f32::consts::PI {
        angle -= two_pi;
    }
    while angle < -core::f32::consts::PI {
        angle += two_pi;
    }
    angle
}

#[cfg(test)]
mod tests {
    use super::{
        ControlInput, ControlSetpoint, ControlSetpointSource, EstimateSnapshot,
        EstimatorResetDelta, FlightControlConfig, FlightControlPipeline, ImuControlInput,
        LocalTrajectorySetpoint, MissionWaypoint, TruthEvidence,
    };
    use crate::control::config::ControlLoopConfig;
    use crate::control::mixing::MixerLimits;
    use crate::control::rate::RateControllerConfig;

    #[test]
    fn portable_control_disarmed_outputs_zero_motors() {
        let output = FlightControlPipeline::default().step_input(ControlInput::new(
            EstimateSnapshot::LEVEL_ORIGIN,
            ImuControlInput::default(),
            ControlSetpoint::local_position_ned([0.0, 0.0, 0.0], 0.0, false),
        ));

        assert!(!output.armed);
        assert!(!output.control_valid);
        assert_eq!(output.motors, [0.0; 4]);
        assert!(!output.clamped);
        assert!(output.debug.attitude_setpoint.is_none());
        assert!(output.debug.rate_setpoint.is_none());
    }

    #[test]
    fn portable_control_invalid_estimate_outputs_zero_motors() {
        let output = FlightControlPipeline::default().step_input(ControlInput::new(
            EstimateSnapshot {
                valid: false,
                ..EstimateSnapshot::LEVEL_ORIGIN
            },
            ImuControlInput::default(),
            ControlSetpoint::ORIGIN_HOLD_ARMED,
        ));

        assert!(output.armed);
        assert!(!output.control_valid);
        assert_eq!(output.motors, [0.0; 4]);
        assert!(output.debug.motor_outputs.is_none());
    }

    #[test]
    fn portable_control_missing_imu_outputs_zero_motors() {
        let output = FlightControlPipeline::default().step_input(ControlInput::without_imu(
            EstimateSnapshot::LEVEL_ORIGIN,
            ControlSetpoint::ORIGIN_HOLD_ARMED,
        ));

        assert!(output.armed);
        assert!(!output.control_valid);
        assert_eq!(output.motors, [0.0; 4]);
    }

    #[test]
    fn portable_control_level_hold_outputs_hover_range_motors() {
        let output = FlightControlPipeline::default().step(
            EstimateSnapshot::LEVEL_ORIGIN,
            ImuControlInput::default(),
            ControlSetpoint::ORIGIN_HOLD_ARMED,
        );

        assert!(output.control_valid);
        assert_eq!(output.debug.throttle, Some(0.5));
        assert!(
            output
                .motors
                .iter()
                .all(|motor| (0.0..=1.0).contains(motor))
        );
        assert_eq!(output.motors, [0.5; 4]);
        assert!(!output.clamped);
    }

    #[test]
    fn portable_control_saturation_sets_clamped_flag() {
        let mut config = ControlLoopConfig::default();
        config.rate = RateControllerConfig {
            roll_gain: 4.0,
            pitch_gain: 4.0,
            yaw_gain: 4.0,
            max_axis_command: 2.0,
            measured_rate_deadband_rps: 0.0,
        };
        config.mixer_limits = MixerLimits::NORMALIZED;
        let pipeline = FlightControlPipeline::new(FlightControlConfig::with_loop_config(config));

        let output = pipeline.step(
            EstimateSnapshot::LEVEL_ORIGIN,
            ImuControlInput {
                accel_mps2: [0.0, 0.0, 0.0],
                gyro_rps: [-1.0, -1.0, 0.0],
            },
            ControlSetpoint::ORIGIN_HOLD_ARMED,
        );

        assert!(output.control_valid);
        assert!(output.clamped);
        assert!(
            output
                .motors
                .iter()
                .all(|motor| (0.0..=1.0).contains(motor))
        );
        assert!(output.motors.iter().any(|motor| *motor >= 1.0));
    }

    #[test]
    fn control_setpoint_is_constructed_from_local_trajectory() {
        let trajectory = LocalTrajectorySetpoint {
            position_ned_m: [1.0, -2.0, -3.0],
            velocity_ned_mps: [0.5, -0.25, 0.0],
            acceleration_ned_mps2: [0.1, 0.2, 0.0],
            yaw_rad: 0.25,
            yaw_rate_rad_s: 0.0,
        };
        let setpoint = ControlSetpoint::from_local_trajectory(trajectory, true);

        assert_eq!(setpoint.position_ned_m(), [1.0, -2.0, -3.0]);
        assert_eq!(setpoint.velocity_ned_mps(), [0.5, -0.25, 0.0]);
        assert_eq!(setpoint.acceleration_ned_mps2(), [0.1, 0.2, 0.0]);
        assert_eq!(setpoint.yaw_rad(), 0.25);
        assert!(setpoint.armed());
        assert_eq!(
            setpoint.source(),
            ControlSetpointSource::EstimatorLocalTrajectory
        );
        assert!(setpoint.finite());
    }

    #[test]
    fn mission_waypoint_must_convert_through_local_trajectory() {
        let mission = MissionWaypoint::new([2.0, 3.0, -1.0], 0.5);
        let setpoint =
            ControlSetpoint::from_local_trajectory(mission.into_local_trajectory(), true);

        assert_eq!(setpoint.position_ned_m(), [2.0, 3.0, -1.0]);
        assert_eq!(setpoint.yaw_rad(), 0.5);
        assert_eq!(
            setpoint.source(),
            ControlSetpointSource::EstimatorLocalTrajectory
        );
    }

    #[test]
    fn truth_evidence_scores_but_does_not_build_control_setpoints() {
        let truth = TruthEvidence::new([3.0, 4.0, -1.0], [0.0, 0.0, 0.0], 0.0);
        let error_m = truth
            .horizontal_error_to([0.0, 0.0, -1.0])
            .expect("finite truth evidence should score target error");

        assert!((error_m - 5.0).abs() < 0.15, "error_m={error_m}");
    }

    #[test]
    fn control_setpoint_adjusts_for_estimator_reset_delta() {
        let setpoint = ControlSetpoint::from_local_trajectory(
            LocalTrajectorySetpoint {
                position_ned_m: [10.0, -2.0, -3.0],
                velocity_ned_mps: [0.4, -0.2, 0.1],
                acceleration_ned_mps2: [0.01, 0.02, 0.0],
                yaw_rad: 3.0,
                yaw_rate_rad_s: 0.0,
            },
            true,
        );
        let adjusted = setpoint
            .adjusted_for_estimator_reset(EstimatorResetDelta {
                xy_reset_counter: 1,
                z_reset_counter: 1,
                velocity_reset_counter: 1,
                heading_reset_counter: 1,
                position_delta_ned_m: [-4.0, 1.5, 0.25],
                velocity_delta_ned_mps: [0.1, 0.2, -0.1],
                heading_delta_rad: 0.5,
            })
            .expect("finite reset delta should adjust setpoint");

        assert_eq!(adjusted.position_ned_m(), [6.0, -0.5, -2.75]);
        assert_eq!(adjusted.velocity_ned_mps(), [0.5, 0.0, 0.0]);
        assert_eq!(adjusted.acceleration_ned_mps2(), [0.01, 0.02, 0.0]);
        assert!(adjusted.yaw_rad() < -2.7);
        assert!(adjusted.armed());
    }
}
