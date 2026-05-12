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
    MotorOrder, MotorOutputs, QUAD_MOTOR_COUNT, QuadXMixer, TorqueCommand,
};
use crate::control::position::{
    PositionController, PositionControllerInput, PositionControllerSetpoint,
};
use crate::control::rate::{BodyRates, RateController};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightControlConfig {
    pub loop_config: ControlLoopConfig,
}

impl Default for FlightControlConfig {
    fn default() -> Self {
        Self {
            loop_config: ControlLoopConfig::default(),
        }
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlSetpoint {
    pub position_ned_m: [f32; 3],
    pub yaw_rad: f32,
    pub armed: bool,
}

impl ControlSetpoint {
    pub const ORIGIN_HOLD_ARMED: Self = Self {
        position_ned_m: [0.0, 0.0, 0.0],
        yaw_rad: 0.0,
        armed: true,
    };

    pub const fn new(position_ned_m: [f32; 3], yaw_rad: f32, armed: bool) -> Self {
        Self {
            position_ned_m,
            yaw_rad,
            armed,
        }
    }

    pub const fn from_position_setpoint(setpoint: PositionControllerSetpoint, armed: bool) -> Self {
        Self {
            position_ned_m: setpoint.position_ned_m,
            yaw_rad: setpoint.yaw_rad,
            armed,
        }
    }

    pub const fn position_setpoint(self) -> PositionControllerSetpoint {
        PositionControllerSetpoint::new(self.position_ned_m, self.yaw_rad)
    }

    pub fn finite(self) -> bool {
        finite_f32x3(self.position_ned_m) && self.yaw_rad.is_finite()
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

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControlDebug {
    pub attitude_setpoint: Option<AttitudeSetpoint>,
    pub rate_setpoint: Option<BodyRateSetpoint>,
    pub throttle: Option<f32>,
    pub torque_command: Option<TorqueCommand>,
    pub motor_outputs: Option<MotorOutputs<QUAD_MOTOR_COUNT>>,
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightControlPipeline {
    position: PositionController,
    altitude: AltitudeController,
    attitude: AttitudeController,
    rate: RateController,
    mixer: QuadXMixer,
}

impl FlightControlPipeline {
    pub fn new(config: FlightControlConfig) -> Self {
        let loop_config = config.loop_config;
        Self {
            position: PositionController::new(loop_config.position),
            altitude: AltitudeController::new(loop_config.altitude),
            attitude: AttitudeController::new(loop_config.attitude),
            rate: RateController::new(loop_config.rate),
            mixer: QuadXMixer::new(loop_config.mixer_limits),
        }
    }

    pub fn set_motor_order(&mut self, motor_order: MotorOrder) {
        self.mixer.set_motor_order(motor_order);
    }

    pub const fn motor_order(&self) -> MotorOrder {
        self.mixer.motor_order()
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
        if !input.setpoint.armed {
            return ControlOutput::invalid_zero(
                input.estimate,
                input.imu.unwrap_or_default(),
                false,
                ControlDebug::default(),
            );
        }

        let Some(imu) = input.imu else {
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
            return ControlOutput::invalid_zero(input.estimate, imu, true, ControlDebug::default());
        }

        let Some(attitude_setpoint) = self.position.update(
            input.setpoint.position_setpoint(),
            PositionControllerInput::new(
                input.estimate.position_ned_m,
                input.estimate.velocity_ned_mps,
            ),
        ) else {
            return ControlOutput::invalid_zero(input.estimate, imu, true, ControlDebug::default());
        };
        let debug = ControlDebug {
            attitude_setpoint: Some(attitude_setpoint),
            ..ControlDebug::default()
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

        let Some(throttle) = self.altitude.update(
            input.setpoint.position_ned_m[2],
            input.estimate.position_ned_m[2],
            input.estimate.velocity_ned_mps[2],
        ) else {
            return ControlOutput::invalid_zero(input.estimate, imu, true, debug);
        };
        let debug = ControlDebug {
            throttle: Some(throttle),
            ..debug
        };

        let torque_command = self.rate.update(
            rate_setpoint,
            BodyRates::from_gyro_rad_s(imu.gyro_rps),
            throttle,
        );
        let motor_outputs = self.mixer.mix(torque_command);
        let debug = ControlDebug {
            torque_command: Some(torque_command),
            motor_outputs: Some(motor_outputs),
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

#[cfg(test)]
mod tests {
    use super::{
        ControlInput, ControlSetpoint, EstimateSnapshot, FlightControlConfig,
        FlightControlPipeline, ImuControlInput,
    };
    use crate::control::config::ControlLoopConfig;
    use crate::control::mixing::MixerLimits;
    use crate::control::rate::RateControllerConfig;

    #[test]
    fn portable_control_disarmed_outputs_zero_motors() {
        let output = FlightControlPipeline::default().step_input(ControlInput::new(
            EstimateSnapshot::LEVEL_ORIGIN,
            ImuControlInput::default(),
            ControlSetpoint {
                armed: false,
                ..ControlSetpoint::ORIGIN_HOLD_ARMED
            },
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
        let pipeline = FlightControlPipeline::new(FlightControlConfig {
            loop_config: config,
        });

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
}
