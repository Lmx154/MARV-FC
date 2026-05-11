use common::control::altitude::AltitudeController;
use common::control::attitude::{AttitudeController, AttitudeSetpoint, BodyRateSetpoint};
use common::control::config::ControlLoopConfig;
use common::control::mixing::{MotorOutputs, QUAD_MOTOR_COUNT, QuadXMixer, TorqueCommand};
use common::control::position::{
    PositionController, PositionControllerInput, PositionControllerSetpoint,
};
use common::control::rate::{BodyRates, RateController};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PureControlConfig {
    pub loop_config: ControlLoopConfig,
}

impl Default for PureControlConfig {
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
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuControlInput {
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
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
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlPipelineTrace {
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
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControlPipeline {
    position: PositionController,
    altitude: AltitudeController,
    attitude: AttitudeController,
    rate: RateController,
    mixer: QuadXMixer,
}

impl ControlPipeline {
    pub fn new(config: PureControlConfig) -> Self {
        let loop_config = config.loop_config;
        Self {
            position: PositionController::new(loop_config.position),
            altitude: AltitudeController::new(loop_config.altitude),
            attitude: AttitudeController::new(loop_config.attitude),
            rate: RateController::new(loop_config.rate),
            mixer: QuadXMixer::new(loop_config.mixer_limits),
        }
    }

    pub fn step(
        &self,
        estimate: EstimateSnapshot,
        imu: ImuControlInput,
        setpoint: ControlSetpoint,
    ) -> ControlPipelineTrace {
        if !setpoint.armed || !estimate.valid {
            return ControlPipelineTrace::invalid_zero(estimate, imu, setpoint.armed);
        }

        let Some(attitude_setpoint) = self.position.update(
            PositionControllerSetpoint::new(setpoint.position_ned_m, setpoint.yaw_rad),
            PositionControllerInput::new(estimate.position_ned_m, estimate.velocity_ned_mps),
        ) else {
            return ControlPipelineTrace::invalid_zero(estimate, imu, true);
        };
        let Some(throttle) = self.altitude.update(
            setpoint.position_ned_m[2],
            estimate.position_ned_m[2],
            estimate.velocity_ned_mps[2],
        ) else {
            return ControlPipelineTrace::invalid_zero(estimate, imu, true);
        };
        let Some(rate_setpoint) = self.attitude.update(attitude_setpoint, estimate.quaternion)
        else {
            return ControlPipelineTrace::invalid_zero(estimate, imu, true);
        };

        let torque_command = self.rate.update(
            rate_setpoint,
            BodyRates::from_gyro_rad_s(imu.gyro_rps),
            throttle,
        );
        let MotorOutputs { commands, clamped } = self.mixer.mix(torque_command);

        ControlPipelineTrace {
            estimate,
            imu,
            attitude_setpoint: Some(attitude_setpoint),
            rate_setpoint: Some(rate_setpoint),
            throttle,
            torque_command,
            motors: commands,
            clamped,
            control_valid: true,
            armed: true,
        }
    }
}

impl Default for ControlPipeline {
    fn default() -> Self {
        Self::new(PureControlConfig::default())
    }
}

impl ControlPipelineTrace {
    fn invalid_zero(estimate: EstimateSnapshot, imu: ImuControlInput, armed: bool) -> Self {
        Self {
            estimate,
            imu,
            attitude_setpoint: None,
            rate_setpoint: None,
            throttle: 0.0,
            torque_command: TorqueCommand::default(),
            motors: [0.0; QUAD_MOTOR_COUNT],
            clamped: false,
            control_valid: false,
            armed,
        }
    }
}
