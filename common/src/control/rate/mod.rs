//! Hardware-independent body-rate control.

use crate::control::attitude::BodyRateSetpoint;
use crate::control::mixing::TorqueCommand;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BodyRates {
    pub roll_rps: f32,
    pub pitch_rps: f32,
    pub yaw_rps: f32,
}

impl BodyRates {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(roll_rps: f32, pitch_rps: f32, yaw_rps: f32) -> Self {
        Self {
            roll_rps,
            pitch_rps,
            yaw_rps,
        }
    }

    pub const fn from_gyro_rad_s(gyro_rad_s: [f32; 3]) -> Self {
        Self::new(gyro_rad_s[0], gyro_rad_s[1], gyro_rad_s[2])
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateControllerConfig {
    pub roll_gain: f32,
    pub pitch_gain: f32,
    pub yaw_gain: f32,
    pub max_axis_command: f32,
    pub measured_rate_deadband_rps: f32,
}

impl Default for RateControllerConfig {
    fn default() -> Self {
        Self {
            roll_gain: 0.08,
            pitch_gain: 0.08,
            yaw_gain: 0.04,
            max_axis_command: 0.25,
            measured_rate_deadband_rps: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RateLimitFlags {
    pub roll: bool,
    pub pitch: bool,
    pub yaw: bool,
    pub throttle_lower: bool,
    pub throttle_upper: bool,
}

impl RateLimitFlags {
    pub const fn any(self) -> bool {
        self.roll || self.pitch || self.yaw || self.throttle_lower || self.throttle_upper
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RateControllerOutput {
    pub command: TorqueCommand,
    pub body_rate_error_rps: BodyRates,
    pub raw_command: TorqueCommand,
    pub limit_flags: RateLimitFlags,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateController {
    config: RateControllerConfig,
}

impl RateController {
    pub const fn new(config: RateControllerConfig) -> Self {
        Self { config }
    }

    pub fn update(
        &self,
        setpoint: BodyRateSetpoint,
        measured: BodyRates,
        throttle: f32,
    ) -> TorqueCommand {
        self.update_with_debug(setpoint, measured, throttle).command
    }

    pub fn update_with_debug(
        &self,
        setpoint: BodyRateSetpoint,
        measured: BodyRates,
        throttle: f32,
    ) -> RateControllerOutput {
        let measured = BodyRates::new(
            soft_deadband(measured.roll_rps, self.config.measured_rate_deadband_rps),
            soft_deadband(measured.pitch_rps, self.config.measured_rate_deadband_rps),
            soft_deadband(measured.yaw_rps, self.config.measured_rate_deadband_rps),
        );
        let body_rate_error_rps = BodyRates::new(
            setpoint.roll_rps - measured.roll_rps,
            setpoint.pitch_rps - measured.pitch_rps,
            setpoint.yaw_rps - measured.yaw_rps,
        );
        let raw_command = TorqueCommand::new(
            self.config.roll_gain * body_rate_error_rps.roll_rps,
            self.config.pitch_gain * body_rate_error_rps.pitch_rps,
            self.config.yaw_gain * body_rate_error_rps.yaw_rps,
            throttle,
        );
        let (roll, roll_limited) =
            clamp_symmetric_with_limit(raw_command.roll, self.config.max_axis_command);
        let (pitch, pitch_limited) =
            clamp_symmetric_with_limit(raw_command.pitch, self.config.max_axis_command);
        let (yaw, yaw_limited) =
            clamp_symmetric_with_limit(raw_command.yaw, self.config.max_axis_command);
        let (throttle, throttle_lower, throttle_upper) = clamp_unit_with_flags(throttle);

        RateControllerOutput {
            command: TorqueCommand::new(roll, pitch, yaw, throttle),
            body_rate_error_rps,
            raw_command,
            limit_flags: RateLimitFlags {
                roll: roll_limited,
                pitch: pitch_limited,
                yaw: yaw_limited,
                throttle_lower,
                throttle_upper,
            },
        }
    }
}

impl Default for RateController {
    fn default() -> Self {
        Self::new(RateControllerConfig::default())
    }
}

fn clamp_symmetric_with_limit(value: f32, limit: f32) -> (f32, bool) {
    if !value.is_finite() || !limit.is_finite() || limit <= 0.0 {
        return (0.0, true);
    }

    let clamped = value.clamp(-limit, limit);
    (clamped, clamped != value)
}

fn clamp_unit_with_flags(value: f32) -> (f32, bool, bool) {
    if !value.is_finite() {
        return (0.0, true, false);
    }

    let clamped = value.clamp(0.0, 1.0);
    (clamped, value < 0.0, value > 1.0)
}

fn soft_deadband(value: f32, deadband: f32) -> f32 {
    if !value.is_finite() || !deadband.is_finite() || deadband <= 0.0 {
        return value;
    }

    let magnitude = value.abs();
    if magnitude <= deadband {
        0.0
    } else {
        value.signum() * (magnitude - deadband)
    }
}

#[cfg(test)]
mod tests {
    use super::{BodyRates, RateController, RateControllerConfig};
    use crate::control::attitude::BodyRateSetpoint;
    use crate::test_helpers::assert_scalar_near;

    #[test]
    fn zero_rate_error_preserves_throttle() {
        let controller = RateController::default();

        let command = controller.update(BodyRateSetpoint::ZERO, BodyRates::ZERO, 0.5);

        assert_scalar_near(command.roll, 0.0, 0.000_001);
        assert_scalar_near(command.pitch, 0.0, 0.000_001);
        assert_scalar_near(command.yaw, 0.0, 0.000_001);
        assert_scalar_near(command.throttle, 0.5, 0.000_001);
    }

    #[test]
    fn rate_error_maps_to_torque_command() {
        let controller = RateController::new(RateControllerConfig {
            roll_gain: 0.1,
            pitch_gain: 0.1,
            yaw_gain: 0.1,
            max_axis_command: 0.25,
            measured_rate_deadband_rps: 0.0,
        });

        let command =
            controller.update(BodyRateSetpoint::new(1.0, -1.0, 0.5), BodyRates::ZERO, 0.5);

        assert_scalar_near(command.roll, 0.1, 0.000_001);
        assert_scalar_near(command.pitch, -0.1, 0.000_001);
        assert_scalar_near(command.yaw, 0.05, 0.000_001);
    }

    #[test]
    fn torque_command_is_limited() {
        let controller = RateController::default();

        let command =
            controller.update(BodyRateSetpoint::new(100.0, 0.0, 0.0), BodyRates::ZERO, 2.0);

        assert_eq!(command.roll, 0.25);
        assert_eq!(command.throttle, 1.0);
    }

    #[test]
    fn rate_error_signs_follow_setpoint_minus_measurement() {
        let controller = RateController::new(RateControllerConfig {
            roll_gain: 0.5,
            pitch_gain: 0.25,
            yaw_gain: 0.125,
            max_axis_command: 10.0,
            measured_rate_deadband_rps: 0.0,
        });

        let command = controller.update(
            BodyRateSetpoint::new(1.0, -2.0, 3.0),
            BodyRates::new(3.0, -1.0, 1.0),
            0.4,
        );

        assert_scalar_near(command.roll, -1.0, 0.000_001);
        assert_scalar_near(command.pitch, -0.25, 0.000_001);
        assert_scalar_near(command.yaw, 0.25, 0.000_001);
    }

    #[test]
    fn rate_controller_saturates_each_axis_symmetrically() {
        let controller = RateController::new(RateControllerConfig {
            roll_gain: 1.0,
            pitch_gain: 1.0,
            yaw_gain: 1.0,
            max_axis_command: 0.2,
            measured_rate_deadband_rps: 0.0,
        });

        let command = controller.update(
            BodyRateSetpoint::new(10.0, -10.0, 10.0),
            BodyRates::ZERO,
            -1.0,
        );

        assert_scalar_near(command.roll, 0.2, 0.000_001);
        assert_scalar_near(command.pitch, -0.2, 0.000_001);
        assert_scalar_near(command.yaw, 0.2, 0.000_001);
        assert_scalar_near(command.throttle, 0.0, 0.000_001);
    }

    #[test]
    fn rate_controller_reports_axis_and_throttle_limits() {
        let controller = RateController::new(RateControllerConfig {
            roll_gain: 1.0,
            pitch_gain: 1.0,
            yaw_gain: 1.0,
            max_axis_command: 0.2,
            measured_rate_deadband_rps: 0.0,
        });

        let upper = controller.update_with_debug(
            BodyRateSetpoint::new(10.0, -10.0, 10.0),
            BodyRates::ZERO,
            1.2,
        );
        let lower = controller.update_with_debug(BodyRateSetpoint::ZERO, BodyRates::ZERO, -0.1);

        assert!(upper.limit_flags.roll);
        assert!(upper.limit_flags.pitch);
        assert!(upper.limit_flags.yaw);
        assert!(upper.limit_flags.throttle_upper);
        assert!(upper.limit_flags.any());
        assert!(lower.limit_flags.throttle_lower);
        assert_scalar_near(upper.raw_command.roll, 10.0, 0.000_001);
        assert_scalar_near(upper.command.roll, 0.2, 0.000_001);
    }

    #[test]
    fn measured_rate_deadband_softens_small_gyro_motion() {
        let controller = RateController::new(RateControllerConfig {
            roll_gain: 1.0,
            pitch_gain: 1.0,
            yaw_gain: 1.0,
            max_axis_command: 1.0,
            measured_rate_deadband_rps: 0.02,
        });

        let quiet = controller.update(
            BodyRateSetpoint::ZERO,
            BodyRates::new(0.01, -0.02, 0.0),
            0.5,
        );
        let moving = controller.update(
            BodyRateSetpoint::ZERO,
            BodyRates::new(0.03, -0.05, 0.0),
            0.5,
        );

        assert_scalar_near(quiet.roll, 0.0, 0.000_001);
        assert_scalar_near(quiet.pitch, 0.0, 0.000_001);
        assert_scalar_near(moving.roll, -0.01, 0.000_001);
        assert_scalar_near(moving.pitch, 0.03, 0.000_001);
    }
}
