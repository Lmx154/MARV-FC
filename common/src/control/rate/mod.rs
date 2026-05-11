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
}

impl Default for RateControllerConfig {
    fn default() -> Self {
        Self {
            roll_gain: 0.08,
            pitch_gain: 0.08,
            yaw_gain: 0.04,
            max_axis_command: 0.25,
        }
    }
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
        TorqueCommand::new(
            clamp_symmetric(
                self.config.roll_gain * (setpoint.roll_rps - measured.roll_rps),
                self.config.max_axis_command,
            ),
            clamp_symmetric(
                self.config.pitch_gain * (setpoint.pitch_rps - measured.pitch_rps),
                self.config.max_axis_command,
            ),
            clamp_symmetric(
                self.config.yaw_gain * (setpoint.yaw_rps - measured.yaw_rps),
                self.config.max_axis_command,
            ),
            clamp_unit(throttle),
        )
    }
}

impl Default for RateController {
    fn default() -> Self {
        Self::new(RateControllerConfig::default())
    }
}

fn clamp_symmetric(value: f32, limit: f32) -> f32 {
    if !value.is_finite() || !limit.is_finite() || limit <= 0.0 {
        return 0.0;
    }

    value.clamp(-limit, limit)
}

fn clamp_unit(value: f32) -> f32 {
    if !value.is_finite() {
        return 0.0;
    }

    value.clamp(0.0, 1.0)
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
}
