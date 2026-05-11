//! Hardware-independent attitude control.

use crate::localization::estimation::core::Quaternion;
use crate::localization::estimation::math::{
    identity_quaternion, normalize_quaternion, quaternion_inverse, quaternion_multiply,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyRateSetpoint {
    pub roll_rps: f32,
    pub pitch_rps: f32,
    pub yaw_rps: f32,
}

impl BodyRateSetpoint {
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);

    pub const fn new(roll_rps: f32, pitch_rps: f32, yaw_rps: f32) -> Self {
        Self {
            roll_rps,
            pitch_rps,
            yaw_rps,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeSetpoint {
    pub quaternion: [f32; 4],
}

impl AttitudeSetpoint {
    pub const LEVEL: Self = Self {
        quaternion: [1.0, 0.0, 0.0, 0.0],
    };

    pub const fn new(quaternion: [f32; 4]) -> Self {
        Self { quaternion }
    }

    pub fn from_euler_rad(roll_rad: f32, pitch_rad: f32, yaw_rad: f32) -> Option<Self> {
        if !roll_rad.is_finite() || !pitch_rad.is_finite() || !yaw_rad.is_finite() {
            return None;
        }

        let half_roll = 0.5 * roll_rad;
        let half_pitch = 0.5 * pitch_rad;
        let half_yaw = 0.5 * yaw_rad;
        let cr = micromath::F32Ext::cos(half_roll);
        let sr = micromath::F32Ext::sin(half_roll);
        let cp = micromath::F32Ext::cos(half_pitch);
        let sp = micromath::F32Ext::sin(half_pitch);
        let cy = micromath::F32Ext::cos(half_yaw);
        let sy = micromath::F32Ext::sin(half_yaw);

        Some(Self::new([
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeControllerConfig {
    pub roll_rate_gain: f32,
    pub pitch_rate_gain: f32,
    pub yaw_rate_gain: f32,
    pub max_rate_rps: f32,
    pub max_yaw_rate_rps: f32,
}

impl Default for AttitudeControllerConfig {
    fn default() -> Self {
        Self {
            roll_rate_gain: 4.0,
            pitch_rate_gain: 4.0,
            yaw_rate_gain: 2.0,
            max_rate_rps: 2.0,
            max_yaw_rate_rps: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeController {
    config: AttitudeControllerConfig,
}

impl AttitudeController {
    pub const fn new(config: AttitudeControllerConfig) -> Self {
        Self { config }
    }

    pub fn update(
        &self,
        setpoint: AttitudeSetpoint,
        attitude_quat: [f32; 4],
    ) -> Option<BodyRateSetpoint> {
        let current = finite_quaternion(attitude_quat)?;
        let target = finite_quaternion(setpoint.quaternion)?;
        let mut error = quaternion_multiply(&target, &quaternion_inverse(&current));

        if error[0] < 0.0 {
            error = -error;
        }

        Some(BodyRateSetpoint::new(
            clamp_symmetric(
                2.0 * self.config.roll_rate_gain * error[1],
                self.config.max_rate_rps,
            ),
            clamp_symmetric(
                2.0 * self.config.pitch_rate_gain * error[2],
                self.config.max_rate_rps,
            ),
            clamp_symmetric(
                2.0 * self.config.yaw_rate_gain * error[3],
                self.config.max_yaw_rate_rps,
            ),
        ))
    }
}

impl Default for AttitudeController {
    fn default() -> Self {
        Self::new(AttitudeControllerConfig::default())
    }
}

fn finite_quaternion(values: [f32; 4]) -> Option<Quaternion<f32>> {
    if !values.iter().all(|value| value.is_finite()) {
        return None;
    }

    let quaternion = Quaternion::<f32>::new(values[0], values[1], values[2], values[3]);
    let normalized = normalize_quaternion(&quaternion);
    if normalized == identity_quaternion() && quaternion.norm() <= f32::EPSILON {
        None
    } else {
        Some(normalized)
    }
}

fn clamp_symmetric(value: f32, limit: f32) -> f32 {
    if !value.is_finite() || !limit.is_finite() || limit <= 0.0 {
        return 0.0;
    }

    value.clamp(-limit, limit)
}

#[cfg(test)]
mod tests {
    use super::{AttitudeController, AttitudeSetpoint};
    use crate::test_helpers::{assert_quaternion_near, assert_scalar_near};

    #[test]
    fn level_attitude_maps_to_zero_rate() {
        let controller = AttitudeController::default();

        let rate = controller
            .update(AttitudeSetpoint::LEVEL, [1.0, 0.0, 0.0, 0.0])
            .expect("valid attitude should produce a setpoint");

        assert_scalar_near(rate.roll_rps, 0.0, 0.000_001);
        assert_scalar_near(rate.pitch_rps, 0.0, 0.000_001);
        assert_scalar_near(rate.yaw_rps, 0.0, 0.000_001);
    }

    #[test]
    fn roll_error_commands_opposing_roll_rate() {
        let controller = AttitudeController::default();

        let rate = controller
            .update(
                AttitudeSetpoint::LEVEL,
                [0.996_194_7, 0.087_155_7, 0.0, 0.0],
            )
            .expect("valid attitude should produce a setpoint");

        assert!(rate.roll_rps < 0.0);
        assert_scalar_near(rate.pitch_rps, 0.0, 0.000_001);
        assert_scalar_near(rate.yaw_rps, 0.0, 0.000_001);
    }

    #[test]
    fn pitch_error_commands_opposing_pitch_rate() {
        let controller = AttitudeController::default();

        let rate = controller
            .update(
                AttitudeSetpoint::LEVEL,
                [0.996_194_7, 0.0, 0.087_155_7, 0.0],
            )
            .expect("valid attitude should produce a setpoint");

        assert_scalar_near(rate.roll_rps, 0.0, 0.000_001);
        assert!(rate.pitch_rps < 0.0);
        assert_scalar_near(rate.yaw_rps, 0.0, 0.000_001);
    }

    #[test]
    fn invalid_quaternion_is_rejected() {
        let controller = AttitudeController::default();

        assert!(
            controller
                .update(AttitudeSetpoint::LEVEL, [0.0, 0.0, 0.0, 0.0])
                .is_none()
        );
    }

    #[test]
    fn yaw_euler_setpoint_matches_yaw_only_quaternion() {
        let setpoint = AttitudeSetpoint::from_euler_rad(0.0, 0.0, core::f32::consts::FRAC_PI_2)
            .expect("finite euler angles should produce a setpoint");

        assert_quaternion_near(
            setpoint.quaternion,
            [0.707_106_77, 0.0, 0.0, 0.707_106_77],
            0.001,
        );
    }

    #[test]
    fn non_finite_euler_setpoint_is_rejected() {
        assert!(AttitudeSetpoint::from_euler_rad(f32::NAN, 0.0, 0.0).is_none());
    }

    #[test]
    fn yaw_rate_uses_independent_limit() {
        let controller = AttitudeController::new(super::AttitudeControllerConfig {
            roll_rate_gain: 4.0,
            pitch_rate_gain: 4.0,
            yaw_rate_gain: 10.0,
            max_rate_rps: 2.0,
            max_yaw_rate_rps: 0.25,
        });

        let rate = controller
            .update(
                AttitudeSetpoint::from_euler_rad(0.0, 0.0, core::f32::consts::PI)
                    .expect("finite euler angles should produce a setpoint"),
                [1.0, 0.0, 0.0, 0.0],
            )
            .expect("valid attitude should produce a setpoint");

        assert_eq!(rate.yaw_rps, 0.25);
    }

    #[test]
    fn negated_equivalent_quaternion_does_not_create_large_error() {
        let controller = AttitudeController::default();

        let rate = controller
            .update(AttitudeSetpoint::LEVEL, [-1.0, 0.0, 0.0, 0.0])
            .expect("negated unit quaternion is valid");

        assert_scalar_near(rate.roll_rps, 0.0, 0.000_001);
        assert_scalar_near(rate.pitch_rps, 0.0, 0.000_001);
        assert_scalar_near(rate.yaw_rps, 0.0, 0.000_001);
    }

    #[test]
    fn attitude_error_uses_target_times_current_inverse_order() {
        let controller = AttitudeController::new(super::AttitudeControllerConfig {
            roll_rate_gain: 1.0,
            pitch_rate_gain: 1.0,
            yaw_rate_gain: 1.0,
            max_rate_rps: 10.0,
            max_yaw_rate_rps: 10.0,
        });
        let yaw_30 = AttitudeSetpoint::from_euler_rad(0.0, 0.0, core::f32::consts::PI / 6.0)
            .expect("finite yaw setpoint");

        let from_level = controller
            .update(yaw_30, AttitudeSetpoint::LEVEL.quaternion)
            .expect("valid attitude should produce a setpoint");
        let back_to_level = controller
            .update(AttitudeSetpoint::LEVEL, yaw_30.quaternion)
            .expect("valid attitude should produce a setpoint");

        assert_scalar_near(from_level.roll_rps, 0.0, 0.000_001);
        assert_scalar_near(from_level.pitch_rps, 0.0, 0.000_001);
        assert!(from_level.yaw_rps > 0.0);
        assert_scalar_near(back_to_level.roll_rps, 0.0, 0.000_001);
        assert_scalar_near(back_to_level.pitch_rps, 0.0, 0.000_001);
        assert!(back_to_level.yaw_rps < 0.0);
        assert_scalar_near(from_level.yaw_rps, -back_to_level.yaw_rps, 0.000_001);
    }
}
