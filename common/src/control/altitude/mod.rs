//! Conservative altitude-hold throttle control.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AltitudeControllerConfig {
    pub hover_throttle: f32,
    pub altitude_gain: f32,
    pub vertical_velocity_gain: f32,
    pub max_altitude_error_m: f32,
    pub max_vertical_velocity_mps: f32,
    pub max_throttle_correction: f32,
}

impl Default for AltitudeControllerConfig {
    fn default() -> Self {
        Self {
            hover_throttle: 0.5,
            altitude_gain: 0.08,
            vertical_velocity_gain: 0.04,
            max_altitude_error_m: 3.0,
            max_vertical_velocity_mps: 2.0,
            max_throttle_correction: 0.2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AltitudeController {
    config: AltitudeControllerConfig,
}

impl AltitudeController {
    pub const fn new(config: AltitudeControllerConfig) -> Self {
        Self { config }
    }

    pub fn update(
        &self,
        setpoint_down_m: f32,
        position_down_m: f32,
        velocity_down_mps: f32,
    ) -> Option<f32> {
        if !setpoint_down_m.is_finite()
            || !position_down_m.is_finite()
            || !velocity_down_mps.is_finite()
        {
            return None;
        }

        let down_error_m = clamp_symmetric(
            position_down_m - setpoint_down_m,
            self.config.max_altitude_error_m,
        );
        let down_velocity_mps =
            clamp_symmetric(velocity_down_mps, self.config.max_vertical_velocity_mps);
        let correction = clamp_symmetric(
            self.config.altitude_gain * down_error_m
                + self.config.vertical_velocity_gain * down_velocity_mps,
            self.config.max_throttle_correction,
        );

        Some(clamp_unit(self.config.hover_throttle + correction))
    }
}

impl Default for AltitudeController {
    fn default() -> Self {
        Self::new(AltitudeControllerConfig::default())
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
    use super::{AltitudeController, AltitudeControllerConfig};

    #[test]
    fn zero_error_holds_hover_throttle() {
        let controller = AltitudeController::default();

        let throttle = controller
            .update(0.0, 0.0, 0.0)
            .expect("finite altitude input should produce throttle");

        assert_eq!(throttle, 0.5);
    }

    #[test]
    fn below_setpoint_commands_more_throttle() {
        let controller = AltitudeController::default();

        let throttle = controller
            .update(-2.0, 0.0, 0.0)
            .expect("finite altitude input should produce throttle");

        assert!(throttle > 0.5);
    }

    #[test]
    fn vertical_descent_commands_more_throttle() {
        let controller = AltitudeController::default();

        let throttle = controller
            .update(0.0, 0.0, 1.0)
            .expect("finite altitude input should produce throttle");

        assert!(throttle > 0.5);
    }

    #[test]
    fn correction_is_limited() {
        let controller = AltitudeController::new(AltitudeControllerConfig {
            hover_throttle: 0.5,
            altitude_gain: 10.0,
            vertical_velocity_gain: 10.0,
            max_altitude_error_m: 100.0,
            max_vertical_velocity_mps: 100.0,
            max_throttle_correction: 0.1,
        });

        let throttle = controller
            .update(-100.0, 100.0, 100.0)
            .expect("finite altitude input should produce throttle");

        assert_eq!(throttle, 0.6);
    }

    #[test]
    fn non_finite_input_is_rejected() {
        let controller = AltitudeController::default();

        assert!(controller.update(f32::NAN, 0.0, 0.0).is_none());
    }
}
