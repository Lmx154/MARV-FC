//! Small PID primitives for cascaded flight-control loops.

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidGains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

impl PidGains {
    pub const fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self { kp, ki, kd }
    }
}

impl Default for PidGains {
    fn default() -> Self {
        Self {
            kp: 0.0,
            ki: 0.0,
            kd: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidLimits {
    pub integrator_min: f32,
    pub integrator_max: f32,
    pub output_min: f32,
    pub output_max: f32,
}

impl PidLimits {
    pub const fn symmetric(integrator: f32, output: f32) -> Self {
        Self {
            integrator_min: -integrator,
            integrator_max: integrator,
            output_min: -output,
            output_max: output,
        }
    }
}

impl Default for PidLimits {
    fn default() -> Self {
        Self::symmetric(f32::INFINITY, f32::INFINITY)
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidController {
    pub gains: PidGains,
    pub limits: PidLimits,
    integrator: f32,
    previous_error: Option<f32>,
}

impl PidController {
    pub const fn new(gains: PidGains, limits: PidLimits) -> Self {
        Self {
            gains,
            limits,
            integrator: 0.0,
            previous_error: None,
        }
    }

    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.previous_error = None;
    }

    pub fn integrator(&self) -> f32 {
        self.integrator
    }

    pub fn update(&mut self, error: f32, dt_s: f32) -> f32 {
        let dt_s = dt_s.max(0.0);
        if dt_s > 0.0 {
            self.integrator = clamp(
                self.integrator + error * dt_s,
                self.limits.integrator_min,
                self.limits.integrator_max,
            );
        }

        let derivative = match (self.previous_error, dt_s > 0.0) {
            (Some(previous), true) => (error - previous) / dt_s,
            _ => 0.0,
        };
        self.previous_error = Some(error);

        clamp(
            self.gains.kp * error + self.gains.ki * self.integrator + self.gains.kd * derivative,
            self.limits.output_min,
            self.limits.output_max,
        )
    }
}

pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
    value.max(min).min(max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_integrator_and_output() {
        let mut pid =
            PidController::new(PidGains::new(2.0, 1.0, 0.0), PidLimits::symmetric(0.5, 1.0));

        assert_eq!(pid.update(10.0, 1.0), 1.0);
        assert_eq!(pid.integrator(), 0.5);
    }

    #[test]
    fn reset_clears_history() {
        let mut pid = PidController::new(
            PidGains::new(0.0, 0.0, 1.0),
            PidLimits::symmetric(1.0, 100.0),
        );

        assert_eq!(pid.update(1.0, 0.1), 0.0);
        assert_eq!(pid.update(2.0, 0.1), 10.0);
        pid.reset();
        assert_eq!(pid.update(2.0, 0.1), 0.0);
    }
}
