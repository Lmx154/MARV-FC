//! Portable helpers for hobby RC servos driven from PWM.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RcServoConfig {
    pub min_pulse_us: u16,
    pub max_pulse_us: u16,
    pub max_angle_deg: u16,
    pub frame_period_us: u32,
}

impl RcServoConfig {
    pub const fn ds3240_180() -> Self {
        Self {
            min_pulse_us: 500,
            max_pulse_us: 2_500,
            max_angle_deg: 180,
            frame_period_us: 20_000,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum ServoError {
    InvalidPulseRange,
    InvalidAngleRange,
    InvalidFramePeriod,
    AngleOutOfRange,
    InvalidPwmTop,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RcServo {
    config: RcServoConfig,
}

impl RcServo {
    pub const fn ds3240_180() -> Self {
        Self {
            config: RcServoConfig::ds3240_180(),
        }
    }

    pub fn try_new(config: RcServoConfig) -> Result<Self, ServoError> {
        if config.min_pulse_us >= config.max_pulse_us {
            return Err(ServoError::InvalidPulseRange);
        }
        if config.max_angle_deg == 0 {
            return Err(ServoError::InvalidAngleRange);
        }
        if config.frame_period_us <= config.max_pulse_us as u32 {
            return Err(ServoError::InvalidFramePeriod);
        }

        Ok(Self { config })
    }

    pub fn config(&self) -> RcServoConfig {
        self.config
    }

    pub fn pulse_for_angle_deg(&self, angle_deg: u16) -> Result<u16, ServoError> {
        if angle_deg > self.config.max_angle_deg {
            return Err(ServoError::AngleOutOfRange);
        }

        let pulse_span_us = (self.config.max_pulse_us - self.config.min_pulse_us) as u32;
        let pulse_us = self.config.min_pulse_us as u32
            + ((pulse_span_us * angle_deg as u32) + (self.config.max_angle_deg as u32 / 2))
                / self.config.max_angle_deg as u32;

        Ok(pulse_us as u16)
    }

    pub fn duty_for_angle_deg(&self, angle_deg: u16, pwm_top: u16) -> Result<u16, ServoError> {
        if pwm_top == 0 {
            return Err(ServoError::InvalidPwmTop);
        }

        let pulse_us = self.pulse_for_angle_deg(angle_deg)? as u32;
        let duty = ((pulse_us * pwm_top as u32) + (self.config.frame_period_us / 2))
            / self.config.frame_period_us;

        Ok(duty.min(pwm_top as u32) as u16)
    }
}

#[cfg(test)]
mod tests {
    use super::{RcServo, RcServoConfig, ServoError};

    #[test]
    fn ds3240_maps_angles_to_documented_pulses() {
        let servo = RcServo::ds3240_180();

        assert_eq!(servo.pulse_for_angle_deg(0), Ok(500));
        assert_eq!(servo.pulse_for_angle_deg(90), Ok(1_500));
        assert_eq!(servo.pulse_for_angle_deg(180), Ok(2_500));
    }

    #[test]
    fn duty_mapping_scales_against_pwm_top() {
        let servo = RcServo::ds3240_180();

        assert_eq!(servo.duty_for_angle_deg(0, 20_000), Ok(500));
        assert_eq!(servo.duty_for_angle_deg(90, 20_000), Ok(1_500));
        assert_eq!(servo.duty_for_angle_deg(180, 20_000), Ok(2_500));
    }

    #[test]
    fn rejects_out_of_range_angles() {
        let servo = RcServo::ds3240_180();

        assert_eq!(
            servo.pulse_for_angle_deg(181),
            Err(ServoError::AngleOutOfRange)
        );
    }

    #[test]
    fn rejects_invalid_configs() {
        assert_eq!(
            RcServo::try_new(RcServoConfig {
                min_pulse_us: 2_500,
                max_pulse_us: 500,
                max_angle_deg: 180,
                frame_period_us: 20_000,
            }),
            Err(ServoError::InvalidPulseRange)
        );
        assert_eq!(
            RcServo::try_new(RcServoConfig {
                min_pulse_us: 500,
                max_pulse_us: 2_500,
                max_angle_deg: 0,
                frame_period_us: 20_000,
            }),
            Err(ServoError::InvalidAngleRange)
        );
        assert_eq!(
            RcServo::try_new(RcServoConfig {
                min_pulse_us: 500,
                max_pulse_us: 2_500,
                max_angle_deg: 180,
                frame_period_us: 2_500,
            }),
            Err(ServoError::InvalidFramePeriod)
        );
    }
}
