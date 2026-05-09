//! Control-domain messages belong here.

use crate::utilities::time::MeasurementTimestamp;

pub const ACTUATOR_MOTOR_COUNT: usize = 4;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RgbLedCommand {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl RgbLedCommand {
    pub const OFF: Self = Self::new(0, 0, 0);

    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self { red, green, blue }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StaticLedCommand {
    pub on: bool,
}

impl StaticLedCommand {
    pub const OFF: Self = Self { on: false };
    pub const ON: Self = Self { on: true };
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ActuatorOutputSample {
    pub motor_command_normalized: [f32; ACTUATOR_MOTOR_COUNT],
    pub valid: bool,
    pub clamped: bool,
}

impl ActuatorOutputSample {
    pub const fn new(
        motor_command_normalized: [f32; ACTUATOR_MOTOR_COUNT],
        valid: bool,
        clamped: bool,
    ) -> Self {
        Self {
            motor_command_normalized,
            valid,
            clamped,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ActuatorOutputStamped {
    pub timestamp: MeasurementTimestamp,
    pub sample: ActuatorOutputSample,
}
