//! Hardware-independent actuator mixing.

pub const QUAD_MOTOR_COUNT: usize = 4;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TorqueCommand {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub throttle: f32,
}

impl TorqueCommand {
    pub const fn new(roll: f32, pitch: f32, yaw: f32, throttle: f32) -> Self {
        Self {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixerLimits {
    pub min: f32,
    pub max: f32,
}

impl MixerLimits {
    pub const NORMALIZED: Self = Self { min: 0.0, max: 1.0 };

    pub const fn new(min: f32, max: f32) -> Self {
        Self { min, max }
    }
}

impl Default for MixerLimits {
    fn default() -> Self {
        Self::NORMALIZED
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputs<const N: usize> {
    pub commands: [f32; N],
    pub clamped: bool,
}

impl<const N: usize> MotorOutputs<N> {
    pub const fn new(commands: [f32; N], clamped: bool) -> Self {
        Self { commands, clamped }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QuadXMixer {
    limits: MixerLimits,
}

impl QuadXMixer {
    pub const fn new(limits: MixerLimits) -> Self {
        Self { limits }
    }

    pub fn mix(&self, command: TorqueCommand) -> MotorOutputs<QUAD_MOTOR_COUNT> {
        let raw = [
            command.throttle + command.roll + command.pitch - command.yaw,
            command.throttle - command.roll + command.pitch + command.yaw,
            command.throttle - command.roll - command.pitch - command.yaw,
            command.throttle + command.roll - command.pitch + command.yaw,
        ];

        clamp_outputs(raw, self.limits)
    }
}

impl Default for QuadXMixer {
    fn default() -> Self {
        Self::new(MixerLimits::NORMALIZED)
    }
}

fn clamp_outputs<const N: usize>(mut commands: [f32; N], limits: MixerLimits) -> MotorOutputs<N> {
    let mut clamped = false;

    for command in commands.iter_mut() {
        if *command < limits.min {
            *command = limits.min;
            clamped = true;
        } else if *command > limits.max {
            *command = limits.max;
            clamped = true;
        }
    }

    MotorOutputs::new(commands, clamped)
}

#[cfg(test)]
mod tests {
    use super::{QuadXMixer, TorqueCommand};

    fn assert_commands_near(actual: [f32; 4], expected: [f32; 4]) {
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert!((actual - expected).abs() < 0.000_001);
        }
    }

    #[test]
    fn quad_x_mixer_passes_throttle_to_all_motors() {
        let mixer = QuadXMixer::default();
        let outputs = mixer.mix(TorqueCommand::new(0.0, 0.0, 0.0, 0.25));

        assert_commands_near(outputs.commands, [0.25, 0.25, 0.25, 0.25]);
        assert!(!outputs.clamped);
    }

    #[test]
    fn quad_x_mixer_uses_documented_motor_equations() {
        let mixer = QuadXMixer::default();
        let outputs = mixer.mix(TorqueCommand::new(0.1, 0.2, 0.05, 0.5));

        assert_commands_near(outputs.commands, [0.75, 0.65, 0.15, 0.45]);
        assert!(!outputs.clamped);
    }

    #[test]
    fn quad_x_mixer_clamps_normalized_outputs() {
        let mixer = QuadXMixer::default();
        let outputs = mixer.mix(TorqueCommand::new(0.4, 0.4, 0.0, 0.5));

        assert_commands_near(outputs.commands, [1.0, 0.5, 0.0, 0.5]);
        assert!(outputs.clamped);
    }
}
