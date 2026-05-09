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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MotorOrder {
    output_for_motor: [usize; QUAD_MOTOR_COUNT],
}

impl MotorOrder {
    pub const IDENTITY: Self = Self {
        output_for_motor: [0, 1, 2, 3],
    };

    pub fn from_one_based(output_for_motor: [u8; QUAD_MOTOR_COUNT]) -> Option<Self> {
        let mut zero_based = [0usize; QUAD_MOTOR_COUNT];
        let mut seen = [false; QUAD_MOTOR_COUNT];

        for (index, output) in output_for_motor.into_iter().enumerate() {
            if !(1..=QUAD_MOTOR_COUNT as u8).contains(&output) {
                return None;
            }

            let output = usize::from(output - 1);
            if seen[output] {
                return None;
            }

            zero_based[index] = output;
            seen[output] = true;
        }

        Some(Self {
            output_for_motor: zero_based,
        })
    }

    pub fn one_based(self) -> [u8; QUAD_MOTOR_COUNT] {
        [
            (self.output_for_motor[0] + 1) as u8,
            (self.output_for_motor[1] + 1) as u8,
            (self.output_for_motor[2] + 1) as u8,
            (self.output_for_motor[3] + 1) as u8,
        ]
    }

    fn apply(self, logical_commands: [f32; QUAD_MOTOR_COUNT]) -> [f32; QUAD_MOTOR_COUNT] {
        let mut output_commands = [0.0; QUAD_MOTOR_COUNT];

        for (motor_index, command) in logical_commands.into_iter().enumerate() {
            output_commands[self.output_for_motor[motor_index]] = command;
        }

        output_commands
    }
}

impl Default for MotorOrder {
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QuadXMixer {
    limits: MixerLimits,
    motor_order: MotorOrder,
}

impl QuadXMixer {
    pub const fn new(limits: MixerLimits) -> Self {
        Self {
            limits,
            motor_order: MotorOrder::IDENTITY,
        }
    }

    pub const fn with_motor_order(limits: MixerLimits, motor_order: MotorOrder) -> Self {
        Self {
            limits,
            motor_order,
        }
    }

    pub fn set_motor_order(&mut self, motor_order: MotorOrder) {
        self.motor_order = motor_order;
    }

    pub const fn motor_order(&self) -> MotorOrder {
        self.motor_order
    }

    pub fn mix(&self, command: TorqueCommand) -> MotorOutputs<QUAD_MOTOR_COUNT> {
        let raw = [
            command.throttle + command.roll + command.pitch - command.yaw,
            command.throttle - command.roll + command.pitch + command.yaw,
            command.throttle - command.roll - command.pitch - command.yaw,
            command.throttle + command.roll - command.pitch + command.yaw,
        ];

        clamp_outputs(self.motor_order.apply(raw), self.limits)
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
    use super::{MotorOrder, QuadXMixer, TorqueCommand};

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
    fn quad_x_mixer_can_remap_logical_motors_to_outputs() {
        let order = MotorOrder::from_one_based([3, 1, 4, 2]).unwrap();
        let mixer = QuadXMixer::with_motor_order(Default::default(), order);
        let outputs = mixer.mix(TorqueCommand::new(0.1, 0.2, 0.05, 0.5));

        assert_commands_near(outputs.commands, [0.65, 0.45, 0.75, 0.15]);
        assert_eq!(mixer.motor_order().one_based(), [3, 1, 4, 2]);
    }

    #[test]
    fn motor_order_rejects_invalid_permutations() {
        assert!(MotorOrder::from_one_based([1, 2, 3, 4]).is_some());
        assert!(MotorOrder::from_one_based([1, 2, 2, 4]).is_none());
        assert!(MotorOrder::from_one_based([0, 2, 3, 4]).is_none());
        assert!(MotorOrder::from_one_based([1, 2, 3, 5]).is_none());
    }

    #[test]
    fn quad_x_mixer_clamps_normalized_outputs() {
        let mixer = QuadXMixer::default();
        let outputs = mixer.mix(TorqueCommand::new(0.4, 0.4, 0.0, 0.5));

        assert_commands_near(outputs.commands, [1.0, 0.5, 0.0, 0.5]);
        assert!(outputs.clamped);
    }
}
