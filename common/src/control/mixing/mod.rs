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
    pub limit_flags: MixerLimitFlags,
}

impl<const N: usize> MotorOutputs<N> {
    pub const fn new(commands: [f32; N], clamped: bool) -> Self {
        Self {
            commands,
            clamped,
            limit_flags: MixerLimitFlags::NONE,
        }
    }

    pub const fn with_limit_flags(commands: [f32; N], limit_flags: MixerLimitFlags) -> Self {
        Self {
            commands,
            clamped: limit_flags.any(),
            limit_flags,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct MixerLimitFlags {
    pub roll_pitch: bool,
    pub yaw: bool,
    pub throttle_lower: bool,
    pub throttle_upper: bool,
}

impl MixerLimitFlags {
    pub const NONE: Self = Self {
        roll_pitch: false,
        yaw: false,
        throttle_lower: false,
        throttle_upper: false,
    };

    pub const fn any(self) -> bool {
        self.roll_pitch || self.yaw || self.throttle_lower || self.throttle_upper
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

        clamp_outputs(self.motor_order.apply(raw), self.limits, command)
    }
}

impl Default for QuadXMixer {
    fn default() -> Self {
        Self::new(MixerLimits::NORMALIZED)
    }
}

fn clamp_outputs<const N: usize>(
    mut commands: [f32; N],
    limits: MixerLimits,
    command: TorqueCommand,
) -> MotorOutputs<N> {
    let mut limit_flags = MixerLimitFlags::NONE;

    for command in commands.iter_mut() {
        if *command < limits.min {
            *command = limits.min;
            limit_flags.throttle_lower = true;
        } else if *command > limits.max {
            *command = limits.max;
            limit_flags.throttle_upper = true;
        }
    }

    if limit_flags.any() {
        limit_flags.roll_pitch = command.roll != 0.0 || command.pitch != 0.0;
        limit_flags.yaw = command.yaw != 0.0;
    }

    MotorOutputs::with_limit_flags(commands, limit_flags)
}

#[cfg(test)]
mod tests {
    use super::{MixerLimits, MotorOrder, QuadXMixer, TorqueCommand};
    use crate::test_helpers::assert_scalar_near;

    fn assert_commands_near(actual: [f32; 4], expected: [f32; 4]) {
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert_scalar_near(actual, expected, 0.000_001);
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
    fn quad_x_mixer_roll_pitch_yaw_basis_vectors_match_documented_signs() {
        let mixer = QuadXMixer::default();

        let roll = mixer.mix(TorqueCommand::new(0.1, 0.0, 0.0, 0.5));
        let pitch = mixer.mix(TorqueCommand::new(0.0, 0.1, 0.0, 0.5));
        let yaw = mixer.mix(TorqueCommand::new(0.0, 0.0, 0.1, 0.5));

        assert_commands_near(roll.commands, [0.6, 0.4, 0.4, 0.6]);
        assert_commands_near(pitch.commands, [0.6, 0.6, 0.4, 0.4]);
        assert_commands_near(yaw.commands, [0.4, 0.6, 0.4, 0.6]);
        assert!(!roll.clamped);
        assert!(!pitch.clamped);
        assert!(!yaw.clamped);
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
    fn motor_order_round_trips_every_valid_permutation() {
        for a in 1..=4 {
            for b in 1..=4 {
                for c in 1..=4 {
                    for d in 1..=4 {
                        let order = [a, b, c, d];
                        if !is_valid_one_based_permutation(order) {
                            continue;
                        }

                        let motor_order = MotorOrder::from_one_based(order)
                            .expect("valid permutation should be accepted");

                        assert_eq!(motor_order.one_based(), order);
                    }
                }
            }
        }
    }

    #[test]
    fn motor_order_rejects_every_invalid_zero_to_five_permutation() {
        let mut valid_count = 0;
        let mut invalid_count = 0;

        for a in 0..=5 {
            for b in 0..=5 {
                for c in 0..=5 {
                    for d in 0..=5 {
                        let order = [a, b, c, d];
                        let result = MotorOrder::from_one_based(order);

                        if is_valid_one_based_permutation(order) {
                            valid_count += 1;
                            assert!(result.is_some(), "valid order rejected: {order:?}");
                        } else {
                            invalid_count += 1;
                            assert!(result.is_none(), "invalid order accepted: {order:?}");
                        }
                    }
                }
            }
        }

        assert_eq!(valid_count, 24);
        assert_eq!(invalid_count, 1_272);
    }

    #[test]
    fn motor_order_remap_can_be_recovered_with_inverse_order() {
        let logical = [0.75, 0.65, 0.15, 0.45];

        for order in valid_motor_orders() {
            let motor_order = MotorOrder::from_one_based(order).unwrap();
            let inverse = MotorOrder::from_one_based(inverse_order(order)).unwrap();

            assert_commands_near(inverse.apply(motor_order.apply(logical)), logical);
        }
    }

    #[test]
    fn quad_x_mixer_clamps_normalized_outputs() {
        let mixer = QuadXMixer::default();
        let outputs = mixer.mix(TorqueCommand::new(0.4, 0.4, 0.0, 0.5));

        assert_commands_near(outputs.commands, [1.0, 0.5, 0.0, 0.5]);
        assert!(outputs.clamped);
        assert!(outputs.limit_flags.roll_pitch);
        assert!(!outputs.limit_flags.yaw);
        assert!(outputs.limit_flags.throttle_lower);
        assert!(outputs.limit_flags.throttle_upper);
    }

    #[test]
    fn quad_x_mixer_respects_custom_output_limits() {
        let mixer = QuadXMixer::new(MixerLimits::new(0.2, 0.8));
        let outputs = mixer.mix(TorqueCommand::new(0.5, 0.0, 0.0, 0.5));

        assert_commands_near(outputs.commands, [0.8, 0.2, 0.2, 0.8]);
        assert!(outputs.clamped);
        assert!(outputs.limit_flags.roll_pitch);
    }

    #[test]
    fn quad_x_mixer_reports_yaw_saturation_separately() {
        let mixer = QuadXMixer::new(MixerLimits::new(0.45, 0.55));
        let outputs = mixer.mix(TorqueCommand::new(0.0, 0.0, 0.2, 0.5));

        assert!(outputs.clamped);
        assert!(outputs.limit_flags.yaw);
        assert!(!outputs.limit_flags.roll_pitch);
        assert!(outputs.limit_flags.throttle_lower);
        assert!(outputs.limit_flags.throttle_upper);
    }

    fn is_valid_one_based_permutation(order: [u8; 4]) -> bool {
        let mut seen = [false; 4];

        for output in order {
            if !(1..=4).contains(&output) {
                return false;
            }

            let index = usize::from(output - 1);
            if seen[index] {
                return false;
            }
            seen[index] = true;
        }

        true
    }

    fn valid_motor_orders() -> [[u8; 4]; 24] {
        [
            [1, 2, 3, 4],
            [1, 2, 4, 3],
            [1, 3, 2, 4],
            [1, 3, 4, 2],
            [1, 4, 2, 3],
            [1, 4, 3, 2],
            [2, 1, 3, 4],
            [2, 1, 4, 3],
            [2, 3, 1, 4],
            [2, 3, 4, 1],
            [2, 4, 1, 3],
            [2, 4, 3, 1],
            [3, 1, 2, 4],
            [3, 1, 4, 2],
            [3, 2, 1, 4],
            [3, 2, 4, 1],
            [3, 4, 1, 2],
            [3, 4, 2, 1],
            [4, 1, 2, 3],
            [4, 1, 3, 2],
            [4, 2, 1, 3],
            [4, 2, 3, 1],
            [4, 3, 1, 2],
            [4, 3, 2, 1],
        ]
    }

    fn inverse_order(order: [u8; 4]) -> [u8; 4] {
        let mut inverse = [0; 4];

        for (motor, output) in order.into_iter().enumerate() {
            inverse[usize::from(output - 1)] = (motor + 1) as u8;
        }

        inverse
    }
}
