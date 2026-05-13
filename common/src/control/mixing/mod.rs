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
    pub desaturation: DesaturationReport,
}

impl<const N: usize> MotorOutputs<N> {
    pub const fn new(commands: [f32; N], clamped: bool) -> Self {
        Self {
            commands,
            clamped,
            limit_flags: MixerLimitFlags::NONE,
            desaturation: DesaturationReport::NONE,
        }
    }

    pub const fn with_limit_flags(commands: [f32; N], limit_flags: MixerLimitFlags) -> Self {
        Self::with_desaturation(
            commands,
            limit_flags,
            DesaturationReport::from_limits(limit_flags),
        )
    }

    pub const fn with_desaturation(
        commands: [f32; N],
        limit_flags: MixerLimitFlags,
        desaturation: DesaturationReport,
    ) -> Self {
        Self {
            commands,
            clamped: limit_flags.any(),
            limit_flags,
            desaturation,
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

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum DesaturationPriority {
    #[default]
    None,
    PreserveThrottle,
    PreserveRollPitchOverYaw,
    PreserveYaw,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DesaturationReport {
    pub priority: DesaturationPriority,
    pub reduced_roll_pitch: bool,
    pub reduced_yaw: bool,
    pub reduced_throttle: bool,
}

impl DesaturationReport {
    pub const NONE: Self = Self {
        priority: DesaturationPriority::None,
        reduced_roll_pitch: false,
        reduced_yaw: false,
        reduced_throttle: false,
    };

    pub const fn from_limits(limit_flags: MixerLimitFlags) -> Self {
        if !limit_flags.any() {
            return Self::NONE;
        }

        let priority = if limit_flags.roll_pitch && limit_flags.yaw {
            DesaturationPriority::PreserveRollPitchOverYaw
        } else if limit_flags.roll_pitch {
            DesaturationPriority::PreserveThrottle
        } else if limit_flags.yaw {
            DesaturationPriority::PreserveYaw
        } else {
            DesaturationPriority::PreserveThrottle
        };

        Self {
            priority,
            reduced_roll_pitch: limit_flags.roll_pitch && !limit_flags.yaw,
            reduced_yaw: limit_flags.yaw && limit_flags.roll_pitch,
            reduced_throttle: limit_flags.throttle_lower || limit_flags.throttle_upper,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpinDirection {
    Clockwise,
    CounterClockwise,
}

impl SpinDirection {
    pub const fn yaw_reaction_sign(self) -> f32 {
        match self {
            Self::Clockwise => -1.0,
            Self::CounterClockwise => 1.0,
        }
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

    pub fn output_index_for_motor(self, logical_motor: usize) -> Option<usize> {
        if !(1..=QUAD_MOTOR_COUNT).contains(&logical_motor) {
            return None;
        }

        Some(self.output_for_motor[logical_motor - 1])
    }

    fn apply(self, logical_commands: [f32; QUAD_MOTOR_COUNT]) -> [f32; QUAD_MOTOR_COUNT] {
        let mut output_commands = [0.0; QUAD_MOTOR_COUNT];

        for (motor_index, command) in logical_commands.into_iter().enumerate() {
            output_commands[self.output_for_motor[motor_index]] = command;
        }

        output_commands
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorSpec {
    pub logical_motor: usize,
    pub output_index: usize,
    pub position_body_m: [f32; 3],
    pub spin_direction: SpinDirection,
    pub thrust_coefficient_n: f32,
    pub reaction_torque_coefficient_nm: f32,
    pub hover_command: f32,
    pub min_command: f32,
    pub max_command: f32,
}

impl MotorSpec {
    pub fn control_effect_signs(self) -> [f32; 3] {
        [
            sign_or_zero(self.position_body_m[1] * self.thrust_coefficient_n),
            sign_or_zero(self.position_body_m[0] * self.thrust_coefficient_n),
            sign_or_zero(
                self.spin_direction.yaw_reaction_sign() * self.reaction_torque_coefficient_nm,
            ),
        ]
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorGeometry {
    pub motors: [MotorSpec; QUAD_MOTOR_COUNT],
    pub hover_throttle: f32,
    pub mass_kg: f32,
    pub gravity_mps2: f32,
    pub roll_inertia_kg_m2: f32,
    pub pitch_inertia_kg_m2: f32,
    pub yaw_inertia_kg_m2: f32,
}

impl MotorGeometry {
    pub const fn quad_x() -> Self {
        let mass_kg = 1.0;
        let gravity_mps2 = 9.806_65;
        let hover_throttle = 0.5;
        let thrust_coefficient_n =
            mass_kg * gravity_mps2 / (QUAD_MOTOR_COUNT as f32 * hover_throttle);

        Self {
            motors: [
                MotorSpec {
                    logical_motor: 1,
                    output_index: 0,
                    position_body_m: [1.0, 1.0, 0.0],
                    spin_direction: SpinDirection::Clockwise,
                    thrust_coefficient_n,
                    reaction_torque_coefficient_nm: 0.1,
                    hover_command: hover_throttle,
                    min_command: 0.0,
                    max_command: 1.0,
                },
                MotorSpec {
                    logical_motor: 2,
                    output_index: 1,
                    position_body_m: [1.0, -1.0, 0.0],
                    spin_direction: SpinDirection::CounterClockwise,
                    thrust_coefficient_n,
                    reaction_torque_coefficient_nm: 0.1,
                    hover_command: hover_throttle,
                    min_command: 0.0,
                    max_command: 1.0,
                },
                MotorSpec {
                    logical_motor: 3,
                    output_index: 2,
                    position_body_m: [-1.0, -1.0, 0.0],
                    spin_direction: SpinDirection::Clockwise,
                    thrust_coefficient_n,
                    reaction_torque_coefficient_nm: 0.1,
                    hover_command: hover_throttle,
                    min_command: 0.0,
                    max_command: 1.0,
                },
                MotorSpec {
                    logical_motor: 4,
                    output_index: 3,
                    position_body_m: [-1.0, 1.0, 0.0],
                    spin_direction: SpinDirection::CounterClockwise,
                    thrust_coefficient_n,
                    reaction_torque_coefficient_nm: 0.1,
                    hover_command: hover_throttle,
                    min_command: 0.0,
                    max_command: 1.0,
                },
            ],
            hover_throttle,
            mass_kg,
            gravity_mps2,
            roll_inertia_kg_m2: 1.0,
            pitch_inertia_kg_m2: 1.0,
            yaw_inertia_kg_m2: 1.0,
        }
    }

    pub const fn f450_xing2_2809_1045_4s_v0() -> Self {
        let hover_command = 630.0 / 1100.0;
        let max_thrust_per_motor_n = 10.0;
        let yaw_moment_per_thrust_m = 0.016;

        Self {
            motors: [
                MotorSpec {
                    logical_motor: 1,
                    output_index: 0,
                    position_body_m: [-0.160_867, -0.160_867, 0.0],
                    spin_direction: SpinDirection::Clockwise,
                    thrust_coefficient_n: max_thrust_per_motor_n,
                    reaction_torque_coefficient_nm: max_thrust_per_motor_n
                        * yaw_moment_per_thrust_m,
                    hover_command,
                    min_command: 0.0,
                    max_command: 1.0,
                },
                MotorSpec {
                    logical_motor: 2,
                    output_index: 1,
                    position_body_m: [0.160_867, -0.160_867, 0.0],
                    spin_direction: SpinDirection::CounterClockwise,
                    thrust_coefficient_n: max_thrust_per_motor_n,
                    reaction_torque_coefficient_nm: max_thrust_per_motor_n
                        * yaw_moment_per_thrust_m,
                    hover_command,
                    min_command: 0.0,
                    max_command: 1.0,
                },
                MotorSpec {
                    logical_motor: 3,
                    output_index: 2,
                    position_body_m: [-0.160_867, 0.160_867, 0.0],
                    spin_direction: SpinDirection::CounterClockwise,
                    thrust_coefficient_n: max_thrust_per_motor_n,
                    reaction_torque_coefficient_nm: max_thrust_per_motor_n
                        * yaw_moment_per_thrust_m,
                    hover_command,
                    min_command: 0.0,
                    max_command: 1.0,
                },
                MotorSpec {
                    logical_motor: 4,
                    output_index: 3,
                    position_body_m: [0.160_867, 0.160_867, 0.0],
                    spin_direction: SpinDirection::Clockwise,
                    thrust_coefficient_n: max_thrust_per_motor_n,
                    reaction_torque_coefficient_nm: max_thrust_per_motor_n
                        * yaw_moment_per_thrust_m,
                    hover_command,
                    min_command: 0.0,
                    max_command: 1.0,
                },
            ],
            hover_throttle: hover_command,
            mass_kg: 1.338,
            gravity_mps2: 9.806_65,
            roll_inertia_kg_m2: 0.010,
            pitch_inertia_kg_m2: 0.011,
            yaw_inertia_kg_m2: 0.019,
        }
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        if !self.hover_throttle.is_finite() || !(0.0..=1.0).contains(&self.hover_throttle) {
            return Err("hover_throttle must be finite and normalized");
        }
        if !positive(self.mass_kg)
            || !positive(self.gravity_mps2)
            || !positive(self.roll_inertia_kg_m2)
            || !positive(self.pitch_inertia_kg_m2)
            || !positive(self.yaw_inertia_kg_m2)
        {
            return Err("mass, gravity, and inertias must be finite positive values");
        }

        let mut seen_logical = [false; QUAD_MOTOR_COUNT];
        let mut seen_output = [false; QUAD_MOTOR_COUNT];
        for motor in self.motors {
            if !(1..=QUAD_MOTOR_COUNT).contains(&motor.logical_motor) {
                return Err("logical_motor must be one-based and in range");
            }
            if motor.output_index >= QUAD_MOTOR_COUNT {
                return Err("output_index must be zero-based and in range");
            }
            if seen_logical[motor.logical_motor - 1] {
                return Err("logical_motor entries must be unique");
            }
            if seen_output[motor.output_index] {
                return Err("output_index entries must be unique");
            }
            if !motor.position_body_m.iter().all(|value| value.is_finite()) {
                return Err("motor positions must be finite");
            }
            if !positive(motor.thrust_coefficient_n)
                || !positive(motor.reaction_torque_coefficient_nm)
            {
                return Err("motor coefficients must be finite positive values");
            }
            if !valid_motor_command_range(motor.min_command, motor.hover_command, motor.max_command)
            {
                return Err("motor command range must be finite, normalized, and ordered");
            }
            seen_logical[motor.logical_motor - 1] = true;
            seen_output[motor.output_index] = true;
        }

        Ok(())
    }
}

impl Default for MotorGeometry {
    fn default() -> Self {
        Self::quad_x()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhysicalControlAllocator {
    geometry: MotorGeometry,
    limits: MixerLimits,
}

impl PhysicalControlAllocator {
    pub const fn new(geometry: MotorGeometry, limits: MixerLimits) -> Self {
        Self { geometry, limits }
    }

    pub const fn quad_x(limits: MixerLimits) -> Self {
        Self::new(MotorGeometry::quad_x(), limits)
    }

    pub const fn geometry(&self) -> MotorGeometry {
        self.geometry
    }

    pub const fn limits(&self) -> MixerLimits {
        self.limits
    }

    pub fn set_motor_order(&mut self, motor_order: MotorOrder) {
        for motor in self.geometry.motors.iter_mut() {
            if let Some(output_index) = motor_order.output_index_for_motor(motor.logical_motor) {
                motor.output_index = output_index;
            }
        }
    }

    pub fn motor_order(&self) -> Option<MotorOrder> {
        let mut output_for_motor = [0u8; QUAD_MOTOR_COUNT];

        for motor in self.geometry.motors {
            if !(1..=QUAD_MOTOR_COUNT).contains(&motor.logical_motor)
                || motor.output_index >= QUAD_MOTOR_COUNT
            {
                return None;
            }
            output_for_motor[motor.logical_motor - 1] = (motor.output_index + 1) as u8;
        }

        MotorOrder::from_one_based(output_for_motor)
    }

    pub fn allocate(&self, command: TorqueCommand) -> MotorOutputs<QUAD_MOTOR_COUNT> {
        let mut commands = [0.0; QUAD_MOTOR_COUNT];

        for motor in self.geometry.motors {
            let signs = motor.control_effect_signs();
            commands[motor.output_index] = command.throttle
                + command.roll * signs[0]
                + command.pitch * signs[1]
                + command.yaw * signs[2];
        }

        clamp_physical_outputs(commands, self.geometry, self.limits, command)
    }
}

impl Default for PhysicalControlAllocator {
    fn default() -> Self {
        Self::quad_x(MixerLimits::NORMALIZED)
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

fn clamp_physical_outputs(
    mut commands: [f32; QUAD_MOTOR_COUNT],
    geometry: MotorGeometry,
    limits: MixerLimits,
    command: TorqueCommand,
) -> MotorOutputs<QUAD_MOTOR_COUNT> {
    let mut limit_flags = MixerLimitFlags::NONE;

    for motor in geometry.motors {
        let min = motor.min_command.max(limits.min);
        let max = motor.max_command.min(limits.max);
        let command = &mut commands[motor.output_index];
        if *command < min {
            *command = min;
            limit_flags.throttle_lower = true;
        } else if *command > max {
            *command = max;
            limit_flags.throttle_upper = true;
        }
    }

    if limit_flags.any() {
        limit_flags.roll_pitch = command.roll != 0.0 || command.pitch != 0.0;
        limit_flags.yaw = command.yaw != 0.0;
    }

    MotorOutputs::with_limit_flags(commands, limit_flags)
}

fn sign_or_zero(value: f32) -> f32 {
    if value > 0.0 {
        1.0
    } else if value < 0.0 {
        -1.0
    } else {
        0.0
    }
}

fn positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn valid_motor_command_range(min: f32, hover: f32, max: f32) -> bool {
    min.is_finite()
        && hover.is_finite()
        && max.is_finite()
        && (0.0..=1.0).contains(&min)
        && (0.0..=1.0).contains(&hover)
        && (0.0..=1.0).contains(&max)
        && min <= hover
        && hover <= max
}

#[cfg(test)]
mod tests {
    use super::{
        DesaturationPriority, MixerLimits, MotorGeometry, MotorOrder, PhysicalControlAllocator,
        QuadXMixer, SpinDirection, TorqueCommand,
    };
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

    #[test]
    fn physical_allocator_default_quad_x_matches_legacy_mixer_basis() {
        let allocator = PhysicalControlAllocator::default();
        let mixer = QuadXMixer::default();

        for command in [
            TorqueCommand::new(0.0, 0.0, 0.0, 0.25),
            TorqueCommand::new(0.1, 0.0, 0.0, 0.5),
            TorqueCommand::new(0.0, 0.1, 0.0, 0.5),
            TorqueCommand::new(0.0, 0.0, 0.1, 0.5),
            TorqueCommand::new(0.1, 0.2, 0.05, 0.5),
        ] {
            assert_commands_near(
                allocator.allocate(command).commands,
                mixer.mix(command).commands,
            );
        }
    }

    #[test]
    fn physical_allocator_reports_axis_specific_limits() {
        let allocator =
            PhysicalControlAllocator::new(MotorGeometry::quad_x(), MixerLimits::new(0.45, 0.55));

        let roll_pitch = allocator.allocate(TorqueCommand::new(0.2, 0.2, 0.0, 0.5));
        let yaw = allocator.allocate(TorqueCommand::new(0.0, 0.0, 0.2, 0.5));

        assert!(roll_pitch.clamped);
        assert!(roll_pitch.limit_flags.roll_pitch);
        assert!(!roll_pitch.limit_flags.yaw);
        assert_eq!(
            roll_pitch.desaturation.priority,
            DesaturationPriority::PreserveThrottle
        );
        assert!(yaw.clamped);
        assert!(yaw.limit_flags.yaw);
        assert!(!yaw.limit_flags.roll_pitch);
        assert_eq!(yaw.desaturation.priority, DesaturationPriority::PreserveYaw);
    }

    #[test]
    fn physical_allocator_reports_roll_pitch_priority_when_yaw_competes() {
        let allocator =
            PhysicalControlAllocator::new(MotorGeometry::quad_x(), MixerLimits::new(0.45, 0.55));

        let outputs = allocator.allocate(TorqueCommand::new(0.2, 0.2, 0.2, 0.5));

        assert!(outputs.clamped);
        assert!(outputs.limit_flags.roll_pitch);
        assert!(outputs.limit_flags.yaw);
        assert_eq!(
            outputs.desaturation.priority,
            DesaturationPriority::PreserveRollPitchOverYaw
        );
        assert!(outputs.desaturation.reduced_yaw);
    }

    #[test]
    fn physical_allocator_can_remap_outputs_from_motor_order() {
        let order = MotorOrder::from_one_based([3, 1, 4, 2]).unwrap();
        let mut allocator = PhysicalControlAllocator::default();
        allocator.set_motor_order(order);

        let outputs = allocator.allocate(TorqueCommand::new(0.1, 0.2, 0.05, 0.5));

        assert_commands_near(outputs.commands, [0.65, 0.45, 0.75, 0.15]);
        assert_eq!(allocator.motor_order(), Some(order));
    }

    #[test]
    fn f450_geometry_matches_recorded_profile_values() {
        let geometry = MotorGeometry::f450_xing2_2809_1045_4s_v0();

        geometry.validate().expect("F450 geometry should validate");
        assert_scalar_near(geometry.mass_kg, 1.338, 0.000_001);
        assert_scalar_near(geometry.gravity_mps2, 9.806_65, 0.000_001);
        assert_scalar_near(geometry.hover_throttle, 630.0 / 1100.0, 0.000_001);
        assert_eq!(geometry.motors[0].logical_motor, 1);
        assert_eq!(geometry.motors[0].output_index, 0);
        assert_eq!(
            geometry.motors[0].position_body_m,
            [-0.160_867, -0.160_867, 0.0]
        );
        assert_eq!(geometry.motors[0].spin_direction, SpinDirection::Clockwise);
        assert_eq!(geometry.motors[3].logical_motor, 4);
        assert_eq!(geometry.motors[3].output_index, 3);
        assert_eq!(
            geometry.motors[3].position_body_m,
            [0.160_867, 0.160_867, 0.0]
        );
        assert_eq!(geometry.motors[3].spin_direction, SpinDirection::Clockwise);
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
