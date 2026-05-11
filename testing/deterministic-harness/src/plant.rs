use common::control::mixing::{QUAD_MOTOR_COUNT, QuadXMixer, TorqueCommand};

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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorSpec {
    pub logical_motor: usize,
    pub output_index: usize,
    pub position_body_m: [f32; 3],
    pub spin_direction: SpinDirection,
    pub thrust_coefficient_n: f32,
    pub reaction_torque_coefficient_nm: f32,
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
    pub fn quad_x() -> Self {
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
                },
                MotorSpec {
                    logical_motor: 2,
                    output_index: 1,
                    position_body_m: [1.0, -1.0, 0.0],
                    spin_direction: SpinDirection::CounterClockwise,
                    thrust_coefficient_n,
                    reaction_torque_coefficient_nm: 0.1,
                },
                MotorSpec {
                    logical_motor: 3,
                    output_index: 2,
                    position_body_m: [-1.0, -1.0, 0.0],
                    spin_direction: SpinDirection::Clockwise,
                    thrust_coefficient_n,
                    reaction_torque_coefficient_nm: 0.1,
                },
                MotorSpec {
                    logical_motor: 4,
                    output_index: 3,
                    position_body_m: [-1.0, 1.0, 0.0],
                    spin_direction: SpinDirection::CounterClockwise,
                    thrust_coefficient_n,
                    reaction_torque_coefficient_nm: 0.1,
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

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StaticPlantState {
    pub velocity_down_mps: f32,
    pub roll_rate_rps: f32,
    pub pitch_rate_rps: f32,
    pub yaw_rate_rps: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpenLoopResponse {
    pub total_thrust_n: f32,
    pub vertical_accel_down_mps2: f32,
    pub roll_accel_rps2: f32,
    pub pitch_accel_rps2: f32,
    pub yaw_accel_rps2: f32,
    pub next_state: StaticPlantState,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OpenLoopPlant {
    pub geometry: MotorGeometry,
    pub state: StaticPlantState,
}

impl OpenLoopPlant {
    pub fn new(geometry: MotorGeometry) -> Self {
        geometry.validate().expect("invalid motor geometry");
        Self {
            geometry,
            state: StaticPlantState::default(),
        }
    }

    pub fn apply_outputs(
        &mut self,
        outputs: [f32; QUAD_MOTOR_COUNT],
        dt_s: f32,
    ) -> OpenLoopResponse {
        assert!(
            dt_s.is_finite() && dt_s >= 0.0,
            "dt_s must be finite and non-negative"
        );
        assert!(
            outputs
                .iter()
                .all(|output| output.is_finite() && (0.0..=1.0).contains(output)),
            "motor outputs must be finite normalized values"
        );

        let mut total_thrust_n = 0.0;
        let mut roll_torque_nm = 0.0;
        let mut pitch_torque_nm = 0.0;
        let mut yaw_torque_nm = 0.0;

        for motor in self.geometry.motors {
            let command = outputs[motor.output_index];
            let thrust_n = command * motor.thrust_coefficient_n;
            total_thrust_n += thrust_n;
            roll_torque_nm += motor.position_body_m[1] * thrust_n;
            pitch_torque_nm += motor.position_body_m[0] * thrust_n;
            yaw_torque_nm += motor.spin_direction.yaw_reaction_sign()
                * command
                * motor.reaction_torque_coefficient_nm;
        }

        let vertical_accel_down_mps2 =
            self.geometry.gravity_mps2 - total_thrust_n / self.geometry.mass_kg;
        let roll_accel_rps2 = roll_torque_nm / self.geometry.roll_inertia_kg_m2;
        let pitch_accel_rps2 = pitch_torque_nm / self.geometry.pitch_inertia_kg_m2;
        let yaw_accel_rps2 = yaw_torque_nm / self.geometry.yaw_inertia_kg_m2;

        self.state.velocity_down_mps += vertical_accel_down_mps2 * dt_s;
        self.state.roll_rate_rps += roll_accel_rps2 * dt_s;
        self.state.pitch_rate_rps += pitch_accel_rps2 * dt_s;
        self.state.yaw_rate_rps += yaw_accel_rps2 * dt_s;

        OpenLoopResponse {
            total_thrust_n,
            vertical_accel_down_mps2,
            roll_accel_rps2,
            pitch_accel_rps2,
            yaw_accel_rps2,
            next_state: self.state,
        }
    }

    pub fn apply_torque_command(&mut self, command: TorqueCommand, dt_s: f32) -> OpenLoopResponse {
        let outputs = QuadXMixer::default().mix(command);
        self.apply_outputs(outputs.commands, dt_s)
    }
}

impl Default for OpenLoopPlant {
    fn default() -> Self {
        Self::new(MotorGeometry::default())
    }
}

fn positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}
