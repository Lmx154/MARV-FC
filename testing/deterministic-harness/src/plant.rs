use common::control::mixing::{QUAD_MOTOR_COUNT, QuadXMixer, TorqueCommand};

pub use common::control::mixing::{MotorGeometry, MotorSpec, SpinDirection};

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
