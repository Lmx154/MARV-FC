use std::collections::BTreeMap;

use common::{
    control::{
        attitude::AttitudeSetpoint,
        mixing::{
            MixerLimits, MotorGeometry, MotorOutputs, MotorSpec, PhysicalControlAllocator,
            QUAD_MOTOR_COUNT, SpinDirection, TorqueCommand,
        },
        pipeline::{EstimateSnapshot, ImuControlInput},
    },
    protocol::hilink::{HilSensorFrame, SimStamp, valid},
};

pub const GAZEBO_G0_DEFAULT_ENDPOINT: &str = "127.0.0.1:9000";
const ESTIMATOR_LEVEL_GRAVITY_BODY_MPS2: [f32; 3] = [0.0, 0.0, -9.806_65];

#[derive(Clone, Debug, PartialEq)]
pub struct SensorFrame {
    pub timestamp_us: u64,
    pub accel_mps2: [f64; 3],
    pub gyro_rps: [f64; 3],
    pub gravity_body_mps2: Option<[f64; 3]>,
    pub mag_body_ut: Option<[f64; 3]>,
    pub gps_position_ned_m: Option<[f64; 3]>,
    pub gps_velocity_ned_mps: Option<[f64; 3]>,
    pub baro_down_m: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GazeboBridgeConfig {
    pub clock_topic: String,
    pub imu_topic: String,
    pub magnetometer_topic: String,
    pub air_pressure_topic: String,
    pub navsat_topic: String,
    pub pose_topic: String,
    pub truth_model_name: String,
    pub actuator_topic: String,
    pub world_control_service: String,
    pub world_control_timeout_ms: u64,
    pub max_rotor_velocity_rad_s: f64,
    pub motor_direction_mode: String,
    pub motor_directions: [f64; 4],
    pub motor_gazebo_indices: [usize; 4],
    pub nominal_battery_voltage_v: f32,
    pub synthetic_sensors: bool,
}

impl GazeboBridgeConfig {
    pub fn parse(input: &str) -> Result<Self, String> {
        let values = parse_key_values(input);
        Ok(Self {
            clock_topic: required_string(&values, "topics.clock")?,
            imu_topic: required_string(&values, "topics.imu")?,
            magnetometer_topic: required_string(&values, "topics.magnetometer")?,
            air_pressure_topic: required_string(&values, "topics.air_pressure")?,
            navsat_topic: required_string(&values, "topics.navsat")?,
            pose_topic: required_string(&values, "topics.pose")?,
            truth_model_name: required_string(&values, "truth.model_name")?,
            actuator_topic: required_string(&values, "actuators.topic")?,
            world_control_service: required_string(&values, "world.control_service")?,
            world_control_timeout_ms: required_parse(&values, "world.control_timeout_ms")?,
            max_rotor_velocity_rad_s: required_parse(
                &values,
                "actuators.max_rotor_velocity_rad_s",
            )?,
            motor_direction_mode: values
                .get("actuators.motor_direction_mode")
                .cloned()
                .unwrap_or_else(|| "gazebo_model".to_string()),
            motor_directions: [
                required_parse(&values, "actuators.motor_1.direction")?,
                required_parse(&values, "actuators.motor_2.direction")?,
                required_parse(&values, "actuators.motor_3.direction")?,
                required_parse(&values, "actuators.motor_4.direction")?,
            ],
            motor_gazebo_indices: [
                optional_parse(&values, "actuators.motor_1.gazebo_index")?.unwrap_or(0),
                optional_parse(&values, "actuators.motor_2.gazebo_index")?.unwrap_or(1),
                optional_parse(&values, "actuators.motor_3.gazebo_index")?.unwrap_or(2),
                optional_parse(&values, "actuators.motor_4.gazebo_index")?.unwrap_or(3),
            ],
            nominal_battery_voltage_v: required_parse(&values, "nominal_battery_voltage_v")?,
            synthetic_sensors: required_bool(&values, "synthetic_sensors")?,
        })
    }

    pub fn motor_speeds_rad_s(&self, normalized: [f32; 4]) -> [f64; 4] {
        [
            self.motor_speed_rad_s(0, normalized[0]),
            self.motor_speed_rad_s(1, normalized[1]),
            self.motor_speed_rad_s(2, normalized[2]),
            self.motor_speed_rad_s(3, normalized[3]),
        ]
    }

    pub fn motor_speed_rad_s(&self, motor_index: usize, normalized: f32) -> f64 {
        let normalized = f64::from(normalized).clamp(0.0, 1.0);
        self.motor_directions[motor_index] * normalized * self.max_rotor_velocity_rad_s
    }

    pub fn gazebo_motor_speeds_rad_s(&self, normalized: [f32; 4]) -> [f64; 4] {
        let mut speeds = [0.0; 4];
        for (motor_index, command) in normalized.into_iter().enumerate() {
            speeds[self.motor_gazebo_indices[motor_index]] =
                self.motor_speed_rad_s(motor_index, command);
        }
        speeds
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GazeboActuatorAxis {
    Roll,
    Pitch,
    Yaw,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GazeboBodyFrame {
    Flu,
    BridgeFrd,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GazeboRotorSpin {
    Clockwise,
    CounterClockwise,
}

impl GazeboRotorSpin {
    pub const fn yaw_moment_sign_flu(self) -> f32 {
        match self {
            Self::Clockwise => 1.0,
            Self::CounterClockwise => -1.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct GazeboAirframeMotorConfig {
    pub output_index: usize,
    pub name: String,
    pub position_flu_m: [f32; 3],
    pub spin_direction: GazeboRotorSpin,
    pub runtime_signs: Option<[f32; 3]>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GazeboAirframeConfig {
    pub name: String,
    pub body_frame: GazeboBodyFrame,
    pub runtime_gyro_frame: GazeboBodyFrame,
    pub mass_kg: f32,
    pub gravity_mps2: f32,
    pub center_of_mass_m: [f32; 3],
    pub inertia_kg_m2: [f32; 6],
    pub motor_xy_magnitude_m: f32,
    pub total_hover_thrust_n: f32,
    pub per_motor_hover_thrust_n: f32,
    pub max_thrust_per_motor_n: f32,
    pub yaw_moment_per_thrust_m: f32,
    pub max_rotor_velocity_rad_s: f32,
    pub motor_constant_n_per_rad_s2: f32,
    pub estimated_hover_omega_rad_s: f32,
    pub estimated_hover_rpm: f32,
    pub motors: [GazeboAirframeMotorConfig; 4],
}

impl GazeboAirframeConfig {
    pub fn parse(input: &str) -> Result<Self, String> {
        let values = parse_key_values(input);
        Ok(Self {
            name: required_string(&values, "airframe.name")?,
            body_frame: required_body_frame(&values, "frame.body")?,
            runtime_gyro_frame: required_body_frame(&values, "runtime.gyro_frame")?,
            mass_kg: required_parse(&values, "mass_kg")?,
            gravity_mps2: required_parse(&values, "gravity_mps2")?,
            center_of_mass_m: required_array3(&values, "center_of_mass_m")?,
            inertia_kg_m2: [
                required_parse(&values, "inertia.ixx_kg_m2")?,
                required_parse(&values, "inertia.iyy_kg_m2")?,
                required_parse(&values, "inertia.izz_kg_m2")?,
                required_parse(&values, "inertia.ixy_kg_m2")?,
                required_parse(&values, "inertia.ixz_kg_m2")?,
                required_parse(&values, "inertia.iyz_kg_m2")?,
            ],
            motor_xy_magnitude_m: required_parse(&values, "geometry.motor_xy_magnitude_m")?,
            total_hover_thrust_n: required_parse(&values, "hover.total_thrust_n")?,
            per_motor_hover_thrust_n: required_parse(&values, "hover.per_motor_thrust_n")?,
            max_thrust_per_motor_n: required_parse(&values, "motor_model.max_thrust_per_motor_n")?,
            yaw_moment_per_thrust_m: required_parse(
                &values,
                "motor_model.yaw_moment_per_thrust_m",
            )?,
            max_rotor_velocity_rad_s: required_parse(
                &values,
                "motor_model.max_rotor_velocity_rad_s",
            )?,
            motor_constant_n_per_rad_s2: required_parse(
                &values,
                "motor_model.motor_constant_n_per_rad_s2",
            )?,
            estimated_hover_omega_rad_s: required_parse(
                &values,
                "motor_model.estimated_hover_omega_rad_s",
            )?,
            estimated_hover_rpm: required_parse(&values, "motor_model.estimated_hover_rpm")?,
            motors: [
                motor_config(&values, 1)?,
                motor_config(&values, 2)?,
                motor_config(&values, 3)?,
                motor_config(&values, 4)?,
            ],
        })
    }

    pub fn validate(&self) -> Result<(), &'static str> {
        if self.body_frame != GazeboBodyFrame::Flu {
            return Err("airframe body frame must be FLU");
        }
        if !positive(self.mass_kg)
            || !positive(self.gravity_mps2)
            || !positive(self.motor_xy_magnitude_m)
            || !positive(self.total_hover_thrust_n)
            || !positive(self.per_motor_hover_thrust_n)
            || !positive(self.max_thrust_per_motor_n)
            || !positive(self.yaw_moment_per_thrust_m)
            || !positive(self.max_rotor_velocity_rad_s)
            || !positive(self.motor_constant_n_per_rad_s2)
            || !positive(self.estimated_hover_omega_rad_s)
            || !positive(self.estimated_hover_rpm)
        {
            return Err("airframe scalar physics values must be finite positive values");
        }
        if !self.center_of_mass_m.iter().all(|value| value.is_finite())
            || !self.inertia_kg_m2.iter().all(|value| value.is_finite())
        {
            return Err("airframe vector values must be finite");
        }

        let hover_from_mass = self.mass_kg * self.gravity_mps2;
        if (hover_from_mass - self.total_hover_thrust_n).abs() > 0.02 {
            return Err("hover thrust must match mass * gravity");
        }
        if self.estimated_hover_omega_rad_s >= self.max_rotor_velocity_rad_s {
            return Err("estimated hover omega must be below max rotor velocity");
        }
        let hover_thrust_from_motor_model =
            self.motor_constant_n_per_rad_s2 * self.estimated_hover_omega_rad_s.powi(2);
        if (hover_thrust_from_motor_model - self.per_motor_hover_thrust_n).abs() > 0.02 {
            return Err("per-motor hover thrust must match motor constant * hover omega^2");
        }
        let max_thrust_from_motor_model =
            self.motor_constant_n_per_rad_s2 * self.max_rotor_velocity_rad_s.powi(2);
        if (max_thrust_from_motor_model - self.max_thrust_per_motor_n).abs() > 0.02 {
            return Err("max motor thrust must match motor constant * max omega^2");
        }

        let mut seen_outputs = [false; 4];
        for motor in &self.motors {
            if motor.output_index >= 4 {
                return Err("motor output index must be in range");
            }
            if seen_outputs[motor.output_index] {
                return Err("motor output indexes must be unique");
            }
            if !motor.position_flu_m.iter().all(|value| value.is_finite()) {
                return Err("motor positions must be finite");
            }
            if let Some(signs) = motor.runtime_signs {
                if !signs.iter().all(|value| value.is_finite() && *value != 0.0) {
                    return Err("runtime motor signs must be finite non-zero values");
                }
            }
            seen_outputs[motor.output_index] = true;
        }

        Ok(())
    }

    pub fn hover_motor_command(&self) -> f32 {
        self.estimated_hover_omega_rad_s / self.max_rotor_velocity_rad_s
    }

    pub fn hover_motors(&self) -> [f32; 4] {
        [self.hover_motor_command(); 4]
    }

    pub fn motor_bump_motors(&self, output_index: usize) -> [f32; 4] {
        let mut motors = self.hover_motors();
        motors[output_index] = self.probe_high_motor_command();
        motors
    }

    pub fn reset_settle_motors(&self) -> [f32; 4] {
        [self.probe_high_motor_command(); 4]
    }

    pub fn motor_runtime_signs(&self, output_index: usize) -> [f32; 3] {
        let motor = self.motor_by_output_index(output_index);
        if let Some(signs) = motor.runtime_signs {
            return signs;
        }

        let flu = [
            motor.position_flu_m[1],
            -motor.position_flu_m[0],
            motor.spin_direction.yaw_moment_sign_flu() * self.yaw_moment_per_thrust_m,
        ];
        self.convert_flu_rates(flu)
    }

    pub fn axis_basis_motors(&self, axis: GazeboActuatorAxis, positive: bool) -> [f32; 4] {
        let axis_index = axis_index(axis);
        let mut motors = self.hover_motors();
        let low = self.probe_low_motor_command();
        let high = self.probe_high_motor_command();
        for motor in &self.motors {
            let sign = self.motor_runtime_signs(motor.output_index)[axis_index];
            motors[motor.output_index] = if (sign >= 0.0) == positive { high } else { low };
        }
        motors
    }

    pub fn runtime_control_motors(&self, command: TorqueCommand) -> MotorOutputs<QUAD_MOTOR_COUNT> {
        PhysicalControlAllocator::new(self.runtime_motor_geometry(), MixerLimits::NORMALIZED)
            .allocate(command)
    }

    pub fn runtime_motor_geometry(&self) -> MotorGeometry {
        let mut motors = [runtime_motor_spec(
            1,
            0,
            [1.0, 1.0, 1.0],
            self.hover_motor_command(),
            self.max_thrust_per_motor_n,
            self.yaw_moment_per_thrust_m,
        ); QUAD_MOTOR_COUNT];

        for motor in &self.motors {
            let signs = self.motor_runtime_signs(motor.output_index);
            motors[motor.output_index] = runtime_motor_spec(
                motor.output_index + 1,
                motor.output_index,
                signs,
                self.hover_motor_command(),
                self.max_thrust_per_motor_n,
                self.yaw_moment_per_thrust_m,
            );
        }

        MotorGeometry {
            motors,
            hover_throttle: self.hover_motor_command(),
            mass_kg: self.mass_kg,
            gravity_mps2: self.gravity_mps2,
            roll_inertia_kg_m2: self.inertia_kg_m2[0],
            pitch_inertia_kg_m2: self.inertia_kg_m2[1],
            yaw_inertia_kg_m2: self.inertia_kg_m2[2],
        }
    }

    fn probe_low_motor_command(&self) -> f32 {
        0.0
    }

    fn probe_high_motor_command(&self) -> f32 {
        1.0
    }

    fn motor_by_output_index(&self, output_index: usize) -> &GazeboAirframeMotorConfig {
        self.motors
            .iter()
            .find(|motor| motor.output_index == output_index)
            .expect("airframe validation ensures every output index is present")
    }

    fn convert_flu_rates(&self, flu: [f32; 3]) -> [f32; 3] {
        match self.runtime_gyro_frame {
            GazeboBodyFrame::Flu => flu,
            GazeboBodyFrame::BridgeFrd => [flu[0], -flu[1], -flu[2]],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GazeboActuatorExpectation {
    AngularRateNearZero,
    AxisSign {
        axis: GazeboActuatorAxis,
        positive: bool,
    },
    MotorBump {
        motor: usize,
        roll_positive: bool,
        pitch_positive: bool,
        yaw_positive: bool,
    },
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GazeboActuatorTruthCase {
    pub name: &'static str,
    pub motors: [f32; 4],
    pub expectation: GazeboActuatorExpectation,
}

pub fn gazebo_g0_actuator_truth_cases(
    airframe: &GazeboAirframeConfig,
) -> Vec<GazeboActuatorTruthCase> {
    vec![
        GazeboActuatorTruthCase {
            name: "zero",
            motors: [0.0, 0.0, 0.0, 0.0],
            expectation: GazeboActuatorExpectation::AngularRateNearZero,
        },
        GazeboActuatorTruthCase {
            name: "hover",
            motors: airframe.hover_motors(),
            expectation: GazeboActuatorExpectation::AngularRateNearZero,
        },
        GazeboActuatorTruthCase {
            name: "motor_1_bump",
            motors: airframe.motor_bump_motors(0),
            expectation: motor_bump_expectation(airframe, 0),
        },
        GazeboActuatorTruthCase {
            name: "motor_2_bump",
            motors: airframe.motor_bump_motors(1),
            expectation: motor_bump_expectation(airframe, 1),
        },
        GazeboActuatorTruthCase {
            name: "motor_3_bump",
            motors: airframe.motor_bump_motors(2),
            expectation: motor_bump_expectation(airframe, 2),
        },
        GazeboActuatorTruthCase {
            name: "motor_4_bump",
            motors: airframe.motor_bump_motors(3),
            expectation: motor_bump_expectation(airframe, 3),
        },
        GazeboActuatorTruthCase {
            name: "roll_positive",
            motors: airframe.axis_basis_motors(GazeboActuatorAxis::Roll, true),
            expectation: GazeboActuatorExpectation::AxisSign {
                axis: GazeboActuatorAxis::Roll,
                positive: true,
            },
        },
        GazeboActuatorTruthCase {
            name: "roll_negative",
            motors: airframe.axis_basis_motors(GazeboActuatorAxis::Roll, false),
            expectation: GazeboActuatorExpectation::AxisSign {
                axis: GazeboActuatorAxis::Roll,
                positive: false,
            },
        },
        GazeboActuatorTruthCase {
            name: "pitch_positive",
            motors: airframe.axis_basis_motors(GazeboActuatorAxis::Pitch, true),
            expectation: GazeboActuatorExpectation::AxisSign {
                axis: GazeboActuatorAxis::Pitch,
                positive: true,
            },
        },
        GazeboActuatorTruthCase {
            name: "pitch_negative",
            motors: airframe.axis_basis_motors(GazeboActuatorAxis::Pitch, false),
            expectation: GazeboActuatorExpectation::AxisSign {
                axis: GazeboActuatorAxis::Pitch,
                positive: false,
            },
        },
        GazeboActuatorTruthCase {
            name: "yaw_positive",
            motors: airframe.axis_basis_motors(GazeboActuatorAxis::Yaw, true),
            expectation: GazeboActuatorExpectation::AxisSign {
                axis: GazeboActuatorAxis::Yaw,
                positive: true,
            },
        },
        GazeboActuatorTruthCase {
            name: "yaw_negative",
            motors: airframe.axis_basis_motors(GazeboActuatorAxis::Yaw, false),
            expectation: GazeboActuatorExpectation::AxisSign {
                axis: GazeboActuatorAxis::Yaw,
                positive: false,
            },
        },
    ]
}

fn motor_bump_expectation(
    airframe: &GazeboAirframeConfig,
    motor_index: usize,
) -> GazeboActuatorExpectation {
    let signs = airframe.motor_runtime_signs(motor_index);
    GazeboActuatorExpectation::MotorBump {
        motor: motor_index + 1,
        roll_positive: signs[0] >= 0.0,
        pitch_positive: signs[1] >= 0.0,
        yaw_positive: signs[2] >= 0.0,
    }
}

fn runtime_motor_spec(
    logical_motor: usize,
    output_index: usize,
    signs: [f32; 3],
    hover_command: f32,
    max_thrust_per_motor_n: f32,
    yaw_moment_per_thrust_m: f32,
) -> MotorSpec {
    MotorSpec {
        logical_motor,
        output_index,
        position_body_m: [signs[1], signs[0], 0.0],
        spin_direction: if signs[2] >= 0.0 {
            SpinDirection::CounterClockwise
        } else {
            SpinDirection::Clockwise
        },
        thrust_coefficient_n: max_thrust_per_motor_n,
        reaction_torque_coefficient_nm: max_thrust_per_motor_n * yaw_moment_per_thrust_m,
        hover_command,
        min_command: 0.0,
        max_command: 1.0,
    }
}

fn axis_index(axis: GazeboActuatorAxis) -> usize {
    match axis {
        GazeboActuatorAxis::Roll => 0,
        GazeboActuatorAxis::Pitch => 1,
        GazeboActuatorAxis::Yaw => 2,
    }
}

pub fn format_gazebo_actuator_line(
    sequence: u64,
    sim_time_us: Option<u64>,
    motors: [f32; 4],
) -> String {
    let motors = motors.map(|motor| motor.clamp(0.0, 1.0));
    match sim_time_us {
        Some(sim_time_us) => format!(
            "ACTUATOR seq={sequence} sim_time_us={sim_time_us} m0={:.3} m1={:.3} m2={:.3} m3={:.3}\n",
            motors[0], motors[1], motors[2], motors[3]
        ),
        None => format!(
            "ACTUATOR seq={sequence} m0={:.3} m1={:.3} m2={:.3} m3={:.3}\n",
            motors[0], motors[1], motors[2], motors[3]
        ),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GazeboSimControlAction {
    Reset,
    Pause,
    Play,
}

impl GazeboSimControlAction {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Reset => "reset",
            Self::Pause => "pause",
            Self::Play => "play",
        }
    }
}

pub fn format_gazebo_sim_control_line(sequence: u64, action: GazeboSimControlAction) -> String {
    format!("SIM_CONTROL seq={sequence} action={}\n", action.as_str())
}

#[derive(Clone, Debug, PartialEq)]
pub struct GazeboSensorLine {
    pub sequence: u64,
    pub sim_time_us: u64,
    pub clock_source: Option<String>,
    pub valid_flags: u32,
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
    pub orientation_quat: Option<[f32; 4]>,
    pub euler_rad: Option<[f32; 3]>,
    pub position_ned_m: Option<[f32; 3]>,
    pub mag_ut: [f32; 3],
    pub pressure_pa: f32,
    pub baro_altitude_m: f32,
    pub temperature_c: f32,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
    pub vel_ned_mps: [f32; 3],
    pub sats: u8,
    pub fix_type: u8,
    pub battery_voltage_v: f32,
    pub rssi_dbm: i16,
    pub snr_db_x100: i16,
    pub loss_pct_x100: u16,
}

impl GazeboSensorLine {
    pub fn parse(line: &str) -> Option<Self> {
        if !line.starts_with("SENSOR") {
            return None;
        }

        let mut frame = Self {
            sequence: 0,
            sim_time_us: 0,
            clock_source: None,
            valid_flags: 0,
            accel_mps2: [0.0, 0.0, -9.81],
            gyro_rps: [0.0, 0.0, 0.0],
            orientation_quat: None,
            euler_rad: None,
            position_ned_m: None,
            mag_ut: [0.0, 0.0, 0.0],
            pressure_pa: 101_325.0,
            baro_altitude_m: 0.0,
            temperature_c: 25.0,
            lat_deg: 0.0,
            lon_deg: 0.0,
            alt_msl_m: 0.0,
            vel_ned_mps: [0.0, 0.0, 0.0],
            sats: 0,
            fix_type: 0,
            battery_voltage_v: 0.0,
            rssi_dbm: 0,
            snr_db_x100: 0,
            loss_pct_x100: 0,
        };
        let mut has_sequence = false;
        let mut has_sim_time = false;

        for token in line.split_whitespace() {
            let Some((key, value)) = token.split_once('=') else {
                continue;
            };
            match key {
                "seq" => parse_into(value, &mut frame.sequence, &mut has_sequence),
                "sim_time_us" => parse_into(value, &mut frame.sim_time_us, &mut has_sim_time),
                "clock" => frame.clock_source = Some(value.to_string()),
                "valid" | "valid_flags" => parse_value(value, &mut frame.valid_flags),
                "ax" => parse_value(value, &mut frame.accel_mps2[0]),
                "ay" => parse_value(value, &mut frame.accel_mps2[1]),
                "az" => parse_value(value, &mut frame.accel_mps2[2]),
                "gx" => parse_value(value, &mut frame.gyro_rps[0]),
                "gy" => parse_value(value, &mut frame.gyro_rps[1]),
                "gz" => parse_value(value, &mut frame.gyro_rps[2]),
                "qw" => parse_option_array4(value, &mut frame.orientation_quat, 0),
                "qx" => parse_option_array4(value, &mut frame.orientation_quat, 1),
                "qy" => parse_option_array4(value, &mut frame.orientation_quat, 2),
                "qz" => parse_option_array4(value, &mut frame.orientation_quat, 3),
                "roll" => parse_option_array3(value, &mut frame.euler_rad, 0),
                "pitch" => parse_option_array3(value, &mut frame.euler_rad, 1),
                "yaw" => parse_option_array3(value, &mut frame.euler_rad, 2),
                "pn" => parse_option_array3(value, &mut frame.position_ned_m, 0),
                "pe" => parse_option_array3(value, &mut frame.position_ned_m, 1),
                "pd" => parse_option_array3(value, &mut frame.position_ned_m, 2),
                "mx" => parse_value(value, &mut frame.mag_ut[0]),
                "my" => parse_value(value, &mut frame.mag_ut[1]),
                "mz" => parse_value(value, &mut frame.mag_ut[2]),
                "pressure_pa" => parse_value(value, &mut frame.pressure_pa),
                "baro_alt_m" => parse_value(value, &mut frame.baro_altitude_m),
                "temp_c" => parse_value(value, &mut frame.temperature_c),
                "lat_deg" => parse_value(value, &mut frame.lat_deg),
                "lon_deg" => parse_value(value, &mut frame.lon_deg),
                "alt_msl_m" => parse_value(value, &mut frame.alt_msl_m),
                "vn" => parse_value(value, &mut frame.vel_ned_mps[0]),
                "ve" => parse_value(value, &mut frame.vel_ned_mps[1]),
                "vd" => parse_value(value, &mut frame.vel_ned_mps[2]),
                "sats" => parse_value(value, &mut frame.sats),
                "fix" => parse_value(value, &mut frame.fix_type),
                "battery_v" => parse_value(value, &mut frame.battery_voltage_v),
                "rssi_dbm" => parse_value(value, &mut frame.rssi_dbm),
                "snr_db_x100" => parse_value(value, &mut frame.snr_db_x100),
                "loss_pct_x100" => parse_value(value, &mut frame.loss_pct_x100),
                _ => {}
            }
        }

        (has_sequence && has_sim_time).then_some(frame)
    }

    pub fn to_hil_sensor_frame(&self) -> HilSensorFrame {
        HilSensorFrame {
            stamp: SimStamp {
                sim_tick: self.sequence,
                sim_time_us: self.sim_time_us,
            },
            valid_flags: self.valid_flags,
            accel_mps2: self.accel_mps2,
            gyro_rps: self.gyro_rps,
            mag_ut: self.mag_ut,
            pressure_pa: self.pressure_pa,
            baro_altitude_m: self.baro_altitude_m,
            temperature_c: self.temperature_c,
            lat_deg: self.lat_deg,
            lon_deg: self.lon_deg,
            alt_msl_m: self.alt_msl_m,
            vel_ned_mps: self.vel_ned_mps,
            sats: self.sats,
            fix_type: self.fix_type,
            reserved0: [0; 3],
            battery_voltage_v: self.battery_voltage_v,
            rssi_dbm: self.rssi_dbm,
            snr_db_x100: self.snr_db_x100,
            loss_pct_x100: self.loss_pct_x100,
        }
    }

    pub fn attitude_quaternion(&self) -> Option<[f32; 4]> {
        if let Some(quaternion) = self.orientation_quat {
            if quaternion.iter().all(|value| value.is_finite()) {
                return Some(quaternion);
            }
        }

        let [roll_rad, pitch_rad, yaw_rad] = self.euler_rad?;
        AttitudeSetpoint::from_euler_rad(roll_rad, pitch_rad, yaw_rad)
            .map(|attitude| attitude.quaternion)
    }

    pub fn to_truth_estimate(&self, origin: GazeboTruthOrigin) -> Option<EstimateSnapshot> {
        Some(EstimateSnapshot {
            position_ned_m: origin.position_ned_m(self)?,
            velocity_ned_mps: self.vel_ned_mps,
            quaternion: self.attitude_quaternion()?,
            valid: true,
        })
    }

    pub fn to_truth_imu(&self) -> ImuControlInput {
        ImuControlInput {
            accel_mps2: self.accel_mps2,
            gyro_rps: self.gyro_rps,
        }
    }

    pub fn to_estimator_sensor_frame(&self, origin: GazeboTruthOrigin) -> Option<SensorFrame> {
        if !self.has_valid(valid::ACCEL) || !self.has_valid(valid::GYRO) {
            return None;
        }

        Some(SensorFrame {
            timestamp_us: self.sim_time_us,
            accel_mps2: estimator_linear_accel_mps2(self.accel_mps2, self.attitude_quaternion())
                .map(f64::from),
            gyro_rps: self.gyro_rps.map(f64::from),
            gravity_body_mps2: Some(
                self.attitude_quaternion()
                    .and_then(gravity_body_mps2_from_quaternion)
                    .unwrap_or(self.accel_mps2)
                    .map(f64::from),
            ),
            mag_body_ut: self
                .has_valid(valid::MAG)
                .then(|| self.mag_ut.map(f64::from)),
            gps_position_ned_m: self
                .has_valid(valid::GPS)
                .then(|| origin.position_ned_m(self))
                .flatten()
                .map(|position| position.map(f64::from)),
            gps_velocity_ned_mps: self
                .has_valid(valid::GPS)
                .then(|| self.vel_ned_mps.map(f64::from)),
            baro_down_m: self
                .has_valid(valid::BARO)
                .then(|| origin.baro_down_m(self))
                .flatten()
                .map(f64::from),
        })
    }

    fn has_valid(&self, flag: u32) -> bool {
        self.valid_flags & flag != 0
    }
}

fn estimator_linear_accel_mps2(
    accel_mps2: [f32; 3],
    attitude_quaternion: Option<[f32; 4]>,
) -> [f32; 3] {
    let gravity_body_mps2 = attitude_quaternion
        .and_then(gravity_body_mps2_from_quaternion)
        .unwrap_or(ESTIMATOR_LEVEL_GRAVITY_BODY_MPS2);

    [
        accel_mps2[0] - gravity_body_mps2[0],
        accel_mps2[1] - gravity_body_mps2[1],
        accel_mps2[2] - gravity_body_mps2[2],
    ]
}

fn gravity_body_mps2_from_quaternion(quaternion: [f32; 4]) -> Option<[f32; 3]> {
    if !quaternion.iter().all(|value| value.is_finite()) {
        return None;
    }

    let norm = (quaternion[0] * quaternion[0]
        + quaternion[1] * quaternion[1]
        + quaternion[2] * quaternion[2]
        + quaternion[3] * quaternion[3])
        .sqrt();
    if norm <= f32::EPSILON {
        return None;
    }

    let w = quaternion[0] / norm;
    let x = quaternion[1] / norm;
    let y = quaternion[2] / norm;
    let z = quaternion[3] / norm;
    let gravity_down_mps2 = ESTIMATOR_LEVEL_GRAVITY_BODY_MPS2[2];

    Some([
        gravity_down_mps2 * (2.0 * (x * z - w * y)),
        gravity_down_mps2 * (2.0 * (y * z + w * x)),
        gravity_down_mps2 * (1.0 - 2.0 * (x * x + y * y)),
    ])
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GazeboTruthOrigin {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
    pub baro_altitude_m: f32,
    pub position_ned_m: Option<[f32; 3]>,
}

impl GazeboTruthOrigin {
    pub fn from_frame(frame: &GazeboSensorLine) -> Option<Self> {
        if !frame.lat_deg.is_finite() || !frame.lon_deg.is_finite() || !frame.alt_msl_m.is_finite()
        {
            return None;
        }

        Some(Self {
            lat_deg: frame.lat_deg,
            lon_deg: frame.lon_deg,
            alt_msl_m: frame.alt_msl_m,
            baro_altitude_m: frame.baro_altitude_m,
            position_ned_m: frame.position_ned_m,
        })
    }

    pub fn position_ned_m(&self, frame: &GazeboSensorLine) -> Option<[f32; 3]> {
        if let (Some(origin), Some(position)) = (self.position_ned_m, frame.position_ned_m) {
            return Some([
                position[0] - origin[0],
                position[1] - origin[1],
                position[2] - origin[2],
            ]);
        }

        if !frame.lat_deg.is_finite() || !frame.lon_deg.is_finite() || !frame.alt_msl_m.is_finite()
        {
            return None;
        }

        let meters_per_lat_deg = 111_320.0_f64;
        let meters_per_lon_deg = meters_per_lat_deg * self.lat_deg.to_radians().cos();
        if meters_per_lon_deg.abs() <= f64::EPSILON {
            return None;
        }

        Some([
            ((frame.lat_deg - self.lat_deg) * meters_per_lat_deg) as f32,
            ((self.lon_deg - frame.lon_deg) * meters_per_lon_deg) as f32,
            self.alt_msl_m - frame.alt_msl_m,
        ])
    }

    pub fn gps_position_ned_m(&self, frame: &GazeboSensorLine) -> Option<[f32; 3]> {
        if !frame.lat_deg.is_finite() || !frame.lon_deg.is_finite() || !frame.alt_msl_m.is_finite()
        {
            return None;
        }

        let meters_per_lat_deg = 111_320.0_f64;
        let meters_per_lon_deg = meters_per_lat_deg * self.lat_deg.to_radians().cos();
        if meters_per_lon_deg.abs() <= f64::EPSILON {
            return None;
        }

        Some([
            ((frame.lat_deg - self.lat_deg) * meters_per_lat_deg) as f32,
            ((self.lon_deg - frame.lon_deg) * meters_per_lon_deg) as f32,
            self.alt_msl_m - frame.alt_msl_m,
        ])
    }

    pub fn baro_down_m(&self, frame: &GazeboSensorLine) -> Option<f32> {
        if self.baro_altitude_m.is_finite() && frame.baro_altitude_m.is_finite() {
            Some(self.baro_altitude_m - frame.baro_altitude_m)
        } else {
            None
        }
    }
}

fn parse_key_values(input: &str) -> BTreeMap<String, String> {
    let mut values = BTreeMap::new();
    for line in input.lines() {
        let without_comment = line.split_once('#').map_or(line, |(value, _)| value).trim();
        if without_comment.is_empty() {
            continue;
        }
        let Some((key, value)) = without_comment.split_once('=') else {
            continue;
        };
        values.insert(key.trim().to_string(), value.trim().to_string());
    }
    values
}

fn required_string(values: &BTreeMap<String, String>, key: &str) -> Result<String, String> {
    values
        .get(key)
        .cloned()
        .ok_or_else(|| format!("missing Gazebo bridge config key {key}"))
}

fn required_body_frame(
    values: &BTreeMap<String, String>,
    key: &str,
) -> Result<GazeboBodyFrame, String> {
    let value = required_string(values, key)?;
    match value.to_ascii_lowercase().as_str() {
        "flu" | "body_flu" => Ok(GazeboBodyFrame::Flu),
        "frd" | "bridge_frd" => Ok(GazeboBodyFrame::BridgeFrd),
        _ => Err(format!("invalid Gazebo airframe frame key {key}: {value}")),
    }
}

fn required_array3(values: &BTreeMap<String, String>, key: &str) -> Result<[f32; 3], String> {
    let value = required_string(values, key)?;
    parse_array3(key, &value)
}

fn optional_array3(
    values: &BTreeMap<String, String>,
    key: &str,
) -> Result<Option<[f32; 3]>, String> {
    values
        .get(key)
        .map(|value| parse_array3(key, value))
        .transpose()
}

fn parse_array3(key: &str, value: &str) -> Result<[f32; 3], String> {
    let mut parts = value.split(',').map(str::trim);
    let parse_part = |part: Option<&str>| -> Result<f32, String> {
        part.ok_or_else(|| format!("invalid Gazebo airframe vector key {key}: {value}"))?
            .parse()
            .map_err(|error| format!("invalid Gazebo airframe vector key {key}: {error}"))
    };
    let parsed = [
        parse_part(parts.next())?,
        parse_part(parts.next())?,
        parse_part(parts.next())?,
    ];
    if parts.next().is_some() {
        return Err(format!("invalid Gazebo airframe vector key {key}: {value}"));
    }
    Ok(parsed)
}

fn motor_config(
    values: &BTreeMap<String, String>,
    motor_number: usize,
) -> Result<GazeboAirframeMotorConfig, String> {
    let prefix = format!("motor.{motor_number}");
    Ok(GazeboAirframeMotorConfig {
        output_index: required_parse(values, &format!("{prefix}.output_index"))?,
        name: required_string(values, &format!("{prefix}.name"))?,
        position_flu_m: required_array3(values, &format!("{prefix}.position_m"))?,
        spin_direction: required_spin_direction(values, &format!("{prefix}.spin_direction"))?,
        runtime_signs: optional_array3(values, &format!("runtime.{prefix}.signs"))?,
    })
}

fn required_spin_direction(
    values: &BTreeMap<String, String>,
    key: &str,
) -> Result<GazeboRotorSpin, String> {
    let value = required_string(values, key)?;
    match value.to_ascii_lowercase().as_str() {
        "cw" | "clockwise" => Ok(GazeboRotorSpin::Clockwise),
        "ccw" | "counterclockwise" | "counter_clockwise" => Ok(GazeboRotorSpin::CounterClockwise),
        _ => Err(format!(
            "invalid Gazebo airframe spin direction key {key}: {value}"
        )),
    }
}

fn required_parse<T>(values: &BTreeMap<String, String>, key: &str) -> Result<T, String>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    let value = values
        .get(key)
        .ok_or_else(|| format!("missing Gazebo bridge config key {key}"))?;
    value
        .parse()
        .map_err(|error| format!("invalid Gazebo bridge config key {key}: {error}"))
}

fn optional_parse<T>(values: &BTreeMap<String, String>, key: &str) -> Result<Option<T>, String>
where
    T: std::str::FromStr,
    T::Err: std::fmt::Display,
{
    values
        .get(key)
        .map(|value| {
            value
                .parse()
                .map_err(|error| format!("invalid Gazebo bridge config key {key}: {error}"))
        })
        .transpose()
}

fn required_bool(values: &BTreeMap<String, String>, key: &str) -> Result<bool, String> {
    let value = values
        .get(key)
        .ok_or_else(|| format!("missing Gazebo bridge config key {key}"))?
        .to_ascii_lowercase();
    match value.as_str() {
        "1" | "true" | "yes" | "on" => Ok(true),
        "0" | "false" | "no" | "off" => Ok(false),
        _ => Err(format!("invalid Gazebo bridge boolean key {key}: {value}")),
    }
}

fn positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn parse_value<T: std::str::FromStr>(value: &str, target: &mut T) {
    if let Ok(parsed) = value.parse() {
        *target = parsed;
    }
}

fn parse_option_array3(value: &str, target: &mut Option<[f32; 3]>, index: usize) {
    if let Ok(parsed) = value.parse() {
        let values = target.get_or_insert([0.0; 3]);
        values[index] = parsed;
    }
}

fn parse_option_array4(value: &str, target: &mut Option<[f32; 4]>, index: usize) {
    if let Ok(parsed) = value.parse() {
        let values = target.get_or_insert([1.0, 0.0, 0.0, 0.0]);
        values[index] = parsed;
    }
}

fn parse_into<T: std::str::FromStr>(value: &str, target: &mut T, parsed_flag: &mut bool) {
    if let Ok(parsed) = value.parse() {
        *target = parsed;
        *parsed_flag = true;
    }
}
