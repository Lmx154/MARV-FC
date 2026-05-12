use common::{control::mixing::TorqueCommand, protocol::hilink::valid};
use deterministic_harness::{
    GazeboActuatorExpectation, GazeboAirframeConfig, GazeboBodyFrame, GazeboBridgeConfig,
    GazeboRotorSpin, GazeboSensorLine, GazeboSimControlAction, GazeboTruthOrigin,
    format_gazebo_sim_control_line, gazebo_g0_actuator_truth_cases,
};

const BRIDGE_CONFIG: &str =
    include_str!("../../../telemetry-app/gazebo_bridge/config/bridge_config");
const AIRFRAME_CONFIG: &str = include_str!("../config/airframes/f450_xing2_2809_1045_4s_v0.cfg");

#[test]
fn p08_gazebo_contract_sensor_line_maps_to_exact_marv_hil_frame_fields() {
    let line = "SENSOR seq=42 sim_time_us=840000 clock=gazebo valid=127 ax=1.0 ay=-2.0 az=3.5 gx=0.1 gy=-0.2 gz=0.3 qw=0.996 qx=0.087 qy=0.0 qz=0.0 roll=0.1745329 pitch=0.0 yaw=0.0 pn=1.25 pe=-2.5 pd=0.75 mx=20.5 my=-3.5 mz=44.25 pressure_pa=100123.25 baro_alt_m=101.5 temp_c=18.75 lat_deg=26.310942 lon_deg=-98.174728 alt_msl_m=99.5 vn=4.5 ve=-5.5 vd=6.5 sats=13 fix=3 battery_v=15.2 rssi_dbm=-42 snr_db_x100=725 loss_pct_x100=12";

    let gazebo = GazeboSensorLine::parse(line).expect("sensor line should parse");
    let hil = gazebo.to_hil_sensor_frame();

    assert_eq!(hil.stamp.sim_tick, 42);
    assert_eq!(hil.stamp.sim_time_us, 840_000);
    assert_eq!(hil.valid_flags, 127);
    assert_eq!(hil.accel_mps2, [1.0, -2.0, 3.5]);
    assert_eq!(hil.gyro_rps, [0.1, -0.2, 0.3]);
    assert_eq!(gazebo.orientation_quat, Some([0.996, 0.087, 0.0, 0.0]));
    assert_eq!(gazebo.euler_rad, Some([0.1745329, 0.0, 0.0]));
    assert_eq!(gazebo.position_ned_m, Some([1.25, -2.5, 0.75]));
    assert_eq!(hil.mag_ut, [20.5, -3.5, 44.25]);
    assert_eq!(hil.pressure_pa, 100_123.25);
    assert_eq!(hil.baro_altitude_m, 101.5);
    assert_eq!(hil.temperature_c, 18.75);
    assert_eq!(hil.lat_deg, 26.310942);
    assert_eq!(hil.lon_deg, -98.174728);
    assert_eq!(hil.alt_msl_m, 99.5);
    assert_eq!(hil.vel_ned_mps, [4.5, -5.5, 6.5]);
    assert_eq!(hil.sats, 13);
    assert_eq!(hil.fix_type, 3);
    assert_eq!(hil.battery_voltage_v, 15.2);
    assert_eq!(hil.rssi_dbm, -42);
    assert_eq!(hil.snr_db_x100, 725);
    assert_eq!(hil.loss_pct_x100, 12);
}

#[test]
fn p08_gazebo_contract_truth_adapter_builds_control_estimate_from_sensor_line() {
    let origin = GazeboSensorLine::parse(
        "SENSOR seq=1 sim_time_us=20000 valid=31 pn=10 pe=20 pd=-5 lat_deg=26.310942 lon_deg=-98.174728 alt_msl_m=100.0 vn=0 ve=0 vd=0 qw=1 qx=0 qy=0 qz=0",
    )
    .expect("origin line should parse");
    let next = GazeboSensorLine::parse(
        "SENSOR seq=2 sim_time_us=40000 valid=31 pn=11.5 pe=18 pd=-6.5 lat_deg=26.311042 lon_deg=-98.174628 alt_msl_m=101.5 vn=1.0 ve=-2.0 vd=0.5 gx=0.1 gy=-0.2 gz=0.3 qw=0.996 qx=0.087 qy=0 qz=0",
    )
    .expect("next line should parse");
    let origin = GazeboTruthOrigin::from_frame(&origin).expect("origin should be finite");
    let estimate = next
        .to_truth_estimate(origin)
        .expect("truth estimate should convert");
    let imu = next.to_truth_imu();

    assert!(estimate.valid);
    assert_eq!(estimate.position_ned_m, [1.5, -2.0, -1.5]);
    assert_eq!(estimate.velocity_ned_mps, [1.0, -2.0, 0.5]);
    assert_eq!(estimate.quaternion, [0.996, 0.087, 0.0, 0.0]);
    assert_eq!(imu.gyro_rps, [0.1, -0.2, 0.3]);
}

#[test]
fn p08_gazebo_contract_estimator_adapter_prefers_pose_ned_for_gazebo_position() {
    let origin = GazeboSensorLine::parse(
        "SENSOR seq=1 sim_time_us=20000 valid=31 pn=10 pe=20 pd=-5 lat_deg=26.310000 lon_deg=-98.170000 alt_msl_m=100.0 baro_alt_m=101.0 vn=0 ve=0 vd=0 ax=0 ay=0 az=-9.81 gx=0 gy=0 gz=0 mx=20 my=0 mz=40",
    )
    .expect("origin line should parse");
    let next = GazeboSensorLine::parse(
        "SENSOR seq=2 sim_time_us=40000 valid=31 pn=999 pe=999 pd=999 lat_deg=26.310010 lon_deg=-98.169990 alt_msl_m=101.5 baro_alt_m=102.25 vn=1.0 ve=-2.0 vd=0.5 ax=1 ay=2 az=-9.80665 gx=0.1 gy=-0.2 gz=0.3 mx=21 my=1 mz=41",
    )
    .expect("next line should parse");
    let origin = GazeboTruthOrigin::from_frame(&origin).expect("origin should be finite");
    let sensor = next
        .to_estimator_sensor_frame(origin)
        .expect("estimator sensor frame should convert");

    assert_eq!(sensor.timestamp_us, 40_000);
    assert_eq!(sensor.accel_mps2[0], 1.0);
    assert_eq!(sensor.accel_mps2[1], 2.0);
    assert_near_f64(sensor.accel_mps2[2], 0.0);
    assert_vec3_near_f64(
        sensor.gravity_body_mps2.expect("gravity"),
        [1.0, 2.0, -9.80665],
    );
    assert_near_f64(sensor.gyro_rps[0], 0.1);
    assert_near_f64(sensor.gyro_rps[1], -0.2);
    assert_near_f64(sensor.gyro_rps[2], 0.3);
    assert_eq!(sensor.mag_body_ut, Some([21.0, 1.0, 41.0]));
    assert_eq!(sensor.gps_position_ned_m, Some([989.0, 979.0, 1004.0]));
    assert_eq!(sensor.gps_velocity_ned_mps, Some([1.0, -2.0, 0.5]));
    assert_eq!(sensor.baro_down_m, Some(-1.25));
}

#[test]
fn p08_gazebo_contract_estimator_adapter_removes_tilted_gravity() {
    let origin = GazeboSensorLine::parse(
        "SENSOR seq=1 sim_time_us=20000 valid=31 lat_deg=26.310000 lon_deg=-98.170000 alt_msl_m=100.0 baro_alt_m=101.0 ax=0 ay=0 az=-9.80665 gx=0 gy=0 gz=0 mx=20 my=0 mz=40",
    )
    .expect("origin line should parse");
    let tilted = GazeboSensorLine::parse(
        "SENSOR seq=2 sim_time_us=40000 valid=31 lat_deg=26.310000 lon_deg=-98.170000 alt_msl_m=100.0 baro_alt_m=101.0 ax=0 ay=-1.7029068 az=-9.657665 gx=0 gy=0 gz=0 qw=0.9961947 qx=0.0871557 qy=0 qz=0 mx=20 my=0 mz=40",
    )
    .expect("tilted line should parse");
    let origin = GazeboTruthOrigin::from_frame(&origin).expect("origin should be finite");
    let sensor = tilted
        .to_estimator_sensor_frame(origin)
        .expect("estimator sensor frame should convert");

    assert_near_f64(sensor.accel_mps2[0], 0.0);
    assert_near_f64(sensor.accel_mps2[1], 0.0);
    assert_near_f64(sensor.accel_mps2[2], 0.0);
    assert_vec3_near_f64(
        sensor.gravity_body_mps2.expect("gravity"),
        [0.0, -1.7029068, -9.657665],
    );
}

#[test]
fn p08_gazebo_contract_runtime_mixer_uses_measured_g0_motor_basis() {
    let airframe = airframe_config();
    let hover = airframe.hover_motor_command();

    assert_motors_near(
        airframe
            .runtime_control_motors(TorqueCommand::new(0.1, 0.0, 0.0, hover))
            .commands,
        [hover - 0.1, hover + 0.1, hover + 0.1, hover - 0.1],
    );
    assert_motors_near(
        airframe
            .runtime_control_motors(TorqueCommand::new(0.0, 0.1, 0.0, hover))
            .commands,
        [hover + 0.1, hover - 0.1, hover + 0.1, hover - 0.1],
    );
    assert_motors_near(
        airframe
            .runtime_control_motors(TorqueCommand::new(0.0, 0.0, 0.1, hover))
            .commands,
        [hover + 0.1, hover + 0.1, hover - 0.1, hover - 0.1],
    );
}

#[test]
fn p08_gazebo_contract_actuator_scaling_maps_normalized_outputs_to_motor_speeds() {
    let config = bridge_config();

    let speeds = config.motor_speeds_rad_s([0.0, 0.25, 0.5, 1.0]);

    assert_eq!(speeds, [0.0, 275.0, 550.0, 1100.0]);
}

#[test]
fn p08_gazebo_contract_motor_speed_limits_cover_zero_hover_and_max() {
    let config = bridge_config();
    let airframe = airframe_config();
    let hover = airframe.hover_motor_command();

    assert_eq!(config.motor_speed_rad_s(0, 0.0), 0.0);
    assert_near_f64(config.motor_speed_rad_s(0, hover), 630.0);
    assert_eq!(config.motor_speed_rad_s(0, 1.0), 1100.0);
    assert_eq!(config.motor_speed_rad_s(0, -0.5), 0.0);
    assert_eq!(config.motor_speed_rad_s(0, 1.5), 1100.0);
}

#[test]
fn p08_gazebo_contract_bridge_max_rotor_velocity_matches_airframe_profile() {
    let config = bridge_config();
    let airframe = airframe_config();

    assert_near_f64(
        config.max_rotor_velocity_rad_s,
        f64::from(airframe.max_rotor_velocity_rad_s),
    );
}

#[test]
fn p08_gazebo_contract_motor_direction_is_explicitly_delegated_to_gazebo_model() {
    let config = bridge_config();

    assert_eq!(config.motor_direction_mode, "gazebo_model");
    assert_eq!(config.motor_directions, [1.0, 1.0, 1.0, 1.0]);
}

#[test]
fn p08_gazebo_contract_topic_config_matches_marv_gazebo_model() {
    let config = bridge_config();

    assert_eq!(config.clock_topic, "/world/marv_field/clock");
    assert_eq!(
        config.imu_topic,
        "/world/marv_field/model/marv_f450/link/base_link/sensor/imu_sensor/imu"
    );
    assert_eq!(
        config.magnetometer_topic,
        "/world/marv_field/model/marv_f450/link/base_link/sensor/magnetometer_sensor/magnetometer"
    );
    assert_eq!(
        config.air_pressure_topic,
        "/world/marv_field/model/marv_f450/link/base_link/sensor/air_pressure_sensor/air_pressure"
    );
    assert_eq!(
        config.navsat_topic,
        "/world/marv_field/model/marv_f450/link/base_link/sensor/navsat_sensor/navsat"
    );
    assert_eq!(config.pose_topic, "/world/marv_field/pose/info");
    assert_eq!(config.truth_model_name, "marv_f450");
    assert_eq!(config.actuator_topic, "/marv_f450/command/motor_speed");
    assert_eq!(config.world_control_service, "/world/marv_field/control");
    assert_eq!(config.world_control_timeout_ms, 1000);
    assert_eq!(config.nominal_battery_voltage_v, 12.3);
    assert!(!config.synthetic_sensors);
}

#[test]
fn p08_gazebo_contract_sim_control_format_matches_bridge_protocol() {
    assert_eq!(
        format_gazebo_sim_control_line(7, GazeboSimControlAction::Reset),
        "SIM_CONTROL seq=7 action=reset\n"
    );
    assert_eq!(
        format_gazebo_sim_control_line(8, GazeboSimControlAction::Pause),
        "SIM_CONTROL seq=8 action=pause\n"
    );
    assert_eq!(
        format_gazebo_sim_control_line(9, GazeboSimControlAction::Play),
        "SIM_CONTROL seq=9 action=play\n"
    );
}

#[test]
fn p08_gazebo_contract_airframe_profile_matches_f450_handoff() {
    let airframe = airframe_config();

    assert_eq!(airframe.name, "f450_xing2_2809_1045_4s_v0");
    assert_eq!(airframe.body_frame, GazeboBodyFrame::Flu);
    assert_eq!(airframe.runtime_gyro_frame, GazeboBodyFrame::BridgeFrd);
    assert_near(airframe.mass_kg, 1.338);
    assert_near(airframe.gravity_mps2, 9.80665);
    assert_eq!(airframe.center_of_mass_m, [0.0, 0.0, -0.013]);
    assert_eq!(airframe.inertia_kg_m2, [0.010, 0.011, 0.019, 0.0, 0.0, 0.0]);
    assert_near(airframe.motor_xy_magnitude_m, 0.160867);
    assert_near(airframe.total_hover_thrust_n, 13.121);
    assert_near(airframe.per_motor_hover_thrust_n, 3.280);
    assert_near(airframe.max_thrust_per_motor_n, 10.0);
    assert_near(airframe.yaw_moment_per_thrust_m, 0.016);
    assert_near(airframe.max_rotor_velocity_rad_s, 1100.0);
    assert_near(airframe.motor_constant_n_per_rad_s2, 8.26e-6);
    assert_near(airframe.estimated_hover_omega_rad_s, 630.0);
    assert_near(airframe.estimated_hover_rpm, 6016.0);

    assert_eq!(airframe.motors[0].output_index, 0);
    assert_eq!(airframe.motors[0].name, "rear_right");
    assert_eq!(
        airframe.motors[0].position_flu_m,
        [-0.160867, -0.160867, 0.0]
    );
    assert_eq!(
        airframe.motors[0].spin_direction,
        GazeboRotorSpin::Clockwise
    );
    assert_eq!(airframe.motors[0].runtime_signs, Some([-1.0, 1.0, 1.0]));

    assert_eq!(airframe.motors[3].output_index, 3);
    assert_eq!(airframe.motors[3].name, "front_left");
    assert_eq!(airframe.motors[3].position_flu_m, [0.160867, 0.160867, 0.0]);
    assert_eq!(
        airframe.motors[3].spin_direction,
        GazeboRotorSpin::Clockwise
    );
    assert_eq!(airframe.motors[3].runtime_signs, Some([-1.0, -1.0, -1.0]));
}

#[test]
fn p08_gazebo_contract_airframe_profile_derives_runtime_oracle() {
    let airframe = airframe_config();
    let cases = gazebo_g0_actuator_truth_cases(&airframe);

    assert_eq!(cases.len(), 12);
    assert_eq!(cases[1].name, "hover");
    assert_motors_near(
        cases[1].motors,
        [0.572_727, 0.572_727, 0.572_727, 0.572_727],
    );
    assert_eq!(cases[6].name, "roll_positive");
    assert_eq!(cases[6].motors, [0.0, 1.0, 1.0, 0.0]);
    assert_eq!(cases[8].name, "pitch_positive");
    assert_eq!(cases[8].motors, [1.0, 0.0, 1.0, 0.0]);
    assert_eq!(cases[10].name, "yaw_positive");
    assert_eq!(cases[10].motors, [1.0, 1.0, 0.0, 0.0]);
    assert_eq!(
        cases[2].expectation,
        GazeboActuatorExpectation::MotorBump {
            motor: 1,
            roll_positive: false,
            pitch_positive: true,
            yaw_positive: true,
        }
    );
    assert_eq!(
        cases[5].expectation,
        GazeboActuatorExpectation::MotorBump {
            motor: 4,
            roll_positive: false,
            pitch_positive: false,
            yaw_positive: false,
        }
    );
}

#[test]
fn p08_gazebo_contract_valid_flag_bits_match_marv_hil_sensor_groups() {
    let gazebo = GazeboSensorLine::parse(
        "SENSOR seq=1 sim_time_us=20000 valid=31 ax=0 ay=0 az=-9.81 gx=0 gy=0 gz=0 mx=1 my=2 mz=3 pressure_pa=101325 baro_alt_m=0 temp_c=15 lat_deg=26 lon_deg=-98 alt_msl_m=100 vn=0 ve=0 vd=0 sats=12 fix=3",
    )
    .expect("sensor line should parse");
    let hil = gazebo.to_hil_sensor_frame();

    assert_eq!(
        hil.valid_flags,
        valid::ACCEL | valid::GYRO | valid::MAG | valid::BARO | valid::GPS
    );
}

fn bridge_config() -> GazeboBridgeConfig {
    GazeboBridgeConfig::parse(BRIDGE_CONFIG).expect("checked-in bridge config should parse")
}

fn airframe_config() -> GazeboAirframeConfig {
    let airframe =
        GazeboAirframeConfig::parse(AIRFRAME_CONFIG).expect("checked-in airframe config parses");
    airframe
        .validate()
        .expect("checked-in airframe config is physically self-consistent");
    airframe
}

fn assert_near(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() < 0.000_1,
        "expected {actual} to be near {expected}"
    );
}

fn assert_near_f64(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 0.000_1,
        "expected {actual} to be near {expected}"
    );
}

fn assert_vec3_near_f64(actual: [f64; 3], expected: [f64; 3]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert_near_f64(actual, expected);
    }
}

fn assert_motors_near(actual: [f32; 4], expected: [f32; 4]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert_near(actual, expected);
    }
}
