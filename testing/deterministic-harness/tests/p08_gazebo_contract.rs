use common::protocol::hilink::valid;
use deterministic_harness::{GazeboBridgeConfig, GazeboSensorLine, MotorGeometry};

const BRIDGE_CONFIG: &str =
    include_str!("../../../telemetry-app/gazebo_bridge/config/bridge_config");

#[test]
fn p08_gazebo_contract_sensor_line_maps_to_exact_marv_hil_frame_fields() {
    let line = "SENSOR seq=42 sim_time_us=840000 clock=gazebo valid=127 ax=1.0 ay=-2.0 az=3.5 gx=0.1 gy=-0.2 gz=0.3 mx=20.5 my=-3.5 mz=44.25 pressure_pa=100123.25 baro_alt_m=101.5 temp_c=18.75 lat_deg=26.310942 lon_deg=-98.174728 alt_msl_m=99.5 vn=4.5 ve=-5.5 vd=6.5 sats=13 fix=3 battery_v=15.2 rssi_dbm=-42 snr_db_x100=725 loss_pct_x100=12";

    let gazebo = GazeboSensorLine::parse(line).expect("sensor line should parse");
    let hil = gazebo.to_hil_sensor_frame();

    assert_eq!(hil.stamp.sim_tick, 42);
    assert_eq!(hil.stamp.sim_time_us, 840_000);
    assert_eq!(hil.valid_flags, 127);
    assert_eq!(hil.accel_mps2, [1.0, -2.0, 3.5]);
    assert_eq!(hil.gyro_rps, [0.1, -0.2, 0.3]);
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
fn p08_gazebo_contract_actuator_scaling_maps_normalized_outputs_to_motor_speeds() {
    let config = bridge_config();

    let speeds = config.motor_speeds_rad_s([0.0, 0.25, 0.5, 1.0]);

    assert_eq!(speeds, [0.0, 250.0, 500.0, 1000.0]);
}

#[test]
fn p08_gazebo_contract_motor_speed_limits_cover_zero_hover_and_max() {
    let config = bridge_config();
    let hover = MotorGeometry::default().hover_throttle;

    assert_eq!(config.motor_speed_rad_s(0, 0.0), 0.0);
    assert_eq!(config.motor_speed_rad_s(0, hover), 500.0);
    assert_eq!(config.motor_speed_rad_s(0, 1.0), 1000.0);
    assert_eq!(config.motor_speed_rad_s(0, -0.5), 0.0);
    assert_eq!(config.motor_speed_rad_s(0, 1.5), 1000.0);
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
    assert_eq!(config.actuator_topic, "/marv_f450/command/motor_speed");
    assert_eq!(config.nominal_battery_voltage_v, 12.3);
    assert!(!config.synthetic_sensors);
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
