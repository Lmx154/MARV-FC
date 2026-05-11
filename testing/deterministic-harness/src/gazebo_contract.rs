use std::collections::BTreeMap;

use common::protocol::hilink::{HilSensorFrame, SimStamp};

#[derive(Clone, Debug, PartialEq)]
pub struct GazeboBridgeConfig {
    pub clock_topic: String,
    pub imu_topic: String,
    pub magnetometer_topic: String,
    pub air_pressure_topic: String,
    pub navsat_topic: String,
    pub actuator_topic: String,
    pub max_rotor_velocity_rad_s: f64,
    pub motor_direction_mode: String,
    pub motor_directions: [f64; 4],
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
            actuator_topic: required_string(&values, "actuators.topic")?,
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
}

#[derive(Clone, Debug, PartialEq)]
pub struct GazeboSensorLine {
    pub sequence: u64,
    pub sim_time_us: u64,
    pub clock_source: Option<String>,
    pub valid_flags: u32,
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
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

fn parse_value<T: std::str::FromStr>(value: &str, target: &mut T) {
    if let Ok(parsed) = value.parse() {
        *target = parsed;
    }
}

fn parse_into<T: std::str::FromStr>(value: &str, target: &mut T, parsed_flag: &mut bool) {
    if let Ok(parsed) = value.parse() {
        *target = parsed;
        *parsed_flag = true;
    }
}
