use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use serde::Serialize;

use super::app_state::{GazeboBridgeStats, HilSourceFrame};

pub struct GazeboBridgeClient {
    endpoint: String,
    stream: Option<TcpStream>,
    read_buffer: String,
    sensor_frames: VecDeque<GazeboSensorFrame>,
    stats: GazeboBridgeStats,
    connected_at: Option<Instant>,
    next_sequence: u64,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct GazeboSensorFrame {
    pub sequence: u64,
    pub sim_time_us: u64,
    pub clock_source: Option<String>,
    pub valid_flags: u32,
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
    pub orientation_quat: Option<[f32; 4]>,
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

impl From<GazeboSensorFrame> for HilSourceFrame {
    fn from(frame: GazeboSensorFrame) -> Self {
        Self {
            sequence: frame.sequence,
            sim_time_us: frame.sim_time_us,
            clock_source: frame.clock_source,
            accel_mps2: frame.accel_mps2,
            gyro_rps: frame.gyro_rps,
            orientation_quat: frame.orientation_quat,
            mag_ut: frame.mag_ut,
            pressure_pa: frame.pressure_pa,
            baro_altitude_m: frame.baro_altitude_m,
            temperature_c: frame.temperature_c,
            lat_deg: frame.lat_deg,
            lon_deg: frame.lon_deg,
            alt_msl_m: frame.alt_msl_m,
            vel_ned_mps: frame.vel_ned_mps,
            sats: frame.sats,
            fix_type: frame.fix_type,
        }
    }
}

impl GazeboBridgeClient {
    pub fn new(endpoint: impl Into<String>) -> Self {
        let endpoint = endpoint.into();

        Self {
            stats: GazeboBridgeStats {
                endpoint: endpoint.clone(),
                ..GazeboBridgeStats::default()
            },
            endpoint,
            stream: None,
            read_buffer: String::new(),
            sensor_frames: VecDeque::new(),
            connected_at: None,
            next_sequence: 1,
        }
    }

    pub fn set_endpoint(&mut self, endpoint: impl Into<String>) {
        let endpoint = endpoint.into();
        self.endpoint = endpoint.clone();
        self.stats.endpoint = endpoint;
    }

    pub fn connect(&mut self) -> Result<(), String> {
        self.stats.connection_attempts = self.stats.connection_attempts.saturating_add(1);
        self.disconnect();

        match TcpStream::connect(&self.endpoint) {
            Ok(stream) => {
                if let Err(error) = stream.set_nonblocking(true) {
                    return Err(format!("failed to configure nonblocking socket: {error}"));
                }

                self.stream = Some(stream);
                self.connected_at = Some(Instant::now());
                self.stats.connected = true;
                self.stats.last_error = None;
                Ok(())
            }
            Err(error) => {
                self.stats.connected = false;
                self.stats.last_error = Some(error.to_string());
                Err(error.to_string())
            }
        }
    }

    pub fn disconnect(&mut self) {
        self.stream = None;
        self.connected_at = None;
        self.stats.connected = false;
        self.stats.connected_for_secs = None;
    }

    pub fn stats(&self) -> GazeboBridgeStats {
        self.stats.clone()
    }

    pub fn drain_sensor_frames(&mut self) -> Vec<GazeboSensorFrame> {
        self.sensor_frames.drain(..).collect()
    }

    #[cfg(test)]
    pub(crate) fn push_test_sensor_frame(&mut self, frame: GazeboSensorFrame) {
        self.stats.sensor_frames_received = self.stats.sensor_frames_received.saturating_add(1);
        self.stats.last_sensor_sequence = Some(frame.sequence);
        self.stats.last_sensor_time_us = Some(frame.sim_time_us);
        self.stats.last_sensor_clock_source = frame.clock_source.clone();
        self.sensor_frames.push_back(frame);
    }

    pub fn connected_for(&self) -> Option<Duration> {
        self.connected_at.map(|instant| instant.elapsed())
    }

    pub fn poll(&mut self) {
        let mut buffer = [0_u8; 1024];
        let mut should_disconnect = false;
        loop {
            let read_result = {
                let Some(stream) = self.stream.as_mut() else {
                    return;
                };
                stream.read(&mut buffer)
            };

            match read_result {
                Ok(0) => {
                    self.stats.last_error = Some("bridge closed the connection".to_string());
                    should_disconnect = true;
                    break;
                }
                Ok(bytes_read) => {
                    let chunk = String::from_utf8_lossy(&buffer[..bytes_read]);
                    self.read_buffer.push_str(&chunk);
                    while let Some(newline) = self.read_buffer.find('\n') {
                        let line = self.read_buffer[..newline].trim().to_string();
                        self.read_buffer.drain(..=newline);
                        self.handle_line(&line);
                    }
                }
                Err(error) if error.kind() == io::ErrorKind::WouldBlock => {
                    break;
                }
                Err(error) => {
                    self.stats.last_error = Some(error.to_string());
                    should_disconnect = true;
                    break;
                }
            }
        }

        if should_disconnect {
            self.disconnect();
        }
    }

    pub fn send_test_actuator_command(&mut self, motor_speed: f32) -> Result<(), String> {
        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.saturating_add(1);

        let payload = format!(
            "ACTUATOR seq={} m0={:.3} m1={:.3} m2={:.3} m3={:.3}\n",
            sequence, motor_speed, motor_speed, motor_speed, motor_speed
        );

        self.write_raw(payload.as_bytes())?;
        self.stats.actuator_frames_sent = self.stats.actuator_frames_sent.saturating_add(1);
        self.stats.last_error = None;
        Ok(())
    }

    pub fn send_hil_actuator_command(
        &mut self,
        sequence: u64,
        sim_time_us: u64,
        motor_cmd: [u16; 4],
    ) -> Result<(), String> {
        let motors = normalized_motor_values(motor_cmd);
        let payload = format!(
            "ACTUATOR seq={} sim_time_us={} m0={:.3} m1={:.3} m2={:.3} m3={:.3}\n",
            sequence, sim_time_us, motors[0], motors[1], motors[2], motors[3]
        );

        self.write_raw(payload.as_bytes())?;
        self.stats.actuator_frames_sent = self.stats.actuator_frames_sent.saturating_add(1);
        self.stats.last_error = None;
        Ok(())
    }

    fn write_raw(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut should_disconnect = false;
        let result = {
            let Some(stream) = self.stream.as_mut() else {
                return Err("not connected".to_string());
            };

            match stream.write_all(bytes) {
                Ok(()) => Ok(()),
                Err(error) => {
                    self.stats.last_error = Some(error.to_string());
                    should_disconnect = true;
                    Err(error.to_string())
                }
            }
        };

        if should_disconnect {
            self.disconnect();
        }

        result
    }

    fn handle_line(&mut self, line: &str) {
        if !line.starts_with("SENSOR") {
            return;
        }

        self.stats.sensor_frames_received = self.stats.sensor_frames_received.saturating_add(1);

        let mut sensor_frame = GazeboSensorFrame {
            sequence: 0,
            sim_time_us: 0,
            clock_source: None,
            valid_flags: 0,
            accel_mps2: [0.0, 0.0, -9.81],
            gyro_rps: [0.0, 0.0, 0.0],
            orientation_quat: None,
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
                "seq" => {
                    let Ok(sequence) = value.parse::<u64>() else {
                        continue;
                    };
                    self.stats.last_sensor_sequence = Some(sequence);
                    sensor_frame.sequence = sequence;
                    has_sequence = true;
                }
                "sim_time_us" => {
                    let Ok(sim_time_us) = value.parse::<u64>() else {
                        continue;
                    };
                    self.stats.last_sensor_time_us = Some(sim_time_us);
                    sensor_frame.sim_time_us = sim_time_us;
                    has_sim_time = true;
                }
                "clock" => {
                    let clock_source = value.to_string();
                    self.stats.last_sensor_clock_source = Some(clock_source.clone());
                    sensor_frame.clock_source = Some(clock_source);
                }
                "valid" | "valid_flags" => parse_u32_into(value, &mut sensor_frame.valid_flags),
                "ax" => parse_f32_into(value, &mut sensor_frame.accel_mps2[0]),
                "ay" => parse_f32_into(value, &mut sensor_frame.accel_mps2[1]),
                "az" => parse_f32_into(value, &mut sensor_frame.accel_mps2[2]),
                "gx" => parse_f32_into(value, &mut sensor_frame.gyro_rps[0]),
                "gy" => parse_f32_into(value, &mut sensor_frame.gyro_rps[1]),
                "gz" => parse_f32_into(value, &mut sensor_frame.gyro_rps[2]),
                "qw" => parse_quat_component(value, &mut sensor_frame.orientation_quat, 0),
                "qx" => parse_quat_component(value, &mut sensor_frame.orientation_quat, 1),
                "qy" => parse_quat_component(value, &mut sensor_frame.orientation_quat, 2),
                "qz" => parse_quat_component(value, &mut sensor_frame.orientation_quat, 3),
                "mx" => parse_f32_into(value, &mut sensor_frame.mag_ut[0]),
                "my" => parse_f32_into(value, &mut sensor_frame.mag_ut[1]),
                "mz" => parse_f32_into(value, &mut sensor_frame.mag_ut[2]),
                "pressure_pa" => parse_f32_into(value, &mut sensor_frame.pressure_pa),
                "baro_alt_m" => parse_f32_into(value, &mut sensor_frame.baro_altitude_m),
                "temp_c" => parse_f32_into(value, &mut sensor_frame.temperature_c),
                "lat_deg" => parse_f64_into(value, &mut sensor_frame.lat_deg),
                "lon_deg" => parse_f64_into(value, &mut sensor_frame.lon_deg),
                "alt_msl_m" => parse_f32_into(value, &mut sensor_frame.alt_msl_m),
                "vn" => parse_f32_into(value, &mut sensor_frame.vel_ned_mps[0]),
                "ve" => parse_f32_into(value, &mut sensor_frame.vel_ned_mps[1]),
                "vd" => parse_f32_into(value, &mut sensor_frame.vel_ned_mps[2]),
                "sats" => parse_u8_into(value, &mut sensor_frame.sats),
                "fix" => parse_u8_into(value, &mut sensor_frame.fix_type),
                "battery_v" => parse_f32_into(value, &mut sensor_frame.battery_voltage_v),
                "rssi_dbm" => parse_i16_into(value, &mut sensor_frame.rssi_dbm),
                "snr_db_x100" => parse_i16_into(value, &mut sensor_frame.snr_db_x100),
                "loss_pct_x100" => parse_u16_into(value, &mut sensor_frame.loss_pct_x100),
                _ => {}
            }
        }

        if has_sequence && has_sim_time {
            self.sensor_frames.push_back(sensor_frame);
        }
    }
}

fn parse_f32_into(value: &str, target: &mut f32) {
    if let Ok(parsed) = value.parse::<f32>() {
        *target = parsed;
    }
}

fn parse_f64_into(value: &str, target: &mut f64) {
    if let Ok(parsed) = value.parse::<f64>() {
        *target = parsed;
    }
}

fn parse_u8_into(value: &str, target: &mut u8) {
    if let Ok(parsed) = value.parse::<u8>() {
        *target = parsed;
    }
}

fn parse_u16_into(value: &str, target: &mut u16) {
    if let Ok(parsed) = value.parse::<u16>() {
        *target = parsed;
    }
}

fn parse_u32_into(value: &str, target: &mut u32) {
    if let Ok(parsed) = value.parse::<u32>() {
        *target = parsed;
    }
}

fn parse_i16_into(value: &str, target: &mut i16) {
    if let Ok(parsed) = value.parse::<i16>() {
        *target = parsed;
    }
}

fn parse_quat_component(value: &str, target: &mut Option<[f32; 4]>, index: usize) {
    if let Ok(parsed) = value.parse::<f32>() {
        let quat = target.get_or_insert([1.0, 0.0, 0.0, 0.0]);
        quat[index] = parsed;
    }
}

fn normalized_motor_values(motor_cmd: [u16; 4]) -> [f32; 4] {
    motor_cmd.map(|motor| (motor as f32 / 65_535.0).clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_gazebo_sensor_frame_for_forwarding() {
        let mut client = GazeboBridgeClient::new("127.0.0.1:9000");

        client.handle_line(
            "SENSOR seq=42 sim_time_us=840000 clock=gazebo ax=1.0 ay=2.0 az=3.0 gx=0.1 gy=0.2 gz=0.3",
        );

        assert_eq!(client.stats.last_sensor_sequence, Some(42));
        assert_eq!(client.stats.last_sensor_time_us, Some(840_000));
        assert_eq!(
            client.stats.last_sensor_clock_source.as_deref(),
            Some("gazebo")
        );
        assert_eq!(
            client.drain_sensor_frames(),
            vec![GazeboSensorFrame {
                sequence: 42,
                sim_time_us: 840_000,
                clock_source: Some("gazebo".to_string()),
                valid_flags: 0,
                accel_mps2: [1.0, 2.0, 3.0],
                gyro_rps: [0.1, 0.2, 0.3],
                orientation_quat: None,
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
            }]
        );
    }

    #[test]
    fn parses_navigation_fields_from_gazebo_sensor_frame() {
        let mut client = GazeboBridgeClient::new("127.0.0.1:9000");

        client.handle_line(
            "SENSOR seq=7 sim_time_us=100000 clock=gazebo valid=31 ax=0 ay=0 az=-9.81 gx=0 gy=0 gz=0 mx=21.1 my=-4.2 mz=42.8 pressure_pa=99123.5 baro_alt_m=182.25 temp_c=23.5 lat_deg=30.2672345 lon_deg=-97.7431567 alt_msl_m=181.75 vn=1.25 ve=-2.5 vd=0.75 sats=14 fix=3 battery_v=12.3 rssi_dbm=0 snr_db_x100=0 loss_pct_x100=0",
        );

        let frames = client.drain_sensor_frames();
        assert_eq!(frames.len(), 1);
        let frame = &frames[0];
        assert_eq!(frame.valid_flags, 31);
        assert_eq!(frame.mag_ut, [21.1, -4.2, 42.8]);
        assert_eq!(frame.pressure_pa, 99_123.5);
        assert_eq!(frame.baro_altitude_m, 182.25);
        assert_eq!(frame.temperature_c, 23.5);
        assert_eq!(frame.lat_deg, 30.2672345);
        assert_eq!(frame.lon_deg, -97.7431567);
        assert_eq!(frame.alt_msl_m, 181.75);
        assert_eq!(frame.vel_ned_mps, [1.25, -2.5, 0.75]);
        assert_eq!(frame.sats, 14);
        assert_eq!(frame.fix_type, 3);
        assert_eq!(frame.battery_voltage_v, 12.3);
    }

    #[test]
    fn normalizes_hil_motor_commands_for_gazebo_bridge_protocol() {
        let motors = normalized_motor_values([0, 32768, 65535, 16384]);

        assert_eq!(motors[0], 0.0);
        assert!((motors[1] - 0.5000076).abs() < 0.00001);
        assert_eq!(motors[2], 1.0);
        assert!((motors[3] - 0.2500038).abs() < 0.00001);
    }
}
