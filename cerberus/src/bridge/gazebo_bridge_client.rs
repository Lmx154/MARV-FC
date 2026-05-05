use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use super::app_state::GazeboBridgeStats;

pub struct GazeboBridgeClient {
    endpoint: String,
    stream: Option<TcpStream>,
    read_buffer: String,
    sensor_frames: VecDeque<GazeboSensorFrame>,
    stats: GazeboBridgeStats,
    connected_at: Option<Instant>,
    next_sequence: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct GazeboSensorFrame {
    pub sequence: u64,
    pub sim_time_us: u64,
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
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
        let motors = motor_cmd.map(|motor| motor as f32);
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
            accel_mps2: [0.0, 0.0, -9.81],
            gyro_rps: [0.0, 0.0, 0.0],
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
                "clock" => self.stats.last_sensor_clock_source = Some(value.to_string()),
                "ax" => parse_f32_into(value, &mut sensor_frame.accel_mps2[0]),
                "ay" => parse_f32_into(value, &mut sensor_frame.accel_mps2[1]),
                "az" => parse_f32_into(value, &mut sensor_frame.accel_mps2[2]),
                "gx" => parse_f32_into(value, &mut sensor_frame.gyro_rps[0]),
                "gy" => parse_f32_into(value, &mut sensor_frame.gyro_rps[1]),
                "gz" => parse_f32_into(value, &mut sensor_frame.gyro_rps[2]),
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
                accel_mps2: [1.0, 2.0, 3.0],
                gyro_rps: [0.1, 0.2, 0.3],
            }]
        );
    }
}
