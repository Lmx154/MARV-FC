#![allow(dead_code)]

use std::{
    io::{BufRead, BufReader, Write},
    net::TcpStream,
    time::{Duration, Instant},
};

use deterministic_harness::{
    GazeboSensorLine, GazeboSimControlAction, format_gazebo_actuator_line,
    format_gazebo_sim_control_line,
};

const LIVE_CONNECT_TIMEOUT: Duration = Duration::from_secs(2);
const LIVE_READ_TIMEOUT: Duration = Duration::from_millis(250);
const LIVE_ACK_TIMEOUT: Duration = Duration::from_secs(2);
const LIVE_SENSOR_TIMEOUT: Duration = Duration::from_secs(4);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Gate {
    C0,
    G0,
    G1,
    G2,
    G2Point5,
    G3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CommandKind {
    Hold,
    LocalPosition,
    LocalTrajectory,
    ActuatorMotors,
    SimReset,
    SimPlay,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScheduledCommand {
    pub at_frame: usize,
    pub kind: CommandKind,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CommandSchedule {
    pub commands: Vec<ScheduledCommand>,
}

impl CommandSchedule {
    pub fn new(commands: Vec<ScheduledCommand>) -> Self {
        assert!(!commands.is_empty(), "scenario schedules must be explicit");
        assert!(
            commands
                .windows(2)
                .all(|pair| pair[0].at_frame <= pair[1].at_frame),
            "scenario commands must be sorted by frame"
        );
        Self { commands }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AcceptanceThresholds {
    pub max_cross_track_error_m: f32,
    pub max_altitude_error_m: f32,
    pub max_roll_pitch_rad: f32,
    pub max_speed_error_mps: f32,
    pub max_clamp_ratio: f32,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioSpec {
    pub id: &'static str,
    pub gate: Gate,
    pub description: &'static str,
    pub reset_required: bool,
    pub command_schedule: CommandSchedule,
    pub thresholds: AcceptanceThresholds,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FailureLayer {
    Reset,
    Estimator,
    Guidance,
    Control,
    Actuator,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioReport {
    pub id: String,
    pub pass: bool,
    pub reset_clean: bool,
    pub estimator_agrees: bool,
    pub max_cross_track_error_m: f32,
    pub max_altitude_error_m: f32,
    pub max_roll_pitch_rad: f32,
    pub max_speed_error_mps: f32,
    pub clamp_ratio: f32,
    pub first_clamp_source: Option<String>,
    pub failure_layer: Option<FailureLayer>,
}

impl ScenarioReport {
    pub fn from_metrics(spec: &ScenarioSpec, metrics: ScenarioMetrics) -> Self {
        let mut pass = true;
        let mut failure_layer = None;

        if spec.reset_required && !metrics.reset_clean {
            pass = false;
            failure_layer = Some(FailureLayer::Reset);
        } else if !metrics.estimator_agrees {
            pass = false;
            failure_layer = Some(FailureLayer::Estimator);
        } else if metrics.max_cross_track_error_m > spec.thresholds.max_cross_track_error_m {
            pass = false;
            failure_layer = Some(FailureLayer::Guidance);
        } else if metrics.max_altitude_error_m > spec.thresholds.max_altitude_error_m
            || metrics.max_roll_pitch_rad > spec.thresholds.max_roll_pitch_rad
            || metrics.max_speed_error_mps > spec.thresholds.max_speed_error_mps
        {
            pass = false;
            failure_layer = Some(FailureLayer::Control);
        } else if metrics.clamp_ratio > spec.thresholds.max_clamp_ratio {
            pass = false;
            failure_layer = Some(FailureLayer::Actuator);
        }

        Self {
            id: spec.id.to_string(),
            pass,
            reset_clean: metrics.reset_clean,
            estimator_agrees: metrics.estimator_agrees,
            max_cross_track_error_m: metrics.max_cross_track_error_m,
            max_altitude_error_m: metrics.max_altitude_error_m,
            max_roll_pitch_rad: metrics.max_roll_pitch_rad,
            max_speed_error_mps: metrics.max_speed_error_mps,
            clamp_ratio: metrics.clamp_ratio,
            first_clamp_source: metrics.first_clamp_source,
            failure_layer,
        }
    }

    pub fn csv_header() -> &'static str {
        "id,pass,reset_clean,estimator_agrees,max_cross_track_error_m,max_altitude_error_m,max_roll_pitch_rad,max_speed_error_mps,clamp_ratio,first_clamp_source,failure_layer"
    }

    pub fn to_csv_row(&self) -> String {
        format!(
            "{},{},{},{},{:.3},{:.3},{:.3},{:.3},{:.3},{},{}",
            self.id,
            self.pass,
            self.reset_clean,
            self.estimator_agrees,
            self.max_cross_track_error_m,
            self.max_altitude_error_m,
            self.max_roll_pitch_rad,
            self.max_speed_error_mps,
            self.clamp_ratio,
            self.first_clamp_source.as_deref().unwrap_or(""),
            self.failure_layer.map(|layer| layer.as_str()).unwrap_or("")
        )
    }

    pub fn to_json_line(&self) -> String {
        format!(
            "{{\"id\":\"{}\",\"pass\":{},\"reset_clean\":{},\"estimator_agrees\":{},\"max_cross_track_error_m\":{:.3},\"max_altitude_error_m\":{:.3},\"max_roll_pitch_rad\":{:.3},\"max_speed_error_mps\":{:.3},\"clamp_ratio\":{:.3},\"first_clamp_source\":{},\"failure_layer\":{}}}",
            self.id,
            self.pass,
            self.reset_clean,
            self.estimator_agrees,
            self.max_cross_track_error_m,
            self.max_altitude_error_m,
            self.max_roll_pitch_rad,
            self.max_speed_error_mps,
            self.clamp_ratio,
            quoted_json_option(self.first_clamp_source.as_deref()),
            quoted_json_option(self.failure_layer.map(|layer| layer.as_str()))
        )
    }
}

impl FailureLayer {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Reset => "reset",
            Self::Estimator => "estimator",
            Self::Guidance => "guidance",
            Self::Control => "control",
            Self::Actuator => "actuator",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct ScenarioMetrics {
    pub reset_clean: bool,
    pub estimator_agrees: bool,
    pub max_cross_track_error_m: f32,
    pub max_altitude_error_m: f32,
    pub max_roll_pitch_rad: f32,
    pub max_speed_error_mps: f32,
    pub clamp_ratio: f32,
    pub first_clamp_source: Option<String>,
}

fn quoted_json_option(value: Option<&str>) -> String {
    match value {
        Some(value) => format!("\"{}\"", value.replace('"', "\\\"")),
        None => "null".to_string(),
    }
}

pub struct LiveGazeboBridge {
    writer: TcpStream,
    reader: BufReader<TcpStream>,
    sequence: u64,
}

impl LiveGazeboBridge {
    pub fn connect(endpoint: &str) -> std::io::Result<Self> {
        let stream = TcpStream::connect_timeout(
            &endpoint
                .parse()
                .map_err(|error| std::io::Error::new(std::io::ErrorKind::InvalidInput, error))?,
            LIVE_CONNECT_TIMEOUT,
        )?;
        stream.set_read_timeout(Some(LIVE_READ_TIMEOUT))?;
        stream.set_write_timeout(Some(LIVE_READ_TIMEOUT))?;
        let reader = BufReader::new(stream.try_clone()?);
        Ok(Self {
            writer: stream,
            reader,
            sequence: 1,
        })
    }

    pub fn reset_and_play(&mut self) -> Result<(), String> {
        self.send_sim_control(GazeboSimControlAction::Reset)?;
        self.send_sim_control(GazeboSimControlAction::Play)
    }

    pub fn run_actuator_case(&mut self, motors: [f32; 4]) -> Result<Vec<GazeboSensorLine>, String> {
        self.reset_and_play()?;
        let settle = self.collect_sensor_frames(8, LIVE_SENSOR_TIMEOUT)?;
        let sim_time_us = settle.last().map(|frame| frame.sim_time_us);

        self.send_actuator(motors, sim_time_us)?;
        let measured = self.collect_sensor_frames(12, LIVE_SENSOR_TIMEOUT)?;
        self.send_actuator([0.0; 4], measured.last().map(|frame| frame.sim_time_us))?;

        Ok(measured)
    }

    pub fn send_sim_control(&mut self, action: GazeboSimControlAction) -> Result<(), String> {
        let sequence = self.next_sequence();
        self.send_raw(&format_gazebo_sim_control_line(sequence, action))?;
        self.wait_for_ack(sequence, action)
    }

    pub fn send_actuator(
        &mut self,
        motors: [f32; 4],
        sim_time_us: Option<u64>,
    ) -> Result<(), String> {
        let sequence = self.next_sequence();
        self.send_raw(&format_gazebo_actuator_line(sequence, sim_time_us, motors))
    }

    pub fn collect_sensor_frames(
        &mut self,
        count: usize,
        timeout: Duration,
    ) -> Result<Vec<GazeboSensorLine>, String> {
        let deadline = Instant::now() + timeout;
        let mut frames = Vec::with_capacity(count);
        while frames.len() < count && Instant::now() < deadline {
            if let Some(line) = self.read_line_until(deadline)? {
                if let Some(frame) = GazeboSensorLine::parse(&line) {
                    frames.push(frame);
                }
            }
        }

        if frames.len() == count {
            Ok(frames)
        } else {
            Err(format!(
                "timed out after receiving {} of {count} SENSOR frames",
                frames.len()
            ))
        }
    }

    pub fn next_sensor_frame(&mut self, timeout: Duration) -> Result<GazeboSensorLine, String> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if let Some(line) = self.read_line_until(deadline)? {
                if let Some(frame) = GazeboSensorLine::parse(&line) {
                    return Ok(frame);
                }
            }
        }

        Err("timed out waiting for SENSOR frame".to_string())
    }

    fn send_raw(&mut self, line: &str) -> Result<(), String> {
        self.writer
            .write_all(line.as_bytes())
            .map_err(|error| error.to_string())?;
        self.writer.flush().map_err(|error| error.to_string())
    }

    fn wait_for_ack(
        &mut self,
        sequence: u64,
        action: GazeboSimControlAction,
    ) -> Result<(), String> {
        let deadline = Instant::now() + LIVE_ACK_TIMEOUT;
        while Instant::now() < deadline {
            if let Some(line) = self.read_line_until(deadline)? {
                if let Some(ack) = SimControlAck::parse(&line) {
                    if ack.sequence == sequence && ack.action == action {
                        return if ack.ok {
                            Ok(())
                        } else {
                            Err(format!(
                                "SIM_CONTROL_ACK seq={} action={} failed: {}",
                                ack.sequence,
                                ack.action.as_str(),
                                ack.message
                            ))
                        };
                    }
                }
            }
        }

        Err(format!(
            "timed out waiting for SIM_CONTROL_ACK seq={sequence} action={}",
            action.as_str()
        ))
    }

    fn read_line_until(&mut self, deadline: Instant) -> Result<Option<String>, String> {
        let mut line = String::new();
        loop {
            match self.reader.read_line(&mut line) {
                Ok(0) => return Err("Gazebo bridge closed the TCP connection".to_string()),
                Ok(_) => return Ok(Some(line.trim_end().to_string())),
                Err(error)
                    if matches!(
                        error.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    if Instant::now() >= deadline {
                        return Ok(None);
                    }
                    line.clear();
                    continue;
                }
                Err(error) => return Err(error.to_string()),
            }
        }
    }

    fn next_sequence(&mut self) -> u64 {
        let sequence = self.sequence;
        self.sequence += 1;
        sequence
    }
}

#[derive(Clone, Debug, PartialEq)]
struct SimControlAck {
    sequence: u64,
    action: GazeboSimControlAction,
    ok: bool,
    message: String,
}

impl SimControlAck {
    fn parse(line: &str) -> Option<Self> {
        if !line.starts_with("SIM_CONTROL_ACK") {
            return None;
        }

        let mut sequence = None;
        let mut action = None;
        let mut ok = None;
        let mut message = String::new();

        for token in line.split_whitespace() {
            let Some((key, value)) = token.split_once('=') else {
                continue;
            };
            match key {
                "seq" => sequence = value.parse().ok(),
                "action" => action = parse_sim_control_action(value),
                "ok" => ok = Some(value == "1"),
                "message" => message = value.to_string(),
                _ => {}
            }
        }

        Some(Self {
            sequence: sequence?,
            action: action?,
            ok: ok?,
            message,
        })
    }
}

fn parse_sim_control_action(value: &str) -> Option<GazeboSimControlAction> {
    match value {
        "reset" => Some(GazeboSimControlAction::Reset),
        "pause" => Some(GazeboSimControlAction::Pause),
        "play" => Some(GazeboSimControlAction::Play),
        _ => None,
    }
}
