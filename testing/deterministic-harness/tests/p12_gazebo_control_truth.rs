use std::env;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use common::control::attitude::AttitudeSetpoint;
use common::control::config::ControlLoopConfig;
use deterministic_harness::{
    ControlPipeline, ControlSetpoint, EstimateSnapshot, GAZEBO_G0_DEFAULT_ENDPOINT,
    GazeboAirframeConfig, GazeboSensorLine, GazeboSimControlAction, GazeboTruthOrigin,
    ImuControlInput, PureControlConfig, format_gazebo_actuator_line,
    format_gazebo_sim_control_line,
};

const AIRFRAME_CONFIG: &str = include_str!("../config/airframes/f450_xing2_2809_1045_4s_v0.cfg");
const DEFAULT_FRAMES: usize = 250;
const DEFAULT_RESET_SETTLE_FRAMES: usize = 50;
const DEFAULT_TIMEOUT_MS: u64 = 8_000;
const DEFAULT_MAX_ATTITUDE_RAD: f32 = 25.0_f32.to_radians();
const DEFAULT_MAX_DOWN_ERROR_M: f32 = 3.0;
const DEFAULT_MAX_CLAMP_RATIO: f32 = 0.25;
const DEFAULT_VERTICAL_STEP_DOWN_M: f32 = -3.0;
const DEFAULT_MIN_VERTICAL_IMPROVEMENT_M: f32 = 0.05;
const DEFAULT_GYRO_DEADBAND_RPS: f32 = 0.02;
const DEFAULT_MAX_HOVER_MOTOR_SPREAD: f32 = 0.08;

#[test]
fn p12_gazebo_control_truth_runtime_mixer_closes_roll_pitch_yaw_signs() {
    let airframe = airframe_config();
    let pipeline = ControlPipeline::new(PureControlConfig {
        loop_config: control_config(&airframe, 0.0),
    });
    let hover = airframe.hover_motor_command();

    let roll = pipeline.step(
        estimate_with_euler_deg(10.0, 0.0, 0.0),
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );
    assert!(roll.torque_command.roll < 0.0);
    let motors = airframe
        .runtime_control_motors(roll.torque_command)
        .commands;
    assert!(motors[0] > hover && motors[3] > hover);
    assert!(motors[1] < hover && motors[2] < hover);

    let pitch = pipeline.step(
        estimate_with_euler_deg(0.0, 10.0, 0.0),
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );
    assert!(pitch.torque_command.pitch < 0.0);
    let motors = airframe
        .runtime_control_motors(pitch.torque_command)
        .commands;
    assert!(motors[1] > hover && motors[3] > hover);
    assert!(motors[0] < hover && motors[2] < hover);

    let yaw = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput {
            accel_mps2: [0.0, 0.0, 0.0],
            gyro_rps: [0.0, 0.0, 1.0],
        },
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );
    assert!(yaw.torque_command.yaw < 0.0);
    let motors = airframe.runtime_control_motors(yaw.torque_command).commands;
    assert!(motors[2] > hover && motors[3] > hover);
    assert!(motors[0] < hover && motors[1] < hover);
}

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_BRIDGE_ADDR if not 127.0.0.1:9000"]
fn p12_gazebo_control_truth_runtime() {
    let settings = RuntimeSettings::from_env();
    let airframe = airframe_config();
    let summary = run_runtime_scenario(
        "origin_hold",
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        &settings,
        &airframe,
    );

    assert!(
        summary.max_attitude_rad <= settings.max_attitude_rad,
        "G1 attitude diverged: summary={summary:?}"
    );
    assert!(
        summary.max_down_error_m <= settings.max_down_error_m,
        "G1 altitude diverged: summary={summary:?}"
    );
    assert!(
        summary.clamp_ratio <= settings.max_clamp_ratio,
        "G1 clamped too often: summary={summary:?}"
    );
    assert!(
        summary.max_motor_spread <= settings.max_hover_motor_spread,
        "G1 hover motor chatter too high: summary={summary:?}"
    );
}

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_BRIDGE_ADDR if not 127.0.0.1:9000"]
fn p12_gazebo_control_truth_vertical_step_runtime() {
    let settings = RuntimeSettings::from_env();
    let airframe = airframe_config();
    let setpoint =
        ControlSetpoint::local_position_ned([0.0, 0.0, settings.vertical_step_down_m], 0.0, true);
    let summary = run_runtime_scenario("vertical_step_up", setpoint, &settings, &airframe);

    assert!(
        summary.max_attitude_rad <= settings.max_attitude_rad,
        "G1 vertical step attitude diverged: summary={summary:?}"
    );
    assert!(
        summary.max_down_error_m <= settings.max_down_error_m,
        "G1 vertical step altitude diverged: summary={summary:?}"
    );
    assert!(
        summary.clamp_ratio <= settings.max_clamp_ratio,
        "G1 vertical step clamped too often: summary={summary:?}"
    );
    assert!(
        summary.final_down_error_m.abs()
            <= summary.initial_down_error_m.abs() - settings.min_vertical_improvement_m,
        "G1 vertical step did not improve enough: summary={summary:?}"
    );
}

#[derive(Clone, Copy, Debug)]
struct RuntimeSummary {
    max_attitude_rad: f32,
    max_down_error_m: f32,
    clamp_ratio: f32,
    initial_down_error_m: f32,
    final_down_error_m: f32,
    mean_throttle: f32,
    mean_motor: f32,
    max_motor: f32,
    mean_motor_spread: f32,
    max_motor_spread: f32,
}

#[derive(Clone, Debug)]
struct RuntimeSettings {
    endpoint: String,
    auto_reset: bool,
    frames: usize,
    reset_settle_frames: usize,
    timeout: Duration,
    max_attitude_rad: f32,
    max_down_error_m: f32,
    max_clamp_ratio: f32,
    vertical_step_down_m: f32,
    min_vertical_improvement_m: f32,
    gyro_deadband_rps: f32,
    max_hover_motor_spread: f32,
}

impl RuntimeSettings {
    fn from_env() -> Self {
        Self {
            endpoint: env::var("MARV_GAZEBO_BRIDGE_ADDR")
                .unwrap_or_else(|_| GAZEBO_G0_DEFAULT_ENDPOINT.to_string()),
            auto_reset: env_bool("MARV_GAZEBO_G1_AUTO_RESET").unwrap_or(false),
            frames: env_usize("MARV_GAZEBO_G1_FRAMES").unwrap_or(DEFAULT_FRAMES),
            reset_settle_frames: env_usize("MARV_GAZEBO_G1_RESET_SETTLE_FRAMES")
                .unwrap_or(DEFAULT_RESET_SETTLE_FRAMES),
            timeout: Duration::from_millis(
                env_u64("MARV_GAZEBO_G1_TIMEOUT_MS").unwrap_or(DEFAULT_TIMEOUT_MS),
            ),
            max_attitude_rad: env_f32("MARV_GAZEBO_G1_MAX_ATTITUDE_RAD")
                .unwrap_or(DEFAULT_MAX_ATTITUDE_RAD),
            max_down_error_m: env_f32("MARV_GAZEBO_G1_MAX_DOWN_ERROR_M")
                .unwrap_or(DEFAULT_MAX_DOWN_ERROR_M),
            max_clamp_ratio: env_f32("MARV_GAZEBO_G1_MAX_CLAMP_RATIO")
                .unwrap_or(DEFAULT_MAX_CLAMP_RATIO),
            vertical_step_down_m: env_f32("MARV_GAZEBO_G1_VERTICAL_STEP_DOWN_M")
                .unwrap_or(DEFAULT_VERTICAL_STEP_DOWN_M),
            min_vertical_improvement_m: env_f32("MARV_GAZEBO_G1_MIN_VERTICAL_IMPROVEMENT_M")
                .unwrap_or(DEFAULT_MIN_VERTICAL_IMPROVEMENT_M),
            gyro_deadband_rps: env_f32("MARV_GAZEBO_G1_GYRO_DEADBAND_RPS")
                .unwrap_or(DEFAULT_GYRO_DEADBAND_RPS),
            max_hover_motor_spread: env_f32("MARV_GAZEBO_G1_MAX_HOVER_MOTOR_SPREAD")
                .unwrap_or(DEFAULT_MAX_HOVER_MOTOR_SPREAD),
        }
    }
}

fn run_runtime_scenario(
    name: &str,
    setpoint: ControlSetpoint,
    settings: &RuntimeSettings,
    airframe: &GazeboAirframeConfig,
) -> RuntimeSummary {
    let mut client = BridgeProbeClient::connect(&settings.endpoint);
    let mut sequence = Sequence::default();
    let pipeline = ControlPipeline::new(PureControlConfig {
        loop_config: control_config(airframe, settings.gyro_deadband_rps),
    });

    if settings.auto_reset {
        send_reset_and_play(
            &mut client,
            &mut sequence,
            settings,
            airframe.hover_motors(),
        );
    }

    let origin_frame = client.read_required_frame(settings.timeout);
    assert_eq!(
        origin_frame.clock_source.as_deref(),
        Some("gazebo"),
        "G1 needs Gazebo-clocked SENSOR frames"
    );
    assert!(
        origin_frame.attitude_quaternion().is_some(),
        "G1 requires bridge SENSOR orientation fields qw/qx/qy/qz or roll/pitch/yaw"
    );
    let origin = GazeboTruthOrigin::from_frame(&origin_frame).expect("finite truth origin");

    let mut max_attitude_rad = 0.0_f32;
    let mut max_down_error_m = 0.0_f32;
    let mut clamp_count = 0usize;
    let mut control_count = 0usize;
    let mut initial_down_error_m = None;
    let mut final_down_error_m = 0.0_f32;
    let mut throttle_sum = 0.0_f32;
    let mut motor_sum = 0.0_f32;
    let mut motor_sample_count = 0usize;
    let mut max_motor = 0.0_f32;
    let mut motor_spread_sum = 0.0_f32;
    let mut max_motor_spread = 0.0_f32;

    for _ in 0..settings.frames {
        let frame = client.read_required_frame(settings.timeout);
        let estimate = frame
            .to_truth_estimate(origin)
            .expect("Gazebo truth frame should convert to EstimateSnapshot");
        let imu = frame.to_truth_imu();
        let output = pipeline.step(estimate, imu, setpoint);
        assert!(
            output.control_valid,
            "invalid G1 control output at sim_time_us={}",
            frame.sim_time_us
        );

        let runtime_motors = airframe.runtime_control_motors(output.torque_command);
        throttle_sum += output.throttle;
        let motor_spread = motor_spread(runtime_motors.commands);
        motor_spread_sum += motor_spread;
        max_motor_spread = max_motor_spread.max(motor_spread);
        for motor in runtime_motors.commands {
            motor_sum += motor;
            motor_sample_count += 1;
            max_motor = max_motor.max(motor);
        }
        if runtime_motors.clamped {
            clamp_count += 1;
        }
        control_count += 1;
        client.send_actuator(&format_gazebo_actuator_line(
            sequence.next(),
            Some(frame.sim_time_us),
            runtime_motors.commands,
        ));

        if let Some(euler) = frame.euler_rad {
            max_attitude_rad = max_attitude_rad.max(euler[0].abs()).max(euler[1].abs());
        }
        final_down_error_m = estimate.position_ned_m[2] - setpoint.position_ned_m()[2];
        initial_down_error_m.get_or_insert(final_down_error_m);
        max_down_error_m = max_down_error_m.max(estimate.position_ned_m[2].abs());
    }

    client.send_actuator(&format_gazebo_actuator_line(
        sequence.next(),
        None,
        [0.0, 0.0, 0.0, 0.0],
    ));

    let summary = RuntimeSummary {
        max_attitude_rad,
        max_down_error_m,
        clamp_ratio: clamp_count as f32 / control_count.max(1) as f32,
        initial_down_error_m: initial_down_error_m.unwrap_or(0.0),
        final_down_error_m,
        mean_throttle: throttle_sum / control_count.max(1) as f32,
        mean_motor: motor_sum / motor_sample_count.max(1) as f32,
        max_motor,
        mean_motor_spread: motor_spread_sum / control_count.max(1) as f32,
        max_motor_spread,
    };
    eprintln!(
        "G1 {name}: {summary:?} mean_throttle={:.4} mean_motor={:.4} max_motor={:.4} mean_motor_spread={:.4} max_motor_spread={:.4}",
        summary.mean_throttle,
        summary.mean_motor,
        summary.max_motor,
        summary.mean_motor_spread,
        summary.max_motor_spread
    );
    summary
}

#[derive(Default)]
struct Sequence {
    next: u64,
}

impl Sequence {
    fn next(&mut self) -> u64 {
        self.next = self.next.saturating_add(1);
        self.next
    }
}

struct BridgeProbeClient {
    stream: TcpStream,
    read_buffer: String,
}

impl BridgeProbeClient {
    fn connect(endpoint: &str) -> Self {
        let stream = TcpStream::connect(endpoint).unwrap_or_else(|error| {
            panic!("failed to connect to Gazebo bridge {endpoint}: {error}")
        });
        stream
            .set_read_timeout(Some(Duration::from_millis(50)))
            .expect("read timeout should configure");
        stream
            .set_write_timeout(Some(Duration::from_millis(250)))
            .expect("write timeout should configure");
        Self {
            stream,
            read_buffer: String::new(),
        }
    }

    fn send_actuator(&mut self, line: &str) {
        self.stream
            .write_all(line.as_bytes())
            .unwrap_or_else(|error| panic!("failed to send actuator line {line:?}: {error}"));
    }

    fn send_sim_control(&mut self, line: &str, sequence: u64, timeout: Duration) {
        self.stream
            .write_all(line.as_bytes())
            .unwrap_or_else(|error| panic!("failed to send sim control line {line:?}: {error}"));
        self.wait_for_sim_control_ack(sequence, timeout);
    }

    fn read_required_frame(&mut self, timeout: Duration) -> GazeboSensorLine {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if let Some(frame) = self.read_frame() {
                return frame;
            }
        }

        panic!("expected Gazebo SENSOR frame within {timeout:?}");
    }

    fn collect_frames(&mut self, count: usize, timeout: Duration) {
        for _ in 0..count {
            let _ = self.read_required_frame(timeout);
        }
    }

    fn read_frame(&mut self) -> Option<GazeboSensorLine> {
        while let Some(line) = self.read_line() {
            if let Some(frame) = GazeboSensorLine::parse(&line) {
                return Some(frame);
            }
        }

        None
    }

    fn read_line(&mut self) -> Option<String> {
        if let Some(line) = take_line(&mut self.read_buffer) {
            return Some(line);
        }

        let mut buffer = [0_u8; 1024];
        match self.stream.read(&mut buffer) {
            Ok(0) => panic!("Gazebo bridge closed the TCP connection"),
            Ok(bytes) => {
                self.read_buffer
                    .push_str(&String::from_utf8_lossy(&buffer[..bytes]));
                take_line(&mut self.read_buffer)
            }
            Err(error)
                if matches!(
                    error.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                ) =>
            {
                None
            }
            Err(error) => panic!("failed to read Gazebo bridge frame: {error}"),
        }
    }

    fn wait_for_sim_control_ack(&mut self, sequence: u64, timeout: Duration) {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let Some(line) = self.read_line() else {
                continue;
            };
            if !line.starts_with("SIM_CONTROL_ACK") {
                continue;
            }
            let ack_sequence =
                field_value(&line, "seq").and_then(|value| value.parse::<u64>().ok());
            if ack_sequence != Some(sequence) {
                continue;
            }
            assert!(
                field_value(&line, "ok") == Some("1"),
                "Gazebo sim control command seq={sequence} failed: {line}"
            );
            return;
        }

        panic!("timed out waiting for Gazebo SIM_CONTROL_ACK seq={sequence}");
    }
}

fn send_reset_and_play(
    client: &mut BridgeProbeClient,
    sequence: &mut Sequence,
    settings: &RuntimeSettings,
    settle_motors: [f32; 4],
) {
    client.send_actuator(&format_gazebo_actuator_line(
        sequence.next(),
        None,
        [0.0, 0.0, 0.0, 0.0],
    ));

    for action in [
        GazeboSimControlAction::Pause,
        GazeboSimControlAction::Reset,
        GazeboSimControlAction::Play,
    ] {
        let sim_control_sequence = sequence.next();
        client.send_sim_control(
            &format_gazebo_sim_control_line(sim_control_sequence, action),
            sim_control_sequence,
            settings.timeout,
        );
    }

    client.send_actuator(&format_gazebo_actuator_line(
        sequence.next(),
        None,
        settle_motors,
    ));
    client.collect_frames(settings.reset_settle_frames, settings.timeout);
}

fn airframe_config() -> GazeboAirframeConfig {
    let airframe =
        GazeboAirframeConfig::parse(AIRFRAME_CONFIG).expect("checked-in airframe config parses");
    airframe
        .validate()
        .expect("checked-in airframe config is physically self-consistent");
    airframe
}

fn control_config(
    airframe: &GazeboAirframeConfig,
    measured_rate_deadband_rps: f32,
) -> ControlLoopConfig {
    let mut config = ControlLoopConfig::default();
    config.altitude.hover_throttle = airframe.hover_motor_command();
    config.altitude.max_throttle_correction = 0.35;
    config.rate.measured_rate_deadband_rps = measured_rate_deadband_rps.max(0.0);
    config
}

fn motor_spread(motors: [f32; 4]) -> f32 {
    let mut min_motor = f32::INFINITY;
    let mut max_motor = f32::NEG_INFINITY;
    for motor in motors {
        min_motor = min_motor.min(motor);
        max_motor = max_motor.max(motor);
    }
    max_motor - min_motor
}

fn estimate_with_euler_deg(roll_deg: f32, pitch_deg: f32, yaw_deg: f32) -> EstimateSnapshot {
    let deg_to_rad = core::f32::consts::PI / 180.0;
    EstimateSnapshot {
        quaternion: AttitudeSetpoint::from_euler_rad(
            roll_deg * deg_to_rad,
            pitch_deg * deg_to_rad,
            yaw_deg * deg_to_rad,
        )
        .expect("finite euler angles should produce a quaternion")
        .quaternion,
        ..EstimateSnapshot::LEVEL_ORIGIN
    }
}

fn take_line(buffer: &mut String) -> Option<String> {
    let newline = buffer.find('\n')?;
    let line = buffer[..newline].trim().to_string();
    buffer.drain(..=newline);
    Some(line)
}

fn field_value<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let prefix = format!("{key}=");
    line.split_whitespace()
        .find_map(|token| token.strip_prefix(&prefix))
}

fn env_bool(key: &str) -> Option<bool> {
    env::var(key)
        .ok()
        .map(|value| matches!(value.as_str(), "1" | "true" | "TRUE" | "yes" | "on"))
}

fn env_usize(key: &str) -> Option<usize> {
    env::var(key).ok()?.parse().ok()
}

fn env_u64(key: &str) -> Option<u64> {
    env::var(key).ok()?.parse().ok()
}

fn env_f32(key: &str) -> Option<f32> {
    env::var(key).ok()?.parse().ok()
}
