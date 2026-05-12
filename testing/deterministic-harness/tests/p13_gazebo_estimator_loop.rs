use std::env;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use common::control::config::ControlLoopConfig;
use deterministic_harness::{
    ControlPipeline, ControlSetpoint, EstimateSnapshot, EstimatorReplayConfig,
    EstimatorReplayDriver, EstimatorReplayTrace, GAZEBO_G0_DEFAULT_ENDPOINT, GazeboAirframeConfig,
    GazeboSensorLine, GazeboSimControlAction, GazeboTruthOrigin, ImuControlInput,
    PureControlConfig, SensorFrame, format_gazebo_actuator_line, format_gazebo_sim_control_line,
};

const AIRFRAME_CONFIG: &str = include_str!("../config/airframes/f450_xing2_2809_1045_4s_v0.cfg");
const DEFAULT_WARMUP_FRAMES: usize = 250;
const DEFAULT_FRAMES: usize = 250;
const DEFAULT_RESET_SETTLE_FRAMES: usize = 50;
const DEFAULT_TIMEOUT_MS: u64 = 8_000;
const DEFAULT_MAX_ATTITUDE_RAD: f32 = 5.0_f32.to_radians();
const DEFAULT_MAX_DOWN_ERROR_M: f32 = 3.0;
const DEFAULT_MAX_HORIZONTAL_ERROR_M: f32 = 0.25;
const DEFAULT_MAX_CLAMP_RATIO: f32 = 0.25;
const DEFAULT_VERTICAL_STEP_DOWN_M: f32 = -3.0;
const DEFAULT_MIN_VERTICAL_IMPROVEMENT_M: f32 = 0.05;
const DEFAULT_MAX_FINAL_DOWN_ERROR_M: f32 = 1.25;
const DEFAULT_GYRO_DEADBAND_RPS: f32 = 0.02;
const DEFAULT_MAX_HOVER_MOTOR_SPREAD: f32 = 0.01;
const DEFAULT_POSITION_GAIN: f32 = 0.35;
const DEFAULT_POSITION_VELOCITY_GAIN: f32 = 0.80;
const DEFAULT_MAX_HORIZONTAL_ACCEL_MPS2: f32 = 3.0;
const DEFAULT_MAX_TILT_RAD: f32 = 20.0_f32.to_radians();
const DEFAULT_ALTITUDE_GAIN: f32 = 0.16;
const DEFAULT_VERTICAL_VELOCITY_GAIN: f32 = 0.12;
const DEFAULT_MAX_THROTTLE_CORRECTION: f32 = 0.35;
const DEFAULT_USE_MAGNETOMETER: bool = false;

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_BRIDGE_ADDR if not 127.0.0.1:9000"]
fn p13_gazebo_estimator_warmup() {
    let settings = RuntimeSettings::from_env();
    let airframe = airframe_config();
    let summary = run_warmup(&settings, &airframe);

    assert!(
        summary.converted_frames >= 2,
        "G2 warmup needs at least two estimator frames: summary={summary:?}"
    );
    assert!(
        summary.valid_frames > 0,
        "G2 estimator never became valid: summary={summary:?}"
    );
    assert!(
        summary.first_valid_tick.is_some(),
        "G2 estimator did not report its first valid tick: summary={summary:?}"
    );
    assert!(
        summary.max_attitude_rad <= settings.max_attitude_rad,
        "G2 warmup attitude diverged: summary={summary:?}"
    );
    assert!(
        summary.max_estimate_down_error_m <= settings.max_down_error_m,
        "G2 warmup estimate altitude diverged: summary={summary:?}"
    );
}

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_BRIDGE_ADDR if not 127.0.0.1:9000"]
fn p13_gazebo_estimator_loop() {
    let settings = RuntimeSettings::from_env();
    let airframe = airframe_config();

    let origin = run_control_scenario(
        "origin_hold",
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        &settings,
        &airframe,
    );
    assert!(
        origin.converted_frames >= origin.control_frames,
        "G2 origin converted fewer frames than it controlled: summary={origin:?}"
    );
    assert!(
        origin.max_truth_down_error_m <= settings.max_down_error_m,
        "G2 origin truth altitude diverged: summary={origin:?}"
    );
    assert!(
        origin.max_estimate_down_error_m <= settings.max_down_error_m,
        "G2 origin estimate altitude diverged: summary={origin:?}"
    );
    assert!(
        origin.max_truth_horizontal_error_m <= settings.max_horizontal_error_m,
        "G2 origin truth drifted horizontally: summary={origin:?}"
    );
    assert!(
        origin.max_estimate_horizontal_error_m <= settings.max_horizontal_error_m,
        "G2 origin estimate drifted horizontally: summary={origin:?}"
    );
    assert!(
        origin.max_attitude_rad <= settings.max_attitude_rad,
        "G2 origin attitude diverged: summary={origin:?}"
    );
    assert!(
        origin.clamp_ratio <= settings.max_clamp_ratio,
        "G2 origin clamped too often: summary={origin:?}"
    );
    assert!(
        origin.max_motor_spread <= settings.max_hover_motor_spread,
        "G2 origin hover motor chatter too high: summary={origin:?}"
    );

    let vertical_setpoint =
        ControlSetpoint::new([0.0, 0.0, settings.vertical_step_down_m], 0.0, true);
    let vertical =
        run_control_scenario("vertical_step_up", vertical_setpoint, &settings, &airframe);
    assert!(
        vertical.converted_frames >= vertical.control_frames,
        "G2 vertical converted fewer frames than it controlled: summary={vertical:?}"
    );
    assert!(
        vertical.max_attitude_rad <= settings.max_attitude_rad,
        "G2 vertical attitude diverged: summary={vertical:?}"
    );
    assert!(
        vertical.max_truth_horizontal_error_m <= settings.max_horizontal_error_m,
        "G2 vertical truth drifted horizontally: summary={vertical:?}"
    );
    assert!(
        vertical.max_estimate_horizontal_error_m <= settings.max_horizontal_error_m,
        "G2 vertical estimate drifted horizontally: summary={vertical:?}"
    );
    assert!(
        vertical.clamp_ratio <= settings.max_clamp_ratio,
        "G2 vertical clamped too often: summary={vertical:?}"
    );
    assert!(
        vertical.final_truth_down_error_m.abs()
            <= vertical.initial_truth_down_error_m.abs() - settings.min_vertical_improvement_m,
        "G2 vertical truth did not improve enough: summary={vertical:?}"
    );
    assert!(
        vertical.final_estimate_down_error_m.abs()
            <= vertical.initial_estimate_down_error_m.abs() - settings.min_vertical_improvement_m,
        "G2 vertical estimate did not improve enough: summary={vertical:?}"
    );
    assert!(
        vertical.final_truth_down_error_m.abs() <= settings.max_final_down_error_m,
        "G2 vertical truth did not settle close enough: summary={vertical:?}"
    );
    assert!(
        vertical.final_estimate_down_error_m.abs() <= settings.max_final_down_error_m,
        "G2 vertical estimate did not settle close enough: summary={vertical:?}"
    );
}

#[derive(Clone, Copy, Debug)]
struct WarmupSummary {
    converted_frames: usize,
    valid_frames: usize,
    first_valid_tick: Option<u64>,
    max_attitude_rad: f32,
    max_estimate_down_error_m: f32,
}

#[derive(Clone, Copy, Debug)]
struct RuntimeSummary {
    converted_frames: usize,
    control_frames: usize,
    max_attitude_rad: f32,
    max_truth_down_error_m: f32,
    max_estimate_down_error_m: f32,
    max_truth_horizontal_error_m: f32,
    max_estimate_horizontal_error_m: f32,
    clamp_ratio: f32,
    initial_truth_down_error_m: f32,
    final_truth_down_error_m: f32,
    initial_estimate_down_error_m: f32,
    final_estimate_down_error_m: f32,
    final_truth_position_ned_m: [f32; 3],
    final_truth_velocity_ned_mps: [f32; 3],
    final_estimate_position_ned_m: [f32; 3],
    final_estimate_velocity_ned_mps: [f32; 3],
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
    warmup_frames: usize,
    frames: usize,
    reset_settle_frames: usize,
    timeout: Duration,
    max_attitude_rad: f32,
    max_down_error_m: f32,
    max_horizontal_error_m: f32,
    max_clamp_ratio: f32,
    vertical_step_down_m: f32,
    min_vertical_improvement_m: f32,
    max_final_down_error_m: f32,
    gyro_deadband_rps: f32,
    max_hover_motor_spread: f32,
    position_gain: f32,
    position_velocity_gain: f32,
    max_horizontal_accel_mps2: f32,
    max_tilt_rad: f32,
    altitude_gain: f32,
    vertical_velocity_gain: f32,
    max_throttle_correction: f32,
    use_magnetometer: bool,
}

impl RuntimeSettings {
    fn from_env() -> Self {
        Self {
            endpoint: env::var("MARV_GAZEBO_BRIDGE_ADDR")
                .unwrap_or_else(|_| GAZEBO_G0_DEFAULT_ENDPOINT.to_string()),
            auto_reset: env_bool("MARV_GAZEBO_G2_AUTO_RESET").unwrap_or(false),
            warmup_frames: env_usize("MARV_GAZEBO_G2_WARMUP_FRAMES")
                .unwrap_or(DEFAULT_WARMUP_FRAMES),
            frames: env_usize("MARV_GAZEBO_G2_FRAMES").unwrap_or(DEFAULT_FRAMES),
            reset_settle_frames: env_usize("MARV_GAZEBO_G2_RESET_SETTLE_FRAMES")
                .unwrap_or(DEFAULT_RESET_SETTLE_FRAMES),
            timeout: Duration::from_millis(
                env_u64("MARV_GAZEBO_G2_TIMEOUT_MS").unwrap_or(DEFAULT_TIMEOUT_MS),
            ),
            max_attitude_rad: env_f32("MARV_GAZEBO_G2_MAX_ATTITUDE_RAD")
                .unwrap_or(DEFAULT_MAX_ATTITUDE_RAD),
            max_down_error_m: env_f32("MARV_GAZEBO_G2_MAX_DOWN_ERROR_M")
                .unwrap_or(DEFAULT_MAX_DOWN_ERROR_M),
            max_horizontal_error_m: env_f32("MARV_GAZEBO_G2_MAX_HORIZONTAL_ERROR_M")
                .unwrap_or(DEFAULT_MAX_HORIZONTAL_ERROR_M),
            max_clamp_ratio: env_f32("MARV_GAZEBO_G2_MAX_CLAMP_RATIO")
                .unwrap_or(DEFAULT_MAX_CLAMP_RATIO),
            vertical_step_down_m: env_f32("MARV_GAZEBO_G2_VERTICAL_STEP_DOWN_M")
                .unwrap_or(DEFAULT_VERTICAL_STEP_DOWN_M),
            min_vertical_improvement_m: env_f32("MARV_GAZEBO_G2_MIN_VERTICAL_IMPROVEMENT_M")
                .unwrap_or(DEFAULT_MIN_VERTICAL_IMPROVEMENT_M),
            max_final_down_error_m: env_f32("MARV_GAZEBO_G2_MAX_FINAL_DOWN_ERROR_M")
                .unwrap_or(DEFAULT_MAX_FINAL_DOWN_ERROR_M),
            gyro_deadband_rps: env_f32("MARV_GAZEBO_G2_GYRO_DEADBAND_RPS")
                .unwrap_or(DEFAULT_GYRO_DEADBAND_RPS),
            max_hover_motor_spread: env_f32("MARV_GAZEBO_G2_MAX_HOVER_MOTOR_SPREAD")
                .unwrap_or(DEFAULT_MAX_HOVER_MOTOR_SPREAD),
            position_gain: env_f32("MARV_GAZEBO_G2_POSITION_GAIN").unwrap_or(DEFAULT_POSITION_GAIN),
            position_velocity_gain: env_f32("MARV_GAZEBO_G2_POSITION_VELOCITY_GAIN")
                .unwrap_or(DEFAULT_POSITION_VELOCITY_GAIN),
            max_horizontal_accel_mps2: env_f32("MARV_GAZEBO_G2_MAX_HORIZONTAL_ACCEL_MPS2")
                .unwrap_or(DEFAULT_MAX_HORIZONTAL_ACCEL_MPS2),
            max_tilt_rad: env_f32("MARV_GAZEBO_G2_MAX_TILT_RAD").unwrap_or(DEFAULT_MAX_TILT_RAD),
            altitude_gain: env_f32("MARV_GAZEBO_G2_ALTITUDE_GAIN").unwrap_or(DEFAULT_ALTITUDE_GAIN),
            vertical_velocity_gain: env_f32("MARV_GAZEBO_G2_VERTICAL_VELOCITY_GAIN")
                .unwrap_or(DEFAULT_VERTICAL_VELOCITY_GAIN),
            max_throttle_correction: env_f32("MARV_GAZEBO_G2_MAX_THROTTLE_CORRECTION")
                .unwrap_or(DEFAULT_MAX_THROTTLE_CORRECTION),
            use_magnetometer: env_bool("MARV_GAZEBO_G2_USE_MAGNETOMETER")
                .unwrap_or(DEFAULT_USE_MAGNETOMETER),
        }
    }
}

fn run_warmup(settings: &RuntimeSettings, airframe: &GazeboAirframeConfig) -> WarmupSummary {
    let mut client = BridgeProbeClient::connect(&settings.endpoint);
    let mut sequence = Sequence::default();
    if settings.auto_reset {
        send_reset_and_play(&mut client, &mut sequence, settings, [0.0; 4]);
    } else {
        client.send_actuator(&format_gazebo_actuator_line(
            sequence.next(),
            None,
            [0.0; 4],
        ));
    }

    let origin_frame = client.read_required_frame(settings.timeout);
    assert_g2_sensor_contract(&origin_frame);
    let origin = GazeboTruthOrigin::from_frame(&origin_frame).expect("finite Gazebo origin");
    let mut driver = EstimatorReplayDriver::new(gazebo_estimator_config());
    let mut last_timestamp_us = None;
    let mut converted_frames = 0usize;
    let mut valid_frames = 0usize;
    let mut first_valid_tick = None;
    let mut max_attitude_rad = 0.0_f32;
    let mut max_estimate_down_error_m = 0.0_f32;

    while converted_frames < settings.warmup_frames {
        let (frame, sensor) =
            client.read_required_estimator_frame(origin, last_timestamp_us, settings.timeout);
        last_timestamp_us = Some(sensor.timestamp_us);
        converted_frames += 1;
        let tick = converted_frames as u64;
        let sensor = estimator_sensor_for_settings(sensor, settings);
        let trace = driver
            .step(tick, &sensor)
            .expect("G2 warmup estimator step should pass");

        if trace.estimate_valid {
            valid_frames += 1;
            first_valid_tick.get_or_insert(tick);
            max_estimate_down_error_m =
                max_estimate_down_error_m.max(trace.position_ned_m[2].abs() as f32);
            max_attitude_rad = max_attitude_rad.max(quaternion_angle_rad(trace.quaternion));
        }
        if let Some(euler) = frame.euler_rad {
            max_attitude_rad = max_attitude_rad.max(euler[0].abs()).max(euler[1].abs());
        }
    }

    client.send_actuator(&format_gazebo_actuator_line(
        sequence.next(),
        None,
        [0.0; 4],
    ));

    let summary = WarmupSummary {
        converted_frames,
        valid_frames,
        first_valid_tick,
        max_attitude_rad,
        max_estimate_down_error_m,
    };
    eprintln!("G2 warmup: {summary:?}");
    let _ = airframe;
    summary
}

fn run_control_scenario(
    name: &str,
    setpoint: ControlSetpoint,
    settings: &RuntimeSettings,
    airframe: &GazeboAirframeConfig,
) -> RuntimeSummary {
    let mut client = BridgeProbeClient::connect(&settings.endpoint);
    let mut sequence = Sequence::default();
    let pipeline = ControlPipeline::new(PureControlConfig {
        loop_config: control_config(airframe, settings),
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
    assert_g2_sensor_contract(&origin_frame);
    let origin = GazeboTruthOrigin::from_frame(&origin_frame).expect("finite Gazebo origin");
    let origin_sensor = origin_frame
        .to_estimator_sensor_frame(origin)
        .expect("G2 origin frame should include fresh IMU");
    let origin_sensor = estimator_sensor_for_settings(origin_sensor, settings);
    let mut driver = EstimatorReplayDriver::new(gazebo_estimator_config());
    driver
        .step(1, &origin_sensor)
        .expect("G2 estimator prewarm should pass");
    let mut last_timestamp_us = Some(origin_sensor.timestamp_us);

    let mut converted_frames = 1usize;
    let mut control_frames = 0usize;
    let mut max_attitude_rad = 0.0_f32;
    let mut max_truth_down_error_m = 0.0_f32;
    let mut max_estimate_down_error_m = 0.0_f32;
    let mut max_truth_horizontal_error_m = 0.0_f32;
    let mut max_estimate_horizontal_error_m = 0.0_f32;
    let mut clamp_count = 0usize;
    let mut initial_truth_down_error_m = None;
    let mut final_truth_down_error_m = 0.0_f32;
    let mut initial_estimate_down_error_m = None;
    let mut final_estimate_down_error_m = 0.0_f32;
    let mut final_truth_position_ned_m = [0.0; 3];
    let mut final_truth_velocity_ned_mps = [0.0; 3];
    let mut final_estimate_position_ned_m = [0.0; 3];
    let mut final_estimate_velocity_ned_mps = [0.0; 3];
    let mut throttle_sum = 0.0_f32;
    let mut motor_sum = 0.0_f32;
    let mut motor_sample_count = 0usize;
    let mut max_motor = 0.0_f32;
    let mut motor_spread_sum = 0.0_f32;
    let mut max_motor_spread = 0.0_f32;

    while control_frames < settings.frames {
        let (frame, sensor) =
            client.read_required_estimator_frame(origin, last_timestamp_us, settings.timeout);
        let sensor = estimator_sensor_for_settings(sensor, settings);
        last_timestamp_us = Some(sensor.timestamp_us);
        converted_frames += 1;
        let trace = driver
            .step(converted_frames as u64, &sensor)
            .expect("G2 estimator step should pass");
        if !trace.estimate_valid {
            continue;
        }

        let estimate = estimate_snapshot(&trace);
        let output = pipeline.step(estimate, imu_from_sensor(&sensor), setpoint);
        assert!(
            output.control_valid,
            "invalid G2 control output at sim_time_us={}",
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
        control_frames += 1;
        client.send_actuator(&format_gazebo_actuator_line(
            sequence.next(),
            Some(frame.sim_time_us),
            runtime_motors.commands,
        ));

        if let Some(euler) = frame.euler_rad {
            max_attitude_rad = max_attitude_rad.max(euler[0].abs()).max(euler[1].abs());
        }
        let truth = frame
            .to_truth_estimate(origin)
            .expect("G2 truth side-channel should convert for evidence");
        final_truth_position_ned_m = truth.position_ned_m;
        final_truth_velocity_ned_mps = truth.velocity_ned_mps;
        final_estimate_position_ned_m = estimate.position_ned_m;
        final_estimate_velocity_ned_mps = estimate.velocity_ned_mps;
        final_truth_down_error_m = truth.position_ned_m[2] - setpoint.position_ned_m[2];
        final_estimate_down_error_m = estimate.position_ned_m[2] - setpoint.position_ned_m[2];
        initial_truth_down_error_m.get_or_insert(final_truth_down_error_m);
        initial_estimate_down_error_m.get_or_insert(final_estimate_down_error_m);
        max_truth_down_error_m = max_truth_down_error_m.max(truth.position_ned_m[2].abs());
        max_estimate_down_error_m = max_estimate_down_error_m.max(estimate.position_ned_m[2].abs());
        max_truth_horizontal_error_m = max_truth_horizontal_error_m.max(horizontal_error_m(
            truth.position_ned_m,
            setpoint.position_ned_m,
        ));
        max_estimate_horizontal_error_m = max_estimate_horizontal_error_m.max(horizontal_error_m(
            estimate.position_ned_m,
            setpoint.position_ned_m,
        ));
    }

    client.send_actuator(&format_gazebo_actuator_line(
        sequence.next(),
        None,
        [0.0; 4],
    ));

    let summary = RuntimeSummary {
        converted_frames,
        control_frames,
        max_attitude_rad,
        max_truth_down_error_m,
        max_estimate_down_error_m,
        max_truth_horizontal_error_m,
        max_estimate_horizontal_error_m,
        clamp_ratio: clamp_count as f32 / control_frames.max(1) as f32,
        initial_truth_down_error_m: initial_truth_down_error_m.unwrap_or(0.0),
        final_truth_down_error_m,
        initial_estimate_down_error_m: initial_estimate_down_error_m.unwrap_or(0.0),
        final_estimate_down_error_m,
        final_truth_position_ned_m,
        final_truth_velocity_ned_mps,
        final_estimate_position_ned_m,
        final_estimate_velocity_ned_mps,
        mean_throttle: throttle_sum / control_frames.max(1) as f32,
        mean_motor: motor_sum / motor_sample_count.max(1) as f32,
        max_motor,
        mean_motor_spread: motor_spread_sum / control_frames.max(1) as f32,
        max_motor_spread,
    };
    eprintln!(
        "G2 {name}: {summary:?} final_truth_pos={:?} final_truth_vel={:?} final_estimate_pos={:?} final_estimate_vel={:?} mean_throttle={:.4} mean_motor={:.4} max_motor={:.4} mean_motor_spread={:.4} max_motor_spread={:.4}",
        summary.final_truth_position_ned_m,
        summary.final_truth_velocity_ned_mps,
        summary.final_estimate_position_ned_m,
        summary.final_estimate_velocity_ned_mps,
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

    fn read_required_estimator_frame(
        &mut self,
        origin: GazeboTruthOrigin,
        after_timestamp_us: Option<u64>,
        timeout: Duration,
    ) -> (GazeboSensorLine, SensorFrame) {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let Some(frame) = self.read_frame() else {
                continue;
            };
            let Some(sensor) = frame.to_estimator_sensor_frame(origin) else {
                continue;
            };
            if after_timestamp_us.is_some_and(|previous| sensor.timestamp_us <= previous) {
                continue;
            }
            return (frame, sensor);
        }

        panic!("expected fresh Gazebo estimator SENSOR frame within {timeout:?}");
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

fn assert_g2_sensor_contract(frame: &GazeboSensorLine) {
    assert_eq!(
        frame.clock_source.as_deref(),
        Some("gazebo"),
        "G2 needs Gazebo-clocked SENSOR frames"
    );
    assert!(
        frame.attitude_quaternion().is_some(),
        "G2 evidence needs bridge SENSOR orientation fields qw/qx/qy/qz or roll/pitch/yaw"
    );
    assert!(
        frame.position_ned_m.is_some(),
        "G2 evidence needs bridge SENSOR pose truth fields pn/pe/pd"
    );
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
    settings: &RuntimeSettings,
) -> ControlLoopConfig {
    let mut config = ControlLoopConfig::default();
    config.position.position_gain = settings.position_gain.max(0.0);
    config.position.velocity_gain = settings.position_velocity_gain.max(0.0);
    config.position.max_horizontal_accel_mps2 = settings.max_horizontal_accel_mps2.max(0.0);
    config.position.max_tilt_rad = settings.max_tilt_rad.max(0.0);
    config.altitude.hover_throttle = airframe.hover_motor_command();
    config.altitude.altitude_gain = settings.altitude_gain.max(0.0);
    config.altitude.vertical_velocity_gain = settings.vertical_velocity_gain.max(0.0);
    config.altitude.max_throttle_correction = settings.max_throttle_correction.max(0.0);
    config.rate.measured_rate_deadband_rps = settings.gyro_deadband_rps.max(0.0);
    config
}

fn gazebo_estimator_config() -> EstimatorReplayConfig {
    EstimatorReplayConfig {
        gps_position_std_m: [0.05, 0.05, 0.10],
        gps_velocity_std_mps: [0.05, 0.05, 0.10],
        baro_std_m: 0.05,
        ..EstimatorReplayConfig::default()
    }
}

fn estimator_sensor_for_settings(
    mut sensor: SensorFrame,
    settings: &RuntimeSettings,
) -> SensorFrame {
    if !settings.use_magnetometer {
        sensor.mag_body_ut = None;
    }
    sensor
}

fn estimate_snapshot(trace: &EstimatorReplayTrace) -> EstimateSnapshot {
    let finite = trace.quaternion.iter().all(|value| value.is_finite())
        && trace.position_ned_m.iter().all(|value| value.is_finite())
        && trace.velocity_ned_mps.iter().all(|value| value.is_finite());

    EstimateSnapshot {
        position_ned_m: trace.position_ned_m.map(|value| value as f32),
        velocity_ned_mps: trace.velocity_ned_mps.map(|value| value as f32),
        quaternion: trace.quaternion.map(|value| value as f32),
        valid: trace.estimate_valid && finite,
    }
}

fn imu_from_sensor(sensor: &SensorFrame) -> ImuControlInput {
    ImuControlInput {
        accel_mps2: sensor.accel_mps2.map(|value| value as f32),
        gyro_rps: sensor.gyro_rps.map(|value| value as f32),
    }
}

fn quaternion_angle_rad(quaternion: [f64; 4]) -> f32 {
    let w = quaternion[0].abs().clamp(-1.0, 1.0);
    (2.0 * w.acos()) as f32
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

fn horizontal_error_m(position_ned_m: [f32; 3], setpoint_ned_m: [f32; 3]) -> f32 {
    let north_error_m = position_ned_m[0] - setpoint_ned_m[0];
    let east_error_m = position_ned_m[1] - setpoint_ned_m[1];
    (north_error_m * north_error_m + east_error_m * east_error_m).sqrt()
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
