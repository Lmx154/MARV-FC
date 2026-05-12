use std::env;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use deterministic_harness::{
    GAZEBO_G0_DEFAULT_ENDPOINT, GazeboActuatorAxis, GazeboActuatorExpectation,
    GazeboActuatorTruthCase, GazeboAirframeConfig, GazeboBridgeConfig, GazeboSensorLine,
    GazeboSimControlAction, format_gazebo_actuator_line, format_gazebo_sim_control_line,
    gazebo_g0_actuator_truth_cases,
};

const BRIDGE_CONFIG: &str =
    include_str!("../../../telemetry-app/gazebo_bridge/config/bridge_config");
const AIRFRAME_CONFIG: &str = include_str!("../config/airframes/f450_xing2_2809_1045_4s_v0.cfg");
const DEFAULT_FRAMES_PER_CASE: usize = 50;
const DEFAULT_RESET_SETTLE_FRAMES: usize = 50;
const DEFAULT_MIN_AXIS_RATE_DELTA_RPS: f32 = 0.02;
const DEFAULT_ZERO_MAX_GYRO_RPS: f32 = 0.08;

#[test]
fn p11_gazebo_actuator_truth_table_cases_match_expected_bridge_protocol() {
    let config = bridge_config();
    let airframe = airframe_config();
    let cases = gazebo_g0_actuator_truth_cases(&airframe);

    assert_eq!(cases.len(), 12);
    assert_case(&cases[0], "zero", [0.0, 0.0, 0.0, 0.0]);
    assert_case_near(
        &cases[1],
        "hover",
        [0.572_727, 0.572_727, 0.572_727, 0.572_727],
    );
    assert_case_near(&cases[6], "roll_positive", [0.0, 1.0, 1.0, 0.0]);
    assert_case_near(&cases[8], "pitch_positive", [1.0, 0.0, 1.0, 0.0]);
    assert_case_near(&cases[10], "yaw_positive", [1.0, 1.0, 0.0, 0.0]);

    let line = format_gazebo_actuator_line(42, Some(840_000), cases[6].motors);
    assert_eq!(
        line,
        "ACTUATOR seq=42 sim_time_us=840000 m0=0.000 m1=1.000 m2=1.000 m3=0.000\n"
    );
    assert_speeds_near(
        config.motor_speeds_rad_s(cases[6].motors),
        [0.0, 1100.0, 1100.0, 0.0],
    );
}

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_BRIDGE_ADDR if not 127.0.0.1:9000"]
fn p11_gazebo_actuator_truth_table_runtime() {
    let settings = RuntimeSettings::from_env();
    let airframe = airframe_config();
    let hover_motors = airframe.hover_motors();
    let reset_settle_motors = airframe.reset_settle_motors();
    let mut client = BridgeProbeClient::connect(&settings.endpoint);
    let mut sequence = Sequence::default();

    if settings.auto_reset {
        send_reset_and_play(&mut client, &mut sequence, &settings, reset_settle_motors);
    }

    let initial = client.collect_frames(settings.frames_per_case, settings.case_timeout);
    assert!(
        initial
            .iter()
            .any(|frame| frame.clock_source.as_deref() == Some("gazebo")),
        "bridge connected at {}, but no Gazebo-clocked SENSOR frames arrived; is the sim running?",
        settings.endpoint
    );

    let mut failures = Vec::new();
    let cases = selected_runtime_cases(&settings.selected_case);
    eprintln!(
        "G0 selected_case={} cases={} endpoint={}",
        settings.selected_case,
        cases.len(),
        settings.endpoint
    );
    for case in cases {
        if settings.auto_reset {
            send_reset_and_play(&mut client, &mut sequence, &settings, reset_settle_motors);
        }
        let baseline = run_case(
            &mut client,
            "baseline_hover",
            hover_motors,
            &mut sequence,
            settings.frames_per_case,
            settings.case_timeout,
        );
        let observed = run_case(
            &mut client,
            case.name,
            case.motors,
            &mut sequence,
            settings.frames_per_case,
            settings.case_timeout,
        );
        if let Err(error) = runtime_expectation_error(&settings, case, baseline, observed) {
            failures.push(error);
        }
    }

    client.send_actuator(&format_gazebo_actuator_line(
        sequence.next(),
        None,
        [0.0, 0.0, 0.0, 0.0],
    ));

    if settings.observe_only {
        if failures.is_empty() {
            eprintln!("Gazebo G0 observation matched configured expectations");
        } else {
            eprintln!("Gazebo G0 observation mismatches:\n{}", failures.join("\n"));
        }
    } else {
        assert!(
            failures.is_empty(),
            "Gazebo G0 actuator truth table failed:\n{}",
            failures.join("\n")
        );
    }
}

#[derive(Clone, Debug)]
struct RuntimeSettings {
    endpoint: String,
    selected_case: String,
    observe_only: bool,
    auto_reset: bool,
    frames_per_case: usize,
    reset_settle_frames: usize,
    case_timeout: Duration,
    min_axis_rate_delta_rps: f32,
    zero_max_gyro_rps: f32,
}

impl RuntimeSettings {
    fn from_env() -> Self {
        Self {
            endpoint: env::var("MARV_GAZEBO_BRIDGE_ADDR")
                .unwrap_or_else(|_| GAZEBO_G0_DEFAULT_ENDPOINT.to_string()),
            selected_case: env::var("MARV_GAZEBO_G0_CASE").unwrap_or_else(|_| "zero".to_string()),
            observe_only: env_bool("MARV_GAZEBO_G0_OBSERVE_ONLY").unwrap_or(false),
            auto_reset: env_bool("MARV_GAZEBO_G0_AUTO_RESET").unwrap_or(false),
            frames_per_case: env_usize("MARV_GAZEBO_G0_FRAMES_PER_CASE")
                .unwrap_or(DEFAULT_FRAMES_PER_CASE),
            reset_settle_frames: env_usize("MARV_GAZEBO_G0_RESET_SETTLE_FRAMES")
                .unwrap_or(DEFAULT_RESET_SETTLE_FRAMES),
            case_timeout: Duration::from_millis(
                env_u64("MARV_GAZEBO_G0_CASE_TIMEOUT_MS").unwrap_or(5_000),
            ),
            min_axis_rate_delta_rps: env_f32("MARV_GAZEBO_G0_MIN_AXIS_RATE_DELTA_RPS")
                .unwrap_or(DEFAULT_MIN_AXIS_RATE_DELTA_RPS),
            zero_max_gyro_rps: env_f32("MARV_GAZEBO_G0_ZERO_MAX_GYRO_RPS")
                .unwrap_or(DEFAULT_ZERO_MAX_GYRO_RPS),
        }
    }
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

#[derive(Clone, Copy, Debug)]
struct CaseObservation {
    mean_gyro_rps: [f32; 3],
    max_abs_gyro_rps: f32,
    mean_vd_mps: f32,
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

    fn collect_frames(&mut self, count: usize, timeout: Duration) -> Vec<GazeboSensorLine> {
        let deadline = Instant::now() + timeout;
        let mut frames = Vec::with_capacity(count);
        while frames.len() < count && Instant::now() < deadline {
            if let Some(frame) = self.read_frame() {
                frames.push(frame);
            }
        }
        assert!(
            frames.len() >= count,
            "expected {count} Gazebo SENSOR frames within {timeout:?}, got {}",
            frames.len()
        );
        frames
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
            settings.case_timeout,
        );
    }

    client.send_actuator(&format_gazebo_actuator_line(
        sequence.next(),
        None,
        settle_motors,
    ));

    if settings.reset_settle_frames > 0 {
        let _ = client.collect_frames(settings.reset_settle_frames, settings.case_timeout);
    }
}

fn run_case(
    client: &mut BridgeProbeClient,
    name: &str,
    motors: [f32; 4],
    sequence: &mut Sequence,
    frames_per_case: usize,
    timeout: Duration,
) -> CaseObservation {
    let line = format_gazebo_actuator_line(sequence.next(), None, motors);
    client.send_actuator(&line);
    let frames = client.collect_frames(frames_per_case, timeout);
    let observation = observe(&frames);
    eprintln!(
        "G0 {name}: motors={motors:?} mean_gyro={:?} max_abs_gyro={:.4} mean_vd={:.4}",
        observation.mean_gyro_rps, observation.max_abs_gyro_rps, observation.mean_vd_mps
    );
    observation
}

fn runtime_expectation_error(
    settings: &RuntimeSettings,
    case: GazeboActuatorTruthCase,
    baseline: CaseObservation,
    observed: CaseObservation,
) -> Result<(), String> {
    let delta = [
        observed.mean_gyro_rps[0] - baseline.mean_gyro_rps[0],
        observed.mean_gyro_rps[1] - baseline.mean_gyro_rps[1],
        observed.mean_gyro_rps[2] - baseline.mean_gyro_rps[2],
    ];

    match case.expectation {
        GazeboActuatorExpectation::AngularRateNearZero => {
            if observed.max_abs_gyro_rps > settings.zero_max_gyro_rps {
                return Err(format!(
                    "case {} expected near-zero angular rate <= {}, observed {:?}",
                    case.name, settings.zero_max_gyro_rps, observed
                ));
            }
        }
        GazeboActuatorExpectation::AxisSign { axis, positive } => {
            axis_sign_error(settings, case.name, axis, positive, delta)?;
        }
        GazeboActuatorExpectation::MotorBump {
            motor,
            roll_positive,
            pitch_positive,
            yaw_positive,
        } => {
            axis_sign_error(
                settings,
                case.name,
                GazeboActuatorAxis::Roll,
                roll_positive,
                delta,
            )?;
            axis_sign_error(
                settings,
                case.name,
                GazeboActuatorAxis::Pitch,
                pitch_positive,
                delta,
            )?;
            axis_sign_error(
                settings,
                case.name,
                GazeboActuatorAxis::Yaw,
                yaw_positive,
                delta,
            )?;
            assert!((1..=4).contains(&motor));
        }
    }

    Ok(())
}

fn axis_sign_error(
    settings: &RuntimeSettings,
    case_name: &str,
    axis: GazeboActuatorAxis,
    positive: bool,
    delta: [f32; 3],
) -> Result<(), String> {
    let index = match axis {
        GazeboActuatorAxis::Roll => 0,
        GazeboActuatorAxis::Pitch => 1,
        GazeboActuatorAxis::Yaw => 2,
    };
    let value = delta[index];
    let passed = if positive {
        value >= settings.min_axis_rate_delta_rps
    } else {
        value <= -settings.min_axis_rate_delta_rps
    };
    if passed {
        Ok(())
    } else {
        Err(format!(
            "case {case_name} expected {axis:?} delta sign positive={positive} with |delta| >= {}, got delta={delta:?}",
            settings.min_axis_rate_delta_rps
        ))
    }
}

fn observe(frames: &[GazeboSensorLine]) -> CaseObservation {
    let mut sum_gyro = [0.0; 3];
    let mut max_abs_gyro = 0.0_f32;
    let mut sum_vd = 0.0_f32;
    for frame in frames {
        for (sum, gyro) in sum_gyro.iter_mut().zip(frame.gyro_rps) {
            *sum += gyro;
            max_abs_gyro = max_abs_gyro.max(gyro.abs());
        }
        sum_vd += frame.vel_ned_mps[2];
    }

    let count = frames.len() as f32;
    CaseObservation {
        mean_gyro_rps: sum_gyro.map(|value| value / count),
        max_abs_gyro_rps: max_abs_gyro,
        mean_vd_mps: sum_vd / count,
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

fn assert_case(case: &GazeboActuatorTruthCase, name: &str, motors: [f32; 4]) {
    assert_eq!(case.name, name);
    assert_eq!(case.motors, motors);
}

fn assert_case_near(case: &GazeboActuatorTruthCase, name: &str, motors: [f32; 4]) {
    assert_eq!(case.name, name);
    for (actual, expected) in case.motors.into_iter().zip(motors) {
        assert!(
            (actual - expected).abs() < 0.000_1,
            "expected motor command {actual} to be near {expected}"
        );
    }
}

fn assert_speeds_near(actual: [f64; 4], expected: [f64; 4]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert!(
            (actual - expected).abs() < 0.000_1,
            "expected speed {actual} to be near {expected}"
        );
    }
}

fn gazebo_g0_actuator_runtime_cases() -> Vec<GazeboActuatorTruthCase> {
    gazebo_g0_actuator_truth_cases(&airframe_config())
}

fn selected_runtime_cases(selected: &str) -> Vec<GazeboActuatorTruthCase> {
    let cases = gazebo_g0_actuator_runtime_cases();
    if selected == "all" {
        return cases;
    }
    let selected = match selected {
        "motor_1_single" => "motor_1_bump",
        "motor_2_single" => "motor_2_bump",
        "motor_3_single" => "motor_3_bump",
        "motor_4_single" => "motor_4_bump",
        other => other,
    };

    cases
        .into_iter()
        .find(|case| case.name == selected)
        .map(|case| vec![case])
        .unwrap_or_else(|| {
            let expected = gazebo_g0_actuator_runtime_cases()
                .iter()
                .map(|case| case.name)
                .collect::<Vec<_>>()
                .join(", ");
            panic!("unknown MARV_GAZEBO_G0_CASE={selected:?}; expected one of: all, {expected}");
        })
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

fn env_usize(name: &str) -> Option<usize> {
    env::var(name).ok()?.parse().ok()
}

fn env_u64(name: &str) -> Option<u64> {
    env::var(name).ok()?.parse().ok()
}

fn env_f32(name: &str) -> Option<f32> {
    env::var(name).ok()?.parse().ok()
}

fn env_bool(name: &str) -> Option<bool> {
    match env::var(name).ok()?.as_str() {
        "1" | "true" | "TRUE" | "yes" | "YES" | "on" | "ON" => Some(true),
        "0" | "false" | "FALSE" | "no" | "NO" | "off" | "OFF" => Some(false),
        _ => None,
    }
}
