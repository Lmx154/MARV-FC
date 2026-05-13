mod support;

use std::{
    io::{BufRead, BufReader, Write},
    net::TcpStream,
    time::{Duration, Instant},
};

use deterministic_harness::{
    GAZEBO_G0_DEFAULT_ENDPOINT, GazeboActuatorAxis, GazeboActuatorExpectation,
    GazeboActuatorTruthCase, GazeboAirframeConfig, GazeboSensorLine, GazeboSimControlAction,
    format_gazebo_actuator_line, format_gazebo_sim_control_line, gazebo_g0_actuator_truth_cases,
};
use support::{
    AcceptanceThresholds, CommandKind, CommandSchedule, Gate, ScenarioMetrics, ScenarioReport,
    ScenarioSpec, ScheduledCommand,
};

const AIRFRAME_CONFIG: &str = include_str!("../config/airframes/f450_xing2_2809_1045_4s_v0.cfg");
const LIVE_CASE_NAMES: [&str; 12] = [
    "zero",
    "hover",
    "motor_1_bump",
    "motor_2_bump",
    "motor_3_bump",
    "motor_4_bump",
    "roll_positive",
    "roll_negative",
    "pitch_positive",
    "pitch_negative",
    "yaw_positive",
    "yaw_negative",
];
const LIVE_CONNECT_TIMEOUT: Duration = Duration::from_secs(2);
const LIVE_READ_TIMEOUT: Duration = Duration::from_millis(250);
const LIVE_ACK_TIMEOUT: Duration = Duration::from_secs(2);
const LIVE_SENSOR_TIMEOUT: Duration = Duration::from_secs(4);

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g0_actuator_truth_cases_cover_reset_hover_motor_bumps_and_axis_basis() {
    let airframe = airframe_config();
    let cases = gazebo_g0_actuator_truth_cases(&airframe);

    assert_eq!(cases.len(), 12);
    assert_eq!(cases[0].name, "zero");
    assert_eq!(cases[1].name, "hover");
    assert_eq!(
        cases[1].expectation,
        GazeboActuatorExpectation::AngularRateNearZero
    );
    assert_axis_case(&cases, "roll_positive", GazeboActuatorAxis::Roll, true);
    assert_axis_case(&cases, "roll_negative", GazeboActuatorAxis::Roll, false);
    assert_axis_case(&cases, "pitch_positive", GazeboActuatorAxis::Pitch, true);
    assert_axis_case(&cases, "pitch_negative", GazeboActuatorAxis::Pitch, false);
    assert_axis_case(&cases, "yaw_positive", GazeboActuatorAxis::Yaw, true);
    assert_axis_case(&cases, "yaw_negative", GazeboActuatorAxis::Yaw, false);

    for case in &cases {
        assert!(
            case.motors.iter().all(|motor| (0.0..=1.0).contains(motor)),
            "{} motor command must stay normalized",
            case.name
        );
        assert!(
            format_gazebo_actuator_line(7, Some(140_000), case.motors)
                .starts_with("ACTUATOR seq=7 sim_time_us=140000"),
            "{} must format through the active bridge actuator protocol",
            case.name
        );
    }

    assert_eq!(
        case_by_name(&cases, "motor_1_bump").expectation,
        GazeboActuatorExpectation::MotorBump {
            motor: 1,
            roll_positive: false,
            pitch_positive: true,
            yaw_positive: true,
        }
    );
}

#[test]
#[ignore = "live Gazebo runtime gate: connects to the bridge, resets the sim, sends actuator commands, and scores SENSOR gyro evidence"]
fn g0_live_actuator_truth_drives_bridge_and_scores_sensor_motion() {
    let airframe = airframe_config();
    let cases = gazebo_g0_actuator_truth_cases(&airframe);
    let endpoint =
        std::env::var("MARV_GAZEBO_ENDPOINT").unwrap_or_else(|_| GAZEBO_G0_DEFAULT_ENDPOINT.into());
    let mut bridge = LiveGazeboBridge::connect(&endpoint).unwrap_or_else(|error| {
        panic!("failed to connect to Gazebo bridge at {endpoint}: {error}")
    });
    let mut reports = Vec::new();

    for name in LIVE_CASE_NAMES {
        let case = *case_by_name(&cases, name);
        let sensor_frames = bridge
            .run_actuator_case(case)
            .unwrap_or_else(|error| panic!("live G0 case {name} failed: {error}"));
        println!(
            "G0 live {name} gyro_peak roll={:.4} pitch={:.4} yaw={:.4}",
            dominant_axis_peak(&sensor_frames, 0),
            dominant_axis_peak(&sensor_frames, 1),
            dominant_axis_peak(&sensor_frames, 2)
        );
        let report = score_live_case(case, &sensor_frames);
        println!("{}", report.to_json_line());
        reports.push(report);
    }

    assert!(
        reports.iter().all(|report| report.pass),
        "one or more live G0 actuator truth cases failed: {:?}",
        reports
    );
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g0_report_shape_identifies_reset_and_actuator_failures() {
    let spec = ScenarioSpec {
        id: "g0_hover_angular_rate_zero",
        gate: Gate::G0,
        description: "equal hover motors should not create large body rates",
        reset_required: true,
        command_schedule: CommandSchedule::new(vec![
            ScheduledCommand {
                at_frame: 0,
                kind: CommandKind::SimReset,
            },
            ScheduledCommand {
                at_frame: 5,
                kind: CommandKind::ActuatorMotors,
            },
        ]),
        thresholds: AcceptanceThresholds {
            max_cross_track_error_m: 0.0,
            max_altitude_error_m: 2.0,
            max_roll_pitch_rad: 0.05,
            max_speed_error_mps: 1.5,
            max_clamp_ratio: 0.0,
        },
    };

    let pass = ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean: true,
            estimator_agrees: true,
            max_cross_track_error_m: 0.0,
            max_altitude_error_m: 0.4,
            max_roll_pitch_rad: 0.02,
            max_speed_error_mps: 0.3,
            clamp_ratio: 0.0,
            first_clamp_source: None,
        },
    );
    assert!(pass.pass);
    assert_eq!(
        ScenarioReport::csv_header().split(',').count(),
        pass.to_csv_row().split(',').count()
    );
    assert!(
        pass.to_json_line()
            .contains("\"id\":\"g0_hover_angular_rate_zero\"")
    );

    let reset_fail = ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean: false,
            estimator_agrees: true,
            max_cross_track_error_m: 0.0,
            max_altitude_error_m: 0.0,
            max_roll_pitch_rad: 0.0,
            max_speed_error_mps: 0.0,
            clamp_ratio: 0.0,
            first_clamp_source: None,
        },
    );
    assert_eq!(reset_fail.failure_layer, Some(support::FailureLayer::Reset));

    let actuator_fail = ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean: true,
            estimator_agrees: true,
            max_cross_track_error_m: 0.0,
            max_altitude_error_m: 0.0,
            max_roll_pitch_rad: 0.0,
            max_speed_error_mps: 0.0,
            clamp_ratio: 0.25,
            first_clamp_source: Some("motor_output_limit".to_string()),
        },
    );
    assert_eq!(
        actuator_fail.failure_layer,
        Some(support::FailureLayer::Actuator)
    );
}

fn airframe_config() -> GazeboAirframeConfig {
    let airframe = GazeboAirframeConfig::parse(AIRFRAME_CONFIG).expect("airframe config parses");
    airframe
        .validate()
        .expect("airframe config is physically self-consistent");
    airframe
}

fn assert_axis_case(
    cases: &[deterministic_harness::GazeboActuatorTruthCase],
    name: &str,
    axis: GazeboActuatorAxis,
    positive: bool,
) {
    assert_eq!(
        case_by_name(cases, name).expectation,
        GazeboActuatorExpectation::AxisSign { axis, positive }
    );
}

fn case_by_name<'a>(
    cases: &'a [deterministic_harness::GazeboActuatorTruthCase],
    name: &str,
) -> &'a deterministic_harness::GazeboActuatorTruthCase {
    cases
        .iter()
        .find(|case| case.name == name)
        .unwrap_or_else(|| panic!("missing G0 actuator truth case {name}"))
}

struct LiveGazeboBridge {
    writer: TcpStream,
    reader: BufReader<TcpStream>,
    sequence: u64,
}

impl LiveGazeboBridge {
    fn connect(endpoint: &str) -> std::io::Result<Self> {
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

    fn run_actuator_case(
        &mut self,
        case: GazeboActuatorTruthCase,
    ) -> Result<Vec<GazeboSensorLine>, String> {
        self.send_sim_control(GazeboSimControlAction::Reset)?;
        self.send_sim_control(GazeboSimControlAction::Play)?;
        let settle = self.collect_sensor_frames(8, LIVE_SENSOR_TIMEOUT)?;
        let sim_time_us = settle.last().map(|frame| frame.sim_time_us);

        self.send_actuator(case.motors, sim_time_us)?;
        let measured = self.collect_sensor_frames(12, LIVE_SENSOR_TIMEOUT)?;
        self.send_actuator([0.0; 4], measured.last().map(|frame| frame.sim_time_us))?;

        Ok(measured)
    }

    fn send_sim_control(&mut self, action: GazeboSimControlAction) -> Result<(), String> {
        let sequence = self.next_sequence();
        self.send_raw(&format_gazebo_sim_control_line(sequence, action))?;
        self.wait_for_ack(sequence, action)
    }

    fn send_actuator(&mut self, motors: [f32; 4], sim_time_us: Option<u64>) -> Result<(), String> {
        let sequence = self.next_sequence();
        self.send_raw(&format_gazebo_actuator_line(sequence, sim_time_us, motors))
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

    fn collect_sensor_frames(
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

fn score_live_case(case: GazeboActuatorTruthCase, frames: &[GazeboSensorLine]) -> ScenarioReport {
    let spec = live_case_spec(case.name);
    let reset_clean = frames_are_monotonic_gazebo_sensor_data(frames);
    let max_abs_gyro = frames
        .iter()
        .map(|frame| {
            frame.gyro_rps[0]
                .abs()
                .max(frame.gyro_rps[1].abs())
                .max(frame.gyro_rps[2].abs())
        })
        .fold(0.0_f32, f32::max);

    let axis_result = match case.expectation {
        GazeboActuatorExpectation::AngularRateNearZero => max_abs_gyro <= 0.40,
        GazeboActuatorExpectation::AxisSign { axis, positive } => {
            let axis_index = axis_index(axis);
            let peak = dominant_axis_peak(frames, axis_index);
            peak.abs() >= 0.015 && (peak > 0.0) == positive
        }
        GazeboActuatorExpectation::MotorBump {
            roll_positive,
            pitch_positive,
            yaw_positive,
            ..
        } => {
            let roll_peak = dominant_axis_peak(frames, 0);
            let pitch_peak = dominant_axis_peak(frames, 1);
            let yaw_peak = dominant_axis_peak(frames, 2);
            roll_peak.abs() >= 0.015
                && pitch_peak.abs() >= 0.015
                && yaw_peak.abs() >= 0.015
                && (roll_peak > 0.0) == roll_positive
                && (pitch_peak > 0.0) == pitch_positive
                && (yaw_peak > 0.0) == yaw_positive
        }
    };

    ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean,
            estimator_agrees: true,
            max_cross_track_error_m: 0.0,
            max_altitude_error_m: 0.0,
            max_roll_pitch_rad: max_abs_gyro.min(10.0),
            max_speed_error_mps: 0.0,
            clamp_ratio: if axis_result { 0.0 } else { 1.0 },
            first_clamp_source: (!axis_result).then(|| "gazebo_actuator_truth".to_string()),
        },
    )
}

fn live_case_spec(name: &'static str) -> ScenarioSpec {
    ScenarioSpec {
        id: name,
        gate: Gate::G0,
        description: "live Gazebo actuator truth case scored from SENSOR gyro evidence",
        reset_required: true,
        command_schedule: CommandSchedule::new(vec![
            ScheduledCommand {
                at_frame: 0,
                kind: CommandKind::SimReset,
            },
            ScheduledCommand {
                at_frame: 8,
                kind: CommandKind::SimPlay,
            },
            ScheduledCommand {
                at_frame: 16,
                kind: CommandKind::ActuatorMotors,
            },
        ]),
        thresholds: AcceptanceThresholds {
            max_cross_track_error_m: 0.25,
            max_altitude_error_m: 2.0,
            max_roll_pitch_rad: 10.0,
            max_speed_error_mps: 10.0,
            max_clamp_ratio: 0.0,
        },
    }
}

fn frames_are_monotonic_gazebo_sensor_data(frames: &[GazeboSensorLine]) -> bool {
    !frames.is_empty()
        && frames
            .iter()
            .all(|frame| frame.clock_source.as_deref() == Some("gazebo"))
        && frames
            .windows(2)
            .all(|pair| pair[1].sim_time_us > pair[0].sim_time_us)
}

fn dominant_axis_peak(frames: &[GazeboSensorLine], axis: usize) -> f32 {
    frames
        .iter()
        .map(|frame| frame.gyro_rps[axis])
        .max_by(|left, right| left.abs().total_cmp(&right.abs()))
        .unwrap_or(0.0)
}

fn axis_index(axis: GazeboActuatorAxis) -> usize {
    match axis {
        GazeboActuatorAxis::Roll => 0,
        GazeboActuatorAxis::Pitch => 1,
        GazeboActuatorAxis::Yaw => 2,
    }
}
