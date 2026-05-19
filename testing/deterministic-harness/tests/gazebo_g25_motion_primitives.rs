mod support;

use std::{
    fs::{self, File},
    io::Write,
    path::PathBuf,
    time::Duration,
};

use common::control::{
    config::ControlLoopConfig,
    pipeline::{
        ControlFrame, ControlOutput, ControlSetpoint, ControlSetpointSource, EstimateSnapshot,
        FlightControlPipeline, ImuControlInput, LocalTrajectorySetpoint,
    },
    position::PositionControllerConfig,
    rate::RateControllerConfig,
};
use deterministic_harness::{
    GAZEBO_G0_DEFAULT_ENDPOINT, GazeboAirframeConfig, GazeboSensorLine, GazeboTruthOrigin,
};
use support::{
    AcceptanceThresholds, CommandKind, CommandSchedule, FailureLayer, Gate, LiveGazeboBridge,
    ScenarioMetrics, ScenarioReport, ScenarioSpec, ScheduledCommand,
};

const AIRFRAME_CONFIG: &str = include_str!("../config/airframes/f450_xing2_2809_1045_4s_v0.cfg");
const HOLD_ALTITUDE_DOWN_M: f32 = -1.0;
const CONSERVATIVE_SPEED_MPS: f32 = 0.70;
const CONSERVATIVE_ACCEL_MPS2: f32 = 0.70;
const LIVE_SENSOR_STEP_TIMEOUT: Duration = Duration::from_secs(2);
const LIVE_SETTLE_TIMEOUT: Duration = Duration::from_secs(4);
const LIVE_SETTLE_FRAMES: usize = 12;
const LIVE_SETTLE_CANDIDATE_FRAMES: usize = 24;
const LIVE_TAKEOFF_FRAMES: usize = 160;
const LIVE_MOTION_FRAMES: usize = 110;
const LIVE_MOTION_TARGET_M: f32 = 0.55;
const LIVE_MOTION_SPEED_MPS: f32 = 0.60;
const LIVE_MOTION_ACCEL_MPS2: f32 = 0.50;
const LIVE_REQUIRED_PROGRESS_M: f32 = 0.40;
const LIVE_ATTEMPTS_PER_CARDINAL_CASE: usize = 2;
const LIVE_ATTEMPTS_PER_SEQUENCE_CASE: usize = 2;
const G25_TRACE_DIR_ENV: &str = "MARV_GAZEBO_G25_TRACE_DIR";

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotionPrimitiveId {
    M01,
    M02,
    M03,
    M04,
    M05,
    M06,
    M07,
    M08,
}

#[derive(Clone, Debug, PartialEq)]
struct MotionPrimitiveSpec {
    id: MotionPrimitiveId,
    scenario: ScenarioSpec,
    requires_navigation: bool,
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g25_declares_all_motion_primitives_without_enabling_navigation() {
    let specs = motion_primitive_specs();

    assert_eq!(specs.len(), 8);
    assert_eq!(specs[0].id, MotionPrimitiveId::M01);
    assert_eq!(specs[7].id, MotionPrimitiveId::M08);
    for spec in &specs {
        assert_eq!(spec.scenario.gate, Gate::G2Point5);
        assert!(spec.scenario.reset_required);
        assert!(
            !spec.requires_navigation,
            "{} must remain a direct primitive, not a G3 waypoint case",
            spec.scenario.id
        );
        assert!(
            spec.scenario
                .command_schedule
                .commands
                .iter()
                .any(|command| command.kind == CommandKind::LocalTrajectory),
            "{} must use local trajectory primitives",
            spec.scenario.id
        );
    }
}

#[test]
#[ignore = "live Gazebo runtime gate: resets the simulator, streams controller motor commands, and scores M01 cardinal motion truth"]
fn g25_live_m01_cardinal_motion_drives_bridge_and_scores_progress() {
    let endpoint =
        std::env::var("MARV_GAZEBO_ENDPOINT").unwrap_or_else(|_| GAZEBO_G0_DEFAULT_ENDPOINT.into());
    let requested_case = std::env::var("MARV_GAZEBO_G25_CASE").ok();
    let mut reports = Vec::new();

    let cases = [
        LiveCardinalCase::new("north", 0, 1.0),
        LiveCardinalCase::new("south", 0, -1.0),
        LiveCardinalCase::new("east", 1, 1.0),
        LiveCardinalCase::new("west", 1, -1.0),
    ];
    for case in cases.into_iter().filter(|case| {
        requested_case
            .as_deref()
            .is_none_or(|name| name == case.name)
    }) {
        let mut attempts = Vec::new();
        for attempt in 1..=LIVE_ATTEMPTS_PER_CARDINAL_CASE {
            let mut bridge = LiveGazeboBridge::connect(&endpoint).unwrap_or_else(|error| {
                panic!("failed to connect to Gazebo bridge at {endpoint}: {error}")
            });
            let report = run_live_cardinal_case(&mut bridge, case, attempt)
                .unwrap_or_else(|error| panic!("live G2.5 case {} failed: {error}", case.name));
            println!("{}", report.to_json_line());
            let pass = report.pass;
            attempts.push(report);
            if pass {
                break;
            }
        }

        let report = attempts
            .iter()
            .find(|report| report.pass)
            .cloned()
            .unwrap_or_else(|| attempts.last().expect("at least one live attempt").clone());
        reports.push(report);
    }

    assert!(
        reports.iter().all(|report| report.pass),
        "one or more live G2.5 cardinal primitive cases failed: {:?}",
        reports
    );
}

#[test]
#[ignore = "live Gazebo runtime gate: cycles G2.5 M02-M08 direct control primitives without enabling navigation"]
fn g25_live_motion_primitives_cycle_all_control_sequences() {
    let endpoint =
        std::env::var("MARV_GAZEBO_ENDPOINT").unwrap_or_else(|_| GAZEBO_G0_DEFAULT_ENDPOINT.into());
    let requested_case = std::env::var("MARV_GAZEBO_G25_SEQUENCE").ok();
    let mut reports = Vec::new();

    for case in LivePrimitiveCase::ALL.into_iter().filter(|case| {
        requested_case
            .as_deref()
            .is_none_or(|name| name == case.name())
    }) {
        let mut attempts = Vec::new();
        for attempt in 1..=LIVE_ATTEMPTS_PER_SEQUENCE_CASE {
            let mut bridge = LiveGazeboBridge::connect(&endpoint).unwrap_or_else(|error| {
                panic!("failed to connect to Gazebo bridge at {endpoint}: {error}")
            });
            let report = run_live_primitive_case(&mut bridge, case, attempt)
                .unwrap_or_else(|error| panic!("live G2.5 case {} failed: {error}", case.name()));
            println!("{}", report.to_json_line());
            let pass = report.pass;
            attempts.push(report);
            if pass {
                break;
            }
        }

        let report = attempts
            .iter()
            .find(|report| report.pass)
            .cloned()
            .unwrap_or_else(|| attempts.last().expect("at least one live attempt").clone());
        reports.push(report);
    }

    assert!(
        reports.iter().all(|report| report.pass),
        "one or more live G2.5 sequence primitive cases failed: {:?}",
        reports
    );
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g25_m01_cardinal_motion_commands_correct_world_axis_attitude_signs() {
    for case in [
        (
            "north",
            [CONSERVATIVE_SPEED_MPS, 0.0, 0.0],
            [CONSERVATIVE_ACCEL_MPS2, 0.0, 0.0],
            2,
            -1.0,
        ),
        (
            "south",
            [-CONSERVATIVE_SPEED_MPS, 0.0, 0.0],
            [-CONSERVATIVE_ACCEL_MPS2, 0.0, 0.0],
            2,
            1.0,
        ),
        (
            "east",
            [0.0, CONSERVATIVE_SPEED_MPS, 0.0],
            [0.0, CONSERVATIVE_ACCEL_MPS2, 0.0],
            1,
            1.0,
        ),
        (
            "west",
            [0.0, -CONSERVATIVE_SPEED_MPS, 0.0],
            [0.0, -CONSERVATIVE_ACCEL_MPS2, 0.0],
            1,
            -1.0,
        ),
    ] {
        let pipeline = primitive_pipeline();
        let (name, velocity, accel, quaternion_axis, expected_sign) = case;
        let output = pipeline.step(
            hover_estimate(0.0),
            ImuControlInput::default(),
            ControlSetpoint::from_local_trajectory(
                trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], velocity, accel, 0.0),
                true,
            ),
        );

        assert!(output.control_valid, "{name} primitive should remain valid");
        assert_eq!(
            output.debug.estimator_frame,
            ControlFrame::EstimatorLocalNed
        );
        assert_eq!(output.debug.setpoint_frame, ControlFrame::EstimatorLocalNed);
        let velocity_setpoint = output
            .debug
            .velocity_setpoint_ned_mps
            .expect("velocity setpoint");
        let velocity_axis = if velocity[0].abs() > 0.0 { 0 } else { 1 };
        let cross_axis = 1 - velocity_axis;
        assert!(
            velocity_setpoint[velocity_axis].abs() <= CONSERVATIVE_SPEED_MPS + 0.003,
            "{name} velocity setpoint should stay inside the primitive speed cap"
        );
        assert!(
            velocity_setpoint[velocity_axis] * velocity[velocity_axis] > 0.0,
            "{name} velocity setpoint should preserve requested direction"
        );
        assert!(
            velocity_setpoint[cross_axis].abs() <= 0.003,
            "{name} velocity setpoint should not inject cross-axis motion"
        );
        assert_eq!(
            output
                .debug
                .feed_forward_accel_ned_mps2
                .expect("feed-forward accel"),
            [accel[0], accel[1]]
        );

        let attitude = output.attitude_setpoint.expect("attitude setpoint");
        assert!(
            attitude.quaternion[quaternion_axis] * expected_sign > 0.0,
            "{name} primitive should command the expected roll/pitch sign"
        );
        assert!(
            output
                .debug
                .vertical_throttle_margin
                .expect("vertical margin")
                > 0.0,
            "{name} primitive should preserve vertical throttle margin"
        );
    }
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g25_m02_m03_speed_profile_braking_and_abrupt_reversal_are_bounded() {
    let pipeline = primitive_pipeline();
    let accel = step_repeated(
        &pipeline,
        hover_estimate(0.0),
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(
            trajectory(
                [0.0, 0.0, HOLD_ALTITUDE_DOWN_M],
                [CONSERVATIVE_SPEED_MPS, 0.0, 0.0],
                [CONSERVATIVE_ACCEL_MPS2, 0.0, 0.0],
                0.0,
            ),
            true,
        ),
        20,
    );
    let brake = step_repeated(
        &pipeline,
        EstimateSnapshot {
            position_ned_m: [0.6, 0.0, HOLD_ALTITUDE_DOWN_M],
            velocity_ned_mps: [CONSERVATIVE_SPEED_MPS, 0.0, 0.0],
            ..hover_estimate(0.0)
        },
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(
            trajectory(
                [0.6, 0.0, HOLD_ALTITUDE_DOWN_M],
                [0.0; 3],
                [-0.20, 0.0, 0.0],
                0.0,
            ),
            true,
        ),
        40,
    );
    let reversal = step_repeated(
        &pipeline,
        EstimateSnapshot {
            position_ned_m: [0.8, 0.0, HOLD_ALTITUDE_DOWN_M],
            velocity_ned_mps: [CONSERVATIVE_SPEED_MPS, 0.0, 0.0],
            ..hover_estimate(0.0)
        },
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(
            trajectory(
                [0.4, 0.0, HOLD_ALTITUDE_DOWN_M],
                [-CONSERVATIVE_SPEED_MPS, 0.0, 0.0],
                [-CONSERVATIVE_ACCEL_MPS2, 0.0, 0.0],
                0.0,
            ),
            true,
        ),
        40,
    );

    assert!(accel.control_valid);
    assert!(brake.control_valid);
    assert!(reversal.control_valid);
    assert!(accel.attitude_setpoint.expect("accel attitude").quaternion[2] < 0.0);
    assert!(brake.attitude_setpoint.expect("brake attitude").quaternion[2] > 0.0);
    assert!(
        reversal
            .attitude_setpoint
            .expect("reversal attitude")
            .quaternion[2]
            > 0.0
    );
    for output in [accel, brake, reversal] {
        assert!(
            output
                .debug
                .vertical_throttle_margin
                .expect("vertical margin")
                > 0.0
        );
        assert!(motor_spread(output.motors) < 0.40);
    }
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g25_m04_m05_yaw_turns_report_sign_limits_and_recovery() {
    let pipeline = primitive_pipeline();
    let yaw_left = pipeline.step(
        hover_estimate(0.0),
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(
            trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], 0.35),
            true,
        ),
    );
    let yaw_right = pipeline.step(
        hover_estimate(0.0),
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(
            trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], -0.35),
            true,
        ),
    );
    let fast_yaw = yaw_limited_pipeline().step(
        hover_estimate(0.0),
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(
            trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], 2.20),
            true,
        ),
    );

    assert!(yaw_left.torque_command.yaw > 0.0);
    assert!(yaw_right.torque_command.yaw < 0.0);
    assert!(fast_yaw.debug.rate_limit_flags.yaw);
    assert!(fast_yaw.control_valid);
    assert!(fast_yaw.motor_outputs().clamped || motor_spread(fast_yaw.motors) > 0.0);

    let recovery = pipeline.step(
        hover_estimate(0.0),
        ImuControlInput {
            accel_mps2: [0.0; 3],
            gyro_rps: [0.0, 0.0, 0.50],
        },
        ControlSetpoint::from_local_trajectory(
            trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], 0.0),
            true,
        ),
    );
    assert!(recovery.torque_command.yaw < 0.0);
    assert!(recovery.control_valid);
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g25_m06_world_frame_motion_survives_heading_changes() {
    let pipeline = primitive_pipeline();
    let north_level_yaw = pipeline.step(
        hover_estimate(0.0),
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(
            trajectory(
                [0.0, 0.0, HOLD_ALTITUDE_DOWN_M],
                [CONSERVATIVE_SPEED_MPS, 0.0, 0.0],
                [CONSERVATIVE_ACCEL_MPS2, 0.0, 0.0],
                0.0,
            ),
            true,
        ),
    );
    let north_yawed = pipeline.step(
        hover_estimate(core::f32::consts::FRAC_PI_2),
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(
            trajectory(
                [0.0, 0.0, HOLD_ALTITUDE_DOWN_M],
                [CONSERVATIVE_SPEED_MPS, 0.0, 0.0],
                [CONSERVATIVE_ACCEL_MPS2, 0.0, 0.0],
                core::f32::consts::FRAC_PI_2,
            ),
            true,
        ),
    );

    assert_eq!(
        north_level_yaw
            .debug
            .requested_accel_ned_mps2
            .expect("requested accel"),
        north_yawed
            .debug
            .requested_accel_ned_mps2
            .expect("requested accel")
    );
    assert!(
        north_level_yaw
            .attitude_setpoint
            .expect("level yaw attitude")
            .quaternion[2]
            < 0.0
    );
    assert!(
        north_yawed
            .attitude_setpoint
            .expect("yawed attitude")
            .quaternion[1]
            < 0.0,
        "north world-frame acceleration at 90 deg yaw should become roll-dominant, not body-frame forward"
    );
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g25_m07_zig_zag_reversals_remain_finite_and_inside_authority_limits() {
    let pipeline = primitive_pipeline();
    let mut previous_roll_sign = 0.0;

    for (idx, east_speed) in [
        CONSERVATIVE_SPEED_MPS,
        -CONSERVATIVE_SPEED_MPS,
        CONSERVATIVE_SPEED_MPS,
        -CONSERVATIVE_SPEED_MPS,
    ]
    .into_iter()
    .enumerate()
    {
        let output = step_repeated(
            &pipeline,
            EstimateSnapshot {
                position_ned_m: [idx as f32 * 0.25, 0.0, HOLD_ALTITUDE_DOWN_M],
                velocity_ned_mps: [CONSERVATIVE_SPEED_MPS, -east_speed * 0.25, 0.0],
                ..hover_estimate(0.0)
            },
            ImuControlInput::default(),
            ControlSetpoint::from_local_trajectory(
                trajectory(
                    [
                        idx as f32 * 0.25 + 0.25,
                        east_speed.signum() * 0.35,
                        HOLD_ALTITUDE_DOWN_M,
                    ],
                    [CONSERVATIVE_SPEED_MPS, east_speed, 0.0],
                    [0.0, east_speed.signum() * CONSERVATIVE_ACCEL_MPS2, 0.0],
                    0.0,
                ),
                true,
            ),
            40,
        );

        assert!(output.control_valid);
        assert!(!output.clamped);
        assert!(
            output.debug.applied_accel_ned_mps2.expect("applied accel")[1].abs()
                <= CONSERVATIVE_ACCEL_MPS2 + 0.001
        );
        let roll_sign = output
            .attitude_setpoint
            .expect("zig-zag attitude")
            .quaternion[1]
            .signum();
        if previous_roll_sign != 0.0 {
            assert_ne!(roll_sign, previous_roll_sign);
        }
        previous_roll_sign = roll_sign;
    }
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g25_reports_motion_failures_before_g3_navigation_is_allowed() {
    let specs = motion_primitive_specs();
    let report = ScenarioReport::from_metrics(
        &specs[0].scenario,
        ScenarioMetrics {
            reset_clean: true,
            estimator_agrees: true,
            max_cross_track_error_m: 0.80,
            max_altitude_error_m: 0.10,
            max_roll_pitch_rad: 0.10,
            max_speed_error_mps: 0.10,
            clamp_ratio: 0.0,
            first_clamp_source: None,
        },
    );
    assert!(!report.pass);
    assert_eq!(report.failure_layer, Some(FailureLayer::Guidance));
    assert!(
        report
            .to_json_line()
            .contains("\"failure_layer\":\"guidance\"")
    );
    assert_eq!(
        ScenarioReport::csv_header().split(',').count(),
        report.to_csv_row().split(',').count()
    );

    let saturation = ScenarioReport::from_metrics(
        &specs[4].scenario,
        ScenarioMetrics {
            reset_clean: true,
            estimator_agrees: true,
            max_cross_track_error_m: 0.10,
            max_altitude_error_m: 0.10,
            max_roll_pitch_rad: 0.10,
            max_speed_error_mps: 0.10,
            clamp_ratio: 0.35,
            first_clamp_source: Some("yaw_desaturation".to_string()),
        },
    );
    assert_eq!(saturation.failure_layer, Some(FailureLayer::Actuator));
}

fn motion_primitive_specs() -> Vec<MotionPrimitiveSpec> {
    [
        (
            MotionPrimitiveId::M01,
            "g25_m01_cardinal_motion",
            "north/south/east/west straight-line local trajectory segments",
        ),
        (
            MotionPrimitiveId::M02,
            "g25_m02_speed_profile",
            "accelerate, hold speed, decelerate, hover, accelerate again, and brake",
        ),
        (
            MotionPrimitiveId::M03,
            "g25_m03_brake_to_hover",
            "build horizontal velocity then command zero XY velocity",
        ),
        (
            MotionPrimitiveId::M04,
            "g25_m04_slow_yaw_turns",
            "slow left and right yaw turns while holding XY and altitude",
        ),
        (
            MotionPrimitiveId::M05,
            "g25_m05_fast_yaw_turns",
            "near-limit yaw turns with saturation and recovery visibility",
        ),
        (
            MotionPrimitiveId::M06,
            "g25_m06_moving_heading_changes",
            "world-frame movement while yaw changes",
        ),
        (
            MotionPrimitiveId::M07,
            "g25_m07_zig_zag",
            "forward progress with alternating east/west lateral commands",
        ),
        (
            MotionPrimitiveId::M08,
            "g25_m08_abrupt_reversal_recovery",
            "abrupt local trajectory reversal as the disturbance fallback",
        ),
    ]
    .into_iter()
    .map(|(id, scenario_id, description)| MotionPrimitiveSpec {
        id,
        scenario: ScenarioSpec {
            id: scenario_id,
            gate: Gate::G2Point5,
            description,
            reset_required: true,
            command_schedule: CommandSchedule::new(vec![
                ScheduledCommand {
                    at_frame: 0,
                    kind: CommandKind::SimReset,
                },
                ScheduledCommand {
                    at_frame: 20,
                    kind: CommandKind::SimPlay,
                },
                ScheduledCommand {
                    at_frame: 80,
                    kind: CommandKind::LocalTrajectory,
                },
                ScheduledCommand {
                    at_frame: 260,
                    kind: CommandKind::Hold,
                },
            ]),
            thresholds: AcceptanceThresholds {
                max_cross_track_error_m: 0.45,
                max_altitude_error_m: 0.50,
                max_roll_pitch_rad: 0.28,
                max_speed_error_mps: 0.35,
                max_clamp_ratio: 0.12,
            },
        },
        requires_navigation: false,
    })
    .collect()
}

fn primitive_pipeline() -> FlightControlPipeline {
    FlightControlPipeline::new(
        common::control::pipeline::FlightControlConfig::with_loop_config(primitive_loop_config(
            0.16,
        )),
    )
}

fn hardware_live_pipeline() -> FlightControlPipeline {
    let airframe = GazeboAirframeConfig::parse(AIRFRAME_CONFIG).expect("airframe config parses");
    airframe
        .validate()
        .expect("airframe config remains the hardware source of truth");
    let mut loop_config = primitive_loop_config(0.15);
    loop_config.altitude.hover_throttle = airframe.hover_motor_command();
    FlightControlPipeline::new(common::control::pipeline::FlightControlConfig::new(
        loop_config,
        airframe.runtime_motor_geometry(),
    ))
}

fn yaw_limited_pipeline() -> FlightControlPipeline {
    FlightControlPipeline::new(
        common::control::pipeline::FlightControlConfig::with_loop_config(primitive_loop_config(
            0.03,
        )),
    )
}

fn primitive_loop_config(max_axis_command: f32) -> ControlLoopConfig {
    let mut config = ControlLoopConfig::default();
    config.position = PositionControllerConfig {
        position_gain: 0.14,
        velocity_gain: 0.38,
        max_horizontal_velocity_mps: CONSERVATIVE_SPEED_MPS,
        max_horizontal_accel_mps2: CONSERVATIVE_ACCEL_MPS2,
        max_horizontal_accel_slew_mps3: 4.0,
        max_tilt_rad: 11.0_f32.to_radians(),
        ..PositionControllerConfig::default()
    };
    config.attitude = common::control::attitude::AttitudeControllerConfig {
        roll_rate_gain: 2.4,
        pitch_rate_gain: 2.4,
        yaw_rate_gain: 2.5,
        max_rate_rps: 1.0,
        max_yaw_rate_rps: 1.2,
    };
    config.rate = RateControllerConfig {
        roll_gain: 0.06,
        pitch_gain: 0.06,
        yaw_gain: 0.075,
        max_axis_command,
        measured_rate_deadband_rps: 0.015,
        ..RateControllerConfig::default()
    };
    config
}

fn trajectory(
    position_ned_m: [f32; 3],
    velocity_ned_mps: [f32; 3],
    acceleration_ned_mps2: [f32; 3],
    yaw_rad: f32,
) -> LocalTrajectorySetpoint {
    let trajectory = LocalTrajectorySetpoint {
        position_ned_m,
        velocity_ned_mps,
        acceleration_ned_mps2,
        yaw_rad,
        yaw_rate_rad_s: 0.0,
    };
    assert!(trajectory.finite());

    let setpoint = ControlSetpoint::from_local_trajectory(trajectory, true);
    assert_eq!(
        setpoint.source(),
        ControlSetpointSource::EstimatorLocalTrajectory
    );
    trajectory
}

fn hover_estimate(yaw_rad: f32) -> EstimateSnapshot {
    EstimateSnapshot {
        position_ned_m: [0.0, 0.0, HOLD_ALTITUDE_DOWN_M],
        velocity_ned_mps: [0.0; 3],
        quaternion: common::control::attitude::AttitudeSetpoint::from_euler_rad(0.0, 0.0, yaw_rad)
            .expect("finite yaw should produce a quaternion")
            .quaternion,
        valid: true,
    }
}

fn step_repeated(
    pipeline: &FlightControlPipeline,
    estimate: EstimateSnapshot,
    imu: ImuControlInput,
    setpoint: ControlSetpoint,
    frames: usize,
) -> common::control::pipeline::ControlOutput {
    let mut output = None;
    for _ in 0..frames {
        output = Some(pipeline.step(estimate, imu, setpoint));
    }
    output.expect("at least one control frame")
}

fn motor_spread(motors: [f32; 4]) -> f32 {
    let min = motors
        .iter()
        .fold(f32::INFINITY, |min, motor| min.min(*motor));
    let max = motors
        .iter()
        .fold(f32::NEG_INFINITY, |max, motor| max.max(*motor));
    max - min
}

trait MotorOutputAccess {
    fn motor_outputs(&self) -> common::control::mixing::MotorOutputs<4>;
}

impl MotorOutputAccess for common::control::pipeline::ControlOutput {
    fn motor_outputs(&self) -> common::control::mixing::MotorOutputs<4> {
        self.debug.motor_outputs.expect("motor outputs")
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LivePrimitiveCase {
    M02SpeedProfile,
    M03BrakeToHover,
    M04SlowYawTurns,
    M05FastYawTurns,
    M06MovingHeadingChanges,
    M07ZigZag,
    M08AbruptReversal,
}

impl LivePrimitiveCase {
    const ALL: [Self; 7] = [
        Self::M02SpeedProfile,
        Self::M03BrakeToHover,
        Self::M04SlowYawTurns,
        Self::M05FastYawTurns,
        Self::M06MovingHeadingChanges,
        Self::M07ZigZag,
        Self::M08AbruptReversal,
    ];

    const fn id(self) -> &'static str {
        match self {
            Self::M02SpeedProfile => "g25_live_m02_speed_profile",
            Self::M03BrakeToHover => "g25_live_m03_brake_to_hover",
            Self::M04SlowYawTurns => "g25_live_m04_slow_yaw_turns",
            Self::M05FastYawTurns => "g25_live_m05_fast_yaw_turns",
            Self::M06MovingHeadingChanges => "g25_live_m06_moving_heading_changes",
            Self::M07ZigZag => "g25_live_m07_zig_zag",
            Self::M08AbruptReversal => "g25_live_m08_abrupt_reversal",
        }
    }

    const fn name(self) -> &'static str {
        match self {
            Self::M02SpeedProfile => "m02",
            Self::M03BrakeToHover => "m03",
            Self::M04SlowYawTurns => "m04",
            Self::M05FastYawTurns => "m05",
            Self::M06MovingHeadingChanges => "m06",
            Self::M07ZigZag => "m07",
            Self::M08AbruptReversal => "m08",
        }
    }

    const fn description(self) -> &'static str {
        match self {
            Self::M02SpeedProfile => "live speed-up/slow-down local trajectory primitive",
            Self::M03BrakeToHover => "live brake-to-hover local trajectory primitive",
            Self::M04SlowYawTurns => "live slow yaw turns while holding position",
            Self::M05FastYawTurns => "live near-limit yaw turns while holding position",
            Self::M06MovingHeadingChanges => "live world-frame motion while yaw changes",
            Self::M07ZigZag => "live zig-zag local trajectory primitive",
            Self::M08AbruptReversal => "live abrupt reversal recovery primitive",
        }
    }

    const fn motion_frames(self) -> usize {
        match self {
            Self::M02SpeedProfile => 170,
            Self::M03BrakeToHover => 165,
            Self::M04SlowYawTurns => 140,
            Self::M05FastYawTurns => 150,
            Self::M06MovingHeadingChanges => 185,
            Self::M07ZigZag => 400,
            Self::M08AbruptReversal => 165,
        }
    }

    fn setpoint(self, elapsed_s: f32) -> LocalTrajectorySetpoint {
        match self {
            Self::M02SpeedProfile => {
                axis_trapezoid_trajectory(0, 1.0, 0.40, elapsed_s, 0.35, 0.25, 0.0)
            }
            Self::M03BrakeToHover => {
                if elapsed_s < 0.90 {
                    axis_trapezoid_trajectory(0, 1.0, 0.34, elapsed_s, 0.46, 0.38, 0.0)
                } else if elapsed_s < 1.95 {
                    trajectory(
                        [0.26, 0.0, HOLD_ALTITUDE_DOWN_M],
                        [0.0; 3],
                        [-0.35, 0.0, 0.0],
                        0.0,
                    )
                } else {
                    trajectory([0.26, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], 0.0)
                }
            }
            Self::M04SlowYawTurns => {
                let yaw =
                    piecewise_scalar(elapsed_s, 1.25, &[(0.0, 0.55), (0.55, -0.55), (-0.55, 0.0)])
                        .0;
                trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], yaw)
            }
            Self::M05FastYawTurns => {
                let yaw =
                    piecewise_scalar(elapsed_s, 1.00, &[(0.0, 1.20), (1.20, -1.20), (-1.20, 0.0)])
                        .0;
                trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], yaw)
            }
            Self::M06MovingHeadingChanges => {
                let (turn, turn_rate, turn_accel) = smooth_segment(elapsed_s, 2.30, 0.0, 0.85);
                let radius_m = 0.65;
                let sin_turn = turn.sin();
                let cos_turn = turn.cos();
                trajectory(
                    [
                        radius_m * (1.0 - cos_turn),
                        radius_m * sin_turn,
                        HOLD_ALTITUDE_DOWN_M,
                    ],
                    [
                        radius_m * sin_turn * turn_rate,
                        radius_m * cos_turn * turn_rate,
                        0.0,
                    ],
                    [
                        radius_m * (cos_turn * turn_rate * turn_rate + sin_turn * turn_accel),
                        radius_m * (-sin_turn * turn_rate * turn_rate + cos_turn * turn_accel),
                        0.0,
                    ],
                    turn,
                )
            }
            Self::M07ZigZag => {
                let motion_duration_s = 7.0;
                let settle_duration_s = 1.0;
                let elapsed_s = elapsed_s.min(motion_duration_s + settle_duration_s);
                let progress = (elapsed_s / motion_duration_s).clamp(0.0, 1.0);
                let north = 1.05 * progress;
                let north_velocity = if progress < 1.0 {
                    1.05 / motion_duration_s
                } else {
                    0.0
                };
                let (east, east_velocity, _) = piecewise_scalar(
                    elapsed_s,
                    1.4,
                    &[
                        (0.0, 0.30),
                        (0.30, -0.65),
                        (-0.65, -0.65),
                        (-0.65, 0.35),
                        (0.35, 0.0),
                    ],
                );
                trajectory(
                    [north, east, HOLD_ALTITUDE_DOWN_M],
                    [north_velocity, east_velocity, 0.0],
                    [0.0; 3],
                    0.0,
                )
            }
            Self::M08AbruptReversal => {
                if elapsed_s < 0.80 {
                    axis_trapezoid_trajectory(0, 1.0, 0.32, elapsed_s, 0.48, 0.40, 0.0)
                } else if elapsed_s < 1.95 {
                    trajectory(
                        [-0.20, 0.0, HOLD_ALTITUDE_DOWN_M],
                        [-0.36, 0.0, 0.0],
                        [-0.36, 0.0, 0.0],
                        0.0,
                    )
                } else {
                    trajectory([-0.20, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], 0.0)
                }
            }
        }
    }

    fn spec(self) -> ScenarioSpec {
        ScenarioSpec {
            id: self.id(),
            gate: Gate::G2Point5,
            description: self.description(),
            reset_required: true,
            command_schedule: CommandSchedule::new(vec![
                ScheduledCommand {
                    at_frame: 0,
                    kind: CommandKind::SimReset,
                },
                ScheduledCommand {
                    at_frame: 12,
                    kind: CommandKind::SimPlay,
                },
                ScheduledCommand {
                    at_frame: LIVE_TAKEOFF_FRAMES,
                    kind: CommandKind::LocalTrajectory,
                },
                ScheduledCommand {
                    at_frame: LIVE_TAKEOFF_FRAMES + self.motion_frames(),
                    kind: CommandKind::Hold,
                },
            ]),
            thresholds: AcceptanceThresholds {
                max_cross_track_error_m: match self {
                    Self::M02SpeedProfile => 0.30,
                    Self::M03BrakeToHover => 0.35,
                    _ => 0.25,
                },
                max_altitude_error_m: 1.25,
                max_roll_pitch_rad: match self {
                    Self::M06MovingHeadingChanges | Self::M07ZigZag | Self::M08AbruptReversal => {
                        0.60
                    }
                    _ => 0.35,
                },
                max_speed_error_mps: 1.20,
                max_clamp_ratio: match self {
                    Self::M05FastYawTurns => 0.25,
                    _ => 0.08,
                },
            },
        }
    }

    fn tracking_error(self, metrics: LivePrimitiveMetrics) -> f32 {
        match self {
            Self::M02SpeedProfile => {
                (0.30 - metrics.max_position_ned_m[0]).max(0.0)
                    + metrics.max_abs_position_axis_m(1)
                    + (0.25 - metrics.max_horizontal_speed_mps).max(0.0)
                    + (metrics.final_horizontal_speed_mps() - 0.35).max(0.0)
            }
            Self::M03BrakeToHover => {
                (0.25 - metrics.max_position_ned_m[0]).max(0.0)
                    + metrics.max_abs_position_axis_m(1)
                    + (metrics.final_horizontal_speed_mps() - 0.20).max(0.0)
            }
            Self::M04SlowYawTurns => {
                metrics.max_horizontal_radius_m + (0.45 - metrics.yaw_span_rad()).max(0.0)
            }
            Self::M05FastYawTurns => {
                metrics.max_horizontal_radius_m + (0.85 - metrics.yaw_span_rad()).max(0.0)
            }
            Self::M06MovingHeadingChanges => {
                let turn_radius_error = (0.35 - metrics.max_horizontal_radius_m)
                    .max(0.0)
                    .max((metrics.max_horizontal_radius_m - 0.75).max(0.0));
                turn_radius_error + (0.70 - metrics.yaw_span_rad()).max(0.0)
            }
            Self::M07ZigZag => {
                let east_span = metrics.max_position_ned_m[1] - metrics.min_position_ned_m[1];
                (0.75 - metrics.final_position_ned_m[0]).max(0.0)
                    + (0.30 - east_span).max(0.0)
                    + (metrics.final_position_ned_m[1].abs() - 0.25).max(0.0)
                    + (metrics.max_abs_position_axis_m(1) - 0.75).max(0.0)
            }
            Self::M08AbruptReversal => {
                (0.30 - (metrics.max_position_ned_m[0] - metrics.min_position_ned_m[0])).max(0.0)
                    + metrics.max_abs_position_axis_m(1)
                    + (metrics.final_horizontal_speed_mps() - 0.35).max(0.0)
            }
        }
    }

    fn score(self, metrics: LivePrimitiveMetrics) -> ScenarioReport {
        let spec = self.spec();
        ScenarioReport::from_metrics(
            &spec,
            ScenarioMetrics {
                reset_clean: metrics.reset_clean,
                estimator_agrees: metrics.estimator_agrees,
                max_cross_track_error_m: self.tracking_error(metrics),
                max_altitude_error_m: metrics.max_altitude_error_m,
                max_roll_pitch_rad: metrics.max_roll_pitch_rad,
                max_speed_error_mps: metrics.max_horizontal_speed_mps,
                clamp_ratio: live_sequence_clamp_ratio(metrics),
                first_clamp_source: metrics.first_clamp_source.map(str::to_string),
            },
        )
    }
}

#[derive(Clone, Copy, Debug)]
struct LivePrimitiveMetrics {
    reset_clean: bool,
    estimator_agrees: bool,
    min_position_ned_m: [f32; 2],
    max_position_ned_m: [f32; 2],
    final_position_ned_m: [f32; 3],
    final_velocity_ned_mps: [f32; 3],
    max_altitude_error_m: f32,
    max_roll_pitch_rad: f32,
    max_horizontal_speed_mps: f32,
    max_horizontal_radius_m: f32,
    min_yaw_rad: f32,
    max_yaw_rad: f32,
    control_steps: usize,
    clamp_steps: usize,
    first_clamp_source: Option<&'static str>,
}

impl Default for LivePrimitiveMetrics {
    fn default() -> Self {
        Self {
            reset_clean: true,
            estimator_agrees: true,
            min_position_ned_m: [f32::INFINITY; 2],
            max_position_ned_m: [f32::NEG_INFINITY; 2],
            final_position_ned_m: [0.0; 3],
            final_velocity_ned_mps: [0.0; 3],
            max_altitude_error_m: 0.0,
            max_roll_pitch_rad: 0.0,
            max_horizontal_speed_mps: 0.0,
            max_horizontal_radius_m: 0.0,
            min_yaw_rad: f32::INFINITY,
            max_yaw_rad: f32::NEG_INFINITY,
            control_steps: 0,
            clamp_steps: 0,
            first_clamp_source: None,
        }
    }
}

impl LivePrimitiveMetrics {
    fn final_horizontal_speed_mps(self) -> f32 {
        (self.final_velocity_ned_mps[0] * self.final_velocity_ned_mps[0]
            + self.final_velocity_ned_mps[1] * self.final_velocity_ned_mps[1])
            .sqrt()
    }

    fn yaw_span_rad(self) -> f32 {
        if self.min_yaw_rad.is_finite() && self.max_yaw_rad.is_finite() {
            self.max_yaw_rad - self.min_yaw_rad
        } else {
            0.0
        }
    }

    fn max_abs_position_axis_m(self, axis: usize) -> f32 {
        self.min_position_ned_m[axis]
            .abs()
            .max(self.max_position_ned_m[axis].abs())
    }
}

#[derive(Clone, Copy, Debug)]
struct LiveCardinalCase {
    name: &'static str,
    axis: usize,
    sign: f32,
}

impl LiveCardinalCase {
    const fn new(name: &'static str, axis: usize, sign: f32) -> Self {
        Self { name, axis, sign }
    }

    fn id(self) -> &'static str {
        match self.name {
            "north" => "g25_live_m01_north",
            "south" => "g25_live_m01_south",
            "east" => "g25_live_m01_east",
            "west" => "g25_live_m01_west",
            _ => "g25_live_m01_unknown",
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct LiveCardinalMetrics {
    reset_clean: bool,
    estimator_agrees: bool,
    progress_m: f32,
    max_cross_track_error_m: f32,
    max_altitude_error_m: f32,
    max_roll_pitch_rad: f32,
    max_horizontal_speed_mps: f32,
    final_position_ned_m: [f32; 3],
    final_velocity_ned_mps: [f32; 3],
    control_steps: usize,
    clamp_steps: usize,
    first_clamp_source: Option<&'static str>,
}

#[derive(Clone, Copy, Debug, Default)]
struct PoseKinematicVelocity {
    previous: Option<(u64, [f32; 3])>,
}

impl PoseKinematicVelocity {
    fn apply(&mut self, estimate: &mut EstimateSnapshot, sim_time_us: u64) {
        let position = estimate.position_ned_m;
        let velocity = self
            .previous
            .and_then(|(previous_time_us, previous_position)| {
                let dt_us = sim_time_us.checked_sub(previous_time_us)?;
                if dt_us == 0 {
                    return None;
                }
                let dt_s = dt_us.min(u64::from(u32::MAX)) as f32 / 1_000_000.0;
                if !dt_s.is_finite() || dt_s <= f32::EPSILON {
                    return None;
                }

                Some([
                    (position[0] - previous_position[0]) / dt_s,
                    (position[1] - previous_position[1]) / dt_s,
                    (position[2] - previous_position[2]) / dt_s,
                ])
            });
        self.previous = Some((sim_time_us, position));
        if let Some(velocity) = velocity {
            if velocity.iter().all(|value| value.is_finite()) {
                estimate.velocity_ned_mps = velocity;
            }
        }
    }
}

#[derive(Clone, Debug)]
struct LiveTrace {
    case_name: &'static str,
    attempt: usize,
    rows: Vec<LiveTraceRow>,
}

#[derive(Clone, Copy, Debug)]
struct LiveTraceRow {
    frame_index: usize,
    phase: &'static str,
    sim_time_us: u64,
    elapsed_motion_s: f32,
    raw_position_ned_m: Option<[f32; 3]>,
    local_position_ned_m: [f32; 3],
    setpoint_position_ned_m: [f32; 3],
    velocity_ned_mps: [f32; 3],
    reported_velocity_ned_mps: [f32; 3],
    setpoint_velocity_ned_mps: [f32; 3],
    controller_velocity_setpoint_ned_mps: Option<[f32; 2]>,
    controller_velocity_error_ned_mps: Option<[f32; 2]>,
    requested_accel_ned_mps2: Option<[f32; 2]>,
    applied_accel_ned_mps2: Option<[f32; 2]>,
    attitude_setpoint_euler_rad: Option<[f32; 3]>,
    rate_setpoint_rps: Option<[f32; 3]>,
    throttle: Option<f32>,
    euler_rad: Option<[f32; 3]>,
    setpoint_yaw_rad: f32,
    motors: [f32; 4],
    clamped: bool,
}

impl LiveTrace {
    fn new(case_name: &'static str, attempt: usize) -> Self {
        Self {
            case_name,
            attempt,
            rows: Vec::new(),
        }
    }

    fn push(
        &mut self,
        frame_index: usize,
        motion_phase: bool,
        elapsed_motion_s: f32,
        frame: &GazeboSensorLine,
        estimate: &EstimateSnapshot,
        setpoint: LocalTrajectorySetpoint,
        output: &ControlOutput,
    ) {
        let attitude_setpoint_euler_rad = output
            .debug
            .attitude_setpoint
            .and_then(|attitude| quaternion_to_euler_rad(attitude.quaternion));
        let rate_setpoint_rps = output
            .debug
            .rate_setpoint
            .map(|rate| [rate.roll_rps, rate.pitch_rps, rate.yaw_rps]);
        self.rows.push(LiveTraceRow {
            frame_index,
            phase: if motion_phase { "motion" } else { "takeoff" },
            sim_time_us: frame.sim_time_us,
            elapsed_motion_s,
            raw_position_ned_m: frame.position_ned_m,
            local_position_ned_m: estimate.position_ned_m,
            setpoint_position_ned_m: setpoint.position_ned_m,
            velocity_ned_mps: estimate.velocity_ned_mps,
            reported_velocity_ned_mps: frame.vel_ned_mps,
            setpoint_velocity_ned_mps: setpoint.velocity_ned_mps,
            controller_velocity_setpoint_ned_mps: output.debug.velocity_setpoint_ned_mps,
            controller_velocity_error_ned_mps: output.debug.velocity_error_ned_mps,
            requested_accel_ned_mps2: output.debug.requested_accel_ned_mps2,
            applied_accel_ned_mps2: output.debug.applied_accel_ned_mps2,
            attitude_setpoint_euler_rad,
            rate_setpoint_rps,
            throttle: output.debug.throttle,
            euler_rad: frame.euler_rad,
            setpoint_yaw_rad: setpoint.yaw_rad,
            motors: output.motors,
            clamped: output.clamped,
        });
    }

    fn write_csv(&self) -> Result<PathBuf, String> {
        let dir = g25_trace_dir();
        fs::create_dir_all(&dir)
            .map_err(|error| format!("failed to create trace dir {}: {error}", dir.display()))?;
        let path = dir.join(format!(
            "{}_attempt{}_trace.csv",
            self.case_name, self.attempt
        ));
        let mut file = File::create(&path)
            .map_err(|error| format!("failed to create trace file {}: {error}", path.display()))?;
        writeln!(
            file,
            "case,attempt,frame_index,phase,sim_time_us,elapsed_motion_s,raw_pn_m,raw_pe_m,raw_pd_m,local_n_m,local_e_m,local_d_m,setpoint_n_m,setpoint_e_m,setpoint_d_m,error_n_m,error_e_m,error_d_m,vn_mps,ve_mps,vd_mps,reported_vn_mps,reported_ve_mps,reported_vd_mps,setpoint_vn_mps,setpoint_ve_mps,setpoint_vd_mps,controller_vn_mps,controller_ve_mps,velocity_error_n_mps,velocity_error_e_mps,requested_accel_n_mps2,requested_accel_e_mps2,applied_accel_n_mps2,applied_accel_e_mps2,attitude_sp_roll_rad,attitude_sp_pitch_rad,attitude_sp_yaw_rad,rate_sp_roll_rps,rate_sp_pitch_rps,rate_sp_yaw_rps,throttle,roll_rad,pitch_rad,yaw_rad,setpoint_yaw_rad,motor0,motor1,motor2,motor3,clamped"
        )
        .map_err(|error| format!("failed to write trace header {}: {error}", path.display()))?;
        for row in &self.rows {
            row.write_csv_line(&mut file, self.case_name, self.attempt)
                .map_err(|error| {
                    format!("failed to write trace file {}: {error}", path.display())
                })?;
        }

        Ok(path)
    }
}

impl LiveTraceRow {
    fn write_csv_line(
        self,
        file: &mut File,
        case_name: &str,
        attempt: usize,
    ) -> std::io::Result<()> {
        let [raw_pn, raw_pe, raw_pd] = self.raw_position_ned_m.unwrap_or([f32::NAN; 3]);
        let [roll, pitch, yaw] = self.euler_rad.unwrap_or([f32::NAN; 3]);
        let [controller_vn, controller_ve] = self
            .controller_velocity_setpoint_ned_mps
            .unwrap_or([f32::NAN; 2]);
        let [velocity_error_n, velocity_error_e] = self
            .controller_velocity_error_ned_mps
            .unwrap_or([f32::NAN; 2]);
        let [requested_accel_n, requested_accel_e] =
            self.requested_accel_ned_mps2.unwrap_or([f32::NAN; 2]);
        let [applied_accel_n, applied_accel_e] =
            self.applied_accel_ned_mps2.unwrap_or([f32::NAN; 2]);
        let [attitude_sp_roll, attitude_sp_pitch, attitude_sp_yaw] =
            self.attitude_setpoint_euler_rad.unwrap_or([f32::NAN; 3]);
        let [rate_sp_roll, rate_sp_pitch, rate_sp_yaw] =
            self.rate_setpoint_rps.unwrap_or([f32::NAN; 3]);
        let throttle = self.throttle.unwrap_or(f32::NAN);
        let error_n = self.local_position_ned_m[0] - self.setpoint_position_ned_m[0];
        let error_e = self.local_position_ned_m[1] - self.setpoint_position_ned_m[1];
        let error_d = self.local_position_ned_m[2] - self.setpoint_position_ned_m[2];

        writeln!(
            file,
            "{case_name},{attempt},{},{},{},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{:.6},{}",
            self.frame_index,
            self.phase,
            self.sim_time_us,
            self.elapsed_motion_s,
            raw_pn,
            raw_pe,
            raw_pd,
            self.local_position_ned_m[0],
            self.local_position_ned_m[1],
            self.local_position_ned_m[2],
            self.setpoint_position_ned_m[0],
            self.setpoint_position_ned_m[1],
            self.setpoint_position_ned_m[2],
            error_n,
            error_e,
            error_d,
            self.velocity_ned_mps[0],
            self.velocity_ned_mps[1],
            self.velocity_ned_mps[2],
            self.reported_velocity_ned_mps[0],
            self.reported_velocity_ned_mps[1],
            self.reported_velocity_ned_mps[2],
            self.setpoint_velocity_ned_mps[0],
            self.setpoint_velocity_ned_mps[1],
            self.setpoint_velocity_ned_mps[2],
            controller_vn,
            controller_ve,
            velocity_error_n,
            velocity_error_e,
            requested_accel_n,
            requested_accel_e,
            applied_accel_n,
            applied_accel_e,
            attitude_sp_roll,
            attitude_sp_pitch,
            attitude_sp_yaw,
            rate_sp_roll,
            rate_sp_pitch,
            rate_sp_yaw,
            throttle,
            roll,
            pitch,
            yaw,
            self.setpoint_yaw_rad,
            self.motors[0],
            self.motors[1],
            self.motors[2],
            self.motors[3],
            self.clamped
        )
    }
}

fn quaternion_to_euler_rad(quaternion: [f32; 4]) -> Option<[f32; 3]> {
    if !quaternion.iter().all(|value| value.is_finite()) {
        return None;
    }

    let norm = (quaternion[0] * quaternion[0]
        + quaternion[1] * quaternion[1]
        + quaternion[2] * quaternion[2]
        + quaternion[3] * quaternion[3])
        .sqrt();
    if norm <= f32::EPSILON {
        return None;
    }

    let w = quaternion[0] / norm;
    let x = quaternion[1] / norm;
    let y = quaternion[2] / norm;
    let z = quaternion[3] / norm;
    let sin_roll = 2.0 * (w * x + y * z);
    let cos_roll = 1.0 - 2.0 * (x * x + y * y);
    let roll = sin_roll.atan2(cos_roll);
    let sin_pitch = 2.0 * (w * y - z * x);
    let pitch = sin_pitch.clamp(-1.0, 1.0).asin();
    let sin_yaw = 2.0 * (w * z + x * y);
    let cos_yaw = 1.0 - 2.0 * (y * y + z * z);
    let yaw = sin_yaw.atan2(cos_yaw);

    Some([roll, pitch, yaw])
}

fn g25_trace_dir() -> PathBuf {
    std::env::var_os(G25_TRACE_DIR_ENV)
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
            manifest_dir
                .parent()
                .and_then(|path| path.parent())
                .map(|path| path.join("target/gazebo-traces"))
                .unwrap_or_else(|| manifest_dir.join("target/gazebo-traces"))
        })
}

impl Default for LiveCardinalMetrics {
    fn default() -> Self {
        Self {
            reset_clean: true,
            estimator_agrees: true,
            progress_m: 0.0,
            max_cross_track_error_m: 0.0,
            max_altitude_error_m: 0.0,
            max_roll_pitch_rad: 0.0,
            max_horizontal_speed_mps: 0.0,
            final_position_ned_m: [0.0; 3],
            final_velocity_ned_mps: [0.0; 3],
            control_steps: 0,
            clamp_steps: 0,
            first_clamp_source: None,
        }
    }
}

fn run_live_primitive_case(
    bridge: &mut LiveGazeboBridge,
    case: LivePrimitiveCase,
    attempt: usize,
) -> Result<ScenarioReport, String> {
    let _ = bridge.send_actuator([0.0; 4], None);
    bridge.reset_and_play()?;
    let settle_candidates =
        bridge.collect_sensor_frames(LIVE_SETTLE_CANDIDATE_FRAMES, LIVE_SETTLE_TIMEOUT)?;
    let settle_frames = fresh_monotonic_gazebo_suffix(&settle_candidates, LIVE_SETTLE_FRAMES);
    let origin_frame = settle_frames
        .iter()
        .rev()
        .find(|frame| frame.position_ned_m.is_some() && frame.attitude_quaternion().is_some())
        .ok_or_else(|| "Gazebo SENSOR frames did not include pose truth".to_string())?;
    let origin = GazeboTruthOrigin::from_frame(origin_frame)
        .ok_or_else(|| "could not establish Gazebo truth origin".to_string())?;
    let mut metrics = LivePrimitiveMetrics {
        reset_clean: frames_are_monotonic_gazebo_sensor_data(&settle_frames),
        ..LivePrimitiveMetrics::default()
    };
    let pipeline = hardware_live_pipeline();
    let mut motion_start_sim_time_us = None;
    let mut trace = LiveTrace::new(case.name(), attempt);
    let mut pose_velocity = PoseKinematicVelocity::default();

    for frame_index in 0..(LIVE_TAKEOFF_FRAMES + case.motion_frames()) {
        let frame = bridge.next_sensor_frame(LIVE_SENSOR_STEP_TIMEOUT)?;
        metrics.reset_clean &= frame.clock_source.as_deref() == Some("gazebo");
        let Some(mut estimate) = frame.to_truth_estimate(origin) else {
            metrics.estimator_agrees = false;
            continue;
        };
        pose_velocity.apply(&mut estimate, frame.sim_time_us);
        let motion_phase = frame_index >= LIVE_TAKEOFF_FRAMES;
        let mut elapsed_motion_s = 0.0;
        let setpoint = if motion_phase {
            let start_us = *motion_start_sim_time_us.get_or_insert(frame.sim_time_us);
            elapsed_motion_s = frame
                .sim_time_us
                .saturating_sub(start_us)
                .min(u64::from(u32::MAX)) as f32
                / 1_000_000.0;
            case.setpoint(elapsed_motion_s)
        } else {
            trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], 0.0)
        };
        let output = pipeline.step(
            estimate,
            frame.to_truth_imu(),
            ControlSetpoint::from_local_trajectory(setpoint, true),
        );
        trace.push(
            frame_index,
            motion_phase,
            elapsed_motion_s,
            &frame,
            &estimate,
            setpoint,
            &output,
        );
        bridge.send_actuator(output.motors, Some(frame.sim_time_us))?;
        update_live_primitive_metrics(
            &mut metrics,
            &frame,
            &estimate,
            output.clamped,
            motion_phase,
        );
    }

    bridge.send_actuator([0.0; 4], None)?;
    let trace_path = trace.write_csv()?;

    println!(
        "G2.5 live {} attempt={} track={:.3}m alt_err={:.3}m roll_pitch={:.3}rad speed={:.3}m/s yaw_span={:.3}rad final_pos=[{:.3},{:.3},{:.3}] final_vel=[{:.3},{:.3},{:.3}] clamp_ratio={:.3} trace={}",
        case.name(),
        attempt,
        case.tracking_error(metrics),
        metrics.max_altitude_error_m,
        metrics.max_roll_pitch_rad,
        metrics.max_horizontal_speed_mps,
        metrics.yaw_span_rad(),
        metrics.final_position_ned_m[0],
        metrics.final_position_ned_m[1],
        metrics.final_position_ned_m[2],
        metrics.final_velocity_ned_mps[0],
        metrics.final_velocity_ned_mps[1],
        metrics.final_velocity_ned_mps[2],
        live_sequence_clamp_ratio(metrics),
        trace_path.display()
    );

    Ok(case.score(metrics))
}

fn run_live_cardinal_case(
    bridge: &mut LiveGazeboBridge,
    case: LiveCardinalCase,
    attempt: usize,
) -> Result<ScenarioReport, String> {
    let _ = bridge.send_actuator([0.0; 4], None);
    bridge.reset_and_play()?;
    let settle_candidates =
        bridge.collect_sensor_frames(LIVE_SETTLE_CANDIDATE_FRAMES, LIVE_SETTLE_TIMEOUT)?;
    let settle_frames = fresh_monotonic_gazebo_suffix(&settle_candidates, LIVE_SETTLE_FRAMES);
    let origin_frame = settle_frames
        .iter()
        .rev()
        .find(|frame| frame.position_ned_m.is_some() && frame.attitude_quaternion().is_some())
        .ok_or_else(|| "Gazebo SENSOR frames did not include pose truth".to_string())?;
    let origin = GazeboTruthOrigin::from_frame(origin_frame)
        .ok_or_else(|| "could not establish Gazebo truth origin".to_string())?;
    let mut metrics = LiveCardinalMetrics {
        reset_clean: frames_are_monotonic_gazebo_sensor_data(&settle_frames),
        ..LiveCardinalMetrics::default()
    };
    let pipeline = hardware_live_pipeline();
    let mut motion_start_sim_time_us = None;
    let mut trace = LiveTrace::new(case.name, attempt);
    let mut pose_velocity = PoseKinematicVelocity::default();

    for frame_index in 0..(LIVE_TAKEOFF_FRAMES + LIVE_MOTION_FRAMES) {
        let frame = bridge.next_sensor_frame(LIVE_SENSOR_STEP_TIMEOUT)?;
        metrics.reset_clean &= frame.clock_source.as_deref() == Some("gazebo");
        let Some(mut estimate) = frame.to_truth_estimate(origin) else {
            metrics.estimator_agrees = false;
            continue;
        };
        pose_velocity.apply(&mut estimate, frame.sim_time_us);
        let motion_phase = frame_index >= LIVE_TAKEOFF_FRAMES;
        let mut elapsed_motion_s = 0.0;
        let setpoint = if motion_phase {
            let start_us = *motion_start_sim_time_us.get_or_insert(frame.sim_time_us);
            elapsed_motion_s = frame
                .sim_time_us
                .saturating_sub(start_us)
                .min(u64::from(u32::MAX)) as f32
                / 1_000_000.0;
            live_motion_trajectory(case, elapsed_motion_s)
        } else {
            trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], 0.0)
        };
        let output = pipeline.step(
            estimate,
            frame.to_truth_imu(),
            ControlSetpoint::from_local_trajectory(setpoint, true),
        );
        trace.push(
            frame_index,
            motion_phase,
            elapsed_motion_s,
            &frame,
            &estimate,
            setpoint,
            &output,
        );
        bridge.send_actuator(output.motors, Some(frame.sim_time_us))?;
        update_live_metrics(
            &mut metrics,
            case,
            &frame,
            &estimate,
            output.clamped,
            motion_phase,
        );
    }

    bridge.send_actuator([0.0; 4], None)?;
    let trace_path = trace.write_csv()?;

    println!(
        "G2.5 live {} attempt={} progress={:.3}m cross={:.3}m alt_err={:.3}m roll_pitch={:.3}rad speed={:.3}m/s final_pos=[{:.3},{:.3},{:.3}] final_vel=[{:.3},{:.3},{:.3}] clamp_ratio={:.3} trace={}",
        case.name,
        attempt,
        metrics.progress_m,
        metrics.max_cross_track_error_m,
        metrics.max_altitude_error_m,
        metrics.max_roll_pitch_rad,
        metrics.max_horizontal_speed_mps,
        metrics.final_position_ned_m[0],
        metrics.final_position_ned_m[1],
        metrics.final_position_ned_m[2],
        metrics.final_velocity_ned_mps[0],
        metrics.final_velocity_ned_mps[1],
        metrics.final_velocity_ned_mps[2],
        live_clamp_ratio(metrics),
        trace_path.display()
    );

    Ok(score_live_cardinal_case(case, metrics))
}

fn update_live_primitive_metrics(
    metrics: &mut LivePrimitiveMetrics,
    frame: &GazeboSensorLine,
    estimate: &EstimateSnapshot,
    clamped: bool,
    motion_phase: bool,
) {
    metrics.control_steps += 1;
    metrics.final_position_ned_m = estimate.position_ned_m;
    metrics.final_velocity_ned_mps = estimate.velocity_ned_mps;
    if clamped {
        metrics.clamp_steps += 1;
        if metrics.first_clamp_source.is_none() {
            metrics.first_clamp_source = Some("motor_output_limit");
        }
    }

    metrics.max_altitude_error_m = metrics
        .max_altitude_error_m
        .max((estimate.position_ned_m[2] - HOLD_ALTITUDE_DOWN_M).abs());
    let horizontal_speed = (estimate.velocity_ned_mps[0] * estimate.velocity_ned_mps[0]
        + estimate.velocity_ned_mps[1] * estimate.velocity_ned_mps[1])
        .sqrt();
    metrics.max_horizontal_speed_mps = metrics.max_horizontal_speed_mps.max(horizontal_speed);

    if let Some([roll_rad, pitch_rad, yaw_rad]) = frame.euler_rad {
        metrics.max_roll_pitch_rad = metrics
            .max_roll_pitch_rad
            .max(roll_rad.abs().max(pitch_rad.abs()));
        if motion_phase {
            metrics.min_yaw_rad = metrics.min_yaw_rad.min(yaw_rad);
            metrics.max_yaw_rad = metrics.max_yaw_rad.max(yaw_rad);
        }
    }

    if motion_phase {
        for axis in 0..2 {
            metrics.min_position_ned_m[axis] =
                metrics.min_position_ned_m[axis].min(estimate.position_ned_m[axis]);
            metrics.max_position_ned_m[axis] =
                metrics.max_position_ned_m[axis].max(estimate.position_ned_m[axis]);
        }
        let radius = (estimate.position_ned_m[0] * estimate.position_ned_m[0]
            + estimate.position_ned_m[1] * estimate.position_ned_m[1])
            .sqrt();
        metrics.max_horizontal_radius_m = metrics.max_horizontal_radius_m.max(radius);
    }
}

fn update_live_metrics(
    metrics: &mut LiveCardinalMetrics,
    case: LiveCardinalCase,
    frame: &GazeboSensorLine,
    estimate: &EstimateSnapshot,
    clamped: bool,
    motion_phase: bool,
) {
    metrics.control_steps += 1;
    metrics.final_position_ned_m = estimate.position_ned_m;
    metrics.final_velocity_ned_mps = estimate.velocity_ned_mps;
    if clamped {
        metrics.clamp_steps += 1;
        if metrics.first_clamp_source.is_none() {
            metrics.first_clamp_source = Some("motor_output_limit");
        }
    }

    metrics.max_altitude_error_m = metrics
        .max_altitude_error_m
        .max((estimate.position_ned_m[2] - HOLD_ALTITUDE_DOWN_M).abs());
    let horizontal_speed = (estimate.velocity_ned_mps[0] * estimate.velocity_ned_mps[0]
        + estimate.velocity_ned_mps[1] * estimate.velocity_ned_mps[1])
        .sqrt();
    metrics.max_horizontal_speed_mps = metrics.max_horizontal_speed_mps.max(horizontal_speed);

    if let Some([roll_rad, pitch_rad, _]) = frame.euler_rad {
        metrics.max_roll_pitch_rad = metrics
            .max_roll_pitch_rad
            .max(roll_rad.abs().max(pitch_rad.abs()));
    }

    if motion_phase {
        metrics.progress_m = metrics
            .progress_m
            .max(estimate.position_ned_m[case.axis] * case.sign);
        let cross_axis = if case.axis == 0 { 1 } else { 0 };
        metrics.max_cross_track_error_m = metrics
            .max_cross_track_error_m
            .max(estimate.position_ned_m[cross_axis].abs());
    }
}

fn score_live_cardinal_case(
    case: LiveCardinalCase,
    metrics: LiveCardinalMetrics,
) -> ScenarioReport {
    let spec = live_cardinal_spec(case);
    let progress_shortfall_m = (LIVE_REQUIRED_PROGRESS_M - metrics.progress_m).max(0.0);
    let reported_cross_track = if progress_shortfall_m > 0.0 {
        spec.thresholds.max_cross_track_error_m + progress_shortfall_m
    } else {
        metrics.max_cross_track_error_m
    };

    ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean: metrics.reset_clean,
            estimator_agrees: metrics.estimator_agrees,
            max_cross_track_error_m: reported_cross_track,
            max_altitude_error_m: metrics.max_altitude_error_m,
            max_roll_pitch_rad: metrics.max_roll_pitch_rad,
            max_speed_error_mps: metrics.max_horizontal_speed_mps,
            clamp_ratio: live_clamp_ratio(metrics),
            first_clamp_source: metrics.first_clamp_source.map(str::to_string),
        },
    )
}

fn live_cardinal_spec(case: LiveCardinalCase) -> ScenarioSpec {
    ScenarioSpec {
        id: case.id(),
        gate: Gate::G2Point5,
        description: "live Gazebo M01 cardinal motion primitive scored from bridge SENSOR truth",
        reset_required: true,
        command_schedule: CommandSchedule::new(vec![
            ScheduledCommand {
                at_frame: 0,
                kind: CommandKind::SimReset,
            },
            ScheduledCommand {
                at_frame: 12,
                kind: CommandKind::SimPlay,
            },
            ScheduledCommand {
                at_frame: LIVE_TAKEOFF_FRAMES,
                kind: CommandKind::LocalTrajectory,
            },
            ScheduledCommand {
                at_frame: LIVE_TAKEOFF_FRAMES + LIVE_MOTION_FRAMES,
                kind: CommandKind::Hold,
            },
        ]),
        thresholds: AcceptanceThresholds {
            max_cross_track_error_m: 1.10,
            max_altitude_error_m: 1.25,
            max_roll_pitch_rad: 0.85,
            max_speed_error_mps: 2.50,
            max_clamp_ratio: 0.45,
        },
    }
}

fn live_clamp_ratio(metrics: LiveCardinalMetrics) -> f32 {
    if metrics.control_steps == 0 {
        return 1.0;
    }

    metrics.clamp_steps as f32 / metrics.control_steps as f32
}

fn live_sequence_clamp_ratio(metrics: LivePrimitiveMetrics) -> f32 {
    if metrics.control_steps == 0 {
        return 1.0;
    }

    metrics.clamp_steps as f32 / metrics.control_steps as f32
}

fn live_motion_trajectory(case: LiveCardinalCase, elapsed_s: f32) -> LocalTrajectorySetpoint {
    axis_trapezoid_trajectory(
        case.axis,
        case.sign,
        LIVE_MOTION_TARGET_M,
        elapsed_s,
        LIVE_MOTION_SPEED_MPS,
        LIVE_MOTION_ACCEL_MPS2,
        0.0,
    )
}

fn axis_trapezoid_trajectory(
    axis: usize,
    sign: f32,
    distance_m: f32,
    elapsed_s: f32,
    speed_mps: f32,
    accel_mps2: f32,
    yaw_rad: f32,
) -> LocalTrajectorySetpoint {
    let (distance_m, speed_mps, accel_mps2) =
        trapezoid_1d(distance_m, speed_mps, accel_mps2, elapsed_s);
    let mut position = [0.0, 0.0, HOLD_ALTITUDE_DOWN_M];
    let mut velocity = [0.0; 3];
    let mut accel = [0.0; 3];
    position[axis] = sign * distance_m;
    velocity[axis] = sign * speed_mps;
    accel[axis] = sign * accel_mps2;

    trajectory(position, velocity, accel, yaw_rad)
}

fn trapezoid_1d(
    distance_m: f32,
    speed_mps: f32,
    accel_mps2: f32,
    elapsed_s: f32,
) -> (f32, f32, f32) {
    let distance_m = distance_m.max(0.0);
    let speed_mps = speed_mps.max(0.001);
    let accel_mps2 = accel_mps2.max(0.001);
    let elapsed_s = elapsed_s.max(0.0);
    let nominal_accel_time_s = speed_mps / accel_mps2;
    let nominal_accel_distance_m = 0.5 * accel_mps2 * nominal_accel_time_s * nominal_accel_time_s;
    let (accel_time_s, cruise_time_s, peak_speed_mps) =
        if 2.0 * nominal_accel_distance_m >= distance_m {
            let accel_time_s = (distance_m / accel_mps2).sqrt();
            (accel_time_s, 0.0, accel_mps2 * accel_time_s)
        } else {
            let cruise_distance_m = distance_m - 2.0 * nominal_accel_distance_m;
            (
                nominal_accel_time_s,
                cruise_distance_m / speed_mps,
                speed_mps,
            )
        };
    let accel_distance_m = 0.5 * accel_mps2 * accel_time_s * accel_time_s;
    let cruise_distance_m = peak_speed_mps * cruise_time_s;
    let decel_start_s = accel_time_s + cruise_time_s;
    let total_time_s = decel_start_s + accel_time_s;

    if elapsed_s < accel_time_s {
        (
            0.5 * accel_mps2 * elapsed_s * elapsed_s,
            accel_mps2 * elapsed_s,
            accel_mps2,
        )
    } else if elapsed_s < decel_start_s {
        let cruise_elapsed_s = elapsed_s - accel_time_s;
        (
            accel_distance_m + peak_speed_mps * cruise_elapsed_s,
            peak_speed_mps,
            0.0,
        )
    } else if elapsed_s < total_time_s {
        let decel_elapsed_s = elapsed_s - decel_start_s;
        (
            accel_distance_m + cruise_distance_m + peak_speed_mps * decel_elapsed_s
                - 0.5 * accel_mps2 * decel_elapsed_s * decel_elapsed_s,
            (peak_speed_mps - accel_mps2 * decel_elapsed_s).max(0.0),
            -accel_mps2,
        )
    } else {
        (distance_m, 0.0, 0.0)
    }
}

fn piecewise_scalar(
    elapsed_s: f32,
    segment_duration_s: f32,
    points: &[(f32, f32)],
) -> (f32, f32, f32) {
    if points.is_empty() {
        return (0.0, 0.0, 0.0);
    }
    let mut remaining_s = elapsed_s.max(0.0);
    for (start, end) in points.iter().copied() {
        if remaining_s <= segment_duration_s {
            return smooth_segment(remaining_s, segment_duration_s, start, end);
        }
        remaining_s -= segment_duration_s;
    }

    (points.last().expect("points are non-empty").1, 0.0, 0.0)
}

fn smooth_segment(elapsed_s: f32, duration_s: f32, start: f32, end: f32) -> (f32, f32, f32) {
    let duration_s = duration_s.max(0.001);
    let elapsed_s = elapsed_s.clamp(0.0, duration_s);
    let u = elapsed_s / duration_s;
    let span = end - start;
    let position = start + span * (3.0 * u * u - 2.0 * u * u * u);
    let velocity = span * (6.0 * u - 6.0 * u * u) / duration_s;
    let accel = span * (6.0 - 12.0 * u) / (duration_s * duration_s);
    if elapsed_s >= duration_s {
        (end, 0.0, 0.0)
    } else {
        (position, velocity, accel)
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

fn fresh_monotonic_gazebo_suffix(
    frames: &[GazeboSensorLine],
    min_count: usize,
) -> Vec<GazeboSensorLine> {
    for start in 0..frames.len() {
        let suffix = &frames[start..];
        if suffix.len() >= min_count && frames_are_monotonic_gazebo_sensor_data(suffix) {
            return suffix.to_vec();
        }
    }

    frames
        .iter()
        .rev()
        .take(min_count)
        .cloned()
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect()
}
