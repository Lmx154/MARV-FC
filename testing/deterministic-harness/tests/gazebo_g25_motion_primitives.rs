mod support;

use std::time::Duration;

use common::control::{
    config::ControlLoopConfig,
    pipeline::{
        ControlFrame, ControlSetpoint, ControlSetpointSource, EstimateSnapshot,
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
const CONSERVATIVE_SPEED_MPS: f32 = 0.45;
const CONSERVATIVE_ACCEL_MPS2: f32 = 0.35;
const LIVE_SENSOR_STEP_TIMEOUT: Duration = Duration::from_secs(2);
const LIVE_SETTLE_TIMEOUT: Duration = Duration::from_secs(4);
const LIVE_SETTLE_FRAMES: usize = 12;
const LIVE_SETTLE_CANDIDATE_FRAMES: usize = 24;
const LIVE_TAKEOFF_FRAMES: usize = 120;
const LIVE_MOTION_FRAMES: usize = 110;
const LIVE_MOTION_TARGET_M: f32 = 0.55;
const LIVE_MOTION_SPEED_MPS: f32 = 0.25;
const LIVE_MOTION_ACCEL_MPS2: f32 = 0.15;
const LIVE_REQUIRED_PROGRESS_M: f32 = 0.12;
const LIVE_ATTEMPTS_PER_CARDINAL_CASE: usize = 2;

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
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g25_m01_cardinal_motion_commands_correct_world_axis_attitude_signs() {
    let pipeline = primitive_pipeline();

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
        assert_vec2_near(
            output
                .debug
                .velocity_setpoint_ned_mps
                .expect("velocity setpoint"),
            [
                velocity[0].clamp(-CONSERVATIVE_SPEED_MPS, CONSERVATIVE_SPEED_MPS),
                velocity[1].clamp(-CONSERVATIVE_SPEED_MPS, CONSERVATIVE_SPEED_MPS),
            ],
            0.003,
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
            0.20,
        )),
    )
}

fn hardware_live_pipeline() -> FlightControlPipeline {
    let airframe = GazeboAirframeConfig::parse(AIRFRAME_CONFIG).expect("airframe config parses");
    airframe
        .validate()
        .expect("airframe config remains the hardware source of truth");
    let mut loop_config = primitive_loop_config(0.20);
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
        max_horizontal_velocity_mps: CONSERVATIVE_SPEED_MPS,
        max_horizontal_accel_mps2: CONSERVATIVE_ACCEL_MPS2,
        max_horizontal_accel_slew_mps3: 1.20,
        max_tilt_rad: 8.0_f32.to_radians(),
        ..PositionControllerConfig::default()
    };
    config.rate = RateControllerConfig {
        max_axis_command,
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

fn assert_vec2_near(actual: [f32; 2], expected: [f32; 2], tolerance: f32) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}"
        );
    }
}

trait MotorOutputAccess {
    fn motor_outputs(&self) -> common::control::mixing::MotorOutputs<4>;
}

impl MotorOutputAccess for common::control::pipeline::ControlOutput {
    fn motor_outputs(&self) -> common::control::mixing::MotorOutputs<4> {
        self.debug.motor_outputs.expect("motor outputs")
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

    for frame_index in 0..(LIVE_TAKEOFF_FRAMES + LIVE_MOTION_FRAMES) {
        let frame = bridge.next_sensor_frame(LIVE_SENSOR_STEP_TIMEOUT)?;
        metrics.reset_clean &= frame.clock_source.as_deref() == Some("gazebo");
        let Some(estimate) = frame.to_truth_estimate(origin) else {
            metrics.estimator_agrees = false;
            continue;
        };
        let motion_phase = frame_index >= LIVE_TAKEOFF_FRAMES;
        let setpoint = if motion_phase {
            let start_us = *motion_start_sim_time_us.get_or_insert(frame.sim_time_us);
            let elapsed_s = frame
                .sim_time_us
                .saturating_sub(start_us)
                .min(u64::from(u32::MAX)) as f32
                / 1_000_000.0;
            live_motion_trajectory(case, elapsed_s)
        } else {
            trajectory([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], [0.0; 3], 0.0)
        };
        let output = pipeline.step(
            estimate,
            frame.to_truth_imu(),
            ControlSetpoint::from_local_trajectory(setpoint, true),
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

    println!(
        "G2.5 live {} attempt={} progress={:.3}m cross={:.3}m alt_err={:.3}m roll_pitch={:.3}rad speed={:.3}m/s final_pos=[{:.3},{:.3},{:.3}] final_vel=[{:.3},{:.3},{:.3}] clamp_ratio={:.3}",
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
        live_clamp_ratio(metrics)
    );

    Ok(score_live_cardinal_case(case, metrics))
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

fn live_motion_trajectory(case: LiveCardinalCase, elapsed_s: f32) -> LocalTrajectorySetpoint {
    let elapsed_s = elapsed_s.max(0.0);
    let accel_time_s = LIVE_MOTION_SPEED_MPS / LIVE_MOTION_ACCEL_MPS2;
    let accel_distance_m = 0.5 * LIVE_MOTION_ACCEL_MPS2 * accel_time_s * accel_time_s;
    let (distance_m, speed_mps, accel_mps2) = if elapsed_s < accel_time_s {
        (
            0.5 * LIVE_MOTION_ACCEL_MPS2 * elapsed_s * elapsed_s,
            LIVE_MOTION_ACCEL_MPS2 * elapsed_s,
            LIVE_MOTION_ACCEL_MPS2,
        )
    } else {
        (
            accel_distance_m + LIVE_MOTION_SPEED_MPS * (elapsed_s - accel_time_s),
            LIVE_MOTION_SPEED_MPS,
            0.0,
        )
    };
    let distance_m = distance_m.min(LIVE_MOTION_TARGET_M);
    let at_target = distance_m >= LIVE_MOTION_TARGET_M - 0.001;
    let mut position = [0.0, 0.0, HOLD_ALTITUDE_DOWN_M];
    let mut velocity = [0.0; 3];
    let mut accel = [0.0; 3];
    position[case.axis] = case.sign * distance_m;
    if !at_target {
        velocity[case.axis] = case.sign * speed_mps.min(LIVE_MOTION_SPEED_MPS);
        accel[case.axis] = case.sign * accel_mps2;
    }

    trajectory(position, velocity, accel, 0.0)
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
