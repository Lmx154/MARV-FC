mod support;

use std::time::Duration;

use common::control::{
    attitude::AttitudeSetpoint,
    config::ControlLoopConfig,
    guidance::{
        GuidanceCommand, GuidancePhase, GuidanceStateMachine, TakeoffNavGate, TakeoffNavGateConfig,
        TakeoffNavGateSample, TrajectoryLimits,
    },
    pipeline::{
        ControlFrame, ControlSetpoint, ControlSetpointSource, EstimateSnapshot,
        EstimatorLocalState, EstimatorResetDelta, FlightControlPipeline, ImuControlInput,
        MissionWaypoint,
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
const G3_STEP_TARGET_M: f32 = 0.45;
const G3_REQUIRED_PROGRESS_M: f32 = 0.30;
const G3_MAX_LIVE_FRAMES: usize = 380;
const G3_NAV_FRAMES: usize = 150;
const G3_ROUTE_MAX_LIVE_FRAMES: usize = 1400;
const G3_ROUTE_MIN_LEG_FRAMES: usize = 35;
const G3_ROUTE_LEG_ACCEPTANCE_M: f32 = 0.14;
const G3_ROUTE_LEG_MAX_SPEED_MPS: f32 = 0.14;
const G3_LIVE_ATTEMPTS_PER_CASE: usize = 2;
const LIVE_SENSOR_STEP_TIMEOUT: Duration = Duration::from_secs(2);
const LIVE_SETTLE_TIMEOUT: Duration = Duration::from_secs(4);
const LIVE_SETTLE_CANDIDATE_FRAMES: usize = 24;
const LIVE_SETTLE_FRAMES: usize = 12;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum NavigationAcceptanceId {
    AirborneOriginLatch,
    SmallNorthStep,
    SmallEastStep,
    SquareReturnToOrigin,
    YawedNorthStep,
}

#[derive(Clone, Debug, PartialEq)]
struct NavigationAcceptanceSpec {
    id: NavigationAcceptanceId,
    scenario: ScenarioSpec,
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once G2.5 is green"]
fn g3_declares_navigation_acceptance_cases_as_mission_waypoints() {
    let specs = navigation_acceptance_specs();

    assert_eq!(specs.len(), 5);
    assert_eq!(specs[0].id, NavigationAcceptanceId::AirborneOriginLatch);
    for spec in &specs {
        assert_eq!(spec.scenario.gate, Gate::G3);
        assert!(spec.scenario.reset_required);
        assert!(
            spec.scenario
                .command_schedule
                .commands
                .iter()
                .any(|command| command.kind == CommandKind::MissionWaypoint),
            "{} must enter G3 through a mission waypoint",
            spec.scenario.id
        );
        assert!(
            !spec
                .scenario
                .command_schedule
                .commands
                .iter()
                .any(|command| command.kind == CommandKind::LocalPosition),
            "{} must not bypass guidance with raw local position setpoints",
            spec.scenario.id
        );
    }
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once G2.5 is green"]
fn g3_takeoff_gate_guidance_and_control_pipeline_use_waypoint_path() {
    let mut gate = TakeoffNavGate::new(TakeoffNavGateConfig {
        min_stable_frames: 2,
        ..TakeoffNavGateConfig::default()
    });
    let mut guidance = GuidanceStateMachine::new(g3_route_trajectory_limits());
    let pipeline = hardware_live_pipeline();
    let estimate = estimate_at([3.0, -2.0, HOLD_ALTITUDE_DOWN_M], [0.02, -0.01, 0.01], 0.0);

    let first_hold = gate.update(takeoff_sample_from_estimate(estimate, 0.0), 0.0);
    assert_eq!(
        first_hold.latched_takeoff_origin_ned_m,
        [3.0, -2.0, HOLD_ALTITUDE_DOWN_M]
    );
    assert!(!first_hold.nav_ready);
    let ready = gate.update(takeoff_sample_from_estimate(estimate, 0.0), 0.0);
    assert!(ready.nav_ready);

    let waypoint = MissionWaypoint::new(
        [
            ready.latched_takeoff_origin_ned_m[0] + G3_STEP_TARGET_M,
            ready.latched_takeoff_origin_ned_m[1],
            HOLD_ALTITUDE_DOWN_M,
        ],
        0.0,
    );
    let guidance_output = guidance
        .update(
            estimator_state_from_estimate(estimate, 0.0),
            GuidanceCommand::NavigateLeg(waypoint),
            0.10,
        )
        .expect("G3 guidance should produce a shaped trajectory");
    assert_eq!(guidance.phase(), GuidancePhase::NavigateLeg);
    assert_eq!(guidance_output.phase, GuidancePhase::NavigateLeg);
    assert!(
        guidance_output.trajectory.position_ned_m[0] < waypoint.position_ned_m[0],
        "first G3 output should be a trajectory step, not a raw waypoint jump"
    );
    assert!(
        guidance_output.trajectory.velocity_ned_mps[0] > 0.0,
        "north waypoint should request positive north trajectory velocity"
    );

    let setpoint = ControlSetpoint::from_local_trajectory(guidance_output.trajectory, true);
    assert_eq!(
        setpoint.source(),
        ControlSetpointSource::EstimatorLocalTrajectory
    );
    let output = pipeline.step(estimate, ImuControlInput::default(), setpoint);
    assert!(output.control_valid);
    assert_eq!(
        output.debug.estimator_frame,
        ControlFrame::EstimatorLocalNed
    );
    assert_eq!(output.debug.setpoint_frame, ControlFrame::EstimatorLocalNed);
    assert_eq!(
        output.debug.control_setpoint_ned_m,
        Some(guidance_output.trajectory.position_ned_m)
    );
    assert!(
        output
            .debug
            .velocity_setpoint_ned_mps
            .expect("velocity setpoint")[0]
            > 0.0
    );
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once G2.5 is green"]
fn g3_estimator_reset_shifts_gate_origin_and_guidance_trajectory() {
    let mut gate = TakeoffNavGate::new(TakeoffNavGateConfig {
        min_stable_frames: 1,
        ..TakeoffNavGateConfig::default()
    });
    let mut guidance = GuidanceStateMachine::new(g3_trajectory_limits());
    let estimate = estimate_at([10.0, -4.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], 0.0);
    let ready = gate.update(takeoff_sample_from_estimate(estimate, 0.0), 0.0);
    assert!(ready.nav_ready);

    let waypoint = MissionWaypoint::new([10.6, -4.0, HOLD_ALTITUDE_DOWN_M], 0.0);
    let before = guidance
        .update(
            estimator_state_from_estimate(estimate, 0.0),
            GuidanceCommand::NavigateLeg(waypoint),
            0.10,
        )
        .expect("initial guidance update");
    let delta = EstimatorResetDelta {
        xy_reset_counter: 1,
        position_delta_ned_m: [0.25, -0.10, 0.0],
        ..EstimatorResetDelta::NONE
    };

    assert!(gate.apply_estimator_reset(delta));
    assert!(guidance.apply_estimator_reset(delta));
    let shifted_estimate = estimate_at([10.25, -4.10, HOLD_ALTITUDE_DOWN_M], [0.0; 3], 0.0);
    let shifted = gate.update(takeoff_sample_from_estimate(shifted_estimate, 0.0), 0.0);
    assert_eq!(
        shifted.latched_takeoff_origin_ned_m,
        [10.25, -4.10, HOLD_ALTITUDE_DOWN_M]
    );

    let after = guidance
        .update(
            estimator_state_from_estimate(shifted_estimate, 0.0),
            GuidanceCommand::NavigateLeg(MissionWaypoint::new(
                [10.85, -4.10, HOLD_ALTITUDE_DOWN_M],
                0.0,
            )),
            0.10,
        )
        .expect("guidance should continue after estimator reset");
    assert!(
        (after.trajectory.position_ned_m[0] - before.trajectory.position_ned_m[0] - 0.25).abs()
            < 0.02
    );
    assert!(
        (after.trajectory.position_ned_m[1] - before.trajectory.position_ned_m[1] + 0.10).abs()
            < 0.02
    );
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once G2.5 is green"]
fn g3_square_return_to_origin_shapes_waypoints_without_position_bypass() {
    let mut guidance = GuidanceStateMachine::new(g3_trajectory_limits());
    let mut estimate = estimate_at([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], 0.0);
    let route = [
        MissionWaypoint::new([0.35, 0.35, HOLD_ALTITUDE_DOWN_M], 0.0),
        MissionWaypoint::new([0.35, -0.35, HOLD_ALTITUDE_DOWN_M], 0.0),
        MissionWaypoint::new([-0.35, -0.35, HOLD_ALTITUDE_DOWN_M], 0.0),
        MissionWaypoint::new([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], 0.0),
    ];

    let mut max_speed_mps: f32 = 0.0;
    for waypoint in route {
        for _ in 0..60 {
            let output = guidance
                .update(
                    estimator_state_from_estimate(estimate, 0.0),
                    GuidanceCommand::NavigateLeg(waypoint),
                    0.10,
                )
                .expect("finite route waypoint guidance");
            assert_eq!(output.phase, GuidancePhase::NavigateLeg);
            let setpoint = ControlSetpoint::from_local_trajectory(output.trajectory, true);
            assert_eq!(
                setpoint.source(),
                ControlSetpointSource::EstimatorLocalTrajectory
            );
            max_speed_mps = max_speed_mps.max(horizontal_norm([
                output.trajectory.velocity_ned_mps[0],
                output.trajectory.velocity_ned_mps[1],
            ]));
            estimate.position_ned_m = output.trajectory.position_ned_m;
            estimate.velocity_ned_mps = output.trajectory.velocity_ned_mps;
            if output.target_reached {
                break;
            }
        }
    }

    assert!(estimate.position_ned_m[0].abs() < 0.11);
    assert!(estimate.position_ned_m[1].abs() < 0.11);
    assert!(max_speed_mps <= g3_trajectory_limits().max_velocity_mps + 0.002);
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once G2.5 is green"]
fn g3_yawed_navigation_keeps_world_frame_motion_and_yaw_target() {
    let mut guidance = GuidanceStateMachine::new(g3_trajectory_limits());
    let pipeline = hardware_live_pipeline();
    let yaw_rad = 0.75;
    let estimate = estimate_at([0.0, 0.0, HOLD_ALTITUDE_DOWN_M], [0.0; 3], yaw_rad);
    let waypoint = MissionWaypoint::new([G3_STEP_TARGET_M, 0.0, HOLD_ALTITUDE_DOWN_M], yaw_rad);
    let guidance_output = guidance
        .update(
            estimator_state_from_estimate(estimate, yaw_rad),
            GuidanceCommand::NavigateLeg(waypoint),
            0.10,
        )
        .expect("yawed G3 guidance should produce a trajectory");

    assert!(guidance_output.trajectory.velocity_ned_mps[0] > 0.0);
    assert!(guidance_output.trajectory.velocity_ned_mps[1].abs() < 0.002);
    assert!((guidance_output.trajectory.yaw_rad - yaw_rad).abs() < 0.002);

    let output = pipeline.step(
        estimate,
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(guidance_output.trajectory, true),
    );
    assert!(output.control_valid);
    assert_eq!(
        output.debug.estimator_frame,
        ControlFrame::EstimatorLocalNed
    );
    assert!(
        output
            .debug
            .velocity_setpoint_ned_mps
            .expect("velocity setpoint")[0]
            > 0.0
    );
}

#[test]
#[ignore = "live Gazebo runtime gate: resets the simulator, uses production guidance, and scores small local navigation steps"]
fn g3_live_small_local_navigation_steps_drive_bridge_and_score_acceptance() {
    let endpoint =
        std::env::var("MARV_GAZEBO_ENDPOINT").unwrap_or_else(|_| GAZEBO_G0_DEFAULT_ENDPOINT.into());
    let requested_case = std::env::var("MARV_GAZEBO_G3_CASE").ok();
    let mut reports = Vec::new();

    for case in [
        LiveNavigationCase::new("north", 0, 1.0),
        LiveNavigationCase::new("east", 1, 1.0),
    ]
    .into_iter()
    .filter(|case| {
        requested_case
            .as_deref()
            .is_none_or(|name| name == case.name)
    }) {
        let mut attempts = Vec::new();
        for attempt in 1..=G3_LIVE_ATTEMPTS_PER_CASE {
            let mut bridge = LiveGazeboBridge::connect(&endpoint).unwrap_or_else(|error| {
                panic!("failed to connect to Gazebo bridge at {endpoint}: {error}")
            });
            let report = run_live_navigation_case(&mut bridge, case, attempt)
                .unwrap_or_else(|error| panic!("live G3 case {} failed: {error}", case.name));
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
        "one or more live G3 navigation cases failed: {:?}",
        reports
    );
}

#[test]
#[ignore = "live Gazebo runtime gate: runs multi-leg G3 mission waypoints through production guidance"]
fn g3_live_route_navigation_square_and_yawed_cases() {
    let endpoint =
        std::env::var("MARV_GAZEBO_ENDPOINT").unwrap_or_else(|_| GAZEBO_G0_DEFAULT_ENDPOINT.into());
    let requested_case = std::env::var("MARV_GAZEBO_G3_ROUTE").ok();
    let mut reports = Vec::new();

    for case in [LiveRouteCase::SquareReturn, LiveRouteCase::YawedNorth]
        .into_iter()
        .filter(|case| {
            requested_case
                .as_deref()
                .is_none_or(|name| name == case.name())
        })
    {
        let mut attempts = Vec::new();
        for attempt in 1..=G3_LIVE_ATTEMPTS_PER_CASE {
            let mut bridge = LiveGazeboBridge::connect(&endpoint).unwrap_or_else(|error| {
                panic!("failed to connect to Gazebo bridge at {endpoint}: {error}")
            });
            let report = run_live_route_case(&mut bridge, case, attempt)
                .unwrap_or_else(|error| panic!("live G3 route {} failed: {error}", case.name()));
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
        "one or more live G3 route cases failed: {:?}",
        reports
    );
}

fn navigation_acceptance_specs() -> Vec<NavigationAcceptanceSpec> {
    vec![
        NavigationAcceptanceSpec {
            id: NavigationAcceptanceId::AirborneOriginLatch,
            scenario: g3_scenario(
                "g3_airborne_origin_latch",
                "takeoff gate latches estimator-local XY before waypoint navigation starts",
            ),
        },
        NavigationAcceptanceSpec {
            id: NavigationAcceptanceId::SmallNorthStep,
            scenario: g3_scenario(
                "g3_small_north_local_step",
                "small north mission waypoint reaches bounded local target",
            ),
        },
        NavigationAcceptanceSpec {
            id: NavigationAcceptanceId::SmallEastStep,
            scenario: g3_scenario(
                "g3_small_east_local_step",
                "small east mission waypoint reaches bounded local target",
            ),
        },
        NavigationAcceptanceSpec {
            id: NavigationAcceptanceId::SquareReturnToOrigin,
            scenario: g3_scenario(
                "g3_square_return_to_origin",
                "multi-leg mission waypoints return to local origin without growing error",
            ),
        },
        NavigationAcceptanceSpec {
            id: NavigationAcceptanceId::YawedNorthStep,
            scenario: g3_scenario(
                "g3_yawed_north_local_step",
                "world-frame north waypoint remains correct while yawed",
            ),
        },
    ]
}

fn g3_scenario(id: &'static str, description: &'static str) -> ScenarioSpec {
    ScenarioSpec {
        id,
        gate: Gate::G3,
        description,
        reset_required: true,
        command_schedule: CommandSchedule::new(vec![
            ScheduledCommand {
                at_frame: 0,
                kind: CommandKind::SimReset,
            },
            ScheduledCommand {
                at_frame: LIVE_SETTLE_FRAMES,
                kind: CommandKind::SimPlay,
            },
            ScheduledCommand {
                at_frame: LIVE_SETTLE_FRAMES + 1,
                kind: CommandKind::Hold,
            },
            ScheduledCommand {
                at_frame: LIVE_SETTLE_FRAMES + 2,
                kind: CommandKind::MissionWaypoint,
            },
        ]),
        thresholds: AcceptanceThresholds {
            max_cross_track_error_m: 0.45,
            max_altitude_error_m: 1.25,
            max_roll_pitch_rad: 0.85,
            max_speed_error_mps: 2.50,
            max_clamp_ratio: 0.45,
        },
    }
}

#[derive(Clone, Copy, Debug)]
struct LiveNavigationCase {
    name: &'static str,
    axis: usize,
    sign: f32,
}

impl LiveNavigationCase {
    const fn new(name: &'static str, axis: usize, sign: f32) -> Self {
        Self { name, axis, sign }
    }

    fn id(self) -> &'static str {
        match (self.axis, self.sign > 0.0) {
            (0, true) => "g3_live_small_north_local_step",
            (1, true) => "g3_live_small_east_local_step",
            _ => "g3_live_small_local_step",
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct LiveNavigationMetrics {
    reset_clean: bool,
    estimator_agrees: bool,
    nav_started: bool,
    guidance_outputs: usize,
    progress_m: f32,
    final_target_error_m: f32,
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

impl Default for LiveNavigationMetrics {
    fn default() -> Self {
        Self {
            reset_clean: true,
            estimator_agrees: true,
            nav_started: false,
            guidance_outputs: 0,
            progress_m: 0.0,
            final_target_error_m: f32::INFINITY,
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

fn run_live_navigation_case(
    bridge: &mut LiveGazeboBridge,
    case: LiveNavigationCase,
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

    let pipeline = hardware_live_pipeline();
    let mut gate = TakeoffNavGate::new(TakeoffNavGateConfig {
        max_takeoff_down_error_m: 0.35,
        max_vertical_speed_mps: 0.35,
        max_horizontal_speed_mps: 0.25,
        min_stable_frames: 8,
        ..TakeoffNavGateConfig::default()
    });
    let mut guidance = GuidanceStateMachine::new(g3_trajectory_limits());
    let mut metrics = LiveNavigationMetrics {
        reset_clean: frames_are_monotonic_gazebo_sensor_data(&settle_frames),
        ..LiveNavigationMetrics::default()
    };
    let mut pose_velocity = PoseKinematicVelocity::default();
    let mut target = None;
    let mut nav_frames = 0usize;

    for frame_index in 0..G3_MAX_LIVE_FRAMES {
        let frame = bridge.next_sensor_frame(LIVE_SENSOR_STEP_TIMEOUT)?;
        metrics.reset_clean &= frame.clock_source.as_deref() == Some("gazebo");
        let Some(mut estimate) = frame.to_truth_estimate(origin) else {
            metrics.estimator_agrees = false;
            continue;
        };
        pose_velocity.apply(&mut estimate, frame.sim_time_us);
        let yaw_rad = frame.euler_rad.map(|euler| euler[2]).unwrap_or(0.0);
        let gate_decision = gate.update(
            takeoff_sample_from_estimate(estimate, live_clamp_ratio(metrics)),
            yaw_rad,
        );
        let (guidance_command, nav_active) = if gate_decision.nav_ready {
            let waypoint = *target.get_or_insert_with(|| {
                let mut target = gate_decision.latched_takeoff_origin_ned_m;
                target[case.axis] += case.sign * G3_STEP_TARGET_M;
                target[2] = HOLD_ALTITUDE_DOWN_M;
                target
            });
            (
                GuidanceCommand::NavigateLeg(MissionWaypoint::new(waypoint, yaw_rad)),
                true,
            )
        } else {
            (GuidanceCommand::Takeoff(gate_decision.hold_setpoint), false)
        };
        let guidance_output = guidance
            .update(
                estimator_state_from_estimate(estimate, yaw_rad),
                guidance_command,
                0.10,
            )
            .ok_or_else(|| "G3 guidance did not produce a finite trajectory".to_string())?;
        metrics.guidance_outputs += 1;
        metrics.nav_started |= nav_active;
        if nav_active {
            nav_frames += 1;
        }

        let output = pipeline.step(
            estimate,
            frame.to_truth_imu(),
            ControlSetpoint::from_local_trajectory(guidance_output.trajectory, true),
        );
        bridge.send_actuator(output.motors, Some(frame.sim_time_us))?;
        update_live_navigation_metrics(
            &mut metrics,
            case,
            &frame,
            &estimate,
            output.clamped,
            target,
            nav_active,
        );

        if frame_index > 0 && nav_frames >= G3_NAV_FRAMES {
            break;
        }
    }

    bridge.send_actuator([0.0; 4], None)?;
    println!(
        "G3 live {} attempt={} nav_started={} progress={:.3}m target_err={:.3}m cross={:.3}m alt_err={:.3}m roll_pitch={:.3}rad speed={:.3}m/s final_pos=[{:.3},{:.3},{:.3}] final_vel=[{:.3},{:.3},{:.3}] clamp_ratio={:.3}",
        case.name,
        attempt,
        metrics.nav_started,
        metrics.progress_m,
        metrics.final_target_error_m,
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

    Ok(score_live_navigation_case(case, metrics))
}

fn update_live_navigation_metrics(
    metrics: &mut LiveNavigationMetrics,
    case: LiveNavigationCase,
    frame: &GazeboSensorLine,
    estimate: &EstimateSnapshot,
    clamped: bool,
    target: Option<[f32; 3]>,
    nav_active: bool,
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
    let horizontal_speed =
        horizontal_norm([estimate.velocity_ned_mps[0], estimate.velocity_ned_mps[1]]);
    metrics.max_horizontal_speed_mps = metrics.max_horizontal_speed_mps.max(horizontal_speed);

    if let Some([roll_rad, pitch_rad, _]) = frame.euler_rad {
        metrics.max_roll_pitch_rad = metrics
            .max_roll_pitch_rad
            .max(roll_rad.abs().max(pitch_rad.abs()));
    }

    if nav_active {
        let cross_axis = if case.axis == 0 { 1 } else { 0 };
        metrics.progress_m = metrics
            .progress_m
            .max(estimate.position_ned_m[case.axis] * case.sign);
        metrics.max_cross_track_error_m = metrics
            .max_cross_track_error_m
            .max(estimate.position_ned_m[cross_axis].abs());
        if let Some(target) = target {
            metrics.final_target_error_m = horizontal_norm([
                target[0] - estimate.position_ned_m[0],
                target[1] - estimate.position_ned_m[1],
            ]);
        }
    }
}

fn score_live_navigation_case(
    case: LiveNavigationCase,
    metrics: LiveNavigationMetrics,
) -> ScenarioReport {
    let spec = ScenarioSpec {
        id: case.id(),
        gate: Gate::G3,
        description: "live Gazebo small local navigation step through production guidance",
        reset_required: true,
        command_schedule: CommandSchedule::new(vec![
            ScheduledCommand {
                at_frame: 0,
                kind: CommandKind::SimReset,
            },
            ScheduledCommand {
                at_frame: LIVE_SETTLE_FRAMES,
                kind: CommandKind::SimPlay,
            },
            ScheduledCommand {
                at_frame: LIVE_SETTLE_FRAMES + 1,
                kind: CommandKind::MissionWaypoint,
            },
        ]),
        thresholds: AcceptanceThresholds {
            max_cross_track_error_m: 0.55,
            max_altitude_error_m: 1.25,
            max_roll_pitch_rad: 0.85,
            max_speed_error_mps: 2.50,
            max_clamp_ratio: 0.45,
        },
    };
    let progress_shortfall_m = (G3_REQUIRED_PROGRESS_M - metrics.progress_m).max(0.0);
    let target_miss_m = if metrics.final_target_error_m.is_finite() {
        (metrics.final_target_error_m - 0.35).max(0.0)
    } else {
        spec.thresholds.max_cross_track_error_m
    };
    let reported_cross_track =
        metrics.max_cross_track_error_m + progress_shortfall_m + target_miss_m;
    let mut report = ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean: metrics.reset_clean,
            estimator_agrees: metrics.estimator_agrees && metrics.guidance_outputs > 0,
            max_cross_track_error_m: reported_cross_track,
            max_altitude_error_m: metrics.max_altitude_error_m,
            max_roll_pitch_rad: metrics.max_roll_pitch_rad,
            max_speed_error_mps: metrics.max_horizontal_speed_mps,
            clamp_ratio: live_clamp_ratio(metrics),
            first_clamp_source: metrics.first_clamp_source.map(str::to_string),
        },
    );
    if !metrics.nav_started {
        report.pass = false;
        report.failure_layer = Some(FailureLayer::Guidance);
    }
    report
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LiveRouteCase {
    SquareReturn,
    YawedNorth,
}

impl LiveRouteCase {
    const fn name(self) -> &'static str {
        match self {
            Self::SquareReturn => "square_return",
            Self::YawedNorth => "yawed_north",
        }
    }

    const fn id(self) -> &'static str {
        match self {
            Self::SquareReturn => "g3_live_square_return_to_origin",
            Self::YawedNorth => "g3_live_yawed_north_local_step",
        }
    }

    const fn description(self) -> &'static str {
        match self {
            Self::SquareReturn => {
                "live Gazebo square return-to-origin route through production guidance"
            }
            Self::YawedNorth => "live Gazebo yawed north waypoint through production guidance",
        }
    }

    fn targets(self, origin: [f32; 3], yaw_rad: f32) -> Vec<MissionWaypoint> {
        match self {
            Self::SquareReturn => vec![
                MissionWaypoint::new([origin[0] + 0.30, origin[1], HOLD_ALTITUDE_DOWN_M], yaw_rad),
                MissionWaypoint::new(
                    [origin[0] + 0.30, origin[1] + 0.30, HOLD_ALTITUDE_DOWN_M],
                    yaw_rad,
                ),
                MissionWaypoint::new([origin[0], origin[1] + 0.30, HOLD_ALTITUDE_DOWN_M], yaw_rad),
                MissionWaypoint::new([origin[0], origin[1], HOLD_ALTITUDE_DOWN_M], yaw_rad),
            ],
            Self::YawedNorth => vec![MissionWaypoint::new(
                [
                    origin[0] + G3_STEP_TARGET_M,
                    origin[1],
                    HOLD_ALTITUDE_DOWN_M,
                ],
                yaw_rad + 0.75,
            )],
        }
    }

    const fn required_progress_m(self) -> f32 {
        match self {
            Self::SquareReturn => 0.24,
            Self::YawedNorth => G3_REQUIRED_PROGRESS_M,
        }
    }

    const fn final_error_allowance_m(self) -> f32 {
        match self {
            Self::SquareReturn => 0.30,
            Self::YawedNorth => 0.35,
        }
    }

    const fn max_cross_track_threshold_m(self) -> f32 {
        match self {
            Self::SquareReturn => 0.55,
            Self::YawedNorth => 0.55,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct LiveRouteMetrics {
    reset_clean: bool,
    estimator_agrees: bool,
    nav_started: bool,
    guidance_outputs: usize,
    completed_legs: usize,
    progress_m: f32,
    final_target_error_m: f32,
    final_origin_error_m: f32,
    max_cross_track_error_m: f32,
    max_altitude_error_m: f32,
    max_roll_pitch_rad: f32,
    max_horizontal_speed_mps: f32,
    min_yaw_rad: f32,
    max_yaw_rad: f32,
    final_position_ned_m: [f32; 3],
    final_velocity_ned_mps: [f32; 3],
    control_steps: usize,
    clamp_steps: usize,
    first_clamp_source: Option<&'static str>,
}

impl Default for LiveRouteMetrics {
    fn default() -> Self {
        Self {
            reset_clean: true,
            estimator_agrees: true,
            nav_started: false,
            guidance_outputs: 0,
            completed_legs: 0,
            progress_m: 0.0,
            final_target_error_m: f32::INFINITY,
            final_origin_error_m: f32::INFINITY,
            max_cross_track_error_m: 0.0,
            max_altitude_error_m: 0.0,
            max_roll_pitch_rad: 0.0,
            max_horizontal_speed_mps: 0.0,
            min_yaw_rad: f32::INFINITY,
            max_yaw_rad: f32::NEG_INFINITY,
            final_position_ned_m: [0.0; 3],
            final_velocity_ned_mps: [0.0; 3],
            control_steps: 0,
            clamp_steps: 0,
            first_clamp_source: None,
        }
    }
}

fn run_live_route_case(
    bridge: &mut LiveGazeboBridge,
    case: LiveRouteCase,
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

    let pipeline = hardware_live_pipeline();
    let mut gate = TakeoffNavGate::new(TakeoffNavGateConfig {
        max_takeoff_down_error_m: 0.35,
        max_vertical_speed_mps: 0.35,
        max_horizontal_speed_mps: 0.25,
        min_stable_frames: 8,
        ..TakeoffNavGateConfig::default()
    });
    let mut guidance = GuidanceStateMachine::new(g3_trajectory_limits());
    let mut metrics = LiveRouteMetrics {
        reset_clean: frames_are_monotonic_gazebo_sensor_data(&settle_frames),
        ..LiveRouteMetrics::default()
    };
    let mut pose_velocity = PoseKinematicVelocity::default();
    let mut route: Option<Vec<MissionWaypoint>> = None;
    let mut leg_index = 0usize;
    let mut leg_frames = 0usize;
    let mut takeoff_origin = [0.0, 0.0, HOLD_ALTITUDE_DOWN_M];

    for _ in 0..G3_ROUTE_MAX_LIVE_FRAMES {
        let frame = bridge.next_sensor_frame(LIVE_SENSOR_STEP_TIMEOUT)?;
        metrics.reset_clean &= frame.clock_source.as_deref() == Some("gazebo");
        let Some(mut estimate) = frame.to_truth_estimate(origin) else {
            metrics.estimator_agrees = false;
            continue;
        };
        pose_velocity.apply(&mut estimate, frame.sim_time_us);
        let yaw_rad = frame.euler_rad.map(|euler| euler[2]).unwrap_or(0.0);
        let gate_decision = gate.update(
            takeoff_sample_from_estimate(estimate, live_route_clamp_ratio(metrics)),
            yaw_rad,
        );

        let (guidance_command, nav_active, active_target) = if gate_decision.nav_ready {
            let targets = route.get_or_insert_with(|| {
                takeoff_origin = gate_decision.latched_takeoff_origin_ned_m;
                case.targets(takeoff_origin, yaw_rad)
            });
            let active = targets[leg_index.min(targets.len() - 1)];
            (GuidanceCommand::NavigateLeg(active), true, Some(active))
        } else {
            (
                GuidanceCommand::Takeoff(gate_decision.hold_setpoint),
                false,
                None,
            )
        };
        let guidance_output = guidance
            .update(
                estimator_state_from_estimate(estimate, yaw_rad),
                guidance_command,
                0.10,
            )
            .ok_or_else(|| "G3 route guidance did not produce a finite trajectory".to_string())?;
        metrics.guidance_outputs += 1;
        metrics.nav_started |= nav_active;

        let output = pipeline.step(
            estimate,
            frame.to_truth_imu(),
            ControlSetpoint::from_local_trajectory(guidance_output.trajectory, true),
        );
        bridge.send_actuator(output.motors, Some(frame.sim_time_us))?;
        update_live_route_metrics(
            &mut metrics,
            case,
            &frame,
            &estimate,
            output.clamped,
            takeoff_origin,
            active_target,
            nav_active,
        );

        if nav_active {
            leg_frames += 1;
            if let Some(target) = active_target {
                let target_error_m = horizontal_norm([
                    target.position_ned_m[0] - estimate.position_ned_m[0],
                    target.position_ned_m[1] - estimate.position_ned_m[1],
                ]);
                let horizontal_speed_mps =
                    horizontal_norm([estimate.velocity_ned_mps[0], estimate.velocity_ned_mps[1]]);
                if target_error_m < G3_ROUTE_LEG_ACCEPTANCE_M
                    && horizontal_speed_mps < G3_ROUTE_LEG_MAX_SPEED_MPS
                    && leg_frames >= G3_ROUTE_MIN_LEG_FRAMES
                {
                    leg_index += 1;
                    pipeline.reset_position_integrators();
                    metrics.completed_legs = metrics.completed_legs.max(leg_index);
                    leg_frames = 0;
                    if let Some(targets) = route.as_ref() {
                        if leg_index >= targets.len() {
                            break;
                        }
                    }
                }
            }
        }
    }

    bridge.send_actuator([0.0; 4], None)?;
    println!(
        "G3 live route {} attempt={} nav_started={} legs={} progress={:.3}m target_err={:.3}m origin_err={:.3}m cross={:.3}m alt_err={:.3}m roll_pitch={:.3}rad speed={:.3}m/s yaw_span={:.3}rad final_pos=[{:.3},{:.3},{:.3}] final_vel=[{:.3},{:.3},{:.3}] clamp_ratio={:.3}",
        case.name(),
        attempt,
        metrics.nav_started,
        metrics.completed_legs,
        metrics.progress_m,
        metrics.final_target_error_m,
        metrics.final_origin_error_m,
        metrics.max_cross_track_error_m,
        metrics.max_altitude_error_m,
        metrics.max_roll_pitch_rad,
        metrics.max_horizontal_speed_mps,
        route_yaw_span_rad(metrics),
        metrics.final_position_ned_m[0],
        metrics.final_position_ned_m[1],
        metrics.final_position_ned_m[2],
        metrics.final_velocity_ned_mps[0],
        metrics.final_velocity_ned_mps[1],
        metrics.final_velocity_ned_mps[2],
        live_route_clamp_ratio(metrics)
    );

    Ok(score_live_route_case(case, metrics))
}

fn update_live_route_metrics(
    metrics: &mut LiveRouteMetrics,
    case: LiveRouteCase,
    frame: &GazeboSensorLine,
    estimate: &EstimateSnapshot,
    clamped: bool,
    takeoff_origin: [f32; 3],
    active_target: Option<MissionWaypoint>,
    nav_active: bool,
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
    let horizontal_speed =
        horizontal_norm([estimate.velocity_ned_mps[0], estimate.velocity_ned_mps[1]]);
    metrics.max_horizontal_speed_mps = metrics.max_horizontal_speed_mps.max(horizontal_speed);

    if let Some([roll_rad, pitch_rad, yaw_rad]) = frame.euler_rad {
        metrics.max_roll_pitch_rad = metrics
            .max_roll_pitch_rad
            .max(roll_rad.abs().max(pitch_rad.abs()));
        if nav_active {
            metrics.min_yaw_rad = metrics.min_yaw_rad.min(yaw_rad);
            metrics.max_yaw_rad = metrics.max_yaw_rad.max(yaw_rad);
        }
    }

    if nav_active {
        match case {
            LiveRouteCase::SquareReturn => {
                let dx = estimate.position_ned_m[0] - takeoff_origin[0];
                let dy = estimate.position_ned_m[1] - takeoff_origin[1];
                metrics.progress_m = metrics.progress_m.max(dx.abs()).max(dy.abs());
                let box_overshoot_m = (dx.abs() - 0.55).max(0.0).max((dy.abs() - 0.55).max(0.0));
                metrics.max_cross_track_error_m =
                    metrics.max_cross_track_error_m.max(box_overshoot_m);
            }
            LiveRouteCase::YawedNorth => {
                metrics.progress_m = metrics
                    .progress_m
                    .max(estimate.position_ned_m[0] - takeoff_origin[0]);
                metrics.max_cross_track_error_m = metrics
                    .max_cross_track_error_m
                    .max((estimate.position_ned_m[1] - takeoff_origin[1]).abs());
            }
        }
        metrics.final_origin_error_m = horizontal_norm([
            estimate.position_ned_m[0] - takeoff_origin[0],
            estimate.position_ned_m[1] - takeoff_origin[1],
        ]);
        if let Some(target) = active_target {
            metrics.final_target_error_m = horizontal_norm([
                target.position_ned_m[0] - estimate.position_ned_m[0],
                target.position_ned_m[1] - estimate.position_ned_m[1],
            ]);
        }
    }
}

fn score_live_route_case(case: LiveRouteCase, metrics: LiveRouteMetrics) -> ScenarioReport {
    let spec = ScenarioSpec {
        id: case.id(),
        gate: Gate::G3,
        description: case.description(),
        reset_required: true,
        command_schedule: CommandSchedule::new(vec![
            ScheduledCommand {
                at_frame: 0,
                kind: CommandKind::SimReset,
            },
            ScheduledCommand {
                at_frame: LIVE_SETTLE_FRAMES,
                kind: CommandKind::SimPlay,
            },
            ScheduledCommand {
                at_frame: LIVE_SETTLE_FRAMES + 1,
                kind: CommandKind::MissionWaypoint,
            },
        ]),
        thresholds: AcceptanceThresholds {
            max_cross_track_error_m: case.max_cross_track_threshold_m(),
            max_altitude_error_m: 1.25,
            max_roll_pitch_rad: 0.85,
            max_speed_error_mps: 2.50,
            max_clamp_ratio: 0.45,
        },
    };
    let required_legs: usize = match case {
        LiveRouteCase::SquareReturn => 4,
        LiveRouteCase::YawedNorth => 1,
    };
    let incomplete_penalty_m = required_legs.saturating_sub(metrics.completed_legs) as f32 * 0.50;
    let progress_shortfall_m = (case.required_progress_m() - metrics.progress_m).max(0.0);
    let target_miss_m = if metrics.final_target_error_m.is_finite() {
        (metrics.final_target_error_m - case.final_error_allowance_m()).max(0.0)
    } else {
        spec.thresholds.max_cross_track_error_m
    };
    let origin_miss_m = match case {
        LiveRouteCase::SquareReturn if metrics.final_origin_error_m.is_finite() => {
            (metrics.final_origin_error_m - case.final_error_allowance_m()).max(0.0)
        }
        _ => 0.0,
    };
    let yaw_shortfall_m = match case {
        LiveRouteCase::YawedNorth => (0.35 - route_yaw_span_rad(metrics)).max(0.0),
        LiveRouteCase::SquareReturn => 0.0,
    };
    let reported_cross_track = metrics.max_cross_track_error_m
        + incomplete_penalty_m
        + progress_shortfall_m
        + target_miss_m
        + origin_miss_m
        + yaw_shortfall_m;

    let mut report = ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean: metrics.reset_clean,
            estimator_agrees: metrics.estimator_agrees && metrics.guidance_outputs > 0,
            max_cross_track_error_m: reported_cross_track,
            max_altitude_error_m: metrics.max_altitude_error_m,
            max_roll_pitch_rad: metrics.max_roll_pitch_rad,
            max_speed_error_mps: metrics.max_horizontal_speed_mps,
            clamp_ratio: live_route_clamp_ratio(metrics),
            first_clamp_source: metrics.first_clamp_source.map(str::to_string),
        },
    );
    if !metrics.nav_started {
        report.pass = false;
        report.failure_layer = Some(FailureLayer::Guidance);
    }
    report
}

fn route_yaw_span_rad(metrics: LiveRouteMetrics) -> f32 {
    if !metrics.min_yaw_rad.is_finite() || !metrics.max_yaw_rad.is_finite() {
        return 0.0;
    }
    metrics.max_yaw_rad - metrics.min_yaw_rad
}

fn live_route_clamp_ratio(metrics: LiveRouteMetrics) -> f32 {
    if metrics.control_steps == 0 {
        return 0.0;
    }
    metrics.clamp_steps as f32 / metrics.control_steps as f32
}

fn hardware_live_pipeline() -> FlightControlPipeline {
    let airframe = GazeboAirframeConfig::parse(AIRFRAME_CONFIG).expect("airframe config parses");
    airframe
        .validate()
        .expect("airframe config remains the hardware source of truth");
    let mut loop_config = g3_loop_config();
    loop_config.altitude.hover_throttle = airframe.hover_motor_command();
    FlightControlPipeline::new(common::control::pipeline::FlightControlConfig::new(
        loop_config,
        airframe.runtime_motor_geometry(),
    ))
}

fn g3_loop_config() -> ControlLoopConfig {
    let mut config = ControlLoopConfig::default();
    config.position = PositionControllerConfig {
        position_gain: 0.14,
        velocity_gain: 0.38,
        max_horizontal_velocity_mps: 0.70,
        max_horizontal_accel_mps2: 0.70,
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
        max_axis_command: 0.15,
        measured_rate_deadband_rps: 0.015,
        ..RateControllerConfig::default()
    };
    config
}

fn g3_trajectory_limits() -> TrajectoryLimits {
    TrajectoryLimits {
        max_velocity_mps: 0.40,
        max_accel_mps2: 0.45,
        max_jerk_mps3: 2.5,
        acceptance_radius_m: 0.10,
        max_yaw_rate_rad_s: 45.0_f32.to_radians(),
    }
}

fn g3_route_trajectory_limits() -> TrajectoryLimits {
    TrajectoryLimits {
        max_velocity_mps: 0.22,
        max_accel_mps2: 0.22,
        max_jerk_mps3: 1.2,
        acceptance_radius_m: 0.08,
        max_yaw_rate_rad_s: 35.0_f32.to_radians(),
    }
}

fn estimate_at(
    position_ned_m: [f32; 3],
    velocity_ned_mps: [f32; 3],
    yaw_rad: f32,
) -> EstimateSnapshot {
    EstimateSnapshot {
        position_ned_m,
        velocity_ned_mps,
        quaternion: AttitudeSetpoint::from_euler_rad(0.0, 0.0, yaw_rad)
            .expect("finite yaw should produce a quaternion")
            .quaternion,
        valid: true,
    }
}

fn estimator_state_from_estimate(estimate: EstimateSnapshot, yaw_rad: f32) -> EstimatorLocalState {
    EstimatorLocalState::new(
        estimate.position_ned_m,
        estimate.velocity_ned_mps,
        yaw_rad,
        estimate.valid,
    )
}

fn takeoff_sample_from_estimate(
    estimate: EstimateSnapshot,
    clamp_ratio: f32,
) -> TakeoffNavGateSample {
    TakeoffNavGateSample {
        estimate_position_ned_m: estimate.position_ned_m,
        estimate_velocity_ned_mps: estimate.velocity_ned_mps,
        truth_position_ned_m: Some(estimate.position_ned_m),
        truth_velocity_ned_mps: Some(estimate.velocity_ned_mps),
        yaw_error_rad: Some(0.0),
        clamp_ratio,
        estimate_valid: estimate.valid,
    }
}

fn horizontal_norm(values: [f32; 2]) -> f32 {
    (values[0] * values[0] + values[1] * values[1]).sqrt()
}

fn live_clamp_ratio(metrics: LiveNavigationMetrics) -> f32 {
    if metrics.control_steps == 0 {
        return 0.0;
    }
    metrics.clamp_steps as f32 / metrics.control_steps as f32
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
