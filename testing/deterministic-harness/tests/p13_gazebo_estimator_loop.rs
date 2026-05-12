use std::env;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::{Duration, Instant};

use common::control::config::ControlLoopConfig;
use common::control::guidance::{TakeoffNavGate, TakeoffNavGateConfig, TakeoffNavGateSample};
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
const DEFAULT_MAX_DOWN_ERROR_M: f32 = 3.25;
const DEFAULT_MAX_HORIZONTAL_ERROR_M: f32 = 0.25;
const DEFAULT_MAX_CLAMP_RATIO: f32 = 0.25;
const DEFAULT_MAX_VERTICAL_SPEED_MPS: f32 = 6.0;
const DEFAULT_MAX_DISTURBANCE_HORIZONTAL_ERROR_M: f32 = 1.50;
const DEFAULT_MAX_LIMITED_HORIZONTAL_ERROR_M: f32 = 6.0;
const DEFAULT_MAX_LANDING_IMPACT_SPEED_MPS: f32 = 2.5;
const DEFAULT_GROUND_CONTACT_EPSILON_M: f32 = 0.05;
const DEFAULT_MIN_LANDING_SPOOLDOWN_FRAMES: usize = 75;
const DEFAULT_VERTICAL_STEP_DOWN_M: f32 = -3.0;
const DEFAULT_MIN_VERTICAL_IMPROVEMENT_M: f32 = 0.05;
const DEFAULT_MIN_YAW_IMPROVEMENT_RAD: f32 = 0.05;
const DEFAULT_MIN_DISTURBANCE_YAW_RAD: f32 = 30.0_f32.to_radians();
const DEFAULT_MIN_LIMITED_SETPOINT_M: f32 = 10.0;
const DEFAULT_MAX_FINAL_DOWN_ERROR_M: f32 = 1.25;
const DEFAULT_GYRO_DEADBAND_RPS: f32 = 0.02;
const DEFAULT_MAX_HOVER_MOTOR_SPREAD: f32 = 0.01;
const DEFAULT_POSITION_GAIN: f32 = 0.12;
const DEFAULT_POSITION_VELOCITY_GAIN: f32 = 0.80;
const DEFAULT_POSITION_MAX_HORIZONTAL_VELOCITY_MPS: f32 = 0.5;
const DEFAULT_POSITION_HORIZONTAL_INTEGRAL_GAIN: f32 = 0.0;
const DEFAULT_POSITION_HORIZONTAL_INTEGRAL_LEAK: f32 = 0.999;
const DEFAULT_POSITION_MAX_HORIZONTAL_INTEGRAL_ACCEL_MPS2: f32 = 0.45;
const DEFAULT_MAX_HORIZONTAL_ACCEL_MPS2: f32 = 3.0;
const DEFAULT_MAX_TILT_RAD: f32 = 20.0_f32.to_radians();
const DEFAULT_MAX_YAW_RATE_RPS: f32 = 0.03;
const DEFAULT_ALTITUDE_GAIN: f32 = 0.16;
const DEFAULT_VERTICAL_VELOCITY_GAIN: f32 = 0.12;
const DEFAULT_MAX_THROTTLE_CORRECTION: f32 = 0.35;
const DEFAULT_USE_MAGNETOMETER: bool = false;
const DEFAULT_MIN_NAVIGATION_PROGRESS_M: f32 = 0.35;
const DEFAULT_MAX_NAVIGATION_FINAL_ERROR_M: f32 = 0.85;
const DEFAULT_MAX_NAVIGATION_CROSS_TRACK_M: f32 = 0.45;
const DEFAULT_MAX_NAVIGATION_OVERSHOOT_M: f32 = 0.75;
const DEFAULT_MAX_NAVIGATION_HORIZONTAL_SPEED_MPS: f32 = 1.25;
const DEFAULT_NAVIGATION_TAKEOFF_DOWN_M: f32 = -2.0;
const DEFAULT_MIN_NAVIGATION_AIRBORNE_M: f32 = 0.5;
const DEFAULT_NAVIGATION_LEG_FRAMES: usize = 800;
const DEFAULT_NAVIGATION_SETPOINT_SLEW_M_PER_FRAME: f32 = 1.0;
const DEFAULT_MIN_NAVIGATION_SETTLE_FRAMES: usize = 20;
const DEFAULT_MAX_NAVIGATION_START_VERTICAL_SPEED_MPS: f32 = 0.25;
const DEFAULT_MAX_NAVIGATION_START_HORIZONTAL_SPEED_MPS: f32 = 0.15;
const DEFAULT_MAX_NAVIGATION_START_ESTIMATE_TRUTH_DOWN_ERROR_M: f32 = 0.35;
const DEFAULT_MAX_NAVIGATION_START_YAW_ERROR_RAD: f32 = 5.0_f32.to_radians();
const DEFAULT_G3_ALTITUDE_GAIN: f32 = 0.24;
const DEFAULT_G3_VERTICAL_VELOCITY_GAIN: f32 = 0.24;
const DEFAULT_G3_MAX_THROTTLE_CORRECTION: f32 = 0.45;
const DEFAULT_G3_MAX_YAW_RATE_RPS: f32 = 0.25;

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
        ControlSetpoint::local_position_ned([0.0, 0.0, settings.vertical_step_down_m], 0.0, true);
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

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_BRIDGE_ADDR if not 127.0.0.1:9000"]
fn p14_gazebo_g2_control_hardening() {
    let settings = RuntimeSettings::from_env();
    let airframe = airframe_config();

    let origin = run_control_scenario(
        "p14_origin_hold_baseline",
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        &settings,
        &airframe,
    );
    assert_g2_runtime_envelope("p14 origin", origin, &settings);
    assert!(
        origin.max_motor_spread <= settings.max_hover_motor_spread,
        "P14 origin hover motor chatter too high: summary={origin:?}"
    );

    for target_down_m in [-0.5, -1.0, -2.0, -3.0] {
        let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, target_down_m], 0.0, true);
        let summary = run_control_scenario(
            &format!("p14_altitude_ladder_{target_down_m:.1}m"),
            setpoint,
            &settings,
            &airframe,
        );

        assert_g2_runtime_envelope("p14 altitude ladder", summary, &settings);
        assert!(
            summary.max_truth_vertical_speed_mps <= settings.max_vertical_speed_mps,
            "P14 altitude ladder truth vertical speed too high: target_down_m={target_down_m} summary={summary:?}"
        );
        assert!(
            summary.max_estimate_vertical_speed_mps <= settings.max_vertical_speed_mps,
            "P14 altitude ladder estimate vertical speed too high: target_down_m={target_down_m} summary={summary:?}"
        );
        assert!(
            summary.final_truth_down_error_m.abs() <= summary.initial_truth_down_error_m.abs(),
            "P14 altitude ladder truth moved away from target: target_down_m={target_down_m} summary={summary:?}"
        );
        if target_down_m.abs() > 1.0 {
            assert!(
                summary.final_estimate_down_error_m.abs()
                    <= summary.initial_estimate_down_error_m.abs(),
                "P14 altitude ladder estimate moved away from target: target_down_m={target_down_m} summary={summary:?}"
            );
        } else {
            assert!(
                summary.final_estimate_down_error_m.abs() <= settings.max_final_down_error_m,
                "P14 small altitude step estimate did not stay bounded near pad: target_down_m={target_down_m} summary={summary:?}"
            );
        }
    }

    for yaw_rad in [
        45.0_f32.to_radians(),
        90.0_f32.to_radians(),
        180.0_f32.to_radians(),
    ] {
        let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, -2.0], yaw_rad, true);
        let summary = run_control_scenario(
            &format!("p14_yawed_hover_{:.0}deg", yaw_rad.to_degrees()),
            setpoint,
            &settings,
            &airframe,
        );

        assert_g2_runtime_envelope("p14 yawed hover", summary, &settings);
        assert!(
            summary.final_yaw_error_rad.abs()
                <= summary.initial_yaw_error_rad.abs() - settings.min_yaw_improvement_rad,
            "P14 yawed hover did not reduce yaw error enough: yaw_rad={yaw_rad} summary={summary:?}"
        );
        assert!(
            summary.final_truth_down_error_m.abs() <= summary.initial_truth_down_error_m.abs(),
            "P14 yawed hover moved away from altitude target: yaw_rad={yaw_rad} summary={summary:?}"
        );
    }
}

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_BRIDGE_ADDR if not 127.0.0.1:9000"]
fn p14_gazebo_g2_runtime_disturbance_and_demand_limits() {
    let settings = RuntimeSettings::from_env();
    let airframe = airframe_config();

    let yaw_disturbance = run_control_schedule(
        "p14_runtime_yaw_disturbance_return",
        |control_frame| {
            if control_frame < settings.frames / 3 {
                ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true)
            } else if control_frame < (settings.frames * 2) / 3 {
                ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 45.0_f32.to_radians(), true)
            } else {
                ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true)
            }
        },
        &settings,
        &airframe,
    );

    assert_g2_runtime_limit_evidence("p14 runtime yaw disturbance", yaw_disturbance, &settings);
    assert!(
        yaw_disturbance.max_abs_setpoint_yaw_rad >= settings.min_disturbance_yaw_rad,
        "P14 runtime disturbance did not apply a meaningful yaw setpoint: summary={yaw_disturbance:?}"
    );
    assert!(
        yaw_disturbance.max_truth_horizontal_error_m <= settings.max_disturbance_horizontal_error_m,
        "P14 runtime disturbance truth response escaped the bounded recovery envelope: summary={yaw_disturbance:?}"
    );
    assert!(
        yaw_disturbance.max_estimate_horizontal_error_m
            <= settings.max_disturbance_horizontal_error_m,
        "P14 runtime disturbance estimate response escaped the bounded recovery envelope: summary={yaw_disturbance:?}"
    );
    assert!(
        yaw_disturbance.final_truth_origin_horizontal_m
            <= settings.max_disturbance_horizontal_error_m * 0.5,
        "P14 runtime disturbance did not return near origin truth hold: summary={yaw_disturbance:?}"
    );
    assert!(
        yaw_disturbance.max_truth_origin_horizontal_m
            <= settings.max_disturbance_horizontal_error_m,
        "P14 runtime disturbance truth moved too far from origin: summary={yaw_disturbance:?}"
    );
    assert!(
        yaw_disturbance.final_estimate_origin_horizontal_m
            <= settings.max_disturbance_horizontal_error_m * 0.5,
        "P14 runtime disturbance did not return near origin estimate hold: summary={yaw_disturbance:?}"
    );
    assert!(
        yaw_disturbance.max_estimate_origin_horizontal_m
            <= settings.max_disturbance_horizontal_error_m,
        "P14 runtime disturbance estimate moved too far from origin: summary={yaw_disturbance:?}"
    );

    let demand_limited = run_control_schedule(
        "p14_runtime_extreme_demand_limited",
        |control_frame| {
            let pulse_start = settings.frames / 3;
            if control_frame < pulse_start {
                ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true)
            } else if control_frame < pulse_start + 5 {
                ControlSetpoint::local_position_ned(
                    [50.0, -50.0, -2.0],
                    180.0_f32.to_radians(),
                    true,
                )
            } else {
                ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true)
            }
        },
        &settings,
        &airframe,
    );

    assert_g2_runtime_limit_evidence("p14 runtime demand limit", demand_limited, &settings);
    assert!(
        demand_limited.max_setpoint_horizontal_m >= settings.min_limited_setpoint_m,
        "P14 runtime demand-limit case did not apply an extreme setpoint: summary={demand_limited:?}"
    );
    assert!(
        demand_limited.max_truth_origin_horizontal_m <= settings.max_limited_horizontal_error_m,
        "P14 runtime demand-limit truth response escaped the bounded envelope: summary={demand_limited:?}"
    );
    assert!(
        demand_limited.max_estimate_origin_horizontal_m <= settings.max_limited_horizontal_error_m,
        "P14 runtime demand-limit estimate response escaped the bounded envelope: summary={demand_limited:?}"
    );
}

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_BRIDGE_ADDR if not 127.0.0.1:9000"]
fn p14_gazebo_g2_runtime_lateral_setpoint_disturbance() {
    let mut settings = RuntimeSettings::from_env();
    settings.frames = settings.frames.max(750);
    let airframe = airframe_config();

    let lateral = run_control_schedule(
        "p14_runtime_lateral_setpoint_disturbance",
        |control_frame| {
            if control_frame < settings.frames / 3 {
                ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true)
            } else if control_frame < (settings.frames * 2) / 3 {
                ControlSetpoint::local_position_ned([0.75, 0.0, -1.0], 0.0, true)
            } else {
                ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true)
            }
        },
        &settings,
        &airframe,
    );

    assert_g2_runtime_limit_evidence("p14 runtime lateral setpoint", lateral, &settings);
    assert!(
        lateral.max_setpoint_horizontal_m >= 0.75,
        "P14 runtime lateral disturbance did not apply the target setpoint: summary={lateral:?}"
    );
    assert!(
        lateral.max_truth_origin_horizontal_m <= settings.max_disturbance_horizontal_error_m,
        "P14 runtime lateral disturbance truth moved too far from origin: summary={lateral:?}"
    );
    assert!(
        lateral.final_truth_origin_horizontal_m
            <= settings.max_disturbance_horizontal_error_m * 0.5,
        "P14 runtime lateral disturbance did not return near origin truth hold: summary={lateral:?}"
    );
    assert!(
        lateral.max_estimate_origin_horizontal_m <= settings.max_disturbance_horizontal_error_m,
        "P14 runtime lateral disturbance estimate moved too far from origin: summary={lateral:?}"
    );
    assert!(
        lateral.final_estimate_origin_horizontal_m
            <= settings.max_disturbance_horizontal_error_m * 0.5,
        "P14 runtime lateral disturbance did not return near origin estimate hold: summary={lateral:?}"
    );
}

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_BRIDGE_ADDR if not 127.0.0.1:9000"]
fn p15_gazebo_takeoff_hover_land() {
    let mut settings = RuntimeSettings::from_env();
    settings.frames = settings.frames.max(1_200);
    let airframe = airframe_config();

    let landing = run_takeoff_hover_land_mission("p15_takeoff_hover_land", &settings, &airframe);

    assert!(
        landing.converted_frames >= landing.control_frames,
        "P15 converted fewer frames than it controlled: summary={landing:?}"
    );
    assert!(
        landing.first_contact_frame.is_some(),
        "P15 never reached near-ground/contact truth state: summary={landing:?}"
    );
    assert!(
        landing.min_truth_down_m <= -0.75,
        "P15 staged takeoff did not climb high enough: summary={landing:?}"
    );
    assert!(
        landing.max_hover_truth_down_error_m <= settings.max_final_down_error_m,
        "P15 hover hold wandered outside altitude band: summary={landing:?}"
    );
    assert!(
        landing.max_attitude_rad <= settings.max_attitude_rad,
        "P15 attitude diverged: summary={landing:?}"
    );
    assert!(
        landing.max_truth_origin_horizontal_m <= settings.max_disturbance_horizontal_error_m,
        "P15 truth moved too far from origin during vertical mission: summary={landing:?}"
    );
    assert!(
        landing.max_estimate_origin_horizontal_m <= settings.max_disturbance_horizontal_error_m,
        "P15 estimate moved too far from origin during vertical mission: summary={landing:?}"
    );
    assert!(
        landing.max_truth_vertical_speed_mps <= settings.max_vertical_speed_mps,
        "P15 truth vertical speed too high: summary={landing:?}"
    );
    assert!(
        landing.max_estimate_vertical_speed_mps <= settings.max_vertical_speed_mps,
        "P15 estimate vertical speed too high: summary={landing:?}"
    );
    assert!(
        landing.max_landing_impact_speed_mps <= settings.max_landing_impact_speed_mps,
        "P15 landing impact was too hard: summary={landing:?}"
    );
    assert!(
        landing.clamp_ratio <= settings.max_clamp_ratio,
        "P15 clamped too often: summary={landing:?}"
    );
    assert_eq!(
        landing.pre_contact_zero_motor_frames, 0,
        "P15 motors went idle before contact criteria were met: summary={landing:?}"
    );
    assert!(
        landing.post_contact_frames >= settings.min_landing_spooldown_frames,
        "P15 did not collect enough post-contact spooldown evidence: summary={landing:?}"
    );
    assert!(
        landing.post_contact_max_motor <= 0.001,
        "P15 motors did not spool down after contact: summary={landing:?}"
    );
    assert!(
        landing.final_truth_down_m.abs() <= settings.ground_contact_epsilon_m,
        "P15 final truth did not remain near the pad: summary={landing:?}"
    );
    assert!(
        landing.final_truth_vertical_speed_mps.abs() <= settings.max_landing_impact_speed_mps,
        "P15 final truth vertical speed relaunched or fell through the pad: summary={landing:?}"
    );
    assert!(
        landing.final_estimate_down_m.abs() <= settings.max_final_down_error_m,
        "P15 final estimate did not remain near the pad: summary={landing:?}"
    );
    assert!(
        !landing.final_armed && !landing.final_control_valid,
        "P15 final command should be disarmed zero-output: summary={landing:?}"
    );
}

#[test]
#[ignore = "requires Gazebo and cerberus_gazebo_bridge running; set MARV_GAZEBO_G3_AUTO_RESET=1 for reset/play"]
fn p17_gazebo_navigation_validation() {
    let mut settings = RuntimeSettings::from_env();
    apply_g3_navigation_env(&mut settings);
    settings.frames = settings.frames.max(2_400);
    let airframe = airframe_config();

    let north = run_airborne_navigation_scenario(
        "p17_navigation_north_step",
        [1.0, 0.0],
        0.0,
        &settings,
        &airframe,
    );
    assert_navigation_mission("p17 north step", north, [1.0, 0.0], &settings);

    let east = run_airborne_navigation_scenario(
        "p17_navigation_east_step",
        [0.0, 1.0],
        0.0,
        &settings,
        &airframe,
    );
    assert_navigation_mission("p17 east step", east, [0.0, 1.0], &settings);

    let diagonal = run_airborne_navigation_scenario(
        "p17_navigation_diagonal_step",
        [1.0, 1.0],
        0.0,
        &settings,
        &airframe,
    );
    assert_navigation_mission("p17 diagonal step", diagonal, [1.0, 1.0], &settings);

    let yawed = run_airborne_navigation_scenario(
        "p17_navigation_yawed_north_step",
        [1.0, 0.0],
        45.0_f32.to_radians(),
        &settings,
        &airframe,
    );
    assert_navigation_mission("p17 yawed north step", yawed, [1.0, 0.0], &settings);
    assert!(
        yawed.max_abs_setpoint_yaw_rad >= 30.0_f32.to_radians(),
        "P17 yawed navigation did not apply a meaningful yaw setpoint: summary={yawed:?}"
    );

    let square = run_airborne_navigation_schedule(
        "p17_navigation_small_waypoint_square",
        |nav_frame, nav_frames| {
            let leg = nav_frame / (nav_frames / 4).max(1);
            match leg {
                0 => ([1.0, 0.0], 0.0),
                1 => ([1.0, 1.0], 0.0),
                2 => ([0.0, 1.0], 0.0),
                _ => ([0.0, 0.0], 0.0),
            }
        },
        &settings,
        &airframe,
    );
    assert_navigation_mission("p17 waypoint square", square, [0.0, 0.0], &settings);
    assert!(
        square.max_setpoint_horizontal_m >= 2.0_f32.sqrt(),
        "P17 waypoint square did not apply the diagonal leg: summary={square:?}"
    );
    assert!(
        square.final_truth_target_error_m <= settings.max_navigation_final_error_m(),
        "P17 waypoint square did not return near airborne origin: summary={square:?}"
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
    max_truth_origin_horizontal_m: f32,
    final_truth_origin_horizontal_m: f32,
    max_estimate_origin_horizontal_m: f32,
    final_estimate_origin_horizontal_m: f32,
    max_truth_vertical_speed_mps: f32,
    max_estimate_vertical_speed_mps: f32,
    clamp_ratio: f32,
    initial_truth_down_error_m: f32,
    final_truth_down_error_m: f32,
    initial_estimate_down_error_m: f32,
    final_estimate_down_error_m: f32,
    initial_yaw_error_rad: f32,
    final_yaw_error_rad: f32,
    final_truth_position_ned_m: [f32; 3],
    final_truth_velocity_ned_mps: [f32; 3],
    final_estimate_position_ned_m: [f32; 3],
    final_estimate_velocity_ned_mps: [f32; 3],
    mean_throttle: f32,
    mean_motor: f32,
    max_motor: f32,
    mean_motor_spread: f32,
    max_motor_spread: f32,
    max_setpoint_horizontal_m: f32,
    max_abs_setpoint_yaw_rad: f32,
    max_attitude_setpoint_tilt_rad: f32,
    final_attitude_setpoint_roll_pitch_rad: [f32; 2],
    final_truth_roll_pitch_rad: [f32; 2],
    max_yaw_rate_setpoint_rps: f32,
    max_throttle_correction: f32,
    max_axis_command: f32,
}

#[derive(Clone, Copy, Debug)]
struct TakeoffHoverLandSummary {
    converted_frames: usize,
    control_frames: usize,
    first_contact_frame: Option<usize>,
    post_contact_frames: usize,
    max_attitude_rad: f32,
    min_truth_down_m: f32,
    max_hover_truth_down_error_m: f32,
    max_truth_origin_horizontal_m: f32,
    max_estimate_origin_horizontal_m: f32,
    max_truth_vertical_speed_mps: f32,
    max_estimate_vertical_speed_mps: f32,
    max_landing_impact_speed_mps: f32,
    clamp_ratio: f32,
    pre_contact_zero_motor_frames: usize,
    post_contact_max_motor: f32,
    final_truth_down_m: f32,
    final_estimate_down_m: f32,
    final_truth_vertical_speed_mps: f32,
    final_armed: bool,
    final_control_valid: bool,
}

#[derive(Clone, Copy, Debug)]
struct NavigationMissionSummary {
    converted_frames: usize,
    control_frames: usize,
    nav_start_frame: usize,
    nav_frames: usize,
    nav_origin_truth_position_ned_m: [f32; 3],
    nav_origin_estimate_position_ned_m: [f32; 3],
    final_gate_truth_position_ned_m: [f32; 3],
    final_gate_estimate_position_ned_m: [f32; 3],
    final_gate_truth_velocity_ned_mps: [f32; 3],
    final_gate_estimate_velocity_ned_mps: [f32; 3],
    final_gate_truth_horizontal_speed_mps: f32,
    final_gate_estimate_horizontal_speed_mps: f32,
    final_truth_delta_ned_m: [f32; 3],
    final_estimate_delta_ned_m: [f32; 3],
    max_attitude_rad: f32,
    max_truth_vertical_speed_mps: f32,
    max_estimate_vertical_speed_mps: f32,
    max_truth_horizontal_speed_mps: f32,
    max_estimate_horizontal_speed_mps: f32,
    max_truth_cross_track_m: f32,
    max_estimate_cross_track_m: f32,
    max_truth_yaw_error_rad: f32,
    final_truth_yaw_error_rad: f32,
    max_estimate_yaw_error_rad: f32,
    final_estimate_yaw_error_rad: f32,
    max_estimate_truth_horizontal_error_m: f32,
    final_estimate_truth_horizontal_error_m: f32,
    final_truth_track_progress_m: f32,
    final_estimate_track_progress_m: f32,
    final_truth_cross_track_m: f32,
    final_estimate_cross_track_m: f32,
    final_truth_target_error_m: f32,
    final_estimate_target_error_m: f32,
    final_truth_overshoot_m: f32,
    final_estimate_overshoot_m: f32,
    final_truth_down_error_m: f32,
    final_estimate_down_error_m: f32,
    clamp_ratio: f32,
    max_setpoint_horizontal_m: f32,
    max_abs_setpoint_yaw_rad: f32,
    max_attitude_setpoint_tilt_rad: f32,
    max_yaw_rate_setpoint_rps: f32,
    max_throttle_correction: f32,
    max_axis_command: f32,
    max_motor: f32,
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
    max_vertical_speed_mps: f32,
    max_disturbance_horizontal_error_m: f32,
    max_limited_horizontal_error_m: f32,
    max_landing_impact_speed_mps: f32,
    ground_contact_epsilon_m: f32,
    min_landing_spooldown_frames: usize,
    vertical_step_down_m: f32,
    min_vertical_improvement_m: f32,
    min_yaw_improvement_rad: f32,
    min_disturbance_yaw_rad: f32,
    min_limited_setpoint_m: f32,
    max_final_down_error_m: f32,
    gyro_deadband_rps: f32,
    max_hover_motor_spread: f32,
    position_gain: f32,
    position_velocity_gain: f32,
    position_max_horizontal_velocity_mps: f32,
    position_horizontal_integral_gain: f32,
    position_horizontal_integral_leak: f32,
    position_max_horizontal_integral_accel_mps2: f32,
    max_horizontal_accel_mps2: f32,
    max_tilt_rad: f32,
    max_yaw_rate_rps: f32,
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
            max_vertical_speed_mps: env_f32("MARV_GAZEBO_G2_MAX_VERTICAL_SPEED_MPS")
                .unwrap_or(DEFAULT_MAX_VERTICAL_SPEED_MPS),
            max_disturbance_horizontal_error_m: env_f32(
                "MARV_GAZEBO_G2_MAX_DISTURBANCE_HORIZONTAL_ERROR_M",
            )
            .unwrap_or(DEFAULT_MAX_DISTURBANCE_HORIZONTAL_ERROR_M),
            max_limited_horizontal_error_m: env_f32(
                "MARV_GAZEBO_G2_MAX_LIMITED_HORIZONTAL_ERROR_M",
            )
            .unwrap_or(DEFAULT_MAX_LIMITED_HORIZONTAL_ERROR_M),
            max_landing_impact_speed_mps: env_f32("MARV_GAZEBO_G2_MAX_LANDING_IMPACT_SPEED_MPS")
                .unwrap_or(DEFAULT_MAX_LANDING_IMPACT_SPEED_MPS),
            ground_contact_epsilon_m: env_f32("MARV_GAZEBO_G2_GROUND_CONTACT_EPSILON_M")
                .unwrap_or(DEFAULT_GROUND_CONTACT_EPSILON_M),
            min_landing_spooldown_frames: env_usize("MARV_GAZEBO_G2_MIN_LANDING_SPOOLDOWN_FRAMES")
                .unwrap_or(DEFAULT_MIN_LANDING_SPOOLDOWN_FRAMES),
            vertical_step_down_m: env_f32("MARV_GAZEBO_G2_VERTICAL_STEP_DOWN_M")
                .unwrap_or(DEFAULT_VERTICAL_STEP_DOWN_M),
            min_vertical_improvement_m: env_f32("MARV_GAZEBO_G2_MIN_VERTICAL_IMPROVEMENT_M")
                .unwrap_or(DEFAULT_MIN_VERTICAL_IMPROVEMENT_M),
            min_yaw_improvement_rad: env_f32("MARV_GAZEBO_G2_MIN_YAW_IMPROVEMENT_RAD")
                .unwrap_or(DEFAULT_MIN_YAW_IMPROVEMENT_RAD),
            min_disturbance_yaw_rad: env_f32("MARV_GAZEBO_G2_MIN_DISTURBANCE_YAW_RAD")
                .unwrap_or(DEFAULT_MIN_DISTURBANCE_YAW_RAD),
            min_limited_setpoint_m: env_f32("MARV_GAZEBO_G2_MIN_LIMITED_SETPOINT_M")
                .unwrap_or(DEFAULT_MIN_LIMITED_SETPOINT_M),
            max_final_down_error_m: env_f32("MARV_GAZEBO_G2_MAX_FINAL_DOWN_ERROR_M")
                .unwrap_or(DEFAULT_MAX_FINAL_DOWN_ERROR_M),
            gyro_deadband_rps: env_f32("MARV_GAZEBO_G2_GYRO_DEADBAND_RPS")
                .unwrap_or(DEFAULT_GYRO_DEADBAND_RPS),
            max_hover_motor_spread: env_f32("MARV_GAZEBO_G2_MAX_HOVER_MOTOR_SPREAD")
                .unwrap_or(DEFAULT_MAX_HOVER_MOTOR_SPREAD),
            position_gain: env_f32("MARV_GAZEBO_G2_POSITION_GAIN").unwrap_or(DEFAULT_POSITION_GAIN),
            position_velocity_gain: env_f32("MARV_GAZEBO_G2_POSITION_VELOCITY_GAIN")
                .unwrap_or(DEFAULT_POSITION_VELOCITY_GAIN),
            position_max_horizontal_velocity_mps: env_f32(
                "MARV_GAZEBO_G2_POSITION_MAX_HORIZONTAL_VELOCITY_MPS",
            )
            .unwrap_or(DEFAULT_POSITION_MAX_HORIZONTAL_VELOCITY_MPS),
            position_horizontal_integral_gain: env_f32(
                "MARV_GAZEBO_G2_POSITION_HORIZONTAL_INTEGRAL_GAIN",
            )
            .unwrap_or(DEFAULT_POSITION_HORIZONTAL_INTEGRAL_GAIN),
            position_horizontal_integral_leak: env_f32(
                "MARV_GAZEBO_G2_POSITION_HORIZONTAL_INTEGRAL_LEAK",
            )
            .unwrap_or(DEFAULT_POSITION_HORIZONTAL_INTEGRAL_LEAK),
            position_max_horizontal_integral_accel_mps2: env_f32(
                "MARV_GAZEBO_G2_POSITION_MAX_HORIZONTAL_INTEGRAL_ACCEL_MPS2",
            )
            .unwrap_or(DEFAULT_POSITION_MAX_HORIZONTAL_INTEGRAL_ACCEL_MPS2),
            max_horizontal_accel_mps2: env_f32("MARV_GAZEBO_G2_MAX_HORIZONTAL_ACCEL_MPS2")
                .unwrap_or(DEFAULT_MAX_HORIZONTAL_ACCEL_MPS2),
            max_tilt_rad: env_f32("MARV_GAZEBO_G2_MAX_TILT_RAD").unwrap_or(DEFAULT_MAX_TILT_RAD),
            max_yaw_rate_rps: env_f32("MARV_GAZEBO_G2_MAX_YAW_RATE_RPS")
                .unwrap_or(DEFAULT_MAX_YAW_RATE_RPS),
            altitude_gain: env_f32("MARV_GAZEBO_G2_ALTITUDE_GAIN").unwrap_or(DEFAULT_ALTITUDE_GAIN),
            vertical_velocity_gain: env_f32("MARV_GAZEBO_G2_VERTICAL_VELOCITY_GAIN")
                .unwrap_or(DEFAULT_VERTICAL_VELOCITY_GAIN),
            max_throttle_correction: env_f32("MARV_GAZEBO_G2_MAX_THROTTLE_CORRECTION")
                .unwrap_or(DEFAULT_MAX_THROTTLE_CORRECTION),
            use_magnetometer: env_bool("MARV_GAZEBO_G2_USE_MAGNETOMETER")
                .unwrap_or(DEFAULT_USE_MAGNETOMETER),
        }
    }

    fn min_navigation_progress_m(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MIN_NAVIGATION_PROGRESS_M")
            .unwrap_or(DEFAULT_MIN_NAVIGATION_PROGRESS_M)
            .max(0.0)
    }

    fn max_navigation_final_error_m(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MAX_NAVIGATION_FINAL_ERROR_M")
            .unwrap_or(DEFAULT_MAX_NAVIGATION_FINAL_ERROR_M)
            .max(0.0)
    }

    fn max_navigation_cross_track_m(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MAX_NAVIGATION_CROSS_TRACK_M")
            .unwrap_or(DEFAULT_MAX_NAVIGATION_CROSS_TRACK_M)
            .max(0.0)
    }

    fn max_navigation_overshoot_m(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MAX_NAVIGATION_OVERSHOOT_M")
            .unwrap_or(DEFAULT_MAX_NAVIGATION_OVERSHOOT_M)
            .max(0.0)
    }

    fn max_navigation_horizontal_speed_mps(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MAX_NAVIGATION_HORIZONTAL_SPEED_MPS")
            .unwrap_or(DEFAULT_MAX_NAVIGATION_HORIZONTAL_SPEED_MPS)
            .max(0.0)
    }

    fn navigation_takeoff_down_m(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_NAVIGATION_TAKEOFF_DOWN_M")
            .unwrap_or(DEFAULT_NAVIGATION_TAKEOFF_DOWN_M)
            .min(-self.ground_contact_epsilon_m.abs())
    }

    fn min_navigation_airborne_m(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MIN_NAVIGATION_AIRBORNE_M")
            .unwrap_or(DEFAULT_MIN_NAVIGATION_AIRBORNE_M)
            .max(0.0)
    }

    fn navigation_leg_frames(&self) -> usize {
        env_usize("MARV_GAZEBO_G3_NAVIGATION_LEG_FRAMES")
            .unwrap_or(DEFAULT_NAVIGATION_LEG_FRAMES)
            .max(1)
    }

    fn navigation_setpoint_slew_m_per_frame(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_NAVIGATION_SETPOINT_SLEW_M_PER_FRAME")
            .unwrap_or(DEFAULT_NAVIGATION_SETPOINT_SLEW_M_PER_FRAME)
            .max(0.0)
    }

    fn min_navigation_settle_frames(&self) -> usize {
        env_usize("MARV_GAZEBO_G3_MIN_NAVIGATION_SETTLE_FRAMES")
            .unwrap_or(DEFAULT_MIN_NAVIGATION_SETTLE_FRAMES)
            .max(1)
    }

    fn max_navigation_start_vertical_speed_mps(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MAX_NAVIGATION_START_VERTICAL_SPEED_MPS")
            .unwrap_or(DEFAULT_MAX_NAVIGATION_START_VERTICAL_SPEED_MPS)
            .max(0.0)
    }

    fn max_navigation_start_horizontal_speed_mps(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MAX_NAVIGATION_START_HORIZONTAL_SPEED_MPS")
            .unwrap_or(DEFAULT_MAX_NAVIGATION_START_HORIZONTAL_SPEED_MPS)
            .max(0.0)
    }

    fn max_navigation_start_estimate_truth_down_error_m(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MAX_NAVIGATION_START_ESTIMATE_TRUTH_DOWN_ERROR_M")
            .unwrap_or(DEFAULT_MAX_NAVIGATION_START_ESTIMATE_TRUTH_DOWN_ERROR_M)
            .max(0.0)
    }

    fn max_navigation_start_yaw_error_rad(&self) -> f32 {
        env_f32("MARV_GAZEBO_G3_MAX_NAVIGATION_START_YAW_ERROR_RAD")
            .unwrap_or(DEFAULT_MAX_NAVIGATION_START_YAW_ERROR_RAD)
            .max(0.0)
    }
}

fn apply_g3_navigation_env(settings: &mut RuntimeSettings) {
    settings.auto_reset = env_bool("MARV_GAZEBO_G3_AUTO_RESET").unwrap_or(settings.auto_reset);
    settings.frames = env_usize("MARV_GAZEBO_G3_FRAMES").unwrap_or(settings.frames);
    settings.reset_settle_frames =
        env_usize("MARV_GAZEBO_G3_RESET_SETTLE_FRAMES").unwrap_or(settings.reset_settle_frames);
    settings.timeout = Duration::from_millis(
        env_u64("MARV_GAZEBO_G3_TIMEOUT_MS").unwrap_or_else(|| settings.timeout.as_millis() as u64),
    );
    settings.max_attitude_rad =
        env_f32("MARV_GAZEBO_G3_MAX_ATTITUDE_RAD").unwrap_or(settings.max_attitude_rad);
    settings.max_clamp_ratio =
        env_f32("MARV_GAZEBO_G3_MAX_CLAMP_RATIO").unwrap_or(settings.max_clamp_ratio);
    settings.max_disturbance_horizontal_error_m =
        env_f32("MARV_GAZEBO_G3_MAX_DISTURBANCE_HORIZONTAL_ERROR_M")
            .unwrap_or(settings.max_disturbance_horizontal_error_m);
    settings.max_limited_horizontal_error_m =
        env_f32("MARV_GAZEBO_G3_MAX_LIMITED_HORIZONTAL_ERROR_M")
            .unwrap_or(settings.max_limited_horizontal_error_m);
    settings.position_gain =
        env_f32("MARV_GAZEBO_G3_POSITION_GAIN").unwrap_or(settings.position_gain);
    settings.position_velocity_gain =
        env_f32("MARV_GAZEBO_G3_POSITION_VELOCITY_GAIN").unwrap_or(settings.position_velocity_gain);
    settings.position_max_horizontal_velocity_mps =
        env_f32("MARV_GAZEBO_G3_POSITION_MAX_HORIZONTAL_VELOCITY_MPS")
            .unwrap_or(settings.position_max_horizontal_velocity_mps);
    settings.position_horizontal_integral_gain =
        env_f32("MARV_GAZEBO_G3_POSITION_HORIZONTAL_INTEGRAL_GAIN")
            .unwrap_or(settings.position_horizontal_integral_gain);
    settings.position_horizontal_integral_leak =
        env_f32("MARV_GAZEBO_G3_POSITION_HORIZONTAL_INTEGRAL_LEAK")
            .unwrap_or(settings.position_horizontal_integral_leak);
    settings.position_max_horizontal_integral_accel_mps2 =
        env_f32("MARV_GAZEBO_G3_POSITION_MAX_HORIZONTAL_INTEGRAL_ACCEL_MPS2")
            .unwrap_or(settings.position_max_horizontal_integral_accel_mps2);
    settings.max_horizontal_accel_mps2 = env_f32("MARV_GAZEBO_G3_MAX_HORIZONTAL_ACCEL_MPS2")
        .unwrap_or(settings.max_horizontal_accel_mps2);
    settings.max_tilt_rad = env_f32("MARV_GAZEBO_G3_MAX_TILT_RAD").unwrap_or(settings.max_tilt_rad);
    settings.max_yaw_rate_rps =
        env_f32("MARV_GAZEBO_G3_MAX_YAW_RATE_RPS").unwrap_or(DEFAULT_G3_MAX_YAW_RATE_RPS);
    settings.altitude_gain =
        env_f32("MARV_GAZEBO_G3_ALTITUDE_GAIN").unwrap_or(DEFAULT_G3_ALTITUDE_GAIN);
    settings.vertical_velocity_gain = env_f32("MARV_GAZEBO_G3_VERTICAL_VELOCITY_GAIN")
        .unwrap_or(DEFAULT_G3_VERTICAL_VELOCITY_GAIN);
    settings.max_throttle_correction = env_f32("MARV_GAZEBO_G3_MAX_THROTTLE_CORRECTION")
        .unwrap_or(DEFAULT_G3_MAX_THROTTLE_CORRECTION);
    settings.use_magnetometer =
        env_bool("MARV_GAZEBO_G3_USE_MAGNETOMETER").unwrap_or(settings.use_magnetometer);
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
    run_control_schedule(name, |_| setpoint, settings, airframe)
}

fn run_control_schedule(
    name: &str,
    mut setpoint_for_control_frame: impl FnMut(usize) -> ControlSetpoint,
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
    let mut max_truth_origin_horizontal_m = 0.0_f32;
    let mut final_truth_origin_horizontal_m = 0.0_f32;
    let mut max_estimate_origin_horizontal_m = 0.0_f32;
    let mut final_estimate_origin_horizontal_m = 0.0_f32;
    let mut max_truth_vertical_speed_mps = 0.0_f32;
    let mut max_estimate_vertical_speed_mps = 0.0_f32;
    let mut clamp_count = 0usize;
    let mut initial_truth_down_error_m = None;
    let mut final_truth_down_error_m = 0.0_f32;
    let mut initial_estimate_down_error_m = None;
    let mut final_estimate_down_error_m = 0.0_f32;
    let mut initial_yaw_error_rad = None;
    let mut final_yaw_error_rad = 0.0_f32;
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
    let mut max_setpoint_horizontal_m = 0.0_f32;
    let mut max_abs_setpoint_yaw_rad = 0.0_f32;
    let mut max_attitude_setpoint_tilt_rad = 0.0_f32;
    let mut final_attitude_setpoint_roll_pitch_rad = [0.0_f32; 2];
    let mut final_truth_roll_pitch_rad = [0.0_f32; 2];
    let mut max_yaw_rate_setpoint_rps = 0.0_f32;
    let mut max_throttle_correction = 0.0_f32;
    let mut max_axis_command = 0.0_f32;
    let hover_throttle = airframe.hover_motor_command();

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
        let setpoint = setpoint_for_control_frame(control_frames);
        max_setpoint_horizontal_m = max_setpoint_horizontal_m.max(horizontal_norm_m(
            setpoint.position_ned_m()[0],
            setpoint.position_ned_m()[1],
        ));
        max_abs_setpoint_yaw_rad = max_abs_setpoint_yaw_rad.max(setpoint.yaw_rad().abs());
        let output = pipeline.step(estimate, imu_from_sensor(&sensor), setpoint);
        assert!(
            output.control_valid,
            "invalid G2 control output at sim_time_us={}",
            frame.sim_time_us
        );

        let runtime_motors = airframe.runtime_control_motors(output.torque_command);
        if let Some(attitude_setpoint) = output.attitude_setpoint {
            let (roll_rad, pitch_rad) = roll_pitch_from_quaternion(attitude_setpoint.quaternion);
            final_attitude_setpoint_roll_pitch_rad = [roll_rad, pitch_rad];
            max_attitude_setpoint_tilt_rad = max_attitude_setpoint_tilt_rad.max(
                roll_pitch_tilt_from_quaternion(attitude_setpoint.quaternion),
            );
        }
        if let Some(rate_setpoint) = output.rate_setpoint {
            max_yaw_rate_setpoint_rps = max_yaw_rate_setpoint_rps.max(rate_setpoint.yaw_rps.abs());
        }
        max_throttle_correction =
            max_throttle_correction.max((output.throttle - hover_throttle).abs());
        max_axis_command = max_axis_command
            .max(output.torque_command.roll.abs())
            .max(output.torque_command.pitch.abs())
            .max(output.torque_command.yaw.abs());
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
            final_truth_roll_pitch_rad = [euler[0], euler[1]];
            final_yaw_error_rad = yaw_error_rad(setpoint.yaw_rad(), euler[2]);
            initial_yaw_error_rad.get_or_insert(final_yaw_error_rad);
        }
        let truth = frame
            .to_truth_estimate(origin)
            .expect("G2 truth side-channel should convert for evidence");
        final_truth_position_ned_m = truth.position_ned_m;
        final_truth_velocity_ned_mps = truth.velocity_ned_mps;
        final_estimate_position_ned_m = estimate.position_ned_m;
        final_estimate_velocity_ned_mps = estimate.velocity_ned_mps;
        final_truth_origin_horizontal_m =
            horizontal_norm_m(truth.position_ned_m[0], truth.position_ned_m[1]);
        final_estimate_origin_horizontal_m =
            horizontal_norm_m(estimate.position_ned_m[0], estimate.position_ned_m[1]);
        max_truth_origin_horizontal_m =
            max_truth_origin_horizontal_m.max(final_truth_origin_horizontal_m);
        max_estimate_origin_horizontal_m =
            max_estimate_origin_horizontal_m.max(final_estimate_origin_horizontal_m);
        final_truth_down_error_m = truth.position_ned_m[2] - setpoint.position_ned_m()[2];
        final_estimate_down_error_m = estimate.position_ned_m[2] - setpoint.position_ned_m()[2];
        initial_truth_down_error_m.get_or_insert(final_truth_down_error_m);
        initial_estimate_down_error_m.get_or_insert(final_estimate_down_error_m);
        max_truth_down_error_m = max_truth_down_error_m.max(truth.position_ned_m[2].abs());
        max_estimate_down_error_m = max_estimate_down_error_m.max(estimate.position_ned_m[2].abs());
        max_truth_vertical_speed_mps =
            max_truth_vertical_speed_mps.max(truth.velocity_ned_mps[2].abs());
        max_estimate_vertical_speed_mps =
            max_estimate_vertical_speed_mps.max(estimate.velocity_ned_mps[2].abs());
        max_truth_horizontal_error_m = max_truth_horizontal_error_m.max(horizontal_error_m(
            truth.position_ned_m,
            setpoint.position_ned_m(),
        ));
        max_estimate_horizontal_error_m = max_estimate_horizontal_error_m.max(horizontal_error_m(
            estimate.position_ned_m,
            setpoint.position_ned_m(),
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
        max_truth_origin_horizontal_m,
        final_truth_origin_horizontal_m,
        max_estimate_origin_horizontal_m,
        final_estimate_origin_horizontal_m,
        max_truth_vertical_speed_mps,
        max_estimate_vertical_speed_mps,
        clamp_ratio: clamp_count as f32 / control_frames.max(1) as f32,
        initial_truth_down_error_m: initial_truth_down_error_m.unwrap_or(0.0),
        final_truth_down_error_m,
        initial_estimate_down_error_m: initial_estimate_down_error_m.unwrap_or(0.0),
        final_estimate_down_error_m,
        initial_yaw_error_rad: initial_yaw_error_rad.unwrap_or(0.0),
        final_yaw_error_rad,
        final_truth_position_ned_m,
        final_truth_velocity_ned_mps,
        final_estimate_position_ned_m,
        final_estimate_velocity_ned_mps,
        mean_throttle: throttle_sum / control_frames.max(1) as f32,
        mean_motor: motor_sum / motor_sample_count.max(1) as f32,
        max_motor,
        mean_motor_spread: motor_spread_sum / control_frames.max(1) as f32,
        max_motor_spread,
        max_setpoint_horizontal_m,
        max_abs_setpoint_yaw_rad,
        max_attitude_setpoint_tilt_rad,
        final_attitude_setpoint_roll_pitch_rad,
        final_truth_roll_pitch_rad,
        max_yaw_rate_setpoint_rps,
        max_throttle_correction,
        max_axis_command,
    };
    eprintln!(
        "G2 {name}: {summary:?} final_truth_pos={:?} final_truth_vel={:?} final_estimate_pos={:?} final_estimate_vel={:?} final_att_sp={:?} final_truth_rp={:?} mean_throttle={:.4} mean_motor={:.4} max_motor={:.4} mean_motor_spread={:.4} max_motor_spread={:.4}",
        summary.final_truth_position_ned_m,
        summary.final_truth_velocity_ned_mps,
        summary.final_estimate_position_ned_m,
        summary.final_estimate_velocity_ned_mps,
        summary.final_attitude_setpoint_roll_pitch_rad,
        summary.final_truth_roll_pitch_rad,
        summary.mean_throttle,
        summary.mean_motor,
        summary.max_motor,
        summary.mean_motor_spread,
        summary.max_motor_spread
    );
    summary
}

fn run_airborne_navigation_scenario(
    name: &str,
    target_horizontal_ned_m: [f32; 2],
    yaw_rad: f32,
    settings: &RuntimeSettings,
    airframe: &GazeboAirframeConfig,
) -> NavigationMissionSummary {
    run_airborne_navigation_schedule(
        name,
        move |_, _| (target_horizontal_ned_m, yaw_rad),
        settings,
        airframe,
    )
}

fn run_airborne_navigation_schedule(
    name: &str,
    mut nav_setpoint_for_frame: impl FnMut(usize, usize) -> ([f32; 2], f32),
    settings: &RuntimeSettings,
    airframe: &GazeboAirframeConfig,
) -> NavigationMissionSummary {
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
        .expect("G3 origin frame should include fresh IMU");
    let origin_sensor = estimator_sensor_for_settings(origin_sensor, settings);
    let mut driver = EstimatorReplayDriver::new(gazebo_estimator_config());
    driver
        .step(1, &origin_sensor)
        .expect("G3 estimator prewarm should pass");
    let mut last_timestamp_us = Some(origin_sensor.timestamp_us);

    let takeoff_down_m = settings.navigation_takeoff_down_m();
    let nav_frames_required = settings.navigation_leg_frames();
    let min_settle_frames = settings.min_navigation_settle_frames();
    let max_setpoint_step_m = settings.navigation_setpoint_slew_m_per_frame();
    let mut converted_frames = 1usize;
    let mut control_frames = 0usize;
    let mut nav_start_frame = None;
    let mut nav_frames = 0usize;
    let mut commanded_horizontal_ned_m = [0.0_f32; 2];
    let mut takeoff_nav_gate = TakeoffNavGate::new(TakeoffNavGateConfig {
        takeoff_down_m,
        max_takeoff_down_error_m: settings.max_final_down_error_m,
        max_vertical_speed_mps: settings.max_navigation_start_vertical_speed_mps(),
        max_horizontal_speed_mps: settings.max_navigation_start_horizontal_speed_mps(),
        max_estimate_truth_down_error_m: settings
            .max_navigation_start_estimate_truth_down_error_m(),
        max_yaw_error_rad: settings.max_navigation_start_yaw_error_rad(),
        max_clamp_ratio: settings.max_clamp_ratio,
        min_stable_frames: min_settle_frames,
    });
    let mut clamp_count = 0usize;
    let mut nav_origin_truth_position_ned_m = None;
    let mut nav_origin_estimate_position_ned_m = None;
    let mut final_gate_truth_position_ned_m = [0.0_f32; 3];
    let mut final_gate_estimate_position_ned_m = [0.0_f32; 3];
    let mut final_gate_truth_velocity_ned_mps = [0.0_f32; 3];
    let mut final_gate_estimate_velocity_ned_mps = [0.0_f32; 3];
    let mut final_gate_truth_horizontal_speed_mps = 0.0_f32;
    let mut final_gate_estimate_horizontal_speed_mps = 0.0_f32;
    let mut max_attitude_rad = 0.0_f32;
    let mut max_truth_vertical_speed_mps = 0.0_f32;
    let mut max_estimate_vertical_speed_mps = 0.0_f32;
    let mut max_truth_horizontal_speed_mps = 0.0_f32;
    let mut max_estimate_horizontal_speed_mps = 0.0_f32;
    let mut max_truth_cross_track_m = 0.0_f32;
    let mut max_estimate_cross_track_m = 0.0_f32;
    let mut max_truth_yaw_error_rad = 0.0_f32;
    let mut final_truth_yaw_error_rad = 0.0_f32;
    let mut max_estimate_yaw_error_rad = 0.0_f32;
    let mut final_estimate_yaw_error_rad = 0.0_f32;
    let mut max_estimate_truth_horizontal_error_m = 0.0_f32;
    let mut final_estimate_truth_horizontal_error_m = 0.0_f32;
    let mut final_truth_delta_ned_m = [0.0_f32; 3];
    let mut final_estimate_delta_ned_m = [0.0_f32; 3];
    let mut final_truth_track_progress_m = 0.0_f32;
    let mut final_estimate_track_progress_m = 0.0_f32;
    let mut final_truth_cross_track_m = 0.0_f32;
    let mut final_estimate_cross_track_m = 0.0_f32;
    let mut final_truth_target_error_m = 0.0_f32;
    let mut final_estimate_target_error_m = 0.0_f32;
    let mut final_truth_overshoot_m = 0.0_f32;
    let mut final_estimate_overshoot_m = 0.0_f32;
    let mut final_truth_down_error_m = 0.0_f32;
    let mut final_estimate_down_error_m = 0.0_f32;
    let mut max_setpoint_horizontal_m = 0.0_f32;
    let mut max_abs_setpoint_yaw_rad = 0.0_f32;
    let mut max_attitude_setpoint_tilt_rad = 0.0_f32;
    let mut max_yaw_rate_setpoint_rps = 0.0_f32;
    let mut max_throttle_correction = 0.0_f32;
    let mut max_axis_command = 0.0_f32;
    let mut max_motor = 0.0_f32;
    let hover_throttle = airframe.hover_motor_command();

    while control_frames < settings.frames && nav_frames < nav_frames_required {
        let (frame, sensor) =
            client.read_required_estimator_frame(origin, last_timestamp_us, settings.timeout);
        let sensor = estimator_sensor_for_settings(sensor, settings);
        last_timestamp_us = Some(sensor.timestamp_us);
        converted_frames += 1;
        let trace = driver
            .step(converted_frames as u64, &sensor)
            .expect("G3 estimator step should pass");
        if !trace.estimate_valid {
            continue;
        }

        let estimate = estimate_snapshot(&trace);
        let truth = frame
            .to_truth_estimate(origin)
            .expect("G3 truth side-channel should convert for evidence");
        final_gate_truth_position_ned_m = truth.position_ned_m;
        final_gate_estimate_position_ned_m = estimate.position_ned_m;
        final_gate_truth_velocity_ned_mps = truth.velocity_ned_mps;
        final_gate_estimate_velocity_ned_mps = estimate.velocity_ned_mps;
        final_gate_truth_horizontal_speed_mps =
            horizontal_norm_m(truth.velocity_ned_mps[0], truth.velocity_ned_mps[1]);
        final_gate_estimate_horizontal_speed_mps =
            horizontal_norm_m(estimate.velocity_ned_mps[0], estimate.velocity_ned_mps[1]);
        let nav_frame = nav_frames;
        let (requested_horizontal_ned_m, yaw_rad) =
            nav_setpoint_for_frame(nav_frame, nav_frames_required);
        let gate_yaw_error_rad = frame
            .euler_rad
            .map(|euler| yaw_error_rad(yaw_rad, euler[2]).abs());
        let clamp_ratio = if control_frames == 0 {
            0.0
        } else {
            clamp_count as f32 / control_frames as f32
        };
        let gate_decision = takeoff_nav_gate.update(
            TakeoffNavGateSample {
                estimate_position_ned_m: estimate.position_ned_m,
                estimate_velocity_ned_mps: estimate.velocity_ned_mps,
                truth_position_ned_m: Some(truth.position_ned_m),
                truth_velocity_ned_mps: Some(truth.velocity_ned_mps),
                yaw_error_rad: gate_yaw_error_rad,
                clamp_ratio,
                estimate_valid: estimate.valid,
            },
            yaw_rad,
        );
        if nav_start_frame.is_none() {
            let airborne = truth.position_ned_m[2] <= -settings.min_navigation_airborne_m()
                && estimate.position_ned_m[2] <= -settings.min_navigation_airborne_m();
            if gate_decision.nav_ready && airborne {
                nav_start_frame = Some(control_frames);
                nav_origin_truth_position_ned_m = Some(truth.position_ned_m);
                nav_origin_estimate_position_ned_m = Some(estimate.position_ned_m);
            }
        }

        let in_nav_leg = nav_start_frame.is_some();
        if in_nav_leg {
            commanded_horizontal_ned_m = slew_horizontal_target(
                commanded_horizontal_ned_m,
                requested_horizontal_ned_m,
                max_setpoint_step_m,
            );
        } else {
            commanded_horizontal_ned_m = [0.0, 0.0];
        }

        let nav_truth_origin = nav_origin_truth_position_ned_m.unwrap_or([0.0; 3]);
        let nav_estimate_origin = nav_origin_estimate_position_ned_m.unwrap_or([0.0; 3]);
        let absolute_setpoint = if in_nav_leg {
            [
                nav_estimate_origin[0] + commanded_horizontal_ned_m[0],
                nav_estimate_origin[1] + commanded_horizontal_ned_m[1],
                nav_estimate_origin[2],
            ]
        } else {
            gate_decision.hold_setpoint.position_ned_m
        };
        let setpoint = ControlSetpoint::local_position_ned(absolute_setpoint, yaw_rad, true);
        max_setpoint_horizontal_m = max_setpoint_horizontal_m.max(horizontal_norm_m(
            commanded_horizontal_ned_m[0],
            commanded_horizontal_ned_m[1],
        ));
        max_abs_setpoint_yaw_rad = max_abs_setpoint_yaw_rad.max(yaw_rad.abs());

        let output = pipeline.step(estimate, imu_from_sensor(&sensor), setpoint);
        assert!(
            output.control_valid,
            "invalid G3 control output at sim_time_us={}",
            frame.sim_time_us
        );
        let runtime_motors = airframe.runtime_control_motors(output.torque_command);
        client.send_actuator(&format_gazebo_actuator_line(
            sequence.next(),
            Some(frame.sim_time_us),
            runtime_motors.commands,
        ));
        control_frames += 1;

        if let Some(euler) = frame.euler_rad {
            max_attitude_rad = max_attitude_rad.max(euler[0].abs()).max(euler[1].abs());
        }
        if let Some(attitude_setpoint) = output.attitude_setpoint {
            max_attitude_setpoint_tilt_rad = max_attitude_setpoint_tilt_rad.max(
                roll_pitch_tilt_from_quaternion(attitude_setpoint.quaternion),
            );
        }
        if let Some(rate_setpoint) = output.rate_setpoint {
            max_yaw_rate_setpoint_rps = max_yaw_rate_setpoint_rps.max(rate_setpoint.yaw_rps.abs());
        }
        max_throttle_correction =
            max_throttle_correction.max((output.throttle - hover_throttle).abs());
        max_axis_command = max_axis_command
            .max(output.torque_command.roll.abs())
            .max(output.torque_command.pitch.abs())
            .max(output.torque_command.yaw.abs());
        max_motor = max_motor.max(max_motor_command(runtime_motors.commands));
        if runtime_motors.clamped {
            clamp_count += 1;
        }

        max_truth_vertical_speed_mps =
            max_truth_vertical_speed_mps.max(truth.velocity_ned_mps[2].abs());
        max_estimate_vertical_speed_mps =
            max_estimate_vertical_speed_mps.max(estimate.velocity_ned_mps[2].abs());

        if in_nav_leg {
            nav_frames += 1;
            final_truth_delta_ned_m = ned_delta(truth.position_ned_m, nav_truth_origin);
            final_estimate_delta_ned_m = ned_delta(estimate.position_ned_m, nav_estimate_origin);
            if let Some(euler) = frame.euler_rad {
                final_truth_yaw_error_rad = yaw_error_rad(yaw_rad, euler[2]).abs();
                max_truth_yaw_error_rad = max_truth_yaw_error_rad.max(final_truth_yaw_error_rad);
            }
            final_estimate_yaw_error_rad =
                yaw_error_rad(yaw_rad, yaw_from_quaternion(estimate.quaternion)).abs();
            max_estimate_yaw_error_rad =
                max_estimate_yaw_error_rad.max(final_estimate_yaw_error_rad);
            final_estimate_truth_horizontal_error_m = horizontal_norm_m(
                truth.position_ned_m[0] - estimate.position_ned_m[0],
                truth.position_ned_m[1] - estimate.position_ned_m[1],
            );
            max_estimate_truth_horizontal_error_m =
                max_estimate_truth_horizontal_error_m.max(final_estimate_truth_horizontal_error_m);
            let truth_track = track_error(final_truth_delta_ned_m, requested_horizontal_ned_m);
            let estimate_track =
                track_error(final_estimate_delta_ned_m, requested_horizontal_ned_m);
            final_truth_track_progress_m = truth_track.progress_m;
            final_estimate_track_progress_m = estimate_track.progress_m;
            final_truth_cross_track_m = truth_track.cross_track_m;
            final_estimate_cross_track_m = estimate_track.cross_track_m;
            final_truth_target_error_m = truth_track.target_error_m;
            final_estimate_target_error_m = estimate_track.target_error_m;
            final_truth_overshoot_m = truth_track.overshoot_m;
            final_estimate_overshoot_m = estimate_track.overshoot_m;
            final_truth_down_error_m = truth.position_ned_m[2] - nav_truth_origin[2];
            final_estimate_down_error_m = estimate.position_ned_m[2] - nav_estimate_origin[2];
            max_truth_cross_track_m = max_truth_cross_track_m.max(truth_track.cross_track_m);
            max_estimate_cross_track_m =
                max_estimate_cross_track_m.max(estimate_track.cross_track_m);
            max_truth_horizontal_speed_mps = max_truth_horizontal_speed_mps.max(horizontal_norm_m(
                truth.velocity_ned_mps[0],
                truth.velocity_ned_mps[1],
            ));
            max_estimate_horizontal_speed_mps = max_estimate_horizontal_speed_mps.max(
                horizontal_norm_m(estimate.velocity_ned_mps[0], estimate.velocity_ned_mps[1]),
            );
        }
    }

    client.send_actuator(&format_gazebo_actuator_line(
        sequence.next(),
        None,
        [0.0; 4],
    ));

    let summary = NavigationMissionSummary {
        converted_frames,
        control_frames,
        nav_start_frame: nav_start_frame.unwrap_or(0),
        nav_frames,
        nav_origin_truth_position_ned_m: nav_origin_truth_position_ned_m.unwrap_or([0.0; 3]),
        nav_origin_estimate_position_ned_m: nav_origin_estimate_position_ned_m.unwrap_or([0.0; 3]),
        final_gate_truth_position_ned_m,
        final_gate_estimate_position_ned_m,
        final_gate_truth_velocity_ned_mps,
        final_gate_estimate_velocity_ned_mps,
        final_gate_truth_horizontal_speed_mps,
        final_gate_estimate_horizontal_speed_mps,
        final_truth_delta_ned_m,
        final_estimate_delta_ned_m,
        max_attitude_rad,
        max_truth_vertical_speed_mps,
        max_estimate_vertical_speed_mps,
        max_truth_horizontal_speed_mps,
        max_estimate_horizontal_speed_mps,
        max_truth_cross_track_m,
        max_estimate_cross_track_m,
        max_truth_yaw_error_rad,
        final_truth_yaw_error_rad,
        max_estimate_yaw_error_rad,
        final_estimate_yaw_error_rad,
        max_estimate_truth_horizontal_error_m,
        final_estimate_truth_horizontal_error_m,
        final_truth_track_progress_m,
        final_estimate_track_progress_m,
        final_truth_cross_track_m,
        final_estimate_cross_track_m,
        final_truth_target_error_m,
        final_estimate_target_error_m,
        final_truth_overshoot_m,
        final_estimate_overshoot_m,
        final_truth_down_error_m,
        final_estimate_down_error_m,
        clamp_ratio: clamp_count as f32 / control_frames.max(1) as f32,
        max_setpoint_horizontal_m,
        max_abs_setpoint_yaw_rad,
        max_attitude_setpoint_tilt_rad,
        max_yaw_rate_setpoint_rps,
        max_throttle_correction,
        max_axis_command,
        max_motor,
    };
    eprintln!(
        "G3 {name}: {summary:?} nav_origin_truth={:?} final_truth_delta={:?} final_estimate_delta={:?}",
        summary.nav_origin_truth_position_ned_m,
        summary.final_truth_delta_ned_m,
        summary.final_estimate_delta_ned_m
    );
    summary
}

fn run_takeoff_hover_land_mission(
    name: &str,
    settings: &RuntimeSettings,
    airframe: &GazeboAirframeConfig,
) -> TakeoffHoverLandSummary {
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

    let takeoff_one_end = settings.frames / 5;
    let takeoff_two_end = (settings.frames * 2) / 5;
    let hover_end = (settings.frames * 3) / 5;
    let contact_threshold_down_m = -settings.ground_contact_epsilon_m.abs();
    let mut contact_latched = false;
    let mut first_contact_frame = None;
    let mut converted_frames = 1usize;
    let mut control_frames = 0usize;
    let mut post_contact_frames = 0usize;
    let mut max_attitude_rad = 0.0_f32;
    let mut min_truth_down_m = f32::INFINITY;
    let mut max_hover_truth_down_error_m = 0.0_f32;
    let mut max_truth_origin_horizontal_m = 0.0_f32;
    let mut max_estimate_origin_horizontal_m = 0.0_f32;
    let mut max_truth_vertical_speed_mps = 0.0_f32;
    let mut max_estimate_vertical_speed_mps = 0.0_f32;
    let mut max_landing_impact_speed_mps = 0.0_f32;
    let mut clamp_count = 0usize;
    let mut pre_contact_zero_motor_frames = 0usize;
    let mut post_contact_max_motor = 0.0_f32;
    let mut final_truth_down_m = 0.0_f32;
    let mut final_estimate_down_m = 0.0_f32;
    let mut final_truth_vertical_speed_mps = 0.0_f32;
    let mut final_armed = true;
    let mut final_control_valid = true;

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
        let truth = frame
            .to_truth_estimate(origin)
            .expect("G2 truth side-channel should convert for evidence");
        let descent_started = control_frames >= hover_end;
        let contact_now = contact_latched
            || (descent_started && truth.position_ned_m[2] >= contact_threshold_down_m);
        if contact_now && !contact_latched {
            first_contact_frame = Some(control_frames + 1);
            max_landing_impact_speed_mps =
                max_landing_impact_speed_mps.max(truth.velocity_ned_mps[2].max(0.0));
            contact_latched = true;
        }

        let setpoint = if contact_now {
            ControlSetpoint::local_position_ned([0.0, 0.0, 0.0], 0.0, false)
        } else if control_frames < takeoff_one_end {
            ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true)
        } else if control_frames < takeoff_two_end {
            ControlSetpoint::local_position_ned([0.0, 0.0, -2.0], 0.0, true)
        } else if control_frames < hover_end {
            ControlSetpoint::local_position_ned([0.0, 0.0, -2.0], 0.0, true)
        } else {
            ControlSetpoint::local_position_ned([0.0, 0.0, 0.05], 0.0, true)
        };
        let output = pipeline.step(estimate, imu_from_sensor(&sensor), setpoint);
        if setpoint.armed() {
            assert!(
                output.control_valid,
                "invalid G2 control output at sim_time_us={}",
                frame.sim_time_us
            );
        }

        let runtime_motors = airframe.runtime_control_motors(output.torque_command);
        control_frames += 1;
        client.send_actuator(&format_gazebo_actuator_line(
            sequence.next(),
            Some(frame.sim_time_us),
            runtime_motors.commands,
        ));

        if let Some(euler) = frame.euler_rad {
            max_attitude_rad = max_attitude_rad.max(euler[0].abs()).max(euler[1].abs());
        }
        min_truth_down_m = min_truth_down_m.min(truth.position_ned_m[2]);
        final_truth_down_m = truth.position_ned_m[2];
        final_estimate_down_m = estimate.position_ned_m[2];
        final_truth_vertical_speed_mps = truth.velocity_ned_mps[2];
        max_truth_origin_horizontal_m = max_truth_origin_horizontal_m.max(horizontal_norm_m(
            truth.position_ned_m[0],
            truth.position_ned_m[1],
        ));
        max_estimate_origin_horizontal_m = max_estimate_origin_horizontal_m.max(horizontal_norm_m(
            estimate.position_ned_m[0],
            estimate.position_ned_m[1],
        ));
        max_truth_vertical_speed_mps =
            max_truth_vertical_speed_mps.max(truth.velocity_ned_mps[2].abs());
        max_estimate_vertical_speed_mps =
            max_estimate_vertical_speed_mps.max(estimate.velocity_ned_mps[2].abs());
        if (takeoff_two_end..hover_end).contains(&(control_frames - 1)) {
            max_hover_truth_down_error_m =
                max_hover_truth_down_error_m.max((truth.position_ned_m[2] + 2.0).abs());
        }
        if runtime_motors.clamped {
            clamp_count += 1;
        }
        let max_motor = max_motor_command(runtime_motors.commands);
        if contact_now {
            post_contact_frames += 1;
            post_contact_max_motor = post_contact_max_motor.max(max_motor);
        } else if max_motor <= 0.001 {
            pre_contact_zero_motor_frames += 1;
        }
        final_armed = output.armed;
        final_control_valid = output.control_valid;
    }

    client.send_actuator(&format_gazebo_actuator_line(
        sequence.next(),
        None,
        [0.0; 4],
    ));

    let summary = TakeoffHoverLandSummary {
        converted_frames,
        control_frames,
        first_contact_frame,
        post_contact_frames,
        max_attitude_rad,
        min_truth_down_m,
        max_hover_truth_down_error_m,
        max_truth_origin_horizontal_m,
        max_estimate_origin_horizontal_m,
        max_truth_vertical_speed_mps,
        max_estimate_vertical_speed_mps,
        max_landing_impact_speed_mps,
        clamp_ratio: clamp_count as f32 / control_frames.max(1) as f32,
        pre_contact_zero_motor_frames,
        post_contact_max_motor,
        final_truth_down_m,
        final_estimate_down_m,
        final_truth_vertical_speed_mps,
        final_armed,
        final_control_valid,
    };
    eprintln!("G2 {name}: {summary:?}");
    summary
}

fn assert_g2_runtime_envelope(label: &str, summary: RuntimeSummary, settings: &RuntimeSettings) {
    assert!(
        summary.converted_frames >= summary.control_frames,
        "{label} converted fewer frames than it controlled: summary={summary:?}"
    );
    assert!(
        summary.max_attitude_rad <= settings.max_attitude_rad,
        "{label} attitude diverged: summary={summary:?}"
    );
    assert!(
        summary.max_truth_down_error_m <= settings.max_down_error_m,
        "{label} truth altitude diverged: summary={summary:?}"
    );
    assert!(
        summary.max_estimate_down_error_m <= settings.max_down_error_m,
        "{label} estimate altitude diverged: summary={summary:?}"
    );
    assert!(
        summary.max_truth_horizontal_error_m <= settings.max_horizontal_error_m,
        "{label} truth drifted horizontally: summary={summary:?}"
    );
    assert!(
        summary.max_estimate_horizontal_error_m <= settings.max_horizontal_error_m,
        "{label} estimate drifted horizontally: summary={summary:?}"
    );
    assert!(
        summary.clamp_ratio <= settings.max_clamp_ratio,
        "{label} clamped too often: summary={summary:?}"
    );
}

fn assert_g2_runtime_limit_evidence(
    label: &str,
    summary: RuntimeSummary,
    settings: &RuntimeSettings,
) {
    assert!(
        summary.converted_frames >= summary.control_frames,
        "{label} converted fewer frames than it controlled: summary={summary:?}"
    );
    assert!(
        summary.max_attitude_rad <= settings.max_tilt_rad + 5.0_f32.to_radians(),
        "{label} actual attitude exceeded limited tilt envelope: summary={summary:?}"
    );
    assert!(
        summary.max_truth_down_error_m <= settings.max_down_error_m,
        "{label} truth altitude diverged: summary={summary:?}"
    );
    assert!(
        summary.max_estimate_down_error_m <= settings.max_down_error_m,
        "{label} estimate altitude diverged: summary={summary:?}"
    );
    assert!(
        summary.max_truth_vertical_speed_mps <= settings.max_vertical_speed_mps,
        "{label} truth vertical speed too high: summary={summary:?}"
    );
    assert!(
        summary.max_estimate_vertical_speed_mps <= settings.max_vertical_speed_mps,
        "{label} estimate vertical speed too high: summary={summary:?}"
    );
    assert!(
        summary.max_attitude_setpoint_tilt_rad <= settings.max_tilt_rad + 0.001,
        "{label} tilt demand was not limited before attitude control: summary={summary:?}"
    );
    assert!(
        summary.max_yaw_rate_setpoint_rps <= settings.max_yaw_rate_rps + 0.001,
        "{label} yaw-rate demand was not limited before rate control: summary={summary:?}"
    );
    assert!(
        summary.max_throttle_correction <= settings.max_throttle_correction + 0.001,
        "{label} throttle correction was not limited: summary={summary:?}"
    );
    assert!(
        summary.max_axis_command <= 0.251,
        "{label} rate-axis command was not limited: summary={summary:?}"
    );
    assert!(
        summary.max_motor <= 1.0,
        "{label} motor command escaped normalized range: summary={summary:?}"
    );
}

fn assert_navigation_mission(
    label: &str,
    summary: NavigationMissionSummary,
    target_horizontal_ned_m: [f32; 2],
    settings: &RuntimeSettings,
) {
    let target_distance_m =
        horizontal_norm_m(target_horizontal_ned_m[0], target_horizontal_ned_m[1]);
    assert!(
        summary.converted_frames >= summary.control_frames,
        "{label} converted fewer frames than it controlled: summary={summary:?}"
    );
    assert!(
        summary
            .final_gate_truth_position_ned_m
            .iter()
            .chain(summary.final_gate_estimate_position_ned_m.iter())
            .chain(summary.final_gate_truth_velocity_ned_mps.iter())
            .chain(summary.final_gate_estimate_velocity_ned_mps.iter())
            .all(|value| value.is_finite())
            && summary.final_gate_truth_horizontal_speed_mps.is_finite()
            && summary.final_gate_estimate_horizontal_speed_mps.is_finite()
            && summary.final_truth_yaw_error_rad.is_finite()
            && summary.final_estimate_yaw_error_rad.is_finite()
            && summary.max_estimate_truth_horizontal_error_m.is_finite()
            && summary.final_estimate_truth_horizontal_error_m.is_finite(),
        "{label} final navigation gate evidence was not finite: summary={summary:?}"
    );
    assert!(
        summary.nav_frames > 0 && summary.nav_start_frame < summary.control_frames,
        "{label} did not collect an airborne navigation leg: summary={summary:?}"
    );
    assert!(
        summary
            .nav_origin_estimate_position_ned_m
            .iter()
            .all(|value| value.is_finite()),
        "{label} estimate navigation origin was not finite: summary={summary:?}"
    );
    assert!(
        summary.nav_origin_truth_position_ned_m[2] <= -settings.min_navigation_airborne_m(),
        "{label} did not establish airborne truth origin before navigation: summary={summary:?}"
    );
    assert!(
        summary.nav_origin_estimate_position_ned_m[2] <= -settings.min_navigation_airborne_m(),
        "{label} did not establish airborne estimate origin before navigation: summary={summary:?}"
    );
    assert!(
        summary.max_attitude_rad <= settings.max_tilt_rad + 5.0_f32.to_radians(),
        "{label} actual attitude exceeded limited tilt envelope: summary={summary:?}"
    );
    assert!(
        summary.max_truth_vertical_speed_mps <= settings.max_vertical_speed_mps,
        "{label} truth vertical speed too high: summary={summary:?}"
    );
    assert!(
        summary.max_estimate_vertical_speed_mps <= settings.max_vertical_speed_mps,
        "{label} estimate vertical speed too high: summary={summary:?}"
    );
    assert!(
        summary.max_truth_horizontal_speed_mps <= settings.max_navigation_horizontal_speed_mps(),
        "{label} truth horizontal speed too high: summary={summary:?}"
    );
    assert!(
        summary.max_estimate_horizontal_speed_mps <= settings.max_navigation_horizontal_speed_mps(),
        "{label} estimate horizontal speed too high: summary={summary:?}"
    );
    assert!(
        summary.max_attitude_setpoint_tilt_rad <= settings.max_tilt_rad + 0.001,
        "{label} tilt demand was not limited before attitude control: summary={summary:?}"
    );
    assert!(
        summary.max_yaw_rate_setpoint_rps <= settings.max_yaw_rate_rps + 0.001,
        "{label} yaw-rate demand was not limited before rate control: summary={summary:?}"
    );
    assert!(
        summary.max_truth_yaw_error_rad <= settings.max_navigation_start_yaw_error_rad(),
        "{label} truth yaw drifted outside the navigation yaw envelope: summary={summary:?}"
    );
    assert!(
        summary.max_estimate_yaw_error_rad <= settings.max_navigation_start_yaw_error_rad(),
        "{label} estimator yaw drifted outside the navigation yaw envelope: summary={summary:?}"
    );
    assert!(
        summary.max_throttle_correction <= settings.max_throttle_correction + 0.001,
        "{label} throttle correction was not limited: summary={summary:?}"
    );
    assert!(
        summary.max_axis_command <= 0.251,
        "{label} rate-axis command was not limited: summary={summary:?}"
    );
    assert!(
        summary.max_motor <= 1.0,
        "{label} motor command escaped normalized range: summary={summary:?}"
    );
    assert!(
        summary.clamp_ratio <= settings.max_clamp_ratio,
        "{label} clamped too often: summary={summary:?}"
    );
    if target_distance_m > f32::EPSILON {
        assert!(
            summary.final_truth_track_progress_m >= settings.min_navigation_progress_m(),
            "{label} truth did not progress enough along target track: summary={summary:?}"
        );
        assert!(
            summary.final_estimate_track_progress_m >= settings.min_navigation_progress_m(),
            "{label} estimate did not progress enough along target track: summary={summary:?}"
        );
        assert!(
            summary.final_truth_overshoot_m <= settings.max_navigation_overshoot_m(),
            "{label} truth overshot target too far: summary={summary:?}"
        );
        assert!(
            summary.final_estimate_overshoot_m <= settings.max_navigation_overshoot_m(),
            "{label} estimate overshot target too far: summary={summary:?}"
        );
    }
    assert!(
        summary.max_truth_cross_track_m <= settings.max_navigation_cross_track_m(),
        "{label} truth cross-track drift too high: summary={summary:?}"
    );
    assert!(
        summary.max_estimate_cross_track_m <= settings.max_navigation_cross_track_m(),
        "{label} estimate cross-track drift too high: summary={summary:?}"
    );
    assert!(
        summary.final_truth_cross_track_m <= settings.max_navigation_cross_track_m(),
        "{label} truth final cross-track drift too high: summary={summary:?}"
    );
    assert!(
        summary.final_estimate_cross_track_m <= settings.max_navigation_cross_track_m(),
        "{label} estimate final cross-track drift too high: summary={summary:?}"
    );
    assert!(
        summary.final_truth_target_error_m <= settings.max_navigation_final_error_m(),
        "{label} truth final target error too high: summary={summary:?}"
    );
    assert!(
        summary.final_estimate_target_error_m <= settings.max_navigation_final_error_m(),
        "{label} estimate final target error too high: summary={summary:?}"
    );
    assert!(
        summary.final_truth_down_error_m.abs() <= settings.max_final_down_error_m,
        "{label} truth altitude did not stay near navigation altitude: summary={summary:?}"
    );
    assert!(
        summary.final_estimate_down_error_m.abs() <= settings.max_final_down_error_m,
        "{label} estimate altitude did not stay near navigation altitude: summary={summary:?}"
    );
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
    config.position.max_horizontal_velocity_mps =
        settings.position_max_horizontal_velocity_mps.max(0.0);
    config.position.horizontal_integral_gain = settings.position_horizontal_integral_gain.max(0.0);
    config.position.horizontal_integral_leak =
        settings.position_horizontal_integral_leak.clamp(0.0, 1.0);
    config.position.max_horizontal_integral_accel_mps2 = settings
        .position_max_horizontal_integral_accel_mps2
        .max(0.0);
    config.position.max_horizontal_accel_mps2 = settings.max_horizontal_accel_mps2.max(0.0);
    config.position.max_tilt_rad = settings.max_tilt_rad.max(0.0);
    config.attitude.max_yaw_rate_rps = settings.max_yaw_rate_rps.max(0.0);
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

fn max_motor_command(motors: [f32; 4]) -> f32 {
    motors.into_iter().fold(0.0, f32::max)
}

fn horizontal_error_m(position_ned_m: [f32; 3], setpoint_ned_m: [f32; 3]) -> f32 {
    let north_error_m = position_ned_m[0] - setpoint_ned_m[0];
    let east_error_m = position_ned_m[1] - setpoint_ned_m[1];
    horizontal_norm_m(north_error_m, east_error_m)
}

fn horizontal_norm_m(north_m: f32, east_m: f32) -> f32 {
    (north_m * north_m + east_m * east_m).sqrt()
}

fn slew_horizontal_target(current: [f32; 2], target: [f32; 2], max_step_m: f32) -> [f32; 2] {
    if max_step_m <= 0.0 {
        return current;
    }

    let delta = [target[0] - current[0], target[1] - current[1]];
    let distance_m = horizontal_norm_m(delta[0], delta[1]);
    if distance_m <= max_step_m || distance_m <= f32::EPSILON {
        return target;
    }

    let scale = max_step_m / distance_m;
    [current[0] + delta[0] * scale, current[1] + delta[1] * scale]
}

#[derive(Clone, Copy, Debug)]
struct TrackError {
    progress_m: f32,
    cross_track_m: f32,
    target_error_m: f32,
    overshoot_m: f32,
}

fn ned_delta(position_ned_m: [f32; 3], origin_ned_m: [f32; 3]) -> [f32; 3] {
    [
        position_ned_m[0] - origin_ned_m[0],
        position_ned_m[1] - origin_ned_m[1],
        position_ned_m[2] - origin_ned_m[2],
    ]
}

fn track_error(delta_ned_m: [f32; 3], target_horizontal_ned_m: [f32; 2]) -> TrackError {
    let target_distance_m =
        horizontal_norm_m(target_horizontal_ned_m[0], target_horizontal_ned_m[1]);
    let target_error_m = horizontal_norm_m(
        target_horizontal_ned_m[0] - delta_ned_m[0],
        target_horizontal_ned_m[1] - delta_ned_m[1],
    );
    if target_distance_m <= f32::EPSILON {
        let distance_from_origin_m = horizontal_norm_m(delta_ned_m[0], delta_ned_m[1]);
        return TrackError {
            progress_m: 0.0,
            cross_track_m: distance_from_origin_m,
            target_error_m: distance_from_origin_m,
            overshoot_m: distance_from_origin_m,
        };
    }

    let unit_north = target_horizontal_ned_m[0] / target_distance_m;
    let unit_east = target_horizontal_ned_m[1] / target_distance_m;
    let progress_m = delta_ned_m[0] * unit_north + delta_ned_m[1] * unit_east;
    let cross_track_m = (delta_ned_m[0] * unit_east - delta_ned_m[1] * unit_north).abs();

    TrackError {
        progress_m,
        cross_track_m,
        target_error_m,
        overshoot_m: (progress_m - target_distance_m).max(0.0),
    }
}

fn roll_pitch_tilt_from_quaternion(quaternion: [f32; 4]) -> f32 {
    let (roll, pitch) = roll_pitch_from_quaternion(quaternion);
    roll.abs().max(pitch.abs())
}

fn roll_pitch_from_quaternion(quaternion: [f32; 4]) -> (f32, f32) {
    let [w, x, y, z] = quaternion;
    let roll = f32::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch = f32::asin((2.0 * (w * y - z * x)).clamp(-1.0, 1.0));
    (roll, pitch)
}

fn yaw_from_quaternion(quaternion: [f32; 4]) -> f32 {
    let [w, x, y, z] = quaternion;
    f32::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
}

fn yaw_error_rad(target: f32, actual: f32) -> f32 {
    let mut error = target - actual;
    while error > core::f32::consts::PI {
        error -= 2.0 * core::f32::consts::PI;
    }
    while error < -core::f32::consts::PI {
        error += 2.0 * core::f32::consts::PI;
    }
    error
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
