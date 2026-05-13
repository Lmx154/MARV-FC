mod support;

use common::control::{
    config::ControlLoopConfig,
    pipeline::{
        ControlSetpoint, EstimateSnapshot, FlightControlConfig, FlightControlPipeline,
        ImuControlInput,
    },
};
use deterministic_harness::GazeboAirframeConfig;
use support::{
    AcceptanceThresholds, CommandKind, CommandSchedule, FailureLayer, Gate, ScenarioMetrics,
    ScenarioReport, ScenarioSpec, ScheduledCommand,
};

const AIRFRAME_CONFIG: &str = include_str!("../config/airframes/f450_xing2_2809_1045_4s_v0.cfg");

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g1_estimator_bypass_control_primitives_use_runtime_airframe_geometry() {
    let airframe = airframe_config();
    let pipeline = pipeline_for_airframe(&airframe);

    let hover = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );
    assert!(hover.control_valid);
    assert!(!hover.clamped);
    assert!(motor_spread(hover.motors) < 0.02);
    for motor in hover.motors {
        assert!(
            (motor - airframe.hover_motor_command()).abs() < 0.08,
            "hover motor {motor} should stay close to airframe hover command"
        );
    }

    let roll_recovery = pipeline.step(
        estimate_with_quaternion([0.996_194_7, 0.087_155_7, 0.0, 0.0]),
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );
    assert!(roll_recovery.torque_command.roll < 0.0);
    assert!(motor_spread(roll_recovery.motors) > 0.001);

    let pitch_recovery = pipeline.step(
        estimate_with_quaternion([0.996_194_7, 0.0, 0.087_155_7, 0.0]),
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );
    assert!(pitch_recovery.torque_command.pitch < 0.0);
    assert!(motor_spread(pitch_recovery.motors) > 0.001);

    let yaw_rate_recovery = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput {
            accel_mps2: [0.0; 3],
            gyro_rps: [0.0, 0.0, 0.75],
        },
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );
    assert!(yaw_rate_recovery.torque_command.yaw < 0.0);

    let step_up = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([0.0, 0.0, -0.75], 0.0, true),
    );
    let step_down = pipeline.step(
        EstimateSnapshot {
            position_ned_m: [0.0, 0.0, -0.75],
            ..EstimateSnapshot::LEVEL_ORIGIN
        },
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );
    assert!(step_up.throttle > hover.throttle);
    assert!(step_down.throttle < hover.throttle);
    assert_eq!(
        step_up.debug.estimator_frame,
        common::control::pipeline::ControlFrame::EstimatorLocalNed
    );
    assert_eq!(
        step_up.debug.setpoint_frame,
        common::control::pipeline::ControlFrame::EstimatorLocalNed
    );
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g1_report_shape_keeps_bypass_control_failures_separate_from_guidance() {
    let spec = ScenarioSpec {
        id: "g1_vertical_step_up",
        gate: Gate::G1,
        description: "estimator-bypass vertical step must improve altitude while holding XY",
        reset_required: true,
        command_schedule: CommandSchedule::new(vec![
            ScheduledCommand {
                at_frame: 0,
                kind: CommandKind::SimReset,
            },
            ScheduledCommand {
                at_frame: 20,
                kind: CommandKind::LocalPosition,
            },
        ]),
        thresholds: AcceptanceThresholds {
            max_cross_track_error_m: 0.35,
            max_altitude_error_m: 0.55,
            max_roll_pitch_rad: 0.25,
            max_speed_error_mps: 0.45,
            max_clamp_ratio: 0.05,
        },
    };

    assert_eq!(spec.gate, Gate::G1);
    assert!(spec.reset_required);
    assert_eq!(spec.command_schedule.commands.len(), 2);
    assert!(spec.description.contains("estimator-bypass"));

    let control_fail = ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean: true,
            estimator_agrees: true,
            max_cross_track_error_m: 0.10,
            max_altitude_error_m: 0.90,
            max_roll_pitch_rad: 0.10,
            max_speed_error_mps: 0.20,
            clamp_ratio: 0.0,
            first_clamp_source: None,
        },
    );
    assert!(!control_fail.pass);
    assert_eq!(control_fail.failure_layer, Some(FailureLayer::Control));
    assert!(
        control_fail
            .to_json_line()
            .contains("\"failure_layer\":\"control\"")
    );
}

fn airframe_config() -> GazeboAirframeConfig {
    let airframe = GazeboAirframeConfig::parse(AIRFRAME_CONFIG).expect("airframe config parses");
    airframe
        .validate()
        .expect("airframe config is physically self-consistent");
    airframe
}

fn pipeline_for_airframe(airframe: &GazeboAirframeConfig) -> FlightControlPipeline {
    let mut loop_config = ControlLoopConfig::default();
    loop_config.altitude.hover_throttle = airframe.hover_motor_command();
    FlightControlPipeline::new(FlightControlConfig::new(
        loop_config,
        airframe.runtime_motor_geometry(),
    ))
}

fn estimate_with_quaternion(quaternion: [f32; 4]) -> EstimateSnapshot {
    EstimateSnapshot {
        quaternion,
        ..EstimateSnapshot::LEVEL_ORIGIN
    }
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
