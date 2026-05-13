mod support;

use common::{
    control::pipeline::{
        ControlSetpoint, EstimateSnapshot, FlightControlPipeline, ImuControlInput,
    },
    messages::{control::ActuatorOutputSample, estimate::StateEstimateSample},
    protocol::hilink::{SimStamp, response_flags},
};
use deterministic_harness::{
    HilSemanticAdapter, HilSemanticFrameConfig, SensorFrame, hil_response_from_samples,
    response_has_flag, sensor_frame_to_hil_frame,
};
use support::{
    AcceptanceThresholds, CommandKind, CommandSchedule, FailureLayer, Gate, ScenarioMetrics,
    ScenarioReport, ScenarioSpec, ScheduledCommand,
};

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g2_estimator_loop_handoff_accepts_takeoff_hover_land_sensor_sequence() {
    let config = HilSemanticFrameConfig::default();
    let mut adapter = HilSemanticAdapter::active();
    let sequence = [
        (1, sensor_frame(20_000, 0.0, 0.0)),
        (2, sensor_frame(40_000, -0.35, -0.20)),
        (3, sensor_frame(60_000, -1.00, -0.05)),
        (4, sensor_frame(80_000, -1.02, 0.00)),
        (5, sensor_frame(100_000, -0.12, 0.18)),
    ];

    for (tick, sensor) in sequence {
        let dispatch = adapter.accept_sensor_frame(tick, &sensor, config);
        assert!(dispatch.accepted, "G2 sensor tick {tick} must enter HIL");
    }

    let published = adapter.published_groups();
    assert_eq!(published.time, 5);
    assert_eq!(published.imu, 5);
    assert_eq!(published.barometer, 5);
    assert_eq!(published.gps, 5);
    assert_eq!(published.magnetometer, 5);
    assert_eq!(adapter.archive().raw_frames.len(), 5);

    let estimate = StateEstimateSample {
        position_ned_m: [0.03, -0.02, -1.01],
        velocity_ned_mps: [0.02, 0.01, 0.0],
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        valid: true,
    };
    let response = hil_response_from_samples(
        SimStamp {
            sim_tick: 4,
            sim_time_us: 80_000,
        },
        true,
        true,
        Some(estimate),
        Some(ActuatorOutputSample::new([0.55; 4], true, false)),
    );

    assert!(response_has_flag(
        response,
        response_flags::SENSOR_INPUT_VALID
    ));
    assert!(response_has_flag(response, response_flags::ESTIMATOR_VALID));
    assert!(response_has_flag(response, response_flags::CONTROL_VALID));
    assert!(response_has_flag(response, response_flags::MOTORS_VALID));
    assert!(!response_has_flag(response, response_flags::FAILSAFE));
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g2_landing_disarms_to_zero_motors_without_relaunch() {
    let pipeline = FlightControlPipeline::default();
    let armed_hover = pipeline.step(
        EstimateSnapshot {
            position_ned_m: [0.0, 0.0, -0.35],
            valid: true,
            ..EstimateSnapshot::LEVEL_ORIGIN
        },
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([0.0, 0.0, -0.35], 0.0, true),
    );
    assert!(armed_hover.control_valid);
    assert!(armed_hover.motors.iter().any(|motor| *motor > 0.0));

    let disarmed = pipeline.step(
        EstimateSnapshot {
            position_ned_m: [0.0, 0.0, -0.02],
            valid: true,
            ..EstimateSnapshot::LEVEL_ORIGIN
        },
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([0.0, 0.0, 0.0], 0.0, false),
    );
    assert!(!disarmed.control_valid);
    assert!(!disarmed.armed);
    assert_eq!(disarmed.motors, [0.0; 4]);

    let response = hil_response_from_samples(
        SimStamp {
            sim_tick: 8,
            sim_time_us: 160_000,
        },
        false,
        false,
        Some(StateEstimateSample {
            position_ned_m: [0.0, 0.0, -0.02],
            velocity_ned_mps: [0.0; 3],
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            valid: true,
        }),
        Some(ActuatorOutputSample::new([0.0; 4], false, false)),
    );
    assert_eq!(response.motor_cmd, [0; 4]);
    assert!(response_has_flag(response, response_flags::MOTORS_VALID));
    assert!(!response_has_flag(response, response_flags::ARMED));
    assert!(!response_has_flag(response, response_flags::FAILSAFE));
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g2_report_shape_scores_estimator_agreement_before_navigation() {
    let spec = ScenarioSpec {
        id: "g2_takeoff_hover_land",
        gate: Gate::G2,
        description: "estimator-in-loop takeoff, hover, land, and post-contact disarm",
        reset_required: true,
        command_schedule: CommandSchedule::new(vec![
            ScheduledCommand {
                at_frame: 0,
                kind: CommandKind::SimReset,
            },
            ScheduledCommand {
                at_frame: 30,
                kind: CommandKind::LocalPosition,
            },
            ScheduledCommand {
                at_frame: 300,
                kind: CommandKind::Hold,
            },
            ScheduledCommand {
                at_frame: 520,
                kind: CommandKind::LocalPosition,
            },
        ]),
        thresholds: AcceptanceThresholds {
            max_cross_track_error_m: 0.45,
            max_altitude_error_m: 0.50,
            max_roll_pitch_rad: 0.22,
            max_speed_error_mps: 0.40,
            max_clamp_ratio: 0.08,
        },
    };

    let estimator_fail = ScenarioReport::from_metrics(
        &spec,
        ScenarioMetrics {
            reset_clean: true,
            estimator_agrees: false,
            max_cross_track_error_m: 0.0,
            max_altitude_error_m: 0.0,
            max_roll_pitch_rad: 0.0,
            max_speed_error_mps: 0.0,
            clamp_ratio: 0.0,
            first_clamp_source: None,
        },
    );

    assert!(!estimator_fail.pass);
    assert_eq!(estimator_fail.failure_layer, Some(FailureLayer::Estimator));
    assert!(
        estimator_fail
            .to_csv_row()
            .starts_with("g2_takeoff_hover_land,false")
    );
    assert_eq!(
        ScenarioReport::csv_header().split(',').count(),
        estimator_fail.to_csv_row().split(',').count()
    );
}

fn sensor_frame(timestamp_us: u64, down_m: f64, vertical_velocity_mps: f64) -> SensorFrame {
    SensorFrame {
        timestamp_us,
        accel_mps2: [0.0, 0.0, 0.0],
        gyro_rps: [0.0, 0.0, 0.0],
        gravity_body_mps2: None,
        mag_body_ut: Some([20.0, -4.0, 42.0]),
        gps_position_ned_m: Some([0.0, 0.0, down_m]),
        gps_velocity_ned_mps: Some([0.0, 0.0, vertical_velocity_mps]),
        baro_down_m: Some(down_m),
    }
}

#[test]
#[ignore = "manual Gazebo runtime gate: run with --ignored once the bridge and simulator are active"]
fn g2_sensor_frame_conversion_preserves_ned_altitude_sign() {
    let frame = sensor_frame_to_hil_frame(
        9,
        &sensor_frame(180_000, -1.25, 0.0),
        HilSemanticFrameConfig::default(),
    );

    assert_eq!(frame.stamp.sim_tick, 9);
    assert!(frame.alt_msl_m > HilSemanticFrameConfig::default().origin_alt_msl_m);
    assert!(frame.baro_altitude_m > HilSemanticFrameConfig::default().origin_alt_msl_m);
}
