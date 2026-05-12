use common::control::attitude::AttitudeSetpoint;
use common::control::config::ControlLoopConfig;
use common::control::mixing::MixerLimits;
use common::control::rate::RateControllerConfig;
use deterministic_harness::{
    ControlPipeline, ControlPipelineTrace, ControlSetpoint, EstimateSnapshot, ImuControlInput,
    PureControlConfig,
};

#[test]
fn p03_control_pipeline_disarmed_outputs_zero_motors() {
    let trace = ControlPipeline::default().step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint {
            armed: false,
            ..ControlSetpoint::ORIGIN_HOLD_ARMED
        },
    );

    assert!(!trace.armed);
    assert!(!trace.control_valid);
    assert_eq!(trace.motors, [0.0; 4]);
    assert!(!trace.clamped);
    assert!(trace.attitude_setpoint.is_none());
    assert!(trace.rate_setpoint.is_none());
}

#[test]
fn p03_control_pipeline_level_hold_zero_rate_near_hover() {
    let trace = armed_origin_step(EstimateSnapshot::LEVEL_ORIGIN, ImuControlInput::default());

    assert_valid_trace_has_required_fields(&trace);
    assert_near(trace.throttle, 0.5, 0.000_001);
    assert_near(trace.torque_command.roll, 0.0, 0.000_001);
    assert_near(trace.torque_command.pitch, 0.0, 0.000_001);
    assert_near(trace.torque_command.yaw, 0.0, 0.000_001);
    assert_commands_near(trace.motors, [0.5, 0.5, 0.5, 0.5], 0.000_001);
    assert!(!trace.clamped);
}

#[test]
fn p03_control_pipeline_positive_roll_disturbance_commands_opposing_torque() {
    let trace = armed_origin_step(
        estimate_with_quaternion(quat_from_euler_deg(10.0, 0.0, 0.0)),
        ImuControlInput::default(),
    );

    assert_valid_trace_has_required_fields(&trace);
    assert!(trace.rate_setpoint.expect("rate setpoint").roll_rps < 0.0);
    assert!(
        trace.torque_command.roll < 0.0,
        "expected positive roll disturbance to command negative roll torque"
    );
}

#[test]
fn p03_control_pipeline_positive_pitch_disturbance_commands_opposing_torque() {
    let trace = armed_origin_step(
        estimate_with_quaternion(quat_from_euler_deg(0.0, 10.0, 0.0)),
        ImuControlInput::default(),
    );

    assert_valid_trace_has_required_fields(&trace);
    assert!(trace.rate_setpoint.expect("rate setpoint").pitch_rps < 0.0);
    assert!(
        trace.torque_command.pitch < 0.0,
        "expected positive pitch disturbance to command negative pitch torque"
    );
}

#[test]
fn p03_control_pipeline_positive_yaw_rate_commands_opposing_yaw() {
    let trace = armed_origin_step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput {
            accel_mps2: [0.0, 0.0, 0.0],
            gyro_rps: [0.0, 0.0, 1.0],
        },
    );

    assert_valid_trace_has_required_fields(&trace);
    assert!(
        trace.torque_command.yaw < 0.0,
        "expected positive measured yaw rate to command negative yaw torque"
    );
}

#[test]
fn p03_control_pipeline_altitude_error_ned_down_sign_is_correct() {
    let pipeline = ControlPipeline::default();
    let below_setpoint = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint {
            position_ned_m: [0.0, 0.0, -2.0],
            yaw_rad: 0.0,
            armed: true,
        },
    );
    let above_setpoint = pipeline.step(
        EstimateSnapshot {
            position_ned_m: [0.0, 0.0, -2.0],
            ..EstimateSnapshot::LEVEL_ORIGIN
        },
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );

    assert_valid_trace_has_required_fields(&below_setpoint);
    assert_valid_trace_has_required_fields(&above_setpoint);
    assert!(
        below_setpoint.throttle > 0.5,
        "below the NED-down setpoint should increase throttle"
    );
    assert!(
        above_setpoint.throttle < 0.5,
        "above the NED-down setpoint should decrease throttle"
    );
}

#[test]
fn p03_control_pipeline_saturation_sets_clamped_flag() {
    let mut config = ControlLoopConfig::default();
    config.rate = RateControllerConfig {
        roll_gain: 4.0,
        pitch_gain: 4.0,
        yaw_gain: 4.0,
        max_axis_command: 2.0,
        measured_rate_deadband_rps: 0.0,
    };
    config.mixer_limits = MixerLimits::NORMALIZED;
    let pipeline = ControlPipeline::new(PureControlConfig {
        loop_config: config,
    });
    let trace = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput {
            accel_mps2: [0.0, 0.0, 0.0],
            gyro_rps: [-1.0, -1.0, 0.0],
        },
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );

    assert_valid_trace_has_required_fields(&trace);
    assert!(trace.clamped);
    assert!(trace.motors.iter().any(|motor| *motor >= 1.0));
}

fn armed_origin_step(estimate: EstimateSnapshot, imu: ImuControlInput) -> ControlPipelineTrace {
    ControlPipeline::default().step(estimate, imu, ControlSetpoint::ORIGIN_HOLD_ARMED)
}

fn estimate_with_quaternion(quaternion: [f32; 4]) -> EstimateSnapshot {
    EstimateSnapshot {
        quaternion,
        ..EstimateSnapshot::LEVEL_ORIGIN
    }
}

fn quat_from_euler_deg(roll_deg: f32, pitch_deg: f32, yaw_deg: f32) -> [f32; 4] {
    let deg_to_rad = core::f32::consts::PI / 180.0;
    AttitudeSetpoint::from_euler_rad(
        roll_deg * deg_to_rad,
        pitch_deg * deg_to_rad,
        yaw_deg * deg_to_rad,
    )
    .expect("finite euler angles should produce a quaternion")
    .quaternion
}

fn assert_valid_trace_has_required_fields(trace: &ControlPipelineTrace) {
    assert!(trace.armed);
    assert!(trace.control_valid);
    assert!(trace.estimate.valid);
    assert!(
        trace
            .estimate
            .position_ned_m
            .iter()
            .all(|value| value.is_finite())
    );
    assert!(
        trace
            .estimate
            .velocity_ned_mps
            .iter()
            .all(|value| value.is_finite())
    );
    assert!(
        trace
            .estimate
            .quaternion
            .iter()
            .all(|value| value.is_finite())
    );
    assert!(trace.imu.accel_mps2.iter().all(|value| value.is_finite()));
    assert!(trace.imu.gyro_rps.iter().all(|value| value.is_finite()));
    assert!(trace.attitude_setpoint.is_some());
    assert!(trace.rate_setpoint.is_some());
    assert!(trace.throttle.is_finite());
    assert!(trace.torque_command.roll.is_finite());
    assert!(trace.torque_command.pitch.is_finite());
    assert!(trace.torque_command.yaw.is_finite());
    assert!(trace.torque_command.throttle.is_finite());
    assert!(trace.motors.iter().all(|motor| motor.is_finite()));
}

fn assert_commands_near(actual: [f32; 4], expected: [f32; 4], tolerance: f32) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert_near(actual, expected, tolerance);
    }
}

fn assert_near(actual: f32, expected: f32, tolerance: f32) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {actual} to be within {tolerance} of {expected}"
    );
}
