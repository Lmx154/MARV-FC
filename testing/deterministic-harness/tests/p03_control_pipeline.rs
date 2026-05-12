use common::control::attitude::AttitudeSetpoint;
use common::control::config::ControlLoopConfig;
use common::control::mixing::MixerLimits;
use common::control::rate::RateControllerConfig;
use deterministic_harness::{
    ClampSource, ControlFrame, ControlNavPhase, ControlPipeline, ControlPipelineTrace,
    ControlSetpoint, EstimateSnapshot, ImuControlInput, LocalTrajectorySetpoint, PureControlConfig,
};

#[test]
fn p03_control_pipeline_disarmed_outputs_zero_motors() {
    let trace = ControlPipeline::default().step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([0.0, 0.0, 0.0], 0.0, false),
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
        ControlSetpoint::local_position_ned([0.0, 0.0, -2.0], 0.0, true),
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
    assert!(trace.debug.mixer_limit_flags.any());
    assert!(trace.debug.mixer_limit_flags.roll_pitch);
    assert!(trace.motors.iter().any(|motor| *motor >= 1.0));
}

#[test]
fn p03_control_pipeline_reports_rate_axis_limits_before_mixing() {
    let mut config = ControlLoopConfig::default();
    config.rate = RateControllerConfig {
        roll_gain: 1.0,
        pitch_gain: 1.0,
        yaw_gain: 1.0,
        max_axis_command: 0.1,
        measured_rate_deadband_rps: 0.0,
    };
    let pipeline = ControlPipeline::new(PureControlConfig {
        loop_config: config,
    });
    let trace = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput {
            accel_mps2: [0.0, 0.0, 0.0],
            gyro_rps: [-1.0, 1.0, -1.0],
        },
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    );

    assert_valid_trace_has_required_fields(&trace);
    assert!(trace.debug.rate_limit_flags.roll);
    assert!(trace.debug.rate_limit_flags.pitch);
    assert!(trace.debug.rate_limit_flags.yaw);
    let rate_output = trace
        .debug
        .rate_controller_output
        .expect("rate output should be present");
    assert_eq!(rate_output.limit_flags, trace.debug.rate_limit_flags);
    assert!(rate_output.raw_command.roll > trace.torque_command.roll);
}

#[test]
fn p03_control_pipeline_preserves_trajectory_velocity_and_accel_feed_forward() {
    let trajectory = LocalTrajectorySetpoint {
        position_ned_m: [0.0, 0.0, 0.0],
        velocity_ned_mps: [1.0, 0.0, 0.0],
        acceleration_ned_mps2: [0.2, 0.0, 0.0],
        yaw_rad: 0.0,
        yaw_rate_rad_s: 0.0,
    };
    let trace = ControlPipeline::default().step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::from_local_trajectory(trajectory, true),
    );

    assert_valid_trace_has_required_fields(&trace);
    assert_eq!(trace.debug.velocity_setpoint_ned_mps, Some([0.5, 0.0]));
    assert_eq!(trace.debug.feed_forward_accel_ned_mps2, Some([0.2, 0.0]));
    assert_eq!(trace.debug.velocity_error_ned_mps, Some([0.5, 0.0]));
    let requested_accel = trace
        .debug
        .requested_accel_ned_mps2
        .expect("trajectory trace should include requested acceleration");
    assert_near(requested_accel[0], 0.375, 0.000_001);
    assert_near(requested_accel[1], 0.0, 0.000_001);
    assert!(
        trace
            .attitude_setpoint
            .expect("trajectory trace should have attitude setpoint")
            .quaternion[2]
            < 0.0,
        "positive north feed-forward should command nose-down pitch"
    );
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
    assert_eq!(trace.debug.estimator_frame, ControlFrame::EstimatorLocalNed);
    assert_eq!(trace.debug.setpoint_frame, ControlFrame::EstimatorLocalNed);
    assert_eq!(trace.debug.nav_phase, ControlNavPhase::DirectSetpoint);
    assert_eq!(trace.debug.takeoff_origin_ned_m, None);
    assert_finite_optional_f32x3(trace.debug.control_setpoint_ned_m);
    assert_finite_optional_f32x3(trace.debug.position_error_ned_m);
    assert_finite_optional_f32x2(trace.debug.velocity_setpoint_ned_mps);
    assert_finite_optional_f32x2(trace.debug.velocity_error_ned_mps);
    assert_finite_optional_f32x2(trace.debug.feed_forward_accel_ned_mps2);
    assert_finite_optional_f32x2(trace.debug.requested_accel_ned_mps2);
    assert_finite_optional_f32x2(trace.debug.applied_accel_ned_mps2);
    assert_finite_optional_f32x2(trace.debug.integral_accel_ned_mps2);
    assert!(trace.debug.thrust_vector_setpoint.is_some());
    assert_finite_optional_scalar(trace.debug.requested_collective_throttle);
    assert_finite_optional_scalar(trace.debug.applied_collective_throttle);
    assert_finite_optional_scalar(trace.debug.tilt_compensation);
    assert_finite_optional_scalar(trace.debug.vertical_throttle_margin);
    assert!(trace.debug.rate_controller_output.is_some());
    assert_eq!(
        trace
            .debug
            .rate_controller_output
            .expect("rate output")
            .limit_flags,
        trace.debug.rate_limit_flags
    );
    assert_eq!(
        trace
            .debug
            .motor_outputs
            .expect("motor outputs")
            .limit_flags,
        trace.debug.mixer_limit_flags
    );
    assert_eq!(
        trace.debug.clamp_source,
        if trace.clamped {
            ClampSource::MotorOutputLimit
        } else {
            ClampSource::None
        }
    );
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

fn assert_finite_optional_f32x2(values: Option<[f32; 2]>) {
    let values = values.expect("debug field should be present for valid control traces");
    assert!(values.iter().all(|value| value.is_finite()));
}

fn assert_finite_optional_f32x3(values: Option<[f32; 3]>) {
    let values = values.expect("debug field should be present for valid control traces");
    assert!(values.iter().all(|value| value.is_finite()));
}

fn assert_finite_optional_scalar(value: Option<f32>) {
    let value = value.expect("debug field should be present for valid control traces");
    assert!(value.is_finite());
}
