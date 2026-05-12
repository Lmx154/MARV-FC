use common::control::{
    altitude::AltitudeControllerConfig, config::ControlLoopConfig, mixing::MixerLimits,
    position::PositionControllerConfig, rate::RateControllerConfig,
};
use deterministic_harness::{
    ControlPipeline, ControlPipelineTrace, ControlSetpoint, EstimateSnapshot, ImuControlInput,
    PureControlConfig,
};

#[derive(Clone, Copy, Debug, PartialEq)]
struct DemandLimitSummary {
    configured_max_tilt_rad: f32,
    measured_tilt_rad: f32,
    configured_max_horizontal_accel_mps2: f32,
    inferred_horizontal_accel_mps2: f32,
    configured_max_throttle_correction: f32,
    measured_throttle_correction: f32,
    configured_max_axis_command: f32,
    measured_axis_command: f32,
    clamped: bool,
    control_valid: bool,
    motors: [f32; 4],
}

#[test]
fn p14_g2_demand_limits_bound_extreme_position_and_altitude_requests() {
    let config = demand_limit_config();
    let pipeline = ControlPipeline::new(PureControlConfig {
        loop_config: config,
    });
    let output = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::new([1_000.0, -1_000.0, -100.0], 90.0_f32.to_radians(), true),
    );
    let summary = summarize_demand_limits(output, config);

    assert!(
        summary.control_valid,
        "control should stay valid: {summary:?}"
    );
    assert!(
        summary.measured_tilt_rad <= summary.configured_max_tilt_rad + 0.000_5,
        "tilt limit not enforced: {summary:?}"
    );
    assert!(
        summary.inferred_horizontal_accel_mps2
            <= summary.configured_max_horizontal_accel_mps2 + 0.01,
        "horizontal acceleration limit not enforced: {summary:?}"
    );
    assert!(
        summary.measured_throttle_correction
            <= summary.configured_max_throttle_correction + 0.000_001,
        "throttle correction limit not enforced: {summary:?}"
    );
    assert!(
        summary.measured_axis_command <= summary.configured_max_axis_command + 0.000_001,
        "rate/torque command limit not enforced: {summary:?}"
    );
    assert!(
        summary
            .motors
            .iter()
            .all(|motor| (0.0..=1.0).contains(motor)),
        "motors should remain normalized: {summary:?}"
    );
}

#[test]
fn p14_g2_demand_limits_report_mixer_saturation() {
    let mut config = demand_limit_config();
    config.mixer_limits = MixerLimits::new(0.45, 0.55);
    config.rate = RateControllerConfig {
        max_axis_command: 0.40,
        ..config.rate
    };
    let pipeline = ControlPipeline::new(PureControlConfig {
        loop_config: config,
    });
    let output = pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::new([50.0, 50.0, -50.0], 180.0_f32.to_radians(), true),
    );
    let summary = summarize_demand_limits(output, config);

    assert!(
        summary.control_valid,
        "control should stay valid: {summary:?}"
    );
    assert!(
        summary.clamped,
        "mixer saturation should be visible: {summary:?}"
    );
    assert!(
        summary
            .motors
            .iter()
            .all(|motor| (config.mixer_limits.min..=config.mixer_limits.max).contains(motor)),
        "clamped motors should stay inside configured mixer limits: {summary:?}"
    );
}

#[test]
fn p14_g2_demand_limits_failsafe_zero_is_final_safe_state_for_bad_demands() {
    let output = ControlPipeline::default().step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::new([0.0, f32::NAN, 0.0], 0.0, true),
    );

    assert!(output.armed);
    assert!(!output.control_valid);
    assert_eq!(output.motors, [0.0; 4]);
    assert!(!output.clamped);
    assert!(output.debug.motor_outputs.is_none());
}

fn demand_limit_config() -> ControlLoopConfig {
    let mut config = ControlLoopConfig::default();
    config.position = PositionControllerConfig {
        position_gain: 8.0,
        velocity_gain: 0.0,
        max_horizontal_accel_mps2: 0.8,
        max_tilt_rad: 7.0_f32.to_radians(),
    };
    config.altitude = AltitudeControllerConfig {
        hover_throttle: 0.5,
        altitude_gain: 4.0,
        vertical_velocity_gain: 0.0,
        max_altitude_error_m: 100.0,
        max_vertical_velocity_mps: 100.0,
        max_throttle_correction: 0.10,
    };
    config.rate = RateControllerConfig {
        roll_gain: 1.0,
        pitch_gain: 1.0,
        yaw_gain: 1.0,
        max_axis_command: 0.12,
        measured_rate_deadband_rps: 0.0,
    };
    config
}

fn summarize_demand_limits(
    output: ControlPipelineTrace,
    config: ControlLoopConfig,
) -> DemandLimitSummary {
    let attitude_setpoint = output
        .attitude_setpoint
        .expect("valid demand should produce an attitude setpoint");
    let (roll_rad, pitch_rad, _) = euler_from_quaternion(attitude_setpoint.quaternion);
    let measured_tilt_rad = roll_rad.abs().max(pitch_rad.abs());
    let inferred_horizontal_accel_mps2 = measured_tilt_rad * 9.806_65;
    let measured_axis_command = output
        .torque_command
        .roll
        .abs()
        .max(output.torque_command.pitch.abs())
        .max(output.torque_command.yaw.abs());

    DemandLimitSummary {
        configured_max_tilt_rad: config.position.max_tilt_rad,
        measured_tilt_rad,
        configured_max_horizontal_accel_mps2: config.position.max_horizontal_accel_mps2,
        inferred_horizontal_accel_mps2,
        configured_max_throttle_correction: config.altitude.max_throttle_correction,
        measured_throttle_correction: (output.throttle - config.altitude.hover_throttle).abs(),
        configured_max_axis_command: config.rate.max_axis_command,
        measured_axis_command,
        clamped: output.clamped,
        control_valid: output.control_valid,
        motors: output.motors,
    }
}

fn euler_from_quaternion(q: [f32; 4]) -> (f32, f32, f32) {
    let [w, x, y, z] = q;
    let roll = f32::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let pitch = f32::asin((2.0 * (w * y - z * x)).clamp(-1.0, 1.0));
    let yaw = f32::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    (roll, pitch, yaw)
}
