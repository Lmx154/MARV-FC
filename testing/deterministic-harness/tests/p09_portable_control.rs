use deterministic_harness::{
    ControlInput, ControlPipeline, ControlSetpoint, EstimateSnapshot, ImuControlInput,
};

#[test]
fn p09_portable_control_invalid_estimate_failsafe_zero_output() {
    let output = ControlPipeline::default().step_input(ControlInput::new(
        EstimateSnapshot {
            valid: false,
            ..EstimateSnapshot::LEVEL_ORIGIN
        },
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    ));

    assert!(output.armed);
    assert!(!output.control_valid);
    assert_eq!(output.motors, [0.0; 4]);
    assert!(!output.clamped);
    assert!(output.debug.motor_outputs.is_none());
}

#[test]
fn p09_portable_control_missing_imu_failsafe_zero_output() {
    let output = ControlPipeline::default().step_input(ControlInput::without_imu(
        EstimateSnapshot::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    ));

    assert!(output.armed);
    assert!(!output.control_valid);
    assert_eq!(output.motors, [0.0; 4]);
    assert!(output.debug.attitude_setpoint.is_none());
}

#[test]
fn p09_portable_control_non_finite_input_failsafe_zero_output() {
    let output = ControlPipeline::default().step_input(ControlInput::new(
        EstimateSnapshot {
            position_ned_m: [0.0, f32::NAN, 0.0],
            ..EstimateSnapshot::LEVEL_ORIGIN
        },
        ImuControlInput::default(),
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    ));

    assert!(output.armed);
    assert!(!output.control_valid);
    assert_eq!(output.motors, [0.0; 4]);
}

#[test]
fn p09_portable_control_disarmed_zero_output_is_not_clamped() {
    let output = ControlPipeline::default().step_input(ControlInput::new(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput::default(),
        ControlSetpoint::local_position_ned([0.0, 0.0, 0.0], 0.0, false),
    ));

    assert!(!output.armed);
    assert!(!output.control_valid);
    assert_eq!(output.motors, [0.0; 4]);
    assert!(!output.clamped);
}
