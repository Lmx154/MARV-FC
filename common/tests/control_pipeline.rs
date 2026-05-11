use common::control::altitude::AltitudeController;
use common::control::attitude::{AttitudeController, AttitudeSetpoint, BodyRateSetpoint};
use common::control::mixing::{MotorOrder, QuadXMixer, TorqueCommand};
use common::control::position::{
    PositionController, PositionControllerInput, PositionControllerSetpoint,
};
use common::control::rate::{BodyRates, RateController};
use common::localization::estimation::{LayeredNavigationStack, Vec3};

const TOLERANCE: f32 = 0.000_001;

#[test]
fn mixer_contract_oracles_cover_basis_clamping_and_motor_order() {
    let mixer = QuadXMixer::default();

    assert_commands_near(
        mixer.mix(TorqueCommand::new(0.0, 0.0, 0.0, 0.25)).commands,
        [0.25, 0.25, 0.25, 0.25],
    );
    assert_commands_near(
        mixer.mix(TorqueCommand::new(0.1, 0.0, 0.0, 0.5)).commands,
        [0.6, 0.4, 0.4, 0.6],
    );
    assert_commands_near(
        mixer.mix(TorqueCommand::new(0.0, 0.1, 0.0, 0.5)).commands,
        [0.6, 0.6, 0.4, 0.4],
    );
    assert_commands_near(
        mixer.mix(TorqueCommand::new(0.0, 0.0, 0.1, 0.5)).commands,
        [0.4, 0.6, 0.4, 0.6],
    );

    let clamped = mixer.mix(TorqueCommand::new(0.4, 0.4, 0.0, 0.5));
    assert_commands_near(clamped.commands, [1.0, 0.5, 0.0, 0.5]);
    assert!(clamped.clamped);

    let remapped = QuadXMixer::with_motor_order(
        Default::default(),
        MotorOrder::from_one_based([3, 1, 4, 2]).expect("valid one-based motor order"),
    )
    .mix(TorqueCommand::new(0.1, 0.2, 0.05, 0.5));
    assert_commands_near(remapped.commands, [0.65, 0.45, 0.75, 0.15]);

    assert!(MotorOrder::from_one_based([1, 2, 3, 4]).is_some());
    assert!(MotorOrder::from_one_based([1, 1, 3, 4]).is_none());
    assert!(MotorOrder::from_one_based([0, 2, 3, 4]).is_none());
    assert!(MotorOrder::from_one_based([1, 2, 3, 5]).is_none());
}

#[test]
fn attitude_and_rate_sign_oracles_oppose_positive_errors() {
    let attitude = AttitudeController::default();
    let rate = RateController::default();

    let positive_roll_to_level = attitude
        .update(AttitudeSetpoint::LEVEL, quat_from_euler(10.0, 0.0, 0.0))
        .expect("positive roll attitude should produce a rate setpoint");
    assert!(
        positive_roll_to_level.roll_rps < 0.0,
        "expected positive roll attitude error to command negative roll rate"
    );

    let positive_pitch_to_level = attitude
        .update(AttitudeSetpoint::LEVEL, quat_from_euler(0.0, 10.0, 0.0))
        .expect("positive pitch attitude should produce a rate setpoint");
    assert!(
        positive_pitch_to_level.pitch_rps < 0.0,
        "expected positive pitch attitude error to command negative pitch rate"
    );

    let positive_yaw_to_level = attitude
        .update(AttitudeSetpoint::LEVEL, quat_from_euler(0.0, 0.0, 10.0))
        .expect("positive yaw attitude should produce a rate setpoint");
    assert!(
        positive_yaw_to_level.yaw_rps < 0.0,
        "expected positive yaw attitude error to command negative yaw rate"
    );

    let zero_error = attitude
        .update(AttitudeSetpoint::LEVEL, AttitudeSetpoint::LEVEL.quaternion)
        .expect("level attitude should produce zero body-rate setpoint");
    assert_scalar_near(zero_error.roll_rps, 0.0, TOLERANCE);
    assert_scalar_near(zero_error.pitch_rps, 0.0, TOLERANCE);
    assert_scalar_near(zero_error.yaw_rps, 0.0, TOLERANCE);

    let damping = rate.update(BodyRateSetpoint::ZERO, BodyRates::new(1.0, 1.0, 1.0), 0.5);
    assert!(
        damping.roll < 0.0,
        "expected positive measured roll rate to command negative roll torque"
    );
    assert!(
        damping.pitch < 0.0,
        "expected positive measured pitch rate to command negative pitch torque"
    );
    assert!(
        damping.yaw < 0.0,
        "expected positive measured yaw rate to command negative yaw torque"
    );
}

#[test]
fn altitude_and_position_sign_oracles_follow_local_ned() {
    let altitude = AltitudeController::default();

    let hover = altitude
        .update(0.0, 0.0, 0.0)
        .expect("finite altitude input should produce throttle");
    let below_setpoint = altitude
        .update(-2.0, 0.0, 0.0)
        .expect("finite altitude input should produce throttle");
    let above_setpoint = altitude
        .update(0.0, -2.0, 0.0)
        .expect("finite altitude input should produce throttle");

    assert!(below_setpoint > hover);
    assert!(above_setpoint < hover);

    let position = PositionController::default();
    let north = position
        .update(
            PositionControllerSetpoint::new([10.0, 0.0, 0.0], 0.0),
            PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        )
        .expect("finite position input should produce attitude");
    let east = position
        .update(
            PositionControllerSetpoint::new([0.0, 10.0, 0.0], 0.0),
            PositionControllerInput::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        )
        .expect("finite position input should produce attitude");

    assert!(
        north.quaternion[2] < 0.0,
        "expected north error to command nose-down pitch"
    );
    assert!(
        east.quaternion[1] > 0.0,
        "expected east error to command right roll"
    );
}

#[test]
fn estimator_static_sanity_keeps_state_finite_and_normalized() {
    let mut stack = LayeredNavigationStack::<f64>::default();

    stack
        .predict(Vec3::<f64>::zeros(), Vec3::<f64>::zeros(), 0.0, Some(0.0))
        .expect("zero-dt first sample should be a no-op prediction");
    assert_eq!(stack.last_prediction_dt_s, Some(0.0));

    for step in 1..=100 {
        let timestamp_s = step as f64 * 0.01;
        stack
            .predict(
                Vec3::<f64>::zeros(),
                Vec3::<f64>::zeros(),
                0.01,
                Some(timestamp_s),
            )
            .expect("static finite IMU prediction should remain valid");
        stack
            .update_gravity_alignment(Vec3::<f64>::new(0.0, 0.0, -9.806_65), None)
            .expect("static gravity update should be accepted");
    }

    let state = stack.state();
    assert_quaternion_normalized(state.quaternion.as_slice());
    assert_slice_finite(state.position_m.as_slice());
    assert_slice_finite(state.velocity_mps.as_slice());
    assert_slice_finite(state.gyro_bias_rps.as_slice());
    assert_slice_finite(state.accel_bias_mps2.as_slice());
    assert_eq!(stack.last_prediction_dt_s, Some(0.01));
    assert_eq!(stack.last_timestamp_s, Some(1.0));
    assert_quaternion_near(state.quaternion.as_slice(), &[1.0, 0.0, 0.0, 0.0], 1.0e-9);
}

fn quat_from_euler(roll_deg: f32, pitch_deg: f32, yaw_deg: f32) -> [f32; 4] {
    let deg_to_rad = core::f32::consts::PI / 180.0;
    AttitudeSetpoint::from_euler_rad(
        roll_deg * deg_to_rad,
        pitch_deg * deg_to_rad,
        yaw_deg * deg_to_rad,
    )
    .expect("finite euler angles should produce a quaternion")
    .quaternion
}

fn assert_commands_near(actual: [f32; 4], expected: [f32; 4]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert_scalar_near(actual, expected, TOLERANCE);
    }
}

fn assert_scalar_near(actual: f32, expected: f32, tolerance: f32) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "expected {actual} to be within {tolerance} of {expected}"
    );
}

fn assert_quaternion_normalized(quaternion: &[f64]) {
    let norm = quaternion
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    assert!(
        (norm - 1.0).abs() <= 1.0e-9,
        "expected normalized quaternion, got norm {norm}"
    );
}

fn assert_quaternion_near(actual: &[f64], expected: &[f64], tolerance: f64) {
    for (actual, expected) in actual.iter().zip(expected) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}"
        );
    }
}

fn assert_slice_finite(values: &[f64]) {
    assert!(
        values.iter().all(|value| value.is_finite()),
        "expected all values to be finite: {values:?}"
    );
}
