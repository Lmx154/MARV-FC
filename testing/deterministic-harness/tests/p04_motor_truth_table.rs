use common::control::mixing::{QuadXMixer, TorqueCommand};
use deterministic_harness::{MotorGeometry, OpenLoopPlant, SpinDirection};

#[test]
fn p04_motor_truth_table_geometry_names_positions_outputs_and_spin_directions() {
    let geometry = MotorGeometry::quad_x();

    geometry
        .validate()
        .expect("default quad-X geometry should be valid");
    assert_eq!(geometry.hover_throttle, 0.5);

    let motors = geometry.motors;
    assert_eq!(motors[0].logical_motor, 1);
    assert_eq!(motors[0].output_index, 0);
    assert_eq!(motors[0].position_body_m, [1.0, 1.0, 0.0]);
    assert_eq!(motors[0].spin_direction, SpinDirection::Clockwise);

    assert_eq!(motors[1].logical_motor, 2);
    assert_eq!(motors[1].output_index, 1);
    assert_eq!(motors[1].position_body_m, [1.0, -1.0, 0.0]);
    assert_eq!(motors[1].spin_direction, SpinDirection::CounterClockwise);

    assert_eq!(motors[2].logical_motor, 3);
    assert_eq!(motors[2].output_index, 2);
    assert_eq!(motors[2].position_body_m, [-1.0, -1.0, 0.0]);
    assert_eq!(motors[2].spin_direction, SpinDirection::Clockwise);

    assert_eq!(motors[3].logical_motor, 4);
    assert_eq!(motors[3].output_index, 3);
    assert_eq!(motors[3].position_body_m, [-1.0, 1.0, 0.0]);
    assert_eq!(motors[3].spin_direction, SpinDirection::CounterClockwise);
}

#[test]
fn p04_motor_truth_table_roll_basis_produces_positive_roll_response() {
    let response = response_for(TorqueCommand::new(0.1, 0.0, 0.0, 0.5));

    assert!(response.roll_accel_rps2 > 0.0);
    assert_abs_lt(response.pitch_accel_rps2, 0.000_001);
    assert_abs_lt(response.yaw_accel_rps2, 0.000_001);
}

#[test]
fn p04_motor_truth_table_pitch_basis_produces_positive_pitch_response() {
    let response = response_for(TorqueCommand::new(0.0, 0.1, 0.0, 0.5));

    assert_abs_lt(response.roll_accel_rps2, 0.000_001);
    assert!(response.pitch_accel_rps2 > 0.0);
    assert_abs_lt(response.yaw_accel_rps2, 0.000_001);
}

#[test]
fn p04_motor_truth_table_yaw_basis_produces_positive_yaw_response() {
    let response = response_for(TorqueCommand::new(0.0, 0.0, 0.1, 0.5));

    assert_abs_lt(response.roll_accel_rps2, 0.000_001);
    assert_abs_lt(response.pitch_accel_rps2, 0.000_001);
    assert!(response.yaw_accel_rps2 > 0.0);
}

#[test]
fn p04_motor_truth_table_invalid_geometry_rejects_duplicate_outputs() {
    let mut geometry = MotorGeometry::quad_x();
    geometry.motors[1].output_index = geometry.motors[0].output_index;

    assert_eq!(
        geometry.validate(),
        Err("output_index entries must be unique")
    );
}

fn response_for(command: TorqueCommand) -> deterministic_harness::OpenLoopResponse {
    let outputs = QuadXMixer::default().mix(command);
    OpenLoopPlant::default().apply_outputs(outputs.commands, 0.0)
}

fn assert_abs_lt(value: f32, limit: f32) {
    assert!(
        value.abs() < limit,
        "expected |{value}| to be less than {limit}"
    );
}
