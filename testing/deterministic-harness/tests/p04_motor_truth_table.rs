use common::control::mixing::{
    DesaturationPriority, MixerLimits, PhysicalControlAllocator, QuadXMixer, TorqueCommand,
};
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
fn p04_motor_truth_table_physical_allocator_matches_legacy_quad_x_basis() {
    let allocator = PhysicalControlAllocator::default();
    let mixer = QuadXMixer::default();

    for command in [
        TorqueCommand::new(0.1, 0.0, 0.0, 0.5),
        TorqueCommand::new(0.0, 0.1, 0.0, 0.5),
        TorqueCommand::new(0.0, 0.0, 0.1, 0.5),
    ] {
        assert_eq!(
            allocator.allocate(command).commands,
            mixer.mix(command).commands
        );
    }
}

#[test]
fn p04_motor_truth_table_common_f450_geometry_matches_profile_constants() {
    let geometry = MotorGeometry::f450_xing2_2809_1045_4s_v0();

    geometry
        .validate()
        .expect("common F450 geometry should be valid");
    assert_near(geometry.mass_kg, 1.338);
    assert_near(geometry.hover_throttle, 630.0 / 1100.0);
    assert_eq!(
        geometry.motors[0].position_body_m,
        [-0.160_867, -0.160_867, 0.0]
    );
    assert_eq!(
        geometry.motors[3].position_body_m,
        [0.160_867, 0.160_867, 0.0]
    );
}

#[test]
fn p04_motor_truth_table_allocator_reports_desaturation_priority() {
    let allocator =
        PhysicalControlAllocator::new(MotorGeometry::quad_x(), MixerLimits::new(0.45, 0.55));

    let outputs = allocator.allocate(TorqueCommand::new(0.2, 0.2, 0.2, 0.5));

    assert!(outputs.clamped);
    assert!(outputs.limit_flags.roll_pitch);
    assert!(outputs.limit_flags.yaw);
    assert_eq!(
        outputs.desaturation.priority,
        DesaturationPriority::PreserveRollPitchOverYaw
    );
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

fn assert_near(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() <= 0.000_001,
        "expected {actual} to be near {expected}"
    );
}
