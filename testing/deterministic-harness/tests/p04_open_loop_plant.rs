use common::control::mixing::TorqueCommand;
use deterministic_harness::OpenLoopPlant;

#[test]
fn p04_open_loop_plant_all_motors_equal_low_throttle_has_no_attitude_torque_and_descends() {
    let mut plant = OpenLoopPlant::default();

    let response = plant.apply_outputs([0.25, 0.25, 0.25, 0.25], 0.1);

    assert_abs_lt(response.roll_accel_rps2, 0.000_001);
    assert_abs_lt(response.pitch_accel_rps2, 0.000_001);
    assert_abs_lt(response.yaw_accel_rps2, 0.000_001);
    assert!(
        response.vertical_accel_down_mps2 > 0.0,
        "low throttle should accelerate down in NED"
    );
    assert!(response.next_state.velocity_down_mps > 0.0);
}

#[test]
fn p04_open_loop_plant_all_motors_equal_hover_throttle_has_near_zero_vertical_accel() {
    let mut plant = OpenLoopPlant::default();

    let response = plant.apply_outputs([0.5, 0.5, 0.5, 0.5], 0.1);

    assert_abs_lt(response.roll_accel_rps2, 0.000_001);
    assert_abs_lt(response.pitch_accel_rps2, 0.000_001);
    assert_abs_lt(response.yaw_accel_rps2, 0.000_001);
    assert_abs_lt(response.vertical_accel_down_mps2, 0.000_001);
    assert_abs_lt(response.next_state.velocity_down_mps, 0.000_001);
}

#[test]
fn p04_open_loop_plant_roll_basis_integrates_positive_roll_rate() {
    let mut plant = OpenLoopPlant::default();

    let response = plant.apply_torque_command(TorqueCommand::new(0.1, 0.0, 0.0, 0.5), 0.1);

    assert!(response.roll_accel_rps2 > 0.0);
    assert!(response.next_state.roll_rate_rps > 0.0);
}

#[test]
fn p04_open_loop_plant_pitch_basis_integrates_positive_pitch_rate() {
    let mut plant = OpenLoopPlant::default();

    let response = plant.apply_torque_command(TorqueCommand::new(0.0, 0.1, 0.0, 0.5), 0.1);

    assert!(response.pitch_accel_rps2 > 0.0);
    assert!(response.next_state.pitch_rate_rps > 0.0);
}

#[test]
fn p04_open_loop_plant_yaw_basis_integrates_positive_yaw_rate() {
    let mut plant = OpenLoopPlant::default();

    let response = plant.apply_torque_command(TorqueCommand::new(0.0, 0.0, 0.1, 0.5), 0.1);

    assert!(response.yaw_accel_rps2 > 0.0);
    assert!(response.next_state.yaw_rate_rps > 0.0);
}

fn assert_abs_lt(value: f32, limit: f32) {
    assert!(
        value.abs() < limit,
        "expected |{value}| to be less than {limit}"
    );
}
