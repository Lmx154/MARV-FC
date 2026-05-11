use common::control::attitude::AttitudeSetpoint;
use common::control::mixing::{MotorOrder, QuadXMixer, TorqueCommand};
use common::localization::navigation::{GeodeticPosition, LocalNedFrame};

#[test]
fn host_harness_can_reference_frozen_common_conventions() {
    let frame = LocalNedFrame::new(GeodeticPosition::new(30.0, -97.0, 120.0));
    let nearby = frame
        .position_ned_m(GeodeticPosition::new(30.000_01, -96.999_99, 125.0))
        .expect("finite position maps into local NED");

    assert!(nearby[0] > 0.0, "positive latitude offset is north");
    assert!(nearby[1] > 0.0, "positive longitude offset is east");
    assert!(nearby[2] < 0.0, "higher altitude is negative down");

    let level = AttitudeSetpoint::LEVEL;
    assert_eq!(level.quaternion, [1.0, 0.0, 0.0, 0.0]);

    let mixer = QuadXMixer::with_motor_order(
        Default::default(),
        MotorOrder::from_one_based([1, 2, 3, 4]).expect("identity order is valid"),
    );
    let outputs = mixer.mix(TorqueCommand::new(0.1, 0.2, 0.05, 0.5));
    assert_commands_near(outputs.commands, [0.75, 0.65, 0.15, 0.45]);
}

fn assert_commands_near(actual: [f32; 4], expected: [f32; 4]) {
    for (actual, expected) in actual.into_iter().zip(expected) {
        assert!(
            (actual - expected).abs() <= 0.000_001,
            "expected {actual} to be within tolerance of {expected}"
        );
    }
}
