use deterministic_harness::{
    ClosedLoopConfig, ClosedLoopRunner, ClosedLoopTrace, ClosedLoopTruthState, ControlPipeline,
    ControlSetpoint, TruthPlant,
};

const DT_S: f32 = 0.01;
const TICKS_2S: usize = 200;
const TICKS_3S: usize = 300;

#[test]
fn p05_closed_loop_truth_static_level_hold_2s() {
    let traces = run_case(
        ClosedLoopTruthState::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        TICKS_2S,
    );
    let final_truth = traces.last().expect("trace").truth;

    assert_all_traces_finite_and_normalized(&traces);
    assert!(final_truth.euler_rad[0].abs() < 0.001);
    assert!(final_truth.euler_rad[1].abs() < 0.001);
    assert!(final_truth.euler_rad[2].abs() < 0.001);
    assert!(final_truth.position_ned_m[2].abs() < 0.001);
    assert!(traces.iter().all(|trace| !trace.control.clamped));
}

#[test]
fn p05_closed_loop_truth_roll_disturbance_recovers() {
    let initial_roll_rad = 10.0_f32.to_radians();
    let initial = ClosedLoopTruthState {
        euler_rad: [initial_roll_rad, 0.0, 0.0],
        ..ClosedLoopTruthState::LEVEL_ORIGIN
    };
    let traces = run_case(initial, ControlSetpoint::ORIGIN_HOLD_ARMED, TICKS_3S);
    let final_roll = traces.last().expect("trace").truth.euler_rad[0];

    assert_all_traces_finite_and_normalized(&traces);
    assert!(
        final_roll.abs() < initial_roll_rad.abs(),
        "expected final roll |{final_roll}| to be less than initial |{initial_roll_rad}|"
    );
    assert!(final_roll.abs() < 5.0_f32.to_radians());
}

#[test]
fn p05_closed_loop_truth_vertical_setpoint_truth_estimator_moves_toward_target() {
    let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true);
    let traces = run_case(ClosedLoopTruthState::LEVEL_ORIGIN, setpoint, TICKS_3S);
    let final_down = traces.last().expect("trace").truth.position_ned_m[2];
    let initial_error = (0.0_f32 - setpoint.position_ned_m()[2]).abs();
    let final_error = (final_down - setpoint.position_ned_m()[2]).abs();

    assert_all_traces_finite_and_normalized(&traces);
    assert!(
        final_down < 0.0,
        "expected upward setpoint to move NED down negative, got {final_down}"
    );
    assert!(
        final_error < initial_error,
        "expected final altitude error {final_error} to improve from {initial_error}"
    );
}

#[test]
fn p05_closed_loop_truth_motor_output_never_nan() {
    let initial = ClosedLoopTruthState {
        euler_rad: [
            8.0_f32.to_radians(),
            -6.0_f32.to_radians(),
            4.0_f32.to_radians(),
        ],
        body_rates_rps: [0.1, -0.1, 0.05],
        ..ClosedLoopTruthState::LEVEL_ORIGIN
    };
    let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, -0.5], 0.0, true);
    let traces = run_case(initial, setpoint, TICKS_3S);

    assert_all_traces_finite_and_normalized(&traces);
}

fn run_case(
    initial: ClosedLoopTruthState,
    setpoint: ControlSetpoint,
    ticks: usize,
) -> Vec<ClosedLoopTrace> {
    let mut runner = ClosedLoopRunner::new(
        ControlPipeline::default(),
        TruthPlant::new(Default::default(), initial),
        ClosedLoopConfig::new(DT_S, ticks),
    );
    runner.run(setpoint)
}

fn assert_all_traces_finite_and_normalized(traces: &[ClosedLoopTrace]) {
    assert!(!traces.is_empty());
    for trace in traces {
        assert!(
            trace.truth.finite(),
            "non-finite truth at tick {}",
            trace.tick
        );
        assert!(
            trace.control.control_valid,
            "invalid control at tick {}",
            trace.tick
        );
        assert!(
            trace
                .control
                .motors
                .iter()
                .all(|motor| motor.is_finite() && (0.0..=1.0).contains(motor)),
            "invalid motors at tick {}: {:?}",
            trace.tick,
            trace.control.motors
        );
        assert!(
            trace.truth.euler_rad.iter().all(|angle| angle.abs() < 0.75),
            "attitude exceeded envelope at tick {}: {:?}",
            trace.tick,
            trace.truth.euler_rad
        );
    }
}
