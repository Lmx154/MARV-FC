use deterministic_harness::{
    ClosedLoopConfig, ClosedLoopEstimatorConfig, ClosedLoopEstimatorReport,
    ClosedLoopEstimatorRunner, ClosedLoopRunner, ClosedLoopTrace, ClosedLoopTruthState,
    ControlPipeline, ControlSetpoint, TruthPlant,
};

const DT_S: f32 = 0.01;
const TICKS_5S: usize = 500;
const TICKS_8S: usize = 800;
const MAX_ATTITUDE_RAD: f32 = 0.75;
const MAX_ALTITUDE_ABS_M: f32 = 3.0;

#[test]
fn p10_control_envelope_static_level_hold_8s_truth_and_estimator() {
    let truth_traces = run_truth_case(
        ClosedLoopTruthState::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        TICKS_8S,
    );
    let estimator_report = run_estimator_case(
        ClosedLoopTruthState::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        TICKS_5S,
    );

    assert_truth_envelope(&truth_traces);
    assert_estimator_envelope(&estimator_report);
    assert_clamp_ratio_truth(&truth_traces, 0.0);
    assert_clamp_ratio_estimator(&estimator_report, 0.0);
    assert!(truth_traces.last().unwrap().truth.position_ned_m[2].abs() < 0.05);
    assert!(estimator_report.last_trace().unwrap().truth.position_ned_m[2].abs() < 0.25);
}

#[test]
fn p10_control_envelope_roll_disturbance_recovers_over_5s() {
    let initial_roll = 10.0_f32.to_radians();
    let initial = ClosedLoopTruthState {
        euler_rad: [initial_roll, 0.0, 0.0],
        ..ClosedLoopTruthState::LEVEL_ORIGIN
    };
    let traces = run_truth_case(initial, ControlSetpoint::ORIGIN_HOLD_ARMED, TICKS_5S);
    let first = traces.first().unwrap();
    let final_roll = traces.last().unwrap().truth.euler_rad[0];

    assert_truth_envelope(&traces);
    assert!(first.control.torque_command.roll < 0.0);
    assert!(
        final_roll.abs() < 2.0_f32.to_radians(),
        "roll did not settle enough: final={final_roll}"
    );
}

#[test]
fn p10_control_envelope_pitch_disturbance_recovers_over_5s() {
    let initial_pitch = 10.0_f32.to_radians();
    let initial = ClosedLoopTruthState {
        euler_rad: [0.0, initial_pitch, 0.0],
        ..ClosedLoopTruthState::LEVEL_ORIGIN
    };
    let traces = run_truth_case(initial, ControlSetpoint::ORIGIN_HOLD_ARMED, TICKS_5S);
    let first = traces.first().unwrap();
    let final_pitch = traces.last().unwrap().truth.euler_rad[1];

    assert_truth_envelope(&traces);
    assert!(first.control.torque_command.pitch < 0.0);
    assert!(
        final_pitch.abs() < 2.0_f32.to_radians(),
        "pitch did not settle enough: final={final_pitch}"
    );
}

#[test]
fn p10_control_envelope_yaw_rate_damps_over_5s() {
    let initial_yaw_rate = 0.1;
    let initial = ClosedLoopTruthState {
        body_rates_rps: [0.0, 0.0, initial_yaw_rate],
        ..ClosedLoopTruthState::LEVEL_ORIGIN
    };
    let traces = run_truth_case(initial, ControlSetpoint::ORIGIN_HOLD_ARMED, TICKS_5S);
    let first = traces.first().unwrap();
    let final_yaw_rate = traces.last().unwrap().truth.body_rates_rps[2];

    assert_truth_envelope(&traces);
    assert!(first.control.torque_command.yaw < 0.0);
    assert!(
        final_yaw_rate.abs() < initial_yaw_rate.abs(),
        "yaw rate did not decay: final={final_yaw_rate}"
    );
}

#[test]
fn p10_control_envelope_vertical_step_up_and_down_improve_error() {
    let step_up = ControlSetpoint {
        position_ned_m: [0.0, 0.0, -1.0],
        yaw_rad: 0.0,
        armed: true,
    };
    let step_down = ControlSetpoint {
        position_ned_m: [0.0, 0.0, 1.0],
        yaw_rad: 0.0,
        armed: true,
    };

    let up = run_truth_case(ClosedLoopTruthState::LEVEL_ORIGIN, step_up, TICKS_5S);
    let down = run_truth_case(ClosedLoopTruthState::LEVEL_ORIGIN, step_down, TICKS_5S);

    assert_truth_envelope(&up);
    assert_truth_envelope(&down);
    assert_vertical_step_improved(&up, step_up.position_ned_m[2], -1.75..0.0);
    assert_vertical_step_improved(&down, step_down.position_ned_m[2], 0.0..1.75);
}

fn run_truth_case(
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

fn run_estimator_case(
    initial: ClosedLoopTruthState,
    setpoint: ControlSetpoint,
    ticks: usize,
) -> ClosedLoopEstimatorReport {
    let mut runner = ClosedLoopEstimatorRunner::new(
        ControlPipeline::default(),
        TruthPlant::new(Default::default(), initial),
        ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, ticks)),
    );
    runner.run(setpoint).expect("estimator loop should pass")
}

fn assert_truth_envelope(traces: &[ClosedLoopTrace]) {
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
            trace
                .truth
                .euler_rad
                .iter()
                .all(|angle| angle.abs() < MAX_ATTITUDE_RAD),
            "attitude exceeded envelope at tick {}: {:?}",
            trace.tick,
            trace.truth.euler_rad
        );
        assert!(
            trace.truth.position_ned_m[2].abs() < MAX_ALTITUDE_ABS_M,
            "altitude exceeded envelope at tick {}: {}",
            trace.tick,
            trace.truth.position_ned_m[2]
        );
    }
}

fn assert_estimator_envelope(report: &ClosedLoopEstimatorReport) {
    assert!(!report.traces.is_empty());
    for trace in &report.traces {
        assert!(
            trace.truth.finite(),
            "non-finite truth at tick {}",
            trace.tick
        );
        assert!(
            trace
                .estimator
                .position_ned_m
                .iter()
                .all(|value| value.is_finite())
                && trace
                    .estimator
                    .velocity_ned_mps
                    .iter()
                    .all(|value| value.is_finite())
                && trace
                    .estimator
                    .quaternion
                    .iter()
                    .all(|value| value.is_finite()),
            "non-finite estimator state at tick {}",
            trace.tick
        );
        assert!(
            trace
                .truth
                .euler_rad
                .iter()
                .all(|angle| angle.abs() < MAX_ATTITUDE_RAD),
            "attitude exceeded envelope at tick {}: {:?}",
            trace.tick,
            trace.truth.euler_rad
        );
        assert!(
            trace.truth.position_ned_m[2].abs() < MAX_ALTITUDE_ABS_M,
            "truth altitude exceeded envelope at tick {}: {}",
            trace.tick,
            trace.truth.position_ned_m[2]
        );
        assert!(
            trace.estimator.position_ned_m[2].abs() < MAX_ALTITUDE_ABS_M as f64,
            "estimate altitude exceeded envelope at tick {}: {}",
            trace.tick,
            trace.estimator.position_ned_m[2]
        );
    }
}

fn assert_clamp_ratio_truth(traces: &[ClosedLoopTrace], max_ratio: f32) {
    let clamped = traces.iter().filter(|trace| trace.control.clamped).count();
    let ratio = clamped as f32 / traces.len() as f32;
    assert!(
        ratio <= max_ratio,
        "clamp ratio {ratio} exceeded {max_ratio}"
    );
}

fn assert_clamp_ratio_estimator(report: &ClosedLoopEstimatorReport, max_ratio: f32) {
    let clamped = report
        .traces
        .iter()
        .filter(|trace| trace.control.clamped)
        .count();
    let ratio = clamped as f32 / report.traces.len() as f32;
    assert!(
        ratio <= max_ratio,
        "clamp ratio {ratio} exceeded {max_ratio}"
    );
}

fn assert_vertical_step_improved(
    traces: &[ClosedLoopTrace],
    setpoint_down_m: f32,
    expected_final_range: core::ops::Range<f32>,
) {
    let initial_error = setpoint_down_m.abs();
    let final_down = traces.last().unwrap().truth.position_ned_m[2];
    let final_error = (final_down - setpoint_down_m).abs();

    assert!(
        expected_final_range.contains(&final_down),
        "final altitude {final_down} outside expected range {expected_final_range:?}"
    );
    assert!(
        final_error < initial_error,
        "final error {final_error} did not improve from {initial_error}"
    );
}
