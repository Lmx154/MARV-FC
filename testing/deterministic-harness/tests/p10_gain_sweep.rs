use common::control::config::ControlLoopConfig;
use deterministic_harness::{
    ClosedLoopConfig, ClosedLoopRunner, ClosedLoopTrace, ClosedLoopTruthState, ControlPipeline,
    ControlSetpoint, PureControlConfig, TruthPlant,
};

const DT_S: f32 = 0.01;
const TICKS_5S: usize = 500;
const MAX_ATTITUDE_RAD: f32 = 0.75;
const MAX_ALTITUDE_ERROR_M: f32 = 1.5;
const MAX_CLAMP_RATIO: f32 = 0.05;

#[derive(Clone, Copy, Debug, PartialEq)]
struct SweepSummary {
    scale: f32,
    max_attitude_rad: f32,
    max_altitude_error_m: f32,
    clamp_ratio: f32,
    final_error_m: f32,
}

#[test]
fn p10_gain_sweep_static_hold_stays_boring_near_current_gains() {
    for scale in [0.75, 1.0, 1.25] {
        let traces = run_scaled_case(
            scale,
            ClosedLoopTruthState::LEVEL_ORIGIN,
            ControlSetpoint::ORIGIN_HOLD_ARMED,
        );
        let summary = summarize(scale, &traces, ControlSetpoint::ORIGIN_HOLD_ARMED);

        assert_summary_passes(summary);
        assert!(
            summary.final_error_m < 0.05,
            "static hold drifted unexpectedly: {summary:?}"
        );
    }
}

#[test]
fn p10_gain_sweep_vertical_step_improves_without_saturation() {
    let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true);

    for scale in [0.75, 1.0, 1.25] {
        let traces = run_scaled_case(scale, ClosedLoopTruthState::LEVEL_ORIGIN, setpoint);
        let summary = summarize(scale, &traces, setpoint);

        assert_summary_passes(summary);
        assert!(
            summary.final_error_m < 1.0,
            "vertical step did not improve from initial error: {summary:?}"
        );
    }
}

#[test]
fn p10_gain_sweep_roll_disturbance_improves_for_nearby_gains() {
    let initial_roll = 10.0_f32.to_radians();
    let initial = ClosedLoopTruthState {
        euler_rad: [initial_roll, 0.0, 0.0],
        ..ClosedLoopTruthState::LEVEL_ORIGIN
    };

    for scale in [0.75, 1.0, 1.25] {
        let traces = run_scaled_case(scale, initial, ControlSetpoint::ORIGIN_HOLD_ARMED);
        let summary = summarize(scale, &traces, ControlSetpoint::ORIGIN_HOLD_ARMED);
        let final_roll = traces.last().unwrap().truth.euler_rad[0].abs();

        assert_summary_passes(summary);
        assert!(
            final_roll < initial_roll.abs(),
            "roll disturbance did not improve: scale={scale} final_roll={final_roll}"
        );
    }
}

fn run_scaled_case(
    scale: f32,
    initial: ClosedLoopTruthState,
    setpoint: ControlSetpoint,
) -> Vec<ClosedLoopTrace> {
    let mut runner = ClosedLoopRunner::new(
        ControlPipeline::new(PureControlConfig::with_loop_config(scaled_config(scale))),
        TruthPlant::new(Default::default(), initial),
        ClosedLoopConfig::new(DT_S, TICKS_5S),
    );
    runner.run(setpoint)
}

fn scaled_config(scale: f32) -> ControlLoopConfig {
    assert!(scale.is_finite() && scale > 0.0);
    let mut config = ControlLoopConfig::default();

    config.position.position_gain *= scale;
    config.position.velocity_gain *= scale;
    config.altitude.altitude_gain *= scale;
    config.altitude.vertical_velocity_gain *= scale;
    config.attitude.roll_rate_gain *= scale;
    config.attitude.pitch_rate_gain *= scale;
    config.attitude.yaw_rate_gain *= scale;
    config.rate.roll_gain *= scale;
    config.rate.pitch_gain *= scale;
    config.rate.yaw_gain *= scale;

    config
}

fn summarize(scale: f32, traces: &[ClosedLoopTrace], setpoint: ControlSetpoint) -> SweepSummary {
    assert!(!traces.is_empty());
    let max_attitude_rad = traces
        .iter()
        .flat_map(|trace| trace.truth.euler_rad)
        .map(f32::abs)
        .fold(0.0, f32::max);
    let max_altitude_error_m = traces
        .iter()
        .map(|trace| (trace.truth.position_ned_m[2] - setpoint.position_ned_m()[2]).abs())
        .fold(0.0, f32::max);
    let clamped = traces.iter().filter(|trace| trace.control.clamped).count();
    let final_error_m =
        (traces.last().unwrap().truth.position_ned_m[2] - setpoint.position_ned_m()[2]).abs();

    SweepSummary {
        scale,
        max_attitude_rad,
        max_altitude_error_m,
        clamp_ratio: clamped as f32 / traces.len() as f32,
        final_error_m,
    }
}

fn assert_summary_passes(summary: SweepSummary) {
    assert!(
        summary.max_attitude_rad < MAX_ATTITUDE_RAD,
        "attitude envelope failed: {summary:?}"
    );
    assert!(
        summary.max_altitude_error_m < MAX_ALTITUDE_ERROR_M,
        "altitude envelope failed: {summary:?}"
    );
    assert!(
        summary.clamp_ratio <= MAX_CLAMP_RATIO,
        "clamp ratio failed: {summary:?}"
    );
}
