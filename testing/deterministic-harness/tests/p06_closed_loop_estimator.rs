use deterministic_harness::{
    ClosedLoopConfig, ClosedLoopEstimatorConfig, ClosedLoopEstimatorReport,
    ClosedLoopEstimatorRunner, ClosedLoopEstimatorTrace, ClosedLoopTruthState, ControlPipeline,
    ControlSetpoint, EstimatorReplayConfig, EstimatorResetInjection, SimulatedSensorConfig,
    TruthPlant,
};

const DT_S: f32 = 0.01;
const TICKS_3S: usize = 300;
const TICKS_5S: usize = 500;

#[test]
fn p06_closed_loop_estimator_static_hold_estimator_in_loop() {
    let report = run_case(
        ClosedLoopTruthState::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, TICKS_3S)),
    );

    assert_all_traces_finite_and_bounded(&report);
    assert!(report.trace_csv().starts_with("tick,sim_time_us,truth_n"));
    assert!(
        !report
            .traces
            .first()
            .expect("first trace")
            .estimator
            .estimate_valid
    );
    assert!(
        report
            .traces
            .iter()
            .skip(1)
            .all(|trace| trace.estimator.estimate_valid)
    );
    assert!(
        report
            .traces
            .iter()
            .skip(1)
            .all(|trace| trace.control.control_valid)
    );

    let final_trace = report.last_trace().expect("last trace");
    assert_abs_lt(final_trace.truth.position_ned_m[2] as f64, 0.25);
    assert_abs_lt(final_trace.estimator.position_ned_m[2], 0.25);

    let final_motor_average = motor_average(final_trace);
    assert!(
        (0.35..=0.65).contains(&final_motor_average),
        "expected near-hover final motor average, got {final_motor_average}"
    );
}

#[test]
fn p06_closed_loop_estimator_vertical_setpoint_estimator_in_loop() {
    let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true);
    let report = run_case(
        ClosedLoopTruthState::LEVEL_ORIGIN,
        setpoint,
        ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, TICKS_5S)),
    );
    let final_trace = report.last_trace().expect("last trace");
    let initial_error = (0.0_f32 - setpoint.position_ned_m()[2]).abs();
    let final_truth_error =
        (final_trace.truth.position_ned_m[2] - setpoint.position_ned_m()[2]).abs();
    let final_estimate_error =
        (final_trace.estimator.position_ned_m[2] as f32 - setpoint.position_ned_m()[2]).abs();

    assert_all_traces_finite_and_bounded(&report);
    assert!(
        final_trace.truth.position_ned_m[2] < 0.0,
        "expected truth altitude to move upward/NED-negative, got {}",
        final_trace.truth.position_ned_m[2]
    );
    assert!(
        final_trace.estimator.position_ned_m[2] < 0.0,
        "expected estimated altitude to move upward/NED-negative, got {}",
        final_trace.estimator.position_ned_m[2]
    );
    assert!(
        final_truth_error < initial_error,
        "expected truth error {final_truth_error} to improve from {initial_error}"
    );
    assert!(
        final_estimate_error < initial_error,
        "expected estimate error {final_estimate_error} to improve from {initial_error}"
    );
    assert!(
        final_trace.truth.position_ned_m[2] > -1.75,
        "truth altitude overshot bounded envelope: {}",
        final_trace.truth.position_ned_m[2]
    );
}

#[test]
fn p06_closed_loop_estimator_gps_dropout_does_not_explode() {
    let config = ClosedLoopEstimatorConfig {
        sensors: SimulatedSensorConfig::new().with_gps_dropout(100, 250),
        ..ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, TICKS_5S))
    };
    let report = run_case(
        ClosedLoopTruthState::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        config,
    );

    assert_all_traces_finite_and_bounded(&report);
    assert_eq!(report.gps_updates, (TICKS_5S - 1 - 151) as u32);
    assert!(
        report
            .traces
            .iter()
            .filter(|trace| (100..=250).contains(&trace.tick))
            .all(|trace| !trace.estimator.gps_used),
        "GPS updates must be absent during configured dropout"
    );
    assert!(
        report
            .traces
            .iter()
            .filter(|trace| trace.tick > 250)
            .any(|trace| trace.estimator.gps_used),
        "GPS should reacquire after the dropout window"
    );
    let final_trace = report.last_trace().expect("last trace");
    assert_abs_lt(final_trace.estimator.position_ned_m[2], 0.5);
    assert_abs_lt(final_trace.truth.position_ned_m[2] as f64, 0.5);
}

#[test]
fn p06_closed_loop_estimator_baro_spike_rejected_or_bounded() {
    let spike_tick = 120;
    let config = ClosedLoopEstimatorConfig {
        estimator: EstimatorReplayConfig {
            max_baro_step_m: Some(1.0),
            ..EstimatorReplayConfig::default()
        },
        sensors: SimulatedSensorConfig::new().with_baro_spike(spike_tick, 25.0),
        ..ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, TICKS_3S))
    };
    let report = run_case(
        ClosedLoopTruthState::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        config,
    );

    assert_all_traces_finite_and_bounded(&report);
    assert_eq!(report.rejected_baro_spikes, 1);
    let spike = trace_at(&report, spike_tick);
    assert!(spike.estimator.baro_rejected);

    let before = trace_at(&report, spike_tick - 1);
    let after = trace_at(&report, spike_tick + 1);
    assert!(
        (motor_average(spike) - motor_average(before)).abs() < 0.15,
        "unsafe motor jump at spike tick: before {:?}, spike {:?}",
        before.control.motors,
        spike.control.motors
    );
    assert!(
        (motor_average(after) - motor_average(spike)).abs() < 0.15,
        "unsafe motor jump after spike: spike {:?}, after {:?}",
        spike.control.motors,
        after.control.motors
    );
}

#[test]
fn p06_estimator_xy_reset_while_holding_does_not_command_jump() {
    let reset_tick = 160;
    let config = ClosedLoopEstimatorConfig {
        sensors: SimulatedSensorConfig::new().with_estimator_reset(
            reset_tick,
            EstimatorResetInjection {
                position_delta_ned_m: [0.75, -0.5, 0.0],
                ..EstimatorResetInjection::NONE
            },
        ),
        ..ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, TICKS_3S))
    };
    let report = run_case(
        ClosedLoopTruthState::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        config,
    );

    assert_all_traces_finite_and_bounded(&report);
    assert_reset_tick_is_absorbed(&report, reset_tick);
    let reset = trace_at(&report, reset_tick);
    assert_eq!(reset.estimator.reset_delta.xy_reset_counter, 1);
}

#[test]
fn p06_estimator_z_reset_during_takeoff_does_not_command_throttle_jump() {
    let reset_tick = 180;
    let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true);
    let config = ClosedLoopEstimatorConfig {
        sensors: SimulatedSensorConfig::new().with_estimator_reset(
            reset_tick,
            EstimatorResetInjection {
                position_delta_ned_m: [0.0, 0.0, -0.35],
                ..EstimatorResetInjection::NONE
            },
        ),
        ..ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, TICKS_5S))
    };
    let report = run_case(ClosedLoopTruthState::LEVEL_ORIGIN, setpoint, config);

    assert_all_traces_finite_and_bounded(&report);
    assert_reset_tick_is_absorbed(&report, reset_tick);
    let reset = trace_at(&report, reset_tick);
    assert_eq!(reset.estimator.reset_delta.z_reset_counter, 1);
}

#[test]
fn p06_estimator_velocity_reset_while_holding_does_not_command_jump() {
    let reset_tick = 170;
    let config = ClosedLoopEstimatorConfig {
        sensors: SimulatedSensorConfig::new().with_estimator_reset(
            reset_tick,
            EstimatorResetInjection {
                velocity_delta_ned_mps: [0.2, -0.15, 0.1],
                ..EstimatorResetInjection::NONE
            },
        ),
        ..ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, TICKS_3S))
    };
    let report = run_case(
        ClosedLoopTruthState::LEVEL_ORIGIN,
        ControlSetpoint::ORIGIN_HOLD_ARMED,
        config,
    );

    assert_all_traces_finite_and_bounded(&report);
    assert_reset_tick_is_absorbed(&report, reset_tick);
    let reset = trace_at(&report, reset_tick);
    assert_eq!(reset.estimator.reset_delta.velocity_reset_counter, 1);
}

#[test]
fn p06_estimator_heading_reset_during_yawed_hold_does_not_command_yaw_jump() {
    let reset_tick = 180;
    let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, -0.5], 0.6, true);
    let config = ClosedLoopEstimatorConfig {
        sensors: SimulatedSensorConfig::new().with_estimator_reset(
            reset_tick,
            EstimatorResetInjection {
                heading_delta_rad: 0.35,
                ..EstimatorResetInjection::NONE
            },
        ),
        ..ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, TICKS_5S))
    };
    let report = run_case(ClosedLoopTruthState::LEVEL_ORIGIN, setpoint, config);

    assert_all_traces_finite_and_bounded(&report);
    assert_reset_tick_is_absorbed(&report, reset_tick);
    let reset = trace_at(&report, reset_tick);
    assert_eq!(reset.estimator.reset_delta.heading_reset_counter, 1);
}

fn run_case(
    initial: ClosedLoopTruthState,
    setpoint: ControlSetpoint,
    config: ClosedLoopEstimatorConfig,
) -> ClosedLoopEstimatorReport {
    let mut runner = ClosedLoopEstimatorRunner::new(
        ControlPipeline::default(),
        TruthPlant::new(Default::default(), initial),
        config,
    );
    runner
        .run(setpoint)
        .expect("closed-loop estimator run should pass")
}

fn assert_all_traces_finite_and_bounded(report: &ClosedLoopEstimatorReport) {
    assert!(!report.traces.is_empty());
    for trace in &report.traces {
        assert!(
            trace.truth.finite(),
            "non-finite truth at tick {}",
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
                .estimator
                .quaternion
                .iter()
                .all(|value| value.is_finite())
                && trace
                    .estimator
                    .position_ned_m
                    .iter()
                    .all(|value| value.is_finite())
                && trace
                    .estimator
                    .velocity_ned_mps
                    .iter()
                    .all(|value| value.is_finite()),
            "non-finite estimator state at tick {}",
            trace.tick
        );
        assert!(
            trace.truth.euler_rad.iter().all(|angle| angle.abs() < 0.75),
            "attitude exceeded envelope at tick {}: {:?}",
            trace.tick,
            trace.truth.euler_rad
        );
        assert!(
            trace.truth.position_ned_m[2].abs() < 3.0,
            "truth altitude exceeded envelope at tick {}: {}",
            trace.tick,
            trace.truth.position_ned_m[2]
        );
        assert!(
            trace.estimator.position_ned_m[2].abs() < 3.0,
            "estimate altitude exceeded envelope at tick {}: {}",
            trace.tick,
            trace.estimator.position_ned_m[2]
        );
    }
}

fn trace_at(report: &ClosedLoopEstimatorReport, tick: u64) -> &ClosedLoopEstimatorTrace {
    report
        .traces
        .iter()
        .find(|trace| trace.tick == tick)
        .expect("trace tick should exist")
}

fn motor_average(trace: &ClosedLoopEstimatorTrace) -> f32 {
    trace.control.motors.iter().sum::<f32>() / trace.control.motors.len() as f32
}

fn assert_reset_tick_is_absorbed(report: &ClosedLoopEstimatorReport, reset_tick: u64) {
    let before = trace_at(report, reset_tick - 1);
    let reset = trace_at(report, reset_tick);
    let after = trace_at(report, reset_tick + 1);

    assert!(
        reset.estimator.reset_delta.has_reset(),
        "expected reset delta at tick {reset_tick}"
    );
    assert_motor_jump_lt(before, reset, 0.18);
    assert_motor_jump_lt(reset, after, 0.18);
    assert!(
        reset.control.control_valid,
        "control should remain valid at reset tick"
    );
}

fn assert_motor_jump_lt(
    before: &ClosedLoopEstimatorTrace,
    after: &ClosedLoopEstimatorTrace,
    limit: f32,
) {
    for (index, (a, b)) in before
        .control
        .motors
        .iter()
        .zip(after.control.motors)
        .enumerate()
    {
        assert!(
            (a - b).abs() < limit,
            "motor {} jump too large between ticks {} and {}: before={:?} after={:?}",
            index + 1,
            before.tick,
            after.tick,
            before.control.motors,
            after.control.motors
        );
    }
}

fn assert_abs_lt(value: f64, limit: f64) {
    assert!(
        value.abs() < limit,
        "expected |{value}| to be less than {limit}"
    );
}
