use common::control::config::ControlLoopConfig;
use common::control::mixing::MixerLimits;
use common::control::rate::RateControllerConfig;
use deterministic_harness::{
    ClosedLoopConfig, ClosedLoopRunner, ClosedLoopTrace, ClosedLoopTruthState, ControlPipeline,
    ControlSetpoint, EstimateSnapshot, ImuControlInput, MotorGeometry, PureControlConfig,
    TruthPlant,
};

const DT_S: f32 = 0.01;
const TICKS_6S: usize = 600;
const TICKS_8S: usize = 800;
const MAX_ROLL_PITCH_RAD: f32 = 0.65;
const MAX_ALTITUDE_ABS_M: f32 = 4.5;
const MAX_VERTICAL_SPEED_MPS: f32 = 3.0;
const MAX_CLAMP_RATIO: f32 = 0.02;

#[derive(Clone, Copy, Debug, PartialEq)]
struct HardeningSummary {
    max_roll_pitch_rad: f32,
    max_altitude_abs_m: f32,
    max_vertical_speed_mps: f32,
    clamp_ratio: f32,
    initial_down_error_m: f32,
    final_down_error_m: f32,
    initial_yaw_error_rad: f32,
    final_yaw_error_rad: f32,
    final_state: ClosedLoopTruthState,
}

#[test]
fn p14_g2_control_hardening_altitude_ladder_stays_bounded() {
    for target_down_m in [-0.5, -1.0, -2.0, -3.0] {
        let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, target_down_m], 0.0, true);
        let traces = run_case(ClosedLoopTruthState::LEVEL_ORIGIN, setpoint, TICKS_8S);
        let summary = summarize(&traces, setpoint);

        assert_hardening_envelope(summary);
        assert!(
            summary.final_down_error_m.abs() < summary.initial_down_error_m.abs(),
            "altitude case did not improve: target_down_m={target_down_m} summary={summary:?}"
        );
        assert!(
            summary.final_state.position_ned_m[0].abs() < 0.001
                && summary.final_state.position_ned_m[1].abs() < 0.001,
            "altitude ladder should not create horizontal drift: {summary:?}"
        );
    }
}

#[test]
fn p14_g2_control_hardening_yawed_hover_setpoints_stay_level() {
    for yaw_rad in [
        45.0_f32.to_radians(),
        90.0_f32.to_radians(),
        180.0_f32.to_radians(),
    ] {
        let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, 0.0], yaw_rad, true);
        let traces = run_case(ClosedLoopTruthState::LEVEL_ORIGIN, setpoint, TICKS_8S);
        let summary = summarize(&traces, setpoint);

        assert_hardening_envelope(summary);
        assert!(
            summary.final_yaw_error_rad.abs() < summary.initial_yaw_error_rad.abs(),
            "yaw case did not reduce heading error: yaw_rad={yaw_rad} summary={summary:?}"
        );
        assert!(
            summary.final_state.euler_rad[0].abs() < 1.0_f32.to_radians()
                && summary.final_state.euler_rad[1].abs() < 1.0_f32.to_radians(),
            "yawed hover should stay level: {summary:?}"
        );
    }
}

#[test]
fn p14_g2_control_hardening_return_to_zero_yaw_recovers() {
    let initial = ClosedLoopTruthState {
        euler_rad: [0.0, 0.0, 90.0_f32.to_radians()],
        ..ClosedLoopTruthState::LEVEL_ORIGIN
    };
    let setpoint = ControlSetpoint::ORIGIN_HOLD_ARMED;
    let traces = run_case(initial, setpoint, TICKS_8S);
    let summary = summarize(&traces, setpoint);

    assert_hardening_envelope(summary);
    assert!(
        summary.final_yaw_error_rad.abs() < summary.initial_yaw_error_rad.abs(),
        "return-to-zero yaw did not recover: {summary:?}"
    );
}

#[test]
fn p14_g2_control_hardening_attitude_and_rate_disturbances_recover() {
    let attitude_cases = [
        [10.0_f32.to_radians(), 0.0, 0.0],
        [20.0_f32.to_radians(), 0.0, 0.0],
        [30.0_f32.to_radians(), 0.0, 0.0],
        [0.0, 10.0_f32.to_radians(), 0.0],
        [0.0, 20.0_f32.to_radians(), 0.0],
        [0.0, 30.0_f32.to_radians(), 0.0],
    ];

    for euler_rad in attitude_cases {
        let initial = ClosedLoopTruthState {
            euler_rad,
            ..ClosedLoopTruthState::LEVEL_ORIGIN
        };
        let traces = run_case(initial, ControlSetpoint::ORIGIN_HOLD_ARMED, TICKS_6S);
        let summary = summarize(&traces, ControlSetpoint::ORIGIN_HOLD_ARMED);
        let initial_tilt = euler_rad[0].abs().max(euler_rad[1].abs());
        let final_tilt = summary.final_state.euler_rad[0]
            .abs()
            .max(summary.final_state.euler_rad[1].abs());

        assert_hardening_envelope(summary);
        assert!(
            final_tilt < initial_tilt,
            "attitude disturbance did not recover toward level: initial={euler_rad:?} summary={summary:?}"
        );
    }

    for yaw_rate_rps in [0.2, -0.2, 0.4, -0.4] {
        let initial = ClosedLoopTruthState {
            body_rates_rps: [0.0, 0.0, yaw_rate_rps],
            ..ClosedLoopTruthState::LEVEL_ORIGIN
        };
        let traces = run_case(initial, ControlSetpoint::ORIGIN_HOLD_ARMED, TICKS_6S);
        let summary = summarize(&traces, ControlSetpoint::ORIGIN_HOLD_ARMED);

        assert_hardening_envelope(summary);
        assert!(
            summary.final_state.body_rates_rps[2].abs() < yaw_rate_rps.abs(),
            "yaw-rate impulse did not damp: yaw_rate_rps={yaw_rate_rps} summary={summary:?}"
        );
    }
}

#[test]
fn p14_g2_control_hardening_parameter_margins_remain_finite() {
    for (name, geometry) in [
        ("heavy_frame", scaled_geometry(1.15, 1.0)),
        ("light_frame", scaled_geometry(0.85, 1.0)),
        ("battery_derated", scaled_geometry(1.0, 0.90)),
        ("strong_motors", scaled_geometry(1.0, 1.10)),
    ] {
        let setpoint = ControlSetpoint::local_position_ned([0.0, 0.0, -1.0], 0.0, true);
        let traces = run_case_with_geometry(
            ClosedLoopTruthState::LEVEL_ORIGIN,
            setpoint,
            TICKS_6S,
            geometry,
        );
        let summary = summarize(&traces, setpoint);

        assert_hardening_envelope(summary);
        assert!(
            summary.final_down_error_m.abs() < 2.0,
            "parameter margin did not stay close enough: name={name} summary={summary:?}"
        );
    }
}

#[test]
fn p14_g2_control_hardening_reports_yaw_mixer_saturation() {
    let trace = saturated_pipeline_step([0.0, 0.0, -1.0]);

    assert!(trace.control_valid);
    assert!(trace.clamped);
    assert!(trace.debug.mixer_limit_flags.yaw);
    assert!(!trace.debug.mixer_limit_flags.roll_pitch);
    assert!(trace.debug.mixer_limit_flags.throttle_lower);
    assert!(trace.debug.mixer_limit_flags.throttle_upper);
}

#[test]
fn p14_g2_control_hardening_reports_roll_pitch_mixer_saturation() {
    let trace = saturated_pipeline_step([-1.0, -1.0, 0.0]);

    assert!(trace.control_valid);
    assert!(trace.clamped);
    assert!(trace.debug.mixer_limit_flags.roll_pitch);
    assert!(!trace.debug.mixer_limit_flags.yaw);
    assert!(trace.debug.mixer_limit_flags.throttle_lower);
    assert!(trace.debug.mixer_limit_flags.throttle_upper);
}

fn run_case(
    initial: ClosedLoopTruthState,
    setpoint: ControlSetpoint,
    ticks: usize,
) -> Vec<ClosedLoopTrace> {
    run_case_with_geometry(initial, setpoint, ticks, MotorGeometry::default())
}

fn run_case_with_geometry(
    initial: ClosedLoopTruthState,
    setpoint: ControlSetpoint,
    ticks: usize,
    geometry: MotorGeometry,
) -> Vec<ClosedLoopTrace> {
    let mut runner = ClosedLoopRunner::new(
        ControlPipeline::default(),
        TruthPlant::new(geometry, initial),
        ClosedLoopConfig::new(DT_S, ticks),
    );
    runner.run(setpoint)
}

fn saturated_pipeline_step(gyro_rps: [f32; 3]) -> deterministic_harness::ControlPipelineTrace {
    let mut config = ControlLoopConfig::default();
    config.rate = RateControllerConfig {
        roll_gain: 1.0,
        pitch_gain: 1.0,
        yaw_gain: 1.0,
        max_axis_command: 0.4,
        measured_rate_deadband_rps: 0.0,
    };
    config.mixer_limits = MixerLimits::new(0.45, 0.55);
    let pipeline = ControlPipeline::new(PureControlConfig {
        loop_config: config,
    });

    pipeline.step(
        EstimateSnapshot::LEVEL_ORIGIN,
        ImuControlInput {
            accel_mps2: [0.0, 0.0, 0.0],
            gyro_rps,
        },
        ControlSetpoint::ORIGIN_HOLD_ARMED,
    )
}

fn scaled_geometry(mass_scale: f32, thrust_scale: f32) -> MotorGeometry {
    let mut geometry = MotorGeometry::default();
    geometry.mass_kg *= mass_scale;
    for motor in geometry.motors.iter_mut() {
        motor.thrust_coefficient_n *= thrust_scale;
    }
    geometry
}

fn summarize(traces: &[ClosedLoopTrace], setpoint: ControlSetpoint) -> HardeningSummary {
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
    }

    let max_roll_pitch_rad = traces
        .iter()
        .map(|trace| {
            trace.truth.euler_rad[0]
                .abs()
                .max(trace.truth.euler_rad[1].abs())
        })
        .fold(0.0, f32::max);
    let max_altitude_abs_m = traces
        .iter()
        .map(|trace| trace.truth.position_ned_m[2].abs())
        .fold(0.0, f32::max);
    let max_vertical_speed_mps = traces
        .iter()
        .map(|trace| trace.truth.velocity_ned_mps[2].abs())
        .fold(0.0, f32::max);
    let clamped = traces.iter().filter(|trace| trace.control.clamped).count();
    let first = traces.first().unwrap();
    let last = traces.last().unwrap();

    HardeningSummary {
        max_roll_pitch_rad,
        max_altitude_abs_m,
        max_vertical_speed_mps,
        clamp_ratio: clamped as f32 / traces.len() as f32,
        initial_down_error_m: first.truth.position_ned_m[2] - setpoint.position_ned_m()[2],
        final_down_error_m: last.truth.position_ned_m[2] - setpoint.position_ned_m()[2],
        initial_yaw_error_rad: yaw_error_rad(setpoint.yaw_rad(), first.truth.euler_rad[2]),
        final_yaw_error_rad: yaw_error_rad(setpoint.yaw_rad(), last.truth.euler_rad[2]),
        final_state: last.truth,
    }
}

fn assert_hardening_envelope(summary: HardeningSummary) {
    assert!(
        summary.max_roll_pitch_rad <= MAX_ROLL_PITCH_RAD,
        "roll/pitch envelope failed: {summary:?}"
    );
    assert!(
        summary.max_altitude_abs_m <= MAX_ALTITUDE_ABS_M,
        "altitude envelope failed: {summary:?}"
    );
    assert!(
        summary.max_vertical_speed_mps <= MAX_VERTICAL_SPEED_MPS,
        "vertical speed envelope failed: {summary:?}"
    );
    assert!(
        summary.clamp_ratio <= MAX_CLAMP_RATIO,
        "clamp ratio failed: {summary:?}"
    );
}

fn yaw_error_rad(target: f32, actual: f32) -> f32 {
    let mut error = target - actual;
    while error > core::f32::consts::PI {
        error -= 2.0 * core::f32::consts::PI;
    }
    while error < -core::f32::consts::PI {
        error += 2.0 * core::f32::consts::PI;
    }
    error
}
