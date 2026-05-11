use deterministic_harness::{EstimatorReplayConfig, Fixture, SensorFrame, replay_estimator_frames};

#[test]
fn p02_estimator_replay_static_on_pad_remains_bounded() {
    let report = replay_fixture(
        "static_on_pad_10s.csv",
        include_str!("../fixtures/static_on_pad_10s.csv"),
        EstimatorReplayConfig::default(),
    );

    let first = report.traces.first().expect("first trace");
    assert!(!first.estimate_valid);
    assert_eq!(first.dt_s, None);

    let last = report.last_trace().expect("last trace");
    assert!(last.estimate_valid);
    assert_quaternion_normalized(last.quaternion);
    assert_abs_lt(last.position_ned_m[0], 0.001);
    assert_abs_lt(last.position_ned_m[1], 0.001);
    assert_abs_lt(last.position_ned_m[2], 0.001);
    assert_abs_lt(last.velocity_ned_mps[0], 0.001);
    assert_abs_lt(last.velocity_ned_mps[1], 0.001);
    assert_abs_lt(last.velocity_ned_mps[2], 0.001);
}

#[test]
fn p02_estimator_replay_vertical_ascent_moves_down_position_negative() {
    let report = replay_fixture(
        "vertical_ascent_5s.csv",
        include_str!("../fixtures/vertical_ascent_5s.csv"),
        EstimatorReplayConfig::default(),
    );

    let last = report.last_trace().expect("last trace");
    assert!(last.estimate_valid);
    assert!(
        last.position_ned_m[2] < 0.0,
        "expected upward/ascent fixture to move NED down position negative, got {}",
        last.position_ned_m[2]
    );
    assert!(last.velocity_ned_mps.iter().all(|value| value.is_finite()));
}

#[test]
fn p02_estimator_replay_gps_dropout_stays_valid_without_stale_gps_updates() {
    let report = replay_fixture(
        "gps_dropout_5s.csv",
        include_str!("../fixtures/gps_dropout_5s.csv"),
        EstimatorReplayConfig::default(),
    );

    assert_eq!(report.gps_updates, 2);
    assert_eq!(
        report.traces.iter().filter(|trace| trace.gps_used).count(),
        2
    );
    for trace in report.traces.iter().skip(1) {
        assert!(trace.estimate_valid);
        assert_quaternion_normalized(trace.quaternion);
        assert!(trace.position_ned_m.iter().all(|value| value.is_finite()));
        assert!(trace.velocity_ned_mps.iter().all(|value| value.is_finite()));
    }
}

#[test]
fn p02_estimator_replay_baro_spike_is_rejected_and_state_stays_bounded() {
    let report = replay_fixture(
        "baro_spike_5s.csv",
        include_str!("../fixtures/baro_spike_5s.csv"),
        EstimatorReplayConfig {
            max_baro_step_m: Some(1.0),
            ..EstimatorReplayConfig::default()
        },
    );

    assert_eq!(report.rejected_baro_spikes, 1);
    assert!(report.traces.iter().any(|trace| trace.baro_rejected));
    let last = report.last_trace().expect("last trace");
    assert!(last.position_ned_m[2].abs() < 1.0);
    assert!(last.velocity_ned_mps.iter().all(|value| value.is_finite()));
}

#[test]
fn p02_estimator_replay_imu_nan_fixture_is_rejected_before_estimator() {
    let error = Fixture::from_csv_str(
        "imu_nan_rejection.csv",
        include_str!("../fixtures/imu_nan_rejection.csv"),
    )
    .expect_err("fixture loader should reject asserted non-finite IMU fields");

    assert!(error.message.contains("non-finite"));
}

fn replay_fixture(
    name: &str,
    csv: &str,
    config: EstimatorReplayConfig,
) -> deterministic_harness::EstimatorReplayReport {
    let fixture = Fixture::from_csv_str(name, csv).expect("fixture should parse");
    let frames = SensorFrame::from_fixture(&fixture).expect("sensor frames should parse");
    replay_estimator_frames(&frames, config).expect("estimator replay should pass")
}

fn assert_quaternion_normalized(quaternion: [f64; 4]) {
    let norm = quaternion
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    assert!(
        (norm - 1.0).abs() < 1.0e-9,
        "expected normalized quaternion, got norm {norm}"
    );
}

fn assert_abs_lt(value: f64, limit: f64) {
    assert!(
        value.abs() < limit,
        "expected |{value}| to be less than {limit}"
    );
}
