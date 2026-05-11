use common::localization::estimation::{LayeredNavigationStack, Vec3};

#[test]
fn static_replay_first_sample_waits_for_dt_then_remains_bounded() {
    let mut stack = LayeredNavigationStack::<f64>::default();
    let samples = [
        ReplaySample::static_at(0.0, 0.0),
        ReplaySample::static_at(0.01, 0.0),
        ReplaySample::static_at(0.02, 0.0),
        ReplaySample::static_at(0.03, 0.0),
    ];

    let mut previous_timestamp_s = None;
    let mut valid_updates = 0;
    for sample in samples {
        if let Some(previous) = previous_timestamp_s {
            let dt = sample.timestamp_s - previous;
            predict_and_update(&mut stack, sample, dt);
            valid_updates += 1;
        }
        previous_timestamp_s = Some(sample.timestamp_s);
    }

    assert_eq!(valid_updates, 3);
    let state = stack.state();
    assert_quaternion_normalized(state.quaternion.as_slice());
    assert_vec3_abs_lt(state.position_m.as_slice(), 0.001);
    assert_vec3_abs_lt(state.velocity_mps.as_slice(), 0.001);
}

#[test]
fn vertical_ascent_replay_moves_ned_down_negative() {
    let mut stack = LayeredNavigationStack::<f64>::default();
    let samples = [
        ReplaySample::static_at(0.0, 0.0),
        ReplaySample::static_at(0.01, -0.2),
        ReplaySample::static_at(0.02, -0.4),
        ReplaySample::static_at(0.03, -0.6),
    ];

    let mut previous_timestamp_s = None;
    for sample in samples {
        if let Some(previous) = previous_timestamp_s {
            let dt = sample.timestamp_s - previous;
            predict_and_update(&mut stack, sample, dt);
        }
        previous_timestamp_s = Some(sample.timestamp_s);
    }

    let state = stack.state();
    assert!(
        state.position_m[2] < 0.0,
        "expected ascent replay to move NED down negative, got {}",
        state.position_m[2]
    );
    assert!(state.velocity_mps.iter().all(|value| value.is_finite()));
}

#[derive(Clone, Copy)]
struct ReplaySample {
    timestamp_s: f64,
    baro_down_m: f64,
}

impl ReplaySample {
    const fn static_at(timestamp_s: f64, baro_down_m: f64) -> Self {
        Self {
            timestamp_s,
            baro_down_m,
        }
    }
}

fn predict_and_update(stack: &mut LayeredNavigationStack<f64>, sample: ReplaySample, dt: f64) {
    stack
        .predict(
            Vec3::<f64>::zeros(),
            Vec3::<f64>::zeros(),
            dt,
            Some(sample.timestamp_s),
        )
        .expect("predict should pass");
    stack
        .update_gravity_alignment(Vec3::<f64>::new(0.0, 0.0, -9.806_65), None)
        .expect("gravity update should pass");
    stack
        .update_magnetic_field(
            Vec3::<f64>::new(20.0, 0.0, 40.0),
            Vec3::<f64>::new(20.0, 0.0, 40.0),
            None,
        )
        .expect("mag update should pass");
    stack
        .update_position(Vec3::<f64>::new(0.0, 0.0, sample.baro_down_m), None)
        .expect("position update should pass");
    stack
        .update_velocity(Vec3::<f64>::zeros(), None)
        .expect("velocity update should pass");
    stack
        .update_barometric_altitude(sample.baro_down_m, None)
        .expect("baro update should pass");
}

fn assert_quaternion_normalized(quaternion: &[f64]) {
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

fn assert_vec3_abs_lt(values: &[f64], limit: f64) {
    for value in values {
        assert!(value.abs() < limit, "expected |{value}| < {limit}");
    }
}
