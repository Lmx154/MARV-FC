use common::{protocol::hilink::response_flags, services::hil::HilSensorFrameRejection};
use deterministic_harness::{
    ClosedLoopConfig, ClosedLoopEstimatorConfig, ClosedLoopEstimatorReport,
    ClosedLoopEstimatorRunner, ClosedLoopEstimatorTrace, ClosedLoopTruthState, ControlPipeline,
    ControlSetpoint, HilSemanticAdapter, HilSemanticFrameConfig, SensorFrame, TruthPlant,
    hil_response_from_pipeline, response_has_flag, response_is_failsafe_zero_output,
    sensor_frame_to_hil_frame, stale_hil_response_from_pipeline,
};

const DT_S: f32 = 0.01;

#[test]
fn p07_hil_semantics_duplicate_tick_rejected() {
    let mut adapter = HilSemanticAdapter::active();
    let first = sensor_frame_to_hil_frame(2, &sensor_at(20_000), HilSemanticFrameConfig::default());
    let duplicate =
        sensor_frame_to_hil_frame(2, &sensor_at(30_000), HilSemanticFrameConfig::default());

    assert!(adapter.accept_frame(first).accepted);
    let dispatch = adapter.accept_frame(duplicate);

    assert!(!dispatch.accepted);
    assert_eq!(
        dispatch.rejection,
        Some(HilSensorFrameRejection::DuplicateTick)
    );
    assert_eq!(dispatch.counters.duplicate_tick, 1);
}

#[test]
fn p07_hil_semantics_out_of_order_time_rejected() {
    let mut adapter = HilSemanticAdapter::active();
    let first = sensor_frame_to_hil_frame(2, &sensor_at(20_000), HilSemanticFrameConfig::default());
    let out_of_order =
        sensor_frame_to_hil_frame(3, &sensor_at(10_000), HilSemanticFrameConfig::default());

    assert!(adapter.accept_frame(first).accepted);
    let dispatch = adapter.accept_frame(out_of_order);

    assert!(!dispatch.accepted);
    assert_eq!(
        dispatch.rejection,
        Some(HilSensorFrameRejection::OutOfOrderTick)
    );
    assert_eq!(dispatch.counters.out_of_order_tick, 1);
}

#[test]
fn p07_hil_semantics_partial_valid_flags_publish_only_asserted_groups() {
    let mut adapter = HilSemanticAdapter::active();
    let mut sensor = sensor_at(20_000);
    sensor.mag_body_ut = None;
    sensor.gps_position_ned_m = None;
    sensor.gps_velocity_ned_mps = None;
    sensor.baro_down_m = None;
    let frame = sensor_frame_to_hil_frame(2, &sensor, HilSemanticFrameConfig::default());

    let dispatch = adapter.accept_frame(frame);
    let published = adapter.published_groups();

    assert!(dispatch.accepted);
    assert!(dispatch.dispatch.tick.is_some());
    assert!(dispatch.dispatch.imu_published);
    assert!(!dispatch.dispatch.barometer_published);
    assert!(!dispatch.dispatch.gps_published);
    assert!(!dispatch.dispatch.magnetometer_published);
    assert_eq!(published.time, 1);
    assert_eq!(published.imu, 1);
    assert_eq!(published.barometer, 0);
    assert_eq!(published.gps, 0);
    assert_eq!(published.magnetometer, 0);
}

#[test]
fn p07_hil_semantics_non_finite_asserted_group_rejected() {
    let mut adapter = HilSemanticAdapter::active();
    let mut frame =
        sensor_frame_to_hil_frame(2, &sensor_at(20_000), HilSemanticFrameConfig::default());
    frame.gyro_rps[0] = f32::NAN;

    let dispatch = adapter.accept_frame(frame);

    assert!(!dispatch.accepted);
    assert_eq!(
        dispatch.rejection,
        Some(HilSensorFrameRejection::InvalidSample)
    );
    assert_eq!(dispatch.counters.invalid_non_finite_sample, 1);
}

#[test]
fn p07_hil_semantics_response_matches_latest_accepted_stamp() {
    let trace = closed_loop_trace_at(2);
    let mut adapter = HilSemanticAdapter::active();
    let frame = sensor_frame_to_hil_frame(
        trace.tick,
        &trace.sensors,
        HilSemanticFrameConfig::default(),
    );
    let dispatch = adapter.accept_frame(frame);
    let response = hil_response_from_pipeline(
        frame.stamp,
        dispatch.accepted && dispatch.dispatch.imu_published,
        &trace.estimator,
        &trace.control,
    );

    adapter.push_response(response);

    assert!(dispatch.accepted);
    assert_eq!(response.stamp, frame.stamp);
    assert_eq!(
        adapter.correlation().latest_accepted_stamp,
        Some(frame.stamp)
    );
    assert_eq!(adapter.correlation().current_in_flight_tick, None);
    assert!(response_has_flag(
        response,
        response_flags::SENSOR_INPUT_VALID
    ));
    assert!(response_has_flag(response, response_flags::ESTIMATOR_VALID));
    assert!(response_has_flag(response, response_flags::CONTROL_VALID));
    assert!(response_has_flag(response, response_flags::MOTORS_VALID));
    assert!(!response_has_flag(response, response_flags::FAILSAFE));
    assert!(response.motor_cmd.iter().any(|command| *command > 0));
}

#[test]
fn p07_hil_semantics_stale_estimate_enters_failsafe_zero_output() {
    let trace = closed_loop_trace_at(2);
    let frame = sensor_frame_to_hil_frame(
        trace.tick,
        &trace.sensors,
        HilSemanticFrameConfig::default(),
    );
    let response =
        stale_hil_response_from_pipeline(frame.stamp, 1, true, &trace.estimator, &trace.control);

    assert!(response_has_flag(response, response_flags::ARMED));
    assert!(response_has_flag(response, response_flags::FAILSAFE));
    assert!(response_has_flag(response, response_flags::CONTROL_VALID));
    assert!(response_has_flag(response, response_flags::MOTORS_VALID));
    assert!(!response_has_flag(
        response,
        response_flags::ESTIMATOR_VALID
    ));
    assert!(response_is_failsafe_zero_output(response));
}

fn sensor_at(timestamp_us: u64) -> SensorFrame {
    SensorFrame {
        timestamp_us,
        accel_mps2: [0.0, 0.0, 0.0],
        gyro_rps: [0.0, 0.0, 0.0],
        mag_body_ut: Some([20.0, 0.0, 40.0]),
        gps_position_ned_m: Some([0.0, 0.0, 0.0]),
        gps_velocity_ned_mps: Some([0.0, 0.0, 0.0]),
        baro_down_m: Some(0.0),
    }
}

fn closed_loop_trace_at(tick: u64) -> ClosedLoopEstimatorTrace {
    let report = run_closed_loop_estimator();
    report
        .traces
        .into_iter()
        .find(|trace| trace.tick == tick)
        .expect("trace tick should exist")
}

fn run_closed_loop_estimator() -> ClosedLoopEstimatorReport {
    let mut runner = ClosedLoopEstimatorRunner::new(
        ControlPipeline::default(),
        TruthPlant::new(Default::default(), ClosedLoopTruthState::LEVEL_ORIGIN),
        ClosedLoopEstimatorConfig::new(ClosedLoopConfig::new(DT_S, 3)),
    );
    runner
        .run(ControlSetpoint::ORIGIN_HOLD_ARMED)
        .expect("closed-loop estimator run should pass")
}
