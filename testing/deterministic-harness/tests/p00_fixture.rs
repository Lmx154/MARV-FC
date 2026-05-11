use deterministic_harness::Fixture;

#[test]
fn fixture_loader_accepts_monotonic_timestamps() {
    let fixture = Fixture::from_csv_str(
        "monotonic",
        "\
timestamp_us,accel_x,accel_y
1000,0.0,1.0
2000,0.1,1.1
3000,0.2,1.2
",
    )
    .expect("fixture should parse");

    assert_eq!(fixture.columns, ["timestamp_us", "accel_x", "accel_y"]);
    assert_eq!(fixture.samples.len(), 3);
    assert_eq!(fixture.samples[2].timestamp_us, 3000);
    assert_eq!(fixture.samples[2].values, [0.2, 1.2]);
}

#[test]
fn fixture_loader_rejects_non_monotonic_timestamps() {
    let error = Fixture::from_csv_str(
        "bad-time",
        "\
timestamp_us,accel_x
1000,0.0
1000,0.1
",
    )
    .expect_err("duplicate timestamp should be rejected");

    assert_eq!(error.sim_time_us, 1000);
    assert!(error.message.contains("non-monotonic timestamp"));
}

#[test]
fn fixture_loader_rejects_non_finite_values() {
    let error = Fixture::from_csv_str(
        "bad-value",
        "\
timestamp_us,accel_x
1000,NaN
",
    )
    .expect_err("non-finite asserted fixture value should be rejected");

    assert_eq!(error.sim_time_us, 1000);
    assert!(error.message.contains("non-finite"));
}
