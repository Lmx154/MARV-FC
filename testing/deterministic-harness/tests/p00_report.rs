use deterministic_harness::{HarnessReport, StepTrace, TestCase};

#[test]
fn report_contains_failure_tick_and_last_trace() {
    let case = TestCase::new("phase2-report", 0.01, 10).with_seed(42);
    let mut report = HarnessReport::new(case);
    report.push_trace(StepTrace::new(1, 10_000, 0.01, "first"));
    report.push_trace(StepTrace::new(2, 20_000, 0.01, "second").with_motors([0.1, 0.2, 0.3, 0.4]));

    let failure = report.fail("expected bounded state");

    assert!(!report.passed());
    assert_eq!(failure.tick, 2);
    assert_eq!(failure.sim_time_us, 20_000);
    assert_eq!(failure.message, "expected bounded state");
    assert_eq!(
        failure.last_trace.as_ref().expect("last trace").label,
        "second"
    );
    assert_eq!(
        failure.last_trace.expect("last trace").motors,
        Some([0.1, 0.2, 0.3, 0.4])
    );
}
