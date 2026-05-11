use deterministic_harness::{ClockSnapshot, LockstepClock};

#[test]
fn p00_clock_advances_only_when_step_called() {
    let mut clock = LockstepClock::new();

    assert_eq!(
        clock.snapshot(),
        ClockSnapshot {
            tick: 0,
            sim_time_us: 0,
            dt_s: 0.0,
        }
    );

    let first = clock.advance(0.01).expect("positive dt should advance");
    assert_eq!(first.previous.tick, 0);
    assert_eq!(first.previous.sim_time_us, 0);
    assert_eq!(first.current.tick, 1);
    assert_eq!(first.current.sim_time_us, 10_000);
    assert_eq!(first.current.dt_s, 0.01);

    assert_eq!(clock.snapshot(), first.current);

    let second = clock.advance(0.02).expect("positive dt should advance");
    assert_eq!(second.previous, first.current);
    assert_eq!(second.current.tick, 2);
    assert_eq!(second.current.sim_time_us, 30_000);
    assert_eq!(second.current.dt_s, 0.02);
}

#[test]
fn p00_clock_rejects_zero_or_negative_dt() {
    let mut clock = LockstepClock::new();

    assert!(clock.advance(0.0).is_err());
    assert!(clock.advance(-0.01).is_err());
    assert!(clock.advance(f32::NAN).is_err());
    assert_eq!(clock.snapshot().tick, 0);
    assert_eq!(clock.snapshot().sim_time_us, 0);
}
