//! Host tests of position hold at the step level (decision-nav-eskf-20261120, Amendment P and Addendum P-1; invariants
//! N1, N2, N3, N5, N6, N9, N10 and the log fill, P5). The estimator is the fake of `althold_tests`: it returns the state
//! the test sets. The controller-level invariants (N4, N5's point masses) are in `pid.rs`. Every expected number comes
//! from `marv_card` or `Tuning`, or is a direction worked out by hand.

extern crate std;

use super::althold_tests::{ALTITUDE, DT_US, TestFlight, flight, input, mot_mode_byte, nav_at, step_rc};
use super::*;
use core::cell::Cell;
use std::vec::Vec;

/// Everything Position needs.
const POSITION: Valid = Valid {
    heading: true,
    velocity_ne: true,
    position_ne: true,
    ..ALTITUDE
};

/// Level at heading `yaw`, at (n, e, d) with velocity `v`, with `valid`.
fn pnav(n: f32, e: f32, d: f32, v: [f32; 3], yaw: f32, valid: Valid) -> NavState {
    NavState {
        t_us: 0,
        attitude: [libm::cosf(yaw / 2.0), 0.0, 0.0, libm::sinf(yaw / 2.0)],
        rates_rads: [0.0; 3],
        velocity_ms: v,
        position_m: [n, e, d],
        valid,
        sigma: None,
    }
}

/// One step with a fresh report carrying all four sticks.
fn step_sticks(
    f: &mut TestFlight,
    t_us: u64,
    arm: bool,
    [roll, pitch, throttle, yaw]: [f32; 4],
    switch6: Switch3,
) -> StepOut {
    f.step(&input(
        t_us,
        Some(RcSample {
            sticks: Sticks {
                roll,
                pitch,
                throttle,
                yaw,
            },
            arm,
            switch6,
            t_us,
        }),
    ))
}

const HOVER: f32 = 0.5;

/// Arms (switch cycle off then on, throttle `throttle`) with the mode switch at `switch6`.
fn arm(f: &mut TestFlight, t: &mut u64, switch6: Switch3, throttle: f32) -> StepOut {
    let mut out = None;
    for on in [false, true] {
        *t += DT_US;
        out = Some(step_rc(f, *t, on, throttle, switch6));
    }
    let out = out.unwrap();
    assert!(out.armed, "did not arm");
    out
}

/// The acceleration (north, east) of a point mass held at the weight's vertical share by attitude `q`.
fn accel_ne(q: [f32; 4]) -> [f32; 2] {
    let [w, x, y, z] = q;
    let col = [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), w * w - x * x - y * y + z * z];
    [-GRAVITY_MSS * col[0] / col[2], -GRAVITY_MSS * col[1] / col[2]]
}

fn near(a: f32, b: f32, tol: f32) {
    assert!((a - b).abs() <= tol, "{a} vs {b} (tol {tol})");
}

fn position_of(out: &StepOut) -> ([f32; 3], [f32; 3], f32) {
    match out.setpoint {
        Setpoint::Position {
            position_m,
            velocity_ms,
            heading_rad,
            ..
        } => (position_m, velocity_ms, heading_rad),
        other => panic!("not Position: {other:?}"),
    }
}

/// A flight in the air in Position: two steps with the stick at the hover centre bring it to Position.
fn airborne<'a>(cell: &'a Cell<NavState>, h: &'static Handshake, tuning: &Tuning, t: &mut u64) -> TestFlight<'a> {
    let mut f = flight(cell, h, tuning);
    // These flights are disarmed: the latch is forced so that the other rules can be tested.
    f.airborne_forced = true;
    *t += DT_US;
    let out = step_sticks(&mut f, *t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    assert_eq!(out.mode, Mode::Altitude, "not airborne at construction");
    *t += DT_US;
    let out = step_sticks(&mut f, *t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    assert_eq!(out.mode, Mode::Position);
    f
}

// ---- N1: Position only when covered and airborne ----

/// The coverage part: each flag of `Setpoint::Position.requires()` dropped for one step, in a flight with the latch forced
/// (disarmed steps, so the flags are the only variable). `ignore_cov` is the negative control. Returns the failures.
fn n1_failures(ignore_cov: bool) -> Vec<&'static str> {
    static H0: Handshake = Handshake::new();
    static H1: Handshake = Handshake::new();
    let h = if ignore_cov { &H1 } else { &H0 };
    let tuning = Tuning::DEFAULT;
    let mut bad = Vec::new();
    let mut check = |ok: bool, what: &'static str| {
        if !ok {
            bad.push(what);
        }
    };
    // (flag, valid, fallback of the ladder, whether Position returns the very step the flag does: when the step between
    // had an Ok demand at the weight; with tilt or rates false even Angle is refused, which clears no latch but counts as a
    // low step, and Position returns at once since the latch is still set).
    let flags: [(&str, Valid, Mode); 7] = [
        ("tilt", Valid { tilt: false, ..POSITION }, Mode::Angle),
        ("heading", Valid { heading: false, ..POSITION }, Mode::Altitude),
        ("rates", Valid { rates: false, ..POSITION }, Mode::Angle),
        ("velocity_ne", Valid { velocity_ne: false, ..POSITION }, Mode::Altitude),
        ("velocity_d", Valid { velocity_d: false, ..POSITION }, Mode::Angle),
        ("position_ne", Valid { position_ne: false, ..POSITION }, Mode::Altitude),
        ("position_d", Valid { position_d: false, ..POSITION }, Mode::Angle),
    ];
    let mut t = 1_000;
    let cell = Cell::new(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
    let mut f = flight(&cell, h, &tuning);
    f.ignore_coverage = ignore_cov;
    f.airborne_forced = true;
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    check(out.mode == Mode::Altitude, "Altitude at the first step: not airborne at construction");
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    check(out.mode == Mode::Position, "Position once covered and airborne");
    check(mot_mode_byte(&out) == 3, "MOT carries code 3");
    for (_, valid, fallback) in flags {
        cell.set(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, valid));
        t += DT_US;
        let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        check(out.mode != Mode::Position, "Position never with a flag false");
        check(out.mode == fallback, "the fallback of the ladder");
        check(f.shaper.target_ne.is_none(), "no position target outside Position");
        cell.set(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
        t += DT_US;
        let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        check(out.mode == Mode::Position, "Position the step the flag returns");
        t += DT_US;
        step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        check(f.shaper.target_ne.is_some(), "a fresh capture on return");
    }
    bad
}

#[test]
fn n1_position_only_when_covered() {
    let bad = n1_failures(false);
    assert!(bad.is_empty(), "{bad:?}");
}

/// Negative control: coverage ignored.
#[test]
fn n1_control_ignoring_coverage_fails() {
    let bad = n1_failures(true);
    std::println!("coverage ignored: {bad:?}");
    assert!(bad.contains(&"Position never with a flag false"));
}

/// Sets the fake estimate so that the vertical loop asks `frac` x m g in the next step (the stick at the hover centre, the
/// height at the target, tilt 0): the climb rate cancels the integrator's part. `frac` 0 puts the demand on the idle floor.
/// `ne` is the estimate's north-east position, `vn` its north velocity.
fn collective(cell: &Cell<NavState>, f: &TestFlight, frac: f32, ne: [f32; 2], vn: f32, valid: Valid) {
    let i = f.controller.vertical_integrator();
    let vd = -(GRAVITY_MSS * (1.0 - frac) - i) / Tuning::DEFAULT.psc_velz_p;
    cell.set(pnav(ne[0], ne[1], -2.0, [vn, 0.0, vd], 0.0, valid));
}

/// One armed Position-switch step with the collective `frac` x m g.
fn armed_step(f: &mut TestFlight, cell: &Cell<NavState>, t: &mut u64, frac: f32) -> StepOut {
    collective(cell, f, frac, [0.0, 0.0], 0.0, POSITION);
    *t += DT_US;
    step_sticks(f, *t, true, [0.0, 0.0, HOVER, 0.0], Switch3::High)
}

/// The latch table (P3 second revision) on an armed flight in Position. The controls: `timer_off`, the previous rule
/// (`threshold_rule`: low is a collective below the ground threshold) and `ignore_cov`. Returns the failures.
fn n1_latch_failures(timer_off: bool, threshold_rule: bool, ignore_cov: bool) -> Vec<&'static str> {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let land = tuning.land_detect_us / DT_US; // low steps that reach land_detect_us: 800
    assert_eq!(land * DT_US, tuning.land_detect_us);
    let thirty = 30_000_000 / DT_US;
    let mut bad = Vec::new();
    let mut check = |ok: bool, what: &'static str| {
        if !ok {
            bad.push(what);
        }
    };
    let cell = Cell::new(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
    let mut f = flight(&cell, h, &tuning);
    f.timer_off = timer_off;
    f.threshold_rule = threshold_rule;
    f.ignore_coverage = ignore_cov;
    let mut t = 1_000;
    check(!f.airborne, "false at construction");
    for _ in 0..3 {
        collective(&cell, &f, 1.0, [0.0; 2], 0.0, POSITION);
        t += DT_US;
        step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        check(!f.airborne, "false while disarmed, even with the collective up");
    }
    collective(&cell, &f, 0.0, [0.0; 2], 0.0, POSITION);
    arm(&mut f, &mut t, Switch3::High, 0.0);
    // The idle floor since arming: false for 30 s, never Position. Then between the floor and the threshold, 30 s.
    for (frac, what) in [(0.0, "the floor since arming: false for 30 s"), (0.7, "between the floor and the threshold since arming: false for 30 s")] {
        let mut ok = true;
        for _ in 0..thirty {
            let out = armed_step(&mut f, &cell, &mut t, frac);
            let floor = out.allocation.is_some_and(|a| a.saturation.thrust_low);
            assert_eq!(floor, frac == 0.0, "frac {frac}: the allocator's floor flag");
            ok &= !f.airborne && out.mode != Mode::Position;
        }
        check(ok, what);
    }
    // The step at the threshold: true after it, Position the step after.
    let out = armed_step(&mut f, &cell, &mut t, 1.0);
    check(out.mode != Mode::Position && f.airborne, "true after the step that first reaches the threshold");
    let out = armed_step(&mut f, &cell, &mut t, 1.0);
    check(out.mode == Mode::Position, "Position the step after");
    // Once set, 30 s between the floor and the threshold: still true, still Position.
    let mut ok = true;
    for _ in 0..thirty {
        let out = armed_step(&mut f, &cell, &mut t, 0.7);
        ok &= f.airborne && out.mode == Mode::Position;
    }
    check(ok, "once set, 30 s between the floor and the threshold is still true");
    if !f.airborne {
        // The control rule cleared it: go on from a set latch.
        for _ in 0..3 {
            armed_step(&mut f, &cell, &mut t, 1.0);
        }
    }
    // The floor one step short of land_detect_us: true; one non-low step restarts the count.
    for round in 0..2 {
        let mut ok = true;
        for _ in 1..land {
            let out = armed_step(&mut f, &cell, &mut t, 0.0);
            ok &= f.airborne && out.mode == Mode::Position;
        }
        check(ok, "true (Position) one step short of land_detect_us on the floor");
        if round == 0 {
            armed_step(&mut f, &cell, &mut t, 0.7);
        }
    }
    // The step that reaches land_detect_us clears it; the next step is Altitude.
    armed_step(&mut f, &cell, &mut t, 0.0);
    check(!f.airborne, "false at land_detect_us");
    let out = armed_step(&mut f, &cell, &mut t, 0.0);
    check(out.mode == Mode::Altitude, "Altitude after the latch clears");
    // Take off again, then a refused demand counts as low.
    for _ in 0..3 {
        armed_step(&mut f, &cell, &mut t, 1.0);
    }
    check(f.airborne, "airborne again");
    cell.set(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, Valid::NONE));
    for _ in 0..land {
        t += DT_US;
        let out = step_sticks(&mut f, t, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        check(out.mode != Mode::Position, "never Position while the estimate does not cover it");
    }
    check(!f.airborne, "a refused demand counts as low");
    // Disarm: false at once.
    for _ in 0..3 {
        armed_step(&mut f, &cell, &mut t, 1.0);
    }
    collective(&cell, &f, 1.0, [0.0; 2], 0.0, POSITION);
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    check(!out.armed && !f.airborne && f.low_us == 0, "false and the timer 0 after a disarm");
    bad
}

#[test]
fn n1_the_airborne_latch_table() {
    let bad = n1_latch_failures(false, false, false);
    assert!(bad.is_empty(), "{bad:?}");
}

/// Negative control: the timer ignored.
#[test]
fn n1_control_ignoring_the_timer_fails() {
    let bad = n1_latch_failures(true, false, false);
    std::println!("timer ignored: {bad:?}");
    assert!(bad.contains(&"false at land_detect_us"));
}

/// Negative control: the previous rule (low below the ground threshold) fails the 30 s between the floor and the threshold.
#[test]
fn n1_control_the_previous_rule_fails() {
    let bad = n1_latch_failures(false, true, false);
    std::println!("previous rule: {bad:?}");
    assert!(bad.contains(&"once set, 30 s between the floor and the threshold is still true"));
}

/// Negative control: coverage ignored.
#[test]
fn n1_control_ignoring_coverage_fails_the_latch_table() {
    let bad = n1_latch_failures(false, false, true);
    std::println!("coverage ignored: {bad:?}");
    assert!(bad.contains(&"never Position while the estimate does not cover it"));
}

// ---- N1b: no blip ----

/// Armed in Position with a velocity error that winds the horizontal integrator and a locked target; the collective: the
/// weight for 0.25 s, a forced 0.5 s dip to the floor, the weight for 0.25 s, then 5 s at 0.70 m g (between the floor and
/// the threshold). Compares every step with a twin that has the weight throughout: the mode, the position target and the
/// integrator must be bit-identical. Returns whether they are, and the steps in Position.
fn n1b_run(per_step_rule: bool, threshold_rule: bool) -> (bool, u32) {
    let mk = |per_step: bool, threshold: bool| {
        let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
        h.set_permit(true);
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(pnav(
            0.0, 0.0, -2.0, [0.3, 0.0, 0.0], 0.0, POSITION,
        ))));
        let mut f = flight(cell, h, &Tuning::DEFAULT);
        f.per_step_rule = per_step;
        f.threshold_rule = threshold;
        (cell, f)
    };
    let ((ca, mut a), (cb, mut b)) = (mk(per_step_rule, threshold_rule), mk(false, false));
    let (mut ta, mut tb) = (1_000, 1_000);
    collective(ca, &a, 0.0, [0.0; 2], 0.3, POSITION);
    collective(cb, &b, 0.0, [0.0; 2], 0.3, POSITION);
    arm(&mut a, &mut ta, Switch3::High, 0.0);
    arm(&mut b, &mut tb, Switch3::High, 0.0);
    let (mut same, mut in_position) = (true, 0);
    let (up, dip, five_s) = (200, 400, 4_000);
    for k in 0..(up + dip + up + five_s) {
        let frac = if (up..up + dip).contains(&k) {
            0.0
        } else if k >= up + dip + up {
            0.7
        } else {
            1.0
        };
        collective(ca, &a, frac, [0.0; 2], 0.3, POSITION);
        collective(cb, &b, 1.0, [0.0; 2], 0.3, POSITION);
        ta += DT_US;
        tb += DT_US;
        let oa = step_sticks(&mut a, ta, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        let ob = step_sticks(&mut b, tb, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        if k >= 2 && oa.mode == Mode::Position {
            in_position += 1;
        }
        let (ia, ib) = (a.controller.horizontal_integrator(), b.controller.horizontal_integrator());
        same &= oa.mode == ob.mode
            && a.shaper.target_ne == b.shaper.target_ne
            && ia[0].to_bits() == ib[0].to_bits()
            && ia[1].to_bits() == ib[1].to_bits();
        if k == up - 1 {
            assert!(ia[0].abs() > 0.01, "{ia:?}");
        }
    }
    (same, in_position)
}

#[test]
fn n1b_a_dip_to_the_floor_and_5_s_at_0_70_mg_keep_position_target_and_integrator() {
    let (same, in_position) = n1b_run(false, false);
    std::println!("N1b: {in_position} steps in Position, identical to the twin: {same}");
    assert!(same && in_position >= 200 + 400 + 200 + 4_000 - 4);
}

/// Negative controls: the old per-step rule fails in the dip; the previous latch rule fails in the 5 s at 0.70 m g.
#[test]
fn n1b_controls_the_old_per_step_rule_and_the_previous_rule_fail() {
    assert!(!n1b_run(true, false).0);
    assert!(!n1b_run(false, true).0);
}

// ---- N6: armed in Position on the ground, the landing and the aborted take-off ----

/// (a) Armed in Position at rest on the ground for 30 s with the estimate wandering by 0.5 m, the collective on the floor:
/// Altitude, the horizontal integrator 0. (b) Then up (airborne), then the floor for `land_detect_us`: Altitude from the
/// step after. (c) An aborted take-off (the weight for 0.3 s, then the floor): Position for at most `land_detect_us` after
/// the drop, then Altitude. Returns (a: Position steps, a: peak integrator, b: Position steps after the clear, c: Position
/// steps in the low phase, c: steps until Altitude).
fn n6_run(timer_off: bool) -> (u32, f32, u32, u32, u32) {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(pnav(0.0, 0.0, 0.0, [0.0; 3], 0.0, POSITION));
    let mut f = flight(&cell, h, &tuning);
    f.timer_off = timer_off;
    let mut t = 1_000;
    collective(&cell, &f, 0.0, [0.0; 2], 0.0, POSITION);
    arm(&mut f, &mut t, Switch3::High, 0.0);
    let (mut in_position, mut worst) = (0, 0.0_f32);
    let dt = DT_US as f32 * 1e-6;
    let wander = |cell: &Cell<NavState>, f: &TestFlight, k: u32, frac: f32| {
        let s = k as f32 * dt;
        collective(cell, f, frac, [0.5 * libm::sinf(0.9 * s), 0.5 * libm::cosf(0.7 * s)], 0.0, POSITION);
    };
    let a_steps = (30.0 / dt) as u32;
    for k in 0..a_steps {
        wander(&cell, &f, k, 0.0);
        t += DT_US;
        let out = step_sticks(&mut f, t, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        assert!(out.armed);
        if out.mode == Mode::Position {
            in_position += 1;
        }
        let [i_n, i_e] = f.controller.horizontal_integrator();
        worst = worst.max(libm::sqrtf(i_n * i_n + i_e * i_e));
    }
    let land = (tuning.land_detect_us / DT_US) as u32;
    for k in 0..400 {
        wander(&cell, &f, a_steps + k, 1.0);
        t += DT_US;
        step_sticks(&mut f, t, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    }
    assert!(f.airborne);
    let mut after_clear = 0;
    for k in 0..(2 * land) {
        wander(&cell, &f, a_steps + 400 + k, 0.0);
        t += DT_US;
        let out = step_sticks(&mut f, t, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        if k >= land + 1 && out.mode == Mode::Position {
            after_clear += 1;
        }
    }
    for k in 0..240 {
        wander(&cell, &f, k, 1.0);
        t += DT_US;
        step_sticks(&mut f, t, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    }
    let (mut low_in_position, mut until_alt) = (0, 0);
    for k in 0..(2 * land) {
        wander(&cell, &f, k, 0.0);
        t += DT_US;
        let out = step_sticks(&mut f, t, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        if out.mode == Mode::Position {
            low_in_position += 1;
        } else if until_alt == 0 {
            until_alt = k + 1;
        }
    }
    (in_position, worst, after_clear, low_in_position, until_alt)
}

#[test]
fn n6_on_the_ground_airborne_then_landing_and_an_aborted_take_off() {
    let land = (Tuning::DEFAULT.land_detect_us / DT_US) as u32;
    let (a_pos, a_i, b_after, c_pos, c_until) = n6_run(false);
    std::println!("N6: (a) {a_pos} Position steps, peak |I| {a_i}; (b) {b_after} Position steps after the clear; (c) {c_pos} Position steps in the low phase, Altitude from step {c_until}");
    assert_eq!((a_pos, a_i), (0, 0.0));
    assert_eq!(b_after, 0);
    assert!(c_pos <= land && c_until > 0 && c_until <= land + 1);
}

/// Negative control: a latch that never clears fails (b) and (c).
#[test]
fn n6_control_a_latch_that_never_clears_fails_the_landing() {
    let (a_pos, _, b_after, c_pos, c_until) = n6_run(true);
    std::println!("N6 control: (a) {a_pos}, (b) {b_after} Position steps after the clear, (c) {c_pos}, Altitude from {c_until}");
    assert_eq!(a_pos, 0, "(a) does not depend on the timer");
    assert!(b_after > 0, "(b) fails");
    assert_eq!(c_until, 0, "(c) never reaches Altitude");
}

/// N6 (d): landed in Angle at part throttle (the latch stays set), then the switch to Position with the stick low: Position,
/// then the vertical loop (the target descends, the vehicle cannot) takes the collective to the floor and the latch clears
/// `land_detect_us` later. Returns (steps in Position, the peak tilt setpoint in Position, rad).
fn n6d_run() -> (u32, f32) {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
    let mut f = flight(&cell, h, &tuning);
    let mut t = 1_000;
    collective(&cell, &f, 0.0, [0.0; 2], 0.0, POSITION);
    arm(&mut f, &mut t, Switch3::High, 0.0);
    for _ in 0..3 {
        armed_step(&mut f, &cell, &mut t, 1.0);
    }
    assert!(f.airborne);
    // Landed: Angle, part throttle (0.3 of the stick: 0.6 m g, between the floor and the threshold), the vehicle at rest.
    cell.set(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
    for _ in 0..(5_000_000 / DT_US) {
        t += DT_US;
        let out = step_sticks(&mut f, t, true, [0.0, 0.0, 0.3, 0.0], Switch3::Low);
        assert_eq!(out.mode, Mode::Angle);
        assert!(f.airborne, "the latch stays set in Angle at part throttle");
    }
    // The switch to Position, the stick low, the vehicle on the ground (the estimate wanders by 0.5 m).
    let (mut in_position, mut peak) = (0, 0.0_f32);
    let dt = DT_US as f32 * 1e-6;
    for k in 0..(6_000_000 / DT_US) {
        let s = k as f32 * dt;
        cell.set(pnav(0.5 * libm::sinf(0.9 * s), 0.5 * libm::cosf(0.7 * s), -2.0, [0.0; 3], 0.0, POSITION));
        t += DT_US;
        let out = step_sticks(&mut f, t, true, [0.0, 0.0, 0.0, 0.0], Switch3::High);
        if k == 0 {
            assert_eq!(out.mode, Mode::Position, "Position on the switch");
        }
        if out.mode == Mode::Position {
            in_position += 1;
            let [w, x, y, z] = out.attitude_sp;
            peak = peak.max(libm::acosf((w * w - x * x - y * y + z * z).min(1.0)));
        } else {
            assert!(!f.airborne);
        }
    }
    (in_position, peak)
}

#[test]
fn n6d_landed_in_angle_at_part_throttle_then_position_ends_within_3_s() {
    let (in_position, peak) = n6d_run();
    let secs = in_position as f32 * DT_US as f32 * 1e-6;
    std::println!("N6 (d): Position for {secs:.3} s after the switch, peak tilt setpoint {:.3} deg", peak.to_degrees());
    assert!(in_position >= 1 && secs <= 3.0, "{secs}");
}

// ---- P3: never Position on the arming step ----

/// Disarmed in Position with the collective up, then the arming step with the throttle at the floor; the latch is false
/// after every disarmed step. `control`: the latch forced true at every step. Returns (the arming step's mode, the latch
/// after each disarmed step, the first armed step's mode).
fn arming_step(control: bool) -> (Mode, bool, Mode) {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
    let mut f = flight(&cell, h, &tuning);
    f.airborne_forced = control;
    let mut t = 1_000;
    let mut latch = false;
    for _ in 0..5 {
        t += DT_US;
        let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        latch |= f.airborne;
        assert!(out.demand.is_ok_and(|d| d.thrust_n >= f.ground_thrust_n));
    }
    t += DT_US;
    let out = step_sticks(&mut f, t, true, [0.0, 0.0, 0.0, 0.0], Switch3::High);
    assert!(out.armed, "the arming step");
    let arming = out.mode;
    t += DT_US;
    let first_armed = step_sticks(&mut f, t, true, [0.0, 0.0, 0.0, 0.0], Switch3::High).mode;
    (arming, latch, first_armed)
}

#[test]
fn the_first_armed_step_is_never_position_and_the_latch_is_false_after_each_disarm() {
    let (arming, latch, first) = arming_step(false);
    assert!(arming != Mode::Position && !latch && first != Mode::Position);
}

/// Negative control: with the latch forced the arming step is Position.
#[test]
fn control_with_the_latch_forced_the_arming_step_is_position() {
    assert_eq!(arming_step(true).0, Mode::Position);
}

// ---- N2: the capture ----

fn n2_failures(any_speed: bool) -> Vec<&'static str> {
    static H0: Handshake = Handshake::new();
    static H1: Handshake = Handshake::new();
    let h = if any_speed { &H1 } else { &H0 };
    let tuning = Tuning::DEFAULT;
    let hold = tuning.pilot_hold_speed_ms;
    let mut bad = Vec::new();
    let mut check = |ok: bool, what: &'static str| {
        if !ok {
            bad.push(what);
        }
    };
    let cell = Cell::new(pnav(10.0, 0.0, -2.0, [3.0, 0.0, 0.0], 0.0, POSITION));
    let mut t = 1_000;
    let mut f = airborne(&cell, h, &tuning, &mut t);
    f.shaper.capture_any_speed = any_speed;
    let centred = [0.0, 0.0, HOVER, 0.0];
    // Switched on while moving at 3 m/s: brakes (no target, error exactly 0, velocity-only).
    t += DT_US;
    let out = step_sticks(&mut f, t, false, centred, Switch3::High);
    let (p, v, _) = position_of(&out);
    check(f.shaper.target_ne.is_none(), "no capture at 3 m/s");
    check(p[0] == 10.0 && p[1] == 0.0, "unlocked: the target is this step's position");
    check(v[0] == 0.0 && v[1] == 0.0, "no feed-forward with the sticks centred");
    // Slowing: exactly the hold speed is not below it; just under it is.
    cell.set(pnav(11.5, 0.2, -2.0, [hold, 0.0, 0.0], 0.0, POSITION));
    t += DT_US;
    step_sticks(&mut f, t, false, centred, Switch3::High);
    check(f.shaper.target_ne.is_none(), "no capture at the hold speed itself");
    cell.set(pnav(12.3, 4.5, -2.0, [hold * 0.999, 0.0, 0.0], 0.0, POSITION));
    t += DT_US;
    let out = step_sticks(&mut f, t, false, centred, Switch3::High);
    check(f.shaper.target_ne == Some([12.3, 4.5]), "captured at the capture step's N, E");
    check(position_of(&out).0[..2] == [12.3, 4.5], "the setpoint carries the capture");
    // Drifting afterwards: the target stays.
    cell.set(pnav(13.0, 5.0, -2.0, [0.05, 0.0, 0.0], 0.0, POSITION));
    t += DT_US;
    let out = step_sticks(&mut f, t, false, centred, Switch3::High);
    check(position_of(&out).0[..2] == [12.3, 4.5], "held while the sticks are centred");
    // Pitch stick 0.55: through the dead zone 0.5 of the speed, the target released, the error exactly 0.
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.55, HOVER, 0.0], Switch3::High);
    let (p, v, _) = position_of(&out);
    check(f.shaper.target_ne.is_none(), "released when the sticks move");
    check(p[0] == 13.0 && p[1] == 5.0, "unlocked: position error exactly 0");
    check((v[0] - 0.5 * tuning.pilot_speed_xy_ms).abs() <= 1e-5 && v[1] == 0.0, "the velocity feed-forward");
    check(p.iter().chain(v.iter()).all(|x| x.is_finite()), "finite");
    // Released at 3 m/s: brakes, no capture until it is slow.
    cell.set(pnav(20.0, 5.0, -2.0, [3.0, 0.0, 0.0], 0.0, POSITION));
    t += DT_US;
    step_sticks(&mut f, t, false, centred, Switch3::High);
    check(f.shaper.target_ne.is_none(), "no capture at release while fast");
    cell.set(pnav(21.0, 5.1, -2.0, [0.3, 0.1, 0.0], 0.0, POSITION));
    t += DT_US;
    step_sticks(&mut f, t, false, centred, Switch3::High);
    check(f.shaper.target_ne == Some([21.0, 5.1]), "captured when slow");
    bad
}

#[test]
fn n2_the_target_is_the_capture_steps_position_only_when_stopped() {
    let bad = n2_failures(false);
    assert!(bad.is_empty(), "{bad:?}");
}

/// Negative control: capture at any speed.
#[test]
fn n2_control_capturing_at_any_speed_fails() {
    let bad = n2_failures(true);
    std::println!("capture at any speed: {bad:?}");
    assert!(bad.contains(&"no capture at 3 m/s"));
}

// ---- N3: Altitude <-> Position continuity ----

/// Two armed flights, one in Altitude all along, one changing between Altitude and Position every 400 steps, with a
/// throttle excursion and a climb-rate error that winds the vertical integrator. Returns whether the target height, the
/// vertical integrator and the thrust are bit-identical at every step, and how many steps the second was in Position.
fn n3_run(clear_flag: bool) -> (bool, u32, f32) {
    let tuning = Tuning::DEFAULT;
    let mk = || -> &'static Handshake {
        let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
        h.set_permit(true);
        h
    };
    let (ca, cb) = (
        Cell::new(pnav(0.0, 0.0, -2.0, [0.0, 0.0, 0.3], 0.4, POSITION)),
        Cell::new(pnav(0.0, 0.0, -2.0, [0.0, 0.0, 0.3], 0.4, POSITION)),
    );
    let (mut a, mut b) = (flight(&ca, mk(), &tuning), flight(&cb, mk(), &tuning));
    b.controller.position_clears_alt_active = clear_flag;
    let (mut ta, mut tb) = (1_000, 1_000);
    arm(&mut a, &mut ta, Switch3::Middle, 0.0);
    arm(&mut b, &mut tb, Switch3::Middle, 0.0);
    let (mut same, mut in_position, mut worst_i) = (true, 0, 0.0_f32);
    for k in 0..4_000u32 {
        let throttle = if (1_200..1_500).contains(&k) { 0.8 } else { HOVER };
        let switch_b = if (k / 400) % 2 == 1 { Switch3::High } else { Switch3::Middle };
        ta += DT_US;
        tb += DT_US;
        let oa = step_sticks(&mut a, ta, true, [0.0, 0.0, throttle, 0.0], Switch3::Middle);
        let ob = step_sticks(&mut b, tb, true, [0.0, 0.0, throttle, 0.0], switch_b);
        if ob.mode == Mode::Position {
            in_position += 1;
        }
        let (da, db) = (oa.demand.unwrap().thrust_n, ob.demand.unwrap().thrust_n);
        same &= da.to_bits() == db.to_bits()
            && a.controller.vertical_integrator().to_bits() == b.controller.vertical_integrator().to_bits()
            && a.shaper.target_d.map(f32::to_bits) == b.shaper.target_d.map(f32::to_bits);
        worst_i = worst_i.max(libm::fabsf(a.controller.vertical_integrator()));
    }
    (same, in_position, worst_i)
}

#[test]
fn n3_altitude_and_position_leave_the_target_height_and_the_vertical_integrator_bit_identical() {
    let (same, in_position, wound) = n3_run(false);
    std::println!("N3: {in_position} Position steps, vertical integrator reached {wound:.4} m/s^2, bit-identical: {same}");
    assert!(in_position >= 1_000 && wound > 0.05);
    assert!(same);
}

/// Negative control: the vertical integrator's live flag cleared in Position (what any other setpoint does).
#[test]
fn n3_control_clearing_alt_active_in_position_breaks_it() {
    let (same, in_position, _) = n3_run(true);
    assert!(in_position >= 1_000);
    assert!(!same);
}

#[test]
fn the_targets_live_only_in_their_modes() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(pnav(1.0, 2.0, -2.0, [0.0; 3], 0.0, POSITION));
    let mut t = 1_000;
    let mut f = airborne(&cell, &H, &tuning, &mut t);
    let held = f.shaper.target_d;
    assert!(f.shaper.target_ne.is_some() && held.is_some());
    // Position -> Altitude: the height target kept, the position target dropped.
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::Middle);
    assert_eq!(out.mode, Mode::Altitude);
    assert_eq!(f.shaper.target_d.map(f32::to_bits), held.map(f32::to_bits));
    assert!(f.shaper.target_ne.is_none());
    // Altitude -> Position: a fresh capture, the same height target.
    cell.set(pnav(1.5, 2.5, -2.0, [0.0; 3], 0.0, POSITION));
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    assert_eq!(out.mode, Mode::Position);
    assert_eq!(f.shaper.target_ne, Some([1.5, 2.5]));
    assert_eq!(f.shaper.target_d.map(f32::to_bits), held.map(f32::to_bits));
    // Position -> Angle: both dropped.
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::Low);
    assert_eq!(out.mode, Mode::Angle);
    assert!(f.shaper.target_ne.is_none() && f.shaper.target_d.is_none());
}

// ---- N5: frames, through the shaper and the controller ----

/// Full stick in each of four directions at each of four headings: the velocity feed-forward and the direction of the
/// acceleration the tilt gives (the truth table is worked out by hand: forward is along the nose, right is 90 degrees
/// clockwise seen from above).
#[test]
fn n5_frames_truth_table_four_headings_four_stick_directions() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let v = tuning.pilot_speed_xy_ms;
    let (n, s, e, w) = ([v, 0.0], [-v, 0.0], [0.0, v], [0.0, -v]);
    // (heading, rad) -> (forward, back, right, left) in NED.
    let table: [(&str, f32, [[f32; 2]; 4]); 4] = [
        ("north", 0.0, [n, s, e, w]),
        ("east", core::f32::consts::FRAC_PI_2, [e, w, s, n]),
        ("south", core::f32::consts::PI, [s, n, w, e]),
        ("west", -core::f32::consts::FRAC_PI_2, [w, e, n, s]),
    ];
    // (roll, pitch) of forward, back, right, left.
    let sticks = [(0.0, 1.0), (0.0, -1.0), (1.0, 0.0), (-1.0, 0.0)];
    for (name, yaw, want) in table {
        for (k, &(roll, pitch)) in sticks.iter().enumerate() {
            let cell = Cell::new(pnav(0.0, 0.0, -2.0, [0.0; 3], yaw, POSITION));
            let mut t = 1_000;
            let mut f = airborne(&cell, &H, &tuning, &mut t);
            t += DT_US;
            let out = step_sticks(&mut f, t, false, [roll, pitch, HOVER, 0.0], Switch3::High);
            let (_, vff, heading) = position_of(&out);
            near(vff[0], want[k][0], 1e-4);
            near(vff[1], want[k][1], 1e-4);
            // The heading of the setpoint is the estimate's yaw (as Angle).
            near(libm::cosf(heading), libm::cosf(yaw), 1e-5);
            near(libm::sinf(heading), libm::sinf(yaw), 1e-5);
            // The tilt accelerates the same way.
            let a = accel_ne(out.attitude_sp);
            let (an, wn) = (libm::sqrtf(a[0] * a[0] + a[1] * a[1]), v);
            assert!(an > 1.0, "{name} {k}: |a| {an}");
            near((a[0] * want[k][0] + a[1] * want[k][1]) / (an * wn), 1.0, 1e-3);
        }
    }
}

#[test]
fn the_dead_zone_and_the_diagonal_scale() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let dz = tuning.pilot_xy_dz;
    let cell = Cell::new(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
    let mut t = 1_000;
    let mut f = airborne(&cell, &H, &tuning, &mut t);
    let ff = |f: &mut TestFlight, t: &mut u64, roll: f32, pitch: f32| {
        *t += DT_US;
        let out = step_sticks(f, *t, false, [roll, pitch, HOVER, 0.0], Switch3::High);
        let v = position_of(&out).1;
        (v[0], v[1])
    };
    // Inside, at the edge: nothing, and the capture holds (the sticks count as centred).
    assert_eq!(ff(&mut f, &mut t, dz, -dz), (0.0, 0.0));
    assert!(f.shaper.target_ne.is_some());
    // Just outside: small, of the stick's sign, the target released.
    let (n, e) = ff(&mut f, &mut t, -(dz + 0.01), dz + 0.01);
    assert!(n > 0.0 && n < 0.02 * tuning.pilot_speed_xy_ms && e < 0.0 && e > -0.02 * tuning.pilot_speed_xy_ms);
    assert!(f.shaper.target_ne.is_none());
    // Full travel is the full speed; half of the travel above the dead zone is half.
    let v = tuning.pilot_speed_xy_ms;
    let (n, _) = ff(&mut f, &mut t, 0.0, 1.0);
    near(n, v, 1e-5);
    let (n, _) = ff(&mut f, &mut t, 0.0, dz + 0.5 * (1.0 - dz));
    near(n, 0.5 * v, 1e-5);
    // Both sticks full: scaled to hypot 1, so the speed is the limit, not 1.41 times it.
    let (n, e) = ff(&mut f, &mut t, 1.0, 1.0);
    near(libm::sqrtf(n * n + e * e), v, 1e-4);
    near(n, e, 1e-5);
}

// ---- N9: a step with a reset delta is not Position ----

/// Position steady; the step with the NE flags false (what an NE reset delta gives for exactly that step, eskf_out's
/// `t5b_the_flags_of_the_fields_a_reset_moved_are_false_for_exactly_that_step`); the next step the position is that of
/// the new frame. Returns the failures.
fn n9_failures(ignore_cov: bool) -> Vec<&'static str> {
    static H0: Handshake = Handshake::new();
    static H1: Handshake = Handshake::new();
    let h = if ignore_cov { &H1 } else { &H0 };
    let tuning = Tuning::DEFAULT;
    let mut bad = Vec::new();
    let mut check = |ok: bool, what: &'static str| {
        if !ok {
            bad.push(what);
        }
    };
    let cell = Cell::new(pnav(3.0, 4.0, -2.0, [0.0; 3], 0.0, POSITION));
    let mut t = 1_000;
    let mut f = airborne(&cell, h, &tuning, &mut t);
    t += DT_US;
    step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    check(f.shaper.target_ne == Some([3.0, 4.0]), "locked at the first position");
    f.ignore_coverage = ignore_cov;
    // The reset: the estimate moves by (-1.5, +2.0) and both NE flags are false for that step.
    for lost in [
        Valid { position_ne: false, velocity_ne: false, ..POSITION },
        Valid { heading: false, ..POSITION },
    ] {
        cell.set(pnav(1.5, 6.0, -2.0, [0.0; 3], 0.0, lost));
        t += DT_US;
        let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        check(out.mode != Mode::Position, "the reset step is not Position");
        check(f.shaper.target_ne.is_none(), "the reset step drops the target");
        cell.set(pnav(1.5, 6.0, -2.0, [0.0; 3], 0.0, POSITION));
        t += DT_US;
        let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
        check(out.mode == Mode::Position, "Position again the next step");
        check(f.shaper.target_ne == Some([1.5, 6.0]), "re-captured at the new position, not the old target");
    }
    bad
}

#[test]
fn n9_a_reset_step_is_not_position_and_drops_the_target() {
    let bad = n9_failures(false);
    assert!(bad.is_empty(), "{bad:?}");
}

/// Negative control: coverage ignored, the reset step stays Position with the old target.
#[test]
fn n9_control_ignoring_coverage_fails() {
    let bad = n9_failures(true);
    std::println!("coverage ignored: {bad:?}");
    assert!(bad.contains(&"the reset step is not Position"));
}

// ---- N10: no NE or heading field read while its flag is false ----

/// Two flights fed the same sequence, one with huge finite junk in the NE position and velocity while their flags are
/// false: their outputs are bit-identical in every selected mode. Returns whether they are.
fn n10_same(ignore_cov: bool) -> bool {
    let tuning = Tuning::DEFAULT;
    let flags = Valid { position_ne: false, velocity_ne: false, ..POSITION };
    let mut all_same = true;
    for switch in [Switch3::Low, Switch3::Middle, Switch3::High] {
        let mk = |junk: f32| {
            let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
            h.set_permit(true);
            let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(pnav(
                junk, -junk, -2.0, [junk, junk, 0.0], 0.0, flags,
            ))));
            let mut f = flight(cell, h, &tuning);
            f.ignore_coverage = ignore_cov;
            (cell, f)
        };
        let ((_, mut a), (_, mut b)) = (mk(0.0), mk(1.0e6));
        let (mut ta, mut tb) = (1_000, 1_000);
        arm(&mut a, &mut ta, switch, 0.0);
        arm(&mut b, &mut tb, switch, 0.0);
        for k in 0..400u32 {
            let throttle = if k < 5 { 0.2 } else { HOVER };
            ta += DT_US;
            tb += DT_US;
            let oa = step_sticks(&mut a, ta, true, [0.1, -0.2, throttle, 0.05], switch);
            let ob = step_sticks(&mut b, tb, true, [0.1, -0.2, throttle, 0.05], switch);
            // The estimate itself differs (that is the junk): compare everything the controller made.
            all_same &= std::format!("{:?}{:?}{:?}{:?}", oa.setpoint, oa.demand, oa.allocation, oa.mode)
                == std::format!("{:?}{:?}{:?}{:?}", ob.setpoint, ob.demand, ob.allocation, ob.mode);
        }
    }
    all_same
}

#[test]
fn n10_no_ne_field_is_read_while_its_flag_is_false() {
    assert!(n10_same(false));
}

/// Negative control: with coverage ignored, Position reads the junk.
#[test]
fn n10_control_ignoring_coverage_reads_the_junk() {
    assert!(!n10_same(true));
}

// ---- P9: the speed parameter, latched at arming ----

#[test]
fn the_speed_parameter_range_is_enforced_by_the_flight_core_too() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ALTITUDE));
    let mut f = flight(&cell, &H, &tuning);
    assert_eq!((PILOT_SPEED_XY_MIN_MS, PILOT_SPEED_XY_MAX_MS), (3.0, 20.0));
    assert_eq!(tuning.pilot_speed_xy_ms, 5.0);
    for ok in [3.0, 5.0, 12.5, 20.0] {
        assert!(f.set_pilot_speed_xy(ok), "{ok}");
        assert_eq!(f.speed_pending, ok);
    }
    for bad in [2.999, 20.001, 0.0, -5.0, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(!f.set_pilot_speed_xy(bad), "{bad}");
        assert_eq!(f.speed_pending, 20.0, "{bad} left the value alone");
    }
}

/// The speed in force (shaper and controller) at the end of a script: set 8 while disarmed, arm, try 12 while armed,
/// disarm, then 12 takes effect at the next arming. `latch_off` is the negative control.
fn latch_run(latch_off: bool) -> Vec<(&'static str, f32, f32)> {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
    let mut f = flight(&cell, h, &tuning);
    f.latch_off = latch_off;
    let mut seen = Vec::new();
    let mut t = 1_000;
    fn note(seen: &mut Vec<(&'static str, f32, f32)>, f: &TestFlight, what: &'static str) {
        seen.push((what, f.shaper.pilot_speed_xy_ms, f.controller.speed_limit_xy()));
    }
    t += DT_US;
    step_sticks(&mut f, t, false, [0.0, 0.0, 0.0, 0.0], Switch3::Low);
    note(&mut seen, &f, "default");
    assert!(f.set_pilot_speed_xy(8.0));
    t += DT_US;
    step_sticks(&mut f, t, false, [0.0, 0.0, 0.0, 0.0], Switch3::Low);
    note(&mut seen, &f, "set 8 disarmed");
    // Arm in Position at the floor, then up to the hover: Position.
    arm(&mut f, &mut t, Switch3::High, 0.0);
    note(&mut seen, &f, "armed");
    t += DT_US;
    step_sticks(&mut f, t, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    t += DT_US;
    let out = step_sticks(&mut f, t, true, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    assert_eq!(out.mode, Mode::Position);
    t += DT_US;
    let out = step_sticks(&mut f, t, true, [0.0, 1.0, HOVER, 0.0], Switch3::High);
    seen.push(("full stick while armed", position_of(&out).1[0], 0.0));
    assert!(f.set_pilot_speed_xy(12.0));
    t += DT_US;
    let out = step_sticks(&mut f, t, true, [0.0, 1.0, HOVER, 0.0], Switch3::High);
    seen.push(("full stick, 12 set while armed", position_of(&out).1[0], 0.0));
    note(&mut seen, &f, "12 set while armed");
    // Disarm (the switch off), then the next disarmed step latches 12; arm again.
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, 0.0, 0.0], Switch3::High);
    assert!(!out.armed);
    t += DT_US;
    step_sticks(&mut f, t, false, [0.0, 0.0, 0.0, 0.0], Switch3::High);
    note(&mut seen, &f, "disarmed again");
    arm(&mut f, &mut t, Switch3::High, 0.0);
    note(&mut seen, &f, "armed again");
    seen
}

#[test]
fn the_speed_is_latched_at_arming_and_does_not_change_while_armed() {
    let seen = latch_run(false);
    std::println!("{seen:?}");
    let get = |what: &str| seen.iter().find(|s| s.0 == what).copied().unwrap();
    assert_eq!(get("default").1, 5.0);
    assert_eq!(get("set 8 disarmed").1, 8.0);
    assert_eq!(get("armed").1, 8.0);
    assert_eq!(get("armed").2, 8.0);
    near(get("full stick while armed").1, 8.0, 1e-5);
    near(get("full stick, 12 set while armed").1, 8.0, 1e-5);
    assert_eq!((get("12 set while armed").1, get("12 set while armed").2), (8.0, 8.0));
    assert_eq!((get("disarmed again").1, get("disarmed again").2), (12.0, 12.0));
    assert_eq!((get("armed again").1, get("armed again").2), (12.0, 12.0));
}

/// Negative control: with the latch off the value changes in flight.
#[test]
fn control_without_the_latch_the_speed_changes_while_armed() {
    let seen = latch_run(true);
    let get = |what: &str| seen.iter().find(|s| s.0 == what).copied().unwrap();
    std::println!("{seen:?}");
    assert_eq!(get("12 set while armed").1, 12.0);
}

// ---- P5: the log ----

#[test]
fn the_log_in_position_spn_setp_kind_3_and_mot_code_3() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(pnav(1.25, -0.5, -3.0, [0.0; 3], 0.5, POSITION));
    let mut t = 1_000;
    let mut f = airborne(&cell, &H, &tuning, &mut t);
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.3, 0.6, 0.6, 0.2], Switch3::High);
    assert_eq!(out.mode, Mode::Position);
    let (p, v, hdg) = position_of(&out);
    let f32_at = |b: &[u8], at: usize| f32::from_le_bytes(b[at..at + 4].try_into().unwrap());
    // SPN: PN, PE, PD, VN, VE, VD, Hdg.
    let mut b = [0u8; 39];
    assert_eq!(encode_spn(&out, &mut b), 39);
    let fields: Vec<f32> = (0..7).map(|i| f32_at(&b, 11 + 4 * i)).collect();
    assert_eq!(fields, [p[0], p[1], p[2], v[0], v[1], v[2], hdg]);
    assert!(fields[..2] == [1.25, -0.5], "unlocked (the stick moved): the position is the estimate's");
    // SETP kind 3 with the controller's attitude setpoint; MOT code 3.
    let mut buf = [0u8; 200];
    let n = encode_step(&out, &mut buf);
    assert_eq!(n, 40 + 44 + 29 + 29);
    let setp = usize::from(dflog::NAV.length);
    // SETP: header 3, TimeUS 8, Kind 1, then Q1..Q4.
    assert_eq!(buf[setp + 11], 3);
    let q: Vec<f32> = (0..4).map(|i| f32_at(&buf[setp..], 12 + 4 * i)).collect();
    assert_eq!(q, out.attitude_sp);
    assert!(q.iter().any(|x| *x != 0.0));
    let norm = q.iter().map(|x| x * x).sum::<f32>();
    near(norm, 1.0, 1e-5);
    assert_eq!(mot_mode_byte(&out), 3);
    // Altitude's kind 2 is unchanged, with zero attitude columns.
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::Middle);
    assert_eq!(out.attitude_sp, [0.0; 4]);
    let n = encode_step(&out, &mut buf);
    assert_eq!(n, 40 + 44 + 29 + 29);
    assert_eq!(buf[setp + 11], 2);
    assert!((0..4).all(|i| f32_at(&buf[setp..], 12 + 4 * i) == 0.0));
}

/// The step's setpoint tilts at most `angle_max_rad`, finite, at every step of a scripted flight with sticks and a
/// moving estimate (c7's host form).
#[test]
fn the_position_tilt_setpoint_never_exceeds_the_limit() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
    let mut t = 1_000;
    let mut f = airborne(&cell, &H, &tuning, &mut t);
    let mut worst = 0.0_f32;
    for k in 0..6_000u32 {
        let s = k as f32 * 0.01;
        cell.set(pnav(20.0 * libm::sinf(0.3 * s), 15.0 * libm::cosf(0.2 * s), -2.0, [9.0 * libm::cosf(0.3 * s), -2.0, 0.0], 0.1 * s, POSITION));
        t += DT_US;
        let sticks = [libm::sinf(0.5 * s), libm::cosf(0.37 * s), HOVER, 0.0];
        let out = step_sticks(&mut f, t, false, sticks, Switch3::High);
        if out.mode == Mode::Position {
            let [w, x, y, z] = out.attitude_sp;
            worst = worst.max(libm::acosf((w * w - x * x - y * y + z * z).min(1.0)));
            assert!(out.attitude_sp.iter().all(|v| v.is_finite()));
        }
    }
    std::println!("peak tilt setpoint {:.4} deg (limit {:.4})", worst.to_degrees(), tuning.angle_max_rad.to_degrees());
    assert!(worst > 0.1 && worst <= tuning.angle_max_rad + 1e-5);
}

