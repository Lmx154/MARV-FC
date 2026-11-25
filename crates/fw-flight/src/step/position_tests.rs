//! Host tests of position hold at the step level.
//! And the log fill)

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
            source: fw_core::rc::RcSource::Crsf,
            mission: false,
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
    f.airborne_forced = true;
    *t += DT_US;
    let out = step_sticks(&mut f, *t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    assert_eq!(out.mode, Mode::Altitude, "not airborne at construction");
    *t += DT_US;
    let out = step_sticks(&mut f, *t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    assert_eq!(out.mode, Mode::Position);
    f
}

// ---- Position only when covered and airborne ----

/// The coverage part: each flag of `Setpoint::Position.requires()` dropped for one step, in a flight with the latch
/// forced (disarmed steps, so the flags are the only variable). `ignore_cov` is the negative control.
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

/// Sets the fake estimate so that the vertical loop asks `frac` x m g in the next step.
/// the height at the target, tilt 0)
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

/// The latch table on an armed flight in Position.
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
    let out = armed_step(&mut f, &cell, &mut t, 1.0);
    check(out.mode != Mode::Position && f.airborne, "true after the step that first reaches the threshold");
    let out = armed_step(&mut f, &cell, &mut t, 1.0);
    check(out.mode == Mode::Position, "Position the step after");
    let mut ok = true;
    for _ in 0..thirty {
        let out = armed_step(&mut f, &cell, &mut t, 0.7);
        ok &= f.airborne && out.mode == Mode::Position;
    }
    check(ok, "once set, 30 s between the floor and the threshold is still true");
    if !f.airborne {
        for _ in 0..3 {
            armed_step(&mut f, &cell, &mut t, 1.0);
        }
    }
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
    armed_step(&mut f, &cell, &mut t, 0.0);
    check(!f.airborne, "false at land_detect_us");
    let out = armed_step(&mut f, &cell, &mut t, 0.0);
    check(out.mode == Mode::Altitude, "Altitude after the latch clears");
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

/// Negative control: the previous rule (low below the ground threshold) fails the 30 s between the floor and the
/// threshold.
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

// ---- No blip ----

/// Armed in Position with a velocity error that winds the horizontal integrator and a locked target; the collective:
/// the weight for 0.25 s, a forced 0.5 s dip to the floor, the weight for 0.25 s.
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

// ---- Armed in Position on the ground, the landing and the aborted take-off ----

/// (a) Armed in Position at rest on the ground for 30 s with the estimate wandering by 0.5 m, the collective on the
/// floor: Altitude, the horizontal integrator 0. (b) Then up (airborne), then the floor for `land_detect_us`.
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

/// Negative control: a latch that never clears fails (b) and (c)
#[test]
fn n6_control_a_latch_that_never_clears_fails_the_landing() {
    let (a_pos, _, b_after, c_pos, c_until) = n6_run(true);
    std::println!("N6 control: (a) {a_pos}, (b) {b_after} Position steps after the clear, (c) {c_pos}, Altitude from {c_until}");
    assert_eq!(a_pos, 0, "(a) does not depend on the timer");
    assert!(b_after > 0, "(b) fails");
    assert_eq!(c_until, 0, "(c) never reaches Altitude");
}

/// Landed in Angle at part throttle (the latch stays set), then the switch to Position with the stick low; then the vertical loop.
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
    cell.set(pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION));
    for _ in 0..(5_000_000 / DT_US) {
        t += DT_US;
        let out = step_sticks(&mut f, t, true, [0.0, 0.0, 0.3, 0.0], Switch3::Low);
        assert_eq!(out.mode, Mode::Angle);
        assert!(f.airborne, "the latch stays set in Angle at part throttle");
    }
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

// ---- Never Position on the arming step ----

/// Disarmed in Position with the collective up, then the arming step with the throttle at the floor; the latch is
/// false after every disarmed step. `control`: the latch forced true at every step.
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

// ---- The capture ----

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
    t += DT_US;
    let out = step_sticks(&mut f, t, false, centred, Switch3::High);
    let (p, v, _) = position_of(&out);
    check(f.shaper.target_ne.is_none(), "no capture at 3 m/s");
    check(p[0] == 10.0 && p[1] == 0.0, "unlocked: the target is this step's position");
    check(v[0] == 0.0 && v[1] == 0.0, "no feed-forward with the sticks centred");
    cell.set(pnav(11.5, 0.2, -2.0, [hold, 0.0, 0.0], 0.0, POSITION));
    t += DT_US;
    step_sticks(&mut f, t, false, centred, Switch3::High);
    check(f.shaper.target_ne.is_none(), "no capture at the hold speed itself");
    cell.set(pnav(12.3, 4.5, -2.0, [hold * 0.999, 0.0, 0.0], 0.0, POSITION));
    t += DT_US;
    let out = step_sticks(&mut f, t, false, centred, Switch3::High);
    check(f.shaper.target_ne == Some([12.3, 4.5]), "captured at the capture step's N, E");
    check(position_of(&out).0[..2] == [12.3, 4.5], "the setpoint carries the capture");
    cell.set(pnav(13.0, 5.0, -2.0, [0.05, 0.0, 0.0], 0.0, POSITION));
    t += DT_US;
    let out = step_sticks(&mut f, t, false, centred, Switch3::High);
    check(position_of(&out).0[..2] == [12.3, 4.5], "held while the sticks are centred");
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.55, HOVER, 0.0], Switch3::High);
    let (p, v, _) = position_of(&out);
    check(f.shaper.target_ne.is_none(), "released when the sticks move");
    check(p[0] == 13.0 && p[1] == 5.0, "unlocked: position error exactly 0");
    check((v[0] - 0.5 * tuning.pilot_speed_xy_ms).abs() <= 1e-5 && v[1] == 0.0, "the velocity feed-forward");
    check(p.iter().chain(v.iter()).all(|x| x.is_finite()), "finite");
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

// ---- Altitude <-> Position continuity ----

/// Two armed flights, one in Altitude all along, one changing between Altitude and Position every 400 steps, with a
/// throttle excursion and a climb-rate error that winds the vertical integrator.
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

/// Negative control: the vertical integrator's live flag cleared in Position (what any other setpoint does)
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
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::Middle);
    assert_eq!(out.mode, Mode::Altitude);
    assert_eq!(f.shaper.target_d.map(f32::to_bits), held.map(f32::to_bits));
    assert!(f.shaper.target_ne.is_none());
    cell.set(pnav(1.5, 2.5, -2.0, [0.0; 3], 0.0, POSITION));
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::High);
    assert_eq!(out.mode, Mode::Position);
    assert_eq!(f.shaper.target_ne, Some([1.5, 2.5]));
    assert_eq!(f.shaper.target_d.map(f32::to_bits), held.map(f32::to_bits));
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::Low);
    assert_eq!(out.mode, Mode::Angle);
    assert!(f.shaper.target_ne.is_none() && f.shaper.target_d.is_none());
}

// ---- Frames, through the shaper and the controller ----

/// Full stick in each of four directions at each of four headings: the velocity feed-forward and the direction of the
/// Acceleration the tilt gives.
#[test]
fn n5_frames_truth_table_four_headings_four_stick_directions() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let v = tuning.pilot_speed_xy_ms;
    let (n, s, e, w) = ([v, 0.0], [-v, 0.0], [0.0, v], [0.0, -v]);
    let table: [(&str, f32, [[f32; 2]; 4]); 4] = [
        ("north", 0.0, [n, s, e, w]),
        ("east", core::f32::consts::FRAC_PI_2, [e, w, s, n]),
        ("south", core::f32::consts::PI, [s, n, w, e]),
        ("west", -core::f32::consts::FRAC_PI_2, [w, e, n, s]),
    ];
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
            near(libm::cosf(heading), libm::cosf(yaw), 1e-5);
            near(libm::sinf(heading), libm::sinf(yaw), 1e-5);
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
    assert_eq!(ff(&mut f, &mut t, dz, -dz), (0.0, 0.0));
    assert!(f.shaper.target_ne.is_some());
    let (n, e) = ff(&mut f, &mut t, -(dz + 0.01), dz + 0.01);
    assert!(n > 0.0 && n < 0.02 * tuning.pilot_speed_xy_ms && e < 0.0 && e > -0.02 * tuning.pilot_speed_xy_ms);
    assert!(f.shaper.target_ne.is_none());
    let v = tuning.pilot_speed_xy_ms;
    let (n, _) = ff(&mut f, &mut t, 0.0, 1.0);
    near(n, v, 1e-5);
    let (n, _) = ff(&mut f, &mut t, 0.0, dz + 0.5 * (1.0 - dz));
    near(n, 0.5 * v, 1e-5);
    let (n, e) = ff(&mut f, &mut t, 1.0, 1.0);
    near(libm::sqrtf(n * n + e * e), v, 1e-4);
    near(n, e, 1e-5);
}

/// The yaw stick's dead zone is the same in Angle, Altitude and Position: +-0.025 commands exactly zero, full stick
/// the full rate. `dz` is the control when 0.
fn yaw_rates(dz: f32, yaw: f32) -> [f32; 3] {
    static H: Handshake = Handshake::new();
    let mut tuning = Tuning::DEFAULT;
    tuning.pilot_y_dz = dz;
    let nav = pnav(0.0, 0.0, -2.0, [0.0; 3], 0.0, POSITION);
    let cell = Cell::new(nav);
    let mut f = flight(&cell, &H, &tuning);
    f.shaper.sticks = Sticks { roll: 0.0, pitch: 0.0, throttle: HOVER, yaw };
    [Mode::Angle, Mode::Altitude, Mode::Position].map(|mode| {
        match f.shaper.setpoint(mode, &nav, 1_000) {
            Setpoint::Attitude { heading_rate_rads, .. }
            | Setpoint::Altitude { heading_rate_rads, .. }
            | Setpoint::Position { heading_rate_rads, .. } => heading_rate_rads,
            _ => panic!("unexpected setpoint"),
        }
    })
}

#[test]
fn the_yaw_dead_zone_is_the_same_in_every_mode() {
    let full = Tuning::DEFAULT.pilot_y_rate_rads;
    let dz = Tuning::DEFAULT.pilot_y_dz;
    for y in [0.025, -0.025, dz, -dz] {
        assert_eq!(yaw_rates(dz, y), [0.0; 3], "yaw {y}");
    }
    for r in yaw_rates(dz, 1.0) {
        near(r, full, 1e-5);
    }
    for r in yaw_rates(dz, -1.0) {
        near(r, -full, 1e-5);
    }
    for r in yaw_rates(dz, dz + 0.01) {
        assert!(r > 0.0 && r < 0.02 * full, "{r}");
    }
    for r in yaw_rates(0.0, 0.025) {
        near(r, 0.025 * full, 1e-6);
    }
    for r in yaw_rates(0.0, -0.025) {
        assert!(r < 0.0);
    }
}

// ---- A step with a reset delta is not Position ----

/// Position steady; the step with the NE flags false.
/// `t5b_the_flags_of_the_fields_a_reset_moved_are_false_for_exactly_that_step`)
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

// ---- No NE or heading field read while its flag is false ----

/// Two flights fed the same sequence, one with huge finite junk in the NE position and velocity while their flags are
/// false: their outputs are bit-identical in every selected mode.
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

// ---- The speed parameter, latched at arming ----

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

// ---- The log ----

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
    let mut b = [0u8; 39];
    assert_eq!(encode_spn(&out, &mut b), 39);
    let fields: Vec<f32> = (0..7).map(|i| f32_at(&b, 11 + 4 * i)).collect();
    assert_eq!(fields, [p[0], p[1], p[2], v[0], v[1], v[2], hdg]);
    assert!(fields[..2] == [1.25, -0.5], "unlocked (the stick moved): the position is the estimate's");
    let mut buf = [0u8; 300];
    let n = encode_step(&out, &mut buf);
    assert_eq!(n, 40 + 44 + 29 + 29 + 59 + 40);
    let setp = usize::from(dflog::NAV.length);
    assert_eq!(buf[setp + 11], 3);
    let q: Vec<f32> = (0..4).map(|i| f32_at(&buf[setp..], 12 + 4 * i)).collect();
    assert_eq!(q, out.attitude_sp);
    assert!(q.iter().any(|x| *x != 0.0));
    let norm = q.iter().map(|x| x * x).sum::<f32>();
    near(norm, 1.0, 1e-5);
    assert_eq!(mot_mode_byte(&out), 3);
    t += DT_US;
    let out = step_sticks(&mut f, t, false, [0.0, 0.0, HOVER, 0.0], Switch3::Middle);
    assert_eq!(out.attitude_sp, [0.0; 4]);
    let n = encode_step(&out, &mut buf);
    assert_eq!(n, 40 + 44 + 29 + 29 + 59 + 40);
    assert_eq!(buf[setp + 11], 2);
    assert!((0..4).all(|i| f32_at(&buf[setp..], 12 + 4 * i) == 0.0));
}

/// The step's setpoint tilts at most `angle_max_rad`, finite, at every step of a scripted flight with sticks and a
/// moving estimate (c7's host form)
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


// ---- The shadow guidance does not touch the flight ----

/// What one G2 run found: steps whose `StepOut` or whose encoded records differ between the two flights, steps in
/// Position, steps where guidance made a setpoint, and the last target index it was at.
struct G2 {
    differ: usize,
    position: usize,
    guided: usize,
    last_active: u8,
}

/// The inputs of a vehicle that moves north at 2 m/s (the P-2 run of `step.rs`): armed at 2 s, Position selected, GNSS
/// with velocity.
fn g2_inputs(secs: u64) -> Vec<FlightInputs> {
    use crate::eskf::tests::{PERIOD_US, World};
    let mut world = World::new(&Tuning::DEFAULT);
    world.yaw = 0.4;
    world.v = [2.0, 0.0, 0.0];
    world.gps_vel = true;
    (0..secs * 1_000_000 / PERIOD_US)
        .map(|k| {
            let sensors = world.step();
            let t_us = sensors.imu1.t_us;
            let rc = (k % 10 == 0).then_some(RcSample {
                sticks: Sticks { roll: 0.0, pitch: 0.0, throttle: if t_us < 3_000_000 { 0.0 } else { 0.5 }, yaw: 0.0 },
                arm: t_us >= 2_000_000,
                switch6: Switch3::High,
                source: fw_core::rc::RcSource::Crsf,
                mission: false,
                t_us,
            });
            FlightInputs { sensors, rc, erpm: [None; LINES] }
        })
        .collect()
}

/// Two `Flight`s with the real steering estimator get the same inputs (`b_inputs` for the second); beside the first a
/// `Guidance` is anchored and stepped every step from copies of its `nav`.
fn g2_run(handshakes: &'static [Handshake; 2], a_inputs: &[FlightInputs], b_inputs: &[FlightInputs]) -> G2 {
    use crate::alloc::CardAllocator;
    use crate::eskf::Eskf;
    use crate::eskf::tests::{ORIGIN, geo};
    use crate::eskf_out::EskfSteer;
    use crate::gnc::GeoPoint;
    use crate::guidance::{Guidance, status};
    use crate::pid::CascadedPid;
    let tuning = Tuning::DEFAULT;
    let mut cores = [std::boxed::Box::new(Eskf::new(&tuning).unwrap()), std::boxed::Box::new(Eskf::new(&tuning).unwrap())];
    let [ca, cb] = &mut cores;
    let mut a = Flight::new(EskfSteer::new(ca, &tuning).unwrap(), CascadedPid::new(&tuning, 800.0), CardAllocator::new(), &tuning, &handshakes[0]);
    let mut b = Flight::new(EskfSteer::new(cb, &tuning).unwrap(), CascadedPid::new(&tuning, 800.0), CardAllocator::new(), &tuning, &handshakes[1]);
    handshakes[0].set_permit(true);
    handshakes[1].set_permit(true);
    a.airborne_forced = true;
    b.airborne_forced = true;
    let target = |n: f64, e: f64| {
        let (lat_e7, lon_e7) = geo([n, e, 0.0]);
        GeoPoint { lat_e7, lon_e7, alt_mm: ORIGIN.alt_mm }
    };
    let mut g = Guidance::<4>::new();
    g.set_targets(&[target(20.0, 0.0), target(20.0, 25.0)], 2.0);
    let mut r = G2 { differ: 0, position: 0, guided: 0, last_active: 0 };
    let (mut bytes_a, mut bytes_b) = ([[0u8; 512]; 3], [[0u8; 512]; 3]);
    for (ia, ib) in a_inputs.iter().zip(b_inputs) {
        let (oa, ob) = (a.step(ia), b.step(ib));
        let (nav, origin, speed) = (oa.nav, a.estimator().origin(), a.speed_xy_in_force());
        g.anchor(&nav);
        let go = g.step(&nav, origin, speed, ia.sensors.imu1.t_us);
        r.position += usize::from(oa.mode == Mode::Position);
        r.guided += usize::from(go.setpoint.is_some());
        r.last_active = r.last_active.max(go.active);
        let lens = [
            [encode_step(&oa, &mut bytes_a[0]), encode_spn(&oa, &mut bytes_a[1]), encode_mode(&oa, &mut bytes_a[2])],
            [encode_step(&ob, &mut bytes_b[0]), encode_spn(&ob, &mut bytes_b[1]), encode_mode(&ob, &mut bytes_b[2])],
        ];
        let same_bytes = lens[0] == lens[1] && (0..3).all(|i| bytes_a[i][..lens[0][i]] == bytes_b[i][..lens[1][i]]);
        r.differ += usize::from(oa != ob || !same_bytes);
        assert!(go.setpoint.is_none() || go.status & status::INVALID == 0);
    }
    r
}

/// The step outputs and the encoded STEP, SPN and MODE records are identical with and without guidance running beside
/// The flight, over a Position-hold run with the real estimator.
#[test]
fn g2_guidance_beside_the_flight_changes_no_step_output_and_no_record_byte() {
    static H: [Handshake; 4] = [Handshake::new(), Handshake::new(), Handshake::new(), Handshake::new()];
    let inputs = g2_inputs(14);
    let plain = g2_run(<&[Handshake; 2]>::try_from(&H[0..2]).unwrap(), &inputs, &inputs);
    std::println!(
        "g2: {} steps, {} in Position, guidance setpoints {}, last target {}, steps that differ {}",
        inputs.len(), plain.position, plain.guided, plain.last_active, plain.differ
    );
    assert!(plain.position > 3_000, "the flight was in Position for {} steps", plain.position);
    assert!(plain.guided > 3_000, "guidance made {} setpoints", plain.guided);
    assert_eq!(plain.last_active, 1, "guidance reached the second target");
    assert_eq!(plain.differ, 0, "{} steps differ with guidance beside the flight", plain.differ);

    let mut moved = inputs.clone();
    let at = moved.iter().position(|i| i.sensors.gnss.is_some_and(|g| g.t_us >= 8_000_000)).unwrap();
    let g = moved[at].sensors.gnss.as_mut().unwrap();
    g.reading.lat_e7 = g.reading.lat_e7.map(|l| l + 30);
    let control = g2_run(<&[Handshake; 2]>::try_from(&H[2..4]).unwrap(), &inputs, &moved);
    std::println!("g2 control: steps that differ {}", control.differ);
    assert!(control.differ > 0, "the perturbed GNSS epoch is not seen by the comparison");
}
