//! Host tests of crash rule K in shadow (decision-mavlink-mode-20261123, Amendment N, N5 and N7): the real step with the
//! fake estimator of `althold_tests`, FS_HOLD_EN 0, sticks present on every step (Angle, switch Low). IMU1 reads the
//! specific force at rest for the estimate's attitude plus `extra_ned` (so the filtered acceleration settles at it).

extern crate std;

use super::althold_tests::{ALTITUDE, ANGLE, DT_US, TestFlight, flight, input, nav_at};
use super::*;
use core::cell::Cell;

/// Steps in N5's 2.0 s at `DT_US` (the design's number, not `CRASH_TRIGGER_US`, so the control with that set to 0 builds).
const TRIGGER_STEPS: u32 = (2_000_000 / DT_US) as u32;

/// Velocity NE and D valid besides Angle's needs.
const VELOCITY: Valid = Valid {
    velocity_ne: true,
    ..ALTITUDE
};

/// Everything Position needs.
const POSITION: Valid = Valid {
    heading: true,
    position_ne: true,
    ..VELOCITY
};

/// A roll of `deg` degrees about the body x axis.
fn roll(deg: f32) -> [f32; 4] {
    let h = deg.to_radians() / 2.0;
    [libm::cosf(h), libm::sinf(h), 0.0, 0.0]
}

struct Crash {
    f: TestFlight<'static>,
    cell: &'static Cell<NavState>,
    t: u64,
    /// Added to the specific force at rest, NED, m/s^2: the filtered acceleration's input.
    extra_ned: [f32; 3],
    /// The mode switch of every report (Low: Angle).
    switch6: Switch3,
}

impl Crash {
    /// Armed with the throttle at 0 on the ground, coverage `valid`, FS_HOLD_EN 0.
    fn armed(valid: Valid) -> Crash {
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(-10.0, valid))));
        let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
        h.set_permit(true);
        let mut f = flight(cell, h, &Tuning::DEFAULT);
        f.set_fs_hold_en(false);
        let mut c = Crash { f, cell, t: 1_000_000, extra_ned: [0.0; 3], switch6: Switch3::Low };
        c.step(roll(0.0), [0.0; 3], false);
        c.step(roll(0.0), [0.0; 3], true);
        assert!(c.f.gate.armed(), "armed");
        c
    }

    /// As `armed`, then 0.5 s level at half throttle: ever airborne.
    fn flown(valid: Valid) -> Crash {
        let mut c = Crash::armed(valid);
        for _ in 0..400 {
            c.fly(roll(0.0), [0.0, 0.0, 0.5]);
        }
        assert!(c.f.airborne && c.f.hands_off.ever_airborne() && !c.f.crash_would_fire());
        c
    }

    /// One step, arm ON: the estimate's attitude `q`, sticks [roll, pitch, throttle]; K's status after it.
    fn fly(&mut self, q: [f32; 4], sticks: [f32; 3]) -> CrashStatus {
        let o = self.step(q, sticks, true);
        assert!(o.armed && o.motors.throttles().is_some() && self.f.hands_off_state() == State::Pilot, "K stops nothing");
        self.f.crash_status()
    }

    fn step(&mut self, q: [f32; 4], [roll, pitch, throttle]: [f32; 3], arm: bool) -> StepOut {
        self.t += DT_US;
        let mut n = self.cell.get();
        n.attitude = q;
        self.cell.set(n);
        // f_body = R(q)^T (-g down + extra), so that the earth-frame acceleration with gravity removed is `extra_ned`.
        let [e0, e1, e2] = self.extra_ned;
        let [_, fx, fy, fz] = quat::mul(quat::mul(quat::conj(q), [0.0, e0, e1, e2 - GRAVITY_MSS]), q);
        let mut i = input(self.t, Some(RcSample { sticks: Sticks { roll, pitch, throttle, yaw: 0.0 }, arm, switch6: self.switch6, source: fw_core::rc::RcSource::Crsf, mission: false, t_us: self.t }));
        i.sensors.imu1.reading.accel_ms2 = [fx, fy, fz];
        self.f.step(&i)
    }

    fn set_velocity(&self, v: [f32; 3]) {
        let mut n = self.cell.get();
        n.velocity_ms = v;
        self.cell.set(n);
    }

    fn set_rates(&self, r: [f32; 3]) {
        let mut n = self.cell.get();
        n.rates_rads = r;
        self.cell.set(n);
    }

    /// `steps` steps at attitude `q` and sticks `sticks`; whether K would fire after any of them.
    fn fires_within(&mut self, steps: u32, q: [f32; 4], sticks: [f32; 3]) -> bool {
        (0..steps).fold(false, |any, _| self.fly(q, sticks).would_fire() || any)
    }
}

/// The named cosines are those of ArduPilot's 15 and 30 deg.
#[test]
fn k_constants_are_arducopters() {
    assert!((CRASH_COS_LEAN_MIN - libm::cosf(15f32.to_radians())).abs() < 1e-7);
    assert!((CRASH_COS_ERROR_MIN - libm::cosf(30f32.to_radians())).abs() < 1e-7);
    assert_eq!((CRASH_TRIGGER_US, CRASH_ACCEL_MAX_MSS, CRASH_SPEED_MAX_MS), (2_000_000, 3.0, 10.0));
}

/// N7: upside down at rest after a flight (lean 180, the setpoint level, the throttle still at half so TD's (a) fails):
/// K would fire after exactly 2.0 s of steps, not after 1.99 s; with the four terms set.
#[test]
fn k_upside_down_fires_at_2_0_s_not_at_1_99_s() {
    let mut c = Crash::flown(ANGLE);
    for k in 1..=TRIGGER_STEPS {
        let s = c.fly(roll(180.0), [0.0, 0.0, 0.5]);
        assert!(s.lean && s.error && s.accel && s.speed, "step {k}: {s:?}");
        assert_eq!(s.held_us, u64::from(k) * DT_US);
        if k == TRIGGER_STEPS - 8 {
            assert!(!s.would_fire(), "1.99 s");
        }
        assert_eq!(s.would_fire(), k == TRIGGER_STEPS, "step {k}");
    }
    assert!(c.fly(roll(180.0), [0.0, 0.0, 0.5]).would_fire(), "stays");
}

/// N7: on its side (90 deg) and caught tilted 40 deg against something, the controller commanding level and no
/// acceleration: K would fire at 2.0 s; with the velocity valid and at rest too.
#[test]
fn k_on_its_side_and_caught_tilted_40_deg_fire() {
    for (deg, valid) in [(90.0, ANGLE), (40.0, ANGLE), (-40.0, VELOCITY), (90.0, VELOCITY)] {
        let mut c = Crash::flown(valid);
        assert!(!c.fires_within(TRIGGER_STEPS - 1, roll(deg), [0.0, 0.0, 0.5]), "{deg} deg: not before 2.0 s");
        assert!(c.fly(roll(deg), [0.0, 0.0, 0.5]).would_fire(), "{deg} deg, {valid:?}: at 2.0 s");
    }
    // Altitude (the setpoint's attitude) and Position (the controller's attitude setpoint), sticks centred: tilted 40 deg
    // K would fire at 2.0 s. At 90 deg their tilt factor (pid.rs `tilt_factor`, 0 at or past sideways) puts the collective
    // under the ground threshold, so TD holds at 1.0 s and K, which needs TD not held, never fires.
    for (switch6, valid, mode) in [(Switch3::Middle, ALTITUDE, Mode::Altitude), (Switch3::High, POSITION, Mode::Position)] {
        for deg in [40.0, 90.0] {
            let mut c = Crash::flown(valid);
            c.switch6 = switch6;
            for _ in 0..80 {
                c.fly(roll(0.0), [0.0, 0.0, 0.5]);
            }
            let o = c.step(roll(deg), [0.0, 0.0, 0.5], true);
            assert_eq!(o.mode, mode);
            let early = c.fires_within(TRIGGER_STEPS - 2, roll(deg), [0.0, 0.0, 0.5]);
            let s = c.fly(roll(deg), [0.0, 0.0, 0.5]);
            let td = c.f.still_us >= c.f.land_detect_us;
            if deg == 40.0 {
                assert!(!early && s.would_fire() && !td, "{mode:?} {deg} deg: at 2.0 s {s:?}");
            } else {
                assert!(!early && !s.would_fire() && td, "{mode:?} {deg} deg: TD held, K never {s:?}");
            }
        }
    }
}

/// N7: each term alone false keeps K from firing (4 s of steps, the timer at 0): lean 10 deg with the setpoint at -30
/// (error 40); lean 20 deg tracking a 20 deg setpoint; the filtered acceleration at 3.5 m/s^2; |v| 10.5 m/s valid.
/// Just inside each bound the same cases fire.
#[test]
fn k_each_term_alone_blocks_it() {
    let block = |c: &mut Crash, q: [f32; 4], sticks: [f32; 3], what: &str| {
        for k in 0..2 * TRIGGER_STEPS {
            let s = c.fly(q, sticks);
            assert!(!s.would_fire() && s.held_us == 0, "{what}: step {k} {s:?}");
        }
        c.f.crash_status()
    };
    let s = block(&mut Crash::flown(ANGLE), roll(10.0), [-1.0, 0.0, 0.5], "lean 10");
    assert!(!s.lean && s.error && s.accel && s.speed, "{s:?}");
    assert!(Crash::flown(ANGLE).fires_within(TRIGGER_STEPS, roll(15.5), [-1.0, 0.0, 0.5]), "lean 15.5 fires");
    let s = block(&mut Crash::flown(ANGLE), roll(20.0), [20.0 / 30.0, 0.0, 0.5], "error 0");
    assert!(s.lean && !s.error && s.accel && s.speed, "{s:?}");
    assert!(Crash::flown(ANGLE).fires_within(TRIGGER_STEPS, roll(50.5), [20.0 / 30.0, 0.0, 0.5]), "error 30.5 fires");
    let mut c = Crash::flown(ANGLE);
    c.extra_ned = [3.5, 0.0, 0.0];
    for _ in 0..4_000 {
        c.fly(roll(0.0), [0.0, 0.0, 0.5]);
    }
    let s = block(&mut c, roll(90.0), [0.0, 0.0, 0.5], "accel 3.5");
    assert!(s.lean && s.error && !s.accel && s.speed, "{s:?}");
    let mut c = Crash::flown(ANGLE);
    c.extra_ned = [0.0, 2.9, 0.0];
    for _ in 0..4_000 {
        c.fly(roll(0.0), [0.0, 0.0, 0.5]);
    }
    assert!(c.fires_within(TRIGGER_STEPS, roll(90.0), [0.0, 0.0, 0.5]), "accel 2.9 fires");
    let mut c = Crash::flown(VELOCITY);
    c.set_velocity([6.0, 8.0, 3.2]);
    let s = block(&mut c, roll(90.0), [0.0, 0.0, 0.5], "speed 10.5");
    assert!(s.lean && s.error && s.accel && !s.speed, "{s:?}");
    let mut c = Crash::flown(VELOCITY);
    c.set_velocity([6.0, 7.9, 0.0]);
    assert!(c.fires_within(TRIGGER_STEPS, roll(90.0), [0.0, 0.0, 0.5]), "speed 9.9 fires");
    // The same 10.5 m/s with the velocity not valid: the term is true.
    let mut c = Crash::flown(ANGLE);
    c.set_velocity([6.0, 8.0, 3.2]);
    assert!(c.fires_within(TRIGGER_STEPS, roll(90.0), [0.0, 0.0, 0.5]), "velocity not valid fires");
}

/// N7: one step with a term false restarts the timer: after it, a full 2.0 s is needed again.
#[test]
fn k_a_one_step_break_restarts_the_timer() {
    let mut c = Crash::flown(ANGLE);
    assert!(!c.fires_within(TRIGGER_STEPS - 1, roll(90.0), [0.0, 0.0, 0.5]));
    let s = c.fly(roll(10.0), [0.0, 0.0, 0.5]);
    assert!(!s.lean && s.held_us == 0, "{s:?}");
    assert!(!c.fires_within(TRIGGER_STEPS - 1, roll(90.0), [0.0, 0.0, 0.5]), "1.99875 s after the break");
    assert!(c.fly(roll(90.0), [0.0, 0.0, 0.5]).would_fire(), "2.0 s after the break");
}

/// N7: never before the first airborne latch since arming (armed on the ground at throttle 0, rocking at 30 deg/s so TD
/// does not hold), upside down for 4 s; never while TD is held (after a flight, upside down and still with the throttle
/// at 0: the timer runs until TD holds at 1.0 s, then K is reset); tilt or rates not valid: no terms, timer reset.
#[test]
fn k_never_before_ever_airborne_never_while_td_held_nor_without_tilt() {
    let mut c = Crash::armed(ANGLE);
    c.set_rates([30f32.to_radians(), 0.0, 0.0]);
    for k in 0..2 * TRIGGER_STEPS {
        let o = c.step(roll(180.0), [0.0; 3], true);
        let s = c.f.crash_status();
        assert!(o.armed && !c.f.hands_off.ever_airborne() && c.f.still_us == 0, "step {k}");
        assert!(!s.lean && s.held_us == 0, "never airborne, step {k}: {s:?}");
    }
    let mut c = Crash::flown(ANGLE);
    let mut most = 0;
    for k in 0..2 * TRIGGER_STEPS {
        let s = c.fly(roll(180.0), [0.0; 3]);
        assert!(!s.would_fire(), "step {k}");
        most = most.max(s.held_us);
        if c.f.still_us >= c.f.land_detect_us {
            assert_eq!(s.held_us, 0, "TD held, step {k}");
        }
    }
    assert!(c.f.still_us >= c.f.land_detect_us && c.f.hands_off.ever_airborne(), "TD held at the end, ever airborne");
    assert!(most > 0 && most < CRASH_TRIGGER_US, "the timer ran before TD held: {most}");
    for valid in [Valid { tilt: false, ..ANGLE }, Valid { rates: false, ..ANGLE }] {
        let mut c = Crash::flown(ANGLE);
        assert!(!c.fires_within(TRIGGER_STEPS - 1, roll(90.0), [0.0, 0.0, 0.5]));
        let mut n = c.cell.get();
        n.valid = valid;
        c.cell.set(n);
        c.step(roll(90.0), [0.0, 0.0, 0.5], true);
        let s = c.f.crash_status();
        assert_eq!((s.lean, s.error, s.accel, s.speed, s.held_us), (false, false, false, false, 0), "{valid:?}");
    }
}

/// N7: recoverable manoeuvres do not fire: Angle at the 30 deg limit tracking its setpoint for 5 s (the IMU at rest, so
/// lean, acceleration and speed hold; the error is 0); a 60 deg upset of 0.5 s against a level setpoint that recovers,
/// repeated four times with 0.1 s level between.
#[test]
fn k_recoverable_manoeuvres_do_not_fire() {
    let mut c = Crash::flown(ANGLE);
    for k in 0..4_000 {
        let s = c.fly(roll(30.0), [1.0, 0.0, 0.5]);
        assert!(s.lean && !s.error && !s.would_fire(), "step {k}: {s:?}");
    }
    let mut c = Crash::flown(ANGLE);
    for _ in 0..4 {
        for k in 0..400 {
            let s = c.fly(roll(60.0), [0.0, 0.0, 0.5]);
            assert!(!s.would_fire(), "upset step {k}: {s:?}");
        }
        assert_eq!(c.f.crash_status().held_us, 400 * DT_US, "the upset held all four terms");
        for _ in 0..80 {
            assert!(!c.fly(roll(0.0), [0.0, 0.0, 0.5]).would_fire());
        }
    }
}

/// K touches no output: a non-finite estimate or acceleration gives false terms, and a crash that would fire leaves the
/// step's outputs equal to those of the same flight with K's terms forced false (the same `StepOut`, bit for bit).
#[test]
fn k_non_finite_inputs_are_false_and_k_changes_no_output() {
    let nav = |q: [f32; 4], v: [f32; 3]| NavState { attitude: q, velocity_ms: v, ..nav_at(0.0, VELOCITY) };
    let level = Some(roll(0.0));
    assert_eq!(crash_terms(&nav(roll(90.0), [0.0; 3]), level, [0.0; 3]), (true, true, true, true));
    assert_eq!(crash_terms(&nav([f32::NAN, 1.0, 0.0, 0.0], [0.0; 3]), level, [0.0; 3]), (false, false, true, true));
    assert_eq!(crash_terms(&nav([0.0; 4], [0.0; 3]), level, [0.0; 3]), (false, false, true, true));
    assert_eq!(crash_terms(&nav(roll(90.0), [0.0; 3]), Some([f32::NAN; 4]), [0.0; 3]), (true, false, true, true));
    assert_eq!(crash_terms(&nav(roll(90.0), [0.0; 3]), Some([0.0; 4]), [0.0; 3]), (true, false, true, true));
    assert_eq!(crash_terms(&nav(roll(90.0), [0.0; 3]), None, [0.0; 3]), (true, false, true, true));
    assert_eq!(crash_terms(&nav(roll(90.0), [0.0; 3]), level, [f32::NAN, 0.0, 0.0]), (true, true, false, true));
    assert_eq!(crash_terms(&nav(roll(90.0), [f32::INFINITY, 0.0, 0.0]), level, [0.0; 3]), (true, true, true, false));
    // The same crash flown twice, the twin with K's timer reset before every step (it never fires): equal outputs.
    let mut a = Crash::flown(ANGLE);
    let mut b = Crash::flown(ANGLE);
    for k in 0..2 * TRIGGER_STEPS {
        let oa = a.step(roll(180.0), [0.0, 0.0, 0.5], true);
        b.f.crash.held_us = 0;
        let ob = b.step(roll(180.0), [0.0, 0.0, 0.5], true);
        assert_eq!(std::format!("{oa:?}"), std::format!("{ob:?}"), "step {k}");
    }
    assert!(a.f.crash_would_fire() && !b.f.crash_would_fire());
}

/// The board's K MSG (`crash_msg`): nothing while K would not fire and none was logged, "K 1 1111" at the step it
/// would fire, nothing while it stays, "K 0" at the step it clears; a dropped MSG (`logged` not updated) is offered again.
/// Over a flight with an upset and a recovery that never fires, with FS_HOLD_EN 0, no MSG at all; a crash, then
/// level again: exactly "K 1 1111" at 2.0 s and "K 0" at the first level step. The texts fit MSG's 64 bytes.
#[test]
fn k_msg_is_edge_triggered_and_absent_from_a_flight_that_never_fires() {
    let s = |held_us: u64| CrashStatus { lean: true, error: true, accel: true, speed: true, held_us };
    let text = |m: Option<[u8; 8]>| -> Option<std::string::String> { m.map(|b| std::string::String::from_utf8_lossy(&b).trim_end_matches('\0').into()) };
    assert_eq!(text(crash_msg(false, &s(CRASH_TRIGGER_US - 1))), None);
    assert_eq!(text(crash_msg(false, &s(CRASH_TRIGGER_US))).as_deref(), Some("K 1 1111"));
    assert_eq!(text(crash_msg(true, &s(CRASH_TRIGGER_US))), None);
    assert_eq!(text(crash_msg(true, &s(0))).as_deref(), Some("K 0"));
    assert_eq!(text(crash_msg(false, &s(0))), None);
    let partial = CrashStatus { lean: true, error: false, accel: true, speed: false, held_us: CRASH_TRIGGER_US };
    assert_eq!(text(crash_msg(false, &partial)).as_deref(), Some("K 1 1010"), "the bits in order");
    let mut b = [0u8; 75];
    assert_eq!(dflog::encode_msg(&mut b, 1, &crash_msg(false, &s(CRASH_TRIGGER_US)).unwrap()), 75);
    assert_eq!(&b[11..20], b"K 1 1111\0", "whole, with its terminator");
    // The board's loop over real steps.
    let board = |c: &mut Crash, script: &[(f32, u32)]| {
        let (mut logged, mut msgs) = (false, std::vec::Vec::new());
        for &(deg, steps) in script {
            for _ in 0..steps {
                let s = c.fly(roll(deg), [0.0, 0.0, 0.5]);
                if let Some(m) = crash_msg(logged, &s) {
                    msgs.push((c.t, text(Some(m)).unwrap()));
                    logged = s.would_fire();
                }
            }
        }
        msgs
    };
    let mut c = Crash::flown(ANGLE);
    assert!(!c.f.fs_hold_in_force());
    assert_eq!(board(&mut c, &[(0.0, 4_000), (60.0, 400), (0.0, 4_000), (30.0, 800), (0.0, 800)]), []);
    let mut c = Crash::flown(ANGLE);
    let t0 = c.t;
    let msgs = board(&mut c, &[(0.0, 800), (180.0, 3_000), (0.0, 800)]);
    let fire = t0 + (800 + u64::from(TRIGGER_STEPS)) * DT_US;
    let clear = t0 + (800 + 3_000 + 1) * DT_US;
    assert_eq!(msgs, [(fire, "K 1 1111".into()), (clear, "K 0".into())]);
}
