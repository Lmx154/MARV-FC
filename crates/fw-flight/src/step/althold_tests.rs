//! Host tests of the mode switch, the effective mode and (from the second stage) altitude hold
//! (decision-nav-eskf-20261120, Amendment F). The estimator is a fake that returns the state the test sets, so each
//! test chooses what the estimate covers. Every expected number comes from `marv_card` or `Tuning`.

extern crate std;

use super::*;
use crate::alloc::CardAllocator;
use crate::imu::{ImuReading, ImuSample};
use crate::pid::CascadedPid;
use core::cell::Cell;

/// The step period of these tests, us.
pub(super) const DT_US: u64 = 1_250;

/// An estimator that returns the state in its cell, stamped with the step time.
pub(super) struct Fake<'a>(pub &'a Cell<NavState>);

impl Estimator for Fake<'_> {
    fn update(&mut self, sensors: &Sensors) -> NavState {
        let mut nav = self.0.get();
        nav.t_us = sensors.imu1.t_us;
        nav
    }
}

pub(super) type TestFlight<'a> = Flight<Fake<'a>, CascadedPid, CardAllocator>;

/// What Angle needs.
pub(super) const ANGLE: Valid = Valid {
    tilt: true,
    rates: true,
    ..Valid::NONE
};

/// What Altitude needs.
pub(super) const ALTITUDE: Valid = Valid {
    velocity_d: true,
    position_d: true,
    ..ANGLE
};

/// Level, at rest, at height `down_m` (down positive), with `valid`.
pub(super) fn nav_at(down_m: f32, valid: Valid) -> NavState {
    NavState {
        t_us: 0,
        attitude: [1.0, 0.0, 0.0, 0.0],
        rates_rads: [0.0; 3],
        velocity_ms: [0.0; 3],
        position_m: [0.0, 0.0, down_m],
        valid,
        sigma: None,
    }
}

pub(super) fn flight<'a>(
    cell: &'a Cell<NavState>,
    handshake: &'static Handshake,
    tuning: &Tuning,
) -> TestFlight<'a> {
    Flight::new(
        Fake(cell),
        CascadedPid::new(tuning),
        CardAllocator::new(),
        tuning,
        handshake,
    )
}

/// A stick report with the arm switch, the throttle stick and the mode switch; the other sticks centred.
pub(super) fn report(t_us: u64, arm: bool, throttle: f32, switch6: Switch3) -> RcSample {
    RcSample {
        sticks: Sticks {
            roll: 0.0,
            pitch: 0.0,
            throttle,
            yaw: 0.0,
        },
        arm,
        switch6,
        t_us,
    }
}

pub(super) fn input(t_us: u64, rc: Option<RcSample>) -> FlightInputs {
    FlightInputs {
        sensors: Sensors {
            imu1: ImuSample {
                reading: ImuReading::default(),
                t_us,
                seq: 0,
            },
            imu2: None,
            baro: None,
            mag: None,
            gnss: None,
            external: None,
        },
        rc,
    }
}

/// One step at `t_us` with a fresh report (so RC loss never fires and the switch and sticks update).
pub(super) fn step_rc(
    f: &mut TestFlight,
    t_us: u64,
    arm: bool,
    throttle: f32,
    switch6: Switch3,
) -> StepOut {
    f.step(&input(t_us, Some(report(t_us, arm, throttle, switch6))))
}

/// The mode byte MOT carries in the flight records of `out`.
pub(super) fn mot_mode_byte(out: &StepOut) -> u8 {
    let mut buf = [0u8; 256];
    let len = encode_step(out, &mut buf);
    let at = usize::from(dflog::NAV.length)
        + usize::from(dflog::SETP.length)
        + usize::from(dflog::DMD.length);
    assert!(len >= at + usize::from(dflog::MOT.length));
    // MOT: header 3, TimeUS 8, Arm 1, then Mode.
    buf[at + 12]
}

#[test]
fn the_effective_mode_rule() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let not_covered = [
        Valid {
            position_d: false,
            ..ALTITUDE
        },
        Valid {
            velocity_d: false,
            ..ALTITUDE
        },
        Valid {
            tilt: false,
            ..ALTITUDE
        },
        Valid {
            rates: false,
            ..ALTITUDE
        },
        ANGLE,
        Valid::NONE,
    ];
    // (switch, selected, effective when Altitude is covered, effective when it is not)
    let table = [
        (Switch3::Low, Mode::Angle, Mode::Angle, Mode::Angle),
        (Switch3::Middle, Mode::Altitude, Mode::Altitude, Mode::Angle),
        // Position counts as Altitude until it is built.
        (Switch3::High, Mode::Position, Mode::Altitude, Mode::Angle),
    ];
    for (n, &(switch, selected, covered, uncovered)) in table.iter().enumerate() {
        let cell = Cell::new(nav_at(0.0, ALTITUDE));
        let mut f = flight(&cell, &H, &tuning);
        let mut t = 1_000;
        let out = step_rc(&mut f, t, false, 0.0, switch);
        assert_eq!((out.selected, out.mode), (selected, covered), "row {n}");
        assert_eq!(mot_mode_byte(&out), mode_code(covered));
        for (k, valid) in not_covered.iter().enumerate() {
            cell.set(nav_at(0.0, *valid));
            t += DT_US;
            let out = step_rc(&mut f, t, false, 0.0, switch);
            assert_eq!((out.selected, out.mode), (selected, uncovered), "row {n}/{k}");
            assert_eq!(mot_mode_byte(&out), mode_code(uncovered), "row {n}/{k}");
        }
    }
}

#[test]
fn before_the_first_report_the_mode_is_angle_and_the_last_switch_holds() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ALTITUDE));
    let mut f = flight(&cell, &H, &tuning);
    let out = f.step(&input(1_000, None));
    assert_eq!((out.selected, out.mode), (Mode::Angle, Mode::Angle));
    let out = step_rc(&mut f, 2_000, false, 0.0, Switch3::Middle);
    assert_eq!((out.selected, out.mode), (Mode::Altitude, Mode::Altitude));
    // No new report: the switch holds.
    let out = f.step(&input(3_000, None));
    assert_eq!((out.selected, out.mode), (Mode::Altitude, Mode::Altitude));
    let out = step_rc(&mut f, 4_000, false, 0.0, Switch3::Low);
    assert_eq!((out.selected, out.mode), (Mode::Angle, Mode::Angle));
}

/// Negative control: a Flight whose effective mode ignores coverage is Altitude where the rule says Angle.
#[test]
fn the_coverage_switch_changes_the_effective_mode() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ANGLE));
    let mut f = flight(&cell, &H, &tuning);
    assert_eq!(step_rc(&mut f, 1_000, false, 0.0, Switch3::Middle).mode, Mode::Angle);
    f.ignore_coverage = true;
    assert_eq!(step_rc(&mut f, 2_000, false, 0.0, Switch3::Middle).mode, Mode::Altitude);
}

#[test]
fn mode_is_written_at_the_first_step_and_at_every_change_of_either() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ALTITUDE));
    let mut f = flight(&cell, &H, &tuning);
    let mut log = ModeLog::new();
    let mut t = 1_000;
    let mut written = std::vec::Vec::new();
    let mut run = |f: &mut TestFlight, switch: Switch3, t: &mut u64, written: &mut std::vec::Vec<[u8; 13]>| {
        *t += DT_US;
        let out = step_rc(f, *t, false, 0.0, switch);
        if log.pending(&out) {
            let mut b = [0u8; 13];
            assert_eq!(encode_mode(&out, &mut b), 13);
            written.push(b);
            log.done(&out);
        }
    };
    // Steps: Low x3, Middle x3 (Eff 1 -> 2), the estimate drops (Eff 2 -> 1, Sel stays 2), returns, High, Low.
    for _ in 0..3 {
        run(&mut f, Switch3::Low, &mut t, &mut written);
    }
    for _ in 0..3 {
        run(&mut f, Switch3::Middle, &mut t, &mut written);
    }
    cell.set(nav_at(0.0, ANGLE));
    for _ in 0..3 {
        run(&mut f, Switch3::Middle, &mut t, &mut written);
    }
    cell.set(nav_at(0.0, ALTITUDE));
    for _ in 0..3 {
        run(&mut f, Switch3::Middle, &mut t, &mut written);
    }
    run(&mut f, Switch3::High, &mut t, &mut written);
    run(&mut f, Switch3::Low, &mut t, &mut written);
    let codes: std::vec::Vec<(u8, u8)> = written.iter().map(|b| (b[11], b[12])).collect();
    // (Sel, Eff): first step, Middle, Altitude lost, Altitude back, High (Eff stays 2: Position acts as Altitude), Low.
    assert_eq!(codes, [(1, 1), (2, 2), (2, 1), (2, 2), (3, 2), (1, 1)]);
    for b in &written {
        assert_eq!(b[..3], [0xA3, 0x95, dflog::ID_MODE]);
    }
    // The first record is at the first step's time.
    assert_eq!(u64::from_le_bytes(written[0][3..11].try_into().unwrap()), 1_000 + DT_US);
    // A record the ring dropped is offered again: `pending` stays true until `done`.
    let out = step_rc(&mut f, t + DT_US, false, 0.0, Switch3::Middle);
    assert!(log.pending(&out));
    assert!(log.pending(&out));
    // Too small a buffer writes nothing.
    assert_eq!(encode_mode(&out, &mut [0u8; 12]), 0);
}

// ---- Altitude hold (stage 2): the shaper, the loop around it, the log record ----

use marv_card::MASS as CARD_MASS;

/// Arms (switch cycle off then on, throttle `throttle`) with the mode switch at `switch6`; returns the last output.
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

fn near(a: f32, b: f32, tol: f32) {
    assert!((a - b).abs() <= tol, "{a} vs {b} (tol {tol})");
}

/// Reads a layout field (f32 at byte `at`) from an encoded record.
fn f32_at(b: &[u8], at: usize) -> f32 {
    f32::from_le_bytes(b[at..at + 4].try_into().unwrap())
}

#[test]
fn the_stick_law_at_the_edges_and_across_the_travel() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ALTITUDE));
    let mut f = flight(&cell, &H, &tuning);
    let (dz, up, dn) = (
        tuning.pilot_thr_dz,
        tuning.pilot_speed_up_ms,
        tuning.pilot_speed_dn_ms,
    );
    let (low, high) = (0.5 - dz, 0.5 + dz);
    let rate = |f: &TestFlight, thr: f32| f.shaper.stick_rate_d(thr);
    // The dead zone, both edges inclusive, holds.
    assert_eq!(rate(&f, low), 0.0);
    assert_eq!(rate(&f, 0.5), 0.0);
    assert_eq!(rate(&f, high), 0.0);
    // Just outside the edges the rate is small and of the right sign (down positive: below the zone descends).
    assert!(rate(&f, low - 1e-3) > 0.0 && rate(&f, low - 1e-3) < 1e-2 * dn);
    assert!(rate(&f, high + 1e-3) < 0.0 && rate(&f, high + 1e-3) > -1e-2 * up);
    // Linear scaling to full travel: the full speeds at the ends.
    near(rate(&f, 0.0), dn, 1e-6);
    near(rate(&f, 1.0), -up, 1e-6);
    near(rate(&f, 0.2), dn * (low - 0.2) / low, 1e-6);
    near(rate(&f, 0.8), -up * (0.8 - high) / low, 1e-6);
    // Through the shaper: the setpoint carries the feed-forward.
    let mut t = 1_000;
    for thr in [0.0, 0.2, 0.5, 0.8, 1.0] {
        t += DT_US;
        let out = step_rc(&mut f, t, false, thr, Switch3::Middle);
        match out.setpoint {
            Setpoint::Altitude { velocity_d_ms, .. } => assert_eq!(velocity_d_ms, rate(&f, thr)),
            other => panic!("not Altitude: {other:?}"),
        }
    }
}

#[test]
fn entry_captures_the_height_and_zeroes_the_integrator_at_every_entry() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let hover = CARD_MASS * fw_core::calib::GRAVITY_MSS;
    for entry_switch in [Switch3::Middle, Switch3::High] {
        let cell = Cell::new(nav_at(5.0, ALTITUDE));
        let mut f = flight(&cell, &H, &tuning);
        H.set_permit(true);
        let mut t = 1_000;
        arm(&mut f, &mut t, Switch3::Low, 0.0);
        // In Angle there is no target.
        t += DT_US;
        step_rc(&mut f, t, true, 0.5, Switch3::Low);
        assert_eq!(f.shaper.target_d, None);
        for (round, d) in [3.7_f32, 2.2, -1.4].into_iter().enumerate() {
            let mut n = nav_at(d, ALTITUDE);
            n.velocity_ms = [0.0, 0.0, 0.5];
            cell.set(n);
            // First Altitude step: target = D exactly, no jump, the integrator 0, the thrust the weight less the
            // velocity term only (e_v = -0.5 here: the vehicle descends, the setpoint is 0).
            t += DT_US;
            let out = step_rc(&mut f, t, true, 0.5, entry_switch);
            assert_eq!(out.mode, Mode::Altitude);
            match out.setpoint {
                Setpoint::Altitude {
                    down_m,
                    velocity_d_ms,
                    ..
                } => {
                    assert_eq!(down_m, d, "round {round}: no jump");
                    assert_eq!(velocity_d_ms, 0.0);
                }
                other => panic!("not Altitude: {other:?}"),
            }
            assert_eq!(f.controller.vertical_integrator(), 0.0, "round {round}");
            let want = CARD_MASS * (fw_core::calib::GRAVITY_MSS - tuning.psc_velz_p * -0.5);
            near(out.demand.unwrap().thrust_n, want, 1e-3 * hover);
            // Wind it for 2 s: it goes negative, the target stays (the stick is centred).
            for _ in 0..1_600 {
                t += DT_US;
                step_rc(&mut f, t, true, 0.5, entry_switch);
            }
            let wound = f.controller.vertical_integrator();
            assert!(wound < 0.0, "round {round}: {wound}");
            assert_eq!(f.shaper.target_d, Some(d));
            // Leave: no target in Angle.
            for _ in 0..3 {
                t += DT_US;
                step_rc(&mut f, t, true, 0.5, Switch3::Low);
            }
            assert_eq!(f.shaper.target_d, None);
        }
    }
}

#[test]
fn the_lead_cap_limits_how_far_the_target_runs_and_the_control_shows_it_runs_away() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let (up, dn) = (tuning.pilot_speed_up_ms, tuning.pilot_speed_dn_ms);
    let dt_s = DT_US as f32 * 1e-6;
    // The vehicle cannot follow (it stays at D = 0): the stick asks for a descent, then for a climb, for 10 s.
    let gap = |throttle: f32, cap_off: bool| {
        let cell = Cell::new(nav_at(0.0, ALTITUDE));
        let mut f = flight(&cell, &H, &tuning);
        f.shaper.lead_cap_off = cap_off;
        let mut t = 1_000;
        let mut out = step_rc(&mut f, t, false, throttle, Switch3::Middle);
        for _ in 0..8_000 {
            t += DT_US;
            out = step_rc(&mut f, t, false, throttle, Switch3::Middle);
        }
        match out.setpoint {
            Setpoint::Altitude { down_m, .. } => down_m,
            other => panic!("not Altitude: {other:?}"),
        }
    };
    // Down: stops at dn x ALT_LEAD_S (plus at most one step's travel). Up: at up x ALT_LEAD_S.
    let down = gap(0.0, false);
    assert!(down >= dn * ALT_LEAD_S && down <= dn * ALT_LEAD_S + dn * dt_s + 1e-5, "{down}");
    let climb = gap(1.0, false);
    assert!(-climb >= up * ALT_LEAD_S && -climb <= up * ALT_LEAD_S + up * dt_s + 1e-5, "{climb}");
    // Control: with the cap off the target runs at the stick's rate for the whole 10 s.
    let down_off = gap(0.0, true);
    let climb_off = gap(1.0, true);
    assert!(down_off > 9.0 * dn && -climb_off > 9.0 * up, "{down_off} {climb_off}");
    std::println!("lead cap: down {down:.3} m (limit {:.3}), up {:.3} m (limit {:.3}); cap off {down_off:.1} / {climb_off:.1} m",
        dn * ALT_LEAD_S, -climb, up * ALT_LEAD_S);
}

#[test]
fn armed_in_altitude_with_the_stick_low_the_first_thrust_is_bounded_then_falls_to_0() {
    let tuning = Tuning::DEFAULT;
    let hover = CARD_MASS * fw_core::calib::GRAVITY_MSS;
    let (dz, dn) = (tuning.pilot_thr_dz, tuning.pilot_speed_dn_ms);
    // The slowest descent the stick (at most arm_thr_max) can ask for: the smallest velocity error the first step has.
    let e_min = dn * ((0.5 - dz) - tuning.arm_thr_max) / (0.5 - dz);
    let bound = CARD_MASS * (fw_core::calib::GRAVITY_MSS - tuning.psc_velz_p * e_min);
    assert!(bound <= tuning.ground_thrust_g * hover, "below the ground threshold");
    for (n, disarmed_steps) in [1_usize, 1_600].into_iter().enumerate() {
        // Each scenario has its own handshake: one static per run.
        let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
        h.set_permit(true);
        let cell = Cell::new(nav_at(0.0, ALTITUDE));
        let mut f = flight(&cell, h, &tuning);
        let mut t = 1_000;
        // Disarmed in Altitude with the stick low: the target is captured and runs down (up to the lead limit).
        for _ in 0..disarmed_steps {
            t += DT_US;
            step_rc(&mut f, t, false, tuning.arm_thr_max, Switch3::Middle);
        }
        t += DT_US;
        let first = step_rc(&mut f, t, true, tuning.arm_thr_max, Switch3::Middle);
        assert!(first.armed && first.mode == Mode::Altitude);
        let thrust = first.demand.unwrap().thrust_n;
        std::println!(
            "A4 run {n}: first armed thrust {:.4} m g, bound {:.4} m g (ground threshold {:.4} m g)",
            thrust / hover,
            bound / hover,
            tuning.ground_thrust_g
        );
        assert!(thrust <= bound, "{thrust} > {bound}");
        // Then the thrust falls to the allocator's floor (every rotor at idle: the demand is below what four rotors at
        // IDLE_LEVEL give, `limited.thrust_low`) and the integrator stops there, F5, long before it could wind up.
        let mut floor_at = None;
        for k in 1..=4_000_u32 {
            t += DT_US;
            let out = step_rc(&mut f, t, true, tuning.arm_thr_max, Switch3::Middle);
            assert!(out.armed);
            let a = out.allocation.unwrap();
            if a.saturation.thrust_low {
                assert_eq!(a.levels.level, [marv_card::IDLE_LEVEL; 4]);
                floor_at = Some(k);
                break;
            }
        }
        let k0 = floor_at.expect("the thrust never reached the allocator's floor");
        let secs = k0 as f32 * DT_US as f32 * 1e-6;
        std::println!(
            "A4 run {n}: allocator floor (idle) after {secs:.2} s, integrator {:.3} m/s^2",
            f.controller.vertical_integrator()
        );
        let at_floor = f.controller.vertical_integrator();
        for _ in 0..800 {
            t += DT_US;
            let out = step_rc(&mut f, t, true, tuning.arm_thr_max, Switch3::Middle);
            assert!(out.allocation.unwrap().saturation.thrust_low);
            assert!(out.demand.unwrap().thrust_n <= bound);
        }
        // Frozen from the first floored step on: it has not moved (the floor is reached with e_v > 0).
        assert_eq!(f.controller.vertical_integrator(), at_floor, "the integrator stopped");
        let at_zero = at_floor;
        assert!(at_zero > 0.0 && at_zero <= tuning.psc_velz_imax);
    }
}

/// I6 on the fake estimator: the failures found (empty when every part holds). With `ignore` the Flight's test switch
/// makes the effective mode ignore coverage (the negative control).
fn i6_failures(ignore: bool) -> std::vec::Vec<&'static str> {
    static H0: Handshake = Handshake::new();
    static H1: Handshake = Handshake::new();
    let h = if ignore { &H1 } else { &H0 };
    let tuning = Tuning::DEFAULT;
    let hover = CARD_MASS * fw_core::calib::GRAVITY_MSS;
    let mut bad = std::vec::Vec::new();
    let mut check = |ok: bool, what: &'static str| {
        if !ok {
            bad.push(what);
        }
    };
    h.set_permit(true);
    let cell = Cell::new(nav_at(2.0, ALTITUDE));
    let mut f = flight(&cell, h, &tuning);
    f.ignore_coverage = ignore;
    let mut log = ModeLog::new();
    let mut t = 1_000;
    arm(&mut f, &mut t, Switch3::Middle, 0.0);
    let throttle = 0.3;
    let angle_thrust = 2.0 * throttle * hover;
    for lost in [
        Valid { velocity_d: false, ..ALTITUDE },
        Valid { position_d: false, ..ALTITUDE },
    ] {
        // Altitude, covered, for a while.
        cell.set(nav_at(2.0, ALTITUDE));
        let mut out = StepOut { ..step_rc(&mut f, t + DT_US, true, throttle, Switch3::Middle) };
        t += DT_US;
        for _ in 0..50 {
            t += DT_US;
            out = step_rc(&mut f, t, true, throttle, Switch3::Middle);
        }
        check(out.mode == Mode::Altitude, "Altitude while covered");
        if log.pending(&out) {
            log.done(&out);
        }
        // The flag drops: Angle from that very step, MODE logged, the Angle stick thrust, no target.
        cell.set(nav_at(1.0, lost));
        t += DT_US;
        out = step_rc(&mut f, t, true, throttle, Switch3::Middle);
        check(out.selected == Mode::Altitude, "selected stays Altitude");
        check(out.mode == Mode::Angle, "effective Angle from the step the flag drops");
        check(matches!(out.setpoint, Setpoint::Attitude { .. }), "an Angle setpoint");
        check(
            out.demand.is_ok_and(|d| (d.thrust_n - angle_thrust).abs() <= 1e-3 * hover),
            "the Angle stick thrust",
        );
        check(f.shaper.target_d.is_none(), "no target outside Altitude");
        check(log.pending(&out), "MODE is logged at the change");
        let mut b = [0u8; 13];
        encode_mode(&out, &mut b);
        check(b[11] == 2 && b[12] == 1, "MODE says Sel 2, Eff 1");
        log.done(&out);
        for _ in 0..10 {
            t += DT_US;
            out = step_rc(&mut f, t, true, throttle, Switch3::Middle);
            check(out.mode == Mode::Angle, "Angle for as long as the flag is down");
        }
        // Coverage returns at another height: Altitude again with a fresh capture and the integrator at 0.
        cell.set(nav_at(0.4, ALTITUDE));
        t += DT_US;
        out = step_rc(&mut f, t, true, throttle, Switch3::Middle);
        check(out.mode == Mode::Altitude, "Altitude again when covered");
        check(
            matches!(out.setpoint, Setpoint::Altitude { down_m, .. } if down_m == 0.4),
            "a fresh capture at the current height",
        );
        check(f.controller.vertical_integrator() == 0.0, "integrator 0 at the return");
        check(log.pending(&out), "MODE is logged at the return");
        log.done(&out);
    }
    bad
}

#[test]
fn i6_the_effective_mode_never_exceeds_coverage() {
    let bad = i6_failures(false);
    assert!(bad.is_empty(), "{bad:?}");
}

/// Negative control: a Flight that ignores coverage fails I6.
#[test]
fn i6_control_ignoring_coverage_fails() {
    let bad = i6_failures(true);
    assert!(!bad.is_empty());
    assert!(bad.contains(&"effective Angle from the step the flag drops"));
}

#[test]
fn rc_loss_in_altitude_stops_after_the_timeout_exactly_as_in_angle() {
    let tuning = Tuning::DEFAULT;
    let stop_after = |switch6: Switch3| {
        let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
        h.set_permit(true);
        let cell = Cell::new(nav_at(0.0, ALTITUDE));
        let mut f = flight(&cell, h, &tuning);
        let mut t = 1_000;
        arm(&mut f, &mut t, switch6, 0.0);
        for _ in 0..100 {
            t += DT_US;
            let out = step_rc(&mut f, t, true, 0.5, switch6);
            assert!(out.armed && out.motors.throttles().is_some());
        }
        let last_report = t;
        for k in 1..=2_000_u64 {
            let out = f.step(&input(last_report + k * DT_US, None));
            if !out.armed {
                assert_eq!(out.motors, MotorCommand::STOP);
                return k;
            }
            assert!(out.motors.throttles().is_some(), "still spinning before the timeout");
        }
        panic!("never stopped");
    };
    let angle = stop_after(Switch3::Low);
    let altitude = stop_after(Switch3::Middle);
    // Fresh while the age is at most the timeout: the first step past it stops.
    let expected = tuning.rc_fs_timeout_us / DT_US + 1;
    assert_eq!((angle, altitude), (expected, expected));
}

/// A 1-D point mass on the vertical axis, the plant of the closed-loop tests: thrust scale `scale` (the vehicle gives
/// `scale` x the commanded thrust), level, perfect estimate. Returns (time s, D, target D, vD) per step after the
/// entry. The pilot's throttle is `script(t)`; the run starts at hover (D0 = -2 m, climb rate `v0`) with the switch in
/// Altitude from the first step.
fn fly(
    scale: f32,
    tuning: &Tuning,
    v0: f32,
    seconds: f32,
    script: impl Fn(f32) -> f32,
) -> std::vec::Vec<(f32, f32, f32, f32)> {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    let cell = Cell::new(nav_at(-2.0, ALTITUDE));
    let mut f = flight(&cell, h, tuning);
    let mut t_us = 1_000;
    // Arm in Angle at the stick floor, one more Angle step with the stick centred; the plant waits.
    arm(&mut f, &mut t_us, Switch3::Low, 0.0);
    t_us += DT_US;
    step_rc(&mut f, t_us, true, 0.5, Switch3::Low);
    let (mut d, mut v) = (-2.0_f32, v0);
    let dt = DT_US as f32 * 1e-6;
    let mut log = std::vec::Vec::new();
    let steps = (seconds / dt) as u32;
    for k in 0..steps {
        let mut nav = nav_at(d, ALTITUDE);
        nav.velocity_ms = [0.0, 0.0, v];
        cell.set(nav);
        t_us += DT_US;
        let time = k as f32 * dt;
        let out = step_rc(&mut f, t_us, true, script(time), Switch3::Middle);
        let target = match out.setpoint {
            Setpoint::Altitude { down_m, .. } => down_m,
            other => panic!("not Altitude: {other:?}"),
        };
        // Down positive: a = g - T/m.
        let thrust = out.demand.unwrap().thrust_n * scale;
        let a = fw_core::calib::GRAVITY_MSS - thrust / CARD_MASS;
        v += a * dt;
        d += v * dt;
        log.push((time, d, target, v));
    }
    log
}

#[test]
fn closed_loop_hold_climb_and_a_wrong_thrust_scale() {
    let tuning = Tuning::DEFAULT;
    let dz = tuning.pilot_thr_dz;
    let g = fw_core::calib::GRAVITY_MSS;
    let dt = DT_US as f32 * 1e-6;
    // 1. From hover, centred stick: the entry holds the height.
    let run = fly(1.0, &tuning, 0.0, 10.0, |_| 0.5);
    let worst = run.iter().map(|r| (r.1 - r.2).abs()).fold(0.0, f32::max);
    std::println!("hold from hover: max |D - target| {worst:.6} m over 10 s");
    assert!(worst <= 1e-3);
    // 1b. Entered while climbing at 2 m/s (Angle's stick holds the weight, so the climb goes on): the overshoot, then
    // back to the captured height.
    let run = fly(1.0, &tuning, -2.0, 20.0, |_| 0.5);
    let entry = run[0].2;
    let overshoot = run.iter().map(|r| entry - r.1).fold(0.0, f32::max);
    let last = run.last().unwrap();
    std::println!("entry at a 2 m/s climb: overshoot {overshoot:.3} m, error after 20 s {:.4} m", last.1 - last.2);
    assert!(overshoot > 0.0 && overshoot < 1.0);
    assert!((last.1 - last.2).abs() <= 0.01);
    // 2. A stick-up of 1 s at half the scaled travel climbs by about the speed x scale x 1 s and holds there.
    let half_up = 0.5 + dz + 0.5 * (0.5 - dz);
    let run = fly(1.0, &tuning, 0.0, 15.0, |t| if (1.0..2.0).contains(&t) { half_up } else { 0.5 });
    let entry = run[0].2;
    let last = run.last().unwrap();
    let want = -tuning.pilot_speed_up_ms * 0.5 * 1.0;
    let lag = run.iter().map(|r| (r.1 - r.2).abs()).fold(0.0, f32::max);
    std::println!("climb 1 s at half travel: moved {:.3} m (want {want:.3}), peak lag {lag:.3} m, final error {:.4} m", last.1 - entry, last.1 - last.2);
    near(last.1 - entry, want, 0.02);
    assert!(run.iter().rev().take((1.0 / dt) as usize).all(|r| (r.1 - last.1).abs() <= 0.01), "holds");
    // 3. A thrust scale of 0.7 and 1.3: the integrator trims it.
    for scale in [0.7_f32, 1.3] {
        let run = fly(scale, &tuning, 0.0, 40.0, |_| 0.5);
        let last = run.last().unwrap();
        let settle = run.iter().rev().find(|r| (r.1 - r.2).abs() > 0.05).map_or(0.0, |r| r.0);
        let peak = run.iter().map(|r| r.1 - r.2).fold(0.0, |a: f32, e| if e.abs() > a.abs() { e } else { a });
        std::println!("scale {scale}: steady error {:.5} m, settled (within 5 cm) at {settle:.1} s, peak error {peak:.3} m", last.1 - last.2);
        assert!((last.1 - last.2).abs() <= 0.01, "scale {scale}");
        // Control: with psc_velz_i = 0 an offset stays: the thrust shortfall over the loop's stiffness.
        let mut no_i = tuning;
        no_i.psc_velz_i = 0.0;
        let run = fly(scale, &no_i, 0.0, 40.0, |_| 0.5);
        let last = run.last().unwrap();
        let offset = g * (1.0 / scale - 1.0) / (tuning.psc_velz_p * tuning.psc_posz_p);
        std::println!("scale {scale}, psc_velz_i = 0: offset {:.4} m (expected {offset:.4})", last.1 - last.2);
        near(last.1 - last.2, offset, 0.05 * offset.abs());
    }
}

#[test]
fn spn_is_written_only_in_altitude_with_the_target_and_the_feed_forward() {
    static H: Handshake = Handshake::new();
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(-1.25, ALTITUDE));
    let mut f = flight(&cell, &H, &tuning);
    let mut b = [0u8; 39];
    let out = step_rc(&mut f, 5_000, false, 0.0, Switch3::Low);
    assert_eq!(encode_spn(&out, &mut b), 0, "none in Angle");
    let out = step_rc(&mut f, 6_000, false, 0.0, Switch3::Middle);
    let v_ff = match out.setpoint {
        Setpoint::Altitude { velocity_d_ms, .. } => velocity_d_ms,
        other => panic!("not Altitude: {other:?}"),
    };
    assert_eq!(v_ff, tuning.pilot_speed_dn_ms);
    assert_eq!(encode_spn(&out, &mut b), 39);
    assert_eq!(b[..3], [0xA3, 0x95, 150 + 1]);
    assert_eq!(u64::from_le_bytes(b[3..11].try_into().unwrap()), 6_000);
    // PN, PE, PD, VN, VE, VD, Hdg.
    let fields: std::vec::Vec<f32> = (0..7).map(|i| f32_at(&b, 11 + 4 * i)).collect();
    assert_eq!(fields, [0.0, 0.0, -1.25, 0.0, 0.0, v_ff, 0.0]);
    assert_eq!(encode_spn(&out, &mut [0u8; 38]), 0);
}
