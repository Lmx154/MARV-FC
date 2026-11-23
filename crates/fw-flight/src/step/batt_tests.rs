//! Host tests of the low-battery descent at the step level (decision-nav-eskf-20261120, Amendment BATT, T2): with the
//! request off (`BATT_FS_CRT_ACT` 0, or CRITICAL not reached) every output is bit for bit what it was; with it on the
//! effective mode is at least Altitude when covered and the throttle stick reads 0.0; a change of the mode switch cancels
//! it until disarm; it never disarms and never touches the gate. The estimator is the fake of `althold_tests`.

extern crate std;

use super::althold_tests::{ALTITUDE, ANGLE, DT_US, TestFlight, flight, input, nav_at, step_rc};
use super::*;
use core::cell::Cell;
use std::string::String;
use std::vec::Vec;

/// Everything Position needs.
const POSITION: Valid = Valid {
    heading: true,
    velocity_ne: true,
    position_ne: true,
    ..ALTITUDE
};

/// A handshake of its own per scenario, with the permit set.
fn handshake() -> &'static Handshake {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    h
}

/// Arms with the switch cycle at `switch6` and the throttle floor; returns the time.
fn arm(f: &mut TestFlight, t: &mut u64, switch6: Switch3) {
    for on in [false, true] {
        *t += DT_US;
        step_rc(f, *t, on, 0.0, switch6);
    }
    assert!(f.gate.armed(), "did not arm");
}

fn descent_target(out: &StepOut) -> (f32, f32) {
    match out.setpoint {
        Setpoint::Altitude { down_m, velocity_d_ms, .. } => (down_m, velocity_d_ms),
        Setpoint::Position { position_m, velocity_ms, .. } => (position_m[2], velocity_ms[2]),
        other => panic!("neither Altitude nor Position: {other:?}"),
    }
}

/// A scripted flight: arm in Angle, then the mode switch through Low, Middle, High, Low with the throttle stick moving and
/// the estimate covering, then not covering Altitude; one line of `{:?}` per step. `call` is what is done to the flight
/// before each step (the request, or nothing).
fn script(call: impl Fn(&mut TestFlight, usize)) -> Vec<String> {
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(-2.0, POSITION));
    let mut f = flight(&cell, handshake(), &tuning);
    let mut t = 1_000;
    let mut log = Vec::new();
    arm(&mut f, &mut t, Switch3::Low);
    f.airborne_forced = true;
    for k in 0..4_000usize {
        let switch = match k / 500 {
            0 | 4 | 7 => Switch3::Low,
            1 | 5 => Switch3::Middle,
            2 | 6 => Switch3::High,
            _ => Switch3::Low,
        };
        let valid = if k >= 3_000 { ANGLE } else { POSITION };
        cell.set(nav_at(-2.0, valid));
        let throttle = 0.5 + 0.4 * libm::sinf(k as f32 * 0.01);
        t += DT_US;
        call(&mut f, k);
        let out = step_rc(&mut f, t, true, throttle, switch);
        log.push(std::format!("{out:?}"));
    }
    log
}

/// B-I2: with the request never made, or made false at every step, the whole `StepOut` of a scripted flight (modes, setpoints,
/// demands, allocations, motor command) is the same, step by step.
#[test]
fn action_0_is_bit_identical_whether_the_request_is_never_made_or_false() {
    let never = script(|_, _| {});
    let off = script(|f, _| f.set_batt_descend(false));
    assert_eq!(never.len(), 4_000);
    assert_eq!(never, off);
    // Negative control: the same flight with the request on is not the same (the descent changes the outputs).
    let on = script(|f, _| f.set_batt_descend(true));
    let differing = never.iter().zip(&on).filter(|(a, b)| a != b).count();
    assert!(differing > 1_000, "{differing}");
}

#[test]
fn the_request_does_nothing_while_disarmed() {
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ALTITUDE));
    let mut f = flight(&cell, handshake(), &tuning);
    f.set_batt_descend(true);
    let mut t = 1_000;
    for _ in 0..100 {
        t += DT_US;
        let out = step_rc(&mut f, t, false, 1.0, Switch3::Low);
        assert!(!out.armed);
        assert_eq!((out.selected, out.mode), (Mode::Angle, Mode::Angle));
        assert_eq!(f.batt_descent(), BattDescent::Off);
    }
}

/// Armed in Angle with Altitude covered: the descent makes the effective mode Altitude, the throttle stick reads 0.0
/// (F2's descent speed) whatever the stick is, and the target runs down within the lead cap.
#[test]
fn the_descent_lifts_angle_to_altitude_and_reads_the_throttle_stick_as_zero() {
    let tuning = Tuning::DEFAULT;
    let (dn, lead) = (tuning.pilot_speed_dn_ms, tuning.pilot_alt_lead_s);
    let dt_s = DT_US as f32 * 1e-6;
    for stick in [0.0_f32, 0.5, 1.0] {
        let cell = Cell::new(nav_at(-2.0, ALTITUDE));
        let mut f = flight(&cell, handshake(), &tuning);
        let mut t = 1_000;
        arm(&mut f, &mut t, Switch3::Low);
        t += DT_US;
        let out = step_rc(&mut f, t, true, stick, Switch3::Low);
        assert_eq!(out.mode, Mode::Angle, "no request: Angle");
        f.set_batt_descend(true);
        t += DT_US;
        let out = step_rc(&mut f, t, true, stick, Switch3::Low);
        assert_eq!((out.selected, out.mode), (Mode::Angle, Mode::Altitude), "stick {stick}");
        assert_eq!(f.batt_descent(), BattDescent::Descending);
        let (first, v_ff) = descent_target(&out);
        assert_eq!((first, v_ff), (-2.0, dn), "captured at the current height, F2's descent speed");
        let mut last = first;
        for k in 0..4_000 {
            t += DT_US;
            let out = step_rc(&mut f, t, true, stick, Switch3::Low);
            assert!(out.armed && out.motors.throttles().is_some());
            let (target, v_ff) = descent_target(&out);
            assert_eq!(v_ff, dn);
            assert!(target >= last, "the target never rises");
            if k < 100 {
                assert!((target - last - dn * dt_s).abs() < 1e-5);
            }
            last = target;
        }
        // The vehicle stays at D = -2 m: the target stops at the lead cap, dn x pilot_alt_lead_s below it (one step's travel more).
        assert!(last - -2.0 <= dn * lead + dn * dt_s + 1e-5 && last - -2.0 >= dn * lead, "{last}");
    }
}

/// Selected Altitude behaves the same; selected Position stays Position (airborne) with its height descending.
#[test]
fn altitude_and_position_selected_descend_through_their_own_vertical_loop() {
    let tuning = Tuning::DEFAULT;
    let dn = tuning.pilot_speed_dn_ms;
    for (switch, valid, want) in [
        (Switch3::Middle, ALTITUDE, Mode::Altitude),
        (Switch3::High, POSITION, Mode::Position),
    ] {
        let cell = Cell::new(nav_at(-2.0, valid));
        let mut f = flight(&cell, handshake(), &tuning);
        f.airborne_forced = true;
        let mut t = 1_000;
        arm(&mut f, &mut t, switch);
        f.set_batt_descend(true);
        for _ in 0..50 {
            t += DT_US;
            let out = step_rc(&mut f, t, true, 1.0, switch);
            assert_eq!(out.mode, want);
            assert_eq!(descent_target(&out).1, dn);
        }
        // Control: without the request the full stick climbs.
        f.set_batt_descend(false);
        t += DT_US;
        let out = step_rc(&mut f, t, true, 1.0, switch);
        assert!(descent_target(&out).1 < 0.0);
    }
}

/// Uncovered twin: no height estimate means Angle with the pilot's throttle, exactly the outputs of a flight with no
/// request, and the state says so (core 0 raises "no height estimate: land now").
#[test]
fn without_a_height_estimate_the_pilot_keeps_angle_and_the_throttle() {
    let tuning = Tuning::DEFAULT;
    let run = |request: bool| {
        let cell = Cell::new(nav_at(-2.0, ANGLE));
        let mut f = flight(&cell, handshake(), &tuning);
        let mut t = 1_000;
        arm(&mut f, &mut t, Switch3::Low);
        f.set_batt_descend(request);
        let mut log = Vec::new();
        let mut states = Vec::new();
        for k in 0..200 {
            t += DT_US;
            let out = step_rc(&mut f, t, true, 0.3 + 0.002 * k as f32, Switch3::Low);
            assert_eq!(out.mode, Mode::Angle);
            log.push(std::format!("{out:?}"));
            states.push(f.batt_descent());
        }
        (log, states)
    };
    let (with, states) = run(true);
    let (without, _) = run(false);
    assert_eq!(with, without);
    assert!(states.iter().all(|s| *s == BattDescent::NoHeight));
}

/// Any change of the mode switch cancels the descent until disarm; the pilot has the sticks back; a new arming starts it
/// again while the request stands.
#[test]
fn a_change_of_the_mode_switch_cancels_until_disarm() {
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(-2.0, ALTITUDE));
    let mut f = flight(&cell, handshake(), &tuning);
    let mut t = 1_000;
    arm(&mut f, &mut t, Switch3::Low);
    f.set_batt_descend(true);
    for _ in 0..20 {
        t += DT_US;
        let out = step_rc(&mut f, t, true, 1.0, Switch3::Low);
        assert_eq!(out.mode, Mode::Altitude);
    }
    // Control: a step with the same switch position and no report keeps it going.
    t += DT_US;
    assert_eq!(f.step(&input(t, None)).mode, Mode::Altitude);
    // The switch moves Low -> Middle: cancelled. Altitude is selected now, with the pilot's full stick: a climb.
    t += DT_US;
    let out = step_rc(&mut f, t, true, 1.0, Switch3::Middle);
    assert_eq!(f.batt_descent(), BattDescent::Cancelled);
    assert_eq!((out.selected, out.mode), (Mode::Altitude, Mode::Altitude));
    assert!(descent_target(&out).1 < 0.0, "the pilot's throttle again");
    // Back to Low: still cancelled, Angle with the pilot's throttle, whatever the request says.
    for sw in [Switch3::Low, Switch3::High, Switch3::Low] {
        t += DT_US;
        let out = step_rc(&mut f, t, true, 1.0, sw);
        assert_eq!(f.batt_descent(), BattDescent::Cancelled);
        assert!(out.armed);
        if sw == Switch3::Low {
            assert_eq!(out.mode, Mode::Angle);
        }
    }
    // Disarm (switch off), then arm again: core 0 clears the request at the disarm and raises it after the new arming (an
    // edge while armed), and the descent starts again.
    t += DT_US;
    let out = step_rc(&mut f, t, false, 0.0, Switch3::Low);
    assert!(!out.armed);
    // The state reads the armed state the previous step left, so the next step shows it.
    f.set_batt_descend(false);
    t += DT_US;
    step_rc(&mut f, t, false, 0.0, Switch3::Low);
    assert_eq!(f.batt_descent(), BattDescent::Off);
    t += DT_US;
    step_rc(&mut f, t, true, 0.0, Switch3::Low);
    assert!(f.gate.armed());
    f.set_batt_descend(true);
    for _ in 0..3 {
        t += DT_US;
        step_rc(&mut f, t, true, 0.0, Switch3::Low);
    }
    assert_eq!(f.batt_descent(), BattDescent::Descending);
}

/// A request that is already true when the arming step runs (disarm and re-arm within one supervisor tick: core 0 has not
/// cleared the byte) starts nothing and so wipes no cancel; a later false then true is an edge and starts the descent.
#[test]
fn a_request_standing_across_a_quick_rearm_starts_nothing_until_a_new_edge() {
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(-2.0, ALTITUDE));
    let mut f = flight(&cell, handshake(), &tuning);
    let mut t = 1_000;
    arm(&mut f, &mut t, Switch3::Low);
    f.set_batt_descend(true);
    for _ in 0..5 {
        t += DT_US;
        step_rc(&mut f, t, true, 1.0, Switch3::Low);
    }
    assert_eq!(f.batt_descent(), BattDescent::Descending);
    // The request stays true through a disarm and a re-arm.
    for _ in 0..3 {
        t += DT_US;
        step_rc(&mut f, t, false, 0.0, Switch3::Low);
    }
    assert_eq!(f.batt_descent(), BattDescent::Off);
    arm(&mut f, &mut t, Switch3::Low);
    for _ in 0..20 {
        t += DT_US;
        let out = step_rc(&mut f, t, true, 1.0, Switch3::Low);
        assert!(out.armed);
        assert_eq!((out.mode, f.batt_descent()), (Mode::Angle, BattDescent::Off));
    }
    // The pilot's switch change is no cancel of a descent that never started.
    t += DT_US;
    step_rc(&mut f, t, true, 1.0, Switch3::Middle);
    assert_eq!(f.batt_descent(), BattDescent::Off);
    // False, then true: a new edge while armed starts it.
    f.set_batt_descend(false);
    t += DT_US;
    step_rc(&mut f, t, true, 1.0, Switch3::Low);
    assert_eq!(f.batt_descent(), BattDescent::Off);
    f.set_batt_descend(true);
    for _ in 0..3 {
        t += DT_US;
        step_rc(&mut f, t, true, 1.0, Switch3::Low);
    }
    assert_eq!(f.batt_descent(), BattDescent::Descending);
}

/// The first report and a report that repeats the switch are no change; a switch change BEFORE the request is no cancel.
#[test]
fn only_a_change_while_the_descent_is_asked_for_cancels() {
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(-2.0, ALTITUDE));
    let mut f = flight(&cell, handshake(), &tuning);
    let mut t = 1_000;
    arm(&mut f, &mut t, Switch3::Low);
    for sw in [Switch3::Middle, Switch3::Low, Switch3::Middle] {
        t += DT_US;
        step_rc(&mut f, t, true, 0.5, sw);
    }
    f.set_batt_descend(true);
    for _ in 0..10 {
        t += DT_US;
        let out = step_rc(&mut f, t, true, 0.5, Switch3::Middle);
        assert_eq!(out.mode, Mode::Altitude);
        assert_eq!(f.batt_descent(), BattDescent::Descending);
        assert_eq!(descent_target(&out).1, tuning.pilot_speed_dn_ms);
    }
}

/// B-I1, B-I5: 60 s armed with the descent in force and the vehicle not following: still armed, the motors still spin
/// (never a Stop, never a thrust cut: the collective is the controller's, at or above the idle floor), no disarm; then RC
/// loss disarms and stops, and the arm switch disarms, exactly as without the request.
#[test]
fn sixty_seconds_armed_critical_stays_armed_and_rc_loss_and_the_arm_switch_still_win() {
    let tuning = Tuning::DEFAULT;
    for rc_loss in [true, false] {
        let cell = Cell::new(nav_at(-2.0, ALTITUDE));
        let mut f = flight(&cell, handshake(), &tuning);
        let mut t = 1_000;
        arm(&mut f, &mut t, Switch3::Low);
        f.set_batt_descend(true);
        let steps = 60_000_000 / DT_US;
        for _ in 0..steps {
            t += DT_US;
            let out = step_rc(&mut f, t, true, 0.7, Switch3::Low);
            assert!(out.armed, "disarmed");
            let levels = out.motors.throttles().expect("Stop while armed");
            assert!(levels.iter().all(|l| l.level() >= marv_card::IDLE_LEVEL));
        }
        assert_eq!(f.batt_descent(), BattDescent::Descending);
        if rc_loss {
            // No report for longer than the timeout: Stop and disarmed.
            let mut out = None;
            for _ in 0..(tuning.rc_fs_timeout_us / DT_US + 20) {
                t += DT_US;
                out = Some(f.step(&input(t, None)));
            }
            let out = out.unwrap();
            assert!(!out.armed);
            assert_eq!(out.motors, MotorCommand::STOP);
        } else {
            t += DT_US;
            let out = step_rc(&mut f, t, false, 0.7, Switch3::Low);
            assert!(!out.armed);
            assert_eq!(out.motors, MotorCommand::STOP);
        }
    }
}
