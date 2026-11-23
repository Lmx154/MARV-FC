//! Host tests of the RPM notch at the step level (decision-rpm-notch-20261123): the eRPM age rule, the switch latched on
//! disarmed steps, the controller's torques with no usable eRPM, and the P, I, D of the rate log against the demand. The
//! estimator is the fake of `althold_tests`.

extern crate std;

use super::althold_tests::{ANGLE, DT_US, TestFlight, flight, input, nav_at};
use super::*;
use crate::tuning::Tuning;
use core::cell::Cell;
use fw_core::esc::ANSWER_US;
use fw_core::rc::{RcSample, Sticks};
use std::vec::Vec;

/// eRPM of a rotor at 100 Hz with 14 poles: 100 x 60 x 14 / 2.
const ERPM_100_HZ: u32 = 42_000;

/// One step with the arm switch `arm`, the sticks `[roll, pitch]` and the hover throttle once armed (the gate arms
/// only with the throttle at the bottom); `erpm` is each line's value and its age, us.
fn armed_step(
    f: &mut TestFlight,
    t: u64,
    arm: bool,
    sticks: [f32; 2],
    erpm: [Option<(u32, u64)>; LINES],
) -> StepOut {
    let rc = RcSample {
        sticks: Sticks {
            roll: sticks[0],
            pitch: sticks[1],
            throttle: if f.gate.armed() { 0.5 } else { 0.0 },
            yaw: 0.0,
        },
        arm,
        switch6: Switch3::Low,
        t_us: t,
    };
    let mut inputs = input(t, Some(rc));
    inputs.erpm = erpm.map(|e| e.map(|(v, age)| (v, t.saturating_sub(age))));
    f.step(&inputs)
}

/// The rates of step `k`: a 120 Hz tone, a slow motion, noise from a generator, and signed zeros now and then.
fn rates(k: u32, seed: &mut u32, amp: f32) -> [f32; 3] {
    let mut noise = || {
        *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (*seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
    };
    let t = k as f32 * DT_US as f32 * 1e-6;
    let tone = amp * libm::sinf(core::f32::consts::TAU * 120.0 * t);
    if k % 89 == 0 {
        return [-0.0, 0.0, -0.0];
    }
    [0.3 * libm::sinf(2.0 * t) + tone + 0.05 * noise(), 0.2 * noise() + tone, 0.1 * noise()]
}

/// The outputs of 600 armed steps with the switch set to `on` (`None`: `set_notch` never called) and each line's eRPM
/// `(value, age)`; the rates carry a tone of amplitude `amp` and the roll stick moves.
fn run(on: Option<bool>, erpm: [Option<(u32, u64)>; LINES], amp: f32) -> Vec<StepOut> {
    // A handshake of its own: the gate's `try_arm` fails while the flag a previous run set is still up.
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ANGLE));
    let mut f = flight(&cell, h, &tuning);
    if let Some(on) = on {
        assert!(f.set_notch(on, 14));
    }
    // The time starts above ANSWER_US, so that an eRPM's capture time is never clamped at 0.
    let mut t = 1_000_000;
    for arm in [false, false, true] {
        t += DT_US;
        armed_step(&mut f, t, arm, [0.0; 2], [None; LINES]);
    }
    let mut seed = 11;
    (0..600u32)
        .map(|k| {
            t += DT_US;
            let mut n = nav_at(0.0, ANGLE);
            n.rates_rads = rates(k, &mut seed, amp);
            cell.set(n);
            let roll = 0.4 * libm::sinf(k as f32 * 0.02);
            armed_step(&mut f, t, true, [roll, 0.1], erpm)
        })
        .collect()
}

fn torque_bits(o: &StepOut) -> [u32; 4] {
    let d = o.demand.unwrap();
    [d.thrust_n.to_bits(), d.torque_nm[0].to_bits(), d.torque_nm[1].to_bits(), d.torque_nm[2].to_bits()]
}

fn same_torques(a: &[StepOut], b: &[StepOut]) -> bool {
    a.iter().zip(b.iter()).all(|(x, y)| torque_bits(x) == torque_bits(y))
}

#[test]
fn the_switch_off_no_erpm_or_low_frequencies_give_the_torques_of_a_controller_without_notch() {
    let live = [Some((ERPM_100_HZ, 0)); LINES];
    let base = run(None, [None; LINES], 0.4);
    assert!(base.iter().all(|o| o.armed && o.demand.is_ok()));
    // The switch off with live eRPM; the switch on with none; on with eRPM giving f1 16 Hz (3 x 16 < 50).
    assert!(same_torques(&base, &run(Some(false), live, 0.4)), "off");
    assert!(same_torques(&base, &run(Some(true), [None; LINES], 0.4)), "on, no eRPM");
    assert!(same_torques(&base, &run(Some(true), [Some((6_720, 0)); LINES], 0.4)), "on, every h f1 below 50 Hz");
    assert!(same_torques(&base, &run(Some(true), [Some((0, 0)); LINES], 0.4)), "on, eRPM 0");
    // Control: the same run with the notch live differs, and the rate the loop used is not the estimate's.
    let on = run(Some(true), live, 0.4);
    assert!(!same_torques(&base, &on), "control");
    assert!(on.iter().any(|o| o.rate.rate_rads != o.nav.rates_rads));
    assert!(base.iter().all(|o| o.rate.rate_rads == o.nav.rates_rads && !o.rate.on));
}

#[test]
fn an_erpm_of_exactly_100_ms_is_missing_and_one_just_under_is_used() {
    let base = run(None, [None; LINES], 0.4);
    let old = run(Some(true), [Some((ERPM_100_HZ, ANSWER_US)); LINES], 0.4);
    assert!(same_torques(&base, &old), "exactly ANSWER_US old: bypassed");
    assert!(old.iter().all(|o| o.rate.f1_hz == [0.0; LINES] && o.rate.on));
    let young = run(Some(true), [Some((ERPM_100_HZ, ANSWER_US - 1)); LINES], 0.4);
    assert!(!same_torques(&base, &young), "one us under: used");
    assert!(young.iter().all(|o| o.rate.f1_hz == [100.0; LINES]));
    // A line at a time: only the old line stays out; the report names each line's frequency.
    let mixed = run(
        Some(true),
        [Some((ERPM_100_HZ, ANSWER_US)), Some((ERPM_100_HZ, 0)), None, Some((ERPM_100_HZ + 4_200, ANSWER_US - 1))],
        0.4,
    );
    assert!(mixed.iter().all(|o| o.rate.f1_hz == [0.0, 100.0, 0.0, 110.0]));
}

#[test]
fn p_i_and_d_of_the_rate_log_sum_to_the_demanded_torque_bit_for_bit() {
    let live = [Some((ERPM_100_HZ, 0)), Some((ERPM_100_HZ + 8_400, 0)), Some((ERPM_100_HZ - 8_400, 10_000)), None];
    let mut checked = 0;
    let mut nonzero = [false; 3];
    // The tone of 0.4 rad/s, and a violent one that saturates the allocator.
    for (on, amp) in [(Some(true), 0.4), (Some(false), 0.4), (None, 0.4), (Some(true), 30.0)] {
        for o in run(on, live, amp) {
            let Ok(d) = o.demand else { continue };
            for axis in 0..3 {
                let sum = (o.rate.p_nm[axis] + o.rate.i_nm[axis]) + o.rate.d_nm[axis];
                assert_eq!(sum.to_bits(), d.torque_nm[axis].to_bits(), "axis {axis}");
            }
            nonzero[0] |= o.rate.p_nm[0] != 0.0;
            nonzero[1] |= o.rate.i_nm[0] != 0.0;
            nonzero[2] |= o.rate.d_nm[0] != 0.0;
            checked += 1;
        }
    }
    assert_eq!(checked, 4 * 600);
    assert_eq!(nonzero, [true; 3], "the check is not vacuous");
}

#[test]
fn the_switch_and_the_poles_are_taken_on_disarmed_steps_only() {
    static H: Handshake = Handshake::new();
    H.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ANGLE));
    let mut f = flight(&cell, &H, &tuning);
    let live = [Some((ERPM_100_HZ, 0)); LINES];
    let mut t = 0;
    let mut step = |f: &mut TestFlight, arm: bool| {
        t += DT_US;
        armed_step(f, t, arm, [0.0; 2], live)
    };
    assert!(!step(&mut f, false).rate.on, "off until set");
    assert!(f.set_notch(true, 14));
    assert!(step(&mut f, false).rate.on, "taken at a disarmed step");
    let armed = step(&mut f, true);
    assert!(armed.armed && armed.rate.on);
    // 14 poles: 42 000 eRPM is 100 Hz.
    assert_eq!(armed.rate.f1_hz, [100.0; LINES]);
    // Changed while armed: no effect, neither the switch nor the poles.
    assert!(f.set_notch(false, 28));
    let still = step(&mut f, true);
    assert!(still.armed && still.rate.on && still.rate.f1_hz == [100.0; LINES]);
    // The step that disarms is still armed at its start; the next one is disarmed and takes the values.
    assert!(!step(&mut f, false).armed);
    let after = step(&mut f, false);
    assert!(!after.rate.on);
    assert_eq!(after.rate.f1_hz, [50.0; LINES]);
    // The parameter's range: poles 2 to 64; a refused value changes nothing.
    assert!(!f.set_notch(true, 1) && !f.set_notch(true, 65) && !f.set_notch(true, 0) && !f.set_notch(true, -14));
    assert!(!step(&mut f, false).rate.on);
    assert!(f.set_notch(true, 2) && f.set_notch(true, 64));
}

#[test]
fn a_refused_demand_logs_the_frequencies_and_the_switch_and_nothing_of_the_loop() {
    static H: Handshake = Handshake::new();
    H.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ANGLE));
    let mut f = flight(&cell, &H, &tuning);
    assert!(f.set_notch(true, 14));
    let live = [Some((ERPM_100_HZ, 0)); LINES];
    let mut t = 0;
    for arm in [false, false, true, true, true] {
        t += DT_US;
        armed_step(&mut f, t, arm, [0.0; 2], live);
    }
    let mut n = nav_at(0.0, ANGLE);
    n.valid = Valid::NONE;
    cell.set(n);
    t += DT_US;
    let out = armed_step(&mut f, t, true, [0.3, 0.0], live);
    assert!(out.demand.is_err());
    assert_eq!(out.rate, RateLog { f1_hz: [100.0; LINES], on: true, ..RateLog::NONE });
}
