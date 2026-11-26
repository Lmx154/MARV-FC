//! Host tests of the RPM notch at the step level: the eRPM age rule, the poles latched on disarmed steps, the
//! controller's torques with no usable eRPM, and the P, I, D of the rate log against the demand.

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

/// One step with the arm switch `arm`, the sticks `[roll, pitch]` and the hover throttle once armed.
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
        source: fw_core::rc::RcSource::Crsf,
        mission: false,
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

/// The outputs of 600 armed steps with each line's eRPM `(value, age)`; the rates carry a tone of amplitude `amp` and the
/// roll stick moves.
fn run(erpm: [Option<(u32, u64)>; LINES], amp: f32) -> Vec<StepOut> {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ANGLE));
    let mut f = flight(&cell, h, &tuning);
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
fn no_erpm_or_low_frequencies_give_the_torques_of_a_controller_without_notch() {
    let live = [Some((ERPM_100_HZ, 0)); LINES];
    let base = run([None; LINES], 0.4);
    assert!(base.iter().all(|o| o.armed && o.demand.is_ok()));
    assert!(same_torques(&base, &run([Some((6_720, 0)); LINES], 0.4)), "every h f1 below 50 Hz");
    assert!(same_torques(&base, &run([Some((0, 0)); LINES], 0.4)), "eRPM 0");
    let on = run(live, 0.4);
    assert!(!same_torques(&base, &on), "control");
    assert!(on.iter().any(|o| o.rate.rate_rads != o.nav.rates_rads));
    assert!(base.iter().all(|o| o.rate.rate_rads == o.nav.rates_rads && o.rate.on));
}

#[test]
fn an_erpm_of_exactly_100_ms_is_missing_and_one_just_under_is_used() {
    let base = run([None; LINES], 0.4);
    let old = run([Some((ERPM_100_HZ, ANSWER_US)); LINES], 0.4);
    assert!(same_torques(&base, &old), "exactly ANSWER_US old: bypassed");
    assert!(old.iter().all(|o| o.rate.f1_hz == [0.0; LINES] && o.rate.on));
    let young = run([Some((ERPM_100_HZ, ANSWER_US - 1)); LINES], 0.4);
    assert!(!same_torques(&base, &young), "one us under: used");
    assert!(young.iter().all(|o| o.rate.f1_hz == [100.0; LINES]));
    let mixed = run(
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
    for (erpm, amp) in [(live, 0.4), ([None; LINES], 0.4), (live, 30.0)] {
        for o in run(erpm, amp) {
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
    assert_eq!(checked, 3 * 600);
    assert_eq!(nonzero, [true; 3], "the check is not vacuous");
}

#[test]
fn the_poles_are_taken_on_disarmed_steps_only() {
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
    let first = step(&mut f, false);
    assert!(first.rate.on && first.rate.f1_hz == [100.0; LINES], "on, 14 poles before any set");
    let armed = step(&mut f, true);
    assert!(armed.armed && armed.rate.on);
    assert_eq!(armed.rate.f1_hz, [100.0; LINES]);
    assert!(f.set_notch(28));
    let still = step(&mut f, true);
    assert!(still.armed && still.rate.on && still.rate.f1_hz == [100.0; LINES]);
    assert!(!step(&mut f, false).armed);
    let after = step(&mut f, false);
    assert!(after.rate.on);
    assert_eq!(after.rate.f1_hz, [50.0; LINES]);
    assert!(!f.set_notch(1) && !f.set_notch(65) && !f.set_notch(0) && !f.set_notch(-14));
    assert_eq!(step(&mut f, false).rate.f1_hz, [50.0; LINES]);
    assert!(f.set_notch(2) && f.set_notch(64));
}

#[test]
fn a_refused_demand_logs_the_frequencies_and_the_switch_and_nothing_of_the_loop() {
    static H: Handshake = Handshake::new();
    H.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ANGLE));
    let mut f = flight(&cell, &H, &tuning);
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
