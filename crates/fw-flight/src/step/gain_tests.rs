//! Host tests of the gain parameters at the step level (decision-gain-params-20261123, packet P1): the defaults change
//! nothing, a set takes effect at the next arming only, an out-of-range set is refused, and `set_gains` leaves the
//! controller where `CascadedPid::new` would have. The estimator is the fake of `althold_tests`.

extern crate std;

use super::althold_tests::{ANGLE, DT_US, TestFlight, flight, input, nav_at};
use super::*;
use crate::pid::CascadedPid;
use crate::tuning::{GAIN_COUNT, GAIN_NAMES, Gains, Tuning};
use core::cell::Cell;
use fw_core::rc::{RcSample, Sticks};
use std::string::String;
use std::vec::Vec;

/// Steps of the script, with the arm switch on from step 3 to 699 and again from 720 to 1399.
const STEPS: usize = 1_450;
/// The first step with the arm switch off after the first armed span.
const DISARM_AT: usize = 700;
/// The first step of the second armed span.
const REARM_AT: usize = 720;

fn arm_switch(k: usize) -> bool {
    (3..DISARM_AT).contains(&k) || (REARM_AT..1_400).contains(&k)
}

/// One step's output as text, its encoded records, and the gains in force after it.
type Record = (String, Vec<u8>, Gains);

/// A scripted flight: `STEPS` steps with two armed spans, moving rates and a moving roll stick; `hook(k, flight)` runs
/// before step `k`.
fn script(mut hook: impl FnMut(usize, &mut TestFlight)) -> Vec<Record> {
    // A handshake of its own: the gate's `try_arm` fails while the flag a previous run set is still up.
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ANGLE));
    let mut f = flight(&cell, h, &tuning);
    let mut t = 1_000_000;
    let mut seed = 5u32;
    let mut out = Vec::new();
    for k in 0..STEPS {
        hook(k, &mut f);
        t += DT_US;
        let mut noise = || {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        let x = k as f32 * 0.02;
        let mut n = nav_at(0.0, ANGLE);
        n.rates_rads = [
            0.5 * libm::sinf(2.0 * x) + 0.1 * noise(),
            0.3 * libm::cosf(3.0 * x) + 0.1 * noise(),
            0.2 * libm::sinf(x) + 0.05 * noise(),
        ];
        cell.set(n);
        let rc = RcSample {
            sticks: Sticks {
                roll: 0.4 * libm::sinf(x),
                pitch: 0.1,
                throttle: if f.gate.armed() { 0.5 } else { 0.0 },
                yaw: 0.2 * libm::cosf(0.7 * x),
            },
            arm: arm_switch(k),
            switch6: Switch3::Low,
            source: fw_core::rc::RcSource::Crsf,
            mission: false,
            t_us: t,
        };
        let o = f.step(&input(t, Some(rc)));
        let mut records = [0u8; 256];
        let len = encode_step(&o, &mut records);
        out.push((std::format!("{o:?}"), records[..len].to_vec(), f.gains_in_force()));
    }
    out
}

/// The step outputs and records of two runs, without the gains in force.
fn same_outputs(a: &[Record], b: &[Record], range: core::ops::Range<usize>) -> bool {
    range.into_iter().all(|k| a[k].0 == b[k].0 && a[k].1 == b[k].1)
}

fn defaults() -> Gains {
    Gains::of(&Tuning::DEFAULT)
}

fn rll_p(factor: f32) -> Gains {
    let mut g = defaults();
    g.0[2] *= factor;
    g
}

/// The script has two armed spans with a real demand.
fn check_script_is_not_vacuous(base: &[Record]) {
    let armed = |range: core::ops::Range<usize>| range.filter(|&k| base[k].0.contains("armed: true")).count();
    assert!(armed(100..DISARM_AT) > 500, "{}", armed(100..DISARM_AT));
    assert!(armed(REARM_AT + 100..1_400) > 500, "{}", armed(REARM_AT + 100..1_400));
    assert!(armed(DISARM_AT + 5..REARM_AT) == 0, "disarmed between the spans");
    assert!(base[300].0.contains("torque_nm"), "a demand is made");
}

#[test]
fn set_gains_of_the_defaults_on_every_step_changes_no_output_and_no_record_byte() {
    let base = script(|_, _| {});
    check_script_is_not_vacuous(&base);
    let same = script(|_, f| assert!(f.set_gains(defaults())));
    assert!(same_outputs(&base, &same, 0..STEPS));
    assert!(same.iter().all(|r| r.2 == defaults()));
    assert!(base.iter().all(|r| r.2 == defaults()));
}

#[test]
fn a_set_while_disarmed_applies_from_the_next_arming() {
    let base = script(|_, _| {});
    let tuned = rll_p(1.5);
    let run = script(|k, f| {
        if k == 1 {
            assert!(f.set_gains(tuned));
        }
    });
    assert_eq!(run[0].2, defaults());
    assert_eq!(run[1].2, tuned);
    // Disarmed demands may differ (the controller is called); what flies is what differs after arming.
    let armed_diff = (0..STEPS).find(|&k| base[k].0.contains("armed: true") && base[k].0 != run[k].0);
    assert!(armed_diff.is_some_and(|k| k >= 3), "{armed_diff:?}");
    assert!(run[3..DISARM_AT].iter().all(|r| r.2 == tuned));
}

#[test]
fn a_set_while_armed_has_no_effect_until_after_a_disarm_and_then_applies_from_the_next_arming() {
    let base = script(|_, _| {});
    let tuned = rll_p(1.5);
    let run = script(|k, f| {
        if k == 300 {
            assert!(f.set_gains(tuned));
        }
    });
    // Identical through the whole first armed span, up to and including the step that disarms; gains in force unchanged.
    assert!(same_outputs(&base, &run, 0..DISARM_AT + 1));
    assert!(run[..DISARM_AT + 1].iter().all(|r| r.2 == defaults()));
    // The step that disarms is still armed at its start; the next one is disarmed and latches the set.
    assert!(!run[DISARM_AT].0.contains("armed: true"));
    assert_eq!(run[DISARM_AT + 1].2, tuned);
    // The second armed span flies with it.
    let armed_diff = (REARM_AT..STEPS).find(|&k| base[k].0.contains("armed: true") && base[k].0 != run[k].0);
    assert!(armed_diff.is_some(), "outputs differ after the next arming");
    assert!(run[REARM_AT..1_400].iter().all(|r| r.2 == tuned));
}

#[test]
fn a_refused_set_returns_false_keeps_the_previous_set_and_changes_nothing() {
    let base = script(|_, _| {});
    let mut bad = std::vec![
        ("NaN", {
            let mut g = defaults();
            g.0[2] = f32::NAN;
            g
        }),
        ("negative", rll_p(-1.0)),
        ("x3.01", rll_p(3.01)),
        ("infinite", {
            let mut g = defaults();
            g.0[13] = f32::INFINITY;
            g
        }),
        ("zero", rll_p(0.0)),
    ];
    // Below the minimum and above the maximum of every gain.
    for i in 0..GAIN_COUNT {
        let mut lo = defaults();
        lo.0[i] = f32::from_bits(Gains::MIN.0[i].to_bits() - 1);
        let mut hi = defaults();
        hi.0[i] = f32::from_bits(Gains::MAX.0[i].to_bits() + 1);
        bad.push((GAIN_NAMES[i], lo));
        bad.push((GAIN_NAMES[i], hi));
    }
    for (what, g) in bad {
        // Every step, disarmed or armed, and the same flight as the baseline.
        let run = script(|_, f| assert!(!f.set_gains(g), "{what}"));
        assert!(same_outputs(&base, &run, 0..STEPS), "{what}");
        assert!(run.iter().all(|r| r.2 == defaults()), "{what}");
    }
    // A refused set after an accepted one leaves the accepted one pending.
    let tuned = rll_p(1.5);
    let run = script(|k, f| {
        if k == 1 {
            assert!(f.set_gains(tuned));
        }
        if k == 2 {
            assert!(!f.set_gains(rll_p(3.01)));
        }
    });
    assert_eq!(run[2].2, tuned);
    assert_eq!(run[STEPS - 1].2, tuned);
}

#[test]
fn the_ends_of_every_range_are_accepted() {
    let cell = Cell::new(nav_at(0.0, ANGLE));
    static H: Handshake = Handshake::new();
    let mut f = flight(&cell, &H, &Tuning::DEFAULT);
    assert!(f.set_gains(Gains::MIN) && f.set_gains(Gains::MAX));
    assert_eq!(f.gains_in_force(), defaults(), "nothing is latched before a step");
    f.step(&input(DT_US, None));
    assert_eq!(f.gains_in_force(), Gains::MAX);
}

/// A controller driven through `steps` of attitude and rate setpoints with moving rates; the torques, the thrust and
/// the rate log of every step as text.
fn drive(pid: &mut CascadedPid, steps: usize, mut hook: impl FnMut(usize, &mut CascadedPid)) -> Vec<String> {
    let sat = Saturation { roll: false, pitch: false, yaw: false, thrust_low: false, thrust_high: false };
    let mut seed = 9u32;
    (0..steps)
        .map(|k| {
            hook(k, pid);
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let noise = (seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5;
            let x = k as f32 * 0.02;
            let mut nav = nav_at(0.0, ANGLE);
            nav.t_us = 1_000_000 + k as u64 * DT_US;
            // Large rates, so that the integrators reach their limits and the D term is live.
            nav.rates_rads = [3.0 * libm::sinf(x) + noise, 2.0 * libm::cosf(1.3 * x), 1.5 * libm::sinf(0.5 * x) + noise];
            let sp = if k % 400 < 200 {
                Setpoint::Attitude {
                    attitude: [0.9, 0.3, 0.1, 0.2].map(|v| v / libm::sqrtf(0.9 * 0.9 + 0.3 * 0.3 + 0.1 * 0.1 + 0.2 * 0.2)),
                    heading_rate_rads: 1.0 * libm::sinf(x),
                    thrust_n: MASS * GRAVITY_MSS,
                }
            } else {
                Setpoint::Rates { rates_rads: [4.0, -4.0, 40.0], thrust_n: MASS * GRAVITY_MSS }
            };
            let d = pid.update(&nav, &sp, &sat);
            std::format!("{d:?} {:?}", pid.rate_log())
        })
        .collect()
}

/// A tuned set that moves gain `i` by 1.4 (inside every range), or all 14 for `None`.
fn tuned_set(i: Option<usize>) -> Gains {
    let mut g = defaults();
    for (j, v) in g.0.iter_mut().enumerate() {
        if i.is_none_or(|i| i == j) {
            *v *= 1.4;
        }
    }
    assert!(g.in_range());
    g
}

#[test]
fn new_of_the_tuned_tuning_equals_new_of_the_defaults_then_set_gains() {
    let cases = (0..GAIN_COUNT).map(Some).chain([None]).chain([Some(99)]);
    let plain = drive(&mut CascadedPid::new(&Tuning::DEFAULT, 800.0), 1_000, |_, _| {});
    for case in cases {
        let g = match case {
            Some(99) => Gains::MAX,
            c => tuned_set(c),
        };
        let name = case.and_then(|i| GAIN_NAMES.get(i)).copied().unwrap_or("all or MAX");
        let built = drive(&mut CascadedPid::new(&Tuning::DEFAULT.with_gains(g), 800.0), 1_000, |_, _| {});
        let set = drive(&mut CascadedPid::new(&Tuning::DEFAULT, 800.0), 1_000, |k, p| {
            if k == 0 {
                p.set_gains(&g);
            }
        });
        assert_eq!(built, set, "{name}");
        assert_ne!(built, plain, "{name}: the run is sensitive to this gain");
    }
}

#[test]
fn set_gains_leaves_the_integrators_and_the_filter_memories_alone() {
    let base = drive(&mut CascadedPid::new(&Tuning::DEFAULT, 800.0), 1_000, |_, _| {});
    let every = drive(&mut CascadedPid::new(&Tuning::DEFAULT, 800.0), 1_000, |_, p| p.set_gains(&defaults()));
    assert_eq!(base, every);
}

/// The digest of `script` (every step's `Debug` text and its step block) on the flight step before the hands-off wiring
/// (commit c270814).
const SCRIPT_GOLDEN: u64 = 0x2ed6_7fa3_9e94_9fb8;

/// Decision-mavlink-mode-20261123, I2: with FS_HOLD_EN 0 (the default) the scripted flight is bit-identical to the step
/// before the hands-off wiring.
#[test]
fn the_script_is_bit_identical_to_the_step_before_the_hands_off_wiring() {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for (text, bytes, _) in script(|_, _| {}) {
        for &b in text.as_bytes().iter().chain(bytes.iter()) {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0100_0000_01b3);
        }
    }
    std::println!("script digest {h:#018x}");
    assert_eq!(h, SCRIPT_GOLDEN);
}
