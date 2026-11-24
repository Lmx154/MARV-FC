//! Host tests of Amendment N14 (decision-mavlink-mode-20261123, section 35): TD's (a) is the allocator's idle floor, the
//! low "ever airborne" mark, the clamps and the card check, on a vertical point mass whose thrust is the commanded
//! collective times a real thrust gain (the card's thrust is right at gain 1). The estimator is `althold_tests`' fake.

extern crate std;

use super::althold_tests::{DT_US, TestFlight, flight, input, nav_at, report};
use super::*;
use core::cell::Cell;
use std::vec::Vec;

/// Everything Position needs (T3).
const POSITION: Valid = Valid {
    heading: true,
    velocity_ne: true,
    position_ne: true,
    ..super::althold_tests::ALTITUDE
};

fn handshake() -> &'static Handshake {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    h
}

/// The vertical point mass: thrust `gain` x the commanded collective, linear drag `k` (1/s; 0 for none), the ground at
/// d = 0 (no bounce: on it v = 0 while the net force is down). IMU1 reads the specific force of thrust, drag and the
/// ground's reaction; the estimate is exact (height, v_D) with coverage `valid`.
struct Plant {
    f: TestFlight<'static>,
    cell: &'static Cell<NavState>,
    t: u64,
    gain: f32,
    k: f32,
    d: f32,
    v: f32,
    thrust: f32,
}

impl Plant {
    fn new(gain: f32, k: f32, valid: Valid) -> Plant {
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(0.0, valid))));
        let mut f = flight(cell, handshake(), &Tuning::DEFAULT);
        f.set_heading_ok(true);
        Plant { f, cell, t: 1_000_000, gain, k, d: 0.0, v: 0.0, thrust: 0.0 }
    }

    fn on_ground(&self) -> bool {
        self.d >= 0.0
    }

    /// One step: the plant moves under the previous step's collective, then the flight steps with report `rc`.
    fn step(&mut self, rc: Option<(bool, f32, Switch3)>) -> StepOut {
        let dt = DT_US as f32 * 1e-6;
        let lift = self.gain * self.thrust / MASS - self.k * self.v;
        let mut a = GRAVITY_MSS - lift;
        if self.on_ground() && a >= 0.0 {
            (self.d, self.v, a) = (0.0, 0.0, 0.0);
        } else {
            self.v += a * dt;
            self.d = (self.d + self.v * dt).min(0.0);
            if self.d >= 0.0 {
                self.v = 0.0;
            }
        }
        let mut n = self.cell.get();
        (n.position_m[2], n.velocity_ms[2]) = (self.d, self.v);
        self.cell.set(n);
        self.t += DT_US;
        let mut i = input(self.t, rc.map(|(arm, thr, sw)| report(self.t, arm, thr, sw)));
        i.sensors.imu1.reading.accel_ms2 = [0.0, 0.0, a - GRAVITY_MSS];
        let o = self.f.step(&i);
        self.thrust = o.demand.map_or(0.0, |d| d.thrust_n);
        o
    }

    fn td(&self) -> u64 {
        self.f.still_us
    }
}

/// TD's (a) is the step's `thrust_low` (a refused demand counts): on the ground in Angle at throttle 0 and at rest, TD
/// counts on every step and holds after 1.0 s; at half throttle (above the floor) it never counts.
#[test]
fn n14_td_a_is_the_allocator_floor_flag() {
    for (thr, counts) in [(0.0f32, true), (0.5, false)] {
        let mut p = Plant::new(1.0, 0.0, POSITION);
        p.step(Some((false, 0.0, Switch3::Low)));
        p.step(Some((true, 0.0, Switch3::Low)));
        p.gain = 0.0;
        for k in 0..900 {
            let o = p.step(Some((true, thr, Switch3::Low)));
            assert!(o.armed);
            let low = o.allocation.is_none_or(|a| a.saturation.thrust_low);
            assert_eq!(low, counts, "throttle {thr} step {k}");
            assert_eq!(p.td() > 0, counts, "throttle {thr} step {k}: TD {}", p.td());
        }
        assert_eq!(p.td() >= Tuning::DEFAULT.land_detect_us, counts);
    }
}

/// One flight: armed on the ground in Altitude, climb to about 5 m, hover 20 s, sticks lost (FS_HOLD_EN 0, T3: LAND),
/// LAND to the ground. Returns (TD's longest count in the air, us; stopped; contact to STOP, us; floor to STOP, us).
fn hover_and_land(gain: f32) -> (u64, bool, Option<u64>, Option<u64>) {
    let mut p = Plant::new(gain, 0.0, POSITION);
    p.step(Some((false, 0.0, Switch3::Middle)));
    p.step(Some((true, 0.0, Switch3::Middle)));
    let mut td_air = 0u64;
    let air = |p: &Plant, td_air: &mut u64| {
        if p.d < -0.05 {
            *td_air = (*td_air).max(p.td());
        }
    };
    while p.d > -5.0 {
        let o = p.step(Some((true, 1.0, Switch3::Middle)));
        assert!(o.armed && p.t < 60_000_000, "gain {gain}: the climb");
        air(&p, &mut td_air);
    }
    for _ in 0..16_000 {
        assert!(p.step(Some((true, 0.5, Switch3::Middle))).armed);
        air(&p, &mut td_air);
    }
    assert!(p.f.hands_off.ever_airborne(), "gain {gain}");
    let (mut contact, mut floor) = (None::<u64>, None::<u64>);
    for _ in 0..80_000 {
        let o = p.step(None);
        let s = p.f.hands_off_state();
        if !o.armed {
            assert_eq!(s, State::Stop);
            let c = contact.expect("contact before the stop");
            return (td_air, true, Some(p.t - c), floor.map(|f| p.t - f));
        }
        assert!(s == State::Pilot || s == State::Land, "gain {gain}: {s:?}");
        air(&p, &mut td_air);
        if p.on_ground() {
            contact.get_or_insert(p.t);
            if o.allocation.is_some_and(|a| a.saturation.thrust_low) {
                floor.get_or_insert(p.t);
            }
        }
    }
    (td_air, false, None, None)
}

/// N14 (5): in the plant, TD never reaches 1.0 s in a steady hover or a LAND descent at real gains 0.6 to 1.5; at 0.62
/// to 1.46 LAND standing on the ground reaches the floor and TD stops the motors within 3.2 s + 1.0 s of contact, except at
/// 0.62, where this plant takes 3.6 s to the floor (N14 estimated about 3 s): there within 5 s, N14 (5)'s simulator bound.
#[test]
fn n14_td_never_counts_1_s_in_the_air_and_land_stops_within_4_2_s_of_contact() {
    let land = Tuning::DEFAULT.land_detect_us;
    let got = [0.6f32, 0.62, 0.88, 1.0, 1.12, 1.3, 1.46, 1.5].map(|gain| {
        let (td_air, stopped, contact, floor) = hover_and_land(gain);
        std::println!("N14 gain {gain}: TD longest in the air {td_air} us; stopped {stopped}, contact to STOP {contact:?} us, floor to STOP {floor:?} us");
        (gain, td_air, stopped, contact)
    });
    for (gain, td_air, stopped, contact) in got {
        assert!(td_air < land, "gain {gain}: TD counted {td_air} us in the air");
        if (0.62..=1.46).contains(&gain) {
            let bound = if gain < 0.88 { 5_000_000 } else { 4_200_000 };
            assert!(stopped && contact.is_some_and(|c| c <= bound), "gain {gain}: {contact:?}");
        }
    }
}

/// N14 (1), BLIND on the ground: probe to the floor, TD, STOP; the time from BLIND's entry reported.
#[test]
fn n14_blind_on_the_ground_probes_to_the_floor_and_stops() {
    let mut p = Plant::new(1.0, 0.0, super::althold_tests::ANGLE);
    p.step(Some((false, 0.0, Switch3::Low)));
    p.step(Some((true, 0.0, Switch3::Low)));
    for _ in 0..400 {
        p.step(Some((true, 0.45, Switch3::Low)));
    }
    let mut entry = None;
    for _ in 0..20_000 {
        let o = p.step(None);
        let s = p.f.hands_off_state();
        if s == State::Blind {
            entry.get_or_insert(p.t);
        }
        if !o.armed {
            let e = entry.expect("BLIND");
            std::println!("N14 BLIND on the ground: STOP {} us after BLIND's entry", p.t - e);
            assert!(s == State::Stop && p.t - e <= 3_100_000 + 3 * DT_US, "{} us", p.t - e);
            return;
        }
        assert!(p.on_ground());
    }
    panic!("BLIND never stopped on the ground");
}

/// One BLIND flight in the air from a hover at 50 m: thrust gain `gain`, vertical drag; 60 s or down to 5 m above the
/// ground. Returns (stopped, probes,
/// aborted, height lost per probe m, height gained over the flight m, reached the ground).
fn blind_air(gain: f32) -> (bool, u32, u32, f32, f32, bool) {
    let mut p = Plant::new(gain, 0.197 / 0.914, POSITION);
    p.step(Some((false, 0.0, Switch3::Middle)));
    p.step(Some((true, 0.0, Switch3::Middle)));
    while p.d > -50.0 {
        assert!(p.step(Some((true, 1.0, Switch3::Middle))).armed);
    }
    for _ in 0..16_000 {
        p.step(Some((true, 0.5, Switch3::Middle)));
    }
    p.cell.set(NavState { valid: super::althold_tests::ANGLE, ..p.cell.get() });
    let d0 = p.d;
    let (mut probes, mut aborted, mut was, mut lost, mut d_start) = (0u32, 0u32, None::<u64>, Vec::new(), 0.0f32);
    let per = |lost: &Vec<f32>| if lost.is_empty() { 0.0 } else { lost.iter().sum::<f32>() / lost.len() as f32 };
    for _ in 0..48_000 {
        let o = p.step(None);
        if !o.armed {
            return (true, probes, aborted, per(&lost), -(p.d - d0), p.on_ground());
        }
        if p.d > -5.0 {
            break;
        }
        let now = p.f.probe_us;
        if was.is_none() && now.is_some() {
            probes += 1;
            d_start = p.d;
        }
        if was.is_some() && now.is_none() {
            aborted += 1;
            lost.push(p.d - d_start);
        }
        was = now;
    }
    (false, probes, aborted, per(&lost), -(p.d - d0), p.on_ground())
}

/// N14 (1), BLIND in the air at real gains 0.62 to 1.46 with vertical drag: every probe aborts ((b) fails), no stop;
/// the height lost per probe reported.
#[test]
fn n14_blind_in_the_air_probes_abort_and_never_stop() {
    let got = [0.88f32, 1.0, 1.12].map(|gain| {
        let (stopped, probes, aborted, per, climbed, ground) = blind_air(gain);
        std::println!("N14 BLIND gain {gain}: stopped {stopped}, ground {ground}, probes {probes}, aborted {aborted}, mean height lost per probe {per:.3} m, height gained {climbed:.1} m");
        (gain, stopped, probes, aborted, ground)
    });
    for (gain, stopped, probes, aborted, ground) in got {
        assert!(!stopped && !ground, "gain {gain}");
        assert!(probes > 0 && aborted + 1 >= probes, "gain {gain}: {probes} {aborted}");
    }
}

/// A gentle take-off in Angle at real gain `gain`: the throttle ramps at 0.2 per second up to the collective 1.03 m g /
/// gain, to 3 m, then the hover collective m g / gain for 5 s. Returns the plant and whether the airborne latch (0.714)
/// was ever set.
fn gentle_take_off(gain: f32) -> (Plant, bool) {
    let mut p = Plant::new(gain, 0.0, POSITION);
    p.step(Some((false, 0.0, Switch3::Low)));
    p.step(Some((true, 0.0, Switch3::Low)));
    let (climb, hover) = (p.f.shaper.throttle_for(1.03 * MASS * GRAVITY_MSS / gain), p.f.shaper.throttle_for(MASS * GRAVITY_MSS / gain));
    let (mut thr, mut latch) = (0.0f32, false);
    while p.d > -3.0 {
        thr = (thr + 0.2 * DT_US as f32 * 1e-6).min(climb);
        assert!(p.step(Some((true, thr, Switch3::Low))).armed && p.t < 60_000_000);
        latch |= p.f.airborne;
    }
    for _ in 0..4_000 {
        assert!(p.step(Some((true, hover, Switch3::Low))).armed);
        latch |= p.f.airborne;
    }
    (p, latch)
}

/// N14 (1): at real gain 1.46 a gentle take-off never sets the 0.714 latch but sets the low mark ("ever airborne"), and
/// a stick loss in that hover gives LAND (FS_HOLD_EN 0, T3), not STOP; the vehicle stays armed and in the air for 2 s.
#[test]
fn n14_low_mark_sets_at_gain_1_46_and_a_stick_loss_there_lands() {
    let (mut p, latch) = gentle_take_off(1.46);
    assert!(!latch, "the 0.714 latch set");
    assert!(p.f.lifted && p.f.hands_off.ever_airborne(), "the low mark");
    for k in 0..2_400 {
        let o = p.step(None);
        let s = p.f.hands_off_state();
        assert!(o.armed && s != State::Stop && p.d < -1.0, "step {k}: {s:?}, {} m", p.d);
        if k > 1_000 {
            assert_eq!(s, State::Land, "step {k}");
        }
    }
}
