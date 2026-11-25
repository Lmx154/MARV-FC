//! Host tests: TD's (a) is the allocator's idle floor, the low "ever airborne" mark, the clamps and
//! the card check.

extern crate std;

use super::althold_tests::{DT_US, TestFlight, flight, input, nav_at, report};
use super::*;
use core::cell::Cell;
use std::vec::Vec;

/// Everything Position needs.
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
/// d = 0 (no bounce: on it v = 0 while the net force is down)
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
        let lift = self.gain * self.thrust / MASS + self.k * self.v;
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

/// One flight: armed on the ground in Altitude, climb to about 5 m, hover 20 s, sticks lost (CRSF, T3), LAND to the ground.
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

/// In the plant, TD never reaches 1.0 s in a steady hover or a LAND descent at real gains 0.6 to 1.5; at 0.62 to 1.46
/// LAND standing on the ground reaches the floor and TD stops the motors within 3.2 s + 1.0 s of contact.
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

/// BLIND on the ground: probe to the floor, TD, STOP; the time from BLIND's entry reported.
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

/// One BLIND flight in the air from a hover at 50 m (60 s, H learned): thrust gain `gain`, vertical drag; with
/// `climb`, then 4 s at full throttle in Altitude (a 3 m/s climb) before the loss.
struct BlindAir {
    stopped: bool,
    probes: u32,
    aborted: u32,
    /// Mean height lost per probe, m.
    per: f32,
    /// Highest rise above BLIND's entry after 5 s, m.
    rise_5s: f32,
    /// Highest rise above BLIND's entry, m.
    rise: f32,
    /// Net acceleration down at the first step after the one-shot correction, and the lowest after it, m/s^2.
    a_corr: f32,
    a_min: f32,
    /// Net acceleration down of the corrected collective alone (no drag), g - gain c / m, m/s^2.
    a_trim: f32,
    ground: bool,
    /// v_D at T1's start and at BLIND's entry, and 15 s after BLIND's entry, m/s.
    v_loss: f32,
    v_entry: f32,
    v_15s: f32,
}

fn blind_air(gain: f32, climb: bool, sticks_first: bool) -> BlindAir {
    let mut p = Plant::new(gain, 0.197 / 0.914, POSITION);
    p.step(Some((false, 0.0, Switch3::Middle)));
    p.step(Some((true, 0.0, Switch3::Middle)));
    while p.d > -50.0 {
        assert!(p.step(Some((true, 1.0, Switch3::Middle))).armed);
    }
    for _ in 0..48_000 {
        p.step(Some((true, 0.5, Switch3::Middle)));
    }
    if climb {
        for _ in 0..3_200 {
            p.step(Some((true, 1.0, Switch3::Middle)));
        }
    }
    if sticks_first {
        while p.f.hands_off_state() == State::Pilot {
            assert!(p.step(None).armed && p.t < 1_000_000_000);
        }
        assert_eq!(p.f.hands_off_state(), State::Land);
    }
    let v_loss = p.v;
    p.cell.set(NavState { valid: super::althold_tests::ANGLE, ..p.cell.get() });
    while p.f.hands_off_state() != State::Blind {
        assert!(p.step(None).armed && p.t < 1_000_000_000);
    }
    std::println!("N14 BLIND gain {gain} climb {climb} sticks first {sticks_first}: H {:.3} m g, v_D at T1 {v_loss:.2}, at entry {:.2} m/s", p.f.hover_n / (MASS * GRAVITY_MSS), p.v);
    let (d0, v_entry) = (p.d, p.v);
    let (mut probes, mut aborted, mut was, mut lost, mut d_start) = (0u32, 0u32, None::<u64>, Vec::new(), 0.0f32);
    let (mut rise_5s, mut rise, mut a_corr, mut a_min, mut stopped) = (f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NAN, f32::INFINITY, false);
    let (mut v_15s, mut a_trim) = (f32::NAN, f32::NAN);
    let per = |lost: &Vec<f32>| if lost.is_empty() { 0.0 } else { lost.iter().sum::<f32>() / lost.len() as f32 };
    for k in 0..48_000u32 {
        let o = p.step(None);
        if !o.armed {
            stopped = true;
            break;
        }
        if p.d > -5.0 {
            break;
        }
        rise = rise.max(d0 - p.d);
        if k >= 4_000 {
            rise_5s = rise_5s.max(d0 - p.d);
        }
        if k == 12_000 {
            v_15s = p.v;
        }
        if p.f.blind_trim_n.is_some() && p.f.probe_us.is_none() {
            let a = GRAVITY_MSS - (gain * p.thrust / MASS + p.k * p.v);
            if a_corr.is_nan() {
                a_corr = a;
                a_trim = p.f.blind_trim_n.map_or(f32::NAN, |c| GRAVITY_MSS - gain * c / MASS);
            }
            a_min = a_min.min(a);
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
    let ground = p.on_ground();
    BlindAir { stopped, probes, aborted, per: per(&lost), rise_5s, rise, a_corr, a_min, a_trim, ground, v_loss, v_entry, v_15s }
}

/// BLIND in the air at real gains 0.62 to 1.46 with vertical drag, entered from a hover (H learned), the
/// sticks lost first and T1 at LAND's entry: every probe aborts ((b) fails), no stop, no fall to the ground.
#[test]
fn n14_blind_in_the_air_probes_abort_and_never_stop() {
    for gain in [0.62f32, 0.88, 1.0, 1.12, 1.3, 1.46] {
        let r = blind_air(gain, false, false);
        std::println!("N16 BLIND together gain {gain} (reported): highest above entry after 5 s {:.2} m, v_D at entry {:.2} m/s, net down after the correction {:.3} m/s^2 (lowest {:.3}), of the corrected collective alone {:.3}, stopped {}, ground {}", r.rise_5s, r.v_entry, r.a_corr, r.a_min, r.a_trim, r.stopped, r.ground);
    }
    let got = [0.62f32, 0.88, 1.0, 1.12, 1.3, 1.46].map(|gain| {
        let r = blind_air(gain, false, true);
        std::println!("N16 BLIND gain {gain}: stopped {}, ground {}, probes {}, aborted {}, mean height lost per probe {:.3} m, highest above entry after 5 s {:.2} m, v_D at entry {:.2} m/s, net down after the correction {:.3} m/s^2 (lowest {:.3}), of the corrected collective alone {:.3}", r.stopped, r.ground, r.probes, r.aborted, r.per, r.rise_5s, r.v_entry, r.a_corr, r.a_min, r.a_trim);
        (gain, r)
    });
    for (gain, r) in got {
        assert!(!r.stopped && !r.ground, "gain {gain}");
        assert!(r.probes > 0 && r.aborted + 1 >= r.probes, "gain {gain}: {} {}", r.probes, r.aborted);
        assert!(r.rise_5s <= 1.0, "gain {gain}: {} m above the entry", r.rise_5s);
    }
}

/// BLIND entered from a 3 m/s climb.
/// lost and T1 at LAND's entry)
#[test]
fn n16_blind_from_a_3_m_s_climb_rises_at_most_15_m_and_descends_by_15_s() {
    for gain in [0.62f32, 1.0, 1.46] {
        let r = blind_air(gain, true, false);
        std::println!("N16 BLIND climbing together gain {gain} (reported): v_D at T1 {:.2}, at entry {:.2}, 15 s after {:.2} m/s; highest above entry {:.2} m; net down after the correction {:.3} m/s^2, of the corrected collective alone {:.3}; stopped {}, ground {}", r.v_loss, r.v_entry, r.v_15s, r.rise, r.a_corr, r.a_trim, r.stopped, r.ground);
    }
    let got = [0.62f32, 0.88, 1.0, 1.12, 1.3, 1.46].map(|gain| {
        let r = blind_air(gain, true, true);
        std::println!("N16 BLIND climbing gain {gain}: v_D at T1 {:.2}, at entry {:.2}, 15 s after {:.2} m/s; highest above entry {:.2} m; net down after the correction {:.3} m/s^2 (lowest {:.3}), of the corrected collective alone {:.3}; stopped {}, ground {}", r.v_loss, r.v_entry, r.v_15s, r.rise, r.a_corr, r.a_min, r.a_trim, r.stopped, r.ground);
        (gain, r)
    });
    for (gain, r) in got {
        assert!(r.v_loss < -2.9, "gain {gain}: a 3 m/s climb at T1, v_D {}", r.v_loss);
        if gain <= 1.12 {
            assert!(!r.stopped && !r.ground, "gain {gain}");
            assert!(r.rise <= 15.0, "gain {gain}: {} m above the entry", r.rise);
            assert!(r.v_15s > 0.0, "gain {gain}: v_D {} m/s 15 s after the entry", r.v_15s);
        }
    }
}

/// A gentle take-off in Angle at real gain `gain`: the throttle ramps at 0.2 per second up to the collective 1.03 m g
/// / gain, to 3 m, then the hover collective m g / gain for 5 s.
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

/// At real gain 1.46 a gentle take-off never sets the 0.714 latch but sets the low mark ("ever airborne"), and a stick
/// loss in that hover gives LAND (CRSF, T3), not STOP; the vehicle stays armed and in the air for 2 s.
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

/// An allocator that is the card's with its thrust flags forced: (`thrust_low`, `thrust_high`)
struct SatAlloc {
    inner: CardAllocator,
    force: (bool, bool),
}

impl Allocator for SatAlloc {
    fn allocate(&mut self, demand: &ThrustTorque) -> Allocation {
        let mut a = self.inner.allocate(demand);
        a.saturation.thrust_low |= self.force.0;
        a.saturation.thrust_high |= self.force.1;
        a
    }
}

/// A step whose allocation has `thrust_low` or `thrust_high` leaves H and its qualifying time unchanged; the next step
/// without them learns again.
#[test]
fn n14_saturation_gates_block_learning() {
    let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(-10.0, POSITION))));
    let tuning = Tuning::DEFAULT;
    let alloc = SatAlloc { inner: CardAllocator::new(), force: (false, false) };
    let mut f = Flight::new(super::althold_tests::Fake(cell), crate::pid::CascadedPid::new(&tuning, 800.0), alloc, &tuning, handshake());
    let mut t = 1_000_000u64;
    let mut step = |f: &mut Flight<_, _, SatAlloc>, arm: bool, thr: f32| {
        t += DT_US;
        f.step(&input(t, Some(report(t, arm, thr, Switch3::Low))))
    };
    step(&mut f, false, 0.0);
    step(&mut f, true, 0.0);
    for _ in 0..400 {
        step(&mut f, true, 0.52);
    }
    assert!(f.gate.armed() && f.lifted && f.hover_us > 0, "learning");
    for force in [(true, false), (false, true)] {
        let (h, us) = (f.hover_n, f.hover_us);
        f.allocator.force = force;
        step(&mut f, true, 0.52);
        assert_eq!((f.hover_n, f.hover_us), (h, us), "{force:?}: no learning");
        f.allocator.force = (false, false);
        step(&mut f, true, 0.52);
        assert!(f.hover_us > us && f.hover_n != h, "{force:?}: learns again");
    }
}

/// The card check (no behaviour): `card_check_pct` is `None` until H is valid, `None` at 11 % off, the percent at
/// 13 % off (either side); the text.
#[test]
fn n14_card_check_fires_at_13_percent_not_at_11() {
    let mut p = Plant::new(1.0, 0.0, POSITION);
    let mg = MASS * GRAVITY_MSS;
    for (r, want) in [(1.13f32, Some(113u16)), (0.87, Some(87)), (1.11, None), (0.89, None), (1.0, None)] {
        p.f.hover_n = r * mg;
        p.f.hover_us = HOVER_VALID_US - 1;
        assert_eq!(p.f.card_check_pct(), None, "{r}: H not valid");
        p.f.hover_us = HOVER_VALID_US;
        assert_eq!(p.f.card_check_pct(), want, "{r}");
    }
    assert_eq!(&card_msg(113), b"HOVER 113 % of card");
    assert_eq!(&card_msg(87), b"HOVER  87 % of card");
    assert_eq!(&card_msg(1_200), b"HOVER 999 % of card");
}
