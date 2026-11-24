//! Host tests of the vertical descent laws on a vertical point mass with a lag: LAND's E2 stages from a hover (FX-2
//! item 3) and the descent review's cases (decision-mavlink-mode-20261123, section 48) on quadratic drag. The estimator is
//! `althold_tests`' fake; the estimate is exact.

extern crate std;

use super::althold_tests::{DT_US, TestFlight, flight, input, nav_at, report};
use super::*;
use core::cell::Cell;

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

/// The vertical point mass: thrust `gain` x the commanded collective through a first-order motor lag `tau` (s), linear
/// drag `k` (1/s) and quadratic drag `kq` (1/m), the ground at d = 0 (no bounce). IMU1 reads the specific force; the
/// estimate (height, v_D) is exact unless `vd_frozen` holds a v_D.
struct Plant {
    f: TestFlight<'static>,
    cell: &'static Cell<NavState>,
    t: u64,
    gain: f32,
    tau: f32,
    k: f32,
    kq: f32,
    d: f32,
    v: f32,
    demand: f32,
    thrust: f32,
    vd_frozen: Option<f32>,
    /// The station is heard on the radio link at every step.
    station: bool,
}

impl Plant {
    fn new(gain: f32, tau: f32, k: f32, kq: f32) -> Plant {
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(0.0, POSITION))));
        let mut f = flight(cell, handshake(), &Tuning::DEFAULT);
        f.set_heading_ok(true);
        Plant { f, cell, t: 1_000_000, gain, tau, k, kq, d: 0.0, v: 0.0, demand: 0.0, thrust: 0.0, vd_frozen: None, station: false }
    }

    fn on_ground(&self) -> bool {
        self.d >= 0.0
    }

    /// Net acceleration down, m/s^2.
    fn accel(&self) -> f32 {
        GRAVITY_MSS - (self.gain * self.thrust / MASS + self.k * self.v + self.kq * self.v * libm::fabsf(self.v))
    }

    /// One step: the plant moves under the lagged collective, then the flight steps with report `rc`.
    fn step(&mut self, rc: Option<(bool, f32, Switch3)>) -> StepOut {
        let dt = DT_US as f32 * 1e-6;
        self.thrust += if self.tau > 0.0 { (self.demand - self.thrust) * (dt / self.tau).min(1.0) } else { self.demand - self.thrust };
        let mut a = self.accel();
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
        (n.position_m[2], n.velocity_ms[2]) = (self.d, self.vd_frozen.unwrap_or(self.v));
        self.cell.set(n);
        self.t += DT_US;
        if self.station {
            self.f.set_last_radio_heartbeat_us(Some(self.t));
        }
        let mut i = input(self.t, rc.map(|(arm, thr, sw)| report(self.t, arm, thr, sw)));
        i.sensors.imu1.reading.accel_ms2 = [0.0, 0.0, a - GRAVITY_MSS];
        let o = self.f.step(&i);
        self.demand = o.demand.map_or(0.0, |d| d.thrust_n);
        o
    }

    /// Armed on the ground in Position, a climb at full stick to `h` m, then `hover_s` of hover at mid stick.
    fn hover_at(&mut self, h: f32, hover_s: f32) {
        self.step(Some((false, 0.0, Switch3::Middle)));
        self.step(Some((true, 0.0, Switch3::Middle)));
        while self.d > -h {
            assert!(self.step(Some((true, 1.0, Switch3::Middle))).armed && self.t < 200_000_000);
        }
        for _ in 0..(hover_s / 1.25e-3) as u32 {
            assert!(self.step(Some((true, 0.5, Switch3::Middle))).armed);
        }
    }
}

/// The plant of the LAND tests: gain 1, a 50 ms motor lag, the linear drag of `n14_tests` (0.197 / 0.914 1/s).
fn land_plant() -> Plant {
    Plant::new(1.0, 0.05, 0.197 / 0.914, 0.0)
}

/// LAND's descent from its entry to 0.3 m above the ground (FX-2 item 3): (entry height, highest rise above it, time from
/// the entry to 2 m below it, mean rate from 2 m below the entry to 11 m, the largest |v_D - 1.5| from 3 s after the entry to
/// 11 m, the largest |v_D - 0.5| at 9 m and below, the largest thrust change in one step between 12 m and 9 m, the largest
/// stage rate). Every step armed with the motors running and in LAND.
struct Descent {
    h0: f32,
    rise: f32,
    t_2m: f32,
    mean: f32,
    dv_high: f32,
    dv_low: f32,
    dthrust: f32,
    rate_max: f32,
}

fn descend(p: &mut Plant) -> Descent {
    assert_eq!(p.f.hands_off_state(), State::Land);
    let h0 = -p.d;
    let (mut k, mut rise, mut t_2m, mut t_11) = (0u32, 0.0f32, None::<f32>, None::<f32>);
    let (mut dv_high, mut dv_low, mut dthrust, mut rate_max) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    let mut last = p.demand;
    while p.d < -0.3 {
        let o = p.step(None);
        k += 1;
        assert!(o.armed && o.motors.throttles().is_some() && k < 200_000, "step {k}");
        assert_eq!(p.f.hands_off_state(), State::Land);
        let (h, t) = (-p.d, k as f32 * DT_US as f32 * 1e-6);
        rise = rise.max(h - h0);
        rate_max = rate_max.max(p.f.land_rate_ms.unwrap_or(0.0));
        if h <= h0 - 2.0 {
            t_2m.get_or_insert(t);
        }
        if h <= 11.0 {
            t_11.get_or_insert(t);
        }
        if t >= 3.0 && h > 11.0 {
            dv_high = dv_high.max(libm::fabsf(p.v - LAND_SPEED_HIGH_MS));
        }
        if h <= 9.0 {
            dv_low = dv_low.max(libm::fabsf(p.v - LAND_SPEED_MS));
        }
        if (9.0..=12.0).contains(&h) {
            dthrust = dthrust.max(libm::fabsf(p.demand - last));
        }
        last = p.demand;
    }
    let (t_2m, t_11) = (t_2m.unwrap_or(f32::NAN), t_11.unwrap_or(f32::NAN));
    Descent { h0, rise, t_2m, mean: (h0 - 2.0 - 11.0) / (t_11 - t_2m), dv_high, dv_low, dthrust, rate_max }
}

/// A 10 s hover at 5 m in Position with FS_HOLD_EN 1, the station heard, the square uploaded with return height `top`;
/// the sticks lost (HOLD), 1 s later Return at home: the climb in place to `top`, Home, LAND.
fn return_at_home(top: f32) -> Plant {
    use crate::hands_off::{Command, Link, StationCommand};
    let mut p = land_plant();
    p.station = true;
    p.f.set_fs_hold_en(true);
    p.f.set_battery_ok(true);
    let mut m = super::auto_tests::square();
    m.return_m = top;
    p.f.set_mission(m);
    p.hover_at(5.0, 10.0);
    while p.f.hands_off_state() == State::Pilot {
        assert!(p.step(None).armed && p.t < 100_000_000);
    }
    for _ in 0..800 {
        p.step(None);
    }
    assert_eq!(p.f.hands_off_state(), State::Hold);
    p.f.set_command(StationCommand { command: Command::Return, link: Link::Radio });
    while p.f.hands_off_state() != State::Land {
        assert!(p.step(None).armed && p.t < 200_000_000);
        assert!(matches!(p.f.hands_off_state(), State::Hold | State::Return | State::Land));
    }
    p
}

/// A 10 s hover at `h` m in Position, then the sticks lost with FS_HOLD_EN 0: LAND (T3) at once.
fn pilot_hover_lost(h: f32) -> Plant {
    let mut p = land_plant();
    p.hover_at(h, 10.0);
    while p.f.hands_off_state() == State::Pilot {
        assert!(p.step(None).armed && p.t < 100_000_000);
    }
    p
}

/// FX-2 item 3 (section 56, TO LOOK AT 1): LAND from a hover at 20 m and at 50 m on a plant with a lag, after RETURN's
/// climb in place at home and after a stick loss: no climb after the entry (at most 0.3 m above it), 2 m below the entry
/// within 3 s, the mean rate from 2 m below the entry to 11 m at least 1.35 m/s, 1.5 m/s +- 0.15 from 3 s after the entry
/// to 11 m, 0.5 m/s +- 0.05 at 9 m and below, no thrust step at the stage change (under 0.05 N per step between 12 m and
/// 9 m). Before the fix RETURN reported Home while the vehicle still climbed at 3 m/s, 0.6 m below H_r, and LAND's first
/// rate was that climb (N11 (2)): it rose 5.2 m above the entry and took 13.5 s from 19.4 m to 10.5 m.
#[test]
fn fx2_land_from_a_hover_at_20_and_50_m_reaches_e2s_first_stage() {
    for (name, top) in [("RETURN at home", 20.0f32), ("RETURN at home", 50.0), ("stick loss", 20.0), ("stick loss", 50.0)] {
        let mut p = if name == "stick loss" { pilot_hover_lost(top) } else { return_at_home(top) };
        let v0 = p.v;
        let r = descend(&mut p);
        std::println!(
            "FX-2 (3) {name} {top} m: LAND at {:.2} m, v_D {v0:.2} m/s; rise {:.2} m; 2 m below in {:.2} s; mean {:.3} m/s to 11 m; |v-1.5| max {:.3}, |v-0.5| max at 9 m and below {:.3} m/s; thrust step max {:.4} N",
            r.h0, r.rise, r.t_2m, r.mean, r.dv_high, r.dv_low, r.dthrust
        );
        assert!(r.rise <= 0.3 && r.t_2m <= 3.0, "{name} {top}: rise {} t {}", r.rise, r.t_2m);
        assert!(r.mean >= 1.35 && r.dv_high <= 0.15 && r.dv_low <= 0.05, "{name} {top}: {} {} {}", r.mean, r.dv_high, r.dv_low);
        assert!(r.dthrust < 0.05, "{name} {top}: {}", r.dthrust);
    }
}

/// FX-2 item 3: LAND from a 5 m hover is unchanged: the stage rate is 0.5 m/s throughout (from v_D 0 at the entry), no rise.
#[test]
fn fx2_land_from_5_m_descends_at_0_5_m_s() {
    let mut p = pilot_hover_lost(5.0);
    let r = descend(&mut p);
    std::println!("FX-2 (3) 5 m: LAND at {:.2} m; largest stage rate {:.3} m/s; rise {:.3} m", r.h0, r.rate_max, r.rise);
    assert!(r.rate_max <= LAND_SPEED_MS && r.rise <= 0.1);
}

/// The quadratic-drag plant of section 48: no linear drag, a 50 ms motor lag, gain `gain`, and kq such that the terminal
/// fall at the allocator's idle floor is 15 m/s: kq = (g - gain floor / m) / 15^2 (1/m).
fn quadratic_plant(gain: f32) -> Plant {
    let mut p = Plant::new(gain, 0.05, 0.0, 0.0);
    p.kq = (GRAVITY_MSS - gain * p.f.floor_n / MASS) / (15.0 * 15.0);
    p
}

/// FX-2 item 4 (a), section 48: a pilot throttle chop in Angle of 5 s from 60 m, then a stick loss (FS_HOLD_EN 0): the
/// accepted residual. On this plant the 5 s chop reaches the ground before the loss, so a 3 s chop is flown too.
/// Asserted only what the code is designed to do: a loss in the air gives LAND, or STOP when TD already held at the
/// loss (WR1); the numbers printed.
#[test]
fn fx2_quadratic_drag_throttle_chop_5_s_from_60_m_then_stick_loss() {
  for chop_s in [5.0f32, 3.0] {
    let mut p = quadratic_plant(1.0);
    p.hover_at(60.0, 5.0);
    for _ in 0..(chop_s / 1.25e-3) as u32 {
        p.step(Some((true, 0.0, Switch3::Low)));
    }
    let (h_loss, v_loss, td_loss) = (-p.d, p.v, p.f.still_us);
    let held = td_loss >= Tuning::DEFAULT.land_detect_us;
    let (mut k, mut first, mut stop_h, mut v_min_h) = (0u32, None::<State>, None::<f32>, (f32::NAN, f32::NAN));
    while p.d < -0.05 && k < 80_000 {
        let o = p.step(None);
        k += 1;
        let s = p.f.hands_off_state();
        if s != State::Pilot {
            first.get_or_insert(s);
        }
        if !o.armed {
            stop_h.get_or_insert(-p.d);
        }
        if k == 8_000 {
            v_min_h = (p.v, -p.d);
            std::println!("FX-2 (4a) chop {chop_s} s, 10 s after the loss: {:?} in {:?}, latch {}, ever airborne {}, H valid {}", p.f.hands_off_state(), o.mode, p.f.airborne, p.f.hands_off.ever_airborne(), p.f.hover_us >= HOVER_VALID_US);
        }
    }
    std::println!(
        "FX-2 (4a) chop {chop_s} s from 60 m: at the loss {h_loss:.1} m, v_D {v_loss:.2} m/s, TD {td_loss} us (held {held}); first hands-off state {first:?}; stop at height {stop_h:?} m; v_D {:.2} m/s at {:.1} m 10 s after the loss; end after {:.1} s at {:.2} m/s",
        v_min_h.0, v_min_h.1, k as f32 * 1.25e-3, p.v
    );
    match first {
        _ if h_loss < 0.05 => {}
        Some(State::Land) => {}
        Some(State::Stop) => assert!(held, "STOP at the loss without TD held"),
        s => panic!("{s:?}"),
    }
  }
}

/// FX-2 item 4 (a): BLIND entered from a 5 m/s descent (Angle at throttle 0.3 from a hover at 60 m, then the sticks and
/// the height lost together; from 150 m so that 10 s of BLIND fit): the probe aborts, no stop in the air, BLIND's collective never above its entry value; the
/// descent rate 10 s after the entry printed.
#[test]
fn fx2_quadratic_drag_blind_from_a_5_m_s_descent() {
    let mut p = quadratic_plant(1.0);
    p.hover_at(150.0, 20.0);
    while p.v < 5.0 {
        assert!(p.step(Some((true, 0.3, Switch3::Low))).armed && p.d < -10.0);
    }
    let v_loss = p.v;
    p.cell.set(NavState { valid: super::althold_tests::ANGLE, ..p.cell.get() });
    while p.f.hands_off_state() != State::Blind {
        assert!(p.step(None).armed && p.d < -5.0);
    }
    let (h0, v0, c0) = (-p.d, p.v, p.demand);
    let (mut probes, mut aborted, mut was, mut c_max, mut v_10s, mut k) = (0u32, 0u32, p.f.probe_us, c0, f32::NAN, 0u32);
    while p.d < -1.0 && k < 48_000 {
        let o = p.step(None);
        k += 1;
        assert!(o.armed, "a stop in the air at {:.1} m", -p.d);
        assert_eq!(p.f.hands_off_state(), State::Blind);
        c_max = c_max.max(p.demand);
        let now = p.f.probe_us;
        probes += u32::from(was.is_none() && now.is_some());
        aborted += u32::from(was.is_some() && now.is_none());
        was = now;
        if k == 8_000 {
            v_10s = p.v;
        }
    }
    std::println!(
        "FX-2 (4a) BLIND from {v_loss:.2} m/s: entry at {h0:.1} m, v_D {v0:.2} m/s, collective {c0:.3} N (largest {c_max:.3}); probes {probes}, aborted {aborted}; v_D 10 s after the entry {v_10s:.2} m/s; {:.1} m after {:.1} s",
        -p.d, k as f32 * 1.25e-3
    );
    assert!(probes > 0 && aborted + 1 >= probes, "{probes} {aborted}");
    assert!(c_max <= c0 + 1e-3, "{c_max} {c0}");
}

/// FX-2 item 4 (a): LAND from a hover at 20 m with the v_D estimate frozen at a faulty value from the stick loss on
/// (FS_HOLD_EN 0): +3 m/s (down, the loop adds thrust) and -3 m/s (up, the loop commands the idle floor). Printed: the
/// time until the commanded collective first reaches the floor, the ground contact and whether TD then holds in the air
/// (20 s); asserted: no stop within the first 3 s.
#[test]
fn fx2_quadratic_drag_land_with_v_d_frozen() {
    for vd in [3.0f32, -3.0] {
        let mut p = quadratic_plant(1.0);
        p.hover_at(20.0, 10.0);
        p.vd_frozen = Some(vd);
        let (mut k, mut floor, mut stop) = (0u32, None::<f32>, None::<(f32, f32, f32)>);
        let mut contact = None::<f32>;
        while k < 16_000 {
            let o = p.step(None);
            k += 1;
            let t = k as f32 * 1.25e-3;
            if p.on_ground() {
                contact.get_or_insert(t);
            }
            if o.allocation.is_none_or(|a| a.saturation.thrust_low) {
                floor.get_or_insert(t);
            }
            if !o.armed {
                stop = Some((t, -p.d, p.v));
                break;
            }
            assert!(t > 3.0 || o.armed);
        }
        std::println!(
            "FX-2 (4a) LAND with v_D frozen at {vd:+} m/s from 20 m: floor first after {floor:?} s; ground contact {contact:?} s; stop (s, height m, v_D m/s) {stop:?}; at the end {:.1} m, {:.2} m/s after {:.1} s, {:?}",
            -p.d, p.v, k as f32 * 1.25e-3, p.f.hands_off_state()
        );
        assert!(stop.is_none_or(|(t, _, _)| t > 3.0));
    }
}

/// FX-2 item 4 (b), section 48: a gentle take-off in Angle at thrust gain 1.46 never sets the 0.714 latch; the stick loss
/// with FS_HOLD_EN 1 (station heard) and with FS_HOLD_EN 0 lands without a stop, and LAND's first mode is Altitude, not
/// Position (the (state, mode) pairs in order printed).
#[test]
fn fx2_hover_below_the_latch_hold_and_land_fly_altitude() {
    for fs in [true, false] {
        let mut p = Plant::new(1.46, 0.0, 0.0, 0.0);
        p.station = true;
        p.f.set_fs_hold_en(fs);
        p.step(Some((false, 0.0, Switch3::Low)));
        p.step(Some((true, 0.0, Switch3::Low)));
        let (climb, hover) = (p.f.shaper.throttle_for(1.03 * MASS * GRAVITY_MSS / 1.46), p.f.shaper.throttle_for(MASS * GRAVITY_MSS / 1.46));
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
        assert!(!latch && p.f.hands_off.ever_airborne());
        let (mut modes, mut states) = (std::vec::Vec::new(), std::vec::Vec::new());
        for k in 0..2_400 {
            let o = p.step(None);
            let s = p.f.hands_off_state();
            if !states.contains(&s) {
                states.push(s);
            }
            assert!(o.armed && s != State::Stop && p.d < -1.0, "step {k}: {s:?}");
            if !modes.contains(&(s, o.mode)) {
                modes.push((s, o.mode));
            }
        }
        std::println!("FX-2 (4b) gain 1.46 below the latch, FS_HOLD_EN {fs}: (state, mode) {modes:?}; states {states:?}; latch at the end {}", p.f.airborne);
        assert_eq!(modes.get(1), Some(&(State::Land, Mode::Altitude)), "{modes:?}");
    }
}

/// FX-2 item 4 (c), section 48: WR1's modelled clear agrees with the real one, deterministically (the wr3 walk counts it
/// without requiring it). HOLD at 10 m on the fake estimator, the estimate then 3 m above the held height (the vertical
/// loop at the idle floor), IMU1 at rest: at a hands-off step with TD held and the airborne latch still set the
/// test clears the latch; that step ends disarmed in STOP. TD is put at its hold time directly, as the latch clears itself
/// on the same idle-floor second TD needs (that is why the walk rarely sees the case). Without the clear the step stays
/// armed in HOLD and STOP follows when the latch clears itself.
#[test]
fn fx2_wr1_modelled_clear_agrees_with_the_real_one() {
    let mut results = std::vec::Vec::new();
    for modelled in [true, false] {
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(-10.0, POSITION))));
        let mut f = flight(cell, handshake(), &Tuning::DEFAULT);
        f.set_heading_ok(true);
        f.set_fs_hold_en(true);
        let mut t = 1_000_000u64;
        let mut step = |f: &mut TestFlight<'static>, rc: Option<(bool, f32)>| {
            t += DT_US;
            f.set_last_radio_heartbeat_us(Some(t));
            let mut i = input(t, rc.map(|(arm, thr)| report(t, arm, thr, Switch3::High)));
            i.sensors.imu1.reading.accel_ms2 = [0.0, 0.0, -GRAVITY_MSS];
            f.step(&i)
        };
        step(&mut f, Some((false, 0.0)));
        step(&mut f, Some((true, 0.0)));
        for _ in 0..400 {
            step(&mut f, Some((true, 0.5)));
        }
        assert!(f.airborne);
        while f.hands_off_state() == State::Pilot {
            step(&mut f, None);
        }
        assert_eq!(f.hands_off_state(), State::Hold);
        let mut n = cell.get();
        n.position_m[2] = -13.0;
        cell.set(n);
        let mut n_steps = 0u32;
        while f.still_us == 0 {
            step(&mut f, None);
            n_steps += 1;
            assert!(n_steps < 8_000, "TD never counts");
        }
        assert!(f.still_us > 0 && f.airborne && f.hands_off_state() == State::Hold, "{} {} {:?}", f.still_us, f.airborne, f.hands_off_state());
        f.still_us = Tuning::DEFAULT.land_detect_us;
        if modelled {
            f.airborne = false;
        }
        let o = step(&mut f, None);
        let first = (o.armed, f.hands_off_state());
        let mut k = 1u32;
        while f.gate.armed() {
            step(&mut f, None);
            k += 1;
            assert!(k < 20_000, "never stopped");
        }
        std::println!("FX-2 (4c) modelled clear {modelled}: TD counting after {n_steps} steps; the step after TD held: (armed, state) {first:?}; STOP after {k} steps");
        results.push((first, k));
    }
    assert_eq!(results[0], ((false, State::Stop), 1), "the modelled clear: STOP on that step");
    assert!(results[1].0 == (true, State::Hold) && results[1].1 > 1, "the latch still set: armed until it clears itself");
}
