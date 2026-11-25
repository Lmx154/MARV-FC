//! Host tests of the vertical descent laws on a vertical point mass with a lag: LAND's E2 stages from a hover.
//! item 3) and the descent review's cases on quadratic drag.

extern crate std;

use super::althold_tests::{DT_US, TestFlight, flight, input, nav_at, report};
use super::*;
use core::cell::Cell;

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

/// The vertical point mass: thrust `gain` x the commanded collective through a first-order motor lag `tau` (s), linear
/// drag `k` (1/s) and quadratic drag `kq` (1/m), the ground at d = 0 (no bounce)
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
    /// The height estimate's error, m down (the estimate is d + this)
    d_off: f32,
    /// Where the reports come from.
    source: fw_core::rc::RcSource,
}

impl Plant {
    fn new(gain: f32, tau: f32, k: f32, kq: f32) -> Plant {
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(0.0, POSITION))));
        let mut f = flight(cell, handshake(), &Tuning::DEFAULT);
        f.set_heading_ok(true);
        Plant { f, cell, t: 1_000_000, gain, tau, k, kq, d: 0.0, v: 0.0, demand: 0.0, thrust: 0.0, vd_frozen: None, station: false, d_off: 0.0, source: fw_core::rc::RcSource::Crsf }
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
        (n.position_m[2], n.velocity_ms[2]) = (self.d + self.d_off, self.vd_frozen.unwrap_or(self.v));
        self.cell.set(n);
        self.t += DT_US;
        if self.station {
            self.f.set_last_radio_heartbeat_us(Some(self.t));
        }
        let mut i = input(self.t, rc.map(|(arm, thr, sw)| RcSample { source: self.source, ..report(self.t, arm, thr, sw) }));
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

/// The plant of the LAND tests: gain 1, a 50 ms motor lag, the linear drag of `n14_tests` (0.197 / 0.914 1/s)
fn land_plant() -> Plant {
    Plant::new(1.0, 0.05, 0.197 / 0.914, 0.0)
}

/// LAND's descent from its entry to 0.3 m above the ground.
/// from the entry to 2 m below it, mean rate from 2 m below the entry to 11 m.
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
    p.source = fw_core::rc::RcSource::Receiver;
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

/// A 10 s hover at `h` m in Position, then the sticks lost with FS_HOLD_EN 0: LAND at once.
fn pilot_hover_lost(h: f32) -> Plant {
    let mut p = land_plant();
    p.hover_at(h, 10.0);
    while p.f.hands_off_state() == State::Pilot {
        assert!(p.step(None).armed && p.t < 100_000_000);
    }
    p
}

/// LAND from a hover at 20 m and at 50 m on a plant with a lag, after RETURN's climb in place at home and
/// after a stick loss: no climb after the entry (at most 0.3 m above it), 2 m below the entry within 3 s.
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

/// LAND from a 5 m hover is unchanged: the stage rate is 0.5 m/s throughout (from v_D 0 at the entry), no
/// rise.
#[test]
fn fx2_land_from_5_m_descends_at_0_5_m_s() {
    let mut p = pilot_hover_lost(5.0);
    let r = descend(&mut p);
    std::println!("FX-2 (3) 5 m: LAND at {:.2} m; largest stage rate {:.3} m/s; rise {:.3} m", r.h0, r.rate_max, r.rise);
    assert!(r.rate_max <= LAND_SPEED_MS && r.rise <= 0.1);
}

/// The quadratic-drag plant of section 48: no linear drag, a 50 ms motor lag, gain `gain`, and kq such that the
/// terminal fall at the allocator's idle floor is 15 m/s: kq = (g - gain floor / m) / 15^2 (1/m)
fn quadratic_plant(gain: f32) -> Plant {
    let mut p = Plant::new(gain, 0.05, 0.0, 0.0);
    p.kq = (GRAVITY_MSS - gain * p.f.floor_n / MASS) / (15.0 * 15.0);
    p
}

/// A pilot throttle chop in Angle of 5 s from 60 m, then a stick loss (FS_HOLD_EN 0): the
/// accepted residual.
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

/// BLIND entered from a 5 m/s descent.
/// and the height lost together; from 150 m so that 10 s of BLIND fit): the probe aborts, no stop in the air.
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

/// LAND from a hover at 20 m with the v_D estimate frozen at a faulty value from the stick loss on +3
/// m/s (down, the loop adds thrust) and -3 m/s (up, the loop commands the idle floor)
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

/// A gentle take-off in Angle at thrust gain 1.46 never sets the 0.714 latch; the stick
/// loss with FS_HOLD_EN 1 (station heard) and with FS_HOLD_EN 0 lands without a stop.
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

/// The modelled clear agrees with the real one, deterministically.
/// it without requiring it)
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
            let mut i = input(t, rc.map(|(arm, thr)| RcSample { source: fw_core::rc::RcSource::Receiver, ..report(t, arm, thr, Switch3::High) }));
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

/// LAND from its entry to the motors stopped.
/// a 5 s window from then to contact, m; the largest true descent rate after the stop, m/s.
struct LandRun {
    stopped_s: f32,
    min_drop_5s: f32,
    v_max: f32,
    v_min: f32,
    contact_s: f32,
    stop_after_s: f32,
}

fn land_to_stop(p: &mut Plant, max_s: f32, mut each: impl FnMut(&mut Plant, f32)) -> LandRun {
    let dt = DT_US as f32 * 1e-6;
    let (mut k, mut stopped, mut contact) = (0u32, None::<f32>, None::<f32>);
    let (mut v_max, mut v_min, mut min_drop, mut window) = (0.0f32, f32::INFINITY, f32::INFINITY, std::vec::Vec::new());
    loop {
        k += 1;
        let t = k as f32 * dt;
        assert!(t < max_s, "no STOP within {max_s} s: {:.2} m, v_D {:.2}", -p.d, p.v);
        each(p, t);
        let o = p.step(None);
        if !o.armed {
            assert!(p.on_ground(), "a stop in the air at {:.2} m", -p.d);
            break;
        }
        if p.on_ground() {
            contact.get_or_insert(t);
        }
        if t >= 1.0 && contact.is_none() {
            v_min = v_min.min(p.v);
        }
        if stopped.is_none() && p.v <= LAND_SPEED_HIGH_MS + 0.05 {
            stopped = Some(t);
        }
        if stopped.is_some() && contact.is_none() {
            v_max = v_max.max(p.v);
            window.push(p.d);
            let n = (5.0 / dt) as usize;
            if window.len() > n {
                min_drop = min_drop.min(window[window.len() - 1] - window[window.len() - 1 - n]);
            }
        }
    }
    let contact = contact.unwrap_or(f32::NAN);
    LandRun { stopped_s: stopped.unwrap_or(f32::NAN), min_drop_5s: min_drop, v_max, v_min, contact_s: contact, stop_after_s: k as f32 * dt - contact }
}

/// The chop case of N18 (c) on the quadratic-drag plant: 3 s at throttle 0 in Angle from a 60 m hover, the sticks lost
/// at about 31 m falling at 15 m/s (FS_HOLD_EN 0: LAND)
fn chop_land(seat_off: bool) -> LandRun {
    let mut p = quadratic_plant(1.0);
    p.f.shaper.seat_off = seat_off;
    p.hover_at(60.0, 5.0);
    for _ in 0..2_400 {
        p.step(Some((true, 0.0, Switch3::Low)));
    }
    let (h, v) = (-p.d, p.v);
    while p.f.hands_off_state() == State::Pilot {
        p.step(None);
    }
    assert_eq!(p.f.hands_off_state(), State::Land);
    std::println!("N18 chop: sticks lost at {h:.1} m falling at {v:.2} m/s; LAND at {:.1} m, {:.2} m/s", -p.d, p.v);
    land_to_stop(&mut p, 120.0, |_, _| {})
}

/// After the chop LAND stops the fall, descends at no more than 1.5 m/s, never hovers.
/// window until contact), and TD stops the motors on the ground.
#[test]
fn n18_chop_then_stick_loss_land_stops_the_fall_descends_and_td_stops() {
    let r = chop_land(false);
    std::println!(
        "N18 chop: fall stopped {:.2} s after LAND; smallest drop in 5 s {:.2} m; largest descent after {:.2} m/s; contact {:.1} s; STOP {:.2} s after contact",
        r.stopped_s, r.min_drop_5s, r.v_max, r.contact_s, r.stop_after_s
    );
    assert!(r.min_drop_5s >= 1.0 && r.v_max <= LAND_SPEED_HIGH_MS + 0.15, "{} {}", r.min_drop_5s, r.v_max);
}

/// Control: the seat removed, the chop case hovers (no STOP within 120 s)
#[test]
#[should_panic(expected = "no STOP within")]
fn n18_control_without_the_seat_the_chop_case_hovers() {
    chop_land(true);
}

/// LAND entered while descending at 3 m/s and at 6 m/s.
/// that holds that speed against the drag, then the sticks lost): v_D never negative from 1 s after the entry.
#[test]
fn n18_land_entered_descending_at_3_and_6_m_s_never_climbs() {
    for v_entry in [3.0f32, 6.0] {
        let mut p = quadratic_plant(1.0);
        p.hover_at(60.0, 5.0);
        while p.v < v_entry {
            assert!(p.step(Some((true, 0.3, Switch3::Low))).armed && p.d < -20.0);
        }
        let thr = p.f.shaper.throttle_for(MASS * (GRAVITY_MSS - p.kq * v_entry * v_entry));
        for _ in 0..400 {
            p.step(Some((true, thr, Switch3::Low)));
        }
        while p.f.hands_off_state() == State::Pilot {
            p.step(None);
        }
        let (h0, v0) = (-p.d, p.v);
        let r = land_to_stop(&mut p, 200.0, |_, _| {});
        std::println!("N18 LAND entered at {h0:.1} m descending {v0:.2} m/s: smallest v_D from 1 s {:.3} m/s; contact {:.1} s; STOP {:.2} s after contact", r.v_min, r.contact_s, r.stop_after_s);
        assert!(r.v_min >= 0.0, "v_entry {v_entry}: v_D {}", r.v_min);
    }
}

/// A 2 m downward step of the height estimate during LAND at 15 m (from a 20 m hover, FS_HOLD_EN 0): no climb back
/// (true v_D never negative after the step), the descent continues, touchdown, TD STOP.
#[test]
fn n18_a_2_m_downward_step_of_the_estimate_in_land_does_not_climb_back() {
    let mut p = land_plant();
    p.hover_at(20.0, 10.0);
    while p.f.hands_off_state() == State::Pilot {
        p.step(None);
    }
    let mut at = None::<f32>;
    let mut v_after = f32::INFINITY;
    let r = land_to_stop(&mut p, 120.0, |p, t| {
        if at.is_none() && p.d > -15.0 {
            p.d_off = 2.0;
            at = Some(t);
        }
        if at.is_some() && !p.on_ground() {
            v_after = v_after.min(p.v);
        }
    });
    std::println!("N18 estimate step +2 m down at {:?} s: smallest true v_D after {v_after:.3} m/s; contact {:.1} s; STOP {:.2} s after contact", at, r.contact_s, r.stop_after_s);
    assert!(at.is_some() && v_after >= 0.0, "{v_after}");
}

/// RETURN at home climbing to 20 m; from the first step the guidance reports Home the v_D estimate reads -0.8 m/s
/// (above `HOME_VD_MAX_MS`)
fn home_wait_with_v_d_held(wait_off: bool) -> Option<f32> {
    use crate::hands_off::{Command, Link, StationCommand};
    let mut p = land_plant();
    p.station = true;
    p.f.home_wait_off = wait_off;
    p.f.set_fs_hold_en(true);
    p.source = fw_core::rc::RcSource::Receiver;
    p.f.set_battery_ok(true);
    p.f.set_mission(super::auto_tests::square());
    p.hover_at(5.0, 10.0);
    while p.f.hands_off_state() == State::Pilot {
        p.step(None);
    }
    for _ in 0..800 {
        p.step(None);
    }
    p.f.set_command(StationCommand { command: Command::Return, link: Link::Radio });
    while p.f.legs.phase() != LegPhase::Home {
        assert!(p.step(None).armed && p.t < 100_000_000);
        assert_ne!(p.f.hands_off_state(), State::Land);
        if p.f.legs.phase() == LegPhase::HomeLeg {
            p.vd_frozen = Some(-0.8);
        }
    }
    p.vd_frozen = Some(-0.8);
    for k in 1..=16_000u32 {
        p.step(None);
        if p.f.hands_off_state() == State::Land {
            return Some(k as f32 * DT_US as f32 * 1e-6);
        }
        assert_eq!(p.f.hands_off_state(), State::Return);
    }
    None
}

/// With v_D held at -0.8 m/s after arrival, LAND comes `HOME_WAIT_US` (3.0 s) after Home, within one step; control:
/// the bound removed, no LAND in 20 s.
#[test]
fn fx4_home_wait_is_bounded_at_3_s() {
    let t = home_wait_with_v_d_held(false);
    let off = home_wait_with_v_d_held(true);
    std::println!("FX-4 (1) v_D held at -0.8 m/s: LAND {t:?} s after Home (bound {} s); control without the bound {off:?}", HOME_WAIT_US as f32 * 1e-6);
    let want = HOME_WAIT_US as f32 * 1e-6;
    assert!(t.is_some_and(|t| t >= want && t <= want + 2.0 * DT_US as f32 * 1e-6), "{t:?}");
    assert_eq!(off, None);
}

/// LAND in the 0.5 m/s stage at about 4.3 m; the height estimate reads 1.0 m lower for one step.
#[test]
fn fx4_a_one_step_1_m_downward_spike_of_the_estimate_in_land() {
    let mut p = pilot_hover_lost(5.0);
    for _ in 0..2_400 {
        p.step(None);
    }
    let (h0, dt) = (-p.d, DT_US as f32 * 1e-6);
    p.d_off = 1.0;
    p.step(None);
    p.d_off = 0.0;
    let (mut v_max, mut last_off, mut floor) = (0.0f32, 0.0f32, false);
    let n = 4_000u32;
    for k in 1..=n {
        let o = p.step(None);
        assert!(o.armed && !p.on_ground());
        v_max = v_max.max(p.v);
        floor |= o.allocation.is_none_or(|a| a.saturation.thrust_low);
        if libm::fabsf(p.v - LAND_SPEED_MS) > 0.05 {
            last_off = k as f32 * dt;
        }
    }
    let extra = (h0 + p.d) - LAND_SPEED_MS * (n + 1) as f32 * dt;
    std::println!("FX-4 (2) spike at {h0:.2} m: largest true descent {v_max:.3} m/s; extra descent {extra:.3} m; rate within 0.05 of 0.5 from {last_off:.2} s; idle floor {floor}");
    let limit = Tuning::DEFAULT.pilot_speed_dn_ms;
    assert!(v_max <= limit + 0.05 && (0.7..=1.3).contains(&extra) && last_off <= 3.0 && !floor, "{v_max} {extra} {last_off} {floor}");
}

/// Estimate noise uniform +-0.2 m at every step for 10 s in LAND's 0.5 m/s stage (from about 10 m): the mean and the
/// largest true descent rate reported; the largest at or below the down-speed limit.
#[test]
fn fx4_estimate_noise_0_2_m_in_lands_slow_stage() {
    let mut p = pilot_hover_lost(10.0);
    for _ in 0..1_600 {
        p.step(None);
    }
    assert!(p.f.land_rate_ms == Some(LAND_SPEED_MS) && p.d < -8.0, "{:?} {}", p.f.land_rate_ms, p.d);
    let mut x = 0x5eed_f404u64;
    let (h0, mut v_max, mut floor) = (-p.d, 0.0f32, 0u32);
    for _ in 0..8_000 {
        x = x.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        p.d_off = ((x >> 40) as f32 / (1u64 << 24) as f32 - 0.5) * 0.4;
        let o = p.step(None);
        assert!(o.armed && !p.on_ground());
        v_max = v_max.max(p.v);
        floor += u32::from(o.allocation.is_none_or(|a| a.saturation.thrust_low));
    }
    let mean = (h0 + p.d) / 10.0;
    std::println!("FX-4 (2) noise +-0.2 m for 10 s from {h0:.2} m: mean true descent {mean:.3} m/s, largest {v_max:.3} m/s, steps on the idle floor {floor}");
    assert!(v_max <= Tuning::DEFAULT.pilot_speed_dn_ms, "{v_max}");
}
