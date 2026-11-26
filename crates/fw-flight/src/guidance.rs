//! Guidance: AUTO's and RETURN's leg law over the mission's waypoints. A pure module.

use crate::eskf::ned_from_geo;
use crate::gnc::GeoPoint;
use crate::tuning::Tuning;
use fw_core::mission::{GDN_TARGETS, GdnTargets};
use libm::{hypotf, sqrtf};

/// A leg shorter than this, m, has no direction: the carrot stays at its start (a divisor guard, not a tuning value).
const LEG_MIN_M: f32 = 1e-3;

/// The vertical carrot's climb rate toward D_k, m/s (also the climb in place of RETURN).
pub const LEG_CLIMB_MS: f32 = 3.0;
/// The vertical carrot's descent rate toward D_k, m/s.
pub const LEG_DESCENT_MS: f32 = 1.5;
/// The vertical carrot's rate changes by at most this, m/s^2 (ArduPilot AC_WPNav
/// WP_ACC_Z_DEFAULT 1.0; the same value as `LAND_RATE_ACC_MSS`).
pub const LEG_ACC_Z_MSS: f32 = 1.0;
/// The horizontal distance vehicle to carrot at which the carrot stops advancing while the vehicle is behind it along the
/// leg, m (PX4 MPC_XY_ERR_MAX).
pub const LAG_E_MAX_M: f32 = 2.0;
/// The arrival radius is at most the track-fault bound X
/// less this, m: the lag limit `LAG_E_MAX_M` plus 1.0 m, since after an arrival the carrot sits at the reached waypoint.
pub const ARRIVAL_MARGIN_M: f32 = LAG_E_MAX_M + 1.0;
/// The AUTO and RETURN cruise speed is min(speed in force, this), m/s.
pub const LEG_SPEED_MAX_MS: f32 = 6.8;
/// The carrot closer than this to the leg's end, m, is at the end and at rest (f32 rounding, not a tuning value: 2 km
/// out, a 1 ms braking step this close to the end is below half an ulp of `s`).
const LEG_SNAP_M: f32 = 1e-3;

/// Home: NE from the origin (m) and D_home (m down).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Home {
    /// North and east from the origin, m.
    pub ne: [f32; 2],
    /// D_home, m down.
    pub d: f32,
}

/// What the law is doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegPhase {
    /// Not entered (or the entry was refused): the outputs are not a setpoint.
    Idle,
    /// AUTO: on the leg to waypoint `active`.
    Leg,
    /// AUTO: the last waypoint was reached; the carrot is frozen at it and its height.
    Done,
    /// RETURN: climbing in place to H_r.
    Climb,
    /// RETURN: on the straight leg to home at H_r.
    HomeLeg,
    /// RETURN: arrived within the radius of home; the carrot is frozen at home and H_r.
    Home,
}

/// This step's event. `Done` and `Home` are reported on every step from the arrival on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LegEvent {
    /// Nothing happened.
    None,
    /// Waypoint k (not the last) was reached; the leg to k + 1 started at it.
    Arrived(u8),
    /// The last waypoint was reached (HOLD at its height).
    Done,
    /// Within the radius of home (LAND).
    Home,
}

/// One step's result: `Setpoint::Position` with velocity feed-forward is built from it by the wiring.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LegOut {
    /// The horizontal carrot, NE from the origin, m.
    pub position_ne: [f32; 2],
    /// The horizontal carrot's velocity (the feed-forward), m/s.
    pub velocity_ne: [f32; 2],
    /// The vertical carrot, m down.
    pub down_m: f32,
    /// The vertical carrot's rate, m/s down.
    pub velocity_d_ms: f32,
    /// The vertical carrot's acceleration (the feed-forward), m/s^2 down: its rate's change over the step before the
    /// snap to D_k, within +-`LEG_ACC_Z_MSS`; 0 while held, outside the lead cap, at Done and Home and for dt 0.
    pub accel_d_mss: f32,
    /// e: horizontal distance vehicle to the carrot after this step, m (the track-fault input).
    pub e_m: f32,
    /// The active waypoint index.
    pub active: u8,
    /// The phase after this step.
    pub phase: LegPhase,
    /// This step's event.
    pub event: LegEvent,
    /// The position input was not usable (invalid or not finite): the carrot was held, its velocities are 0 and `e_m`
    /// is the last usable one.
    pub held: bool,
}

/// A straight leg (a leg shorter than `LEG_MIN_M` has length 0 and no direction).
#[derive(Clone, Copy, Debug)]
struct Line {
    start: [f32; 2],
    end: [f32; 2],
    len: f32,
    dir: [f32; 2],
}

impl Line {
    fn new(start: [f32; 2], end: [f32; 2]) -> Line {
        let d = [end[0] - start[0], end[1] - start[1]];
        let len = hypotf(d[0], d[1]);
        if len > LEG_MIN_M { Line { start, end, len, dir: [d[0] / len, d[1] / len] } } else { Line { start, end, len: 0.0, dir: [0.0; 2] } }
    }

    /// The point `p`'s projection on the leg, within [0, len].
    fn project(&self, p: [f32; 2]) -> f32 {
        let s = (p[0] - self.start[0]) * self.dir[0] + (p[1] - self.start[1]) * self.dir[1];
        s.max(0.0).min(self.len)
    }
}

/// The AUTO legs and the RETURN leg over up to 16 waypoints (`GDN_TARGETS`).
#[derive(Clone, Copy, Debug)]
pub struct LegGuidance {
    /// Waypoints: N, E from the origin and D_k = D_home - z_k, m.
    wp: [[f32; 3]; GDN_TARGETS],
    count: usize,
    radius_m: f32,
    cruise_ms: f32,
    idx: usize,
    phase: LegPhase,
    line: Line,
    home_ne: [f32; 2],
    /// The horizontal carrot's distance along `line`, m, and its speed (before the lag scale), m/s.
    s: f32,
    v: f32,
    /// The vertical carrot and its goal, m down, and its rate, m/s down.
    d: f32,
    d_goal: f32,
    vd: f32,
    e_m: f32,
    /// Test control: the lag scale from the unsigned distance also when the vehicle is ahead (the carrot stall).
    #[cfg(test)]
    lag_unsigned: bool,
}

impl Default for LegGuidance {
    fn default() -> Self {
        Self::new()
    }
}

impl LegGuidance {
    /// Idle.
    pub const fn new() -> Self {
        LegGuidance {
            wp: [[0.0; 3]; GDN_TARGETS],
            count: 0,
            radius_m: 0.0,
            cruise_ms: 0.0,
            idx: 0,
            phase: LegPhase::Idle,
            line: Line { start: [0.0; 2], end: [0.0; 2], len: 0.0, dir: [0.0; 2] },
            home_ne: [0.0; 2],
            s: 0.0,
            v: 0.0,
            d: 0.0,
            d_goal: 0.0,
            vd: 0.0,
            e_m: 0.0,
            #[cfg(test)]
            lag_unsigned: false,
        }
    }

    /// AUTO entry or re-entry: waypoints of `mission` at `origin`, first leg from the vehicle to waypoint `start`, both carrots at the vehicle and at rest. Refused (false, `Idle`) when the position or home is not finite or `start` is not a waypoint.
    pub fn enter(&mut self, position_m: [f32; 3], home: Home, origin: &GeoPoint, mission: &GdnTargets, start: u8, speed_ms: f32) -> bool {
        let [pn, pe, pd] = position_m;
        let count = usize::from(mission.count).min(GDN_TARGETS);
        let start = usize::from(start);
        let finite = [pn, pe, pd, home.ne[0], home.ne[1], home.d].iter().all(|x| x.is_finite());
        if !finite || start >= count {
            self.phase = LegPhase::Idle;
            return false;
        }
        let mut d_prev = pd;
        for (dst, (&(lat_e7, lon_e7), &z)) in self.wp.iter_mut().zip(mission.targets.iter().zip(mission.alts_m.iter())).take(count) {
            let [n, e, _] = ned_from_geo(origin, &GeoPoint { lat_e7, lon_e7, alt_mm: origin.alt_mm });
            // Heights are checked at upload and Start; a non-finite one keeps the previous height.
            let d = home.d - z;
            let d = if d.is_finite() { d } else { d_prev };
            d_prev = d;
            *dst = [n, e, d];
        }
        self.count = count;
        self.radius_m = Self::radius(mission.radius_m, speed_ms);
        self.cruise_ms = Self::cruise(speed_ms);
        self.home_ne = home.ne;
        self.idx = start;
        let [n, e, d] = self.waypoint(start);
        self.line = Line::new([pn, pe], [n, e]);
        self.begin(pd, d);
        self.phase = LegPhase::Leg;
        true
    }

    /// RETURN entry: H_r = max(`return_m` above home, the present height), latched here (a non-finite `return_m` counts
    /// as the present height); climb in place, then the leg to home.
    pub fn enter_return(&mut self, position_m: [f32; 3], home: Home, return_m: f32, radius_m: f32, speed_ms: f32) -> bool {
        let [pn, pe, pd] = position_m;
        let finite = [pn, pe, pd, home.ne[0], home.ne[1], home.d].iter().all(|x| x.is_finite());
        if !finite {
            self.phase = LegPhase::Idle;
            return false;
        }
        let d_r = home.d - return_m;
        let d_r = if d_r.is_finite() { d_r.min(pd) } else { pd };
        self.radius_m = Self::radius(radius_m, speed_ms);
        self.cruise_ms = Self::cruise(speed_ms);
        self.home_ne = home.ne;
        self.line = Line::new([pn, pe], [pn, pe]);
        self.begin(pd, d_r);
        self.phase = LegPhase::Climb;
        true
    }

    /// Back to `Idle`.
    pub fn stop(&mut self) {
        self.phase = LegPhase::Idle;
    }

    /// What the law is doing.
    pub fn phase(&self) -> LegPhase {
        self.phase
    }

    /// The active waypoint index.
    pub fn active(&self) -> u8 {
        u8::try_from(self.idx).unwrap_or(u8::MAX)
    }

    /// The vertical carrot, m down.
    pub fn down_m(&self) -> f32 {
        self.d
    }

    /// The lead cap of the vertical carrot: the vehicle at `d_m` (m down) is within
    /// `lead_s` seconds of the carrot's rate in the direction it moves. False for a non-finite input.
    pub fn lead_ok(&self, d_m: f32, lead_s: f32) -> bool {
        let rate = if self.d_goal > self.d { LEG_DESCENT_MS } else { LEG_CLIMB_MS };
        libm::fabsf(d_m - self.d) < rate * lead_s
    }

    /// One step of `dt_s` seconds. `position_m` is the vehicle's NED position, `valid` its NE flag; `lead_ok` lets the
    /// vertical carrot move (the lead cap, from the wiring).
    #[allow(clippy::manual_clamp)] // not `clamp`: flight-core code, no panic path
    pub fn step(&mut self, dt_s: f32, position_m: [f32; 3], valid: bool, lead_ok: bool) -> LegOut {
        let dt = if dt_s.is_finite() && dt_s > 0.0 { dt_s } else { 0.0 };
        let [pn, pe, pd] = position_m;
        if !valid || ![pn, pe, pd].iter().all(|x| x.is_finite()) {
            return self.out([0.0; 2], 0.0, 0.0, true, self.level_event());
        }
        let veh = [pn, pe];
        let mut event = LegEvent::None;
        match self.phase {
            LegPhase::Leg => {
                if Self::dist(veh, self.line.end) <= self.radius_m && self.d == self.d_goal {
                    if self.idx + 1 >= self.count {
                        self.phase = LegPhase::Done;
                        self.s = self.line.len;
                        self.v = 0.0;
                    } else {
                        // The new leg starts at the reached target; the carrot at the vehicle's projection on it.
                        event = LegEvent::Arrived(self.active());
                        self.idx += 1;
                        let [n, e, d] = self.waypoint(self.idx);
                        self.line = Line::new(self.line.end, [n, e]);
                        self.s = self.line.project(veh);
                        self.v = 0.0;
                        self.d_goal = d;
                    }
                }
            }
            LegPhase::Climb => {
                if self.d == self.d_goal {
                    self.phase = LegPhase::HomeLeg;
                    self.line = Line::new(self.line.end, self.home_ne);
                    self.s = 0.0;
                    self.v = 0.0;
                }
            }
            LegPhase::HomeLeg => {
                if Self::dist(veh, self.line.end) <= self.radius_m {
                    self.phase = LegPhase::Home;
                    self.s = self.line.len;
                    self.v = 0.0;
                }
            }
            LegPhase::Idle | LegPhase::Done | LegPhase::Home => {}
        }
        if !matches!(self.phase, LegPhase::Leg | LegPhase::Climb | LegPhase::HomeLeg) {
            return self.out([0.0; 2], 0.0, 0.0, false, self.level_event());
        }
        // The advance is scaled by k (the profile runs on k dt); 1 while the vehicle is ahead of the carrot along the leg.
        let c = self.carrot_ne();
        let ahead = (pn - c[0]) * self.line.dir[0] + (pe - c[1]) * self.line.dir[1] > 0.0;
        #[cfg(test)]
        let ahead = ahead && !self.lag_unsigned;
        let k = if ahead { 1.0 } else { (1.0 - Self::dist(veh, c) / LAG_E_MAX_M).max(0.0).min(1.0) };
        let dte = k * dt;
        if self.s < self.line.len {
            let (a, r) = (Tuning::GUIDANCE_ACC_MS2, self.line.len - self.s);
            let brake = 0.5 * (sqrtf((a * a * dte * dte + 8.0 * a * r - 4.0 * a * dte * self.v).max(0.0)) - a * dte);
            let target_v = self.cruise_ms.min(brake.max(0.0));
            let max_dv = a * dte;
            let v_new = self.v + (target_v - self.v).max(-max_dv).min(max_dv);
            let s_new = self.s + 0.5 * (self.v + v_new) * dte;
            (self.s, self.v) = if s_new >= self.line.len - LEG_SNAP_M { (self.line.len, 0.0) } else { (s_new, v_new) };
        } else {
            self.v = 0.0;
        }
        let vel = [self.line.dir[0] * self.v * k, self.line.dir[1] * self.v * k];
        let mut accel_d = 0.0;
        if lead_ok {
            let gap = self.d_goal - self.d;
            let sign = libm::copysignf(1.0, gap);
            let (a, r, v) = (LEG_ACC_Z_MSS, libm::fabsf(gap), self.vd * sign);
            let cap = if gap > 0.0 { LEG_DESCENT_MS } else { LEG_CLIMB_MS };
            let brake = 0.5 * (sqrtf((a * a * dt * dt + 8.0 * a * r - 4.0 * a * dt * v).max(0.0)) - a * dt);
            let target_v = cap.min(brake.max(0.0));
            let max_dv = a * dt;
            let v_new = v + (target_v - v).max(-max_dv).min(max_dv);
            // The rate's change before the snap, so that the snap step does not spike.
            if dt > 0.0 {
                accel_d = (sign * (v_new - v) / dt).max(-LEG_ACC_Z_MSS).min(LEG_ACC_Z_MSS);
            }
            let r_new = r - 0.5 * (v + v_new) * dt;
            (self.d, self.vd) = if r_new <= LEG_SNAP_M { (self.d_goal, 0.0) } else { (self.d_goal - sign * r_new, sign * v_new) };
        } else {
            self.vd = 0.0;
        }
        self.e_m = Self::dist(veh, self.carrot_ne());
        self.out(vel, self.vd, accel_d, false, event)
    }

    /// Both carrots at the vehicle (`s` 0 on `line`, which starts there) and at rest; the vertical goal `d_goal`.
    fn begin(&mut self, d: f32, d_goal: f32) {
        self.s = 0.0;
        self.v = 0.0;
        self.d = d;
        self.d_goal = d_goal;
        self.vd = 0.0;
        self.e_m = 0.0;
    }

    fn waypoint(&self, k: usize) -> [f32; 3] {
        self.wp.get(k).copied().unwrap_or([self.line.end[0], self.line.end[1], self.d_goal])
    }

    fn carrot_ne(&self) -> [f32; 2] {
        let l = &self.line;
        if self.s >= l.len { l.end } else { [l.start[0] + l.dir[0] * self.s, l.start[1] + l.dir[1] * self.s] }
    }

    fn level_event(&self) -> LegEvent {
        match self.phase {
            LegPhase::Done => LegEvent::Done,
            LegPhase::Home => LegEvent::Home,
            _ => LegEvent::None,
        }
    }

    fn out(&self, velocity_ne: [f32; 2], velocity_d_ms: f32, accel_d_mss: f32, held: bool, event: LegEvent) -> LegOut {
        LegOut {
            position_ne: self.carrot_ne(),
            velocity_ne,
            down_m: self.d,
            velocity_d_ms,
            accel_d_mss,
            e_m: self.e_m,
            active: self.active(),
            phase: self.phase,
            event,
            held,
        }
    }

    /// The arrival radius: min(GDN_RADIUS, X - `ARRIVAL_MARGIN_M`), X the track-fault bound of `speed_ms`.
    fn radius(radius_m: f32, speed_ms: f32) -> f32 {
        let cap = crate::hands_off::track_fault_m(speed_ms) - ARRIVAL_MARGIN_M;
        if radius_m.is_finite() { radius_m.min(cap).max(0.0) } else { 0.0 }
    }

    #[allow(clippy::manual_clamp)] // not `clamp`: flight-core code, no panic path
    fn cruise(speed_ms: f32) -> f32 {
        if speed_ms.is_finite() { speed_ms.max(0.0).min(LEG_SPEED_MAX_MS) } else { 0.0 }
    }

    fn dist(a: [f32; 2], b: [f32; 2]) -> f32 {
        hypotf(a[0] - b[0], a[1] - b[1])
    }
}

#[cfg(test)]
mod leg_tests;
