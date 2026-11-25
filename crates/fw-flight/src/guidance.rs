//! Guidance: geodetic targets to `Setpoint::Position`. A pure module. Per leg (from the
//! previous target, or from the vehicle's position at the first valid call, to the active target) a carrot of state `s`
//! (along the leg) and `v` (its speed) moves toward the target at the cruise speed, braking so that it stops there. The
//! controller tracks the carrot's position and velocity; the vertical axis is the vehicle's own (horizontal only).

use crate::eskf::ned_from_geo;
use crate::gnc::{GeoPoint, NavState, Setpoint};
use crate::tuning::Tuning;
use fw_core::mission::{GDN_TARGETS, GdnTargets};
use libm::{atan2f, hypotf, sqrtf};

/// `GuidanceOut::status` bits.
pub mod status {
    /// A setpoint is being made.
    pub const RUNNING: u8 = 1;
    /// This call's vehicle position was within the radius of the active target (the index advanced, or the last target
    /// was reached).
    pub const ARRIVED: u8 = 2;
    /// The last target was reached; the output holds there.
    pub const DONE: u8 = 4;
    /// The input was not usable (NE position flag false, no origin, a non-finite value): no setpoint.
    pub const INVALID: u8 = 8;
}

/// A leg shorter than this, m, has no direction: the carrot stays at its start (a divisor guard, not a tuning value).
const LEG_MIN_M: f32 = 1e-3;

/// One call's result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GuidanceOut {
    /// What the controller would track; `None` when the input is not usable or no target is set.
    pub setpoint: Option<Setpoint>,
    /// Index of the active target (the last one once done).
    pub active: u8,
    /// `status` bits.
    pub status: u8,
    /// The active target, north and east, m, from the origin.
    pub target_ne: [f32; 2],
    /// Horizontal distance from the vehicle to the active target, m.
    pub dist_m: f32,
    /// Bearing from the vehicle to the active target, rad, clockwise from north.
    pub bearing_rad: f32,
}

/// The leg the carrot is on.
struct Leg {
    start: [f32; 2],
    end: [f32; 2],
    len: f32,
    dir: [f32; 2],
}

impl Leg {
    fn new(start: [f32; 2], end: [f32; 2]) -> Leg {
        let d = [end[0] - start[0], end[1] - start[1]];
        let len = hypotf(d[0], d[1]);
        let dir = if len > LEG_MIN_M { [d[0] / len, d[1] / len] } else { [0.0; 2] };
        Leg { start, end, len, dir }
    }

    /// The vehicle's projection on the leg clamped to [0, L], and its speed along the leg clamped to [0, cruise].
    fn project(&self, p: [f32; 2], v: [f32; 2], cruise: f32) -> (f32, f32) {
        let s = (p[0] - self.start[0]) * self.dir[0] + (p[1] - self.start[1]) * self.dir[1];
        let along = v[0] * self.dir[0] + v[1] * self.dir[1];
        (s.max(0.0).min(self.len), along.max(0.0).min(cruise))
    }
}

/// Guidance over up to `N` targets.
#[derive(Clone, Copy, Debug)]
pub struct Guidance<const N: usize> {
    targets: [GeoPoint; N],
    count: usize,
    radius_m: f32,
    idx: usize,
    done: bool,
    /// The vehicle's NE at the first valid call, the start of the first leg.
    start_ne: Option<[f32; 2]>,
    /// The carrot's distance along the leg, m, and its speed, m/s.
    s: f32,
    v: f32,
    last_t_us: Option<u64>,
    /// The vehicle's NE position and velocity at `anchor`, taken by the next `step` (the leg needs the origin).
    pending: Option<([f32; 2], [f32; 2])>,
}

impl<const N: usize> Default for Guidance<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Guidance<N> {
    /// No targets.
    pub const fn new() -> Self {
        Guidance {
            targets: [GeoPoint { lat_e7: 0, lon_e7: 0, alt_mm: 0 }; N],
            count: 0,
            radius_m: 0.0,
            idx: 0,
            done: false,
            start_ne: None,
            s: 0.0,
            v: 0.0,
            last_t_us: None,
            pending: None,
        }
    }

    /// The targets (the first `N` of `targets`) and the arrival radius, m. Starts over from the first target.
    pub fn set_targets(&mut self, targets: &[GeoPoint], radius_m: f32) {
        self.count = targets.len().min(N);
        self.targets[..self.count].copy_from_slice(&targets[..self.count]);
        self.radius_m = if radius_m.is_finite() { radius_m.max(0.0) } else { 0.0 };
        self.reset();
    }

    /// Back to the first target, with no leg started. The targets stay.
    pub fn reset(&mut self) {
        self.idx = 0;
        self.done = false;
        self.drop_run();
        self.pending = None;
    }

    /// The vehicle's NE position and velocity, for the next `step` to place the carrot on the leg: `s` at the vehicle's
    /// projection on the leg clamped to [0, L], `v` at its speed along the leg clamped to [0, cruise]. Unusable input.
    pub fn anchor(&mut self, nav: &NavState) {
        let [pn, pe, _] = nav.position_m;
        let [vn, ve, _] = nav.velocity_ms;
        let ok = nav.valid.position_ne && pn.is_finite() && pe.is_finite() && vn.is_finite() && ve.is_finite();
        self.pending = ok.then_some(([pn, pe], if nav.valid.velocity_ne { [vn, ve] } else { [0.0; 2] }));
    }

    /// One step. `speed_ms` is the cruise speed; `t_us` the step time.
    pub fn step(&mut self, nav: &NavState, origin: Option<GeoPoint>, speed_ms: f32, t_us: u64) -> GuidanceOut {
        let pending = self.pending.take();
        let [pn, pe, pd] = nav.position_m;
        let [vn, ve, _] = nav.velocity_ms;
        // The heading as the stick shaper has it (step.rs, `position`).
        let [w, _, _, z] = nav.attitude;
        let heading = 2.0 * libm::atan2f(z, w);
        let finite = [pn, pe, pd, vn, ve, heading, speed_ms].iter().all(|x| x.is_finite());
        let origin = match origin {
            Some(o) if nav.valid.position_ne && finite && self.count > 0 => o,
            _ => {
                self.drop_run();
                let status = if self.count > 0 { status::INVALID } else { 0 };
                return GuidanceOut {
                    setpoint: None,
                    active: self.idx as u8,
                    status,
                    target_ne: [0.0; 2],
                    dist_m: 0.0,
                    bearing_rad: 0.0,
                };
            }
        };
        let cruise = speed_ms.max(0.0);
        let veh = [pn, pe];
        let vel = if nav.valid.velocity_ne { [vn, ve] } else { [0.0; 2] };
        let first = *self.start_ne.get_or_insert(veh);
        let dt_s = match self.last_t_us.replace(t_us) {
            Some(t0) => t_us.saturating_sub(t0) as f32 * 1e-6,
            None => 0.0,
        };
        let mut status = 0;

        if !self.done {
            let mut leg = self.leg(&origin, first);
            if let Some((p, v)) = pending {
                (self.s, self.v) = leg.project(p, v, cruise);
            }
            if hypotf(leg.end[0] - pn, leg.end[1] - pe) <= self.radius_m {
                status |= status::ARRIVED;
                if self.idx + 1 >= self.count {
                    self.done = true;
                } else {
                    // The next leg starts at this target; the carrot is placed on it from the vehicle, as `anchor` does.
                    self.idx += 1;
                    leg = self.leg(&origin, first);
                    (self.s, self.v) = leg.project(veh, vel, cruise);
                }
            }
            if !self.done {
                self.s = self.s.min(leg.len);
                let rem = (leg.len - self.s - self.v * dt_s).max(0.0);
                let target_v = cruise.min(sqrtf(2.0 * Tuning::GUIDANCE_ACC_MS2 * rem));
                let max_dv = Tuning::GUIDANCE_ACC_MS2 * dt_s;
                let v_new = self.v + (target_v - self.v).max(-max_dv).min(max_dv);
                self.s = (self.s + 0.5 * (self.v + v_new) * dt_s).min(leg.len);
                self.v = v_new;
                let (dist_m, bearing_rad) = Self::range(leg.end, veh);
                return GuidanceOut {
                    setpoint: Some(Setpoint::Position {
                        position_m: [leg.start[0] + leg.dir[0] * self.s, leg.start[1] + leg.dir[1] * self.s, pd],
                        velocity_ms: [leg.dir[0] * self.v, leg.dir[1] * self.v, 0.0],
                        heading_rad: heading,
                        heading_rate_rads: 0.0,
                        accel_d_mss: 0.0,
                    }),
                    active: self.idx as u8,
                    status: status | status::RUNNING,
                    target_ne: leg.end,
                    dist_m,
                    bearing_rad,
                };
            }
        }
        // Done: hold at the last target, at rest.
        self.s = 0.0;
        self.v = 0.0;
        let end = self.ne_of(&origin, self.idx);
        let (dist_m, bearing_rad) = Self::range(end, veh);
        GuidanceOut {
            setpoint: Some(Setpoint::Position {
                position_m: [end[0], end[1], pd],
                velocity_ms: [0.0; 3],
                heading_rad: heading,
                heading_rate_rads: 0.0,
                accel_d_mss: 0.0,
            }),
            active: self.idx as u8,
            status: status | status::DONE,
            target_ne: end,
            dist_m,
            bearing_rad,
        }
    }

    /// The first leg's start is forgotten and the carrot is at rest: the next valid call starts afresh. The index stays.
    fn drop_run(&mut self) {
        self.start_ne = None;
        self.s = 0.0;
        self.v = 0.0;
        self.last_t_us = None;
    }

    /// Target `k` in NE from `origin`, at the origin's altitude, converted on every call.
    fn ne_of(&self, origin: &GeoPoint, k: usize) -> [f32; 2] {
        let at = GeoPoint { alt_mm: origin.alt_mm, ..self.targets[k] };
        let [n, e, _] = ned_from_geo(origin, &at);
        [n, e]
    }

    /// The leg to the active target: from the previous target, or from `first` for the first one.
    fn leg(&self, origin: &GeoPoint, first: [f32; 2]) -> Leg {
        let start = if self.idx == 0 { first } else { self.ne_of(origin, self.idx - 1) };
        Leg::new(start, self.ne_of(origin, self.idx))
    }

    /// Distance and bearing (clockwise from north) from `veh` to `end`.
    fn range(end: [f32; 2], veh: [f32; 2]) -> (f32, f32) {
        let (dn, de) = (end[0] - veh[0], end[1] - veh[1]);
        (hypotf(dn, de), atan2f(de, dn))
    }
}


/// The vertical carrot's climb rate toward D_k, m/s (also the climb in place of RETURN).
pub const LEG_CLIMB_MS: f32 = 3.0;
/// The vertical carrot's descent rate toward D_k, m/s.
pub const LEG_DESCENT_MS: f32 = 1.5;
/// The vertical carrot's rate changes by at most this, m/s^2 (ArduPilot AC_WPNav
/// WP_ACC_Z_DEFAULT 1.0; the same value as `LAND_RATE_ACC_MSS`).
pub const LEG_ACC_Z_MSS: f32 = 1.0;
/// The horizontal distance vehicle to carrot at which the carrot stops advancing, m (PX4 MPC_XY_ERR_MAX).
pub const LAG_E_MAX_M: f32 = 2.0;
/// The arrival radius is at most the track-fault bound X
/// less this, m: the lag limit `LAG_E_MAX_M` plus 1.0 m, since after an arrival the carrot sits at the reached waypoint.
pub const ARRIVAL_MARGIN_M: f32 = LAG_E_MAX_M + 1.0;
/// The AUTO and RETURN cruise speed is min(speed in force, this), m/s.
pub const LEG_SPEED_MAX_MS: f32 = 6.8;
/// The carrot closer than this to the leg's end, m, is at the end and at rest (f32 rounding, not a tuning value: 2 km out,
/// a 1 ms braking step this close to the end is below half an ulp of `s`). The stop from there is at most.
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
        }
    }

    /// AUTO entry or re-entry: the waypoints of `mission` converted at `origin`, the
    /// first leg from the vehicle to waypoint `start`, both carrots at the vehicle and at rest. The radius is the.
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

    /// RETURN entry: H_r = max(`return_m` above home, the present height), latched here (a non-finite `return_m`
    /// counts as the present height); climb in place, then the leg to home. Refused (false, `Idle`) when the position.
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
        // The advance is scaled by k (the profile runs on k dt).
        let k = (1.0 - Self::dist(veh, self.carrot_ne()) / LAG_E_MAX_M).max(0.0).min(1.0);
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

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use crate::gnc::Valid;
    use std::vec::Vec;

    const A: f32 = Tuning::GUIDANCE_ACC_MS2;
    const DT_US: u64 = 10_000;
    const DT: f32 = 0.01;
    /// Invented, in an ocean: lat 2.5 deg, lon -140 deg, 40 m.
    const ORIGIN: GeoPoint = GeoPoint { lat_e7: 25_000_000, lon_e7: -1_400_000_000, alt_mm: 40_000 };
    const OK: Valid = Valid { position_ne: true, velocity_ne: true, heading: true, ..Valid::NONE };

    fn nav(p: [f32; 3], v: [f32; 2], yaw: f32, valid: Valid) -> NavState {
        NavState {
            t_us: 0,
            attitude: [libm::cosf(yaw / 2.0), 0.0, 0.0, libm::sinf(yaw / 2.0)],
            rates_rads: [0.0; 3],
            velocity_ms: [v[0], v[1], 0.0],
            position_m: p,
            valid,
            sigma: None,
        }
    }

    /// The point `n_m` north and `e_m` east of `origin`, by an f64 inverse written for this test (not `ned_from_geo`).
    fn geo_at(origin: &GeoPoint, n_m: f64, e_m: f64, alt_mm: i32) -> GeoPoint {
        let f = f64::from(crate::eskf::WGS84_F);
        let a = f64::from(crate::eskf::WGS84_A_M);
        let e2 = f * (2.0 - f);
        let lat0 = (f64::from(origin.lat_e7) * 1e-7).to_radians();
        let w = 1.0 - e2 * lat0.sin().powi(2);
        let (rm, rn) = (a * (1.0 - e2) / (w * w.sqrt()), a / w.sqrt());
        GeoPoint {
            lat_e7: origin.lat_e7 + ((n_m / rm).to_degrees() * 1e7).round() as i32,
            lon_e7: origin.lon_e7 + ((e_m / (rn * lat0.cos())).to_degrees() * 1e7).round() as i32,
            alt_mm,
        }
    }

    fn at_rest(n: f32, e: f32) -> NavState {
        nav([n, e, -1.5], [0.0; 2], 0.3, OK)
    }

    fn position_of(out: &GuidanceOut) -> ([f32; 3], [f32; 3], f32) {
        match out.setpoint {
            Some(Setpoint::Position { position_m, velocity_ms, heading_rad, heading_rate_rads, .. }) => {
                assert_eq!(heading_rate_rads, 0.0);
                (position_m, velocity_ms, heading_rad)
            }
            other => panic!("not Position: {other:?}"),
        }
    }

    fn finite(o: &GuidanceOut) {
        assert!(o.target_ne.iter().chain([&o.dist_m, &o.bearing_rad]).all(|x| x.is_finite()), "{o:?}");
        if let Some(Setpoint::Position { position_m, velocity_ms, heading_rad, heading_rate_rads, .. }) = o.setpoint {
            assert!(position_m.iter().chain(&velocity_ms).chain([&heading_rad, &heading_rate_rads]).all(|x| x.is_finite()), "{o:?}");
        }
    }

    /// A vehicle that follows the setpoint exactly (a point mass with no lag): its position is the last setpoint's, its.
    fn follow(g: &mut Guidance<4>, start: NavState, speed: f32, max_steps: usize, stop_when_done: bool) -> Vec<GuidanceOut> {
        let mut outs = Vec::new();
        g.anchor(&start);
        let mut cur = start;
        for k in 0..max_steps {
            let o = g.step(&cur, Some(ORIGIN), speed, k as u64 * DT_US);
            finite(&o);
            let (p, v, _) = position_of(&o);
            cur.position_m = [p[0], p[1], cur.position_m[2]];
            cur.velocity_ms = v;
            outs.push(o);
            if stop_when_done && o.status & status::DONE != 0 {
                break;
            }
        }
        outs
    }

    /// The target is converted with `ned_from_geo` at the origin's altitude, on every call.
    #[test]
    fn h1_the_target_is_converted_at_the_origin() {
        let mut g = Guidance::<4>::new();
        let t = geo_at(&ORIGIN, 300.0, 200.0, ORIGIN.alt_mm + 500_000);
        g.set_targets(&[t], 2.0);
        let o = g.step(&at_rest(0.0, 0.0), Some(ORIGIN), 5.0, 0);
        assert!((o.target_ne[0] - 300.0).abs() < 0.02 && (o.target_ne[1] - 200.0).abs() < 0.02, "{:?}", o.target_ne);
        let [n, e, d] = ned_from_geo(&ORIGIN, &GeoPoint { alt_mm: ORIGIN.alt_mm, ..t });
        assert_eq!(d, 0.0);
        assert_eq!(o.target_ne, [n, e]);
        assert!((o.dist_m - hypotf(300.0, 200.0)).abs() < 0.05, "{}", o.dist_m);
        let moved = GeoPoint { lat_e7: ORIGIN.lat_e7 + 10_000, ..ORIGIN };
        let o2 = g.step(&at_rest(0.0, 0.0), Some(moved), 5.0, DT_US);
        let [n2, e2, _] = ned_from_geo(&moved, &GeoPoint { alt_mm: moved.alt_mm, ..t });
        assert_eq!(o2.target_ne, [n2, e2]);
        assert!((o2.target_ne[0] - o.target_ne[0]).abs() > 50.0);
    }

    /// The bearing (clockwise from north) and the velocity direction for targets due N, E, S and W.
    #[test]
    fn h2_bearing_and_velocity_direction_for_the_four_compass_targets() {
        for (tn, te) in [(200.0f32, 0.0f32), (0.0, 200.0), (-200.0, 0.0), (0.0, -200.0)] {
            let mut g = Guidance::<4>::new();
            g.set_targets(&[geo_at(&ORIGIN, f64::from(tn), f64::from(te), ORIGIN.alt_mm)], 2.0);
            let start = at_rest(10.0, -10.0);
            g.anchor(&start);
            let mut cur = start;
            let mut last = None;
            for k in 0..50 {
                let o = g.step(&cur, Some(ORIGIN), 5.0, k * DT_US);
                let (p, v, _) = position_of(&o);
                cur.position_m = [p[0], p[1], cur.position_m[2]];
                cur.velocity_ms = v;
                last = Some(o);
            }
            let o = last.unwrap();
            let (dn, de) = (tn - 10.0, te + 10.0);
            let d = hypotf(dn, de);
            assert!((libm::cosf(o.bearing_rad) - dn / d).abs() < 1e-3 && (libm::sinf(o.bearing_rad) - de / d).abs() < 1e-3, "bearing {} for {tn} {te}", o.bearing_rad);
            let (_, v, _) = position_of(&o);
            let speed = hypotf(v[0], v[1]);
            assert!(speed > 0.1, "{speed}");
            assert!((v[0] / speed - dn / d).abs() < 1e-3 && (v[1] / speed - de / d).abs() < 1e-3, "velocity {v:?} for {tn} {te}");
            assert_eq!(v[2], 0.0);
        }
        for (te_n, te_e, brg) in [(200.0, 0.0, 0.0), (0.0, 200.0, core::f32::consts::FRAC_PI_2), (0.0, -200.0, -core::f32::consts::FRAC_PI_2)] {
            let mut g = Guidance::<4>::new();
            g.set_targets(&[geo_at(&ORIGIN, te_n, te_e, ORIGIN.alt_mm)], 2.0);
            let o = g.step(&at_rest(0.0, 0.0), Some(ORIGIN), 5.0, 0);
            assert!((o.bearing_rad - brg).abs() < 1e-3, "{} vs {brg}", o.bearing_rad);
        }
        let mut g = Guidance::<4>::new();
        g.set_targets(&[geo_at(&ORIGIN, -200.0, 0.0, ORIGIN.alt_mm)], 2.0);
        let o = g.step(&at_rest(0.0, 0.0), Some(ORIGIN), 5.0, 0);
        assert!((o.bearing_rad.abs() - core::f32::consts::PI).abs() < 1e-3, "{}", o.bearing_rad);
    }

    /// The setpoint's other fields: the vehicle's own D, vD 0, its current yaw, rate 0.
    #[test]
    fn the_setpoint_carries_the_vehicle_s_own_height_and_yaw() {
        let mut g = Guidance::<4>::new();
        g.set_targets(&[geo_at(&ORIGIN, 100.0, 0.0, ORIGIN.alt_mm)], 2.0);
        let nav = nav([1.0, 2.0, -7.25], [0.0; 2], 0.7, OK);
        let o = g.step(&nav, Some(ORIGIN), 5.0, 0);
        let (p, v, h) = position_of(&o);
        assert_eq!(p[2], -7.25);
        assert_eq!(v[2], 0.0);
        let [w, _, _, z] = nav.attitude;
        assert_eq!(h, 2.0 * libm::atan2f(z, w));
        assert!((h - 0.7).abs() < 1e-5);
        assert_eq!([p[0], p[1]], [1.0, 2.0]);
        assert_eq!(o.status, status::RUNNING);
    }

    /// The speed profile of one leg.
    #[test]
    fn h3_the_carrot_speed_is_bounded_by_cruise_the_acceleration_and_the_braking_curve() {
        let mut g = Guidance::<4>::new();
        g.set_targets(&[geo_at(&ORIGIN, 100.0, 0.0, ORIGIN.alt_mm)], 0.05);
        let start = at_rest(0.0, 0.0);
        g.anchor(&start);
        let cruise = 5.0;
        let (mut cur, mut v_prev, mut top) = (start, 0.0f32, 0.0f32);
        let mut steps = 0;
        for k in 0..5_000u64 {
            let (s_before, len) = (g.s, hypotf(g.targets_ne()[0], g.targets_ne()[1]));
            let o = g.step(&cur, Some(ORIGIN), cruise, k * DT_US);
            if o.status & status::DONE != 0 {
                break;
            }
            let (p, vel, _) = position_of(&o);
            let v = hypotf(vel[0], vel[1]);
            assert!(v <= cruise + 1e-4, "step {k}: {v}");
            assert!((v - v_prev).abs() <= A * DT + 1e-4, "step {k}: dv {}", v - v_prev);
            let r = (len - s_before).max(0.0);
            assert!(v <= sqrtf(2.0 * A * r) + A * DT + 1e-3, "step {k}: v {v} r {r}");
            v_prev = v;
            top = top.max(v);
            cur.position_m = [p[0], p[1], cur.position_m[2]];
            cur.velocity_ms = vel;
            steps += 1;
        }
        assert!((top - cruise).abs() < 1e-3, "cruise reached: {top}");
        assert!(steps > 1_000 && steps < 5_000, "{steps}");
    }

    /// The index advances exactly once per target, then done.
    #[test]
    fn h4_the_index_advances_once_per_target_then_done() {
        let t: [GeoPoint; 3] = [
            geo_at(&ORIGIN, 100.0, 0.0, ORIGIN.alt_mm),
            geo_at(&ORIGIN, 100.0, 100.0, ORIGIN.alt_mm),
            geo_at(&ORIGIN, 0.0, 100.0, ORIGIN.alt_mm),
        ];
        let mut g = Guidance::<4>::new();
        g.set_targets(&t, 2.0);
        let ne = |k: usize| {
            let [n, e, _] = ned_from_geo(&ORIGIN, &GeoPoint { alt_mm: ORIGIN.alt_mm, ..t[k] });
            (n, e)
        };
        let mut tick = 0u64;
        let mut step = |g: &mut Guidance<4>, n: f32, e: f32| {
            tick += 1;
            g.step(&at_rest(n, e), Some(ORIGIN), 5.0, tick * DT_US)
        };
        let (n0, e0) = ne(0);
        for _ in 0..5 {
            let o = step(&mut g, n0 - 3.0, e0);
            assert_eq!((o.active, o.status), (0, status::RUNNING));
        }
        let o = step(&mut g, n0 - 1.9, e0);
        assert_eq!((o.active, o.status), (1, status::ARRIVED | status::RUNNING));
        for _ in 0..5 {
            let o = step(&mut g, n0 - 1.9, e0);
            assert_eq!((o.active, o.status), (1, status::RUNNING));
        }
        let (n1, e1) = ne(1);
        let o = step(&mut g, n1, e1 - 1.0);
        assert_eq!((o.active, o.status), (2, status::ARRIVED | status::RUNNING));
        let (n2, e2) = ne(2);
        let o = step(&mut g, n2 + 1.0, e2);
        assert_eq!((o.active, o.status), (2, status::ARRIVED | status::DONE));
        let (p, v, _) = position_of(&o);
        assert_eq!([p[0], p[1]], [n2, e2]);
        assert_eq!(v, [0.0; 3]);
        for (n, e) in [(n2 + 1.0, e2), (n2 + 50.0, e2 - 30.0), (0.0, 0.0)] {
            let o = step(&mut g, n, e);
            assert_eq!((o.active, o.status), (2, status::DONE));
            let (p, v, _) = position_of(&o);
            assert_eq!(([p[0], p[1]], v), ([n2, e2], [0.0; 3]));
        }
        g.reset();
        let o = step(&mut g, 0.0, 0.0);
        assert_eq!((o.active, o.status), (0, status::RUNNING));
    }

    /// Unusable input gives no setpoint, no NaN, and leaves the index.
    #[test]
    fn h5_invalid_input_gives_none_no_nan_and_the_index_stays() {
        let t = [geo_at(&ORIGIN, 100.0, 0.0, ORIGIN.alt_mm), geo_at(&ORIGIN, 100.0, 100.0, ORIGIN.alt_mm)];
        let mut g = Guidance::<4>::new();
        g.set_targets(&t, 2.0);
        let [n0, e0, _] = ned_from_geo(&ORIGIN, &GeoPoint { alt_mm: ORIGIN.alt_mm, ..t[0] });
        let o = g.step(&at_rest(n0, e0), Some(ORIGIN), 5.0, 0);
        assert_eq!(o.active, 1);
        let good = at_rest(0.0, 0.0);
        let mut bad = Vec::new();
        bad.push((nav(good.position_m, [0.0; 2], 0.0, Valid { position_ne: false, ..OK }), Some(ORIGIN), 5.0));
        bad.push((good, None, 5.0));
        bad.push((nav([f32::NAN, 0.0, 0.0], [0.0; 2], 0.0, OK), Some(ORIGIN), 5.0));
        bad.push((nav([0.0, f32::INFINITY, 0.0], [0.0; 2], 0.0, OK), Some(ORIGIN), 5.0));
        bad.push((nav([0.0, 0.0, f32::NAN], [0.0; 2], 0.0, OK), Some(ORIGIN), 5.0));
        bad.push((nav([0.0; 3], [f32::NAN, 0.0], 0.0, OK), Some(ORIGIN), 5.0));
        bad.push((nav([0.0; 3], [0.0; 2], f32::NAN, OK), Some(ORIGIN), 5.0));
        bad.push((good, Some(ORIGIN), f32::NAN));
        for (k, (n, o, s)) in bad.into_iter().enumerate() {
            let out = g.step(&n, o, s, (k as u64 + 1) * DT_US);
            assert_eq!(out.setpoint, None, "case {k}");
            assert_eq!((out.active, out.status), (1, status::INVALID), "case {k}");
            finite(&out);
        }
        let mut g = Guidance::<4>::new();
        g.set_targets(&t, 2.0);
        let _ = g.step(&at_rest(0.0, 0.0), Some(ORIGIN), 5.0, 0);
        let none = g.step(&good, None, 5.0, DT_US);
        assert_eq!(none.setpoint, None);
        let o = g.step(&at_rest(20.0, 30.0), Some(ORIGIN), 5.0, 2 * DT_US);
        let (p, v, _) = position_of(&o);
        assert_eq!(([p[0], p[1]], [v[0], v[1]]), ([20.0, 30.0], [0.0, 0.0]), "a fresh start at the new position");
        let mut off = Guidance::<4>::new();
        let o = off.step(&good, Some(ORIGIN), 5.0, 0);
        assert_eq!((o.setpoint, o.status), (None, 0));
        g.set_targets(&t, f32::NAN);
        finite(&g.step(&good, Some(ORIGIN), 0.0, 0));
        finite(&g.step(&good, Some(ORIGIN), -3.0, DT_US));
    }

    /// Anchored once, a point mass that follows the setpoint exactly reaches every target.
    #[test]
    fn h6_a_point_mass_that_follows_exactly_reaches_every_target() {
        let t = [
            geo_at(&ORIGIN, 150.0, 20.0, ORIGIN.alt_mm),
            geo_at(&ORIGIN, 150.0, -120.0, ORIGIN.alt_mm + 5_000),
            geo_at(&ORIGIN, -60.0, -120.0, ORIGIN.alt_mm),
            geo_at(&ORIGIN, -60.0, 40.0, ORIGIN.alt_mm - 5_000),
        ];
        let mut g = Guidance::<4>::new();
        g.set_targets(&t, 2.0);
        let outs = follow(&mut g, at_rest(0.0, 0.0), 5.0, 20_000, true);
        let last = outs.last().unwrap();
        assert!(last.status & status::DONE != 0, "not done after {} steps", outs.len());
        let mut adv = Vec::new();
        for (k, w) in outs.windows(2).enumerate() {
            if w[1].active != w[0].active {
                assert_eq!(w[1].active, w[0].active + 1);
                assert!(w[1].status & status::ARRIVED != 0);
                adv.push(k);
            }
        }
        assert_eq!(adv.len(), 3, "advances");
        for (i, &k) in adv.iter().enumerate() {
            assert!(outs[k].dist_m <= 2.0 + 5.0 * DT, "target {i}: {}", outs[k].dist_m);
        }
        let [n, e, _] = ned_from_geo(&ORIGIN, &GeoPoint { alt_mm: ORIGIN.alt_mm, ..t[3] });
        let (p, v, _) = position_of(last);
        assert_eq!(([p[0], p[1]], v), ([n, e], [0.0; 3]));
    }

    /// With an anchor at every step the carrot is the vehicle's projection: a vehicle off the leg gets the point on it.
    #[test]
    fn anchor_places_the_carrot_on_the_leg_clamped() {
        let mut g = Guidance::<4>::new();
        g.set_targets(&[geo_at(&ORIGIN, 100.0, 0.0, ORIGIN.alt_mm)], 2.0);
        let _ = g.step(&at_rest(0.0, 0.0), Some(ORIGIN), 5.0, 0);
        let off = nav([40.0, 7.0, -1.0], [3.0, 4.0], 0.0, OK);
        g.anchor(&off);
        let o = g.step(&off, Some(ORIGIN), 5.0, DT_US);
        let (p, v, _) = position_of(&o);
        assert!((p[0] - (40.0 + 3.0 * DT)).abs() < 0.01 && p[1].abs() < 1e-3, "{p:?}");
        assert!((v[0] - (3.0 + A * DT)).abs() < 1e-3, "{v:?}");
        let back = nav([-10.0, 0.0, 0.0], [-2.0, 0.0], 0.0, OK);
        g.anchor(&back);
        let o = g.step(&back, Some(ORIGIN), 5.0, 2 * DT_US);
        let (p, v, _) = position_of(&o);
        assert!(p[0].abs() < 1e-3 && (v[0] - A * DT).abs() < 1e-3, "{p:?} {v:?}");
        let fast = nav([10.0, 0.0, 0.0], [9.0, 0.0], 0.0, OK);
        g.anchor(&fast);
        let o = g.step(&fast, Some(ORIGIN), 5.0, 3 * DT_US);
        let (_, v, _) = position_of(&o);
        assert!((v[0] - 5.0).abs() < 1e-3, "{v:?}");
    }

    impl<const N: usize> Guidance<N> {
        /// The first target's NE, for the tests.
        fn targets_ne(&self) -> [f32; 2] {
            self.ne_of(&ORIGIN, 0)
        }
    }
}
