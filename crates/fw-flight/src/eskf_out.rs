//! The output predictor (decision-nav-eskf-20261120, Amendment G, G1 and G2; packet P-G2): [`EskfSteer`] wraps the
//! delayed-horizon navigation filter ([`Eskf`], `Aiding::Full`) and returns a steering estimate at the CURRENT IMU
//! sample. PX4's `output_predictor.cpp` in its earth-frame form: a second strapdown (the output) integrates every IMU
//! sample with the core's own `nominal_step` and its current biases; every `nav_predict_us` its state is pushed into a
//! ring; when the core has integrated the sample of the oldest ring entry (the core stores that sample's nominal state,
//! `Eskf::set_snap_request`), the output history is corrected toward it: the attitude by an earth-frame rate
//! (`nav_out_att_gain` / T_d, PX4 :301-311), the velocity and the position by a proportional-integral law on every ring
//! entry and the current state (PX4 :326-361). A reset of the core's state (`Eskf::last_reset`) is applied to the output
//! as the exact delta, so the output never steps when the core resets. The core is not changed by the wrapper.
//!
//! Where this implementation takes a reading of the specification:
//! - [`EskfSteer::new`] returns `Result`: Err when the ring (compiled from `Tuning::DEFAULT`, [`OUT`] entries) is too
//!   small for the tuning's `nav_buffer_us / nav_predict_us + 1`, or `nav_predict_us` is 0.
//! - The step's `dt` is `min(now - previous propagated sample time, DT_MAX_US)`, as the core's `integrate`; a
//!   non-finite IMU sample propagates nothing and does not move that time. T_d and the push interval use it too.
//! - T_d is floored at [`DT_CORR_MIN_S`] (PX4 :280's lower bound) so that a repeated timestamp cannot divide by zero.
//! - "Moved" for the one-step flag rule is decided by exact zero tests of the core's `Reset`: heading when `dq[3] != 0`
//!   (a core yaw reset is a rotation about down, and a step that changed nothing has `dq` exactly the identity, as
//!   `q x conj(q)` is exactly `[w, 0, 0, 0]` in f32); the NE pair when any of `dv[0..2]`, `dp[0..2]` is non-zero; the
//!   vertical pair when `dv[2]` or `dp[2]` is non-zero.
//! - The alignment copy takes the core's nominal state at its HORIZON (the core's alignment refers to the horizon); the
//!   output is then advanced by its own integration from the current sample, and the first correction (one delay
//!   later) takes out what the copy left. At rest, where alignment happens, that is nothing.
//! - The velocity and position integrals are not limited (PX4 does not); they restart at every alignment copy.
//! - `rates_rads` come from a PT1 of the raw gyro that starts at the first finite sample, independent of `live`; the
//!   rates flag is the core's.
//! - Not guarded: a finite but overflowing accelerometer value (above about 1e38) would make the OUTPUT non-finite at
//!   once and the core's 430 ms later (the IMU driver's range is far below that).

// Fixed-size arrays only; every index is a loop bound or a constant below the array length.
#![allow(clippy::indexing_slicing, clippy::needless_range_loop)]

use crate::eskf::{Eskf, Reset};
use crate::eskf_n0::{self as n0, Nominal, rotation};
use crate::gnc::{Estimator, GeoPoint, NavState, Sensors};
use crate::mahony::DT_MAX_US;
use crate::quat::{conj, mul, normalise, pt1};
use crate::tuning::Tuning;
use libm::sqrtf;

#[cfg(test)]
extern crate std;

/// Entries of the output ring: `nav_buffer_us / nav_predict_us + 1`, 51 with the default tuning (G1). The same sizing
/// rule as `eskf::BARO_WAIT`.
pub const OUT: usize = (Tuning::DEFAULT.nav_buffer_us / Tuning::DEFAULT.nav_predict_us + 1) as usize;
/// Lower clamp of the time between two corrections and of T_d, s: PX4 `output_predictor.cpp:280`, `:326` (0.0001).
pub const DT_CORR_MIN_S: f32 = 1e-4;
/// Upper clamp of the time between two corrections, s: PX4 `output_predictor.cpp:280` (0.03).
pub const DT_CORR_MAX_S: f32 = 0.03;
/// Upper clamp of the velocity and position time constants, s: PX4 `output_predictor.cpp:326-327` (10).
pub const TAU_MAX_S: f32 = 10.0;

/// The constructor's refusal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutError {
    /// `nav_predict_us` is 0, or `nav_buffer_us / nav_predict_us + 1` exceeds [`OUT`].
    RingTooSmall,
}

/// One entry of the output ring, 48 B: the output state at the IMU sample of time `t_us`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct OutEntry {
    t_us: u64,
    q: [f32; 4],
    v: [f32; 3],
    p: [f32; 3],
}

/// First-in first-out ring of [`OUT`] entries; a push into a full ring drops the oldest.
#[derive(Clone, Copy, Debug)]
struct OutRing {
    buf: [OutEntry; OUT],
    head: usize,
    len: usize,
}

impl OutRing {
    fn new() -> Self {
        OutRing { buf: [OutEntry::default(); OUT], head: 0, len: 0 }
    }
    fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }
    fn front(&self) -> Option<OutEntry> {
        (self.len > 0).then(|| self.buf[self.head])
    }
    fn pop(&mut self) -> Option<OutEntry> {
        let e = self.front()?;
        self.head = (self.head + 1) % OUT;
        self.len -= 1;
        Some(e)
    }
    fn push(&mut self, e: OutEntry) {
        if self.len == OUT {
            self.pop();
        }
        self.buf[(self.head + self.len) % OUT] = e;
        self.len += 1;
    }
    fn for_each(&mut self, mut f: impl FnMut(&mut OutEntry)) {
        for i in 0..self.len {
            f(&mut self.buf[(self.head + i) % OUT]);
        }
    }
}

/// The last correction's tracking errors, for the log (G9: NKI Src 10 and 11).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Tracking {
    /// Magnitude of the attitude error, rad (2 |dq.xyz|, PX4 `_output_tracking_error(0)`).
    pub att_rad: f32,
    /// Magnitude of the velocity error, m/s.
    pub vel_ms: f32,
    /// Magnitude of the position error, m.
    pub pos_m: f32,
    /// T_d of that correction, s.
    pub t_d_s: f32,
}

/// The tuning numbers the predictor uses.
#[derive(Clone, Copy, Debug)]
struct Gains {
    tau_vel_s: f32,
    tau_pos_s: f32,
    att_gain: f32,
    int_gain: f32,
    gyro_lpf_hz: f32,
    predict_us: u64,
}

/// What a reset delta moved in this step.
#[derive(Clone, Copy, Debug, Default)]
struct Moved {
    heading: bool,
    ne: bool,
    d: bool,
}

/// The output state, its ring and the correction state (G1).
#[derive(Clone, Debug)]
struct Predictor {
    nom: Nominal,
    t_us: u64,
    live: bool,
    buf: OutRing,
    rate_corr: [f32; 3],
    iv: [f32; 3],
    ip: [f32; 3],
    since_push_us: u64,
    t_corr_prev: Option<u64>,
    rates_lpf: [f32; 3],
    rates_prev_us: Option<u64>,
    skipped: u32,
    pushed: u32,
    corrected: u32,
    track: Tracking,
    g: Gains,
}

#[cfg(test)]
std::thread_local! {
    /// Test control (T12): correct against the core's final horizon time instead of the snapshot.
    static MATCH_HORIZON: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Test mutation m1: the sign of the attitude error.
    static FLIP_ATT: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Test mutation m2: the slew on the body side.
    static SLEW_BODY: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Test mutation m3: the reset delta not applied to the ring.
    static RESET_SKIP_RING: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Test mutation m4: the biases not copied from the core.
    static NO_BIAS_COPY: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Test mutation m5: the rates from the raw gyro.
    static RAW_RATES: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
    /// Test control (T4): the reset rotation on the body side.
    static RESET_BODY_SIDE: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
}

/// `x` limited to `[lo, hi]` by explicit comparisons (not `f32::clamp`: its panic message formats the bounds, and the
/// bounds here are not constants; see `pid.rs`'s `tilt_factor`).
fn limit(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

fn norm3(a: &[f32; 3]) -> f32 {
    sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
}

impl Predictor {
    fn new(t: &Tuning) -> Self {
        Predictor {
            nom: Nominal { q: [1.0, 0.0, 0.0, 0.0], v: [0.0; 3], p: [0.0; 3], bg: [0.0; 3], ba: [0.0; 3], dvel: [0.0; 3], dt_sum: 0.0 },
            t_us: 0,
            live: false,
            buf: OutRing::new(),
            rate_corr: [0.0; 3],
            iv: [0.0; 3],
            ip: [0.0; 3],
            since_push_us: 0,
            t_corr_prev: None,
            rates_lpf: [0.0; 3],
            rates_prev_us: None,
            skipped: 0,
            pushed: 0,
            corrected: 0,
            track: Tracking::default(),
            g: Gains {
                tau_vel_s: t.nav_out_tau_vel_s,
                tau_pos_s: t.nav_out_tau_pos_s,
                att_gain: t.nav_out_att_gain,
                int_gain: t.nav_out_int_gain,
                gyro_lpf_hz: t.ins_gyro_lpf_hz,
                predict_us: t.nav_predict_us,
            },
        }
    }

    /// G1 step 3: PX4's align by copy.
    fn copy_from(&mut self, core: &Eskf, now: u64) {
        let (q, v, p) = core.nominal();
        let (bg, ba) = core.biases();
        self.nom = Nominal { q, v, p, bg, ba, dvel: [0.0; 3], dt_sum: 0.0 };
        self.buf.clear();
        self.rate_corr = [0.0; 3];
        self.iv = [0.0; 3];
        self.ip = [0.0; 3];
        self.t_us = now;
        self.since_push_us = 0;
        self.t_corr_prev = None;
        self.track = Tracking::default();
        self.live = true;
    }

    /// The ring entry at exactly `t_e`: older entries are popped and counted as skipped, a newer front stays.
    fn take(&mut self, t_e: u64) -> Option<OutEntry> {
        while let Some(f) = self.buf.front() {
            if f.t_us > t_e {
                break;
            }
            self.buf.pop();
            if f.t_us == t_e {
                return Some(f);
            }
            self.skipped = self.skipped.wrapping_add(1);
        }
        None
    }

    /// G1 step 4a: the correction toward the core's snapshot, at most one per step.
    fn correct(&mut self, core: &Eskf, now: u64, dt: f32) {
        let snap = core.snapshot();
        #[cfg(test)]
        let snap = if MATCH_HORIZON.get() {
            core.horizon_us().map(|t| {
                let (q, v, p) = core.nominal();
                (t, q, v, p)
            })
        } else {
            snap
        };
        let Some((t_e, q_h, v_h, p_h)) = snap else {
            return;
        };
        let Some(e) = self.take(t_e) else {
            return;
        };
        let mut dq = normalise(mul(q_h, conj(e.q)));
        if dq[0] < 0.0 {
            dq = dq.map(|x| -x);
        }
        #[allow(unused_mut)]
        let mut e_th = [2.0 * dq[1], 2.0 * dq[2], 2.0 * dq[3]];
        #[cfg(test)]
        if FLIP_ATT.get() {
            e_th = e_th.map(|x| -x);
        }
        let mut t_d = now.saturating_sub(t_e) as f32 * 1e-6;
        if t_d < dt {
            t_d = dt;
        }
        if t_d < DT_CORR_MIN_S {
            t_d = DT_CORR_MIN_S;
        }
        let k = self.g.att_gain / t_d;
        self.rate_corr = [e_th[0] * k, e_th[1] * k, e_th[2] * k];
        let dt_c = match self.t_corr_prev {
            Some(prev) => limit(t_e.saturating_sub(prev) as f32 * 1e-6, DT_CORR_MIN_S, DT_CORR_MAX_S),
            None => self.g.predict_us as f32 * 1e-6,
        };
        self.t_corr_prev = Some(t_e);
        let g_v = dt_c / limit(self.g.tau_vel_s, dt_c, TAU_MAX_S);
        let g_p = dt_c / limit(self.g.tau_pos_s, dt_c, TAU_MAX_S);
        let e_v: [f32; 3] = core::array::from_fn(|i| v_h[i] - e.v[i]);
        let e_p: [f32; 3] = core::array::from_fn(|i| p_h[i] - e.p[i]);
        let mut dv = [0.0f32; 3];
        let mut dp = [0.0f32; 3];
        for i in 0..3 {
            self.iv[i] += e_v[i];
            self.ip[i] += e_p[i];
            dv[i] = e_v[i] * g_v + self.iv[i] * g_v * g_v * self.g.int_gain;
            dp[i] = e_p[i] * g_p + self.ip[i] * g_p * g_p * self.g.int_gain;
        }
        self.buf.for_each(|x| {
            for i in 0..3 {
                x.v[i] += dv[i];
                x.p[i] += dp[i];
            }
        });
        for i in 0..3 {
            self.nom.v[i] += dv[i];
            self.nom.p[i] += dp[i];
        }
        self.track = Tracking { att_rad: norm3(&e_th), vel_ms: norm3(&e_v), pos_m: norm3(&e_p), t_d_s: t_d };
        self.corrected = self.corrected.wrapping_add(1);
    }

    /// G1 step 4b: the core's reset as an exact delta on the current state, every entry and the rate correction.
    fn apply_reset(&mut self, r: Option<Reset>) -> Moved {
        let Some(r) = r else {
            return Moved::default();
        };
        let rot = |q: [f32; 4]| {
            #[cfg(test)]
            if RESET_BODY_SIDE.get() {
                return normalise(mul(q, r.dq));
            }
            normalise(mul(r.dq, q))
        };
        self.nom.q = rot(self.nom.q);
        for i in 0..3 {
            self.nom.v[i] += r.dv[i];
            self.nom.p[i] += r.dp[i];
        }
        #[cfg(test)]
        let skip_ring = RESET_SKIP_RING.get();
        #[cfg(not(test))]
        let skip_ring = false;
        if !skip_ring {
            self.buf.for_each(|x| {
                x.q = rot(x.q);
                for i in 0..3 {
                    x.v[i] += r.dv[i];
                    x.p[i] += r.dp[i];
                }
            });
        }
        let m = rotation(&r.dq);
        let c = self.rate_corr;
        self.rate_corr = core::array::from_fn(|i| m[i][0] * c[0] + m[i][1] * c[1] + m[i][2] * c[2]);
        Moved {
            heading: r.dq[3] != 0.0,
            ne: r.dv[0] != 0.0 || r.dv[1] != 0.0 || r.dp[0] != 0.0 || r.dp[1] != 0.0,
            d: r.dv[2] != 0.0 || r.dp[2] != 0.0,
        }
    }

    /// G1 step 4c: one IMU sample of propagation, the earth-side slew, the periodic push.
    fn propagate(&mut self, core: &Eskf, s: &Sensors, dt_us: u64) {
        let now = s.imu1.t_us;
        let dt = dt_us as f32 * 1e-6;
        let (bg, ba) = core.biases();
        #[cfg(test)]
        let keep = NO_BIAS_COPY.get();
        #[cfg(not(test))]
        let keep = false;
        if !keep {
            self.nom.bg = bg;
            self.nom.ba = ba;
        }
        let r = &s.imu1.reading;
        n0::nominal_step(&mut self.nom, &r.gyro_rads, &r.accel_ms2, dt);
        self.nom.dvel = [0.0; 3];
        self.nom.dt_sum = 0.0;
        let c = self.rate_corr;
        let slew = [1.0, 0.5 * c[0] * dt, 0.5 * c[1] * dt, 0.5 * c[2] * dt];
        #[cfg(test)]
        let body = SLEW_BODY.get();
        #[cfg(not(test))]
        let body = false;
        self.nom.q = if body { normalise(mul(self.nom.q, slew)) } else { normalise(mul(slew, self.nom.q)) };
        self.t_us = now;
        self.since_push_us += dt_us;
        if self.since_push_us >= self.g.predict_us {
            self.buf.push(OutEntry { t_us: now, q: self.nom.q, v: self.nom.v, p: self.nom.p });
            self.pushed = self.pushed.wrapping_add(1);
            self.since_push_us = 0;
        }
    }
}

/// The steering estimator: the delayed-horizon core and its output predictor (G1). It borrows the core, which the
/// board builds as a static.
#[derive(Debug)]
pub struct EskfSteer<'a> {
    core: &'a mut Eskf,
    out: Predictor,
}

impl<'a> EskfSteer<'a> {
    /// The wrapper over `core` with the output numbers of `tuning`. Err when the output ring is too small for
    /// `tuning.nav_buffer_us / tuning.nav_predict_us + 1` entries.
    pub fn new(core: &'a mut Eskf, tuning: &Tuning) -> Result<Self, OutError> {
        if tuning.nav_predict_us == 0 || tuning.nav_buffer_us / tuning.nav_predict_us + 1 > OUT as u64 {
            return Err(OutError::RingTooSmall);
        }
        Ok(EskfSteer { core, out: Predictor::new(tuning) })
    }

    /// The wrapped core (its horizon state, statistics and events).
    pub fn core(&self) -> &Eskf {
        self.core
    }

    /// The last correction's tracking errors and T_d.
    pub fn tracking(&self) -> Tracking {
        self.out.track
    }

    /// Ring entries popped without being the snapshot's time (0 in every run: a count of lost corrections).
    pub fn skipped(&self) -> u32 {
        self.out.skipped
    }

    /// Entries pushed into the ring.
    pub fn pushed(&self) -> u32 {
        self.out.pushed
    }

    /// Corrections applied.
    pub fn corrected(&self) -> u32 {
        self.out.corrected
    }
}

impl Estimator for EskfSteer<'_> {
    fn update(&mut self, sensors: &Sensors) -> NavState {
        let now = sensors.imu1.t_us;
        let r = &sensors.imu1.reading;
        let finite = r.gyro_rads.iter().chain(&r.accel_ms2).all(|v| v.is_finite());
        // 1. The snapshot request is the time of the oldest output entry; the core's update.
        self.core.set_snap_request(self.out.buf.front().map(|e| e.t_us));
        let h = self.core.update(sensors);
        let core = &*self.core;
        let o = &mut self.out;
        let dt_us = now.saturating_sub(o.t_us).min(DT_MAX_US);
        let mut moved = Moved::default();
        if !core.tilt_aligned() {
            // 2.
            o.live = false;
        } else if !o.live {
            // 3.
            o.copy_from(core, now);
        } else {
            // 4a, 4b, 4c.
            o.correct(core, now, dt_us as f32 * 1e-6);
            moved = o.apply_reset(core.last_reset());
            if finite {
                o.propagate(core, sensors, dt_us);
            }
        }
        // 5. The PT1 rates, Mahony's.
        if finite {
            o.rates_lpf = match o.rates_prev_us {
                Some(prev) => {
                    let dt = now.saturating_sub(prev).min(DT_MAX_US) as f32 * 1e-6;
                    let hz = o.g.gyro_lpf_hz;
                    let [a, b, c] = o.rates_lpf;
                    [pt1(a, r.gyro_rads[0], dt, hz), pt1(b, r.gyro_rads[1], dt, hz), pt1(c, r.gyro_rads[2], dt, hz)]
                }
                None => r.gyro_rads,
            };
            o.rates_prev_us = Some(now);
        }
        // 6. The NavState.
        let (bg, _) = core.biases();
        #[cfg(test)]
        let raw = RAW_RATES.get();
        #[cfg(not(test))]
        let raw = false;
        let g = if raw { r.gyro_rads } else { o.rates_lpf };
        let mut valid = h.valid;
        valid.tilt = valid.tilt && o.live;
        if moved.d {
            valid.velocity_d = false;
            valid.position_d = false;
        }
        if moved.ne {
            valid.velocity_ne = false;
            valid.position_ne = false;
        }
        if moved.heading {
            valid.heading = false;
        }
        let q = o.nom.q;
        NavState {
            t_us: now,
            attitude: if q[0] < 0.0 { q.map(|x| -x) } else { q },
            rates_rads: [g[0] - bg[0], g[1] - bg[1], g[2] - bg[2]],
            velocity_ms: o.nom.v,
            position_m: o.nom.p,
            valid,
            sigma: h.sigma,
        }
    }

    fn origin(&self) -> Option<GeoPoint> {
        self.core.origin()
    }

    fn set_armed(&mut self, armed: bool) {
        self.core.set_armed(armed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eskf::tests::{Fault, PERIOD_US, World, pressure_at};
    use crate::mahony::Mahony;
    use core::f32::consts::PI;
    use libm::{cosf, fabsf, sinf};
    use std::println;
    use std::vec::Vec;

    #[test]
    fn sizes_and_the_ring_capacity_rule() {
        println!("size_of EskfSteer {} Predictor {} OutEntry {} OutRing {} OUT {OUT}", size_of::<EskfSteer>(), size_of::<Predictor>(), size_of::<OutEntry>(), size_of::<OutRing>());
        assert_eq!(size_of::<OutEntry>(), 48);
        assert_eq!(OUT, 51);
        let mut core = Eskf::new(&Tuning::DEFAULT).unwrap();
        let long = Tuning { nav_buffer_us: Tuning::DEFAULT.nav_buffer_us + Tuning::DEFAULT.nav_predict_us, ..Tuning::DEFAULT };
        assert!(matches!(EskfSteer::new(&mut core, &long), Err(OutError::RingTooSmall)));
        let zero = Tuning { nav_predict_us: 0, ..Tuning::DEFAULT };
        assert!(matches!(EskfSteer::new(&mut core, &zero), Err(OutError::RingTooSmall)));
        assert!(EskfSteer::new(&mut core, &Tuning::DEFAULT).is_ok());
    }

    /// The 100 s stream of the G1-e Full run without the re-initialisation: yawed, moving at constant velocity, a
    /// yaw alignment (first compass), the GPS reset at arming (5 s), a C3 reset (a 10-sigma north fault from 20 s), a yaw
    /// reset (compass turned 90 deg from 60 s), the height reset (barometer 40 m too high from 85 s).
    fn stream(t: &Tuning, secs: u64) -> Vec<Sensors> {
        let mut w = World::new(t);
        w.yaw = 0.4;
        w.v = [2.0, 0.5, 0.0];
        w.gps_vel = true;
        let sigma = (f64::from(w.std_mm) * 1e-3).max(f64::from(t.nav_gps_pos_sigma_min_m));
        let mut out = Vec::new();
        while w.t_us() < secs * 1_000_000 {
            if w.t_us() >= 20_000_000 && w.fault == Fault::None {
                w.fault = Fault::North(w.t_us(), 10.0 * sigma);
            }
            if w.t_us() >= 60_000_000 {
                w.mag_rot = 90.0_f64.to_radians();
            }
            let mut s = w.step();
            if let Some(b) = s.baro.as_mut().filter(|_| w.t_us() >= 85_000_000) {
                b.reading.pressure_pa = pressure_at(40.0 - w.p[2]);
            }
            out.push(s);
        }
        out
    }

    fn armed_at(s: &Sensors) -> bool {
        s.imu1.t_us >= 5_000_000
    }

    /// The core's state that no wrapper may touch, as bits: nominal, biases, sigmas (the diagonal of P), horizon.
    fn core_bits(e: &Eskf) -> Vec<u32> {
        let (q, v, p) = e.nominal();
        let (bg, ba) = e.biases();
        let mut b: Vec<u32> = q.iter().chain(&v).chain(&p).chain(&bg).chain(&ba).map(|x| x.to_bits()).collect();
        b.extend(e.sigmas().map(f32::to_bits));
        let h = e.horizon_us().unwrap_or(u64::MAX);
        b.extend([h as u32, (h >> 32) as u32, u32::from(e.tilt_aligned()), u32::from(e.yaw_aligned())]);
        b
    }

    #[test]
    fn t1_z_out_the_output_is_at_the_imu_sample_time_and_the_bare_core_lags_by_the_delay() {
        let t = Tuning::DEFAULT;
        let st = stream(&t, 20);
        let mut core = Eskf::new(&t).unwrap();
        let mut bare = Eskf::new(&t).unwrap();
        let mut steer = EskfSteer::new(&mut core, &t).unwrap();
        let (mut checked, mut lag_ok) = (0, 0);
        for s in &st {
            steer.set_armed(armed_at(s));
            bare.set_armed(armed_at(s));
            let nav = steer.update(s);
            let b = bare.update(s);
            assert_eq!(nav.t_us, s.imu1.t_us);
            checked += 1;
            if s.imu1.t_us >= 2 * t.nav_gps_delay_us {
                // Control: the bare core's t_us is the horizon time, one delay (on this grid exactly) behind.
                assert_eq!(s.imu1.t_us - b.t_us, t.nav_gps_delay_us);
                lag_ok += 1;
            }
        }
        println!("t1: {checked} steps with t_us == imu1.t_us; the bare core lagged by {} us on {lag_ok} of them", t.nav_gps_delay_us);
        assert!(lag_ok > 10_000);
    }

    #[test]
    fn t2_f_core_the_wrapped_core_equals_a_bare_core_bit_for_bit_over_100_s() {
        let t = Tuning::DEFAULT;
        let st = stream(&t, 100);
        let mut core = Eskf::new(&t).unwrap();
        let mut bare = Eskf::new(&t).unwrap();
        let mut steer = EskfSteer::new(&mut core, &t).unwrap();
        let (mut resets, mut nav_sigma_eq) = (0, 0);
        for (k, s) in st.iter().enumerate() {
            steer.set_armed(armed_at(s));
            bare.set_armed(armed_at(s));
            let nav = steer.update(s);
            let b = bare.update(s);
            assert_eq!(core_bits(steer.core()), core_bits(&bare), "step {k}");
            assert_eq!(steer.core().stats(), bare.stats(), "step {k}");
            assert!(steer.core().events().eq(bare.events()), "step {k}");
            assert_eq!(steer.core().last_reset(), bare.last_reset(), "step {k}");
            assert_eq!(steer.origin(), bare.origin(), "step {k}");
            assert_eq!(nav.sigma, b.sigma, "step {k}");
            resets += usize::from(bare.last_reset().is_some());
            nav_sigma_eq += 1;
        }
        let s = *bare.stats();
        println!("t2: {nav_sigma_eq} steps identical; {resets} reset steps; yaw aligns {} resets {}, gps resets {}, height resets {}", s.yaw_aligns, s.yaw_resets, s.gps_resets, s.height_resets);
        assert!(s.yaw_aligns >= 1 && s.yaw_resets >= 1 && s.gps_resets >= 2 && s.height_resets >= 1);
    }

    /// The steering output and Mahony on the same moving gyro for `secs`; with the core's bias at 0 every step's rates
    /// must equal bit for bit. Returns the largest rate seen.
    fn rates_vs_mahony(t: &Tuning, secs: u64) -> f32 {
        let mut core = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        let mut m = Mahony::new(t);
        let mut w = World::new(t);
        w.yaw = 0.4;
        let mut moved = 0.0f32;
        for k in 0..secs * 1_000_000 / PERIOD_US {
            let mut s = w.step();
            let ph = k as f32 * PERIOD_US as f32 * 1e-6;
            s.imu1.reading.gyro_rads = [0.3 * sinf(2.0 * PI * 1.7 * ph), 0.2 * cosf(2.0 * PI * 0.6 * ph), 0.1 * sinf(2.0 * PI * 3.1 * ph)];
            let armed = w.t_us() >= 4_000_000;
            steer.set_armed(armed);
            m.set_armed(armed);
            let (a, b) = (steer.update(&s), m.update(&s));
            assert_eq!(steer.core().biases().0.map(f32::to_bits), [0; 3], "step {k}: the gyro bias moved");
            assert_eq!(a.rates_rads.map(f32::to_bits), b.rates_rads.map(f32::to_bits), "step {k}: with bias 0 the rates are Mahony's");
            moved = moved.max(a.rates_rads.iter().fold(0.0, |x, v| fabsf(*v).max(x)));
        }
        moved
    }

    #[test]
    fn t10_r_out_rates_are_mahonys_with_bias_learning_off_and_differ_by_the_bias_on_all_three_axes_with_it_on() {
        // Learning off: no initial bias variance and no random walk; the bias stays exactly 0 and the rates are Mahony's.
        let off = Tuning { nav_p0_bg_rads: 0.0, nav_bg_rw: 0.0, ..Tuning::DEFAULT };
        let moved = rates_vs_mahony(&off, 10);
        assert!(moved > 0.1, "the gyro did not move: {moved}");
        // Learning on, a constant gyro offset on every axis: the difference is the bias estimate, to one rounding.
        let t = Tuning::DEFAULT;
        let mut core = Eskf::new(&t).unwrap();
        let mut steer = EskfSteer::new(&mut core, &t).unwrap();
        let mut m = Mahony::new(&t);
        let mut w = World::new(&t);
        let off = [0.02f32, -0.015, 0.01];
        let (mut worst, mut bg) = (0.0f64, [0.0f32; 3]);
        for _ in 0..60_000_000 / PERIOD_US {
            let mut s = w.step();
            s.imu1.reading.gyro_rads = off;
            steer.set_armed(w.t_us() >= 4_000_000);
            let (a, b) = (steer.update(&s), m.update(&s));
            bg = steer.core().biases().0;
            for i in 0..3 {
                let d = f64::from(b.rates_rads[i]) - f64::from(a.rates_rads[i]) - f64::from(bg[i]);
                let tol = f64::from(f32::EPSILON) * f64::from(b.rates_rads[i].abs().max(bg[i].abs()));
                assert!(d.abs() <= tol, "axis {i}: difference minus bias {d:e}, tolerance {tol:e}");
                worst = worst.max(d.abs());
            }
        }
        println!("t10: gyro offset {off:?}, bias estimate after 60 s {bg:?}, worst |difference - bias| {worst:e}");
        assert!(bg.iter().all(|b| fabsf(*b) > 1e-4), "every axis must have learned: {bg:?}");
    }
}
