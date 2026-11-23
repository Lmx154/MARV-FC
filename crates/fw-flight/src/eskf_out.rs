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

use crate::eskf::{Eskf, EskfN, Reset};
#[cfg(test)]
use crate::eskf::EskfZ;
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
    /// True when the ring was full and the oldest entry was dropped.
    fn push(&mut self, e: OutEntry) -> bool {
        let full = self.len == OUT;
        if full {
            self.pop();
        }
        self.buf[(self.head + self.len) % OUT] = e;
        self.len += 1;
        full
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
    reset_older: u32,
    ring_dropped: u32,
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
    /// Test control (T5): the old reset rule, without the velocity step's effect on the position.
    static RESET_NO_DV_TERM: core::cell::Cell<bool> = const { core::cell::Cell::new(false) };
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

/// Seconds from `t0` to `t` (0 if `t` is not later), us counters.
fn now_minus(t: u64, t0: u64) -> f32 {
    t.saturating_sub(t0) as f32 * 1e-6
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
            reset_older: 0,
            ring_dropped: 0,
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

    /// G1 step 4b: the core's reset as an exact delta on the current state, every entry and the rate correction. The
    /// core's velocity step `dv` at its horizon time `t_r` also moves its position afterwards (P-G2b, lead's ruling):
    /// an entry of time `t >= t_r` gets `p += dp + dv (t - t_r)`, the current state, which is at its own time `t_us` (the previous propagated sample; step 4c then integrates the new
    /// sample with the new velocity), `p += dp + dv (t_us - t_r)`. An entry
    /// older than `t_r` (none should exist after the pop) gets `dp` alone and is counted.
    fn apply_reset(&mut self, r: Option<Reset>, t_r: u64) -> Moved {
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
        #[cfg(test)]
        let dv_term = if RESET_NO_DV_TERM.get() { 0.0 } else { 1.0 };
        #[cfg(not(test))]
        let dv_term = 1.0f32;
        let ahead = |t: u64| now_minus(t, t_r) * dv_term;
        self.nom.q = rot(self.nom.q);
        let dt_now = ahead(self.t_us);
        for i in 0..3 {
            self.nom.v[i] += r.dv[i];
            self.nom.p[i] += r.dp[i] + r.dv[i] * dt_now;
        }
        #[cfg(test)]
        let skip_ring = RESET_SKIP_RING.get();
        #[cfg(not(test))]
        let skip_ring = false;
        if !skip_ring {
            let mut older = 0u32;
            self.buf.for_each(|x| {
                x.q = rot(x.q);
                let dt = if x.t_us >= t_r {
                    ahead(x.t_us)
                } else {
                    older += 1;
                    0.0
                };
                for i in 0..3 {
                    x.v[i] += r.dv[i];
                    x.p[i] += r.dp[i] + r.dv[i] * dt;
                }
            });
            self.reset_older = self.reset_older.wrapping_add(older);
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
            if self.buf.push(OutEntry { t_us: now, q: self.nom.q, v: self.nom.v, p: self.nom.p }) {
                self.ring_dropped = self.ring_dropped.wrapping_add(1);
            }
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

    /// Ring entries older than a reset's horizon time when the reset was applied (0 in every run).
    pub fn reset_older(&self) -> u32 {
        self.out.reset_older
    }

    /// Oldest entries dropped because the ring was full when a new one was pushed (0 in every run: the core's snapshot
    /// pops them first).
    pub fn ring_dropped(&self) -> u32 {
        self.out.ring_dropped
    }

    /// Corrections applied.
    pub fn corrected(&self) -> u32 {
        self.out.corrected
    }
}

/// Test access for the `Flight` test T6 in `step.rs`: an earth-frame tilt offset put into the output's current state.
#[cfg(test)]
impl EskfSteer<'_> {
    pub(crate) fn inject_tilt(&mut self, rad: f32) {
        let inj = [libm::cosf(0.5 * rad), libm::sinf(0.5 * rad), 0.0, 0.0];
        self.out.nom.q = normalise(mul(inj, self.out.nom.q));
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
            moved = o.apply_reset(core.last_reset(), core.horizon_us().unwrap_or(now));
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

/// The composite steering estimator (Amendment G-1, G-1b): attitude, rates and the horizontal state from the output
/// predictor ([`EskfSteer`]), the vertical state (D, v_D, their sigmas and flags) from the zero-delay filter ([`EskfN`], `Aiding::ImuBaro`, ring
/// size `RV`: the host tests use `EskfZ`, ring 1; the board `Eskf`, one monomorphisation of the filter in RAM). Both run on every step with the same sensors and the same armed state. A fixed composition, not
/// a switch.
///
/// Readings of the specification: the sigma is the output predictor's with the two vertical fields from the zero-delay
/// filter when both estimators give one; if either has none the composite has none (a vertical sigma of the other
/// filter would break "vertical fields only from the zero-delay filter", and the log reads `None` as infinite).
#[derive(Debug)]
pub struct SteerMix<'a, const RV: usize = 1> {
    h: EskfSteer<'a>,
    v: &'a mut EskfN<RV>,
    /// The zero-delay filter's own state of the last `update`, for the log (G-1b: I = 1).
    z: Option<NavState>,
}

impl<'a, const RV: usize> SteerMix<'a, RV> {
    /// The composite of the output predictor `h` and the zero-delay filter `v`.
    pub fn new(h: EskfSteer<'a>, v: &'a mut EskfN<RV>) -> Self {
        SteerMix { h, v, z: None }
    }

    /// The output predictor (attitude, rates, horizontal state).
    pub fn steer(&self) -> &EskfSteer<'a> {
        &self.h
    }

    /// The zero-delay filter (vertical state).
    pub fn vert(&self) -> &EskfN<RV> {
        self.v
    }

    /// The zero-delay filter's own `NavState` of the last `update` (what it returned before the composition took its
    /// vertical part); none before the first.
    pub fn vert_nav(&self) -> Option<&NavState> {
        self.z.as_ref()
    }
}

/// Test access for the `Flight` tests M2 and M3 in `step.rs`: the zero-delay filter, to force its state.
#[cfg(test)]
impl<const RV: usize> SteerMix<'_, RV> {
    pub(crate) fn vert_mut(&mut self) -> &mut EskfN<RV> {
        self.v
    }
}

impl<const RV: usize> SteerMix<'_, RV> {
    /// One step of the composition, the body of `Estimator::update`.
    #[inline]
    fn compose(&mut self, sensors: &Sensors) -> NavState {
        let mut nav = self.h.update(sensors);
        let z = self.v.update(sensors);
        self.z = Some(z);
        // A step in which the zero-delay filter's reset moved D or v_D: its vertical flags are false for that step.
        let vmoved = self.v.last_reset().is_some_and(|r| r.dp[2] != 0.0 || r.dv[2] != 0.0);
        nav.position_m[2] = z.position_m[2];
        nav.velocity_ms[2] = z.velocity_ms[2];
        nav.valid.velocity_d = z.valid.velocity_d && !vmoved;
        nav.valid.position_d = z.valid.position_d && !vmoved;
        nav.sigma = match (nav.sigma, z.sigma) {
            (Some(mut s), Some(zs)) => {
                s.position_d_m = zs.position_d_m;
                s.velocity_d_ms = zs.velocity_d_ms;
                Some(s)
            }
            _ => None,
        };
        nav
    }
}

/// The `Estimator` impls are written for the two ring sizes in use, not generically: a generic impl would be
/// instantiated again in the crate that names `SteerMix<RING>`, and with it a second copy of `EskfN::update` (17 KB in
/// RAM on the board). The board's `RV` is `RING`; the host tests use 1 (`EskfZ`).
macro_rules! steer_mix_estimator {
    ($ring:expr) => {
        impl Estimator for SteerMix<'_, { $ring }> {
            fn update(&mut self, sensors: &Sensors) -> NavState {
                self.compose(sensors)
            }

            fn origin(&self) -> Option<GeoPoint> {
                self.h.origin()
            }

            fn set_armed(&mut self, armed: bool) {
                self.h.set_armed(armed);
                self.v.set_armed(armed);
            }
        }
    };
}
steer_mix_estimator!(crate::eskf::RING);
#[cfg(test)]
steer_mix_estimator!(1);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eskf::tests::{Fault, PERIOD_US, World, pressure_at};
    use crate::mahony::Mahony;
    use core::f32::consts::PI;
    use libm::{cosf, fabsf, sinf};
    use std::println;
    use std::vec::Vec;

    /// The scenario of t3, t3b, t5 (velocity part), t5d and t7 was designed at the LC76G's GNSS delay, 430 ms; they keep it whatever the default.
    const LC76G_GPS_DELAY_US: u64 = 430_000;

    #[test]
    fn the_ring_reports_the_oldest_entry_it_drops_when_full() {
        let (mut r, e) = (OutRing::new(), OutEntry::default());
        for i in 0..OUT {
            assert!(!r.push(OutEntry { t_us: i as u64, ..e }));
        }
        assert!(r.push(OutEntry { t_us: OUT as u64, ..e }) && r.front().unwrap().t_us == 1);
    }

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
        stream_yaw(t, secs, 0.4)
    }

    fn stream_yaw(t: &Tuning, secs: u64, yaw0: f64) -> Vec<Sensors> {
        let mut w = World::new(t);
        w.yaw = yaw0;
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

    /// Armed from 5 s but disarmed from `from_us` to `to_us`: G6-B resets no yaw while armed, so a test that needs a yaw reset
    /// after its compass turn disarms before the timeout runs out (the disarmed reset, the same `yaw_set` path).
    fn armed_but(s: &Sensors, from_us: u64, to_us: u64) -> bool {
        armed_at(s) && !(from_us..to_us).contains(&s.imu1.t_us)
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
            // Disarmed from 61 s to 70 s (the compass turned at 60 s): the disarmed yaw reset.
            steer.set_armed(armed_but(s, 61_000_000, 70_000_000));
            bare.set_armed(armed_but(s, 61_000_000, 70_000_000));
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
        // The output integrates the gyro minus the core's CURRENT bias: after the bias has converged it tracks the core's
        // attitude. With the biases not copied it would hold bias x T_d / gain = 0.02 x 0.43 / 0.5, 1 degree, away.
        let att = steer.tracking().att_rad;
        println!("t10: attitude tracking error at 60 s {att:.2e} rad");
        assert!(att < 1e-4, "tracking error {att}");
    }

    // ---- Stage 2: the correction ----

    fn qmul64(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
        [
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
        ]
    }

    /// Earth-frame error vector 2 vec(a x conj(b)) (small-angle rotation vector taking b to a), sign-normalised.
    fn err_vec(a: [f32; 4], b: [f32; 4]) -> [f64; 3] {
        let (a, b) = (a.map(f64::from), b.map(f64::from));
        let d = qmul64(a, [b[0], -b[1], -b[2], -b[3]]);
        let s = if d[0] < 0.0 { -2.0 } else { 2.0 };
        [s * d[1], s * d[2], s * d[3]]
    }

    /// The rotation angle between two attitudes, rad.
    fn angle(a: [f32; 4], b: [f32; 4]) -> f64 {
        let (a, b) = (a.map(f64::from), b.map(f64::from));
        let d = qmul64(a, [b[0], -b[1], -b[2], -b[3]]);
        2.0 * (d[1] * d[1] + d[2] * d[2] + d[3] * d[3]).sqrt().atan2(d[0].abs())
    }

    /// The angle between the two down-in-body vectors (the tilt difference), rad.
    fn tilt_diff(a: [f32; 4], b: [f32; 4]) -> f64 {
        let (x, y) = (crate::quat::down_in_body(a).map(f64::from), crate::quat::down_in_body(b).map(f64::from));
        let c = (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]).clamp(-1.0, 1.0);
        c.acos()
    }

    fn yaw_q(yaw: f64) -> [f32; 4] {
        [(0.5 * yaw).cos() as f32, 0.0, 0.0, (0.5 * yaw).sin() as f32]
    }

    #[derive(Default)]
    struct Stat {
        n: usize,
        s: f64,
        m: f64,
    }
    impl Stat {
        fn add(&mut self, x: f64) {
            self.n += 1;
            self.s += x * x;
            self.m = self.m.max(x);
        }
        fn rms(&self) -> f64 {
            (self.s / self.n.max(1) as f64).sqrt()
        }
    }

    /// Errors against the truth at the CURRENT time over a window: the output's and the horizon state's.
    #[derive(Default)]
    struct Errs {
        att: Stat,
        tilt: Stat,
        v: Stat,
        p: Stat,
        h_att: Stat,
        h_tilt: Stat,
        h_v: Stat,
        h_p: Stat,
    }

    fn report(name: &str, e: &Errs) {
        let d = |x: f64| x.to_degrees();
        println!(
            "{name}: n {} | output att rms/max {:.3}/{:.3} deg, tilt {:.3}/{:.3} deg, v {:.3}/{:.3} m/s, p {:.3}/{:.3} m | horizon passed through: att {:.2}/{:.2} deg, tilt {:.2}/{:.2} deg, v {:.3}/{:.3} m/s, p {:.3}/{:.3} m",
            e.att.n, d(e.att.rms()), d(e.att.m), d(e.tilt.rms()), d(e.tilt.m), e.v.rms(), e.v.m, e.p.rms(), e.p.m,
            d(e.h_att.rms()), d(e.h_att.m), d(e.h_tilt.rms()), d(e.h_tilt.m), e.h_v.rms(), e.h_v.m, e.h_p.rms(), e.h_p.m
        );
    }

    fn add_errs(e: &mut Errs, steer: &EskfSteer, nav: &NavState, q_true: [f32; 4], v_true: [f64; 3], p_true: [f64; 3]) {
        let n3 = |a: [f32; 3], b: [f64; 3]| (0..3).map(|i| (f64::from(a[i]) - b[i]).powi(2)).sum::<f64>().sqrt();
        e.att.add(angle(nav.attitude, q_true));
        e.tilt.add(tilt_diff(nav.attitude, q_true));
        e.v.add(n3(nav.velocity_ms, v_true));
        e.p.add(n3(nav.position_m, p_true));
        let (qh, vh, ph) = steer.core().nominal();
        e.h_att.add(angle(qh, q_true));
        e.h_tilt.add(tilt_diff(qh, q_true));
        e.h_v.add(n3(vh, v_true));
        e.h_p.add(n3(ph, p_true));
    }

    /// NE of the truth in the core's frame: the world's NED minus the origin's offset from the world's ORIGIN.
    fn truth_p(steer: &EskfSteer, w: &World) -> [f64; 3] {
        let off = steer.origin().map_or([0.0; 3], |o| crate::eskf::ned_from_geo(&crate::eskf::tests::ORIGIN, &o).map(f64::from));
        [w.p[0] - off[0], w.p[1] - off[1], w.p[2]]
    }

    #[test]
    fn t3_truth_the_output_tracks_the_truth_at_the_current_time_through_turns_and_accelerations() {
        let t = Tuning { nav_gps_delay_us: LC76G_GPS_DELAY_US, ..Tuning::DEFAULT };
        let mut core = Eskf::new(&t).unwrap();
        let mut steer = EskfSteer::new(&mut core, &t).unwrap();
        let mut w = World::new(&t);
        w.yaw = 0.4;
        w.v = [2.0, 0.5, 0.0];
        w.gps_vel = true;
        let (mut all, mut turn) = (Errs::default(), Errs::default());
        while w.t_us() < 70_000_000 {
            let secs = w.t_us() as f64 * 1e-6;
            w.yaw_rate = if (20.0..25.0).contains(&secs) || (50.0..53.0).contains(&secs) { 100.0_f64.to_radians() } else { 0.0 };
            w.a = if (30.0..35.0).contains(&secs) {
                [1.5, -1.0, 0.0]
            } else if (40.0..45.0).contains(&secs) {
                [-1.5, 1.0, 0.0]
            } else {
                [0.0; 3]
            };
            let s = w.step();
            steer.set_armed(s.imu1.t_us >= 5_000_000);
            let nav = steer.update(&s);
            let secs = w.t_us() as f64 * 1e-6;
            if secs >= 15.0 {
                let (qt, vt, pt) = (yaw_q(w.yaw), w.v, truth_p(&steer, &w));
                add_errs(&mut all, &steer, &nav, qt, vt, pt);
                // Mid-turn: the filter has had 1 s of the steady rate and the core's delay is in the whole error.
                if (21.0..25.0).contains(&secs) || (51.0..53.0).contains(&secs) {
                    add_errs(&mut turn, &steer, &nav, qt, vt, pt);
                }
            }
        }
        report("t3 all", &all);
        report("t3 turns at 100 deg/s", &turn);
        let order = 100.0_f64.to_radians() * 0.43;
        println!("t3: rate x delay = {:.1} deg", order.to_degrees());
        // Control: the horizon state passed through as "current" is off by about rate x delay (43 deg) in the turn.
        assert!(turn.h_att.m > 0.5 * order && turn.h_att.m < 1.5 * order, "horizon error {}", turn.h_att.m);
        // The output is two orders of magnitude better.
        assert!(turn.att.m < 0.03 * turn.h_att.m, "output attitude error {} rad", turn.att.m);
        assert!(turn.att.m < 0.1_f64.to_radians());
        // During the accelerations the horizon is off by acceleration x delay in velocity; the output is not.
        assert!(all.h_v.m > 0.5, "the manoeuvres must cost the horizon something: {}", all.h_v.m);
        assert!(all.v.m < 0.05 && all.p.m < 0.1, "v {} p {}", all.v.m, all.p.m);
        assert!(all.v.m < 0.35 * all.h_v.m && all.p.m < 0.35 * all.h_p.m);
    }

    /// A vehicle rocking in roll in place, 30 deg at 0.6 Hz (peak rate 113 deg/s) from 15 s: the World's stationary
    /// sensors with the IMU and the compass replaced by those of the rolled body.
    fn rock_step(w: &mut World) -> (Sensors, f64) {
        let (amp, f) = (30.0_f64.to_radians(), 0.6);
        let g = f64::from(fw_core::calib::GRAVITY_MSS);
        let mut s = w.step();
        let ts = w.t_us() as f64 * 1e-6;
        let (phi, rate) = if ts >= 15.0 {
            let ph = 2.0 * core::f64::consts::PI * f * (ts - 15.0);
            (amp * ph.sin(), amp * 2.0 * core::f64::consts::PI * f * ph.cos())
        } else {
            (0.0, 0.0)
        };
        let (c, sn) = (phi.cos(), phi.sin());
        s.imu1.reading.gyro_rads = [rate as f32, 0.0, 0.0];
        s.imu1.reading.accel_ms2 = [0.0, (-g * sn) as f32, (-g * c) as f32];
        if let Some(m) = s.mag.as_mut() {
            let f = m.reading.field_ut.map(f64::from);
            m.reading.field_ut = [f[0] as f32, (c * f[1] + sn * f[2]) as f32, (-sn * f[1] + c * f[2]) as f32];
        }
        (s, phi)
    }

    fn rock(w: &mut World, steer: &mut EskfSteer, secs: f64, e: &mut Errs) {
        while (w.t_us() as f64) < secs * 1e6 {
            let (s, phi) = rock_step(w);
            let ts = w.t_us() as f64 * 1e-6;
            steer.set_armed(s.imu1.t_us >= 5_000_000);
            let nav = steer.update(&s);
            if ts >= 20.0 {
                let qt = [(0.5 * phi).cos() as f32, (0.5 * phi).sin() as f32, 0.0, 0.0];
                let pt = truth_p(steer, w);
                add_errs(e, steer, &nav, qt, [0.0; 3], pt);
            }
        }
    }

    #[test]
    fn t3b_a_roll_rocking_at_113_deg_s_has_a_small_tilt_error_in_the_output_and_a_large_one_in_the_horizon() {
        let t = Tuning { nav_gps_delay_us: LC76G_GPS_DELAY_US, ..Tuning::DEFAULT };
        let mut core = Eskf::new(&t).unwrap();
        let mut steer = EskfSteer::new(&mut core, &t).unwrap();
        let mut w = World::new(&t);
        let mut e = Errs::default();
        rock(&mut w, &mut steer, 50.0, &mut e);
        report("t3b rocking", &e);
        // The horizon is one delay behind a 30 deg swing at up to 113 deg/s: tens of degrees of tilt.
        assert!(e.h_tilt.m > 15.0_f64.to_radians(), "horizon tilt error {}", e.h_tilt.m);
        assert!(e.tilt.m < 0.1 * e.h_tilt.m, "output tilt error {} against the horizon's {}", e.tilt.m, e.h_tilt.m);
        assert!(e.att.m < 5.0_f64.to_radians(), "output attitude error {}", e.att.m);
        assert_eq!(steer.skipped(), 0);
    }

    // ---- T7: the tracking law with the core frozen ----

    /// A resting vehicle whose slow sensors are withheld: after alignment the core only integrates the IMU, so its state
    /// stays where it is. 3 s of settling, then `each(wrapper, true)` (the injection), then `secs` of steps with
    /// `each(wrapper, false)` after every step.
    fn frozen(t: &Tuning, secs: f64, mut each: impl FnMut(&mut EskfSteer, bool)) {
        let mut core = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        let mut w = World::new(t);
        let step = |steer: &mut EskfSteer, w: &mut World| {
            // Turned to a yaw of 0.9 rad between 1 and 2 s (0.9 rad/s), so that an earth-frame and a body-frame rotation
            // about the same vector differ; the compass is withheld, so the core's yaw is its gyro's.
            w.yaw_rate = if (1_000_000..2_000_000).contains(&w.t_us()) { 0.9 } else { 0.0 };
            let mut s = w.step();
            (s.baro, s.mag, s.gnss) = (None, None, None);
            steer.update(&s);
        };
        while w.t_us() < 3_000_000 {
            step(&mut steer, &mut w);
        }
        assert!(steer.out.live && steer.corrected() > 100);
        each(&mut steer, true);
        for _ in 0..(secs * 1e6) as u64 / PERIOD_US {
            step(&mut steer, &mut w);
            each(&mut steer, false);
        }
    }

    /// Dominant root x of x + g exp(-x) = 0 (the loop e'(t) = -(g/D) e(t - D) in x = s D), by complex Newton.
    fn dominant_pole(g: f64) -> (f64, f64) {
        let (mut re, mut im) = (-0.3f64, 0.6f64);
        for _ in 0..100 {
            let (er, ei) = ((-re).exp() * im.cos(), -(-re).exp() * im.sin());
            let (fr, fi) = (re + g * er, im + g * ei);
            // f' = 1 - g exp(-x)
            let (dr, di) = (1.0 - g * er, -g * ei);
            let den = dr * dr + di * di;
            let (nr, ni) = ((fr * dr + fi * di) / den, (fi * dr - fr * di) / den);
            re -= nr;
            im -= ni;
        }
        (re, im)
    }

    /// The same loop in continuous time, e(0) = 1, entries older than the injection 0, by small Euler steps: the first
    /// time e <= 1/e, and the series at 0.1 ms.
    fn dde(k: f64, d: f64, secs: f64) -> (f64, Vec<f64>) {
        let h = 1e-4;
        let (nd, n) = ((d / h).round() as usize, (secs / h) as usize);
        let mut e = std::vec![0.0; n + 1];
        e[0] = 1.0;
        let mut t63 = f64::NAN;
        for i in 0..n {
            let delayed = if i >= nd { e[i - nd] } else { 0.0 };
            e[i + 1] = e[i] - h * k * delayed;
            if t63.is_nan() && e[i + 1] <= (-1.0f64).exp() {
                t63 = (i + 1) as f64 * h;
            }
        }
        (t63, e)
    }

    /// The attitude offset of `theta` rad about north, put into the output's current state at the end of settling
    /// (not into the ring: the entries before the injection are right, the later ones carry it). Returns the signed
    /// error along north after every step, the model's value (the f64 law driven by the wrapper's push and correction
    /// events) and the step times.
    fn att_series(t: &Tuning, theta: f64, secs: f64) -> (Vec<f64>, Vec<f64>) {
        use std::collections::VecDeque;
        let (mut meas, mut model) = (Vec::new(), Vec::new());
        let d = t.nav_gps_delay_us as f64 * 1e-6;
        let dt = PERIOD_US as f64 * 1e-6;
        let (mut m_eps, mut rate, mut hist, mut counts) = (0.0f64, 0.0f64, VecDeque::<f64>::new(), (0, 0));
        frozen(t, secs, |st, inject| {
            if inject {
                let inj = [(0.5 * theta).cos() as f32, (0.5 * theta).sin() as f32, 0.0, 0.0];
                st.out.nom.q = normalise(mul(inj, st.out.nom.q));
                let h = st.core().nominal().0;
                m_eps = err_vec(st.out.nom.q, h)[0];
                hist.extend(std::iter::repeat_n(0.0, st.out.buf.len));
                counts = (st.pushed(), st.corrected());
                return;
            }
            // The wrapper's order in a step: correction (a new rate), slew, push.
            if st.corrected() > counts.1 {
                rate = (st.out.g.att_gain as f64 / d) * hist.pop_front().unwrap();
                counts.1 = st.corrected();
            }
            m_eps -= dt * rate;
            if st.pushed() > counts.0 {
                hist.push_back(m_eps);
                counts.0 = st.pushed();
            }
            let h = st.core().nominal().0;
            meas.push(err_vec(st.out.nom.q, h)[0]);
            model.push(m_eps);
        });
        (meas, model)
    }

    #[test]
    fn t7_attitude_offset_decays_by_the_delayed_first_order_loop_and_never_with_gain_zero() {
        // Derivation. The output integrates the same gyro as the core, so the earth-frame error eps between them changes
        // only by the slew: eps'(t) = -rate_corr(t). A correction measures the error of the ring entry at the core's time
        // t - D (D = T_d = now - t_e = nav_gps_delay_us once the delay is above the step) and sets rate_corr = g eps(t - D) / D,
        // g = nav_out_att_gain. Entries older than the injection hold no error. So eps'(t) = -k eps(t - D), k = g / D, with
        // eps = theta for 0 <= t < D (nothing is measured yet), a delayed first-order loop. Its roots: x = s D with
        // x + g exp(-x) = 0; for g = 0.5 they are complex (monotone only for g <= 1/e), so the decay is a damped
        // oscillation: the envelope falls monotonically, with a small undershoot.
        let t = Tuning { nav_gps_delay_us: LC76G_GPS_DELAY_US, ..Tuning::DEFAULT };
        let theta = 5.0_f64.to_radians();
        let secs = 12.0;
        let d = t.nav_gps_delay_us as f64 * 1e-6;
        let (meas, model) = att_series(&t, theta, secs);
        let dt = PERIOD_US as f64 * 1e-6;
        // 1. The f64 law, driven by the wrapper's own events, matches the measured error at every step.
        let worst = meas.iter().zip(&model).fold(0.0f64, |a, (m, e)| a.max((m - e).abs()));
        println!("t7 attitude: worst |measured - model| {:.3e} rad ({:.2e} of theta)", worst, worst / theta);
        assert!(worst < 2e-3 * theta, "worst {worst}");
        // 2. The continuous loop gives the 63 % time and the dominant pole.
        let th0 = meas[0];
        let (t63_cont, _) = dde(t.nav_out_att_gain as f64 / d, d, secs);
        let t63 = (meas.iter().position(|x| *x <= th0 * (-1.0f64).exp()).unwrap() + 1) as f64 * dt;
        let (xr, xi) = dominant_pole(t.nav_out_att_gain as f64);
        println!("t7 attitude: 63 % at {t63:.3} s (continuous loop {t63_cont:.3} s); dominant pole x = {xr:.4} {xi:+.4}i, sigma {:.3} /s, damping {:.2}", xr / d, -xr / (xr * xr + xi * xi).sqrt());
        // Tolerance 3 %: the correction is held for nav_predict_us and sampled on the 1.25 ms grid: about 5 ms of extra
        // delay on a 0.55 s rise, 1 %.
        assert!(fabsf(((t63 - t63_cont) / t63_cont) as f32) < 0.03);
        // 3. Envelope: successive extrema of the signed error shrink, the error never exceeds theta, undershoot below 10 %.
        let mut ext: Vec<f64> = Vec::new();
        for i in 1..meas.len() - 1 {
            if (meas[i] - meas[i - 1]) * (meas[i + 1] - meas[i]) < 0.0 && fabsf((meas[i] / th0) as f32) > 1e-3 {
                ext.push(meas[i]);
            }
        }
        println!("t7 attitude: extrema / theta {:?}", ext.iter().map(|x| (x / th0 * 1e4).round() / 1e4).collect::<Vec<_>>());
        assert!(ext.len() >= 2);
        assert!(ext.windows(2).all(|w| fabsf(w[1] as f32) < fabsf(w[0] as f32)), "{ext:?}");
        assert!(meas.iter().all(|x| *x <= th0 * 1.001 && *x >= -0.1 * th0));
        // The ratio of successive extrema is the dominant pole's: -exp(sigma * half period), half period = pi D / Im(x).
        let ratio = ext[1] / ext[0];
        let want = (xr / d * (core::f64::consts::PI / (xi / d))).exp();
        println!("t7 attitude: successive-extrema ratio {ratio:.4}, pole prediction {:.4}", -want);
        assert!(fabsf(((-ratio - want) / want) as f32) < 0.15);
        assert!(fabsf((*meas.last().unwrap() / th0) as f32) < 1e-3);
        // Control: gain 0 never converges.
        let zero = Tuning { nav_out_att_gain: 0.0, ..t };
        let (m0, _) = att_series(&zero, theta, secs);
        println!("t7 attitude control (gain 0): error after {secs} s is {:.4} of theta", m0.last().unwrap() / m0[0]);
        assert!(m0.iter().all(|x| fabsf((x / m0[0] - 1.0) as f32) < 1e-3));
    }

    /// An offset of 1 (m/s or m, north) in the output's velocity (`pos` false) or position, put into the current state
    /// AND every ring entry (a constant error in the history, as an accumulated drift is). Returns, after every
    /// correction, the measured offset (output minus core) and the model's: each correction measures e = -E, then
    /// I += e, dE = e g + I g^2 k_i with g = nav_predict_us / tau.
    fn pi_series(t: &Tuning, pos: bool, secs: f64) -> (Vec<f64>, Vec<f64>) {
        let (mut meas, mut model) = (Vec::new(), Vec::new());
        let tau = if pos { t.nav_out_tau_pos_s } else { t.nav_out_tau_vel_s } as f64;
        let g = t.nav_predict_us as f64 * 1e-6 / tau;
        let (mut e_model, mut integ, mut seen) = (1.0f64, 0.0f64, 0);
        frozen(t, secs, |st, inject| {
            let core = st.core().nominal();
            if inject {
                st.out.nom.v[0] += if pos { 0.0 } else { 1.0 };
                st.out.nom.p[0] += if pos { 1.0 } else { 0.0 };
                st.out.buf.for_each(|x| {
                    x.v[0] += if pos { 0.0 } else { 1.0 };
                    x.p[0] += if pos { 1.0 } else { 0.0 };
                });
                seen = st.corrected();
                return;
            }
            if st.corrected() > seen {
                assert_eq!(st.corrected(), seen + 1, "one correction per step");
                seen = st.corrected();
                let e = -e_model;
                integ += e;
                e_model += e * g + integ * g * g * f64::from(t.nav_out_int_gain);
                meas.push(f64::from(if pos { st.out.nom.p[0] - core.2[0] } else { st.out.nom.v[0] - core.1[0] }));
                model.push(e_model);
            }
        });
        (meas, model)
    }

    /// The offset's continuous-time law: e'' + e'/tau + k_i e / tau^2 = 0, e(0) = 1, e'(0) = -1/tau (the proportional
    /// term alone starts at the slope -1/tau). Returns the first time e <= 1/e.
    fn pi_t63(tau: f64, ki: f64) -> f64 {
        let disc = (1.0 / (tau * tau) - 4.0 * ki / (tau * tau)).sqrt();
        let (s1, s2) = ((-1.0 / tau + disc) / 2.0, (-1.0 / tau - disc) / 2.0);
        let a = (s2 + 1.0 / tau) / (s2 - s1);
        let e = |t: f64| a * (s1 * t).exp() + (1.0 - a) * (s2 * t).exp();
        (0..100_000).map(|i| i as f64 * 1e-4).find(|t| e(*t) <= (-1.0f64).exp()).unwrap()
    }

    #[test]
    fn t7_velocity_and_position_offsets_follow_the_tau_and_integral_gain_law_and_a_slow_tau_does_not_converge() {
        let t = Tuning::DEFAULT;
        let secs = 6.0;
        for pos in [false, true] {
            let name = if pos { "position 1 m" } else { "velocity 1 m/s" };
            let (meas, model) = pi_series(&t, pos, secs);
            let worst = meas.iter().zip(&model).fold(0.0f64, |a, (m, e)| a.max((m - e).abs()));
            let tau = if pos { t.nav_out_tau_pos_s } else { t.nav_out_tau_vel_s } as f64;
            // The offset measured after n corrections: the first is after the first one.
            let dtc = t.nav_predict_us as f64 * 1e-6;
            let t63 = (meas.iter().position(|x| *x <= (-1.0f64).exp()).unwrap() + 1) as f64 * dtc;
            let t63_cont = pi_t63(tau, f64::from(t.nav_out_int_gain));
            let min = meas.iter().cloned().fold(f64::MAX, f64::min);
            println!("t7 {name}: worst |measured - recursion| {worst:.2e}; 63 % at {t63:.3} s (continuous law {t63_cont:.3} s); undershoot {min:.4}; after {secs} s {:.4}", meas.last().unwrap());
            assert!(worst < 1e-3, "{name}: {worst}");
            // The recursion is forward Euler with step nav_predict_us / tau = 4 % of the time constant, so the time
            // differs from the continuous law by that order: tolerance 8 %.
            assert!(fabsf(((t63 - t63_cont) / t63_cont) as f32) < 0.08, "{name}: {t63} against {t63_cont}");
            // The integral term makes the tail slow and overshoot past zero (overdamped pair, roots -0.45 and -3.55 /s).
            assert!(min < 0.0 && min > -0.2 && fabsf(*meas.last().unwrap() as f32) < 0.05);
            // Control: tau at its upper clamp and no integral term: after 2 s the offset has not fallen to 1/e.
            let slow = if pos {
                Tuning { nav_out_tau_pos_s: TAU_MAX_S, nav_out_int_gain: 0.0, ..t }
            } else {
                Tuning { nav_out_tau_vel_s: TAU_MAX_S, nav_out_int_gain: 0.0, ..t }
            };
            let (m, _) = pi_series(&slow, pos, 2.0);
            println!("t7 {name} control (tau 10 s, no integral): offset after 2 s {:.3}", m.last().unwrap());
            assert!(*m.last().unwrap() > 0.75);
        }
    }

    // ---- T12: exact time ----

    /// The World's samples re-timed: spacing varying +-30 % of 1250 us (a fixed pseudo-random sequence), every 500th
    /// sample double-spaced, a NaN gyro at two samples.
    fn jittered(t: &Tuning, secs: u64) -> Vec<Sensors> {
        let mut w = World::new(t);
        w.yaw = 0.4;
        w.v = [2.0, 0.5, 0.0];
        w.gps_vel = true;
        let (mut out, mut now, mut lcg) = (Vec::new(), 0u64, 12345u32);
        while w.t_us() < secs * 1_000_000 {
            let mut s = w.step();
            lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let u = f64::from(lcg >> 8) / f64::from(1u32 << 24) * 2.0 - 1.0;
            let gap = if out.len() % 500 == 499 { 2.0 * 1250.0 } else { 1250.0 * (1.0 + 0.3 * u) };
            now += gap.round() as u64;
            s.imu1.t_us = now;
            if out.len() == 20_000 || out.len() == 30_001 {
                s.imu1.reading.gyro_rads[0] = f32::NAN;
            }
            out.push(s);
        }
        out
    }

    fn jitter_run(t: &Tuning, control: bool) -> (u32, u32, u32, usize, usize) {
        let st = jittered(t, 60);
        let mut core = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        MATCH_HORIZON.set(control);
        let (mut exact, mut snaps) = (0usize, 0usize);
        for s in &st {
            steer.set_armed(s.imu1.t_us >= 5_000_000);
            let front = steer.out.buf.front();
            steer.update(s);
            if let Some((t_e, ..)) = steer.core().snapshot() {
                snaps += 1;
                // The snapshot is of the entry that was the oldest: its time is exactly that entry's.
                exact += usize::from(Some(t_e) == front.map(|e| e.t_us));
            }
        }
        MATCH_HORIZON.set(false);
        (steer.skipped(), steer.corrected(), steer.pushed(), exact, snaps)
    }

    #[test]
    fn t12_exact_time_every_correction_uses_the_snapshot_of_the_popped_entry_on_a_jittered_stream() {
        let t = Tuning::DEFAULT;
        let (skipped, corrected, pushed, exact, snaps) = jitter_run(&t, false);
        println!("t12: pushed {pushed}, corrected {corrected}, skipped {skipped}; snapshots {snaps}, equal to the popped entry's time {exact}");
        assert_eq!(skipped, 0);
        assert!(corrected > 4_000 && exact == snaps && snaps as u32 == corrected);
        // Control: matching on the final horizon time loses entries.
        let (cskipped, ccorrected, _, _, _) = jitter_run(&t, true);
        println!("t12 control (match on the horizon time): skipped {cskipped}, corrected {ccorrected}");
        assert!(cskipped > 100);
    }

    // ---- Stage 3: resets and flags ----

    /// One reset step as watched from outside.
    #[derive(Clone, Copy, Debug)]
    struct Rec {
        k: usize,
        src: u8,
        t_us: u64,
        tilt_step: f64,
        tilt_bound: f64,
        yaw_err: f64,
        yaw_bound: f64,
        dpsi: f64,
    }

    fn yaw64(q: [f32; 4]) -> f64 {
        let r = rotation(&q);
        f64::from(r[1][0]).atan2(f64::from(r[0][0]))
    }

    fn wrap64(a: f64) -> f64 {
        let t = 2.0 * core::f64::consts::PI;
        a - t * (a / t).round()
    }

    /// `update` that records the step if it applied a reset delta to a live output.
    fn watch(steer: &mut EskfSteer, s: &Sensors, k: usize, recs: &mut Vec<Rec>) -> NavState {
        let (live, q0, t0) = (steer.out.live, steer.out.nom.q, steer.out.t_us);
        let nav = steer.update(s);
        if let (true, true, Some(r)) = (live, steer.out.live, steer.core().last_reset()) {
            let d = |q: [f32; 4]| crate::quat::down_in_body(q).map(f64::from);
            let (d0, d1) = (d(q0), d(steer.out.nom.q));
            let tilt_step = (0..3).map(|i| (d1[i] - d0[i]).powi(2)).sum::<f64>().sqrt();
            let dt = s.imu1.t_us.saturating_sub(t0).min(DT_MAX_US) as f64 * 1e-6;
            let n3 = |a: [f32; 3]| a.iter().map(|x| f64::from(*x).powi(2)).sum::<f64>().sqrt();
            let bound = (n3(s.imu1.reading.gyro_rads) + n3(steer.core().biases().0) + n3(steer.out.rate_corr)) * dt + 4e-6;
            let dpsi = 2.0 * f64::from(r.dq[3]).atan2(f64::from(r.dq[0]));
            let yaw_err = wrap64(yaw64(steer.out.nom.q) - yaw64(q0) - dpsi);
            let src = steer.core().events().find(|e| [6, 8, 9].contains(&e.src) && e.ok == 1).map_or(255, |e| e.src);
            recs.push(Rec { k, src, t_us: s.imu1.t_us, tilt_step, tilt_bound: bound, yaw_err, yaw_bound: bound + 1e-5, dpsi });
        }
        nav
    }

    fn watch_all(t: &Tuning, st: &[Sensors], armed: impl Fn(&Sensors) -> bool) -> Vec<Rec> {
        let mut core = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        let mut recs = Vec::new();
        for (k, s) in st.iter().enumerate() {
            steer.set_armed(armed(s));
            watch(&mut steer, s, k, &mut recs);
        }
        recs
    }

    fn rocking_stream(t: &Tuning, secs: f64) -> Vec<Sensors> {
        let mut w = World::new(t);
        let mut out = Vec::new();
        while (w.t_us() as f64) < secs * 1e6 {
            if w.t_us() >= 20_000_000 {
                w.mag_rot = 90.0_f64.to_radians();
            }
            out.push(rock_step(&mut w).0);
        }
        out
    }

    fn worst_ratio(recs: &[Rec]) -> (f64, f64) {
        recs.iter().fold((0.0, 0.0), |(a, b): (f64, f64), r| (a.max(r.tilt_step / r.tilt_bound), b.max(fabsf((r.yaw_err / r.yaw_bound) as f32) as f64)))
    }

    #[test]
    fn t4_no_tilt_step_across_yaw_alignment_resets_src9_c3_and_height_and_the_yaw_moves_by_the_reset() {
        let t = Tuning::DEFAULT;
        // Level, yawed 90 deg from the start: the yaw alignment by about 90 deg; the GPS reset at arming (5 s), the C3
        // reset (north fault from 20 s), a yaw reset (compass turned 90 deg from 60 s, disarmed from 61 s to 70 s: G6-B resets
        // no yaw while armed), the height reset (85 s).
        let a = watch_all(&t, &stream_yaw(&t, 100, 90.0_f64.to_radians()), |s| armed_but(s, 61_000_000, 70_000_000));
        // Disarmed throughout, compass turned 90 deg from 30 s: the disarmed yaw reset.
        let mut w = World::new(&t);
        w.yaw = 0.4;
        let b_stream: Vec<Sensors> = (0..45_000_000 / PERIOD_US)
            .map(|_| {
                w.mag_rot = if w.t_us() >= 30_000_000 { 90.0_f64.to_radians() } else { 0.0 };
                w.step()
            })
            .collect();
        let b = watch_all(&t, &b_stream, |_| false);
        for (name, recs) in [("A", &a), ("B", &b)] {
            for r in recs.iter() {
                println!("t4 {name}: step {} t {:.3} s src {} dpsi {:+.2} deg: tilt step {:.2e} (bound {:.2e}), yaw error {:+.2e} (bound {:.2e})", r.k, r.t_us as f64 * 1e-6, r.src, r.dpsi.to_degrees(), r.tilt_step, r.tilt_bound, r.yaw_err, r.yaw_bound);
            }
        }
        let count = |recs: &[Rec], src: u8| recs.iter().filter(|r| r.src == src).count();
        assert!(a.iter().any(|r| r.src == 8 && fabsf(r.dpsi.to_degrees() as f32) > 80.0), "a yaw alignment by about 90 deg");
        assert!(count(&a, 9) >= 2 && count(&a, 6) >= 1 && count(&a, 8) >= 2, "A: kinds {} {} {}", count(&a, 9), count(&a, 6), count(&a, 8));
        assert!(count(&b, 8) >= 2, "B: the alignment and the disarmed yaw reset");
        for r in a.iter().chain(&b) {
            assert!(r.tilt_step <= r.tilt_bound, "{r:?}");
            assert!(fabsf(r.yaw_err as f32) as f64 <= r.yaw_bound, "{r:?}");
        }
        // Rolling 30 deg at up to 113 deg/s with the compass turned at 20 s, disarmed from 21 s to 28 s (G6-B resets no yaw while
        // armed): a disarmed yaw reset while tilted. Here the left and right products differ, so this is where the control can
        // fail the test.
        let c_stream = rocking_stream(&t, 40.0);
        let c = watch_all(&t, &c_stream, |s| armed_but(s, 21_000_000, 28_000_000));
        let (tc, yc) = worst_ratio(&c);
        println!("t4 C (rocking): {} reset steps, worst tilt step / bound {tc:.3}, worst yaw error / bound {yc:.3}", c.len());
        assert!(c.iter().any(|r| r.src == 8 && fabsf(r.dpsi.to_degrees() as f32) > 45.0 && r.t_us > 20_000_000), "the yaw reset while rocking");
        assert!(tc <= 1.0 && yc <= 1.0);
        // Control: the delta on the body (right) side.
        RESET_BODY_SIDE.set(true);
        let c2 = watch_all(&t, &c_stream, |s| armed_but(s, 21_000_000, 28_000_000));
        RESET_BODY_SIDE.set(false);
        let (tc2, _) = worst_ratio(&c2);
        println!("t4 control (delta on the body side): worst tilt step / bound {tc2:.1}");
        assert!(tc2 > 20.0, "the control must fail the test: {tc2}");
    }

    /// The first step after `k0` in which `tracking()` changed, and the largest velocity and position error over the
    /// next `n` corrections.
    fn errors_after(t: &Tuning, st: &[Sensors], armed: impl Fn(&Sensors) -> bool, k0: usize, n: usize) -> (f32, f32, usize) {
        let mut core = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        let (mut ev, mut ep, mut seen, mut last) = (0.0f32, 0.0f32, 0usize, 0u32);
        for (k, s) in st.iter().enumerate() {
            steer.set_armed(armed(s));
            steer.update(s);
            if k == k0 {
                last = steer.corrected();
            }
            if k > k0 && steer.corrected() > last && seen < n {
                last = steer.corrected();
                seen += 1;
                ev = ev.max(steer.tracking().vel_ms);
                ep = ep.max(steer.tracking().pos_m);
            }
        }
        (ev, ep, seen)
    }

    /// A vehicle at rest, armed from 5 s; with `fault` the GNSS north is 3 m (10 sigma) off from 20 s (the C3 reset
    /// about 5 s later), with `baro_off` the barometer reads 40 m too high from 20 s (the height reset 5 s later).
    fn rest_stream(t: &Tuning, fault: bool, baro_off: bool) -> Vec<Sensors> {
        let mut w = World::new(t);
        if fault {
            w.fault = Fault::North(20_000_000, 3.0);
        }
        (0..32_000_000 / PERIOD_US)
            .map(|_| {
                let mut s = w.step();
                if let Some(b) = s.baro.as_mut().filter(|_| baro_off && w.t_us() >= 20_000_000) {
                    b.reading.pressure_pa = pressure_at(40.0 - w.p[2]);
                }
                s
            })
            .collect()
    }

    /// The step and delta of the first reset after 20 s in which `moved` is true.
    fn reset_after_20s(t: &Tuning, st: &[Sensors], moved: impl Fn(&crate::eskf::Reset) -> bool) -> (usize, crate::eskf::Reset) {
        let mut core = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        for (k, s) in st.iter().enumerate() {
            steer.set_armed(armed_at(s));
            steer.update(s);
            if let Some(r) = steer.core().last_reset().filter(|r| s.imu1.t_us > 20_000_000 && moved(r)) {
                return (k, r);
            }
        }
        panic!("no such reset");
    }

    #[test]
    fn t5_reset_exactness_a_reset_leaves_no_tracking_error_and_a_ring_without_the_delta_shows_the_delta() {
        // At rest the core's velocity and the receiver's agree, so the C3 reset moves the position only; the height
        // reset (armed) moves D only. Reset deltas with a velocity part are not exact: see the module's notes.
        let t = Tuning::DEFAULT;
        let armed = |s: &Sensors| s.imu1.t_us >= 5_000_000;
        for (name, fault, baro_off, ne) in [("C3 (position N/E)", true, false, true), ("height reset (D)", false, true, false)] {
            let st = rest_stream(&t, fault, baro_off);
            let twin = rest_stream(&t, false, false);
            let (k0, r) = reset_after_20s(&t, &st, |r| if ne { r.dp[0] != 0.0 } else { r.dp[2] != 0.0 });
            let delta = if ne { norm3(&r.dp) } else { fabsf(r.dp[2]) };
            let dv = norm3(&r.dv);
            let (ev_twin, ep_twin, n0) = errors_after(&t, &twin, armed, k0, 10);
            let (ev, ep, n1) = errors_after(&t, &st, armed, k0, 10);
            // The window is the next 10 corrections (100 ms): the next GNSS epoch, whose fusion would inject a new, legitimate
            // difference (the covariance was just reset, the gain is large), comes 200 ms after the reset.
            // Round-off: the twin's own error (small fusion injections) plus 16 eps of the larger of the values (about
            // the delta) and 1 on each of the p and v sums.
            let tol = 16.0 * f32::EPSILON * delta.max(1.0);
            println!("t5 {name}: reset at step {k0}, delta {delta:.3} m, dv {dv:.2e}; next {n1} corrections: e_v {ev:.3e} (twin {ev_twin:.3e}), e_p {ep:.3e} (twin {ep_twin:.3e}); tolerance {tol:.2e} over the twin");
            assert!(n0 == 10 && n1 == 10 && delta > 2.0 && dv < 1e-3);
            assert!(ev <= ev_twin + tol && ep <= ep_twin + tol, "{name}");
            // Control: the delta not applied to the ring leaves an error equal to the delta.
            RESET_SKIP_RING.set(true);
            let (_, epc, _) = errors_after(&t, &st, armed, k0, 10);
            RESET_SKIP_RING.set(false);
            println!("t5 {name} control (no delta on the ring): e_p {epc:.3} against the delta {delta:.3}");
            assert!(fabsf(epc - delta) < 0.05 * delta && epc > 1000.0 * tol);
        }
    }

    /// The GNSS reset at arming (`arm_from_us`) on the moving stream: its step and delta, the no-aid scalars the core
    /// dropped (its `noaid_dropped`) and fused in the 90 steps after it, and the largest e_v and e_p of the next 10
    /// corrections.
    fn vel_reset_case(t: &Tuning, st: &[Sensors], arm_from_us: u64) -> (usize, crate::eskf::Reset, u32, u32, f32, f32, u32) {
        let mut core = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        let (mut at, mut noaid0, mut noaid1) = (None, 0, 0);
        let (mut ev, mut ep, mut seen, mut last) = (0.0f32, 0.0f32, 0u32, 0u32);
        for (k, s) in st.iter().enumerate() {
            steer.set_armed(s.imu1.t_us >= arm_from_us);
            steer.update(s);
            if at.is_none() && steer.core().stats().gps_resets == 1 {
                at = Some((k, steer.core().last_reset().unwrap()));
                noaid0 = steer.core().stats().noaid;
                last = steer.corrected();
            }
            if let Some((k0, _)) = at {
                if k > k0 && k <= k0 + 90 {
                    noaid1 = steer.core().stats().noaid;
                }
                if k > k0 && steer.corrected() > last && seen < 10 {
                    last = steer.corrected();
                    seen += 1;
                    ev = ev.max(steer.tracking().vel_ms);
                    ep = ep.max(steer.tracking().pos_m);
                }
            }
        }
        let (k0, r) = at.expect("a GNSS reset");
        (k0, r, noaid1 - noaid0, steer.core().stats().noaid_dropped, ev, ep, steer.reset_older())
    }

    #[test]
    fn t5_reset_exactness_with_a_velocity_part_the_position_follows_dv_times_the_time_since_the_reset() {
        // P-G2b/c. A moving vehicle (2, 0.5 m/s), the default stream: the GNSS reset at arming sets the velocity too,
        // dv = (2, 0.5). The core's position after the reset is then dp + dv (t - t_r) from the old one, so the ring
        // entries (and the current state) must be moved by that, or the next corrections see about |dv| x the entry's age.
        // On this stream a barometer set's no-aid N stage is queued when the reset runs; the core drops it (P-G2c), where
        // it used to fuse it toward the old hold position (-9 m, K 9e-4: p_N moved by 8 mm).
        let t = Tuning { nav_gps_delay_us: LC76G_GPS_DELAY_US, ..Tuning::DEFAULT };
        let st = stream(&t, 12);
        let (k0, r, fused_after, dropped, ev, ep, older) = vel_reset_case(&t, &st, 5_000_000);
        let (ev_twin, ep_twin, _) = errors_after(&t, &st, |_| false, k0, 10);
        let (dv, dp) = (norm3(&r.dv), norm3(&r.dp));
        // Round-off: positions here are up to about 30 m, dv (t - t_r) up to 1 m: 16 eps of the larger of 30 and the delta.
        let tol = 16.0 * f32::EPSILON * dp.max(30.0);
        println!("t5 GNSS reset with velocity: step {k0}, dv {dv:.3} m/s, dp {dp:.3} m; no-aid scalars dropped {dropped}, fused in the next 90 steps {fused_after}; next 10 corrections: e_v {ev:.3e} (twin {ev_twin:.3e}), e_p {ep:.3e} (twin {ep_twin:.3e}); tolerance {tol:.2e} over the twin; ring entries older than the reset {older}");
        assert!(dv > 1.5 && older == 0 && dropped == 1 && fused_after == 0);
        assert!(ev <= ev_twin + tol && ep <= ep_twin + tol);
        // Control: the old rule, without the velocity step's effect on the position: about |dv| times the entry's age.
        RESET_NO_DV_TERM.set(true);
        let (_, _, _, _, evo, epo, _) = vel_reset_case(&t, &st, 5_000_000);
        RESET_NO_DV_TERM.set(false);
        println!("t5 GNSS reset with velocity, control (old rule): e_v {evo:.3e}, e_p {epo:.3} m (|dv| x D = {:.3} m)", dv * t.nav_gps_delay_us as f32 * 1e-6);
        assert!(epo > 0.1 && epo > 1000.0 * tol && evo <= ev_twin + tol);
    }

    /// The output (position, velocity) after every step of `st`, with the step and delta of the first GNSS reset.
    fn outputs(t: &Tuning, st: &[Sensors], arm_from_us: u64) -> (Vec<([f32; 3], [f32; 3])>, Option<(usize, crate::eskf::Reset, u64, u64)>) {
        let mut core = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        let (mut out, mut at) = (Vec::new(), None);
        for (k, s) in st.iter().enumerate() {
            steer.set_armed(s.imu1.t_us >= arm_from_us);
            let t_prev = steer.out.t_us;
            let n = steer.update(s);
            out.push((n.position_m, n.velocity_ms));
            if at.is_none() && steer.core().stats().gps_resets == 1 {
                at = Some((k, steer.core().last_reset().unwrap(), steer.core().horizon_us().unwrap(), t_prev));
            }
        }
        (out, at)
    }

    #[test]
    fn t5c_the_output_steps_by_the_exact_delta_at_the_reset_step_and_follows_the_twin_plus_the_delta_trajectory() {
        // P-G2c. In the step of a GNSS reset with a velocity part the output's current position moves by the ordinary
        // motion of the step plus dp + dv (t_prev - t_r) (the state's own time to the core's reset time) and the velocity by
        // dv; afterwards it equals the run without the reset (disarmed, same samples) plus dp + dv (t - t_r), the core's
        // own trajectory shift, until the next epoch's fusion.
        let t = Tuning::DEFAULT;
        let st = stream(&t, 12);
        let (a, at) = outputs(&t, &st, 5_000_000);
        let (twin, _) = outputs(&t, &st, u64::MAX);
        let (k0, r, t_r, t_prev) = at.expect("a GNSS reset");
        assert!(norm3(&r.dv) > 1.5);
        let now = st[k0].imu1.t_us;
        let dt = (now - t_prev) as f32 * 1e-6;
        let (p0, v0) = a[k0 - 1];
        let before = (t_prev as i64 - t_r as i64) as f32 * 1e-6;
        // Expected: the delta on the state at its own time, then the step's trapezoid with the new velocity (the
        // acceleration of the step, about 1e-6 m/s^2 here, is far below the tolerance).
        let step_p: [f32; 3] = core::array::from_fn(|i| r.dp[i] + r.dv[i] * before + (v0[i] + r.dv[i]) * dt);
        let step_v: [f32; 3] = core::array::from_fn(|i| r.dv[i]);
        let tol_p = 16.0 * f32::EPSILON * 30.0;
        let tol_v = 16.0 * f32::EPSILON * 3.0;
        let (mut ep, mut ev) = (0.0f32, 0.0f32);
        for i in 0..3 {
            ep = ep.max(fabsf(a[k0].0[i] - p0[i] - step_p[i]));
            ev = ev.max(fabsf(a[k0].1[i] - v0[i] - step_v[i]));
        }
        println!("t5c: step {k0}: position step {:?} expected {:?} (worst difference {ep:.2e}, tolerance {tol_p:.2e}); velocity worst difference {ev:.2e} (tolerance {tol_v:.2e})", [0, 1, 2].map(|i| a[k0].0[i] - p0[i]), step_p);
        assert!(ep <= tol_p && ev <= tol_v);
        // The trajectory over the next 150 steps (before the next epoch, 160 steps later): the run without the reset
        // plus the core's shift. Round-off 64 eps of 30 m on each of two outputs.
        let tol_w = 64.0 * f32::EPSILON * 30.0;
        let mut worst = (0.0f32, 0.0f32);
        for k in k0..k0 + 150 {
            let age = (st[k].imu1.t_us as i64 - t_r as i64) as f32 * 1e-6;
            for i in 0..3 {
                let want_p = twin[k].0[i] + r.dp[i] + r.dv[i] * age;
                let want_v = twin[k].1[i] + r.dv[i];
                worst.0 = worst.0.max(fabsf(a[k].0[i] - want_p));
                worst.1 = worst.1.max(fabsf(a[k].1[i] - want_v));
            }
        }
        println!("t5c: next 150 steps: worst |output - (twin + delta trajectory)| position {:.2e} m (tolerance {tol_w:.2e}), velocity {:.2e} m/s", worst.0, worst.1);
        assert!(worst.0 <= tol_w && worst.1 <= tol_w);
    }

    /// The earth-frame attitude error vector of every correction in steps `from..to` (`step`, the vector the correction
    /// measured: core snapshot against the popped entry), with a tilt offset of `inject` rad about north put into the
    /// output at step `at` (0: none).
    fn att_errors(t: &Tuning, st: &[Sensors], inject: f32, at: usize, from: usize, to: usize) -> (Vec<(usize, [f64; 3])>, Option<crate::eskf::Reset>) {
        let mut core = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        let (mut v, mut reset) = (Vec::new(), None);
        for (k, s) in st.iter().enumerate().take(to) {
            steer.set_armed(armed_but(s, 21_000_000, 28_000_000));
            if k == at && inject != 0.0 {
                steer.inject_tilt(inject);
            }
            let front = steer.out.buf.front();
            steer.update(s);
            if let (Some(sn), Some(f)) = (steer.core().snapshot(), front) {
                if k >= from {
                    v.push((k, err_vec(sn.1, f.q)));
                }
            }
            if k >= from && steer.core().stats().yaw_resets > 0 && reset.is_none() {
                reset = steer.core().last_reset();
            }
        }
        (v, reset)
    }

    #[test]
    fn t5d_the_rate_correction_is_rotated_by_a_yaw_reset_and_the_tracking_continues_on_the_new_axis() {
        // P-G2c. The rocker world, the compass turned 90 deg from 20 s, disarmed from 21 s to 28 s (G6-B resets no yaw while armed): the disarmed yaw reset about 5 s later. A 0.05 rad
        // tilt offset about north is put into the output 800 steps (1 s) before the reset step (the correction sees it one delay,
        // 344 steps, later; at the reset it has been correcting for 456 steps: the earth-frame error vector is about
        // north, the rate correction too). The reset rotates both about down by the reset yaw; the
        // correction measured from the first entry made after the reset (one delay later) is the old one's, rotated and
        // reduced by the slews in between (along the vector), not turned. With the rate correction left unrotated the
        // slew of the 10 ms up to the next correction is about north on an error about east: the vector's direction
        // moves by g x 10 ms / T_d = 0.66 deg and stays moved.
        let t = Tuning { nav_gps_delay_us: LC76G_GPS_DELAY_US, ..Tuning::DEFAULT };
        let st = rocking_stream(&t, 40.0);
        let recs = watch_all(&t, &st, |s| armed_but(s, 21_000_000, 28_000_000));
        let rr = recs.iter().find(|r| r.src == 8 && r.t_us > 20_000_000).expect("a yaw reset while rocking");
        let k_r = rr.k;
        let theta = 0.05f32;
        let (x, reset) = att_errors(&t, &st, theta, k_r - 800, k_r - 20, k_r + 400);
        let (y, _) = att_errors(&t, &st, 0.0, 0, k_r - 20, k_r + 400);
        let r = reset.expect("the reset");
        // The offset's effect at each correction: X minus Y.
        let d: Vec<(usize, [f64; 3])> = x.iter().zip(&y).map(|(a, b)| (a.0, [a.1[0] - b.1[0], a.1[1] - b.1[1], a.1[2] - b.1[2]])).collect();
        assert!(x.iter().zip(&y).all(|(a, b)| a.0 == b.0));
        let pre = d.iter().rev().find(|(k, _)| *k <= k_r).unwrap();
        // A correction measures the entry one delay (344 steps) old: the first entry made after the reset step is measured
        // 344 steps later, so the stale slew of the first 8 steps shows from k_r + 352 on.
        let post = d.iter().find(|(k, _)| *k > k_r + 360).unwrap();
        let m = rotation(&r.dq);
        let rot = |v: [f64; 3]| -> [f64; 3] { core::array::from_fn(|i| (0..3).map(|j| f64::from(m[i][j]) * v[j]).sum()) };
        let n = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        let ang = |a: [f64; 3], b: [f64; 3]| ((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (n(a) * n(b))).clamp(-1.0, 1.0).acos().to_degrees();
        let want = rot(pre.1);
        let (a_rot, a_raw) = (ang(post.1, want), ang(post.1, pre.1));
        println!("t5d: yaw reset at step {k_r} (dpsi {:+.1} deg); offset's error vector before |{:.4}| after |{:.4}| rad; direction against the rotated one {a_rot:.3} deg, against the unrotated one {a_raw:.1} deg", rr.dpsi.to_degrees(), n(pre.1), n(post.1));
        assert!(n(pre.1) > 0.04 && n(post.1) < n(pre.1));
        assert!(a_raw > 60.0, "the reset must turn the error vector: {a_raw}");
        assert!(a_rot < 0.15, "rotated direction {a_rot} deg");
    }

    #[test]
    fn t5b_the_flags_of_the_fields_a_reset_moved_are_false_for_exactly_that_step() {
        let t = Tuning::DEFAULT;
        let st = stream(&t, 100);
        let mut core = Eskf::new(&t).unwrap();
        let mut bare = Eskf::new(&t).unwrap();
        let mut steer = EskfSteer::new(&mut core, &t).unwrap();
        let (mut differ, mut kinds) = (Vec::new(), [0usize; 3]);
        for (k, s) in st.iter().enumerate() {
            // Disarmed from 61 s to 70 s (the compass turned at 60 s): the disarmed yaw reset.
            steer.set_armed(armed_but(s, 61_000_000, 70_000_000));
            bare.set_armed(armed_but(s, 61_000_000, 70_000_000));
            let live = steer.out.live;
            let (n, b) = (steer.update(s), bare.update(s));
            let r = bare.last_reset().filter(|_| live && steer.out.live);
            let (ov, bv) = (n.valid, b.valid);
            let mut want = bv;
            if let Some(r) = r {
                if r.dv[2] != 0.0 || r.dp[2] != 0.0 {
                    want.velocity_d = false;
                    want.position_d = false;
                    kinds[0] += 1;
                }
                if r.dv[0] != 0.0 || r.dv[1] != 0.0 || r.dp[0] != 0.0 || r.dp[1] != 0.0 {
                    want.velocity_ne = false;
                    want.position_ne = false;
                    kinds[1] += 1;
                }
                if r.dq[3] != 0.0 {
                    want.heading = false;
                    kinds[2] += 1;
                }
            }
            assert_eq!(ov, want, "step {k}");
            if ov != bv {
                differ.push(k);
            }
        }
        println!("t5b: steps whose flags differ from the core's: {differ:?}; D resets {}, NE resets {}, yaw resets {}", kinds[0], kinds[1], kinds[2]);
        // Each differing step is isolated: its neighbours carry the core's flags (the assert above, per step).
        assert!(differ.windows(2).all(|w| w[1] > w[0] + 1), "a field was false for two steps in a row");
        assert!(kinds.iter().all(|k| *k >= 1) && differ.len() >= 4);
    }

    #[test]
    fn t5b_p2_after_a_gnss_outage_longer_than_the_timeout_the_reset_step_drops_the_ne_flags_and_the_output_steps_by_the_exact_delta() {
        // Addendum P-2: the first accepted epoch after the outage is a Src 9 reset (started cleared at the timeout); the output's
        // NE flags are false for exactly that step and its position steps by the exact delta (t5c's formula).
        let t = Tuning::DEFAULT;
        let mut w = World::new(&t);
        w.yaw = 0.4;
        w.v = [2.0, 0.5, 0.0];
        w.gps_vel = true;
        w.fault = Fault::Gone(25_000_000, 33_000_000);
        let mut st = Vec::new();
        while w.t_us() < 45_000_000 {
            st.push(w.step());
        }
        let mut core = Eskf::new(&t).unwrap();
        let mut bare = Eskf::new(&t).unwrap();
        let mut steer = EskfSteer::new(&mut core, &t).unwrap();
        let (mut prev, mut prev2, mut dropped_at, mut at) = (None::<([f32; 3], [f32; 3])>, None::<[f32; 3]>, None, None);
        let (mut ne_false_steps, mut resets_after) = (0usize, 0usize);
        for (k, s) in st.iter().enumerate() {
            steer.set_armed(armed_at(s));
            bare.set_armed(armed_at(s));
            let t_prev = steer.out.t_us;
            let live = steer.out.live;
            let (n, b) = (steer.update(s), bare.update(s));
            if s.imu1.t_us > 25_000_000 && dropped_at.is_none() && !b.valid.position_ne {
                dropped_at = Some(k);
            }
            let reset_now = s.imu1.t_us > 30_000_000 && bare.last_reset().is_some() && bare.stats().gps_resets > 1 && at.is_none();
            if reset_now {
                let r = bare.last_reset().unwrap();
                at = Some(k);
                // Core flags are back, the output's are false for this step only (live: the delta was applied).
                assert!(live && steer.out.live);
                assert!(b.valid.position_ne && b.valid.velocity_ne && !n.valid.position_ne && !n.valid.velocity_ne, "step {k}: core {:?} output {:?}", b.valid, n.valid);
                let (p0, v0) = prev.unwrap();
                // The ordinary change of the velocity in a step (the output's own tracking correction) is the step before's.
                let dv_ord: [f32; 3] = core::array::from_fn(|i| v0[i] - prev2.unwrap()[i]);
                let t_r = steer.core().horizon_us().unwrap();
                let dt = (s.imu1.t_us - t_prev) as f32 * 1e-6;
                let before = (t_prev as i64 - t_r as i64) as f32 * 1e-6;
                let step_p: [f32; 3] = core::array::from_fn(|i| r.dp[i] + r.dv[i] * before + (v0[i] + r.dv[i]) * dt);
                let tol_p = 16.0 * f32::EPSILON * 30.0;
                let tol_v = 16.0 * f32::EPSILON * 3.0;
                let (mut ep, mut ev) = (0.0f32, 0.0f32);
                for i in 0..3 {
                    ep = ep.max(fabsf(n.position_m[i] - p0[i] - step_p[i]));
                    ev = ev.max(fabsf(n.velocity_ms[i] - v0[i] - r.dv[i] - dv_ord[i]));
                }
                println!("t5b P-2: outage reset at step {k} (t {} us), dp {:?}, dv {:?}; exact-delta difference position {ep:.2e} (tol {tol_p:.2e}) velocity {ev:.2e} (tol {tol_v:.2e}); core NE flags dropped at step {dropped_at:?}", s.imu1.t_us, r.dp, r.dv);
                assert!(norm3(&r.dp) > 3.0, "the outage moved the hold position away: {:?}", r.dp);
                assert!(ep <= tol_p && ev <= tol_v);
            } else if let Some(kr) = at {
                if k == kr + 1 {
                    assert!(n.valid.position_ne && n.valid.velocity_ne, "NE flags back one step after the reset");
                }
                if k > kr {
                    resets_after += usize::from(bare.last_reset().is_some());
                }
            } else if s.imu1.t_us > 30_000_000 {
                ne_false_steps += usize::from(!n.valid.position_ne);
            }
            prev2 = prev.map(|p| p.1);
            prev = Some((n.position_m, n.velocity_ms));
        }
        let kr = at.expect("a Src 9 reset after the outage");
        assert!(dropped_at.is_some_and(|d| d < kr), "NE flags false from the timeout to the reset");
        assert!(ne_false_steps > 0 && resets_after == 0, "no second reset after it: {resets_after}");
    }

    #[test]
    fn t8_a_non_finite_core_state_drops_tilt_the_same_step_keeps_the_output_finite_and_the_next_alignment_recopies() {
        let t = Tuning::DEFAULT;
        let st = stream(&t, 20);
        let mut core = Eskf::new(&t).unwrap();
        let mut steer = EskfSteer::new(&mut core, &t).unwrap();
        let poke = st.iter().position(|s| s.imu1.t_us >= 10_000_000).unwrap();
        let (mut dropped, mut back) = (None, None);
        for (k, s) in st.iter().enumerate() {
            steer.set_armed(armed_at(s));
            if k == poke {
                assert!(steer.out.live && steer.corrected() > 100);
                crate::eskf::tests::poison(steer.core);
            }
            let nav = steer.update(s);
            assert!(nav.attitude.iter().chain(&nav.rates_rads).chain(&nav.velocity_ms).chain(&nav.position_m).all(|v| v.is_finite()), "step {k}");
            if k == poke {
                assert!(!nav.valid.tilt && !steer.out.live, "tilt must drop in the step of the re-initialisation");
                dropped = Some(k);
            }
            if k > poke && back.is_none() && nav.valid.tilt {
                // The re-copy: the output is the core's nominal state, the ring is empty, no rate correction.
                let (q, v, p) = steer.core().nominal();
                assert!(steer.out.live && steer.out.buf.len == 0 && steer.out.rate_corr == [0.0; 3]);
                assert_eq!((steer.out.nom.q, steer.out.nom.v, steer.out.nom.p), (q, v, p));
                assert_eq!(nav.velocity_ms, v);
                back = Some(k);
            }
        }
        println!("t8: tilt dropped at step {dropped:?}, back at step {back:?} ({} steps later); reinits {}", back.unwrap() - poke, steer.core().stats().reinits);
        assert_eq!(steer.core().stats().reinits, 1);
        assert!(back.unwrap() > poke);
    }

    /// The first step at or after `from` whose flag (read by `f`) is false, for the wrapper and for the bare core.
    fn drop_step(t: &Tuning, st: &[Sensors], from: usize, f: impl Fn(&NavState) -> bool) -> (usize, usize) {
        let mut core = Eskf::new(t).unwrap();
        let mut bare = Eskf::new(t).unwrap();
        let mut steer = EskfSteer::new(&mut core, t).unwrap();
        let (mut a, mut b) = (None, None);
        for (k, s) in st.iter().enumerate() {
            steer.set_armed(armed_at(s));
            bare.set_armed(armed_at(s));
            let (n, m) = (steer.update(s), bare.update(s));
            if k >= from {
                if a.is_none() && !f(&n) {
                    a = Some(k);
                }
                if b.is_none() && !f(&m) {
                    b = Some(k);
                }
            }
        }
        (a.expect("the flag never dropped"), b.expect("the core's flag never dropped"))
    }

    #[test]
    fn t9_loss_latency_the_flags_drop_the_timeout_plus_the_horizon_delay_after_the_last_measurement() {
        let t = Tuning::DEFAULT;
        let st = stream(&t, 60);
        let cut = st.iter().position(|s| s.imu1.t_us >= 30_000_000).unwrap();
        let step = PERIOD_US;
        let d = t.nav_gps_delay_us;
        // Barometer stops: its last sample's validity time is arrival - nav_baro_delay_us; the flag drops one timeout plus
        // the delay after it (the core ages the aid in horizon time: horizon = now - delay).
        let mut no_baro = st.clone();
        let last_baro = no_baro[..cut].iter().rev().find_map(|s| s.baro.map(|b| b.t_us)).unwrap();
        for s in &mut no_baro[cut..] {
            s.baro = None;
        }
        let (a, b) = drop_step(&t, &no_baro, cut, |n| n.valid.velocity_d && n.valid.position_d);
        let t_drop = no_baro[a].imu1.t_us;
        let want = last_baro - t.nav_baro_delay_us + t.nav_aid_timeout_us + d;
        println!("t9 barometer: last sample {last_baro} us; flags dropped at {t_drop} us = last + {:.4} s (timeout + delay - baro delay = {:.4} s); wrapper step {a}, bare core step {b}", (t_drop - last_baro) as f64 * 1e-6, (want - last_baro) as f64 * 1e-6);
        assert_eq!(a, b, "the wrapper's flag drops in the core's step");
        assert!(t_drop + step >= want && t_drop <= want + 3 * step, "dropped at {t_drop}, expected {want}");
        // GNSS stops: an epoch is fused on arrival at the horizon (arrival - delay): the NE flags drop one timeout after
        // the arrival, which is the timeout plus the delay after the measurement's own time (arrival - delay).
        let mut no_gps = st.clone();
        let last_gps = no_gps[..cut].iter().rev().find_map(|s| s.gnss.map(|g| g.t_us)).unwrap();
        for s in &mut no_gps[cut..] {
            s.gnss = None;
        }
        let (a, b) = drop_step(&t, &no_gps, cut, |n| n.valid.velocity_ne && n.valid.position_ne);
        let t_drop = no_gps[a].imu1.t_us;
        let want = last_gps + t.nav_aid_timeout_us;
        println!("t9 GNSS: last epoch {last_gps} us; flags dropped at {t_drop} us = last + {:.4} s (timeout = {:.4} s; = timeout + delay after the measurement time); wrapper step {a}, bare core step {b}", (t_drop - last_gps) as f64 * 1e-6, t.nav_aid_timeout_us as f64 * 1e-6);
        assert_eq!(a, b);
        assert!(t_drop + step >= want && t_drop <= want + 6 * step, "dropped at {t_drop}, expected {want}");
    }

    #[test]
    fn t11_i2_the_core_does_at_most_one_job_and_the_wrapper_one_correction_per_step() {
        let t = Tuning::DEFAULT;
        let st = stream(&t, 100);
        let mut core = Eskf::new(&t).unwrap();
        let mut steer = EskfSteer::new(&mut core, &t).unwrap();
        let (mut worst, mut steps_with) = (0u32, 0usize);
        for s in &st {
            steer.set_armed(armed_at(s));
            let c0 = steer.corrected();
            steer.update(s);
            let n = steer.corrected() - c0;
            worst = worst.max(n);
            steps_with += usize::from(n == 1);
        }
        let jobs = steer.core().stats().max_jobs_per_step;
        println!("t11: max jobs per step {jobs}, max corrections per step {worst}, steps with a correction {steps_with}");
        assert!(jobs <= 1 && worst == 1 && steps_with > 5_000);
    }

    /// The stream of the SteerMix tests: yawed, moving north-east at constant velocity, a vertical manoeuvre (a climb
    /// from 10 s, at 2 m/s from 12 s; a reversal 20 s to 24 s to a descent at 2 m/s; the descent ended 30 s to 32 s), the
    /// first compass alignment, GNSS with a 10-sigma north fault from 25 s (C3 reset), and from 36 s the barometer 40 m
    /// too high (the height reset about 5 s later, armed).
    fn mix_stream(t: &Tuning, secs: u64) -> Vec<Sensors> {
        let mut w = World::new(t);
        w.yaw = 0.4;
        w.v = [2.0, 0.5, 0.0];
        w.gps_vel = true;
        let sigma = (f64::from(w.std_mm) * 1e-3).max(f64::from(t.nav_gps_pos_sigma_min_m));
        let mut out = Vec::new();
        while w.t_us() < secs * 1_000_000 {
            let ts = w.t_us() as f64 * 1e-6;
            w.a[2] = if (10.0..12.0).contains(&ts) {
                -1.0
            } else if (20.0..24.0).contains(&ts) {
                1.0
            } else if (30.0..32.0).contains(&ts) {
                -1.0
            } else {
                0.0
            };
            if w.t_us() >= 25_000_000 && w.fault == Fault::None {
                w.fault = Fault::North(w.t_us(), 10.0 * sigma);
            }
            let mut s = w.step();
            if let Some(b) = s.baro.as_mut().filter(|_| w.t_us() >= 36_000_000) {
                b.reading.pressure_pa = pressure_at(40.0 - w.p[2]);
            }
            out.push(s);
        }
        out
    }

    fn zero_delay(t: &Tuning) -> EskfZ {
        EskfZ::with_aiding(t, crate::eskf::Aiding::ImuBaro).unwrap()
    }

    fn nonvertical_bits(n: &NavState) -> Vec<u32> {
        let mut b: Vec<u32> = std::vec![n.t_us as u32, (n.t_us >> 32) as u32];
        b.extend(n.attitude.iter().chain(&n.rates_rads).chain(&n.velocity_ms[..2]).chain(&n.position_m[..2]).map(|x| x.to_bits()));
        let v = n.valid;
        b.extend([v.tilt, v.heading, v.rates, v.velocity_ne, v.position_ne].map(u32::from));
        let s = n.sigma.unwrap();
        b.extend([s.tilt_rad, s.heading_rad, s.velocity_ne_ms, s.position_ne_m].map(f32::to_bits));
        b
    }

    #[test]
    fn m1_the_mix_equals_an_output_predictor_twin_off_the_vertical_and_a_zero_delay_twin_on_it() {
        let t = Tuning::DEFAULT;
        let st = mix_stream(&t, 45);
        let (mut core, mut vz) = (Eskf::new(&t).unwrap(), zero_delay(&t));
        let mut mix = SteerMix::new(EskfSteer::new(&mut core, &t).unwrap(), &mut vz);
        let mut core_b = Eskf::new(&t).unwrap();
        let mut bare_h = EskfSteer::new(&mut core_b, &t).unwrap();
        let mut bare_z = zero_delay(&t);
        let (mut h_vs_z, mut moved_steps, mut rule_mattered, mut valid_steps) = (0usize, 0usize, 0usize, 0usize);
        let (mut vd_min, mut vd_max, mut d_min, mut d_max, mut worst) = (f32::MAX, f32::MIN, f32::MAX, f32::MIN, 0.0f32);
        for (k, s) in st.iter().enumerate() {
            let armed = armed_at(s);
            mix.set_armed(armed);
            bare_h.set_armed(armed);
            bare_z.set_armed(armed);
            let m = mix.update(s);
            let h = bare_h.update(s);
            let z = bare_z.update(s);
            // Everything but D, v_D, their sigmas and their flags is the output predictor's, bit for bit.
            assert_eq!(nonvertical_bits(&m), nonvertical_bits(&h), "step {k}");
            // D, v_D and their sigmas are the zero-delay filter's, bit for bit.
            assert_eq!(m.position_m[2].to_bits(), z.position_m[2].to_bits(), "step {k}: D");
            assert_eq!(m.velocity_ms[2].to_bits(), z.velocity_ms[2].to_bits(), "step {k}: v_D");
            let (ms, zs) = (m.sigma.unwrap(), z.sigma.unwrap());
            assert_eq!(ms.position_d_m.to_bits(), zs.position_d_m.to_bits(), "step {k}: sigma D");
            assert_eq!(ms.velocity_d_ms.to_bits(), zs.velocity_d_ms.to_bits(), "step {k}: sigma v_D");
            // Their flags are the zero-delay filter's, except false in a step whose reset moved D or v_D.
            let vmoved = bare_z.last_reset().is_some_and(|r| r.dp[2] != 0.0 || r.dv[2] != 0.0);
            assert_eq!(m.valid.position_d, z.valid.position_d && !vmoved, "step {k}: flag D");
            assert_eq!(m.valid.velocity_d, z.valid.velocity_d && !vmoved, "step {k}: flag v_D");
            moved_steps += usize::from(vmoved);
            rule_mattered += usize::from(vmoved && z.valid.position_d && z.valid.velocity_d);
            valid_steps += usize::from(m.valid.position_d && m.valid.velocity_d);
            h_vs_z += usize::from(h.position_m[2].to_bits() != z.position_m[2].to_bits());
            if m.valid.position_d {
                (vd_min, vd_max) = (vd_min.min(m.velocity_ms[2]), vd_max.max(m.velocity_ms[2]));
                (d_min, d_max) = (d_min.min(m.position_m[2]), d_max.max(m.position_m[2]));
                worst = worst.max(fabsf(m.position_m[2] - h.position_m[2]));
            }
        }
        println!(
            "m1: {} steps; D {d_min:.2} .. {d_max:.2} m, vD {vd_min:.2} .. {vd_max:.2} m/s; flags true in {valid_steps} steps; ESKF2 reset moved D or vD in {moved_steps} step(s), of which {rule_mattered} with ESKF2's own flags true; the output's D differs from ESKF2's in {h_vs_z} steps, by up to {worst:.4} m",
            st.len()
        );
        assert!(vd_min < -1.9 && vd_max > 1.9 && d_max - d_min > 10.0, "the manoeuvre must be flown: {vd_min} {vd_max} {d_min} {d_max}");
        assert!(moved_steps >= 1 && rule_mattered >= 1, "the vmoved rule must matter in this stream");
        assert!(h_vs_z > 1_000 && worst > 1e-4, "taking D from the output predictor must be visible: {h_vs_z}, {worst}");
    }

    // ---- Amendment V, packet P-V1: the zero-delay filter with the VelD option ----

    #[test]
    fn v8_with_veld_on_the_mix_and_its_core_equal_the_mix_with_it_off_off_the_vertical() {
        let t = Tuning::DEFAULT;
        let st = mix_stream(&t, 45);
        let (mut core_a, mut vz_a) = (Eskf::new(&t).unwrap(), EskfZ::with_gps_vd(&t).unwrap());
        assert!(!core_a.gps_vd() && vz_a.gps_vd());
        let mut on = SteerMix::new(EskfSteer::new(&mut core_a, &t).unwrap(), &mut vz_a);
        let (mut core_b, mut vz_b) = (Eskf::new(&t).unwrap(), zero_delay(&t));
        let mut off = SteerMix::new(EskfSteer::new(&mut core_b, &t).unwrap(), &mut vz_b);
        let mut differ = 0usize;
        for (k, s) in st.iter().enumerate() {
            let armed = armed_at(s);
            on.set_armed(armed);
            off.set_armed(armed);
            let (m, o) = (on.update(s), off.update(s));
            assert_eq!(nonvertical_bits(&m), nonvertical_bits(&o), "step {k}");
            assert!(on.steer().core().events().eq(off.steer().core().events()), "step {k}");
            assert_eq!(on.steer().core().nominal(), off.steer().core().nominal(), "step {k}");
            differ += usize::from(m.position_m[2] != o.position_m[2] || m.velocity_ms[2] != o.velocity_ms[2]);
        }
        assert_eq!(on.steer().core().stats(), off.steer().core().stats());
        let vs = on.vert().stats();
        println!("v8: {} steps: non-vertical fields and the ESKF3 core bit-identical with VelD on and off; VelD fused {} gated {}; D or vD differ in {differ} steps (control)", st.len(), vs.gps_vd_fused, vs.gps_vd_gated);
        assert!(vs.gps_vd_fused > 100 && differ > 1_000, "the control: VelD must change the vertical fields");
    }

    #[test]
    fn v10_with_veld_on_the_mix_still_takes_only_the_vertical_from_eskf2() {
        let t = Tuning::DEFAULT;
        let st = mix_stream(&t, 45);
        let (mut core, mut vz) = (Eskf::new(&t).unwrap(), EskfZ::with_gps_vd(&t).unwrap());
        let mut mix = SteerMix::new(EskfSteer::new(&mut core, &t).unwrap(), &mut vz);
        let mut core_b = Eskf::new(&t).unwrap();
        let mut bare_h = EskfSteer::new(&mut core_b, &t).unwrap();
        let mut bare_z = EskfZ::with_gps_vd(&t).unwrap();
        let (mut h_vs_z, mut moved_steps) = (0usize, 0usize);
        for (k, s) in st.iter().enumerate() {
            let armed = armed_at(s);
            mix.set_armed(armed);
            bare_h.set_armed(armed);
            bare_z.set_armed(armed);
            let (m, h, z) = (mix.update(s), bare_h.update(s), bare_z.update(s));
            assert_eq!(nonvertical_bits(&m), nonvertical_bits(&h), "step {k}");
            assert_eq!(m.position_m[2].to_bits(), z.position_m[2].to_bits(), "step {k}: D");
            assert_eq!(m.velocity_ms[2].to_bits(), z.velocity_ms[2].to_bits(), "step {k}: v_D");
            let (ms, zs) = (m.sigma.unwrap(), z.sigma.unwrap());
            assert_eq!(ms.position_d_m.to_bits(), zs.position_d_m.to_bits(), "step {k}: sigma D");
            assert_eq!(ms.velocity_d_ms.to_bits(), zs.velocity_d_ms.to_bits(), "step {k}: sigma v_D");
            let vmoved = bare_z.last_reset().is_some_and(|r| r.dp[2] != 0.0 || r.dv[2] != 0.0);
            assert_eq!(m.valid.position_d, z.valid.position_d && !vmoved, "step {k}: flag D");
            assert_eq!(m.valid.velocity_d, z.valid.velocity_d && !vmoved, "step {k}: flag v_D");
            moved_steps += usize::from(vmoved);
            h_vs_z += usize::from(h.position_m[2].to_bits() != z.position_m[2].to_bits());
        }
        let vs = bare_z.stats();
        println!("v10: {} steps; VelD fused {} gated {}; ESKF2 reset moved D or vD in {moved_steps} step(s); the output's D differs from ESKF2's in {h_vs_z} steps (control)", st.len(), vs.gps_vd_fused, vs.gps_vd_gated);
        assert!(vs.gps_vd_fused > 100 && moved_steps >= 1);
        assert!(h_vs_z > 1_000, "the control: taking D from the output predictor must be visible: {h_vs_z}");
    }

    #[test]
    fn size_of_the_mix() {
        println!("size_of SteerMix {} EskfSteer {} EskfZ {} Eskf {}", size_of::<SteerMix>(), size_of::<EskfSteer>(), size_of::<EskfZ>(), size_of::<Eskf>());
    }
}
