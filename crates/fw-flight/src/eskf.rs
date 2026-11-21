//! The shadow navigation filter (decision-nav-eskf-20261120, sections 1a-1c, 3, 6, 12; Amendment B, B2 where it
//! stands; Amendment C; Amendment C-1; packet E1b): a 15-state error-state Kalman filter on IMU1, barometer, compass
//! and GNSS at a delayed time horizon, t_now - nav_gps_delay_us (C2a), so that every measurement is fused at its own
//! time. It never steers in phase 1 and is not in the flight image (packet E2 puts it on the board in shadow).
//!
//! The arithmetic is the N0 bench's ([`crate::eskf_n0`]): nominal integration on every IMU1 sample (at the horizon),
//! the structured F P F^T + Q every nav_predict_us, sequential scalar fusions. Work rule I2: each step integrates the
//! nominal state and does at most ONE heavy job (one covariance propagation, one scalar fusion, an alignment or a
//! reset), taken from a queue of one GNSS epoch (4 jobs), one barometer set (3) and one compass sample (1); a newer set
//! of the same kind replaces an unprocessed one (C9). Pure arithmetic: no lock, timer or log. The log codes of C8 are
//! returned by [`Eskf::events`] for the caller to write.
//!
//! [`NavState::t_us`] of this estimator is the horizon time, the time the state refers to (C8: "for I = 1, TimeUS is
//! the horizon time"); there is no output predictor in phase 1.
// Fixed-size arrays only; every index is a loop bound or a constant below the array length.
#![allow(clippy::indexing_slicing, clippy::needless_range_loop)]

use crate::eskf_n0::{
    self as n0, ACCEL_BIAS, ATT, Cov, GYRO_BIAS, N, Noise, Nominal, POS, VEL, rotation,
};
use crate::gnc::{Estimator, GeoPoint, NavSigma, NavState, Sensors, Valid};
use crate::mahony::{ACCEL_MAX_G, ACCEL_MIN_G, DT_MAX_US};
use crate::mahony_baro::isa_height_m;
use crate::tuning::Tuning;
use core::f32::consts::PI;
use fw_core::calib::GRAVITY_MSS;
use fw_core::sample::{GnssSample, MagExpected};
use libm::{atan2f, cosf, fabsf, sinf, sqrtf};

/// IMU1 samples the ring holds: nav_buffer_us (500 ms) at IMU1's 803.5 Hz, 402 samples of 32 B (C8).
pub const RING: usize = 402;
/// WGS 84 semi-major axis, m (physical constant, as the simulator's GNSS model).
pub const WGS84_A_M: f32 = 6_378_137.0;
/// WGS 84 flattening (physical constant, as the simulator's GNSS model).
pub const WGS84_F: f32 = 1.0 / 298.257_23;
/// The span of the data the near-zero-speed screen (E3b) was validated on, N0c/N0e at 1 Hz, us: the
/// position-difference speed is taken between the current epoch and the newest earlier epoch at least this much older.
/// A derived rule, for the owner's approval.
pub const SPD_SPAN_US: u64 = 1_000_000;
/// Fixes kept for that span: 1 s at the receiver's highest rate, 10 Hz (section 4a), the current one, and one for jitter.
const FIX_HIST: usize = 12;
/// The barometer's rate, Hz (section 4a: "baro 50"), for the waiting buffer.
const BARO_RATE_HZ: u64 = 50;
/// The compass's rate, Hz (section 4a: "compass 100"), for the waiting buffer.
const MAG_RATE_HZ: u64 = 100;
/// Barometer samples that can wait for the horizon: nav_buffer_us (the largest delay the constructor accepts) times the
/// rate, 25.
pub const BARO_WAIT: usize = (Tuning::DEFAULT.nav_buffer_us * BARO_RATE_HZ).div_ceil(1_000_000) as usize;
/// Compass samples that can wait: the same window times the rate, 50. The once-per-second fusion does not shrink the
/// worst case: which sample is used is decided at the horizon (the health test needs the horizon tilt, and a refused
/// sample does not use the slot), so any sample of the window may be the one; samples that arrive before the next
/// fusion slot while the yaw is aligned are not kept.
pub const MAG_WAIT: usize = (Tuning::DEFAULT.nav_buffer_us * MAG_RATE_HZ).div_ceil(1_000_000) as usize;
/// 360 degrees in 1e-7 degree.
const E7_TURN: i64 = 3_600_000_000;

/// North, east and down (m) of `point` from `origin` (decision-sim-seams 10): i64 differences first, the longitude
/// difference wrapped to [-180, 180) degrees, scaled on the tangent plane at the origin with the WGS 84 meridian and
/// prime-vertical radii (the simulator's form); down = -(alt difference) / 1000.
pub fn ned_from_geo(origin: &GeoPoint, point: &GeoPoint) -> [f32; 3] {
    let e2 = WGS84_F * (2.0 - WGS84_F);
    let lat0 = origin.lat_e7 as f32 * 1e-7 * PI / 180.0;
    let s = sinf(lat0);
    let w = 1.0 - e2 * s * s;
    let (rm, rn) = (WGS84_A_M * (1.0 - e2) / (w * sqrtf(w)), WGS84_A_M / sqrtf(w));
    let dlat = i64::from(point.lat_e7) - i64::from(origin.lat_e7);
    let dlon = (i64::from(point.lon_e7) - i64::from(origin.lon_e7) + E7_TURN / 2).rem_euclid(E7_TURN) - E7_TURN / 2;
    let rad = |e7: i64| e7 as f32 * 1e-7 * PI / 180.0;
    let dalt = i64::from(point.alt_mm) - i64::from(origin.alt_mm);
    [rad(dlat) * rm, rad(dlon) * rn * cosf(lat0), -(dalt as f32) * 1e-3]
}

/// NKI record codes (section 8c, B5, C8).
pub mod code {
    /// Src: GNSS position north.
    pub const GPS_N: u8 = 0;
    /// Src: GNSS position east.
    pub const GPS_E: u8 = 1;
    /// Src: GNSS velocity north.
    pub const VN: u8 = 2;
    /// Src: GNSS velocity east.
    pub const VE: u8 = 3;
    /// Src: barometer.
    pub const BARO: u8 = 4;
    /// Src: no-aid position (one record per axis, N then E).
    pub const NOAID: u8 = 5;
    /// Src: height reset.
    pub const HEIGHT_RESET: u8 = 6;
    /// Src: compass yaw.
    pub const MAG_YAW: u8 = 7;
    /// Src: yaw alignment or reset (Inn = applied change, Var = new variance).
    pub const YAW_RESET: u8 = 8;
    /// Src: GNSS position/velocity reset.
    pub const GPS_RESET: u8 = 9;
    /// Ok: gated out.
    pub const GATED: u8 = 0;
    /// Ok: fused (or applied, for a reset).
    pub const FUSED: u8 = 1;
    /// Ok: refused by the compass health test.
    pub const REFUSED: u8 = 2;
    /// Ok: screened (frozen fix E1s, or near-zero speed E3b).
    pub const SCREENED: u8 = 3;
}

/// One NKI record's content (section 8c): source, innovation, its variance, outcome. TimeUS is the horizon time.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Nki {
    /// Horizon time, us.
    pub t_us: u64,
    /// [`code`] Src.
    pub src: u8,
    /// Innovation (or the applied change of a reset).
    pub inn: f32,
    /// Innovation variance (or the new variance of a reset).
    pub var: f32,
    /// [`code`] Ok.
    pub ok: u8,
}

/// Events of one step: one heavy job, one GNSS screen at arrival, one compass refusal at the horizon.
pub const EVENTS: usize = 3;

/// Counters for tests, the simulator's report and the log.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EskfStats {
    /// Covariance propagations.
    pub predicts: u32,
    /// Barometer scalars fused, gated.
    pub baro_fused: u32,
    /// Barometer scalars gated out.
    pub baro_gated: u32,
    /// Height resets (Src 6).
    pub height_resets: u32,
    /// No-aid scalars fused.
    pub noaid: u32,
    /// Compass samples refused by the health test or for a vanishing horizontal field (Ok 2; every one counted, one
    /// NKI record per nav_mag_fuse_us at most).
    pub mag_refused: u32,
    /// Compass yaw fusions accepted.
    pub mag_fused: u32,
    /// Compass yaw fusions gated out.
    pub mag_gated: u32,
    /// Yaw alignments (Src 8, the first).
    pub yaw_aligns: u32,
    /// Yaw resets after the timeout (Src 8).
    pub yaw_resets: u32,
    /// GNSS samples seen.
    pub gps_epochs: u32,
    /// Epochs failing 5c acceptance (fix type, position, GST lat/lon, interval).
    pub gps_unaccepted: u32,
    /// Accepted epochs before the yaw alignment (not fused).
    pub gps_prealign: u32,
    /// Accepted epochs while disarmed (not fused, C-1).
    pub gps_disarmed: u32,
    /// Epochs screened whole as a frozen fix (E1s).
    pub gps_e1s: u32,
    /// Epochs whose velocity was screened (E3b).
    pub gps_e3b: u32,
    /// GNSS position scalars fused.
    pub gps_pos_fused: u32,
    /// GNSS position scalars gated out.
    pub gps_pos_gated: u32,
    /// GNSS velocity scalars fused.
    pub gps_vel_fused: u32,
    /// GNSS velocity scalars gated out.
    pub gps_vel_gated: u32,
    /// GNSS resets (Src 9): at the first epoch after arming, and C3.
    pub gps_resets: u32,
    /// Re-initialisations on a non-finite state.
    pub reinits: u32,
    /// Heavy jobs in the step that had the most (the I2 check: never above 1).
    pub max_jobs_per_step: u32,
    /// Queued jobs replaced by a newer set of the same kind before they ran.
    pub replaced: u32,
    /// Waiting barometer or compass samples dropped because their buffer was full.
    pub dropped: u32,
    /// Barometer, compass or GNSS samples dropped on arrival because their time (validity time, sample time, or arrival
    /// time minus nav_gps_delay_us) is older than the IMU ring's span, now - nav_buffer_us.
    pub stale: u32,
}

/// The constructor's refusal (C2a).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EskfError {
    /// nav_gps_delay_us > nav_buffer_us.
    DelayBeyondBuffer,
}

/// A first-in first-out buffer of fixed capacity.
#[derive(Clone, Copy, Debug)]
struct Fifo<T: Copy + Default, const C: usize> {
    buf: [T; C],
    head: usize,
    len: usize,
}

impl<T: Copy + Default, const C: usize> Fifo<T, C> {
    fn new() -> Self {
        Fifo { buf: [T::default(); C], head: 0, len: 0 }
    }
    fn full(&self) -> bool {
        self.len == C
    }
    fn get(&self, i: usize) -> T {
        self.buf[(self.head + i) % C]
    }
    fn front(&self) -> Option<T> {
        (self.len > 0).then(|| self.buf[self.head])
    }
    fn pop(&mut self) -> Option<T> {
        let v = self.front()?;
        self.head = (self.head + 1) % C;
        self.len -= 1;
        Some(v)
    }
    /// Pushes; when full, the oldest is dropped and returned.
    fn push(&mut self, v: T) -> Option<T> {
        let dropped = if self.full() { self.pop() } else { None };
        self.buf[(self.head + self.len) % C] = v;
        self.len += 1;
        dropped
    }
}

/// One raw IMU1 sample, 32 B.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct ImuEntry {
    t_us: u64,
    gyro: [f32; 3],
    accel: [f32; 3],
}

/// A barometer sample waiting for the horizon: its validity time and pressure.
#[derive(Clone, Copy, Debug, Default)]
struct BaroWait {
    t_valid_us: u64,
    pressure_pa: f32,
}

/// A compass sample waiting for the horizon (compass delay 0); `expected` says whether the sample carried the expected
/// field (its value is the newest one carried, [`Eskf`]'s `mag_expected`).
#[derive(Clone, Copy, Debug, Default)]
struct MagWait {
    t_us: u64,
    field_ut: [f32; 3],
    expected: bool,
}

/// The barometer set: the barometer scalar, then no-aid N and E (B2).
#[derive(Clone, Copy, Debug)]
struct BaroJob {
    h_b: f32,
    stage: u8,
}

/// What an accepted, armed, aligned GNSS epoch does.
#[derive(Clone, Copy, Debug)]
enum GpsPlan {
    /// Src 9: position (and velocity when there is one) set to the fix.
    Reset,
    /// Four scalars (two when the velocity is screened or absent); `stage` the next one.
    Fuse { stage: u8, bad: bool },
}

#[derive(Clone, Copy, Debug)]
struct GpsJob {
    plan: GpsPlan,
    ne: [f32; 2],
    sigma_ne: [f32; 2],
    /// Velocity N, E and its sigma.
    vel: Option<([f32; 2], f32)>,
}

/// The 15-state shadow filter. See the module text.
#[derive(Clone, Debug)]
pub struct Eskf {
    t: Tuning,
    noise: Noise,
    ring: Fifo<ImuEntry, RING>,
    baro_wait: Fifo<BaroWait, BARO_WAIT>,
    mag_wait: Fifo<MagWait, MAG_WAIT>,
    /// Fixes since arming (t_us, lat, lon), for the position-difference speed over SPD_SPAN_US.
    fix_hist: Fifo<(u64, i32, i32), FIX_HIST>,
    mag_expected: Option<MagExpected>,
    nom: Nominal,
    p: Cov,
    /// Time of the newest sample integrated at the horizon.
    horizon_us: Option<u64>,
    /// Gyro of that sample (the rates output).
    horizon_gyro: [f32; 3],
    since_predict_us: u64,
    predict_due: bool,
    /// Raw specific force times dt and the time, summed over the current interval before the tilt alignment.
    align_dv: [f32; 3],
    align_us: u64,
    tilt_aligned: bool,
    yaw_aligned: bool,
    armed: bool,
    /// The barometer datum (ISA height of the first barometer sample after the tilt alignment), m; set once.
    datum_m: Option<f32>,
    last_baro_aid_us: Option<u64>,
    /// Set by a re-initialisation: the next barometer job initialises the height (as `MahonyBaro` after its own).
    baro_reinit: bool,
    origin: Option<GeoPoint>,
    /// Previous fix (fix type >= 3 with a position): t_us, lat, lon.
    gps_prev: Option<(u64, i32, i32)>,
    /// Horizontal aiding is running: an epoch after arming has reset the position (C-1).
    started: bool,
    last_pos_us: u64,
    latest_ok: bool,
    /// Horizon time of the first gate rejection of each position axis in the current run of rejections.
    rej_since: [Option<u64>; 2],
    aiding_prev: bool,
    hold_ne: [f32; 2],
    next_mag_us: u64,
    /// Sample time from which the next compass refusal is written as an NKI record.
    next_refusal_us: u64,
    last_mag_acc_us: u64,
    mag_rej_since: Option<u64>,
    gps_job: Option<GpsJob>,
    baro_job: Option<BaroJob>,
    mag_job: Option<[f32; 3]>,
    events: [Option<Nki>; EVENTS],
    stats: EskfStats,
    /// The accelerometer-bias inhibit of the no-aid scalars (B2); off only in a test's negative control.
    noaid_inhibit: bool,
}

const NOMINAL0: Nominal = Nominal {
    q: [1.0, 0.0, 0.0, 0.0],
    v: [0.0; 3],
    p: [0.0; 3],
    bg: [0.0; 3],
    ba: [0.0; 3],
    dvel: [0.0; 3],
    dt_sum: 0.0,
};

#[inline(always)]
fn qmul(a: &[f32; 4], b: &[f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

#[inline(always)]
fn normalized(q: [f32; 4]) -> [f32; 4] {
    let inv = 1.0 / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    [q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv]
}

/// An angle wrapped to [-pi, pi] (inputs within one turn of it).
fn wrap(a: f32) -> f32 {
    if a > PI {
        a - 2.0 * PI
    } else if a < -PI {
        a + 2.0 * PI
    } else {
        a
    }
}

/// The C4 health test on a compass field already rotated to NED.
pub fn healthy(m_ned: &[f32; 3], e: &MagExpected) -> bool {
    let norm = sqrtf(m_ned[0] * m_ned[0] + m_ned[1] * m_ned[1] + m_ned[2] * m_ned[2]);
    let inc = atan2f(m_ned[2], sqrtf(m_ned[0] * m_ned[0] + m_ned[1] * m_ned[1]));
    e.strength_ut > 0.0
        && fabsf(norm / e.strength_ut - 1.0) <= e.tol_strength_frac
        && fabsf(inc - e.inclination_rad) <= e.tol_inclination_rad
}

/// The compass field `m_b` rotated to NED by the attitude `q`: m = R(q) m_b.
fn mag_ned(q: &[f32; 4], m_b: &[f32; 3]) -> [f32; 3] {
    let r = rotation(q);
    core::array::from_fn(|i| r[i][0] * m_b[0] + r[i][1] * m_b[1] + r[i][2] * m_b[2])
}

/// The C1b yaw innovation: wrap(dec - atan2(m_E, m_N)), m = R(q) m_b.
pub fn yaw_innov(q: &[f32; 4], m_b: &[f32; 3], dec: f32) -> f32 {
    let m = mag_ned(q, m_b);
    wrap(dec - atan2f(m[1], m[0]))
}

/// The C1b yaw row on the NED attitude error, as corrected by the review (ruling 1, 2026-11-21): the exact Jacobian of
/// the predicted azimuth atan2(m_E, m_N) of m = R(q) m_b, the error composed on the left (q <- [1, dθ/2] q, so
/// m <- m + dθ x m): [-m_N m_D / c, -m_E m_D / c, 1], c = m_N^2 + m_E^2. None when c <= f32::EPSILON |m|^2: a
/// horizontal component below sqrt(f32::EPSILON) (3.5e-4) of the field, where its direction is set by the rounding
/// of m (one ulp of |m| per component) to worse than 3.5e-4 rad and the row's entries exceed 1/sqrt(f32::EPSILON).
pub fn yaw_h(q: &[f32; 4], m_b: &[f32; 3]) -> Option<[f32; 3]> {
    let m = mag_ned(q, m_b);
    let c = m[0] * m[0] + m[1] * m[1];
    if !(c > f32::EPSILON * (c + m[2] * m[2])) {
        return None;
    }
    Some([-m[0] * m[2] / c, -m[1] * m[2] / c, 1.0])
}

impl Eskf {
    /// Not aligned, disarmed, no sample seen. Err when nav_gps_delay_us > nav_buffer_us (C2a).
    pub fn new(tuning: &Tuning) -> Result<Self, EskfError> {
        if tuning.nav_gps_delay_us > tuning.nav_buffer_us {
            return Err(EskfError::DelayBeyondBuffer);
        }
        let t = *tuning;
        Ok(Eskf {
            noise: Noise {
                gyro: t.nav_arw * t.nav_arw,
                accel: t.nav_vrw * t.nav_vrw,
                gyro_bias: t.nav_bg_rw * t.nav_bg_rw,
                accel_bias: t.nav_ba_rw * t.nav_ba_rw,
            },
            t,
            ring: Fifo::new(),
            baro_wait: Fifo::new(),
            mag_wait: Fifo::new(),
            fix_hist: Fifo::new(),
            mag_expected: None,
            nom: NOMINAL0,
            p: [[0.0; N]; N],
            horizon_us: None,
            horizon_gyro: [0.0; 3],
            since_predict_us: 0,
            predict_due: false,
            align_dv: [0.0; 3],
            align_us: 0,
            tilt_aligned: false,
            yaw_aligned: false,
            armed: false,
            datum_m: None,
            last_baro_aid_us: None,
            baro_reinit: false,
            origin: None,
            gps_prev: None,
            started: false,
            last_pos_us: 0,
            latest_ok: false,
            rej_since: [None; 2],
            aiding_prev: false,
            hold_ne: [0.0; 2],
            next_mag_us: 0,
            next_refusal_us: 0,
            last_mag_acc_us: 0,
            mag_rej_since: None,
            gps_job: None,
            baro_job: None,
            mag_job: None,
            events: [None; EVENTS],
            stats: EskfStats::default(),
            noaid_inhibit: true,
        })
    }

    /// The counters.
    pub fn stats(&self) -> &EskfStats {
        &self.stats
    }

    /// This step's NKI records (C8), in order.
    pub fn events(&self) -> impl Iterator<Item = &Nki> {
        self.events.iter().flatten()
    }

    /// Tilt aligned (B2).
    pub fn tilt_aligned(&self) -> bool {
        self.tilt_aligned
    }

    /// Yaw aligned to a healthy compass (C1a).
    pub fn yaw_aligned(&self) -> bool {
        self.yaw_aligned
    }

    /// Gyro bias, rad/s, and accelerometer bias, m/s^2, body frame.
    pub fn biases(&self) -> ([f32; 3], [f32; 3]) {
        (self.nom.bg, self.nom.ba)
    }

    /// One standard deviation of each of the 15 error states, from P's diagonal.
    pub fn sigmas(&self) -> [f32; N] {
        core::array::from_fn(|i| sqrtf(self.p[i][i].max(0.0)))
    }

    fn event(&mut self, src: u8, inn: f32, var: f32, ok: u8) {
        let t_us = self.horizon_us.unwrap_or(0);
        if let Some(slot) = self.events.iter_mut().find(|e| e.is_none()) {
            *slot = Some(Nki { t_us, src, inn, var, ok });
        }
    }

    fn horizon(&self) -> u64 {
        self.horizon_us.unwrap_or(0)
    }

    /// Back to "not aligned", keeping the origin, the barometer datum, the buffers and the counters (B2, C3).
    fn reinitialise(&mut self) {
        self.nom = NOMINAL0;
        self.p = [[0.0; N]; N];
        self.since_predict_us = 0;
        self.predict_due = false;
        self.align_dv = [0.0; 3];
        self.align_us = 0;
        self.tilt_aligned = false;
        self.yaw_aligned = false;
        self.last_baro_aid_us = None;
        self.baro_reinit = true;
        self.started = false;
        self.latest_ok = false;
        self.rej_since = [None; 2];
        self.aiding_prev = false;
        self.hold_ne = [0.0; 2];
        self.mag_rej_since = None;
        self.gps_job = None;
        self.baro_job = None;
        self.mag_job = None;
        self.stats.reinits = self.stats.reinits.wrapping_add(1);
    }

    /// The nominal integration of one sample at the horizon; the alignment sums before the tilt alignment.
    fn integrate(&mut self, s: ImuEntry) {
        let dt_us = match self.horizon_us {
            Some(h) => s.t_us.saturating_sub(h).min(DT_MAX_US),
            None => 0,
        };
        self.horizon_us = Some(s.t_us);
        self.horizon_gyro = s.gyro;
        let dt = dt_us as f32 * 1e-6;
        if self.tilt_aligned {
            n0::nominal_step(&mut self.nom, &s.gyro, &s.accel, dt);
        } else {
            for i in 0..3 {
                self.align_dv[i] += s.accel[i] * dt;
            }
            self.align_us += dt_us;
        }
        self.since_predict_us += dt_us;
        if self.since_predict_us >= self.t.nav_predict_us {
            self.predict_due = true;
        }
    }

    /// The interval's job: the covariance propagation, or before the tilt alignment the alignment test (B2).
    fn predict(&mut self) {
        self.predict_due = false;
        self.since_predict_us = 0;
        if self.tilt_aligned {
            let (dvel, dt) = (self.nom.dvel, self.nom.dt_sum);
            n0::covariance_predict(&mut self.p, &self.nom.q, &dvel, dt, &self.noise);
            // The diagonal held at or above 0, as after a fusion (section 1b).
            for i in 0..N {
                self.p[i][i] = self.p[i][i].max(0.0);
            }
            self.nom.dvel = [0.0; 3];
            self.nom.dt_sum = 0.0;
            self.stats.predicts = self.stats.predicts.wrapping_add(1);
            return;
        }
        let t = self.align_us as f32 * 1e-6;
        let dv = self.align_dv;
        self.align_dv = [0.0; 3];
        self.align_us = 0;
        if t <= 0.0 {
            return;
        }
        let f = [dv[0] / t, dv[1] / t, dv[2] / t];
        let nf = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
        if !(nf >= ACCEL_MIN_G * GRAVITY_MSS && nf <= ACCEL_MAX_G * GRAVITY_MSS && f[2] < 0.0) {
            return;
        }
        let roll = atan2f(-f[1], -f[2]);
        let pitch = atan2f(f[0], sqrtf(f[1] * f[1] + f[2] * f[2]));
        let (cr, sr, cp, sp) = (cosf(0.5 * roll), sinf(0.5 * roll), cosf(0.5 * pitch), sinf(0.5 * pitch));
        self.nom = Nominal { q: normalized([cr * cp, sr * cp, cr * sp, -sr * sp]), ..NOMINAL0 };
        self.p = [[0.0; N]; N];
        let t = &self.t;
        let sig = [
            t.nav_p0_tilt_rad,
            t.nav_p0_tilt_rad,
            t.nav_p0_tilt_rad,
            t.nav_p0_vel_ms,
            t.nav_p0_vel_ms,
            t.nav_p0_vel_ms,
            t.nav_noaid_sigma_dis_m,
            t.nav_noaid_sigma_dis_m,
            t.nav_baro_sigma_m,
            t.nav_p0_bg_rads,
            t.nav_p0_bg_rads,
            t.nav_p0_bg_rads,
            t.nav_p0_ba_ms2,
            t.nav_p0_ba_ms2,
            t.nav_p0_ba_ms2,
        ];
        for i in 0..N {
            self.p[i][i] = sig[i] * sig[i];
        }
        self.tilt_aligned = true;
        self.hold_ne = [0.0; 2];
        self.aiding_prev = false;
    }

    /// Zeroes row and column `i` of P and sets its variance.
    fn reset_state_var(&mut self, i: usize, var: f32) {
        for j in 0..N {
            self.p[i][j] = 0.0;
            self.p[j][i] = 0.0;
        }
        self.p[i][i] = var;
    }

    /// The error-state injection (as N0's).
    fn inject(&mut self, dx: &[f32; N]) {
        let dq = [1.0, 0.5 * dx[ATT], 0.5 * dx[ATT + 1], 0.5 * dx[ATT + 2]];
        self.nom.q = normalized(qmul(&dq, &self.nom.q));
        for i in 0..3 {
            self.nom.v[i] += dx[VEL + i];
            self.nom.p[i] += dx[POS + i];
            self.nom.bg[i] += dx[GYRO_BIAS + i];
            self.nom.ba[i] += dx[ACCEL_BIAS + i];
        }
    }

    /// A scalar fusion of state `idx` (H a single 1), gated at nav_gate_sigma; returns (accepted, innovation variance).
    fn fuse_unit(&mut self, idx: usize, innov: f32, var: f32) -> (bool, f32) {
        let s = self.p[idx][idx] + var;
        let ok = if (POS..POS + 3).contains(&idx) {
            n0::fuse_position(&mut self.nom, &mut self.p, idx - POS, innov, var, self.t.nav_gate_sigma)
        } else {
            let mut h = [0.0f32; N];
            h[idx] = 1.0;
            n0::fuse(&mut self.nom, &mut self.p, &h, innov, var, self.t.nav_gate_sigma)
        };
        (ok, s)
    }

    /// A no-aid scalar on position `axis` (B2): not gated; with the inhibit, K's accelerometer-bias entries are zeroed and P is updated in Joseph form, (I - K H) P (I - K H)^T + K R K^T, which with H = e_k and P H^T = ph is
    /// P - K ph^T - ph K^T + K K^T s.
    fn fuse_noaid(&mut self, axis: usize, innov: f32, var: f32) {
        let k = POS + axis;
        let ph = self.p[k];
        let s = ph[k] + var;
        if s.is_nan() || s <= 0.0 {
            return;
        }
        let mut kg: [f32; N] = core::array::from_fn(|i| ph[i] / s);
        if self.noaid_inhibit {
            kg[ACCEL_BIAS] = 0.0;
            kg[ACCEL_BIAS + 1] = 0.0;
            kg[ACCEL_BIAS + 2] = 0.0;
        }
        for i in 0..N {
            for j in i..N {
                let v = self.p[i][j] - kg[i] * ph[j] - ph[i] * kg[j] + kg[i] * kg[j] * s;
                self.p[i][j] = v;
                self.p[j][i] = v;
            }
            if self.p[i][i] < 0.0 {
                self.p[i][i] = 0.0;
            }
        }
        let dx: [f32; N] = core::array::from_fn(|i| kg[i] * innov);
        self.inject(&dx);
        self.stats.noaid = self.stats.noaid.wrapping_add(1);
        self.event(code::NOAID, innov, s, code::FUSED);
    }

    /// Horizontal aiding (C-1): armed, started, and a GNSS position fused within nav_aid_timeout_us.
    fn aiding(&self) -> bool {
        self.armed && self.started && self.horizon().saturating_sub(self.last_pos_us) < self.t.nav_aid_timeout_us
    }

    fn baro_job(&mut self, mut job: BaroJob) {
        let horizon = self.horizon();
        match job.stage {
            0 if self.baro_reinit => {
                // After a re-initialisation the height is initialised at once (D = -h_b, its row and column
                // re-initialised), as `MahonyBaro` re-initialises; logged and counted as a height reset.
                let r = self.t.nav_baro_sigma_m * self.t.nav_baro_sigma_m;
                let change = -job.h_b - self.nom.p[2];
                self.nom.p[2] = -job.h_b;
                self.reset_state_var(POS + 2, r);
                self.last_baro_aid_us = Some(horizon);
                self.baro_reinit = false;
                self.stats.height_resets = self.stats.height_resets.wrapping_add(1);
                self.event(code::HEIGHT_RESET, change, r, code::FUSED);
            }
            0 => {
                let r = self.t.nav_baro_sigma_m * self.t.nav_baro_sigma_m;
                // h = -D: fused as D = -h_b on the position's down error.
                let innov = -job.h_b - self.nom.p[2];
                let (ok, s) = self.fuse_unit(POS + 2, innov, r);
                if ok {
                    self.last_baro_aid_us = Some(horizon);
                    self.stats.baro_fused = self.stats.baro_fused.wrapping_add(1);
                    self.event(code::BARO, innov, s, code::FUSED);
                } else {
                    self.stats.baro_gated = self.stats.baro_gated.wrapping_add(1);
                    let since = self.last_baro_aid_us.unwrap_or(horizon);
                    if horizon.saturating_sub(since) >= self.t.nav_aid_timeout_us {
                        // As B1 (EKF3 ResetHeight): D to the barometer, its row and column re-initialised; vD to 0
                        // disarmed, kept armed.
                        let change = -job.h_b - self.nom.p[2];
                        self.nom.p[2] = -job.h_b;
                        if !self.armed {
                            self.nom.v[2] = 0.0;
                        }
                        self.reset_state_var(POS + 2, r);
                        self.last_baro_aid_us = Some(horizon);
                        self.stats.height_resets = self.stats.height_resets.wrapping_add(1);
                        self.event(code::HEIGHT_RESET, change, r, code::FUSED);
                    } else {
                        self.event(code::BARO, innov, s, code::GATED);
                    }
                }
            }
            axis => {
                let axis = usize::from(axis - 1);
                let sg = if self.armed { self.t.nav_noaid_sigma_m } else { self.t.nav_noaid_sigma_dis_m };
                let innov = self.hold_ne[axis] - self.nom.p[axis];
                self.fuse_noaid(axis, innov, sg * sg);
            }
        }
        job.stage += 1;
        // The no-aid pair only while horizontal aiding is not running.
        self.baro_job = (job.stage < 3 && !self.aiding()).then_some(job);
    }

    /// Yaw := yaw + dpsi (a rotation about NED down); the yaw row and column of P re-initialised to the compass noise;
    /// b_g,z, b_a,x, b_a,y to 0 with their initial variances and cross terms zeroed (C1a, C3).
    fn yaw_set(&mut self, dpsi: f32) {
        let dq = [cosf(0.5 * dpsi), 0.0, 0.0, sinf(0.5 * dpsi)];
        self.nom.q = normalized(qmul(&dq, &self.nom.q));
        let sm = self.t.nav_mag_heading_sigma_rad;
        self.reset_state_var(ATT + 2, sm * sm);
        self.nom.bg[2] = 0.0;
        self.nom.ba[0] = 0.0;
        self.nom.ba[1] = 0.0;
        let (bg, ba) = (self.t.nav_p0_bg_rads, self.t.nav_p0_ba_ms2);
        self.reset_state_var(GYRO_BIAS + 2, bg * bg);
        self.reset_state_var(ACCEL_BIAS, ba * ba);
        self.reset_state_var(ACCEL_BIAS + 1, ba * ba);
        self.last_mag_acc_us = self.horizon();
        self.mag_rej_since = None;
        self.event(code::YAW_RESET, dpsi, sm * sm, code::FUSED);
    }

    /// The compass job (C1): alignment, the yaw fusion with the full H row, or the reset after the timeout.
    fn mag_job(&mut self, m_b: [f32; 3]) {
        let q = self.nom.q;
        let Some(hy) = yaw_h(&q, &m_b) else {
            // The horizontal field vanishes: no heading in it.
            self.mag_refuse(self.horizon());
            return;
        };
        let y = yaw_innov(&q, &m_b, self.t.nav_mag_declination_rad);
        if !y.is_finite() {
            return;
        }
        if !self.yaw_aligned {
            self.yaw_set(y);
            self.yaw_aligned = true;
            self.stats.yaw_aligns = self.stats.yaw_aligns.wrapping_add(1);
            return;
        }
        if !hy.iter().all(|v| v.is_finite()) {
            return;
        }
        let mut h = [0.0f32; N];
        h[ATT..ATT + 3].copy_from_slice(&hy);
        let var = self.t.nav_mag_heading_sigma_rad * self.t.nav_mag_heading_sigma_rad;
        let s: f32 = var + (0..3).map(|i| (0..3).map(|j| hy[i] * self.p[ATT + i][ATT + j] * hy[j]).sum::<f32>()).sum::<f32>();
        let horizon = self.horizon();
        if n0::fuse(&mut self.nom, &mut self.p, &h, y, var, self.t.nav_gate_sigma) {
            self.last_mag_acc_us = horizon;
            self.mag_rej_since = None;
            self.stats.mag_fused = self.stats.mag_fused.wrapping_add(1);
            self.event(code::MAG_YAW, y, s, code::FUSED);
            return;
        }
        self.stats.mag_gated = self.stats.mag_gated.wrapping_add(1);
        match self.mag_rej_since {
            None => {
                self.mag_rej_since = Some(horizon);
                self.event(code::MAG_YAW, y, s, code::GATED);
            }
            Some(t0) if horizon.saturating_sub(t0) >= self.t.nav_aid_timeout_us => {
                self.yaw_set(y);
                self.stats.yaw_resets = self.stats.yaw_resets.wrapping_add(1);
            }
            Some(_) => self.event(code::MAG_YAW, y, s, code::GATED),
        }
    }

    /// A refused compass sample (Ok 2): counted; one NKI record per nav_mag_fuse_us at most (the log budget).
    fn mag_refuse(&mut self, t_us: u64) {
        self.stats.mag_refused = self.stats.mag_refused.wrapping_add(1);
        if t_us >= self.next_refusal_us {
            self.next_refusal_us = t_us.saturating_add(self.t.nav_mag_fuse_us);
            self.event(code::MAG_YAW, 0.0, 0.0, code::REFUSED);
        }
    }

    /// A compass sample at the horizon: skipped inside the fusion period once aligned; the health test (cheap); a
    /// healthy one queues the compass job.
    fn mag_due(&mut self, w: MagWait) {
        if !self.tilt_aligned {
            return;
        }
        if self.yaw_aligned && w.t_us < self.next_mag_us {
            return;
        }
        let Some(e) = self.mag_expected.filter(|_| w.expected) else {
            // No expected field: the compass is never used (C4).
            return;
        };
        let r = rotation(&self.nom.q);
        let m_b = w.field_ut;
        let m: [f32; 3] = core::array::from_fn(|i| r[i][0] * m_b[0] + r[i][1] * m_b[1] + r[i][2] * m_b[2]);
        if !healthy(&m, &e) || yaw_h(&self.nom.q, &m_b).is_none() {
            self.mag_refuse(w.t_us);
            return;
        }
        self.next_mag_us = w.t_us.saturating_add(self.t.nav_mag_fuse_us);
        if self.mag_job.replace(m_b).is_some() {
            self.stats.replaced = self.stats.replaced.wrapping_add(1);
        }
    }

    /// A GNSS epoch on arrival (its time is the horizon's, C2a): acceptance (5c, C2b), the origin (6a), the C-1 rules,
    /// the frozen-fix and near-zero-speed screens (C2d), and the plan: the Src 9 reset or the scalars.
    fn gnss_arrival(&mut self, g: &GnssSample) {
        self.stats.gps_epochs = self.stats.gps_epochs.wrapping_add(1);
        let r = &g.reading;
        let (Some(lat), Some(lon)) = (r.lat_e7, r.lon_e7) else {
            self.stats.gps_unaccepted = self.stats.gps_unaccepted.wrapping_add(1);
            return;
        };
        if r.fix_type < 3 {
            self.stats.gps_unaccepted = self.stats.gps_unaccepted.wrapping_add(1);
            return;
        }
        let prev = self.gps_prev.replace((g.t_us, lat, lon));
        let max_interval_us = 1e6 / self.t.nav_gps_min_rate_hz;
        let in_rate = prev.is_some_and(|(t0, _, _)| (g.t_us.saturating_sub(t0) as f32) <= max_interval_us);
        let (Some(lat_std), Some(lon_std)) = (r.lat_std_mm, r.lon_std_mm) else {
            self.stats.gps_unaccepted = self.stats.gps_unaccepted.wrapping_add(1);
            return;
        };
        if !in_rate {
            self.stats.gps_unaccepted = self.stats.gps_unaccepted.wrapping_add(1);
            return;
        }
        let alt = r.alt_mm.unwrap_or(0);
        let d_mm = (self.nom.p[2] * 1000.0) as i32;
        let origin = *self.origin.get_or_insert(GeoPoint { lat_e7: lat, lon_e7: lon, alt_mm: alt.saturating_add(d_mm) });
        let here = GeoPoint { lat_e7: lat, lon_e7: lon, alt_mm: origin.alt_mm };
        let ne3 = ned_from_geo(&origin, &here);
        let ne = [ne3[0], ne3[1]];
        // The exact repeat of the previous fix, and the receiver's position-difference speed over at least SPD_SPAN_US
        // (from the newest fix since arming that is that much older; none yet: no screen decision).
        let repeat = prev.is_some_and(|(_, la, lo)| la == lat && lo == lon);
        let pd_speed = (0..self.fix_hist.len)
            .rev()
            .map(|i| self.fix_hist.get(i))
            .find(|&(t0, _, _)| g.t_us.saturating_sub(t0) >= SPD_SPAN_US)
            .map(|(t0, la, lo)| {
                let d = ned_from_geo(&GeoPoint { lat_e7: la, lon_e7: lo, alt_mm: origin.alt_mm }, &here);
                sqrtf(d[0] * d[0] + d[1] * d[1]) / (g.t_us.saturating_sub(t0) as f32 * 1e-6)
            });
        self.fix_hist.push((g.t_us, lat, lon));
        if !self.yaw_aligned {
            self.stats.gps_prealign = self.stats.gps_prealign.wrapping_add(1);
            return;
        }
        if !self.armed {
            self.stats.gps_disarmed = self.stats.gps_disarmed.wrapping_add(1);
            return;
        }
        if repeat {
            self.stats.gps_e1s = self.stats.gps_e1s.wrapping_add(1);
            self.event(code::GPS_N, 0.0, 0.0, code::SCREENED);
            return;
        }
        let sv = self.t.nav_gps_vel_sigma_ms;
        let (vel, h_speed) = if let Some(v) = r.vel_ned_mm_s {
            let vn = [v[0] as f32 * 1e-3, v[1] as f32 * 1e-3];
            (Some((vn, sv)), sqrtf(vn[0] * vn[0] + vn[1] * vn[1]))
        } else if let Some(sp) = r.speed_mm_s {
            let s = sp as f32 * 1e-3;
            if s >= self.t.nav_gps_course_min_ms {
                let v = r.course_cdeg.map(|c| {
                    let a = f32::from(c) * 0.01 * PI / 180.0;
                    ([s * cosf(a), s * sinf(a)], sv)
                });
                (v, s)
            } else {
                (Some(([0.0, 0.0], sqrtf(s * s + sv * sv))), s)
            }
        } else {
            (None, 0.0)
        };
        let floor = self.t.nav_gps_pos_sigma_min_m;
        let sigma_ne = [(lat_std as f32 * 1e-3).max(floor), (lon_std as f32 * 1e-3).max(floor)];
        let horizon = self.horizon();
        let rejected_long = self
            .rej_since
            .iter()
            .flatten()
            .any(|&t0| horizon.saturating_sub(t0) >= self.t.nav_aid_timeout_us);
        let plan = if !self.started || rejected_long {
            GpsPlan::Reset
        } else {
            GpsPlan::Fuse { stage: 0, bad: false }
        };
        let mut vel = vel;
        if matches!(plan, GpsPlan::Fuse { .. })
            && vel.is_some()
            && pd_speed.is_some_and(|pd| pd >= self.t.nav_gps_spd_min_ms && h_speed < self.t.nav_gps_spd_ratio * pd)
        {
            self.stats.gps_e3b = self.stats.gps_e3b.wrapping_add(1);
            self.event(code::VN, 0.0, 0.0, code::SCREENED);
            vel = None;
        }
        if self.gps_job.replace(GpsJob { plan, ne, sigma_ne, vel }).is_some() {
            self.stats.replaced = self.stats.replaced.wrapping_add(1);
        }
    }

    fn gps_job(&mut self, job: GpsJob) {
        let horizon = self.horizon();
        match job.plan {
            GpsPlan::Reset => {
                let change = job.ne[0] - self.nom.p[0];
                self.nom.p[0] = job.ne[0];
                self.nom.p[1] = job.ne[1];
                self.reset_state_var(POS, job.sigma_ne[0] * job.sigma_ne[0]);
                self.reset_state_var(POS + 1, job.sigma_ne[1] * job.sigma_ne[1]);
                if let Some((v, _)) = job.vel {
                    let sv = self.t.nav_gps_vel_sigma_ms;
                    self.nom.v[0] = v[0];
                    self.nom.v[1] = v[1];
                    self.reset_state_var(VEL, sv * sv);
                    self.reset_state_var(VEL + 1, sv * sv);
                }
                self.started = true;
                self.rej_since = [None; 2];
                self.last_pos_us = horizon;
                self.latest_ok = true;
                self.stats.gps_resets = self.stats.gps_resets.wrapping_add(1);
                self.event(code::GPS_RESET, change, job.sigma_ne[0] * job.sigma_ne[0], code::FUSED);
                self.gps_job = None;
            }
            GpsPlan::Fuse { stage, bad } => {
                let i = usize::from(stage);
                let (ok, s, innov, src, pos) = if i < 2 {
                    let innov = job.ne[i] - self.nom.p[i];
                    let (ok, s) = self.fuse_unit(POS + i, innov, job.sigma_ne[i] * job.sigma_ne[i]);
                    (ok, s, innov, [code::GPS_N, code::GPS_E][i], true)
                } else if let Some((v, sv)) = job.vel {
                    let innov = v[i - 2] - self.nom.v[i - 2];
                    let (ok, s) = self.fuse_unit(VEL + i - 2, innov, sv * sv);
                    (ok, s, innov, [code::VN, code::VE][i - 2], false)
                } else {
                    (true, 0.0, 0.0, 0, false)
                };
                if i < 2 || job.vel.is_some() {
                    self.event(src, innov, s, if ok { code::FUSED } else { code::GATED });
                }
                if pos {
                    if ok {
                        self.rej_since[i] = None;
                        self.last_pos_us = horizon;
                        self.stats.gps_pos_fused = self.stats.gps_pos_fused.wrapping_add(1);
                    } else {
                        self.rej_since[i].get_or_insert(horizon);
                        self.stats.gps_pos_gated = self.stats.gps_pos_gated.wrapping_add(1);
                    }
                } else if job.vel.is_some() {
                    if ok {
                        self.stats.gps_vel_fused = self.stats.gps_vel_fused.wrapping_add(1);
                    } else {
                        self.stats.gps_vel_gated = self.stats.gps_vel_gated.wrapping_add(1);
                    }
                }
                let bad = bad || !ok;
                let next = stage + 1;
                let last = if job.vel.is_some() { 4 } else { 2 };
                if next >= last {
                    self.latest_ok = !bad;
                    self.gps_job = None;
                } else {
                    self.gps_job = Some(GpsJob { plan: GpsPlan::Fuse { stage: next, bad }, ..job });
                }
            }
        }
    }

    /// The one heavy job of the step, if any: the propagation first, then GNSS, barometer, compass.
    fn one_job(&mut self) -> u32 {
        if self.predict_due {
            self.predict();
        } else if let Some(job) = self.gps_job {
            self.gps_job(job);
        } else if let Some(job) = self.baro_job {
            self.baro_job(job);
        } else if let Some(m) = self.mag_job.take() {
            self.mag_job(m);
        } else {
            return 0;
        }
        1
    }

    fn finite(&self) -> bool {
        let n = &self.nom;
        n.q.iter().chain(&n.v).chain(&n.p).chain(&n.bg).chain(&n.ba).all(|v| v.is_finite())
            && self.p.iter().flatten().all(|v| v.is_finite())
    }

    fn nav(&self) -> NavState {
        let n = &self.nom;
        let horizon = self.horizon();
        let al = self.tilt_aligned;
        let aid_to = self.t.nav_aid_timeout_us;
        let vert = al && self.last_baro_aid_us.is_some_and(|t0| horizon.saturating_sub(t0) < aid_to);
        let heading = self.armed
            && al
            && self.yaw_aligned
            && sqrtf(self.p[ATT + 2][ATT + 2].max(0.0)) <= self.t.nav_heading_valid_sigma_rad
            && horizon.saturating_sub(self.last_mag_acc_us) < aid_to;
        let ne = self.armed
            && al
            && self.yaw_aligned
            && self.origin.is_some()
            && self.started
            && horizon.saturating_sub(self.last_pos_us) < aid_to
            && self.latest_ok;
        let q = if n.q[0] < 0.0 { n.q.map(|x| -x) } else { n.q };
        // Variances from the diagonal floored at 0.
        let v = |i: usize| self.p[i][i].max(0.0);
        let sigma = if al {
            NavSigma {
                tilt_rad: sqrtf(v(ATT) + v(ATT + 1)),
                heading_rad: sqrtf(v(ATT + 2)),
                velocity_ne_ms: sqrtf(v(VEL) + v(VEL + 1)),
                velocity_d_ms: sqrtf(v(VEL + 2)),
                position_ne_m: sqrtf(v(POS) + v(POS + 1)),
                position_d_m: sqrtf(v(POS + 2)),
            }
        } else {
            NavSigma {
                tilt_rad: f32::INFINITY,
                heading_rad: f32::INFINITY,
                velocity_ne_ms: f32::INFINITY,
                velocity_d_ms: f32::INFINITY,
                position_ne_m: f32::INFINITY,
                position_d_m: f32::INFINITY,
            }
        };
        let g = self.horizon_gyro;
        NavState {
            t_us: horizon,
            attitude: q,
            rates_rads: [g[0] - n.bg[0], g[1] - n.bg[1], g[2] - n.bg[2]],
            velocity_ms: n.v,
            position_m: n.p,
            valid: Valid {
                tilt: al,
                heading,
                rates: self.horizon_us.is_some(),
                velocity_ne: ne,
                velocity_d: vert,
                position_ne: ne,
                position_d: vert,
            },
            sigma: Some(sigma),
        }
    }
}

impl Estimator for Eskf {
    fn update(&mut self, sensors: &Sensors) -> NavState {
        self.events = [None; EVENTS];
        let now = sensors.imu1.t_us;
        let r = &sensors.imu1.reading;
        // The ring: the sample in (a full ring first gives its oldest sample to the horizon), then the horizon advanced
        // to now - nav_gps_delay_us.
        if r.gyro_rads.iter().chain(&r.accel_ms2).all(|v| v.is_finite()) {
            let e = ImuEntry { t_us: now, gyro: r.gyro_rads, accel: r.accel_ms2 };
            if let Some(old) = self.ring.push(e) {
                self.integrate(old);
            }
        }
        while let Some(e) = self.ring.front() {
            if e.t_us.saturating_add(self.t.nav_gps_delay_us) > now {
                break;
            }
            self.ring.pop();
            self.integrate(e);
        }
        // New slow samples: barometer and compass wait for the horizon; a GNSS epoch is due on arrival. A sample whose
        // time is older than the ring's span is dropped and counted.
        let buffer_us = self.t.nav_buffer_us;
        let stale = |t_us: u64| t_us.saturating_add(buffer_us) < now;
        if let Some(b) = sensors.baro.filter(|b| b.reading.pressure_pa.is_finite() && b.reading.pressure_pa > 0.0) {
            let w = BaroWait { t_valid_us: b.t_us.saturating_sub(self.t.nav_baro_delay_us), pressure_pa: b.reading.pressure_pa };
            if stale(w.t_valid_us) {
                self.stats.stale = self.stats.stale.wrapping_add(1);
            } else if self.baro_wait.push(w).is_some() {
                self.stats.dropped = self.stats.dropped.wrapping_add(1);
            }
        }
        if let Some(m) = sensors.mag.filter(|m| {
            let old = stale(m.t_us);
            if old {
                self.stats.stale = self.stats.stale.wrapping_add(1);
            }
            !old
        }) {
            if m.expected.is_some() {
                self.mag_expected = m.expected;
            }
            let w = MagWait { t_us: m.t_us, field_ut: m.reading.field_ut, expected: m.expected.is_some() };
            // Aligned, a sample before the next fusion slot is skipped at the horizon anyway (the slot only moves later).
            let skipped = self.yaw_aligned && m.t_us < self.next_mag_us;
            if !skipped && self.mag_wait.push(w).is_some() {
                self.stats.dropped = self.stats.dropped.wrapping_add(1);
            }
        }
        if let Some(g) = sensors.gnss {
            if stale(g.t_us.saturating_sub(self.t.nav_gps_delay_us)) {
                self.stats.stale = self.stats.stale.wrapping_add(1);
            } else {
                self.gnss_arrival(&g);
            }
        }
        let horizon = self.horizon();
        while let Some(b) = self.baro_wait.front() {
            if self.horizon_us.is_none() || b.t_valid_us > horizon {
                break;
            }
            self.baro_wait.pop();
            if !self.tilt_aligned {
                continue;
            }
            let h_abs = isa_height_m(b.pressure_pa);
            let datum = *self.datum_m.get_or_insert(h_abs);
            if self.baro_job.replace(BaroJob { h_b: h_abs - datum, stage: 0 }).is_some() {
                self.stats.replaced = self.stats.replaced.wrapping_add(1);
            }
        }
        while let Some(m) = self.mag_wait.front() {
            if self.horizon_us.is_none() || m.t_us > horizon {
                break;
            }
            self.mag_wait.pop();
            self.mag_due(m);
        }
        // The no-aid hold position: where the filter is when horizontal aiding stops.
        let aiding = self.aiding();
        if self.aiding_prev && !aiding {
            self.hold_ne = [self.nom.p[0], self.nom.p[1]];
        }
        self.aiding_prev = aiding;
        let jobs = self.one_job();
        self.stats.max_jobs_per_step = self.stats.max_jobs_per_step.max(jobs);
        if self.tilt_aligned && !self.finite() {
            self.reinitialise();
        }
        self.nav()
    }

    fn origin(&self) -> Option<GeoPoint> {
        self.origin
    }

    fn set_armed(&mut self, armed: bool) {
        if armed != self.armed {
            // GNSS only while armed; the first accepted epoch after arming resets the horizontal state (C-1).
            self.started = false;
            self.gps_job = None;
            self.rej_since = [None; 2];
            if armed {
                self.fix_hist = Fifo::new();
            }
        }
        self.armed = armed;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::imu::{ImuReading, ImuSample};
    use crate::mahony_baro::{ISA_GAS_CONSTANT, ISA_LAPSE_K_M, ISA_P0_PA, ISA_T0_K};
    use fw_core::sample::{BaroReading, BaroSample, GnssReading, MagReading, MagSample};
    use std::collections::VecDeque;
    use std::println;
    use std::vec::Vec;

    /// IMU period, us: 800 Hz (decision-sim-conventions 6).
    const PERIOD_US: u64 = 1_250;
    /// Barometer 50 Hz, compass 100 Hz, GNSS 5 Hz (section 3b, 4a; the board's GPS_RATE_MS 200).
    const BARO_EVERY: u64 = 16;
    const MAG_EVERY: u64 = 8;
    const GNSS_EVERY: u64 = 160;
    /// A synthetic site, not anyone's place: next to the 180 deg meridian so east motion crosses the wrap.
    const ORIGIN: GeoPoint = GeoPoint { lat_e7: 100_000_000, lon_e7: 1_799_999_000, alt_mm: 100_000 };
    /// The synthetic field: 50 uT, 60 deg down; tolerances of the stored defaults MAG_HLT_STR 10 %, MAG_HLT_INC 10 deg.
    const STRENGTH_UT: f64 = 50.0;
    const INC_DEG: f64 = 60.0;
    /// B3: the test-only sigma honesty check.
    const SIGMAS: f32 = 3.0;

    fn expected() -> MagExpected {
        MagExpected {
            strength_ut: STRENGTH_UT as f32,
            inclination_rad: (INC_DEG as f32).to_radians(),
            tol_strength_frac: 0.10,
            tol_inclination_rad: 10.0_f32.to_radians(),
        }
    }

    fn pressure_at(height_m: f64) -> f32 {
        let e = f64::from(GRAVITY_MSS) / (f64::from(ISA_LAPSE_K_M) * f64::from(ISA_GAS_CONSTANT));
        let t0 = f64::from(ISA_T0_K);
        (f64::from(ISA_P0_PA) * ((t0 - f64::from(ISA_LAPSE_K_M) * height_m) / t0).powf(e)) as f32
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    enum Fault {
        None,
        /// From this arrival time to the next, the fix repeats the last one and speed reads 0 (the receiver's hold).
        Freeze(u64, u64),
        /// From this arrival time on, north is off by this many metres.
        North(u64, f64),
    }

    /// A level vehicle with a yaw, flying a constant NED acceleration; perfect sensors at the delays of `Tuning`.
    struct World {
        k: u64,
        t: Tuning,
        p: [f64; 3],
        v: [f64; 3],
        a: [f64; 3],
        yaw: f64,
        yaw_rate: f64,
        hist: VecDeque<(u64, [f64; 3], [f64; 3], f64)>,
        expected: Option<MagExpected>,
        /// Compass turned about body z by this, rad (passes the health test).
        mag_rot: f64,
        /// A disturbed field: strength factor and inclination, deg.
        mag_field: (f64, f64),
        gps_every: u64,
        gps_vel: bool,
        std_mm: u32,
        fault: Fault,
        last_fix: Option<GnssReading>,
    }

    impl World {
        fn new(t: &Tuning) -> World {
            World {
                k: 0,
                t: *t,
                p: [0.0; 3],
                v: [0.0; 3],
                a: [0.0; 3],
                yaw: 0.0,
                yaw_rate: 0.0,
                hist: VecDeque::new(),
                expected: Some(expected()),
                mag_rot: 0.0,
                mag_field: (1.0, INC_DEG),
                gps_every: GNSS_EVERY,
                gps_vel: false,
                std_mm: 300,
                fault: Fault::None,
                last_fix: None,
            }
        }

        fn t_us(&self) -> u64 {
            self.k * PERIOD_US
        }

        /// Truth (p, v, yaw) at `t_us` (nearest kept step).
        fn at(&self, t_us: u64) -> ([f64; 3], [f64; 3], f64) {
            let e = self.hist.iter().rev().find(|h| h.0 <= t_us).or(self.hist.front()).copied().unwrap();
            (e.1, e.2, e.3)
        }

        fn step(&mut self) -> Sensors {
            self.k += 1;
            let dt = PERIOD_US as f64 * 1e-6;
            for i in 0..3 {
                self.p[i] += self.v[i] * dt + 0.5 * self.a[i] * dt * dt;
                self.v[i] += self.a[i] * dt;
            }
            self.yaw += self.yaw_rate * dt;
            let t_us = self.t_us();
            self.hist.push_back((t_us, self.p, self.v, self.yaw));
            while self.hist.len() > 1_000 {
                self.hist.pop_front();
            }
            let (c, s) = (self.yaw.cos(), self.yaw.sin());
            let g = f64::from(GRAVITY_MSS);
            let a = self.a;
            let accel = [c * a[0] + s * a[1], -s * a[0] + c * a[1], a[2] - g].map(|x| x as f32);
            let seq = self.k as u32;
            let imu1 = ImuSample {
                reading: ImuReading { gyro_rads: [0.0, 0.0, self.yaw_rate as f32], accel_ms2: accel, temperature_c: 0.0 },
                t_us,
                seq,
            };
            let baro = self.k.is_multiple_of(BARO_EVERY).then(|| {
                let (p, _, _) = self.at(t_us.saturating_sub(self.t.nav_baro_delay_us));
                BaroSample { reading: BaroReading { pressure_pa: pressure_at(-p[2]), temperature_c: 15.0 }, t_us, seq }
            });
            let mag = self.k.is_multiple_of(MAG_EVERY).then(|| {
                let dec = f64::from(self.t.nav_mag_declination_rad);
                let (sf, inc) = (self.mag_field.0, self.mag_field.1.to_radians());
                let m = [inc.cos() * dec.cos(), inc.cos() * dec.sin(), inc.sin()].map(|x| x * STRENGTH_UT * sf);
                let y = self.yaw + self.mag_rot;
                let (c, s) = (y.cos(), y.sin());
                let mb = [c * m[0] + s * m[1], -s * m[0] + c * m[1], m[2]].map(|x| x as f32);
                MagSample { reading: MagReading { field_ut: mb, temperature_c: 0.0 }, t_us, seq, expected: self.expected }
            });
            let gnss = self.k.is_multiple_of(self.gps_every).then(|| {
                let (mut p, v, _) = self.at(t_us.saturating_sub(self.t.nav_gps_delay_us));
                if let Fault::North(from, m) = self.fault {
                    if t_us >= from {
                        p[0] += m;
                    }
                }
                let (lat, lon) = geo(p);
                // One unit (about 1 cm) of dither on every other epoch: a perfect receiver at rest would repeat its fix
                // exactly, which E1s screens while armed.
                let lat = lat + (self.k / self.gps_every % 2) as i32;
                let speed = v[0].hypot(v[1]);
                let course = (v[1].atan2(v[0]).to_degrees().rem_euclid(360.0) * 100.0) as u16;
                let mut r = GnssReading {
                    fix_type: 3,
                    lat_e7: Some(lat),
                    lon_e7: Some(lon),
                    alt_mm: Some(ORIGIN.alt_mm - (p[2] * 1e3) as i32),
                    speed_mm_s: Some((speed * 1e3) as u32),
                    course_cdeg: Some(course % 36_000),
                    sats_used: Some(12),
                    hdop_x100: Some(80),
                    vdop_x100: Some(120),
                    time: None,
                    date: None,
                    lat_std_mm: Some(self.std_mm),
                    lon_std_mm: Some(self.std_mm),
                    alt_std_mm: Some(self.std_mm),
                    vel_ned_mm_s: self.gps_vel.then(|| v.map(|x| (x * 1e3) as i32)),
                };
                if let (Fault::Freeze(from, to), Some(last)) = (self.fault, self.last_fix) {
                    if t_us >= from && t_us < to {
                        r.lat_e7 = last.lat_e7;
                        r.lon_e7 = last.lon_e7;
                        r.speed_mm_s = Some(0);
                        r.vel_ned_mm_s = r.vel_ned_mm_s.map(|_| [0; 3]);
                    }
                }
                self.last_fix = Some(r);
                GnssSample { reading: r, t_us, seq }
            });
            Sensors { imu1, imu2: None, baro, mag, gnss, external: None }
        }
    }

    /// Lat, lon (1e-7 deg) of NED `p` from ORIGIN: the inverse of `ned_from_geo`, f64.
    fn geo(p: [f64; 3]) -> (i32, i32) {
        let f = f64::from(WGS84_F);
        let a = f64::from(WGS84_A_M);
        let e2 = f * (2.0 - f);
        let lat0 = (f64::from(ORIGIN.lat_e7) * 1e-7).to_radians();
        let w = 1.0 - e2 * lat0.sin().powi(2);
        let (rm, rn) = (a * (1.0 - e2) / (w * w.sqrt()), a / w.sqrt());
        let lat = i64::from(ORIGIN.lat_e7) + ((p[0] / rm).to_degrees() * 1e7).round() as i64;
        let lon = i64::from(ORIGIN.lon_e7) + ((p[1] / (rn * lat0.cos())).to_degrees() * 1e7).round() as i64;
        (lat as i32, ((lon + E7_TURN / 2).rem_euclid(E7_TURN) - E7_TURN / 2) as i32)
    }

    /// Runs `secs` of steps; `armed(t_s)`; `each` sees the filter, its output and the world after every step.
    fn run(e: &mut Eskf, w: &mut World, secs: f64, armed: impl Fn(f64) -> bool, mut each: impl FnMut(&Eskf, &NavState, &World)) {
        for _ in 0..(secs * 1e6) as u64 / PERIOD_US {
            let s = w.step();
            e.set_armed(armed(w.t_us() as f64 * 1e-6));
            let nav = e.update(&s);
            each(e, &nav, w);
        }
    }

    fn yaw_of(q: &[f32; 4]) -> f64 {
        let r = rotation(q);
        f64::from(r[1][0]).atan2(f64::from(r[0][0]))
    }

    #[test]
    fn new_refuses_a_delay_beyond_the_buffer() {
        let t = Tuning { nav_gps_delay_us: Tuning::DEFAULT.nav_buffer_us + 1, ..Tuning::DEFAULT };
        assert_eq!(Eskf::new(&t).err(), Some(EskfError::DelayBeyondBuffer));
        assert!(Eskf::new(&Tuning::DEFAULT).is_ok());
    }

    #[test]
    fn o1_ned_from_geo_across_the_wrap_and_by_hand() {
        // On the equator at the 180 deg meridian: 0.0002 deg of longitude across the wrap and 0.001 deg of latitude.
        let o = GeoPoint { lat_e7: 0, lon_e7: 1_799_999_000, alt_mm: 0 };
        let p = GeoPoint { lat_e7: 10_000, lon_e7: -1_799_999_000, alt_mm: -2_500 };
        let ned = ned_from_geo(&o, &p);
        // By hand: east = a * 0.0002 deg in rad; north = a (1 - e2) * 0.001 deg in rad; down 2.5 m.
        let a = 6_378_137.0_f64;
        let e2 = (1.0 / 298.257_223_563) * (2.0 - 1.0 / 298.257_223_563);
        let east = a * 0.0002_f64.to_radians();
        let north = a * (1.0 - e2) * 0.001_f64.to_radians();
        println!("o1: ned {ned:?}, by hand [{north:.4}, {east:.4}, 2.5]");
        assert!((f64::from(ned[0]) - north).abs() < 0.01 && (f64::from(ned[1]) - east).abs() < 0.01);
        assert_eq!(ned[2], 2.5);
        let back = ned_from_geo(&p, &o);
        assert!((back[1] + ned[1]).abs() < 0.01, "{back:?}");
    }

    #[test]
    fn c11_health_boundaries_h_against_finite_differences_of_the_innovation_and_a_known_yaw_recovery() {
        let e = expected();
        let inc = e.inclination_rad;
        let field = |sf: f32, inc: f32| [sf * e.strength_ut * cosf(inc), 0.0, sf * e.strength_ut * sinf(inc)];
        let d = 1e-3;
        assert!(healthy(&field(1.0 + e.tol_strength_frac - d, inc), &e));
        assert!(!healthy(&field(1.0 + e.tol_strength_frac + d, inc), &e));
        assert!(healthy(&field(1.0 - e.tol_strength_frac + d, inc), &e));
        assert!(!healthy(&field(1.0 - e.tol_strength_frac - d, inc), &e));
        assert!(healthy(&field(1.0, inc + e.tol_inclination_rad - d), &e));
        assert!(!healthy(&field(1.0, inc + e.tol_inclination_rad + d), &e));
        assert!(!healthy(&field(1.0, inc - e.tol_inclination_rad - d), &e));
        // H (review ruling 1): against central differences of the innovation actually used, yaw_innov, under the error
        // composed on the left (q <- [1, dθ/2] q): H_j = -(y(+eps e_j) - y(-eps e_j)) / (2 eps). Inclinations 0, 60 and
        // 75 deg; tilts 0 and 30 deg (roll, pitch and both); eight yaws; a 3 deg attitude error about a generic axis so
        // that the innovation is not zero. eps 1e-3 rad: the f32 rounding of y (about 2e-7) gives about 2e-4 of noise,
        // the truncation (eps^2 / 6 times the third derivative) below 1e-4 at 75 deg; tolerance 2e-3 max(1, |H_j|).
        let dec = Tuning::DEFAULT.nav_mag_declination_rad;
        let axis_q = |ax: usize, ang: f32| {
            let mut q = [cosf(0.5 * ang), 0.0, 0.0, 0.0];
            q[ax + 1] = sinf(0.5 * ang);
            q
        };
        let (mut worst, mut n, mut h60) = (0.0f32, 0, None);
        for inc_deg in [0.0f32, 60.0, 75.0] {
            let inc = inc_deg.to_radians();
            let m_ned = [cosf(inc) * cosf(dec), cosf(inc) * sinf(dec), sinf(inc)].map(|x| x * e.strength_ut);
            for (roll, pitch) in [(0.0f32, 0.0f32), (30.0, 0.0), (0.0, 30.0), (-30.0, 0.0), (21.0, -21.0)] {
                for k in 0..8 {
                    let yaw = (k as f32 * 45.0 + 10.0).to_radians();
                    let q_true = qmul(&qmul(&axis_q(2, yaw), &axis_q(1, pitch.to_radians())), &axis_q(0, roll.to_radians()));
                    let r = rotation(&q_true);
                    let m_b: [f32; 3] = core::array::from_fn(|i| (0..3).map(|j| r[j][i] * m_ned[j]).sum());
                    let err = 3.0f32.to_radians();
                    let q = normalized(qmul(&[cosf(0.5 * err), 0.6 * sinf(0.5 * err), -0.48 * sinf(0.5 * err), 0.64 * sinf(0.5 * err)], &q_true));
                    let h = yaw_h(&q, &m_b).expect("a horizontal field");
                    if inc_deg == 60.0 && roll == 0.0 && pitch == 0.0 && k == 0 {
                        h60 = Some(yaw_h(&q_true, &m_b).unwrap());
                    }
                    for j in 0..3 {
                        let eps = 1e-3f32;
                        let mut dq = [1.0, 0.0, 0.0, 0.0];
                        dq[j + 1] = 0.5 * eps;
                        let up = yaw_innov(&normalized(qmul(&dq, &q)), &m_b, dec);
                        dq[j + 1] = -0.5 * eps;
                        let dn = yaw_innov(&normalized(qmul(&dq, &q)), &m_b, dec);
                        let fd = -wrap(up - dn) / (2.0 * eps);
                        worst = worst.max(fabsf(fd - h[j]) / fabsf(h[j]).max(1.0));
                        n += 1;
                    }
                }
            }
        }
        println!("c11: H against finite differences of the innovation, {n} entries, worst relative difference {worst:e}; H at 60 deg, level {h60:?}");
        assert!(worst < 2e-3, "H differs by {worst}");
        // The guard: a vertical field has no heading.
        assert_eq!(yaw_h(&[1.0, 0.0, 0.0, 0.0], &[0.0, 0.0, 50.0]), None);
        // Known yaw: a vehicle at 40 deg aligns to it.
        let t = Tuning::DEFAULT;
        let mut f = Eskf::new(&t).unwrap();
        let mut w = World::new(&t);
        w.yaw = 40.0_f64.to_radians();
        let mut err = None;
        run(&mut f, &mut w, 3.0, |_| false, |f, nav, w| {
            if f.yaw_aligned() {
                err = Some(wrap((yaw_of(&nav.attitude) - w.at(nav.t_us).2) as f32).to_degrees());
            }
        });
        println!("c11: known-yaw recovery, error {err:?} deg");
        assert!(err.unwrap().abs() < 0.1);
    }

    #[test]
    fn c12_b4_a_burst_still_gives_one_heavy_job_per_step_and_the_queue_drains() {
        let t = Tuning::DEFAULT;
        let mut f = Eskf::new(&t).unwrap();
        let mut w = World::new(&t);
        w.v = [1.0, 0.5, 0.0];
        run(&mut f, &mut w, 10.0, |_| true, |_, _, _| {});
        assert!(f.stats().gps_pos_fused > 0 && f.stats().mag_fused > 0 && f.stats().baro_fused > 0, "{:?}", f.stats());
        let before = *f.stats();
        // Every slow sensor on every step for 2 s.
        let mut max_queue = 0;
        for _ in 0..2_000_000 / PERIOD_US {
            let mut s = w.step();
            let k = w.k;
            let (p, _, _) = w.at(w.t_us() - t.nav_gps_delay_us);
            let (lat, lon) = geo(p);
            s.baro = Some(BaroSample { reading: BaroReading { pressure_pa: pressure_at(-p[2]), temperature_c: 15.0 }, t_us: w.t_us(), seq: k as u32 });
            if s.mag.is_none() {
                let mut m = w.clone_mag();
                m.t_us = w.t_us();
                s.mag = Some(m);
            }
            if s.gnss.is_none() {
                let mut r = w.last_fix.unwrap();
                r.lat_e7 = Some(lat);
                r.lon_e7 = Some(lon);
                s.gnss = Some(GnssSample { reading: r, t_us: w.t_us(), seq: k as u32 });
            }
            f.set_armed(true);
            f.update(&s);
            let q = usize::from(f.gps_job.is_some()) + usize::from(f.baro_job.is_some()) + usize::from(f.mag_job.is_some());
            max_queue = max_queue.max(q);
        }
        assert_eq!(f.stats().max_jobs_per_step, 1, "I2: more than one heavy job in a step");
        // Quiet: the queue and the waiting samples (up to nav_gps_delay_us of them) drain.
        let mut drained = None;
        let limit = (t.nav_gps_delay_us / PERIOD_US) as usize + 16;
        for i in 0..limit + 16 {
            let mut s = w.step();
            s.baro = None;
            s.mag = None;
            s.gnss = None;
            f.update(&s);
            let empty = f.baro_wait.len == 0 && f.mag_wait.len == 0;
            if drained.is_none() && empty && f.gps_job.is_none() && f.baro_job.is_none() && f.mag_job.is_none() {
                drained = Some(i + 1);
            }
        }
        println!(
            "c12: burst: queue depth max {max_queue} kinds, replaced {} (normal rates: {}), drained {drained:?} steps after",
            f.stats().replaced,
            before.replaced
        );
        assert_eq!(before.replaced, 0, "at normal rates no job is replaced unprocessed");
        assert!(drained.is_some_and(|n| n <= limit), "queue did not drain: {drained:?}");
        assert_eq!(f.stats().max_jobs_per_step, 1);
    }

    impl World {
        fn clone_mag(&self) -> MagSample {
            let dec = f64::from(self.t.nav_mag_declination_rad);
            let inc = INC_DEG.to_radians();
            let m = [inc.cos() * dec.cos(), inc.cos() * dec.sin(), inc.sin()].map(|x| x * STRENGTH_UT);
            let (c, s) = (self.yaw.cos(), self.yaw.sin());
            let mb = [c * m[0] + s * m[1], -s * m[0] + c * m[1], m[2]].map(|x| x as f32);
            MagSample { reading: MagReading { field_ut: mb, temperature_c: 0.0 }, t_us: 0, seq: 0, expected: self.expected }
        }
    }

    #[test]
    fn c3_without_the_expected_field_heading_and_ne_are_never_valid_and_gps_never_fused() {
        let t = Tuning::DEFAULT;
        let mut f = Eskf::new(&t).unwrap();
        let mut w = World::new(&t);
        w.expected = None;
        w.v = [2.0, 0.0, 0.0];
        let mut bad = 0;
        run(&mut f, &mut w, 30.0, |s| s > 5.0, |_, nav, _| {
            bad += usize::from(nav.valid.heading || nav.valid.velocity_ne || nav.valid.position_ne);
        });
        let s = f.stats();
        println!("c3: {s:?}");
        assert_eq!(bad, 0);
        assert_eq!((s.gps_pos_fused, s.gps_vel_fused, s.gps_resets, s.yaw_aligns), (0, 0, 0, 0));
        assert!(s.gps_prealign > 100);
    }

    #[test]
    fn c5_no_gps_before_alignment_none_disarmed_and_src9_at_the_first_epoch_after_arming() {
        let t = Tuning::DEFAULT;
        let mut f = Eskf::new(&t).unwrap();
        let mut w = World::new(&t);
        w.expected = None;
        w.v = [1.0, 0.0, 0.0];
        // Armed from 2 s, no healthy compass until 10 s.
        run(&mut f, &mut w, 10.0, |s| s > 2.0, |_, _, _| {});
        let s0 = *f.stats();
        assert!(s0.gps_prealign > 0 && s0.gps_pos_fused == 0 && s0.gps_resets == 0, "{s0:?}");
        w.expected = Some(expected());
        let mut first_gps_event = None;
        run(&mut f, &mut w, 5.0, |s| s > 2.0, |f, _, _| {
            if first_gps_event.is_none() {
                first_gps_event = f.events().find(|e| e.src <= code::VE || e.src == code::GPS_RESET).copied();
            }
        });
        println!("c5: before alignment {s0:?}; first GNSS event after it {first_gps_event:?}");
        assert_eq!(first_gps_event.map(|e| e.src), Some(code::GPS_RESET));
        // Disarmed: nothing fused, and the next arming resets again.
        let s1 = *f.stats();
        run(&mut f, &mut w, 3.0, |_| false, |_, nav, _| assert!(!nav.valid.position_ne && !nav.valid.heading));
        let s2 = *f.stats();
        assert_eq!((s2.gps_pos_fused, s2.gps_vel_fused), (s1.gps_pos_fused, s1.gps_vel_fused));
        assert!(s2.gps_disarmed > s1.gps_disarmed);
        run(&mut f, &mut w, 1.0, |_| true, |_, _, _| {});
        assert_eq!(f.stats().gps_resets, s2.gps_resets + 1);
    }

    #[test]
    fn i4_a_1_hz_stream_is_refused_at_the_default_and_fused_after_lowering_the_rate() {
        let fused = |t: &Tuning| {
            let mut f = Eskf::new(t).unwrap();
            let mut w = World::new(t);
            w.gps_every = 800;
            run(&mut f, &mut w, 20.0, |_| true, |_, _, _| {});
            *f.stats()
        };
        let d = fused(&Tuning::DEFAULT);
        let low = fused(&Tuning { nav_gps_min_rate_hz: 0.9, ..Tuning::DEFAULT });
        println!("i4: default {} fused / {} unaccepted; lowered {} fused", d.gps_pos_fused, d.gps_unaccepted, low.gps_pos_fused);
        assert_eq!(d.gps_pos_fused + d.gps_resets, 0);
        assert!(low.gps_pos_fused > 0 && low.gps_resets == 1);
    }

    /// Armed from the start, moving north at 2 m/s; returns the filter and the world after `secs`.
    fn flying(t: &Tuning, secs: f64, v: [f64; 3]) -> (Eskf, World) {
        let mut f = Eskf::new(t).unwrap();
        let mut w = World::new(t);
        w.v = v;
        run(&mut f, &mut w, secs, |_| true, |_, _, _| {});
        (f, w)
    }

    #[test]
    fn c6_a_frozen_fix_armed_is_screened_ne_drops_at_the_timeout_and_returns_after_a_fusion_or_reset() {
        let t = Tuning::DEFAULT;
        let (mut f, mut w) = flying(&t, 20.0, [2.0, 0.0, 0.0]);
        let (from, to) = (w.t_us(), w.t_us() + 10_000_000);
        w.fault = Fault::Freeze(from, to);
        let s0 = *f.stats();
        let mut drop = None;
        let mut last_valid = None;
        run(&mut f, &mut w, 10.0, |_| true, |_, nav, _| {
            if nav.valid.position_ne {
                last_valid = Some(nav.t_us);
            } else if drop.is_none() {
                drop = Some(nav.t_us);
            }
        });
        let s1 = *f.stats();
        // R1 (lead, 2026-11-21): C3 governs. After the hold the next fresh epoch is fused if it passes the gate; if it is
        // gated, the C3 reset follows after the timeout. Either way the flags return only after an accepted position
        // fusion or a reset.
        let (mut fused_or_reset, mut outcome, mut back) = (false, None, None);
        run(&mut f, &mut w, 8.0, |_| true, |f, nav, _| {
            for e in f.events() {
                let pos = e.src == code::GPS_N || e.src == code::GPS_E;
                if outcome.is_none() && (pos || e.src == code::GPS_RESET) && e.ok != code::SCREENED {
                    outcome = Some(match (e.src, e.ok) {
                        (code::GPS_RESET, _) => "reset",
                        (_, code::FUSED) => "fused",
                        _ => "gated (C3 reset after the timeout)",
                    });
                }
                fused_or_reset |= (pos && e.ok == code::FUSED) || e.src == code::GPS_RESET;
            }
            if back.is_none() && nav.valid.position_ne {
                back = Some((nav.t_us, fused_or_reset));
            }
        });
        let horizon_from = from - t.nav_gps_delay_us;
        println!(
            "c6: screened {} epochs; NE dropped {:?} s after the hold began (horizon); the first fresh epoch: {outcome:?}; \
             NE back {:?} s after the hold began; stats {:?}",
            s1.gps_e1s - s0.gps_e1s,
            drop.map(|d| (d - horizon_from) as f64 * 1e-6),
            back.map(|b| (b.0 - horizon_from) as f64 * 1e-6),
            f.stats()
        );
        assert_eq!(s1.gps_e1s - s0.gps_e1s, 49, "every frozen epoch but the first (which repeats a moving fix) screened");
        let d = drop.expect("NE dropped") - horizon_from;
        assert!(d >= t.nav_aid_timeout_us - GNSS_EVERY * PERIOD_US && d <= t.nav_aid_timeout_us + GNSS_EVERY * PERIOD_US, "dropped at {d} us");
        assert!(last_valid.is_some_and(|v| v < from - t.nav_gps_delay_us + t.nav_aid_timeout_us + GNSS_EVERY * PERIOD_US));
        let (_, after_fusion) = back.expect("NE returns");
        assert!(after_fusion, "NE returned without an accepted fusion or a reset");
        assert!(outcome.is_some());
    }

    #[test]
    fn c6_disarmed_repeats_are_not_screened() {
        // C-1 adaptation: disarmed GNSS is never fused, so a repeat at rest disarmed is neither screened nor fused.
        let t = Tuning::DEFAULT;
        let mut f = Eskf::new(&t).unwrap();
        let mut w = World::new(&t);
        w.fault = Fault::Freeze(2_000_000, 12_000_000);
        run(&mut f, &mut w, 12.0, |_| false, |_, _, _| {});
        let s = f.stats();
        assert_eq!((s.gps_e1s, s.gps_pos_fused), (0, 0), "{s:?}");
        assert!(s.gps_disarmed > 40);
    }

    #[test]
    fn c7_a_one_axis_fault_of_ten_sigma_held_resets_at_the_timeout() {
        let t = Tuning::DEFAULT;
        let (mut f, mut w) = flying(&t, 10.0, [0.0; 3]);
        let sigma = (f64::from(w.std_mm) * 1e-3).max(f64::from(t.nav_gps_pos_sigma_min_m));
        w.fault = Fault::North(w.t_us(), 10.0 * sigma);
        let mut first_rej = None;
        let mut reset = None;
        run(&mut f, &mut w, 7.0, |_| true, |f, nav, _| {
            for e in f.events() {
                if e.src == code::GPS_N && e.ok == code::GATED && first_rej.is_none() {
                    first_rej = Some(nav.t_us);
                }
                if e.src == code::GPS_RESET && reset.is_none() {
                    reset = Some(nav.t_us);
                }
            }
        });
        let (a, b) = (first_rej.expect("rejected"), reset.expect("reset"));
        println!("c7: first rejection to reset {} s; east gated {}", (b - a) as f64 * 1e-6, f.stats().gps_pos_gated);
        assert!(b - a >= t.nav_aid_timeout_us && b - a <= t.nav_aid_timeout_us + GNSS_EVERY * PERIOD_US);
        let (p, _, _) = w.at(f.horizon());
        assert!((f64::from(f.nom.p[0]) - (p[0] + 10.0 * sigma)).abs() < 3.0 * sigma, "position follows the faulted fix");
    }

    #[test]
    fn c8_a_compass_turned_90_deg_after_alignment_resets_the_yaw_at_the_timeout_with_the_biases() {
        let t = Tuning::DEFAULT;
        // 10 s after the alignment the yaw sigma still grows by the re-initialised z gyro bias sigma (P0_bg) between the
        // 1 Hz fusions, and the 5-sigma gate reopens after 4 gated samples (measured: 4 gated, then fused). After 60 s
        // the bias has converged.
        let mut out = std::string::String::new();
        for secs in [10.0, 60.0] {
            let (mut f, mut w) = flying(&t, secs, [0.0; 3]);
            w.mag_rot = 90.0_f64.to_radians();
            let sig = (f.sigmas()[ATT + 2], f.sigmas()[GYRO_BIAS + 2]);
            run(&mut f, &mut w, 8.0, |_| true, |_, _, _| {});
            out += &std::format!(" after {secs} s: yaw sigma {:.3} rad, bg_z sigma {:.4} rad/s, gated {}, resets {};", sig.0, sig.1, f.stats().mag_gated, f.stats().yaw_resets);
        }
        println!("c8:{out}");
        // I3 (report only): the compass turned 90 deg 10 s after the alignment, followed for 40 s.
        let (mut f, mut w) = flying(&t, 10.0, [0.0; 3]);
        w.mag_rot = 90.0_f64.to_radians();
        let t0 = f.horizon();
        let (mut follow, mut valid_wrong, mut first_invalid, mut resets_at) = (None, 0.0f64, None, Vec::new());
        run(&mut f, &mut w, 40.0, |_| true, |f, nav, w| {
            let truth = w.at(nav.t_us).2;
            let est = yaw_of(&nav.attitude);
            let to_compass = wrap((est - truth - 90f64.to_radians()) as f32).abs().to_degrees();
            let to_truth = wrap((est - truth) as f32).abs().to_degrees();
            let s = (nav.t_us.saturating_sub(t0)) as f64 * 1e-6;
            if follow.is_none() && to_compass < 15.0 {
                follow = Some(s);
            }
            if nav.valid.heading && to_truth > 15.0 {
                valid_wrong += PERIOD_US as f64 * 1e-6;
            }
            if first_invalid.is_none() && !nav.valid.heading {
                first_invalid = Some(s);
            }
            if f.events().any(|e| e.src == code::YAW_RESET) {
                resets_at.push(s);
            }
        });
        println!(
            "c8/I3: turned 10 s after alignment: yaw within 15 deg of the turned compass after {follow:?} s; yaw resets at \
             {resets_at:?} s; gated {} fused {}; heading first invalid at {first_invalid:?} s; heading valid while more than 15 deg \
             from truth {valid_wrong:.2} s (of 40 s)",
            f.stats().mag_gated,
            f.stats().mag_fused
        );
        let (mut f, mut w) = flying(&t, 60.0, [0.0; 3]);
        w.mag_rot = 90.0_f64.to_radians();
        let (mut first_gated, mut reset, mut biases) = (None, None, None);
        run(&mut f, &mut w, 8.0, |_| true, |f, nav, _| {
            for e in f.events() {
                if e.src == code::MAG_YAW && e.ok == code::GATED && first_gated.is_none() {
                    first_gated = Some(nav.t_us);
                }
                if e.src == code::YAW_RESET && reset.is_none() {
                    reset = Some(nav.t_us);
                    biases = Some(f.biases());
                }
            }
        });
        let (a, b) = (first_gated.expect("gated"), reset.unwrap_or_else(|| panic!("no yaw reset: {:?}", f.stats())));
        let (bg, ba) = biases.unwrap();
        println!("c8: first gated to reset {} s; biases at the reset {bg:?} {ba:?}; resets {}", (b - a) as f64 * 1e-6, f.stats().yaw_resets);
        assert_eq!(f.stats().yaw_resets, 1);
        assert!(b - a >= t.nav_aid_timeout_us && b - a <= t.nav_aid_timeout_us + t.nav_mag_fuse_us);
        assert_eq!((bg[2], ba[0], ba[1]), (0.0, 0.0, 0.0));
    }

    #[test]
    fn c9_a_velocity_jump_at_rest_drops_ne_at_the_next_epoch_and_it_returns_after_the_reset() {
        let t = Tuning::DEFAULT;
        let (mut f, mut w) = flying(&t, 10.0, [0.0; 3]);
        assert!(f.nav().valid.position_ne);
        let resets = f.stats().gps_resets;
        f.nom.v[0] += 5.0;
        let (mut dropped_after_epochs, mut epochs, mut back) = (None, 0, None);
        run(&mut f, &mut w, 8.0, |_| true, |f, nav, _| {
            if f.events().any(|e| e.src == code::GPS_E) {
                epochs += 1;
            }
            if !nav.valid.position_ne && dropped_after_epochs.is_none() {
                dropped_after_epochs = Some(epochs);
            }
            if dropped_after_epochs.is_some() && nav.valid.position_ne && back.is_none() {
                back = Some(f.stats().gps_resets);
            }
        });
        println!("c9: dropped after {dropped_after_epochs:?} epochs; back with resets {back:?} (before {resets})");
        assert!(dropped_after_epochs.is_some_and(|n| n <= 1));
        assert!(back.is_some_and(|r| r > resets), "NE returned only after a reset");
    }

    #[test]
    fn c10_receiver_velocity_and_speed_course_paths_agree() {
        let t = Tuning::DEFAULT;
        let v = [1.5, 0.8, 0.0];
        let (a, _) = flying(&t, 20.0, v);
        let mut w = World::new(&t);
        w.gps_vel = true;
        w.v = v;
        let mut b = Eskf::new(&t).unwrap();
        run(&mut b, &mut w, 20.0, |_| true, |_, _, _| {});
        let (na, nb) = (a.nav(), b.nav());
        let s = na.sigma.unwrap().velocity_ne_ms.max(nb.sigma.unwrap().velocity_ne_ms);
        let d = (na.velocity_ms[0] - nb.velocity_ms[0]).hypot(na.velocity_ms[1] - nb.velocity_ms[1]);
        println!("c10: speed/course {:?} receiver {:?}; difference {d} m/s, sigma {s}", na.velocity_ms, nb.velocity_ms);
        assert!(b.stats().gps_vel_fused > 0 && a.stats().gps_vel_fused > 0);
        assert!(d <= SIGMAS * s);
    }

    #[test]
    fn b5_no_aid_leaves_the_accel_bias_and_without_the_inhibit_it_moves() {
        let t = Tuning::DEFAULT;
        let moved = |inhibit: bool| {
            let mut f = Eskf::new(&t).unwrap();
            f.noaid_inhibit = inhibit;
            let mut w = World::new(&t);
            w.expected = None;
            run(&mut f, &mut w, 2.0, |_| true, |_, _, _| {});
            let ba0 = f.biases().1;
            // A horizontal manoeuvre: 1 m/s^2 north then south, 2 s each, then 1 s still. The change of b_a x and y, and
            // of b_a z, in the steps whose job was a no-aid scalar (the nominal integration never changes a bias).
            let (mut by_noaid, mut by_noaid_z) = (0.0f32, 0.0f32);
            let mut prev = f.biases().1;
            for a in [1.0, -1.0, 0.0] {
                w.a = [a, 0.0, 0.0];
                run(&mut f, &mut w, 2.0, |_| true, |f, _, _| {
                    let ba = f.biases().1;
                    if f.events().any(|e| e.src == code::NOAID) {
                        by_noaid += (ba[0] - prev[0]).abs() + (ba[1] - prev[1]).abs();
                        by_noaid_z += (ba[2] - prev[2]).abs();
                    }
                    prev = ba;
                });
            }
            let ba = f.biases().1;
            // b_a z directly: one no-aid scalar on north with P holding a north/b_a z correlation of 0.5.
            let c = 0.5 * sqrtf(f.p[POS][POS] * f.p[ACCEL_BIAS + 2][ACCEL_BIAS + 2]);
            f.p[POS][ACCEL_BIAS + 2] = c;
            f.p[ACCEL_BIAS + 2][POS] = c;
            let z0 = f.nom.ba[2];
            f.fuse_noaid(0, 1.0, t.nav_noaid_sigma_m * t.nav_noaid_sigma_m);
            let direct_z = (f.nom.ba[2] - z0).abs();
            (by_noaid, by_noaid_z, direct_z, (ba[0] - ba0[0]).abs() + (ba[1] - ba0[1]).abs(), f.stats().noaid)
        };
        let (with, without) = (moved(true), moved(false));
        println!(
            "b5: |d b_a x|+|d b_a y| and |d b_a z| by the no-aid scalars, |d b_a z| by one no-aid scalar with a north/b_a z \
             correlation of 0.5, |d b_a x|+|d b_a y| in total (barometer included), no-aid scalars: with the inhibit \
             {with:?}, without {without:?}"
        );
        assert!(with.4 > 0 && with.0 == 0.0, "the no-aid scalars moved b_a x/y: {with:?}");
        assert!(with.1 == 0.0 && with.2 == 0.0, "the no-aid scalars moved b_a z: {with:?}");
        assert!(without.0 > 0.0, "negative control: without the inhibit the no-aid scalars move b_a x/y");
        assert!(without.2 > 0.0, "negative control: without the inhibit a no-aid scalar moves b_a z");
    }

    #[test]
    fn b6_an_injected_velocity_disarmed_is_smaller_after_4_s_with_the_disarmed_no_aid_noise() {
        let left = |dis: f32| {
            let t = Tuning { nav_noaid_sigma_dis_m: dis, ..Tuning::DEFAULT };
            let mut f = Eskf::new(&t).unwrap();
            let mut w = World::new(&t);
            run(&mut f, &mut w, 3.0, |_| false, |_, _, _| {});
            f.nom.v[0] += 5.0;
            let (mut sum, mut n, mut course) = (0.0f32, 0, Vec::new());
            run(&mut f, &mut w, 4.0, |_| false, |f, _, _| {
                sum += f.nom.v[0].abs();
                n += 1;
                if n % 400 == 0 {
                    course.push(f.nom.v[0]);
                }
            });
            (sum / n as f32, course, f.sigmas()[VEL])
        };
        let (a, b) = (left(Tuning::DEFAULT.nav_noaid_sigma_dis_m), left(Tuning::DEFAULT.nav_noaid_sigma_m));
        println!("b6: mean |vN| over 4 s after a 5 m/s injection, vN every 0.5 s, vN sigma at the end: disarmed noise {a:?}; armed noise {b:?}");
        let overshoot = a.1.iter().fold(0.0f32, |m, v| m.min(*v));
        let (ea, eb) = (a.1.last().unwrap().abs(), b.1.last().unwrap().abs());
        println!("b6 (R2, judged at the end of the 4 s): |vN| {ea} m/s (0.5 m) against {eb} m/s (10 m); overshoot of the 0.5 m case {overshoot} m/s (reported)");
        assert!(ea < eb, "the injected velocity is not smaller at the end of the window with the disarmed noise");
    }

    #[test]
    fn b7_a_non_finite_state_reinitialises_and_the_flags_drop() {
        let t = Tuning::DEFAULT;
        let (mut f, mut w) = flying(&t, 10.0, [0.0; 3]);
        assert!(f.nav().valid.tilt && f.nav().valid.position_d && f.nav().valid.heading);
        f.nom.v[1] = f32::NAN;
        let mut navs = Vec::new();
        run(&mut f, &mut w, 1.0, |_| true, |_, nav, _| navs.push(*nav));
        let n0 = navs[0];
        println!("b7: reinits {}, first step {:?}", f.stats().reinits, n0.valid);
        assert_eq!(f.stats().reinits, 1);
        assert_eq!(n0.valid, Valid { rates: true, ..Valid::NONE });
        for n in &navs {
            assert!(n.attitude.iter().chain(&n.velocity_ms).chain(&n.position_m).chain(&n.rates_rads).all(|v| v.is_finite()));
        }
        assert!(navs.last().unwrap().valid.tilt, "realigned");
    }

    #[test]
    fn b8_after_a_reinitialisation_10_m_above_the_datum_the_vertical_flags_return() {
        let t = Tuning::DEFAULT;
        let mut f = Eskf::new(&t).unwrap();
        let mut w = World::new(&t);
        // At the first start the barometer initialises the height at once: the time from the first horizon to the flags.
        let (mut h0, mut first) = (None, None);
        run(&mut f, &mut w, 10.0, |_| false, |_, nav, _| {
            let h0 = *h0.get_or_insert(nav.t_us);
            if first.is_none() && nav.valid.position_d && nav.valid.velocity_d {
                first = Some(nav.t_us - h0);
            }
        });
        assert!(f.nav().valid.position_d);
        // 10 m above the datum; once the horizon has passed the move (1 s, inside the timeout), a re-initialisation, as a
        // non-finite state causes.
        w.p[2] = -10.0;
        run(&mut f, &mut w, 1.0, |_| false, |_, _, _| {});
        f.reinitialise();
        let t0 = f.horizon();
        let mut back = None;
        run(&mut f, &mut w, 8.0, |_| false, |f, nav, _| {
            if back.is_none() && nav.valid.position_d && nav.valid.velocity_d {
                back = Some((nav.t_us - t0, nav.position_m[2], nav.sigma.unwrap().position_d_m, f.stats().height_resets));
            }
        });
        println!(
            "b8: vertical flags at the first start after {first:?} us; after the re-initialisation (us, D, sigma D, height \
             resets) {back:?}"
        );
        let (dt, d, sd, _) = back.expect("the vertical flags never returned");
        // As at the first start, within one barometer period.
        assert!(dt <= first.unwrap() + 1_000_000 / BARO_RATE_HZ, "returned after {dt} us");
        assert!((d + 10.0).abs() <= SIGMAS * sd, "D {d} m");
    }

    #[test]
    fn c13_compass_refusals_are_written_once_per_fusion_interval_and_all_counted() {
        let t = Tuning::DEFAULT;
        let mut f = Eskf::new(&t).unwrap();
        let mut w = World::new(&t);
        w.mag_field = (0.5, INC_DEG);
        let mut events = 0u32;
        run(&mut f, &mut w, 10.0, |_| false, |f, _, _| {
            events += f.events().filter(|e| e.src == code::MAG_YAW && e.ok == code::REFUSED).count() as u32;
        });
        let refused = f.stats().mag_refused;
        println!("c13: 10 s of unhealthy compass samples: {refused} refused, {events} refusal records");
        assert!(refused > 900, "every refusal counted: {refused}");
        assert!(u64::from(events) <= 10_000_000 / t.nav_mag_fuse_us + 1, "{events} records");
        assert!(events > 0);
    }

    #[test]
    fn c14_a_sample_older_than_the_ring_is_dropped_and_counted() {
        let t = Tuning::DEFAULT;
        let (mut f, mut w) = flying(&t, 10.0, [1.0, 0.0, 0.0]);
        let quiet = |w: &mut World| {
            let mut s = w.step();
            s.baro = None;
            s.mag = None;
            s.gnss = None;
            s
        };
        // Each sample stamped 0.6 s ago (older than now - nav_buffer_us, 0.5 s), on a step with no other slow sample.
        let old = |w: &World| w.t_us() + PERIOD_US - 600_000;
        let s0 = *f.stats();
        let mut s = quiet(&mut w);
        let t_b = w.t_us() - 600_000;
        s.baro = Some(BaroSample { reading: BaroReading { pressure_pa: pressure_at(50.0), temperature_c: 15.0 }, t_us: t_b, seq: 0 });
        f.update(&s);
        let after_baro = (f.stats().stale, f.baro_job.map(|j| j.h_b));
        let mut s = quiet(&mut w);
        let mut m = w.clone_mag();
        m.t_us = w.t_us() - 600_000;
        m.reading.field_ut = [0.0, 50.0, 0.0];
        s.mag = Some(m);
        f.update(&s);
        let after_mag = (f.stats().stale, f.mag_job, (0..f.mag_wait.len).any(|i| f.mag_wait.get(i).t_us == m.t_us));
        let t_g = old(&w);
        let mut s = quiet(&mut w);
        let mut r = w.last_fix.unwrap();
        r.lat_e7 = r.lat_e7.map(|l| l + 1_000);
        s.gnss = Some(GnssSample { reading: r, t_us: t_g, seq: 0 });
        f.update(&s);
        let s1 = *f.stats();
        println!("c14: stale after the barometer {after_baro:?}, the compass {after_mag:?}, GNSS {}; GNSS epochs {} -> {}", s1.stale, s0.gps_epochs, s1.gps_epochs);
        assert_eq!(after_baro.0, s0.stale + 1);
        assert!(after_baro.1.is_none_or(|h| h < 10.0), "the stale barometer sample was queued");
        assert_eq!(after_mag.0, s0.stale + 2);
        assert!(after_mag.1.is_none() && !after_mag.2, "the stale compass sample was kept");
        assert_eq!((s1.stale, s1.gps_epochs), (s0.stale + 3, s0.gps_epochs), "the stale GNSS sample was used");
    }

    #[test]
    fn n1_a_tiny_negative_variance_gives_finite_sigmas_and_stays_finite_through_a_propagation_and_a_fusion() {
        let t = Tuning::DEFAULT;
        let (mut f, _) = flying(&t, 10.0, [0.0; 3]);
        // Down velocity and position uncorrelated with everything and -1e-9 (rounding below 0): the propagation adds
        // nothing to the position variance (no position process noise, no velocity variance).
        f.reset_state_var(VEL + 2, -1e-9);
        f.reset_state_var(POS + 2, -1e-9);
        let all = |f: &Eskf| {
            let s = f.nav().sigma.unwrap();
            [s.tilt_rad, s.heading_rad, s.velocity_ne_ms, s.velocity_d_ms, s.position_ne_m, s.position_d_m]
        };
        let before = all(&f);
        f.predict();
        let diag_min = (0..N).map(|i| f.p[i][i]).fold(f32::INFINITY, f32::min);
        let (ok, _) = f.fuse_unit(POS + 2, 0.1, t.nav_baro_sigma_m * t.nav_baro_sigma_m);
        let after = all(&f);
        println!("n1: sigmas with the negative entries {before:?}; smallest diagonal after the propagation {diag_min:e}; fused {ok}, sigmas {after:?}");
        assert!(before.iter().all(|v| v.is_finite()), "{before:?}");
        assert!(diag_min >= 0.0);
        assert!(f.finite() && after.iter().all(|v| v.is_finite()) && (0..N).all(|i| f.p[i][i] >= 0.0));
    }

    #[test]
    fn c15_no_speed_screen_record_for_an_epoch_without_velocity() {
        let t = Tuning::DEFAULT;
        let (mut f, mut w) = flying(&t, 10.0, [2.0, 0.0, 0.0]);
        let s0 = *f.stats();
        let mut screened = 0;
        for _ in 0..5_000_000 / PERIOD_US {
            let mut s = w.step();
            if let Some(g) = s.gnss.as_mut() {
                g.reading.speed_mm_s = None;
                g.reading.vel_ned_mm_s = None;
            }
            f.set_armed(true);
            f.update(&s);
            screened += f.events().filter(|e| e.src == code::VN && e.ok == code::SCREENED).count();
        }
        let s1 = *f.stats();
        println!("c15: 5 s of epochs without speed at 2 m/s: screen records {screened}, E3b {} -> {}, positions fused {} -> {}", s0.gps_e3b, s1.gps_e3b, s0.gps_pos_fused, s1.gps_pos_fused);
        assert_eq!((screened, s1.gps_e3b), (0, s0.gps_e3b));
        assert!(s1.gps_pos_fused > s0.gps_pos_fused + 40);
    }
}
