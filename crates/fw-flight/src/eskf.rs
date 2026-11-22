//! The shadow navigation filter (decision-nav-eskf-20261120, sections 1a-1c, 3, 6, 12; Amendment B, B2 where it
//! stands; Amendment C; Amendment C-1; packet E1b): a 15-state error-state Kalman filter on IMU1, barometer, compass
//! and GNSS at a delayed time horizon, t_now - nav_gps_delay_us (C2a), so that every measurement is fused at its own
//! time. It never steers in phase 1 and is not in the flight image (packet E2 puts it on the board in shadow).
//!
//! D1 (Amendment D): for `ImuBaro` and `Imu`, [`NavState::rates_rads`] is the PT1 of the raw gyro (`Tuning::ins_gyro_lpf_hz`,
//! Mahony's form: dt from successive sample times capped at `DT_MAX_US`, started at the first sample with its value) minus
//! the gyro-bias estimates x and y, subtracted when the NavState is built; with those at 0 it equals `Mahony`'s rates bit
//! for bit. Amendment H (H2): the z gyro bias is NOT subtracted from the rate output, so `rates_rads[2]` is the PT1 value,
//! `Mahony`'s bit for bit for any z bias estimate (the yaw rate hold reads it: with the stick centred the vehicle turns at
//! the raw gyro's z offset, not at the estimated bias minus the true one); the attitude still integrates the gyro minus the
//! full bias. `Full` keeps the raw gyro of its horizon sample minus the full bias.
//!
//! D2 (Amendment D): for `ImuBaro` and `Imu`, one tilt re-alignment at the arming edge. Each covariance
//! propagation closing an interval that was disarmed throughout keeps its mean bias-corrected specific force (the first
//! one after a disarm is skipped, and the disarm clears it). In the first `update` after the armed state
//! went false to true, when the tilt is aligned and that force is inside the alignment window, roll and pitch are set from it
//! as in the first alignment with the yaw (the Euler yaw, atan2(R10, R00), of the attitude before the step) kept; v_N and v_E
//! go to 0, the hold position to the current N and E, and the cross terms of the three attitude and the N/E velocity rows
//! and columns of P are zeroed with every variance kept. It is that step's one heavy job; the queued jobs run later.
//! Outside the window, or with no stored interval, nothing changes. `Full` never does it.
//!
//! Amendment E (E1) and Amendment H (H1): for `ImuBaro` the barometer scalar's Kalman gain has support on position D,
//! velocity D and accelerometer-bias z only (a 3-state vertical filter beside the attitude filter): every other entry is
//! set to zero and P is updated in Joseph form with that gain for every entry (as the no-aid scalars do for the bias
//! entries), so in `ImuBaro` and `Imu` b_a,x and b_a,y are exactly 0.0 on every step, from the alignment and after every
//! re-initialisation, and the barometer no longer moves the attitude, the gyro bias or the horizontal velocity and
//! position; b_a,z is still learned through its covariance with D. `Full` and `Imu` are untouched. Without GPS the
//! horizontal accelerometer bias is not estimated: the real one (about +0.1 to +0.2 m/s^2 in x per ESKF3) remains as a
//! tilt offset of up to about 1 deg, as with Mahony. The mask is constant in production; test-only fields restore the
//! full gain or also mask b_a,z for a test's negative control.
//!
//! The arithmetic is the N0 bench's ([`crate::eskf_n0`]): nominal integration on every IMU1 sample (at the horizon),
//! the structured F P F^T + Q every nav_predict_us, sequential scalar fusions. Work rule I2: each step integrates the
//! nominal state and does at most ONE heavy job (one covariance propagation, one scalar fusion, an alignment or a
//! reset), taken from a queue of one GNSS epoch (4 jobs), one barometer set (3) and one compass sample (1); a newer set
//! of the same kind replaces an unprocessed one (C9). Pure arithmetic: no lock, timer or log. The log codes of C8 are
//! returned by [`Eskf::events`] for the caller to write.
//!
//! Three variants, chosen at construction ([`Aiding`]): `Full` (ESKF3) is the filter described here; `ImuBaro` (ESKF2)
//! ignores GNSS and compass, has no horizon delay (every IMU1 sample is integrated in the step it arrives, so the state
//! refers to the current sample), never aligns the yaw and holds N and E with each barometer set; `Imu` (ESKF1) also
//! ignores the barometer and holds N, E and D at the tilt alignment's values, one set per barometer period of horizon
//! time (no-aid source code 5 for all three axes).
//!
//! [`NavState::t_us`] of this estimator is the horizon time, the time the state refers to (C8: "for I = 1, TimeUS is
//! the horizon time"); there is no output predictor in phase 1.
//!
//! Where the implementation takes a reading of the specification (checked against the code below, 2026-11-21):
//! - The Src 9 reset fires at the first accepted, aligned, non-repeated epoch after EVERY arming; any change of the
//!   armed state drops a queued GNSS job and the reject timers.
//! - Horizontal aiding = armed AND started (a Src 9 reset since arming) AND a GNSS position fused or reset within
//!   nav_aid_timeout_us. Otherwise the no-aid hold, at the position captured in the step aiding stops (N = E = 0 at
//!   the tilt alignment).
//! - The first fix (fix type >= 3 with a position) only starts the interval rule and is not accepted. Any such fix,
//!   with or without GST, becomes the interval's and the frozen-fix screen's previous fix.
//! - The accuracy (GST) part of the acceptance needs the latitude and longitude sigmas only.
//! - The origin latches at the first accepted epoch, normally the second fix, armed or not, aligned or not; its
//!   altitude is the fix's plus the filter's D, so that D keeps its datum.
//! - A waiting compass sample is used only if it carried an `expected` field itself, with the newest value carried.
//! - No velocity when the speed is at or above nav_gps_course_min_ms but the course is missing (nor when there is no
//!   speed); then the speed screen (E3b) does not fire.
//! - Job order: propagation, GNSS, barometer, compass.
//! - The height reset counts from the last barometer aid (fusion, reset, or the initialisation after a
//!   re-initialisation). After a re-initialisation the next barometer sample initialises the height at once, as
//!   `MahonyBaro` re-initialises; it is logged and counted as a height reset.
//! - No position process noise (Q on attitude, velocity and the biases only).
//! - The declination is `Tuning::nav_mag_declination_rad`; the simulator sets it from its field model.
//! - `ned_from_geo` uses the WGS 84 meridian and prime-vertical radii at the origin.
//! - A Src 9 or C3 reset counts as "a GNSS position fused" for the flags and the aiding timeout.
//! - velocity_ne and position_ne are one flag; it needs the yaw aligned, not the heading flag.
//! - The first epoch after arming (a Src 9 reset) takes the receiver's velocity unscreened, its variance
//!   nav_gps_vel_sigma_ms^2 also on the low-speed path; the C3 reset likewise.
//! - Armed at rest, exactly repeated fixes are screened, the first epoch after arming included: GNSS is neither reset
//!   nor fused until the fix changes.
//! - C1b's H row is the exact Jacobian of the innovation in use (review ruling 1, [`yaw_h`]); a compass sample whose
//!   NED field has no usable horizontal component is a refusal.
//! - Compass refusals (Ok 2) are all counted; one NKI record per nav_mag_fuse_us at most.
//! - A barometer, compass or GNSS sample whose time (validity time, sample time, arrival time minus
//!   nav_gps_delay_us) is older than the IMU ring's span, now - nav_buffer_us, is dropped and counted
//!   ([`EskfStats::stale`]).
// Fixed-size arrays only; every index is a loop bound or a constant below the array length.
#![allow(clippy::indexing_slicing, clippy::needless_range_loop)]

use crate::eskf_n0::{
    self as n0, ACCEL_BIAS, ATT, Cov, GYRO_BIAS, N, Noise, Nominal, POS, VEL, rotation,
};
use crate::gnc::{Estimator, GeoPoint, NavSigma, NavState, Sensors, Valid};
use crate::mahony::{ACCEL_MAX_G, ACCEL_MIN_G, DT_MAX_US};
use crate::mahony_baro::isa_height_m;
use crate::quat::pt1;
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
    /// Barometer sets whose no-aid N/E stages were dropped because horizontal aiding had started (a GNSS reset between
    /// the stages).
    pub noaid_dropped: u32,
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

/// The aiding set of an [`Eskf`], fixed at construction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Aiding {
    /// IMU1 only (ESKF1).
    Imu,
    /// IMU1 and barometer (ESKF2).
    ImuBaro,
    /// IMU1, barometer, compass and GNSS (ESKF3).
    Full,
}

/// The constructor's refusal (C2a).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EskfError {
    /// nav_gps_delay_us > nav_buffer_us.
    DelayBeyondBuffer,
    /// An IMU ring of no samples, or a non-zero horizon delay with a ring shorter than [`RING`].
    RingTooSmall,
}

/// The change a reset made to the nominal state in one `update` (G1): `q_after = dq x q_before` (Hamilton, body to NED,
/// [w, x, y, z]; the rotation is on the earth side), `v_after = v_before + dv`, `p_after = p_before + dp`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reset {
    /// normalise(q_after x conj(q_before)).
    pub dq: [f32; 4],
    /// v_after - v_before, m/s.
    pub dv: [f32; 3],
    /// p_after - p_before, m.
    pub dp: [f32; 3],
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

/// The barometer set: the barometer scalar, then no-aid N and E (B2). For [`Aiding::Imu`] the hold set: stages 1 to 3,
/// no-aid N, E and D.
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

/// The 15-state shadow filter with an IMU ring of `R` samples. See the module text.
#[derive(Clone, Debug)]
pub struct EskfN<const R: usize> {
    t: Tuning,
    aiding_set: Aiding,
    noise: Noise,
    ring: Fifo<ImuEntry, R>,
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
    /// Horizon time from which the next [`Aiding::Imu`] hold set is queued.
    next_hold_us: u64,
    /// D1: PT1 of the raw gyro of the integrated samples ([`Aiding::ImuBaro`], [`Aiding::Imu`]).
    rates_lpf: [f32; 3],
    /// D2: mean bias-corrected specific force of the last completed disarmed covariance interval, body frame.
    arm_f: Option<[f32; 3]>,
    /// D2: the armed state went false to true and no `update` has run since.
    arm_edge: bool,
    /// D2: the current covariance interval began armed (the armed state went true to false since the last propagation).
    interval_armed: bool,
    /// D2: the arming-edge re-alignment is on; off only in a test's negative control.
    arm_realign: bool,
    /// G1: the time of the sample whose nominal state [`EskfN::snapshot`] is to keep.
    snap_req: Option<u64>,
    /// G1: that sample's time and nominal q, v, p right after its integration, in the step it was integrated.
    snap: Option<(u64, [f32; 4], [f32; 3], [f32; 3])>,
    /// G1: the reset of this step's job, if it was one.
    reset: Option<Reset>,
    /// G1: the job of this step was a reset of the nominal state.
    reset_job: bool,
    /// H1 test control: the `ImuBaro` barometer scalar uses the full gain, every entry (the production gain has support
    /// on D, v_D and b_a,z only).
    #[cfg(test)]
    baro_full_gain: bool,
    /// E1 test control: the same scalar also leaves b_a,z alone.
    #[cfg(test)]
    baro_abias_z_mask: bool,
}

/// The filter with the board's ring, [`RING`] samples (the horizon delay up to nav_buffer_us).
pub type Eskf = EskfN<RING>;
/// The filter for the zero-delay variants, whose ring never holds more than the sample in hand.
pub type EskfZ = EskfN<1>;

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
    let norm2 = c + m[2] * m[2];
    if !norm2.is_finite() || c <= f32::EPSILON * norm2 {
        return None;
    }
    Some([-m[0] * m[2] / c, -m[1] * m[2] / c, 1.0])
}

impl<const R: usize> EskfN<R> {
    /// Not aligned, disarmed, no sample seen. Err when nav_gps_delay_us > nav_buffer_us (C2a).
    pub fn new(tuning: &Tuning) -> Result<Self, EskfError> {
        Self::with_aiding(tuning, Aiding::Full)
    }

    /// As [`EskfN::new`] with the aiding set `aiding`. Err with [`EskfError::RingTooSmall`] when `R` is 0, or the horizon delay
    /// (`Full` with nav_gps_delay_us > 0) meets a ring shorter than [`RING`]; with no delay any ring of one or more is
    /// the same filter (the sample pushed is integrated in the same step).
    pub fn with_aiding(tuning: &Tuning, aiding: Aiding) -> Result<Self, EskfError> {
        if R == 0 {
            return Err(EskfError::RingTooSmall);
        }
        if tuning.nav_gps_delay_us > tuning.nav_buffer_us {
            return Err(EskfError::DelayBeyondBuffer);
        }
        if aiding == Aiding::Full && tuning.nav_gps_delay_us > 0 && R < RING {
            return Err(EskfError::RingTooSmall);
        }
        let t = *tuning;
        Ok(EskfN {
            aiding_set: aiding,
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
            next_hold_us: 0,
            rates_lpf: [0.0; 3],
            arm_f: None,
            arm_edge: false,
            interval_armed: false,
            arm_realign: true,
            snap_req: None,
            snap: None,
            reset: None,
            reset_job: false,
            #[cfg(test)]
            baro_full_gain: false,
            #[cfg(test)]
            baro_abias_z_mask: false,
        })
    }

    /// The aiding set.
    pub fn aiding(&self) -> Aiding {
        self.aiding_set
    }

    /// The horizon delay: nav_gps_delay_us, none without GNSS.
    fn delay_us(&self) -> u64 {
        if self.aiding_set == Aiding::Full { self.t.nav_gps_delay_us } else { 0 }
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

    /// Time of the newest sample integrated at the horizon.
    pub fn horizon_us(&self) -> Option<u64> {
        self.horizon_us
    }

    /// The nominal attitude (body to NED, [w, x, y, z], not sign-normalised), velocity and position at the horizon.
    pub fn nominal(&self) -> ([f32; 4], [f32; 3], [f32; 3]) {
        (self.nom.q, self.nom.v, self.nom.p)
    }

    /// Asks for the nominal state right after the integration of the sample at `t_us`.
    pub fn set_snap_request(&mut self, t_us: Option<u64>) {
        self.snap_req = t_us;
    }

    /// The requested sample's time and nominal q, v, p right after its nominal step, in the `update` that integrated it;
    /// none in every other step (with a repeated timestamp, the last integrated in the step).
    pub fn snapshot(&self) -> Option<(u64, [f32; 4], [f32; 3], [f32; 3])> {
        self.snap
    }

    /// The reset of the nominal state the job of the last `update` was, if it was one; none after a fusion or a
    /// re-initialisation.
    pub fn last_reset(&self) -> Option<Reset> {
        self.reset
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
        self.arm_f = None;
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
        if self.aiding_set != Aiding::Full {
            let hz = self.t.ins_gyro_lpf_hz;
            self.rates_lpf = if self.horizon_us.is_some() {
                let dt = dt_us as f32 * 1e-6;
                let [r0, r1, r2] = self.rates_lpf;
                [pt1(r0, s.gyro[0], dt, hz), pt1(r1, s.gyro[1], dt, hz), pt1(r2, s.gyro[2], dt, hz)]
            } else {
                s.gyro
            };
        }
        self.horizon_us = Some(s.t_us);
        self.horizon_gyro = s.gyro;
        let dt = dt_us as f32 * 1e-6;
        if self.tilt_aligned {
            n0::nominal_step(&mut self.nom, &s.gyro, &s.accel, dt);
            if Some(s.t_us) == self.snap_req {
                self.snap = Some((s.t_us, self.nom.q, self.nom.v, self.nom.p));
            }
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
            if !self.armed && !self.interval_armed && self.aiding_set != Aiding::Full && dt > 0.0 {
                self.arm_f = Some([dvel[0] / dt, dvel[1] / dt, dvel[2] / dt]);
            }
            self.interval_armed = false;
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

    /// D2: the tilt re-alignment of the first `update` after arming (never `Full`), given the attitude before the step. True
    /// when it ran, which is the step's one heavy job.
    fn realign_at_arming(&mut self, q_pre: &[f32; 4]) -> bool {
        let edge = self.arm_edge;
        self.arm_edge = false;
        if !edge || !self.arm_realign || !self.tilt_aligned || self.aiding_set == Aiding::Full {
            return false;
        }
        let Some(f) = self.arm_f.take() else {
            return false;
        };
        let nf = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
        if !(nf >= ACCEL_MIN_G * GRAVITY_MSS && nf <= ACCEL_MAX_G * GRAVITY_MSS && f[2] < 0.0) {
            return false;
        }
        let roll = atan2f(-f[1], -f[2]);
        let pitch = atan2f(f[0], sqrtf(f[1] * f[1] + f[2] * f[2]));
        let r = rotation(q_pre);
        let yaw = atan2f(r[1][0], r[0][0]);
        let (cr, sr, cp, sp) = (cosf(0.5 * roll), sinf(0.5 * roll), cosf(0.5 * pitch), sinf(0.5 * pitch));
        let qrp = [cr * cp, sr * cp, cr * sp, -sr * sp];
        self.nom.q = normalized(qmul(&[cosf(0.5 * yaw), 0.0, 0.0, sinf(0.5 * yaw)], &qrp));
        self.reset_job = true;
        self.nom.v[0] = 0.0;
        self.nom.v[1] = 0.0;
        self.hold_ne = [self.nom.p[0], self.nom.p[1]];
        for i in [ATT, ATT + 1, ATT + 2, VEL, VEL + 1] {
            for j in 0..N {
                if j != i {
                    self.p[i][j] = 0.0;
                    self.p[j][i] = 0.0;
                }
            }
        }
        true
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

    /// The `ImuBaro` barometer scalar on D (E1, H1), gated as [`n0::fuse_position`] gates (innovation^2 <= gate^2 s, s = P_DD + var):
    /// K has support on D, v_D and b_a,z only (every other entry zeroed) and P is updated in Joseph form with that K for every
    /// entry, as `fuse_noaid` does, then mirrored and the diagonal held at or above 0. Returns (accepted, innovation variance);
    /// nothing changes when gated out.
    fn fuse_baro_masked(&mut self, innov: f32, var: f32) -> (bool, f32) {
        let k = POS + 2;
        let ph = self.p[k];
        let s = ph[k] + var;
        let gate = self.t.nav_gate_sigma;
        if s.is_nan() || s <= 0.0 || innov * innov > gate * gate * s {
            return (false, s);
        }
        let inv = 1.0 / s;
        let mut kg: [f32; N] = core::array::from_fn(|i| ph[i] * inv);
        #[cfg(test)]
        let (vertical_only, mask_z) = (!self.baro_full_gain, self.baro_abias_z_mask);
        #[cfg(not(test))]
        let (vertical_only, mask_z) = (true, false);
        if vertical_only {
            for i in 0..N {
                if i != POS + 2 && i != VEL + 2 && i != ACCEL_BIAS + 2 {
                    kg[i] = 0.0;
                }
            }
        }
        if mask_z {
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
        (true, s)
    }

    /// Horizontal aiding (C-1): armed, started, and a GNSS position fused within nav_aid_timeout_us.
    fn horizontal_aiding(&self) -> bool {
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
                self.reset_job = true;
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
                let (ok, s) = if self.aiding_set == Aiding::ImuBaro {
                    self.fuse_baro_masked(innov, r)
                } else {
                    self.fuse_unit(POS + 2, innov, r)
                };
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
                        self.reset_job = true;
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
                // The no-aid scalars are for the time horizontal aiding is not running: a GNSS reset may have started it
                // since stage 0 (the reset job runs between a barometer set's stages), and the hold position would then
                // pull the position back toward the old fix by the whole offset. The rest of the job is dropped.
                if self.horizontal_aiding() {
                    self.stats.noaid_dropped = self.stats.noaid_dropped.wrapping_add(1);
                    self.baro_job = None;
                    return;
                }
                let axis = usize::from(axis - 1);
                let sg = if self.armed { self.t.nav_noaid_sigma_m } else { self.t.nav_noaid_sigma_dis_m };
                let held = if axis < 2 { self.hold_ne[axis] } else { 0.0 };
                let innov = held - self.nom.p[axis];
                self.fuse_noaid(axis, innov, sg * sg);
            }
        }
        job.stage += 1;
        // The no-aid pair only while horizontal aiding is not running.
        let last = if self.aiding_set == Aiding::Imu { 4 } else { 3 };
        self.baro_job = (job.stage < last && !self.horizontal_aiding()).then_some(job);
    }

    /// Yaw := yaw + dpsi (a rotation about NED down); the yaw row and column of P re-initialised to the compass noise;
    /// b_g,z, b_a,x, b_a,y to 0 with their initial variances and cross terms zeroed (C1a, C3).
    fn yaw_set(&mut self, dpsi: f32) {
        let dq = [cosf(0.5 * dpsi), 0.0, 0.0, sinf(0.5 * dpsi)];
        self.nom.q = normalized(qmul(&dq, &self.nom.q));
        self.reset_job = true;
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
                self.reset_job = true;
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
        let g = if self.aiding_set == Aiding::Full { self.horizon_gyro } else { self.rates_lpf };
        NavState {
            t_us: horizon,
            attitude: q,
            rates_rads: [g[0] - n.bg[0], g[1] - n.bg[1], if self.aiding_set == Aiding::Full { g[2] - n.bg[2] } else { g[2] }],
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

impl<const R: usize> Estimator for EskfN<R> {
    fn update(&mut self, sensors: &Sensors) -> NavState {
        self.events = [None; EVENTS];
        self.snap = None;
        self.reset = None;
        self.reset_job = false;
        let q_pre = self.nom.q;
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
            if e.t_us.saturating_add(self.delay_us()) > now {
                break;
            }
            self.ring.pop();
            self.integrate(e);
        }
        // New slow samples: barometer and compass wait for the horizon; a GNSS epoch is due on arrival. A sample whose
        // time is older than the ring's span is dropped and counted.
        let buffer_us = self.t.nav_buffer_us;
        let stale = |t_us: u64| t_us.saturating_add(buffer_us) < now;
        let full = self.aiding_set == Aiding::Full;
        let baro = sensors.baro.filter(|_| self.aiding_set != Aiding::Imu);
        if let Some(b) = baro.filter(|b| b.reading.pressure_pa.is_finite() && b.reading.pressure_pa > 0.0) {
            let w = BaroWait { t_valid_us: b.t_us.saturating_sub(self.t.nav_baro_delay_us), pressure_pa: b.reading.pressure_pa };
            if stale(w.t_valid_us) {
                self.stats.stale = self.stats.stale.wrapping_add(1);
            } else if self.baro_wait.push(w).is_some() {
                self.stats.dropped = self.stats.dropped.wrapping_add(1);
            }
        }
        if let Some(m) = sensors.mag.filter(|_| full).filter(|m| {
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
        if let Some(g) = sensors.gnss.filter(|_| full) {
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
        if self.aiding_set == Aiding::Imu && self.tilt_aligned && self.horizon_us.is_some() && horizon >= self.next_hold_us {
            self.next_hold_us = horizon.saturating_add(1_000_000 / BARO_RATE_HZ);
            if self.baro_job.replace(BaroJob { h_b: 0.0, stage: 1 }).is_some() {
                self.stats.replaced = self.stats.replaced.wrapping_add(1);
            }
        }
        // The no-aid hold position: where the filter is when horizontal aiding stops.
        let aiding = self.horizontal_aiding();
        if self.aiding_prev && !aiding {
            self.hold_ne = [self.nom.p[0], self.nom.p[1]];
        }
        self.aiding_prev = aiding;
        let (q_before, v_before, p_before) = (self.nom.q, self.nom.v, self.nom.p);
        let jobs = if self.realign_at_arming(&q_pre) { 1 } else { self.one_job() };
        self.stats.max_jobs_per_step = self.stats.max_jobs_per_step.max(jobs);
        if self.reset_job {
            self.reset = Some(Reset {
                dq: normalized(qmul(&self.nom.q, &[q_before[0], -q_before[1], -q_before[2], -q_before[3]])),
                dv: core::array::from_fn(|i| self.nom.v[i] - v_before[i]),
                dp: core::array::from_fn(|i| self.nom.p[i] - p_before[i]),
            });
        }
        if self.tilt_aligned && !self.finite() {
            self.reinitialise();
            self.reset = None;
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
            self.arm_edge = armed;
            if !armed {
                self.arm_f = None;
                self.interval_armed = true;
            }
            if armed {
                self.fix_hist = Fifo::new();
            }
        }
        self.armed = armed;
    }
}

#[cfg(test)]
pub(crate) mod tests {
    extern crate std;
    use super::*;
    use crate::imu::{ImuReading, ImuSample};
    use crate::mahony_baro::{ISA_GAS_CONSTANT, ISA_LAPSE_K_M, ISA_P0_PA, ISA_T0_K};
    use fw_core::sample::{BaroReading, BaroSample, GnssReading, MagReading, MagSample};
    use std::collections::VecDeque;
    use std::println;
    use std::vec::Vec;

    /// IMU period, us: 800 Hz (decision-sim-conventions 6).
    pub(crate) const PERIOD_US: u64 = 1_250;
    /// Barometer 50 Hz, compass 100 Hz, GNSS 5 Hz (section 3b, 4a; the board's GPS_RATE_MS 200).
    const BARO_EVERY: u64 = 16;
    const MAG_EVERY: u64 = 8;
    const GNSS_EVERY: u64 = 160;
    /// A synthetic site, not anyone's place: next to the 180 deg meridian so east motion crosses the wrap.
    pub(crate) const ORIGIN: GeoPoint = GeoPoint { lat_e7: 100_000_000, lon_e7: 1_799_999_000, alt_mm: 100_000 };
    /// The synthetic field: 50 uT, 60 deg down; tolerances of the stored defaults MAG_HLT_STR 10 %, MAG_HLT_INC 10 deg.
    const STRENGTH_UT: f64 = 50.0;
    const INC_DEG: f64 = 60.0;
    /// B3: the test-only sigma honesty check.
    const SIGMAS: f32 = 3.0;

    pub(crate) fn expected() -> MagExpected {
        MagExpected {
            strength_ut: STRENGTH_UT as f32,
            inclination_rad: (INC_DEG as f32).to_radians(),
            tol_strength_frac: 0.10,
            tol_inclination_rad: 10.0_f32.to_radians(),
        }
    }

    pub(crate) fn pressure_at(height_m: f64) -> f32 {
        let e = f64::from(GRAVITY_MSS) / (f64::from(ISA_LAPSE_K_M) * f64::from(ISA_GAS_CONSTANT));
        let t0 = f64::from(ISA_T0_K);
        (f64::from(ISA_P0_PA) * ((t0 - f64::from(ISA_LAPSE_K_M) * height_m) / t0).powf(e)) as f32
    }

    #[derive(Clone, Copy, Debug, PartialEq)]
    pub(crate) enum Fault {
        None,
        /// From this arrival time to the next, the fix repeats the last one and speed reads 0 (the receiver's hold).
        Freeze(u64, u64),
        /// From this arrival time on, north is off by this many metres.
        North(u64, f64),
    }

    /// A level vehicle with a yaw, flying a constant NED acceleration; perfect sensors at the delays of `Tuning`.
    pub(crate) struct World {
        pub(crate) k: u64,
        pub(crate) t: Tuning,
        pub(crate) p: [f64; 3],
        pub(crate) v: [f64; 3],
        pub(crate) a: [f64; 3],
        pub(crate) yaw: f64,
        pub(crate) yaw_rate: f64,
        pub(crate) hist: VecDeque<(u64, [f64; 3], [f64; 3], f64)>,
        pub(crate) expected: Option<MagExpected>,
        /// Compass turned about body z by this, rad (passes the health test).
        pub(crate) mag_rot: f64,
        /// A disturbed field: strength factor and inclination, deg.
        pub(crate) mag_field: (f64, f64),
        pub(crate) gps_every: u64,
        pub(crate) gps_vel: bool,
        pub(crate) std_mm: u32,
        pub(crate) fault: Fault,
        pub(crate) last_fix: Option<GnssReading>,
    }

    impl World {
        pub(crate) fn new(t: &Tuning) -> World {
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

        pub(crate) fn t_us(&self) -> u64 {
            self.k * PERIOD_US
        }

        /// Truth (p, v, yaw) at `t_us` (nearest kept step).
        pub(crate) fn at(&self, t_us: u64) -> ([f64; 3], [f64; 3], f64) {
            let e = self.hist.iter().rev().find(|h| h.0 <= t_us).or(self.hist.front()).copied().unwrap();
            (e.1, e.2, e.3)
        }

        pub(crate) fn step(&mut self) -> Sensors {
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
    pub(crate) fn geo(p: [f64; 3]) -> (i32, i32) {
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

    /// G2 tests: a non-finite velocity in the core's state, as `b7` makes it.
    pub(crate) fn poison(e: &mut Eskf) {
        e.nom.v[1] = f32::NAN;
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

    #[test]
    fn p21_full_is_new_bit_for_bit() {
        let t = Tuning::DEFAULT;
        let (mut a, mut b) = (Eskf::new(&t).unwrap(), Eskf::with_aiding(&t, Aiding::Full).unwrap());
        assert_eq!((a.aiding(), b.aiding()), (Aiding::Full, Aiding::Full));
        let mut w = World::new(&t);
        w.v = [1.0, 0.5, 0.0];
        for _ in 0..30_000_000 / PERIOD_US {
            let s = w.step();
            let armed = w.t_us() >= 5_000_000;
            a.set_armed(armed);
            b.set_armed(armed);
            assert_eq!(a.update(&s), b.update(&s));
        }
        assert_eq!(a.stats(), b.stats());
        assert!(a.stats().gps_pos_fused > 0 && a.stats().mag_fused > 0 && a.stats().baro_fused > 0);
    }

    #[test]
    fn p21_imu_baro_ignores_gnss_and_compass_and_has_no_delay() {
        let t = Tuning::DEFAULT;
        let mut f = Eskf::with_aiding(&t, Aiding::ImuBaro).unwrap();
        assert_eq!(f.aiding(), Aiding::ImuBaro);
        let mut w = World::new(&t);
        let mut vert = false;
        for _ in 0..30_000_000 / PERIOD_US {
            let s = w.step();
            f.set_armed(true);
            let nav = f.update(&s);
            assert_eq!(nav.t_us, s.imu1.t_us);
            assert!(!nav.valid.heading && !nav.valid.velocity_ne && !nav.valid.position_ne);
            assert!(f.origin().is_none() && !f.yaw_aligned());
            assert_eq!(f.events().filter(|e| e.src != code::BARO && e.src != code::NOAID && e.src != code::HEIGHT_RESET).count(), 0);
            vert |= nav.valid.velocity_d && nav.valid.position_d;
        }
        let st = f.stats();
        println!("p21: imu_baro stats {st:?}");
        assert!(vert && f.tilt_aligned());
        assert_eq!(
            (st.gps_epochs, st.gps_unaccepted, st.gps_prealign, st.gps_disarmed, st.gps_e1s, st.gps_e3b, st.gps_pos_fused, st.gps_vel_fused, st.gps_resets),
            (0, 0, 0, 0, 0, 0, 0, 0, 0)
        );
        assert_eq!((st.mag_refused, st.mag_fused, st.mag_gated, st.yaw_aligns, st.yaw_resets, st.stale, st.dropped), (0, 0, 0, 0, 0, 0, 0));
        assert!(f.mag_expected.is_none());
        assert!(st.baro_fused > 0 && st.noaid > 0);
        assert_eq!(st.max_jobs_per_step, 1);
    }

    #[test]
    fn p21_imu_only_holds_three_axes_and_ignores_the_barometer() {
        let t = Tuning::DEFAULT;
        let mut f = Eskf::with_aiding(&t, Aiding::Imu).unwrap();
        assert_eq!(f.aiding(), Aiding::Imu);
        let mut w = World::new(&t);
        let mut worst = [0.0f32; 3];
        let mut tilt = false;
        for _ in 0..40_000_000 / PERIOD_US {
            let s = w.step();
            f.set_armed(w.t_us() >= 20_000_000);
            let nav = f.update(&s);
            assert_eq!(nav.t_us, s.imu1.t_us);
            assert!(!nav.valid.velocity_d && !nav.valid.position_d);
            assert!(!nav.valid.heading && !nav.valid.velocity_ne && !nav.valid.position_ne);
            assert!(nav.attitude.iter().chain(&nav.velocity_ms).chain(&nav.position_m).all(|v| v.is_finite()));
            tilt |= nav.valid.tilt;
            if w.t_us() >= 10_000_000 && w.t_us() < 20_000_000 {
                let sg = f.sigmas();
                for i in 0..3 {
                    worst[i] = worst[i].max(sg[POS + i]);
                }
            }
        }
        let st = f.stats();
        println!("p21: imu worst position sigma N/E/D after 10 s disarmed {worst:?} (limit {}), stats {st:?}", t.nav_noaid_sigma_dis_m);
        assert!(tilt && f.origin().is_none() && !f.yaw_aligned() && f.datum_m.is_none());
        assert_eq!((st.baro_fused, st.baro_gated, st.height_resets, st.stale, st.dropped, st.gps_epochs, st.mag_fused), (0, 0, 0, 0, 0, 0, 0));
        assert!(st.noaid > 0);
        assert_eq!(st.max_jobs_per_step, 1);
        assert!(worst.iter().all(|&x| x <= t.nav_noaid_sigma_dis_m), "{worst:?}");
    }

    #[test]
    fn p21_imu_baro_and_imu_align_a_tilted_start() {
        let t = Tuning::DEFAULT;
        for aiding in [Aiding::ImuBaro, Aiding::Imu] {
            let mut f = Eskf::with_aiding(&t, aiding).unwrap();
            let mut w = World::new(&t);
            let roll = 20.0_f64.to_radians();
            let g = f64::from(GRAVITY_MSS);
            let mut nav = None;
            for _ in 0..5_000_000 / PERIOD_US {
                let mut s = w.step();
                s.imu1.reading.accel_ms2 = [0.0, (-g * roll.sin()) as f32, (-g * roll.cos()) as f32];
                nav = Some(f.update(&s));
            }
            let nav = nav.unwrap();
            assert!(nav.valid.tilt);
            // Truth: the body-frame down axis of a vehicle rolled by `roll` is (0, sin, cos); the estimate's is the third
            // row of R(q).
            let r = rotation(&nav.attitude);
            let dot = f64::from(r[2][1]) * roll.sin() + f64::from(r[2][2]) * roll.cos();
            let err_deg = dot.clamp(-1.0, 1.0).acos().to_degrees();
            println!("p21: {aiding:?} tilted start, down-axis error {err_deg:e} deg");
            assert!(err_deg < 0.1, "{aiding:?}: {err_deg}");
        }
    }

    // ---- Amendment D, P1: D1 (rates) and D2 (arming-edge re-alignment) ----

    use crate::mahony::Mahony;

    fn tilt_err_deg(q: &[f32; 4]) -> f64 {
        f64::from(rotation(q)[2][2]).clamp(-1.0, 1.0).acos().to_degrees()
    }

    fn roll_pitch(q: &[f32; 4]) -> (f64, f64) {
        let r = rotation(q);
        (f64::from(r[2][1]).atan2(f64::from(r[2][2])), -f64::from(r[2][0]).asin())
    }

    /// Every field of the filter a re-alignment could touch, as bit patterns (NaN-safe equality).
    fn bits(e: &Eskf) -> Vec<u32> {
        let n = &e.nom;
        let mut v: Vec<u32> = n.q.iter().chain(&n.v).chain(&n.p).chain(&n.bg).chain(&n.ba).chain(&n.dvel).chain(&e.hold_ne).map(|x| x.to_bits()).collect();
        v.extend(e.p.iter().flatten().map(|x| x.to_bits()));
        v.push(n.dt_sum.to_bits());
        v
    }

    fn rate_bits(n: &NavState) -> [u32; 3] {
        n.rates_rads.map(f32::to_bits)
    }

    /// Heavy jobs run so far, from the counters a job moves (the ones the zero-delay variants have).
    fn heavy(e: &Eskf) -> u32 {
        let s = e.stats();
        s.predicts + s.baro_fused + s.baro_gated + s.noaid + s.height_resets
    }

    #[test]
    fn r1_zero_delay_rates_equal_mahonys_bit_for_bit_with_bias_learning_off() {
        // Bias learning off: no initial bias variance and no bias random walk, so K's bias rows stay 0.
        let t = Tuning { nav_p0_bg_rads: 0.0, nav_bg_rw: 0.0, ..Tuning::DEFAULT };
        for aiding in [Aiding::ImuBaro, Aiding::Imu] {
            let mut f = Eskf::with_aiding(&t, aiding).unwrap();
            let mut m = Mahony::new(&t);
            let mut w = World::new(&t);
            let mut moved = 0.0f32;
            for k in 0..10_000_000 / PERIOD_US {
                let mut s = w.step();
                let ph = k as f32 * PERIOD_US as f32 * 1e-6;
                s.imu1.reading.gyro_rads = [0.3 * sinf(2.0 * PI * 1.7 * ph), 0.2 * cosf(2.0 * PI * 0.6 * ph), 0.1 * sinf(2.0 * PI * 3.1 * ph)];
                let armed = w.t_us() >= 4_000_000;
                f.set_armed(armed);
                m.set_armed(armed);
                let (nf, nm) = (f.update(&s), m.update(&s));
                assert_eq!(rate_bits(&nf), rate_bits(&nm), "{aiding:?} step {k}");
                assert_eq!(f.biases().0.map(f32::to_bits), [0; 3], "{aiding:?} step {k}: the gyro bias moved");
                moved = moved.max(nf.rates_rads.iter().fold(0.0, |a, v| fabsf(*v).max(a)));
            }
            assert!(moved > 0.1, "the gyro did not move: {moved}");
        }
    }

    #[test]
    fn r2_a_constant_gyro_offset_with_learning_on_differs_from_mahony_by_the_bias_estimate() {
        let t = Tuning::DEFAULT;
        let mut f = Eskf::with_aiding(&t, Aiding::ImuBaro).unwrap();
        let mut m = Mahony::new(&t);
        let mut w = World::new(&t);
        let off = [0.02f32, -0.015, 0.0];
        let (mut worst, mut bg) = (0.0f64, [0.0f32; 3]);
        for _ in 0..60_000_000 / PERIOD_US {
            let mut s = w.step();
            s.imu1.reading.gyro_rads = off;
            let (nf, nm) = (f.update(&s), m.update(&s));
            bg = f.biases().0;
            // H2: the z rate is Mahony's PT1 value bit for bit, whatever the z bias estimate.
            assert_eq!(nf.rates_rads[2].to_bits(), nm.rates_rads[2].to_bits(), "z rate differs from Mahony's");
            for i in 0..2 {
                // nf = fl(x - b) with x Mahony's filtered gyro: |x - nf - b| <= u |x - b|, u = EPSILON / 2; the bound used
                // is twice that, EPSILON max(|x|, |b|).
                let d = f64::from(nm.rates_rads[i]) - f64::from(nf.rates_rads[i]) - f64::from(bg[i]);
                let tol = f64::from(f32::EPSILON) * f64::from(nm.rates_rads[i].abs().max(bg[i].abs()));
                assert!(d.abs() <= tol, "axis {i}: difference minus bias {d:e}, tolerance {tol:e}");
                worst = worst.max(d.abs());
            }
        }
        println!("r2: gyro offset {off:?}, bias estimate after 60 s {bg:?}, worst |difference - bias| {worst:e}");
        assert!(bg[0] != 0.0 && bg[1] != 0.0, "the bias did not move: {bg:?}");
    }

    /// The disarmed handling scenario: level at rest, tilt and yaw driven into the filter by a gyro pulse the vehicle did
    /// not make (0.5 s at [0.2, -0.1, 0.3] rad/s from 1 s: about 5.7 deg of roll, 2.9 deg of pitch, 8.6 deg of yaw),
    /// then at rest until 4 s. Returns the filter after that, 80 further samples (at rest, or with `accelerate` a
    /// forward acceleration of 0.6 g on the accelerometer), and whether the tilt flag stayed true from its first step.
    fn handled(t: &Tuning, aiding: Aiding, accelerate: bool) -> (Eskf, Vec<Sensors>, bool) {
        let mut f = Eskf::with_aiding(t, aiding).unwrap();
        let mut w = World::new(t);
        let (mut seen, mut ok) = (false, true);
        while w.t_us() < 4_000_000 {
            let mut s = w.step();
            if w.t_us() >= 1_000_000 && w.t_us() < 1_500_000 {
                s.imu1.reading.gyro_rads = [0.2, -0.1, 0.3];
            }
            f.set_armed(false);
            let nav = f.update(&s);
            ok &= nav.valid.tilt || !seen;
            seen |= nav.valid.tilt;
        }
        let g = GRAVITY_MSS;
        let next = (0..80)
            .map(|_| {
                let mut s = w.step();
                if accelerate {
                    s.imu1.reading.accel_ms2 = [0.6 * g, 0.0, -g];
                }
                s
            })
            .collect();
        (f, next, ok && seen)
    }

    struct Edge {
        pre: Eskf,
        a: Eskf,
        b: Eskf,
        nav_pre: NavState,
        na: NavState,
        nb: NavState,
        /// Heavy jobs the disabled run ran in the edge step; the re-aligned run's.
        jobs_b: u32,
        jobs_a: u32,
        tilt_true: bool,
    }

    /// Arms `base` after `j` of the `next` samples, with the re-alignment on (`a`) and off (`b`); 40 further armed steps.
    fn edge_at(base: &Eskf, next: &[Sensors], j: usize) -> Edge {
        let (mut a, mut b) = (base.clone(), base.clone());
        b.arm_realign = false;
        let (mut nav_pre, mut tilt_true) = (a.nav(), true);
        for s in &next[..j] {
            a.set_armed(false);
            b.set_armed(false);
            nav_pre = a.update(s);
            assert_eq!(nav_pre, b.update(s));
        }
        let pre = a.clone();
        a.set_armed(true);
        b.set_armed(true);
        let (ha, hb) = (heavy(&a), heavy(&b));
        let na = a.update(&next[j]);
        let nb = b.update(&next[j]);
        let edge = Edge { pre, nav_pre, na, nb, jobs_a: heavy(&a) - ha, jobs_b: heavy(&b) - hb, tilt_true: true, a: a.clone(), b: b.clone() };
        tilt_true &= na.valid.tilt && nb.valid.tilt;
        for s in &next[j + 1..j + 41] {
            a.set_armed(true);
            b.set_armed(true);
            tilt_true &= a.update(s).valid.tilt && b.update(s).valid.tilt;
        }
        Edge { tilt_true, ..edge }
    }

    #[test]
    fn al1_handled_set_down_armed_realigns_the_tilt_and_nothing_else() {
        let t = Tuning::DEFAULT;
        for aiding in [Aiding::ImuBaro, Aiding::Imu] {
            let (base, next, tilt_ok) = handled(&t, aiding, false);
            assert!(tilt_ok);
            let (mut with_job, mut without_job) = (0, 0);
            let (mut before_min, mut after_max) = (f64::INFINITY, 0.0f64);
            let (mut yaw_worst, mut rp_worst) = (0.0f64, 0.0f64);
            for j in 0..16 {
                let e = edge_at(&base, &next, j);
                assert!(e.tilt_true, "{aiding:?} j {j}: the tilt flag dropped");
                let (before, after) = (tilt_err_deg(&e.nav_pre.attitude), tilt_err_deg(&e.na.attitude));
                assert!(after < before, "{aiding:?} j {j}: error {before} deg before, {after} deg after");
                before_min = before_min.min(before);
                after_max = after_max.max(after);
                // Yaw (atan2(R10, R00)) unchanged to round-off, and not trivially 0.
                let (y0, y1) = (yaw_of(&e.pre.nom.q), yaw_of(&e.na.attitude));
                assert!(y0.abs() > 0.05, "yaw {y0}");
                yaw_worst = yaw_worst.max((y1 - y0).abs());
                assert!((y1 - y0).abs() < 1e-6, "{aiding:?} j {j}: yaw {y0} -> {y1}");
                // Roll and pitch are the first alignment's of the stored interval.
                let f = e.pre.arm_f.expect("a stored interval").map(f64::from);
                let roll = (-f[1]).atan2(-f[2]);
                let pitch = f[0].atan2(f[1].hypot(f[2]));
                let (r1, p1) = roll_pitch(&e.na.attitude);
                rp_worst = rp_worst.max((r1 - roll).abs()).max((p1 - pitch).abs());
                assert!((r1 - roll).abs() < 1e-6 && (p1 - pitch).abs() < 1e-6, "{aiding:?} j {j}: roll {r1} vs {roll}, pitch {p1} vs {pitch}");
                // v_N = v_E = 0, hold at the current N/E.
                assert_eq!((e.na.velocity_ms[0], e.na.velocity_ms[1]), (0.0, 0.0));
                assert_eq!(e.a.hold_ne, [e.a.nom.p[0], e.a.nom.p[1]]);
                // Biases and the 15 variances are bit-identical to the filter before the step: the re-alignment is the step's
                // job and the integration does not touch them.
                let diag = |f: &Eskf| (0..N).map(|i| f.p[i][i].to_bits()).collect::<Vec<_>>();
                assert_eq!((e.a.nom.bg, e.a.nom.ba), (e.pre.nom.bg, e.pre.nom.ba));
                assert_eq!(diag(&e.a), diag(&e.pre));
                // The cross terms of the five rows and columns are gone, nothing else of P moved.
                for i in 0..N {
                    for k in 0..N {
                        let five = |x: usize| (ATT..ATT + 3).contains(&x) || x == VEL || x == VEL + 1;
                        if i != k && (five(i) || five(k)) {
                            assert_eq!(e.a.p[i][k], 0.0, "P[{i}][{k}]");
                        } else {
                            assert_eq!(e.a.p[i][k].to_bits(), e.pre.p[i][k].to_bits(), "P[{i}][{k}]");
                        }
                    }
                }
                // One heavy job at most; a queued job is not consumed by it.
                assert_eq!(e.a.stats().max_jobs_per_step, 1);
                assert_eq!(e.jobs_a, 0, "the re-alignment replaced a counted job's slot, it ran one too");
                assert!(e.jobs_b <= 1);
                if e.jobs_b == 0 {
                    without_job += 1;
                    // The clone without the re-alignment ran no job: D, vD, the biases and the variances agree with it.
                    assert_eq!((e.a.nom.p[2].to_bits(), e.a.nom.v[2].to_bits()), (e.b.nom.p[2].to_bits(), e.b.nom.v[2].to_bits()));
                    assert_eq!((e.a.nom.bg, e.a.nom.ba), (e.b.nom.bg, e.b.nom.ba));
                    assert_eq!(diag(&e.a), diag(&e.b));
                } else {
                    with_job += 1;
                    // The job the clone ran is still queued in the re-aligned filter: it runs in a later step.
                    assert!(e.a.predict_due || e.a.baro_job.is_some() || e.a.gps_job.is_some());
                }
            }
            println!(
                "al1 {aiding:?}: tilt error before the edge >= {before_min:.4} deg, after <= {after_max:.6} deg; worst yaw change {yaw_worst:e} rad, worst roll/pitch difference {rp_worst:e} rad; edges with a queued job {with_job}, without {without_job}"
            );
            assert!(with_job > 0 && without_job > 0, "{with_job} {without_job}");
        }
    }

    #[test]
    fn al2_negative_control_without_the_realignment_the_error_does_not_improve() {
        let t = Tuning::DEFAULT;
        for aiding in [Aiding::ImuBaro, Aiding::Imu] {
            let (base, next, _) = handled(&t, aiding, false);
            let (mut before_min, mut worst_change) = (f64::INFINITY, 0.0f64);
            for j in 0..16 {
                let e = edge_at(&base, &next, j);
                let (before, after) = (tilt_err_deg(&e.nav_pre.attitude), tilt_err_deg(&e.nb.attitude));
                // The gyro is 0 at rest, so one step turns the attitude by at most |bias| dt (plus the f32 rounding).
                let bg = e.b.nom.bg.iter().fold(0.0f32, |m, v| m.max(fabsf(*v)));
                let bound = (f64::from(bg) * PERIOD_US as f64 * 1e-6 * 3.0_f64.sqrt()).to_degrees() + 1e-4;
                assert!((after - before).abs() <= bound, "{aiding:?} j {j}: {before} -> {after}, bound {bound:e}");
                assert!(before > 1.0, "the handling error is gone before the edge: {before}");
                before_min = before_min.min(before);
                worst_change = worst_change.max((after - before).abs());
            }
            println!("al2 {aiding:?}: error before the edge >= {before_min:.4} deg, largest change across the edge without the re-alignment {worst_change:e} deg");
        }
    }

    #[test]
    fn al3_a_stored_interval_outside_the_window_changes_nothing() {
        let t = Tuning::DEFAULT;
        for aiding in [Aiding::ImuBaro, Aiding::Imu] {
            let (base, next, _) = handled(&t, aiding, true);
            for j in 16..32 {
                let e = edge_at(&base, &next, j);
                let f = e.pre.arm_f.expect("a stored interval");
                let nf = sqrtf(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
                assert!(nf > ACCEL_MAX_G * GRAVITY_MSS, "{aiding:?} j {j}: force {nf} is inside the window");
                assert_eq!(bits(&e.a), bits(&e.b), "{aiding:?} j {j}");
                assert_eq!(e.na, e.nb);
                assert!(e.tilt_true);
            }
        }
    }

    #[test]
    fn f1_full_never_realigns_at_the_arming_edge_and_keeps_raw_gyro_minus_bias() {
        let t = Tuning::DEFAULT;
        let (base, next, tilt_ok) = handled(&t, Aiding::Full, false);
        assert!(tilt_ok);
        for j in [0usize, 3, 8, 13] {
            let (mut a, mut b) = (base.clone(), base.clone());
            b.arm_realign = false;
            for (i, s) in next.iter().enumerate() {
                let armed = i >= j;
                a.set_armed(armed);
                b.set_armed(armed);
                let (na, nb) = (a.update(s), b.update(s));
                assert_eq!(na, nb, "j {j} step {i}");
                assert_eq!(bits(&a), bits(&b), "j {j} step {i}");
                assert!(a.arm_f.is_none() && na.valid.tilt);
                let (g, bg) = (a.horizon_gyro, a.biases().0);
                assert_eq!(rate_bits(&na), [g[0] - bg[0], g[1] - bg[1], g[2] - bg[2]].map(f32::to_bits));
            }
        }
    }

    #[test]
    fn z1_negative_control_full_lags_by_the_gps_delay_and_the_zero_delay_variants_do_not() {
        let t = Tuning::DEFAULT;
        assert_eq!(t.nav_gps_delay_us % PERIOD_US, 0);
        for (aiding, lag) in [(Aiding::Full, t.nav_gps_delay_us), (Aiding::ImuBaro, 0), (Aiding::Imu, 0)] {
            let mut f = Eskf::with_aiding(&t, aiding).unwrap();
            let mut w = World::new(&t);
            let mut checked = 0;
            for _ in 0..3_000_000 / PERIOD_US {
                let s = w.step();
                let nav = f.update(&s);
                if s.imu1.t_us >= t.nav_gps_delay_us {
                    assert_eq!(s.imu1.t_us - nav.t_us, lag, "{aiding:?} at {}", s.imu1.t_us);
                    checked += 1;
                }
            }
            assert!(checked > 1000);
        }
    }

    #[test]
    fn b1_barometer_lost_while_armed_vertical_flags_drop_at_the_timeout_and_the_tilt_stays() {
        let t = Tuning::DEFAULT;
        let mut f = Eskf::with_aiding(&t, Aiding::ImuBaro).unwrap();
        let mut w = World::new(&t);
        let stop_us = 10_000_000;
        let (mut last_aid, mut dropped, mut after) = (None, None, 0u64);
        let mut seen = false;
        for _ in 0..60_000_000 / PERIOD_US {
            let mut s = w.step();
            if w.t_us() > stop_us {
                s.baro = None;
            }
            f.set_armed(true);
            let nav = f.update(&s);
            if w.t_us() <= stop_us || dropped.is_none() {
                last_aid = f.last_baro_aid_us;
            }
            seen |= nav.valid.tilt;
            assert!(nav.valid.tilt == seen && !nav.valid.heading && !nav.valid.velocity_ne && !nav.valid.position_ne);
            let finite = nav.attitude.iter().chain(&nav.rates_rads).chain(&nav.velocity_ms).chain(&nav.position_m).all(|v| v.is_finite());
            let sg = nav.sigma.unwrap();
            // The sigmas are infinite by design before the tilt alignment.
            assert!(finite && (!nav.valid.tilt || [sg.tilt_rad, sg.heading_rad, sg.velocity_ne_ms, sg.velocity_d_ms, sg.position_ne_m, sg.position_d_m].iter().all(|v| v.is_finite())));
            let vert = nav.valid.velocity_d || nav.valid.position_d;
            assert_eq!(nav.valid.velocity_d, nav.valid.position_d);
            if dropped.is_none() && !vert && w.t_us() > stop_us {
                dropped = Some(nav.t_us);
            } else if dropped.is_some() {
                assert!(!vert, "the vertical flags came back");
                after += 1;
            }
        }
        let (last, drop) = (last_aid.unwrap(), dropped.expect("the flags never dropped"));
        let late = drop - (last + t.nav_aid_timeout_us);
        println!("b1: last barometer aid at {last} us, flags dropped at {drop} us: {late} us after the timeout ({} steps); {after} finite steps after", late / PERIOD_US);
        assert!(late < PERIOD_US, "{late}");
        assert!(after * PERIOD_US >= 30_000_000, "{after}");
    }

    /// `base` armed after `first` of the `next` samples (re-alignment per `realign_first`), armed for 20 steps, disarmed for
    /// `k` steps (the disarm edge is the first of them; at most 8 is inside one propagation, nav_predict_us / PERIOD_US).
    /// Returns the filter before the next step, which is index 24 + k, and that index.
    fn disarmed_again(base: &Eskf, next: &[Sensors], first: usize, realign_first: bool, k: usize) -> (Eskf, usize) {
        let mut f = base.clone();
        f.arm_realign = realign_first;
        for (i, s) in next[..24 + k].iter().enumerate() {
            f.set_armed(i >= first && i < 24);
            f.update(s);
        }
        f.arm_realign = true;
        (f, 24 + k)
    }

    /// Steps `a` (re-alignment on) and `c` (off) over the same samples, armed or disarmed, and says whether they agree.
    fn agree(a: &mut Eskf, c: &mut Eskf, samples: &[Sensors], armed: bool) -> bool {
        let mut same = true;
        for s in samples {
            a.set_armed(armed);
            c.set_armed(armed);
            let (na, nc) = (a.update(s), c.update(s));
            same &= na == nc && bits(a) == bits(c);
        }
        same
    }

    #[test]
    fn al4_a_rearm_before_a_complete_disarmed_interval_does_nothing() {
        let t = Tuning::DEFAULT;
        for aiding in [Aiding::ImuBaro, Aiding::Imu] {
            let (base, next, _) = handled(&t, aiding, false);
            for k in 1..=8 {
                let (mut a, idx) = disarmed_again(&base, &next, 4, true, k);
                assert!(a.arm_f.is_none(), "{aiding:?} k {k}: a force is stored");
                let mut c = a.clone();
                c.arm_realign = false;
                assert!(agree(&mut a, &mut c, &next[idx..idx + 20], true), "{aiding:?} k {k}: the second arming did something");
            }
        }
    }

    #[test]
    fn al5_the_disarm_edge_does_nothing() {
        let t = Tuning::DEFAULT;
        for aiding in [Aiding::ImuBaro, Aiding::Imu] {
            let (base, next, _) = handled(&t, aiding, false);
            // Armed without the first re-alignment, so the handling error is still there.
            let mut f = base.clone();
            f.arm_realign = false;
            let mut nav = f.nav();
            for (i, s) in next[..24].iter().enumerate() {
                f.set_armed(i >= 4);
                nav = f.update(s);
            }
            let (mut a, mut c) = (f.clone(), f.clone());
            a.arm_realign = true;
            c.arm_realign = false;
            let before = tilt_err_deg(&nav.attitude);
            a.set_armed(false);
            c.set_armed(false);
            assert!(a.arm_f.is_none());
            // A force inside the window forced in after the edge: only an edge flag set by the disarm could use it.
            let level = Some([0.0, 0.0, -GRAVITY_MSS]);
            a.arm_f = level;
            c.arm_f = level;
            let (na, nc) = (a.update(&next[24]), c.update(&next[24]));
            assert_eq!(na, nc);
            assert_eq!(bits(&a), bits(&c));
            let after = tilt_err_deg(&na.attitude);
            println!("al5 {aiding:?}: tilt error {before:.4} deg before the disarm step, {after:.4} deg after");
            assert!(before > 1.0 && (after - before).abs() < 0.01);
        }
    }

    #[test]
    fn al6_the_stored_force_is_used_once() {
        let t = Tuning::DEFAULT;
        for aiding in [Aiding::ImuBaro, Aiding::Imu] {
            for accelerate in [false, true] {
                let (base, next, _) = handled(&t, aiding, accelerate);
                let mut a = base.clone();
                let j = if accelerate { 20 } else { 4 };
                for s in &next[..j] {
                    a.set_armed(false);
                    a.update(s);
                }
                assert!(a.arm_f.is_some());
                a.set_armed(true);
                a.update(&next[j]);
                assert!(a.arm_f.is_none(), "{aiding:?} accelerate {accelerate}: the force was not consumed");
                // No second re-alignment without a new edge.
                let mut c = a.clone();
                c.arm_realign = false;
                assert!(agree(&mut a, &mut c, &next[j + 1..j + 41], true));
            }
            // After a re-alignment, a re-arm with no complete disarmed interval in between does nothing.
            let (base, next, _) = handled(&t, aiding, false);
            for k in 1..=3 {
                let (mut a, idx) = disarmed_again(&base, &next, 4, true, k);
                assert!(a.arm_f.is_none());
                let mut c = a.clone();
                c.arm_realign = false;
                assert!(agree(&mut a, &mut c, &next[idx..idx + 5], true), "{aiding:?} k {k}");
            }
        }
    }

    /// The angle between the filter's body-frame down axis (third row of R(q)) and `d`, rad.
    fn down_err_rad(q: &[f32; 4], d: [f64; 3]) -> f64 {
        let r = rotation(q);
        let e = [f64::from(r[2][0]), f64::from(r[2][1]), f64::from(r[2][2])];
        let c = [e[1] * d[2] - e[2] * d[1], e[2] * d[0] - e[0] * d[2], e[0] * d[1] - e[1] * d[0]];
        (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt().atan2(e[0] * d[0] + e[1] * d[1] + e[2] * d[2])
    }

    #[test]
    fn al7_large_tilt_and_large_yaw_are_realigned_with_the_yaw_kept() {
        let t = Tuning::DEFAULT;
        let (roll, pitch) = (35.0_f64.to_radians(), 20.0_f64.to_radians());
        // Truth at rest, tilted: the body-frame down axis d and the specific force -g d.
        let d = [-pitch.sin(), roll.sin() * pitch.cos(), roll.cos() * pitch.cos()];
        let g = f64::from(GRAVITY_MSS);
        let force = d.map(|x| (-g * x) as f32);
        for rate in [1.0f64, -1.0] {
            for aiding in [Aiding::ImuBaro, Aiding::Imu] {
                let mut f = Eskf::with_aiding(&t, aiding).unwrap();
                let mut w = World::new(&t);
                let sample = |w: &mut World, gyro: [f64; 3]| {
                    let mut s = w.step();
                    s.imu1.reading.accel_ms2 = force;
                    s.imu1.reading.gyro_rads = gyro.map(|x| x as f32);
                    s
                };
                // A true turn about the vertical at `rate` rad/s: the body rates are rate * d; 3.05 s gives 174.7 deg.
                // Then a 0.5 s pulse the vehicle did not make, and rest until 5 s.
                while w.t_us() < 5_000_000 {
                    let gyro = if w.t_us() < 3_050_000 {
                        d.map(|x| rate * x)
                    } else if w.t_us() >= 3_500_000 && w.t_us() < 4_000_000 {
                        [0.2, -0.1, 0.0]
                    } else {
                        [0.0; 3]
                    };
                    let s = sample(&mut w, gyro);
                    f.set_armed(false);
                    f.update(&s);
                }
                let next: Vec<Sensors> = (0..80).map(|_| sample(&mut w, [0.0; 3])).collect();
                let (mut yaw_worst, mut before_min, mut after_worst, mut ba_worst) = (0.0f64, f64::INFINITY, 0.0f64, 0.0f64);
                let yaw0 = yaw_of(&f.nom.q);
                assert!(yaw0.abs() > 170.0_f64.to_radians() && yaw0.abs() < 178.0_f64.to_radians(), "yaw {yaw0}");
                for j in 0..16 {
                    let e = edge_at(&f, &next, j);
                    let (before, after) = (down_err_rad(&e.nav_pre.attitude, d), down_err_rad(&e.na.attitude, d));
                    let dy = yaw_of(&e.na.attitude) - yaw_of(&e.pre.nom.q);
                    assert!(e.tilt_true);
                    assert!(before > 1.0_f64.to_radians(), "j {j}: error before {before}");
                    // The input is the bias-corrected force: the truth differs from it by the learned accelerometer bias, at
                    // most |ba| / |f| rad (|ba| the norm of the three), and the alignment of the stored force is exact.
                    let ba = e.pre.nom.ba.map(f64::from);
                    let bound = (ba[0] * ba[0] + ba[1] * ba[1] + ba[2] * ba[2]).sqrt() / g + 1e-6;
                    assert!(after <= bound, "{aiding:?} rate {rate} j {j}: down-axis error after {after} rad, bound {bound:e}");
                    let f = e.pre.arm_f.expect("a stored interval").map(f64::from);
                    let (roll_a, pitch_a) = ((-f[1]).atan2(-f[2]), f[0].atan2(f[1].hypot(f[2])));
                    let (r1, p1) = roll_pitch(&e.na.attitude);
                    assert!((r1 - roll_a).abs() < 1e-6 && (p1 - pitch_a).abs() < 1e-6, "j {j}: roll {r1} vs {roll_a}, pitch {p1} vs {pitch_a}");
                    ba_worst = ba_worst.max(bound - 1e-6);
                    assert!(dy.abs() < 1e-6, "{aiding:?} rate {rate} j {j}: yaw change {dy}");
                    before_min = before_min.min(before);
                    after_worst = after_worst.max(after);
                    yaw_worst = yaw_worst.max(dy.abs());
                }
                println!(
                    "al7 {aiding:?} rate {rate}: yaw before the edge {:.2} deg; down-axis error before >= {:.3} deg, after <= {after_worst:e} rad (|ba|/g up to {ba_worst:e}); worst yaw change {yaw_worst:e} rad",
                    yaw0.to_degrees(),
                    before_min.to_degrees()
                );
            }
        }
    }

    // ---- Amendment E, packet E-1: the ImuBaro barometer scalar does not update b_a,x and b_a,y ----

    /// Specific force, body frame, of a vehicle at rest rolled by `roll` and pitched by `pitch` (the inverse of the first
    /// alignment's formulas), plus the constant accelerometer offset `off`.
    fn rest_force(roll: f64, pitch: f64, off: [f32; 3]) -> [f32; 3] {
        let g = f64::from(GRAVITY_MSS);
        let f = [g * pitch.sin(), -g * pitch.cos() * roll.sin(), -g * pitch.cos() * roll.cos()];
        core::array::from_fn(|i| f[i] as f32 + off[i])
    }

    /// `secs` of an `ImuBaro` filter held tilted (`tilt` = roll, pitch, rad) at the World's constant barometer height with the
    /// accelerometer offset `off`, disarmed until `arm_s` s and armed after. Returns the filter, the largest |b_a,x| and |b_a,y|
    /// seen, whether b_a,x and b_a,y were bit-exact 0.0 after every step, and the largest |v_D| over the last half.
    fn held_tilted(t: &Tuning, tilt: (f64, f64), off: [f32; 3], secs: f64, arm_s: f64, masked: bool) -> (Eskf, [f32; 2], bool, f32) {
        let mut f = Eskf::with_aiding(t, Aiding::ImuBaro).unwrap();
        f.baro_full_gain = !masked;
        let mut w = World::new(t);
        let (mut worst, mut exact, mut vd) = ([0.0f32; 2], true, 0.0f32);
        for _ in 0..(secs * 1e6) as u64 / PERIOD_US {
            let mut s = w.step();
            s.imu1.reading.accel_ms2 = rest_force(tilt.0, tilt.1, off);
            f.set_armed(w.t_us() as f64 * 1e-6 >= arm_s);
            let nav = f.update(&s);
            let ba = f.biases().1;
            exact &= ba[0].to_bits() == 0 && ba[1].to_bits() == 0;
            worst = [worst[0].max(fabsf(ba[0])), worst[1].max(fabsf(ba[1]))];
            if w.t_us() as f64 * 1e-6 >= 0.5 * secs {
                vd = vd.max(fabsf(nav.velocity_ms[2]));
            }
        }
        (f, worst, exact, vd)
    }

    #[test]
    fn e1_t1_imu_baro_tilted_with_a_horizontal_accelerometer_offset_keeps_b_a_x_and_y_at_exactly_zero() {
        let t = Tuning::DEFAULT;
        // 10 deg roll and 20 deg pitch: the tilt is inside angle_max_rad (30 deg) and both horizontal body axes have a
        // vertical component, which is what lets the barometer's height error reach b_a,x and b_a,y.
        let tilt = (10.0_f64.to_radians(), 20.0_f64.to_radians());
        assert!(tilt.0.hypot(tilt.1) < f64::from(t.angle_max_rad));
        let off = [0.15f32, -0.1, 0.0];
        let (f, _, exact, _) = held_tilted(&t, tilt, off, 70.0, 25.0, true);
        assert!(exact, "b_a,x or b_a,y left 0.0");
        assert!(f.stats().baro_fused > 1000 && f.stats().predicts > 1000);
        // Control: the mask off, the same input.
        let (c, worst, exact_c, _) = held_tilted(&t, tilt, off, 70.0, 25.0, false);
        println!("e1 t1: offset {off:?}; masked ba {:?} (bit-exact zero every step); mask off: ba {:?}, largest |ba_x| {:e}, |ba_y| {:e}", f.biases().1, c.biases().1, worst[0], worst[1]);
        assert!(!exact_c && worst[0] > 0.0 && worst[1] > 0.0, "the control did not move: {worst:?}");
    }

    #[test]
    fn e1_t2_imu_baro_level_with_a_vertical_offset_still_learns_b_a_z() {
        let t = Tuning::DEFAULT;
        let off = [0.0f32, 0.0, 0.2];
        let run = |z_masked: bool| {
            let mut f = Eskf::with_aiding(&t, Aiding::ImuBaro).unwrap();
            f.baro_abias_z_mask = z_masked;
            let mut w = World::new(&t);
            let (mut sq, mut n, mut vmax) = (0.0f64, 0u32, 0.0f64);
            for _ in 0..60_000_000 / PERIOD_US {
                let mut s = w.step();
                s.imu1.reading.accel_ms2 = rest_force(0.0, 0.0, off);
                f.set_armed(w.t_us() >= 20_000_000);
                let nav = f.update(&s);
                if w.t_us() >= 30_000_000 {
                    // Truth: at rest, v_D = 0.
                    let e = f64::from(nav.velocity_ms[2]);
                    sq += e * e;
                    vmax = vmax.max(e.abs());
                    n += 1;
                }
            }
            (f.biases().1, (sq / f64::from(n)).sqrt(), vmax)
        };
        let (ba, rms, vmax) = run(false);
        let (ba_m, rms_m, vmax_m) = run(true);
        println!("e1 t2: offset {off:?}; ba_z after 60 s {:e} (offset {}); v_D error over the last 30 s rms {rms:e} max {vmax:e}; z masked: ba_z {:e}, rms {rms_m:e} max {vmax_m:e}", ba[2], off[2], ba_m[2]);
        assert_eq!((ba[0].to_bits(), ba[1].to_bits()), (0, 0));
        assert!(ba[2] > 0.0 && fabsf(ba[2] - off[2]) < off[2], "ba_z {} does not move toward {}", ba[2], off[2]);
        assert_eq!((ba_m[0].to_bits(), ba_m[1].to_bits(), ba_m[2].to_bits()), (0, 0, 0));
        assert!(rms_m > rms, "z masked rms {rms_m} not larger than {rms}");
    }

    #[test]
    fn e1_t3_the_arming_edge_aligns_to_the_raw_horizontal_force_with_the_barometer_running() {
        let t = Tuning::DEFAULT;
        let off = [0.15f32, -0.1, 0.0];
        let g = f64::from(GRAVITY_MSS);
        // The AL1 fixture with the offset on every sample, with the handling made physical: held tilted (10 deg roll, 20 deg
        // pitch, inside angle_max_rad) for 60 s, where the barometer's height error reaches b_a,x and b_a,y when the mask is
        // off; set down level in 0.5 s (a gyro pulse of the same angles); rest to 64 s; then 80 samples at rest.
        let tilt = (10.0_f64.to_radians(), 20.0_f64.to_radians());
        let fixture = |masked: bool| {
            let mut f = Eskf::with_aiding(&t, Aiding::ImuBaro).unwrap();
            f.baro_full_gain = !masked;
            let mut w = World::new(&t);
            while w.t_us() < 64_000_000 {
                let mut s = w.step();
                let held = w.t_us() < 60_000_000;
                s.imu1.reading.accel_ms2 = if held { rest_force(tilt.0, tilt.1, off) } else { rest_force(0.0, 0.0, off) };
                if (60_000_000..60_500_000).contains(&w.t_us()) {
                    s.imu1.reading.gyro_rads = [-2.0 * tilt.0 as f32, -2.0 * tilt.1 as f32, 0.0];
                }
                f.set_armed(false);
                f.update(&s);
            }
            let next: Vec<Sensors> = (0..80)
                .map(|_| {
                    let mut s = w.step();
                    s.imu1.reading.accel_ms2 = rest_force(0.0, 0.0, off);
                    s
                })
                .collect();
            (f, next)
        };
        let mut departures = [0.0f64; 2];
        for (k, masked) in [true, false].into_iter().enumerate() {
            let (base, next) = fixture(masked);
            let (mut worst, mut bound_max, mut ba_x) = (0.0f64, 0.0f64, 0.0f32);
            for j in 0..16 {
                let e = edge_at(&base, &next, j);
                let f = e.pre.arm_f.expect("a stored interval").map(f64::from);
                // The raw mean force of the interval: the samples are constant, so it is the sample, with the z of the sample.
                let raw = [f64::from(off[0]), f64::from(off[1]), -g];
                if masked {
                    // x and y of the stored force are the raw ones up to the f32 mean of constant samples.
                    assert!((f[0] - raw[0]).abs() <= 1e-6 * raw[0].abs() && (f[1] - raw[1]).abs() <= 1e-6 * raw[1].abs(), "j {j}: stored {f:?} raw {raw:?}");
                }
                let roll = (-raw[1]).atan2(-raw[2]);
                let pitch = raw[0].atan2(raw[1].hypot(raw[2]));
                let (r1, p1) = roll_pitch(&e.na.attitude);
                let d = (r1 - roll).abs().max((p1 - pitch).abs());
                // The stored z is the raw z minus b_a,z; roll = atan2(-f_y, -f_z) changes by at most |f_y| d / (f_y^2 + f_z^2) and
                // pitch = atan2(f_x, hypot(f_y, f_z)) by at most |f_x| d / hypot(f_y, f_z)^2 for a change d of f_z, with
                // d = |b_a,z| and hypot >= g - d; the bound adds 1e-6 rad for the f32 rounding of the alignment.
                let dz = f64::from(e.pre.nom.ba[2]).abs();
                let bound = raw[0].abs().max(raw[1].abs()) * dz / ((g - dz) * (g - dz)) + 1e-6;
                if masked {
                    assert_eq!((e.pre.nom.ba[0].to_bits(), e.pre.nom.ba[1].to_bits()), (0, 0));
                    assert!(d <= bound, "j {j}: roll {r1} vs {roll}, pitch {p1} vs {pitch}, bound {bound:e}");
                    let y0 = yaw_of(&e.pre.nom.q);
                    assert!((yaw_of(&e.na.attitude) - y0).abs() < 1e-6);
                } else {
                    ba_x = ba_x.max(fabsf(e.pre.nom.ba[0]));
                    assert!(d > 100.0 * bound, "j {j}: the mask-off tilt differs by only {d:e}, bound {bound:e}");
                    // About the learned bias over g: the pitch departure is b_a,x / g to the bound, the roll's b_a,y / g.
                    let (ba, ratio) = (f64::from(e.pre.nom.ba[0]), (p1 - pitch).abs() * g);
                    assert!((ratio - ba.abs()).abs() <= 0.05 * ba.abs() + bound * g, "j {j}: pitch departure * g {ratio:e}, |b_a,x| {}", ba.abs());
                }
                worst = worst.max(d);
                bound_max = bound_max.max(bound);
            }
            departures[k] = worst;
            println!("e1 t3: mask {masked}: largest roll/pitch departure from the raw-force alignment {worst:e} rad (bound {bound_max:e}; offset/g = {:e}, ba_x before the edge {ba_x:e})", f64::from(off[0]) / g);
        }
        assert!(departures[1] > departures[0]);
    }

    #[test]
    fn e1_t4_a_rejected_barometer_changes_nothing_resets_the_height_in_time_and_p_stays_a_covariance() {
        let t = Tuning::DEFAULT;
        // (a) Warm filter, then a direct call beyond the gate: bit-identical, rejected.
        let (f, _, _, _) = held_tilted(&t, (0.2, 0.3), [0.15, -0.1, 0.1], 20.0, 5.0, true);
        let r = t.nav_baro_sigma_m * t.nav_baro_sigma_m;
        let s = f.p[POS + 2][POS + 2] + r;
        let innov = 1.001 * t.nav_gate_sigma * sqrtf(s);
        let (mut a, before) = (f.clone(), bits(&f));
        assert_eq!(a.fuse_baro_masked(innov, r), (false, s));
        assert_eq!(bits(&a), before);
        assert_eq!(a.fuse_baro_masked(-innov, r), (false, s));
        assert_eq!(bits(&a), before);
        assert!(a.fuse_baro_masked(0.999 * t.nav_gate_sigma * sqrtf(s), r).0, "just inside the gate was rejected");
        // The unmasked path gates on the same test.
        let mut u = f.clone();
        assert!(!u.fuse_unit(POS + 2, innov, r).0 && bits(&u) == before);
        // (b) End to end: the barometer 100 m high from 20 s on, armed; counted as gated, nothing fused, and the height reset
        // at the timeout after the last accepted sample.
        let mut f = Eskf::with_aiding(&t, Aiding::ImuBaro).unwrap();
        let mut w = World::new(&t);
        let (mut last_aid, mut reset_at) = (0u64, None);
        for _ in 0..40_000_000 / PERIOD_US {
            let mut s = w.step();
            s.imu1.reading.accel_ms2 = rest_force(0.2, 0.3, [0.0; 3]);
            if w.t_us() >= 20_000_000 {
                if let Some(b) = s.baro.as_mut() {
                    b.reading.pressure_pa = pressure_at(100.0);
                }
            }
            f.set_armed(w.t_us() >= 5_000_000);
            let (aid, resets) = (f.last_baro_aid_us, f.stats().height_resets);
            f.update(&s);
            if reset_at.is_none() && w.t_us() >= 20_000_000 && f.stats().height_resets > resets {
                (last_aid, reset_at) = (aid.unwrap(), Some(f.horizon()));
            }
        }
        let gated = f.stats().baro_gated;
        let at = reset_at.expect("no height reset");
        println!("e1 t4: last accepted barometer aid at {last_aid} us, height reset at {at} us ({} us later, timeout {}), gated samples {gated}", at - last_aid, t.nav_aid_timeout_us);
        assert!(gated > 0 && at - last_aid >= t.nav_aid_timeout_us && at - last_aid < t.nav_aid_timeout_us + 100_000, "{}", at - last_aid);
        assert!(f.stats().baro_fused > 1000);
        // (c) 120 s of mixed motion with arm changes and a barometer dropout and a bad stretch: P a covariance throughout.
        let mut f = Eskf::with_aiding(&t, Aiding::ImuBaro).unwrap();
        let mut w = World::new(&t);
        let mut bad = 0u32;
        for k in 0..120_000_000 / PERIOD_US {
            let sec = w.t_us() as f64 * 1e-6;
            let phase = (sec / 7.0) as u32;
            w.a = [[0.0, 1.5, -1.0, 0.5][phase as usize % 4], [0.5, 0.0, 1.0, -2.0][phase as usize % 4], [0.3, -0.4, 0.0, 0.8][phase as usize % 4]];
            w.yaw_rate = 0.3 * ((phase % 3) as f64 - 1.0);
            let mut s = w.step();
            let tilt = (0.3 * (sec * 0.7).sin(), 0.4 * (sec * 0.45).cos());
            s.imu1.reading.accel_ms2 = {
                let a = rest_force(tilt.0, tilt.1, [0.15, -0.1, 0.1]);
                [a[0] + 0.2 * s.imu1.reading.accel_ms2[0], a[1] + 0.2 * s.imu1.reading.accel_ms2[1], a[2] + 0.2 * (s.imu1.reading.accel_ms2[2] + GRAVITY_MSS)]
            };
            s.imu1.reading.gyro_rads = [(0.2 * (sec * 1.3).sin()) as f32, (0.15 * (sec * 0.9).cos()) as f32, s.imu1.reading.gyro_rads[2]];
            if (60.0..75.0).contains(&sec) {
                s.baro = None;
            } else if (90.0..100.0).contains(&sec) {
                if let Some(b) = s.baro.as_mut() {
                    b.reading.pressure_pa = pressure_at(60.0);
                }
            }
            f.set_armed(sec >= 10.0 && (sec < 50.0 || sec >= 80.0));
            f.update(&s);
            if k % 8 == 0 {
                let sym = (0..N).all(|i| (0..N).all(|j| f.p[i][j].to_bits() == f.p[j][i].to_bits()));
                let fin = f.finite() && (0..N).all(|i| f.p[i][i] >= 0.0);
                bad += u32::from(!(sym && fin));
            }
        }
        let st = f.stats();
        println!("e1 t4: 120 s mixed motion, steps with P not symmetric/finite/non-negative {bad}, stats baro fused {} gated {} resets {} reinits {}", st.baro_fused, st.baro_gated, st.height_resets, st.reinits);
        assert_eq!(bad, 0);
        assert!(st.baro_fused > 1000 && st.baro_gated > 0);
    }

    #[test]
    fn e1_t5_the_masked_update_equals_an_independent_f64_joseph_reference() {
        let t = Tuning::DEFAULT;
        let (f, _, _, _) = held_tilted(&t, (0.2, 0.3), [0.15, -0.1, 0.1], 20.0, 5.0, true);
        let r = f64::from(t.nav_baro_sigma_m * t.nav_baro_sigma_m);
        let innov = 0.3f32;
        let k = POS + 2;
        let p: Vec<Vec<f64>> = f.p.iter().map(|row| row.iter().map(|&x| f64::from(x)).collect()).collect();
        let s = p[k][k] + r;
        // K = P H^T / s with every entry but D, v_D and b_a,z zeroed (H1); (I - K H) P (I - K H)^T + K R K^T.
        let mut kg: Vec<f64> = (0..N).map(|i| p[i][k] / s).collect();
        for (i, v) in kg.iter_mut().enumerate() {
            if i != POS + 2 && i != VEL + 2 && i != ACCEL_BIAS + 2 {
                *v = 0.0;
            }
        }
        let imkh = |i: usize, j: usize| f64::from(u8::from(i == j)) - kg[i] * f64::from(u8::from(j == k));
        let reference: Vec<Vec<f64>> = (0..N)
            .map(|i| {
                (0..N)
                    .map(|j| {
                        let mut v = kg[i] * r * kg[j];
                        for a in 0..N {
                            for b in 0..N {
                                v += imkh(i, a) * p[a][b] * imkh(j, b);
                            }
                        }
                        v
                    })
                    .collect()
            })
            .collect();
        let mut g = f.clone();
        assert!(g.fuse_baro_masked(innov, r as f32).0);
        let mut worst = 0.0f64;
        for i in 0..N {
            for j in 0..N {
                let scale = (p[i][i] * p[j][j]).sqrt().max(1e-30);
                worst = worst.max((f64::from(g.p[i][j]) - reference[i][j]).abs() / scale);
            }
        }
        // The state: dx = K innovation, so b_a,x and b_a,y stay 0 and b_a,z moves by K_z innovation.
        let dz = kg[ACCEL_BIAS + 2] * f64::from(innov);
        let moved = f64::from(g.nom.ba[2] - f.nom.ba[2]);
        println!("e1 t5: worst |P - reference| / sqrt(P_ii P_jj) {worst:e}; b_a,z step {moved:e} vs reference {dz:e}; unmasked K of b_a,x {:e}", p[ACCEL_BIAS][k] / s);
        assert!(worst < 1e-5, "{worst:e}");
        assert!(p[ACCEL_BIAS][k].abs() > 0.0 && p[ACCEL_BIAS + 1][k].abs() > 0.0, "the masked entries were already 0");
        assert_eq!((g.nom.ba[0].to_bits(), g.nom.ba[1].to_bits()), (f.nom.ba[0].to_bits(), f.nom.ba[1].to_bits()));
        // The difference of two f32 values near |b_a,z|: rounding of 4 ulp of it, plus the 1e-6 of dz of the f32 K.
        assert!((moved - dz).abs() <= 4.0 * f64::from(f32::EPSILON) * f64::from(g.nom.ba[2].abs()) + 1e-6 * dz.abs(), "{moved:e} vs {dz:e}");
    }

    #[test]
    fn e1_f_i_full_and_imu_are_bit_identical_with_the_mask_on_and_off() {
        let t = Tuning::DEFAULT;
        for aiding in [Aiding::Full, Aiding::Imu] {
            let (mut a, mut b) = (Eskf::with_aiding(&t, aiding).unwrap(), Eskf::with_aiding(&t, aiding).unwrap());
            b.baro_full_gain = true;
            let mut w = World::new(&t);
            w.v = [1.0, 0.5, 0.0];
            let mut ba_moved = false;
            for _ in 0..30_000_000 / PERIOD_US {
                let s = w.step();
                let armed = w.t_us() >= 5_000_000;
                a.set_armed(armed);
                b.set_armed(armed);
                assert_eq!(a.update(&s), b.update(&s));
                assert_eq!(a.biases(), b.biases());
                assert_eq!(bits(&a), bits(&b));
                ba_moved |= a.biases().1 != [0.0; 3];
            }
            assert_eq!(a.stats(), b.stats());
            println!("e1 f/i {aiding:?}: stats {:?}, accelerometer bias ever non-zero {ba_moved}", a.stats());
            if aiding == Aiding::Full {
                assert!(a.stats().gps_pos_fused > 0 && a.stats().mag_fused > 0 && a.stats().baro_fused > 0);
            } else {
                assert!(a.stats().noaid > 0 && a.stats().baro_fused == 0);
            }
        }
    }

    // ---- Amendment H, packet H-1: the ImuBaro barometer scalar has support on D, v_D and b_a,z only; the z rate is the PT1 ----

    /// One sample of the warm-up and gusty runs: the World moving (accelerations changing every 7 s, a yaw rate), the body
    /// tilted by slow rolls and pitches, a constant accelerometer offset and a gyro x and y wobble.
    fn moving_sample(w: &mut World, sec: f64) -> Sensors {
        let phase = (sec / 7.0) as u32;
        w.a = [[0.0, 1.5, -1.0, 0.5][phase as usize % 4], [0.5, 0.0, 1.0, -2.0][phase as usize % 4], [0.3, -0.4, 0.0, 0.8][phase as usize % 4]];
        w.yaw_rate = 0.3 * ((phase % 3) as f64 - 1.0);
        let mut s = w.step();
        let tilt = (0.3 * (sec * 0.7).sin(), 0.4 * (sec * 0.45).cos());
        s.imu1.reading.accel_ms2 = {
            let a = rest_force(tilt.0, tilt.1, [0.15, -0.1, 0.1]);
            [a[0] + 0.2 * s.imu1.reading.accel_ms2[0], a[1] + 0.2 * s.imu1.reading.accel_ms2[1], a[2] + 0.2 * (s.imu1.reading.accel_ms2[2] + GRAVITY_MSS)]
        };
        s.imu1.reading.gyro_rads = [(0.2 * (sec * 1.3).sin()) as f32, (0.15 * (sec * 0.9).cos()) as f32, s.imu1.reading.gyro_rads[2]];
        s
    }

    /// A warm `ImuBaro` filter: `secs` of `moving_sample`, armed from 10 s.
    fn warm_imu_baro(t: &Tuning, secs: u64) -> Eskf {
        let mut f = Eskf::with_aiding(t, Aiding::ImuBaro).unwrap();
        let mut w = World::new(t);
        for _ in 0..secs * 1_000_000 / PERIOD_US {
            let sec = w.t_us() as f64 * 1e-6;
            let s = moving_sample(&mut w, sec);
            f.set_armed(sec >= 10.0);
            f.update(&s);
        }
        f
    }

    #[test]
    fn h_t1_one_imu_baro_barometer_fusion_changes_only_d_vd_and_ba_z() {
        let t = Tuning::DEFAULT;
        let f = warm_imu_baro(&t, 40);
        let (d, vd, bz) = (POS + 2, VEL + 2, ACCEL_BIAS + 2);
        let outside = |i: usize| i != d && i != vd && i != bz;
        // Precondition: P has non-zero cross terms between D / v_D and the attitude, the horizontal velocity and the gyro bias.
        for k in [d, vd] {
            for j in (ATT..ATT + 3).chain(VEL..VEL + 2).chain(GYRO_BIAS..GYRO_BIAS + 3) {
                assert!(f.p[k][j] != 0.0 && f.p[k][j].is_finite(), "P[{k}][{j}] = {}", f.p[k][j]);
            }
        }
        let r = f.t.nav_baro_sigma_m * f.t.nav_baro_sigma_m;
        let innov = 0.3f32;
        assert!(innov * innov <= f.t.nav_gate_sigma * f.t.nav_gate_sigma * (f.p[d][d] + r), "the innovation is outside the gate");
        let mut a = f.clone();
        assert!(a.fuse_baro_masked(innov, r).0, "the fusion was not accepted");
        let q_bits = |e: &Eskf| e.nom.q.map(f32::to_bits);
        assert_eq!(q_bits(&a), q_bits(&f), "attitude");
        assert_eq!(a.nom.bg.map(f32::to_bits), f.nom.bg.map(f32::to_bits), "gyro bias");
        assert_eq!([a.nom.v[0].to_bits(), a.nom.v[1].to_bits()], [f.nom.v[0].to_bits(), f.nom.v[1].to_bits()], "v_N, v_E");
        assert_eq!([a.nom.p[0].to_bits(), a.nom.p[1].to_bits()], [f.nom.p[0].to_bits(), f.nom.p[1].to_bits()], "p_N, p_E");
        assert_eq!([a.nom.ba[0].to_bits(), a.nom.ba[1].to_bits()], [f.nom.ba[0].to_bits(), f.nom.ba[1].to_bits()], "b_a,x, b_a,y");
        for i in (0..N).filter(|&i| outside(i)) {
            for j in (0..N).filter(|&j| outside(j)) {
                assert_eq!(a.p[i][j].to_bits(), f.p[i][j].to_bits(), "P[{i}][{j}]");
            }
        }
        let moved = [a.nom.p[2] != f.nom.p[2], a.nom.v[2] != f.nom.v[2], a.nom.ba[2] != f.nom.ba[2]];
        assert!(moved.iter().all(|m| *m), "D, v_D, b_a,z moved: {moved:?}");
        let inside_changed = (0..N).filter(|&i| !outside(i)).any(|i| (0..N).any(|j| a.p[i][j].to_bits() != f.p[i][j].to_bits()));
        assert!(inside_changed, "P did not change inside the vertical rows");
        // Control: the full gain (the test-only switch) on the same filter and innovation moves the attitude and the gyro bias.
        // Tolerance: 100 x f32 round-off (EPSILON) of the magnitude of the quantity (the largest |component|).
        let mut c = f.clone();
        c.baro_full_gain = true;
        assert!(c.fuse_baro_masked(innov, r).0);
        let dq = (0..4).fold(0.0f32, |m, i| m.max(fabsf(c.nom.q[i] - f.nom.q[i])));
        let tol_q = 100.0 * f32::EPSILON * f.nom.q.iter().fold(0.0f32, |m, v| m.max(fabsf(*v)));
        let dbg = (0..3).fold(0.0f32, |m, i| m.max(fabsf(c.nom.bg[i] - f.nom.bg[i])));
        let tol_bg = 100.0 * f32::EPSILON * f.nom.bg.iter().fold(0.0f32, |m, v| m.max(fabsf(*v)));
        println!(
            "h t1: masked: D {:e} -> {:e}, vD {:e} -> {:e}, ba_z {:e} -> {:e}, attitude/bg/vNE/pNE/ba_xy bit-identical; full gain: max |dq| {dq:e} (tolerance {tol_q:e}), max |dbg| {dbg:e} rad/s (tolerance {tol_bg:e}), dv_N {:e}",
            f.nom.p[2], a.nom.p[2], f.nom.v[2], a.nom.v[2], f.nom.ba[2], a.nom.ba[2], c.nom.v[0] - f.nom.v[0]
        );
        assert!(dq > tol_q && dbg > tol_bg, "the control did not move: {dq:e} vs {tol_q:e}, {dbg:e} vs {tol_bg:e}");
    }

    #[test]
    fn h_t2_a_long_gusty_run_keeps_p_a_covariance() {
        let t = Tuning::DEFAULT;
        // The checker: symmetric (bit-equal [i][j] and [j][i]), finite, a non-negative diagonal.
        let covariance = |f: &Eskf| (0..N).all(|i| (0..N).all(|j| f.p[i][j].to_bits() == f.p[j][i].to_bits())) && f.finite() && (0..N).all(|i| f.p[i][i] >= 0.0);
        let mut f = Eskf::with_aiding(&t, Aiding::ImuBaro).unwrap();
        let mut w = World::new(&t);
        let mut bad = 0u32;
        let mut baro_off = 0.0f64;
        for _ in 0..120_000_000 / PERIOD_US {
            let sec = w.t_us() as f64 * 1e-6;
            let mut s = moving_sample(&mut w, sec);
            // Barometer steps of 1 to 2 m (gusts), held for several seconds.
            baro_off = if (20.0..28.0).contains(&sec) {
                1.5
            } else if (40.0..46.0).contains(&sec) {
                -2.0
            } else if (60.0..64.0).contains(&sec) || (70.0..71.0).contains(&sec) {
                1.0
            } else if (85.0..95.0).contains(&sec) {
                -1.5
            } else if (100.0..101.0).contains(&sec) {
                2.0
            } else {
                0.0
            };
            if baro_off != 0.0 {
                if let Some(b) = s.baro.as_mut() {
                    b.reading.pressure_pa = pressure_at(-w.p[2] + baro_off);
                }
            }
            f.set_armed(sec >= 10.0 && (sec < 50.0 || (sec >= 55.0 && sec < 80.0) || sec >= 90.0));
            f.update(&s);
            bad += u32::from(!covariance(&f));
        }
        let st = f.stats();
        println!("h t2: 120 s gusty, steps with P not symmetric/finite/non-negative {bad}, baro fused {} gated {} resets {} reinits {} (last offset {baro_off})", st.baro_fused, st.baro_gated, st.height_resets, st.reinits);
        assert_eq!(bad, 0);
        assert!(st.baro_fused > 1000 && st.baro_gated + st.height_resets > 0, "the gusts did not reach the gate or the reset");
        // Control: the checker flags an asymmetry of one ulp, a negative diagonal and a NaN.
        let mut c = f.clone();
        c.p[3][7] = f32::from_bits(c.p[3][7].to_bits() ^ 1);
        assert!(!covariance(&c));
        let mut c = f.clone();
        c.p[4][4] = -1e-9;
        assert!(!covariance(&c));
        let mut c = f.clone();
        c.p[POS + 2][POS + 2] = f32::NAN;
        assert!(!covariance(&c));
    }

    #[test]
    fn h_t3_ba_z_still_converges_while_held_tilted_about_20_deg() {
        let t = Tuning::DEFAULT;
        let off = [0.0f32, 0.0, 0.2];
        // 20 deg of pitch (inside angle_max_rad): the offset on the body z axis has a vertical component cos 20 deg.
        let tilt = (0.0_f64, 20.0_f64.to_radians());
        let run = |z_masked: bool| {
            let mut f = Eskf::with_aiding(&t, Aiding::ImuBaro).unwrap();
            f.baro_abias_z_mask = z_masked;
            let mut w = World::new(&t);
            let (mut sq, mut n, mut vmax) = (0.0f64, 0u32, 0.0f64);
            for _ in 0..60_000_000 / PERIOD_US {
                let mut s = w.step();
                s.imu1.reading.accel_ms2 = rest_force(tilt.0, tilt.1, off);
                f.set_armed(w.t_us() >= 20_000_000);
                let nav = f.update(&s);
                if w.t_us() >= 30_000_000 {
                    // Truth: at rest, v_D = 0.
                    let e = f64::from(nav.velocity_ms[2]);
                    sq += e * e;
                    vmax = vmax.max(e.abs());
                    n += 1;
                }
            }
            (f.biases().1, (sq / f64::from(n)).sqrt(), vmax)
        };
        let (ba, rms, vmax) = run(false);
        let (ba_m, rms_m, vmax_m) = run(true);
        println!("h t3: tilt 20 deg, offset {off:?}; ba_z after 60 s {:e} (offset {}); v_D error over the last 30 s rms {rms:e} max {vmax:e}; z masked: ba_z {:e}, rms {rms_m:e} max {vmax_m:e}", ba[2], off[2], ba_m[2]);
        assert_eq!((ba[0].to_bits(), ba[1].to_bits()), (0, 0));
        assert!(ba[2] > 0.0 && fabsf(ba[2] - off[2]) < off[2], "ba_z {} does not move toward {}", ba[2], off[2]);
        assert_eq!((ba_m[0].to_bits(), ba_m[1].to_bits(), ba_m[2].to_bits()), (0, 0, 0));
        assert!(rms_m > rms, "z masked rms {rms_m} not larger than {rms}");
    }

    #[test]
    fn h_t5_imu_baro_and_imu_rates_z_is_mahonys_for_any_z_bias_and_full_is_unchanged() {
        let t = Tuning::DEFAULT;
        for aiding in [Aiding::ImuBaro, Aiding::Imu, Aiding::Full] {
            let mut f = Eskf::with_aiding(&t, aiding).unwrap();
            let mut m = Mahony::new(&t);
            let mut w = World::new(&t);
            let (mut worst_ctl, mut ctl_differs, mut bg_min, mut aligned, mut nonzero) = (0.0f64, 0u32, f32::INFINITY, false, 0u32);
            for k in 0..20_000_000 / PERIOD_US {
                let mut s = w.step();
                let ph = k as f32 * PERIOD_US as f32 * 1e-6;
                s.imu1.reading.gyro_rads = [0.3 * sinf(2.0 * PI * 1.7 * ph), 0.2 * cosf(2.0 * PI * 0.6 * ph), 0.1 * sinf(2.0 * PI * 3.1 * ph) + 0.03];
                let armed = w.t_us() >= 4_000_000;
                f.set_armed(armed);
                m.set_armed(armed);
                // The z bias estimate is set directly before every step (the alignment and the filter's own learning would
                // otherwise leave it at or near 0 here).
                f.nom.bg[2] = 0.05;
                let (nf, nm) = (f.update(&s), m.update(&s));
                // The step that aligns the tilt resets the bias; the next one is the first checked.
                let checked = aligned;
                aligned = nf.valid.tilt;
                if !checked {
                    continue;
                }
                let bg = f.biases().0;
                bg_min = bg_min.min(fabsf(bg[2]));
                if aiding == Aiding::Full {
                    // The compass alignment resets b_g,z once; the identity is checked on every step, the bias non-zero on most.
                    nonzero += u32::from(bg[2] != 0.0);
                    // Unchanged: the raw gyro of the horizon sample minus the full bias.
                    for i in 0..3 {
                        assert_eq!(nf.rates_rads[i].to_bits(), (f.horizon_gyro[i] - bg[i]).to_bits(), "Full step {k} axis {i}");
                    }
                    continue;
                }
                assert!(bg[2] != 0.0, "{aiding:?} step {k}: the z bias is 0");
                assert_eq!(nf.rates_rads[2].to_bits(), nm.rates_rads[2].to_bits(), "{aiding:?} step {k}: z differs from Mahony's");
                for i in 0..2 {
                    assert_eq!(nf.rates_rads[i].to_bits(), (nm.rates_rads[i] - bg[i]).to_bits(), "{aiding:?} step {k} axis {i}");
                }
                // Control: the D1 form for z, PT1 - bg_z (in f64, the difference of two f32 values is exact up to its own
                // rounding, 2 EPSILON_64 |bg_z|), differs from the output by bg_z.
                let d1 = f64::from(nm.rates_rads[2]) - f64::from(bg[2]);
                let diff = f64::from(nf.rates_rads[2]) - d1;
                worst_ctl = worst_ctl.max((diff - f64::from(bg[2])).abs() / f64::from(bg[2]).abs());
                ctl_differs += u32::from((nm.rates_rads[2] - bg[2]).to_bits() != nf.rates_rads[2].to_bits());
                assert!((diff - f64::from(bg[2])).abs() <= 2.0 * f64::EPSILON * f64::from(bg[2]).abs(), "{aiding:?} step {k}: D1 form minus output {diff:e}, bg_z {}", bg[2]);
            }
            if aiding == Aiding::Full {
                println!("h t5 Full: rates = horizon gyro - full bias bit for bit on every step; b_g,z non-zero on {nonzero} steps");
                assert!(nonzero > 10_000);
            } else {
                println!("h t5 {aiding:?}: smallest |bg_z| over the run {bg_min:e}; D1 form differs from the output by bg_z (worst relative departure {worst_ctl:e}); f32 D1 form differs in bits on {ctl_differs} steps");
                assert!(ctl_differs > 0);
            }
        }
    }

    // ---- Packet P-G1: the ring length as a const generic, the snapshot and the reset hooks ----

    /// The 60 s stream of the G1 tests: yawed, moving at constant velocity, barometer, compass and GNSS; a non-finite IMU
    /// sample (index 20 000), a repeated timestamp (24 000) and one 20 ms gap before index 30 000.
    fn g1_stream(t: &Tuning, secs: u64) -> Vec<Sensors> {
        let mut w = World::new(t);
        w.yaw = 0.4;
        w.v = [2.0, 0.5, 0.0];
        w.gps_vel = true;
        let mut out: Vec<Sensors> = Vec::new();
        while w.t_us() < secs * 1_000_000 {
            let mut s = w.step();
            match out.len() {
                20_000 => s.imu1.reading.accel_ms2[1] = f32::NAN,
                24_000 => s.imu1.t_us = out.last().unwrap().imu1.t_us,
                30_000 => {
                    for _ in 0..15 {
                        w.step();
                    }
                    s = w.step();
                }
                _ => {}
            }
            out.push(s);
        }
        out
    }

    fn g1_finite(s: &Sensors) -> bool {
        let r = &s.imu1.reading;
        r.gyro_rads.iter().chain(&r.accel_ms2).all(|v| v.is_finite())
    }

    /// Every output bit of a NavState.
    fn g1_nav_bits(n: &NavState) -> Vec<u32> {
        let mut v: Vec<u32> = n.attitude.iter().chain(&n.rates_rads).chain(&n.velocity_ms).chain(&n.position_m).map(|x| x.to_bits()).collect();
        v.push(n.t_us as u32);
        v.push((n.t_us >> 32) as u32);
        let f = n.valid;
        v.extend([f.tilt, f.heading, f.rates, f.velocity_ne, f.velocity_d, f.position_ne, f.position_d].map(u32::from));
        match n.sigma {
            Some(s) => v.extend([s.tilt_rad, s.heading_rad, s.velocity_ne_ms, s.velocity_d_ms, s.position_ne_m, s.position_d_m].map(f32::to_bits)),
            None => v.push(u32::MAX),
        }
        v
    }

    /// The nominal state, the covariance, the biases and the sigmas of a filter, as bits.
    fn g1_bits<const R: usize>(e: &EskfN<R>) -> Vec<u32> {
        let n = &e.nom;
        let (bg, ba) = e.biases();
        let mut v: Vec<u32> = n.q.iter().chain(&n.v).chain(&n.p).chain(&n.bg).chain(&n.ba).chain(&n.dvel).chain(&e.hold_ne).chain(&e.rates_lpf).chain(&bg).chain(&ba).map(|x| x.to_bits()).collect();
        v.extend(e.p.iter().flatten().map(|x| x.to_bits()));
        v.extend(e.sigmas().map(f32::to_bits));
        v.push(n.dt_sum.to_bits());
        v
    }

    #[test]
    fn g1_a_a_ring_of_one_equals_the_full_ring_for_every_zero_delay_variant() {
        let zero = Tuning { nav_gps_delay_us: 0, ..Tuning::DEFAULT };
        for (aiding, t) in [(Aiding::ImuBaro, Tuning::DEFAULT), (Aiding::Imu, Tuning::DEFAULT), (Aiding::Full, zero)] {
            let stream = g1_stream(&t, 60);
            let mut a = EskfN::<1>::with_aiding(&t, aiding).unwrap();
            let mut b = EskfN::<RING>::with_aiding(&t, aiding).unwrap();
            for (k, s) in stream.iter().enumerate() {
                let armed = s.imu1.t_us >= 5_000_000;
                a.set_armed(armed);
                b.set_armed(armed);
                let (na, nb) = (a.update(s), b.update(s));
                assert_eq!(g1_nav_bits(&na), g1_nav_bits(&nb), "{aiding:?} step {k}: NavState");
                assert_eq!(g1_bits(&a), g1_bits(&b), "{aiding:?} step {k}: state");
                assert_eq!(a.stats(), b.stats(), "{aiding:?} step {k}: stats");
                assert!(a.events().eq(b.events()), "{aiding:?} step {k}: events");
                assert_eq!((a.ring.len, b.ring.len), (0, 0), "{aiding:?} step {k}: the ring is empty between steps");
            }
            let st = a.stats();
            println!("g1-a: {aiding:?}: predicts {} baro fused {} mag fused {} gps resets {} pos fused {}", st.predicts, st.baro_fused, st.mag_fused, st.gps_resets, st.gps_pos_fused);
            assert!(st.predicts > 1000);
            if aiding != Aiding::Imu {
                assert!(st.baro_fused > 100);
            }
            if aiding == Aiding::Full {
                assert!(st.mag_fused > 10 && st.gps_resets >= 1 && st.gps_pos_fused > 10, "{st:?}");
            }
        }
    }

    #[test]
    fn g1_b_the_constructor_refuses_a_short_ring_only_with_a_horizon_delay() {
        assert_eq!(EskfN::<1>::with_aiding(&Tuning::DEFAULT, Aiding::Full).err(), Some(EskfError::RingTooSmall));
        assert_eq!(EskfN::<1>::new(&Tuning::DEFAULT).err(), Some(EskfError::RingTooSmall));
        assert_eq!(EskfN::<{ RING - 1 }>::new(&Tuning::DEFAULT).err(), Some(EskfError::RingTooSmall));
        assert!(EskfN::<RING>::new(&Tuning::DEFAULT).is_ok());
        let zero = Tuning { nav_gps_delay_us: 0, ..Tuning::DEFAULT };
        assert!(EskfN::<1>::with_aiding(&zero, Aiding::Full).is_ok());
        assert!(EskfN::<7>::with_aiding(&zero, Aiding::Full).is_ok());
        // No delay without GNSS: a ring of one for the zero-delay variants at the default tuning.
        assert!(EskfN::<1>::with_aiding(&Tuning::DEFAULT, Aiding::ImuBaro).is_ok());
        assert!(EskfN::<1>::with_aiding(&Tuning::DEFAULT, Aiding::Imu).is_ok());
        // A delay beyond the buffer is still refused first.
        let far = Tuning { nav_gps_delay_us: Tuning::DEFAULT.nav_buffer_us + 1, ..Tuning::DEFAULT };
        assert_eq!(EskfN::<1>::new(&far).err(), Some(EskfError::DelayBeyondBuffer));
    }

    #[test]
    fn g1_f_a_ring_of_zero_is_refused_for_every_variant_and_delay() {
        let zero = Tuning { nav_gps_delay_us: 0, ..Tuning::DEFAULT };
        for aiding in [Aiding::Full, Aiding::ImuBaro, Aiding::Imu] {
            for t in [Tuning::DEFAULT, zero] {
                assert_eq!(EskfN::<0>::with_aiding(&t, aiding).err(), Some(EskfError::RingTooSmall), "{aiding:?}");
            }
        }
        assert_eq!(EskfN::<0>::new(&Tuning::DEFAULT).err(), Some(EskfError::RingTooSmall));
        // Beyond-buffer delay with R = 0: either refusal is a refusal.
        let far = Tuning { nav_gps_delay_us: Tuning::DEFAULT.nav_buffer_us + 1, ..Tuning::DEFAULT };
        assert!(EskfN::<0>::new(&far).is_err());
    }

    #[test]
    fn g1_g_a_reset_and_a_reinitialisation_in_one_update_report_no_reset() {
        // The step of the first GNSS reset (at arming) is found on a dry run; in the real run the gyro-bias x of the
        // state is made NaN just before that step. The step's job is still the reset (it does not read the bias), and
        // the state is non-finite afterwards, so the filter re-initialises in the same `update`.
        let t = Tuning::DEFAULT;
        let mut w = World::new(&t);
        w.v = [2.0, 0.5, 0.0];
        w.gps_vel = true;
        let stream: Vec<Sensors> = (0..8_000_000 / PERIOD_US).map(|_| w.step()).collect();
        let armed = |s: &Sensors| s.imu1.t_us >= 5_000_000;
        let mut dry = Eskf::new(&t).unwrap();
        let mut at = None;
        for (k, s) in stream.iter().enumerate() {
            dry.set_armed(armed(s));
            dry.update(s);
            if at.is_none() && dry.last_reset().is_some() && dry.stats().gps_resets == 1 {
                at = Some(k);
            }
        }
        let k0 = at.expect("a GNSS reset");
        let mut f = Eskf::new(&t).unwrap();
        for (k, s) in stream.iter().enumerate().take(k0 + 3) {
            f.set_armed(armed(s));
            if k == k0 {
                assert_eq!(f.stats().reinits, 0);
                f.nom.bg[0] = f32::NAN;
            }
            let (resets, reinits) = (f.stats().gps_resets, f.stats().reinits);
            f.update(s);
            if k == k0 {
                assert_eq!(f.stats().gps_resets, resets + 1, "the step's job was the GNSS reset");
                assert_eq!(f.stats().reinits, reinits + 1, "and the state went non-finite");
                assert!(!f.tilt_aligned());
                assert_eq!(f.last_reset(), None, "a re-initialisation leaves no reset to apply");
            }
        }
    }

    #[test]
    fn g1_c_the_ring_is_the_whole_size_difference() {
        let (big, small, entry) = (core::mem::size_of::<Eskf>(), core::mem::size_of::<EskfZ>(), core::mem::size_of::<ImuEntry>());
        println!("g1-c: size_of Eskf {big} B, EskfZ {small} B, ImuEntry {entry} B, difference {} B, (RING - 1) * ImuEntry {} B", big - small, (RING - 1) * entry);
        assert_eq!(entry, 32);
        assert_eq!(big - small, (RING - 1) * entry);
    }

    /// What `update` does to the nominal state before its job, rebuilt from the filter as it was before the step with
    /// N0's nominal step only: the state just before the job, the state right after the integration of the sample at `req`
    /// (the last one in the step with that time), and the number of samples integrated.
    #[allow(clippy::type_complexity)]
    fn g1_after_integration<const R: usize>(pre: &EskfN<R>, s: &Sensors, req: Option<u64>) -> (([f32; 4], [f32; 3], [f32; 3]), Option<(u64, [f32; 4], [f32; 3], [f32; 3])>, usize) {
        let now = s.imu1.t_us;
        let r = &s.imu1.reading;
        let mut ring: VecDeque<ImuEntry> = (0..pre.ring.len).map(|i| pre.ring.get(i)).collect();
        let mut order = Vec::new();
        if g1_finite(s) {
            if ring.len() == R {
                order.push(ring.pop_front().unwrap());
            }
            ring.push_back(ImuEntry { t_us: now, gyro: r.gyro_rads, accel: r.accel_ms2 });
        }
        while let Some(e) = ring.front().copied() {
            if e.t_us.saturating_add(pre.delay_us()) > now {
                break;
            }
            ring.pop_front();
            order.push(e);
        }
        let (mut nom, mut h, mut snap) = (pre.nom, pre.horizon_us, None);
        for e in &order {
            let dt_us = match h {
                Some(hh) => e.t_us.saturating_sub(hh).min(DT_MAX_US),
                None => 0,
            };
            h = Some(e.t_us);
            if pre.tilt_aligned {
                n0::nominal_step(&mut nom, &e.gyro, &e.accel, dt_us as f32 * 1e-6);
                if Some(e.t_us) == req {
                    snap = Some((e.t_us, nom.q, nom.v, nom.p));
                }
            }
        }
        ((nom.q, nom.v, nom.p), snap, order.len())
    }

    fn g1_snap_bits(s: Option<(u64, [f32; 4], [f32; 3], [f32; 3])>) -> Option<(u64, Vec<u32>)> {
        s.map(|(t, q, v, p)| (t, q.iter().chain(&v).chain(&p).map(|x| x.to_bits()).collect()))
    }

    /// Checks `f`'s snapshot after the step against the rebuilt one; true when there was one.
    fn g1_check_snap(f: &Eskf, pre: &Eskf, s: &Sensors, req: Option<u64>, k: usize) -> bool {
        let (_, expected, integrated) = g1_after_integration(pre, s, req);
        let snap = f.snapshot();
        assert_eq!(g1_snap_bits(snap), g1_snap_bits(expected), "step {k}: request {req:?}, {integrated} integrated");
        if let Some((_, q, v, p)) = snap {
            // One sample integrated and no fusion or reset after it: nominal() is the snapshot.
            if integrated == 1 && f.events().count() == 0 && f.last_reset().is_none() {
                assert_eq!(g1_snap_bits(Some((0, q, v, p))), g1_snap_bits(Some((0, f.nominal().0, f.nominal().1, f.nominal().2))), "step {k}");
            }
        }
        snap.is_some()
    }

    #[test]
    fn g1_d_the_snapshot_is_the_nominal_state_right_after_the_requested_sample_and_changes_no_output() {
        let t = Tuning::DEFAULT;
        let stream = g1_stream(&t, 60);
        let regular = |i: usize| i >= 1 && i + 1 < stream.len() && g1_finite(&stream[i]) && stream[i].imu1.t_us != stream[i - 1].imu1.t_us && stream[i].imu1.t_us != stream[i + 1].imu1.t_us;
        let (mut plain, mut sticky, mut front, mut never) = (Eskf::new(&t).unwrap(), Eskf::new(&t).unwrap(), Eskf::new(&t).unwrap(), Eskf::new(&t).unwrap());
        let mut req: Option<u64> = None;
        let (mut seen_sticky, mut seen_front, mut same_as_nominal) = (0, 0, 0);
        let nan_time = stream[20_000].imu1.t_us;
        assert!(!g1_finite(&stream[20_000]));
        for (k, s) in stream.iter().enumerate() {
            let armed = s.imu1.t_us >= 5_000_000;
            for f in [&mut plain, &mut sticky, &mut front, &mut never] {
                f.set_armed(armed);
            }
            let np = plain.update(s);
            // Held until the sample is integrated: a sample 10 steps back, still in the ring (the delay is 344 steps).
            if req.is_none() && k >= 10_000 && k % 503 == 0 && regular(k - 10) {
                req = Some(stream[k - 10].imu1.t_us);
                assert!(sticky.ring.len > 10);
            }
            let pre = sticky.clone();
            sticky.set_snap_request(req);
            let ns = sticky.update(s);
            if g1_check_snap(&sticky, &pre, s, req, k) {
                seen_sticky += 1;
                same_as_nominal += usize::from(sticky.events().count() == 0 && sticky.nominal().0.map(f32::to_bits) == sticky.snapshot().unwrap().1.map(f32::to_bits));
                req = None;
            }
            // The wrapper's own policy: the oldest sample of the ring, every step.
            let req_f = front.ring.front().map(|e| e.t_us);
            let pre = front.clone();
            front.set_snap_request(req_f);
            let nf = front.update(s);
            seen_front += usize::from(g1_check_snap(&front, &pre, s, req_f, k));
            // Times that are never integrated: the non-finite sample, a time before every sample, one after.
            never.set_snap_request([Some(nan_time), Some(1), Some(u64::MAX)][k % 3]);
            let nn = never.update(s);
            assert!(never.snapshot().is_none(), "step {k}: a snapshot for a time never integrated");
            for (name, n, f) in [("sticky", ns, &sticky), ("front", nf, &front), ("never", nn, &never)] {
                assert_eq!(g1_nav_bits(&np), g1_nav_bits(&n), "{name} step {k}: NavState");
                assert_eq!(g1_bits(&plain), g1_bits(f), "{name} step {k}: state");
                assert_eq!(plain.stats(), f.stats(), "{name} step {k}: stats");
            }
            assert!(plain.snapshot().is_none());
        }
        println!("g1-d: sticky requests answered {seen_sticky} times ({same_as_nominal} equal to nominal() in the step), front-of-ring requests answered {seen_front} of {} steps", stream.len());
        assert!(seen_sticky >= 50 && same_as_nominal >= 1, "{seen_sticky} {same_as_nominal}");
        assert!(seen_front > 40_000);
    }

    /// Rebuilds every step of `f` over `stream` with `g1_after_integration` and checks `last_reset` against it. `each` runs
    /// before every step (poking or re-initialising the filter). Returns the steps of each reset kind and the number of
    /// non-reset steps with a fusion. Tolerances: the quaternion product to 2e-6 per component, v and p to 8 f32 epsilon of
    /// the larger of the two values and 1.
    fn g1_run_resets<const R: usize>(f: &mut EskfN<R>, stream: &[Sensors], armed: impl Fn(&Sensors) -> bool, mut each: impl FnMut(usize, &mut EskfN<R>)) -> ([Vec<usize>; 4], usize) {
        let (mut kinds, mut fusion_steps) = ([Vec::new(), Vec::new(), Vec::new(), Vec::new()], 0);
        for (k, s) in stream.iter().enumerate() {
            each(k, f);
            f.set_armed(armed(s));
            let pre = f.clone();
            let st0 = *f.stats();
            f.update(s);
            let st1 = *f.stats();
            let (before, _, _) = g1_after_integration(&pre, s, None);
            let after = f.nominal();
            let d = |a: u32, b: u32| a.wrapping_sub(b);
            let yaw = d(st1.yaw_aligns + st1.yaw_resets, st0.yaw_aligns + st0.yaw_resets);
            let (gps, height, reinit) = (d(st1.gps_resets, st0.gps_resets), d(st1.height_resets, st0.height_resets), d(st1.reinits, st0.reinits));
            let realign = pre.arm_edge && pre.arm_realign && pre.tilt_aligned && pre.aiding_set != Aiding::Full && pre.arm_f.is_some();
            assert!(yaw + gps + height <= 1, "step {k}: two resets in one step");
            let expect = (yaw + gps + height > 0 || realign) && reinit == 0;
            assert_eq!(f.last_reset().is_some(), expect, "step {k}: yaw {yaw} gps {gps} height {height} realign {realign} reinit {reinit}");
            let Some(r) = f.last_reset() else {
                let fused = d(st1.baro_fused + st1.gps_pos_fused + st1.gps_vel_fused + st1.mag_fused + st1.noaid, st0.baro_fused + st0.gps_pos_fused + st0.gps_vel_fused + st0.mag_fused + st0.noaid);
                fusion_steps += usize::from(fused > 0);
                continue;
            };
            let pred = normalized(qmul(&r.dq, &before.0));
            for i in 0..4 {
                assert!(fabsf(pred[i] - after.0[i]) <= 2e-6, "step {k}: q {pred:?} against {:?}", after.0);
            }
            assert!(fabsf(r.dq.iter().map(|x| x * x).sum::<f32>() - 1.0) <= 2e-6);
            for i in 0..3 {
                for (b, a, dx) in [(before.1[i], after.1[i], r.dv[i]), (before.2[i], after.2[i], r.dp[i])] {
                    assert!(fabsf(a - (b + dx)) <= 8.0 * f32::EPSILON * fabsf(a).max(fabsf(b)).max(1.0), "step {k}: axis {i}: {b} + {dx} against {a}");
                }
            }
            let kind = if yaw > 0 {
                // A rotation about NED down, nothing else moved.
                assert!(fabsf(r.dq[1]) <= 1e-6 && fabsf(r.dq[2]) <= 1e-6, "step {k}: yaw dq {:?}", r.dq);
                assert!(r.dq[3] != 0.0 && r.dv == [0.0; 3] && r.dp == [0.0; 3], "step {k}: {r:?}");
                0
            } else if gps > 0 {
                assert!(fabsf(r.dq[1]) + fabsf(r.dq[2]) + fabsf(r.dq[3]) <= 3e-6, "step {k}: GPS reset moved the attitude {:?}", r.dq);
                assert!(r.dp[2] == 0.0 && r.dv[2] == 0.0, "step {k}: {r:?}");
                1
            } else if height > 0 {
                assert!(fabsf(r.dq[1]) + fabsf(r.dq[2]) + fabsf(r.dq[3]) <= 3e-6 && r.dp[0] == 0.0 && r.dp[1] == 0.0 && r.dv[0] == 0.0 && r.dv[1] == 0.0, "step {k}: {r:?}");
                assert!(r.dp[2] != 0.0, "step {k}: a height reset that moved nothing");
                // Disarmed, the timeout branch also sets v_D to 0; armed, v_D is kept.
                if !pre.armed {
                    assert!(after.1[2] == 0.0, "step {k}: v_D {} after a disarmed height reset", after.1[2]);
                } else if pre.armed {
                    assert!(r.dv[2] == 0.0, "step {k}: {r:?}");
                }
                2
            } else {
                // The arming-edge re-alignment: the tilt moves, the horizontal velocity goes to 0, nothing else.
                assert!(r.dq[0] < 0.9999 || r.dq[0] > 1.0001 || fabsf(r.dq[1]) + fabsf(r.dq[2]) > 1e-4, "step {k}: the realignment moved no tilt {:?}", r.dq);
                assert!(after.1[0] == 0.0 && after.1[1] == 0.0 && r.dv[2] == 0.0 && r.dp == [0.0; 3], "step {k}: {r:?}");
                3
            };
            kinds[kind].push(k);
        }
        (kinds, fusion_steps)
    }

    /// Level at rest with a gyro pulse the vehicle did not make (as `handled`) from 1 to 1.5 s, the barometer `baro_off_m`
    /// too high from `baro_from_s`; `secs` long.
    fn g1_zero_delay_stream(t: &Tuning, secs: u64, baro_from_s: u64, baro_off_m: f64) -> Vec<Sensors> {
        let mut w = World::new(t);
        let mut out = Vec::new();
        while w.t_us() < secs * 1_000_000 {
            let mut s = w.step();
            if w.t_us() >= 1_000_000 && w.t_us() < 1_500_000 {
                s.imu1.reading.gyro_rads = [0.2, -0.1, 0.3];
            }
            if let Some(b) = s.baro.as_mut().filter(|_| w.t_us() >= baro_from_s * 1_000_000) {
                b.reading.pressure_pa = pressure_at(baro_off_m - w.p[2]);
            }
            out.push(s);
        }
        out
    }

    #[test]
    fn g1_e_resets_are_reported_exactly_and_ordinary_fusions_are_not() {
        let t = Tuning::DEFAULT;
        // Full: yaw alignment (first compass), GPS reset at arming (5 s), C3 reset (a 10-sigma north fault from 20 s), a
        // yaw reset (compass turned 90 deg from 60 s), the height reset after the timeout (barometer 40 m too high from
        // 85 s, armed) and the height initialisation after a re-initialisation (at 100 s).
        let mut w = World::new(&t);
        w.yaw = 0.4;
        w.v = [2.0, 0.5, 0.0];
        w.gps_vel = true;
        let sigma = (f64::from(w.std_mm) * 1e-3).max(f64::from(t.nav_gps_pos_sigma_min_m));
        let mut stream = Vec::new();
        while w.t_us() < 108_000_000 {
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
            stream.push(s);
        }
        let mut f = Eskf::new(&t).unwrap();
        let reinit_at = stream.iter().position(|s| s.imu1.t_us >= 100_000_000).unwrap();
        let (kinds, fused) = g1_run_resets(&mut f, &stream, |s| s.imu1.t_us >= 5_000_000, |k, f| {
            if k == reinit_at {
                f.reinitialise();
            }
        });
        let st = f.stats();
        println!("g1-e Full: yaw resets at {:?}, GPS resets at {:?}, height resets at {:?}, realign at {:?}; {fused} non-reset steps with a fusion; {st:?}", kinds[0], kinds[1], kinds[2], kinds[3]);
        assert_eq!(st.yaw_aligns, 2, "the first alignment and the one after the re-initialisation");
        assert_eq!((kinds[0].len() as u32, kinds[1].len() as u32, kinds[2].len() as u32), (st.yaw_aligns + st.yaw_resets, st.gps_resets, st.height_resets));
        assert!(st.yaw_resets >= 1 && st.gps_resets >= 2 && st.height_resets >= 2 && kinds[3].is_empty());
        assert!(kinds[2].iter().any(|&k| k > reinit_at) && kinds[2].iter().any(|&k| k < reinit_at), "both height-reset branches");
        assert!(fused > 5_000, "{fused}");

        // The zero-delay variants: the arming-edge re-alignment at 4 s and the height reset 5 s after the barometer's
        // jump at 12 s (armed), for `ImuBaro` and `Imu` (no barometer, so no height reset).
        for aiding in [Aiding::ImuBaro, Aiding::Imu] {
            let stream = g1_zero_delay_stream(&t, 25, 12, 40.0);
            let mut f = EskfZ::with_aiding(&t, aiding).unwrap();
            let (kinds, fused) = g1_run_resets(&mut f, &stream, |s| s.imu1.t_us >= 4_000_000, |_, _| {});
            println!("g1-e {aiding:?}: realign at {:?}, height resets at {:?}; {fused} non-reset steps with a fusion", kinds[3], kinds[2]);
            assert!(kinds[0].is_empty() && kinds[1].is_empty());
            assert_eq!(kinds[3].len(), 1, "the arming-edge re-alignment, once");
            assert_eq!(stream[kinds[3][0]].imu1.t_us, 4_000_000, "at the first armed sample");
            assert_eq!(kinds[2].len() as u32, f.stats().height_resets);
            assert_eq!(kinds[2].is_empty(), aiding == Aiding::Imu);
            assert!(fused > 1_000);
        }
        // The disarmed branch of the height reset zeroes v_D: a v_D of 1.5 m/s put in at 14.9 s (the barometer jumps at 15 s).
        let stream = g1_zero_delay_stream(&t, 25, 15, 40.0);
        let mut f = EskfZ::with_aiding(&t, Aiding::ImuBaro).unwrap();
        let poke = stream.iter().position(|s| s.imu1.t_us >= 14_900_000).unwrap();
        let (kinds, _) = g1_run_resets(&mut f, &stream, |_| false, |k, f| {
            if k == poke {
                f.nom.v[2] = 1.5;
            }
        });
        println!("g1-e disarmed ImuBaro: height resets at {:?}", kinds[2]);
        assert!(kinds[2].len() == 1 && kinds[2][0] > poke);
        assert!(kinds[3].is_empty() && !kinds[2].is_empty());
    }
}
