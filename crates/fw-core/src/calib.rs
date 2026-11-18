//! Gyroscope, one-position accelerometer and compass calibration: the accumulation and the
//! decision, pure.
//!
//! Samples go in (per IMU, with the sample's `seq`: a sample is counted once), a per-IMU offset or
//! the reason for the failure comes out. No clock, no hardware, `f32` only (the compass takes the
//! sample's own time stamp).
//!
//! - Gyroscope: Betaflight `performGyroCalibration` (`src/main/sensors/gyro.c`): the samples of a
//!   fixed duration, `gyroCalibrationDuration` = 125 (1.25 s, `pgResetFn_gyroConfig`), their mean is
//!   the offset; moving when the standard deviation (`devStandardDeviation`, `common/maths.c`, n - 1)
//!   of any axis exceeds `gyroMovementCalibrationThreshold` = 48 raw counts, at the 2000 dps scale
//!   Betaflight runs every gyro at (`GYRO_SCALE_2000DPS` = 2000 / 2^15 dps per count,
//!   `drivers/accgyro/accgyro.h`): 2.93 dps. Betaflight starts over when moving; here it fails.
//! - Accelerometer, one position: ArduPilot `AP_InertialSensor::simple_accel_cal`: averages of 50
//!   samples; two successive averages closer than `accel_convergence_limit` = 0.05 m/s^2 (length of
//!   the difference) converge, the offset is their mean minus gravity (the specific force a level
//!   vehicle reads, (0, 0, -g) in the body frame, in the board frame); not converged within "up to
//!   10 seconds": failed. The largest offset accepted is ArduPilot's `AccelCalibrator::accept_result`
//!   (`GRAVITY_MSS` per axis): `simple_accel_cal` itself has no limit.
//! - Compass, minimum and maximum: Betaflight `compassUpdate` (`src/main/sensors/compass.c`, tags up
//!   to 4.4.3; master and 4.5 fit a bias estimator instead): for 30 s (`(currentTimeUs - tCal) <
//!   30000000`) the minimum and maximum of every axis are tracked, and the offset is
//!   `(min + max) / 2`. Betaflight accepts whatever comes out; here the radius seen on an axis,
//!   `(max - min) / 2`, must lie strictly between ArduPilot's `FIELD_RADIUS_MIN` = 150 mG and
//!   `FIELD_RADIUS_MAX` = 950 mG (`CompassCalibrator.cpp`, `fit_acceptable_status`:
//!   `radius <= FIELD_RADIUS_MIN || radius >= FIELD_RADIUS_MAX` is `FAILED_RADIUS`) on every axis.

use core::f32::consts::PI;

use crate::rotation::Rotation;

/// Standard gravity, m/s^2 (ArduPilot `GRAVITY_MSS`).
pub const GRAVITY_MSS: f32 = 9.806_65;

/// Gyro samples averaged: Betaflight's 1.25 s at the 100 samples per second core 0 sees per IMU.
pub const GYRO_SAMPLES: u32 = 125;

/// Betaflight's moving limit, 48 counts at 2000 / 2^15 dps per count, in rad/s (standard deviation).
pub const GYRO_MOVING_SD_RADS: f32 = 48.0 * (2000.0 / 32768.0) * PI / 180.0;

/// The gyros' full scale, rad/s: 2000 dps, the scale Betaflight runs every gyro at
/// (`GYRO_SCALE_2000DPS`). The range of the gyro offset parameters: the store holds any rate a
/// working gyro can report. Neither Betaflight nor ArduPilot limits the offset otherwise.
pub const GYRO_FULL_SCALE_RADS: f32 = 2000.0 * PI / 180.0;

/// Samples per accelerometer average (ArduPilot `simple_accel_cal`: 50 points).
pub const ACCEL_WINDOW: u32 = 50;

/// Successive averages closer than this converge, m/s^2 (`accel_convergence_limit`).
pub const ACCEL_CONVERGENCE_MSS: f32 = 0.05;

/// Averages tried before the accelerometer calibration fails: 10 s at 100 samples per second.
pub const ACCEL_WINDOWS: u32 = 20;

/// The largest accelerometer offset accepted per axis, m/s^2 (`AccelCalibrator::accept_result`).
pub const ACCEL_OFFSET_MAX_MSS: f32 = GRAVITY_MSS;

/// The number of IMUs a run covers.
pub const IMUS: usize = 2;

/// The compass calibration's duration, microseconds (Betaflight: 30 s).
pub const MAG_DURATION_US: u64 = 30_000_000;

/// The smallest field radius accepted per axis, microtesla: ArduPilot `FIELD_RADIUS_MIN` = 150 mG.
pub const MAG_RADIUS_MIN_UT: f32 = 15.0;

/// The largest field radius accepted per axis, microtesla: ArduPilot `FIELD_RADIUS_MAX` = 950 mG.
pub const MAG_RADIUS_MAX_UT: f32 = 95.0;

/// The largest compass offset the parameters hold per axis, microtesla (`MAG_OFFX/Y/Z`).
pub const MAG_OFFSET_MAX_UT: f32 = 200.0;

/// Mean and variance per axis, Welford's update in `f32` (Betaflight `devPush`, `devVariance`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Stats3 {
    n: u32,
    mean: [f32; 3],
    m2: [f32; 3],
}

impl Stats3 {
    /// No sample yet.
    pub const fn new() -> Self {
        Self {
            n: 0,
            mean: [0.0; 3],
            m2: [0.0; 3],
        }
    }

    /// Adds one sample.
    pub fn push(&mut self, x: [f32; 3]) {
        self.n = self.n.saturating_add(1);
        let n = self.n as f32;
        for ((mean, m2), x) in self.mean.iter_mut().zip(self.m2.iter_mut()).zip(x) {
            let old = *mean;
            *mean = old + (x - old) / n;
            *m2 += (x - old) * (x - *mean);
        }
    }

    /// The number of samples.
    pub const fn count(&self) -> u32 {
        self.n
    }

    /// The mean per axis (0 with no sample).
    pub const fn mean(&self) -> [f32; 3] {
        self.mean
    }

    /// The sample variance per axis (n - 1; 0 below two samples).
    pub fn variance(&self) -> [f32; 3] {
        if self.n < 2 {
            return [0.0; 3];
        }
        let d = (self.n - 1) as f32;
        self.m2.map(|m2| m2 / d)
    }
}

/// The square root by Newton's method (`core` has no `f32::sqrt`); 0 for `s <= 0`.
pub fn sqrt(s: f32) -> f32 {
    if s.is_nan() || s <= 0.0 {
        return 0.0;
    }
    let mut x = s.max(1.0);
    for _ in 0..40 {
        x = 0.5 * (x + s / x);
    }
    x
}

/// The length of a vector.
pub fn norm3(v: [f32; 3]) -> f32 {
    sqrt(v.iter().map(|x| x * x).sum::<f32>())
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    let [ax, ay, az] = a;
    let [bx, by, bz] = b;
    [ax - bx, ay - by, az - bz]
}

fn add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    let [ax, ay, az] = a;
    let [bx, by, bz] = b;
    [ax + bx, ay + by, az + bz]
}

/// One IMU's offsets, what the adapter subtracts from every sample.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuOffsets {
    /// Gyro offsets, rad/s, in the board frame (after the chip's fixed rotation, before `AHRS_ORIENTATION`).
    pub gyro_rads: [f32; 3],
    /// Accelerometer offsets, m/s^2, in the board frame.
    pub accel_ms2: [f32; 3],
}

impl ImuOffsets {
    /// No offset.
    pub const ZERO: Self = Self {
        gyro_rads: [0.0; 3],
        accel_ms2: [0.0; 3],
    };

    /// The calibrated sample: `raw - offsets` (gyro, accelerometer).
    pub fn remove(&self, gyro_rads: [f32; 3], accel_ms2: [f32; 3]) -> ([f32; 3], [f32; 3]) {
        (
            sub(gyro_rads, self.gyro_rads),
            sub(accel_ms2, self.accel_ms2),
        )
    }

    /// The published sample from a chip sample, in the order the adapters apply: `mount` (the
    /// chip's fixed rotation), then the offsets removed (they are in the board frame), then `board`
    /// (`AHRS_ORIENTATION`). The result is in the body frame.
    #[inline(always)]
    pub fn publish(
        &self,
        mount: Rotation,
        board: Rotation,
        gyro_rads: [f32; 3],
        accel_ms2: [f32; 3],
    ) -> ([f32; 3], [f32; 3]) {
        let (gyro, accel) = self.remove(mount.rotate(gyro_rads), mount.rotate(accel_ms2));
        (board.rotate(gyro), board.rotate(accel))
    }

    /// The board-frame sample (`mount * chip`, offsets not removed) from a published one:
    /// `board^T * published + offsets`. Core 0 un-rotates with the `board` core 1 applied (the boot
    /// value), so the calibrations measure in the frame the offsets are stored in.
    pub fn unpublish(
        &self,
        board: Rotation,
        gyro_rads: [f32; 3],
        accel_ms2: [f32; 3],
    ) -> ([f32; 3], [f32; 3]) {
        self.restore(
            board.rotate_inverse(gyro_rads),
            board.rotate_inverse(accel_ms2),
        )
    }

    /// The raw sample again from a calibrated one: `calibrated + offsets`.
    pub fn restore(&self, gyro_rads: [f32; 3], accel_ms2: [f32; 3]) -> ([f32; 3], [f32; 3]) {
        (
            add(gyro_rads, self.gyro_rads),
            add(accel_ms2, self.accel_ms2),
        )
    }
}

/// Why a calibration produced no offsets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Failure {
    /// No IMU was Ok at the start.
    NoImu,
    /// The vehicle moved (by the source's criterion).
    Moving,
    /// An offset is beyond the accepted limit.
    OffsetTooLarge,
    /// Compass: a radius is not between the two bounds (the vehicle was not rotated through all
    /// orientations).
    NotRotated,
    /// Compass: the offsets would fall outside the parameters' range.
    OffsetOutOfRange,
    /// Cancelled: nothing is written.
    Cancelled,
}

impl Failure {
    /// The text after "<calibration>: ".
    pub const fn text(self) -> &'static str {
        match self {
            Self::NoImu => "no IMU Ok",
            Self::Moving => "moving",
            Self::OffsetTooLarge => "offset too large",
            Self::NotRotated => "not rotated",
            Self::OffsetOutOfRange => "offset out of range",
            Self::Cancelled => "cancelled",
        }
    }
}

/// Counts a sample once: `true` for a `seq` not seen just before.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Fresh(Option<u32>);

impl Fresh {
    fn take(&mut self, seq: u32) -> bool {
        let fresh = self.0 != Some(seq);
        self.0 = Some(seq);
        fresh
    }
}

/// One IMU's gyro calibration.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GyroCal {
    fresh: Fresh,
    stats: Stats3,
}

impl GyroCal {
    /// No sample yet.
    pub const fn new() -> Self {
        Self {
            fresh: Fresh(None),
            stats: Stats3::new(),
        }
    }

    /// Adds a raw sample (rad/s) unless its `seq` was the previous one or enough are in.
    pub fn push(&mut self, seq: u32, gyro_rads: [f32; 3]) {
        if self.fresh.take(seq) && self.stats.count() < GYRO_SAMPLES {
            self.stats.push(gyro_rads);
        }
    }

    /// The samples counted.
    pub const fn count(&self) -> u32 {
        self.stats.count()
    }

    /// 0 to 100.
    pub fn percent(&self) -> u8 {
        (self.stats.count().min(GYRO_SAMPLES) * 100 / GYRO_SAMPLES) as u8
    }

    /// `None` until [`GYRO_SAMPLES`] are in; then the mean (the offset) or the failure.
    pub fn result(&self) -> Option<Result<[f32; 3], Failure>> {
        if self.stats.count() < GYRO_SAMPLES {
            return None;
        }
        let limit = GYRO_MOVING_SD_RADS * GYRO_MOVING_SD_RADS;
        Some(if self.stats.variance().iter().any(|v| *v > limit) {
            Err(Failure::Moving)
        } else {
            Ok(self.stats.mean())
        })
    }
}

/// The gravity a level, still accelerometer reads in the body frame (FRD), m/s^2: specific force,
/// up (ArduPilot `simple_accel_cal`: `rotated_gravity(0, 0, -GRAVITY_MSS)`).
pub const BODY_GRAVITY_MSS: [f32; 3] = [0.0, 0.0, -GRAVITY_MSS];

/// The gravity a level, still accelerometer reads in the board frame, m/s^2: `board^T * (0, 0, -g)`
/// (the samples the calibration sees are un-rotated with the `board` core 1 applied).
pub fn board_gravity(board: Rotation) -> [f32; 3] {
    board.rotate_inverse(BODY_GRAVITY_MSS)
}

/// One IMU's one-position accelerometer calibration (the vehicle level, +Z of the body down).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AccelCal {
    expected: [f32; 3],
    fresh: Fresh,
    sum: [f32; 3],
    in_window: u32,
    windows: u32,
    last: Option<[f32; 3]>,
    done: Option<Result<[f32; 3], Failure>>,
}

impl AccelCal {
    /// No sample yet. `expected_ms2` is what a level, still accelerometer reads in the frame of the
    /// samples pushed: the board frame, where it is [`board_gravity`] (`board^T * (0, 0, -g)`).
    pub const fn new(expected_ms2: [f32; 3]) -> Self {
        Self {
            expected: expected_ms2,
            fresh: Fresh(None),
            sum: [0.0; 3],
            in_window: 0,
            windows: 0,
            last: None,
            done: None,
        }
    }

    /// Adds a raw sample (m/s^2, board frame) unless its `seq` was the previous one or the result
    /// is in.
    pub fn push(&mut self, seq: u32, accel_ms2: [f32; 3]) {
        if !self.fresh.take(seq) || self.done.is_some() {
            return;
        }
        self.sum = add(self.sum, accel_ms2);
        self.in_window += 1;
        if self.in_window < ACCEL_WINDOW {
            return;
        }
        let average = self.sum.map(|s| s / ACCEL_WINDOW as f32);
        self.sum = [0.0; 3];
        self.in_window = 0;
        self.windows += 1;
        let diff = self.last.map(|last| sub(last, average));
        if let (Some(last), Some(d)) = (self.last, diff)
            && d.iter().map(|x| x * x).sum::<f32>() < ACCEL_CONVERGENCE_MSS * ACCEL_CONVERGENCE_MSS
        {
            let mean = add(last, average).map(|x| 0.5 * x);
            let offset = sub(mean, self.expected);
            self.done = Some(if offset.iter().all(|o| o.abs() <= ACCEL_OFFSET_MAX_MSS) {
                Ok(offset)
            } else {
                Err(Failure::OffsetTooLarge)
            });
        } else if self.windows >= ACCEL_WINDOWS {
            self.done = Some(Err(Failure::Moving));
        } else {
            self.last = Some(average);
        }
    }

    /// The averages completed.
    pub const fn windows(&self) -> u32 {
        self.windows
    }

    /// 0 to 100 (100 when the result is in).
    pub fn percent(&self) -> u8 {
        if self.done.is_some() {
            100
        } else {
            (self.windows.min(ACCEL_WINDOWS) * 100 / ACCEL_WINDOWS) as u8
        }
    }

    /// `None` until two averages converge or the averages run out; then the offset or the failure.
    pub const fn result(&self) -> Option<Result<[f32; 3], Failure>> {
        self.done
    }
}

/// A compass calibration's measurement: the offsets (midpoints) and the radii seen, microtesla.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagFit {
    /// `(max + min) / 2` per axis.
    pub offset_ut: [f32; 3],
    /// `(max - min) / 2` per axis.
    pub radius_ut: [f32; 3],
}

/// The raw field again from a published one: `published + offsets` (the adapter publishes
/// `raw - offsets`), microtesla.
pub fn mag_raw(published_ut: [f32; 3], offsets_ut: [f32; 3]) -> [f32; 3] {
    add(published_ut, offsets_ut)
}

/// Where a compass calibration stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MagOutcome {
    /// Still collecting.
    Running,
    /// Accepted: write these offsets.
    Done(MagFit),
    /// Nothing is written; the measurement the failure came from, when there is one.
    Failed(Failure, Option<MagFit>),
}

/// The compass calibration, minimum and maximum of the raw field per axis over
/// [`MAG_DURATION_US`] of the samples' own time (a sample is counted once by its `seq`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagCal {
    fresh: Fresh,
    start_us: Option<u64>,
    elapsed_us: u64,
    min: [f32; 3],
    max: [f32; 3],
    cancelled: bool,
}

impl Default for MagCal {
    fn default() -> Self {
        Self::new()
    }
}

impl MagCal {
    /// No sample yet.
    pub const fn new() -> Self {
        Self {
            fresh: Fresh(None),
            start_us: None,
            elapsed_us: 0,
            min: [f32::INFINITY; 3],
            max: [f32::NEG_INFINITY; 3],
            cancelled: false,
        }
    }

    /// Adds a raw sample (microtesla, the offsets in force added back) taken at `t_us`, unless its
    /// `seq` was the previous one, the time is up, or the run was cancelled. The first sample
    /// starts the clock; a sample at or after the duration ends the run and is not counted
    /// (Betaflight: `(currentTimeUs - tCal) < 30000000`).
    pub fn push(&mut self, seq: u32, t_us: u64, field_ut: [f32; 3]) {
        if self.cancelled || self.done() || !self.fresh.take(seq) {
            return;
        }
        let start = *self.start_us.get_or_insert(t_us);
        self.elapsed_us = t_us.saturating_sub(start);
        if self.done() {
            return;
        }
        for ((min, max), x) in self.min.iter_mut().zip(self.max.iter_mut()).zip(field_ut) {
            if x < *min {
                *min = x;
            }
            if x > *max {
                *max = x;
            }
        }
    }

    fn done(&self) -> bool {
        self.elapsed_us >= MAG_DURATION_US
    }

    /// 0 to 100: the share of the duration gone.
    pub fn percent(&self) -> u8 {
        (self.elapsed_us.min(MAG_DURATION_US) * 100 / MAG_DURATION_US) as u8
    }

    /// Stops the run: it ends [`Failure::Cancelled`] and gives no offset.
    pub fn cancel(&mut self) {
        self.cancelled = true;
    }

    /// Where the run stands. The radii are checked first (ArduPilot's bounds, strictly inside),
    /// then the offsets against the parameters' range.
    pub fn outcome(&self) -> MagOutcome {
        if self.cancelled {
            return MagOutcome::Failed(Failure::Cancelled, None);
        }
        if !self.done() {
            return MagOutcome::Running;
        }
        let mut fit = MagFit {
            offset_ut: [0.0; 3],
            radius_ut: [0.0; 3],
        };
        for (i, (min, max)) in self.min.iter().zip(self.max.iter()).enumerate() {
            if let (Some(o), Some(r)) = (fit.offset_ut.get_mut(i), fit.radius_ut.get_mut(i)) {
                *o = 0.5 * (max + min);
                *r = 0.5 * (max - min);
            }
        }
        if !fit
            .radius_ut
            .iter()
            .all(|r| *r > MAG_RADIUS_MIN_UT && *r < MAG_RADIUS_MAX_UT)
        {
            return MagOutcome::Failed(Failure::NotRotated, Some(fit));
        }
        if !fit.offset_ut.iter().all(|o| o.abs() <= MAG_OFFSET_MAX_UT) {
            return MagOutcome::Failed(Failure::OffsetOutOfRange, Some(fit));
        }
        MagOutcome::Done(fit)
    }
}

/// Which calibration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// Gyro, both IMUs.
    Gyro,
    /// One-position accelerometer, both IMUs.
    Accel,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Imu {
    Gyro(GyroCal),
    Accel(AccelCal),
}

impl Imu {
    fn result(&self) -> Option<Result<[f32; 3], Failure>> {
        match self {
            Self::Gyro(g) => g.result(),
            Self::Accel(a) => a.result(),
        }
    }

    fn percent(&self) -> u8 {
        match self {
            Self::Gyro(g) => g.percent(),
            Self::Accel(a) => a.percent(),
        }
    }
}

/// Where a run stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Outcome {
    /// Still collecting.
    Running,
    /// Every IMU in the run has its offset; `None` for an IMU skipped at the start.
    Done([Option<[f32; 3]>; IMUS]),
    /// No offset is written; the IMU the failure came from, if one.
    Failed(Failure, Option<usize>),
}

/// One calibration over both IMUs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Run {
    kind: Kind,
    imus: [Option<Imu>; IMUS],
    cancelled: bool,
}

impl Run {
    /// A run over the IMUs that are Ok; [`Failure::NoImu`] when none is. `board` is the
    /// `AHRS_ORIENTATION` core 1 applies (the boot value), which the accelerometer reference needs.
    pub fn start(kind: Kind, ok: [bool; IMUS], board: Rotation) -> Result<Self, Failure> {
        if !ok.iter().any(|o| *o) {
            return Err(Failure::NoImu);
        }
        let imus = ok.map(|o| {
            o.then_some(match kind {
                Kind::Gyro => Imu::Gyro(GyroCal::new()),
                Kind::Accel => Imu::Accel(AccelCal::new(board_gravity(board))),
            })
        });
        Ok(Self {
            kind,
            imus,
            cancelled: false,
        })
    }

    /// Which calibration.
    pub const fn kind(&self) -> Kind {
        self.kind
    }

    /// Whether IMU `imu` (0 or 1) is in the run.
    pub fn includes(&self, imu: usize) -> bool {
        matches!(self.imus.get(imu), Some(Some(_)))
    }

    /// A raw sample of IMU `imu` (ignored for an IMU not in the run, a repeated `seq`, or after
    /// a cancel).
    pub fn push(&mut self, imu: usize, seq: u32, gyro_rads: [f32; 3], accel_ms2: [f32; 3]) {
        if self.cancelled {
            return;
        }
        match self.imus.get_mut(imu) {
            Some(Some(Imu::Gyro(g))) => g.push(seq, gyro_rads),
            Some(Some(Imu::Accel(a))) => a.push(seq, accel_ms2),
            _ => {}
        }
    }

    /// The progress, 0 to 100: the slowest IMU in the run.
    pub fn percent(&self) -> u8 {
        self.imus
            .iter()
            .flatten()
            .map(Imu::percent)
            .min()
            .unwrap_or(100)
    }

    /// Stops the run: it ends [`Failure::Cancelled`] and gives no offset.
    pub fn cancel(&mut self) {
        self.cancelled = true;
    }

    /// Where the run stands; a failure of any IMU fails the run.
    pub fn outcome(&self) -> Outcome {
        if self.cancelled {
            return Outcome::Failed(Failure::Cancelled, None);
        }
        let mut offsets = [None; IMUS];
        let mut running = false;
        for (i, (slot, out)) in self.imus.iter().zip(offsets.iter_mut()).enumerate() {
            match slot.as_ref().map(Imu::result) {
                None => {}
                Some(None) => running = true,
                Some(Some(Err(why))) => return Outcome::Failed(why, Some(i)),
                Some(Some(Ok(offset))) => *out = Some(offset),
            }
        }
        if running {
            Outcome::Running
        } else {
            Outcome::Done(offsets)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BIAS: [f32; 3] = [0.012, -0.034, 0.005];

    fn close(a: [f32; 3], b: [f32; 3], tol: f32) -> bool {
        a.iter().zip(b).all(|(x, y)| (x - y).abs() <= tol)
    }

    /// Feeds both IMUs until the run is no longer running (bounded).
    fn feed(run: &mut Run, sample: impl Fn(u32) -> ([f32; 3], [f32; 3])) -> Outcome {
        for seq in 0..10_000u32 {
            let (g, a) = sample(seq);
            run.push(0, seq, g, a);
            run.push(1, seq, g, a);
            if run.outcome() != Outcome::Running {
                return run.outcome();
            }
        }
        run.outcome()
    }

    #[test]
    fn constants_are_the_sources_numbers() {
        // 48 counts * 2000 / 32768 dps = 2.9296875 dps.
        assert!((GYRO_MOVING_SD_RADS * 180.0 / PI - 2.929_687_5).abs() < 1e-5);
        assert!((GYRO_FULL_SCALE_RADS - 34.906_585).abs() < 1e-4);
        assert_eq!(MAG_DURATION_US, 30_000_000);
        // ArduPilot FIELD_RADIUS_MIN 150 and FIELD_RADIUS_MAX 950 milligauss.
        assert_eq!(MAG_RADIUS_MIN_UT, 15.0);
        assert_eq!(MAG_RADIUS_MAX_UT, 95.0);
        assert_eq!(GYRO_SAMPLES, 125);
        assert_eq!(ACCEL_WINDOW * ACCEL_WINDOWS, 1000);
        assert_eq!(ACCEL_OFFSET_MAX_MSS, 9.806_65);
    }

    #[test]
    fn a_still_gyro_record_gives_its_mean_as_the_offset() {
        let mut run = Run::start(Kind::Gyro, [true, true], Rotation::None).unwrap();
        // +-1 mrad/s noise around the bias, mean exactly the bias over whole pairs.
        let out = feed(&mut run, |seq| {
            let n = if seq % 2 == 0 { 0.001 } else { -0.001 };
            (BIAS.map(|b| b + n), BODY_GRAVITY_MSS)
        });
        let Outcome::Done([Some(a), Some(b)]) = out else {
            panic!("{out:?}");
        };
        // 125 samples: 63 high, 62 low, the mean 8 urad/s above the bias.
        assert!(close(a, BIAS, 2e-5), "{a:?}");
        assert_eq!(a, b);
        assert_eq!(run.percent(), 100);
    }

    #[test]
    fn the_gyro_moving_limit_is_betaflights_standard_deviation() {
        // Alternating +-d on X, n odd: the sample standard deviation is d * sqrt((n + 1) / n).
        let n = GYRO_SAMPLES as f32;
        let factor = ((n + 1.0) / n).sqrt();
        let at = GYRO_MOVING_SD_RADS / factor;
        for (scale, expect_moving) in [(0.99, false), (1.01, true)] {
            let d = at * scale;
            let mut run = Run::start(Kind::Gyro, [true, false], Rotation::None).unwrap();
            let out = feed(&mut run, |seq| {
                let x = if seq % 2 == 0 { d } else { -d };
                ([x, 0.0, 0.0], [0.0; 3])
            });
            if expect_moving {
                assert_eq!(out, Outcome::Failed(Failure::Moving, Some(0)), "{scale}");
            } else {
                assert!(
                    matches!(out, Outcome::Done([Some(_), None])),
                    "{scale} {out:?}"
                );
            }
        }
    }

    #[test]
    fn a_large_still_gyro_mean_is_the_offset_there_is_no_limit_but_the_stores() {
        // 20 rad/s (1146 dps) steady: a working gyro at its scale; the store's range decides.
        let mut run = Run::start(Kind::Gyro, [true, true], Rotation::None).unwrap();
        let out = feed(&mut run, |_| ([0.0, 20.0, -20.0], [0.0; 3]));
        let Outcome::Done([Some(a), Some(_)]) = out else {
            panic!("{out:?}");
        };
        assert!(close(a, [0.0, 20.0, -20.0], 1e-4));
    }

    #[test]
    fn a_repeated_seq_is_counted_once() {
        let mut g = GyroCal::new();
        for _ in 0..10 {
            g.push(7, BIAS);
        }
        assert_eq!(g.count(), 1);
        g.push(8, BIAS);
        g.push(8, BIAS);
        assert_eq!(g.count(), 2);
        let mut a = AccelCal::new(BODY_GRAVITY_MSS);
        for _ in 0..(ACCEL_WINDOW * 3) {
            a.push(3, [0.0, 0.0, -GRAVITY_MSS]);
        }
        assert_eq!(a.windows(), 0);
        let mut run = Run::start(Kind::Gyro, [true, true], Rotation::None).unwrap();
        for seq in 0..(GYRO_SAMPLES - 1) {
            run.push(0, seq, BIAS, [0.0; 3]);
            run.push(0, seq, BIAS, [0.0; 3]);
            run.push(1, seq, BIAS, [0.0; 3]);
        }
        assert_eq!(run.outcome(), Outcome::Running);
        assert_eq!(run.percent(), 99);
    }

    #[test]
    fn the_level_accel_offset_is_the_mean_minus_body_gravity_on_z() {
        let mean = [0.15, -0.21, -GRAVITY_MSS + 0.08];
        let mut run = Run::start(Kind::Accel, [true, true], Rotation::None).unwrap();
        let out = feed(&mut run, |seq| {
            let n = if seq % 2 == 0 { 0.01 } else { -0.01 };
            ([0.0; 3], mean.map(|m| m + n))
        });
        let Outcome::Done([Some(a), Some(b)]) = out else {
            panic!("{out:?}");
        };
        assert!(close(a, [0.15, -0.21, 0.08], 1e-4), "{a:?}");
        assert_eq!(a, b);
        // Converged on the second average: 1 s of samples.
        let mut cal = AccelCal::new(BODY_GRAVITY_MSS);
        for seq in 0..(2 * ACCEL_WINDOW) {
            cal.push(seq, mean);
        }
        assert!(matches!(cal.result(), Some(Ok(_))));
        assert_eq!(cal.windows(), 2);
    }

    #[test]
    fn the_accel_convergence_limit_is_ardupilots() {
        // Each average differs from the previous one by `step` on X.
        for (step, converges) in [(0.049f32, true), (0.051, false)] {
            let mut cal = AccelCal::new(BODY_GRAVITY_MSS);
            for seq in 0..(ACCEL_WINDOW * ACCEL_WINDOWS + 10) {
                let w = (seq / ACCEL_WINDOW) as f32;
                cal.push(seq, [w * step, 0.0, -GRAVITY_MSS]);
            }
            if converges {
                assert!(matches!(cal.result(), Some(Ok(_))), "{step}");
                assert_eq!(cal.windows(), 2);
            } else {
                assert_eq!(cal.result(), Some(Err(Failure::Moving)), "{step}");
                assert_eq!(cal.windows(), ACCEL_WINDOWS);
            }
        }
    }

    #[test]
    fn an_accel_offset_beyond_gravity_fails() {
        // Z reading about 0 (on its side): the Z offset is about +g (the level reading is -g); a
        // little beyond fails.
        for (z, ok) in [(-0.01f32, true), (0.01, false)] {
            let mut run = Run::start(Kind::Accel, [true, false], Rotation::None).unwrap();
            let out = feed(&mut run, |_| ([0.0; 3], [0.0, 0.0, z]));
            if ok {
                assert!(matches!(out, Outcome::Done([Some(_), None])), "{z} {out:?}");
            } else {
                assert_eq!(
                    out,
                    Outcome::Failed(Failure::OffsetTooLarge, Some(0)),
                    "{z}"
                );
            }
        }
    }

    #[test]
    fn a_cancelled_run_gives_no_offset() {
        let mut run = Run::start(Kind::Gyro, [true, true], Rotation::None).unwrap();
        for seq in 0..60 {
            run.push(0, seq, BIAS, [0.0; 3]);
            run.push(1, seq, BIAS, [0.0; 3]);
        }
        run.cancel();
        for seq in 60..400 {
            run.push(0, seq, BIAS, [0.0; 3]);
            run.push(1, seq, BIAS, [0.0; 3]);
        }
        assert_eq!(run.outcome(), Outcome::Failed(Failure::Cancelled, None));
    }

    #[test]
    fn a_missing_imu_is_skipped_and_none_fails() {
        assert_eq!(
            Run::start(Kind::Gyro, [false, false], Rotation::None),
            Err(Failure::NoImu)
        );
        assert_eq!(
            Run::start(Kind::Accel, [false, false], Rotation::None),
            Err(Failure::NoImu)
        );
        let mut run = Run::start(Kind::Gyro, [false, true], Rotation::None).unwrap();
        assert!(!run.includes(0) && run.includes(1));
        // Samples for the skipped IMU are ignored; the run ends on IMU2 alone.
        let out = feed(&mut run, |_| (BIAS, [0.0; 3]));
        let Outcome::Done([None, Some(b)]) = out else {
            panic!("{out:?}");
        };
        assert!(close(b, BIAS, 1e-6));
        // One IMU moving fails the whole run, naming it.
        let mut run = Run::start(Kind::Gyro, [true, true], Rotation::None).unwrap();
        for seq in 0..GYRO_SAMPLES {
            let x = if seq % 2 == 0 { 0.2 } else { -0.2 };
            run.push(0, seq, BIAS, [0.0; 3]);
            run.push(1, seq, [x, 0.0, 0.0], [0.0; 3]);
        }
        assert_eq!(run.outcome(), Outcome::Failed(Failure::Moving, Some(1)));
    }

    /// H3: with the board rotation B = YAW_90 (not its own inverse) and nonzero offsets on every
    /// axis, the path chip sample -> published (`publish`, as the adapters apply it) -> core 0
    /// (`unpublish`) gives the chip sample rotated by the mount M, and both calibrations recover the
    /// injected offsets. The accel reference is `B^T * (0, 0, -g)`.
    const MOUNT: Rotation = Rotation::Roll180;
    const GYRO_OFF: [f32; 3] = [0.012, -0.034, 0.005];
    const ACCEL_OFF: [f32; 3] = [0.15, -0.21, 0.08];

    /// A still, level vehicle seen by the chip: the board-frame truth `B^T * body gravity` plus the
    /// accelerometer offset and the gyro offset, as the chip reports them (M is its own inverse).
    fn chip_sample(board: Rotation) -> ([f32; 3], [f32; 3]) {
        let accel_board = board_gravity(board);
        let accel_board = [
            accel_board[0] + ACCEL_OFF[0],
            accel_board[1] + ACCEL_OFF[1],
            accel_board[2] + ACCEL_OFF[2],
        ];
        (
            MOUNT.rotate_inverse(GYRO_OFF),
            MOUNT.rotate_inverse(accel_board),
        )
    }

    #[test]
    fn publish_and_unpublish_give_the_mounted_chip_sample_with_a_non_involutive_board() {
        let board = Rotation::Yaw90;
        let off = ImuOffsets {
            gyro_rads: GYRO_OFF,
            accel_ms2: ACCEL_OFF,
        };
        let (gyro, accel) = ([0.3, -0.7, 1.1], [1.0, -2.0, 3.0]);
        let (pg, pa) = off.publish(MOUNT, board, gyro, accel);
        // Body frame: B * (M * raw - off).
        assert!(close(
            pg,
            board.rotate(sub(MOUNT.rotate(gyro), GYRO_OFF)),
            1e-6
        ));
        assert!(close(
            pa,
            board.rotate(sub(MOUNT.rotate(accel), ACCEL_OFF)),
            1e-6
        ));
        // Core 0 gets M * raw back (not B * M * raw, not B^T^T).
        let (g2, a2) = off.unpublish(board, pg, pa);
        assert!(close(g2, MOUNT.rotate(gyro), 1e-5), "{g2:?}");
        assert!(close(a2, MOUNT.rotate(accel), 1e-5), "{a2:?}");
    }

    #[test]
    fn the_calibrations_recover_the_injected_offsets_through_a_yaw_90_board() {
        let board = Rotation::Yaw90;
        // The offsets in force while measuring are zero: the published sample is B * (M * chip).
        let (chip_gyro, chip_accel) = chip_sample(board);
        let (pg, pa) = ImuOffsets::ZERO.publish(MOUNT, board, chip_gyro, chip_accel);
        // Level and still: the body frame reads (0, 0, -g) plus the offset rotated by B.
        let (g, a) = ImuOffsets::ZERO.unpublish(board, pg, pa);
        let mut run = Run::start(Kind::Gyro, [true, true], board).unwrap();
        let out = feed(&mut run, |_| (g, a));
        let Outcome::Done([Some(og), Some(_)]) = out else {
            panic!("{out:?}");
        };
        assert!(close(og, GYRO_OFF, 1e-5), "{og:?}");
        let mut run = Run::start(Kind::Accel, [true, true], board).unwrap();
        let out = feed(&mut run, |_| (g, a));
        let Outcome::Done([Some(oa), Some(ob)]) = out else {
            panic!("{out:?}");
        };
        assert!(close(oa, ACCEL_OFF, 1e-4), "{oa:?}");
        assert_eq!(oa, ob);
    }

    /// The accel reference is (0, 0, -g) for B = 0 and (0, 0, +g) for B = ROLL_180; B^T for YAW_90.
    #[test]
    fn the_accel_reference_is_the_body_gravity_in_the_board_frame() {
        assert_eq!(board_gravity(Rotation::None), [0.0, 0.0, -GRAVITY_MSS]);
        assert_eq!(board_gravity(Rotation::Roll180), [0.0, 0.0, GRAVITY_MSS]);
        // YAW_90 leaves z alone and the level gravity has no x or y.
        assert!(close(
            board_gravity(Rotation::Yaw90),
            [0.0, 0.0, -GRAVITY_MSS],
            1e-6
        ));
        // A tilted body vector shows the transpose: B^T (1, 0, 0) for YAW_90 is (0, -1, 0).
        assert_eq!(
            Rotation::Yaw90.rotate_inverse([1.0, 0.0, 0.0]),
            [0.0, -1.0, 0.0]
        );
    }

    #[test]
    fn offsets_are_removed_and_restored() {
        let off = ImuOffsets {
            gyro_rads: BIAS,
            accel_ms2: [0.1, -0.2, 0.3],
        };
        let (g, a) = off.remove([0.1, 0.2, 0.3], [1.0, 2.0, 3.0]);
        assert!(close(g, [0.088, 0.234, 0.295], 1e-6));
        assert!(close(a, [0.9, 2.2, 2.7], 1e-6));
        let (g2, a2) = off.restore(g, a);
        assert!(close(g2, [0.1, 0.2, 0.3], 1e-6) && close(a2, [1.0, 2.0, 3.0], 1e-6));
        assert_eq!(ImuOffsets::ZERO.remove(BIAS, BIAS), (BIAS, BIAS));
    }

    #[test]
    fn stats_and_norm() {
        let mut s = Stats3::new();
        assert_eq!(s.variance(), [0.0; 3]);
        for x in [1.0f32, 2.0, 3.0, 4.0] {
            s.push([x, -x, 0.0]);
        }
        assert!(close(s.mean(), [2.5, -2.5, 0.0], 1e-6));
        assert!(close(s.variance(), [5.0 / 3.0, 5.0 / 3.0, 0.0], 1e-6));
        assert!((norm3([3.0, 4.0, 0.0]) - 5.0).abs() < 1e-6);
        assert!((norm3([0.003, 0.004, 0.0]) - 0.005).abs() < 1e-8);
        assert_eq!(norm3([0.0; 3]), 0.0);
        assert!((sqrt(2.5e-5) - 0.005).abs() < 1e-8);
        assert_eq!(sqrt(-1.0), 0.0);
    }

    const CENTRE: [f32; 3] = [10.0, -20.0, 30.0];

    /// Sample `i` of `n` on the sphere of `radius` about `centre` (golden-angle spiral).
    fn sphere(i: u32, n: u32, radius: f32, centre: [f32; 3]) -> [f32; 3] {
        let z = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
        let r = (1.0 - z * z).sqrt();
        let a = i as f32 * 2.399_963;
        [
            centre[0] + radius * r * a.cos(),
            centre[1] + radius * r * a.sin(),
            centre[2] + radius * z,
        ]
    }

    /// 100 samples per second for 31 s (bounded), through `sample(i)`; the outcome at the end.
    fn mag_run(sample: impl Fn(u32) -> [f32; 3]) -> MagOutcome {
        let mut cal = MagCal::new();
        for i in 0..3_100u32 {
            cal.push(i, 1_000_000 + i as u64 * 10_000, sample(i));
        }
        cal.outcome()
    }

    #[test]
    fn a_full_rotation_gives_the_centre_and_passes() {
        let out = mag_run(|i| sphere(i, 3_000, 50.0, CENTRE));
        let MagOutcome::Done(fit) = out else {
            panic!("{out:?}");
        };
        assert!(close(fit.offset_ut, CENTRE, 0.5), "{fit:?}");
        assert!(close(fit.radius_ut, [50.0; 3], 0.5), "{fit:?}");
    }

    #[test]
    fn the_result_is_the_same_with_offsets_already_in_force() {
        // What core 0 sees is `raw - offsets`; the measurement adds the offsets back.
        let run = |off: [f32; 3]| {
            mag_run(|i| {
                let raw = sphere(i, 3_000, 50.0, CENTRE);
                mag_raw(sub(raw, off), off)
            })
        };
        let (MagOutcome::Done(none), MagOutcome::Done(some)) =
            (run([0.0; 3]), run([3.0, -4.0, 5.0]))
        else {
            panic!();
        };
        assert!(close(none.offset_ut, some.offset_ut, 1e-4));
        assert!(close(none.offset_ut, CENTRE, 0.5));
    }

    #[test]
    fn a_vehicle_at_rest_is_not_rotated() {
        let out = mag_run(|i| {
            let n = if i % 2 == 0 { 0.5 } else { -0.5 };
            [CENTRE[0] + n, CENTRE[1] - n, CENTRE[2] + n]
        });
        let MagOutcome::Failed(Failure::NotRotated, Some(fit)) = out else {
            panic!("{out:?}");
        };
        assert!(fit.radius_ut.iter().all(|r| *r <= 1.0), "{fit:?}");
    }

    #[test]
    fn a_rotation_about_one_axis_is_not_rotated() {
        // A circle in X-Y about Z: Z stays put, its radius is 0.
        let out = mag_run(|i| {
            let a = i as f32 * 0.1;
            [
                CENTRE[0] + 50.0 * a.cos(),
                CENTRE[1] + 50.0 * a.sin(),
                CENTRE[2],
            ]
        });
        let MagOutcome::Failed(Failure::NotRotated, Some(fit)) = out else {
            panic!("{out:?}");
        };
        assert!(fit.radius_ut[2] < 1.0 && fit.radius_ut[0] > 40.0, "{fit:?}");
    }

    #[test]
    fn the_radius_bounds_are_ardupilots_and_exclusive() {
        // The X radius is `r` (min and max samples on X), Y and Z radius 50.
        let run = |r: f32| {
            mag_run(|i| match i % 4 {
                0 => [CENTRE[0] - r, CENTRE[1], CENTRE[2]],
                1 => [CENTRE[0] + r, CENTRE[1], CENTRE[2]],
                2 => [CENTRE[0], CENTRE[1] + 50.0, CENTRE[2] - 50.0],
                _ => [CENTRE[0], CENTRE[1] - 50.0, CENTRE[2] + 50.0],
            })
        };
        assert!(matches!(run(15.2), MagOutcome::Done(_)));
        assert!(matches!(run(94.8), MagOutcome::Done(_)));
        for r in [14.8, 15.0, 95.0, 95.2] {
            assert!(
                matches!(run(r), MagOutcome::Failed(Failure::NotRotated, Some(_))),
                "{r}"
            );
        }
    }

    #[test]
    fn an_offset_beyond_the_parameter_range_fails() {
        let run = |x: f32| mag_run(|i| sphere(i, 3_000, 50.0, [x, 0.0, 0.0]));
        assert!(matches!(run(199.0), MagOutcome::Done(_)));
        assert!(matches!(
            run(201.0),
            MagOutcome::Failed(Failure::OffsetOutOfRange, Some(_))
        ));
    }

    #[test]
    fn the_clock_is_the_sample_time_and_the_first_sample_starts_it() {
        let mut cal = MagCal::new();
        cal.push(0, 5_000_000, CENTRE);
        assert_eq!(cal.percent(), 0);
        assert_eq!(cal.outcome(), MagOutcome::Running);
        cal.push(1, 20_000_000, CENTRE);
        assert_eq!(cal.percent(), 50);
        cal.push(2, 5_000_000 + MAG_DURATION_US - 1, CENTRE);
        assert_eq!(cal.outcome(), MagOutcome::Running);
        cal.push(3, 5_000_000 + MAG_DURATION_US, CENTRE);
        assert_eq!(cal.percent(), 100);
        assert!(matches!(
            cal.outcome(),
            MagOutcome::Failed(Failure::NotRotated, _)
        ));
    }

    #[test]
    fn a_repeated_mag_seq_is_counted_once() {
        let mut cal = MagCal::new();
        cal.push(7, 0, [1.0, 1.0, 1.0]);
        // Same seq, a different field and a later time: ignored.
        cal.push(7, 29_000_000, [100.0, 100.0, 100.0]);
        assert_eq!(cal.percent(), 0);
        cal.push(8, 29_000_000, [-1.0, -1.0, -1.0]);
        assert_eq!(cal.percent(), 96);
        cal.push(8, MAG_DURATION_US + 1, [100.0, 100.0, 100.0]);
        cal.push(9, MAG_DURATION_US + 1, [100.0, 100.0, 100.0]);
        let MagOutcome::Failed(Failure::NotRotated, Some(fit)) = cal.outcome() else {
            panic!();
        };
        // Only the two counted samples: radius 1 on every axis.
        assert_eq!(fit.radius_ut, [1.0; 3]);
        assert_eq!(fit.offset_ut, [0.0; 3]);
    }

    #[test]
    fn a_cancelled_mag_run_gives_no_offset() {
        let mut cal = MagCal::new();
        for i in 0..1_000u32 {
            cal.push(i, i as u64 * 10_000, sphere(i, 3_000, 50.0, CENTRE));
        }
        cal.cancel();
        for i in 1_000..3_100u32 {
            cal.push(i, i as u64 * 10_000, sphere(i, 3_000, 50.0, CENTRE));
        }
        assert_eq!(cal.outcome(), MagOutcome::Failed(Failure::Cancelled, None));
    }
}
