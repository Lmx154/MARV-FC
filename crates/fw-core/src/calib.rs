//! Gyroscope and one-position accelerometer calibration: the accumulation and the decision, pure.
//!
//! Samples go in (per IMU, with the sample's `seq`: a sample is counted once), a per-IMU offset or
//! the reason for the failure comes out. No clock, no hardware, `f32` only.
//!
//! - Gyroscope: Betaflight `performGyroCalibration` (`src/main/sensors/gyro.c`): the samples of a
//!   fixed duration, `gyroCalibrationDuration` = 125 (1.25 s, `pgResetFn_gyroConfig`), their mean is
//!   the offset; moving when the standard deviation (`devStandardDeviation`, `common/maths.c`, n - 1)
//!   of any axis exceeds `gyroMovementCalibrationThreshold` = 48 raw counts, at the 2000 dps scale
//!   Betaflight runs every gyro at (`GYRO_SCALE_2000DPS` = 2000 / 2^15 dps per count,
//!   `drivers/accgyro/accgyro.h`): 2.93 dps. Betaflight starts over when moving; here it fails.
//! - Accelerometer, one position: ArduPilot `AP_InertialSensor::simple_accel_cal`: averages of 50
//!   samples; two successive averages closer than `accel_convergence_limit` = 0.05 m/s^2 (length of
//!   the difference) converge, the offset is their mean minus gravity; not converged within "up to
//!   10 seconds": failed. The largest offset accepted is ArduPilot's `AccelCalibrator::accept_result`
//!   (`GRAVITY_MSS` per axis): `simple_accel_cal` itself has no limit.

use core::f32::consts::PI;

/// Standard gravity, m/s^2 (ArduPilot `GRAVITY_MSS`).
pub const GRAVITY_MSS: f32 = 9.806_65;

/// Gyro samples averaged: Betaflight's 1.25 s at the 100 samples per second core 0 sees per IMU.
pub const GYRO_SAMPLES: u32 = 125;

/// Betaflight's moving limit, 48 counts at 2000 / 2^15 dps per count, in rad/s (standard deviation).
pub const GYRO_MOVING_SD_RADS: f32 = 48.0 * (2000.0 / 32768.0) * PI / 180.0;

/// The largest gyro offset accepted per axis, rad/s: 10 dps, ten times the largest typical
/// zero-rate level of the two chips (LSM6DSK320X `G_TyOff` +-1 dps; ICM-45686 +-0.4 dps board level,
/// +-0.005 dps/C). Neither Betaflight nor ArduPilot limits it (`INS_GYROFFS_*` has no range).
pub const GYRO_OFFSET_MAX_RADS: f32 = 10.0 * PI / 180.0;

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
    /// Gyro offsets, rad/s.
    pub gyro_rads: [f32; 3],
    /// Accelerometer offsets, m/s^2.
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
        let mean = self.stats.mean();
        Some(if self.stats.variance().iter().any(|v| *v > limit) {
            Err(Failure::Moving)
        } else if mean
            .iter()
            .any(|m| m.is_nan() || m.abs() > GYRO_OFFSET_MAX_RADS)
        {
            Err(Failure::OffsetTooLarge)
        } else {
            Ok(mean)
        })
    }
}

/// One IMU's one-position accelerometer calibration (the IMU's +Z axis up).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AccelCal {
    fresh: Fresh,
    sum: [f32; 3],
    in_window: u32,
    windows: u32,
    last: Option<[f32; 3]>,
    done: Option<Result<[f32; 3], Failure>>,
}

impl AccelCal {
    /// No sample yet.
    pub const fn new() -> Self {
        Self {
            fresh: Fresh(None),
            sum: [0.0; 3],
            in_window: 0,
            windows: 0,
            last: None,
            done: None,
        }
    }

    /// Adds a raw sample (m/s^2) unless its `seq` was the previous one or the result is in.
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
            let offset = sub(mean, [0.0, 0.0, GRAVITY_MSS]);
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
    /// A run over the IMUs that are Ok; [`Failure::NoImu`] when none is.
    pub fn start(kind: Kind, ok: [bool; IMUS]) -> Result<Self, Failure> {
        if !ok.iter().any(|o| *o) {
            return Err(Failure::NoImu);
        }
        let imus = ok.map(|o| {
            o.then_some(match kind {
                Kind::Gyro => Imu::Gyro(GyroCal::new()),
                Kind::Accel => Imu::Accel(AccelCal::new()),
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
        assert!((GYRO_OFFSET_MAX_RADS - 0.174_532_9).abs() < 1e-6);
        assert_eq!(GYRO_SAMPLES, 125);
        assert_eq!(ACCEL_WINDOW * ACCEL_WINDOWS, 1000);
        assert_eq!(ACCEL_OFFSET_MAX_MSS, 9.806_65);
    }

    #[test]
    fn a_still_gyro_record_gives_its_mean_as_the_offset() {
        let mut run = Run::start(Kind::Gyro, [true, true]).unwrap();
        // +-1 mrad/s noise around the bias, mean exactly the bias over whole pairs.
        let out = feed(&mut run, |seq| {
            let n = if seq % 2 == 0 { 0.001 } else { -0.001 };
            (BIAS.map(|b| b + n), [0.0, 0.0, GRAVITY_MSS])
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
            let mut run = Run::start(Kind::Gyro, [true, false]).unwrap();
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
    fn a_gyro_offset_beyond_the_limit_fails() {
        let mut run = Run::start(Kind::Gyro, [true, true]).unwrap();
        let big = GYRO_OFFSET_MAX_RADS * 1.01;
        let out = feed(&mut run, |_| ([0.0, big, 0.0], [0.0; 3]));
        assert_eq!(out, Outcome::Failed(Failure::OffsetTooLarge, Some(0)));
        let mut run = Run::start(Kind::Gyro, [true, true]).unwrap();
        let out = feed(&mut run, |_| {
            ([0.0, GYRO_OFFSET_MAX_RADS * 0.99, 0.0], [0.0; 3])
        });
        assert!(matches!(out, Outcome::Done(_)));
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
        let mut a = AccelCal::new();
        for _ in 0..(ACCEL_WINDOW * 3) {
            a.push(3, [0.0, 0.0, GRAVITY_MSS]);
        }
        assert_eq!(a.windows(), 0);
        let mut run = Run::start(Kind::Gyro, [true, true]).unwrap();
        for seq in 0..(GYRO_SAMPLES - 1) {
            run.push(0, seq, BIAS, [0.0; 3]);
            run.push(0, seq, BIAS, [0.0; 3]);
            run.push(1, seq, BIAS, [0.0; 3]);
        }
        assert_eq!(run.outcome(), Outcome::Running);
        assert_eq!(run.percent(), 99);
    }

    #[test]
    fn the_level_accel_offset_is_the_mean_minus_gravity_on_z() {
        let mean = [0.15, -0.21, GRAVITY_MSS + 0.08];
        let mut run = Run::start(Kind::Accel, [true, true]).unwrap();
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
        let mut cal = AccelCal::new();
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
            let mut cal = AccelCal::new();
            for seq in 0..(ACCEL_WINDOW * ACCEL_WINDOWS + 10) {
                let w = (seq / ACCEL_WINDOW) as f32;
                cal.push(seq, [w * step, 0.0, GRAVITY_MSS]);
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
        // Z reading about 0 (on its side): the Z offset is about -g; a little beyond fails.
        for (z, ok) in [(0.01f32, true), (-0.01, false)] {
            let mut run = Run::start(Kind::Accel, [true, false]).unwrap();
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
        let mut run = Run::start(Kind::Gyro, [true, true]).unwrap();
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
        assert_eq!(Run::start(Kind::Gyro, [false, false]), Err(Failure::NoImu));
        assert_eq!(Run::start(Kind::Accel, [false, false]), Err(Failure::NoImu));
        let mut run = Run::start(Kind::Gyro, [false, true]).unwrap();
        assert!(!run.includes(0) && run.includes(1));
        // Samples for the skipped IMU are ignored; the run ends on IMU2 alone.
        let out = feed(&mut run, |_| (BIAS, [0.0; 3]));
        let Outcome::Done([None, Some(b)]) = out else {
            panic!("{out:?}");
        };
        assert!(close(b, BIAS, 1e-6));
        // One IMU moving fails the whole run, naming it.
        let mut run = Run::start(Kind::Gyro, [true, true]).unwrap();
        for seq in 0..GYRO_SAMPLES {
            let x = if seq % 2 == 0 { 0.2 } else { -0.2 };
            run.push(0, seq, BIAS, [0.0; 3]);
            run.push(1, seq, [x, 0.0, 0.0], [0.0; 3]);
        }
        assert_eq!(run.outcome(), Outcome::Failed(Failure::Moving, Some(1)));
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
}
