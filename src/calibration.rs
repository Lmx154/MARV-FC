//! The gyro and one-position accelerometer calibrations (both IMUs, on core 0) and the IMU offsets'
//! path to the flight core.
//!
//! Offsets: per IMU one `TripleBuffer<ImuOffsets>`, written here on core 0, read wait-free by that
//! IMU's adapter on core 1 with every sample, which subtracts them before the sample is published.
//! Pushed at boot before core 1 starts ([`init`]), and within one supervisor tick of a change of
//! the stored values ([`raise_push`]: a parameter set (the Live mask), the calibration defaults, a
//! reload, a calibration result). Pushes wait while a calibration runs, so the offsets in force
//! stay constant during the measurement.
//!
//! Calibration: the samples are the flight rows' latest ones as the supervisor mirrors them every
//! 10 ms (about 100 per second per IMU), each counted once by its `seq`; the current offsets are
//! added back, so the measurement is of the raw values: first un-rotated with the boot's
//! `AHRS_ORIENTATION` (the offsets are in the board frame, between the chip's fixed rotation and
//! the board's), so the measurement is in the frame the offsets are stored in. The decision is
//! `fw_core::calib`. A result is written into the parameters and pushed; nothing is saved.
//!
//! The compass calibration is the same on the MAG row's latest sample (core 0's own device): it
//! measures the raw field r the adapter hands beside the calibrated one (body frame, nothing to
//! undo), and the result is written as one coherent set (`MAG_OFF*`, S the identity,
//! `MAG_CAL_TYPE` 1, `MAG_FLD_UT` 0), which the adapter reads from the store for every sample (no
//! push is needed). Its result or
//! failure text is queued before the action ends, and the final ACK waits until the text queue is
//! empty (`actions::next_ack`), so the text precedes the ACK on the link.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use fw_core::calib::{
    ACCEL_OFFSET_MAX_MSS, Failure, GYRO_FULL_SCALE_RADS, IMUS, ImuOffsets, Kind, MagCal, MagFit,
    MagOutcome, Outcome, Run, Stats3, norm3, sqrt,
};
use fw_core::health::State;
use fw_core::rotation::Rotation;
use fw_core::xcore::{TripleBuffer, TripleReader, TripleWriter};
use fw_settings::imu::{ACC_OFF, DEVICES, GYR_OFF};
use mavlink_device::dialect::MavSeverity;
use mavlink_device::marv::{MarvDevice, MavResult};
use mavlink_device::{ParamName, ParamStore, ParamValue};

use crate::device::{Device, IMU1, IMU2, MAG, Reading, Sample};
use crate::store;

/// Per IMU (IMU1, IMU2) the offsets the adapter subtracts.
static OFFSETS: [TripleBuffer<ImuOffsets>; IMUS] = [
    TripleBuffer::new(ImuOffsets::ZERO),
    TripleBuffer::new(ImuOffsets::ZERO),
];

const ALL: u32 = (1 << DEVICES[0]) | (1 << DEVICES[1]);

/// Core 0's side: the writers, the offsets last pushed, the running calibration, the means for the
/// `[STAT]` line.
struct Core0 {
    writers: [TripleWriter<'static, ImuOffsets>; IMUS],
    in_force: [ImuOffsets; IMUS],
    /// The `AHRS_ORIENTATION` core 1 applies, from the boot.
    board: Rotation,
    run: Option<Run>,
    mag: Option<MagCal>,
    seen: [Option<u32>; IMUS],
    gyro: [Stats3; IMUS],
    accel: [Stats3; IMUS],
}

static CORE0: Mutex<CriticalSectionRawMutex, RefCell<Option<Core0>>> =
    Mutex::new(RefCell::new(None));
/// The IMU device bits whose offsets are to be pushed.
static PENDING: AtomicU32 = AtomicU32::new(0);
static CALIBRATING: AtomicBool = AtomicBool::new(false);

/// What the running calibration did this tick.
pub enum Step {
    Progress(u8),
    End(MavResult),
}

const MAG_NAME: &str = "compass calibration";

fn name(kind: Kind) -> &'static str {
    match kind {
        Kind::Gyro => "gyro calibration",
        Kind::Accel => "accel calibration",
    }
}

/// The adapters' readers (IMU1, IMU2), for core 1's entry. Called once, on core 0, before core 1
/// starts.
pub fn readers() -> [TripleReader<'static, ImuOffsets>; IMUS] {
    let [a, b] = &OFFSETS;
    match (a.take_reader(), b.take_reader()) {
        (Some(a), Some(b)) => [a, b],
        _ => defmt::panic!("[FAIL] IMU offset readers taken twice"),
    }
}

/// Takes the writers and pushes the stored offsets. Call after `settings::boot` and before core 1
/// starts, so the first samples are corrected with them. `board` is the `AHRS_ORIENTATION` the
/// adapters apply (the boot value, never the store's pending one): the samples core 0 sees are in
/// the body frame, and are un-rotated with it before the offsets are added back.
pub fn init(board: Rotation) {
    let [a, b] = &OFFSETS;
    let (Some(a), Some(b)) = (a.take_writer(), b.take_writer()) else {
        defmt::panic!("[FAIL] IMU offset writers taken twice");
    };
    CORE0.lock(|c| {
        *c.borrow_mut() = Some(Core0 {
            writers: [a, b],
            in_force: [ImuOffsets::ZERO; IMUS],
            board,
            run: None,
            mag: None,
            seen: [None; IMUS],
            gyro: [Stats3::new(); IMUS],
            accel: [Stats3::new(); IMUS],
        });
    });
    push(ALL, "boot");
}

/// The stored offsets of `mask`'s IMUs are pushed at the next supervisor tick (bits of other
/// devices are ignored, but device 0's: its Live parameters include `PILOT_SPEED_XY`, `RPM_NOTCH_EN`, the `GDN_*` targets and the 14 `ATC_*` gains, pushed now by
/// `flight::push_speed`, `flight::push_notch`, `flight::push_gdn` and `flight::push_gains`, and the ESC device's: `MOT_POLES`, pushed by `flight::push_notch`).
pub fn raise_push(mask: u32) {
    PENDING.fetch_or(mask & ALL, Ordering::Relaxed);
    // Device 0's Live parameters: `PILOT_SPEED_XY` goes to the flight core at once (the flight code latches it at arming).
    if mask & 1 != 0 {
        crate::flight::push_speed("parameters changed");
    }
    // `RPM_NOTCH_EN` (device 0) and `MOT_POLES` (the ESC device) go to the flight core together (the flight code latches
    // them at arming).
    if mask & (1 | 1 << MarvDevice::MARV_DEVICE_ESC as u32) != 0 {
        crate::flight::push_notch("parameters changed");
    }
    // The `GDN_*` targets (device 0) go to the flight core, which takes them on a disarmed step.
    if mask & 1 != 0 {
        crate::flight::push_gdn("parameters changed");
    }
    // The 14 `ATC_*` gains (device 0) go to the flight core as one set (the flight code latches them at arming).
    if mask & 1 != 0 {
        crate::flight::push_gains("parameters changed");
    }
}

/// A calibration is running (HEARTBEAT `system_status` CALIBRATING).
pub fn calibrating() -> bool {
    CALIBRATING.load(Ordering::Relaxed)
}

fn stored(imu: usize) -> ImuOffsets {
    let read = |names: Option<&[&str; 3]>| {
        store::with(|p| {
            names.map_or([0.0; 3], |n| {
                n.map(|n| match p.get_named(n) {
                    Some(ParamValue::F32(v)) => v,
                    _ => 0.0,
                })
            })
        })
        .unwrap_or([0.0; 3])
    };
    ImuOffsets {
        gyro_rads: read(GYR_OFF.get(imu)),
        accel_ms2: read(ACC_OFF.get(imu)),
    }
}

fn push(mask: u32, why: &str) {
    for (imu, dev) in DEVICES.iter().enumerate() {
        if mask & (1 << dev) == 0 {
            continue;
        }
        let off = stored(imu);
        CORE0.lock(|c| {
            if let Some(core) = c.borrow_mut().as_mut()
                && let (Some(w), Some(f)) = (core.writers.get_mut(imu), core.in_force.get_mut(imu))
            {
                w.write(off);
                *f = off;
            }
        });
        defmt::info!(
            "[INFO] IMU{=usize} offsets pushed to the flight core ({=str}): gyro {} rad/s, accel {} m/s^2",
            imu + 1,
            why,
            off.gyro_rads,
            off.accel_ms2
        );
    }
}

/// The latest mirrored sample of each IMU row and whether the row is Ok.
fn latest() -> [(bool, Option<Sample>); IMUS] {
    let rows: [&'static Device; IMUS] = [&IMU1, &IMU2];
    rows.map(|d| (d.status().state == State::Ok, d.latest.try_get()))
}

/// Feeds the `[STAT]` means (the samples as published, offsets removed) and the running
/// calibration (offsets added back). Returns the outcome when a calibration runs.
fn observe() -> Option<(Kind, Outcome, u8)> {
    let samples = latest();
    CORE0.lock(|c| {
        let mut c = c.borrow_mut();
        let core = c.as_mut()?;
        for (imu, (_, sample)) in samples.iter().enumerate() {
            let Some(Sample {
                reading:
                    Reading::Imu {
                        gyro_rads,
                        accel_ms2,
                        ..
                    },
                seq,
                ..
            }) = *sample
            else {
                continue;
            };
            if let (Some(seen), Some(g), Some(a)) = (
                core.seen.get_mut(imu),
                core.gyro.get_mut(imu),
                core.accel.get_mut(imu),
            ) && *seen != Some(seq)
            {
                *seen = Some(seq);
                g.push(gyro_rads);
                a.push(accel_ms2);
            }
            let off = core.in_force.get(imu).copied().unwrap_or(ImuOffsets::ZERO);
            if let Some(run) = core.run.as_mut() {
                let (g, a) = off.unpublish(core.board, gyro_rads, accel_ms2);
                run.push(imu, seq, g, a);
            }
        }
        core.run
            .as_ref()
            .map(|r| (r.kind(), r.outcome(), r.percent()))
    })
}

/// Starts a calibration over the IMUs that are Ok (a skipped one gets a text). `false`: none is
/// Ok, the text is sent and the action ends FAILED.
pub fn start(kind: Kind) -> bool {
    let ok = latest().map(|(ok, _)| ok);
    let board = CORE0.lock(|c| c.borrow().as_ref().map(|core| core.board));
    let run = match Run::start(kind, ok, board.unwrap_or_default()) {
        Ok(run) => run,
        Err(why) => {
            defmt::warn!("[INFO] {=str}: {=str}", name(kind), why.text());
            crate::text_fmt(
                MavSeverity::MAV_SEVERITY_WARNING,
                format_args!("{}: {}", name(kind), why.text()),
            );
            return false;
        }
    };
    for (imu, ok) in ok.iter().enumerate() {
        if !ok {
            crate::text_fmt(
                MavSeverity::MAV_SEVERITY_WARNING,
                format_args!("{}: IMU{} not Ok, skipped", name(kind), imu + 1),
            );
        }
    }
    CORE0.lock(|c| {
        if let Some(core) = c.borrow_mut().as_mut() {
            core.run = Some(run);
        }
    });
    CALIBRATING.store(true, Ordering::Relaxed);
    defmt::info!(
        "[INFO] {=str} started: IMU1 {=str}, IMU2 {=str}; HEARTBEAT system_status CALIBRATING",
        name(kind),
        if run.includes(0) { "in" } else { "skipped" },
        if run.includes(1) { "in" } else { "skipped" }
    );
    true
}

fn stop(why: &str) {
    let dropped = CORE0.lock(|c| {
        c.borrow_mut().as_mut().and_then(|core| {
            let imu = core.run.take().map(|r| name(r.kind()));
            let mag = core.mag.take().map(|_| MAG_NAME);
            imu.or(mag)
        })
    });
    CALIBRATING.store(false, Ordering::Relaxed);
    if let Some(name) = dropped {
        defmt::info!("[INFO] {=str} stopped ({=str}): nothing written", name, why);
    }
}

/// The all-zero PREFLIGHT_CALIBRATION: the run is dropped; nothing is written.
pub fn cancel() {
    stop("cancelled");
}

/// Every supervisor tick while the book runs a calibration: progress, or the end.
pub fn step() -> Step {
    if CORE0.lock(|c| c.borrow().as_ref().is_some_and(|core| core.mag.is_some())) {
        return step_mag();
    }
    let Some((kind, outcome, percent)) = observe() else {
        return Step::End(MavResult::MAV_RESULT_FAILED);
    };
    match outcome {
        Outcome::Running => Step::Progress(percent),
        Outcome::Failed(why, imu) => {
            stop(why.text());
            match (why, imu) {
                (Failure::OffsetTooLarge, Some(i)) => crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_WARNING,
                    format_args!("{}: IMU{} {}", name(kind), i + 1, why.text()),
                ),
                _ => crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_WARNING,
                    format_args!("{}: {}", name(kind), why.text()),
                ),
            }
            Step::End(if why == Failure::Cancelled {
                MavResult::MAV_RESULT_CANCELLED
            } else {
                MavResult::MAV_RESULT_FAILED
            })
        }
        Outcome::Done(offsets) => {
            CORE0.lock(|c| {
                if let Some(core) = c.borrow_mut().as_mut() {
                    core.run = None;
                }
            });
            CALIBRATING.store(false, Ordering::Relaxed);
            let Some(live) = write(kind, offsets) else {
                defmt::warn!("[INFO] {=str}: offset refused by the store", name(kind));
                crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_WARNING,
                    format_args!("{}: offset out of range", name(kind)),
                );
                return Step::End(MavResult::MAV_RESULT_FAILED);
            };
            raise_push(live);
            result_text(kind, offsets);
            Step::End(MavResult::MAV_RESULT_ACCEPTED)
        }
    }
}

/// Every supervisor tick while no calibration runs: a run the book ended (time limit) is dropped,
/// and the pending pushes are done.
pub fn idle() {
    observe();
    if calibrating() {
        stop("ended by the book");
    }
    let mask = PENDING.swap(0, Ordering::Relaxed);
    if mask != 0 {
        push(mask, "parameters changed");
    }
}

/// Writes the offsets into the parameters (the store is dirty; nothing is saved). Returns the
/// Live mask, or `None` with nothing written when a value is outside the parameters' range
/// (checked before the first write) or the store refuses one.
fn write(kind: Kind, offsets: [Option<[f32; 3]>; IMUS]) -> Option<u32> {
    let (table, limit) = match kind {
        Kind::Gyro => (&GYR_OFF, GYRO_FULL_SCALE_RADS),
        Kind::Accel => (&ACC_OFF, ACCEL_OFFSET_MAX_MSS),
    };
    if !offsets.iter().flatten().flatten().all(|v| v.abs() <= limit) {
        return None;
    }
    let all_stored = store::with(|p| {
        let mut all = true;
        for (names, off) in table.iter().zip(offsets) {
            let Some(off) = off else {
                continue;
            };
            for (n, v) in names.iter().zip(off) {
                let index = ParamName::new(n).ok().and_then(|n| p.find(&n));
                let written = index.map(|i| p.set(i, ParamValue::F32(v)));
                if !matches!(written, Some(Ok(_))) {
                    defmt::error!("[INFO] calibration: {=str} not written", *n);
                    all = false;
                }
            }
        }
        let _ = (p.take_notice(), p.take_configure_mask());
        (all, p.take_live_mask())
    })?;
    all_stored.0.then_some(all_stored.1)
}

fn result_text(kind: Kind, offsets: [Option<[f32; 3]>; IMUS]) {
    let [a, b] = offsets;
    defmt::info!(
        "[INFO] {=str} done: IMU1 {} IMU2 {} ({=str}), written to the parameters, not saved",
        name(kind),
        a,
        b,
        match kind {
            Kind::Gyro => "rad/s",
            Kind::Accel => "m/s^2",
        }
    );
    let (unit, short) = match kind {
        Kind::Gyro => ("urad/s", "gyro cal"),
        Kind::Accel => ("mm/s^2", "accel cal"),
    };
    let scale = if kind == Kind::Gyro { 1e6 } else { 1e3 };
    let f = |o: Option<[f32; 3]>| o.map(|o| (norm3(o) * scale) as i32);
    match (f(a), f(b)) {
        (Some(a), Some(b)) => crate::text_fmt(
            MavSeverity::MAV_SEVERITY_INFO,
            format_args!("{}: |IMU1| {} |IMU2| {} {}", short, a, b, unit),
        ),
        (Some(a), None) => crate::text_fmt(
            MavSeverity::MAV_SEVERITY_INFO,
            format_args!("{}: |IMU1| {} {}, IMU2 skipped", short, a, unit),
        ),
        (None, Some(b)) => crate::text_fmt(
            MavSeverity::MAV_SEVERITY_INFO,
            format_args!("{}: IMU1 skipped, |IMU2| {} {}", short, b, unit),
        ),
        (None, None) => {}
    }
}

/// Starts the compass calibration. `false`: the MAG row is not Ok, the text is sent and the action
/// ends FAILED.
pub fn start_mag() -> bool {
    if MAG.status().state != State::Ok {
        defmt::warn!("[INFO] {=str}: MAG not ok", MAG_NAME);
        crate::text(MavSeverity::MAV_SEVERITY_WARNING, "compass cal: MAG not ok");
        return false;
    }
    CORE0.lock(|c| {
        if let Some(core) = c.borrow_mut().as_mut() {
            core.mag = Some(MagCal::new());
        }
    });
    CALIBRATING.store(true, Ordering::Relaxed);
    defmt::info!(
        "[INFO] {=str} started: rotate the vehicle through all orientations for 30 s; HEARTBEAT system_status CALIBRATING",
        MAG_NAME
    );
    true
}

/// The synthetic input of the test feature `inject-mag-rotation`: the six points of a sphere of
/// radius 50 uT about (10, -20, 30) uT, each held for 100 samples, in a cycle.
#[cfg(feature = "inject-mag-rotation")]
fn injected(seq: u32) -> [f32; 3] {
    let mut p = [10.0, -20.0, 30.0];
    let i = (seq / 100 % 6) as usize;
    if let Some(x) = p.get_mut(i / 2) {
        *x += if i.is_multiple_of(2) { 50.0 } else { -50.0 };
    }
    p
}

/// Feeds the compass calibration the MAG row's latest raw sample r (body frame, before the
/// calibration in force).
fn observe_mag() -> Option<(MagOutcome, u8)> {
    let sample = MAG.latest.try_get();
    CORE0.lock(|c| {
        let mut c = c.borrow_mut();
        let cal = c.as_mut()?.mag.as_mut()?;
        if let Some(Sample {
            reading: Reading::Mag { raw_ut: raw, .. },
            t_us,
            seq,
            ..
        }) = sample
        {
            #[cfg(feature = "inject-mag-rotation")]
            let raw = {
                let _ = raw;
                injected(seq)
            };
            cal.push(seq, t_us, raw);
        }
        Some((cal.outcome(), cal.percent()))
    })
}

/// Integer microtesla, rounded to the nearest.
fn ut(x: f32) -> i32 {
    (x + if x < 0.0 { -0.5 } else { 0.5 }) as i32
}

fn step_mag() -> Step {
    let Some((outcome, percent)) = observe_mag() else {
        return Step::End(MavResult::MAV_RESULT_FAILED);
    };
    match outcome {
        MagOutcome::Running => Step::Progress(percent),
        MagOutcome::Failed(why, fit) => {
            stop(why.text());
            match (why, fit) {
                (Failure::NotRotated, Some(f)) => crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_WARNING,
                    format_args!(
                        "compass cal: not rotated, radius {} {} {} uT",
                        ut(f.radius_ut[0]),
                        ut(f.radius_ut[1]),
                        ut(f.radius_ut[2])
                    ),
                ),
                (Failure::OffsetOutOfRange, Some(f)) => crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_WARNING,
                    format_args!(
                        "compass cal: offset out of range {} {} {} uT",
                        ut(f.offset_ut[0]),
                        ut(f.offset_ut[1]),
                        ut(f.offset_ut[2])
                    ),
                ),
                _ => crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_WARNING,
                    format_args!("compass cal: {}", why.text()),
                ),
            }
            Step::End(if why == Failure::Cancelled {
                MavResult::MAV_RESULT_CANCELLED
            } else {
                MavResult::MAV_RESULT_FAILED
            })
        }
        MagOutcome::Done(fit) => {
            CORE0.lock(|c| {
                if let Some(core) = c.borrow_mut().as_mut() {
                    core.mag = None;
                }
            });
            CALIBRATING.store(false, Ordering::Relaxed);
            if !write_mag(&fit) {
                crate::text(
                    MavSeverity::MAV_SEVERITY_WARNING,
                    "compass cal: offset out of range",
                );
                return Step::End(MavResult::MAV_RESULT_FAILED);
            }
            defmt::info!(
                "[INFO] {=str} done: offsets {} uT, radii {} uT, written to the parameters, not saved",
                MAG_NAME,
                fit.offset_ut,
                fit.radius_ut
            );
            crate::text_fmt(
                MavSeverity::MAV_SEVERITY_INFO,
                format_args!(
                    "compass cal: off {} {} {} r {} {} {} uT",
                    ut(fit.offset_ut[0]),
                    ut(fit.offset_ut[1]),
                    ut(fit.offset_ut[2]),
                    ut(fit.radius_ut[0]),
                    ut(fit.radius_ut[1]),
                    ut(fit.radius_ut[2])
                ),
            );
            Step::End(MavResult::MAV_RESULT_ACCEPTED)
        }
    }
}

/// Writes the quick result as one coherent set in one store access (`MAG_OFFX/Y/Z`, S the
/// identity, `MAG_CAL_TYPE` 1, `MAG_FLD_UT` 0: `fw_settings::mag::write_quick_result`; the store
/// is dirty; nothing is saved; the adapter reads them from the next sample on). `false` when the
/// store refuses a value (the pure check keeps the offsets inside the parameters' range, so it
/// does not happen).
fn write_mag(fit: &MagFit) -> bool {
    store::with(|p| {
        let all = fw_settings::mag::write_quick_result(p, fit.offset_ut);
        let _ = (p.take_notice(), p.take_configure_mask(), p.take_live_mask());
        all
    })
    .unwrap_or(false)
}

/// Every 10 s: per IMU the mean and standard deviation of the samples as published (offsets
/// removed) since the previous line.
pub fn stat() {
    let means = CORE0.lock(|c| {
        c.borrow_mut().as_mut().map(|core| {
            let out = (core.gyro, core.accel);
            core.gyro = [Stats3::new(); IMUS];
            core.accel = [Stats3::new(); IMUS];
            out
        })
    });
    let Some((gyro, accel)) = means else {
        return;
    };
    for (imu, (g, a)) in gyro.iter().zip(accel.iter()).enumerate() {
        defmt::info!(
            "[STAT] IMU{=usize} mean of {=u32} samples: gyro {} mrad/s (sd {}), accel {} m/s^2 (sd {})",
            imu + 1,
            g.count(),
            g.mean().map(|x| x * 1000.0),
            g.variance().map(|v| sqrt(v) * 1000.0),
            a.mean(),
            a.variance().map(sqrt)
        );
    }
}
