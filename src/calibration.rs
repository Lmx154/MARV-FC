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
//! added back, so the measurement is of the raw values. The decision is `fw_core::calib`. A result
//! is written into the parameters and pushed; nothing is saved.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use fw_core::calib::{Failure, IMUS, ImuOffsets, Kind, Outcome, Run, Stats3, norm3, sqrt};
use fw_core::health::State;
use fw_core::xcore::{TripleBuffer, TripleReader, TripleWriter};
use fw_settings::imu::{ACC_OFF, DEVICES, GYR_OFF};
use mavlink_device::dialect::MavSeverity;
use mavlink_device::marv::MavResult;
use mavlink_device::{ParamName, ParamStore, ParamValue};

use crate::device::{Device, IMU1, IMU2, Reading, Sample};
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
    run: Option<Run>,
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
/// starts, so the first samples are corrected with them.
pub fn init() {
    let [a, b] = &OFFSETS;
    let (Some(a), Some(b)) = (a.take_writer(), b.take_writer()) else {
        defmt::panic!("[FAIL] IMU offset writers taken twice");
    };
    CORE0.lock(|c| {
        *c.borrow_mut() = Some(Core0 {
            writers: [a, b],
            in_force: [ImuOffsets::ZERO; IMUS],
            run: None,
            seen: [None; IMUS],
            gyro: [Stats3::new(); IMUS],
            accel: [Stats3::new(); IMUS],
        });
    });
    push(ALL, "boot");
}

/// The stored offsets of `mask`'s IMUs are pushed at the next supervisor tick (bits of other
/// devices are ignored).
pub fn raise_push(mask: u32) {
    PENDING.fetch_or(mask & ALL, Ordering::Relaxed);
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
                let (g, a) = off.restore(gyro_rads, accel_ms2);
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
    let run = match Run::start(kind, ok) {
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
        c.borrow_mut()
            .as_mut()
            .and_then(|core| core.run.take())
            .map(|r| r.kind())
    });
    CALIBRATING.store(false, Ordering::Relaxed);
    if let Some(kind) = dropped {
        defmt::info!(
            "[INFO] {=str} stopped ({=str}): nothing written",
            name(kind),
            why
        );
    }
}

/// The all-zero PREFLIGHT_CALIBRATION: the run is dropped; nothing is written.
pub fn cancel() {
    stop("cancelled");
}

/// Every supervisor tick while the book runs a calibration: progress, or the end.
pub fn step() -> Step {
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
            let live = write(kind, offsets);
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
/// Live mask.
fn write(kind: Kind, offsets: [Option<[f32; 3]>; IMUS]) -> u32 {
    let table = match kind {
        Kind::Gyro => &GYR_OFF,
        Kind::Accel => &ACC_OFF,
    };
    store::with(|p| {
        for (names, off) in table.iter().zip(offsets) {
            let Some(off) = off else {
                continue;
            };
            for (n, v) in names.iter().zip(off) {
                let index = ParamName::new(n).ok().and_then(|n| p.find(&n));
                let written = index.map(|i| p.set(i, ParamValue::F32(v)));
                if !matches!(written, Some(Ok(_))) {
                    defmt::error!("[INFO] calibration: {=str} not written", *n);
                }
            }
        }
        let _ = (p.take_notice(), p.take_configure_mask());
        p.take_live_mask()
    })
    .unwrap_or(0)
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
