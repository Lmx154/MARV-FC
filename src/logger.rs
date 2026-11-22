//! The log collector (decision-sd-logging Q4, Q5; milestone SD3): `log_collect`, a core-0
//! thread-mode task on a 10 ms `Ticker`, the one producer of the log ring [`RING`]; the SD task
//! (`sd`) is its one consumer and writes the ring to `LOGS/NNNNNNNN.BIN` (Q4-I1).
//!
//! Records go in only while a log is open: [`OPEN`] holds the open log's number, stored by the SD
//! task after it wrote the header; while it is 0 nothing is pushed and nothing counts as dropped.
//! When a log opens the collector first commits [`PAD`] filler bytes, which the SD task consumes
//! without writing them (they put the ring's read index and the file offset in step modulo 4096,
//! so that whole pieces are written in place), then a PARM for every parameter and a DEV for
//! every row. Then, per Q5: IMU2 at 25 Hz from its seqlock (every 4th tick, only a new `seq`,
//! `TimeUS` the sample's own time; IMU1 comes from the flight core, below), BARO, MAG, BAT and
//! GPS (each with a GPA, and a GPV when it has a velocity) at every new sample, RCIN at 10 Hz while stick reports are fresh, DEV for every row at
//! 1 Hz and at every state change, MSG for every text (the text ring's third cursor, so the boot
//! texts come first), PARM after every change of a parameter's value, whatever made it (a set,
//! a calibration result, a reset, a
//! reload: every tick the store is compared with the values last logged), and DSF once per second from the figures the SD task hands over
//! in [`DSF_FIGURES`] (it owns the ring's statistics); Dp is counted here. A record that does not
//! fit is dropped and counted by the ring. The collector never awaits the card and takes no lock
//! that the SD task holds across a card call (Q4-I4): its only locks are short critical sections.
//!
//! The flight records (decision-flight-logging-20261119, section 3): the collector is the one
//! consumer of `flight::LOG`, which IMU1's `Stepper` fills on core 1. Every tick, open log or not,
//! `fw_core::relay::relay` moves its records into [`RING`] (into nothing while no log is open), so
//! the flight ring is always drained and a full [`RING`] never backs up into core 1; with DSF, FDS
//! (the flight ring's drops since the log opened and its smallest free space).

use core::cell::Cell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Ticker};
use fw_core::dflog::{self, Dev, Dsf, Gps};
use fw_core::extring::{ExtLogRing, LogProducer};
use fw_core::health::State;
use fw_core::relay::relay_with;
use fw_core::sample::{BaroReading, GnssReading, MagReading};
use fw_core::sys_status::{StatusInput, device_status};
use fw_core::watchdog::task;
use fw_core::xcore::LogConsumer;
use fw_flight::tagged;
use mavlink_device::marv::MARV_DEVICE_STATUS_DATA;
use mavlink_device::{ParamStore, ParamValue};

use crate::device::{BARO, BATT, DEVICE_COUNT, DEVICES, Device, GNSS, MAG, Reading, Sample};
use crate::{LINK_LOG, ROWS};

/// The ring's size (decision-flight-logging-20261119, section 2: 64 KiB).
pub const RING_BYTES: usize = 64 * 1024;
/// The writer's piece: the SD task writes whole pieces of this size in place.
pub const PIECE: usize = 4096;
/// The log ring: the collector's records, the SD task's bytes. Its storage is in the PSRAM (`psram::LOG_BUF`),
/// bound by `psram::init`; without a working PSRAM it stays unbound, `take_producer` and `take_consumer` give
/// `None`, and nothing is logged (decision-psram-offload-20261121, D3).
pub static RING: ExtLogRing<RING_BYTES> = ExtLogRing::new();
/// Collector -> SD task: at least one piece waits in the ring.
pub static DATA: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// SD task -> collector: the open log's number, 0 while no log is open.
pub static OPEN: AtomicU32 = AtomicU32::new(0);
/// SD task -> collector: filler bytes to commit before the first record of the log in `OPEN`
/// (stored before `OPEN`).
pub static PAD: AtomicU32 = AtomicU32::new(0);
/// SD task -> collector: the DSF figures of the last second (`dropped` is filled in here).
pub static DSF_FIGURES: Mutex<CriticalSectionRawMutex, Cell<Option<Dsf>>> =
    Mutex::new(Cell::new(None));

type Producer = LogProducer<'static, RING_BYTES>;
type FlightConsumer = LogConsumer<'static, { crate::flight::LOG_BYTES }>;

const TICK: Duration = Duration::from_millis(10);
/// IMU records every 4th tick (25 Hz).
const IMU_EVERY: u32 = 4;
/// RCIN every 10th tick (10 Hz).
const RCIN_EVERY: u32 = 10;
/// DEV of every row every 100th tick (1 Hz).
const DEV_EVERY: u32 = 100;

#[embassy_executor::task]
pub async fn task() -> ! {
    tagged(&crate::watchdog::RUNNING0, task::LOG, collect()).await
}

async fn collect() -> ! {
    // `None` when the ring is unbound (no PSRAM: `psram::init` bound nothing). The tasks start after `psram::init`,
    // so the ring is bound by now or never; there is no retry.
    let mut producer = RING.take_producer();
    if producer.is_none() && !RING.is_bound() {
        defmt::info!(
            "[INFO] log: no PSRAM, the log ring is not bound: nothing is logged, the flight ring is drained into nothing"
        );
    }
    let Some(flight) = crate::flight::LOG.take_consumer() else {
        defmt::panic!("[FAIL] flight log ring: consumer taken twice");
    };
    let mut ticker = Ticker::every(TICK);
    let mut c = Collector {
        tick: 0,
        open: 0,
        dropped_at_open: 0,
        flight,
        flight_dropped_at_open: 0,
        imu_due: false,
        imu_seq: None,
        seq: [None; 4],
        gnss_t_us: None,
        rows: [None; ROWS],
        params: [None; fw_settings::COUNT],
    };
    loop {
        ticker.next().await;
        match producer.as_mut() {
            // `psram::ready()` once per tick: after a FAULT at run time (the canary failed after a flash
            // operation) the ring's bytes are not touched any more (the tick's pushes are the PSRAM batch).
            Some(p) if crate::psram::ready() => c.tick(p, Instant::now().as_micros()),
            _ => c.drain_flight(),
        }
    }
}

struct Collector {
    tick: u32,
    /// The log being collected for (0: none).
    open: u32,
    /// The ring's drop count when that log opened.
    dropped_at_open: u32,
    /// The consumer of `flight::LOG`.
    flight: FlightConsumer,
    /// The flight ring's drop count when that log opened.
    flight_dropped_at_open: u32,
    /// IMU2: a record is due (set every 4th tick, cleared by a read of the seqlock).
    imu_due: bool,
    imu_seq: Option<u32>,
    /// The last `seq` logged of BARO, MAG, GNSS, BATT.
    seq: [Option<u32>; 4],
    /// The `t_us` of the last GNSS sample logged in this log (GPA's `Delta`).
    gnss_t_us: Option<u64>,
    /// Per row the state (and stuck flag) of its last DEV record.
    rows: [Option<(State, bool)>; ROWS],
    /// Per parameter the value of its last PARM record.
    params: [Option<ParamValue>; fw_settings::COUNT],
}

impl Collector {
    /// No ring to fill (no PSRAM, or a FAULT since): the flight ring is still drained, into nothing, so that it
    /// never backs up into core 1 (`flight::LOG` drops stay 0).
    fn drain_flight(&mut self) {
        relay_with(&mut self.flight, None::<fn(&[u8]) -> bool>);
    }

    fn tick(&mut self, p: &mut Producer, now: u64) {
        let tick = self.tick;
        self.tick = tick.wrapping_add(1);
        if tick.is_multiple_of(IMU_EVERY) {
            self.imu_due = true;
        }
        let open = OPEN.load(Ordering::Acquire);
        if open != self.open {
            self.open = open;
            if open != 0 {
                self.start(p);
            }
        }
        // Every tick: the flight ring is drained whether a log is open or not.
        relay_with(
            &mut self.flight,
            (self.open != 0).then(|| |record: &[u8]| p.push(record)),
        );
        if self.open == 0 {
            return;
        }
        self.parms(p, now);
        self.imus(p);
        self.sensors(p);
        if tick.is_multiple_of(RCIN_EVERY)
            && let Some(rc) = crate::rc::rc_channels(now)
        {
            let channels = [
                rc.chan1_raw,
                rc.chan2_raw,
                rc.chan3_raw,
                rc.chan4_raw,
                rc.chan5_raw,
                rc.chan6_raw,
                rc.chan7_raw,
                rc.chan8_raw,
                rc.chan9_raw,
                rc.chan10_raw,
                rc.chan11_raw,
                rc.chan12_raw,
                rc.chan13_raw,
                rc.chan14_raw,
            ];
            p.push_with(dflog::RCIN.length.into(), |b| {
                dflog::encode_rcin(b, now, &channels)
            });
        }
        self.devs(p, now, tick.is_multiple_of(DEV_EVERY));
        while let Some((_, line)) = crate::take_text(LINK_LOG) {
            p.push_with(dflog::MSG.length.into(), |b| {
                dflog::encode_msg(b, now, line.as_str().as_bytes())
            });
        }
        if let Some(mut dsf) = DSF_FIGURES.lock(Cell::take) {
            dsf.dropped = p.dropped().saturating_sub(self.dropped_at_open);
            p.push_with(dflog::DSF.length.into(), |b| {
                dflog::encode_dsf(b, now, &dsf)
            });
            let dropped = self
                .flight
                .dropped()
                .saturating_sub(self.flight_dropped_at_open);
            let free_min = self.flight.take_stats().free_min;
            p.push_with(dflog::FDS.length.into(), |b| {
                dflog::encode_fds(b, now, dropped, free_min)
            });
        }
        if RING.readable() >= PIECE {
            DATA.signal(());
        }
    }

    /// A log opened: the filler; a PARM for every parameter, every row's DEV and IMU2 follow in
    /// this tick.
    fn start(&mut self, p: &mut Producer) {
        let pad = PAD.load(Ordering::Relaxed) as usize;
        if pad > 0 {
            match p.reserve(pad) {
                Some(grant) => grant.commit(),
                None => defmt::warn!("[INFO] log: the {=usize} B filler did not fit", pad),
            }
        }
        self.dropped_at_open = p.dropped();
        self.flight_dropped_at_open = self.flight.dropped();
        // FDS's first FMn covers this log only.
        self.flight.take_stats();
        self.rows = [None; ROWS];
        self.gnss_t_us = None;
        self.imu_due = true;
        self.params = [None; fw_settings::COUNT];
        DSF_FIGURES.lock(|c| c.set(None));
    }

    /// A PARM for every parameter whose value differs from its last record (all of them right
    /// after a log opened).
    fn parms(&mut self, p: &mut Producer, now: u64) {
        let current: [Option<ParamValue>; fw_settings::COUNT] =
            crate::store::with(|s| core::array::from_fn(|i| s.get(i as u16)))
                .unwrap_or([None; fw_settings::COUNT]);
        for (i, (value, last)) in current.into_iter().zip(self.params.iter_mut()).enumerate() {
            if same(value, *last) {
                continue;
            }
            *last = value;
            if value.is_some() {
                parm(p, now, i as u16);
            }
        }
    }

    /// IMU2 (instance 1); instance 0 is IMU1's, from the flight core.
    fn imus(&mut self, p: &mut Producer) {
        if !self.imu_due {
            return;
        }
        let Some(status) = crate::imu2::CELLS.read() else {
            return;
        };
        self.imu_due = false;
        let Some(s) = status.latest else {
            return;
        };
        if self.imu_seq == Some(s.seq) {
            return;
        }
        self.imu_seq = Some(s.seq);
        let r = s.reading;
        p.push_with(dflog::IMU.length.into(), |b| {
            dflog::encode_imu(b, s.t_us, 1, r.gyro_rads, r.accel_ms2, r.temperature_c)
        });
    }

    fn sensors(&mut self, p: &mut Producer) {
        let [baro, mag, gnss, batt] = &mut self.seq;
        if let Some(Sample {
            reading:
                Reading::Baro(BaroReading {
                    pressure_pa,
                    temperature_c,
                }),
            t_us,
            ..
        }) = fresh(&BARO, baro)
        {
            p.push_with(dflog::BARO.length.into(), |b| {
                dflog::encode_baro(b, t_us, 0, pressure_pa, temperature_c)
            });
        }
        if let Some(Sample {
            reading:
                Reading::Mag {
                    reading: MagReading { field_ut, .. },
                    ..
                },
            t_us,
            ..
        }) = fresh(&MAG, mag)
        {
            let mgauss = field_ut.map(|ut| {
                let v = ut * 10.0;
                (if v < 0.0 { v - 0.5 } else { v + 0.5 }) as i16
            });
            p.push_with(dflog::MAG.length.into(), |b| {
                dflog::encode_mag(b, t_us, 0, mgauss)
            });
        }
        if let Some(Sample {
            reading:
                Reading::Batt {
                    voltage_v,
                    current_a,
                },
            t_us,
            ..
        }) = fresh(&BATT, batt)
        {
            p.push_with(dflog::BAT.length.into(), |b| {
                dflog::encode_bat(b, t_us, 0, voltage_v, current_a)
            });
        }
        if let Some(sample) = fresh(&GNSS, gnss)
            && let Some(g) = gps(&sample.reading)
        {
            p.push_with(dflog::GPS.length.into(), |b| {
                dflog::encode_gps(b, sample.t_us, &g)
            });
            if let Reading::Gnss(GnssReading {
                vdop_x100,
                lat_std_mm,
                lon_std_mm,
                alt_std_mm,
                ..
            }) = sample.reading
            {
                let delta_ms =
                    dflog::gpa_delta_ms(self.gnss_t_us.replace(sample.t_us), sample.t_us);
                p.push_with(dflog::GPA.length.into(), |b| {
                    dflog::encode_gpa(
                        b,
                        sample.t_us,
                        vdop_x100.unwrap_or(u16::MAX),
                        lat_std_mm,
                        lon_std_mm,
                        alt_std_mm,
                        delta_ms,
                    )
                });
            }
            if let Reading::Gnss(GnssReading {
                time,
                vel_ned_mm_s: Some(vel),
                ..
            }) = sample.reading
            {
                let pvt = crate::gnss::pvt_log(time);
                p.push_with(dflog::GPV.length.into(), |b| {
                    dflog::encode_gpv(
                        b,
                        sample.t_us,
                        vel,
                        pvt.map_or(0, |x| x.tow_ms),
                        pvt.and_then(|x| x.speed_mm_s).unwrap_or(u32::MAX),
                        pvt.and_then(|x| x.heading_cdeg).unwrap_or(u16::MAX),
                    )
                });
            }
        }
    }

    /// DEV of every row (`all`) or of each row whose state changed since its last record.
    fn devs(&mut self, p: &mut Producer, now: u64, all: bool) {
        for (i, last) in self.rows.iter_mut().enumerate() {
            let message = match DEVICES.get(i) {
                Some(dev) => {
                    let status = dev.status();
                    let stuck = dev.stuck.load(Ordering::Relaxed);
                    if !all && *last == Some((status.state, stuck)) {
                        continue;
                    }
                    *last = Some((status.state, stuck));
                    row_message(dev, status, stuck)
                }
                None if i == DEVICE_COUNT => {
                    let state = crate::rc::row().state;
                    if !all && *last == Some((state, false)) {
                        continue;
                    }
                    *last = Some((state, false));
                    crate::rc::message(now)
                }
                None => continue,
            };
            let d = Dev {
                id: message.device_id as u8,
                state: message.state as u8,
                err: message.last_error as u8,
                samples: message.samples,
                errors: message.errors,
                timeouts: message.timeouts,
                recoveries: message.recoveries,
                attempts: message.attempts,
                overflows: message.overflows,
            };
            p.push_with(dflog::DEV.length.into(), |b| dflog::encode_dev(b, now, &d));
        }
    }
}

/// A registry row as its MARV_DEVICE_STATUS would carry it (built here rather than with
/// `Device::message`, which counts the rows sent during a self-test).
fn row_message(
    dev: &Device,
    status: crate::device::DeviceStatus,
    stuck: bool,
) -> MARV_DEVICE_STATUS_DATA {
    device_status(&StatusInput {
        device: dev.id,
        state: status.state,
        counters: status.health,
        stuck,
        samples: status.samples,
        last_sample_age_ms: None,
        hil: false,
        self_test: false,
        time_boot_ms: 0,
    })
}

/// The device's latest sample if its `seq` is not the one last logged.
fn fresh(dev: &Device, last: &mut Option<u32>) -> Option<Sample> {
    let sample = dev.latest.try_get()?;
    if *last == Some(sample.seq) {
        return None;
    }
    *last = Some(sample.seq);
    Some(sample)
}

/// The GPS record of a receiver sample: Status 0 none, 1 no fix, 2 2D, 3 3D (DGPS counts as 3D);
/// week and milliseconds from the UTC date and time, 0/0 while either is unknown.
fn gps(reading: &Reading) -> Option<Gps> {
    let Reading::Gnss(GnssReading {
        fix_type,
        lat_e7,
        lon_e7,
        alt_mm,
        speed_mm_s,
        course_cdeg,
        sats_used,
        hdop_x100,
        time,
        date,
        ..
    }) = *reading
    else {
        return None;
    };
    let (gwk, gms) = match (date, time) {
        (Some(d), Some(t)) => {
            dflog::gps_week_ms(d.year, d.month, d.day, t.hour, t.minute, t.second, t.millis)
        }
        _ => (0, 0),
    };
    Some(Gps {
        inst: 0,
        status: fix_type.min(3),
        gms,
        gwk,
        nsats: sats_used.unwrap_or(0),
        hdop_x100: hdop_x100.unwrap_or(u16::MAX),
        lat_e7: lat_e7.unwrap_or(0),
        lng_e7: lon_e7.unwrap_or(0),
        alt_cm: alt_mm.unwrap_or(0) / 10,
        speed_m_s: speed_mm_s.map_or(0.0, |v| v as f32 / 1000.0),
        course_deg: course_cdeg.map_or(0.0, |v| f32::from(v) / 100.0),
    })
}

/// The same value (a float compared bit for bit).
fn same(a: Option<ParamValue>, b: Option<ParamValue>) -> bool {
    match (a, b) {
        (Some(ParamValue::I32(a)), Some(ParamValue::I32(b))) => a == b,
        (Some(ParamValue::F32(a)), Some(ParamValue::F32(b))) => a.to_bits() == b.to_bits(),
        (None, None) => true,
        _ => false,
    }
}

/// PARM of parameter `i`: its name, value and default (an I32 as `f32`).
fn parm(p: &mut Producer, now: u64, i: u16) {
    let Some((Some(name), Some(value))) = crate::store::with(|s| (s.name(i), s.get(i))) else {
        return;
    };
    let Some(def) = fw_settings::TABLE.get(usize::from(i)) else {
        return;
    };
    let as_f32 = |v: ParamValue| match v {
        ParamValue::I32(v) => v as f32,
        ParamValue::F32(v) => v,
    };
    let (value, default) = (as_f32(value), as_f32(def.default));
    p.push_with(dflog::PARM.length.into(), |b| {
        dflog::encode_parm(b, now, name.as_str().as_bytes(), value, default)
    });
}
