//! The flight step on the board (decision-sim-seams-20261119, section 8; decision-first-pair-wiring-20261119, A1-A3).
//! Core 1 only: IMU1's task owns a [`Stepper`] and runs `fw_flight::step::Flight::step` once per good IMU1 sample,
//! right after publishing it, with the newest CRSF stick report from `crate::rc::FLIGHT_RC` if one arrived since the
//! previous step, and the newest barometer, compass and GNSS samples from [`FLIGHT_BARO`], [`FLIGHT_MAG`] and
//! [`FLIGHT_GNSS`] likewise, then publishes the result through [`OUT`], which the ESC task sends to the motors, and then
//! pushes the step's log records into [`LOG`] (decision-flight-logging-20261119, section 3). On core 0 `logger`'s
//! collector touches this module as [`LOG`]'s consumer, and the barometer, compass and GNSS device tasks as the writers
//! of the three sample cells (decision-nav-eskf-20261120, section 4); core 0 learns "armed" from
//! `settings::HANDSHAKE` and "flight throttles sent" from `esc::ESC`.
//! The step runs Mahony, the cascaded PID and the card's allocator with the constant gains of `fw_flight::tuning`;
//! Mahony ignores the barometer, compass and GNSS samples.
//! No status line: a `[STAT] FLT:` line from core 0 was tried on 2026-11-19 and removed, because that one
//! extra log line lengthened the flight core's longest wait behind the log (README, Flight-core device loop).

use fw_core::dflog;
use fw_core::rc::RcSample;
use fw_core::sample::{BaroReading, BaroSample, GnssReading, GnssSample, MagReading, MagSample};
use fw_core::xcore::{LogProducer, LogRing, Seqlock, SeqlockWriter, TripleBuffer, TripleReader};
use fw_flight::alloc::CardAllocator;
use fw_flight::esc::log_fields;
use fw_flight::gate::MotorCommand;
use fw_flight::gnc::Sensors;
use fw_flight::imu::{ImuSample, SampleSink};
use fw_flight::mahony::Mahony;
use fw_flight::pid::CascadedPid;
use fw_flight::step::{Flight, FlightInputs, encode_step, log_due};
use fw_flight::tuning::Tuning;

/// The flight code this image runs; edit this line to change it (seams decision, section 9).
pub type BoardFlight = Flight<Mahony, CascadedPid, CardAllocator>;

/// What the flight step publishes after every step.
#[derive(Clone, Copy)]
pub struct FlightOut {
    /// The latest step's time, us.
    pub t_us: u64,
    /// The latest step's command.
    pub motors: MotorCommand,
}

/// Written by IMU1's task (core 1) after every step; read by the ESC task (core 1) at every exchange.
pub static OUT: Seqlock<FlightOut> = Seqlock::new(FlightOut {
    t_us: 0,
    motors: MotorCommand::STOP,
});

/// Core 0 -> flight core: the barometer's newest sample with its core-0 time (one writer, the barometer's
/// adapter as it publishes to the row; one reader, IMU1's [`Stepper`]).
pub static FLIGHT_BARO: TripleBuffer<BaroSample> = TripleBuffer::new(BaroSample {
    reading: BaroReading {
        pressure_pa: 0.0,
        temperature_c: 0.0,
    },
    t_us: 0,
    seq: 0,
});

/// Core 0 -> flight core: the compass's newest sample, the calibrated field (one writer, the compass's adapter as
/// it publishes to the row; one reader, IMU1's [`Stepper`]).
pub static FLIGHT_MAG: TripleBuffer<MagSample> = TripleBuffer::new(MagSample {
    reading: MagReading {
        field_ut: [0.0; 3],
        temperature_c: 0.0,
    },
    t_us: 0,
    seq: 0,
    expected: None,
});

/// Core 0 -> flight core: the receiver's newest epoch (one writer, the GNSS adapter as it publishes to the row;
/// one reader, IMU1's [`Stepper`]).
pub static FLIGHT_GNSS: TripleBuffer<GnssSample> = TripleBuffer::new(GnssSample {
    reading: GnssReading {
        fix_type: 1,
        lat_e7: None,
        lon_e7: None,
        alt_mm: None,
        speed_mm_s: None,
        course_cdeg: None,
        sats_used: None,
        hdop_x100: None,
        vdop_x100: None,
        time: None,
        date: None,
        lat_std_mm: None,
        lon_std_mm: None,
        alt_std_mm: None,
        vel_ned_mm_s: None,
    },
    t_us: 0,
    seq: 0,
});

/// The flight log ring's size (decision-flight-logging-20261119, section 3).
pub const LOG_BYTES: usize = 16384;
/// The flight records: producer IMU1's [`Stepper`] (core 1), consumer `logger`'s collector (core 0).
pub static LOG: LogRing<LOG_BYTES> = LogRing::new();

/// One step block: IMU1's sample, then NAV, SETP, DMD, MOT (182 bytes).
const BLOCK: usize = dflog::IMU.length as usize
    + dflog::NAV.length as usize
    + dflog::SETP.length as usize
    + dflog::DMD.length as usize
    + dflog::MOT.length as usize;

/// IMU1's sample sink: the flight code and the writer of [`OUT`]. Built inside IMU1's task, on core 1.
pub struct Stepper {
    flight: BoardFlight,
    out: SeqlockWriter<'static, FlightOut>,
    rc: TripleReader<'static, RcSample>,
    baro: TripleReader<'static, BaroSample>,
    mag: TripleReader<'static, MagSample>,
    gnss: TripleReader<'static, GnssSample>,
    log: LogProducer<'static, LOG_BYTES>,
    /// Steps since the start, for `log_due`.
    step: u32,
    /// `TimeUS` of the previous ESC record.
    esc_since_us: u64,
}

impl Stepper {
    /// `None` if the writer of [`OUT`], the producer of [`LOG`], the reader of `crate::rc::FLIGHT_RC` (`rc`) or the
    /// reader of [`FLIGHT_BARO`], [`FLIGHT_MAG`] or [`FLIGHT_GNSS`] was taken already.
    pub fn new(rc: Option<TripleReader<'static, RcSample>>) -> Option<Self> {
        Some(Self {
            flight: Flight::new(
                Mahony::new(&Tuning::DEFAULT),
                CascadedPid::new(&Tuning::DEFAULT),
                CardAllocator::new(),
                &Tuning::DEFAULT,
                &crate::settings::HANDSHAKE,
            ),
            out: OUT.take_writer()?,
            rc: rc?,
            baro: FLIGHT_BARO.take_reader()?,
            mag: FLIGHT_MAG.take_reader()?,
            gnss: FLIGHT_GNSS.take_reader()?,
            log: LOG.take_producer()?,
            step: 0,
            esc_since_us: 0,
        })
    }
}

impl SampleSink for Stepper {
    /// One flight step for the IMU1 sample just published, with the stick report and the barometer, compass and
    /// GNSS samples that are new since the previous step. IMU2 comes with later steps.
    fn sample(&mut self, sample: &ImuSample) {
        let (report, fresh) = self.rc.read();
        let (baro, baro_fresh) = self.baro.read();
        let (mag, mag_fresh) = self.mag.read();
        let (gnss, gnss_fresh) = self.gnss.read();
        let out = self.flight.step(&FlightInputs {
            sensors: Sensors {
                imu1: *sample,
                imu2: None,
                baro: baro_fresh.then_some(baro),
                mag: mag_fresh.then_some(mag),
                gnss: gnss_fresh.then_some(gnss),
                external: None,
            },
            rc: fresh.then_some(report),
        });
        self.out.write(FlightOut {
            t_us: out.t_us,
            motors: out.motors,
        });
        // The log, after the command is out. A record that does not fit is dropped and counted by the ring.
        let due = log_due(self.step, out.armed);
        self.step = self.step.wrapping_add(1);
        if due.block {
            let r = &sample.reading;
            self.log.push_with(BLOCK, |b| {
                let n =
                    dflog::encode_imu(b, sample.t_us, 0, r.gyro_rads, r.accel_ms2, r.temperature_c);
                n + encode_step(&out, b.get_mut(n..).unwrap_or_default())
            });
        }
        if due.esc
            && let Some(status) = crate::esc::ESC.try_read(1)
            && status.exchanges > 0
        {
            let f = log_fields(&status, self.esc_since_us);
            let t = status.last_exchange_us;
            self.esc_since_us = t;
            self.log.push_with(dflog::ESC.length.into(), |b| {
                dflog::encode_esc(b, t, f.erpm, f.volts_cv, f.temp_c, f.flags)
            });
        }
        if fresh {
            let s = report.sticks;
            self.log.push_with(dflog::RCS.length.into(), |b| {
                dflog::encode_rcs(
                    b,
                    report.t_us,
                    [s.roll, s.pitch, s.throttle, s.yaw],
                    report.arm,
                )
            });
        }
    }
}
