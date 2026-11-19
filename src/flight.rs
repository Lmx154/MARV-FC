//! The flight step on the board (decision-sim-seams-20261119, section 8; decision-first-pair-wiring-20261119, A1-A3).
//! Core 1 only: IMU1's task owns a [`Stepper`] and runs `fw_flight::step::Flight::step` once per good IMU1 sample,
//! right after publishing it, with the newest CRSF stick report from `crate::rc::FLIGHT_RC` if one arrived since the
//! previous step, then publishes the result through [`OUT`], which the ESC task sends to the motors, and then pushes
//! the step's log records into [`LOG`] (decision-flight-logging-20261119, section 3). On core 0 only `logger`'s
//! collector touches this module, as [`LOG`]'s consumer; core 0 learns "armed" from `settings::HANDSHAKE` and "flight
//! throttles sent" from `esc::ESC`.
//! The step runs Mahony, the cascaded PID and the card's allocator with the constant gains of `fw_flight::tuning`.
//! No status line: a `[STAT] FLT:` line from core 0 was tried on 2026-11-19 and removed, because that one
//! extra log line lengthened the flight core's longest wait behind the log (README, Flight-core device loop).

use fw_core::dflog;
use fw_core::rc::RcSample;
use fw_core::xcore::{LogProducer, LogRing, Seqlock, SeqlockWriter, TripleReader};
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
    log: LogProducer<'static, LOG_BYTES>,
    /// Steps since the start, for `log_due`.
    step: u32,
    /// `TimeUS` of the previous ESC record.
    esc_since_us: u64,
}

impl Stepper {
    /// `None` if the writer of [`OUT`], the producer of [`LOG`] or the reader of `crate::rc::FLIGHT_RC` (`rc`) was
    /// taken already.
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
            log: LOG.take_producer()?,
            step: 0,
            esc_since_us: 0,
        })
    }
}

impl SampleSink for Stepper {
    /// One flight step for the IMU1 sample just published, with the stick report if it is new since the
    /// previous step. IMU2 and the slow sensors come with later steps.
    fn sample(&mut self, sample: &ImuSample) {
        let (report, fresh) = self.rc.read();
        let out = self.flight.step(&FlightInputs {
            sensors: Sensors {
                imu1: *sample,
                imu2: None,
                baro: None,
                mag: None,
                gnss: None,
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
