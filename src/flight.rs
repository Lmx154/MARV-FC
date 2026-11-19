//! The flight step on the board (decision-sim-seams-20261119, section 8; build step 3). Core 1 only: IMU1's
//! task owns a [`Stepper`] and runs `fw_flight::step::Flight::step` once per good IMU1 sample, right
//! after publishing it, then publishes the result through [`OUT`]. Nothing on core 0 touches this module.
//! Step 5: the step runs Mahony, the cascaded PID and the card's allocator, but with no stick report (`rc: None`) the
//! gate never arms, and the ESC task does not read [`OUT`] yet.
//! No status line: a `[STAT] FLT:` line from core 0 was tried on 2026-11-19 and removed, because that one
//! extra log line lengthened the flight core's longest wait behind the log (README, Flight-core device loop).

use fw_core::xcore::{Seqlock, SeqlockWriter};
use fw_flight::alloc::CardAllocator;
use fw_flight::gate::MotorCommand;
use fw_flight::gnc::Sensors;
use fw_flight::imu::{ImuSample, SampleSink};
use fw_flight::mahony::Mahony;
use fw_flight::pid::CascadedPid;
use fw_flight::step::{Flight, FlightInputs};
use fw_flight::tuning::Tuning;

/// The flight code this image runs; edit this line to change it (seams decision, section 9).
pub type BoardFlight = Flight<Mahony, CascadedPid, CardAllocator>;

/// What the flight step publishes after every step.
#[derive(Clone, Copy)]
#[allow(dead_code)] // no reader yet: the ESC task reads both fields from step 5 on
pub struct FlightOut {
    /// The latest step's time, us.
    pub t_us: u64,
    /// The latest step's command.
    pub motors: MotorCommand,
}

/// Written by IMU1's task (core 1) after every step. Its reader, the ESC task (core 1), comes in step 5.
pub static OUT: Seqlock<FlightOut> = Seqlock::new(FlightOut {
    t_us: 0,
    motors: MotorCommand::STOP,
});

/// IMU1's sample sink: the flight code and the writer of [`OUT`]. Built inside IMU1's task, on core 1.
pub struct Stepper {
    flight: BoardFlight,
    out: SeqlockWriter<'static, FlightOut>,
}

impl Stepper {
    /// `None` if the writer of [`OUT`] was taken already.
    pub fn new() -> Option<Self> {
        Some(Self {
            flight: Flight::new(
                Mahony::new(&Tuning::DEFAULT),
                CascadedPid::new(&Tuning::DEFAULT),
                CardAllocator::new(),
                &Tuning::DEFAULT,
                &crate::settings::HANDSHAKE,
            ),
            out: OUT.take_writer()?,
        })
    }
}

impl SampleSink for Stepper {
    /// One flight step for the IMU1 sample just published. IMU2, the slow sensors and RC come with
    /// later steps.
    fn sample(&mut self, sample: &ImuSample) {
        let out = self.flight.step(&FlightInputs {
            sensors: Sensors {
                imu1: *sample,
                imu2: None,
                baro: None,
                mag: None,
                gnss: None,
                external: None,
            },
            rc: None,
        });
        self.out.write(FlightOut {
            t_us: out.t_us,
            motors: out.motors,
        });
    }
}
