//! The flight step on the board (decision-sim-seams-20261119, section 8; build step 3). Core 1: IMU1's
//! task owns a [`Stepper`] and runs `fw_flight::step::Flight::step` once per good IMU1 sample, right
//! after publishing it, then publishes the result through [`OUT`]. Core 0: [`stat`] only reads [`OUT`].
//! Step 3: the stand-ins give Stop on every step and the ESC task does not read [`OUT`] yet (step 5).

use core::sync::atomic::{AtomicU32, Ordering};

use fw_core::xcore::{Seqlock, SeqlockWriter};
use fw_flight::gate::MotorCommand;
use fw_flight::gnc::{NoAllocator, NoController, NoEstimator, Sensors};
use fw_flight::imu::{ImuSample, SampleSink};
use fw_flight::step::{Flight, FlightInputs};

/// The flight code this image runs; edit this line to change it (seams decision, section 9).
pub type BoardFlight = Flight<NoEstimator, NoController, NoAllocator>;

/// What the flight step publishes after every step.
#[derive(Clone, Copy)]
pub struct FlightOut {
    /// Steps since boot (wrapping).
    pub steps: u32,
    /// The latest step's time, us.
    pub t_us: u64,
    /// The latest step's command.
    pub motors: MotorCommand,
}

/// Written by IMU1's task (core 1) after every step; read by [`stat`] (core 0).
pub static OUT: Seqlock<FlightOut> = Seqlock::new(FlightOut {
    steps: 0,
    t_us: 0,
    motors: MotorCommand::STOP,
});
/// Core 0: the step count at the previous `[STAT] FLT:` line.
static STAT_STEPS: AtomicU32 = AtomicU32::new(0);
/// Copies the core-0 reader attempts before it gives up for this line.
const READ_ATTEMPTS: u32 = 64;

/// IMU1's sample sink: the flight code and the writer of [`OUT`]. Built inside IMU1's task, on core 1.
pub struct Stepper {
    flight: BoardFlight,
    out: SeqlockWriter<'static, FlightOut>,
    steps: u32,
}

impl Stepper {
    /// `None` if the writer of [`OUT`] was taken already.
    pub fn new() -> Option<Self> {
        Some(Self {
            flight: Flight::new(NoEstimator, NoController, NoAllocator),
            out: OUT.take_writer()?,
            steps: 0,
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
        self.steps = self.steps.wrapping_add(1);
        self.out.write(FlightOut {
            steps: self.steps,
            t_us: out.t_us,
            motors: out.motors,
        });
    }
}

/// The `[STAT] FLT:` line (core 0, every 10 s): steps since boot, steps since the previous line,
/// whether the latest command is Stop.
pub fn stat() {
    let Some(o) = OUT.try_read(READ_ATTEMPTS) else {
        defmt::info!("[STAT] FLT: no consistent copy");
        return;
    };
    let last = STAT_STEPS.swap(o.steps, Ordering::Relaxed);
    defmt::info!(
        "[STAT] FLT: {=u32} steps, {=u32} in the last 10 s, latest at {=u64} us, command {=str}",
        o.steps,
        o.steps.wrapping_sub(last),
        o.t_us,
        if o.motors == MotorCommand::STOP {
            "Stop"
        } else {
            "spin"
        }
    );
}
