//! The flight step: one call per IMU1 sample, on core 1 (decision-sim-seams-20261119, sections 7 and 11).
//! [`Flight::step`] runs the stages in the order of rule R1, each exactly once. No clock: the step time is
//! `sensors.imu1.t_us`.

use crate::gate::{Gate, MotorCommand};
use crate::gnc::{
    Allocation, Allocator, Controller, Estimator, Mode, NavState, Refused, Saturation, Sensors,
    Setpoint, ThrustTorque,
};
use fw_core::rc::{RcSample, Sticks};

/// One step's inputs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlightInputs {
    /// What is new since the previous step.
    pub sensors: Sensors,
    /// This step's new stick report, if one arrived.
    pub rc: Option<RcSample>,
}

/// Everything one step decided, by value: the ESC command plus what the log records (firmware change 5).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StepOut {
    /// The step time, `sensors.imu1.t_us`.
    pub t_us: u64,
    /// The gate's command.
    pub motors: MotorCommand,
    /// The gate's state after this step.
    pub armed: bool,
    /// The selected mode.
    pub mode: Mode,
    /// The estimator's state.
    pub nav: NavState,
    /// The stick shaper's setpoint.
    pub setpoint: Setpoint,
    /// The controller's demand, or `Err` when it refused or was not called.
    pub demand: Result<ThrustTorque, Refused>,
    /// `None` when the demand was refused.
    pub allocation: Option<Allocation>,
}

/// Turns (mode, held sticks, nav) into a `Setpoint`. Step-3 stand-in, replaced in step 5 (stick scaling): a zero
/// `Setpoint::Rates`.
struct StickShaper;

impl StickShaper {
    fn setpoint(&self, _mode: Mode, _sticks: Option<&Sticks>, _nav: &NavState) -> Setpoint {
        Setpoint::Rates {
            rates_rads: [0.0; 3],
            thrust_n: 0.0,
        }
    }
}

/// The flight code: estimator, controller, allocator, gate. Only [`Flight::step`] reaches them.
pub struct Flight<E: Estimator, C: Controller, A: Allocator> {
    estimator: E,
    controller: C,
    allocator: A,
    gate: Gate,
    shaper: StickShaper,
    mode: Mode,
    limited: Saturation,
    sticks: Option<Sticks>,
}

impl<E: Estimator, C: Controller, A: Allocator> Flight<E, C, A> {
    /// Further arguments (stick scaling, gate settings) come with step 5.
    /// Step 3: the mode is `Mode::Angle`, replaced in step 5.
    pub fn new(estimator: E, controller: C, allocator: A) -> Self {
        Self {
            estimator,
            controller,
            allocator,
            gate: Gate {},
            shaper: StickShaper,
            mode: Mode::Angle,
            limited: Saturation {
                roll: false,
                pitch: false,
                yaw: false,
                thrust_low: false,
                thrust_high: false,
            },
            sticks: None,
        }
    }

    /// One call per IMU1 sample.
    pub fn step(&mut self, inputs: &FlightInputs) -> StepOut {
        let t_us = inputs.sensors.imu1.t_us;
        if let Some(rc) = inputs.rc {
            self.sticks = Some(rc.sticks);
        }
        // R1.1 to R1.7.
        let nav = self.estimator.update(&inputs.sensors);
        if !self.gate.armed() {
            self.controller.reset();
        }
        let setpoint = self.shaper.setpoint(self.mode, self.sticks.as_ref(), &nav);
        let demand = if nav.valid.covers(&setpoint.requires()) {
            self.controller.update(&nav, &setpoint, &self.limited)
        } else {
            Err(Refused)
        };
        let allocation = demand.ok().map(|d| self.allocator.allocate(&d));
        if let Some(a) = allocation {
            self.limited = a.saturation;
        }
        let levels = allocation.as_ref().map(|a| &a.levels);
        let motors = self.gate.apply(t_us, inputs.rc.as_ref(), &nav, levels);
        StepOut {
            t_us,
            motors,
            armed: self.gate.armed(),
            mode: self.mode,
            nav,
            setpoint,
            demand,
            allocation,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnc::{MotorLevels, NoAllocator, NoController, NoEstimator, Valid};
    use crate::imu::{ImuReading, ImuSample};
    use core::cell::Cell;

    fn inputs(t_us: u64) -> FlightInputs {
        FlightInputs {
            sensors: Sensors {
                imu1: ImuSample {
                    reading: ImuReading::default(),
                    t_us,
                    seq: 7,
                },
                imu2: None,
                baro: None,
                mag: None,
                gnss: None,
                external: None,
            },
            rc: None,
        }
    }

    #[test]
    fn stand_ins_give_stop() {
        let mut flight = Flight::new(NoEstimator, NoController, NoAllocator);
        let out = flight.step(&inputs(1234));
        assert_eq!(out.motors, MotorCommand::STOP);
        assert_eq!(out.motors.throttles(), None);
        assert_eq!(out.demand, Err(Refused));
        assert_eq!(out.allocation, None);
        assert_eq!(out.t_us, 1234);
        assert_eq!(out.nav.t_us, 1234);
        assert!(!out.armed);
    }

    /// Estimator, controller and allocator in one: each call appends its digit to `log`
    /// (1 estimate, 2 reset, 5 control, 6 allocate).
    #[derive(Clone, Copy)]
    struct Rec<'a> {
        log: &'a Cell<u32>,
        valid: Valid,
        accept: bool,
    }

    impl Rec<'_> {
        fn note(&self, digit: u32) {
            self.log.set(self.log.get() * 10 + digit);
        }
    }

    impl Estimator for Rec<'_> {
        fn update(&mut self, sensors: &Sensors) -> NavState {
            self.note(1);
            let mut nav = NoEstimator.update(sensors);
            nav.valid = self.valid;
            nav
        }
    }

    impl Controller for Rec<'_> {
        fn update(
            &mut self,
            _: &NavState,
            _: &Setpoint,
            _: &Saturation,
        ) -> Result<ThrustTorque, Refused> {
            self.note(5);
            if self.accept {
                Ok(ThrustTorque {
                    thrust_n: 1.0,
                    torque_nm: [0.0; 3],
                })
            } else {
                Err(Refused)
            }
        }

        fn reset(&mut self) {
            self.note(2);
        }
    }

    impl Allocator for Rec<'_> {
        fn allocate(&mut self, _: &ThrustTorque) -> Allocation {
            self.note(6);
            let mut a = NoAllocator.allocate(&ThrustTorque {
                thrust_n: 0.0,
                torque_nm: [0.0; 3],
            });
            a.levels = MotorLevels { level: [100; 4] };
            a
        }
    }

    #[test]
    fn stages_run_in_r1_order() {
        let rates = Valid {
            rates: true,
            ..Valid::NONE
        };
        // (valid, controller accepts, calls, allocation made)
        let cases = [
            (Valid::NONE, true, 12, false), // covers fails: no controller, no allocator
            (rates, false, 125, false),     // the controller refuses: no allocator
            (rates, true, 1256, true),      // the full order
        ];
        for (valid, accept, calls, allocated) in cases {
            let log = Cell::new(0);
            let rec = Rec {
                log: &log,
                valid,
                accept,
            };
            let mut flight = Flight::new(rec, rec, rec);
            let out = flight.step(&inputs(10));
            assert_eq!(log.get(), calls);
            assert_eq!(out.allocation.is_some(), allocated);
            assert_eq!(out.demand.is_ok(), allocated);
            // The gate comes last and is not armed in step 3: Stop whatever the allocator said.
            assert_eq!(out.motors, MotorCommand::STOP);
        }
    }
}
