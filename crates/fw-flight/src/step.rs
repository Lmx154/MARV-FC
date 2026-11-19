//! The flight step: one call per IMU1 sample, on core 1 (decision-sim-seams-20261119, sections 7 and 11).
//! [`Flight::step`] runs the stages in the order of rule R1, each exactly once. No clock: the step time is
//! `sensors.imu1.t_us`.

use crate::gate::{Gate, MotorCommand};
use crate::gnc::{
    Allocation, Allocator, Controller, Estimator, Mode, NavState, Refused, Saturation, Sensors,
    Setpoint, ThrustTorque, Valid,
};
use crate::quat;
use crate::tuning::Tuning;
use fw_core::calib::GRAVITY_MSS;
use fw_core::dflog::{self, FLIGHT_TYPES};
use fw_core::esc::LINES;
use fw_core::handshake::Handshake;
use fw_core::rc::{RcSample, Sticks};
use marv_card::{K_F, MASS, MAX_ROTOR_SPEED};

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

/// Turns (mode, the newest sticks, nav) into a `Setpoint`: Angle mode (decision-first-pair-20261119, section 4).
struct StickShaper {
    /// The newest sticks; before the first report, throttle 0 and centred (4e).
    sticks: Sticks,
    angle_max_rad: f32,
    pilot_y_rate_rads: f32,
    pilot_thrust_max_g: f32,
}

impl StickShaper {
    fn setpoint(&self, mode: Mode, nav: &NavState) -> Setpoint {
        // Angle for every mode: Acro, Altitude and Position are declared only; the mode source comes in step 7.
        match mode {
            Mode::Acro | Mode::Angle | Mode::Altitude | Mode::Position => self.angle(nav),
        }
    }

    /// q_sp = twist(q) (x) q_tilt, the heading rate from yaw, the collective from throttle (4a, 4b).
    fn angle(&self, nav: &NavState) -> Setpoint {
        let Sticks {
            roll,
            pitch,
            throttle,
            yaw,
        } = self.sticks;
        // Tilt vector v = ANGLE_MAX [roll, -pitch, 0], limited to |v| <= ANGLE_MAX, as a rotation about v.
        let (vx, vy) = (self.angle_max_rad * roll, -self.angle_max_rad * pitch);
        let n = libm::sqrtf(vx * vx + vy * vy);
        let half = n.min(self.angle_max_rad) / 2.0;
        let tilt = if n > 0.0 {
            let s = libm::sinf(half) / n;
            [libm::cosf(half), s * vx, s * vy, 0.0]
        } else {
            [1.0, 0.0, 0.0, 0.0]
        };
        // Today's heading: the estimate's twist about down (identity while tilt is not valid, rule R5).
        let [w, _, _, z] = nav.attitude;
        let heading = if nav.valid.tilt {
            quat::normalise([w, 0.0, 0.0, z])
        } else {
            [1.0, 0.0, 0.0, 0.0]
        };
        // Collective, piecewise linear: 0 -> 0, 0.5 -> hover m g, 1 -> min(pilot_thrust_max_g m g, 4 T_hi)
        // (Amendment C: 2.0 m g by default, one straight line; 4 T_hi is four rotors at full speed).
        let hover = MASS * GRAVITY_MSS;
        let full =
            (self.pilot_thrust_max_g * hover).min(4.0 * K_F * MAX_ROTOR_SPEED * MAX_ROTOR_SPEED);
        let thrust_n = if throttle <= 0.5 {
            2.0 * throttle * hover
        } else {
            hover + (2.0 * throttle - 1.0) * (full - hover)
        };
        Setpoint::Attitude {
            attitude: quat::mul(heading, tilt),
            heading_rate_rads: yaw * self.pilot_y_rate_rads,
            thrust_n,
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
}

impl<E: Estimator, C: Controller, A: Allocator> Flight<E, C, A> {
    /// `tuning`: the gains and limits; `handshake`: where the gate stores the armed state for core 0.
    /// The mode is `Mode::Angle` until step 7 brings a mode source.
    pub fn new(
        estimator: E,
        controller: C,
        allocator: A,
        tuning: &Tuning,
        handshake: &'static Handshake,
    ) -> Self {
        Self {
            estimator,
            controller,
            allocator,
            gate: Gate::new(tuning, handshake),
            shaper: StickShaper {
                sticks: Sticks {
                    roll: 0.0,
                    pitch: 0.0,
                    throttle: 0.0,
                    yaw: 0.0,
                },
                angle_max_rad: tuning.angle_max_rad,
                pilot_y_rate_rads: tuning.pilot_y_rate_rads,
                pilot_thrust_max_g: tuning.pilot_thrust_max_g,
            },
            mode: Mode::Angle,
            limited: Saturation {
                roll: false,
                pitch: false,
                yaw: false,
                thrust_low: false,
                thrust_high: false,
            },
        }
    }

    /// One call per IMU1 sample.
    pub fn step(&mut self, inputs: &FlightInputs) -> StepOut {
        let t_us = inputs.sensors.imu1.t_us;
        if let Some(rc) = inputs.rc {
            self.shaper.sticks = rc.sticks;
        }
        // The armed state the previous step left, then R1.1 to R1.7.
        self.estimator.set_armed(self.gate.armed());
        // A non-finite IMU value never reaches the estimator: no tilt, so the demand is refused and the gate stops.
        let reading = &inputs.sensors.imu1.reading;
        let finite = reading.gyro_rads.iter().all(|v| v.is_finite())
            && reading.accel_ms2.iter().all(|v| v.is_finite());
        let nav = if finite {
            self.estimator.update(&inputs.sensors)
        } else {
            NavState {
                t_us,
                attitude: [1.0, 0.0, 0.0, 0.0],
                rates_rads: [0.0; 3],
                velocity_ms: [0.0; 3],
                position_m: [0.0; 3],
                valid: Valid::NONE,
                sigma: None,
            }
        };
        if !self.gate.armed() {
            self.controller.reset();
        }
        let setpoint = self.shaper.setpoint(self.mode, &nav);
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

/// Writes the four flight records of one step (NAV, SETP, DMD, MOT; decision-first-pair-wiring-20261119,
/// A6) at the start of `buf`. Returns the bytes written, or 0 (nothing written) if `buf` is too small.
pub fn encode_step(out: &StepOut, buf: &mut [u8]) -> usize {
    let need: usize = FLIGHT_TYPES.iter().map(|d| usize::from(d.length)).sum();
    if buf.len() < need {
        return 0;
    }
    let t = out.t_us;
    let Valid {
        tilt,
        heading,
        rates,
        velocity_ne,
        velocity_d,
        position_ne,
        position_d,
    } = out.nav.valid;
    let valid = u8::from(tilt)
        | u8::from(heading) << 1
        | u8::from(rates) << 2
        | u8::from(velocity_ne) << 3
        | u8::from(velocity_d) << 4
        | u8::from(position_ne) << 5
        | u8::from(position_d) << 6;
    let (kind, attitude, sp_rates, thrust) = match out.setpoint {
        Setpoint::Rates {
            rates_rads,
            thrust_n,
        } => (0, [1.0, 0.0, 0.0, 0.0], rates_rads, thrust_n),
        Setpoint::Attitude {
            attitude,
            heading_rate_rads,
            thrust_n,
        } => (1, attitude, [0.0, 0.0, heading_rate_rads], thrust_n),
        Setpoint::Altitude { .. } => (2, [0.0; 4], [0.0; 3], 0.0),
        Setpoint::Position { .. } => (3, [0.0; 4], [0.0; 3], 0.0),
        Setpoint::Waypoint { .. } => (4, [0.0; 4], [0.0; 3], 0.0),
    };
    let demand = out.demand.unwrap_or(ThrustTorque {
        thrust_n: 0.0,
        torque_nm: [0.0; 3],
    });
    let saturation = out.allocation.map_or(0, |a| {
        let s = a.saturation;
        u8::from(s.roll)
            | u8::from(s.pitch) << 1
            | u8::from(s.yaw) << 2
            | u8::from(s.thrust_low) << 3
            | u8::from(s.thrust_high) << 4
    });
    let mode = match out.mode {
        Mode::Acro => 0,
        Mode::Angle => 1,
        Mode::Altitude => 2,
        Mode::Position => 3,
    };
    let levels = out.allocation.map_or([0; LINES], |a| a.levels.level);
    let command = out
        .motors
        .throttles()
        .map_or([0; LINES], |th| th.map(|x| x.level()));
    let mut n = 0;
    n += dflog::encode_nav(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        out.nav.attitude,
        out.nav.rates_rads,
        valid,
    );
    n += dflog::encode_setp(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        kind,
        attitude,
        sp_rates,
        thrust,
    );
    n += dflog::encode_dmd(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        out.demand.is_ok(),
        demand.thrust_n,
        demand.torque_nm,
        saturation,
    );
    n += dflog::encode_mot(
        buf.get_mut(n..).unwrap_or_default(),
        t,
        out.armed,
        mode,
        levels,
        command,
    );
    n
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::alloc::CardAllocator;
    use crate::gnc::MotorLevels;
    use crate::imu::{ImuReading, ImuSample};
    use crate::mahony::Mahony;
    use crate::pid::CascadedPid;
    use core::cell::Cell;

    static HANDSHAKE: Handshake = Handshake::new();

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

    /// Amendment C: the stick-to-collective map, 0 -> 0, 0.5 -> m g, 1 -> min(cap m g, 4 K_F w_max^2).
    #[test]
    fn collective_follows_the_capped_map() {
        let hover = MASS * GRAVITY_MSS;
        let physical = 4.0 * K_F * MAX_ROTOR_SPEED * MAX_ROTOR_SPEED;
        let nav = NavState {
            t_us: 0,
            attitude: [1.0, 0.0, 0.0, 0.0],
            rates_rads: [0.0; 3],
            velocity_ms: [0.0; 3],
            position_m: [0.0; 3],
            valid: Valid {
                tilt: true,
                ..Valid::NONE
            },
            sigma: None,
        };
        let thrust = |cap: f32, throttle: f32| {
            let mut tuning = Tuning::DEFAULT;
            tuning.pilot_thrust_max_g = cap;
            let mut flight = Flight::new(
                Mahony::new(&tuning),
                CascadedPid::new(&tuning),
                CardAllocator::new(),
                &tuning,
                &HANDSHAKE,
            );
            flight.shaper.sticks.throttle = throttle;
            match flight.shaper.angle(&nav) {
                Setpoint::Attitude { thrust_n, .. } => thrust_n,
                _ => panic!("not an attitude setpoint"),
            }
        };
        let near = |a: f32, b: f32| assert!((a - b).abs() <= 1e-4 * b.max(1.0), "{a} vs {b}");
        near(thrust(2.0, 0.0), 0.0);
        near(thrust(2.0, 0.25), 0.5 * hover);
        near(thrust(2.0, 0.5), hover);
        near(thrust(2.0, 0.75), 1.5 * hover);
        near(thrust(2.0, 1.0), 2.0 * hover);
        near(thrust(100.0, 1.0), physical);
    }

    #[test]
    fn board_types_give_stop_without_rc() {
        let tuning = Tuning::DEFAULT;
        let mut flight = Flight::new(
            Mahony::new(&tuning),
            CascadedPid::new(&tuning),
            CardAllocator::new(),
            &tuning,
            &HANDSHAKE,
        );
        let mut out = flight.step(&inputs(1234));
        assert_eq!(out.motors, MotorCommand::STOP);
        assert_eq!(out.motors.throttles(), None);
        assert_eq!(out.demand, Err(Refused));
        assert_eq!(out.allocation, None);
        assert_eq!(out.t_us, 1234);
        assert_eq!(out.nav.t_us, 1234);
        assert!(!out.armed);

        // I5: a NaN gyro sample while armed gives Stop and disarm without a call to the estimator; the next good
        // sample gives a finite nav.
        let level = |t_us: u64, gyro: [f32; 3]| {
            let mut i = inputs(t_us);
            i.sensors.imu1.reading.accel_ms2 = [0.0, 0.0, -9.81];
            i.sensors.imu1.reading.gyro_rads = gyro;
            i
        };
        let report = |t_us: u64, arm: bool| RcSample {
            sticks: Sticks {
                roll: 0.0,
                pitch: 0.0,
                throttle: 0.0,
                yaw: 0.0,
            },
            arm,
            t_us,
        };
        let mut t = 2_000;
        for arm in [false, true] {
            t += 1_000;
            let mut i = level(t, [0.0; 3]);
            i.rc = Some(report(t, arm));
            out = flight.step(&i);
        }
        assert!(out.armed && HANDSHAKE.armed());
        assert!(out.motors.throttles().is_some());
        t += 1_000;
        let out = flight.step(&level(t, [0.0, f32::NAN, 0.0]));
        assert_eq!(out.motors, MotorCommand::STOP);
        assert!(!out.armed && !HANDSHAKE.armed());
        assert_eq!(out.nav.valid, Valid::NONE);
        assert_eq!(out.demand, Err(Refused));
        t += 1_000;
        let out = flight.step(&level(t, [0.0; 3]));
        assert!(out.nav.valid.tilt);
        assert!(
            out.nav
                .attitude
                .iter()
                .chain(&out.nav.rates_rads)
                .all(|v| v.is_finite())
        );
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
            NavState {
                t_us: sensors.imu1.t_us,
                attitude: [1.0, 0.0, 0.0, 0.0],
                rates_rads: [0.0; 3],
                velocity_ms: [0.0; 3],
                position_m: [0.0; 3],
                valid: self.valid,
                sigma: None,
            }
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
            Allocation {
                levels: MotorLevels { level: [100; 4] },
                saturation: Saturation {
                    roll: false,
                    pitch: false,
                    yaw: false,
                    thrust_low: false,
                    thrust_high: false,
                },
            }
        }
    }

    #[test]
    fn stages_run_in_r1_order() {
        let attitude = Valid {
            tilt: true,
            rates: true,
            ..Valid::NONE
        };
        // (valid, controller accepts, calls, allocation made)
        let cases = [
            (Valid::NONE, true, 12, false), // covers fails: no controller, no allocator
            (attitude, false, 125, false),  // the controller refuses: no allocator
            (attitude, true, 1256, true),   // the full order
        ];
        for (valid, accept, calls, allocated) in cases {
            let log = Cell::new(0);
            let rec = Rec {
                log: &log,
                valid,
                accept,
            };
            let mut flight = Flight::new(rec, rec, rec, &Tuning::DEFAULT, &HANDSHAKE);
            let out = flight.step(&inputs(10));
            assert_eq!(log.get(), calls);
            assert_eq!(out.allocation.is_some(), allocated);
            assert_eq!(out.demand.is_ok(), allocated);
            // The gate comes last and has no stick report: Stop whatever the allocator said.
            assert_eq!(out.motors, MotorCommand::STOP);
        }
    }

    /// A6: the four records of one step, decoded by hand, field by field.
    #[test]
    fn encode_step_writes_the_four_records() {
        extern crate std;
        let out = StepOut {
            t_us: 0x0102_0304_0506,
            motors: MotorCommand::spinning([111, 222, 333, 444]),
            armed: true,
            mode: Mode::Angle,
            nav: NavState {
                t_us: 0x0102_0304_0506,
                attitude: [0.5, -0.5, 0.25, 0.75],
                rates_rads: [0.1, -0.2, 0.3],
                velocity_ms: [9.0; 3],
                position_m: [9.0; 3],
                valid: Valid {
                    tilt: true,
                    rates: true,
                    position_d: true,
                    ..Valid::NONE
                },
                sigma: None,
            },
            setpoint: Setpoint::Attitude {
                attitude: [0.9, 0.1, -0.2, 0.3],
                heading_rate_rads: 1.5,
                thrust_n: 7.25,
            },
            demand: Ok(ThrustTorque {
                thrust_n: 7.5,
                torque_nm: [0.01, -0.02, 0.03],
            }),
            allocation: Some(Allocation {
                levels: MotorLevels {
                    level: [120, 130, 140, 150],
                },
                saturation: Saturation {
                    roll: false,
                    pitch: true,
                    yaw: true,
                    thrust_low: false,
                    thrust_high: true,
                },
            }),
        };
        let mut buf = [0u8; 200];
        assert_eq!(encode_step(&out, &mut buf), 40 + 44 + 29 + 29);
        assert_eq!(encode_step(&out, &mut [0u8; 141]), 0);
        let f = |at: usize| f32::from_le_bytes(buf[at..at + 4].try_into().unwrap());
        let h = |at: usize| u16::from_le_bytes(buf[at..at + 2].try_into().unwrap());
        let fs = |at: usize, n: usize| (0..n).map(|i| f(at + 4 * i)).collect::<std::vec::Vec<_>>();
        let time = 0x0102_0304_0506u64.to_le_bytes();
        // NAV at 0: header, time, Q1..Q4, GX..GZ, Vld (tilt 1 + rates 4 + position_d 64).
        assert_eq!(buf[0..3], [0xA3, 0x95, 139]);
        assert_eq!(buf[3..11], time);
        assert_eq!(fs(11, 4), out.nav.attitude);
        assert_eq!(fs(27, 3), out.nav.rates_rads);
        assert_eq!(buf[39], 1 | 4 | 64);
        // SETP at 40: K 1, Q, RX RY 0, RZ the heading rate, Thr.
        assert_eq!(buf[40..43], [0xA3, 0x95, 140]);
        assert_eq!(buf[43..51], time);
        assert_eq!(buf[51], 1);
        assert_eq!(fs(52, 4), [0.9, 0.1, -0.2, 0.3]);
        assert_eq!(fs(68, 3), [0.0, 0.0, 1.5]);
        assert_eq!(f(80), 7.25);
        // DMD at 84: Ok, Thr, TX..TZ, Sat (pitch 2 + yaw 4 + thrust_high 16).
        assert_eq!(buf[84..87], [0xA3, 0x95, 141]);
        assert_eq!(buf[87..95], time);
        assert_eq!(buf[95], 1);
        assert_eq!(fs(96, 4), [7.5, 0.01, -0.02, 0.03]);
        assert_eq!(buf[112], 2 | 4 | 16);
        // MOT at 113: Arm, Mode (Angle 1), A0..A3, C0..C3.
        assert_eq!(buf[113..116], [0xA3, 0x95, 142]);
        assert_eq!(buf[116..124], time);
        assert_eq!(buf[124..126], [1, 1]);
        let levels: std::vec::Vec<u16> = (0..8).map(|i| h(126 + 2 * i)).collect();
        assert_eq!(levels, [120, 130, 140, 150, 111, 222, 333, 444]);
        assert!(buf[142..].iter().all(|&b| b == 0));
    }
}
