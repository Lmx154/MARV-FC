//! The flight step: one call per IMU1 sample, on core 1 (decision-sim-seams-20261119, sections 7 and 11).
//! [`Flight::step`] runs the stages in the order of rule R1, each exactly once. No clock: the step time is
//! `sensors.imu1.t_us`.

use crate::gate::{Gate, MotorCommand};
use crate::gnc::{
    Allocation, Allocator, Controller, Estimator, Mode, NavState, Refused, Saturation, Sensors,
    Setpoint, ThrustTorque, Valid,
};
use crate::mahony::DT_MAX_US;
use crate::quat;
use crate::tuning::{Tuning, full_thrust_n};
use fw_core::calib::GRAVITY_MSS;
use fw_core::dflog::{self, FLIGHT_TYPES};
use fw_core::esc::LINES;
use fw_core::handshake::Handshake;
use fw_core::rc::{RcSample, Sticks, Switch3};
use marv_card::MASS;
#[cfg(test)]
use marv_card::{K_F, MAX_ROTOR_SPEED};

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
    /// The mode the pilot selected with the switch on channel 6 (Angle until the first stick report).
    pub selected: Mode,
    /// The mode in effect this step: Altitude if `selected` is Altitude or Position and the estimate covers what
    /// `Setpoint::Altitude` needs, otherwise Angle (decision-nav-eskf-20261120, 5e, F1). MOT's `Mode`.
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

/// How far ahead of the vehicle the altitude target may run, in seconds of the limit speed of its direction: the target
/// stops integrating while the vehicle is that far from it (Betaflight `alt_hold_multirotor.c:146`, 1 s, the target
/// velocity is still passed on at :141; Amendment F, F2; PROPOSED to the owner).
pub const ALT_LEAD_S: f32 = 1.0;

/// The throttle stick's centre in altitude hold: the dead zone is this plus and minus `pilot_thr_dz`
/// (ArduPilot `Attitude.cpp:97-112`, the centre fixed at half travel).
const THROTTLE_CENTRE: f32 = 0.5;

/// Turns (mode, the newest sticks, nav) into a `Setpoint`: Angle mode (decision-first-pair-20261119, section 4) and
/// altitude hold (decision-nav-eskf-20261120, F1, F2).
struct StickShaper {
    /// The newest sticks; before the first report, throttle 0 and centred (4e).
    sticks: Sticks,
    angle_max_rad: f32,
    pilot_y_rate_rads: f32,
    pilot_thrust_max_g: f32,
    pilot_speed_up_ms: f32,
    pilot_speed_dn_ms: f32,
    pilot_thr_dz: f32,
    /// The altitude target, m down (positive down); `None` outside Altitude. Captured at the first step whose
    /// effective mode is Altitude.
    target_d: Option<f32>,
    /// The previous step's time, us; `None` before the first.
    t_prev_us: Option<u64>,
    /// Tests only: the lead cap is off.
    #[cfg(test)]
    lead_cap_off: bool,
}

impl StickShaper {
    /// The setpoint for the effective `mode` at step time `t_us`. Acro and Position are declared only: Angle.
    fn setpoint(&mut self, mode: Mode, nav: &NavState, t_us: u64) -> Setpoint {
        let dt_s = self
            .t_prev_us
            .map_or(0.0, |prev| t_us.saturating_sub(prev).min(DT_MAX_US) as f32 * 1e-6);
        self.t_prev_us = Some(t_us);
        match mode {
            Mode::Altitude => self.altitude(nav, dt_s),
            Mode::Acro | Mode::Angle | Mode::Position => {
                self.target_d = None;
                self.angle(nav)
            }
        }
    }

    /// The throttle stick's climb rate, m/s, down positive (F2, ArduPilot's law): 0 within `pilot_thr_dz` of the
    /// centre; below it `pilot_speed_dn_ms` x (distance below the dead zone / the travel left below it), reaching
    /// `pilot_speed_dn_ms` at 0; above it minus `pilot_speed_up_ms` x the same, reaching it at 1.
    fn stick_rate_d(&self, throttle: f32) -> f32 {
        let thr = throttle.clamp(0.0, 1.0);
        let low = THROTTLE_CENTRE - self.pilot_thr_dz;
        let high = THROTTLE_CENTRE + self.pilot_thr_dz;
        // The travel on each side of the dead zone is `low` (= 1 - high).
        if thr < low {
            self.pilot_speed_dn_ms * (low - thr) / low
        } else if thr > high {
            -self.pilot_speed_up_ms * (thr - high) / low
        } else {
            0.0
        }
    }

    /// Altitude hold (F1, F2): roll, pitch and yaw as Angle; the target height captured at the first call after any
    /// other mode (no jump: it is the current height), then integrated at the stick's rate while the vehicle is
    /// within [`ALT_LEAD_S`] of the limit speed from it. Down is positive.
    fn altitude(&mut self, nav: &NavState, dt_s: f32) -> Setpoint {
        let (attitude, heading_rate_rads, _) = self.attitude_setpoint(nav);
        let d = nav.position_m[2];
        let v_ff = self.stick_rate_d(self.sticks.throttle);
        let target = match self.target_d {
            None => d,
            Some(target) => {
                // The limit speed of the direction the target moves in: down is the descent speed.
                let limit = if v_ff > 0.0 {
                    self.pilot_speed_dn_ms
                } else {
                    self.pilot_speed_up_ms
                };
                let within = libm::fabsf(d - target) < limit * ALT_LEAD_S;
                #[cfg(test)]
                let within = within || self.lead_cap_off;
                if within { target + v_ff * dt_s } else { target }
            }
        };
        self.target_d = Some(target);
        Setpoint::Altitude {
            attitude,
            heading_rate_rads,
            down_m: target,
            velocity_d_ms: v_ff,
        }
    }

    /// q_sp = twist(q) (x) q_tilt and the heading rate from yaw (4a, 4b), and the stick collective, N.
    fn attitude_setpoint(&self, nav: &NavState) -> ([f32; 4], f32, f32) {
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
        let full = full_thrust_n(self.pilot_thrust_max_g);
        let thrust_n = if throttle <= 0.5 {
            2.0 * throttle * hover
        } else {
            hover + (2.0 * throttle - 1.0) * (full - hover)
        };
        (quat::mul(heading, tilt), yaw * self.pilot_y_rate_rads, thrust_n)
    }

    /// Angle: [`StickShaper::attitude_setpoint`] as a `Setpoint`.
    fn angle(&self, nav: &NavState) -> Setpoint {
        let (attitude, heading_rate_rads, thrust_n) = self.attitude_setpoint(nav);
        Setpoint::Attitude {
            attitude,
            heading_rate_rads,
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
    /// The mode the switch selects; the newest report's switch holds until the next report.
    selected: Mode,
    limited: Saturation,
    /// Tests only: the effective mode ignores what the estimate covers.
    #[cfg(test)]
    ignore_coverage: bool,
}

impl<E: Estimator, C: Controller, A: Allocator> Flight<E, C, A> {
    /// `tuning`: the gains and limits; `handshake`: where the gate stores the armed state for core 0.
    /// The selected mode is `Mode::Angle` until the first stick report brings the switch.
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
                pilot_speed_up_ms: tuning.pilot_speed_up_ms,
                pilot_speed_dn_ms: tuning.pilot_speed_dn_ms,
                pilot_thr_dz: tuning.pilot_thr_dz,
                target_d: None,
                t_prev_us: None,
                #[cfg(test)]
                lead_cap_off: false,
            },
            selected: Mode::Angle,
            limited: Saturation {
                roll: false,
                pitch: false,
                yaw: false,
                thrust_low: false,
                thrust_high: false,
            },
            #[cfg(test)]
            ignore_coverage: false,
        }
    }

    /// The estimator, read only (the log of its own counters, packet E2).
    pub fn estimator(&self) -> &E {
        &self.estimator
    }

    /// [`Flight::step`], then `publish` with its output (the board sends the command there), then the shadow estimator
    /// (decision-nav-eskf-20261120, 8a): the gate's armed state after the step and the same sensors. The step's output
    /// is computed and published before the shadow runs and nothing of the shadow reaches the flight code; its state is
    /// returned for the log only. `None`: no shadow, the plain step.
    pub fn step_with_shadow<S: Estimator>(
        &mut self,
        inputs: &FlightInputs,
        shadow: Option<&mut S>,
        publish: impl FnOnce(&StepOut),
    ) -> (StepOut, Option<NavState>) {
        let out = self.step(inputs);
        publish(&out);
        let nav = shadow.map(|s| {
            s.set_armed(out.armed);
            s.update(&inputs.sensors)
        });
        (out, nav)
    }

    /// One call per IMU1 sample.
    pub fn step(&mut self, inputs: &FlightInputs) -> StepOut {
        let t_us = inputs.sensors.imu1.t_us;
        if let Some(rc) = inputs.rc {
            self.shaper.sticks = rc.sticks;
            self.selected = match rc.switch6 {
                Switch3::Low => Mode::Angle,
                Switch3::Middle => Mode::Altitude,
                Switch3::High => Mode::Position,
            };
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
        // The effective mode (5e, F1): Altitude if selected (Position until it is built counts as Altitude) and the
        // estimate covers it, otherwise Angle. Evaluated every step; Acro cannot be selected.
        #[cfg(test)]
        let covered = self.ignore_coverage || altitude_covered(&nav.valid);
        #[cfg(not(test))]
        let covered = altitude_covered(&nav.valid);
        let mode = match self.selected {
            Mode::Altitude | Mode::Position if covered => Mode::Altitude,
            _ => Mode::Angle,
        };
        let setpoint = self.shaper.setpoint(mode, &nav, t_us);
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
            selected: self.selected,
            mode,
            nav,
            setpoint,
            demand,
            allocation,
        }
    }
}

/// True when `valid` covers what an Altitude setpoint needs (`Setpoint::requires`, the single definition).
fn altitude_covered(valid: &Valid) -> bool {
    let probe = Setpoint::Altitude {
        attitude: [1.0, 0.0, 0.0, 0.0],
        heading_rate_rads: 0.0,
        down_m: 0.0,
        velocity_d_ms: 0.0,
    };
    valid.covers(&probe.requires())
}

/// What the log writes at one flight step (decision-flight-logging-20261119, section 1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogDue {
    /// Write the IMU1 sample this step (every step, armed or not).
    pub imu: bool,
    /// Write the four flight records (NAV, SETP, DMD, MOT) this step.
    pub block: bool,
    /// Write an ESC record this step.
    pub esc: bool,
}

/// The decimation rule. `step` counts flight steps (the caller adds 1 after each). The IMU1 sample is
/// logged every step. Armed: the four flight records every step and an ESC record every 4th; disarmed:
/// the four flight records and the ESC record every 8th step.
pub fn log_due(step: u32, armed: bool) -> LogDue {
    if armed {
        LogDue {
            imu: true,
            block: true,
            esc: step % 4 == 0,
        }
    } else {
        let every_8th = step % 8 == 0;
        LogDue {
            imu: true,
            block: every_8th,
            esc: every_8th,
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
    let valid = valid_bits(&out.nav.valid);
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
    let mode = mode_code(out.mode);
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

/// MOT's and MODE's code of a mode: 0 Acro, 1 Angle, 2 Altitude, 3 Position.
pub fn mode_code(mode: Mode) -> u8 {
    match mode {
        Mode::Acro => 0,
        Mode::Angle => 1,
        Mode::Altitude => 2,
        Mode::Position => 3,
    }
}

/// When to write MODE (decision-nav-eskf-20261120, 8c, F6): at the first step and at every change of the selected or
/// the effective mode. The caller asks [`ModeLog::pending`], writes [`encode_mode`] and only then calls
/// [`ModeLog::done`], so a record the ring dropped is offered again at the next step.
#[derive(Clone, Copy, Debug, Default)]
pub struct ModeLog {
    /// (selected, effective) codes of the last record written; `None` before the first.
    last: Option<(u8, u8)>,
}

impl ModeLog {
    /// Nothing written yet.
    pub const fn new() -> Self {
        Self { last: None }
    }

    /// This step's (selected, effective) differs from the last record, or none was written yet.
    pub fn pending(&self, out: &StepOut) -> bool {
        self.last != Some((mode_code(out.selected), mode_code(out.mode)))
    }

    /// The record for `out` was written.
    pub fn done(&mut self, out: &StepOut) {
        self.last = Some((mode_code(out.selected), mode_code(out.mode)));
    }
}

/// MODE of this step at the start of `buf`: `TimeUS` = `out.t_us`, the selected and the effective mode. Returns the
/// bytes written (13), or 0 (nothing written) if `buf` is too small.
pub fn encode_mode(out: &StepOut, buf: &mut [u8]) -> usize {
    dflog::encode_mode(buf, out.t_us, mode_code(out.selected), mode_code(out.mode))
}

/// SPN of this step at the start of `buf` (decision-nav-eskf-20261120, 8c, F1): only when the setpoint is Altitude,
/// `TimeUS` = `out.t_us`, PD the target height (down positive), VD the climb-rate feed-forward (down positive); PN, PE,
/// VN, VE and Hdg are 0 (Position will fill them). Returns the bytes written (39), or 0 (nothing written) if the
/// setpoint is not Altitude or `buf` is too small.
pub fn encode_spn(out: &StepOut, buf: &mut [u8]) -> usize {
    match out.setpoint {
        Setpoint::Altitude {
            down_m,
            velocity_d_ms,
            ..
        } => dflog::encode_spn(
            buf,
            out.t_us,
            [0.0, 0.0, down_m],
            [0.0, 0.0, velocity_d_ms],
            0.0,
        ),
        _ => 0,
    }
}

/// NAV's and NKF1's valid bits: tilt 1, heading 2, rates 4, velocity NE 8, velocity D 16, position NE 32,
/// position D 64.
fn valid_bits(v: &Valid) -> u8 {
    let Valid {
        tilt,
        heading,
        rates,
        velocity_ne,
        velocity_d,
        position_ne,
        position_d,
    } = *v;
    u8::from(tilt)
        | u8::from(heading) << 1
        | u8::from(rates) << 2
        | u8::from(velocity_ne) << 3
        | u8::from(velocity_d) << 4
        | u8::from(position_ne) << 5
        | u8::from(position_d) << 6
}

/// NKF1 then NKF2 of one estimator's state (decision-nav-eskf-20261120, 8c, B5) at the start of `buf`: instance
/// `inst`, `TimeUS` = `nav.t_us`, the biases as given, the sigmas from `nav.sigma` (infinite when `None`). Returns the
/// bytes written (113), or 0 (nothing written) if `buf` is too small.
pub fn encode_nkf(
    buf: &mut [u8],
    inst: u8,
    nav: &NavState,
    gyro_bias_rads: [f32; 3],
    accel_bias_ms2: [f32; 3],
) -> usize {
    let need = usize::from(dflog::NKF1.length) + usize::from(dflog::NKF2.length);
    if buf.len() < need {
        return 0;
    }
    let inf = f32::INFINITY;
    let sigmas = nav.sigma.map_or([inf; 6], |s| {
        [
            s.tilt_rad,
            s.heading_rad,
            s.velocity_ne_ms,
            s.velocity_d_ms,
            s.position_ne_m,
            s.position_d_m,
        ]
    });
    let n = dflog::encode_nkf1(
        buf,
        nav.t_us,
        inst,
        nav.attitude,
        nav.velocity_ms,
        nav.position_m,
        valid_bits(&nav.valid),
    );
    n + dflog::encode_nkf2(
        buf.get_mut(n..).unwrap_or_default(),
        nav.t_us,
        inst,
        gyro_bias_rads,
        accel_bias_ms2,
        sigmas,
    )
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
            switch6: Switch3::Low,
            t_us,
        };
        let mut t = 2_000;
        // The first sample (zero accel) seeds the accelerometer filter: the tilt is valid only once the filtered |f|
        // reaches 0.9 g, tau x ln(10) after the first level step. Run the level steps that takes, plus two.
        const STEP_US: u64 = 1_000;
        let tau_s = 1.0 / (2.0 * core::f32::consts::PI * tuning.ins_accel_lpf_hz);
        let settle = (libm::logf(10.0) * tau_s / (STEP_US as f32 * 1e-6)).ceil() as u32 + 2;
        t += STEP_US;
        out = flight.step(&level(t, [0.0; 3]));
        assert!(!out.nav.valid.tilt);
        for _ in 1..settle {
            t += STEP_US;
            out = flight.step(&level(t, [0.0; 3]));
        }
        HANDSHAKE.set_permit(true);
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

    /// Decision-nav-eskf-20261120, section 4 (packet F1): with Mahony the barometer, compass and GNSS samples change
    /// nothing. The same step sequence (level, arm, sticks moving, a moving IMU) with and without the three samples
    /// gives the same outputs, bit for bit (every field's exact `Debug` text and the step's log records).
    #[test]
    fn slow_sensor_samples_do_not_change_a_mahony_step() {
        extern crate std;
        use fw_core::sample::{
            BaroReading, BaroSample, GnssReading, GnssSample, MagReading, MagSample, UtcTime,
        };
        static WITHOUT: Handshake = Handshake::new();
        static WITH: Handshake = Handshake::new();
        let run = |handshake: &'static Handshake, slow: bool| {
            let tuning = Tuning::DEFAULT;
            let mut flight = Flight::new(
                Mahony::new(&tuning),
                CascadedPid::new(&tuning),
                CardAllocator::new(),
                &tuning,
                handshake,
            );
            handshake.set_permit(true);
            let mut outs = std::vec::Vec::new();
            for n in 0..3000u32 {
                let t_us = 10_000 + u64::from(n) * 1_245;
                let x = n as f32 * 0.01;
                let mut i = inputs(t_us);
                i.sensors.imu1.seq = n;
                i.sensors.imu1.reading.gyro_rads = [
                    0.2 * libm::sinf(x),
                    0.1 * libm::cosf(1.3 * x),
                    0.05 * libm::sinf(0.7 * x),
                ];
                i.sensors.imu1.reading.accel_ms2 = [
                    0.3 * libm::sinf(0.9 * x),
                    -0.2 * libm::cosf(x),
                    -9.81 + 0.1 * libm::sinf(x),
                ];
                // Arm after 200 steps (throttle low), then sticks that move.
                let stick = |k: f32| {
                    if n < 400 {
                        0.0
                    } else {
                        0.3 * libm::sinf(k * x)
                    }
                };
                i.rc = (n % 10 == 0).then_some(RcSample {
                    sticks: Sticks {
                        roll: stick(1.1),
                        pitch: stick(0.8),
                        throttle: if n < 400 { 0.0 } else { 0.5 + stick(0.5) },
                        yaw: stick(1.7),
                    },
                    arm: n >= 200,
                    switch6: Switch3::Low,
                    t_us,
                });
                if slow {
                    i.sensors.baro = (n % 16 == 3).then_some(BaroSample {
                        reading: BaroReading {
                            pressure_pa: 101_325.0 - x,
                            temperature_c: 25.0,
                        },
                        t_us: t_us - 500,
                        seq: n / 16,
                    });
                    i.sensors.mag = (n % 8 == 5).then_some(MagSample {
                        reading: MagReading {
                            field_ut: [20.0 * libm::cosf(x), 20.0 * libm::sinf(x), 40.0],
                            temperature_c: 30.0,
                        },
                        t_us: t_us - 300,
                        seq: n / 8,
                        expected: None,
                    });
                    i.sensors.gnss = (n % 160 == 7).then_some(GnssSample {
                        reading: GnssReading {
                            fix_type: 3,
                            lat_e7: Some(254_000_000 + n as i32),
                            lon_e7: Some(-801_000_000 - n as i32),
                            alt_mm: Some(12_000),
                            speed_mm_s: Some(n),
                            course_cdeg: Some(9_000),
                            sats_used: Some(12),
                            hdop_x100: Some(80),
                            vdop_x100: Some(120),
                            time: Some(UtcTime {
                                hour: 12,
                                minute: 0,
                                second: (n / 160) as u8,
                                millis: 0,
                            }),
                            date: None,
                            lat_std_mm: Some(1_200),
                            lon_std_mm: Some(900),
                            alt_std_mm: Some(2_500),
                            vel_ned_mm_s: None,
                        },
                        t_us: t_us - 430_000.min(t_us),
                        seq: n / 160,
                    });
                }
                let out = flight.step(&i);
                let mut records = [0u8; 256];
                let len = encode_step(&out, &mut records);
                outs.push((std::format!("{out:?}"), records[..len].to_vec(), out));
            }
            outs
        };
        let (without, with) = (run(&WITHOUT, false), run(&WITH, true));
        assert_eq!(without.len(), with.len());
        for (n, (a, b)) in without.iter().zip(&with).enumerate() {
            assert_eq!(a.0, b.0, "step {n}: outputs differ");
            assert_eq!(a.1, b.1, "step {n}: log records differ");
        }
        // The sequence is not trivial: it armed, and the motors ran with commands that change.
        let last = &with[with.len() - 1].2;
        assert!(last.armed && WITH.armed());
        let throttles: std::vec::Vec<_> =
            with.iter().filter_map(|o| o.2.motors.throttles()).collect();
        assert!(throttles.len() > 2000);
        assert!(throttles.windows(2).any(|w| w[0] != w[1]));
    }

    extern crate std;

    /// The sequence of `slow_sensor_samples_do_not_change_a_mahony_step` at step `n`, with the barometer, compass and
    /// GNSS samples (`mag_expected` gives the compass samples an expected field).
    fn slow_inputs(n: u32, mag_expected: Option<fw_core::sample::MagExpected>) -> FlightInputs {
        use fw_core::sample::{
            BaroReading, BaroSample, GnssReading, GnssSample, MagReading, MagSample, UtcTime,
        };
        let t_us = 10_000 + u64::from(n) * 1_245;
        let x = n as f32 * 0.01;
        let mut i = inputs(t_us);
        i.sensors.imu1.seq = n;
        i.sensors.imu1.reading.gyro_rads = [
            0.2 * libm::sinf(x),
            0.1 * libm::cosf(1.3 * x),
            0.05 * libm::sinf(0.7 * x),
        ];
        i.sensors.imu1.reading.accel_ms2 = [
            0.3 * libm::sinf(0.9 * x),
            -0.2 * libm::cosf(x),
            -9.81 + 0.1 * libm::sinf(x),
        ];
        let stick = |k: f32| {
            if n < 400 {
                0.0
            } else {
                0.3 * libm::sinf(k * x)
            }
        };
        i.rc = (n % 10 == 0).then_some(RcSample {
            sticks: Sticks {
                roll: stick(1.1),
                pitch: stick(0.8),
                throttle: if n < 400 { 0.0 } else { 0.5 + stick(0.5) },
                yaw: stick(1.7),
            },
            arm: n >= 200,
            switch6: Switch3::Low,
            t_us,
        });
        i.sensors.baro = (n % 16 == 3).then_some(BaroSample {
            reading: BaroReading {
                pressure_pa: 101_325.0 - x,
                temperature_c: 25.0,
            },
            t_us: t_us - 500,
            seq: n / 16,
        });
        i.sensors.mag = (n % 8 == 5).then_some(MagSample {
            reading: MagReading {
                field_ut: [20.0 * libm::cosf(x), 20.0 * libm::sinf(x), 40.0],
                temperature_c: 30.0,
            },
            t_us: t_us - 300,
            seq: n / 8,
            expected: mag_expected,
        });
        i.sensors.gnss = (n % 160 == 7).then_some(GnssSample {
            reading: GnssReading {
                fix_type: 3,
                lat_e7: Some(254_000_000 + n as i32),
                lon_e7: Some(-801_000_000 - n as i32),
                alt_mm: Some(12_000),
                speed_mm_s: Some(n),
                course_cdeg: Some(9_000),
                sats_used: Some(12),
                hdop_x100: Some(80),
                vdop_x100: Some(120),
                time: Some(UtcTime {
                    hour: 12,
                    minute: 0,
                    second: (n / 160) as u8,
                    millis: 0,
                }),
                date: None,
                lat_std_mm: Some(1_200),
                lon_std_mm: Some(900),
                alt_std_mm: Some(2_500),
                vel_ned_mm_s: None,
            },
            t_us: t_us - 430_000.min(t_us),
            seq: n / 160,
        });
        i
    }

    /// The parts of a step output the flight uses and logs outside the vertical estimate: motors, armed, mode,
    /// setpoint, demand, allocation, attitude, rates, the tilt, heading, rates and NE flags, and the step's records
    /// with NAV's velocity-D and position-D bits cleared.
    fn flight_part(out: &StepOut) -> (std::string::String, std::vec::Vec<u8>) {
        let v = out.nav.valid;
        let text = std::format!(
            "{:?} {:?} {:?} {:?} {:?} {:?} {:?} {:?} {} {} {} {} {}",
            out.t_us,
            out.motors,
            out.armed,
            out.mode,
            out.setpoint,
            out.demand,
            out.allocation,
            (
                out.nav.attitude.map(f32::to_bits),
                out.nav.rates_rads.map(f32::to_bits)
            ),
            out.nav.t_us,
            v.tilt,
            v.heading,
            v.rates,
            v.velocity_ne || v.position_ne,
        );
        let mut records = [0u8; 256];
        let len = encode_step(out, &mut records);
        records[39] &= !(16 | 64);
        (text, records[..len].to_vec())
    }

    /// Packet E2, item 1: the board's estimator `MahonyBaro` flies as `Mahony` did. The sequence of
    /// `slow_sensor_samples_do_not_change_a_mahony_step` (with the barometer, compass and GNSS samples) gives the same
    /// motors, setpoints, demands, allocations, attitude, rates and tilt flags, bit for bit, with either estimator; only
    /// the vertical estimate (velocity D, position D, their flags and sigmas) differs, and it does.
    #[test]
    fn mahony_baro_steps_as_mahony_bit_for_bit() {
        extern crate std;
        use crate::mahony_baro::MahonyBaro;
        static A: Handshake = Handshake::new();
        static B: Handshake = Handshake::new();
        let tuning = Tuning::DEFAULT;
        let mut mahony = Flight::new(
            Mahony::new(&tuning),
            CascadedPid::new(&tuning),
            CardAllocator::new(),
            &tuning,
            &A,
        );
        let mut baro = Flight::new(
            MahonyBaro::new(&tuning),
            CascadedPid::new(&tuning),
            CardAllocator::new(),
            &tuning,
            &B,
        );
        A.set_permit(true);
        B.set_permit(true);
        let (mut spins, mut vertical) = (0u32, 0u32);
        for n in 0..3000u32 {
            let i = slow_inputs(n, None);
            let (a, b) = (mahony.step(&i), baro.step(&i));
            assert_eq!(flight_part(&a), flight_part(&b), "step {n}");
            spins += u32::from(b.motors.throttles().is_some());
            vertical += u32::from(b.nav.valid.position_d && b.nav.sigma.is_some());
            assert!(a.nav.sigma.is_none());
        }
        assert!(A.armed() && B.armed());
        assert!(spins > 2000, "{spins}");
        assert!(vertical > 2000, "{vertical}");
        assert!(baro.estimator().vert().fused() > 100);
    }

    /// Packet E2, item 6: the shadow never changes the step. The board's flight code over the same sequence with no
    /// shadow, with the `Eskf` at the default tuning and with an `Eskf` of another tuning gives identical step outputs
    /// and records (every field's exact `Debug` text), while the two shadows' states differ.
    #[test]
    fn the_shadow_never_changes_the_step() {
        extern crate std;
        use crate::eskf::Eskf;
        use crate::mahony_baro::MahonyBaro;
        use fw_core::sample::MagExpected;
        static H: [Handshake; 3] = [Handshake::new(), Handshake::new(), Handshake::new()];
        let mut other = Tuning::DEFAULT;
        other.nav_gate_sigma *= 0.5;
        other.nav_baro_sigma_m *= 3.0;
        other.nav_predict_us *= 2;
        let expected = Some(MagExpected {
            strength_ut: libm::sqrtf(20.0 * 20.0 + 40.0 * 40.0),
            inclination_rad: libm::atanf(40.0 / 20.0),
            tol_strength_frac: 0.2,
            tol_inclination_rad: 0.2,
        });
        let run = |handshake: &'static Handshake, shadow: Option<Tuning>| {
            let tuning = Tuning::DEFAULT;
            let mut flight = Flight::new(
                MahonyBaro::new(&tuning),
                CascadedPid::new(&tuning),
                CardAllocator::new(),
                &tuning,
                handshake,
            );
            let mut eskf = shadow.map(|t| std::boxed::Box::new(Eskf::new(&t).unwrap()));
            handshake.set_permit(true);
            let mut outs = std::vec::Vec::new();
            let mut navs = std::vec::Vec::new();
            for n in 0..3000u32 {
                let i = slow_inputs(n, expected);
                let (out, nav) = flight.step_with_shadow(&i, eskf.as_deref_mut(), |_| {});
                let mut records = [0u8; 256];
                let len = encode_step(&out, &mut records);
                outs.push((std::format!("{out:?}"), records[..len].to_vec()));
                navs.push(nav.map(|v| std::format!("{v:?}")));
            }
            (outs, navs)
        };
        let (plain, none) = run(&H[0], None);
        let (default, a) = run(&H[1], Some(Tuning::DEFAULT));
        let (changed, b) = run(&H[2], Some(other));
        assert!(none.iter().all(Option::is_none));
        for n in 0..plain.len() {
            assert_eq!(
                plain[n], default[n],
                "step {n}: the default shadow changed the step"
            );
            assert_eq!(
                plain[n], changed[n],
                "step {n}: the other shadow changed the step"
            );
        }
        assert!(a.iter().all(Option::is_some));
        assert!(a.iter().zip(&b).filter(|(x, y)| x != y).count() > 1000);
        assert!(H.iter().all(Handshake::armed));
    }

    /// NKF1 and NKF2 of one state: the instance, the horizon time, the valid bits, the sigmas, infinite without them.
    #[test]
    fn encode_nkf_writes_the_two_records() {
        use crate::gnc::NavSigma;
        let mut nav = NavState {
            t_us: 0x0102_0304_0506,
            attitude: [0.5, -0.5, 0.25, 0.75],
            rates_rads: [9.0; 3],
            velocity_ms: [0.1, -0.2, 0.3],
            position_m: [1.0, -2.0, 3.0],
            valid: Valid {
                tilt: true,
                velocity_d: true,
                position_d: true,
                ..Valid::NONE
            },
            sigma: Some(NavSigma {
                tilt_rad: 0.01,
                heading_rad: 0.02,
                velocity_ne_ms: 0.03,
                velocity_d_ms: 0.04,
                position_ne_m: 0.05,
                position_d_m: 0.06,
            }),
        };
        let f = |b: &[u8], at: usize| f32::from_le_bytes(b[at..at + 4].try_into().unwrap());
        let mut buf = [0xEEu8; 120];
        assert_eq!(
            encode_nkf(&mut buf, 1, &nav, [0.001, 0.002, 0.003], [0.1, 0.2, 0.3]),
            113
        );
        assert_eq!(encode_nkf(&mut [0u8; 112], 1, &nav, [0.0; 3], [0.0; 3]), 0);
        let time = 0x0102_0304_0506u64.to_le_bytes();
        assert_eq!(buf[0..3], [0xA3, 0x95, 147]);
        assert_eq!(buf[3..11], time);
        assert_eq!(buf[11], 1);
        let q: [f32; 4] = core::array::from_fn(|k| f(&buf, 12 + 4 * k));
        assert_eq!(q, nav.attitude);
        let vp: [f32; 6] = core::array::from_fn(|k| f(&buf, 28 + 4 * k));
        assert_eq!(vp, [0.1, -0.2, 0.3, 1.0, -2.0, 3.0]);
        assert_eq!(buf[52], 1 | 16 | 64);
        assert_eq!(buf[53..56], [0xA3, 0x95, 148]);
        assert_eq!(buf[56..64], time);
        assert_eq!(buf[64], 1);
        let rest: [f32; 12] = core::array::from_fn(|k| f(&buf, 65 + 4 * k));
        assert_eq!(
            rest,
            [
                0.001, 0.002, 0.003, 0.1, 0.2, 0.3, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06
            ]
        );
        assert_eq!(buf[113..], [0xEE; 7]);
        nav.sigma = None;
        encode_nkf(&mut buf, 0, &nav, [0.0; 3], [0.0; 3]);
        assert_eq!(buf[11], 0);
        assert!((0..6).all(|k| f(&buf, 65 + 24 + 4 * k) == f32::INFINITY));
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
            selected: Mode::Angle,
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

    #[test]
    fn log_due_decimation_over_64_steps() {
        let count = |armed| {
            (0..64u32)
                .map(|n| log_due(n, armed))
                .fold((0, 0, 0), |(i, b, e), d| {
                    (
                        i + u32::from(d.imu),
                        b + u32::from(d.block),
                        e + u32::from(d.esc),
                    )
                })
        };
        assert_eq!(count(true), (64, 64, 16));
        assert_eq!(count(false), (64, 8, 8));
        assert_eq!(
            log_due(0, false),
            LogDue {
                imu: true,
                block: true,
                esc: true
            }
        );
        assert_eq!(
            log_due(1, false),
            LogDue {
                imu: true,
                block: false,
                esc: false
            }
        );
        assert_eq!(
            log_due(4, true),
            LogDue {
                imu: true,
                block: true,
                esc: true
            }
        );
        assert_eq!(
            log_due(5, true),
            LogDue {
                imu: true,
                block: true,
                esc: false
            }
        );
    }

    #[test]
    fn log_due_disarmed_imu_every_step_block_every_8th() {
        for k in 0..64u32 {
            let d = log_due(k, false);
            assert!(d.imu);
            assert_eq!(d.block, k % 8 == 0);
            assert_eq!(d.esc, k % 8 == 0);
            let a = log_due(k, true);
            assert!(a.imu && a.block);
            assert_eq!(a.esc, k % 4 == 0);
        }
    }
}

#[cfg(test)]
mod althold_tests;
