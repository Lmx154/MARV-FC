//! The controller: an attitude P loop feeding a PID rate loop per axis (decision-first-pair-20261119, section 3).
//! Torques in N.m about FRD, rates in rad/s, collective thrust in N. No feed-forward.

use crate::gnc::{Controller, NavState, Refused, Saturation, Setpoint, ThrustTorque};
use crate::mahony::DT_MAX_US;
use crate::quat::{conj, down_in_body, mul, pt1};
use crate::tuning::Tuning;

/// One rate-loop axis: gains and its memory.
#[derive(Clone, Copy, Debug)]
struct Axis {
    /// P, N.m/(rad/s).
    kp: f32,
    /// I, N.m/rad.
    ki: f32,
    /// D, N.m.s/rad.
    kd: f32,
    /// Integrator limit, N.m.
    imax: f32,
    /// The integrator, N.m.
    i: f32,
    /// The D filter's output, the PT1 of the measured rate, rad/s.
    m: f32,
}

impl Axis {
    fn new(kp: f32, ki: f32, kd: f32, imax: f32) -> Self {
        Axis {
            kp,
            ki,
            kd,
            imax,
            i: 0.0,
            m: 0.0,
        }
    }

    /// Torque, N.m, for rate error `err` and measured `rate`, rad/s, over `dt_s`. `first`: the first step after a
    /// reset (dt 0, the D filter starts at the measurement). `limited`: this axis's flag from the previous allocation.
    fn step(
        &mut self,
        err: f32,
        rate: f32,
        dt_s: f32,
        first: bool,
        limited: bool,
        fltd_hz: f32,
    ) -> f32 {
        // D on the filtered measurement, not the error (decision 3c).
        let m = if first {
            rate
        } else {
            pt1(self.m, rate, dt_s, fltd_hz)
        };
        let d = if dt_s > 0.0 {
            -self.kd * (m - self.m) / dt_s
        } else {
            0.0
        };
        self.m = m;
        // I: keep i' unless the axis is limited and i' grows in magnitude, then clamp (ArduPilot's rule, 3c).
        let i_next = self.i + self.ki * err * dt_s;
        if !(limited && i_next.abs() > self.i.abs()) {
            self.i = i_next;
        }
        self.i = self.i.max(-self.imax).min(self.imax);
        self.kp * err + self.i + d
    }
}

/// The cascaded PID. Accepts `Setpoint::Rates` and `Setpoint::Attitude`; refuses the rest.
#[derive(Clone, Copy, Debug)]
pub struct CascadedPid {
    /// Angle P for roll and pitch, 1/s.
    k_rp: f32,
    /// Angle P for yaw, 1/s.
    k_y: f32,
    /// D filter cut-off, Hz.
    fltd_hz: f32,
    /// Roll, pitch, yaw.
    axes: [Axis; 3],
    /// The previous step's `nav.t_us`; `None` after `reset`.
    t_prev_us: Option<u64>,
}

impl CascadedPid {
    /// Gains, limits and the D filter from `tuning`; integrators zero.
    pub fn new(tuning: &Tuning) -> Self {
        let [p0, p1, p2] = tuning.atc_rat_p;
        let [i0, i1, i2] = tuning.atc_rat_i;
        let [d0, d1, d2] = tuning.atc_rat_d;
        let [m0, m1, m2] = tuning.atc_rat_imax;
        CascadedPid {
            k_rp: tuning.atc_ang_rp_p,
            k_y: tuning.atc_ang_yaw_p,
            fltd_hz: tuning.atc_rat_fltd_hz,
            axes: [
                Axis::new(p0, i0, d0, m0),
                Axis::new(p1, i1, d1, m1),
                Axis::new(p2, i2, d2, m2),
            ],
            t_prev_us: None,
        }
    }

    /// Body rate setpoint, rad/s, from an attitude setpoint (decision 3b): q_e = conj(q) (x) q_sp with w >= 0,
    /// e = 2 [x, y, z], rate = [K_rp e_x, K_rp e_y, K_y e_z] + heading_rate * (NED down in the body).
    fn attitude_to_rates(&self, q: [f32; 4], q_sp: [f32; 4], heading_rate_rads: f32) -> [f32; 3] {
        let mut q_e = mul(conj(q), q_sp);
        if q_e[0] < 0.0 {
            q_e = q_e.map(|v| -v);
        }
        let [_, x, y, z] = q_e;
        let [dx, dy, dz] = down_in_body(q);
        [
            self.k_rp * 2.0 * x + heading_rate_rads * dx,
            self.k_rp * 2.0 * y + heading_rate_rads * dy,
            self.k_y * 2.0 * z + heading_rate_rads * dz,
        ]
    }
}

impl Controller for CascadedPid {
    fn update(
        &mut self,
        nav: &NavState,
        sp: &Setpoint,
        limited: &Saturation,
    ) -> Result<ThrustTorque, Refused> {
        let (rate_sp, thrust_n) = match *sp {
            Setpoint::Rates {
                rates_rads,
                thrust_n,
            } => (rates_rads, thrust_n),
            Setpoint::Attitude {
                attitude,
                heading_rate_rads,
                thrust_n,
            } => (
                self.attitude_to_rates(nav.attitude, attitude, heading_rate_rads),
                thrust_n,
            ),
            Setpoint::Altitude { .. } | Setpoint::Position { .. } | Setpoint::Waypoint { .. } => {
                return Err(Refused);
            }
        };
        // dt in s from successive nav.t_us, capped; 0 on the first step after reset (decision 3d).
        let first = self.t_prev_us.is_none();
        let dt = match self.t_prev_us {
            Some(prev) => nav.t_us.saturating_sub(prev).min(DT_MAX_US) as f32 * 1e-6,
            None => 0.0,
        };
        self.t_prev_us = Some(nav.t_us);

        let [sx, sy, sz] = rate_sp;
        let [mx, my, mz] = nav.rates_rads;
        let hz = self.fltd_hz;
        let [roll, pitch, yaw] = &mut self.axes;
        Ok(ThrustTorque {
            thrust_n: thrust_n.max(0.0),
            torque_nm: [
                roll.step(sx - mx, mx, dt, first, limited.roll, hz),
                pitch.step(sy - my, my, dt, first, limited.pitch, hz),
                yaw.step(sz - mz, mz, dt, first, limited.yaw, hz),
            ],
        })
    }

    fn reset(&mut self) {
        for axis in self.axes.iter_mut() {
            axis.i = 0.0;
            axis.m = 0.0;
        }
        self.t_prev_us = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnc::Valid;

    const PERIOD_US: u64 = 1_250;

    fn nav(t_us: u64) -> NavState {
        NavState {
            t_us,
            attitude: [1.0, 0.0, 0.0, 0.0],
            rates_rads: [0.0; 3],
            velocity_ms: [0.0; 3],
            position_m: [0.0; 3],
            valid: Valid {
                tilt: true,
                rates: true,
                ..Valid::NONE
            },
            sigma: None,
        }
    }

    fn limited(roll: bool) -> Saturation {
        Saturation {
            roll,
            pitch: false,
            yaw: false,
            thrust_low: false,
            thrust_high: false,
        }
    }

    fn rates(roll_rads: f32) -> Setpoint {
        Setpoint::Rates {
            rates_rads: [roll_rads, 0.0, 0.0],
            thrust_n: 1.0,
        }
    }

    #[test]
    fn integrator_holds_when_its_axis_is_limited() {
        let tuning = Tuning::DEFAULT;
        let imax = tuning.atc_rat_imax[0];
        let mut pid = CascadedPid::new(&tuning);
        let mut t = 0;
        let mut step = |pid: &mut CascadedPid, sp: f32, lim: bool| {
            pid.update(&nav(t), &rates(sp), &limited(lim)).unwrap();
            t += PERIOD_US;
            pid.axes[0].i
        };
        // Free: a constant +1 rad/s error makes I grow every step after the first (dt 0).
        let mut i = step(&mut pid, 1.0, false);
        assert_eq!(i, 0.0);
        for _ in 0..100 {
            let next = step(&mut pid, 1.0, false);
            assert!(next > i, "{next} <= {i}");
            i = next;
        }
        // Limited: the same error does not grow it.
        for _ in 0..100 {
            let next = step(&mut pid, 1.0, true);
            assert!(next.abs() <= i.abs(), "{next} grew from {i}");
            i = next;
        }
        assert!(i > 0.0);
        // Limited, opposite error: it may still shrink.
        for _ in 0..10 {
            let next = step(&mut pid, -1.0, true);
            assert!(next < i, "{next} did not shrink from {i}");
            i = next;
        }
        // Free for long enough to reach IMAX (0.166 N.m/rad * 1 rad/s reaches 0.273 N.m in 1.6 s): never above.
        for _ in 0..4000 {
            i = step(&mut pid, 1.0, false);
            assert!(i.abs() <= imax, "{i} > {imax}");
        }
        assert_eq!(i, imax);
        // reset() zeroes it, and the next step (dt 0) adds nothing.
        pid.reset();
        assert_eq!(pid.axes[0].i, 0.0);
        assert_eq!(step(&mut pid, 1.0, false), 0.0);
        // A setpoint the controller does not track.
        let position = Setpoint::Position {
            position_m: [0.0; 3],
            velocity_ms: [0.0; 3],
            heading_rad: 0.0,
            heading_rate_rads: 0.0,
        };
        assert_eq!(
            pid.update(&nav(t), &position, &limited(false)),
            Err(Refused)
        );
    }
}
