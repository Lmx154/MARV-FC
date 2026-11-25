//! The controller: an attitude P loop feeding a PID rate loop per axis, and, for `Setpoint::Altitude` and
//! `Setpoint::Position`, a vertical loop that makes the collective; for `Setpoint::Position` also a horizontal
//! position P, velocity PI loop whose acceleration is turned into a tilt.
//! Torques in N.m about FRD, rates in rad/s, collective thrust in N. No feed-forward in the attitude and rate loops.

use crate::gnc::{Controller, NavState, RateLog, Refused, Saturation, Setpoint, ThrustTorque};
use crate::mahony::DT_MAX_US;
use crate::notch::{Notch, usable};
use crate::quat::{conj, down_in_body, mul, pt1};
use crate::tuning::{Gains, Tuning, full_thrust_n};
use fw_core::calib::GRAVITY_MSS;
use fw_core::esc::LINES;
use marv_card::MASS;

/// The thrust multiplier for a tilt whose cosine is `cos` (third component of the estimated down vector in the body):
/// clamp(gain cos, 0, 1) / clamp(cos, cos_min, 1), with `gain` = `thr_inverted_gain` and `cos_min` = `thr_tilt_cos_min`.
/// 1 when level, 1/cos to the floor, then up to the gain, and 0 at or past upright-sideways (cos <= 0).
fn tilt_factor(cos: f32, cos_min: f32, gain: f32) -> f32 {
    let inverted = (gain * cos).clamp(0.0, 1.0);
    // Not `cos.clamp(cos_min, 1.0)`: with a bound that is not a constant, `f32::clamp` keeps its "min > max" panic and
    // that panic's float formatting, which lies in flash (tools/check-ram-code.py). Same result for cos_min <= 1.
    let lift = if cos < cos_min {
        cos_min
    } else if cos > 1.0 {
        1.0
    } else {
        cos
    };
    inverted / lift
}

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

    /// The terms [P, I, D], N.m, of the torque for rate error `err` and measured `rate`, rad/s, over `dt_s`; the torque
    /// is P + I + D, summed in that order. `first`: the first step after a reset (dt 0, the D filter starts at the
    /// measurement). `limited`: this axis's flag from the previous allocation. `ground`: the integrator is set to 0 and
    /// adds nothing to this step's torque.
    fn step(
        &mut self,
        err: f32,
        rate: f32,
        dt_s: f32,
        first: bool,
        limited: bool,
        ground: bool,
        fltd_hz: f32,
    ) -> [f32; 3] {
        // D on the filtered measurement, not the error.
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
        // I: keep i' unless the axis is limited and i' grows in magnitude, then clamp (ArduPilot's rule).
        if ground {
            self.i = 0.0;
        } else {
            let i_next = self.i + self.ki * err * dt_s;
            if !(limited && i_next.abs() > self.i.abs()) {
                self.i = i_next;
            }
            self.i = self.i.max(-self.imax).min(self.imax);
        }
        [self.kp * err, self.i, d]
    }
}

/// `v` scaled to length `limit` if it is longer; true when it was.
fn limit_length(v: [f32; 2], limit: f32) -> ([f32; 2], bool) {
    let n = libm::sqrtf(v[0] * v[0] + v[1] * v[1]);
    if n > limit {
        let k = limit / n;
        ([v[0] * k, v[1] * k], true)
    } else {
        (v, false)
    }
}

/// The cascaded PID. Accepts `Setpoint::Rates`, `Setpoint::Attitude`, `Setpoint::Altitude` and `Setpoint::Position`;
/// refuses the rest.
#[derive(Clone, Copy, Debug)]
pub struct CascadedPid {
    /// Angle P for roll and pitch, 1/s.
    k_rp: f32,
    /// Angle P for yaw, 1/s.
    k_y: f32,
    /// D filter cut-off, Hz.
    fltd_hz: f32,
    /// The collective below which the vehicle counts as on the ground, N (`ground_thrust_g` x m g).
    ground_thrust_n: f32,
    /// Roll, pitch, yaw.
    axes: [Axis; 3],
    /// The previous step's `nav.t_us`; `None` after `reset`.
    t_prev_us: Option<u64>,
    /// Height error to climb-rate setpoint, 1/s.
    posz_p: f32,
    /// Climb-rate error to acceleration, (m/s^2)/(m/s).
    velz_p: f32,
    /// Climb-rate error integral to acceleration, (m/s^2)/m.
    velz_i: f32,
    /// Limit of the vertical integrator, m/s^2.
    velz_imax: f32,
    /// Largest climb-rate setpoint (up), m/s.
    speed_up_ms: f32,
    /// Largest descent-rate setpoint (down), m/s.
    speed_dn_ms: f32,
    /// The smallest cos(tilt) of the tilt compensation (`thr_tilt_cos_min`).
    tilt_cos_min: f32,
    /// The factor on cos(tilt) in the tilt compensation's numerator (`thr_inverted_gain`).
    tilt_inverted_gain: f32,
    /// The largest collective the vertical loop asks for, N: the same maximum Angle's stick map reaches.
    thrust_full_n: f32,
    /// The vertical integrator, m/s^2, down positive (NED), like the acceleration it adds to.
    alt_i: f32,
    /// The previous setpoint was `Altitude` or `Position`: the integrator is live. False after `reset` and after any
    /// other setpoint, so the first `Altitude` or `Position` setpoint starts the integrator at 0 and a change between
    /// those two does not.
    alt_active: bool,
    /// Position error to velocity setpoint, 1/s.
    posxy_p: f32,
    /// Velocity error to horizontal acceleration, (m/s^2)/(m/s).
    velxy_p: f32,
    /// Velocity error integral to horizontal acceleration, (m/s^2)/m.
    velxy_i: f32,
    /// The largest horizontal velocity setpoint, m/s (`Controller::set_speed_limit_xy`).
    speed_xy_ms: f32,
    /// The largest tilt, rad (`angle_max_rad`).
    angle_max_rad: f32,
    /// The largest horizontal acceleration and the limit of the horizontal integrator, m/s^2: g tan(angle_max_rad).
    a_max: f32,
    /// The horizontal integrator, m/s^2, north and east.
    xy_i: [f32; 2],
    /// The previous setpoint was `Position`: the horizontal integrator is live. False after `reset` and after any other
    /// setpoint.
    pos_active: bool,
    /// The attitude setpoint of the latest `Position` update, [0; 4] after any other update.
    q_sp_pos: [f32; 4],
    /// The RPM notch on the measured rates.
    notch: Notch,
    /// The notch switch of the latest `set_rotor_hz`.
    notch_on: bool,
    /// The rate loop of the latest `update`, with the latest rotor frequencies and switch.
    rate_log: RateLog,
    /// Tests only: the anti-windup rule is off.
    #[cfg(test)]
    pub(crate) antiwindup_off: bool,
    /// Tests only: a `Position` setpoint clears the vertical integrator's live flag, as any other setpoint does.
    #[cfg(test)]
    pub(crate) position_clears_alt_active: bool,
    /// Tests only: the rotation from north-east to the heading frame has the opposite sign.
    #[cfg(test)]
    pub(crate) rotation_flipped: bool,
    /// Tests only: the horizontal integrator is not zeroed at the entry.
    #[cfg(test)]
    pub(crate) entry_keeps_integrator: bool,
}

impl CascadedPid {
    /// Gains, limits and the D filter from `tuning`; integrators zero. `fs_hz`: the configured IMU1 rate, Hz, the sample
    /// rate of the notch; the notch is off until `set_rotor_hz` switches it on.
    pub fn new(tuning: &Tuning, fs_hz: f32) -> Self {
        let [p0, p1, p2] = tuning.atc_rat_p;
        let [i0, i1, i2] = tuning.atc_rat_i;
        let [d0, d1, d2] = tuning.atc_rat_d;
        let [m0, m1, m2] = tuning.atc_rat_imax;
        CascadedPid {
            k_rp: tuning.atc_ang_rp_p,
            k_y: tuning.atc_ang_yaw_p,
            fltd_hz: tuning.atc_rat_fltd_hz,
            ground_thrust_n: tuning.ground_thrust_g * MASS * GRAVITY_MSS,
            axes: [
                Axis::new(p0, i0, d0, m0),
                Axis::new(p1, i1, d1, m1),
                Axis::new(p2, i2, d2, m2),
            ],
            t_prev_us: None,
            posz_p: tuning.psc_posz_p,
            velz_p: tuning.psc_velz_p,
            velz_i: tuning.psc_velz_i,
            velz_imax: tuning.psc_velz_imax,
            speed_up_ms: tuning.pilot_speed_up_ms,
            speed_dn_ms: tuning.pilot_speed_dn_ms,
            tilt_cos_min: tuning.thr_tilt_cos_min,
            tilt_inverted_gain: tuning.thr_inverted_gain,
            thrust_full_n: full_thrust_n(tuning.pilot_thrust_max_g),
            alt_i: 0.0,
            alt_active: false,
            posxy_p: tuning.psc_posxy_p,
            velxy_p: tuning.psc_velxy_p,
            velxy_i: tuning.psc_velxy_i,
            speed_xy_ms: tuning.pilot_speed_xy_ms,
            angle_max_rad: tuning.angle_max_rad,
            a_max: GRAVITY_MSS * libm::tanf(tuning.angle_max_rad),
            xy_i: [0.0; 2],
            pos_active: false,
            q_sp_pos: [0.0; 4],
            notch: Notch::new(tuning, fs_hz),
            notch_on: false,
            rate_log: RateLog::NONE,
            #[cfg(test)]
            antiwindup_off: false,
            #[cfg(test)]
            position_clears_alt_active: false,
            #[cfg(test)]
            rotation_flipped: false,
            #[cfg(test)]
            entry_keeps_integrator: false,
        }
    }

    /// The collective of altitude hold, N. Signs: NED, down positive for height D, climb rate vD,
    /// acceleration a_d and the integrator; the thrust is positive up. `down_m` and `v_ff` are the shaper's target
    /// height and its climb-rate feed-forward (down positive); `a_ff` the acceleration feed-forward, m/s^2 down positive
    /// (the vertical carrot's; 0 for `Altitude`). `dt_s` is 0 on the first step after `reset`.
    fn vertical_thrust(
        &mut self,
        nav: &NavState,
        down_m: f32,
        v_ff: f32,
        a_ff: f32,
        dt_s: f32,
        limited: &Saturation,
    ) -> f32 {
        // The first `Altitude` setpoint after any other setpoint or `reset`: integrator 0, and it does not advance in
        // this step (the entry thrust is the weight over the tilt factor).
        let entry = !self.alt_active;
        if entry {
            self.alt_i = 0.0;
            self.alt_active = true;
        }
        let (d, v_d) = (nav.position_m[2], nav.velocity_ms[2]);
        // vD_sp = P (D_target - D) + v_ff, down positive, limited to the up and down speeds. D_target below D (greater)
        // asks for a descent (positive vD_sp).
        let v_sp = (self.posz_p * (down_m - d) + v_ff)
            .max(-self.speed_up_ms)
            .min(self.speed_dn_ms);
        // e_v > 0: the vehicle moves up or hovers where it should move down more; a_d > 0 asks to accelerate down.
        let e_v = v_sp - v_d;
        let a_d = self.velz_p * e_v + self.alt_i + a_ff;
        // Thrust, N, positive up: m (g - a_d) holds at a_d = 0 and falls when a_d > 0; then the tilt factor.
        let cos = down_in_body(nav.attitude)[2];
        let wanted = MASS * (GRAVITY_MSS - a_d) * tilt_factor(cos, self.tilt_cos_min, self.tilt_inverted_gain);
        let thrust_n = wanted.max(0.0).min(self.thrust_full_n);
        // No integration into a saturated thrust, from either source: our own clamp or the allocator's flags.
        let at_min = wanted <= 0.0 || limited.thrust_low;
        let at_max = wanted >= self.thrust_full_n || limited.thrust_high;
        let frozen = (at_min && e_v >= 0.0) || (at_max && e_v <= 0.0);
        #[cfg(test)]
        let frozen = frozen && !self.antiwindup_off;
        if !entry && !frozen {
            self.alt_i += self.velz_i * e_v * dt_s;
        }
        self.alt_i = self.alt_i.max(-self.velz_imax).min(self.velz_imax);
        thrust_n
    }

    /// Tests only: the vertical integrator, m/s^2.
    #[cfg(test)]
    pub(crate) fn vertical_integrator(&self) -> f32 {
        self.alt_i
    }

    /// Tests only: the largest horizontal velocity setpoint in force, m/s.
    #[cfg(test)]
    pub(crate) fn speed_limit_xy(&self) -> f32 {
        self.speed_xy_ms
    }

    /// Tests only: the horizontal integrator, m/s^2, north and east.
    #[cfg(test)]
    pub(crate) fn horizontal_integrator(&self) -> [f32; 2] {
        self.xy_i
    }

    /// The attitude setpoint of position hold: `target` and `v_ff` the setpoint's NED position and
    /// velocity feed-forward, `heading_rad` its heading. v_sp = posxy_p (target - p) + v_ff limited to the speed;
    /// e_v = v_sp - v; a = velxy_p e_v + I limited to `a_max`; the integrator adds velxy_i e_v dt unless the acceleration
    /// was clipped and it would grow, and stays within `a_max`. `entry`: the first `Position` setpoint, the integrator is
    /// 0 and does not advance. The tilt is atan(|a_h|/g) about (a_y, -a_x, 0) / |a_h|, a_h = a in the heading frame.
    fn horizontal(
        &mut self,
        nav: &NavState,
        target: [f32; 3],
        v_ff: [f32; 3],
        heading_rad: f32,
        dt_s: f32,
        entry: bool,
    ) -> [f32; 4] {
        #[cfg(test)]
        let keep = self.entry_keeps_integrator;
        #[cfg(not(test))]
        let keep = false;
        if entry && !keep {
            self.xy_i = [0.0; 2];
        }
        let [pn, pe, _] = nav.position_m;
        let [vn, ve, _] = nav.velocity_ms;
        let (v_sp, _) = limit_length(
            [
                self.posxy_p * (target[0] - pn) + v_ff[0],
                self.posxy_p * (target[1] - pe) + v_ff[1],
            ],
            self.speed_xy_ms,
        );
        let e_v = [v_sp[0] - vn, v_sp[1] - ve];
        let [i_n, i_e] = self.xy_i;
        let (a, clipped) = limit_length(
            [self.velxy_p * e_v[0] + i_n, self.velxy_p * e_v[1] + i_e],
            self.a_max,
        );
        #[cfg(test)]
        let clipped = clipped && !self.antiwindup_off;
        // Conditional integration, Axis::step's rule: keep I' unless the acceleration was clipped and I' is longer.
        let next = [
            i_n + self.velxy_i * e_v[0] * dt_s,
            i_e + self.velxy_i * e_v[1] * dt_s,
        ];
        let longer = next[0] * next[0] + next[1] * next[1] > i_n * i_n + i_e * i_e;
        if !entry && !(clipped && longer) {
            self.xy_i = next;
        }
        self.xy_i = limit_length(self.xy_i, self.a_max).0;
        // a in the heading frame: Rz(-heading) a.
        let (s, c) = (libm::sinf(heading_rad), libm::cosf(heading_rad));
        #[cfg(test)]
        let s = if self.rotation_flipped { -s } else { s };
        let (ax, ay) = (c * a[0] + s * a[1], -s * a[0] + c * a[1]);
        let n = libm::sqrtf(ax * ax + ay * ay);
        let tilt = if n > 0.0 {
            let theta = libm::atanf(n / GRAVITY_MSS);
            let half = (if theta > self.angle_max_rad { self.angle_max_rad } else { theta }) / 2.0;
            let k = libm::sinf(half) / n;
            [libm::cosf(half), k * ay, -k * ax, 0.0]
        } else {
            [1.0, 0.0, 0.0, 0.0]
        };
        let h = heading_rad / 2.0;
        mul([libm::cosf(h), 0.0, 0.0, libm::sinf(h)], tilt)
    }

    /// Body rate setpoint, rad/s, from an attitude setpoint: q_e = conj(q) (x) q_sp with w >= 0,
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
        let is_position = matches!(*sp, Setpoint::Position { .. });
        let entry_xy = is_position && !self.pos_active;
        self.pos_active = is_position;
        self.q_sp_pos = [0.0; 4];
        // dt in s from successive nav.t_us, capped; 0 on the first step after reset.
        let first = self.t_prev_us.is_none();
        let dt = match self.t_prev_us {
            Some(prev) => nav.t_us.saturating_sub(prev).min(DT_MAX_US) as f32 * 1e-6,
            None => 0.0,
        };
        // The rate setpoint, the stick collective (None for Altitude and Position: the vertical loop makes it) and the
        // target height (down_m, velocity feed-forward).
        let (rate_sp, thrust_n, alt) = match *sp {
            Setpoint::Rates {
                rates_rads,
                thrust_n,
            } => (rates_rads, Some(thrust_n), None),
            Setpoint::Attitude {
                attitude,
                heading_rate_rads,
                thrust_n,
            } => (
                self.attitude_to_rates(nav.attitude, attitude, heading_rate_rads),
                Some(thrust_n),
                None,
            ),
            Setpoint::Altitude {
                attitude,
                heading_rate_rads,
                down_m,
                velocity_d_ms,
            } => (
                self.attitude_to_rates(nav.attitude, attitude, heading_rate_rads),
                None,
                Some((down_m, velocity_d_ms, 0.0)),
            ),
            Setpoint::Position {
                position_m,
                velocity_ms,
                heading_rad,
                heading_rate_rads,
                accel_d_mss,
            } => {
                // The vertical integrator stays live across Altitude and Position.
                #[cfg(test)]
                if self.position_clears_alt_active {
                    self.alt_active = false;
                }
                let q_sp = self.horizontal(nav, position_m, velocity_ms, heading_rad, dt, entry_xy);
                self.q_sp_pos = q_sp;
                (
                    self.attitude_to_rates(nav.attitude, q_sp, heading_rate_rads),
                    None,
                    Some((position_m[2], velocity_ms[2], accel_d_mss)),
                )
            }
            Setpoint::Waypoint { .. } => {
                self.alt_active = false;
                return Err(Refused);
            }
        };
        self.t_prev_us = Some(nav.t_us);
        let thrust_n = match (thrust_n, alt) {
            (_, Some((down_m, v_ff, a_ff))) => self.vertical_thrust(nav, down_m, v_ff, a_ff, dt, limited),
            (stick, None) => {
                self.alt_active = false;
                stick.unwrap_or(0.0)
            }
        };

        let [sx, sy, sz] = rate_sp;
        // The notch filters the measurement once; the rate error and the D filter both take its output.
        let rates = if self.notch_on {
            self.notch.apply(nav.rates_rads)
        } else {
            nav.rates_rads
        };
        let [mx, my, mz] = rates;
        let hz = self.fltd_hz;
        // On the ground (the collective below the threshold; in Altitude this step's computed, clamped collective):
        // the integrators are 0.
        let ground = thrust_n < self.ground_thrust_n;
        let [roll, pitch, yaw] = &mut self.axes;
        let [rp, ri, rd] = roll.step(sx - mx, mx, dt, first, limited.roll, ground, hz);
        let [pp, pi, pd] = pitch.step(sy - my, my, dt, first, limited.pitch, ground, hz);
        let [yp, yi, yd] = yaw.step(sz - mz, mz, dt, first, limited.yaw, ground, hz);
        self.rate_log = RateLog {
            target_rads: rate_sp,
            p_nm: [rp, pp, yp],
            i_nm: [ri, pi, yi],
            d_nm: [rd, pd, yd],
            rate_rads: rates,
            ..self.rate_log
        };
        Ok(ThrustTorque {
            thrust_n: thrust_n.max(0.0),
            torque_nm: [rp + ri + rd, pp + pi + pd, yp + yi + yd],
        })
    }

    fn reset(&mut self) {
        for axis in self.axes.iter_mut() {
            axis.i = 0.0;
            axis.m = 0.0;
        }
        self.t_prev_us = None;
        self.alt_i = 0.0;
        self.alt_active = false;
        self.xy_i = [0.0; 2];
        self.pos_active = false;
        self.q_sp_pos = [0.0; 4];
        self.notch.reset();
        self.rate_log = RateLog {
            f1_hz: self.rate_log.f1_hz,
            on: self.rate_log.on,
            ..RateLog::NONE
        };
    }

    fn set_speed_limit_xy(&mut self, ms: f32) {
        self.speed_xy_ms = ms;
    }

    fn set_gains(&mut self, gains: &Gains) {
        let Gains([rp, yaw, p0, p1, p2, i0, i1, i2, d0, d1, m0, m1, m2, fltd]) = *gains;
        self.k_rp = rp;
        self.k_y = yaw;
        self.fltd_hz = fltd;
        let [a0, a1, a2] = &mut self.axes;
        (a0.kp, a0.ki, a0.kd, a0.imax) = (p0, i0, d0, m0);
        (a1.kp, a1.ki, a1.kd, a1.imax) = (p1, i1, d1, m1);
        (a2.kp, a2.ki, a2.imax) = (p2, i2, m2);
    }

    fn attitude_setpoint(&self) -> [f32; 4] {
        self.q_sp_pos
    }

    fn set_rotor_hz(&mut self, f1_hz: [Option<f32>; LINES], on: bool) {
        self.notch_on = on;
        self.rate_log.on = on;
        self.rate_log.f1_hz = f1_hz.map(|f| usable(f).unwrap_or(0.0));
        if on {
            self.notch.set_rotor_hz(f1_hz);
        }
    }

    fn rate_log(&self) -> RateLog {
        self.rate_log
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::gnc::Valid;
    use std::vec::Vec;

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
            thrust_n: MASS * GRAVITY_MSS,
        }
    }

    #[test]
    fn integrator_holds_when_its_axis_is_limited() {
        let tuning = Tuning::DEFAULT;
        let imax = tuning.atc_rat_imax[0];
        let mut pid = CascadedPid::new(&tuning, 800.0);
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
        let waypoint = Setpoint::Waypoint {
            position_m: [0.0; 3],
            heading_rad: 0.0,
        };
        assert_eq!(
            pid.update(&nav(t), &waypoint, &limited(false)),
            Err(Refused)
        );
    }

    #[test]
    fn integrator_zero_below_ground_thrust() {
        let tuning = Tuning::DEFAULT;
        // The threshold must cover every stance window (0.5/0.7 of the weight) and stay below the lift-off of the
        // strongest spread point (1/1.3 of the weight), for a thrust spread of +/-30 %.
        assert!(
            0.5_f32 / 0.7_f32 <= tuning.ground_thrust_g
                && tuning.ground_thrust_g < 1.0_f32 / 1.3_f32,
            "ground_thrust_g {} outside [0.5/0.7, 1/1.3)",
            tuning.ground_thrust_g
        );
        let threshold = tuning.ground_thrust_g * MASS * GRAVITY_MSS;
        let below = threshold * 0.999;
        let above = threshold * 1.001;
        assert!(below < threshold && threshold < above);
        let setpoint = |thrust_n: f32| Setpoint::Rates {
            rates_rads: [1.0, -0.5, 0.25],
            thrust_n,
        };
        let mut pid = CascadedPid::new(&tuning, 800.0);
        let mut t = 0;
        let mut step = |pid: &mut CascadedPid, thrust_n: f32| {
            let out = pid
                .update(&nav(t), &setpoint(thrust_n), &limited(false))
                .unwrap();
            t += PERIOD_US;
            out.torque_nm
        };
        let p_part = [
            tuning.atc_rat_p[0] * 1.0,
            tuning.atc_rat_p[1] * -0.5,
            tuning.atc_rat_p[2] * 0.25,
        ];
        // Below: a constant error for 1000 steps leaves every integrator 0 and the torque is P (D is 0, the rate is 0).
        for _ in 0..1000 {
            let torque = step(&mut pid, below);
            assert_eq!(torque, p_part);
            for axis in &pid.axes {
                assert_eq!(axis.i, 0.0);
            }
        }
        // Above: the same error grows every integrator.
        for _ in 0..100 {
            let before = [pid.axes[0].i, pid.axes[1].i, pid.axes[2].i];
            step(&mut pid, above);
            assert!(pid.axes[0].i > before[0], "roll I did not grow");
            assert!(pid.axes[1].i < before[1], "pitch I did not grow");
            assert!(pid.axes[2].i > before[2], "yaw I did not grow");
        }
        assert!(pid.axes.iter().all(|axis| axis.i != 0.0));
        // Back below: 0 on the first step, and the torque has no I part.
        let torque = step(&mut pid, below);
        assert_eq!(torque, p_part);
        for axis in &pid.axes {
            assert_eq!(axis.i, 0.0);
        }
    }

    // ---- Altitude hold: the vertical loop ----

    /// Level, at height `d` and climb rate `v_d` (both down positive), Altitude covered.
    fn alt_nav(t_us: u64, d: f32, v_d: f32) -> NavState {
        let mut n = nav(t_us);
        n.position_m = [0.0, 0.0, d];
        n.velocity_ms = [0.0, 0.0, v_d];
        n.valid.velocity_d = true;
        n.valid.position_d = true;
        n
    }

    /// Altitude setpoint at target `down_m`, level, with the feed-forward `v_ff` (down positive).
    fn alt(down_m: f32, v_ff: f32) -> Setpoint {
        Setpoint::Altitude {
            attitude: [1.0, 0.0, 0.0, 0.0],
            heading_rate_rads: 0.0,
            down_m,
            velocity_d_ms: v_ff,
        }
    }

    fn flags(low: bool, high: bool) -> Saturation {
        Saturation {
            thrust_low: low,
            thrust_high: high,
            ..limited(false)
        }
    }

    /// Runs `steps` Altitude steps at the target D = 0 on a vehicle at D = 0 with climb rate `v_d`; the first step is
    /// the entry (integrator 0, no advance). Returns (integrator, last thrust N).
    fn run_alt(pid: &mut CascadedPid, v_ff: f32, v_d: f32, sat: Saturation, steps: u32) -> (f32, f32) {
        let mut thrust = 0.0;
        for k in 0..=steps {
            let t = u64::from(k) * PERIOD_US;
            thrust = pid
                .update(&alt_nav(t, 0.0, v_d), &alt(0.0, v_ff), &sat)
                .unwrap()
                .thrust_n;
        }
        (pid.vertical_integrator(), thrust)
    }

    #[test]
    fn altitude_thrust_is_the_weight_at_rest_and_follows_the_velocity_law() {
        let t = Tuning::DEFAULT;
        let hover = MASS * GRAVITY_MSS;
        let mut pid = CascadedPid::new(&t, 800.0);
        // Entry, level, at the target, at rest: m g exactly.
        let out = pid.update(&alt_nav(0, 3.0, 0.0), &alt(3.0, 0.0), &limited(false)).unwrap();
        assert_eq!(out.thrust_n, hover);
        assert_eq!(pid.vertical_integrator(), 0.0);
        // Next step (integrator still 0 at dt > 0 with e_v = 0): the height error alone. Target 0.2 m below (down
        // positive): vD_sp = posz_p x 0.2, within the descent limit, thrust = m (g - velz_p vD_sp).
        let err = 0.2;
        let out = pid
            .update(&alt_nav(PERIOD_US, 3.0, 0.0), &alt(3.0 + err, 0.0), &limited(false))
            .unwrap();
        let v_sp = t.psc_posz_p * err;
        assert!(v_sp < t.pilot_speed_dn_ms);
        let want = MASS * (GRAVITY_MSS - t.psc_velz_p * v_sp);
        assert!((out.thrust_n - want).abs() <= 1e-4 * hover, "{} vs {want}", out.thrust_n);
        // A target far below saturates the setpoint at the descent speed, far above at the climb speed.
        let mut pid = CascadedPid::new(&t, 800.0);
        pid.update(&alt_nav(0, 0.0, 0.0), &alt(0.0, 0.0), &limited(false)).unwrap();
        let out = pid.update(&alt_nav(PERIOD_US, 0.0, 0.0), &alt(100.0, 0.0), &limited(false)).unwrap();
        let want = MASS * (GRAVITY_MSS - t.psc_velz_p * t.pilot_speed_dn_ms);
        assert!((out.thrust_n - want).abs() <= 1e-3 * hover);
        let out = pid.update(&alt_nav(2 * PERIOD_US, 0.0, 0.0), &alt(-100.0, 0.0), &limited(false)).unwrap();
        let want = (MASS * (GRAVITY_MSS + t.psc_velz_p * t.pilot_speed_up_ms))
            .min(full_thrust_n(t.pilot_thrust_max_g));
        assert!((out.thrust_n - want).abs() <= 1e-3 * hover);
    }

    #[test]
    fn the_tilt_factor_at_each_cosine_and_the_thrust_it_gives() {
        let t = Tuning::DEFAULT;
        let hover = MASS * GRAVITY_MSS;
        let full = full_thrust_n(t.pilot_thrust_max_g);
        // clamp(10 cos, 0, 1) / clamp(cos, 0.1, 1)
        let table = [(1.0, 1.0), (0.5, 2.0), (0.1, 10.0), (0.05, 5.0), (0.0, 0.0), (-0.1, 0.0)];
        for (cos, factor) in table {
            let (cos_min, gain) = (t.thr_tilt_cos_min, t.thr_inverted_gain);
            assert!((tilt_factor(cos, cos_min, gain) - factor).abs() <= 1e-5, "cos {cos}: {}", tilt_factor(cos, cos_min, gain));
            // Through the controller: tilted about x by acos(cos), at rest at the target, a_d = 0.
            let a = libm::acosf(cos);
            let mut n = alt_nav(0, 0.0, 0.0);
            n.attitude = [libm::cosf(a / 2.0), libm::sinf(a / 2.0), 0.0, 0.0];
            let mut pid = CascadedPid::new(&t, 800.0);
            let out = pid.update(&n, &alt(0.0, 0.0), &limited(false)).unwrap();
            let want = (hover * factor).min(full);
            assert!((out.thrust_n - want).abs() <= 1e-3 * hover, "cos {cos}: {} vs {want}", out.thrust_n);
            assert!(out.thrust_n >= 0.0 && out.thrust_n <= full);
        }
    }

    #[test]
    fn anti_windup_freezes_the_integrator_for_each_flag_and_direction() {
        let t = Tuning::DEFAULT;
        // (low flag, high flag, v_ff, v_d, frozen, sign of the integrator when it moves). e_v = v_ff - v_d.
        let cases = [
            ("low flag, e_v >= 0", true, false, 1.0, 0.0, true, 1.0),
            ("low flag, e_v < 0", true, false, -1.0, 0.0, false, -1.0),
            ("high flag, e_v <= 0", false, true, -1.0, 0.0, true, -1.0),
            ("high flag, e_v > 0", false, true, 1.0, 0.0, false, 1.0),
        ];
        for (name, low, high, v_ff, v_d, frozen, sign) in cases {
            let mut pid = CascadedPid::new(&t, 800.0);
            let (i, _) = run_alt(&mut pid, v_ff, v_d, flags(low, high), 200);
            if frozen {
                assert_eq!(i, 0.0, "{name}");
                // Control: with the rule off the integrator grows in that direction.
                let mut off = CascadedPid::new(&t, 800.0);
                off.antiwindup_off = true;
                let (i, _) = run_alt(&mut off, v_ff, v_d, flags(low, high), 200);
                assert!(i * sign > 0.0, "{name}: control {i}");
            } else {
                assert!(i * sign > 0.0, "{name}: {i}");
            }
        }
        // Our own clamp, no flags: thrust at 0 with e_v >= 0 (the setpoint at the descent speed, climbing at it too).
        let full = full_thrust_n(t.pilot_thrust_max_g);
        let mut pid = CascadedPid::new(&t, 800.0);
        let (i, thrust) = run_alt(&mut pid, t.pilot_speed_dn_ms, -t.pilot_speed_dn_ms, flags(false, false), 200);
        assert_eq!(thrust, 0.0);
        assert_eq!(i, 0.0, "own minimum clamp");
        let mut off = CascadedPid::new(&t, 800.0);
        off.antiwindup_off = true;
        assert!(run_alt(&mut off, t.pilot_speed_dn_ms, -t.pilot_speed_dn_ms, flags(false, false), 200).0 > 0.0);
        // Our own clamp at the maximum with e_v <= 0: descending fast at a target that holds.
        let mut pid = CascadedPid::new(&t, 800.0);
        let (i, thrust) = run_alt(&mut pid, 0.0, 3.0 * t.pilot_speed_dn_ms, flags(false, false), 200);
        assert_eq!(thrust, full);
        assert_eq!(i, 0.0, "own maximum clamp");
        let mut off = CascadedPid::new(&t, 800.0);
        off.antiwindup_off = true;
        assert!(run_alt(&mut off, 0.0, 3.0 * t.pilot_speed_dn_ms, flags(false, false), 200).0 < 0.0);
    }

    #[test]
    fn the_integrator_starts_at_0_at_each_entry_and_stays_within_its_limit() {
        let t = Tuning::DEFAULT;
        let mut pid = CascadedPid::new(&t, 800.0);
        // Wind it (descending slowly at a held target: e_v < 0, a thrust inside the limits).
        let (i, thrust) = run_alt(&mut pid, 0.0, 0.5, flags(false, false), 400);
        assert!(i < 0.0 && thrust > 0.0);
        // Any other setpoint, then Altitude again: the integrator is 0 after the first Altitude step.
        pid.update(&nav(10_000), &rates(0.0), &limited(false)).unwrap();
        let out = pid.update(&alt_nav(10_000 + PERIOD_US, 0.0, 0.5), &alt(0.0, 0.0), &flags(false, false)).unwrap();
        assert_eq!(pid.vertical_integrator(), 0.0);
        // The entry thrust has no integrator part: e_v = 0 - 0.5, a_d = velz_p e_v.
        let want = MASS * (GRAVITY_MSS - t.psc_velz_p * -0.5);
        assert!((out.thrust_n - want).abs() <= 1e-4 * want, "{} vs {want}", out.thrust_n);
        // reset() does the same.
        run_alt(&mut pid, 0.0, 0.5, flags(false, false), 400);
        assert!(pid.vertical_integrator() < 0.0);
        pid.reset();
        assert_eq!(pid.vertical_integrator(), 0.0);
        let (_, _) = run_alt(&mut pid, 0.0, 0.5, flags(false, false), 0);
        assert_eq!(pid.vertical_integrator(), 0.0);
        // The limit: a wound-up value beyond psc_velz_imax is clamped on the next step.
        pid.alt_i = 10.0 * t.psc_velz_imax;
        pid.update(&alt_nav(20_000, 0.0, 0.0), &alt(0.0, 0.0), &flags(false, false)).unwrap();
        assert_eq!(pid.vertical_integrator(), t.psc_velz_imax);
        pid.alt_i = -10.0 * t.psc_velz_imax;
        pid.update(&alt_nav(21_250, 0.0, 0.0), &alt(0.0, 0.0), &flags(false, false)).unwrap();
        assert_eq!(pid.vertical_integrator(), -t.psc_velz_imax);
    }

    #[test]
    fn the_ground_rule_in_altitude_uses_this_steps_clamped_thrust() {
        let t = Tuning::DEFAULT;
        let ground = t.ground_thrust_g * MASS * GRAVITY_MSS;
        // A roll error (the setpoint tilted, the vehicle level) makes the roll integrator grow unless on the ground.
        let tilted = |v_ff: f32| Setpoint::Altitude {
            attitude: [libm::cosf(0.05), libm::sinf(0.05), 0.0, 0.0],
            heading_rate_rads: 0.0,
            down_m: 0.0,
            velocity_d_ms: v_ff,
        };
        // Descent demanded at the limit: a_d = velz_p x speed, thrust m (g - a_d), below the ground threshold.
        let low = MASS * (GRAVITY_MSS - t.psc_velz_p * t.pilot_speed_dn_ms);
        assert!(low < ground);
        let mut pid = CascadedPid::new(&t, 800.0);
        for k in 0..200u64 {
            let out = pid.update(&alt_nav(k * PERIOD_US, 0.0, 0.0), &tilted(t.pilot_speed_dn_ms), &limited(false)).unwrap();
            assert!(out.thrust_n < ground);
            assert_eq!(pid.axes[0].i, 0.0, "step {k}");
        }
        // At the hover thrust (above the threshold) it grows.
        let mut pid = CascadedPid::new(&t, 800.0);
        for k in 0..200u64 {
            pid.update(&alt_nav(k * PERIOD_US, 0.0, 0.0), &tilted(0.0), &limited(false)).unwrap();
        }
        assert!(pid.axes[0].i > 0.0);
    }
    // ---- Position hold: the horizontal loop ----

    /// The linear rotor drag of the real vehicle, N/(m/s) (the simulator card's air axis).
    const DRAG_N_PER_MS: f32 = 0.197;

    /// All fields valid, level at heading `yaw`, at `p` (NED, m) with velocity `v` (NED, m/s).
    fn pnav(t_us: u64, p: [f32; 3], v: [f32; 3], yaw: f32) -> NavState {
        let mut n = nav(t_us);
        n.attitude = [libm::cosf(yaw / 2.0), 0.0, 0.0, libm::sinf(yaw / 2.0)];
        n.position_m = p;
        n.velocity_ms = v;
        n.valid = Valid {
            tilt: true,
            heading: true,
            rates: true,
            velocity_ne: true,
            velocity_d: true,
            position_ne: true,
            position_d: true,
        };
        n
    }

    fn pos(target: [f32; 3], v_ff: [f32; 3], heading_rad: f32) -> Setpoint {
        Setpoint::Position {
            position_m: target,
            velocity_ms: v_ff,
            heading_rad,
            heading_rate_rads: 0.0,
            accel_d_mss: 0.0,
        }
    }

    /// The horizontal acceleration (north, east) m/s^2 of a point mass held at the weight's vertical share by an
    /// attitude exactly `q`: g tan(tilt) along the thrust vector's horizontal part, from R(q)'s third column only.
    fn accel_ne(q: [f32; 4]) -> [f32; 2] {
        let [w, x, y, z] = q;
        let col = [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), w * w - x * x - y * y + z * z];
        [-GRAVITY_MSS * col[0] / col[2], -GRAVITY_MSS * col[1] / col[2]]
    }

    fn tilt_of(q: [f32; 4]) -> f32 {
        let [w, x, y, z] = q;
        libm::acosf((w * w - x * x - y * y + z * z).min(1.0))
    }

    /// A point mass on a plane under the controller: ideal attitude (the vehicle takes the attitude setpoint at once),
    /// the vertical axis held, plus `extra(v)` the acceleration of the air on the mass. Starts at `p0` at rest, heading
    /// `yaw`, target the origin. Returns (position, the controller) after `seconds`.
    fn point_mass(
        pid: &mut CascadedPid,
        yaw: f32,
        p0: [f32; 2],
        seconds: f32,
        extra: impl Fn([f32; 2]) -> [f32; 2],
    ) -> [f32; 2] {
        let dt = PERIOD_US as f32 * 1e-6;
        let (mut p, mut v) = (p0, [0.0_f32; 2]);
        for k in 0..(seconds / dt) as u64 {
            let t = k * PERIOD_US;
            pid.update(
                &pnav(t, [p[0], p[1], 0.0], [v[0], v[1], 0.0], yaw),
                &pos([0.0; 3], [0.0; 3], yaw),
                &limited(false),
            )
            .unwrap();
            let a = accel_ne(pid.attitude_setpoint());
            let e = extra(v);
            v = [v[0] + (a[0] + e[0]) * dt, v[1] + (a[1] + e[1]) * dt];
            p = [p[0] + v[0] * dt, p[1] + v[1] * dt];
        }
        p
    }

    const YAWS: [f32; 4] = [
        0.0,
        core::f32::consts::FRAC_PI_2,
        core::f32::consts::PI,
        3.0 * core::f32::consts::FRAC_PI_2,
    ];

    /// The frames. The acceleration from the tilt is the commanded one for every heading and every direction of the
    /// command, and the sign of each tilt component is the Angle stick's (forward lowers the nose, right lowers the right
    /// side).
    #[test]
    fn n5_the_tilt_gives_the_commanded_acceleration_at_every_heading() {
        let t = Tuning::DEFAULT;
        // Commands as velocity feed-forward at the heading's frame, with the vehicle at rest at the target: a = velxy_p v_ff
        // (within the limits), in NED.
        let cmds: [[f32; 2]; 4] = [[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]];
        for yaw in YAWS {
            for c in cmds {
                let mut pid = CascadedPid::new(&t, 800.0);
                pid.update(&pnav(0, [0.0; 3], [0.0; 3], yaw), &pos([0.0; 3], [c[0], c[1], 0.0], yaw), &limited(false)).unwrap();
                let a = accel_ne(pid.attitude_setpoint());
                let want = [t.psc_velxy_p * c[0], t.psc_velxy_p * c[1]];
                near_f(a[0], want[0], 1e-3);
                near_f(a[1], want[1], 1e-3);
                // The body-frame tilt: the heading-frame command Rz(-yaw) c.
                let (s, co) = (libm::sinf(yaw), libm::cosf(yaw));
                let (fwd, right) = (co * c[0] + s * c[1], -s * c[0] + co * c[1]);
                let q = pid.attitude_setpoint();
                let body = down_in_body(q);
                // The down vector in the body: x > 0 is the nose down (forward), y > 0 the right side down.
                if libm::fabsf(fwd) > 0.5 {
                    assert!(body[0] * fwd > 0.0, "yaw {yaw} cmd {c:?}: nose {body:?}");
                }
                if libm::fabsf(right) > 0.5 {
                    assert!(body[1] * right > 0.0, "yaw {yaw} cmd {c:?}: right side {body:?}");
                }
            }
        }
    }

    /// A point mass 2 m from the target at each of the four headings converges; with the rotation's sign flipped it
    /// diverges at 90 and 270 degrees.
    #[test]
    fn n5_a_point_mass_converges_from_2_m_at_every_heading_and_the_flipped_rotation_diverges() {
        let t = Tuning::DEFAULT;
        let norm = |p: [f32; 2]| libm::sqrtf(p[0] * p[0] + p[1] * p[1]);
        for (n, yaw) in YAWS.into_iter().enumerate() {
            let mut pid = CascadedPid::new(&t, 800.0);
            let end = point_mass(&mut pid, yaw, [1.2, 1.6], 40.0, |_| [0.0; 2]);
            std::println!("yaw {} deg: 2 m -> {:.5} m after 40 s", n * 90, norm(end));
            assert!(norm(end) < 0.01, "yaw {yaw}: {end:?}");
            // Control.
            let mut flipped = CascadedPid::new(&t, 800.0);
            flipped.rotation_flipped = true;
            let end = point_mass(&mut flipped, yaw, [1.2, 1.6], 10.0, |_| [0.0; 2]);
            std::println!("  control, rotation sign flipped: {:.3} m after 10 s", norm(end));
            if n % 2 == 1 {
                assert!(norm(end) > 2.0, "flipped at yaw {yaw} should diverge: {end:?}");
            } else {
                assert!(norm(end) < 2.0, "flipped at yaw {yaw} is the same rotation: {end:?}");
            }
        }
    }

    /// A point mass with the vehicle's linear drag in a steady wind. Without the integrator the offset
    /// settles at a_w / (posxy_p velxy_p), a_w the drag acceleration at rest in the wind; with it the offset goes to 0.
    #[test]
    fn n5_p1_drag_and_steady_wind_the_offset_without_the_integrator_and_zero_with_it() {
        let t = Tuning::DEFAULT;
        let c = DRAG_N_PER_MS / MASS;
        let norm = |p: [f32; 2]| libm::sqrtf(p[0] * p[0] + p[1] * p[1]);
        for (n, yaw) in YAWS.into_iter().enumerate() {
            for wind in [[5.0_f32, 0.0], [0.0, 5.0]] {
                let drag = |v: [f32; 2]| [-c * (v[0] - wind[0]), -c * (v[1] - wind[1])];
                let a_w = c * norm(wind);
                let want = a_w / (t.psc_posxy_p * t.psc_velxy_p);
                let mut no_i = t;
                no_i.psc_velxy_i = 0.0;
                let mut pid = CascadedPid::new(&no_i, 800.0);
                let end = point_mass(&mut pid, yaw, [0.0; 2], 80.0, drag);
                std::println!("yaw {} wind {wind:?}: no integrator offset {:.4} m (a_w/(kp kv) = {want:.4}); ", n * 90, norm(end));
                near_f(norm(end), want, 0.02 * want);
                // The offset is along the wind (downwind).
                assert!(end[0] * wind[0] + end[1] * wind[1] > 0.0);
                let mut pid = CascadedPid::new(&t, 800.0);
                let end = point_mass(&mut pid, yaw, [0.0; 2], 200.0, drag);
                std::println!("  with the integrator: {:.5} m after 200 s", norm(end));
                assert!(norm(end) < 0.01, "{end:?}");
            }
        }
    }

    fn near_f(a: f32, b: f32, tol: f32) {
        assert!((a - b).abs() <= tol, "{a} vs {b} (tol {tol})");
    }

    /// The tilt never exceeds `angle_max_rad`, the horizontal integrator never exceeds `a_max`, and it does not grow
    /// while the acceleration is clipped (control: the rule off lets it grow to the limit).
    #[test]
    fn n4_tilt_integrator_limit_and_anti_windup() {
        let t = Tuning::DEFAULT;
        let a_max = GRAVITY_MSS * libm::tanf(t.angle_max_rad);
        // A far target at rest: v_sp at the speed limit, a = velxy_p x speed > a_max, clipped at every step.
        let run = |off: bool| {
            let mut pid = CascadedPid::new(&t, 800.0);
            pid.antiwindup_off = off;
            let mut worst_tilt: f32 = 0.0;
            let mut worst_i: f32 = 0.0;
            for k in 0..16_000u64 {
                pid.update(&pnav(k * PERIOD_US, [0.0; 3], [0.0; 3], 0.4), &pos([100.0, -60.0, 0.0], [0.0; 3], 0.4), &limited(false)).unwrap();
                worst_tilt = worst_tilt.max(tilt_of(pid.attitude_setpoint()));
                let [i_n, i_e] = pid.horizontal_integrator();
                worst_i = worst_i.max(libm::sqrtf(i_n * i_n + i_e * i_e));
            }
            (worst_tilt, worst_i)
        };
        let (tilt, i) = run(false);
        std::println!("clipped for 20 s: peak tilt {:.4} deg (limit {:.4}), peak |I| {i:.4} (limit {a_max:.4})", tilt.to_degrees(), t.angle_max_rad.to_degrees());
        assert!(tilt <= t.angle_max_rad + 1e-5);
        assert_eq!(i, 0.0, "the integrator did not grow while clipped");
        // Control: the rule off. The integrator grows, and stops at the limit.
        let (tilt, i) = run(true);
        std::println!("control, anti-windup off: peak tilt {:.4} deg, peak |I| {i:.4}", tilt.to_degrees());
        assert!(tilt <= t.angle_max_rad + 1e-5);
        assert!(i > 0.5 * a_max && i <= a_max * (1.0 + 1e-5), "{i}");
        // It may shrink while clipped: I along north at 4 m/s^2, the vehicle 10 m/s too fast the other way round.
        let mut pid = CascadedPid::new(&t, 800.0);
        pid.update(&pnav(0, [0.0; 3], [10.0, 0.0, 0.0], 0.0), &pos([0.0; 3], [0.0; 3], 0.0), &limited(false)).unwrap();
        pid.xy_i = [4.0, 0.0];
        let mut last = 4.0;
        for k in 1..200u64 {
            pid.update(&pnav(k * PERIOD_US, [0.0; 3], [10.0, 0.0, 0.0], 0.0), &pos([0.0; 3], [0.0; 3], 0.0), &limited(false)).unwrap();
            let i = pid.horizontal_integrator()[0];
            assert!(i < last, "step {k}: {i} did not shrink from {last}");
            last = i;
        }
        // Random-ish grid: the tilt limit holds whatever the errors, at every heading.
        let mut worst: f32 = 0.0;
        for yaw in YAWS {
            for (px, py) in [(50.0, 0.0), (0.0, -50.0), (-3.0, 4.0), (0.0, 0.0)] {
                for (vx, vy) in [(0.0, 0.0), (15.0, 0.0), (-8.0, 12.0)] {
                    let mut pid = CascadedPid::new(&t, 800.0);
                    for k in 0..3u64 {
                        pid.update(&pnav(k * PERIOD_US, [0.0; 3], [vx, vy, 0.0], yaw), &pos([px, py, 0.0], [3.0, -2.0, 0.0], yaw), &limited(false)).unwrap();
                        worst = worst.max(tilt_of(pid.attitude_setpoint()));
                    }
                }
            }
        }
        assert!(worst <= t.angle_max_rad + 1e-5, "{worst}");
    }

    /// The velocity setpoint is limited to the speed in force (`set_speed_limit_xy`); with a far target at rest the
    /// acceleration is velxy_p x that speed (below the tilt limit for the slow values).
    #[test]
    fn the_velocity_setpoint_is_limited_to_the_speed_in_force() {
        let t = Tuning::DEFAULT;
        for speed in [3.0_f32, 3.1, t.pilot_speed_xy_ms] {
            let mut pid = CascadedPid::new(&t, 800.0);
            pid.set_speed_limit_xy(speed);
            pid.update(&pnav(0, [0.0; 3], [0.0; 3], 0.0), &pos([100.0, 0.0, 0.0], [0.0; 3], 0.0), &limited(false)).unwrap();
            let a = accel_ne(pid.attitude_setpoint());
            let want = (t.psc_velxy_p * speed).min(GRAVITY_MSS * libm::tanf(t.angle_max_rad));
            near_f(a[0], want, 1e-3);
        }
    }

    /// The first Position setpoint after any other one, or a reset, starts the horizontal integrator
    /// at 0 and does not advance it in that step (control: the integrator kept at the entry).
    #[test]
    fn n2_the_entry_zeroes_the_horizontal_integrator_and_does_not_advance_it() {
        let t = Tuning::DEFAULT;
        for keep in [false, true] {
            let mut pid = CascadedPid::new(&t, 800.0);
            pid.entry_keeps_integrator = keep;
            // Wind it: a steady 0.5 m/s velocity error.
            for k in 0..4_000u64 {
                pid.update(&pnav(k * PERIOD_US, [0.0; 3], [0.0; 3], 0.0), &pos([0.0; 3], [0.5, 0.0, 0.0], 0.0), &limited(false)).unwrap();
            }
            let wound = pid.horizontal_integrator();
            assert!(wound[0] > 0.5, "{wound:?}");
            let others: [(&str, Setpoint); 2] = [
                ("Altitude", alt(0.0, 0.0)),
                ("Attitude", Setpoint::Attitude { attitude: [1.0, 0.0, 0.0, 0.0], heading_rate_rads: 0.0, thrust_n: MASS * GRAVITY_MSS }),
            ];
            for (name, other) in others {
                // Leave Position for one step, then return with a velocity error (unclipped): the entry step does not advance.
                let mut pid2 = pid;
                pid2.update(&pnav(10_000_000, [0.0; 3], [0.0; 3], 0.0), &other, &limited(false)).unwrap();
                pid2.update(&pnav(10_001_250, [0.0; 3], [0.0; 3], 0.0), &pos([0.0; 3], [1.0, 0.0, 0.0], 0.0), &limited(false)).unwrap();
                if keep {
                    assert_eq!(pid2.horizontal_integrator(), wound, "control, {name}: kept");
                } else {
                    assert_eq!(pid2.horizontal_integrator(), [0.0, 0.0], "{name}: entry");
                    // The next step advances it.
                    pid2.update(&pnav(10_002_500, [0.0; 3], [0.0; 3], 0.0), &pos([0.0; 3], [1.0, 0.0, 0.0], 0.0), &limited(false)).unwrap();
                    assert!(pid2.horizontal_integrator()[0] > 0.0);
                }
            }
            // reset() is an entry too.
            if !keep {
                pid.reset();
                assert_eq!(pid.horizontal_integrator(), [0.0, 0.0]);
                pid.update(&pnav(0, [0.0; 3], [0.0; 3], 0.0), &pos([0.0; 3], [1.0, 0.0, 0.0], 0.0), &limited(false)).unwrap();
                assert_eq!(pid.horizontal_integrator(), [0.0, 0.0]);
            }
        }
    }

    /// Changing between Altitude and Position, either way, leaves the vertical integrator and the
    /// thrust bit-identical to staying in Altitude (control: the vertical integrator's live flag cleared in Position).
    #[test]
    fn n3_altitude_and_position_share_the_vertical_loop_bit_for_bit() {
        let t = Tuning::DEFAULT;
        let run = |clear: bool| {
            let mut stay = CascadedPid::new(&t, 800.0);
            let mut change = CascadedPid::new(&t, 800.0);
            change.position_clears_alt_active = clear;
            let mut same = true;
            for k in 0..3_000u64 {
                // Descending slowly at the held target: the vertical integrator winds.
                let n = pnav(k * PERIOD_US, [0.0, 0.0, -2.0], [0.0, 0.0, 0.5], 0.3);
                let a = stay.update(&n, &alt(-2.0, 0.1), &flags(false, false)).unwrap();
                // Position in the blocks of 500 steps from step 500, Altitude otherwise.
                let in_position = (k / 500) % 2 == 1;
                let b = if in_position {
                    change.update(&n, &pos([0.0, 0.0, -2.0], [0.0, 0.0, 0.1], 0.3), &flags(false, false)).unwrap()
                } else {
                    change.update(&n, &alt(-2.0, 0.1), &flags(false, false)).unwrap()
                };
                same &= a.thrust_n.to_bits() == b.thrust_n.to_bits()
                    && stay.vertical_integrator().to_bits() == change.vertical_integrator().to_bits();
            }
            assert!(stay.vertical_integrator() < 0.0);
            same
        };
        assert!(run(false), "the loops differ");
        assert!(!run(true), "control: clearing the flag must make them differ");
    }

    /// The Position setpoint's collective is the vertical loop's, as in Altitude, and the ground rule still applies.
    #[test]
    fn position_with_a_far_target_height_asks_the_same_thrust_as_altitude() {
        let t = Tuning::DEFAULT;
        let mut a = CascadedPid::new(&t, 800.0);
        let mut p = CascadedPid::new(&t, 800.0);
        let n = pnav(0, [0.0, 0.0, 0.0], [0.0; 3], 0.0);
        let ra = a.update(&n, &alt(1.0, 0.0), &limited(false)).unwrap();
        let rp = p.update(&n, &pos([0.0, 0.0, 1.0], [0.0; 3], 0.0), &limited(false)).unwrap();
        assert_eq!(ra.thrust_n.to_bits(), rp.thrust_n.to_bits());
    }

    /// A deterministic rate sequence with 120 Hz content, noise, and signed zeros: step `k` of 800 Hz.
    fn noisy_rates(k: u32, seed: &mut u32) -> [f32; 3] {
        let mut noise = || {
            *seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (*seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5
        };
        let t = k as f32 / 800.0;
        let tone = 0.4 * libm::sinf(core::f32::consts::TAU * 120.0 * t);
        if k % 97 == 0 {
            return [-0.0, 0.0, -0.0];
        }
        [0.3 * libm::sinf(2.0 * t) + tone + 0.05 * noise(), 0.2 * noise() + tone, 0.1 * noise()]
    }

    /// Runs `steps` of the noisy rates through `pid` with a roll rate setpoint, returning the torques and the rates the
    /// loop used.
    fn run_noisy(pid: &mut CascadedPid, steps: u32, each: impl Fn(&mut CascadedPid)) -> Vec<([f32; 3], [f32; 3])> {
        let mut seed = 7;
        let mut out = Vec::new();
        for k in 0..steps {
            each(pid);
            let mut n = nav(u64::from(k) * PERIOD_US);
            n.rates_rads = noisy_rates(k, &mut seed);
            let sp = rates(0.5 * libm::sinf(k as f32 * 0.01));
            let torque = pid.update(&n, &sp, &limited(false)).unwrap().torque_nm;
            out.push((torque, pid.rate_log().rate_rads));
        }
        out
    }

    /// With the switch off, with no usable eRPM, with every h f1 below 50 Hz, with an eRPM of 0: the torques and the rate
    /// the loop used are bit-identical to the controller that never had the notch set. Control: a live notch differs.
    #[test]
    fn a_notch_with_no_weight_leaves_the_torques_bit_identical() {
        let t = Tuning::DEFAULT;
        let base = run_noisy(&mut CascadedPid::new(&t, 800.0), 3000, |_| {});
        let same = |f1: [Option<f32>; LINES], on: bool| {
            let got = run_noisy(&mut CascadedPid::new(&t, 800.0), 3000, |p| p.set_rotor_hz(f1, on));
            got.iter().zip(base.iter()).all(|(a, b)| {
                a.0.iter().zip(b.0.iter()).all(|(x, y)| x.to_bits() == y.to_bits())
                    && a.1.iter().zip(b.1.iter()).all(|(x, y)| x.to_bits() == y.to_bits())
            })
        };
        assert!(same([Some(100.0), Some(110.0), Some(120.0), Some(130.0)], false), "switch off");
        assert!(same([None; LINES], true), "all eRPM missing");
        assert!(same([Some(16.6); LINES], true), "every h f1 below 50 Hz");
        assert!(same([Some(0.0); LINES], true), "eRPM 0");
        assert!(same([Some(f32::NAN), Some(-3.0), None, Some(0.0)], true), "unusable values");
        assert!(!same([Some(100.0), Some(110.0), Some(120.0), Some(130.0)], true), "control: a live notch must differ");
        assert!(!same([Some(100.0), None, None, None], true), "control: one live line must differ");
    }

    /// `rate_log` carries the usable frequencies (0 for the rest), the switch, the setpoint and the three terms, which
    /// sum to the torque; `reset` clears the loop's part and keeps the frequencies and the switch.
    #[test]
    fn rate_log_reports_the_frequencies_the_switch_and_the_loop() {
        let t = Tuning::DEFAULT;
        let mut pid = CascadedPid::new(&t, 800.0);
        pid.set_rotor_hz([Some(100.0), None, Some(f32::NAN), Some(0.0)], true);
        let mut n = nav(0);
        n.rates_rads = [0.25, -0.5, 1.0];
        pid.update(&n, &rates(2.0), &limited(false)).unwrap();
        let log = pid.rate_log();
        assert_eq!(log.f1_hz, [100.0, 0.0, 0.0, 0.0]);
        assert!(log.on);
        assert_eq!(log.target_rads, [2.0, 0.0, 0.0]);
        // The first sample starts the filter's memory, so the rate used is the measurement.
        assert_eq!(log.rate_rads, [0.25, -0.5, 1.0]);
        let torque = pid.update(&nav(PERIOD_US), &rates(2.0), &limited(false)).unwrap().torque_nm;
        let log = pid.rate_log();
        for axis in 0..3 {
            assert_eq!((log.p_nm[axis] + log.i_nm[axis]) + log.d_nm[axis], torque[axis]);
        }
        pid.reset();
        let log = pid.rate_log();
        assert_eq!((log.f1_hz, log.on), ([100.0, 0.0, 0.0, 0.0], true), "reset keeps the frequencies and the switch");
        assert_eq!((log.target_rads, log.p_nm, log.i_nm, log.d_nm, log.rate_rads), ([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]));
    }
}
