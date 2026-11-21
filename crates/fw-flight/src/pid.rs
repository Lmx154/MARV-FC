//! The controller: an attitude P loop feeding a PID rate loop per axis (decision-first-pair-20261119, section 3),
//! and, for `Setpoint::Altitude`, a vertical loop that makes the collective (decision-nav-eskf-20261120, 7b, F4, F5).
//! Torques in N.m about FRD, rates in rad/s, collective thrust in N. No feed-forward in the attitude and rate loops.

use crate::gnc::{Controller, NavState, Refused, Saturation, Setpoint, ThrustTorque};
use crate::mahony::DT_MAX_US;
use crate::quat::{conj, down_in_body, mul, pt1};
use crate::tuning::{Tuning, full_thrust_n};
use fw_core::calib::GRAVITY_MSS;
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

    /// Torque, N.m, for rate error `err` and measured `rate`, rad/s, over `dt_s`. `first`: the first step after a
    /// reset (dt 0, the D filter starts at the measurement). `limited`: this axis's flag from the previous allocation.
    /// `ground`: the integrator is set to 0 and adds nothing to this step's torque.
    fn step(
        &mut self,
        err: f32,
        rate: f32,
        dt_s: f32,
        first: bool,
        limited: bool,
        ground: bool,
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
        if ground {
            self.i = 0.0;
        } else {
            let i_next = self.i + self.ki * err * dt_s;
            if !(limited && i_next.abs() > self.i.abs()) {
                self.i = i_next;
            }
            self.i = self.i.max(-self.imax).min(self.imax);
        }
        self.kp * err + self.i + d
    }
}

/// The cascaded PID. Accepts `Setpoint::Rates`, `Setpoint::Attitude` and `Setpoint::Altitude`; refuses the rest.
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
    /// The previous setpoint was `Altitude`: the integrator is live. False after `reset` and after any other
    /// setpoint, so the first `Altitude` setpoint starts the integrator at 0.
    alt_active: bool,
    /// Tests only: the anti-windup rule is off.
    #[cfg(test)]
    antiwindup_off: bool,
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
            #[cfg(test)]
            antiwindup_off: false,
        }
    }

    /// The collective of altitude hold, N (F1, F4, F5). Signs: NED, down positive for height D, climb rate vD,
    /// acceleration a_d and the integrator; the thrust is positive up. `down_m` and `v_ff` are the shaper's target
    /// height and its climb-rate feed-forward (down positive). `dt_s` is 0 on the first step after `reset`.
    fn vertical_thrust(
        &mut self,
        nav: &NavState,
        down_m: f32,
        v_ff: f32,
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
        let a_d = self.velz_p * e_v + self.alt_i;
        // Thrust, N, positive up: m (g - a_d) holds at a_d = 0 and falls when a_d > 0; then the tilt factor (F4).
        let cos = down_in_body(nav.attitude)[2];
        let wanted = MASS * (GRAVITY_MSS - a_d) * tilt_factor(cos, self.tilt_cos_min, self.tilt_inverted_gain);
        let thrust_n = wanted.max(0.0).min(self.thrust_full_n);
        // F5: no integration into a saturated thrust, from either source: our own clamp or the allocator's flags.
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
        // The rate setpoint, the stick collective (None for Altitude: the vertical loop makes it) and the Altitude
        // target (down_m, velocity feed-forward).
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
                Some((down_m, velocity_d_ms)),
            ),
            Setpoint::Position { .. } | Setpoint::Waypoint { .. } => {
                self.alt_active = false;
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
        let thrust_n = match (thrust_n, alt) {
            (_, Some((down_m, v_ff))) => self.vertical_thrust(nav, down_m, v_ff, dt, limited),
            (stick, None) => {
                self.alt_active = false;
                stick.unwrap_or(0.0)
            }
        };

        let [sx, sy, sz] = rate_sp;
        let [mx, my, mz] = nav.rates_rads;
        let hz = self.fltd_hz;
        // On the ground (the collective below the threshold; in Altitude this step's computed, clamped collective):
        // the integrators are 0 (decision-flight-ground-20261119, section 1).
        let ground = thrust_n < self.ground_thrust_n;
        let [roll, pitch, yaw] = &mut self.axes;
        Ok(ThrustTorque {
            thrust_n: thrust_n.max(0.0),
            torque_nm: [
                roll.step(sx - mx, mx, dt, first, limited.roll, ground, hz),
                pitch.step(sy - my, my, dt, first, limited.pitch, ground, hz),
                yaw.step(sz - mz, mz, dt, first, limited.yaw, ground, hz),
            ],
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
            thrust_n: MASS * GRAVITY_MSS,
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
        let mut pid = CascadedPid::new(&tuning);
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

    // ---- Altitude hold: the vertical loop (decision-nav-eskf-20261120, F1, F4, F5) ----

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
        let mut pid = CascadedPid::new(&t);
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
        let mut pid = CascadedPid::new(&t);
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
            let mut pid = CascadedPid::new(&t);
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
            let mut pid = CascadedPid::new(&t);
            let (i, _) = run_alt(&mut pid, v_ff, v_d, flags(low, high), 200);
            if frozen {
                assert_eq!(i, 0.0, "{name}");
                // Control: with the rule off the integrator grows in that direction.
                let mut off = CascadedPid::new(&t);
                off.antiwindup_off = true;
                let (i, _) = run_alt(&mut off, v_ff, v_d, flags(low, high), 200);
                assert!(i * sign > 0.0, "{name}: control {i}");
            } else {
                assert!(i * sign > 0.0, "{name}: {i}");
            }
        }
        // Our own clamp, no flags: thrust at 0 with e_v >= 0 (the setpoint at the descent speed, climbing at it too).
        let full = full_thrust_n(t.pilot_thrust_max_g);
        let mut pid = CascadedPid::new(&t);
        let (i, thrust) = run_alt(&mut pid, t.pilot_speed_dn_ms, -t.pilot_speed_dn_ms, flags(false, false), 200);
        assert_eq!(thrust, 0.0);
        assert_eq!(i, 0.0, "own minimum clamp");
        let mut off = CascadedPid::new(&t);
        off.antiwindup_off = true;
        assert!(run_alt(&mut off, t.pilot_speed_dn_ms, -t.pilot_speed_dn_ms, flags(false, false), 200).0 > 0.0);
        // Our own clamp at the maximum with e_v <= 0: descending fast at a target that holds.
        let mut pid = CascadedPid::new(&t);
        let (i, thrust) = run_alt(&mut pid, 0.0, 3.0 * t.pilot_speed_dn_ms, flags(false, false), 200);
        assert_eq!(thrust, full);
        assert_eq!(i, 0.0, "own maximum clamp");
        let mut off = CascadedPid::new(&t);
        off.antiwindup_off = true;
        assert!(run_alt(&mut off, 0.0, 3.0 * t.pilot_speed_dn_ms, flags(false, false), 200).0 < 0.0);
    }

    #[test]
    fn the_integrator_starts_at_0_at_each_entry_and_stays_within_its_limit() {
        let t = Tuning::DEFAULT;
        let mut pid = CascadedPid::new(&t);
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
        let mut pid = CascadedPid::new(&t);
        for k in 0..200u64 {
            let out = pid.update(&alt_nav(k * PERIOD_US, 0.0, 0.0), &tilted(t.pilot_speed_dn_ms), &limited(false)).unwrap();
            assert!(out.thrust_n < ground);
            assert_eq!(pid.axes[0].i, 0.0, "step {k}");
        }
        // At the hover thrust (above the threshold) it grows.
        let mut pid = CascadedPid::new(&t);
        for k in 0..200u64 {
            pid.update(&alt_nav(k * PERIOD_US, 0.0, 0.0), &tilted(0.0), &limited(false)).unwrap();
        }
        assert!(pid.axes[0].i > 0.0);
    }
}
