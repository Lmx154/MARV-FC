//! The attitude estimator: Mahony's complementary filter on IMU1, Betaflight's form, no integral term
//! (decision-first-pair-20261119, section 2). q is body to NED, Hamilton, [w, x, y, z]; gyro rad/s and specific
//! force m/s^2, both FRD. Tilt only: no heading, velocity or position.

use crate::gnc::{Estimator, NavState, Sensors, Valid};
use crate::quat::{cross, down_in_body, mul, normalise, pt1};
use crate::tuning::Tuning;
use fw_core::calib::GRAVITY_MSS;

/// Correction gain while armed, 1/s: Betaflight's 2500/10000 (decision 2e, section 6a).
pub const KP_ARMED: f32 = 0.25;
/// Correction gain while disarmed, 1/s: Betaflight's x10 (decision 2e, section 6a).
pub const KP_DISARMED: f32 = 2.5;
/// The accelerometer is used only from this |f|, in g (decision 2b: Betaflight and PX4).
pub const ACCEL_MIN_G: f32 = 0.9;
/// ... up to this |f|, in g (decision 2b).
pub const ACCEL_MAX_G: f32 = 1.1;
/// Longest time step, us: four IMU periods; a longer gap is an IMU fault (decision 2d, 3d). The controller uses it
/// too.
pub const DT_MAX_US: u64 = 5_000;

/// Mahony's filter. Before the first accepted accelerometer sample the attitude is the identity and not valid.
#[derive(Clone, Copy, Debug)]
pub struct Mahony {
    /// Body to NED, [w, x, y, z], unit length, w >= 0.
    q: [f32; 4],
    /// The gyro after the PT1, rad/s, FRD.
    rates: [f32; 3],
    /// The previous sample's `t_us`; `None` before the first sample.
    t_prev_us: Option<u64>,
    /// Set by the first accepted accelerometer sample with d_z > 0 (decision 2f); never cleared.
    tilt: bool,
    /// The gate's state after the previous step: selects the correction gain.
    armed: bool,
    /// Gyro PT1 cut-off, Hz (INS_GYRO_LPF).
    gyro_lpf_hz: f32,
}

impl Mahony {
    /// Disarmed, no sample seen. Takes the gyro filter cut-off, Hz, from `tuning`.
    pub fn new(tuning: &Tuning) -> Self {
        Mahony {
            q: [1.0, 0.0, 0.0, 0.0],
            rates: [0.0; 3],
            t_prev_us: None,
            tilt: false,
            armed: false,
            gyro_lpf_hz: tuning.ins_gyro_lpf_hz,
        }
    }
}

impl Estimator for Mahony {
    fn update(&mut self, sensors: &Sensors) -> NavState {
        let imu = &sensors.imu1;
        let t_us = imu.t_us;
        let omega = imu.reading.gyro_rads;
        let f = imu.reading.accel_ms2;
        // dt in s, capped; 0 on the first sample.
        let dt = match self.t_prev_us {
            Some(prev) => t_us.saturating_sub(prev).min(DT_MAX_US) as f32 * 1e-6,
            None => 0.0,
        };

        // rates_rads: PT1 of the gyro (decision 2h), started at the first sample.
        self.rates = match self.t_prev_us {
            Some(_) => {
                let [r0, r1, r2] = self.rates;
                let [w0, w1, w2] = omega;
                let hz = self.gyro_lpf_hz;
                [
                    pt1(r0, w0, dt, hz),
                    pt1(r1, w1, dt, hz),
                    pt1(r2, w2, dt, hz),
                ]
            }
            None => omega,
        };
        self.t_prev_us = Some(t_us);

        // Measured down, d_meas = -f/|f|, only inside the window (decision 2b).
        let norm = libm::sqrtf(f.iter().map(|v| v * v).sum());
        let accepted = (ACCEL_MIN_G * GRAVITY_MSS..=ACCEL_MAX_G * GRAVITY_MSS).contains(&norm);
        let d_meas = f.map(|v| -v / norm);

        if !self.tilt {
            // Initialisation (decision 2f): the shortest rotation from d_meas to NED down, no yaw.
            let [dx, dy, dz] = d_meas;
            if accepted && dz > 0.0 {
                self.q = normalise([1.0 + dz, dy, -dx, 0.0]);
                self.tilt = true;
            }
        } else {
            // Correction e = d_meas x d_est (decision 2c), then q <- q + 0.5 dt q (x) [0, omega_c] (decision 2d).
            let e = if accepted {
                cross(d_meas, down_in_body(self.q))
            } else {
                [0.0; 3]
            };
            let kp = if self.armed { KP_ARMED } else { KP_DISARMED };
            let [wx, wy, wz] = omega;
            let [ex, ey, ez] = e;
            let q_dot = mul(self.q, [0.0, wx + kp * ex, wy + kp * ey, wz + kp * ez]);
            let mut q = self.q;
            for (qi, di) in q.iter_mut().zip(q_dot) {
                *qi += 0.5 * dt * di;
            }
            q = normalise(q);
            if q[0] < 0.0 {
                q = q.map(|v| -v);
            }
            self.q = q;
        }

        NavState {
            t_us,
            attitude: self.q,
            rates_rads: self.rates,
            velocity_ms: [0.0; 3],
            position_m: [0.0; 3],
            valid: Valid {
                tilt: self.tilt,
                rates: true,
                ..Valid::NONE
            },
            sigma: None,
        }
    }

    fn set_armed(&mut self, armed: bool) {
        self.armed = armed;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::imu::{ImuReading, ImuSample};

    /// IMU period, us: 800 Hz (decision-sim-conventions 6).
    const PERIOD_US: u64 = 1_250;

    fn sensors(t_us: u64, gyro_rads: [f32; 3], accel_ms2: [f32; 3]) -> Sensors {
        Sensors {
            imu1: ImuSample {
                reading: ImuReading {
                    gyro_rads,
                    accel_ms2,
                    temperature_c: 0.0,
                },
                t_us,
                seq: (t_us / PERIOD_US) as u32,
            },
            imu2: None,
            baro: None,
            mag: None,
            gnss: None,
            external: None,
        }
    }

    /// Z-Y-X roll, rad (conventions 7c), on the host with std.
    fn roll(q: [f32; 4]) -> f32 {
        let [w, x, y, z] = q;
        (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y))
    }

    fn check(nav: &NavState, t_us: u64) {
        let q = nav.attitude;
        let n: f32 = q.iter().map(|v| v * v).sum::<f32>().sqrt();
        assert!((n - 1.0).abs() < 1e-6, "|q| = {n} at {t_us}");
        assert!(q[0] >= 0.0, "w < 0 at {t_us}");
        assert!(
            nav.valid.tilt && nav.valid.rates && !nav.valid.heading,
            "{:?}",
            nav.valid
        );
        assert!(!nav.valid.velocity_ne && !nav.valid.velocity_d);
        assert!(!nav.valid.position_ne && !nav.valid.position_d);
        assert_eq!(nav.sigma, None);
        assert_eq!(nav.t_us, t_us);
    }

    #[test]
    fn mahony_converges_to_gravity_with_a_unit_quaternion() {
        // Rolled +20 deg (right side down, conventions 7b): R = Rx(phi) body to NED, so NED down in the body is
        // R^T [0, 0, 1] = [0, sin phi, cos phi], and at rest the accelerometer reads f = -g times that (7g):
        // [0, -g sin 20, -g cos 20]. Agrees with the packet.
        let phi = 20.0f32.to_radians();
        let f = [0.0, -GRAVITY_MSS * phi.sin(), -GRAVITY_MSS * phi.cos()];
        let mut m = Mahony::new(&Tuning::DEFAULT);
        let mut t = 0;
        // 1. Constant tilted gravity, zero gyro, from the first sample on: 2 s.
        for _ in 0..1600 {
            let nav = m.update(&sensors(t, [0.0; 3], f));
            check(&nav, t);
            let r = roll(nav.attitude);
            assert!(
                (r - phi).abs() < 0.1f32.to_radians(),
                "roll {} deg at {t}",
                r.to_degrees()
            );
            t += PERIOD_US;
        }
        let held = m.update(&sensors(t, [0.0; 3], f)).attitude;
        t += PERIOD_US;
        // 2. |f| outside the window (free fall, then 2 g): the attitude does not change. 0.5 s.
        for k in 0..400 {
            let out = if k % 2 == 0 {
                [0.0; 3]
            } else {
                f.map(|v| 2.0 * v)
            };
            let nav = m.update(&sensors(t, [0.0; 3], out));
            check(&nav, t);
            for (a, b) in nav.attitude.iter().zip(held) {
                assert!((a - b).abs() < 1e-6, "{:?} vs {held:?}", nav.attitude);
            }
            t += PERIOD_US;
        }
        // 3. +1 rad/s about x for 0.1 s (80 samples), accelerometer outside the window: roll +0.1 rad within 1 %.
        let before = roll(held);
        let mut nav = m.update(&sensors(t, [0.0; 3], [0.0; 3]));
        for _ in 0..80 {
            t += PERIOD_US;
            nav = m.update(&sensors(t, [1.0, 0.0, 0.0], [0.0; 3]));
            check(&nav, t);
        }
        let gained = roll(nav.attitude) - before;
        assert!((gained - 0.1).abs() < 0.001, "gained {gained} rad");
        // 4. Tilted gravity again: the estimate converges back to +20 deg (Kp 2.5 1/s disarmed: 3 s is 7.5 time
        //    constants).
        for _ in 0..2400 {
            t += PERIOD_US;
            nav = m.update(&sensors(t, [0.0; 3], f));
            check(&nav, t);
        }
        let r = roll(nav.attitude);
        assert!(
            (r - phi).abs() < 0.1f32.to_radians(),
            "roll {} deg",
            r.to_degrees()
        );
    }
}
