//! The attitude estimator: Mahony's complementary filter on IMU1, Betaflight's form, no integral term.
//! The accelerometer is low-passed (PT1, `Tuning::ins_accel_lpf_hz`)
//! before anything is decided on it: the 0.9 g to 1.1 g window and the direction both come from the filtered
//! vector, never from a raw sample. q is body to NED, Hamilton, [w, x, y, z]; gyro rad/s and specific
//! force m/s^2, both FRD. Tilt only: no heading, velocity or position.

use crate::gnc::{Estimator, NavState, Sensors, Valid};
use crate::quat::{cross, down_in_body, mul, normalise, pt1};
use crate::tuning::Tuning;
use fw_core::calib::GRAVITY_MSS;

/// Correction gain while armed, 1/s: Betaflight's 2500/10000.
pub const KP_ARMED: f32 = 0.25;
/// Correction gain while disarmed, 1/s: Betaflight's x10.
pub const KP_DISARMED: f32 = 2.5;
/// The filtered accelerometer is used only from this |f|, in g (Betaflight and PX4).
pub const ACCEL_MIN_G: f32 = 0.9;
/// ... up to this |f|, in g.
pub const ACCEL_MAX_G: f32 = 1.1;
/// Longest time step, us: four IMU periods; a longer gap is an IMU fault. The controller uses it
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
    /// Set by the first accepted accelerometer sample with d_z > 0; never cleared.
    tilt: bool,
    /// The gate's state after the previous step: selects the correction gain.
    armed: bool,
    /// The accelerometer after the PT1, m/s^2, FRD: the window and the direction use it.
    accel_lpf: [f32; 3],
    /// Gyro PT1 cut-off, Hz (INS_GYRO_LPF).
    gyro_lpf_hz: f32,
    /// Accelerometer PT1 cut-off, Hz (`Tuning::ins_accel_lpf_hz`).
    accel_lpf_hz: f32,
}

impl Mahony {
    /// Disarmed, no sample seen. Takes the gyro and accelerometer filter cut-offs, Hz, from `tuning`.
    pub fn new(tuning: &Tuning) -> Self {
        Mahony {
            q: [1.0, 0.0, 0.0, 0.0],
            rates: [0.0; 3],
            t_prev_us: None,
            tilt: false,
            armed: false,
            accel_lpf: [0.0; 3],
            gyro_lpf_hz: tuning.ins_gyro_lpf_hz,
            accel_lpf_hz: tuning.ins_accel_lpf_hz,
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

        match self.t_prev_us {
            Some(_) => {
                let [r0, r1, r2] = self.rates;
                let [w0, w1, w2] = omega;
                let hz = self.gyro_lpf_hz;
                self.rates = [
                    pt1(r0, w0, dt, hz),
                    pt1(r1, w1, dt, hz),
                    pt1(r2, w2, dt, hz),
                ];
                let hz = self.accel_lpf_hz;
                for (a, x) in self.accel_lpf.iter_mut().zip(f) {
                    *a = pt1(*a, x, dt, hz);
                }
            }
            None => {
                self.rates = omega;
                self.accel_lpf = f;
            }
        }
        self.t_prev_us = Some(t_us);

        let fl = self.accel_lpf;
        let norm = libm::sqrtf(fl.iter().map(|v| v * v).sum());
        let accepted = (ACCEL_MIN_G * GRAVITY_MSS..=ACCEL_MAX_G * GRAVITY_MSS).contains(&norm);
        let d_meas = fl.map(|v| -v / norm);

        if !self.tilt {
            // Initialisation: the shortest rotation from d_meas to NED down, no yaw.
            let [dx, dy, dz] = d_meas;
            if accepted && dz > 0.0 {
                self.q = normalise([1.0 + dz, dy, -dx, 0.0]);
                self.tilt = true;
            }
        } else {
            // Correction e = d_meas x d_est, then q <- q + 0.5 dt q (x) [0, omega_c].
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

    /// IMU period, us: 800 Hz.
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

    /// Z-Y-X roll, rad, on the host with std.
    fn roll(q: [f32; 4]) -> f32 {
        let [w, x, y, z] = q;
        (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y))
    }

    /// The angle between two attitudes of the same hemisphere, rad (twice the length of the quaternions' difference).
    fn angle(a: [f32; 4], b: [f32; 4]) -> f32 {
        2.0 * a
            .iter()
            .zip(b)
            .map(|(x, y)| (x - y) * (x - y))
            .sum::<f32>()
            .sqrt()
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
        let phi = 20.0f32.to_radians();
        let f = [0.0, -GRAVITY_MSS * phi.sin(), -GRAVITY_MSS * phi.cos()];
        let mut m = Mahony::new(&Tuning::DEFAULT);
        let mut t = 0;
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
        let f2 = [f[0], -f[1], f[2]];
        let tol_rad = 0.1f32.to_radians();
        for (name, scale, dir) in [("2 g", 2.0, f2), ("0.5 g", 0.5, f2), ("0 g", 0.0, f)] {
            let mut e = Mahony::new(&Tuning::DEFAULT);
            let mut te = 0;
            let mut before = [0.0; 4];
            for _ in 0..1600 {
                before = e.update(&sensors(te, [0.0; 3], f)).attitude;
                te += PERIOD_US;
            }
            let mut worst = 0.0f32;
            let mut end = before;
            for _ in 0..200 {
                let nav = e.update(&sensors(te, [0.0; 3], dir.map(|v| scale * v)));
                check(&nav, te);
                assert!(
                    nav.attitude.iter().all(|v| v.is_finite()),
                    "{name}: {:?}",
                    nav.attitude
                );
                end = nav.attitude;
                worst = worst.max(angle(end, before));
                te += PERIOD_US;
            }
            std::println!(
                "hold {name}: attitude change at the end {:.5} deg, largest during {:.5} deg",
                angle(end, before).to_degrees(),
                worst.to_degrees()
            );
            assert!(
                angle(end, before) <= tol_rad,
                "hold {name}: moved {} deg",
                angle(end, before).to_degrees()
            );
        }
        let before = roll(held);
        let mut nav = m.update(&sensors(t, [0.0; 3], [0.0; 3]));
        for _ in 0..80 {
            t += PERIOD_US;
            nav = m.update(&sensors(t, [1.0, 0.0, 0.0], [0.0; 3]));
            check(&nav, t);
        }
        let gained = roll(nav.attitude) - before;
        assert!((gained - 0.1).abs() < 0.001, "gained {gained} rad");
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

    /// Pitch, rad: sin(pitch) = 2 (w y - z x).
    fn pitch(q: [f32; 4]) -> f32 {
        let [w, x, y, z] = q;
        (2.0 * (w * y - z * x)).asin()
    }

    /// One record of `ground-run-20261120.bin`: a recorded ground run, IMU1, the motors spinning up with the vehicle on
    /// the ground.
    const RECORD_BYTES: usize = 33;
    const GROUND_RUN: &[u8] = include_bytes!("../testdata/ground-run-20261120.bin");

    #[test]
    fn tilt_holds_on_the_ground_with_the_motors_running() {
        assert_eq!(GROUND_RUN.len() % RECORD_BYTES, 0);
        let records = GROUND_RUN.len() / RECORD_BYTES;
        assert_eq!(records, 12_075);
        let f32_at = |r: &[u8], o: usize| f32::from_le_bytes(r[o..o + 4].try_into().unwrap());
        let rows: std::vec::Vec<(u64, [f32; 3], [f32; 3], bool)> = GROUND_RUN
            .chunks_exact(RECORD_BYTES)
            .map(|r| {
                let t_us = u64::from_le_bytes(r[0..8].try_into().unwrap());
                let gyro = [f32_at(r, 8), f32_at(r, 12), f32_at(r, 16)];
                let accel = [f32_at(r, 20), f32_at(r, 24), f32_at(r, 28)];
                (t_us, gyro, accel, r[32] != 0)
            })
            .collect();

        let mut mean = [0.0f64; 3];
        for (_, _, a, _) in &rows {
            for (m, v) in mean.iter_mut().zip(a) {
                *m += f64::from(*v) / records as f64;
            }
        }
        let [dx, dy, dz] = mean.map(|v| -v);
        let ref_roll = dy.atan2(dz).to_degrees();
        let ref_pitch = (-dx).atan2(dy.hypot(dz)).to_degrees();

        let mut m = Mahony::new(&Tuning::DEFAULT);
        let (mut worst_roll, mut worst_pitch) = (0.0f64, 0.0f64);
        let mut seen_armed = false;
        for (t_us, gyro, accel, armed) in rows {
            m.set_armed(armed);
            let nav = m.update(&sensors(t_us, gyro, accel));
            seen_armed |= armed;
            if seen_armed {
                let r = f64::from(roll(nav.attitude).to_degrees());
                let p = f64::from(pitch(nav.attitude).to_degrees());
                worst_roll = worst_roll.max((r - ref_roll).abs());
                worst_pitch = worst_pitch.max((p - ref_pitch).abs());
            }
        }
        std::println!(
            "reference roll {ref_roll:.3} pitch {ref_pitch:.3} deg; largest error roll {worst_roll:.3} pitch {worst_pitch:.3} deg"
        );
        assert!(seen_armed);
        assert!(worst_roll <= 0.5, "roll error {worst_roll} deg");
        assert!(worst_pitch <= 0.5, "pitch error {worst_pitch} deg");
    }
}
