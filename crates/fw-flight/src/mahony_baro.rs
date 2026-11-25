//! The steering estimator: [`Mahony`] plus [`VertKf`], a 3-state vertical Kalman filter on the barometer and the
//! accelerometer. Attitude, rates and the tilt and rates flags are Mahony's, bit for bit. States, f32: D (down from
//! the barometer datum, m), vD (m/s), bD (earth-down accelerometer bias, m/s^2). Pure arithmetic: no lock, timer or
//! log.

use crate::gnc::{Estimator, NavSigma, NavState, Sensors};
use crate::mahony::{DT_MAX_US, Mahony};
use crate::quat::down_in_body;
use crate::tuning::Tuning;
use fw_core::calib::GRAVITY_MSS;
use fw_core::sample::BaroSample;

/// ISA sea-level pressure, Pa (ISO 2533), as the simulator's environment.
pub const ISA_P0_PA: f32 = 101_325.0;
/// ISA sea-level temperature, K (ISO 2533).
pub const ISA_T0_K: f32 = 288.15;
/// ISA troposphere lapse rate, K/m (ISO 2533).
pub const ISA_LAPSE_K_M: f32 = 0.0065;
/// ISA specific gas constant of dry air, J/(kg K) (ISO 2533), as the simulator's barometer model.
pub const ISA_GAS_CONSTANT: f32 = 287.052_87;

/// ISA troposphere height above mean sea level from pressure, m: T0/L (1 - (p/P0)^(L R/g)), the inverse of the
/// simulator's pressure law.
pub fn isa_height_m(pressure_pa: f32) -> f32 {
    let exponent = ISA_LAPSE_K_M * ISA_GAS_CONSTANT / GRAVITY_MSS;
    ISA_T0_K / ISA_LAPSE_K_M * (1.0 - libm::powf(pressure_pa / ISA_P0_PA, exponent))
}

/// The vertical filter. Before its initialisation (the first barometer sample after Mahony's tilt is valid) it holds
/// nothing and propagates nothing.
#[derive(Clone, Copy, Debug)]
pub struct VertKf {
    /// [D, vD, bD].
    x: [f32; 3],
    /// Symmetric, diagonal >= 0.
    p: [[f32; 3]; 3],
    /// The state and P mean something.
    init: bool,
    /// ISA height of the first barometer sample after tilt, m; set once.
    datum_m: Option<f32>,
    /// The previous IMU1 sample's `t_us` since the initialisation.
    t_prev_us: u64,
    /// Capped time since the last covariance propagation, us.
    since_predict_us: u64,
    /// Step time of the last accepted fusion, height reset or initialisation, us.
    last_aid_us: u64,
    fused: u32,
    rejected: u32,
    resets: u32,
    /// Covariance propagations so far (the NKF records' rate).
    predicts: u32,
    /// The latest barometer step's (innovation, its variance), or for a height reset (applied change, new variance),
    /// or at the initialisation (0, the measurement variance): the NKI record's Inn and Var.
    last_baro: (f32, f32),
    predict_us: u64,
    baro_delay_us: u64,
    baro_sigma_m: f32,
    gate_sigma: f32,
    aid_timeout_us: u64,
    vrw: f32,
    ba_rw: f32,
    p0_vel_ms: f32,
    p0_ba_ms2: f32,
}

impl VertKf {
    /// Not initialised; the numbers from `tuning`.
    pub fn new(tuning: &Tuning) -> Self {
        VertKf {
            x: [0.0; 3],
            p: [[0.0; 3]; 3],
            init: false,
            datum_m: None,
            t_prev_us: 0,
            since_predict_us: 0,
            last_aid_us: 0,
            fused: 0,
            rejected: 0,
            resets: 0,
            predicts: 0,
            last_baro: (0.0, 0.0),
            predict_us: tuning.nav_predict_us,
            baro_delay_us: tuning.nav_baro_delay_us,
            baro_sigma_m: tuning.nav_baro_sigma_m,
            gate_sigma: tuning.nav_gate_sigma,
            aid_timeout_us: tuning.nav_aid_timeout_us,
            vrw: tuning.nav_vrw,
            ba_rw: tuning.nav_ba_rw,
            p0_vel_ms: tuning.nav_p0_vel_ms,
            p0_ba_ms2: tuning.nav_p0_ba_ms2,
        }
    }

    /// The earth-down accelerometer bias estimate, m/s^2.
    pub fn bias_d(&self) -> f32 {
        self.x[2]
    }

    /// Its sigma, m/s^2, from P.
    pub fn bias_d_sigma(&self) -> f32 {
        libm::sqrtf(self.p[2][2])
    }

    /// Barometer samples fused so far (the initialising sample counts).
    pub fn fused(&self) -> u32 {
        self.fused
    }

    /// Barometer samples rejected by the gate so far.
    pub fn rejected(&self) -> u32 {
        self.rejected
    }

    /// Height resets so far.
    pub fn resets(&self) -> u32 {
        self.resets
    }

    /// Covariance propagations so far.
    pub fn predicts(&self) -> u32 {
        self.predicts
    }

    /// The latest barometer step's (Inn, Var) for NKI: (innovation, its variance) of a fusion or gated sample,
    /// (applied change, new variance) of a height reset, (0, the measurement variance) of the initialisation.
    pub fn last_baro(&self) -> (f32, f32) {
        self.last_baro
    }

    /// The state and P mean something.
    pub fn initialised(&self) -> bool {
        self.init
    }

    /// One IMU1 step after Mahony's: `q` Mahony's attitude, `tilt` its flag, `f` the raw specific force, m/s^2 FRD.
    fn step(
        &mut self,
        t_us: u64,
        q: [f32; 4],
        tilt: bool,
        f: [f32; 3],
        baro: Option<&BaroSample>,
        armed: bool,
    ) {
        if !tilt {
            return;
        }
        if self.init {
            self.propagate(t_us, q, f);
        }
        // A pressure that is not finite or is <= 0 is ignored, as if missing.
        let baro =
            baro.filter(|b| b.reading.pressure_pa.is_finite() && b.reading.pressure_pa > 0.0);
        if let Some(b) = baro {
            let h_abs = isa_height_m(b.reading.pressure_pa);
            let datum = *self.datum_m.get_or_insert(h_abs);
            let h_b = h_abs - datum;
            if self.init {
                self.fuse(t_us, b.t_us, h_b, armed);
            } else {
                self.initialise(t_us, h_b);
            }
        }
        let finite = self
            .x
            .iter()
            .chain(self.p.iter().flatten())
            .all(|v| v.is_finite());
        if !finite {
            self.init = false;
        }
    }

    /// D = -h_b (0 at the datum sample), vD = bD = 0; sigmas nav_baro_sigma_m, P0_vel, P0_ba.
    fn initialise(&mut self, t_us: u64, h_b: f32) {
        let sb = self.baro_sigma_m;
        self.x = [-h_b, 0.0, 0.0];
        self.p = [[0.0; 3]; 3];
        self.p[0][0] = sb * sb;
        self.p[1][1] = self.p0_vel_ms * self.p0_vel_ms;
        self.p[2][2] = self.p0_ba_ms2 * self.p0_ba_ms2;
        self.init = true;
        self.t_prev_us = t_us;
        self.since_predict_us = 0;
        self.last_aid_us = t_us;
        self.fused = self.fused.wrapping_add(1);
        self.last_baro = (0.0, sb * sb);
    }

    /// The state every IMU1 sample; F P F^T + Q every nav_predict_us of capped time.
    fn propagate(&mut self, t_us: u64, q: [f32; 4], f: [f32; 3]) {
        let dt_us = t_us.saturating_sub(self.t_prev_us).min(DT_MAX_US);
        self.t_prev_us = t_us;
        let dt = dt_us as f32 * 1e-6;
        // Row 3 of R(q) is NED down expressed in the body.
        let down = down_in_body(q);
        let f_d: f32 = down.iter().zip(f).map(|(d, v)| d * v).sum();
        let [d, vd, bd] = self.x;
        let a_d = f_d + GRAVITY_MSS - bd;
        self.x = [d + vd * dt + 0.5 * a_d * dt * dt, vd + a_d * dt, bd];

        self.since_predict_us += dt_us;
        if self.since_predict_us < self.predict_us {
            return;
        }
        let t = self.since_predict_us as f32 * 1e-6;
        self.since_predict_us = 0;
        self.predicts = self.predicts.wrapping_add(1);
        let h = 0.5 * t * t;
        let [[a, b, c], [_, dd, e], [_, _, ff]] = self.p;
        // F P with F = [[1, T, -T^2/2], [0, 1, -T], [0, 0, 1]].
        let fp0 = [a + t * b - h * c, b + t * dd - h * e, c + t * e - h * ff];
        let fp1 = [b - t * c, dd - t * e, e - t * ff];
        let [f00, f01, f02] = fp0;
        let [_, f11, f12] = fp1;
        let vrw2 = self.vrw * self.vrw;
        let p00 = f00 + t * f01 - h * f02 + vrw2 * t * t * t / 3.0;
        let p01 = f01 - t * f02;
        let p02 = f02;
        let p11 = f11 - t * f12 + vrw2 * t;
        let p12 = f12;
        let p22 = ff + self.ba_rw * self.ba_rw * t;
        self.p = [[p00, p01, p02], [p01, p11, p12], [p02, p12, p22]];
        self.floor();
    }

    /// h = -D + vD tau, H = [-1, tau, 0], tau = (t_step - baro.t_us) + nav_baro_delay_us; gated; the height reset
    /// after nav_aid_timeout_us of rejections.
    fn fuse(&mut self, t_us: u64, baro_t_us: u64, h_b: f32, armed: bool) {
        let tau = (t_us.saturating_sub(baro_t_us) + self.baro_delay_us) as f32 * 1e-6;
        let r = self.baro_sigma_m * self.baro_sigma_m;
        let p = self.p;
        let ph = p.map(|row| -row[0] + tau * row[1]);
        let [ph0, ph1, _] = ph;
        let s = -ph0 + tau * ph1 + r;
        let [d, vd, _] = self.x;
        let innov = h_b - (-d + vd * tau);
        let g = self.gate_sigma;
        self.last_baro = (innov, s);
        if innov * innov <= g * g * s {
            for (xi, phi) in self.x.iter_mut().zip(ph) {
                *xi += phi / s * innov;
            }
            for (row, phi) in self.p.iter_mut().zip(ph) {
                for (pij, phj) in row.iter_mut().zip(ph) {
                    *pij -= phi * phj / s;
                }
            }
            self.symmetrise();
            self.floor();
            self.last_aid_us = t_us;
            self.fused = self.fused.wrapping_add(1);
            return;
        }
        self.rejected = self.rejected.wrapping_add(1);
        if t_us.saturating_sub(self.last_aid_us) >= self.aid_timeout_us {
            // EKF3 ResetHeight: D to the barometer, the D row and column of P zeroed, P_DD the measurement variance;
            self.last_baro = (-h_b - self.x[0], r);
            self.x[0] = -h_b;
            if !armed {
                self.x[1] = 0.0;
            }
            for row in self.p.iter_mut() {
                row[0] = 0.0;
            }
            self.p[0] = [r, 0.0, 0.0];
            self.last_aid_us = t_us;
            self.resets = self.resets.wrapping_add(1);
        }
    }

    fn symmetrise(&mut self) {
        let [[_, b, c], [b2, _, e], [c2, e2, _]] = self.p;
        let (b, c, e) = (0.5 * (b + b2), 0.5 * (c + c2), 0.5 * (e + e2));
        self.p[0][1] = b;
        self.p[1][0] = b;
        self.p[0][2] = c;
        self.p[2][0] = c;
        self.p[1][2] = e;
        self.p[2][1] = e;
    }

    fn floor(&mut self) {
        self.p[0][0] = self.p[0][0].max(0.0);
        self.p[1][1] = self.p[1][1].max(0.0);
        self.p[2][2] = self.p[2][2].max(0.0);
    }

    /// velocity_d and position_d: initialised and aided (fusion, reset or initialisation) within nav_aid_timeout_us.
    fn aided(&self, t_us: u64) -> bool {
        self.init && t_us.saturating_sub(self.last_aid_us) < self.aid_timeout_us
    }
}

/// [`Mahony`] plus [`VertKf`].
#[derive(Clone, Copy, Debug)]
pub struct MahonyBaro {
    mahony: Mahony,
    vert: VertKf,
    armed: bool,
}

impl MahonyBaro {
    /// Disarmed, no sample seen.
    pub fn new(tuning: &Tuning) -> Self {
        MahonyBaro {
            mahony: Mahony::new(tuning),
            vert: VertKf::new(tuning),
            armed: false,
        }
    }

    /// The vertical filter, for tests, the simulator and the log.
    pub fn vert(&self) -> &VertKf {
        &self.vert
    }
}

impl Estimator for MahonyBaro {
    fn update(&mut self, sensors: &Sensors) -> NavState {
        let mut nav = self.mahony.update(sensors);
        let v = &mut self.vert;
        v.step(
            nav.t_us,
            nav.attitude,
            nav.valid.tilt,
            sensors.imu1.reading.accel_ms2,
            sensors.baro.as_ref(),
            self.armed,
        );
        let aided = nav.valid.tilt && v.aided(nav.t_us);
        if v.init {
            nav.position_m[2] = v.x[0];
            nav.velocity_ms[2] = v.x[1];
        }
        nav.valid.velocity_d = aided;
        nav.valid.position_d = aided;
        let (sigma_v, sigma_d) = if v.init {
            (libm::sqrtf(v.p[1][1]), libm::sqrtf(v.p[0][0]))
        } else {
            (f32::INFINITY, f32::INFINITY)
        };
        nav.sigma = Some(NavSigma {
            tilt_rad: f32::INFINITY,
            heading_rad: f32::INFINITY,
            velocity_ne_ms: f32::INFINITY,
            velocity_d_ms: sigma_v,
            position_ne_m: f32::INFINITY,
            position_d_m: sigma_d,
        });
        nav
    }

    fn set_armed(&mut self, armed: bool) {
        self.mahony.set_armed(armed);
        self.armed = armed;
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use crate::imu::{ImuReading, ImuSample};
    use fw_core::sample::BaroReading;
    use std::vec::Vec;

    /// IMU period, us: 800 Hz.
    const PERIOD_US: u64 = 1_250;
    /// The barometer every 16 IMU1 samples: 50 Hz.
    const BARO_EVERY: u64 = 16;
    /// Level and at rest: f = -g along body z.
    const LEVEL: [f32; 3] = [0.0, 0.0, -GRAVITY_MSS];

    /// The ISA pressure at `height_m`, Pa: the simulator's law, in f64.
    fn pressure_at(height_m: f64) -> f32 {
        let e = f64::from(GRAVITY_MSS) / (f64::from(ISA_LAPSE_K_M) * f64::from(ISA_GAS_CONSTANT));
        let t0 = f64::from(ISA_T0_K);
        let t = t0 - f64::from(ISA_LAPSE_K_M) * height_m;
        (f64::from(ISA_P0_PA) * (t / t0).powf(e)) as f32
    }

    fn sensors(
        t_us: u64,
        gyro_rads: [f32; 3],
        accel_ms2: [f32; 3],
        pressure: Option<f32>,
    ) -> Sensors {
        let seq = (t_us / PERIOD_US) as u32;
        Sensors {
            imu1: ImuSample {
                reading: ImuReading {
                    gyro_rads,
                    accel_ms2,
                    temperature_c: 0.0,
                },
                t_us,
                seq,
            },
            imu2: None,
            baro: pressure.map(|pressure_pa| BaroSample {
                reading: BaroReading {
                    pressure_pa,
                    temperature_c: 15.0,
                },
                t_us,
                seq,
            }),
            mag: None,
            gnss: None,
            external: None,
        }
    }

    /// `steps` level samples from step `k0`; `baro(t_s)` gives the pressure at every 16th step.
    fn drive(
        e: &mut MahonyBaro,
        k0: u64,
        steps: u64,
        baro: impl Fn(f64) -> Option<f32>,
    ) -> Vec<NavState> {
        (k0..k0 + steps)
            .map(|k| {
                let t = k * PERIOD_US;
                let p = if k % BARO_EVERY == 0 {
                    baro(t as f64 * 1e-6)
                } else {
                    None
                };
                e.update(&sensors(t, [0.0; 3], LEVEL, p))
            })
            .collect()
    }

    /// The value fields are finite on every step, and so is the sigma of a group whose flag is set.
    fn check_finite(nav: &NavState) {
        let values = nav
            .attitude
            .iter()
            .chain(&nav.rates_rads)
            .chain(&nav.velocity_ms)
            .chain(&nav.position_m);
        assert!(values.into_iter().all(|v| v.is_finite()), "{nav:?}");
        let s = nav.sigma.expect("MahonyBaro always gives sigmas");
        assert!(
            !nav.valid.velocity_d || s.velocity_d_ms.is_finite(),
            "{nav:?}"
        );
        assert!(
            !nav.valid.position_d || s.position_d_m.is_finite(),
            "{nav:?}"
        );
        assert!(!nav.valid.heading && !nav.valid.velocity_ne && !nav.valid.position_ne);
        assert_eq!((s.tilt_rad, s.heading_rad), (f32::INFINITY, f32::INFINITY));
        assert_eq!(
            (s.velocity_ne_ms, s.position_ne_m),
            (f32::INFINITY, f32::INFINITY)
        );
    }

    const RECORD_BYTES: usize = 33;
    /// A recorded ground run, in the record format of mahony.rs's test.
    const GROUND_RUN: &[u8] = include_bytes!("../testdata/ground-run-20261120.bin");

    #[test]
    fn a1_attitude_rates_and_flags_are_mahonys_bit_for_bit() {
        let f32_at = |r: &[u8], o: usize| f32::from_le_bytes(r[o..o + 4].try_into().unwrap());
        let rows: Vec<(u64, [f32; 3], [f32; 3], bool)> = GROUND_RUN
            .chunks_exact(RECORD_BYTES)
            .map(|r| {
                let t_us = u64::from_le_bytes(r[0..8].try_into().unwrap());
                (
                    t_us,
                    [f32_at(r, 8), f32_at(r, 12), f32_at(r, 16)],
                    [f32_at(r, 20), f32_at(r, 24), f32_at(r, 28)],
                    r[32] != 0,
                )
            })
            .collect();
        assert_eq!(rows.len(), 12_075);
        let tuning = Tuning::DEFAULT;
        let (mut m, mut mb, mut perturbed) = (
            Mahony::new(&tuning),
            MahonyBaro::new(&tuning),
            Mahony::new(&tuning),
        );
        let same = |a: &NavState, b: &NavState| {
            a.attitude.map(f32::to_bits) == b.attitude.map(f32::to_bits)
                && a.rates_rads.map(f32::to_bits) == b.rates_rads.map(f32::to_bits)
                && a.valid.tilt == b.valid.tilt
                && a.valid.rates == b.valid.rates
        };
        let (mut differ, mut aided) = (0, 0);
        for (i, &(t_us, gyro, accel, armed)) in rows.iter().enumerate() {
            let p = (i as u64 % BARO_EVERY == 0).then_some(ISA_P0_PA);
            let s = sensors(t_us, gyro, accel, p);
            m.set_armed(armed);
            mb.set_armed(armed);
            perturbed.set_armed(armed);
            let (a, b) = (m.update(&s), mb.update(&s));
            assert!(same(&a, &b), "record {i}: Mahony {a:?} MahonyBaro {b:?}");
            aided += usize::from(b.valid.position_d);
            let mut sp = s;
            if i == 1000 {
                sp.imu1.reading.gyro_rads[0] = f32::from_bits(gyro[0].to_bits() + 1);
            }
            differ += usize::from(!same(&perturbed.update(&sp), &b));
        }
        std::println!(
            "a1: {} records identical; VertKf aided on {aided}; negative control differs on {differ}",
            rows.len()
        );
        assert!(aided > 10_000, "VertKf was not exercised: {aided}");
        assert!(
            differ > 0,
            "negative control: a perturbed input must change Mahony's output"
        );
    }

    #[test]
    fn a5_flags_drop_at_the_timeout_and_return_at_the_first_fusion() {
        let t = Tuning::DEFAULT;
        let mut e = MahonyBaro::new(&t);
        let still = |_| Some(pressure_at(0.0));
        let k_baro = 2_000_000 / PERIOD_US;
        let navs = drive(&mut e, 0, k_baro, still);
        assert!(navs.last().unwrap().valid.position_d && navs.last().unwrap().valid.velocity_d);
        let last_fusion_us = (k_baro - 1) / BARO_EVERY * BARO_EVERY * PERIOD_US;
        let k_none = (t.nav_aid_timeout_us + 1_000_000) / PERIOD_US;
        let quiet = drive(&mut e, k_baro, k_none, |_| None);
        let drop = quiet
            .iter()
            .find(|n| !n.valid.position_d)
            .expect("flags drop")
            .t_us;
        assert!(
            quiet
                .iter()
                .all(|n| n.valid.position_d == n.valid.velocity_d && n.valid.tilt)
        );
        assert!(
            quiet
                .iter()
                .skip_while(|n| n.t_us < drop)
                .all(|n| !n.valid.position_d)
        );
        let late = drop - last_fusion_us;
        std::println!(
            "a5: flags dropped {late} us after the last fusion (timeout {} us)",
            t.nav_aid_timeout_us
        );
        assert!(
            late >= t.nav_aid_timeout_us && late < t.nav_aid_timeout_us + PERIOD_US,
            "dropped after {late} us"
        );
        let (fused, resets) = (e.vert().fused(), e.vert().resets());
        let k_back = k_baro + k_none;
        let back = drive(&mut e, k_back, 2 * BARO_EVERY, still);
        let first = back
            .iter()
            .position(|n| n.valid.position_d)
            .expect("flags return");
        assert_eq!(
            (k_back + first as u64) % BARO_EVERY,
            0,
            "not at a barometer sample"
        );
        assert!(back[..first].iter().all(|n| !n.valid.position_d));
        assert_eq!(
            e.vert().resets(),
            resets,
            "returned by a reset, not a fusion"
        );
        assert!(e.vert().fused() > fused);
    }

    /// 2 s still, then a height step four gates wide held; returns (navs from the step on, the step's first sample k.
    fn step_fault(armed: bool, climb_ms: f64) -> (MahonyBaro, Vec<NavState>, u64, f32) {
        step_fault_with(&Tuning { nav_baro_sigma_m: 0.5, ..Tuning::DEFAULT }, armed, climb_ms)
    }

    /// [`step_fault`] with the given tuning.
    fn step_fault_with(t: &Tuning, armed: bool, climb_ms: f64) -> (MahonyBaro, Vec<NavState>, u64, f32) {
        let step_m = f64::from(4.0 * t.nav_gate_sigma * t.nav_baro_sigma_m);
        let mut e = MahonyBaro::new(t);
        e.set_armed(armed);
        let k0 = 10_000_000 / PERIOD_US;
        let ok = drive(&mut e, 0, k0, |s| Some(pressure_at(climb_ms * s)));
        assert!(ok.last().unwrap().valid.position_d);
        let navs = drive(
            &mut e,
            k0,
            (t.nav_aid_timeout_us + 1_000_000) / PERIOD_US,
            |s| Some(pressure_at(climb_ms * s + step_m)),
        );
        let datum = isa_height_m(pressure_at(0.0));
        (e, navs, k0, isa_height_m(pressure_at(step_m)) - datum)
    }

    #[test]
    fn a6_a_held_step_fault_is_rejected_then_reset_at_the_timeout() {
        let t = Tuning::DEFAULT;
        let (e, navs, k0, h_b) = step_fault(false, 0.0);
        assert_eq!(e.vert().resets(), 1, "Src 6: one height reset");
        assert!(e.vert().rejected() as u64 >= t.nav_aid_timeout_us / PERIOD_US / BARO_EVERY);
        let reset = navs
            .iter()
            .position(|n| n.position_m[2] == -h_b)
            .expect("D set to the barometer");
        let after_us = navs[reset].t_us - (k0 - BARO_EVERY) * PERIOD_US;
        std::println!(
            "a6: reset {after_us} us after the last accepted sample; h_b {h_b} m; rejected {}",
            e.vert().rejected()
        );
        assert!(
            after_us >= t.nav_aid_timeout_us
                && after_us < t.nav_aid_timeout_us + BARO_EVERY * PERIOD_US
        );
        for n in &navs[reset..] {
            assert!((n.position_m[2] + h_b).abs() < 1e-3, "{n:?}");
            assert!(n.valid.position_d && n.valid.velocity_d);
            check_finite(n);
        }
    }

    #[test]
    fn a6_b_held_step_fault_at_the_default_tuning_is_rejected_then_reset() {
        let t = Tuning::DEFAULT;
        let (e, navs, _, h_b) = step_fault_with(&t, false, 0.0);
        assert_eq!(e.vert().resets(), 1, "Src 6: one height reset");
        assert!(e.vert().rejected() as u64 >= t.nav_aid_timeout_us / PERIOD_US / BARO_EVERY);
        let reset = navs.iter().position(|n| n.position_m[2] == -h_b).expect("D set to the barometer");
        for n in &navs[reset..] {
            assert!((n.position_m[2] + h_b).abs() < 1e-3, "{n:?}");
            check_finite(n);
        }
    }

    #[test]
    fn a8_a_reset_disarmed_zeroes_vd_and_armed_keeps_it() {
        for armed in [false, true] {
            let (e, navs, _, _) = step_fault(armed, 1.0);
            assert_eq!(e.vert().resets(), 1);
            let i = navs
                .windows(2)
                .position(|w| (w[1].position_m[2] - w[0].position_m[2]).abs() > 1.0)
                .unwrap()
                + 1;
            let (before, at) = (navs[i - 1].velocity_ms[2], navs[i].velocity_ms[2]);
            std::println!("a8: armed {armed}: vD {before} m/s before the reset, {at} m/s after");
            assert!((before + 1.0).abs() < 0.1, "vD before {before}");
            if armed {
                assert!((at - before).abs() < 1e-3, "vD {before} -> {at}");
            } else {
                assert_eq!(at, 0.0);
            }
        }
    }

    #[test]
    fn a7_bad_pressures_are_ignored_and_every_field_stays_finite() {
        let t = Tuning::DEFAULT;
        let bad = [f32::NAN, 0.0, -1.0, f32::INFINITY, f32::NEG_INFINITY];
        let (mut a, mut b) = (MahonyBaro::new(&t), MahonyBaro::new(&t));
        let n = 4_000_000 / PERIOD_US;
        for k in 0..n {
            let tk = k * PERIOD_US;
            let j = k / BARO_EVERY;
            let p = (k % BARO_EVERY == 0).then(|| {
                if j % 3 == 0 {
                    bad[(j / 3) as usize % bad.len()]
                } else {
                    pressure_at(0.0)
                }
            });
            let missing = p.filter(|x| x.is_finite() && *x > 0.0);
            let na = a.update(&sensors(tk, [0.0; 3], LEVEL, p));
            let nb = b.update(&sensors(tk, [0.0; 3], LEVEL, missing));
            assert_eq!(na, nb, "step {k}");
            check_finite(&na);
        }
        assert_eq!(
            (a.vert().fused(), a.vert().rejected()),
            (b.vert().fused(), b.vert().rejected())
        );
        assert!(a.vert().initialised() && a.vert().rejected() == 0);
    }

    #[test]
    fn a_non_finite_state_reinitialises_the_vertical_filter_only() {
        let t = Tuning::DEFAULT;
        let mut e = MahonyBaro::new(&t);
        let mut m = Mahony::new(&t);
        let k0 = 1_000_000 / PERIOD_US;
        drive(&mut e, 0, k0, |_| Some(pressure_at(0.0)));
        e.vert.x[1] = f32::NAN;
        let navs = drive(&mut e, k0, BARO_EVERY, |_| Some(pressure_at(1.0)));
        assert!(!navs[0].valid.position_d && !navs[0].valid.velocity_d && navs[0].valid.tilt);
        assert!(navs[1..].iter().all(|n| !n.valid.position_d));
        navs.iter().for_each(check_finite);
        let n = drive(&mut e, k0 + BARO_EVERY, 1, |_| Some(pressure_at(1.0)))[0];
        let h_b = isa_height_m(pressure_at(1.0)) - isa_height_m(pressure_at(0.0));
        assert!(
            n.valid.position_d && n.position_m[2] == -h_b && n.velocity_ms[2] == 0.0,
            "{n:?}"
        );
        let mut last = None;
        for k in 0..=k0 + BARO_EVERY {
            last = Some(m.update(&sensors(k * PERIOD_US, [0.0; 3], LEVEL, None)));
        }
        assert_eq!(last.unwrap().attitude, n.attitude);
    }
}
