//! Arithmetic of the 15-state error-state Kalman filter jobs, single precision, written to be timed from
//! SRAM on the flight core by the bench binary `src/bin/eskf_bench.rs` and checked here on the host.
//! Not called by the flight step.
//!
//! Error state: attitude error (NED frame), velocity, position, gyro bias, accelerometer
//! bias, 3 entries each. Quaternions are `[w, x, y, z]`, body to NED.
//! Jobs: [`nominal_step`] on every IMU sample; [`covariance_predict`] (the
//! structured P <- F P F^T + Q; [`covariance_predict_dense`] is the naive dense form, for comparison);
//! one scalar fusion, [`fuse`] (dense H row) or [`fuse_position`] (H has a single 1).
// Fixed-size arrays only; every index is a loop bound or a constant below the array length (matrix
// code reads more plainly with indices).
#![allow(clippy::indexing_slicing, clippy::needless_range_loop)]

use fw_core::calib::GRAVITY_MSS;
use libm::{cosf, sinf, sqrtf};

/// Error-state length.
pub const N: usize = 15;
/// First index of the attitude error.
pub const ATT: usize = 0;
/// First index of the velocity error.
pub const VEL: usize = 3;
/// First index of the position error.
pub const POS: usize = 6;
/// First index of the gyro bias error.
pub const GYRO_BIAS: usize = 9;
/// First index of the accelerometer bias error.
pub const ACCEL_BIAS: usize = 12;

/// The covariance: dense, symmetric.
pub type Cov = [[f32; N]; N];

/// The nominal state, and the velocity increment summed for the next covariance propagation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Nominal {
    /// Attitude, body to NED, `[w, x, y, z]`.
    pub q: [f32; 4],
    /// Velocity NED, m/s.
    pub v: [f32; 3],
    /// Position NED, m.
    pub p: [f32; 3],
    /// Gyro bias, rad/s.
    pub bg: [f32; 3],
    /// Accelerometer bias, m/s^2.
    pub ba: [f32; 3],
    /// Bias-corrected specific force times dt, body frame, summed since the last propagation, m/s.
    pub dvel: [f32; 3],
    /// Time summed since the last propagation, s.
    pub dt_sum: f32,
}

/// Process noise densities (variance per second).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Noise {
    /// Angular random walk squared, rad^2/s.
    pub gyro: f32,
    /// Velocity random walk squared, (m/s)^2/s.
    pub accel: f32,
    /// Gyro bias random walk squared, (rad/s)^2/s.
    pub gyro_bias: f32,
    /// Accelerometer bias random walk squared, (m/s^2)^2/s.
    pub accel_bias: f32,
}

/// Hamilton product `a * b`.
#[inline(always)]
fn qmul(a: &[f32; 4], b: &[f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

#[inline(always)]
fn normalized(q: [f32; 4]) -> [f32; 4] {
    let inv = 1.0 / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    [q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv]
}

/// The rotation matrix of `q` (body to NED).
#[inline(always)]
pub fn rotation(q: &[f32; 4]) -> [[f32; 3]; 3] {
    let [w, x, y, z] = *q;
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

#[inline(always)]
fn dot3(a: &[f32; 3], b: &[f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The nominal propagation of one IMU sample: the quaternion from the bias-corrected gyro (exact
/// axis-angle increment, libm `sinf`/`cosf`), the bias-corrected specific force rotated to NED plus
/// gravity, velocity and (trapezoidal) position integration; sums the velocity increment for the
/// next covariance propagation.
#[inline(never)]
pub fn nominal_step(n: &mut Nominal, gyro: &[f32; 3], accel: &[f32; 3], dt: f32) {
    let da = [
        (gyro[0] - n.bg[0]) * dt,
        (gyro[1] - n.bg[1]) * dt,
        (gyro[2] - n.bg[2]) * dt,
    ];
    let angle = sqrtf(dot3(&da, &da));
    let (s, c) = if angle > 1e-6 {
        let half = 0.5 * angle;
        (sinf(half) / angle, cosf(half))
    } else {
        (0.5, 1.0)
    };
    n.q = normalized(qmul(&n.q, &[c, da[0] * s, da[1] * s, da[2] * s]));
    let f = [accel[0] - n.ba[0], accel[1] - n.ba[1], accel[2] - n.ba[2]];
    let r = rotation(&n.q);
    let a = [dot3(&r[0], &f), dot3(&r[1], &f), dot3(&r[2], &f) + GRAVITY_MSS];
    for i in 0..3 {
        let v0 = n.v[i];
        n.v[i] = v0 + a[i] * dt;
        n.p[i] += 0.5 * (v0 + n.v[i]) * dt;
        n.dvel[i] += f[i] * dt;
    }
    n.dt_sum += dt;
}

/// The blocks of F = I + blocks: `rdt` = -R dt (attitude <- gyro bias and velocity <- accelerometer
/// bias), `skew` = -[R dvel]x (velocity <- attitude).
#[inline(always)]
fn blocks(q: &[f32; 4], dvel: &[f32; 3], dt: f32) -> ([[f32; 3]; 3], [[f32; 3]; 3]) {
    let r = rotation(q);
    let a = [dot3(&r[0], dvel), dot3(&r[1], dvel), dot3(&r[2], dvel)];
    let mut rdt = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            rdt[i][j] = -r[i][j] * dt;
        }
    }
    let skew = [[0.0, a[2], -a[1]], [-a[2], 0.0, a[0]], [a[1], -a[0], 0.0]];
    (rdt, skew)
}

#[inline(always)]
fn add_q(p: &mut Cov, dt: f32, noise: &Noise) {
    for i in 0..3 {
        p[ATT + i][ATT + i] += noise.gyro * dt;
        p[VEL + i][VEL + i] += noise.accel * dt;
        p[GYRO_BIAS + i][GYRO_BIAS + i] += noise.gyro_bias * dt;
        p[ACCEL_BIAS + i][ACCEL_BIAS + i] += noise.accel_bias * dt;
    }
}

/// P <- F P F^T + Q over `dt` (the time summed since the last propagation), using F's structure:
/// identity plus -R dt (attitude <- gyro bias, velocity <- accelerometer bias), -[R dvel]x
/// (velocity <- attitude) and dt I (position <- velocity). In place: the row pass (F P) updates
/// position rows before velocity rows before attitude rows, so each reads rows not yet changed; the
/// column pass (. F^T) runs on rows 0 to 8 only (rows 9 to 14 keep their upper part, columns 9 to 14,
/// which F^T does not change), then the upper triangle is mirrored and Q added.
#[inline(never)]
pub fn covariance_predict(p: &mut Cov, q: &[f32; 4], dvel: &[f32; 3], dt: f32, noise: &Noise) {
    let (rdt, skew) = blocks(q, dvel, dt);
    for c in 0..N {
        for i in 0..3 {
            p[POS + i][c] += dt * p[VEL + i][c];
        }
        let th = [p[ATT][c], p[ATT + 1][c], p[ATT + 2][c]];
        let bg = [p[GYRO_BIAS][c], p[GYRO_BIAS + 1][c], p[GYRO_BIAS + 2][c]];
        let ba = [p[ACCEL_BIAS][c], p[ACCEL_BIAS + 1][c], p[ACCEL_BIAS + 2][c]];
        for i in 0..3 {
            p[VEL + i][c] += dot3(&skew[i], &th) + dot3(&rdt[i], &ba);
        }
        for i in 0..3 {
            p[ATT + i][c] += dot3(&rdt[i], &bg);
        }
    }
    for row in p.iter_mut().take(GYRO_BIAS) {
        for i in 0..3 {
            row[POS + i] += dt * row[VEL + i];
        }
        let th = [row[ATT], row[ATT + 1], row[ATT + 2]];
        let bg = [row[GYRO_BIAS], row[GYRO_BIAS + 1], row[GYRO_BIAS + 2]];
        let ba = [row[ACCEL_BIAS], row[ACCEL_BIAS + 1], row[ACCEL_BIAS + 2]];
        for i in 0..3 {
            row[VEL + i] += dot3(&skew[i], &th) + dot3(&rdt[i], &ba);
        }
        for i in 0..3 {
            row[ATT + i] += dot3(&rdt[i], &bg);
        }
    }
    for i in 0..N {
        for j in i + 1..N {
            p[j][i] = p[i][j];
        }
    }
    add_q(p, dt, noise);
}

/// The same propagation as [`covariance_predict`], naive: F built dense, then two dense 15x15
/// products (F P, then (F P) F^T), then Q. For comparison only.
#[inline(never)]
pub fn covariance_predict_dense(p: &mut Cov, q: &[f32; 4], dvel: &[f32; 3], dt: f32, noise: &Noise) {
    let (rdt, skew) = blocks(q, dvel, dt);
    let mut f = [[0.0f32; N]; N];
    for (i, row) in f.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for i in 0..3 {
        for j in 0..3 {
            f[ATT + i][GYRO_BIAS + j] = rdt[i][j];
            f[VEL + i][ATT + j] = skew[i][j];
            f[VEL + i][ACCEL_BIAS + j] = rdt[i][j];
        }
        f[POS + i][VEL + i] = dt;
    }
    let mut t = [[0.0f32; N]; N];
    for i in 0..N {
        for j in 0..N {
            let mut s = 0.0;
            for k in 0..N {
                s += f[i][k] * p[k][j];
            }
            t[i][j] = s;
        }
    }
    for i in 0..N {
        for j in 0..N {
            let mut s = 0.0;
            for k in 0..N {
                s += t[i][k] * f[j][k];
            }
            p[i][j] = s;
        }
    }
    add_q(p, dt, noise);
}

/// The error-state injection: attitude by a left (NED-frame) small-angle quaternion, normalised;
/// the others added. The error state is then zero (reset Jacobian taken as identity).
#[inline(always)]
fn inject(n: &mut Nominal, dx: &[f32; N]) {
    let dq = [1.0, 0.5 * dx[ATT], 0.5 * dx[ATT + 1], 0.5 * dx[ATT + 2]];
    n.q = normalized(qmul(&dq, &n.q));
    for i in 0..3 {
        n.v[i] += dx[VEL + i];
        n.p[i] += dx[POS + i];
        n.bg[i] += dx[GYRO_BIAS + i];
        n.ba[i] += dx[ACCEL_BIAS + i];
    }
}

/// The rest of a scalar fusion once P H^T (`ph`) and the innovation variance `s` are known: the gate
/// (innovation^2 <= gate^2 s), K = P H^T / s, the state correction K innovation injected into the
/// nominal state, and P <- P - K (H P), with H P = (P H^T)^T by symmetry: the upper triangle is
/// computed and mirrored (symmetrised) and the diagonal held at or above 0.
#[inline(always)]
fn finish(n: &mut Nominal, p: &mut Cov, ph: &[f32; N], s: f32, innov: f32, gate: f32) -> bool {
    if s.is_nan() || s <= 0.0 || innov * innov > gate * gate * s {
        return false;
    }
    let inv = 1.0 / s;
    let mut k = [0.0f32; N];
    let mut dx = [0.0f32; N];
    for i in 0..N {
        k[i] = ph[i] * inv;
        dx[i] = k[i] * innov;
    }
    for i in 0..N {
        for j in i..N {
            let v = p[i][j] - k[i] * ph[j];
            p[i][j] = v;
            p[j][i] = v;
        }
        if p[i][i] < 0.0 {
            p[i][i] = 0.0;
        }
    }
    inject(n, &dx);
    true
}

/// One scalar fusion with a general (dense) measurement row `h`: innovation `innov` (measured minus
/// predicted), variance `var`, gate `gate` in sigmas. False (nothing changed) when gated out.
#[inline(never)]
pub fn fuse(n: &mut Nominal, p: &mut Cov, h: &[f32; N], innov: f32, var: f32, gate: f32) -> bool {
    let mut ph = [0.0f32; N];
    for i in 0..N {
        let mut s = 0.0;
        for j in 0..N {
            s += p[i][j] * h[j];
        }
        ph[i] = s;
    }
    let mut s = var;
    for i in 0..N {
        s += h[i] * ph[i];
    }
    finish(n, p, &ph, s, innov, gate)
}

/// One scalar fusion of a position component (`axis` 0 N, 1 E, 2 D: H has a single 1 at
/// `POS + axis`): P H^T is that column of P (its row, by symmetry), H P H^T its diagonal entry.
#[inline(never)]
pub fn fuse_position(n: &mut Nominal, p: &mut Cov, axis: usize, innov: f32, var: f32, gate: f32) -> bool {
    let k = POS + (axis % 3);
    let ph = p[k];
    let s = ph[k] + var;
    finish(n, p, &ph, s, innov, gate)
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    struct Rng(u32);
    impl Rng {
        fn next(&mut self) -> f32 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 17;
            self.0 ^= self.0 << 5;
            (self.0 as f32 / u32::MAX as f32) * 2.0 - 1.0
        }
    }

    const NOISE: Noise = Noise { gyro: 1e-6, accel: 2.5e-3, gyro_bias: 1e-8, accel_bias: 9e-6 };

    /// A symmetric positive definite P with realistic scales: L L^T + diag, rows scaled.
    fn random_p(rng: &mut Rng) -> Cov {
        let scale = [0.05f32, 0.05, 0.1, 0.3, 0.3, 0.3, 3.0, 3.0, 5.0, 1e-3, 1e-3, 1e-3, 0.03, 0.03, 0.03];
        let mut l = [[0.0f32; N]; N];
        for i in 0..N {
            for j in 0..=i {
                l[i][j] = rng.next() * if i == j { 1.0 } else { 0.4 };
            }
        }
        let mut p = [[0.0f32; N]; N];
        for i in 0..N {
            for j in 0..=i {
                let mut s = if i == j { 0.2 } else { 0.0 };
                for k in 0..N {
                    s += l[i][k] * l[j][k];
                }
                p[i][j] = s * scale[i] * scale[j];
                p[j][i] = p[i][j];
            }
        }
        p
    }

    fn random_nominal(rng: &mut Rng) -> Nominal {
        let q = normalized([1.0 + rng.next(), rng.next(), rng.next(), rng.next()]);
        Nominal {
            q,
            v: [rng.next() * 5.0, rng.next() * 5.0, rng.next()],
            p: [rng.next() * 50.0, rng.next() * 50.0, rng.next() * 10.0],
            bg: [rng.next() * 0.01, rng.next() * 0.01, rng.next() * 0.01],
            ba: [rng.next() * 0.2, rng.next() * 0.2, rng.next() * 0.2],
            dvel: [rng.next() * 0.03, rng.next() * 0.03, -0.098 + rng.next() * 0.03],
            dt_sum: 0.01,
        }
    }

    /// |a - b| <= tol (|b| + 1e-3 max|b|), element by element.
    fn assert_close(a: &[f32], b: &[f32], tol: f32, what: &str) -> f32 {
        let max = b.iter().fold(0.0f32, |m, x| m.max(x.abs()));
        let mut worst = 0.0f32;
        for (i, (x, y)) in a.iter().zip(b).enumerate() {
            let rel = (x - y).abs() / (y.abs() + 1e-3 * max);
            worst = worst.max(rel);
            assert!(rel <= tol, "{what}[{i}]: {x} vs {y}, relative {rel}");
        }
        worst
    }

    fn flat(p: &Cov) -> &[f32] {
        p.as_flattened()
    }

    #[test]
    fn structured_propagation_matches_dense() {
        let mut rng = Rng(0x1234_5678);
        let mut worst = 0.0f32;
        for _ in 0..500 {
            let n = random_nominal(&mut rng);
            let p0 = random_p(&mut rng);
            let dt = 0.01 * (1.0 + 0.1 * rng.next());
            let (mut a, mut b) = (p0, p0);
            covariance_predict(&mut a, &n.q, &n.dvel, dt, &NOISE);
            covariance_predict_dense(&mut b, &n.q, &n.dvel, dt, &NOISE);
            worst = worst.max(assert_close(flat(&a), flat(&b), 1e-4, "P"));
            for i in 0..N {
                for j in 0..N {
                    assert_eq!(a[i][j], a[j][i], "structured P not symmetric");
                }
            }
        }
        std::println!("structured vs dense: worst relative difference {worst:e}");
    }

    /// The textbook scalar update: K = P H^T / (H P H^T + R), P = (I - K H) P with full products,
    /// then (P + P^T) / 2 and the diagonal floor; dx = K innovation.
    fn textbook(p: &Cov, h: &[f32; N], innov: f32, var: f32) -> (Cov, [f32; N]) {
        let mut ph = [0.0f32; N];
        for i in 0..N {
            for j in 0..N {
                ph[i] += p[i][j] * h[j];
            }
        }
        let s: f32 = (0..N).map(|i| h[i] * ph[i]).sum::<f32>() + var;
        let k: [f32; N] = core::array::from_fn(|i| ph[i] / s);
        let mut ikh = [[0.0f32; N]; N];
        for i in 0..N {
            for j in 0..N {
                ikh[i][j] = f32::from(u8::from(i == j)) - k[i] * h[j];
            }
        }
        let mut out = [[0.0f32; N]; N];
        for i in 0..N {
            for j in 0..N {
                for m in 0..N {
                    out[i][j] += ikh[i][m] * p[m][j];
                }
            }
        }
        let mut sym = out;
        for i in 0..N {
            for j in 0..N {
                sym[i][j] = 0.5 * (out[i][j] + out[j][i]);
            }
            sym[i][i] = sym[i][i].max(0.0);
        }
        (sym, core::array::from_fn(|i| k[i] * innov))
    }

    fn check_fusion(n0: &Nominal, p0: &Cov, h: &[f32; N], n: &Nominal, p: &Cov, innov: f32, var: f32) {
        let (pt, dx) = textbook(p0, h, innov, var);
        assert_close(flat(p), flat(&pt), 1e-4, "P");
        let mut nt = *n0;
        inject(&mut nt, &dx);
        assert_close(&n.q, &nt.q, 1e-5, "q");
        let got: [f32; 12] = core::array::from_fn(|i| [n.v, n.p, n.bg, n.ba][i / 3][i % 3]);
        let want: [f32; 12] = core::array::from_fn(|i| [nt.v, nt.p, nt.bg, nt.ba][i / 3][i % 3]);
        assert_close(&got, &want, 1e-5, "state");
    }

    #[test]
    fn scalar_fusion_matches_textbook() {
        let mut rng = Rng(0x9e37_79b9);
        for round in 0..500 {
            let n0 = random_nominal(&mut rng);
            let p0 = random_p(&mut rng);
            let innov = rng.next() * 0.5;
            // Dense row.
            let h: [f32; N] = core::array::from_fn(|_| rng.next());
            let (mut n, mut p) = (n0, p0);
            assert!(fuse(&mut n, &mut p, &h, innov, 0.25, 1e3));
            check_fusion(&n0, &p0, &h, &n, &p, innov, 0.25);
            // Position row, and the same row through the general fusion.
            let axis = round % 3;
            let mut e = [0.0f32; N];
            e[POS + axis] = 1.0;
            let (mut n, mut p) = (n0, p0);
            assert!(fuse_position(&mut n, &mut p, axis, innov, 0.09, 1e3));
            check_fusion(&n0, &p0, &e, &n, &p, innov, 0.09);
            let (mut n2, mut p2) = (n0, p0);
            assert!(fuse(&mut n2, &mut p2, &e, innov, 0.09, 1e3));
            assert_eq!((n, p), (n2, p2));
        }
    }

    #[test]
    fn gate_refuses_and_leaves_state() {
        let mut rng = Rng(7);
        let (n0, p0) = (random_nominal(&mut rng), random_p(&mut rng));
        let (mut n, mut p) = (n0, p0);
        let innov = 6.0 * sqrtf(p0[POS][POS] + 0.09);
        assert!(!fuse_position(&mut n, &mut p, 0, innov, 0.09, 5.0));
        assert_eq!((n, p), (n0, p0));
    }

    #[test]
    fn nominal_step_integrates_rate_and_gravity() {
        // Level, at rest: specific force -g on body z; 1 rad/s about z for 803.5 samples (1 s).
        let mut n = Nominal {
            q: [1.0, 0.0, 0.0, 0.0],
            v: [0.0; 3],
            p: [0.0; 3],
            bg: [0.0; 3],
            ba: [0.0; 3],
            dvel: [0.0; 3],
            dt_sum: 0.0,
        };
        let dt = 1.0 / 803.5;
        for _ in 0..803 {
            nominal_step(&mut n, &[0.0, 0.0, 1.0], &[0.0, 0.0, -GRAVITY_MSS], dt);
        }
        nominal_step(&mut n, &[0.0, 0.0, 1.0], &[0.0, 0.0, -GRAVITY_MSS], 0.5 * dt);
        let yaw = 2.0 * libm::atan2f(n.q[3], n.q[0]);
        assert!((yaw - 1.0).abs() < 1e-4, "yaw {yaw}");
        assert!(n.v.iter().chain(&n.p).all(|x| x.abs() < 1e-4), "{:?} {:?}", n.v, n.p);
        assert!((n.dt_sum - 1.0).abs() < 1e-4);
    }
}
