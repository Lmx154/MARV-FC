//! The RPM notch of the rate loop (decision-rpm-notch-20261123, sections 1 and 2): per axis a bank of `LINES` x
//! `HARMONICS` RBJ notch biquads (direct form I, f32) whose centres follow the rotors' frequencies. The reference is the
//! what-if `t292-notch.py`: each stage is the mix y = x + w (notch(x) - x) with the Betaflight weight rule, the stages
//! in series (harmonic outer, line inner), each stage's state holding its own input and its own unweighted output.
//! The coefficients are the same for the three axes, so they are made once per step (`Notch::set_rotor_hz`).

use crate::tuning::Tuning;
use fw_core::esc::LINES;

/// The most harmonics per line: the multiple-angle identities of `multiple` stop at the third.
pub const HARMONICS: usize = 3;

/// A frequency that may centre a notch: finite and above 0. Everything else (no eRPM, eRPM 0) is "no usable value".
pub fn usable(f1_hz: Option<f32>) -> Option<f32> {
    f1_hz.filter(|f| f.is_finite() && *f > 0.0)
}

/// One stage's coefficients: b0 (= b2), b1 (= a1) and a2 of the normalised RBJ notch, and the weight of the mix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Coef {
    pub(crate) b0: f32,
    pub(crate) b1: f32,
    pub(crate) a2: f32,
    pub(crate) w: f32,
}

/// The RBJ notch for a centre with sine `sin_w` and cosine `cos_w` of 2 pi f / fs: alpha = sin / (2 q), b = [1, -2 cos, 1]
/// / (1 + alpha), a = [1, -2 cos / (1 + alpha), (1 - alpha) / (1 + alpha)]; weight `w`.
pub(crate) fn coef(sin_w: f32, cos_w: f32, q: f32, w: f32) -> Coef {
    let alpha = sin_w / (2.0 * q);
    let inv = 1.0 / (1.0 + alpha);
    Coef {
        b0: inv,
        b1: -2.0 * cos_w * inv,
        a2: (1.0 - alpha) * inv,
        w,
    }
}

/// Sine and cosine of `h` times the angle whose sine and cosine are `s` and `c`, for h = 1, 2, 3.
fn multiple(s: f32, c: f32, h: usize) -> (f32, f32) {
    match h {
        1 => (s, c),
        2 => (2.0 * s * c, 2.0 * c * c - 1.0),
        _ => (s * (3.0 - 4.0 * s * s), c * (4.0 * c * c - 3.0)),
    }
}

/// One stage's memory: its last two inputs and its last two (unweighted) outputs.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Delay {
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
}

impl Delay {
    const ZERO: Delay = Delay {
        x1: 0.0,
        x2: 0.0,
        y1: 0.0,
        y2: 0.0,
    };
}

/// One axis: the memory of every line and harmonic.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Bank {
    delay: [[Delay; LINES]; HARMONICS],
}

impl Bank {
    const ZERO: Bank = Bank {
        delay: [[Delay::ZERO; LINES]; HARMONICS],
    };

    /// `x` through the first `harmonics` rows of stages. `seed`: the first sample after a reset, each stage's memory
    /// starts at its own input (the reference starts its filters at the first sample). A stage of weight 0 passes its
    /// input on untouched (a true bypass: `x + 0 (n - x)` would turn -0.0 into 0.0 and a NaN into NaN) but still
    /// runs, so its memory is current when its weight rises.
    fn apply(
        &mut self,
        x: f32,
        coefs: &[[Coef; LINES]; HARMONICS],
        harmonics: usize,
        seed: bool,
    ) -> f32 {
        let mut v = x;
        for (d_row, c_row) in self.delay.iter_mut().zip(coefs.iter()).take(harmonics) {
            for (d, c) in d_row.iter_mut().zip(c_row.iter()) {
                if seed {
                    *d = Delay {
                        x1: v,
                        x2: v,
                        y1: v,
                        y2: v,
                    };
                }
                let y = c.b0 * v + c.b1 * d.x1 + c.b0 * d.x2 - c.b1 * d.y1 - c.a2 * d.y2;
                d.x2 = d.x1;
                d.x1 = v;
                d.y2 = d.y1;
                d.y1 = y;
                if c.w > 0.0 {
                    v += c.w * (y - v);
                }
            }
        }
        v
    }
}

/// The notch of the three rate axes.
#[derive(Clone, Copy, Debug)]
pub struct Notch {
    /// Harmonics in use, 1 to `HARMONICS`.
    harmonics: usize,
    q: f32,
    min_hz: f32,
    fade_hz: f32,
    /// Highest centre, Hz: `notch_max_fs` x the sample rate.
    max_hz: f32,
    /// 2 pi / the sample rate, rad per Hz.
    rad_per_hz: f32,
    /// Sine and cosine of the angle of the lowest and of the highest centre.
    lo_trig: (f32, f32),
    hi_trig: (f32, f32),
    /// The coefficients by harmonic, then line.
    coefs: [[Coef; LINES]; HARMONICS],
    /// Roll, pitch, yaw.
    banks: [Bank; 3],
    /// The memory has been started by a sample since the last reset.
    seeded: bool,
}

impl Notch {
    /// A notch for a sample rate of `fs_hz` (the configured IMU1 rate), the constants from `tuning`. Every weight is 0 and
    /// every centre is the lowest one.
    pub fn new(tuning: &Tuning, fs_hz: f32) -> Self {
        let rad_per_hz = core::f32::consts::TAU / fs_hz;
        let max_hz = tuning.notch_max_fs * fs_hz;
        let trig = |hz: f32| (libm::sinf(rad_per_hz * hz), libm::cosf(rad_per_hz * hz));
        let lo_trig = trig(tuning.notch_min_hz);
        let mut n = Notch {
            harmonics: tuning.notch_harmonics.clamp(1, HARMONICS),
            q: tuning.notch_q,
            min_hz: tuning.notch_min_hz,
            fade_hz: tuning.notch_fade_hz,
            max_hz,
            rad_per_hz,
            lo_trig,
            hi_trig: trig(max_hz),
            coefs: [[coef(0.0, 1.0, 1.0, 0.0); LINES]; HARMONICS],
            banks: [Bank::ZERO; 3],
            seeded: false,
        };
        n.reset();
        n
    }

    /// The memory and the centres go back to the state of `new`: no weight, every centre the lowest, the next sample
    /// starts the memory.
    pub fn reset(&mut self) {
        let (s, c) = self.lo_trig;
        self.coefs = [[coef(s, c, self.q, 0.0); LINES]; HARMONICS];
        self.banks = [Bank::ZERO; 3];
        self.seeded = false;
    }

    /// The rotor frequencies f1, Hz, by DShot line, for the next `apply`. Harmonic h of a line has the centre
    /// clamp(h f1, `notch_min_hz`, `notch_max_fs` fs) and the weight clamp((h f1 - `notch_min_hz`) / `notch_fade_hz`, 0, 1).
    /// A line without a usable value (`usable`) gets weight 0 and keeps its centre. One sine and cosine per line; the
    /// second and third harmonic come from the multiple-angle identities, a clamped centre from the two precomputed pairs.
    pub fn set_rotor_hz(&mut self, f1_hz: [Option<f32>; LINES]) {
        let mut line = [(None, (0.0, 0.0)); LINES];
        for (l, f) in line.iter_mut().zip(f1_hz) {
            if let Some(f) = usable(f) {
                // Above the highest centre every harmonic is clamped, so the angle is not needed.
                let angle = if f < self.max_hz {
                    (libm::sinf(self.rad_per_hz * f), libm::cosf(self.rad_per_hz * f))
                } else {
                    (0.0, 0.0)
                };
                *l = (Some(f), angle);
            }
        }
        for (row, h) in self.coefs.iter_mut().zip(1usize..).take(self.harmonics) {
            for (c, (f, (s1, c1))) in row.iter_mut().zip(line) {
                let Some(f) = f else {
                    c.w = 0.0;
                    continue;
                };
                let fc = h as f32 * f;
                // `f32::clamp` with bounds that are not constants keeps its panic (tools/check-ram-code.py).
                let w = ((fc - self.min_hz) / self.fade_hz).max(0.0).min(1.0);
                let (s, co) = if fc >= self.max_hz {
                    self.hi_trig
                } else if fc <= self.min_hz {
                    self.lo_trig
                } else {
                    multiple(s1, c1, h)
                };
                *c = coef(s, co, self.q, w);
            }
        }
    }

    /// The rates, rad/s, through every stage of their axis.
    pub fn apply(&mut self, rates_rads: [f32; 3]) -> [f32; 3] {
        let seed = !self.seeded;
        self.seeded = true;
        let mut out = rates_rads;
        for (o, bank) in out.iter_mut().zip(self.banks.iter_mut()) {
            *o = bank.apply(*o, &self.coefs, self.harmonics, seed);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f32 = 800.0;

    fn notch() -> Notch {
        Notch::new(&Tuning::DEFAULT, FS)
    }

    /// The RBJ notch's magnitude from its definition, in f64: with c = cos w - cos w0,
    /// |H| = |c| / sqrt(c^2 + (alpha sin w)^2).
    fn rbj_magnitude(f_hz: f64, centre_hz: f64, q: f64) -> f64 {
        let (w, w0) = (
            core::f64::consts::TAU * f_hz / f64::from(FS),
            core::f64::consts::TAU * centre_hz / f64::from(FS),
        );
        let alpha = w0.sin() / (2.0 * q);
        let c = w.cos() - w0.cos();
        c.abs() / (c * c + (alpha * w.sin()).powi(2)).sqrt()
    }

    /// The magnitude of the transfer function of `c` (b0, b1, b0 over 1, b1, a2) at `f_hz`, in f64.
    fn coef_magnitude(c: &Coef, f_hz: f64) -> f64 {
        let w = core::f64::consts::TAU * f_hz / f64::from(FS);
        let z = |k: f64| (libm::cos(k * w), -libm::sin(k * w));
        let (b0, b1, a2) = (f64::from(c.b0), f64::from(c.b1), f64::from(c.a2));
        let (z1, z2) = (z(1.0), z(2.0));
        let num = (b0 + b1 * z1.0 + b0 * z2.0, b1 * z1.1 + b0 * z2.1);
        let den = (1.0 + b1 * z1.0 + a2 * z2.0, b1 * z1.1 + a2 * z2.1);
        (num.0.hypot(num.1)) / (den.0.hypot(den.1))
    }

    #[test]
    fn the_biquad_matches_the_rbj_notch_formula_at_five_frequencies() {
        let centre = 120.0_f32;
        let w0 = core::f32::consts::TAU * centre / FS;
        let c = coef(libm::sinf(w0), libm::cosf(w0), 5.0, 1.0);
        for f in [30.0_f64, 100.0, 115.0, 120.0, 250.0] {
            let want = rbj_magnitude(f, f64::from(centre), 5.0);
            let got = coef_magnitude(&c, f);
            assert!((got - want).abs() <= 1e-4 * want + 1e-6, "{f} Hz: {got} vs {want}");
        }
        // The notch is deep at the centre and has unit gain at DC.
        assert!(coef_magnitude(&c, 120.0) < 1e-4);
        assert!((coef_magnitude(&c, 0.0) - 1.0).abs() < 1e-6);
    }

    /// Steady-state amplitude of a unit sine of `f_hz` (a multiple of 0.5 Hz) through the filter, measured in the time
    /// domain: the rms over the last 1600 samples, a whole number of cycles, times sqrt 2.
    fn gain(n: &mut Notch, f_hz: f32) -> f32 {
        let mut sum = 0.0_f64;
        for i in 0..3200 {
            let x = libm::sinf(core::f32::consts::TAU * f_hz * i as f32 / FS);
            let y = n.apply([x, 0.0, 0.0])[0];
            if i >= 1600 {
                sum += f64::from(y) * f64::from(y);
            }
        }
        ((sum / 1600.0).sqrt() * core::f64::consts::SQRT_2) as f32
    }

    #[test]
    fn the_bank_filters_a_sine_by_the_product_of_its_stages_magnitudes() {
        // Line 0 at 100 Hz, full weight on all three harmonics (100, 200, 300 Hz); the other lines have no value.
        let mut want_f = [(20.0_f32, 0.0), (100.0, 0.0), (150.0, 0.0), (200.0, 0.0), (300.0, 0.0), (350.0, 0.0)];
        for (f, want) in want_f.iter_mut() {
            *want = [100.0_f64, 200.0, 300.0]
                .iter()
                .map(|c| rbj_magnitude(f64::from(*f), *c, 5.0))
                .product::<f64>() as f32;
        }
        for (f, want) in want_f {
            let mut n = notch();
            n.set_rotor_hz([Some(100.0), None, None, None]);
            let got = gain(&mut n, f);
            assert!((got - want).abs() <= 2e-3, "{f} Hz: {got} vs {want}");
        }
    }

    #[test]
    fn the_second_and_third_harmonic_coefficients_equal_the_direct_ones() {
        let mut n = notch();
        for f1 in [20.0_f32, 31.0, 55.0, 80.0, 97.5, 120.0, 127.0, 130.0, 300.0, 383.0, 500.0] {
            n.set_rotor_hz([Some(f1), Some(f1), Some(f1), Some(f1)]);
            for (row, h) in n.coefs.iter().zip(1..=3) {
                let fc = (h as f64 * f64::from(f1)).clamp(50.0, 0.48 * f64::from(FS));
                let w = (fc * core::f64::consts::TAU / f64::from(FS)) as f64;
                let alpha = w.sin() / 10.0;
                let want = (1.0 / (1.0 + alpha), -2.0 * w.cos() / (1.0 + alpha), (1.0 - alpha) / (1.0 + alpha));
                let got = (f64::from(row[0].b0), f64::from(row[0].b1), f64::from(row[0].a2));
                for (g, w) in [(got.0, want.0), (got.1, want.1), (got.2, want.2)] {
                    assert!((g - w).abs() <= 1e-4 * w.abs().max(1e-2), "f1 {f1} h {h}: {g} vs {w}");
                }
            }
        }
    }

    #[test]
    fn the_weight_of_each_harmonic_rises_over_ten_hertz_from_fifty() {
        let mut n = notch();
        n.set_rotor_hz([Some(16.0), Some(20.0), Some(27.5), None]);
        let w = |n: &Notch, h: usize| n.coefs[h].map(|c| c.w);
        // h1: 16, 20, 27.5 Hz are below 50; h2: 32, 40, 55; h3: 48, 60, 82.5.
        assert_eq!(w(&n, 0), [0.0, 0.0, 0.0, 0.0]);
        assert_eq!(w(&n, 1), [0.0, 0.0, 0.5, 0.0]);
        assert_eq!(w(&n, 2), [0.0, 1.0, 1.0, 0.0]);
        n.set_rotor_hz([Some(50.0), Some(55.0), Some(60.0), Some(0.0)]);
        assert_eq!(w(&n, 0), [0.0, 0.5, 1.0, 0.0]);
    }

    #[test]
    fn a_line_without_a_usable_value_keeps_its_centre_and_the_others_are_unaffected() {
        let mut n = notch();
        n.set_rotor_hz([Some(100.0), Some(110.0), Some(120.0), Some(130.0)]);
        let before = n.coefs;
        n.set_rotor_hz([Some(100.0), None, Some(f32::NAN), Some(-5.0)]);
        for (row, old) in n.coefs.iter().zip(before.iter()) {
            assert_eq!(row[0], old[0], "line 0 is as before");
            for l in 1..LINES {
                assert_eq!((row[l].b0, row[l].b1, row[l].a2), (old[l].b0, old[l].b1, old[l].a2), "line {l} centre");
                assert_eq!(row[l].w, 0.0, "line {l} weight");
            }
        }
    }

    #[test]
    fn with_no_weight_the_input_comes_out_bit_for_bit() {
        let mut n = notch();
        n.set_rotor_hz([None; LINES]);
        for x in [[-0.0_f32, 0.0, 1.5], [f32::NAN, f32::INFINITY, -2.0e-30], [0.3, -0.7, 1e20]] {
            let y = n.apply(x);
            for (a, b) in x.iter().zip(y.iter()) {
                assert_eq!(a.to_bits(), b.to_bits());
            }
        }
    }

    #[test]
    fn reset_returns_the_notch_to_its_new_state() {
        let mut a = notch();
        let b = notch();
        a.set_rotor_hz([Some(100.0); LINES]);
        a.apply([1.0, 2.0, 3.0]);
        a.apply([0.5, 0.1, 0.0]);
        a.reset();
        assert_eq!((a.coefs, a.banks, a.seeded), (b.coefs, b.banks, b.seeded));
    }
}
