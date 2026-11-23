//! Heading integrity monitor (decision-heading-monitor-20261123, H0-H6 with the addenda H7, H8 and H9, which override the
//! earlier text where they differ). A pure, read-only module: it never writes the filter and nothing calls it yet (HM2 wires it).
//!
//! Sign: d_hat = true yaw minus the filter's yaw, in the filter's convention (yaw positive from north toward east, i.e.
//! clockwise seen from above; eskf.rs:944 `yaw = atan2(r[1][0], r[0][0])`). B is A rotated by +d_hat in the (N, E) plane,
//! which is what the test `b_rotated_by_d_gives_d_hat` builds and recovers (a vehicle whose true nose is d clockwise of the
//! filter's yaw sees its acceleration turned d clockwise in N, E).
//!
//! Principle: does the filter's yaw carry the strapdown velocity increment A (N, E; the IMU's measured acceleration through
//! the filter attitude) onto the GNSS-measured velocity change B? With the horizontal plane as complex numbers a yaw error d
//! multiplies A by e^{jd}. Per pair of accepted GNSS velocity epochs, with lam = exp(-dt/TAU):
//! Sc = lam Sc + (A_N B_E - A_E B_N), Sd = lam Sd + A.B, Saa = lam Saa + |A|^2; d_hat = atan2(Sc, Sd), sig = sigB / sqrt(Saa).
//!
//! Rules (all from the decision text): R1 a pair needs two consecutive accepted epochs, A is zeroed at each accepted epoch,
//! a broken chain gives no pair (H7). R2 no pair when ||B| - |A|| > GATE x sigB (H7), R2' a pair screened by R2 also breaks
//! the chain, so the bad epoch leaves both differences it enters (H8 (2)). R3 fault = informed AND |d_hat| > D_TOL AND
//! |d_hat| > GATE x sig on every evaluated epoch for T_FAULT; a screened pair neither counts nor resets the count (H7, H8).
//! H10 (1): confirmed = informed AND |d_hat| + CONF_SIGMA x sig <= D_TOL on every evaluated epoch for T_FAULT (same
//! accumulated-time rule), latched. H10 (2): `blind` = airborne and no pair evaluated for `nav_aid_timeout_us`; cleared by the
//! next evaluated pair; `heading_ok` is false while blind. Review items: a non-finite velocity or increment breaks the chain
//! and never enters the sums; a pair whose two epochs are more than the filter's interval rule apart (H10 (i)-(iii): 1 /
//! `nav_gps_min_rate_hz`, 245 ms) is not evaluated and breaks the chain.
//! R4 sums and tests run only while airborne and armed, the sums are cleared at take-off (H8 (2)). Both latches and the sums
//! clear at the arming edge (H3). `heading_ok` = the filter's heading flag AND confirmed AND NOT fault (H0).
//!
//! f32, no heap, no statics, no clock (the caller passes the epoch time), no log. Per epoch: one `atan2f`, one `expf`, three
//! `sqrtf` (|A|, |B|, sqrt Saa).

use crate::tuning::Tuning;

/// Largest filter yaw error tolerated, rad: 25 deg (H4: PX4 isYawFailure, px4-gps_control.cpp:573).
pub const D_TOL_RAD: f32 = 25.0 * core::f32::consts::PI / 180.0;
/// Largest sig for an epoch to be informed, rad: 15 deg (H4: PX4 EKFGSF_yaw_err_max, px4-common.h:395).
pub const SIG_CONF_RAD: f32 = 15.0 * core::f32::consts::PI / 180.0;
/// A fault needs the condition on every evaluated epoch for this long, us: 1.0 s (H4: PX4 EKFGSF_reset_delay,
/// px4-common.h:394). The intervals of the flagged evaluated epochs add up (screened pairs neither add nor reset), so five
/// 0.2 s epochs make it.
pub const T_FAULT_US: u32 = 1_000_000;
/// Memory time constant of the sums, s: 5 s (H4: `nav_aid_timeout_us`, tuning.rs:228).
pub const TAU_S: f32 = 5.0;
/// R2 and R3 gate, in sigmas: the filter's `nav_gate_sigma` (H7 R2, H8 R3; tuning.rs:227).
pub const GATE_SIGMA: f32 = Tuning::DEFAULT.nav_gate_sigma;
/// Noise of B per axis, m/s: 0.1085 (H9 / H4: MEASURED, flights/20261123-headmon round 3, t7_round3.out "binding sigB").
pub const SIG_B_DEFAULT: f32 = 0.1085;
/// Margin of the confirmation test, in sigmas: 2.58 (H10 (1); H4's p99 calibration of d_hat/sig).
pub const CONF_SIGMA: f32 = 2.58;
/// Longest time between the two epochs of a pair, us: the filter's interval rule 1 / `nav_gps_min_rate_hz` = 245 ms (H10).
pub const MAX_DT_US: u32 = (1.0e6 / Tuning::DEFAULT.nav_gps_min_rate_hz + 0.5) as u32;
/// Time without an evaluated pair, airborne, after which the monitor is blind, us: `nav_aid_timeout_us` (H10 (2)).
pub const BLIND_US: u32 = Tuning::DEFAULT.nav_aid_timeout_us as u32;

/// The monitor's output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadingMon {
    /// confirmed AND NOT fault (the monitor's own part of `heading_ok`).
    pub ok: bool,
    /// An informed epoch has had |d_hat| <= D_TOL since the arming edge (latched).
    pub confirmed: bool,
    /// The R3 fault has been raised since the arming edge (latched).
    pub fault: bool,
    /// d_hat of the last evaluated epoch, rad (filter yaw error, predicted -> measured).
    pub delta_rad: f32,
    /// sig of the last evaluated epoch, rad (INFINITY with no information).
    pub sigma_rad: f32,
    /// Saa of the last evaluated epoch, (m/s)^2.
    pub info: f32,
    /// Pairs screened by R2 since the arming edge.
    pub screened_r2: u32,
    /// Pairs dropped because the previous pair was screened by R2 (R2'), since the arming edge.
    pub dropped_r2p: u32,
    /// Epochs or increments with a non-finite value (chain broken), since the arming edge.
    pub non_finite: u32,
    /// Pairs not evaluated because the epochs were more than `MAX_DT_US` apart, since the arming edge.
    pub out_of_interval: u32,
    /// Airborne and no pair evaluated for `BLIND_US`; cleared by the next evaluated pair (H10 (2)).
    pub blind: bool,
}

/// The monitor's state (at most 128 B).
#[derive(Clone, Copy, Debug)]
pub struct HeadingMonitor {
    sig_b: f32,
    sc: f32,
    sd: f32,
    saa: f32,
    a_n: f32,
    a_e: f32,
    v_n: f32,
    v_e: f32,
    delta: f32,
    sigma: f32,
    t_prev_us: u32,
    flagged_us: u32,
    confirm_us: u32,
    last_eval_us: u32,
    non_finite: u32,
    out_of_interval: u32,
    blind: bool,
    screened: u32,
    dropped: u32,
    armed: bool,
    have_prev: bool,
    prev_air: bool,
    prev_bad: bool,
    confirmed: bool,
    fault: bool,
}

impl Default for HeadingMonitor {
    fn default() -> Self {
        Self::new(SIG_B_DEFAULT)
    }
}

impl HeadingMonitor {
    /// A cleared monitor, disarmed. `sig_b` is the noise of B per axis, m/s.
    pub const fn new(sig_b: f32) -> Self {
        Self {
            sig_b,
            sc: 0.0,
            sd: 0.0,
            saa: 0.0,
            a_n: 0.0,
            a_e: 0.0,
            v_n: 0.0,
            v_e: 0.0,
            delta: 0.0,
            sigma: f32::INFINITY,
            t_prev_us: 0,
            flagged_us: 0,
            confirm_us: 0,
            last_eval_us: 0,
            non_finite: 0,
            out_of_interval: 0,
            blind: false,
            screened: 0,
            dropped: 0,
            armed: false,
            have_prev: false,
            prev_air: false,
            prev_bad: false,
            confirmed: false,
            fault: false,
        }
    }

    /// The arming state, every step. The rising edge clears the latches, the sums, the chain and the counters.
    pub fn set_armed(&mut self, armed: bool) {
        if armed && !self.armed {
            *self = Self::new(self.sig_b);
        }
        self.armed = armed;
    }

    /// Add the strapdown velocity increment (N, E, m/s) of one prediction step, taken before any correction.
    pub fn add_increment(&mut self, n: f32, e: f32) {
        self.a_n += n;
        self.a_e += e;
    }

    /// The GNSS chain is broken (a missing, repeated, frozen or out-of-interval epoch) at `t_us` (wrapping microseconds): the
    /// next accepted epoch gives no pair. The time argument lets the blind timer run while no epoch arrives.
    pub fn chain_broken(&mut self, t_us: u32) {
        self.have_prev = false;
        self.prev_bad = false;
        self.a_n = 0.0;
        self.a_e = 0.0;
        self.advance_blind(t_us, self.prev_air, self.prev_air);
    }

    fn advance_blind(&mut self, t_us: u32, air: bool, was_air: bool) {
        if !air || !was_air {
            self.last_eval_us = t_us;
            self.blind = false;
        } else if t_us.wrapping_sub(self.last_eval_us) >= BLIND_US {
            self.blind = true;
        }
    }

    fn break_chain_at(&mut self, t_us: u32, air: bool, was_air: bool) {
        self.have_prev = false;
        self.prev_bad = false;
        self.prev_air = air;
        self.advance_blind(t_us, air, was_air);
    }

    /// An epoch the GNSS job accepted: velocity (N, E) m/s at `t_us` (wrapping microseconds). Takes and zeroes A.
    pub fn accepted_epoch(&mut self, t_us: u32, vn: f32, ve: f32, airborne: bool) {
        let air = airborne && self.armed;
        let was_air = self.prev_air;
        let (an, ae) = (self.a_n, self.a_e);
        self.a_n = 0.0;
        self.a_e = 0.0;
        if !(vn.is_finite() && ve.is_finite() && an.is_finite() && ae.is_finite()) {
            self.non_finite = self.non_finite.wrapping_add(1);
            self.t_prev_us = t_us;
            self.break_chain_at(t_us, air, was_air);
            return;
        }
        let (pvn, pve) = (self.v_n, self.v_e);
        let had = self.have_prev && self.prev_air;
        let dt_us = t_us.wrapping_sub(self.t_prev_us);
        self.v_n = vn;
        self.v_e = ve;
        self.t_prev_us = t_us;
        self.have_prev = true;
        self.prev_air = air;
        self.advance_blind(t_us, air, was_air);
        if !air {
            self.sc = 0.0;
            self.sd = 0.0;
            self.saa = 0.0;
            self.delta = 0.0;
            self.sigma = f32::INFINITY;
            self.flagged_us = 0;
            self.confirm_us = 0;
            self.prev_bad = false;
            return;
        }
        if !had {
            self.prev_bad = false;
            return;
        }
        if dt_us > MAX_DT_US {
            self.out_of_interval = self.out_of_interval.wrapping_add(1);
            self.prev_bad = false;
            return;
        }
        let (bn, be) = (vn - pvn, ve - pve);
        let nb = libm::sqrtf(bn * bn + be * be);
        let na = libm::sqrtf(an * an + ae * ae);
        if (nb - na).abs() > GATE_SIGMA * self.sig_b {
            self.prev_bad = true;
            self.screened = self.screened.wrapping_add(1);
            return;
        }
        if self.prev_bad {
            self.prev_bad = false;
            self.dropped = self.dropped.wrapping_add(1);
            return;
        }
        let lam = libm::expf(-(dt_us as f32) * 1e-6 / TAU_S);
        let sc = lam * self.sc + (an * be - ae * bn);
        let sd = lam * self.sd + (an * bn + ae * be);
        let saa = lam * self.saa + (an * an + ae * ae);
        if !(sc.is_finite() && sd.is_finite() && saa.is_finite()) {
            self.non_finite = self.non_finite.wrapping_add(1);
            self.break_chain_at(t_us, air, was_air);
            return;
        }
        self.sc = sc;
        self.sd = sd;
        self.saa = saa;
        self.last_eval_us = t_us;
        self.blind = false;
        self.delta = libm::atan2f(self.sc, self.sd);
        self.sigma = if self.saa > 0.0 { self.sig_b / libm::sqrtf(self.saa) } else { f32::INFINITY };
        let informed = self.sigma <= SIG_CONF_RAD;
        let ad = self.delta.abs();
        if informed && ad + CONF_SIGMA * self.sigma <= D_TOL_RAD {
            self.confirm_us = self.confirm_us.saturating_add(dt_us);
            if self.confirm_us >= T_FAULT_US {
                self.confirmed = true;
            }
        } else {
            self.confirm_us = 0;
        }
        if informed && ad > D_TOL_RAD && ad > GATE_SIGMA * self.sigma {
            self.flagged_us = self.flagged_us.saturating_add(dt_us);
            if self.flagged_us >= T_FAULT_US {
                self.fault = true;
            }
        } else {
            self.flagged_us = 0;
        }
    }

    /// The monitor's output.
    pub fn output(&self) -> HeadingMon {
        HeadingMon {
            ok: self.confirmed && !self.fault && !self.blind,
            confirmed: self.confirmed,
            fault: self.fault,
            delta_rad: self.delta,
            sigma_rad: self.sigma,
            info: self.saa,
            screened_r2: self.screened,
            dropped_r2p: self.dropped,
            non_finite: self.non_finite,
            out_of_interval: self.out_of_interval,
            blind: self.blind,
        }
    }

    /// `heading_ok` = the filter's heading flag AND confirmed AND NOT fault AND NOT blind (H0, H10).
    pub fn heading_ok(&self, heading_flag: bool) -> bool {
        heading_flag && self.confirmed && !self.fault && !self.blind
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::f32::consts::PI;
    use std::vec::Vec;

    const D2R: f32 = PI / 180.0;
    const DT_US: u32 = 200_000;

    /// A test stand: epochs 0.2 s apart; B is fed as the velocity change, A as the increment.
    struct Rig {
        m: HeadingMonitor,
        t: u32,
        v: (f32, f32),
        k: usize,
    }

    fn rot(a: (f32, f32), d: f32) -> (f32, f32) {
        let (s, c) = (libm::sinf(d), libm::cosf(d));
        (c * a.0 - s * a.1, s * a.0 + c * a.1)
    }

    /// A of magnitude `mag`, turning a quarter turn every epoch so that the velocity stays bounded.
    fn a_of(k: usize, mag: f32) -> (f32, f32) {
        match k % 4 {
            0 => (mag, 0.0),
            1 => (0.0, mag),
            2 => (-mag, 0.0),
            _ => (0.0, -mag),
        }
    }

    impl Rig {
        fn new() -> Self {
            let mut m = HeadingMonitor::default();
            m.set_armed(true);
            Rig { m, t: 1_000_000, v: (0.0, 0.0), k: 0 }
        }
        /// One accepted epoch with the increment `a` and the velocity change `b`.
        fn ep(&mut self, a: (f32, f32), b: (f32, f32), air: bool) {
            self.t = self.t.wrapping_add(DT_US);
            self.v = (self.v.0 + b.0, self.v.1 + b.1);
            self.m.add_increment(a.0 * 0.4, a.1 * 0.4);
            self.m.add_increment(a.0 * 0.6, a.1 * 0.6);
            self.m.accepted_epoch(self.t, self.v.0, self.v.1, air);
        }
        /// `n` airborne epochs with B = A rotated by `d` rad, |A| = `mag`.
        fn run(&mut self, n: usize, mag: f32, d: f32) {
            for _ in 0..n {
                let a = a_of(self.k, mag);
                self.k += 1;
                self.ep(a, rot(a, d), true);
            }
        }
    }

    fn wrap(x: f32) -> f32 {
        let mut x = x;
        while x > PI {
            x -= 2.0 * PI;
        }
        while x < -PI {
            x += 2.0 * PI;
        }
        x
    }

    #[test]
    fn state_is_at_most_128_bytes() {
        std::println!("size_of HeadingMonitor {}", core::mem::size_of::<HeadingMonitor>());
        assert!(core::mem::size_of::<HeadingMonitor>() <= 128, "{}", core::mem::size_of::<HeadingMonitor>());
    }

    #[test]
    fn gate_is_the_filters() {
        assert_eq!(GATE_SIGMA, 5.0);
    }

    #[test]
    fn b_rotated_by_d_gives_d_hat() {
        let mut d = -170;
        while d <= 180 {
            let mut r = Rig::new();
            r.ep((0.0, 0.0), (0.0, 0.0), true);
            r.run(30, 0.5, d as f32 * D2R);
            let o = r.m.output();
            assert_eq!(o.screened_r2, 0, "d {d}");
            assert!(wrap(o.delta_rad - d as f32 * D2R).abs() <= o.sigma_rad.max(0.002), "d {d}: {} sig {}", o.delta_rad / D2R, o.sigma_rad);
            assert!(o.sigma_rad < SIG_CONF_RAD);
            d += 10;
        }
    }

    #[test]
    fn without_a_there_is_no_information_and_no_fault() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        for k in 0..200 {
            let b = rot((0.4, 0.0), k as f32 * 0.7);
            r.ep((0.0, 0.0), b, true);
            let o = r.m.output();
            assert!(o.sigma_rad.is_infinite());
            assert_eq!(o.info, 0.0);
            assert!(!o.confirmed && !o.fault && !o.ok);
        }
        // a large B with A = 0 is screened (R2), also without effect
        for _ in 0..20 {
            r.ep((0.0, 0.0), (3.0, 0.0), true);
            r.ep((0.0, 0.0), (-3.0, 0.0), true);
        }
        let o = r.m.output();
        assert!(o.screened_r2 > 0 && !o.confirmed && !o.fault);
    }

    #[test]
    fn latches_hold_until_the_arming_edge() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(10, 0.5, 0.0);
        assert!(r.m.output().confirmed && !r.m.output().fault);
        r.run(60, 0.5, 90.0 * D2R);
        let o = r.m.output();
        assert!(o.fault && o.confirmed && !o.ok);
        // back to a good heading, quiet spells, a disarm: both latches hold
        r.run(200, 0.5, 0.0);
        for _ in 0..50 {
            r.ep((0.0, 0.0), (0.0, 0.0), true);
        }
        r.m.set_armed(false);
        let o = r.m.output();
        assert!(o.fault && o.confirmed);
        r.m.set_armed(true);
        let o = r.m.output();
        assert!(!o.fault && !o.confirmed && o.info == 0.0 && o.screened_r2 == 0);
    }

    #[test]
    fn confirm_rule() {
        // informed with |d_hat| <= 25 deg confirms; informed beyond 25 deg does not
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(10, 0.5, 10.0 * D2R);
        assert!(r.m.output().confirmed);
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(10, 0.5, 40.0 * D2R);
        assert!(r.m.output().sigma_rad <= SIG_CONF_RAD && !r.m.output().confirmed);
        // not informed (small A) does not confirm, however good d_hat looks; confirms once the sums carry enough A
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(1, 0.2, 0.0);
        assert!(r.m.output().sigma_rad > SIG_CONF_RAD && !r.m.output().confirmed);
        r.run(400, 0.2, 0.0);
        let o = r.m.output();
        assert!(o.sigma_rad <= SIG_CONF_RAD && o.confirmed);
        // a pair on the ground, or before arming, confirms nothing
        let mut m = HeadingMonitor::default();
        m.accepted_epoch(1, 0.0, 0.0, true);
        m.add_increment(0.5, 0.0);
        m.accepted_epoch(200_001, 0.5, 0.0, true);
        assert!(!m.output().confirmed && m.output().info == 0.0);
    }

    #[test]
    fn lam_weights_an_old_epoch_by_one_over_e_after_5_s() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.ep((1.0, 0.0), (1.0, 0.0), true);
        let first = r.m.output().info;
        assert!((first - 1.0).abs() < 1e-6);
        for _ in 0..25 {
            r.ep((0.0, 0.0), (0.0, 0.0), true);
        }
        let o = r.m.output();
        assert!((o.info - first / core::f32::consts::E).abs() < 1e-4, "{}", o.info);
    }

    /// Screened epochs (B 4 x A, then back) for `n` epochs: no pair is evaluated.
    fn screen(r: &mut Rig, n: usize) {
        for _ in 0..n / 2 {
            r.ep((0.5, 0.0), (2.0, 0.0), true);
            r.ep((0.5, 0.0), (-2.0, 0.0), true);
        }
    }

    #[test]
    fn heading_ok_truth_table() {
        // (confirmed, fault, blind) 8 states x the filter's flag = 16 rows; every state is reached by driving the monitor
        for conf in [false, true] {
            for fault in [false, true] {
                for blind in [false, true] {
                    let mut r = Rig::new();
                    r.ep((0.0, 0.0), (0.0, 0.0), true);
                    if conf {
                        r.run(10, 0.5, 0.0);
                    }
                    if fault {
                        r.run(60, 0.5, 90.0 * D2R);
                    }
                    if blind {
                        screen(&mut r, 40);
                    }
                    let o = r.m.output();
                    assert_eq!((o.confirmed, o.fault, o.blind), (conf, fault, blind));
                    for flag in [false, true] {
                        let want = flag && conf && !fault && !blind;
                        assert_eq!(r.m.heading_ok(flag), want, "flag {flag} conf {conf} fault {fault} blind {blind}");
                    }
                    assert_eq!(o.ok, conf && !fault && !blind);
                }
            }
        }
    }

    fn noise(state: &mut u32) -> f32 {
        let mut acc = 0.0;
        for _ in 0..12 {
            *state ^= *state << 13;
            *state ^= *state >> 17;
            *state ^= *state << 5;
            acc += (*state >> 8) as f32 / 16_777_216.0;
        }
        acc - 6.0
    }

    #[test]
    fn non_finite_input_never_enters_the_sums() {
        for kind in 0..3 {
            let mut r = Rig::new();
            r.ep((0.0, 0.0), (0.0, 0.0), true);
            r.run(10, 0.5, 0.0);
            assert!(r.m.output().confirmed && !r.m.output().fault);
            match kind {
                0 => {
                    r.t += DT_US;
                    r.m.add_increment(0.5, 0.0);
                    r.m.accepted_epoch(r.t, f32::NAN, 0.0, true);
                }
                1 => {
                    r.m.add_increment(f32::INFINITY, 0.0);
                    r.ep((0.5, 0.0), (0.5, 0.0), true);
                }
                _ => {
                    r.m.add_increment(f32::NAN, 0.0);
                    r.ep((0.5, 0.0), (0.5, 0.0), true);
                    // the bad value did not survive in A: the next increment is clean
                }
            }
            let o = r.m.output();
            assert_eq!(o.non_finite, 1, "kind {kind}");
            assert!(o.info.is_finite() && o.delta_rad.is_finite() && o.sigma_rad.is_finite());
            r.v = (0.0, 0.0);
            r.ep((0.5, 0.0), (0.0, 0.0), true); // first epoch of the new chain, no pair
            r.run(60, 0.5, 90.0 * D2R);
            let o = r.m.output();
            assert!(o.fault, "kind {kind}");
            assert!(o.info.is_finite() && o.delta_rad.is_finite() && o.sigma_rad.is_finite());
            assert_eq!(o.non_finite, 1);
        }
    }

    #[test]
    fn finite_inputs_that_overflow_the_sums_are_dropped() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(10, 0.5, 0.0);
        let info = r.m.output().info;
        r.ep((1e20, 0.0), (1e20, 0.0), true);
        let o = r.m.output();
        assert_eq!(o.non_finite, 1);
        assert!(o.info == info && o.info.is_finite() && o.delta_rad.is_finite());
    }

    #[test]
    fn a_gap_or_a_backwards_clock_gives_no_pair_and_no_fault() {
        for back in [false, true] {
            let mut r = Rig::new();
            r.ep((0.0, 0.0), (0.0, 0.0), true);
            r.run(4, 0.5, 90.0 * D2R);
            let o = r.m.output();
            assert!(!o.fault && o.delta_rad > 80.0 * D2R);
            let info = o.info;
            r.t = if back { r.t.wrapping_sub(1_000_000) } else { r.t + 3_000_000 };
            r.run(1, 0.5, 90.0 * D2R);
            let o = r.m.output();
            assert!(!o.fault, "back {back}");
            assert_eq!((o.info, o.out_of_interval), (info, 1));
            // the next epoch starts a new chain from the late epoch: a pair again
            r.run(1, 0.5, 90.0 * D2R);
            assert_ne!(r.m.output().info, info);
        }
        assert_eq!(MAX_DT_US, 245_000);
    }

    #[test]
    fn confirmation_needs_the_margin_for_one_second() {
        // clean 0 deg: informed with |d| + 2.58 sig <= 25 deg from Saa about 0.41; confirmed after five such epochs
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        let mut ok_epochs = 0;
        for n in 1..=12 {
            r.run(1, 0.5, 0.0);
            let o = r.m.output();
            if o.sigma_rad <= SIG_CONF_RAD && o.delta_rad.abs() + CONF_SIGMA * o.sigma_rad <= D_TOL_RAD {
                ok_epochs += 1;
                assert!(o.info >= 0.40, "info {}", o.info);
            } else {
                assert_eq!(ok_epochs, 0);
            }
            assert_eq!(o.confirmed, ok_epochs >= 5, "pair {n}");
        }
        assert!(r.m.output().confirmed);
        // one lucky epoch: four good epochs, then a strong 90 deg pair resets the count
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(4, 0.5, 0.0);
        assert!(!r.m.output().confirmed);
        r.run(1, 3.0, 90.0 * D2R);
        r.run(4, 0.5, 0.0);
        assert!(!r.m.output().confirmed);
    }

    #[test]
    fn a_35_deg_error_at_9_to_15_deg_sigma_never_confirms() {
        for (seed, sig_deg) in [(1u32, 9.0f32), (2, 10.0), (3, 12.0), (4, 14.5)] {
            let mut st = 0x9e3779b9u32 ^ seed.wrapping_mul(2654435761);
            let mut r = Rig::new();
            r.ep((0.0, 0.0), (0.0, 0.0), true);
            let mag = mag_for_sig(sig_deg * D2R);
            for _ in 0..3000 {
                let a = a_of(r.k, mag);
                r.k += 1;
                let b = rot(a, 35.0 * D2R);
                let b = (b.0 + 0.1085 * noise(&mut st) / 1.4, b.1 + 0.1085 * noise(&mut st) / 1.4);
                r.ep(a, b, true);
                assert!(!r.m.output().confirmed, "seed {seed} sig {sig_deg}: d_hat {} sig {}", r.m.output().delta_rad / D2R, r.m.output().sigma_rad / D2R);
            }
        }
    }

    #[test]
    fn blind_after_the_aid_timeout_without_an_evaluated_pair() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(10, 0.5, 0.0);
        assert!(r.m.heading_ok(true) && !r.m.output().blind);
        // R2 screening for just under the timeout: not blind; past it: blind, confirmed and not faulted, heading_ok false
        screen(&mut r, 20);
        assert!(!r.m.output().blind && r.m.heading_ok(true));
        screen(&mut r, 10);
        let o = r.m.output();
        assert!(o.blind && o.confirmed && !o.fault && !o.ok && !r.m.heading_ok(true));
        // the next evaluated pair clears it (the chain was left broken by the last screened pair: two epochs)
        r.run(3, 0.5, 0.0);
        assert!(!r.m.output().blind && r.m.heading_ok(true));
        // a broken chain with no epochs: the timer runs on chain_broken calls
        let t = r.t;
        r.m.chain_broken(t + 1_000_000);
        assert!(!r.m.output().blind);
        r.m.chain_broken(t + BLIND_US + 1_000_000);
        assert!(r.m.output().blind);
        // on the ground it is never blind
        r.ep((0.0, 0.0), (0.0, 0.0), false);
        assert!(!r.m.output().blind);
    }

    #[test]
    fn a_calm_hover_with_evaluated_low_information_pairs_is_never_blind() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        for _ in 0..1000 {
            r.run(1, 0.01, 0.0);
            assert!(!r.m.output().blind);
        }
        assert!(r.m.output().info > 0.0 && !r.m.output().confirmed);
    }

    #[test]
    fn repeat_and_catch_up_give_no_pair() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(5, 0.5, 0.0);
        let info = r.m.output().info;
        // a repeated epoch: the filter's job refuses it and the chain is broken; the catch-up epoch carries two
        // intervals of B (rotated, as a stale one is) against one of A
        r.m.chain_broken(r.t);
        r.m.add_increment(0.5, 0.0);
        r.m.add_increment(0.5, 0.0);
        r.v = (r.v.0 + 0.7, r.v.1 + 0.7);
        r.t += 2 * DT_US;
        r.m.accepted_epoch(r.t, r.v.0, r.v.1, true);
        let o = r.m.output();
        assert_eq!(o.info, info);
        assert_eq!((o.screened_r2, o.dropped_r2p), (0, 0));
        // the epoch after it pairs again, from the catch-up epoch's velocity
        r.run(1, 0.5, 0.0);
        assert!(r.m.output().info > info * 0.99);
        assert_ne!(r.m.output().info, info);
    }

    #[test]
    fn a_glitch_epoch_removes_the_pair_into_it_and_the_pair_out_of_it() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        let a = (0.5, 0.0);
        // the glitch epoch: B into it is 3 x A (screened by R2), B out of it is A rotated by 112 deg (passes R2)
        r.ep(a, (1.5, 0.0), true);
        assert_eq!(r.m.output().screened_r2, 1);
        r.ep(a, rot(a, 112.0 * D2R), true);
        let o = r.m.output();
        assert_eq!((o.screened_r2, o.dropped_r2p), (1, 1));
        assert_eq!(o.info, 0.0);
        assert!(!o.confirmed && !o.fault);
        // the pair after that is normal
        r.ep(a, a, true);
        let o = r.m.output();
        assert!((o.info - 0.25).abs() < 1e-6 && o.delta_rad.abs() < 1e-3);
    }

    #[test]
    fn ground_epochs_never_enter_and_the_sums_restart_at_take_off() {
        let mut r = Rig::new();
        for k in 0..30 {
            let a = a_of(k, 0.5);
            r.ep(a, rot(a, 90.0 * D2R), false);
        }
        let o = r.m.output();
        assert!(o.info == 0.0 && !o.confirmed && !o.fault);
        // take-off: the first airborne epoch has a grounded predecessor (no pair), the second pairs
        r.ep(a_of(0, 0.5), a_of(0, 0.5), true);
        assert_eq!(r.m.output().info, 0.0);
        r.ep(a_of(1, 0.5), a_of(1, 0.5), true);
        let o = r.m.output();
        assert!((o.info - 0.25).abs() < 1e-6 && o.delta_rad.abs() < 1e-3);
        r.run(20, 0.5, 0.0);
        assert!(r.m.output().confirmed);
        // landing and a second take-off: sums restart again
        r.ep(a_of(0, 0.5), a_of(0, 0.5), false);
        assert_eq!(r.m.output().info, 0.0);
        r.ep(a_of(0, 0.5), a_of(0, 0.5), true);
        assert_eq!(r.m.output().info, 0.0);
        // not armed: nothing enters even with the airborne flag
        let mut m = HeadingMonitor::default();
        for k in 0..10u32 {
            m.add_increment(0.5, 0.0);
            m.accepted_epoch(k * DT_US, 0.5 * k as f32, 0.0, true);
        }
        assert_eq!(m.output().info, 0.0);
    }

    #[test]
    fn r2_passes_any_pure_rotation_of_a() {
        for mag in [0.2f32, 0.5, 3.0, 12.0] {
            let mut d = -180;
            while d <= 180 {
                let mut r = Rig::new();
                r.ep((0.0, 0.0), (0.0, 0.0), true);
                r.run(12, mag, d as f32 * D2R);
                assert_eq!(r.m.output().screened_r2, 0, "mag {mag} d {d}");
                assert!(r.m.output().info > 0.0);
                d += 5;
            }
        }
    }

    #[test]
    fn r2_screens_a_speed_change() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.ep((0.5, 0.0), (0.5 + 0.55, 0.0), true);
        assert_eq!(r.m.output().screened_r2, 1);
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.ep((0.5, 0.0), (0.5 + 0.5, 0.0), true);
        assert_eq!(r.m.output().screened_r2, 0);
    }

    /// Steady Saa = |A|^2 / (1 - lam): the |A| that gives `sig` at B rotated by `d`.
    fn mag_for_sig(sig: f32) -> f32 {
        let lam = libm::expf(-0.2 / TAU_S);
        SIG_B_DEFAULT / sig * libm::sqrtf(1.0 - lam)
    }

    #[test]
    fn h9_fault_characteristic_sig_d_over_7_58_faults_d_over_4_does_not() {
        for dd in [30.0f32, 45.0, 60.0, 90.0] {
            let d = dd * D2R;
            let mut r = Rig::new();
            r.ep((0.0, 0.0), (0.0, 0.0), true);
            r.run(150, mag_for_sig(d / 7.58), d);
            let o = r.m.output();
            assert!((o.sigma_rad - d / 7.58).abs() < 0.01 * d / 7.58, "{} {}", o.sigma_rad, d / 7.58);
            assert!(o.fault, "d {dd}: sig {} delta {}", o.sigma_rad, o.delta_rad);
            let mut r = Rig::new();
            r.ep((0.0, 0.0), (0.0, 0.0), true);
            r.run(150, mag_for_sig(d / 4.0), d);
            let o = r.m.output();
            assert!((o.sigma_rad - d / 4.0).abs() < 0.01 * d / 4.0);
            assert!(!o.fault, "d {dd}: sig {} delta {}", o.sigma_rad, o.delta_rad);
        }
    }

    #[test]
    fn an_uninformed_epoch_never_faults() {
        // |A| 0.05: sig settles near 25 deg, d_hat 170 deg is beyond d_tol and beyond 5 sig, but nothing is informed
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(300, 0.05, 170.0 * D2R);
        let o = r.m.output();
        assert!(o.sigma_rad > SIG_CONF_RAD && o.delta_rad.abs() > 5.0 * o.sigma_rad);
        assert!(!o.fault && !o.confirmed);
    }

    #[test]
    fn r3_needs_the_5_sig_term_at_15_deg_noise() {
        // sig 15 deg, d_hat 30 deg: beyond d_tol but only 2 sigma
        let d = 30.0 * D2R;
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        r.run(150, mag_for_sig(14.9 * D2R), d);
        let o = r.m.output();
        assert!(o.sigma_rad <= SIG_CONF_RAD && o.delta_rad.abs() > D_TOL_RAD);
        assert!(!o.fault && !o.confirmed);
    }

    #[test]
    fn fault_needs_five_flagged_epochs_and_a_screened_pair_does_not_reset() {
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        // build information at 90 deg first without letting the count start: not possible, so count from the first flag
        let mut flagged = 0;
        for _ in 0..40 {
            let before = r.m.output();
            r.run(1, 0.5, 90.0 * D2R);
            let o = r.m.output();
            let is_flag = o.sigma_rad <= SIG_CONF_RAD && o.delta_rad.abs() > D_TOL_RAD && o.delta_rad.abs() > 5.0 * o.sigma_rad;
            if is_flag {
                flagged += 1;
            } else {
                flagged = 0;
            }
            assert_eq!(o.fault, flagged >= 5 || before.fault, "flagged {flagged}");
            if o.fault {
                break;
            }
        }
        assert!(r.m.output().fault);
        // a screened pair in the middle of a count neither counts nor resets it
        let mut r = Rig::new();
        r.ep((0.0, 0.0), (0.0, 0.0), true);
        // |A| 0.5 at 90 deg is informed and beyond 5 sig from the first pair: one flagged epoch
        r.run(1, 0.5, 90.0 * D2R);
        assert!(r.m.output().delta_rad.abs() > 5.0 * r.m.output().sigma_rad);
        // two more (three counted), then a glitch pair (and the pair after it, R2'), then two more
        r.run(2, 0.5, 90.0 * D2R);
        assert!(!r.m.output().fault);
        r.ep((0.5, 0.0), (2.0, 0.0), true);
        r.ep((0.5, 0.0), (0.0, 0.5), true);
        assert!(!r.m.output().fault);
        r.run(1, 0.5, 90.0 * D2R);
        assert!(!r.m.output().fault);
        r.run(1, 0.5, 90.0 * D2R);
        assert!(r.m.output().fault);
    }

    fn field(s: &str) -> Vec<&str> {
        s.split(',').collect()
    }

    /// Replays a fixture; returns (max |d_hat diff|, max sig diff, evaluated count, first-session time to confirmed).
    fn replay(csv: &str) -> (f32, f32, usize, Vec<f32>, bool, bool) {
        let mut m = HeadingMonitor::default();
        let (mut max_d, mut max_s, mut n_ev, mut faulted) = (0.0f32, 0.0f32, 0usize, false);
        let mut sess = u32::MAX;
        let mut air_start: Option<f64> = None; // take-off time of the session, s (the fixture's #air line)
        let mut tconf: Vec<f32> = Vec::new();
        let mut conf_seen = false;
        let mut blind_seen = false;
        let mut air0: Vec<f64> = Vec::new();
        for line in csv.lines().filter(|l| l.starts_with("#air,")) {
            air0.push(field(line)[2].parse().unwrap());
        }
        for line in csv.lines().filter(|l| !l.starts_with('#') && !l.starts_with("sess")) {
            let f = field(line);
            let s: u32 = f[0].parse().unwrap();
            let t: u64 = f[1].parse().unwrap();
            let (an, ae, vn, ve): (f32, f32, f32, f32) = (f[2].parse().unwrap(), f[3].parse().unwrap(), f[4].parse().unwrap(), f[5].parse().unwrap());
            let (air, brk): (bool, bool) = (f[6] == "1", f[7] == "1");
            let (ev, dh, sg): (bool, f32, f32) = (f[8] == "1", f[9].parse().unwrap(), f[10].parse().unwrap());
            if s != sess {
                if sess != u32::MAX && !conf_seen {
                    tconf.push(f32::INFINITY);
                }
                m.set_armed(false);
                m.set_armed(true);
                sess = s;
                air_start = air0.get(s as usize).copied();
                conf_seen = false;
            }
            if brk {
                m.chain_broken(t as u32);
            }
            m.add_increment(an, ae);
            let scr0 = m.output().screened_r2;
            m.accepted_epoch(t as u32, vn, ve, air);
            let o = m.output();
            assert_eq!(o.screened_r2 - scr0, u32::from(f[11] == "1"), "R2 flag at t {t}");
            blind_seen |= o.blind;
            if ev {
                n_ev += 1;
                max_d = max_d.max(wrap(o.delta_rad - dh).abs());
                max_s = max_s.max((o.sigma_rad - sg).abs());
            }
            if o.confirmed && !conf_seen {
                conf_seen = true;
                tconf.push((t as f64 * 1e-6 - air_start.unwrap()) as f32);
            }
            faulted |= o.fault;
        }
        if !conf_seen {
            tconf.push(f32::INFINITY);
        }
        (max_d, max_s, n_ev, tconf, faulted, blind_seen)
    }

    #[test]
    fn recorded_log_311_reproduces_the_python_monitor() {
        let (max_d, max_s, n_ev, tconf, faulted, blind) = replay(include_str!("../testdata/headmon_311.csv"));
        std::println!("311: evaluated {n_ev} max|d_hat diff| {max_d} rad max|sig diff| {max_s} rad tconf {tconf:?} blind seen {blind}");
        assert!(n_ev > 1000);
        assert!(max_d < 2e-3 && max_s < 1e-4, "{max_d} {max_s}");
        assert!(!faulted);
        // H10 (1) rule, Python (hm3b.py) per session: 70.664, 13.944, 9.064 s after take-off (round 3, old rule: 4.262, 3.363,
        // 1.296 s; t7_round3.out reports 4.3 for the first)
        assert_eq!(tconf.len(), 3);
        for (a, b) in tconf.iter().zip([70.6634, 13.9444, 9.0640]) {
            assert!((a - b).abs() < 0.02, "{tconf:?}");
        }
    }

    #[test]
    fn recorded_log_291_reproduces_the_python_monitor() {
        let (max_d, max_s, n_ev, tconf, faulted, blind) = replay(include_str!("../testdata/headmon_291.csv"));
        std::println!("291: evaluated {n_ev} max|d_hat diff| {max_d} rad max|sig diff| {max_s} rad tconf {tconf:?} blind seen {blind}");
        assert!(n_ev > 500);
        assert!(max_d < 2e-3 && max_s < 1e-4, "{max_d} {max_s}");
        assert!(!faulted);
        // H10 (1) rule, Python (hm3b.py) per session: 12.970, 2.623 s after take-off (round 3, old rule: 5.193, 1.621 s;
        // t7_round3.out reports 5.2 for the first)
        assert_eq!(tconf.len(), 2);
        for (a, b) in tconf.iter().zip([12.9699, 2.6231]) {
            assert!((a - b).abs() < 0.02, "{tconf:?}");
        }
    }
}
