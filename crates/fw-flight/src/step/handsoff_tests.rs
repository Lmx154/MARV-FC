//! Host tests of the hands-off wiring in `Flight::step` (decision-mavlink-mode-20261123, Amendment D: WR1 to WR3, the
//! laws built in H2b-2 and its checklist). The estimator is the fake of `althold_tests`: each test sets what the
//! estimate covers, where the vehicle is and how it moves.

extern crate std;

use super::althold_tests::{ALTITUDE, ANGLE, DT_US, TestFlight, flight, input, nav_at};
use super::*;
use core::cell::Cell;

/// Everything Position needs.
const POSITION: Valid = Valid {
    heading: true,
    velocity_ne: true,
    position_ne: true,
    ..ALTITUDE
};

/// A handshake of its own per run (the gate's `try_arm` fails while a flag a previous run set is still up).
fn handshake() -> &'static Handshake {
    let h: &'static Handshake = std::boxed::Box::leak(std::boxed::Box::new(Handshake::new()));
    h.set_permit(true);
    h
}

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0 >> 33
    }

    fn chance(&mut self, per_1000: u64) -> bool {
        self.next() % 1000 < per_1000
    }

    fn unit(&mut self) -> f32 {
        (self.next() % 10_001) as f32 / 10_000.0
    }
}

/// FNV-1a, 64 bit.
fn fnv(h: &mut u64, bytes: &[u8]) {
    for &b in bytes {
        *h ^= u64::from(b);
        *h = h.wrapping_mul(0x0100_0000_01b3);
    }
}

/// One step's output as the log and the board see it: `StepOut`'s `Debug` text and the step block, SPN and MODE bytes.
fn digest_step(h: &mut u64, o: &StepOut) {
    fnv(h, std::format!("{o:?}").as_bytes());
    let mut b = [0u8; 256];
    let n = encode_step(o, &mut b);
    fnv(h, &b[..n]);
    let n = encode_spn(o, &mut b);
    fnv(h, &b[..n]);
    let n = encode_mode(o, &mut b);
    fnv(h, &b[..n]);
}

/// A scripted flight over random reports, sticks, switches, coverage, positions and silences (RC loss on the ground and
/// in the air), with arming cycles; `setup` runs on the flight first. Returns the digest of every step and counts of
/// what it went through (armed steps, RC-loss disarms in the air, steps per effective mode).
fn walk(seed: u64, steps: usize, setup: impl FnOnce(&mut TestFlight)) -> (u64, [u32; 5]) {
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(0.0, ANGLE));
    let mut f = flight(&cell, handshake(), &tuning);
    setup(&mut f);
    let mut rng = Lcg(seed);
    let mut t = 1_000_000u64;
    let (mut quiet, mut arm, mut switch6) = (0u32, false, Switch3::Low);
    let mut valid = ANGLE;
    let (mut pos, mut vel) = ([0.0f32; 3], [0.0f32; 3]);
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut counts = [0u32; 5];
    let mut was_armed = false;
    for _ in 0..steps {
        t += DT_US;
        if quiet == 0 && rng.chance(2) {
            quiet = 900 + (rng.next() % 600) as u32;
        }
        quiet = quiet.saturating_sub(1);
        if rng.chance(4) {
            arm = !arm;
        }
        if rng.chance(5) {
            switch6 = [Switch3::Low, Switch3::Middle, Switch3::High][(rng.next() % 3) as usize];
        }
        if rng.chance(3) {
            valid = [ANGLE, ALTITUDE, POSITION, POSITION, POSITION][(rng.next() % 5) as usize];
        }
        for k in 0..3 {
            vel[k] = (vel[k] + 0.02 * (rng.unit() - 0.5)).clamp(-3.0, 3.0);
            pos[k] += vel[k] * DT_US as f32 * 1e-6;
        }
        let mut n = nav_at(pos[2], valid);
        n.position_m = pos;
        n.velocity_ms = vel;
        n.rates_rads = [0.2 * (rng.unit() - 0.5), 0.2 * (rng.unit() - 0.5), 0.1 * (rng.unit() - 0.5)];
        cell.set(n);
        let rc = (quiet == 0 && !rng.chance(100)).then(|| RcSample {
            sticks: Sticks {
                roll: rng.unit() - 0.5,
                pitch: rng.unit() - 0.5,
                throttle: if f.gate.armed() { 0.3 + 0.6 * rng.unit() } else { 0.0 },
                yaw: rng.unit() - 0.5,
            },
            arm,
            switch6,
            t_us: t,
        });
        let air = f.airborne;
        let o = f.step(&input(t, rc));
        digest_step(&mut h, &o);
        counts[0] += u32::from(o.armed);
        counts[1] += u32::from(was_armed && !o.armed && air);
        counts[2 + usize::from(mode_code(o.mode).saturating_sub(1)).min(2)] += 1;
        was_armed = o.armed;
    }
    (h, counts)
}

/// The digests of `walk` (40 000 steps) on the flight step before the hands-off wiring (commit c270814), by seed.
const WALK_GOLDEN: [(u64, u64); 3] = [
    (0x5eed_0b01, 0xc20b_89fd_6881_ff30),
    (0x5eed_0b02, 0x80ac_3c4a_1ca4_993f),
    (0x5eed_0b03, 0x73bf_dc63_d210_a3f1),
];

/// I2: with FS_HOLD_EN 0 every step's `StepOut` and its step block, SPN and MODE bytes equal those of the step before
/// the wiring, over three scripted flights with arming cycles, mode changes, coverage drops and RC loss in the air.
#[test]
fn i2_fs_hold_en_0_is_bit_identical_to_the_step_before_the_wiring() {
    let got = WALK_GOLDEN.map(|(seed, _)| {
        let (h, c) = walk(seed, 40_000, |f| f.set_fs_hold_en(false));
        std::println!("walk {seed:#x}: digest {h:#018x}, counts {c:?}");
        assert!(c[0] > 5_000 && c[1] > 0 && c[2] > 100 && c[3] > 100 && c[4] > 100, "not vacuous: {c:?}");
        (seed, h)
    });
    assert_eq!(got, WALK_GOLDEN);
}

/// I2's control: with FS_HOLD_EN 1 the same flights differ from the goldens (the RC losses in the air go to the
/// hands-off states), with and without the heading monitor's verdict.
#[test]
fn i2_control_fs_hold_en_1_changes_the_flights() {
    for (seed, golden) in WALK_GOLDEN {
        let (h, _) = walk(seed, 40_000, |f| f.set_fs_hold_en(true));
        assert_ne!(h, golden, "seed {seed:#x}, heading not OK");
        let (h, _) = walk(seed, 40_000, |f| {
            f.set_fs_hold_en(true);
            f.set_heading_ok(true);
        });
        assert_ne!(h, golden, "seed {seed:#x}, heading OK");
    }
}

/// I2: with FS_HOLD_EN 0, RC loss in the air (airborne latch set, Position) disarms at exactly 1.0 s, as before.
#[test]
fn i2_fs_hold_en_0_rc_loss_in_the_air_disarms_at_exactly_1_0_s() {
    let tuning = Tuning::DEFAULT;
    let cell = Cell::new(nav_at(-10.0, POSITION));
    let mut f = flight(&cell, handshake(), &tuning);
    f.set_heading_ok(true);
    let mut t = 1_000_000;
    let mut last = t;
    for k in 0..400 {
        t += DT_US;
        let arm = k >= 1;
        let throttle = if k < 2 { 0.0 } else { 0.5 };
        f.step(&input(t, Some(super::althold_tests::report(t, arm, throttle, Switch3::High))));
        last = t;
    }
    assert!(f.gate.armed() && f.airborne);
    let timeout = tuning.rc_fs_timeout_us;
    while t + DT_US <= last + timeout {
        t += DT_US;
        let o = f.step(&input(t, None));
        assert!(o.armed && o.motors.throttles().is_some(), "{}", t - last);
    }
    let o = f.step(&input(last + timeout, None));
    assert!(o.armed, "exactly 1.0 s");
    let o = f.step(&input(last + timeout + 1, None));
    assert!(!o.armed && o.motors.throttles().is_none(), "1.0 s + 1 us");
    assert_eq!(f.hands_off_state(), State::Stop);
}
