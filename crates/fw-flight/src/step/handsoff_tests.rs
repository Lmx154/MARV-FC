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

// ---- The real step in the hands-off states ----

/// Full sticks, every axis.
const FULL: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

/// A flight with FS_HOLD_EN `fs`, the heading monitor's verdict `heading_ok` and coverage `valid`, 10 m up: armed in
/// High with the throttle low, then 0.5 s at half throttle so the airborne latch is set.
struct Fly {
    f: TestFlight<'static>,
    cell: &'static Cell<NavState>,
    t: u64,
}

impl Fly {
    fn new(fs: bool, heading_ok: bool, valid: Valid) -> Fly {
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(-10.0, valid))));
        let mut f = flight(cell, handshake(), &Tuning::DEFAULT);
        f.set_fs_hold_en(fs);
        f.set_heading_ok(heading_ok);
        let mut fly = Fly { f, cell, t: 1_000_000 };
        fly.rc(false, [0.0; 4], Switch3::High);
        let o = fly.rc(true, [0.0; 4], Switch3::High);
        assert!(o.armed, "armed");
        for _ in 0..400 {
            fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::High);
        }
        assert!(fly.f.airborne && fly.f.gate.armed(), "airborne");
        assert_eq!(fly.f.hands_off_state(), State::Pilot);
        fly
    }

    /// One step with a report: arm switch, sticks [roll, pitch, throttle, yaw], mode switch.
    fn rc(&mut self, arm: bool, [roll, pitch, throttle, yaw]: [f32; 4], switch6: Switch3) -> StepOut {
        self.t += DT_US;
        let t = self.t;
        self.f.step(&input(
            t,
            Some(RcSample {
                sticks: Sticks { roll, pitch, throttle, yaw },
                arm,
                switch6,
                t_us: t,
            }),
        ))
    }

    /// One step without a report.
    fn quiet(&mut self) -> StepOut {
        self.t += DT_US;
        self.f.step(&input(self.t, None))
    }

    /// Steps without a report until the state leaves PILOT (just past the stick-loss time); that step's output.
    fn lose(&mut self) -> StepOut {
        for _ in 0..2_000 {
            let o = self.quiet();
            if self.f.hands_off_state() != State::Pilot {
                return o;
            }
        }
        panic!("the sticks were never lost");
    }

    fn state(&self) -> State {
        self.f.hands_off_state()
    }

    fn set_valid(&self, valid: Valid) {
        let mut n = self.cell.get();
        n.valid = valid;
        self.cell.set(n);
    }
}

/// The centred-stick setpoint of the hands-off laws: no horizontal or vertical feed-forward and no heading rate (Position),
/// or a level attitude and no heading rate (Altitude).
fn is_centred(o: &StepOut) -> bool {
    match o.setpoint {
        Setpoint::Position { velocity_ms, heading_rate_rads, .. } => {
            velocity_ms[0] == 0.0 && velocity_ms[1] == 0.0 && velocity_ms[2] == 0.0 && heading_rate_rads == 0.0
        }
        Setpoint::Altitude { attitude, heading_rate_rads, velocity_d_ms, .. } => {
            attitude == [1.0, 0.0, 0.0, 0.0] && heading_rate_rads == 0.0 && velocity_d_ms == 0.0
        }
        _ => false,
    }
}

#[test]
fn hold_stick_loss_in_the_air_on_t3_prime_is_hold_with_the_position_law_and_the_gate_armed() {
    let mut fly = Fly::new(true, true, POSITION);
    // The last report had full sticks: none of them reaches the shaper after the loss.
    fly.rc(true, FULL, Switch3::High);
    let o = fly.lose();
    assert_eq!(fly.state(), State::Hold);
    assert!(o.armed && o.motors.throttles().is_some(), "RC loss does not disarm while hands-off");
    assert_eq!(o.mode, Mode::Position);
    assert!(is_centred(&o), "{:?}", o.setpoint);
    for _ in 0..4_000 {
        let o = fly.quiet();
        assert_eq!((fly.state(), o.mode, o.armed), (State::Hold, Mode::Position, true));
        assert!(is_centred(&o) && o.demand.is_ok());
    }
}

#[test]
fn hold_without_the_heading_monitor_stick_loss_in_the_air_is_land_never_hold() {
    let mut fly = Fly::new(true, false, POSITION);
    let o = fly.lose();
    assert_eq!(fly.state(), State::Land);
    assert_eq!(o.mode, Mode::Altitude, "no T3': LAND descends level");
    assert!(o.armed);
}

#[test]
fn i3_full_sticks_and_any_switch_in_takeover_give_the_centred_setpoint_until_the_lock_hands_over() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    assert_eq!(fly.state(), State::Hold);
    // CRSF back with full sticks: Low and Middle before High, then High with arm ON (the lock), all centred.
    for sw in [Switch3::Low, Switch3::Middle, Switch3::Low, Switch3::High, Switch3::High] {
        let o = fly.rc(true, FULL, sw);
        assert_eq!(fly.state(), State::Takeover, "{sw:?}");
        assert!(is_centred(&o) && o.armed, "{sw:?}: {:?}", o.setpoint);
        assert_eq!(o.selected, match sw {
            Switch3::Low => Mode::Angle,
            Switch3::Middle => Mode::Altitude,
            Switch3::High => Mode::Position,
        }, "the log keeps the real switch");
    }
    // The first report after the lock with the switch out of High: PILOT, the real sticks.
    let o = fly.rc(true, FULL, Switch3::Middle);
    assert_eq!(fly.state(), State::Pilot);
    assert_eq!(o.mode, Mode::Altitude);
    assert!(!is_centred(&o), "the real sticks: {:?}", o.setpoint);
}

#[test]
fn i3_control_high_with_arm_off_does_not_open_the_lock() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    fly.rc(false, FULL, Switch3::High);
    let o = fly.rc(true, FULL, Switch3::Middle);
    assert_eq!(fly.state(), State::Takeover, "K, no P");
    assert!(is_centred(&o) && o.armed);
}

#[test]
fn c1_arm_off_through_the_step_holds_before_k_and_disarms_after_k_and_in_pilot() {
    // CRSF back with arm OFF: TAKEOVER, the gate armed, the motors run.
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    for _ in 0..200 {
        let o = fly.rc(false, [0.0, 0.0, 0.5, 0.0], Switch3::High);
        assert_eq!(fly.state(), State::Takeover);
        assert!(o.armed && o.motors.throttles().is_some());
    }
    // Arm ON (K), then OFF: disarmed at that report.
    let o = fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::Low);
    assert!(o.armed);
    let o = fly.rc(false, [0.0, 0.0, 0.5, 0.0], Switch3::Low);
    assert!(!o.armed && o.motors.throttles().is_none(), "after K");
    assert_eq!(fly.state(), State::Stop);
    // PILOT: arm OFF disarms at once, FS_HOLD_EN 1 or 0.
    for fs in [true, false] {
        let mut fly = Fly::new(fs, true, POSITION);
        let o = fly.rc(false, [0.0, 0.0, 0.5, 0.0], Switch3::High);
        assert!(!o.armed, "fs {fs}");
    }
}

#[test]
fn wr2_the_held_arm_off_report_still_newest_when_the_latch_clears_disarms_in_that_step() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    let o = fly.rc(false, [0.0, 0.0, 0.5, 0.0], Switch3::High);
    assert!(o.armed && fly.state() == State::Takeover);
    // The latch as the previous step left it (modelled here: the real one clears in `wr1_touchdown_...`).
    fly.f.airborne = false;
    let o = fly.quiet();
    assert_eq!(fly.state(), State::Stop);
    assert!(!o.armed && o.motors.throttles().is_none(), "disarmed in the step the latch reads false");
}

#[test]
fn b9_fs_hold_en_written_while_armed_takes_effect_after_the_next_disarm() {
    let mut fly = Fly::new(false, true, POSITION);
    fly.f.set_fs_hold_en(true);
    let o = fly.lose();
    assert_eq!(fly.state(), State::Stop);
    assert!(!o.armed, "0 in force: RC loss disarms as before");
    // Re-armed after a disarmed step: the 1 is in force.
    for _ in 0..10 {
        fly.rc(false, [0.0; 4], Switch3::High);
    }
    let o = fly.rc(true, [0.0; 4], Switch3::High);
    assert!(o.armed);
    for _ in 0..400 {
        fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::High);
    }
    let o = fly.lose();
    assert_eq!(fly.state(), State::Hold);
    assert!(o.armed);
    // Writing 0 while in HOLD keeps HOLD with RC loss not disarming.
    fly.f.set_fs_hold_en(false);
    for _ in 0..2_000 {
        let o = fly.quiet();
        assert!(o.armed && fly.state() == State::Hold);
    }
}

#[test]
fn b5_a_0_99_s_coverage_drop_in_hold_keeps_hold_a_1_0_s_drop_lands() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    let steps_099 = (990_000 / DT_US) as usize;
    let steps_100 = (1_000_000 / DT_US) as usize;
    fly.set_valid(ALTITUDE);
    for _ in 0..steps_099 {
        let o = fly.quiet();
        assert_eq!((fly.state(), o.mode), (State::Hold, Mode::Altitude), "the law follows the flags at once");
    }
    fly.set_valid(POSITION);
    let o = fly.quiet();
    assert_eq!((fly.state(), o.mode), (State::Hold, Mode::Position));
    fly.set_valid(ALTITUDE);
    for k in 0..steps_100 {
        fly.quiet();
        assert_eq!(fly.state(), State::Hold, "step {k}");
    }
    let o = fly.quiet();
    assert_eq!(fly.state(), State::Land, "1.0 s");
    assert!(o.armed && o.mode == Mode::Altitude);
}
