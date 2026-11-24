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
/// in the air), with arming cycles; `setup` runs on the flight first. Returns the digest of every step, the digest of the
/// steps before the first airborne stick loss (the first step that starts armed with the airborne latch set and the newest
/// report older than the stick-loss time; `None` if there is none), and counts of what it went through (armed steps,
/// RC-loss disarms in the air, steps per effective mode).
fn walk(seed: u64, steps: usize, setup: impl FnOnce(&mut TestFlight)) -> (u64, Option<u64>, [u32; 5]) {
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
    let (mut newest, mut prefix) = (None::<u64>, None::<u64>);
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
        if rc.is_some() {
            newest = Some(t);
        }
        let lost = newest.is_none_or(|n| t - n > Tuning::DEFAULT.rc_fs_timeout_us);
        if prefix.is_none() && was_armed && air && lost {
            prefix = Some(h);
        }
        let o = f.step(&input(t, rc));
        digest_step(&mut h, &o);
        counts[0] += u32::from(o.armed);
        counts[1] += u32::from(was_armed && !o.armed && air);
        counts[2 + usize::from(mode_code(o.mode).saturating_sub(1)).min(2)] += 1;
        was_armed = o.armed;
    }
    (h, prefix, counts)
}

/// The digests of `walk` (40 000 steps) on the flight step before the hands-off wiring (commit c270814; HEAD dc08e59 gave
/// the same), by seed: the whole walk, and the steps before its first airborne stick loss.
const WALK_C270814: [(u64, u64, u64); 3] = [
    (0x5eed_0b01, 0xc20b_89fd_6881_ff30, 0xe926_b5c2_a522_7ce0),
    (0x5eed_0b02, 0x80ac_3c4a_1ca4_993f, 0x14c8_305a_51f6_5188),
    (0x5eed_0b03, 0x73bf_dc63_d210_a3f1, 0x338c_b533_10c0_bd24),
];

/// The digests of the whole `walk` with Amendment N, N2 (a stick loss in the air at FS_HOLD_EN 0 lands, BLIND on T1), by
/// seed. Every walk has an airborne stick loss, so every digest changed; the prefix before it did not.
const WALK_GOLDEN: [(u64, u64); 3] = [
    (0x5eed_0b01, 0x7629_5a56_9aae_7065),
    (0x5eed_0b02, 0x46a3_bae4_d352_1f51),
    (0x5eed_0b03, 0xef4e_8cca_25eb_2f21),
];

/// I2 as amended by N2: with FS_HOLD_EN 0 every step's `StepOut` and its step block, SPN and MODE bytes before the first
/// airborne stick loss equal those of the step before the wiring (the prefix digest), over three scripted flights with
/// arming cycles, mode changes, coverage drops and RC loss on the ground and in the air; the whole walk equals the
/// re-baselined golden and differs from the old one (the change is real and starts at the loss or later).
#[test]
fn i2_fs_hold_en_0_is_bit_identical_to_the_step_before_the_wiring() {
    let (mut got, mut want) = (std::vec::Vec::new(), std::vec::Vec::new());
    for ((seed, old, old_prefix), (seed_new, golden)) in WALK_C270814.into_iter().zip(WALK_GOLDEN) {
        assert_eq!(seed, seed_new);
        let (h, prefix, c) = walk(seed, 40_000, |f| f.set_fs_hold_en(false));
        std::println!("walk {seed:#x}: digest {h:#018x} (c270814 {old:#018x}), prefix {prefix:x?}, counts {c:?}");
        assert!(c[0] > 5_000 && c[2] > 100 && c[3] > 100 && c[4] > 100, "not vacuous: {c:?}");
        assert_eq!(prefix, Some(old_prefix), "seed {seed:#x}: the steps before the first airborne stick loss are unchanged");
        assert_ne!(h, old, "seed {seed:#x}: the walk has an airborne stick loss, so it changed");
        got.push((seed, h));
        want.push((seed, golden));
    }
    assert_eq!(got, want);
}

/// I2's control as amended by N2: with FS_HOLD_EN 1 and no heading monitor (so no T3' and no HOLD) the flights equal the
/// FS_HOLD_EN 0 goldens, since FS_HOLD_EN now gates only HOLD and missions; with the heading monitor's verdict (HOLD on
/// T3') they differ.
#[test]
fn i2_control_fs_hold_en_1_changes_the_flights() {
    for (seed, golden) in WALK_GOLDEN {
        let (h, _, _) = walk(seed, 40_000, |f| f.set_fs_hold_en(true));
        assert_eq!(h, golden, "seed {seed:#x}, heading not OK");
        let (h, _, _) = walk(seed, 40_000, |f| {
            f.set_fs_hold_en(true);
            f.set_heading_ok(true);
        });
        assert_ne!(h, golden, "seed {seed:#x}, heading OK");
    }
}

/// N2: with FS_HOLD_EN 0, RC loss in the air (airborne latch set, Position, T3') is LAND at the first step past 1.0 s,
/// the gate armed and the motors running, never HOLD and never STOP.
#[test]
fn i2_fs_hold_en_0_rc_loss_in_the_air_lands_at_1_0_s_and_stays_armed() {
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
    assert!(o.armed && f.hands_off_state() == State::Pilot, "exactly 1.0 s");
    let mut t = last + timeout + 1;
    let o = f.step(&input(t, None));
    assert!(o.armed && o.motors.throttles().is_some(), "1.0 s + 1 us");
    assert_eq!(f.hands_off_state(), State::Land);
    for k in 0..400 {
        t += DT_US;
        let o = f.step(&input(t, None));
        assert!(o.armed && o.motors.throttles().is_some() && f.hands_off_state() == State::Land, "step {k}");
    }
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
        Fly::with_speed(fs, heading_ok, valid, Tuning::DEFAULT.pilot_speed_xy_ms)
    }

    /// As `new`, with `PILOT_SPEED_XY` `speed` (m/s) written before the arming.
    fn with_speed(fs: bool, heading_ok: bool, valid: Valid, speed: f32) -> Fly {
        let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(-10.0, valid))));
        let mut f = flight(cell, handshake(), &Tuning::DEFAULT);
        assert!(f.set_pilot_speed_xy(speed));
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

/// LAND's braking law (H14 (1)): Position at this step's navigation position, no horizontal feed-forward, the LAND
/// descent rate, no heading rate.
fn is_braking(o: &StepOut) -> bool {
    match o.setpoint {
        Setpoint::Position { position_m, velocity_ms, heading_rate_rads, .. } => {
            position_m[0] == o.nav.position_m[0]
                && position_m[1] == o.nav.position_m[1]
                && velocity_ms == [0.0, 0.0, LAND_SPEED_MS]
                && heading_rate_rads == 0.0
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
    assert_eq!(fly.state(), State::Land);
    assert!(o.armed, "0 in force: RC loss in the air on T3' lands (Amendment N, N2), no HOLD");
    // The sticks back with arm ON (K), then arm OFF: disarmed.
    fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::High);
    assert!(!fly.rc(false, [0.0; 4], Switch3::High).armed);
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

// ---- LAND, the touchdown disarm, BLIND, the TAKEOVER descent ----

/// The setpoint's target height, m down.
fn target_d(o: &StepOut) -> f32 {
    match o.setpoint {
        Setpoint::Altitude { down_m, .. } => down_m,
        Setpoint::Position { position_m, .. } => position_m[2],
        _ => f32::NAN,
    }
}

/// WR1 touchdown: LAND descends (the height target at 0.5 m/s), the vehicle follows it to the ground (d = 0) and IMU1
/// reads it at rest (WR1's guard, N4 (b)), the real airborne latch clears (TD or land_detect_us on the idle floor,
/// whichever is first: TD, which starts at the ground threshold, above the idle floor), and the gate is disarmed within
/// one step of that: at most land_detect_us plus one step after the first step below the ground threshold.
#[test]
fn wr1_touchdown_land_descends_the_latch_clears_and_the_gate_disarms_within_one_step() {
    let mut fly = Fly::new(true, false, POSITION);
    let o = fly.lose();
    assert_eq!(fly.state(), State::Land);
    let d0 = target_d(&o);
    let mut first_low: Option<u64> = None;
    let mut cleared: Option<u64> = None;
    let mut disarmed: Option<u64> = None;
    let mut rate_checked = false;
    let dt = DT_US as f32 * 1e-6;
    for k in 0..40_000u32 {
        // The vehicle follows the target down at the descent rate, and stops on the ground.
        let mut n = fly.cell.get();
        let d = (n.position_m[2] + LAND_SPEED_MS * dt).min(0.0);
        n.velocity_ms[2] = if d < 0.0 { LAND_SPEED_MS } else { 0.0 };
        n.position_m[2] = d;
        fly.cell.set(n);
        let air = fly.f.airborne;
        let o = fly.quiet_f(AT_REST);
        if k == 800 {
            let rate = (target_d(&o) - d0) / (800.0 * dt);
            assert!((rate - LAND_SPEED_MS).abs() < 0.01, "descent {rate} m/s");
            rate_checked = true;
        }
        let low = !o.demand.is_ok_and(|d| d.thrust_n >= ground_n());
        if o.armed && low {
            first_low.get_or_insert(fly.t);
        } else if o.armed {
            first_low = None;
        }
        if air && !fly.f.airborne && cleared.is_none() {
            cleared = Some(fly.t);
        }
        if !o.armed {
            disarmed = Some(fly.t);
            assert_eq!(fly.state(), State::Stop);
            break;
        }
        assert_eq!(fly.state(), State::Land, "step {k}");
    }
    assert!(rate_checked);
    let (low, clear, dis) = (first_low.unwrap(), cleared.expect("the latch cleared"), disarmed.expect("disarmed"));
    let land = Tuning::DEFAULT.land_detect_us;
    std::println!("below the ground threshold at {low}, latch clear {} us later, disarm {} us after the clear", clear - low, dis - clear);
    assert!(dis - clear <= DT_US, "disarmed within one step of the clear");
    assert!(dis - low <= land + DT_US, "land_detect_us plus one step: {} us", dis - low);
}

#[test]
fn blind_is_level_with_the_collective_0_3_m_s2_below_hover() {
    // T1 for longer than the debounce before the sticks are lost.
    let mut fly = Fly::new(true, true, ANGLE);
    for _ in 0..800 {
        fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::High);
    }
    let o = fly.lose();
    assert_eq!(fly.state(), State::Blind);
    let want = MASS * (GRAVITY_MSS - BLIND_BELOW_HOVER_MSS);
    for o in [o, fly.quiet(), fly.quiet()] {
        assert_eq!(o.mode, Mode::Angle);
        match o.setpoint {
            Setpoint::Attitude { attitude, heading_rate_rads, thrust_n } => {
                assert_eq!((attitude, heading_rate_rads), ([1.0, 0.0, 0.0, 0.0], 0.0));
                assert!((thrust_n - want).abs() < 1e-3 * want, "{thrust_n} vs {want}");
            }
            other => panic!("{other:?}"),
        }
        assert!(o.armed && o.demand.is_ok());
    }
    // The height back: LAND at once (B6).
    fly.set_valid(ALTITUDE);
    let o = fly.quiet();
    assert_eq!((fly.state(), o.mode), (State::Land, Mode::Altitude));
}

#[test]
fn takeover_descent_flies_the_land_law_and_hand_over_stays_possible() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    let o = fly.rc(true, FULL, Switch3::Low);
    assert_eq!(fly.state(), State::Takeover);
    assert!(is_centred(&o));
    fly.f.battery_critical = true;
    let o = fly.rc(true, FULL, Switch3::Low);
    let d0 = target_d(&o);
    assert_eq!(o.mode, Mode::Position);
    match o.setpoint {
        Setpoint::Position { velocity_ms, heading_rate_rads, .. } => {
            assert_eq!((velocity_ms, heading_rate_rads), ([0.0, 0.0, LAND_SPEED_MS], 0.0), "the LAND law, no real stick");
        }
        other => panic!("{other:?}"),
    }
    for _ in 0..800 {
        fly.rc(true, FULL, Switch3::Low);
    }
    let o = fly.rc(true, FULL, Switch3::High);
    let rate = (target_d(&o) - d0) / (802.0 * DT_US as f32 * 1e-6);
    assert!((rate - LAND_SPEED_MS).abs() < 0.01, "{rate}");
    assert_eq!(fly.state(), State::Takeover);
    let o = fly.rc(true, FULL, Switch3::Middle);
    assert_eq!(fly.state(), State::Pilot, "hand-over");
    assert!(!is_centred(&o));
}

// ---- I12 ----

/// I12 across a GNSS outage (Position lost for 2 s) and a height reset (the vertical flags gone for one step), in each
/// hands-off state: every step's setpoint is covered, the demand is made and the gate stays armed.
#[test]
fn i12_no_uncovered_setpoint_and_no_coverage_disarm_in_any_hands_off_state() {
    let outage = |fly: &mut Fly, name: &str, first: Valid| {
        let mut seen = std::vec::Vec::new();
        let script =
            [(first, 400), (POSITION, 10), (ALTITUDE, 1_600), (ANGLE, 1), (ALTITUDE, 10), (POSITION, 10), (ANGLE, 1), (POSITION, 10)];
        for (valid, n) in script {
            fly.set_valid(valid);
            for _ in 0..n {
                let o = fly.quiet();
                assert!(o.nav.valid.covers(&o.setpoint.requires()), "{name}: {:?} with {valid:?}", o.setpoint);
                assert!(o.demand.is_ok() && o.armed && o.motors.throttles().is_some(), "{name}: {o:?}");
                if seen.last() != Some(&fly.state()) {
                    seen.push(fly.state());
                }
            }
        }
        std::println!("{name}: {seen:?}");
        assert!(seen.iter().all(|s| !matches!(s, State::Stop | State::Pilot)), "{name}: {seen:?}");
    };
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    outage(&mut fly, "hold", POSITION);
    // TAKEOVER: reports keep coming (the newest report fresh), so give one every step.
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    fly.rc(true, FULL, Switch3::Low);
    let script = [(POSITION, 10), (ALTITUDE, 1_600), (ANGLE, 1), (ALTITUDE, 10), (POSITION, 10), (ANGLE, 1), (POSITION, 10)];
    for (valid, n) in script {
        fly.set_valid(valid);
        for _ in 0..n {
            let o = fly.rc(true, FULL, Switch3::Low);
            assert!(o.nav.valid.covers(&o.setpoint.requires()) && o.demand.is_ok() && o.armed, "takeover {o:?}");
            assert_eq!(fly.state(), State::Takeover);
        }
    }
    let mut fly = Fly::new(true, false, POSITION);
    fly.lose();
    assert_eq!(fly.state(), State::Land);
    outage(&mut fly, "land", POSITION);
    let mut fly = Fly::new(true, true, ANGLE);
    for _ in 0..800 {
        fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::High);
    }
    fly.lose();
    assert_eq!(fly.state(), State::Blind);
    outage(&mut fly, "blind", ANGLE);
}

// ---- H12: the hold-integrity distance check on the real step ----

impl Fly {
    /// Moves the fake vehicle `v_n` m/s north for one step: position and velocity.
    fn drift(&self, v_n: f32) {
        let mut n = self.cell.get();
        n.velocity_ms[0] = v_n;
        n.position_m[0] += v_n * DT_US as f32 * 1e-6;
        self.cell.set(n);
    }

    fn track_fault(&self) -> bool {
        self.f.hands_off.track_fault()
    }
}

/// A vehicle that drifts away from its hold point at 1 m/s (entered at rest there: the anchor is the hold point, b 0):
/// HOLD with the position law until the step whose position passes X; that step is LAND with the braking law (H14 (1):
/// the track fault is a landing cause of its own, no debounce; H13's anchor distance trips in the step it is measured);
/// the fault stays latched and LAND keeps braking, never captured, when the vehicle comes back and stops.
#[test]
fn h12_a_vehicle_that_drifts_away_in_hold_lands_braking_in_the_step_its_distance_passes_x() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    assert_eq!(fly.state(), State::Hold);
    let target = fly.f.shaper.target_ne.expect("the hold point");
    assert_eq!(fly.f.hands_off.anchor(), Some((target, 0.0)));
    let mut over = false;
    for k in 0..20_000 {
        fly.drift(1.0);
        let o = fly.quiet();
        let p = fly.cell.get().position_m;
        let d = libm::sqrtf((p[0] - target[0]) * (p[0] - target[0]) + (p[1] - target[1]) * (p[1] - target[1]));
        if d > hands_off::track_fault_m(Tuning::DEFAULT.pilot_speed_xy_ms) {
            assert!(fly.track_fault() && fly.state() == State::Land && o.mode == Mode::Position, "step {k}");
            assert!(is_braking(&o), "step {k}: {:?}", o.setpoint);
            over = true;
            break;
        }
        assert_eq!((fly.state(), o.mode), (State::Hold, Mode::Position), "step {k}");
        assert!(!fly.track_fault() && fly.f.shaper.target_ne == Some(target));
    }
    assert!(over, "the distance passed X");
    // Back at the hold point and stopped: still LAND, braking, never captured, the fault latched.
    let mut n = fly.cell.get();
    n.position_m[0] = target[0];
    n.velocity_ms = [0.0; 3];
    fly.cell.set(n);
    for _ in 0..2_000 {
        let o = fly.quiet();
        assert_eq!((fly.state(), o.mode, fly.f.shaper.target_ne), (State::Land, Mode::Position, None));
        assert!(fly.track_fault() && o.armed);
        assert!(is_braking(&o), "{:?}", o.setpoint);
    }
}

/// Case (a): HOLD entered moving at 2 m/s, 57 m from the origin, with no target: until the capture the controller's
/// target is this step's position (distance 0), the capture is at this step's position; a coverage drop (Altitude, no
/// target: the step's SPN would log PN, PE = 0) while the position jumps 6 m back (more than X from the old target and
/// from 0; within X + b of H13's anchor, which a jump of 57 m would leave), then Position again with a fresh capture:
/// the check never sees a stale or zero target and never trips.
#[test]
fn h12_hold_entry_capture_and_recapture_compare_only_with_a_target_of_that_step() {
    let mut fly = Fly::new(true, true, POSITION);
    let mut n = fly.cell.get();
    n.position_m[0] = 57.0;
    n.velocity_ms[0] = 2.0;
    fly.cell.set(n);
    for _ in 0..10 {
        fly.rc(true, [0.0, 1.0, 0.5, 0.0], Switch3::High);
    }
    assert_eq!(fly.f.shaper.target_ne, None, "the stick moves the vehicle: no target");
    fly.lose();
    assert_eq!(fly.state(), State::Hold);
    let mut captured = false;
    for k in 0..4_000 {
        // Braking at 1 m/s^2 to rest.
        let v = (fly.cell.get().velocity_ms[0] - DT_US as f32 * 1e-6).max(0.0);
        fly.drift(v);
        let o = fly.quiet();
        assert_eq!((fly.state(), o.mode), (State::Hold, Mode::Position), "step {k}");
        assert!(!fly.track_fault(), "step {k}");
        match fly.f.shaper.target_ne {
            None => assert_eq!(fly.f.track_error_m, Some(0.0), "step {k}: unlocked"),
            Some(t) => {
                if !captured {
                    assert_eq!(t, [fly.cell.get().position_m[0], 0.0], "captured at this step's position");
                    captured = true;
                }
                assert!(fly.f.track_error_m.is_some_and(|d| d < 1.0), "step {k}: {:?}", fly.f.track_error_m);
            }
        }
    }
    assert!(captured);
    // The coverage drop with a 6 m jump (a position reset), shorter than the debounce.
    fly.set_valid(ALTITUDE);
    let mut n = fly.cell.get();
    let stale = fly.f.shaper.target_ne.expect("captured");
    n.position_m[0] -= 6.0;
    assert!(stale[0] - n.position_m[0] > hands_off::track_fault_m(Tuning::DEFAULT.pilot_speed_xy_ms) && n.position_m[0] > hands_off::track_fault_m(Tuning::DEFAULT.pilot_speed_xy_ms));
    fly.cell.set(n);
    for k in 0..400 {
        let o = fly.quiet();
        assert_eq!((fly.state(), o.mode), (State::Hold, Mode::Altitude), "step {k}");
        assert_eq!((fly.f.shaper.target_ne, fly.f.track_error_m), (None, None), "step {k}");
    }
    fly.set_valid(POSITION);
    for k in 0..2_000 {
        let o = fly.quiet();
        assert_eq!((fly.state(), o.mode), (State::Hold, Mode::Position), "step {k}");
        assert_eq!(fly.f.shaper.target_ne, Some([n.position_m[0], 0.0]), "re-captured at the new position");
        assert_eq!(fly.f.track_error_m, Some(0.0));
        assert!(!fly.track_fault());
    }
}

/// Case (b), TAKEOVER covered (H13 (3): it flies HOLD's law): a drift in TAKEOVER latches the fault, and that step
/// flies TAKEOVER's descent with the braking law (H14, ranked with battery critical: C1); after hand-over the latch
/// stays, so a later stick loss gives LAND, braking, never HOLD.
#[test]
fn h12_a_drift_in_takeover_latches_and_a_later_stick_loss_lands() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    let o = fly.rc(true, FULL, Switch3::Low);
    assert_eq!((fly.state(), o.mode), (State::Takeover, Mode::Position));
    let mut tripped = false;
    for _ in 0..10_000 {
        fly.drift(1.0);
        let o = fly.rc(true, FULL, Switch3::Low);
        assert_eq!(fly.state(), State::Takeover);
        if fly.track_fault() {
            assert!(o.mode == Mode::Position && is_braking(&o), "{:?}", o.setpoint);
            tripped = true;
            break;
        }
        assert!(o.mode == Mode::Position && is_centred(&o));
    }
    assert!(tripped);
    fly.rc(true, FULL, Switch3::High);
    fly.rc(true, FULL, Switch3::High);
    fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::Middle);
    assert_eq!(fly.state(), State::Pilot, "hand-over");
    assert!(fly.track_fault());
    let o = fly.lose();
    assert_eq!(fly.state(), State::Land);
    assert!(is_braking(&o), "{:?}", o.setpoint);
}

/// PILOT: a 20 m drift from the captured point never sets the fault, and stick loss later gives HOLD; with FS_HOLD_EN 0
/// the same, except that stick loss gives LAND (Amendment N, N2).
#[test]
fn h12_in_pilot_and_with_fs_hold_en_0_a_drift_never_sets_it() {
    for fs in [true, false] {
        let mut fly = Fly::new(fs, true, POSITION);
        assert!(fly.f.shaper.target_ne.is_some());
        for _ in 0..16_000 {
            fly.drift(1.0);
            fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::High);
            assert!(!fly.track_fault());
        }
        assert!(fly.f.track_error_m.is_some_and(|d| d > 19.0), "{:?}", fly.f.track_error_m);
        fly.lose();
        assert_eq!(fly.state(), if fs { State::Hold } else { State::Land });
        assert!(!fly.track_fault());
    }
}

// ---- H13: the anchor, the speed cap, the stale pilot target ----

impl Fly {
    /// The horizontal distance of the fake vehicle from `a`, m (the state machine's arithmetic).
    fn distance_from(&self, a: [f32; 2]) -> f32 {
        let p = self.cell.get().position_m;
        libm::sqrtf((p[0] - a[0]) * (p[0] - a[0]) + (p[1] - a[1]) * (p[1] - a[1]))
    }
}

/// H13 (2) on the real step: for PILOT_SPEED_XY 3, 5 and 20 the controller's limit is the speed in force in PILOT
/// and v_ho = min(speed, 5) in HOLD, TAKEOVER and LAND; a drift at 1 m/s from a hold entered at rest trips in the step
/// its distance passes X = 7, 5, 5 m.
#[test]
fn h13_the_controller_s_limit_is_v_ho_in_the_hands_off_states_and_x_follows_the_speed_in_force() {
    for (v, x, v_ho) in [(3.0f32, 7.0f32, 3.0f32), (5.0, 5.0, 5.0), (20.0, 5.0, 5.0)] {
        let mut fly = Fly::with_speed(true, true, POSITION, v);
        assert_eq!(fly.f.controller.speed_limit_xy(), v, "pilot {v}");
        fly.lose();
        assert_eq!(fly.state(), State::Hold);
        assert_eq!(fly.f.controller.speed_limit_xy(), v_ho, "hold {v}");
        let a = fly.f.hands_off.anchor().expect("the anchor").0;
        let mut tripped = false;
        for k in 0..20_000 {
            fly.drift(1.0);
            let over = fly.distance_from(a) > x;
            fly.quiet();
            assert_eq!(fly.track_fault(), over, "{v}: step {k}");
            if over {
                tripped = true;
                break;
            }
        }
        assert!(tripped, "{v}");
        fly.rc(true, FULL, Switch3::Low);
        assert_eq!(fly.state(), State::Takeover);
        assert_eq!(fly.f.controller.speed_limit_xy(), v_ho, "takeover {v}");
        fly.rc(true, FULL, Switch3::High);
        fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::Middle);
        assert_eq!(fly.state(), State::Pilot);
        assert_eq!(fly.f.controller.speed_limit_xy(), v, "pilot after hand-over {v}");
        let mut fly = Fly::with_speed(true, false, POSITION, v);
        fly.lose();
        assert_eq!(fly.state(), State::Land);
        assert_eq!(fly.f.controller.speed_limit_xy(), v_ho, "land {v}");
    }
}

/// H13 brake-forever: HOLD entered moving straight away at 0.8 m/s (not below `pilot_hold_speed_ms`) and held there:
/// never a capture (the target distance stays 0), A and b = 0.8^2 / 6 set at the entry step, and the fault in the step
/// whose distance from A passes X + b, that step LAND braking (H14 (1)).
#[test]
fn h13_brake_forever_a_vehicle_that_never_slows_below_0_8_m_s_trips_at_x_plus_b() {
    let mut fly = Fly::new(true, true, POSITION);
    let v = Tuning::DEFAULT.pilot_hold_speed_ms;
    fly.drift(v);
    let entry = fly.cell.get().position_m;
    fly.lose();
    assert_eq!(fly.state(), State::Hold);
    let b = v * v / (2.0 * Tuning::GUIDANCE_ACC_MS2);
    assert_eq!(fly.f.hands_off.anchor(), Some(([entry[0], entry[1]], b)));
    let bound = hands_off::track_fault_m(Tuning::DEFAULT.pilot_speed_xy_ms) + b;
    for k in 0..20_000 {
        fly.drift(v);
        let over = fly.distance_from([entry[0], entry[1]]) > bound;
        let o = fly.quiet();
        assert_eq!(fly.track_fault(), over, "step {k}");
        if over {
            std::println!("tripped {} m from A (X + b {bound} m)", fly.distance_from([entry[0], entry[1]]));
            assert_eq!((fly.state(), o.mode), (State::Land, Mode::Position));
            assert!(is_braking(&o), "{:?}", o.setpoint);
            return;
        }
        assert_eq!((fly.state(), o.mode, fly.f.shaper.target_ne, fly.f.track_error_m), (State::Hold, Mode::Position, None, Some(0.0)), "step {k}");
    }
    panic!("brake-forever never tripped");
}

/// H13 (1): HOLD entered at rest, then a creep at 0.5 m/s with a one-step heading drop every 0.5 s (the Altitude law:
/// the target is dropped and re-captured at the next Position step): the target distance stays below 0.3 m, A is never
/// re-set, and the fault comes in the step whose distance from A passes X, which the test makes a heading-drop step
/// (an Altitude setpoint: d_A is read in HOLD whatever the setpoint), that step LAND, level (no T3'). The station's
/// heartbeat keeps the link (the trip comes about 10 s after the loss: link loss would land first, and a level LAND does
/// not read d_A, H14 (2)).
#[test]
fn h13_a_creep_with_tier_dips_and_re_captures_keeps_the_anchor_and_trips_on_a_dip_step() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    assert_eq!(fly.state(), State::Hold);
    let anchor = fly.f.hands_off.anchor();
    assert_eq!(anchor.map(|(_, b)| b), Some(0.0));
    let a = anchor.expect("the anchor").0;
    let x = hands_off::track_fault_m(Tuning::DEFAULT.pilot_speed_xy_ms);
    let dip = Valid { heading: false, ..POSITION };
    let mut captures = std::collections::BTreeSet::new();
    for k in 0..20_000u32 {
        fly.drift(0.5);
        let over = fly.distance_from(a) > x;
        fly.set_valid(if over || k % 400 == 399 { dip } else { POSITION });
        fly.f.set_last_radio_heartbeat_us(Some(fly.t));
        let o = fly.quiet();
        assert_eq!(fly.f.hands_off.anchor(), anchor, "step {k}");
        if over {
            assert!(fly.track_fault() && matches!(o.setpoint, Setpoint::Altitude { .. }), "step {k}: {:?}", o.setpoint);
            assert_eq!(fly.state(), State::Land, "step {k}");
            assert!(captures.len() >= 10, "re-captures {}", captures.len());
            return;
        }
        assert!(!fly.track_fault() && fly.state() == State::Hold, "step {k}");
        if let Some(t) = fly.f.shaper.target_ne {
            captures.insert((t[0].to_bits(), t[1].to_bits()));
            assert!(fly.f.track_error_m.is_some_and(|d| d < 0.3), "step {k}: {:?}", fly.f.track_error_m);
        }
    }
    panic!("the creep never tripped");
}

/// H13 (3): PILOT holding a target the vehicle has drifted 5.5 m from (more than X), sticks centred, at rest: stick loss
/// gives HOLD with the target dropped and re-captured where the vehicle is, the anchor there, and no fault.
#[test]
fn h13_a_stale_pilot_target_5_5_m_away_at_stick_loss_does_not_trip() {
    let mut fly = Fly::new(true, true, POSITION);
    let old = fly.f.shaper.target_ne.expect("the pilot's target");
    let mut n = fly.cell.get();
    n.position_m[0] = old[0] + 5.5;
    fly.cell.set(n);
    fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::High);
    assert_eq!((fly.f.shaper.target_ne, fly.f.track_error_m), (Some(old), Some(5.5)), "stale in PILOT");
    fly.lose();
    assert_eq!(fly.state(), State::Hold);
    let here = [n.position_m[0], n.position_m[1]];
    assert_eq!(fly.f.shaper.target_ne, Some(here), "re-captured at the entry");
    assert_eq!(fly.f.hands_off.anchor(), Some((here, 0.0)));
    for k in 0..2_000 {
        let o = fly.quiet();
        assert_eq!((fly.state(), o.mode), (State::Hold, Mode::Position), "step {k}");
        assert!(!fly.track_fault(), "step {k}");
    }
}

// ---- H14: what happens after a track fault ----

impl Fly {
    /// HOLD (stick loss on T3'), then a drift at 1 m/s until the trip; the vehicle at rest after it. LAND.
    fn tripped() -> Fly {
        let mut fly = Fly::new(true, true, POSITION);
        fly.lose();
        assert_eq!(fly.state(), State::Hold);
        while !fly.track_fault() {
            fly.drift(1.0);
            fly.quiet();
        }
        assert_eq!(fly.state(), State::Land);
        fly.drift(0.0);
        fly
    }

    /// [`Fly::drift`] and, in LAND, down at the LAND rate: the vehicle follows the descent, so neither the airborne latch
    /// nor E3 ends the flight.
    fn drift_landing(&self, v_n: f32) {
        self.drift(v_n);
        let land = self.state() == State::Land;
        let mut n = self.cell.get();
        n.velocity_ms[2] = if land { LAND_SPEED_MS } else { 0.0 };
        if land {
            n.position_m[2] += LAND_SPEED_MS * DT_US as f32 * 1e-6;
        }
        self.cell.set(n);
    }
}

/// LAND's horizontal law as the real step flies it, read from the setpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Law {
    /// [`is_braking`] with no target kept.
    Brake,
    /// Position with a captured target.
    Hold,
    /// Altitude, level.
    Level,
}

fn law_of(fly: &Fly, o: &StepOut) -> Law {
    match o.setpoint {
        Setpoint::Position { velocity_ms, .. } if velocity_ms[2] == LAND_SPEED_MS => match fly.f.shaper.target_ne {
            None if is_braking(o) => Law::Brake,
            Some(_) => Law::Hold,
            None => panic!("{:?}", o.setpoint),
        },
        Setpoint::Altitude { attitude, velocity_d_ms, .. } if attitude == [1.0, 0.0, 0.0, 0.0] && velocity_d_ms == LAND_SPEED_MS => {
            Law::Level
        }
        other => panic!("{other:?}"),
    }
}

/// H14 (1): LAND's horizontal law over the 16 rows track fault x Valid.heading x Valid.velocity_ne x debounced T3' on the
/// real step, the heading input true. LAND with the fault: a trip in HOLD; without: battery critical in HOLD. A row with
/// the debounced T3' true and a flag down is the first step of the drop; with it false the drop has lasted 1.0 s; with
/// both flags up it can be false only through another flag (position_ne down 1.0 s), so Position is not covered (I12).
/// Brake: fault and T3'; Hold: no fault and T3' (this step's, so also the debounced); otherwise Level.
#[test]
fn h14_land_law_table_over_16_rows_on_the_real_step() {
    let debounce = (hands_off::TIER_DEBOUNCE_US / DT_US) as usize + 1;
    for row in 0..16u32 {
        let (fault, heading, velocity_ne, debounced) = (row & 8 != 0, row & 4 != 0, row & 2 != 0, row & 1 != 0);
        let mut fly = if fault {
            Fly::tripped()
        } else {
            let mut fly = Fly::new(true, true, POSITION);
            fly.lose();
            fly.f.battery_critical = true;
            fly.quiet();
            fly.f.battery_critical = false;
            fly
        };
        for _ in 0..10 {
            fly.quiet();
        }
        assert_eq!((fly.state(), fly.track_fault()), (State::Land, fault), "row {row}");
        let position_ne = !(heading && velocity_ne && !debounced);
        fly.set_valid(Valid { heading, velocity_ne, position_ne, ..POSITION });
        let mut o = fly.quiet();
        for _ in 1..if debounced { 1 } else { debounce } {
            o = fly.quiet();
        }
        assert_eq!(fly.state(), State::Land, "row {row}");
        let want = match (fault, heading && velocity_ne && debounced) {
            (true, true) => Law::Brake,
            (false, true) => Law::Hold,
            _ => Law::Level,
        };
        let law = law_of(&fly, &o);
        std::println!("row {row:2}: fault {fault:5} heading {heading:5} velocity_ne {velocity_ne:5} debounced T3' {debounced:5}: {law:?}");
        assert_eq!(law, want, "row {row}: fault {fault} heading {heading} velocity_ne {velocity_ne} debounced T3' {debounced}");
    }
}

/// H14 (1): the braking setpoint on the real step after a trip, at 3.0 m/s, just below `pilot_hold_speed_ms`, 0.3 m/s
/// and at rest: every step Position at that step's navigation position, no horizontal feed-forward, no target kept
/// (never captured). A trip in LAND holding position (its point captured) drops that point at the trip step.
#[test]
fn h14_the_braking_setpoint_is_this_step_s_position_with_no_feed_forward_never_captured() {
    let mut fly = Fly::tripped();
    for v in [3.0, Tuning::DEFAULT.pilot_hold_speed_ms - 0.01, 0.3, 0.0] {
        for k in 0..800 {
            fly.drift_landing(v);
            let o = fly.quiet();
            assert_eq!((fly.state(), o.mode, law_of(&fly, &o)), (State::Land, Mode::Position, Law::Brake), "{v} m/s, step {k}");
        }
    }
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    let a = fly.f.hands_off.anchor().expect("the anchor").0;
    fly.f.battery_critical = true;
    fly.quiet();
    assert_eq!(fly.state(), State::Land);
    let captured = fly.f.shaper.target_ne.expect("the hold point");
    let x = hands_off::track_fault_m(Tuning::DEFAULT.pilot_speed_xy_ms);
    for k in 0..20_000 {
        fly.drift_landing(1.0);
        let over = fly.distance_from(a) > x;
        let o = fly.quiet();
        if over {
            assert!(fly.track_fault(), "step {k}");
            assert_eq!((fly.state(), law_of(&fly, &o)), (State::Land, Law::Brake), "step {k}");
            return;
        }
        assert_eq!((fly.track_fault(), law_of(&fly, &o), fly.f.shaper.target_ne), (false, Law::Hold, Some(captured)), "step {k}");
    }
    panic!("LAND holding never tripped");
}

/// H14 (2), the simulator's G5 case on the real step: HOLD, then the heading flag down (NE valid) beyond the debounce:
/// LAND, level, while the vehicle drifts 8 m from A (X + b = 5 m): d_A is not read, no fault. The heading back: that step
/// LAND would hold, so d_A is read and trips: braking from that step on, never level again while Position is covered.
#[test]
fn h14_a_level_land_does_not_read_the_anchor_and_the_flags_back_give_braking() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    let a = fly.f.hands_off.anchor().expect("the anchor").0;
    fly.set_valid(Valid { heading: false, ..POSITION });
    for k in 0..8_000 {
        fly.drift_landing(0.8);
        let o = fly.quiet();
        assert!(!fly.track_fault(), "step {k}: {} m from A in {:?}", fly.distance_from(a), fly.state());
        assert_eq!(o.mode, Mode::Altitude, "step {k}");
    }
    assert_eq!(fly.state(), State::Land);
    assert!(fly.distance_from(a) > 7.9, "{}", fly.distance_from(a));
    fly.set_valid(POSITION);
    for k in 0..800 {
        fly.drift_landing(0.8);
        let o = fly.quiet();
        assert!(fly.track_fault(), "step {k}");
        assert_eq!((fly.state(), law_of(&fly, &o)), (State::Land, Law::Brake), "step {k}");
    }
}

/// The horizontal acceleration (north, east) of a point mass held at the weight's vertical share by attitude `q`.
fn accel_ne(q: [f32; 4]) -> [f32; 2] {
    let [w, x, y, z] = q;
    let col = [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), w * w - x * x - y * y + z * z];
    [-GRAVITY_MSS * col[0] / col[2], -GRAVITY_MSS * col[1] / col[2]]
}

/// H14 (1) on a point-mass plant: HOLD, the vehicle drifting away at 1 m/s (as the drift tests) until the trip; from the
/// trip step the plant is closed: the acceleration of the step's attitude setpoint (taken at once) plus the drag of a
/// steady 5 m/s north wind, k (w - v) with k = 0.197 N/(m/s) / 0.914 kg (H14's numbers, U-H6), while it descends at the
/// LAND rate from 10 m to 1 m (18 s). Returns the horizontal speed at each step after the trip and the distance from the
/// LAND start at the end.
fn brake_run() -> (std::vec::Vec<f32>, f32) {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    let mut o = loop {
        fly.drift(1.0);
        let o = fly.quiet();
        if fly.track_fault() {
            break o;
        }
    };
    assert_eq!(fly.state(), State::Land);
    let start = fly.cell.get().position_m;
    let (k, wind, dt) = (0.197 / 0.914, 5.0, DT_US as f32 * 1e-6);
    let mut speeds = std::vec::Vec::new();
    for _ in 0..(18_000_000 / DT_US) {
        let a = match o.setpoint {
            Setpoint::Position { .. } => accel_ne(o.attitude_sp),
            Setpoint::Altitude { attitude, .. } => accel_ne(attitude),
            ref other => panic!("{other:?}"),
        };
        let mut n = fly.cell.get();
        let [vn, ve, _] = n.velocity_ms;
        let (vn, ve) = (vn + (a[0] + k * (wind - vn)) * dt, ve + (a[1] - k * ve) * dt);
        n.velocity_ms = [vn, ve, LAND_SPEED_MS];
        n.position_m = [n.position_m[0] + vn * dt, n.position_m[1] + ve * dt, n.position_m[2] + LAND_SPEED_MS * dt];
        fly.cell.set(n);
        o = fly.quiet();
        assert_eq!(fly.state(), State::Land);
        speeds.push(libm::sqrtf(vn * vn + ve * ve));
    }
    (speeds, fly.distance_from([start[0], start[1]]))
}

/// The bound: from 2 s after the trip to the end the horizontal speed stays at or below 0.3 m/s, half of what the
/// velocity loop's P alone would leave against this wind (1.08 m/s^2 / 1.8 = 0.6 m/s, H14), so the integrator must hold
/// the trim; it never exceeds the speed at the trip; and the vehicle ends within D_max of the LAND start (H14's
/// simulator pass rule). The level law (the old behaviour) lets the wind take it: towards 5 m/s with time constant
/// 1/k = 4.6 s, about 2.4 m/s at 2 s, tens of metres by the end.
#[test]
fn h14_after_a_trip_in_wind_land_brakes_and_the_speed_stays_near_zero() {
    let (speeds, drift) = brake_run();
    let per_s = (1_000_000 / DT_US) as usize;
    for (s, v) in speeds.iter().step_by(per_s).enumerate() {
        std::println!("{s:2} s after the trip: {v:.3} m/s");
    }
    let late = speeds[2 * per_s..].iter().copied().fold(0.0f32, f32::max);
    std::println!("largest speed from 2 s on {late:.3} m/s; 1 m above ground {drift:.2} m from the LAND start");
    assert!(speeds.iter().all(|&v| v <= 1.0 + 1e-3), "faster than at the trip");
    assert!(late <= 0.3, "{late} m/s");
    assert!(drift <= hands_off::D_MAX_M, "{drift} m");
}

// ---- E3: the touchdown backstop ----

/// Specific force at rest, level, m/s^2 FRD.
const AT_REST: [f32; 3] = [0.0, 0.0, -GRAVITY_MSS];

impl Fly {
    /// One step without a report, IMU1 reading the specific force `f`.
    fn quiet_f(&mut self, f: [f32; 3]) -> StepOut {
        self.t += DT_US;
        let mut i = input(self.t, None);
        i.sensors.imu1.reading.accel_ms2 = f;
        self.f.step(&i)
    }
}

/// The ground threshold, N (`ground_thrust_g` x m g).
fn ground_n() -> f32 {
    Tuning::DEFAULT.ground_thrust_g * MASS * GRAVITY_MSS
}

/// E3 on a touchdown whose height estimate flickers 1.5 m low for one step every 0.1 s: the airborne latch's count
/// restarts at the flickers and never expires, and the backstop disarms 1.0 s (within one step) after the collective went
/// below the ground threshold for good, the accelerometer at rest.
#[test]
fn e3_a_touchdown_with_a_flickering_height_disarms_within_1_0_s_plus_one_step() {
    let mut fly = Fly::new(true, false, POSITION);
    fly.lose();
    assert_eq!(fly.state(), State::Land);
    let dt = DT_US as f32 * 1e-6;
    let land = Tuning::DEFAULT.land_detect_us;
    let (mut ground, mut since, mut restarts, mut low_prev, mut disarmed) = (false, None, 0u32, false, None);
    for k in 0..40_000u32 {
        let mut n = fly.cell.get();
        let d = (n.position_m[2] + LAND_SPEED_MS * dt).min(0.0);
        ground |= d >= 0.0;
        n.velocity_ms[2] = if d < 0.0 { LAND_SPEED_MS } else { 0.0 };
        n.position_m[2] = if ground && k % 80 == 0 { 1.5 } else { d };
        fly.cell.set(n);
        let o = fly.quiet_f(AT_REST);
        if !o.armed {
            disarmed = Some(fly.t);
            assert_eq!(fly.state(), State::Stop);
            break;
        }
        assert_eq!(fly.state(), State::Land, "step {k}");
        assert!(fly.f.low_us < land, "step {k}: the latch's own count expired");
        if !ground {
            continue;
        }
        let low = o.allocation.is_none_or(|a| a.saturation.thrust_low);
        if o.demand.is_ok_and(|d| d.thrust_n >= ground_n()) {
            (since, restarts) = (None, 0);
        } else {
            since.get_or_insert(fly.t);
            restarts += u32::from(low_prev && !low);
        }
        low_prev = low;
    }
    let (since, disarmed) = (since.expect("below the threshold"), disarmed.expect("disarmed"));
    std::println!("collective below the threshold from {since}, disarmed {} us later, latch count restarts {restarts}", disarmed - since);
    assert!(restarts >= 5, "the latch's count restarted: {restarts}");
    assert!(disarmed - since <= land + DT_US && disarmed - since + DT_US >= land, "{} us", disarmed - since);
}

/// E3 in the air: LAND descending steadily for 1 s (the accelerometer at rest), then the vehicle held where it is while
/// the target descends, so the collective falls below the ground threshold (the airborne latch's timer off, so only E3
/// can clear it); the accelerometer reads the commanded collective times a thrust gain of 0.88, 1.0 and 1.12 (falling at
/// 2 m/s^2 or more below the threshold): the backstop's count never reaches 1.0 s. The same flight with the
/// accelerometer at rest (NaN below) disarms: the condition is not vacuous.
#[test]
fn e3_in_the_air_below_the_ground_threshold_the_acceleration_condition_is_false_at_nominal_and_12_percent_gain() {
    for gain in [0.88f32, 1.0, 1.12, f32::NAN] {
        let mut fly = Fly::new(true, false, POSITION);
        fly.lose();
        assert_eq!(fly.state(), State::Land);
        fly.f.timer_off = true;
        let dt = DT_US as f32 * 1e-6;
        let mut thrust = MASS * GRAVITY_MSS;
        for _ in 0..800 {
            let mut n = fly.cell.get();
            n.velocity_ms[2] = LAND_SPEED_MS;
            n.position_m[2] += LAND_SPEED_MS * dt;
            fly.cell.set(n);
            thrust = fly.quiet_f(AT_REST).demand.map_or(0.0, |d| d.thrust_n);
        }
        assert!(thrust > ground_n() && fly.f.accel_ef_mss.iter().all(|a| a.abs() < 0.1), "{thrust} {:?}", fly.f.accel_ef_mss);
        let mut n = fly.cell.get();
        n.velocity_ms[2] = 0.0;
        fly.cell.set(n);
        let (mut low_steps, mut max_still, mut disarmed) = (0u32, 0u64, false);
        for _ in 0..8_000 {
            let f = if gain.is_nan() { AT_REST } else { [0.0, 0.0, -gain * thrust / MASS] };
            let o = fly.quiet_f(f);
            if !o.armed {
                disarmed = true;
                break;
            }
            thrust = o.demand.map_or(0.0, |d| d.thrust_n);
            low_steps += u32::from(thrust < ground_n());
            max_still = max_still.max(fly.f.still_us);
        }
        std::println!("gain {gain}: {low_steps} steps below the threshold, longest E3 count {max_still} us, disarmed {disarmed}");
        assert_eq!(disarmed, gain.is_nan(), "gain {gain}");
        if !gain.is_nan() {
            assert!(low_steps > 7_000 && max_still < Tuning::DEFAULT.land_detect_us / 2, "gain {gain}");
        }
    }
}

/// E3 is inactive outside LAND and the TAKEOVER descent: HOLD, TAKEOVER and PILOT with the vehicle 3 m above its
/// height target (the collective below the threshold), the accelerometer at rest and the latch's timer off: armed.
/// The TAKEOVER descent on the ground with the timer off: disarmed by E3 within 1.0 s plus one step.
#[test]
fn e3_is_inactive_outside_land_and_the_takeover_descent_and_active_in_the_descent() {
    for name in ["hold", "takeover", "pilot"] {
        let mut fly = Fly::new(true, true, POSITION);
        if name != "pilot" {
            fly.lose();
        }
        if name == "takeover" {
            fly.rc(true, FULL, Switch3::Low);
        }
        fly.f.timer_off = true;
        let mut n = fly.cell.get();
        n.position_m[2] -= 3.0;
        fly.cell.set(n);
        let mut low_steps = 0u32;
        for k in 0..4_000 {
            fly.t += DT_US;
            let mut i = input(fly.t, (name != "hold").then(|| RcSample {
                sticks: Sticks { roll: 0.0, pitch: 0.0, throttle: 0.5, yaw: 0.0 },
                arm: true,
                switch6: if name == "takeover" { Switch3::Low } else { Switch3::High },
                t_us: fly.t,
            }));
            i.sensors.imu1.reading.accel_ms2 = AT_REST;
            let o = fly.f.step(&i);
            assert!(o.armed && fly.f.airborne, "{name} step {k}");
            low_steps += u32::from(o.demand.is_ok_and(|d| d.thrust_n < ground_n()));
        }
        assert!(low_steps > 1_600, "{name}: {low_steps}");
        assert_eq!(fly.f.still_us, 0, "{name}");
    }
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    fly.rc(true, FULL, Switch3::Low);
    fly.f.battery_critical = true;
    fly.f.timer_off = true;
    let dt = DT_US as f32 * 1e-6;
    let mut since = None;
    for k in 0..40_000 {
        // The vehicle follows the target down at the descent rate, and stops on the ground.
        let mut n = fly.cell.get();
        n.position_m[2] = (n.position_m[2] + LAND_SPEED_MS * dt).min(0.0);
        n.velocity_ms[2] = if n.position_m[2] < 0.0 { LAND_SPEED_MS } else { 0.0 };
        fly.cell.set(n);
        fly.t += DT_US;
        let mut i = input(fly.t, Some(RcSample { sticks: Sticks { roll: 1.0, pitch: 1.0, throttle: 1.0, yaw: 1.0 }, arm: true, switch6: Switch3::Low, t_us: fly.t }));
        i.sensors.imu1.reading.accel_ms2 = AT_REST;
        let o = fly.f.step(&i);
        if !o.armed {
            let since: u64 = since.expect("below the threshold");
            assert_eq!(fly.state(), State::Stop);
            assert!(fly.t - since <= Tuning::DEFAULT.land_detect_us + DT_US, "{} us", fly.t - since);
            return;
        }
        assert_eq!(fly.state(), State::Takeover, "step {k}");
        if o.demand.is_ok_and(|d| d.thrust_n < ground_n()) {
            since.get_or_insert(fly.t);
        } else {
            since = None;
        }
    }
    panic!("the TAKEOVER descent on the ground never disarmed");
}

/// [w, x, y, z] of the attitude roll `r`, pitch `p`, yaw `y` (rad; Z-Y-X, body to NED), from the half-angle formulas.
fn euler_q(r: f32, p: f32, y: f32) -> [f32; 4] {
    let (sr, cr) = (libm::sinf(r / 2.0), libm::cosf(r / 2.0));
    let (sp, cp) = (libm::sinf(p / 2.0), libm::cosf(p / 2.0));
    let (sy, cy) = (libm::sinf(y / 2.0), libm::cosf(y / 2.0));
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

/// The specific force at rest with roll `r` and pitch `p` (rad), m/s^2 FRD: the ground's reaction to gravity in the body
/// frame, from the rotation matrix's last row (the yaw has no part).
fn at_rest_tilted(r: f32, p: f32) -> [f32; 3] {
    let g = GRAVITY_MSS;
    [g * libm::sinf(p), -g * libm::sinf(r) * libm::cosf(p), -g * libm::cosf(r) * libm::cosf(p)]
}

/// E3 on a slope (review finding F3): LAND to a touchdown on ground tilted roll 20 deg, pitch -15 deg, yaw 90 deg; at
/// rest there the estimate's attitude is that tilt and IMU1 reads the matching specific force; the latch's timer off, so
/// only E3 can clear it: disarmed 1.0 s (within one step) after the collective went below the ground threshold.
#[test]
fn e3_at_rest_on_a_slope_disarms_within_1_0_s_plus_one_step() {
    let (r, p) = (20f32.to_radians(), (-15f32).to_radians());
    let q = euler_q(r, p, 90f32.to_radians());
    let f_slope = at_rest_tilted(r, p);
    let mut fly = Fly::new(true, false, POSITION);
    fly.lose();
    assert_eq!(fly.state(), State::Land);
    fly.f.timer_off = true;
    let dt = DT_US as f32 * 1e-6;
    let land = Tuning::DEFAULT.land_detect_us;
    let mut since = None;
    for k in 0..40_000u32 {
        // The vehicle follows the target down at the descent rate and comes to rest on the slope.
        let mut n = fly.cell.get();
        n.position_m[2] = (n.position_m[2] + LAND_SPEED_MS * dt).min(0.0);
        let ground = n.position_m[2] >= 0.0;
        n.velocity_ms[2] = if ground { 0.0 } else { LAND_SPEED_MS };
        if ground {
            n.attitude = q;
        }
        fly.cell.set(n);
        let o = fly.quiet_f(if ground { f_slope } else { AT_REST });
        if !o.armed {
            let since: u64 = since.expect("below the threshold on the ground");
            assert_eq!(fly.state(), State::Stop);
            std::println!("slope: disarmed {} us after the collective went below the threshold", fly.t - since);
            assert!(fly.t - since <= land + DT_US && fly.t - since + DT_US >= land, "{} us", fly.t - since);
            return;
        }
        assert_eq!(fly.state(), State::Land, "step {k}");
        if ground && !o.demand.is_ok_and(|d| d.thrust_n >= ground_n()) {
            since.get_or_insert(fly.t);
        } else {
            since = None;
        }
    }
    panic!("never disarmed at rest on the slope");
}

/// E3 in the air tilted 20 deg (roll 20 deg, yaw 90 deg): LAND, the estimate's attitude tilted throughout and IMU1
/// reading the commanded collective along the body's up axis; held where it is after 1 s of steady descent while the
/// target descends, so the collective is below the ground threshold (the latch's timer off): the backstop's count never
/// reaches 1.0 s and the gate stays armed.
#[test]
fn e3_in_the_air_tilted_20_deg_below_the_ground_threshold_does_not_disarm() {
    let mut fly = Fly::new(true, false, POSITION);
    fly.lose();
    assert_eq!(fly.state(), State::Land);
    fly.f.timer_off = true;
    let mut n = fly.cell.get();
    n.attitude = euler_q(20f32.to_radians(), 0.0, 90f32.to_radians());
    fly.cell.set(n);
    let dt = DT_US as f32 * 1e-6;
    let mut thrust = MASS * GRAVITY_MSS;
    for _ in 0..800 {
        let mut n = fly.cell.get();
        n.velocity_ms[2] = LAND_SPEED_MS;
        n.position_m[2] += LAND_SPEED_MS * dt;
        fly.cell.set(n);
        thrust = fly.quiet_f([0.0, 0.0, -thrust / MASS]).demand.map_or(0.0, |d| d.thrust_n);
    }
    let mut n = fly.cell.get();
    n.velocity_ms[2] = 0.0;
    fly.cell.set(n);
    let (mut low_steps, mut max_still) = (0u32, 0u64);
    for k in 0..8_000 {
        let o = fly.quiet_f([0.0, 0.0, -thrust / MASS]);
        assert!(o.armed && fly.state() == State::Land, "step {k}");
        thrust = o.demand.map_or(0.0, |d| d.thrust_n);
        low_steps += u32::from(thrust < ground_n());
        max_still = max_still.max(fly.f.still_us);
    }
    std::println!("tilted in the air: {low_steps} steps below the threshold, longest E3 count {max_still} us");
    assert!(low_steps > 7_000 && max_still < Tuning::DEFAULT.land_detect_us / 2, "{low_steps} {max_still}");
}

// ---- E4: the board's HO MSG only while FS_HOLD_EN in force is 1 ----

/// The seam the board's MSG rule reads (`Flight::fs_hold_in_force`): false at every step of a flight with FS_HOLD_EN 0,
/// arming, mode changes, the RC-loss LAND (N2) and the disarm after it included; a 1 written while armed comes into force at the next disarmed
/// step (B9) and a 0 written while armed only after the next disarm.
#[test]
fn e4_fs_hold_in_force_is_false_at_every_step_with_fs_hold_en_0_and_follows_b9() {
    let mut fly = Fly::new(false, true, POSITION);
    assert!(!fly.f.fs_hold_in_force());
    for sw in [Switch3::Low, Switch3::Middle, Switch3::High] {
        for _ in 0..200 {
            fly.rc(true, [0.0, 0.0, 0.5, 0.0], sw);
            assert!(!fly.f.fs_hold_in_force());
        }
    }
    fly.f.set_fs_hold_en(true);
    let o = fly.lose();
    assert!(o.armed && !fly.f.fs_hold_in_force(), "written while armed: not in force on the armed steps");
    fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::High);
    let o = fly.rc(false, [0.0; 4], Switch3::High);
    assert!(!o.armed && !fly.f.fs_hold_in_force(), "written while armed: not in force on the armed steps");
    fly.quiet();
    assert!(fly.f.fs_hold_in_force(), "in force after a disarmed step");
    let mut fly = Fly::new(true, true, POSITION);
    assert!(fly.f.fs_hold_in_force());
    fly.f.set_fs_hold_en(false);
    for _ in 0..200 {
        fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::High);
        assert!(fly.f.fs_hold_in_force(), "written while armed");
    }
    fly.rc(false, [0.0; 4], Switch3::High);
    fly.rc(false, [0.0; 4], Switch3::High);
    assert!(!fly.f.fs_hold_in_force());
}

// ---- WR3 and return_built on the real step ----

/// WR3 a-d on the real `Flight` over random reports, switches, sticks, coverage, silences, FS_HOLD_EN and the heading
/// monitor's verdict; the airborne latch is the real one (it clears in LAND on the idle floor), and now and then the
/// test clears it between steps (a modelled false "landed"): (a) armed at the end of two consecutive steps implies the
/// second is not STOP; (b) only the switch cycle arms; (c) outside the hands-off states silence disarms at the first
/// step past the timeout and not before; (d) a hands-off state that reads the latch false and the previous step's
/// acceleration condition (N4 (b), WR1's guard; IMU1 reads at rest or zero specific force, at random) ends that step
/// disarmed. And return_built false: never AUTO or RETURN.
#[test]
fn wr3_and_return_built_on_the_real_step() {
    let tuning = Tuning::DEFAULT;
    let cell: &'static Cell<NavState> = std::boxed::Box::leak(std::boxed::Box::new(Cell::new(nav_at(-10.0, POSITION))));
    let mut f = flight(cell, handshake(), &tuning);
    let mut rng = Lcg(0x5eed_0d3a);
    let mut t = 1_000_000u64;
    let (mut quiet, mut arm, mut switch6, mut throttle) = (0u32, false, Switch3::High, 0.0f32);
    let mut newest: Option<u64> = None;
    let (mut prev_armed, mut prev_state, mut off_then_disarmed) = (false, State::Stop, false);
    let mut counts = [0u32; 6];
    let mut seen = [0u32; 8];
    let mut rest = true;
    for k in 0..1_000_000u32 {
        t += DT_US;
        if rng.chance(2) {
            f.set_fs_hold_en(!rng.chance(200));
        }
        if rng.chance(2) {
            f.set_heading_ok(!rng.chance(300));
        }
        if rng.chance(1) {
            cell.set(nav_at(-10.0, [ANGLE, ALTITUDE, POSITION, POSITION][(rng.next() % 4) as usize]));
        }
        if quiet == 0 && rng.chance(3) {
            quiet = 900 + (rng.next() % 1_200) as u32;
        }
        quiet = quiet.saturating_sub(1);
        if rng.chance(3) {
            arm = !arm || rng.chance(500);
        }
        if rng.chance(4) {
            switch6 = [Switch3::Low, Switch3::Middle, Switch3::High][(rng.next() % 3) as usize];
        }
        if !f.gate.armed() {
            throttle = 0.0;
        } else if rng.chance(5) || throttle == 0.0 {
            throttle = [0.5, 0.5, 0.6, 0.0][(rng.next() % 4) as usize];
        }
        let modelled_clear = rng.chance(1) && f.airborne;
        if modelled_clear {
            f.airborne = false;
        }
        let air = f.airborne;
        let rc = (quiet == 0).then(|| RcSample {
            sticks: Sticks { roll: rng.unit() - 0.5, pitch: rng.unit() - 0.5, throttle, yaw: rng.unit() - 0.5 },
            arm,
            switch6,
            t_us: t,
        });
        if rc.is_some() {
            newest = Some(t);
        }
        if rng.chance(2) {
            rest = !rest;
        }
        let still = f.still;
        let mut i = input(t, rc);
        if rest {
            i.sensors.imu1.reading.accel_ms2 = AT_REST;
        }
        let o = f.step(&i);
        let state = f.hands_off_state();
        let hands_off = !matches!(state, State::Pilot | State::Stop);
        assert!(!matches!(state, State::Auto | State::Return), "step {k}: return_built false");
        // (a)
        assert!(!(prev_armed && o.armed && state == State::Stop), "a, step {k}");
        // (b)
        if !prev_armed && o.armed {
            let r = rc.expect("b: arming needs a report");
            assert!(r.arm && off_then_disarmed, "b, step {k}");
            counts[0] += 1;
        }
        // (c), from a step outside the hands-off states too (a hands-off state leaves for STOP by WR1 or the arm switch)
        if prev_armed && !hands_off && matches!(prev_state, State::Pilot | State::Stop) && rc.is_none() {
            let age = newest.map_or(u64::MAX, |n| t - n);
            if age > tuning.rc_fs_timeout_us {
                assert!(!o.armed, "c, step {k}: silence did not disarm");
                counts[1] += 1;
            } else if o.allocation.is_some() {
                assert!(o.armed, "c, step {k}: disarmed before the timeout");
            }
        }
        // (d)
        if !matches!(prev_state, State::Pilot | State::Stop) && !air && still {
            assert!(!o.armed, "d, step {k}: {prev_state:?} grounded, still armed");
            counts[2 + usize::from(modelled_clear)] += u32::from(prev_armed);
        }
        counts[4] += u32::from(hands_off && o.armed);
        counts[5] += u32::from(prev_armed && !o.armed && hands_off);
        if let Some(slot) = seen.get_mut(usize::from(state.code() - 1)) {
            *slot += 1;
        }
        if let Some(r) = rc {
            off_then_disarmed = !r.arm && !o.armed;
        }
        prev_armed = o.armed;
        prev_state = state;
    }
    std::println!("armings, silence disarms, real and modelled WR1 stops, hands-off armed steps, ..: {counts:?}; states {seen:?}");
    assert!(counts[0] > 50 && counts[1] > 10 && counts[3] > 5 && counts[4] > 10_000, "{counts:?}");
    for s in [State::Pilot, State::Takeover, State::Hold, State::Land, State::Stop, State::Blind] {
        assert!(seen[usize::from(s.code() - 1)] > 0, "{s:?} reached: {seen:?}");
    }
}

#[test]
fn hands_off_msg_names_every_state_with_its_code() {
    for s in [State::Pilot, State::Takeover, State::Hold, State::Auto, State::Return, State::Land, State::Stop, State::Blind] {
        let text = hands_off_msg(s);
        assert!(text.starts_with(&std::format!("HO {} ", s.code())), "{text}");
        assert!(text.len() <= 64);
    }
    assert!(hands_off_msg(State::Auto).contains("HOLD's law") && hands_off_msg(State::Return).contains("HOLD's law"));
}
