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

// ---- LAND, the touchdown disarm, BLIND, the TAKEOVER descent ----

/// The setpoint's target height, m down.
fn target_d(o: &StepOut) -> f32 {
    match o.setpoint {
        Setpoint::Altitude { down_m, .. } => down_m,
        Setpoint::Position { position_m, .. } => position_m[2],
        _ => f32::NAN,
    }
}

/// WR1 touchdown: LAND descends (the height target at 0.5 m/s), the vehicle follows it to the ground (d = 0), the real
/// airborne latch clears after land_detect_us on the idle floor, and the gate is disarmed within one step of that: at
/// most land_detect_us plus one step after the first step on the idle floor.
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
        let o = fly.quiet();
        if k == 800 {
            let rate = (target_d(&o) - d0) / (800.0 * dt);
            assert!((rate - LAND_SPEED_MS).abs() < 0.01, "descent {rate} m/s");
            rate_checked = true;
        }
        let low = o.allocation.is_none_or(|a| a.saturation.thrust_low);
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
    std::println!("idle floor at {low}, latch clear {} us later, disarm {} us after the clear", clear - low, dis - clear);
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

/// A vehicle that drifts away from its hold point at 1 m/s: HOLD with the position law until the step after the
/// distance passes X, then the Altitude law (level), and LAND, level, exactly 1.0 s plus one step after that step; the
/// fault stays latched when the vehicle comes back.
#[test]
fn h12_a_vehicle_that_drifts_away_in_hold_lands_level_one_debounce_after_the_distance_passes_x() {
    let mut fly = Fly::new(true, true, POSITION);
    fly.lose();
    assert_eq!(fly.state(), State::Hold);
    let target = fly.f.shaper.target_ne.expect("the hold point");
    let mut over: Option<u64> = None;
    let mut land: Option<u64> = None;
    for k in 0..20_000 {
        fly.drift(1.0);
        let o = fly.quiet();
        if over.is_none() {
            assert_eq!((fly.state(), o.mode), (State::Hold, Mode::Position), "step {k}");
            assert!(!fly.track_fault() && fly.f.shaper.target_ne == Some(target));
            if fly.f.track_error_m.is_some_and(|d| d > hands_off::TRACK_FAULT_M) {
                over = Some(fly.t);
            }
        } else if fly.state() == State::Hold {
            assert!(fly.track_fault());
            assert!(o.mode == Mode::Altitude && is_centred(&o), "T3' lost: level: {:?}", o.setpoint);
        } else {
            assert_eq!(fly.state(), State::Land);
            land = Some(fly.t);
            break;
        }
    }
    let (over, land) = (over.expect("the distance passed X"), land.expect("LAND"));
    assert_eq!(land - over, hands_off::TIER_DEBOUNCE_US + DT_US, "1.0 s after the step that read the distance");
    // Back at the hold point and stopped: still LAND, level, the fault latched.
    let mut n = fly.cell.get();
    n.position_m[0] = target[0];
    n.velocity_ms = [0.0; 3];
    fly.cell.set(n);
    for _ in 0..2_000 {
        let o = fly.quiet();
        assert_eq!((fly.state(), o.mode), (State::Land, Mode::Altitude));
        assert!(fly.track_fault() && o.armed);
        match o.setpoint {
            Setpoint::Altitude { attitude, heading_rate_rads, .. } => assert_eq!((attitude, heading_rate_rads), ([1.0, 0.0, 0.0, 0.0], 0.0)),
            other => panic!("{other:?}"),
        }
    }
}

/// Case (a): HOLD entered moving at 2 m/s, 57 m from the origin, with no target: until the capture the controller's
/// target is this step's position (distance 0), the capture is at this step's position; a coverage drop (Altitude, no
/// target: the step's SPN would log PN, PE = 0) while the position jumps 57 m, then Position again with a fresh capture:
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
    // The coverage drop with a 57 m jump (a position reset), shorter than the debounce.
    fly.set_valid(ALTITUDE);
    let mut n = fly.cell.get();
    n.position_m[0] -= 57.0;
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

/// Case (b), the lead's reading (TAKEOVER flies HOLD's law; H12's list does not name it): a drift in TAKEOVER latches
/// the fault (the Altitude law at the next step); after hand-over the latch stays, so a later stick loss gives LAND,
/// never HOLD.
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
        if tripped {
            assert!(o.mode == Mode::Altitude && is_centred(&o));
            break;
        }
        tripped = fly.track_fault();
    }
    assert!(tripped);
    fly.rc(true, FULL, Switch3::High);
    fly.rc(true, FULL, Switch3::High);
    fly.rc(true, [0.0, 0.0, 0.5, 0.0], Switch3::Middle);
    assert_eq!(fly.state(), State::Pilot, "hand-over");
    assert!(fly.track_fault());
    fly.lose();
    assert_eq!(fly.state(), State::Land);
}

/// PILOT: a 20 m drift from the captured point never sets the fault, and stick loss later gives HOLD; with FS_HOLD_EN 0
/// the same (the goldens prove the outputs bit-identical).
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
        assert_eq!(fly.state(), if fs { State::Hold } else { State::Stop });
        assert!(!fly.track_fault());
    }
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

// ---- E4: the board's HO MSG only while FS_HOLD_EN in force is 1 ----

/// The seam the board's MSG rule reads (`Flight::fs_hold_in_force`): false at every step of a flight with FS_HOLD_EN 0,
/// arming, mode changes and the RC-loss disarm included; a 1 written while armed comes into force at the next disarmed
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
/// step past the timeout and not before; (d) a hands-off state that reads the latch false ends that step disarmed.
/// And return_built false: never AUTO or RETURN.
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
    for k in 0..300_000u32 {
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
        let o = f.step(&input(t, rc));
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
        if !matches!(prev_state, State::Pilot | State::Stop) && !air {
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
