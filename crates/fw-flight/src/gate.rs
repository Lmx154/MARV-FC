//! The gate, the last stage of every flight step, and the command it makes
//! (decision-sim-seams-20261119, section 6): arming, disarming and the RC-loss rule of
//! decision-first-pair-20261119, section 5, steps 1 to 7.

use crate::gnc::{MotorLevels, NavState};
use crate::quat::down_in_body;
use crate::tuning::Tuning;
use fw_core::esc::LINES;
use fw_core::handshake::Handshake;
use fw_core::rc::{RcSample, RcSource};
use marv_card::{IDLE_LEVEL, LEVEL_MAX};

/// The gate disarms and the ESC task clears `ARMED` when the gap between flight steps exceeds this, us. 100 ms is the
/// IMU loop's first recovery back-off (`fw_core::health`, `recovery_backoff_ms: [100, 300, 1000, 3000]`).
pub const FLIGHT_GAP_DISARM_US: u64 = 100_000;

/// What the ESC task sends: Stop on all four lines, or four throttles by DShot line. The field is private, so only
/// `Gate::apply` can make a spinning command. No estimator, controller, allocator, runner or binary can.
///
/// ```compile_fail,E0451
/// let _ = fw_flight::gate::MotorCommand { spin: None };
/// ```
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotorCommand {
    spin: Option<[dshot::Throttle; LINES]>,
}

impl MotorCommand {
    /// Stop on all four lines.
    pub const STOP: MotorCommand = MotorCommand { spin: None };

    /// The throttles by DShot line; `None` means Stop.
    pub fn throttles(&self) -> Option<[dshot::Throttle; LINES]> {
        self.spin
    }

    /// Tests only: a spinning command with these levels by DShot line (each 1..=2000).
    #[cfg(test)]
    pub(crate) fn spinning(levels: [u16; LINES]) -> MotorCommand {
        MotorCommand {
            spin: Some(levels.map(|l| dshot::Throttle::new(l).unwrap())),
        }
    }
}

/// Arming and the RC-loss kill. One concrete type, not a trait, owned by `Flight`.
pub struct Gate {
    armed: bool,
    /// The previous step's time, us; `None` before the first.
    last_step_us: Option<u64>,
    /// When the newest stick report was read, us; `None` before the first.
    last_rc_us: Option<u64>,
    /// A fresh report with the switch off was seen since the last disarm or refused arm.
    seen_off: bool,
    /// The flight's source (N19 1(3)): the source of the report the gate armed on, until disarm.
    source: Option<RcSource>,
    /// Where the armed state is stored for core 0; arming also needs its `try_arm` (no save running).
    handshake: &'static Handshake,
    rc_fs_timeout_us: u64,
    arm_thr_max: f32,
    /// cos(ANGLE_MAX): the lowest third component of the estimated down vector that may arm.
    cos_angle_max: f32,
}

impl Gate {
    /// Disarmed. `handshake` is where the gate stores the armed state for core 0.
    pub(crate) fn new(tuning: &Tuning, handshake: &'static Handshake) -> Self {
        Self {
            armed: false,
            last_step_us: None,
            last_rc_us: None,
            seen_off: false,
            source: None,
            handshake,
            rc_fs_timeout_us: tuning.rc_fs_timeout_us,
            arm_thr_max: tuning.arm_thr_max,
            cos_angle_max: libm::cosf(tuning.angle_max_rad),
        }
    }

    /// The gate's armed state.
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// N19 1(3): armed, only a report of the flight's source counts; disarmed, a CRSF report, or a receiver report with
    /// FS_HOLD_EN `fs_hold_en` 1. `Flight::step` hands no other report to the shaper, the state machine or the gate.
    pub(crate) fn accepts(&self, r: &RcSample, fs_hold_en: bool) -> bool {
        match self.source {
            Some(s) if self.armed => r.source == s,
            _ => r.source == RcSource::Crsf || fs_hold_en,
        }
    }

    /// The last stage of every step. `rc` is this step's new stick report. RC loss is the age of the newest report at
    /// `t_us` above `RC_FS_TIMEOUT`. `levels` is `None` when the controller refused. While armed, `hands_off` keeps RC
    /// loss from disarming (decision-mavlink-mode-20261123, A2) and `arm_off_holds` keeps a report with the switch off
    /// from disarming (C1); both are ignored while disarmed, so neither can arm.
    pub fn apply(
        &mut self,
        t_us: u64,
        rc: Option<&RcSample>,
        nav: &NavState,
        levels: Option<&MotorLevels>,
        hands_off: bool,
        arm_off_holds: bool,
    ) -> MotorCommand {
        // 0. A gap above the limit since the previous step, or armed while `ARMED` is clear: disarm.
        let gap = self.last_step_us.map(|last| t_us.saturating_sub(last));
        self.last_step_us = Some(t_us);
        if gap.is_some_and(|g| g > FLIGHT_GAP_DISARM_US) || (self.armed && !self.handshake.armed())
        {
            self.disarm();
        }
        // 1 to 3. RC loss: Stop and disarm.
        if let Some(r) = rc {
            self.last_rc_us = Some(r.t_us);
        }
        let fresh = self
            .last_rc_us
            .is_some_and(|last| t_us.saturating_sub(last) <= self.rc_fs_timeout_us);
        if !fresh && !(self.armed && hands_off) {
            self.disarm();
            return MotorCommand::STOP;
        }
        // 4. A new report: the switch off disarms at once; on, it arms only from a switch cycle with every check met.
        if let Some(r) = rc {
            if !r.arm {
                if !(self.armed && arm_off_holds) {
                    self.disarm();
                    self.seen_off = true;
                }
            } else if !self.armed {
                let ready = self.seen_off
                    && r.sticks.throttle <= self.arm_thr_max
                    && nav.valid.tilt
                    && nav.valid.rates
                    && down_in_body(nav.attitude)[2] >= self.cos_angle_max
                    && levels.is_some()
                    && !(r.source == RcSource::Receiver && r.mission);
                if ready && self.handshake.try_arm() {
                    self.armed = true;
                    self.source = Some(r.source);
                } else {
                    self.seen_off = false;
                }
            }
        }
        // 5, 6. Not armed, or armed with no levels (the controller refused): Stop.
        if !self.armed {
            return MotorCommand::STOP;
        }
        let Some(levels) = levels else {
            self.disarm();
            return MotorCommand::STOP;
        };
        // 7. The four throttles, each level in [IDLE_LEVEL, LEVEL_MAX].
        let spin = levels
            .level
            .map(|l| dshot::Throttle::new(l.clamp(IDLE_LEVEL, LEVEL_MAX)));
        match spin {
            [Some(a), Some(b), Some(c), Some(d)] => MotorCommand {
                spin: Some([a, b, c, d]),
            },
            _ => MotorCommand::STOP,
        }
    }

    /// Disarmed, and the switch must be cycled. Clears the handshake's `ARMED` only when leaving armed, so a flag
    /// the gate did not set (feature `fake-armed`) is left alone. `Flight::step` calls it before `apply` when the hands-off
    /// state machine enters STOP from an armed step (decision-mavlink-mode-20261123, WR2).
    pub(crate) fn disarm(&mut self) {
        if self.armed {
            self.handshake.disarm();
        }
        self.armed = false;
        self.seen_off = false;
        self.source = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnc::Valid;
    use fw_core::rc::{Sticks, Switch3};

    static HANDSHAKE: Handshake = Handshake::new();
    const TIMEOUT: u64 = Tuning::DEFAULT.rc_fs_timeout_us;

    fn report(t_us: u64, arm: bool, throttle: f32) -> RcSample {
        let sticks = Sticks {
            roll: 0.0,
            pitch: 0.0,
            throttle,
            yaw: 0.0,
        };
        RcSample {
            sticks,
            arm,
            switch6: Switch3::Low,
            source: fw_core::rc::RcSource::Crsf,
            mission: false,
            t_us,
        }
    }

    /// Steps the gate with level nav and these levels; returns the command.
    fn step(
        gate: &mut Gate,
        t_us: u64,
        rc: Option<RcSample>,
        levels: Option<[u16; 4]>,
    ) -> MotorCommand {
        step_flags(gate, t_us, rc, levels, false, false)
    }

    fn level_nav(t_us: u64) -> NavState {
        NavState {
            t_us,
            attitude: [1.0, 0.0, 0.0, 0.0],
            rates_rads: [0.0; 3],
            velocity_ms: [0.0; 3],
            position_m: [0.0; 3],
            valid: Valid {
                tilt: true,
                rates: true,
                ..Valid::NONE
            },
            sigma: None,
        }
    }

    /// `step` with the two hands-off inputs.
    fn step_flags(
        gate: &mut Gate,
        t_us: u64,
        rc: Option<RcSample>,
        levels: Option<[u16; 4]>,
        hands_off: bool,
        arm_off_holds: bool,
    ) -> MotorCommand {
        let nav = level_nav(t_us);
        let levels = levels.map(|level| MotorLevels { level });
        gate.apply(t_us, rc.as_ref(), &nav, levels.as_ref(), hands_off, arm_off_holds)
    }

    /// Asserts Stop, disarmed, and the handshake not armed.
    fn assert_stopped(gate: &Gate, cmd: MotorCommand, what: &str) {
        assert_eq!(cmd, MotorCommand::STOP, "{what}");
        assert!(!gate.armed(), "{what}");
        assert!(!HANDSHAKE.armed(), "{what}: handshake");
    }

    #[test]
    fn gate_arming_sequence() {
        HANDSHAKE.set_permit(true);
        let mut gate = Gate::new(&Tuning::DEFAULT, &HANDSHAKE);
        let levels = Some([50, 600, 1500, 2500]);
        let want = [IDLE_LEVEL, 600, 1500, LEVEL_MAX];
        let mut t = 1_000;
        let cycle_on = |gate: &mut Gate, t: &mut u64| {
            *t += 1_000;
            let off = step(gate, *t, Some(report(*t, false, 0.0)), levels);
            assert_stopped(gate, off, "switch off");
            *t += 1_000;
            step(gate, *t, Some(report(*t, true, 0.0)), levels)
        };

        let cmd = step(&mut gate, t, None, levels);
        assert_stopped(&gate, cmd, "no report ever");
        t += 1_000;
        let cmd = step(&mut gate, t, Some(report(t, true, 0.5)), levels);
        assert_stopped(&gate, cmd, "switch on, throttle high");
        t += 1_000;
        let cmd = step(&mut gate, t, Some(report(t, true, 0.0)), levels);
        assert_stopped(&gate, cmd, "throttle low, switch not cycled");

        let cmd = cycle_on(&mut gate, &mut t);
        let levels_out = cmd.throttles().map(|th| th.map(|x| x.level()));
        assert_eq!(levels_out, Some(want));
        assert!(gate.armed() && HANDSHAKE.armed());

        // RC loss: still armed at exactly the timeout, Stop one microsecond later.
        // Stepped every FLIGHT_GAP_DISARM_US (a gap of exactly the limit does not trip the latch), so only RC loss acts.
        let last = t;
        for k in 1..=TIMEOUT / FLIGHT_GAP_DISARM_US {
            let cmd = step(&mut gate, last + k * FLIGHT_GAP_DISARM_US, None, levels);
            assert!(
                cmd.throttles().is_some(),
                "gap of exactly the limit, step {k}"
            );
        }
        assert!(gate.armed() && HANDSHAKE.armed());
        let cmd = step(&mut gate, last + TIMEOUT + 1, None, levels);
        assert_stopped(&gate, cmd, "report older than the timeout");
        t = last + 2 * TIMEOUT;
        let cmd = step(&mut gate, t, Some(report(t, true, 0.0)), levels);
        assert_stopped(&gate, cmd, "reports resume, switch still on");

        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());
        t += 1_000;
        let cmd = step(&mut gate, t, Some(report(t, false, 0.0)), levels);
        assert_stopped(&gate, cmd, "switch off disarms at once");

        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());
        t += 1_000;
        let cmd = step(&mut gate, t, None, None);
        assert_stopped(&gate, cmd, "levels None while armed");

        // A save running: try_arm is refused and the gate stays disarmed.
        assert!(HANDSHAKE.begin_save());
        let cmd = cycle_on(&mut gate, &mut t);
        assert_stopped(&gate, cmd, "try_arm refused");
        HANDSHAKE.end_save();

        // I1: a gap of exactly the limit stays armed; one microsecond more gives Stop and disarm; the switch still on
        // stays Stop; a switch cycle arms again.
        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());
        t += FLIGHT_GAP_DISARM_US;
        let cmd = step(&mut gate, t, None, levels);
        assert!(cmd.throttles().is_some() && gate.armed() && HANDSHAKE.armed());
        t += FLIGHT_GAP_DISARM_US + 1;
        let cmd = step(&mut gate, t, None, levels);
        assert_stopped(&gate, cmd, "gap over the limit");
        t += 1_000;
        let cmd = step(&mut gate, t, Some(report(t, true, 0.0)), levels);
        assert_stopped(&gate, cmd, "after the gap, switch still on");
        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());

        // I2: the gate is never armed at the end of a step while ARMED is clear.
        HANDSHAKE.disarm();
        t += 1_000;
        let cmd = step(&mut gate, t, None, levels);
        assert_stopped(&gate, cmd, "ARMED cleared between steps");
        t += 1_000;
        let cmd = step(&mut gate, t, Some(report(t, true, 0.0)), levels);
        assert_stopped(&gate, cmd, "ARMED cleared, switch still on");
        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());

        // P-I1 (decision-prearm-20261119): arming is refused while core 0's permit is clear; with it set, a cycle arms.
        HANDSHAKE.set_permit(false);
        let cmd = cycle_on(&mut gate, &mut t);
        assert_stopped(&gate, cmd, "permit clear");
        HANDSHAKE.set_permit(true);
        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());

        // P-I2: setting the permit while the switch is on does not arm; a switch cycle does.
        HANDSHAKE.set_permit(false);
        let cmd = cycle_on(&mut gate, &mut t);
        assert_stopped(&gate, cmd, "permit clear, switch on");
        HANDSHAKE.set_permit(true);
        t += 1_000;
        let cmd = step(&mut gate, t, Some(report(t, true, 0.0)), levels);
        assert_stopped(&gate, cmd, "permit set while the switch is on");
        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());

        // P-I3: clearing the permit while armed changes nothing (200 steps with throttles).
        HANDSHAKE.set_permit(false);
        for k in 0..200 {
            t += 1_000;
            let cmd = step(&mut gate, t, Some(report(t, true, 0.0)), levels);
            let levels_out = cmd.throttles().map(|th| th.map(|x| x.level()));
            assert_eq!(
                levels_out,
                Some(want),
                "permit cleared while armed, step {k}"
            );
            assert!(gate.armed() && HANDSHAKE.armed(), "step {k}");
        }
    }

    const LEVELS: Option<[u16; 4]> = Some([600, 600, 600, 600]);

    /// Arms `gate` by a switch cycle from `*t`; flags false.
    fn arm(gate: &mut Gate, t: &mut u64) {
        *t += 1_000;
        step(gate, *t, Some(report(*t, false, 0.0)), LEVELS);
        *t += 1_000;
        assert!(step(gate, *t, Some(report(*t, true, 0.0)), LEVELS).throttles().is_some());
        assert!(gate.armed());
    }

    /// Decision-mavlink-mode A2/I2: with `hands_off` RC loss does not disarm; without it the gate disarms at exactly
    /// the step it does today (the timeout plus one microsecond).
    #[test]
    fn gate_hands_off_keeps_rc_loss_from_disarming() {
        static HS_OFF: Handshake = Handshake::new();
        static HS_ON: Handshake = Handshake::new();
        HS_OFF.set_permit(true);
        HS_ON.set_permit(true);
        let mut off = Gate::new(&Tuning::DEFAULT, &HS_OFF);
        let mut on = Gate::new(&Tuning::DEFAULT, &HS_ON);
        let (mut t_off, mut t_on) = (1_000, 1_000);
        arm(&mut off, &mut t_off);
        arm(&mut on, &mut t_on);
        assert_eq!(t_off, t_on);
        let last = t_on;
        for k in 1..=TIMEOUT / FLIGHT_GAP_DISARM_US {
            let t = last + k * FLIGHT_GAP_DISARM_US;
            assert!(step_flags(&mut off, t, None, LEVELS, false, false).throttles().is_some(), "{k}");
            assert!(step_flags(&mut on, t, None, LEVELS, true, false).throttles().is_some(), "{k}");
        }
        let t = last + TIMEOUT + 1;
        let cmd = step_flags(&mut off, t, None, LEVELS, false, false);
        assert!(cmd == MotorCommand::STOP && !off.armed() && !HS_OFF.armed(), "false: disarmed at timeout + 1");
        let cmd = step_flags(&mut on, t, None, LEVELS, true, false);
        assert!(cmd.throttles().is_some() && on.armed() && HS_ON.armed(), "true: still armed");
        // A minute without reports, then reports resume with the switch on: armed throughout.
        let mut t = t;
        for _ in 0..600 {
            t += FLIGHT_GAP_DISARM_US;
            assert!(step_flags(&mut on, t, None, LEVELS, true, false).throttles().is_some());
        }
        t += 1_000;
        assert!(step_flags(&mut on, t, Some(report(t, true, 0.5)), LEVELS, true, false).throttles().is_some());
        // Unchanged while hands-off: the flight gap, levels None, and RC loss once `hands_off` is false.
        t += FLIGHT_GAP_DISARM_US + 1;
        assert_eq!(step_flags(&mut on, t, None, LEVELS, true, false), MotorCommand::STOP);
        assert!(!on.armed() && !HS_ON.armed(), "flight gap");
        arm(&mut on, &mut t);
        t += 1_000;
        assert_eq!(step_flags(&mut on, t, None, None, true, true), MotorCommand::STOP);
        assert!(!on.armed() && !HS_ON.armed(), "levels None");
        arm(&mut on, &mut t);
        let last = t;
        for k in 1..=TIMEOUT / FLIGHT_GAP_DISARM_US {
            step_flags(&mut on, last + k * FLIGHT_GAP_DISARM_US, None, LEVELS, true, false);
        }
        assert!(on.armed());
        let cmd = step_flags(&mut on, last + TIMEOUT + FLIGHT_GAP_DISARM_US, None, LEVELS, false, false);
        assert!(cmd == MotorCommand::STOP && !on.armed(), "hands_off false while lost: disarm");
        // Ignored while disarmed: RC loss clears seen_off as today, and nothing arms.
        let mut t = last + 2 * TIMEOUT;
        step_flags(&mut on, t, Some(report(t, false, 0.0)), LEVELS, true, false);
        assert!(on.seen_off && !on.armed());
        t += TIMEOUT + 1;
        assert_eq!(step_flags(&mut on, t, None, LEVELS, true, true), MotorCommand::STOP);
        assert!(!on.seen_off && !on.armed());
        t += 1_000;
        assert_eq!(step_flags(&mut on, t, Some(report(t, true, 0.0)), LEVELS, true, true), MotorCommand::STOP);
        assert!(!on.armed() && !HS_ON.armed());
    }

    /// C1: with `arm_off_holds` a report with the switch off leaves `armed` and `seen_off` alone; disarmed, the flag is
    /// ignored and cannot arm.
    #[test]
    fn gate_arm_off_holds_keeps_an_arm_off_report_from_disarming_and_never_arms() {
        static HS_HOLD: Handshake = Handshake::new();
        HS_HOLD.set_permit(true);
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_HOLD);
        let mut t = 1_000;
        arm(&mut gate, &mut t);
        let seen_off = gate.seen_off;
        for _ in 0..50 {
            t += 1_000;
            let cmd = step_flags(&mut gate, t, Some(report(t, false, 0.0)), LEVELS, true, true);
            assert!(cmd.throttles().is_some() && gate.armed() && HS_HOLD.armed());
            assert_eq!(gate.seen_off, seen_off);
        }
        // The flag dropped (K set): the next arm-OFF report disarms at once.
        t += 1_000;
        let cmd = step_flags(&mut gate, t, Some(report(t, false, 0.0)), LEVELS, true, false);
        assert_stopped_on(&gate, &HS_HOLD, cmd, "flag false");
        assert!(gate.seen_off, "as today: the switch-off report counts toward the cycle");
        // Disarmed, both flags true: switch-on reports without a cycle never arm.
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_HOLD);
        for _ in 0..50 {
            t += 1_000;
            let cmd = step_flags(&mut gate, t, Some(report(t, true, 0.0)), LEVELS, true, true);
            assert_stopped_on(&gate, &HS_HOLD, cmd, "no cycle");
        }
        // Disarmed, the flag is ignored: switch off sets seen_off as today, and only the existing cycle arms.
        t += 1_000;
        let cmd = step_flags(&mut gate, t, Some(report(t, false, 0.0)), LEVELS, true, true);
        assert_stopped_on(&gate, &HS_HOLD, cmd, "switch off");
        assert!(gate.seen_off);
        t += 1_000;
        assert!(step_flags(&mut gate, t, Some(report(t, true, 0.0)), LEVELS, true, true).throttles().is_some());
    }

    fn assert_stopped_on(gate: &Gate, hs: &Handshake, cmd: MotorCommand, what: &str) {
        assert_eq!(cmd, MotorCommand::STOP, "{what}");
        assert!(!gate.armed() && !hs.armed(), "{what}");
    }

    struct Rng(u64);

    impl Rng {
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
    }

    /// One random step's facts: time, report, levels and the two flags.
    struct Facts {
        t: u64,
        rc: Option<RcSample>,
        levels: Option<[u16; 4]>,
        hands_off: bool,
        arm_off_holds: bool,
    }

    /// `quiet` counts down a stretch of steps with no report (up to 3 s), so RC loss happens without a flight gap.
    fn random_facts(rng: &mut Rng, t: &mut u64, quiet: &mut u64) -> Facts {
        if *quiet == 0 && rng.chance(3) {
            *quiet = 1 + rng.next() % 300;
        }
        *quiet = quiet.saturating_sub(1);
        *t += match rng.next() % 1000 {
            0 => FLIGHT_GAP_DISARM_US + 1 + rng.next() % 100_000,
            1..=2 => TIMEOUT / 2 + rng.next() % TIMEOUT,
            _ => 1 + rng.next() % 20_000,
        };
        let throttles = [0.0, 0.0, 0.05, 0.5];
        let rc = (*quiet == 0 && rng.chance(600)).then(|| {
            let throttle = throttles[(rng.next() % 4) as usize];
            report(*t, !rng.chance(300), throttle)
        });
        Facts {
            t: *t,
            rc,
            levels: if rng.chance(2) { None } else { LEVELS },
            hands_off: rng.chance(500),
            arm_off_holds: rng.chance(500),
        }
    }

    /// I1 / C1 a5: over random reports, gaps, levels and both flags, disarmed becomes armed only on a switch-on report
    /// with the throttle low and levels present, right after a switch-off report that left the gate disarmed.
    #[test]
    fn gate_walk_flags_never_arm_outside_the_switch_cycle() {
        static HS_WALK: Handshake = Handshake::new();
        HS_WALK.set_permit(true);
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_WALK);
        let mut rng = Rng(0x5eed_00a5);
        let (mut t, mut quiet) = (1_000, 0);
        let mut off_then_disarmed = false;
        let (mut armings, mut held_loss, mut held_off) = (0u32, 0u32, 0u32);
        for _ in 0..300_000 {
            if rng.chance(2) {
                HS_WALK.disarm();
            }
            let f = random_facts(&mut rng, &mut t, &mut quiet);
            let was = gate.armed();
            let lost = gate.last_rc_us.is_some_and(|l| f.t.saturating_sub(l) > TIMEOUT) && f.rc.is_none();
            let cmd = step_flags(&mut gate, f.t, f.rc, f.levels, f.hands_off, f.arm_off_holds);
            assert!(!gate.armed() || HS_WALK.armed());
            assert_eq!(cmd.throttles().is_some(), gate.armed() && f.levels.is_some());
            if !was && gate.armed() {
                let r = f.rc.expect("arming needs a report");
                assert!(r.arm && r.sticks.throttle <= Tuning::DEFAULT.arm_thr_max && f.levels.is_some());
                assert!(off_then_disarmed, "arming without a switch cycle");
                armings += 1;
            }
            if was && gate.armed() {
                held_loss += u32::from(lost && f.hands_off);
                held_off += u32::from(f.rc.is_some_and(|r| !r.arm) && f.arm_off_holds);
            }
            if let Some(r) = f.rc {
                off_then_disarmed = !r.arm && !gate.armed();
            }
        }
        assert!(armings > 100 && held_loss > 100 && held_off > 100, "{armings} {held_loss} {held_off}");
    }

    /// The gate as it was at 047aff0, verbatim but for the handshake argument: the reference for (false, false).
    struct OldGate {
        armed: bool,
        last_step_us: Option<u64>,
        last_rc_us: Option<u64>,
        seen_off: bool,
        handshake: &'static Handshake,
        rc_fs_timeout_us: u64,
        arm_thr_max: f32,
        cos_angle_max: f32,
    }

    impl OldGate {
        fn apply(
            &mut self,
            t_us: u64,
            rc: Option<&RcSample>,
            nav: &NavState,
            levels: Option<&MotorLevels>,
        ) -> MotorCommand {
            let gap = self.last_step_us.map(|last| t_us.saturating_sub(last));
            self.last_step_us = Some(t_us);
            if gap.is_some_and(|g| g > FLIGHT_GAP_DISARM_US) || (self.armed && !self.handshake.armed())
            {
                self.disarm();
            }
            if let Some(r) = rc {
                self.last_rc_us = Some(r.t_us);
            }
            let fresh = self
                .last_rc_us
                .is_some_and(|last| t_us.saturating_sub(last) <= self.rc_fs_timeout_us);
            if !fresh {
                self.disarm();
                return MotorCommand::STOP;
            }
            if let Some(r) = rc {
                if !r.arm {
                    self.disarm();
                    self.seen_off = true;
                } else if !self.armed {
                    let ready = self.seen_off
                        && r.sticks.throttle <= self.arm_thr_max
                        && nav.valid.tilt
                        && nav.valid.rates
                        && down_in_body(nav.attitude)[2] >= self.cos_angle_max
                        && levels.is_some();
                    if ready && self.handshake.try_arm() {
                        self.armed = true;
                    } else {
                        self.seen_off = false;
                    }
                }
            }
            if !self.armed {
                return MotorCommand::STOP;
            }
            let Some(levels) = levels else {
                self.disarm();
                return MotorCommand::STOP;
            };
            let spin = levels
                .level
                .map(|l| dshot::Throttle::new(l.clamp(IDLE_LEVEL, LEVEL_MAX)));
            match spin {
                [Some(a), Some(b), Some(c), Some(d)] => MotorCommand {
                    spin: Some([a, b, c, d]),
                },
                _ => MotorCommand::STOP,
            }
        }

        fn disarm(&mut self) {
            if self.armed {
                self.handshake.disarm();
            }
            self.armed = false;
            self.seen_off = false;
        }
    }

    /// I2: with (false, false) the gate is bit-identical to the gate before the two inputs, step by step, over a
    /// seeded walk of reports, gaps, levels, permit, saves and `ARMED` cleared from outside.
    #[test]
    fn gate_false_false_matches_the_old_gate_step_by_step() {
        static HS_NEW: Handshake = Handshake::new();
        static HS_OLD: Handshake = Handshake::new();
        HS_NEW.set_permit(true);
        HS_OLD.set_permit(true);
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_NEW);
        let tuning = Tuning::DEFAULT;
        let mut old = OldGate {
            armed: false,
            last_step_us: None,
            last_rc_us: None,
            seen_off: false,
            handshake: &HS_OLD,
            rc_fs_timeout_us: tuning.rc_fs_timeout_us,
            arm_thr_max: tuning.arm_thr_max,
            cos_angle_max: libm::cosf(tuning.angle_max_rad),
        };
        let mut rng = Rng(0x5eed_0012);
        let (mut t, mut quiet) = (1_000, 0);
        let (mut armed_steps, mut spins) = (0u32, 0u32);
        for k in 0..300_000 {
            if rng.chance(2) {
                HS_NEW.disarm();
                HS_OLD.disarm();
            }
            if rng.chance(5) {
                let permit = !rng.chance(300);
                HS_NEW.set_permit(permit);
                HS_OLD.set_permit(permit);
            }
            let f = random_facts(&mut rng, &mut t, &mut quiet);
            let levels = f.levels.map(|level| MotorLevels { level });
            let nav = level_nav(f.t);
            let want = old.apply(f.t, f.rc.as_ref(), &nav, levels.as_ref());
            let got = gate.apply(f.t, f.rc.as_ref(), &nav, levels.as_ref(), false, false);
            assert_eq!(got, want, "step {k}");
            assert_eq!(
                (gate.armed, gate.seen_off, gate.last_rc_us, HS_NEW.armed()),
                (old.armed, old.seen_off, old.last_rc_us, HS_OLD.armed()),
                "step {k}"
            );
            armed_steps += u32::from(gate.armed);
            spins += u32::from(got.throttles().is_some());
        }
        assert!(armed_steps > 10_000 && spins > 10_000, "{armed_steps} {spins}");
    }

    /// N19 section 3: a receiver report with the mission switch ON never arms (switch cycle, low throttle and every
    /// check met), with it OFF it does; a receiver report counts while disarmed only at FS_HOLD_EN 1; the source the gate
    /// armed on is latched until disarm (`accepts`), both ways.
    #[test]
    fn n19_mission_switch_on_never_arms_and_the_source_is_latched() {
        static HS_N19: Handshake = Handshake::new();
        HS_N19.set_permit(true);
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_N19);
        let levels = Some([600; 4]);
        let rx = |t: u64, arm: bool, mission: bool| RcSample { source: RcSource::Receiver, mission, ..report(t, arm, 0.0) };
        let mut t = 1_000;
        for _ in 0..5 {
            t += 1_000;
            step(&mut gate, t, Some(rx(t, false, true)), levels);
            t += 1_000;
            let c = step(&mut gate, t, Some(rx(t, true, true)), levels);
            assert!(c == MotorCommand::STOP && !gate.armed() && !HS_N19.armed(), "mission switch ON");
        }
        assert!(gate.accepts(&report(t, true, 0.0), false) && gate.accepts(&report(t, true, 0.0), true));
        assert!(!gate.accepts(&rx(t, true, false), false), "disarmed, FS_HOLD_EN 0: no receiver report");
        assert!(gate.accepts(&rx(t, true, false), true));
        t += 1_000;
        step(&mut gate, t, Some(rx(t, false, false)), levels);
        t += 1_000;
        let c = step(&mut gate, t, Some(rx(t, true, false)), levels);
        assert!(gate.armed() && c.throttles().is_some(), "mission switch OFF arms");
        assert!(gate.accepts(&rx(t, true, true), false), "the flight's source, FS_HOLD_EN aside");
        assert!(!gate.accepts(&report(t, true, 0.0), true), "a CRSF report on a MAVLink flight");
        t += 1_000;
        step(&mut gate, t, Some(rx(t, false, false)), levels);
        assert!(!gate.armed());
        t += 1_000;
        step(&mut gate, t, Some(report(t, false, 0.0)), levels);
        t += 1_000;
        step(&mut gate, t, Some(report(t, true, 0.0)), levels);
        assert!(gate.armed(), "CRSF arms");
        assert!(!gate.accepts(&rx(t, true, false), true), "a receiver report on a CRSF flight");
        assert!(gate.accepts(&report(t, true, 0.0), false));
    }

    /// Amendment D, WR3: the real gate driven by the real hands-off machine. Each step the machine runs first with the
    /// armed state and the airborne latch the previous step left; its `disarm` goes through the gate's disarm path,
    /// then `apply` takes this step's report and the Output's `hands_off` and `arm_off_holds`. The airborne latch lives
    /// in step.rs (land_detect_us on the idle floor): here it is a modelled input, cleared after any disarmed step and
    /// otherwise set and cleared at random, so (d) is checked from the step the latch reads false.
    #[test]
    fn gate_and_hands_off_joint_walk_wr3() {
        use crate::hands_off::{Command, HandsOff, Inputs, Link, MissionFacts, State, StationCommand, Tier};
        static HS_JOINT: Handshake = Handshake::new();
        HS_JOINT.set_permit(true);
        let tuning = Tuning::DEFAULT;
        let mut gate = Gate::new(&tuning, &HS_JOINT);
        let mut h = HandsOff::new(&tuning);
        let mut rng = Rng(0x5eed_00d4);
        let mut i = Inputs {
            now_us: 0,
            fs_hold_en: true,
            crsf: None,
            last_radio_heartbeat_us: None,
            last_radio_status_us: None,
            armed: false,
            airborne: false,
            tier: Tier::T3,
            position_ne_m: [0.0, 0.0],
            position_ne_valid: true,
            velocity_ne_ms: [0.0, 0.0],
            velocity_ne_valid: true,
            position_d_m: -5.0,
            position_d_valid: true,
            command: None,
            mission: MissionFacts {
                plan_id: Some(7),
                has_waypoints: true,
                mission_stored: true,
                z_within_limits: true,
                max_waypoint_distance_m: Some(100.0),
                return_height_m: Some(20.0),
                alt_max_m: fw_core::mission::ALT_MAX_DEFAULT,
            },
            last_waypoint_reached: false,
            home_reached: false,
            heading_ok: true,
            battery_ok: true,
            battery_critical: false,
            return_built: true,
            track_error_m: None,
            speed_xy_ms: tuning.pilot_speed_xy_ms,
            td_held: true,
            lifted: false,
        };
        let (mut t, mut quiet, mut airborne) = (1_000u64, 0u64, false);
        let (mut prev_state, mut prev_armed, mut off_then_disarmed) = (State::Stop, false, false);
        let mut seen = [0u32; 8];
        let (mut armings, mut wr1_stops, mut silence_disarms) = (0u32, 0u32, 0u32);
        for k in 0..400_000 {
            let mut hs_cleared = false;
            if rng.chance(1) {
                HS_JOINT.disarm();
                hs_cleared = true;
            }
            let dt = match rng.next() % 1000 {
                0 => FLIGHT_GAP_DISARM_US + 1 + rng.next() % 100_000,
                1..=2 => TIMEOUT / 2 + rng.next() % TIMEOUT,
                _ => 1 + rng.next() % 20_000,
            };
            t += dt;
            if quiet == 0 && rng.chance(5) {
                quiet = 1 + rng.next() % 300;
            }
            quiet = quiet.saturating_sub(1);
            let rc = (quiet == 0 && rng.chance(600)).then(|| RcSample {
                sticks: Sticks {
                    roll: 0.0,
                    pitch: 0.0,
                    throttle: [0.0, 0.0, 0.05, 0.5][(rng.next() % 4) as usize],
                    yaw: 0.0,
                },
                arm: !rng.chance(50),
                switch6: [Switch3::Low, Switch3::Middle, Switch3::High][(rng.next() % 3) as usize],
                source: if rng.chance(500) { RcSource::Receiver } else { RcSource::Crsf },
                mission: if i.crsf.is_some_and(|c| c.mission) { !rng.chance(20) } else { rng.chance(100) },
                t_us: t,
            });
            let levels = if rng.chance(2) { None } else { LEVELS };
            if rng.chance(5) {
                i.fs_hold_en = !i.fs_hold_en;
            }
            if rng.chance(50) {
                i.tier = [Tier::T3, Tier::T3, Tier::T3, Tier::T2, Tier::T1][(rng.next() % 5) as usize];
            }
            i.heading_ok = !rng.chance(20);
            i.battery_ok = !rng.chance(20);
            i.battery_critical = rng.chance(3);
            if quiet == 0 && rng.chance(200) {
                i.last_radio_heartbeat_us = Some(t);
            }
            i.command = rng.chance(100).then(|| StationCommand {
                command: [Command::Start, Command::Pause, Command::Abort, Command::Land, Command::Return]
                    [(rng.next() % 5) as usize],
                link: Link::Radio,
            });
            i.home_reached = rng.chance(5);
            i.last_waypoint_reached = rng.chance(5);
            i.now_us = t;
            // N19: CRSF and receiver reports at random (MAVLink flights reach every state); the step hands on only what the
            // gate accepts (the flight's source; disarmed at FS_HOLD_EN 0 no receiver report).
            let rc = rc.filter(|r| gate.accepts(r, i.fs_hold_en));
            if rc.is_some() {
                i.crsf = rc;
            }
            i.armed = gate.armed();
            i.airborne = airborne;

            let out = h.step(&i);
            if out.disarm {
                gate.disarm();
            }
            let cmd = step_flags(&mut gate, t, rc, levels, out.hands_off, out.arm_off_holds);
            let armed = gate.armed();

            // (a) armed at the end of two consecutive steps: the second is not STOP.
            assert!(!(prev_armed && armed && out.state == State::Stop), "a, step {k}: {out:?}");
            // (b) only the switch cycle arms.
            if !prev_armed && armed {
                let r = rc.expect("b: arming needs a report");
                assert!(r.arm && r.sticks.throttle <= tuning.arm_thr_max && levels.is_some(), "b, step {k}");
                assert!(r.source == RcSource::Crsf || (!r.mission && i.fs_hold_en), "b, step {k}: N19 receiver arming");
                assert!(off_then_disarmed, "b, step {k}: arming without a switch cycle");
                armings += 1;
            }
            // (c) outside hands-off states (always, with FS_HOLD_EN not in force) silence disarms exactly as the old
            // gate: at the first step the newest report is older than the timeout, and not before.
            if prev_armed && !out.hands_off && rc.is_none() {
                let age = i.crsf.map_or(u64::MAX, |r| t.saturating_sub(r.t_us));
                if age > TIMEOUT {
                    assert!(!armed, "c, step {k}: silence did not disarm");
                    silence_disarms += 1;
                } else if dt <= FLIGHT_GAP_DISARM_US && levels.is_some() && !out.disarm && !hs_cleared {
                    assert!(armed, "c, step {k}: disarmed before the timeout");
                }
            }
            // (d) a hands-off state that reads the latch false: disarmed at the end of that step.
            if !matches!(prev_state, State::Pilot | State::Stop) && !i.airborne {
                assert!(!armed, "d, step {k}: {prev_state:?} grounded, still armed");
                wr1_stops += u32::from(prev_armed);
            }
            assert_eq!(cmd.throttles().is_some(), armed && levels.is_some());

            if let Some(slot) = seen.get_mut(usize::from(out.state.code() - 1)) {
                *slot += 1;
            }
            if let Some(r) = rc {
                off_then_disarmed = !r.arm && !armed;
            }
            airborne = if !armed {
                false
            } else if airborne {
                !rng.chance(3)
            } else {
                rng.chance(100)
            };
            prev_state = out.state;
            prev_armed = armed;
        }
        assert!(seen.iter().all(|&n| n > 0), "every state reached: {seen:?}");
        assert!(armings > 100 && wr1_stops > 20 && silence_disarms > 20, "{armings} {wr1_stops} {silence_disarms}");
    }
}
