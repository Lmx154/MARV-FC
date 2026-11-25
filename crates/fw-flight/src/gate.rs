//! The gate, the last stage of every flight step, and the command it makes arming, disarming and the RC-loss rule of
//! section 5, steps 1 to 7.

use crate::gnc::{MotorLevels, NavState};
use crate::quat::down_in_body;
use crate::tuning::Tuning;
use fw_core::calib::GRAVITY_MSS;
use fw_core::esc::LINES;
use fw_core::handshake::Handshake;
use fw_core::rc::{RcSample, RcSource};
use marv_card::{IDLE_LEVEL, LEVEL_MAX};

/// The gate disarms and the ESC task clears `ARMED` when the gap between flight steps exceeds this, us. 100 ms is the
/// IMU loop's first recovery back-off (`fw_core::health`, `recovery_backoff_ms: [100, 300, 1000, 3000]`)
pub const FLIGHT_GAP_DISARM_US: u64 = 100_000;

/// The in-air restart (b): a switch-on edge restarts only this long after the in-air arm-switch STOP, us.
pub const RESTART_WINDOW_US: u64 = 5_000_000;

/// The in-air restart (c), (d): IMU1's specific force below this, g, is free fall.
pub const FREEFALL_G: f32 = 0.25;

/// The in-air restart (c): samples this long after the STOP neither set nor clear the free-fall latch, us: the prop
/// spin-down transient, 107 ms measured in log 346 (IMU1 >= 0.25 g up to 621.2750 after the STOP at 621.1680)
pub const FALL_SETTLE_US: u64 = 150_000;

/// The board test feature `restart-bench` (props off): every arm-switch STOP starts the restart window and the free-
/// fall latch reads true, so an arm-ON within `RESTART_WINDOW_US` restarts on the bench.
const BENCH: bool = cfg!(feature = "restart-bench");

/// Why a switch-on edge was refused: the first check that failed, in this order (`ARM no <token>`)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// The throttle above `ARM_THR_MAX` (skipped by the restart)
    Thr,
    /// Tilt not valid.
    Tilt,
    /// Rates not valid.
    Rates,
    /// Leaning more than `ANGLE_MAX` (skipped by the restart)
    Lean,
    /// No motor levels (the controller refused)
    Ctl,
    /// A receiver report with the mission switch ON.
    Ms,
    /// Core 0's `try_arm` refused (a save running, or the permit clear)
    Busy,
}

impl Refusal {
    /// The token of the MSG `ARM no <token>`.
    pub fn token(self) -> &'static str {
        match self {
            Refusal::Thr => "thr",
            Refusal::Tilt => "tilt",
            Refusal::Rates => "rates",
            Refusal::Lean => "lean",
            Refusal::Ctl => "ctl",
            Refusal::Ms => "ms",
            Refusal::Busy => "busy",
        }
    }
}

/// What a switch-on edge did, at its step (logged only)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArmNote {
    /// Refused: the first check that failed.
    Refused(Refusal),
    /// Armed with the restart latch true (a restart arming, whatever the throttle and lean)
    Restart {
        /// Since the STOP, ms.
        t_ms: u32,
        /// IMU1's |f| of the edge step x100, m/s^2.
        f_x100: u32,
        /// The edge's throttle x100.
        thr_x100: u32,
    },
}

/// Why the free-fall latch ended (`FALL off <token>`)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FallOff {
    /// An arm-switch STOP taken with the state machine's ground predicate true: no restart window.
    Ground,
    /// `RESTART_WINDOW_US` passed with the latch set.
    Late,
    /// A finite sample at or above `FREEFALL_G` with the latch set.
    Impact,
    /// A non-finite sample with the latch set.
    NotFinite,
}

impl FallOff {
    /// The token of the MSG `FALL off <token>`.
    pub fn token(self) -> &'static str {
        match self {
            FallOff::Ground => "ground",
            FallOff::Late => "late",
            FallOff::Impact => "impact",
            FallOff::NotFinite => "f",
        }
    }
}

/// A change of the free-fall latch, at its step (logged only)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FallNote {
    /// Set.
    On,
    /// Ended, and why.
    Off(FallOff),
}

/// The free-fall latch (c) after an in-air arm-switch STOP.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Fall {
    /// No sample below `FREEFALL_G` yet since the STOP (the props still spinning down)
    Wait,
    /// Set.
    On,
    /// Cleared for good until the next in-air STOP.
    Off,
}

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

    /// Tests only: a spinning command with these levels by DShot line (each 1..=2000)
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
    /// The flight's source: the source of the report the gate armed on, until disarm.
    source: Option<RcSource>,
    /// Where the armed state is stored for core 0; arming also needs its `try_arm` (no save running)
    handshake: &'static Handshake,
    rc_fs_timeout_us: u64,
    arm_thr_max: f32,
    /// cos(ANGLE_MAX): the lowest third component of the estimated down vector that may arm.
    cos_angle_max: f32,
    /// The in-air restart (a): the step time of the arm-switch STOP taken with the ground predicate false, when it was
    /// the last disarm; `None` otherwise (and while armed)
    stop_us: Option<u64>,
    /// the free-fall latch since `stop_us`.
    fall: Fall,
    /// the step time of the latest `fall_step` and IMU1's |f| then, m/s^2 (not a number when not finite)
    f_at: Option<(u64, f32)>,
    /// The switch-on edge of the latest step that had one, and that step's time.
    arm_note: Option<(u64, ArmNote)>,
    /// The latch change of the latest step that had one, and that step's time.
    fall_note: Option<(u64, FallNote)>,
    /// Tests only: `FALL_SETTLE_US` taken as 0 (the control of the settle time)
    #[cfg(test)]
    pub(crate) settle_off: bool,
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
            stop_us: None,
            fall: Fall::Wait,
            f_at: None,
            arm_note: None,
            fall_note: None,
            #[cfg(test)]
            settle_off: false,
        }
    }

    /// The gate's armed state.
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Armed, only a report of the flight's source counts; disarmed, every report. `Flight::step` hands no other
    /// report to the shaper, the state machine or the gate.
    pub(crate) fn accepts(&self, r: &RcSample) -> bool {
        match self.source {
            Some(s) if self.armed => r.source == s,
            _ => true,
        }
    }

    /// The last stage of every step. `rc` is this step's new stick report.
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
        // 4. A new report: the switch off disarms at once; on, it arms only from a switch cycle with every check met
        // (the in-air restart's latch waives the throttle and lean checks, )
        if let Some(r) = rc {
            if !r.arm {
                if !(self.armed && arm_off_holds) {
                    self.disarm();
                    self.seen_off = true;
                }
            } else if !self.armed {
                // The in-air restart: with the latch true the edge skips only the throttle and lean checks; any arming
                // with the latch true is a restart arming.
                let thr_ok = r.sticks.throttle <= self.arm_thr_max;
                let lean_ok = down_in_body(nav.attitude)[2] >= self.cos_angle_max;
                let restart = self.restart_ok(t_us);
                let refused = if !self.seen_off {
                    None
                } else if !restart && !thr_ok {
                    Some(Refusal::Thr)
                } else if !nav.valid.tilt {
                    Some(Refusal::Tilt)
                } else if !nav.valid.rates {
                    Some(Refusal::Rates)
                } else if !restart && !lean_ok {
                    Some(Refusal::Lean)
                } else if levels.is_none() {
                    Some(Refusal::Ctl)
                } else if r.source == RcSource::Receiver && r.mission {
                    Some(Refusal::Ms)
                } else if !self.handshake.try_arm() {
                    Some(Refusal::Busy)
                } else {
                    self.armed = true;
                    self.source = Some(r.source);
                    if restart {
                        let x100 = |v: f32| (v * 100.0 + 0.5) as u32;
                        let t_ms = self.stop_us.map_or(0, |s| t_us.saturating_sub(s) / 1000);
                        let f = self.f_at.map_or(0.0, |(_, f)| f);
                        self.arm_note = Some((
                            t_us,
                            ArmNote::Restart {
                                t_ms: t_ms.min(u64::from(u32::MAX)) as u32,
                                f_x100: x100(f),
                                thr_x100: x100(r.sticks.throttle),
                            },
                        ));
                    }
                    self.stop_us = None;
                    None
                };
                if let Some(why) = refused {
                    self.arm_note = Some((t_us, ArmNote::Refused(why)));
                }
                if !self.armed {
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

    /// Disarmed, and the switch must be cycled. Clears the handshake's `ARMED` only when leaving armed, so a flag the
    /// gate did not set (feature `fake-armed`) is left alone. `Flight::step` calls it before `apply` when the hands-
    pub(crate) fn disarm(&mut self) {
        if self.armed {
            self.handshake.disarm();
            self.stop_us = None;
        }
        self.armed = false;
        self.seen_off = false;
        self.source = None;
    }

    /// The hands-off state machine entered STOP from an armed step: `disarm`, and with `arm_stop_ground` `Some(false)`
    /// .
    pub(crate) fn stop(&mut self, t_us: u64, arm_stop_ground: Option<bool>) {
        let was = self.armed;
        self.disarm();
        match arm_stop_ground {
            Some(ground) if was && (!ground || BENCH) => {
                self.stop_us = Some(t_us);
                self.fall = Fall::Wait;
            }
            Some(_) if was => self.fall_note = Some((t_us, FallNote::Off(FallOff::Ground))),
            _ => {}
        }
    }

    /// The in-air restart's free-fall latch (c), on a disarmed step more than `FALL_SETTLE_US` after the STOP, before
    /// `apply`: set by the first sample with IMU1's specific force `f_ms2` below `FREEFALL_G`.
    pub(crate) fn fall_step(&mut self, t_us: u64, f_ms2: [f32; 3]) {
        let [x, y, z] = f_ms2;
        let f = libm::sqrtf(x * x + y * y + z * z);
        self.f_at = Some((t_us, f));
        #[cfg(test)]
        let settle = if self.settle_off { 0 } else { FALL_SETTLE_US };
        #[cfg(not(test))]
        let settle = FALL_SETTLE_US;
        let Some(stop) = self.stop_us.filter(|&s| !self.armed && t_us.saturating_sub(s) > settle && t_us > s) else {
            return;
        };
        let low = f < FREEFALL_G * GRAVITY_MSS;
        let off = match self.fall {
            Fall::On if t_us.saturating_sub(stop) > RESTART_WINDOW_US => Some(FallOff::Late),
            Fall::On if !f.is_finite() => Some(FallOff::NotFinite),
            Fall::On if !low => Some(FallOff::Impact),
            Fall::Wait if !f.is_finite() => {
                self.fall = Fall::Off;
                None
            }
            Fall::Wait if low => {
                self.fall = Fall::On;
                self.fall_note = Some((t_us, FallNote::On));
                None
            }
            _ => None,
        };
        if let Some(why) = off {
            self.fall = Fall::Off;
            self.fall_note = Some((t_us, FallNote::Off(why)));
        }
    }

    /// The in-air restart latch at `t_us`: (a) the last disarm was an in-air arm-switch STOP, (b) at most
    /// `RESTART_WINDOW_US` ago, (c) the free-fall latch set, (d) this step's sample below `FREEFALL_G`.
    fn restart_ok(&self, t_us: u64) -> bool {
        let low_now = self.f_at.is_some_and(|(t, f)| t == t_us && f < FREEFALL_G * GRAVITY_MSS);
        self.stop_us.is_some_and(|s| t_us.saturating_sub(s) <= RESTART_WINDOW_US)
            && ((self.fall == Fall::On && low_now) || BENCH)
    }

    /// What the switch-on edge of this step did (the step of the latest `apply`), if it had one.
    pub fn arm_note(&self) -> Option<ArmNote> {
        self.arm_note.filter(|&(t, _)| Some(t) == self.last_step_us).map(|(_, n)| n)
    }

    /// The free-fall latch's change at this step (the step of the latest `apply`), if it had one.
    pub fn fall_note(&self) -> Option<FallNote> {
        self.fall_note.filter(|&(t, _)| Some(t) == self.last_step_us).map(|(_, n)| n)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

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

        assert!(HANDSHAKE.begin_save());
        let cmd = cycle_on(&mut gate, &mut t);
        assert_stopped(&gate, cmd, "try_arm refused");
        HANDSHAKE.end_save();

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

        HANDSHAKE.disarm();
        t += 1_000;
        let cmd = step(&mut gate, t, None, levels);
        assert_stopped(&gate, cmd, "ARMED cleared between steps");
        t += 1_000;
        let cmd = step(&mut gate, t, Some(report(t, true, 0.0)), levels);
        assert_stopped(&gate, cmd, "ARMED cleared, switch still on");
        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());

        HANDSHAKE.set_permit(false);
        let cmd = cycle_on(&mut gate, &mut t);
        assert_stopped(&gate, cmd, "permit clear");
        HANDSHAKE.set_permit(true);
        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());

        HANDSHAKE.set_permit(false);
        let cmd = cycle_on(&mut gate, &mut t);
        assert_stopped(&gate, cmd, "permit clear, switch on");
        HANDSHAKE.set_permit(true);
        t += 1_000;
        let cmd = step(&mut gate, t, Some(report(t, true, 0.0)), levels);
        assert_stopped(&gate, cmd, "permit set while the switch is on");
        assert!(cycle_on(&mut gate, &mut t).throttles().is_some());

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
    /// the step it does today (the timeout plus one microsecond)
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
        let mut t = t;
        for _ in 0..600 {
            t += FLIGHT_GAP_DISARM_US;
            assert!(step_flags(&mut on, t, None, LEVELS, true, false).throttles().is_some());
        }
        t += 1_000;
        assert!(step_flags(&mut on, t, Some(report(t, true, 0.5)), LEVELS, true, false).throttles().is_some());
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

    /// With `arm_off_holds` a report with the switch off leaves `armed` and `seen_off` alone; disarmed, the flag is
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
        t += 1_000;
        let cmd = step_flags(&mut gate, t, Some(report(t, false, 0.0)), LEVELS, true, false);
        assert_stopped_on(&gate, &HS_HOLD, cmd, "flag false");
        assert!(gate.seen_off, "as today: the switch-off report counts toward the cycle");
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_HOLD);
        for _ in 0..50 {
            t += 1_000;
            let cmd = step_flags(&mut gate, t, Some(report(t, true, 0.0)), LEVELS, true, true);
            assert_stopped_on(&gate, &HS_HOLD, cmd, "no cycle");
        }
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

    /// The in-air restart's facts as the property walks model them, written from the rule, not from the gate: (a) the
    /// last disarm was an in-air arm-switch STOP, (b) at most `RESTART_WINDOW_US` ago.
    struct LatchModel {
        stop: Option<u64>,
        /// 0: no low sample yet, 1: set, 2: cleared for good.
        fall: u8,
    }

    impl LatchModel {
        const NEW: LatchModel = LatchModel { stop: None, fall: 0 };

        fn low(f: f32) -> bool {
            f.is_finite() && f < 0.25 * GRAVITY_MSS
        }

        /// The gate left armed this step: by an arm-switch STOP in the air (`in_air`) or anything else.
        fn disarmed(&mut self, t: u64, in_air: bool) {
            self.stop = in_air.then_some(t);
            self.fall = 0;
        }

        /// A disarmed step's sample.
        fn sample(&mut self, t: u64, f: f32) {
            let Some(s) = self.stop.filter(|&s| t > s + 150_000) else { return };
            self.fall = match self.fall {
                0 if !f.is_finite() => 2,
                0 if Self::low(f) => 1,
                1 if t - s > RESTART_WINDOW_US || !Self::low(f) => 2,
                other => other,
            };
        }

        fn latch(&self, t: u64, f: f32) -> bool {
            self.stop.is_some_and(|s| t - s <= RESTART_WINDOW_US) && self.fall == 1 && Self::low(f)
        }
    }

    /// The in-air facts of one walk step: IMU1's specific force.
    /// with impacts and non-finite samples; about 1 g otherwise) and an attitude (level, or leaning 40 deg.
    struct AirFacts {
        f_ms2: [f32; 3],
        attitude: [f32; 4],
    }

    fn random_air(rng: &mut Rng, falling: bool) -> AirFacts {
        let unit = |rng: &mut Rng| (rng.next() % 10_001) as f32 / 10_000.0;
        let z = match rng.next() % 1000 {
            0..=2 => f32::NAN,
            3..=12 => -(0.25 * GRAVITY_MSS) - 3.0 * unit(rng),
            _ if falling => -(0.24 * GRAVITY_MSS) * unit(rng),
            _ => -GRAVITY_MSS * (0.8 + 0.4 * unit(rng)),
        };
        let (s, c) = (libm::sinf(0.349_066), libm::cosf(0.349_066));
        AirFacts {
            f_ms2: [0.1 * unit(rng), 0.0, z],
            attitude: if rng.chance(100) { [c, s, 0.0, 0.0] } else { [1.0, 0.0, 0.0, 0.0] },
        }
    }

    /// I1 / C1 a5 / I-arm': over random reports, gaps, levels, both flags, arm-switch STOPs in the air and on the
    /// ground, specific forces and attitudes.
    #[test]
    fn gate_walk_flags_never_arm_outside_the_switch_cycle() {
        static HS_WALK: Handshake = Handshake::new();
        HS_WALK.set_permit(true);
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_WALK);
        let mut rng = Rng(0x5eed_00a5);
        let (mut t, mut quiet) = (1_000, 0);
        let mut off_then_disarmed = false;
        let (mut armings, mut restarts, mut held_loss, mut held_off) = (0u32, 0u32, 0u32, 0u32);
        let (mut model, mut falling, mut hold_off_until) = (LatchModel::NEW, false, 0u64);
        for _ in 0..400_000 {
            if rng.chance(2) {
                HS_WALK.disarm();
            }
            let mut f = random_facts(&mut rng, &mut t, &mut quiet);
            if f.t < hold_off_until {
                f.rc = f.rc.map(|r| RcSample { arm: false, ..r });
            }
            let air = random_air(&mut rng, falling);
            let was = gate.armed();
            let lost = gate.last_rc_us.is_some_and(|l| f.t.saturating_sub(l) > TIMEOUT) && f.rc.is_none();
            let stop = was && f.rc.is_some_and(|r| !r.arm) && !f.arm_off_holds;
            if stop {
                let ground = rng.chance(300);
                gate.stop(f.t, Some(ground));
                model.disarmed(f.t, !ground);
                falling = !ground && !rng.chance(200);
                if rng.chance(20) {
                    hold_off_until = f.t + 4_000_000 + rng.next() % 2_000_000;
                }
            }
            gate.fall_step(f.t, air.f_ms2);
            let [x, y, z] = air.f_ms2;
            let norm = libm::sqrtf(x * x + y * y + z * z);
            if !gate.armed() {
                model.sample(f.t, norm);
            }
            let latch = model.latch(f.t, norm);
            let normal = f.rc.is_some_and(|r| {
                r.sticks.throttle <= Tuning::DEFAULT.arm_thr_max
                    && down_in_body(air.attitude)[2] >= libm::cosf(Tuning::DEFAULT.angle_max_rad)
            });
            let gap = gate.last_step_us.is_some_and(|l| f.t - l > FLIGHT_GAP_DISARM_US);
            let edge = !gate.armed() && gate.seen_off && !gap && f.rc.is_some_and(|r| r.arm);
            let nav = NavState { attitude: air.attitude, ..level_nav(f.t) };
            let levels = f.levels.map(|level| MotorLevels { level });
            let cmd = gate.apply(f.t, f.rc.as_ref(), &nav, levels.as_ref(), f.hands_off, f.arm_off_holds);
            assert!(!gate.armed() || HS_WALK.armed());
            assert_eq!(cmd.throttles().is_some(), gate.armed() && f.levels.is_some());
            if !was && gate.armed() {
                let r = f.rc.expect("arming needs a report");
                assert!(edge, "arming without an edge");
                assert!(r.arm && f.levels.is_some() && (normal || latch), "I-arm'");
                assert!(off_then_disarmed, "arming without a switch cycle");
                armings += 1;
                restarts += u32::from(latch);
                assert_eq!(gate.arm_note().is_some(), latch, "ARM fall exactly on a restart arming");
                model = LatchModel::NEW;
                falling = false;
            } else if edge {
                assert!(!((normal || latch) && f.levels.is_some()), "every check met, refused");
                assert!(!gate.seen_off, "a refused edge clears the cycle");
                assert!(matches!(gate.arm_note(), Some(ArmNote::Refused(_))), "ARM no on every refused edge");
            } else {
                assert!(gate.arm_note().is_none(), "a note without an edge");
            }
            if was && !gate.armed() && !stop {
                model.disarmed(f.t, false);
                falling = false;
            }
            if was && gate.armed() {
                held_loss += u32::from(lost && f.hands_off);
                held_off += u32::from(f.rc.is_some_and(|r| !r.arm) && f.arm_off_holds);
            }
            if let Some(r) = f.rc {
                off_then_disarmed = !r.arm && !gate.armed();
            }
        }
        let normal = armings - restarts;
        std::println!("restart armings {restarts}, normal {normal}, held loss {held_loss}, held off {held_off}");
        assert!(restarts > 100 && normal > 100 && held_loss > 100 && held_off > 100, "{restarts} {normal} {held_loss} {held_off}");
    }

    /// The gate as it was at 047aff0, verbatim but for the handshake argument: the reference for (false, false)
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

    /// With (false, false) the gate is bit-identical to the gate before the two inputs, step by step, over a seeded
    /// walk of reports, gaps, levels, permit, saves and `ARMED` cleared from outside.
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

    /// A receiver report with the mission switch ON never arms (switch cycle, low throttle and every check met), with
    /// it OFF it does; armed, only the flight's source counts.
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
        assert!(gate.accepts(&report(t, true, 0.0)) && gate.accepts(&rx(t, true, false)), "disarmed: both sources");
        t += 1_000;
        step(&mut gate, t, Some(rx(t, false, false)), levels);
        t += 1_000;
        let c = step(&mut gate, t, Some(rx(t, true, false)), levels);
        assert!(gate.armed() && c.throttles().is_some(), "mission switch OFF arms");
        assert!(gate.accepts(&rx(t, true, true)), "the flight's source");
        assert!(!gate.accepts(&report(t, true, 0.0)), "a CRSF report on a MAVLink flight");
        t += 1_000;
        step(&mut gate, t, Some(rx(t, false, false)), levels);
        assert!(!gate.armed());
        t += 1_000;
        step(&mut gate, t, Some(report(t, false, 0.0)), levels);
        t += 1_000;
        step(&mut gate, t, Some(report(t, true, 0.0)), levels);
        assert!(gate.armed(), "CRSF arms");
        assert!(!gate.accepts(&rx(t, true, false)), "a receiver report on a CRSF flight");
        assert!(gate.accepts(&report(t, true, 0.0)));
    }

    /// The real gate driven by the real hands-off machine.
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
            restart: false,
        };
        let (mut t, mut quiet, mut airborne) = (1_000u64, 0u64, false);
        let (mut prev_state, mut prev_armed, mut off_then_disarmed) = (State::Stop, false, false);
        let mut seen = [0u32; 8];
        let (mut armings, mut wr1_stops, mut silence_disarms) = (0u32, 0u32, 0u32);
        let (mut model, mut falling, mut restarts, mut restarted) = (LatchModel::NEW, false, 0u32, false);
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
            let rc = rc.filter(|r| gate.accepts(r));
            if rc.is_some() {
                i.crsf = rc;
            }
            i.armed = gate.armed();
            i.airborne = airborne;
            i.restart = restarted;

            let out = h.step(&i);
            if out.disarm {
                gate.stop(t, out.arm_stop_ground);
                model.disarmed(t, out.arm_stop_ground == Some(false));
                falling = out.arm_stop_ground == Some(false) && !rng.chance(200);
            }
            let air = random_air(&mut rng, falling);
            gate.fall_step(t, air.f_ms2);
            let [x, y, z] = air.f_ms2;
            let norm = libm::sqrtf(x * x + y * y + z * z);
            if !gate.armed() {
                model.sample(t, norm);
            }
            let latch = model.latch(t, norm);
            let cmd = step_flags(&mut gate, t, rc, levels, out.hands_off, out.arm_off_holds);
            let armed = gate.armed();
            if prev_armed && !armed && !out.disarm {
                model.disarmed(t, false);
                falling = false;
            }

            assert!(!(prev_armed && armed && out.state == State::Stop), "a, step {k}: {out:?}");
            if !prev_armed && armed {
                let r = rc.expect("b: arming needs a report");
                let normal = r.sticks.throttle <= tuning.arm_thr_max;
                assert!(r.arm && (normal || latch) && levels.is_some(), "b, step {k}");
                assert!(r.source == RcSource::Crsf || !r.mission, "b, step {k}: N19 receiver arming");
                assert!(off_then_disarmed, "b, step {k}: arming without a switch cycle");
                armings += 1;
                restarts += u32::from(latch);
                restarted = latch;
                model = LatchModel::NEW;
                falling = false;
            }
            if prev_armed && !out.hands_off && rc.is_none() {
                let age = i.crsf.map_or(u64::MAX, |r| t.saturating_sub(r.t_us));
                if age > TIMEOUT {
                    assert!(!armed, "c, step {k}: silence did not disarm");
                    silence_disarms += 1;
                } else if dt <= FLIGHT_GAP_DISARM_US && levels.is_some() && !out.disarm && !hs_cleared {
                    assert!(armed, "c, step {k}: disarmed before the timeout");
                }
            }
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
            restarted &= armed;
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
        let normal = armings - restarts;
        std::println!("restart armings {restarts}, normal {normal}, WR1 stops {wr1_stops}, silence disarms {silence_disarms}");
        assert!(
            restarts > 100 && normal > 100 && wr1_stops > 20 && silence_disarms > 20,
            "{restarts} {normal} {wr1_stops} {silence_disarms}"
        );
    }

    /// One gate step of the restart tests: the free-fall sample `f_z` (m/s^2, body z), then `apply` with level nav.
    /// `nav`), levels present.
    fn fall_apply(gate: &mut Gate, t: u64, rc: Option<RcSample>, f_z: f32, nav: Option<NavState>) -> MotorCommand {
        gate.fall_step(t, [0.0, 0.0, f_z]);
        let nav = nav.unwrap_or_else(|| level_nav(t));
        gate.apply(t, rc.as_ref(), &nav, Some(&MotorLevels { level: [600; 4] }), false, false)
    }

    /// Armed at hover with a switch cycle; returns the time.
    fn armed_at_hover(gate: &mut Gate, t: &mut u64) {
        for on in [false, true] {
            *t += 1_000;
            fall_apply(gate, *t, Some(report(*t, on, 0.0)), -GRAVITY_MSS, None);
        }
        assert!(gate.armed());
        for _ in 0..100 {
            *t += 1_000;
            fall_apply(gate, *t, Some(report(*t, true, 0.6)), -GRAVITY_MSS, None);
        }
    }

    /// Free fall (|f| 0.1 g) with the switch off for 600 ms, then the switch on with the throttle at 0.74: whether it
    /// armed, and the edge's note.
    fn fall_then_on(gate: &mut Gate, t: &mut u64) -> (bool, Option<ArmNote>) {
        for k in 0..600 {
            *t += 1_000;
            let rc = (k % 10 == 0).then(|| report(*t, false, 0.74));
            fall_apply(gate, *t, rc, -0.1 * GRAVITY_MSS, None);
        }
        *t += 1_000;
        let cmd = fall_apply(gate, *t, Some(report(*t, true, 0.74)), -0.1 * GRAVITY_MSS, None);
        assert_eq!(cmd.throttles().is_some(), gate.armed());
        (gate.armed(), gate.arm_note())
    }

    /// The in-air restart at the gate: an in-air arm-switch STOP then free fall arms at throttle 0.74.
    /// twin); a disarm by RC loss, by a flight gap or by the controller's refusal (levels None)
    #[test]
    fn restart_only_after_an_in_air_arm_switch_stop() {
        static HS_R: Handshake = Handshake::new();
        HS_R.set_permit(true);
        let thr = Some(ArmNote::Refused(Refusal::Thr));
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_R);
        let mut t = 1_000;
        armed_at_hover(&mut gate, &mut t);
        t += 1_000;
        gate.stop(t, Some(false));
        fall_apply(&mut gate, t, Some(report(t, false, 0.6)), -GRAVITY_MSS, None);
        assert!(!gate.armed() && !HS_R.armed() && gate.fall_note().is_none());
        t += 1_000;
        fall_apply(&mut gate, t, None, -0.1 * GRAVITY_MSS, None);
        assert_eq!(gate.fall_note(), None, "within FALL_SETTLE_US: neither set nor cleared");
        let (armed, note) = fall_then_on(&mut gate, &mut t);
        assert!(armed && HS_R.armed(), "in-air STOP: restart");
        assert!(matches!(note, Some(ArmNote::Restart { t_ms: 602, f_x100: 98, thr_x100: 74 })), "{note:?}");
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_R);
        armed_at_hover(&mut gate, &mut t);
        for _ in 0..=TIMEOUT / 50_000 {
            t += 50_000;
            fall_apply(&mut gate, t, None, -0.1 * GRAVITY_MSS, None);
        }
        assert!(!gate.armed());
        assert_eq!(fall_then_on(&mut gate, &mut t), (false, thr), "RC loss");
        assert!(!gate.seen_off);
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_R);
        armed_at_hover(&mut gate, &mut t);
        t += FLIGHT_GAP_DISARM_US + 1;
        fall_apply(&mut gate, t, Some(report(t, true, 0.6)), -0.1 * GRAVITY_MSS, None);
        assert!(!gate.armed());
        assert_eq!(fall_then_on(&mut gate, &mut t), (false, thr), "flight gap");
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_R);
        armed_at_hover(&mut gate, &mut t);
        t += 1_000;
        gate.fall_step(t, [0.0, 0.0, -0.1 * GRAVITY_MSS]);
        gate.apply(t, Some(&report(t, true, 0.6)), &level_nav(t), None, false, false);
        assert!(!gate.armed());
        assert_eq!(fall_then_on(&mut gate, &mut t), (false, thr), "levels None");
        for by_machine in [false, true] {
            let mut gate = Gate::new(&Tuning::DEFAULT, &HS_R);
            armed_at_hover(&mut gate, &mut t);
            t += 1_000;
            if by_machine {
                gate.stop(t, None);
            }
            fall_apply(&mut gate, t, Some(report(t, false, 0.6)), -GRAVITY_MSS, None);
            assert!(!gate.armed());
            assert_eq!(fall_then_on(&mut gate, &mut t), (false, thr), "not an arm-switch STOP, {by_machine}");
        }
    }

    /// One "ARM no" per refused edge, the first failing check named; repeated switch-on reports after it are not
    /// edges.
    #[test]
    fn arm_no_once_per_refused_edge_naming_the_first_failed_check() {
        static HS_N: Handshake = Handshake::new();
        HS_N.set_permit(true);
        let mut gate = Gate::new(&Tuning::DEFAULT, &HS_N);
        let mut t = 1_000;
        let level = |t| level_nav(t);
        let (s, c) = (libm::sinf(0.6), libm::cosf(0.6));
        let lean = |t| NavState { attitude: [c, s, 0.0, 0.0], ..level_nav(t) };
        let no_tilt = |t| NavState { valid: Valid { tilt: false, ..level_nav(t).valid }, ..level_nav(t) };
        let no_rates = |t| NavState { valid: Valid { rates: false, ..level_nav(t).valid }, ..level_nav(t) };
        let ms = |t, thr| RcSample { source: RcSource::Receiver, mission: true, ..report(t, true, thr) };
        let cases: [(&str, f32, &dyn Fn(u64) -> NavState, bool, bool, Refusal); 7] = [
            ("thr", 0.5, &level, true, false, Refusal::Thr),
            ("tilt", 0.0, &no_tilt, true, false, Refusal::Tilt),
            ("rates", 0.0, &no_rates, true, false, Refusal::Rates),
            ("lean", 0.0, &lean, true, false, Refusal::Lean),
            ("ctl", 0.0, &level, false, false, Refusal::Ctl),
            ("ms", 0.0, &level, true, true, Refusal::Ms),
            ("busy", 0.0, &level, true, false, Refusal::Busy),
        ];
        for (name, thr, nav, levels, receiver, want) in cases {
            t += 1_000;
            gate.apply(t, Some(&report(t, false, 0.0)), &level(t), Some(&MotorLevels { level: [600; 4] }), false, false);
            if name == "busy" {
                assert!(HS_N.begin_save());
            }
            for k in 0..3 {
                t += 1_000;
                let r = if receiver { ms(t, thr) } else { report(t, true, thr) };
                let lv = levels.then_some(MotorLevels { level: [600; 4] });
                gate.apply(t, Some(&r), &nav(t), lv.as_ref(), false, false);
                assert!(!gate.armed(), "{name}");
                let note = gate.arm_note();
                assert_eq!(note, (k == 0).then_some(ArmNote::Refused(want)), "{name}, report {k}");
                assert_eq!(want.token(), name);
            }
            if name == "busy" {
                HS_N.end_save();
            }
        }
    }
}
