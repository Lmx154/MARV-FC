//! The gate, the last stage of every flight step, and the command it makes
//! (decision-sim-seams-20261119, section 6): arming, disarming and the RC-loss rule of
//! decision-first-pair-20261119, section 5, steps 1 to 7.

use crate::gnc::{MotorLevels, NavState};
use crate::quat::down_in_body;
use crate::tuning::Tuning;
use fw_core::esc::LINES;
use fw_core::handshake::Handshake;
use fw_core::rc::RcSample;
use marv_card::{IDLE_LEVEL, LEVEL_MAX};

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
    /// When the newest stick report was read, us; `None` before the first.
    last_rc_us: Option<u64>,
    /// A fresh report with the switch off was seen since the last disarm or refused arm.
    seen_off: bool,
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
            last_rc_us: None,
            seen_off: false,
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

    /// The last stage of every step. `rc` is this step's new stick report. RC loss is the age of the newest report at
    /// `t_us` above `RC_FS_TIMEOUT`. `levels` is `None` when the controller refused.
    pub fn apply(
        &mut self,
        t_us: u64,
        rc: Option<&RcSample>,
        nav: &NavState,
        levels: Option<&MotorLevels>,
    ) -> MotorCommand {
        // 1 to 3. RC loss: Stop and disarm.
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
        // 4. A new report: the switch off disarms at once; on, it arms only from a switch cycle with every check met.
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
    /// the gate did not set (feature `fake-armed`) is left alone.
    fn disarm(&mut self) {
        if self.armed {
            self.handshake.disarm();
        }
        self.armed = false;
        self.seen_off = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnc::Valid;
    use fw_core::rc::Sticks;

    static HANDSHAKE: Handshake = Handshake::new();
    const TIMEOUT: u64 = Tuning::DEFAULT.rc_fs_timeout_us;

    fn report(t_us: u64, arm: bool, throttle: f32) -> RcSample {
        let sticks = Sticks {
            roll: 0.0,
            pitch: 0.0,
            throttle,
            yaw: 0.0,
        };
        RcSample { sticks, arm, t_us }
    }

    /// Steps the gate with level nav and these levels; returns the command.
    fn step(
        gate: &mut Gate,
        t_us: u64,
        rc: Option<RcSample>,
        levels: Option<[u16; 4]>,
    ) -> MotorCommand {
        let nav = NavState {
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
        };
        let levels = levels.map(|level| MotorLevels { level });
        gate.apply(t_us, rc.as_ref(), &nav, levels.as_ref())
    }

    /// Asserts Stop, disarmed, and the handshake not armed.
    fn assert_stopped(gate: &Gate, cmd: MotorCommand, what: &str) {
        assert_eq!(cmd, MotorCommand::STOP, "{what}");
        assert!(!gate.armed(), "{what}");
        assert!(!HANDSHAKE.armed(), "{what}: handshake");
    }

    #[test]
    fn gate_arming_sequence() {
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
        let last = t;
        assert!(
            step(&mut gate, last + TIMEOUT, None, levels)
                .throttles()
                .is_some()
        );
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
    }
}
