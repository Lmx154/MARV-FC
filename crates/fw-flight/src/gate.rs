//! The gate, the last stage of every flight step, and the command it makes
//! (decision-sim-seams-20261119, section 6). Never armed, always Stop until the arming rules come
//! (decision-first-pair-20261119, section 5).

use crate::gnc::{MotorLevels, NavState};
use crate::tuning::Tuning;
use fw_core::esc::LINES;
use fw_core::handshake::Handshake;
use fw_core::rc::RcSample;

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
    // Read by the arming rules (next packet, decision-first-pair-20261119, section 5).
    #[allow(dead_code)]
    handshake: &'static Handshake,
    #[allow(dead_code)]
    rc_fs_timeout_us: u64,
    #[allow(dead_code)]
    arm_thr_max: f32,
    #[allow(dead_code)]
    angle_max_rad: f32,
}

impl Gate {
    /// Disarmed. `handshake` is where the gate stores the armed state for core 0.
    pub(crate) fn new(tuning: &Tuning, handshake: &'static Handshake) -> Self {
        Self {
            armed: false,
            handshake,
            rc_fs_timeout_us: tuning.rc_fs_timeout_us,
            arm_thr_max: tuning.arm_thr_max,
            angle_max_rad: tuning.angle_max_rad,
        }
    }

    /// The gate's armed state. Nothing arms it yet (the arming rules are the next packet).
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// The last stage of every step. `rc` is this step's new stick report. RC loss is the age of the newest report at
    /// `t_us`, compared with `RC_LOSS_US` (value: owner/step 5). `levels` is `None` when the controller refused.
    /// Always [`MotorCommand::STOP`] until the arming rules come (next packet).
    pub fn apply(
        &mut self,
        _t_us: u64,
        _rc: Option<&RcSample>,
        _nav: &NavState,
        _levels: Option<&MotorLevels>,
    ) -> MotorCommand {
        MotorCommand::STOP
    }
}
