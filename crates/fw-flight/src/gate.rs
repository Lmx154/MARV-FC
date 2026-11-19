//! The gate, the last stage of every flight step, and the command it makes
//! (decision-sim-seams-20261119, section 6). Step 3: never armed, always Stop; the rules come in step 5.

use crate::gnc::{MotorLevels, NavState};
use fw_core::esc::LINES;
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
}

/// Arming and the RC-loss kill. One concrete type, not a trait, owned by `Flight`. The rules come in step 5.
#[derive(Debug)]
pub struct Gate {}

impl Gate {
    /// Step-3 stand-in, replaced in step 5: never armed.
    pub fn armed(&self) -> bool {
        false
    }

    /// The last stage of every step. `rc` is this step's new stick report. RC loss is the age of the newest report at
    /// `t_us`, compared with `RC_LOSS_US` (value: owner/step 5). `levels` is `None` when the controller refused.
    /// Step-3 stand-in, replaced in step 5: always [`MotorCommand::STOP`].
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
