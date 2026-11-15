//! The rule for accepting an action.
//!
//! Two pure parts, no I/O, no clock of its own, no statics:
//!
//! - [`classify`]: a command and its seven parameters become a recognised [`Classified`] action or a
//!   [`Refusal`] (UNSUPPORTED, or DENIED with a short text for a STATUSTEXT).
//! - [`ActionBook`]: the admission order (decision foundation, item 5) and the books of the one long
//!   action that may run at a time, firmware-wide. The firmware passes `armed`, `hil`, the link id and
//!   `now_ms` as plain values, executes the action itself, and reports progress and the result back.
//!
//! The ACKs of a long action: the answer to the command is IN_PROGRESS (0 %); later
//! [`ActionBook::next_ack`] hands out IN_PROGRESS with the percentage at 2 Hz, then the one final ACK,
//! both only for the link the command came from.

use mavlink_device::marv::{MarvDevice, MavCmd, MavResult};
use mavlink_device::{CommandOutcome, CommandRequest, LateAck, Peer};

/// Maximum duration of a calibration, in milliseconds.
pub const CALIBRATION_MAX_MS: u64 = 15_000;
/// Maximum duration of a storage action, in milliseconds.
pub const STORAGE_MAX_MS: u64 = 3_000;
/// Maximum duration of a device action (restart, self-test), in milliseconds.
pub const DEVICE_MAX_MS: u64 = 5_000;
/// Maximum duration of a reboot request, in milliseconds.
pub const REBOOT_MAX_MS: u64 = 3_000;
/// Minimum time between two progress ACKs, in milliseconds (2 Hz).
pub const PROGRESS_PERIOD_MS: u64 = 500;

/// The DENIED text for a command that exists but is not built yet.
const NOT_YET: &str = "not supported yet";

/// A long action the firmware may be asked to run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Action {
    /// Gyroscope calibration (`PREFLIGHT_CALIBRATION` param1 = 1).
    GyroCalibration,
    /// One-position accelerometer calibration (`PREFLIGHT_CALIBRATION` param5 = 4).
    AccelCalibration,
    /// Reload the parameters from flash (`PREFLIGHT_STORAGE` param1 = 0).
    StorageReload,
    /// Save the parameters now (`PREFLIGHT_STORAGE` param1 = 1).
    StorageSave,
    /// All parameters to their defaults (`PREFLIGHT_STORAGE` param1 = 2 or 4).
    StorageDefaultsAll,
    /// The calibration parameters to their defaults (`PREFLIGHT_STORAGE` param1 = 3).
    StorageDefaultsCalibration,
    /// Reboot (`PREFLIGHT_REBOOT_SHUTDOWN` param1 = 1).
    Reboot,
    /// Restart one device (`MAV_CMD_MARV_DEVICE` action 1).
    DeviceRestart(MarvDevice),
    /// Self-test one device (`MAV_CMD_MARV_DEVICE` action 2).
    DeviceSelfTest(MarvDevice),
}

/// The kind of an [`Action`], which decides its maximum duration and the HIL rule.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActionClass {
    /// A sensor calibration: refused in HIL, ends the HEARTBEAT in CALIBRATING.
    Calibration,
    /// A parameter storage action.
    Storage,
    /// A reboot.
    Reboot,
    /// A device restart or self-test.
    Device,
}

impl Action {
    /// The class of the action.
    pub const fn class(self) -> ActionClass {
        match self {
            Self::GyroCalibration | Self::AccelCalibration => ActionClass::Calibration,
            Self::StorageReload
            | Self::StorageSave
            | Self::StorageDefaultsAll
            | Self::StorageDefaultsCalibration => ActionClass::Storage,
            Self::Reboot => ActionClass::Reboot,
            Self::DeviceRestart(_) | Self::DeviceSelfTest(_) => ActionClass::Device,
        }
    }

    /// The time after which a running action ends FAILED by itself, in milliseconds.
    pub const fn max_duration_ms(self) -> u64 {
        match self.class() {
            ActionClass::Calibration => CALIBRATION_MAX_MS,
            ActionClass::Storage => STORAGE_MAX_MS,
            ActionClass::Reboot => REBOOT_MAX_MS,
            ActionClass::Device => DEVICE_MAX_MS,
        }
    }
}

/// What a command means, when the first build recognises it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Classified {
    /// A long action; at most one runs at a time.
    Long(Action),
    /// The all-zero `PREFLIGHT_CALIBRATION`: cancel the running calibration (PX4's convention).
    CancelCalibration,
}

/// Why a command is refused before admission looks at the state of the vehicle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// A command this firmware does not know at all.
    Unsupported,
    /// A known command with parameters it does not accept; the text goes to a STATUSTEXT.
    Denied(&'static str),
}

/// `v` as an integer, only if it is finite and exactly that integer (1.5, NaN, 1e9 select nothing).
fn int(v: f32) -> Option<i32> {
    if !v.is_finite() || v.abs() >= 1.0e9 {
        return None;
    }
    // In range, so the cast does not saturate.
    let n = v as i32;
    (n as f32 == v).then_some(n)
}

/// True when parameter number `idx` (0 based) equals `val` and every other parameter is zero.
fn only(params: &[f32; 7], idx: usize, val: f32) -> bool {
    params
        .iter()
        .enumerate()
        .all(|(i, &v)| if i == idx { v == val } else { v == 0.0 })
}

fn classify_calibration(params: &[f32; 7]) -> Result<Classified, Refusal> {
    if params.iter().all(|&v| v == 0.0) {
        Ok(Classified::CancelCalibration)
    } else if only(params, 0, 1.0) {
        Ok(Classified::Long(Action::GyroCalibration))
    } else if only(params, 4, 4.0) {
        Ok(Classified::Long(Action::AccelCalibration))
    } else {
        Err(Refusal::Denied(NOT_YET))
    }
}

fn classify_storage(params: &[f32; 7]) -> Result<Classified, Refusal> {
    let [p1, ..] = *params;
    match int(p1) {
        Some(0) => Ok(Classified::Long(Action::StorageReload)),
        Some(1) => Ok(Classified::Long(Action::StorageSave)),
        Some(2 | 4) => Ok(Classified::Long(Action::StorageDefaultsAll)),
        Some(3) => Ok(Classified::Long(Action::StorageDefaultsCalibration)),
        _ => Err(Refusal::Denied("unknown storage action")),
    }
}

fn classify_device(params: &[f32; 7]) -> Result<Classified, Refusal> {
    let [p1, p2, ..] = *params;
    let restart = match int(p2) {
        Some(1) => true,
        Some(2) => false,
        _ => return Err(Refusal::Denied("unknown device action")),
    };
    let device = match int(p1) {
        Some(1) => MarvDevice::MARV_DEVICE_IMU1,
        Some(2) => MarvDevice::MARV_DEVICE_IMU2,
        Some(3) => MarvDevice::MARV_DEVICE_BARO,
        Some(4) => MarvDevice::MARV_DEVICE_MAG,
        Some(5) => MarvDevice::MARV_DEVICE_GNSS,
        Some(6) => MarvDevice::MARV_DEVICE_BATT,
        // RC, SD, PSRAM, LED, ESC: devices of the dialect that this build does not serve.
        Some(7 | 9 | 10 | 11 | 12) => return Err(Refusal::Denied(NOT_YET)),
        _ => return Err(Refusal::Denied("unknown device")),
    };
    if restart {
        return Ok(Classified::Long(Action::DeviceRestart(device)));
    }
    match device {
        MarvDevice::MARV_DEVICE_IMU1
        | MarvDevice::MARV_DEVICE_IMU2
        | MarvDevice::MARV_DEVICE_MAG => Ok(Classified::Long(Action::DeviceSelfTest(device))),
        _ => Err(Refusal::Denied(NOT_YET)),
    }
}

/// Classify a command and its seven parameters. Pure: it looks at nothing but its arguments.
///
/// A parameter counts as the integer `n` only if it is finite and exactly `n`.
pub fn classify(command: MavCmd, params: &[f32; 7]) -> Result<Classified, Refusal> {
    match command {
        MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION => classify_calibration(params),
        MavCmd::MAV_CMD_PREFLIGHT_STORAGE => classify_storage(params),
        MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN => {
            if only(params, 0, 1.0) {
                Ok(Classified::Long(Action::Reboot))
            } else {
                Err(Refusal::Denied(NOT_YET))
            }
        }
        MavCmd::MAV_CMD_MARV_DEVICE => classify_device(params),
        _ => Err(Refusal::Unsupported),
    }
}

/// What the firmware must do besides sending the answer of a decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Effect {
    /// Nothing.
    None,
    /// Start this action now (it is the running action in the books).
    Start(Action),
    /// Stop the running calibration; its final ACK (CANCELLED) is already in the books.
    CancelCalibration,
}

/// The decision of [`ActionBook::admit`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Decision {
    /// The answer to the command: what [`Telemetry::command`](mavlink_device::Telemetry::command) returns.
    pub outcome: CommandOutcome,
    /// What the firmware does besides.
    pub effect: Effect,
    /// A short reason for a STATUSTEXT, for a DENIED answer.
    pub text: Option<&'static str>,
}

impl Decision {
    fn plain(result: MavResult) -> Self {
        Self {
            outcome: CommandOutcome::new(result),
            effect: Effect::None,
            text: None,
        }
    }

    fn denied(text: &'static str) -> Self {
        Self {
            text: Some(text),
            ..Self::plain(MavResult::MAV_RESULT_DENIED)
        }
    }
}

impl From<Refusal> for Decision {
    fn from(refusal: Refusal) -> Self {
        match refusal {
            Refusal::Unsupported => Self::plain(MavResult::MAV_RESULT_UNSUPPORTED),
            Refusal::Denied(text) => Self::denied(text),
        }
    }
}

/// The running action and what is still owed for it.
#[derive(Clone, Copy, Debug)]
struct Running {
    action: Action,
    command: MavCmd,
    params: [f32; 7],
    sender: Peer,
    link: u8,
    started_ms: u64,
    percent: u8,
    next_progress_ms: u64,
    /// `Some` once the action has ended: the final ACK is owed until it is handed out.
    result: Option<MavResult>,
    /// The origin link is lost: no ACK is owed, the books are freed when the action ends.
    orphan: bool,
}

/// The books of the one long action that may run at a time, firmware-wide.
///
/// The action stays in the books until its final ACK has been handed out, so a new action cannot
/// start while a final ACK is still owed (I5.3: exactly one final ACK per admitted long action).
#[derive(Clone, Copy, Debug, Default)]
pub struct ActionBook {
    slot: Option<Running>,
}

impl ActionBook {
    /// Empty books: nothing runs.
    pub const fn new() -> Self {
        Self { slot: None }
    }

    /// The action that is running (not yet ended), if any.
    pub fn running(&self) -> Option<Action> {
        self.slot
            .as_ref()
            .filter(|r| r.result.is_none())
            .map(|r| r.action)
    }

    /// End by itself, FAILED, an action past its maximum duration. Returns the action that ended now.
    fn expire(&mut self, now_ms: u64) -> Option<Action> {
        let run = self.slot.as_mut()?;
        let elapsed = now_ms.saturating_sub(run.started_ms);
        let ended = if run.result.is_none() && elapsed >= run.action.max_duration_ms() {
            run.result = Some(MavResult::MAV_RESULT_FAILED);
            Some(run.action)
        } else {
            None
        };
        self.reap();
        ended
    }

    /// Free the books of an ended action whose origin link is lost.
    fn reap(&mut self) {
        if self
            .slot
            .as_ref()
            .is_some_and(|r| r.orphan && r.result.is_some())
        {
            self.slot = None;
        }
    }

    /// Apply the maximum durations at `now_ms`. Returns the action that has just ended FAILED by timeout,
    /// so that the firmware can stop it. Every other method applies the same rule first.
    pub fn tick(&mut self, now_ms: u64) -> Option<Action> {
        self.expire(now_ms)
    }

    /// Decide what happens to a command that arrived on link `link`.
    ///
    /// The order: classification (UNSUPPORTED, DENIED); a retransmission of the running command
    /// (IN_PROGRESS); the cancel; TEMPORARILY_REJECTED if `armed` or another action is in the books;
    /// DENIED for a calibration when `hil`; otherwise the action is admitted and answered IN_PROGRESS 0 %.
    pub fn admit(
        &mut self,
        req: &CommandRequest,
        link: u8,
        armed: bool,
        hil: bool,
        now_ms: u64,
    ) -> Decision {
        self.expire(now_ms);
        let action = match classify(req.command, &req.params) {
            Err(refusal) => return refusal.into(),
            Ok(Classified::CancelCalibration) => return self.cancel(),
            Ok(Classified::Long(action)) => action,
        };
        if let Some(run) = &self.slot {
            let same_params = run
                .params
                .iter()
                .zip(req.params.iter())
                .all(|(a, b)| a.to_bits() == b.to_bits());
            if run.command == req.command && run.sender == req.sender && same_params {
                return Decision {
                    outcome: CommandOutcome::in_progress(run.percent),
                    effect: Effect::None,
                    text: None,
                };
            }
            return Decision::plain(MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        }
        if armed {
            return Decision::plain(MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        }
        if hil && action.class() == ActionClass::Calibration {
            return Decision::denied("calibration not allowed in HIL");
        }
        self.slot = Some(Running {
            action,
            command: req.command,
            params: req.params,
            sender: req.sender,
            link,
            started_ms: now_ms,
            percent: 0,
            next_progress_ms: now_ms.saturating_add(PROGRESS_PERIOD_MS),
            result: None,
            orphan: false,
        });
        Decision {
            outcome: CommandOutcome::in_progress(0),
            effect: Effect::Start(action),
            text: None,
        }
    }

    fn cancel(&mut self) -> Decision {
        match self.slot.as_mut() {
            Some(run) if run.result.is_none() && run.action.class() == ActionClass::Calibration => {
                run.result = Some(MavResult::MAV_RESULT_CANCELLED);
                Decision {
                    outcome: CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED),
                    effect: Effect::CancelCalibration,
                    text: None,
                }
            }
            _ => Decision::denied("nothing to cancel"),
        }
    }

    /// The firmware reports the progress of the running action, in percent (above 100 is cut to 100).
    /// Ignored when nothing runs.
    pub fn set_progress(&mut self, now_ms: u64, percent: u8) {
        self.expire(now_ms);
        if let Some(run) = self.slot.as_mut().filter(|r| r.result.is_none()) {
            run.percent = percent.min(100);
        }
    }

    /// The firmware reports the end of the running action with its final result. Returns false (and
    /// changes nothing) when nothing runs, when the action has already ended (including by timeout at
    /// or before `now_ms`), or when `result` is IN_PROGRESS, which is not a final result.
    pub fn finish(&mut self, now_ms: u64, result: MavResult) -> bool {
        self.expire(now_ms);
        match self.slot.as_mut() {
            Some(run) if run.result.is_none() && result != MavResult::MAV_RESULT_IN_PROGRESS => {
                run.result = Some(result);
                self.reap();
                true
            }
            _ => false,
        }
    }

    /// The next due COMMAND_ACK for link `link`, for the firmware's `Telemetry::acks` of that link: the
    /// final ACK of the ended action (handed out once, then the books are free), or an IN_PROGRESS with
    /// the percentage at most every [`PROGRESS_PERIOD_MS`]; `None` otherwise, and always `None` for a link
    /// that is not the origin of the action.
    pub fn next_ack(&mut self, link: u8, now_ms: u64) -> Option<LateAck> {
        self.expire(now_ms);
        let run = self.slot.as_mut()?;
        if run.link != link {
            return None;
        }
        let (command, target) = (run.command, run.sender);
        if let Some(result) = run.result {
            self.slot = None;
            return Some(LateAck {
                command,
                target,
                outcome: CommandOutcome::new(result),
            });
        }
        if now_ms >= run.next_progress_ms {
            run.next_progress_ms = now_ms.saturating_add(PROGRESS_PERIOD_MS);
            return Some(LateAck {
                command,
                target,
                outcome: CommandOutcome::in_progress(run.percent),
            });
        }
        None
    }

    /// Link `link` can no longer carry its ACKs (it is gone for good): if it is the origin of the
    /// action, the ACKs owed are dropped (the books are freed at once if the action has ended; a running
    /// action keeps running and its books are freed when it ends). Without this call an action whose
    /// origin link is never polled again would block every later action.
    pub fn link_lost(&mut self, link: u8) {
        if let Some(run) = self.slot.as_mut().filter(|r| r.link == link) {
            run.orphan = true;
        }
        self.reap();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::format;
    use std::string::String;
    use std::vec::Vec;

    const GCS: Peer = Peer {
        system_id: 255,
        component_id: 190,
    };
    const OTHER: Peer = Peer {
        system_id: 255,
        component_id: 191,
    };

    fn p(vals: &[(usize, f32)]) -> [f32; 7] {
        let mut a = [0.0; 7];
        for &(i, v) in vals {
            a[i] = v;
        }
        a
    }

    fn req(command: MavCmd, params: [f32; 7], sender: Peer) -> CommandRequest {
        CommandRequest {
            command,
            params,
            sender,
            confirmation: 0,
        }
    }

    const CAL: MavCmd = MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION;
    const STO: MavCmd = MavCmd::MAV_CMD_PREFLIGHT_STORAGE;
    const REB: MavCmd = MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN;
    const DEV: MavCmd = MavCmd::MAV_CMD_MARV_DEVICE;

    fn dev(device: f32, action: f32) -> [f32; 7] {
        p(&[(0, device), (1, action)])
    }

    fn long(a: Action) -> Result<Classified, Refusal> {
        Ok(Classified::Long(a))
    }

    fn not_yet() -> Result<Classified, Refusal> {
        Err(Refusal::Denied(NOT_YET))
    }

    fn is_denied(r: Result<Classified, Refusal>) -> bool {
        matches!(r, Err(Refusal::Denied(_)))
    }

    #[test]
    fn classify_accepts() {
        use Action::*;
        use MarvDevice::*;
        let table: Vec<(MavCmd, [f32; 7], Result<Classified, Refusal>)> = std::vec![
            (CAL, p(&[(0, 1.0)]), long(GyroCalibration)),
            (CAL, p(&[(4, 4.0)]), long(AccelCalibration)),
            (CAL, [0.0; 7], Ok(Classified::CancelCalibration)),
            (CAL, [-0.0; 7], Ok(Classified::CancelCalibration)),
            (STO, p(&[(0, 0.0)]), long(StorageReload)),
            (STO, p(&[(0, 1.0)]), long(StorageSave)),
            (STO, p(&[(0, 2.0)]), long(StorageDefaultsAll)),
            (STO, p(&[(0, 3.0)]), long(StorageDefaultsCalibration)),
            (STO, p(&[(0, 4.0)]), long(StorageDefaultsAll)),
            (REB, p(&[(0, 1.0)]), long(Reboot)),
            (DEV, dev(1.0, 1.0), long(DeviceRestart(MARV_DEVICE_IMU1))),
            (DEV, dev(2.0, 1.0), long(DeviceRestart(MARV_DEVICE_IMU2))),
            (DEV, dev(3.0, 1.0), long(DeviceRestart(MARV_DEVICE_BARO))),
            (DEV, dev(4.0, 1.0), long(DeviceRestart(MARV_DEVICE_MAG))),
            (DEV, dev(5.0, 1.0), long(DeviceRestart(MARV_DEVICE_GNSS))),
            (DEV, dev(6.0, 1.0), long(DeviceRestart(MARV_DEVICE_BATT))),
            (DEV, dev(1.0, 2.0), long(DeviceSelfTest(MARV_DEVICE_IMU1))),
            (DEV, dev(2.0, 2.0), long(DeviceSelfTest(MARV_DEVICE_IMU2))),
            (DEV, dev(4.0, 2.0), long(DeviceSelfTest(MARV_DEVICE_MAG))),
        ];
        for (cmd, params, want) in table {
            assert_eq!(classify(cmd, &params), want, "{cmd:?} {params:?}");
        }
    }

    #[test]
    fn classify_refuses_neighbours() {
        let nan = f32::NAN;
        let denied: Vec<(MavCmd, [f32; 7])> = std::vec![
            (CAL, p(&[(0, 2.0)])),
            (CAL, p(&[(0, 1.0), (4, 4.0)])),
            (CAL, p(&[(0, 1.0), (1, 1.0)])),
            (CAL, p(&[(4, 3.0)])),
            (CAL, p(&[(0, nan)])),
            (CAL, p(&[(6, 1.0)])),
            (STO, p(&[(0, 5.0)])),
            (STO, p(&[(0, 1.5)])),
            (STO, p(&[(0, nan)])),
            (STO, p(&[(0, -1.0)])),
            (STO, p(&[(0, 1.0e9)])),
            (STO, p(&[(0, f32::INFINITY)])),
            (REB, p(&[(0, 2.0)])),
            (REB, p(&[(0, 3.0)])),
            (REB, [0.0; 7]),
            (REB, p(&[(0, 1.0), (1, 1.0)])),
            (DEV, dev(7.0, 1.0)),
            (DEV, dev(8.0, 1.0)),
            (DEV, dev(9.0, 1.0)),
            (DEV, dev(13.0, 1.0)),
            (DEV, dev(0.0, 1.0)),
            (DEV, dev(1.0, 0.0)),
            (DEV, dev(1.0, 3.0)),
            (DEV, dev(3.0, 2.0)),
            (DEV, dev(5.0, 2.0)),
            (DEV, dev(6.0, 2.0)),
            (DEV, dev(10.0, 1.0)),
            (DEV, dev(11.0, 1.0)),
            (DEV, dev(12.0, 1.0)),
            (DEV, dev(1.5, 1.0)),
            (DEV, dev(nan, 1.0)),
            (DEV, dev(1.0, nan)),
        ];
        for (cmd, params) in denied {
            assert!(is_denied(classify(cmd, &params)), "{cmd:?} {params:?}");
        }
        assert_eq!(classify(CAL, &p(&[(0, 2.0)])), not_yet());
        assert_eq!(classify(DEV, &dev(7.0, 1.0)), not_yet());
        assert_eq!(classify(DEV, &dev(3.0, 2.0)), not_yet());
        assert_eq!(
            classify(MavCmd::MAV_CMD_NAV_TAKEOFF, &[0.0; 7]),
            Err(Refusal::Unsupported)
        );
        assert_eq!(
            classify(MavCmd::MAV_CMD_COMPONENT_ARM_DISARM, &p(&[(0, 1.0)])),
            Err(Refusal::Unsupported)
        );
    }

    const GYRO: [f32; 7] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    const SAVE: [f32; 7] = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

    fn admit(b: &mut ActionBook, r: &CommandRequest, armed: bool, hil: bool, now: u64) -> Decision {
        b.admit(r, 0, armed, hil, now)
    }

    #[test]
    fn admission_order() {
        let mut b = ActionBook::new();
        let t = MavCmd::MAV_CMD_NAV_TAKEOFF;
        // UNSUPPORTED beats armed.
        let d = admit(&mut b, &req(t, [0.0; 7], GCS), true, false, 0);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_UNSUPPORTED);
        // DENIED beats armed, and carries its text.
        let d = admit(&mut b, &req(CAL, p(&[(0, 2.0)]), GCS), true, false, 0);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_DENIED);
        assert_eq!(d.text, Some(NOT_YET));
        // A valid command while armed.
        let d = admit(&mut b, &req(CAL, GYRO, GCS), true, false, 0);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        assert_eq!(d.effect, Effect::None);
        assert_eq!(b.running(), None);
        // Another action runs.
        let d = admit(&mut b, &req(STO, SAVE, GCS), false, false, 0);
        assert_eq!(d.outcome, CommandOutcome::in_progress(0));
        assert_eq!(d.effect, Effect::Start(Action::StorageSave));
        let d = admit(&mut b, &req(CAL, GYRO, OTHER), false, false, 10);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        assert_eq!(b.running(), Some(Action::StorageSave));
        // HIL: calibration denied, storage admitted.
        let mut b = ActionBook::new();
        let d = admit(&mut b, &req(CAL, GYRO, GCS), false, true, 0);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_DENIED);
        assert_eq!(d.effect, Effect::None);
        assert_eq!(b.running(), None);
        let d = admit(&mut b, &req(STO, SAVE, GCS), false, true, 0);
        assert_eq!(d.effect, Effect::Start(Action::StorageSave));
    }

    #[test]
    fn retransmission_is_in_progress() {
        let mut b = ActionBook::new();
        let r = req(CAL, GYRO, GCS);
        assert_eq!(
            admit(&mut b, &r, false, false, 0).effect,
            Effect::Start(Action::GyroCalibration)
        );
        b.set_progress(100, 42);
        // Same command, same sender: IN_PROGRESS with the percentage, even if armed meanwhile.
        for armed in [false, true] {
            let d = admit(&mut b, &r, armed, false, 200);
            assert_eq!(d.outcome, CommandOutcome::in_progress(42));
            assert_eq!(d.effect, Effect::None);
        }
        assert_eq!(b.running(), Some(Action::GyroCalibration));
        // Another sender, or other parameters from the same sender: rejected, nothing changed.
        let d = admit(&mut b, &req(CAL, GYRO, OTHER), false, false, 200);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        let d = admit(&mut b, &req(CAL, p(&[(4, 4.0)]), GCS), false, false, 200);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        let d = admit(&mut b, &req(STO, SAVE, GCS), false, false, 200);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        assert_eq!(b.running(), Some(Action::GyroCalibration));
        assert_eq!(
            admit(&mut b, &r, false, false, 300).outcome,
            CommandOutcome::in_progress(42)
        );
        // Progress above 100 is cut.
        b.set_progress(300, 250);
        assert_eq!(
            admit(&mut b, &r, false, false, 300).outcome,
            CommandOutcome::in_progress(100)
        );
    }

    #[test]
    fn cancel_only_while_calibrating() {
        let mut b = ActionBook::new();
        let cancel = req(CAL, [0.0; 7], OTHER);
        let d = admit(&mut b, &cancel, false, false, 0);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_DENIED);
        assert_eq!(d.text, Some("nothing to cancel"));
        // Not while storage runs.
        admit(&mut b, &req(STO, SAVE, GCS), false, false, 0);
        let d = admit(&mut b, &cancel, false, false, 10);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_DENIED);
        assert_eq!(d.effect, Effect::None);
        assert!(b.finish(20, MavResult::MAV_RESULT_ACCEPTED));
        assert!(b.next_ack(0, 30).is_some());
        // While a calibration runs, even if armed meanwhile or from another sender.
        let d = admit(&mut b, &req(CAL, GYRO, GCS), false, false, 100);
        assert_eq!(d.effect, Effect::Start(Action::GyroCalibration));
        let d = admit(&mut b, &cancel, true, false, 200);
        assert_eq!(
            d.outcome,
            CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED)
        );
        assert_eq!(d.effect, Effect::CancelCalibration);
        assert_eq!(b.running(), None);
        // The firmware's own finish after the cancel changes nothing; the other link gets nothing.
        assert!(!b.finish(210, MavResult::MAV_RESULT_ACCEPTED));
        assert_eq!(b.next_ack(1, 700), None);
        let ack = b.next_ack(0, 700);
        assert_eq!(
            ack,
            Some(LateAck {
                command: CAL,
                target: GCS,
                outcome: CommandOutcome::new(MavResult::MAV_RESULT_CANCELLED),
            })
        );
        assert_eq!(b.next_ack(0, 701), None);
        assert_eq!(b.next_ack(0, 5_000), None);
        // A second cancel finds nothing.
        let d = admit(&mut b, &cancel, false, false, 800);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_DENIED);
    }

    #[test]
    fn progress_acks_at_2hz_on_origin_only() {
        let mut b = ActionBook::new();
        b.admit(&req(STO, SAVE, GCS), 1, false, false, 1_000);
        assert_eq!(b.next_ack(1, 1_499), None);
        b.set_progress(1_500, 30);
        let a = b.next_ack(1, 1_500).unwrap();
        assert_eq!(a.outcome, CommandOutcome::in_progress(30));
        assert_eq!(a.target, GCS);
        assert_eq!(b.next_ack(0, 1_500), None);
        assert_eq!(b.next_ack(1, 1_999), None);
        assert_eq!(
            b.next_ack(1, 2_000).unwrap().outcome,
            CommandOutcome::in_progress(30)
        );
        assert!(b.finish(2_100, MavResult::MAV_RESULT_ACCEPTED));
        assert_eq!(b.next_ack(0, 2_100), None);
        assert_eq!(
            b.next_ack(1, 2_100).unwrap().outcome.result,
            MavResult::MAV_RESULT_ACCEPTED
        );
        assert_eq!(b.next_ack(1, 9_000), None);
    }

    #[test]
    fn timeouts_are_exact() {
        let cases: [(MavCmd, [f32; 7], u64); 7] = [
            (CAL, GYRO, 15_000),
            (CAL, p(&[(4, 4.0)]), 15_000),
            (STO, SAVE, 3_000),
            (STO, p(&[(0, 2.0)]), 3_000),
            (DEV, dev(1.0, 1.0), 5_000),
            (DEV, dev(4.0, 2.0), 5_000),
            (REB, p(&[(0, 1.0)]), 3_000),
        ];
        for (cmd, params, max) in cases {
            let t0 = 7_000;
            let mut b = ActionBook::new();
            assert_eq!(
                b.admit(&req(cmd, params, GCS), 0, false, false, t0)
                    .outcome
                    .result,
                MavResult::MAV_RESULT_IN_PROGRESS
            );
            assert_eq!(b.tick(t0 + max - 1), None, "{cmd:?}");
            assert!(b.running().is_some());
            // Polling just before the maximum gives no final ACK.
            assert_ne!(
                b.next_ack(0, t0 + max - 1).map(|a| a.outcome.result),
                Some(MavResult::MAV_RESULT_FAILED)
            );
            assert!(b.tick(t0 + max).is_some(), "{cmd:?}");
            assert_eq!(b.running(), None);
            let a = b.next_ack(0, t0 + max).unwrap();
            assert_eq!(a.outcome.result, MavResult::MAV_RESULT_FAILED, "{cmd:?}");
            assert_eq!(b.next_ack(0, t0 + max + 1), None);
            // A finish at the maximum has lost against the timeout; one just before wins.
            let mut b = ActionBook::new();
            b.admit(&req(cmd, params, GCS), 0, false, false, t0);
            assert!(!b.finish(t0 + max, MavResult::MAV_RESULT_ACCEPTED));
            assert_eq!(
                b.next_ack(0, t0 + max).unwrap().outcome.result,
                MavResult::MAV_RESULT_FAILED
            );
            let mut b = ActionBook::new();
            b.admit(&req(cmd, params, GCS), 0, false, false, t0);
            assert!(b.finish(t0 + max - 1, MavResult::MAV_RESULT_ACCEPTED));
            assert_eq!(
                b.next_ack(0, t0 + max).unwrap().outcome.result,
                MavResult::MAV_RESULT_ACCEPTED
            );
        }
    }

    #[test]
    fn final_ack_blocks_next_action_until_delivered_or_link_lost() {
        let mut b = ActionBook::new();
        b.admit(&req(STO, SAVE, GCS), 1, false, false, 0);
        assert!(b.finish(10, MavResult::MAV_RESULT_ACCEPTED));
        assert!(!b.finish(11, MavResult::MAV_RESULT_FAILED));
        let d = b.admit(&req(STO, p(&[(0, 0.0)]), GCS), 0, false, false, 20);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        b.link_lost(0);
        assert!(b.next_ack(1, 30).is_some());
        assert_eq!(b.running(), None);
        // Link lost while running: no more progress, no final, books free after the end.
        b.admit(&req(STO, SAVE, GCS), 1, false, false, 100);
        b.link_lost(1);
        assert_eq!(b.next_ack(1, 2_999), None);
        let d = b.admit(&req(STO, SAVE, OTHER), 0, false, false, 2_999);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        assert_eq!(b.next_ack(1, 3_100), None);
        assert_eq!(b.running(), None);
        let d = b.admit(&req(STO, SAVE, GCS), 0, false, false, 3_100);
        assert_eq!(d.effect, Effect::Start(Action::StorageSave));
    }

    // ---- I5.3, property style ----

    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
    }

    trait Source {
        fn admit(
            &mut self,
            r: &CommandRequest,
            link: u8,
            armed: bool,
            hil: bool,
            now: u64,
        ) -> Decision;
        fn progress(&mut self, now: u64, pct: u8);
        fn finish(&mut self, now: u64, result: MavResult);
        fn poll(&mut self, link: u8, now: u64) -> Option<LateAck>;
    }

    impl Source for ActionBook {
        fn admit(
            &mut self,
            r: &CommandRequest,
            link: u8,
            armed: bool,
            hil: bool,
            now: u64,
        ) -> Decision {
            ActionBook::admit(self, r, link, armed, hil, now)
        }
        fn progress(&mut self, now: u64, pct: u8) {
            self.set_progress(now, pct);
        }
        fn finish(&mut self, now: u64, result: MavResult) {
            ActionBook::finish(self, now, result);
        }
        fn poll(&mut self, link: u8, now: u64) -> Option<LateAck> {
            self.next_ack(link, now)
        }
    }

    #[derive(Clone, Copy, PartialEq)]
    enum Break {
        /// The final ACK is offered again on the next poll of the link, until the end of the run.
        RepeatFinal,
        /// The final ACK is offered to whichever link asks.
        AnyLink,
        /// A progress ACK is offered on every poll.
        NoPacing,
    }

    /// Test-only variant of the books that breaks I5.3 or the pacing, to prove the checker sees it.
    struct Broken {
        inner: ActionBook,
        mode: Break,
        last_final: Option<LateAck>,
        last_progress: Option<LateAck>,
        origin: u8,
    }

    impl Source for Broken {
        fn admit(
            &mut self,
            r: &CommandRequest,
            link: u8,
            armed: bool,
            hil: bool,
            now: u64,
        ) -> Decision {
            let d = self.inner.admit(r, link, armed, hil, now);
            if matches!(d.effect, Effect::Start(_)) {
                self.origin = link;
            }
            d
        }
        fn progress(&mut self, now: u64, pct: u8) {
            self.inner.set_progress(now, pct);
        }
        fn finish(&mut self, now: u64, result: MavResult) {
            self.inner.finish(now, result);
        }
        fn poll(&mut self, link: u8, now: u64) -> Option<LateAck> {
            let real_link = if self.mode == Break::AnyLink {
                self.origin
            } else {
                link
            };
            if let Some(a) = self.inner.next_ack(real_link, now) {
                if a.outcome.result == MavResult::MAV_RESULT_IN_PROGRESS {
                    self.last_progress = Some(a);
                } else {
                    self.last_final = Some(a);
                }
                return Some(a);
            }
            match self.mode {
                Break::RepeatFinal => self.last_final,
                Break::NoPacing if link == self.origin => self
                    .last_progress
                    .filter(|_| self.inner.running().is_some()),
                _ => None,
            }
        }
    }

    struct Current {
        link: u8,
        command: MavCmd,
        sender: Peer,
        final_seen: bool,
    }

    /// One random sequence; Err names the first violation, Ok carries the number of admitted actions.
    fn run_sequence(src: &mut dyn Source, seed: u64) -> Result<u32, String> {
        let mut rng = Rng(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
        let senders = [GCS, OTHER];
        let cmds: [(MavCmd, [f32; 7]); 7] = [
            (CAL, GYRO),
            (STO, SAVE),
            (REB, p(&[(0, 1.0)])),
            (CAL, [0.0; 7]),
            (DEV, dev(1.0, 1.0)),
            (DEV, dev(4.0, 2.0)),
            (MavCmd::MAV_CMD_NAV_TAKEOFF, [0.0; 7]),
        ];
        let mut now = rng.below(100_000);
        let mut cur: Option<Current> = None;
        let (mut admitted, mut finals) = (0u32, 0u32);
        let mut last_progress: [Option<u64>; 2] = [None, None];
        let steps = 20 + rng.below(80);
        let poll = |src: &mut dyn Source,
                    cur: &mut Option<Current>,
                    link: u8,
                    now: u64,
                    finals: &mut u32,
                    last_progress: &mut [Option<u64>; 2]|
         -> Result<(), String> {
            let Some(ack) = src.poll(link, now) else {
                return Ok(());
            };
            let Some(c) = cur.as_mut() else {
                return Err(format!("ack with no admitted action: {ack:?}"));
            };
            if c.final_seen {
                return Err(format!("ack after the final one: {ack:?}"));
            }
            if c.link != link {
                return Err(format!("ack for link {} on link {link}", c.link));
            }
            if ack.command != c.command || ack.target != c.sender {
                return Err(format!("ack for the wrong command or target: {ack:?}"));
            }
            if ack.outcome.result == MavResult::MAV_RESULT_IN_PROGRESS {
                let slot = last_progress.get_mut(usize::from(link)).ok_or("link")?;
                if let Some(t) = *slot {
                    if now < t + PROGRESS_PERIOD_MS {
                        return Err(format!("progress ACKs {} ms apart on link {link}", now - t));
                    }
                }
                *slot = Some(now);
            } else {
                c.final_seen = true;
                *finals += 1;
            }
            Ok(())
        };
        for _ in 0..steps {
            match rng.below(8) {
                0 | 1 => {
                    let (cmd, params) = cmds[rng.below(7) as usize];
                    let sender = senders[rng.below(2) as usize];
                    let link = rng.below(2) as u8;
                    let (armed, hil) = (rng.below(8) == 0, rng.below(6) == 0);
                    let d = src.admit(&req(cmd, params, sender), link, armed, hil, now);
                    if let Effect::Start(_) = d.effect {
                        if cur.as_ref().is_some_and(|c| !c.final_seen) {
                            return Err(
                                "admitted while the previous action is not finalised".into()
                            );
                        }
                        cur = Some(Current {
                            link,
                            command: cmd,
                            sender,
                            final_seen: false,
                        });
                        admitted += 1;
                        for slot in last_progress.iter_mut() {
                            *slot = None;
                        }
                    }
                }
                2 => src.progress(now, rng.below(130) as u8),
                3 => {
                    let r = [
                        MavResult::MAV_RESULT_ACCEPTED,
                        MavResult::MAV_RESULT_FAILED,
                        MavResult::MAV_RESULT_DENIED,
                    ][rng.below(3) as usize];
                    src.finish(now, r);
                }
                4 => {
                    now += if rng.below(20) == 0 {
                        20_000
                    } else {
                        rng.below(900)
                    }
                }
                5 => poll(src, &mut cur, 0, now, &mut finals, &mut last_progress)?,
                6 => poll(src, &mut cur, 1, now, &mut finals, &mut last_progress)?,
                _ => {
                    poll(src, &mut cur, 0, now, &mut finals, &mut last_progress)?;
                    poll(src, &mut cur, 1, now, &mut finals, &mut last_progress)?;
                }
            }
        }
        // Drain: every action has ended by timeout and its final ACK is polled.
        for _ in 0..3 {
            now += 20_000;
            poll(src, &mut cur, 0, now, &mut finals, &mut last_progress)?;
            poll(src, &mut cur, 1, now, &mut finals, &mut last_progress)?;
        }
        if finals != admitted {
            return Err(format!("{admitted} admitted, {finals} final ACKs"));
        }
        Ok(admitted)
    }

    #[test]
    fn i5_3_random_sequences() {
        let mut total = 0u32;
        let mut free_at_end = 0u32;
        for seed in 1..=10_000u64 {
            let mut book = ActionBook::new();
            match run_sequence(&mut book, seed) {
                Ok(n) => total += n,
                Err(e) => panic!("seed {seed}: {e}"),
            }
            free_at_end += u32::from(book.running().is_none());
        }
        // The run is not vacuous.
        assert!(total > 10_000, "only {total} admitted actions");
        assert_eq!(free_at_end, 10_000);
    }

    #[test]
    fn i5_3_checker_catches_broken_books() {
        for mode in [Break::RepeatFinal, Break::AnyLink, Break::NoPacing] {
            let mut caught = 0u32;
            for seed in 1..=10_000u64 {
                let mut b = Broken {
                    inner: ActionBook::new(),
                    mode,
                    last_final: None,
                    last_progress: None,
                    origin: 0,
                };
                if run_sequence(&mut b, seed).is_err() {
                    caught += 1;
                }
            }
            assert!(caught > 1_000, "mode not caught often enough: {caught}");
        }
    }

    #[test]
    fn checker_reports_a_repeated_final_by_name() {
        let mut b = Broken {
            inner: ActionBook::new(),
            mode: Break::RepeatFinal,
            last_final: None,
            last_progress: None,
            origin: 0,
        };
        let e = (1..=1_000u64).find_map(|s| run_sequence(&mut b, s).err());
        assert!(
            e.is_some_and(|e| e.contains("ack after the final one") || e.contains("no admitted"))
        );
    }
}
