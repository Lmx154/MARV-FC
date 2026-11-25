//! The rule for accepting an action.
//!
//! Two pure parts, no I/O, no clock of its own, no statics:
//!
//! - [`classify`]: a command and its seven parameters become a recognised [`Classified`] action or a
//!   [`Refusal`] (UNSUPPORTED, or DENIED with a short text for a STATUSTEXT).
//! - [`ActionBook`]: the admission order and the books of the one long
//!   action that may run at a time, firmware-wide. The firmware passes `armed`, `hil`, the link id and
//!   `now_ms` as plain values, executes the action itself, and reports progress and the result back.
//!
//! The calibrations are `MAV_CMD_PREFLIGHT_CALIBRATION` (241): param1 = 1 the gyroscope, param5 = 4 the
//! accelerometer (each ends FAILED after [`CALIBRATION_MAX_MS`]), param2 = 1 the compass (the user rotates
//! the vehicle for 30 s; it ends FAILED after [`MAG_CALIBRATION_MAX_MS`]); each only with every other
//! parameter zero. The all-zero 241 cancels the running calibration.
//!
//! `MAV_CMD_DO_MOTOR_TEST` (209) is not a long action: [`MotorTest`] admits it (ACCEPTED at once, no
//! IN_PROGRESS, nothing in the book) and numbers the [`TestCmd`] the firmware hands to the flight core.
//!
//! The ACKs of a long action: the answer to the command is IN_PROGRESS (0 %); later
//! [`ActionBook::next_ack`] hands out IN_PROGRESS with the percentage at 2 Hz, then the one final ACK,
//! both only for the link the command came from.

use mavlink_device::marv::{MarvDevice, MavCmd, MavResult};
use mavlink_device::{CommandOutcome, CommandRequest, LateAck, Peer};

use crate::esc::{TEST_BURST_MAX_US, TEST_CAP, TestCmd, line_of_element};

/// Maximum duration of a gyroscope or accelerometer calibration, in milliseconds.
pub const CALIBRATION_MAX_MS: u64 = 15_000;
/// Maximum duration of the compass calibration, in milliseconds (the user rotates the vehicle for 30 s).
pub const MAG_CALIBRATION_MAX_MS: u64 = 40_000;
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
    /// Compass (magnetometer) calibration (`PREFLIGHT_CALIBRATION` param2 = 1); at most
    /// [`MAG_CALIBRATION_MAX_MS`].
    MagCalibration,
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
            Self::GyroCalibration | Self::AccelCalibration | Self::MagCalibration => {
                ActionClass::Calibration
            }
            Self::StorageReload
            | Self::StorageSave
            | Self::StorageDefaultsAll
            | Self::StorageDefaultsCalibration => ActionClass::Storage,
            Self::Reboot => ActionClass::Reboot,
            Self::DeviceRestart(_) | Self::DeviceSelfTest(_) => ActionClass::Device,
        }
    }

    /// The time after which a running action ends FAILED by itself, in milliseconds: the compass
    /// calibration has [`MAG_CALIBRATION_MAX_MS`], the other calibrations [`CALIBRATION_MAX_MS`].
    pub const fn max_duration_ms(self) -> u64 {
        match self.class() {
            ActionClass::Calibration => match self {
                Self::MagCalibration => MAG_CALIBRATION_MAX_MS,
                _ => CALIBRATION_MAX_MS,
            },
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
    } else if only(params, 1, 1.0) {
        Ok(Classified::Long(Action::MagCalibration))
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
        Some(9) => MarvDevice::MARV_DEVICE_SD,
        // RC, PSRAM, LED, ESC: devices of the dialect that this build does not serve.
        Some(7 | 10 | 11 | 12) => return Err(Refusal::Denied(NOT_YET)),
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

    /// True when an action is in the books (running, or ended with its final ACK still owed): the
    /// state in which [`ActionBook::admit`] answers TEMPORARILY_REJECTED.
    pub fn busy(&mut self, now_ms: u64) -> bool {
        self.expire(now_ms);
        self.slot.is_some()
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
        if run.link != link || run.orphan {
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

// ---------------------------------------------------------------------------------------------
// Motor test.
// ---------------------------------------------------------------------------------------------

/// The highest throttle of a motor test, in percent (param3).
pub const MOTOR_TEST_PERCENT_MAX: f32 = 5.0;
/// The longest spin of a motor test, in seconds (param4).
pub const MOTOR_TEST_SECONDS_MAX: f32 = 3.0;
/// Throttle levels per percent: 2000 levels are 100 % (`dshot_spin` `step_level`,
/// `dshot/hil/src/bin/dshot_spin.rs` 202-206, "each at least 1"; 5 % is level 100).
const LEVELS_PER_PERCENT: f32 = 20.0;

const _: () = assert!(
    (MOTOR_TEST_PERCENT_MAX * LEVELS_PER_PERCENT + 0.5) as u16 <= TEST_CAP,
    "the highest accepted throttle is above TEST_CAP"
);
const _: () = assert!(
    (MOTOR_TEST_SECONDS_MAX * 1.0e6) as u64 <= TEST_BURST_MAX_US,
    "the longest accepted spin is above TEST_BURST_MAX_US"
);

const MT_MOTOR: &str = "motor test: motor must be 1 to 4";
const MT_TYPE: &str = "motor test: type must be 0 (percent)";
const MT_THROTTLE: &str = "motor test: throttle must be 0 to 5 %";
const MT_DURATION: &str = "motor test: duration must be 0 to 3 s";
const MT_COUNT: &str = "motor test: count must be 0 or 1";
const MT_ORDER: &str = "motor test: order must be 0 or 2";
const MT_PARAM7: &str = "motor test: param7 must be 0";

/// What a `MAV_CMD_DO_MOTOR_TEST` asks for, parameters checked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotorTestRequest {
    /// Stop now (param3 = 0).
    Stop,
    /// Spin the motor `motor` (1..=4, the board label M1..M4) at throttle `level` (of 2000) for
    /// `duration_us`.
    Spin {
        /// M1..M4.
        motor: u8,
        /// 1..=[`TEST_CAP`].
        level: u16,
        /// 1..=3 000 000 us.
        duration_us: u64,
    },
}

/// The motor-test table: param1 a whole number 1-4, param2 0, param3 0 (stop) or
/// above 0 up to 5.0, param4 above 0 up to 3.0 when param3 > 0 (ignored for a stop), param5 0 or 1,
/// param6 0 or 2, param7 0. Anything else, NaN included, is refused with a "motor test: ..." text.
/// Returns the motor (1..=4) with the request.
pub fn classify_motor_test(params: &[f32; 7]) -> Result<(u8, MotorTestRequest), &'static str> {
    let [p1, p2, p3, p4, p5, p6, p7] = *params;
    let motor = match int(p1) {
        Some(n @ 1..=4) => n as u8,
        _ => return Err(MT_MOTOR),
    };
    if p2 != 0.0 {
        return Err(MT_TYPE);
    }
    if !(0.0..=MOTOR_TEST_PERCENT_MAX).contains(&p3) {
        return Err(MT_THROTTLE);
    }
    let spin = p3 > 0.0;
    if spin && !(p4 > 0.0 && p4 <= MOTOR_TEST_SECONDS_MAX) {
        return Err(MT_DURATION);
    }
    if p5 != 0.0 && p5 != 1.0 {
        return Err(MT_COUNT);
    }
    if p6 != 0.0 && p6 != 2.0 {
        return Err(MT_ORDER);
    }
    if p7 != 0.0 {
        return Err(MT_PARAM7);
    }
    if !spin {
        return Ok((motor, MotorTestRequest::Stop));
    }
    // p3 is above 0 and at most 5.0 here: adding 0.5 and truncating is round(), and the level is
    // at least 1.
    let level = ((p3 * LEVELS_PER_PERCENT + 0.5) as u16).max(1);
    let duration_us = (p4 * 1.0e6) as u64;
    Ok((
        motor,
        MotorTestRequest::Spin {
            motor,
            level,
            duration_us,
        },
    ))
}

/// The decision of [`MotorTest::admit`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorTestDecision {
    /// The answer to the command: ACCEPTED at once, TEMPORARILY_REJECTED or DENIED.
    pub outcome: CommandOutcome,
    /// For DENIED: the STATUSTEXT, "motor test: <reason>".
    pub text: Option<&'static str>,
    /// For ACCEPTED: the whole command to write to the flight core's cell (a new `seq`).
    pub cmd: Option<TestCmd>,
    /// The motor (1..=4) of an accepted command.
    pub motor: u8,
    /// For an accepted spin: its level (of 2000), else 0.
    pub level: u16,
}

/// Core 0's record of the motor test: the `seq` of the last command it gave the flight core, and until
/// when the last accepted spin runs. Pure: no clock, no cell.
#[derive(Clone, Copy, Debug, Default)]
pub struct MotorTest {
    seq: u32,
    spin_until_us: Option<u64>,
}

impl MotorTest {
    /// Nothing sent yet: the first command gets `seq` 1.
    pub const fn new() -> Self {
        Self {
            seq: 0,
            spin_until_us: None,
        }
    }

    /// True while a spin accepted here is not over: its `until_us` is not reached and no stop was
    /// accepted since. The firmware passes this (or `armed`) as `armed` to [`ActionBook::admit`], so
    /// every other action is TEMPORARILY_REJECTED during a spin.
    pub fn spin_active(&self, now_us: u64) -> bool {
        self.spin_until_us.is_some_and(|until| now_us < until)
    }

    /// The next `seq`: counts from 1 and skips 0 on wrap (the flight core treats 0 as done, and
    /// ignores a change of fields under the same `seq`).
    fn next_seq(&mut self) -> u32 {
        self.seq = match self.seq.wrapping_add(1) {
            0 => 1,
            n => n,
        };
        self.seq
    }

    /// Decide a `MAV_CMD_DO_MOTOR_TEST` with `params` at board time `now_us`. `book_busy`: another long
    /// action is in the books ([`ActionBook::busy`]); `armed` the vehicle is armed.
    ///
    /// DENIED for a parameter outside [`classify_motor_test`]'s table. A stop is ACCEPTED in every state.
    /// A spin is TEMPORARILY_REJECTED if `book_busy` or `armed`; otherwise ACCEPTED at once, and it
    /// replaces a running spin (a spin running here does not refuse it). Every accepted command, spin or
    /// stop, gets a new `seq` and a whole [`TestCmd`].
    pub fn admit(
        &mut self,
        params: &[f32; 7],
        book_busy: bool,
        armed: bool,
        now_us: u64,
    ) -> MotorTestDecision {
        let none = |result| MotorTestDecision {
            outcome: CommandOutcome::new(result),
            text: None,
            cmd: None,
            motor: 0,
            level: 0,
        };
        let (motor, request) = match classify_motor_test(params) {
            Ok(ok) => ok,
            Err(text) => {
                return MotorTestDecision {
                    text: Some(text),
                    ..none(MavResult::MAV_RESULT_DENIED)
                };
            }
        };
        let cmd = match request {
            MotorTestRequest::Stop => {
                self.spin_until_us = None;
                TestCmd {
                    seq: self.next_seq(),
                    line: 0,
                    level: 0,
                    until_us: 0,
                }
            }
            MotorTestRequest::Spin {
                motor,
                level,
                duration_us,
            } => {
                if book_busy || armed {
                    return none(MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
                }
                let until_us = now_us.saturating_add(duration_us);
                self.spin_until_us = Some(until_us);
                TestCmd {
                    seq: self.next_seq(),
                    // `motor` is 1..=4, so the mapping has an answer; 0 is a line that is a stop's.
                    line: line_of_element(usize::from(motor - 1)).map_or(0, |l| l as u8),
                    level,
                    until_us,
                }
            }
        };
        MotorTestDecision {
            cmd: Some(cmd),
            motor,
            level: cmd.level,
            ..none(MavResult::MAV_RESULT_ACCEPTED)
        }
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
            (CAL, p(&[(1, 1.0)]), long(MagCalibration)),
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
            (DEV, dev(9.0, 1.0), long(DeviceRestart(MARV_DEVICE_SD))),
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
            (CAL, p(&[(1, 2.0)])),
            (CAL, p(&[(1, 1.5)])),
            (CAL, p(&[(1, nan)])),
            (CAL, p(&[(0, 1.0), (1, 1.0)])),
            (CAL, p(&[(1, 1.0), (4, 4.0)])),
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
            (DEV, dev(9.0, 2.0)),
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
        assert_eq!(classify(CAL, &p(&[(0, 1.0), (1, 1.0)])), not_yet());
        assert_eq!(classify(CAL, &p(&[(1, 1.0), (4, 4.0)])), not_yet());
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
    const MAG: [f32; 7] = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0];

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
    fn mag_calibration_admission() {
        // Idle and not armed: admitted.
        let mut b = ActionBook::new();
        let d = admit(&mut b, &req(CAL, MAG, GCS), false, false, 0);
        assert_eq!(d.outcome, CommandOutcome::in_progress(0));
        assert_eq!(d.effect, Effect::Start(Action::MagCalibration));
        assert_eq!(b.running(), Some(Action::MagCalibration));
        // Armed: TEMPORARILY_REJECTED, nothing starts.
        let mut b = ActionBook::new();
        let d = admit(&mut b, &req(CAL, MAG, GCS), true, false, 0);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        assert_eq!(d.effect, Effect::None);
        assert_eq!(b.running(), None);
        // Another action runs: TEMPORARILY_REJECTED.
        admit(&mut b, &req(STO, SAVE, GCS), false, false, 0);
        let d = admit(&mut b, &req(CAL, MAG, OTHER), false, false, 10);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        assert_eq!(b.running(), Some(Action::StorageSave));
        // HIL: DENIED.
        let mut b = ActionBook::new();
        let d = admit(&mut b, &req(CAL, MAG, GCS), false, true, 0);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_DENIED);
        assert_eq!(d.text, Some("calibration not allowed in HIL"));
        assert_eq!(d.effect, Effect::None);
        assert_eq!(b.running(), None);
        // A retransmission from the same sender is IN_PROGRESS; another sender is rejected.
        let mut b = ActionBook::new();
        let r = req(CAL, MAG, GCS);
        admit(&mut b, &r, false, false, 0);
        b.set_progress(100, 17);
        let d = admit(&mut b, &r, false, false, 200);
        assert_eq!(d.outcome, CommandOutcome::in_progress(17));
        assert_eq!(d.effect, Effect::None);
        let d = admit(&mut b, &req(CAL, MAG, OTHER), false, false, 200);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        // The all-zero 241 cancels it: one final CANCELLED, on the origin link only.
        let mut b = ActionBook::new();
        b.admit(&req(CAL, MAG, GCS), 1, false, false, 0);
        let d = admit(&mut b, &req(CAL, [0.0; 7], OTHER), false, false, 5_000);
        assert_eq!(
            d.outcome,
            CommandOutcome::new(MavResult::MAV_RESULT_ACCEPTED)
        );
        assert_eq!(d.effect, Effect::CancelCalibration);
        assert_eq!(b.running(), None);
        assert_eq!(b.next_ack(0, 5_001), None);
        assert_eq!(
            b.next_ack(1, 5_001),
            Some(LateAck {
                command: CAL,
                target: GCS,
                outcome: CommandOutcome::new(MavResult::MAV_RESULT_CANCELLED),
            })
        );
        assert_eq!(b.next_ack(1, 5_002), None);
        assert_eq!(b.next_ack(1, 60_000), None);
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
        let cases: [(MavCmd, [f32; 7], u64); 8] = [
            (CAL, GYRO, 15_000),
            (CAL, p(&[(4, 4.0)]), 15_000),
            (CAL, MAG, 40_000),
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
        let cmds: [(MavCmd, [f32; 7]); 8] = [
            (CAL, GYRO),
            (CAL, MAG),
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
                if let Some(t) = *slot
                    && now < t + PROGRESS_PERIOD_MS
                {
                    return Err(format!("progress ACKs {} ms apart on link {link}", now - t));
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
                    let (cmd, params) = cmds[rng.below(8) as usize];
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

    // ---- motor test ----

    /// A spin of motor `m` at `pct` percent for `secs` seconds, the rest of the table at its zeros.
    fn spin(m: f32, pct: f32, secs: f32) -> [f32; 7] {
        p(&[(0, m), (2, pct), (3, secs)])
    }

    fn stop(m: f32) -> [f32; 7] {
        p(&[(0, m)])
    }

    #[test]
    fn motor_test_accepts_every_row_of_the_table() {
        let ok = |params: [f32; 7]| classify_motor_test(&params);
        let sp = |motor, level, duration_us| MotorTestRequest::Spin {
            motor,
            level,
            duration_us,
        };
        assert_eq!(ok(spin(1.0, 5.0, 3.0)), Ok((1, sp(1, 100, 3_000_000))));
        assert_eq!(ok(spin(4.0, 0.05, 0.1)), Ok((4, sp(4, 1, 100_000))));
        assert_eq!(ok(spin(2.0, 1.65, 2.0)), Ok((2, sp(2, 33, 2_000_000))));
        // Count 0 or 1, order 0 or 2.
        for (count, order) in [(0.0, 0.0), (1.0, 0.0), (0.0, 2.0), (1.0, 2.0)] {
            let mut q = spin(3.0, 2.0, 1.0);
            q[4] = count;
            q[5] = order;
            assert_eq!(ok(q), Ok((3, sp(3, 40, 1_000_000))), "{q:?}");
        }
        assert_eq!(ok(stop(1.0)), Ok((1, MotorTestRequest::Stop)));
        assert_eq!(ok(stop(4.0)), Ok((4, MotorTestRequest::Stop)));
        // The duration is ignored for a stop, even when it is nonsense.
        let mut q = stop(2.0);
        q[3] = f32::NAN;
        assert_eq!(ok(q), Ok((2, MotorTestRequest::Stop)));
        q[3] = 99.0;
        assert_eq!(ok(q), Ok((2, MotorTestRequest::Stop)));
        assert_eq!(
            ok(p(&[(0, 1.0), (2, -0.0)])),
            Ok((1, MotorTestRequest::Stop))
        );
    }

    #[test]
    fn motor_test_refuses_every_row_outside_the_table_with_its_text() {
        let nan = f32::NAN;
        let inf = f32::INFINITY;
        let with = |i: usize, v: f32| {
            let mut q = spin(1.0, 2.0, 1.0);
            q[i] = v;
            q
        };
        let rows: Vec<([f32; 7], &str)> = std::vec![
            // param1
            (with(0, 0.0), MT_MOTOR),
            (with(0, 5.0), MT_MOTOR),
            (with(0, -1.0), MT_MOTOR),
            (with(0, 1.5), MT_MOTOR),
            (with(0, nan), MT_MOTOR),
            (with(0, inf), MT_MOTOR),
            (stop(0.0), MT_MOTOR),
            (stop(5.0), MT_MOTOR),
            // param2
            (with(1, 1.0), MT_TYPE),
            (with(1, 2.0), MT_TYPE),
            (with(1, nan), MT_TYPE),
            // param3
            (with(2, 5.01), MT_THROTTLE),
            (with(2, 100.0), MT_THROTTLE),
            (with(2, -0.1), MT_THROTTLE),
            (with(2, nan), MT_THROTTLE),
            (with(2, inf), MT_THROTTLE),
            (with(2, f32::NEG_INFINITY), MT_THROTTLE),
            // param4 (a spin)
            (with(3, 0.0), MT_DURATION),
            (with(3, -1.0), MT_DURATION),
            (with(3, 3.01), MT_DURATION),
            (with(3, nan), MT_DURATION),
            (with(3, inf), MT_DURATION),
            // param5
            (with(4, 2.0), MT_COUNT),
            (with(4, -1.0), MT_COUNT),
            (with(4, 0.5), MT_COUNT),
            (with(4, nan), MT_COUNT),
            // param6
            (with(5, 1.0), MT_ORDER),
            (with(5, 3.0), MT_ORDER),
            (with(5, nan), MT_ORDER),
            // param7
            (with(6, 1.0), MT_PARAM7),
            (with(6, -1.0), MT_PARAM7),
            (with(6, nan), MT_PARAM7),
        ];
        for (params, text) in rows {
            assert_eq!(classify_motor_test(&params), Err(text), "{params:?}");
            // The decision is DENIED with the text, nothing for the flight core, in any state.
            for (busy, armed) in [(false, false), (true, false), (false, true), (true, true)] {
                let mut mt = MotorTest::new();
                let d = mt.admit(&params, busy, armed, 5);
                assert_eq!(d.outcome.result, MavResult::MAV_RESULT_DENIED, "{params:?}");
                assert_eq!(d.text, Some(text));
                assert_eq!(d.cmd, None);
                assert!(!mt.spin_active(6));
            }
        }
        // Stop rows of the other parameters are checked too.
        for i in [1, 4, 5, 6] {
            let mut q = stop(1.0);
            q[i] = 7.0;
            assert!(classify_motor_test(&q).is_err(), "{q:?}");
        }
        for t in [
            MT_MOTOR,
            MT_TYPE,
            MT_THROTTLE,
            MT_DURATION,
            MT_COUNT,
            MT_ORDER,
            MT_PARAM7,
        ] {
            assert!(t.starts_with("motor test: "));
            assert!(t.len() <= 50, "{t}: STATUSTEXT holds 50 bytes");
        }
    }

    #[test]
    fn motor_test_level_is_round_of_twenty_per_percent_at_least_one() {
        let level = |pct: f32| match classify_motor_test(&spin(1.0, pct, 1.0)) {
            Ok((_, MotorTestRequest::Spin { level, .. })) => level,
            other => panic!("{pct}: {other:?}"),
        };
        assert_eq!(level(5.0), TEST_CAP);
        assert_eq!(level(0.01), 1);
        assert_eq!(level(0.02), 1);
        assert_eq!(level(0.03), 1);
        assert_eq!(level(0.04), 1);
        assert_eq!(level(0.07), 1);
        assert_eq!(level(0.08), 2);
        assert_eq!(level(1.0), 20);
        assert_eq!(level(1.65), 33);
        assert_eq!(level(3.3), 66);
        assert_eq!(level(4.99), 100);
        for k in 1..=5000u32 {
            let pct = k as f32 / 1000.0;
            let l = level(pct);
            assert!((1..=TEST_CAP).contains(&l), "{pct}: {l}");
        }
    }

    #[test]
    fn motor_test_motors_map_to_lines_by_line_of_element() {
        let mut mt = MotorTest::new();
        for motor in 1..=4u8 {
            let d = mt.admit(&spin(f32::from(motor), 5.0, 1.0), false, false, 1_000);
            assert_eq!(d.outcome.result, MavResult::MAV_RESULT_ACCEPTED);
            let cmd = d.cmd.unwrap();
            assert_eq!(
                usize::from(cmd.line),
                line_of_element(usize::from(motor) - 1).unwrap()
            );
            assert_eq!((d.motor, d.level, cmd.level), (motor, 100, 100));
            assert_eq!(cmd.until_us, 1_000 + 1_000_000);
        }
        // M1 is line 3 and M4 line 0.
        let m1 = mt.admit(&spin(1.0, 1.0, 1.0), false, false, 0).cmd.unwrap();
        let m4 = mt.admit(&spin(4.0, 1.0, 1.0), false, false, 0).cmd.unwrap();
        assert_eq!((m1.line, m4.line), (3, 0));
    }

    #[test]
    fn motor_test_spin_is_temporarily_rejected_when_the_book_is_busy_or_armed() {
        for (busy, armed) in [(true, false), (false, true), (true, true)] {
            let mut mt = MotorTest::new();
            let d = mt.admit(&spin(1.0, 5.0, 1.0), busy, armed, 10);
            assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
            assert_eq!((d.cmd, d.text), (None, None));
            assert!(!mt.spin_active(11), "a refused spin is not a spin");
            // And it took no seq.
            assert_eq!(mt.seq, 0);
        }
        // With the real book: a long action in it.
        let mut b = ActionBook::new();
        let mut mt = MotorTest::new();
        assert!(!b.busy(0));
        admit(&mut b, &req(CAL, p(&[(0, 1.0)]), GCS), false, false, 0);
        assert!(b.busy(1));
        let d = mt.admit(&spin(1.0, 5.0, 1.0), b.busy(1), false, 1_000);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_TEMPORARILY_REJECTED);
        // After the action's timeout and its final ACK the books are free.
        assert!(b.busy(CALIBRATION_MAX_MS));
        assert!(b.next_ack(0, CALIBRATION_MAX_MS).is_some());
        assert!(!b.busy(CALIBRATION_MAX_MS));
        let d = mt.admit(
            &spin(1.0, 5.0, 1.0),
            b.busy(CALIBRATION_MAX_MS),
            false,
            20_000_000,
        );
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_ACCEPTED);
    }

    #[test]
    fn motor_test_stop_is_accepted_in_every_state() {
        for (busy, armed, spinning) in [
            (false, false, false),
            (true, false, false),
            (false, true, false),
            (true, true, false),
            (false, false, true),
            (true, true, true),
        ] {
            let mut mt = MotorTest::new();
            if spinning {
                mt.admit(&spin(2.0, 5.0, 3.0), false, false, 0);
                assert!(mt.spin_active(1));
            }
            let d = mt.admit(&stop(3.0), busy, armed, 1);
            assert_eq!(d.outcome.result, MavResult::MAV_RESULT_ACCEPTED);
            assert_eq!(d.text, None);
            let cmd = d.cmd.unwrap();
            assert_eq!((cmd.level, cmd.until_us), (0, 0));
            assert!(
                !mt.spin_active(2),
                "a stop ends the spin in core 0's record"
            );
        }
    }

    #[test]
    fn motor_test_spin_during_a_spin_is_accepted_and_replaces_it() {
        let mut mt = MotorTest::new();
        let first = mt
            .admit(&spin(1.0, 5.0, 3.0), false, false, 1_000)
            .cmd
            .unwrap();
        assert!(mt.spin_active(2_000));
        // The same bits again, and another motor, are new commands with new seqs.
        let again = mt.admit(&spin(1.0, 5.0, 3.0), false, false, 2_000);
        assert_eq!(again.outcome.result, MavResult::MAV_RESULT_ACCEPTED);
        let again = again.cmd.unwrap();
        assert_ne!(again.seq, first.seq);
        assert_eq!(again.until_us, 2_000 + 3_000_000);
        let other = mt
            .admit(&spin(3.0, 1.0, 0.5), false, false, 3_000)
            .cmd
            .unwrap();
        assert_ne!(other.seq, again.seq);
        assert_eq!((other.line, other.level), (1, 20));
        // Core 0's record follows the latest spin.
        assert!(mt.spin_active(3_000 + 499_999));
        assert!(!mt.spin_active(3_000 + 500_000));
    }

    #[test]
    fn motor_test_refuses_other_actions_during_a_spin_and_admits_them_after_until() {
        let mut b = ActionBook::new();
        let mut mt = MotorTest::new();
        let now_us = 10_000_000;
        let d = mt.admit(&spin(2.0, 5.0, 2.0), b.busy(now_us / 1000), false, now_us);
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_ACCEPTED);
        // The book holds nothing for a motor test.
        assert!(!b.busy(now_us / 1000));
        let save = req(STO, SAVE, GCS);
        let at = |t_us: u64, b: &mut ActionBook, mt: &MotorTest| {
            b.admit(&save, 0, mt.spin_active(t_us), false, t_us / 1000)
                .outcome
                .result
        };
        assert_eq!(
            at(now_us + 1, &mut b, &mt),
            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
        );
        assert_eq!(
            at(now_us + 1_999_999, &mut b, &mt),
            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
        );
        assert_eq!(
            at(now_us + 2_000_000, &mut b, &mt),
            MavResult::MAV_RESULT_IN_PROGRESS
        );
        // A stop frees the others at once.
        let mut b = ActionBook::new();
        let mut mt = MotorTest::new();
        mt.admit(&spin(2.0, 5.0, 3.0), false, false, now_us);
        assert_eq!(
            at(now_us + 1, &mut b, &mt),
            MavResult::MAV_RESULT_TEMPORARILY_REJECTED
        );
        mt.admit(&stop(2.0), false, false, now_us + 2);
        assert_eq!(
            at(now_us + 3, &mut b, &mt),
            MavResult::MAV_RESULT_IN_PROGRESS
        );
        // And unsupported or denied commands keep their answers during a spin (classification first).
        let mut b = ActionBook::new();
        let mut mt = MotorTest::new();
        mt.admit(&spin(2.0, 5.0, 3.0), false, false, now_us);
        let d = b.admit(
            &req(MavCmd::MAV_CMD_NAV_TAKEOFF, [0.0; 7], GCS),
            0,
            mt.spin_active(now_us + 1),
            false,
            now_us / 1000,
        );
        assert_eq!(d.outcome.result, MavResult::MAV_RESULT_UNSUPPORTED);
    }

    #[test]
    fn motor_test_seq_counts_from_one_skips_zero_and_never_repeats_across_a_wrap() {
        let mut mt = MotorTest::new();
        let first = mt.admit(&spin(1.0, 1.0, 1.0), false, false, 0).cmd.unwrap();
        assert_eq!(first.seq, 1);
        assert_ne!(first.seq, TestCmd::INITIAL.seq);
        // Spin and stop alternate, each with a new seq.
        let mut last = first.seq;
        for i in 0..10u32 {
            let params = if i % 2 == 0 {
                stop(1.0)
            } else {
                spin(1.0, 1.0, 1.0)
            };
            let cmd = mt.admit(&params, false, false, 0).cmd.unwrap();
            assert_eq!(cmd.seq, last + 1);
            last = cmd.seq;
        }
        // Wrap: 0 is skipped, and no seq within a window of the wrap repeats.
        let mut mt = MotorTest::new();
        mt.seq = u32::MAX - 3;
        let mut seen = Vec::new();
        for i in 0..8u32 {
            let params = if i % 3 == 0 {
                stop(2.0)
            } else {
                spin(2.0, 1.0, 1.0)
            };
            seen.push(mt.admit(&params, false, false, 0).cmd.unwrap().seq);
        }
        assert_eq!(
            seen,
            std::vec![u32::MAX - 2, u32::MAX - 1, u32::MAX, 1, 2, 3, 4, 5]
        );
        assert!(!seen.contains(&0));
        // Two neighbours differ always (a change under the same seq would be honoured live).
        assert!(seen.windows(2).all(|w| w[0] != w[1]));
    }
}
