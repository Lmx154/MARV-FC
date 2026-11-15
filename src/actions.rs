//! The firmware's commands: the one `fw_core::admission::ActionBook`, the action each admitted
//! command starts, and how each action ends. Link id 0 is USB; `armed` and `hil` are false until
//! the arming code and the HIL source exist.
//!
//! Built: restart of the barometer and of IMU1, parameters to defaults (all, calibration), save to
//! and reload from the flash settings region (`settings`), reboot (the watchdog monitor resets the
//! board 100 ms after the final ACK was handed to the link; nothing is saved), the gyro and the
//! one-position accelerometer and the compass calibrations (`calibration`). Every other action
//! the book admits ends FAILED at once with one STATUSTEXT "<what>: not built yet".

use core::cell::{Cell, RefCell};

use core::sync::atomic::Ordering;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use fw_core::admission::{Action, ActionBook, Effect};
use fw_core::calib::Kind;

use fw_core::watchdog::task;
use mavlink_device::dialect::MavSeverity;
use mavlink_device::marv::{MarvDevice, MavCmd, MavResult};
use mavlink_device::{CommandOutcome, CommandRequest, LateAck};

use crate::calibration::Step;
use crate::device::{find, raise_apply};
use crate::store;

/// The link id of USB in the books.
const USB: u8 = 0;

static BOOK: Mutex<CriticalSectionRawMutex, RefCell<ActionBook>> =
    Mutex::new(RefCell::new(ActionBook::new()));

/// The number of the restart request the running `DeviceRestart` action raised: the action
/// ends only on that request's outcome (`Device::restart_outcome`), never on an older one.
static RESTART_NUMBER: Mutex<CriticalSectionRawMutex, Cell<u32>> = Mutex::new(Cell::new(0));

fn result_name(result: MavResult) -> &'static str {
    match result {
        MavResult::MAV_RESULT_ACCEPTED => "ACCEPTED",
        MavResult::MAV_RESULT_TEMPORARILY_REJECTED => "TEMPORARILY_REJECTED",
        MavResult::MAV_RESULT_DENIED => "DENIED",
        MavResult::MAV_RESULT_UNSUPPORTED => "UNSUPPORTED",
        MavResult::MAV_RESULT_FAILED => "FAILED",
        MavResult::MAV_RESULT_IN_PROGRESS => "IN_PROGRESS",
        MavResult::MAV_RESULT_CANCELLED => "CANCELLED",
        _ => "other",
    }
}

/// What the action is called in the log and in the "not built yet" text.
fn action_name(action: Action) -> &'static str {
    match action {
        Action::GyroCalibration => "gyro calibration",
        Action::AccelCalibration => "accel calibration",
        Action::MagCalibration => "compass calibration",
        Action::StorageReload => "reload",
        Action::StorageSave => "save",
        Action::StorageDefaultsAll => "defaults",
        Action::StorageDefaultsCalibration => "calibration defaults",
        Action::Reboot => "reboot",
        Action::DeviceRestart(MarvDevice::MARV_DEVICE_IMU1) => "restart IMU1",
        Action::DeviceRestart(MarvDevice::MARV_DEVICE_IMU2) => "restart IMU2",
        Action::DeviceRestart(MarvDevice::MARV_DEVICE_BARO) => "restart BARO",
        Action::DeviceRestart(MarvDevice::MARV_DEVICE_MAG) => "restart MAG",
        Action::DeviceRestart(MarvDevice::MARV_DEVICE_GNSS) => "restart GNSS",
        Action::DeviceRestart(MarvDevice::MARV_DEVICE_BATT) => "restart BATT",
        Action::DeviceRestart(_) => "restart",
        Action::DeviceSelfTest(_) => "self-test",
    }
}

/// `Telemetry::command`: admits the command and starts its action. The answer is the book's
/// (IN_PROGRESS 0 % for an admitted action; its final ACK follows through [`next_ack`]).
pub fn command(req: &CommandRequest, now_ms: u64) -> CommandOutcome {
    let decision = BOOK.lock(|b| b.borrow_mut().admit(req, USB, false, false, now_ms));
    defmt::info!(
        "[INFO] command {=u32} from {=u8}/{=u8}: {=str}{=str}{=str}",
        req.command as u32,
        req.sender.system_id,
        req.sender.component_id,
        result_name(decision.outcome.result),
        if decision.text.is_some() { ", " } else { "" },
        decision.text.unwrap_or("")
    );
    if let Some(reason) = decision.text {
        crate::text(MavSeverity::MAV_SEVERITY_WARNING, reason);
    }
    if let Effect::Start(action) = decision.effect {
        defmt::info!("[INFO] action {=str} starts", action_name(action));
        start(action, now_ms);
    }
    if decision.effect == Effect::CancelCalibration {
        crate::calibration::cancel();
    }
    decision.outcome
}

/// `Telemetry::acks`: the next late ACK for USB.
pub fn next_ack(now_ms: u64) -> Option<LateAck> {
    // The node sends ACKs before it takes texts from `Telemetry::texts`: hold the ACK until the
    // queued text (a calibration's result or failure) has gone out, so the text precedes it.
    if crate::texts_pending() {
        return None;
    }
    let ack = BOOK.lock(|b| b.borrow_mut().next_ack(USB, now_ms))?;
    if ack.outcome.result != MavResult::MAV_RESULT_IN_PROGRESS {
        defmt::info!(
            "[INFO] final ACK of command {=u32}: {=str}",
            ack.command as u32,
            result_name(ack.outcome.result)
        );
        if ack.command == MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
            && ack.outcome.result == MavResult::MAV_RESULT_ACCEPTED
        {
            crate::watchdog::reboot_ack_handed(now_ms as u32);
        }
    }
    Some(ack)
}

/// The USB link went down: no ACK can be delivered for the running action.
pub fn link_lost() {
    BOOK.lock(|b| b.borrow_mut().link_lost(USB));
}

fn start(action: Action, now_ms: u64) {
    let caller = crate::watchdog::RUNNING0.swap(task::ACTION, Ordering::Relaxed);
    run(action, now_ms);
    crate::watchdog::RUNNING0.store(caller, Ordering::Relaxed);
}

fn run(action: Action, now_ms: u64) {
    if let Action::DeviceRestart(id) = action
        && let Some(dev) = find(id)
    {
        let number = dev.request_restart();
        RESTART_NUMBER.lock(|n| n.set(number));
        return;
    }
    match action {
        Action::StorageDefaultsAll => defaults(false, now_ms),
        Action::StorageDefaultsCalibration => defaults(true, now_ms),
        Action::StorageSave => save(),
        Action::StorageReload => reload(),
        Action::Reboot => {
            crate::watchdog::request_reboot(now_ms as u32);
            end(now_ms, action, MavResult::MAV_RESULT_ACCEPTED);
        }
        Action::MagCalibration => {
            if !crate::calibration::start_mag() {
                end(now_ms, action, MavResult::MAV_RESULT_FAILED);
            }
        }
        Action::GyroCalibration | Action::AccelCalibration => {
            let kind = if action == Action::GyroCalibration {
                Kind::Gyro
            } else {
                Kind::Accel
            };
            if !crate::calibration::start(kind) {
                end(now_ms, action, MavResult::MAV_RESULT_FAILED);
            }
        }
        other => {
            crate::text_fmt(
                MavSeverity::MAV_SEVERITY_WARNING,
                format_args!("{}: not built yet", action_name(other)),
            );
            end(now_ms, action, MavResult::MAV_RESULT_FAILED);
        }
    }
}

/// Parameters to their defaults; the devices whose values changed get their APPLY request. Ends
/// ACCEPTED as soon as the store is reset and the requests are raised: it does not wait for the
/// applies (a device that is not Ok picks the values up when it is restarted).
fn defaults(calibration_only: bool, now_ms: u64) {
    let action = if calibration_only {
        Action::StorageDefaultsCalibration
    } else {
        Action::StorageDefaultsAll
    };
    let Some((changed, configure, live)) = store::with(|p| {
        let changed = if calibration_only {
            p.reset_calibration()
        } else {
            p.reset_all()
        };
        (changed, p.take_configure_mask(), p.take_live_mask())
    }) else {
        end(now_ms, action, MavResult::MAV_RESULT_FAILED);
        return;
    };
    defmt::info!(
        "[INFO] action {=str}: {=u16} values changed, apply mask {=u32:#x}, live mask {=u32:#x}",
        action_name(action),
        changed,
        configure,
        live
    );
    raise_apply(configure);
    crate::calibration::raise_push(live);
    end(now_ms, action, MavResult::MAV_RESULT_ACCEPTED);
}

/// Writes the store's values that differ from their defaults to flash (blocking; the flash stall
/// stops both cores). FAILED with one text when it is refused or does not verify.
fn save() {
    let result = match crate::settings::save() {
        Ok(_) => MavResult::MAV_RESULT_ACCEPTED,
        Err(why) => {
            crate::text_fmt(
                MavSeverity::MAV_SEVERITY_WARNING,
                format_args!("save failed: {}", why),
            );
            MavResult::MAV_RESULT_FAILED
        }
    };
    end(now(), Action::StorageSave, result);
}

/// The boot rule again; ACCEPTED also when no valid record exists (defaults, with the boot's text).
fn reload() {
    let result = match crate::settings::reload() {
        Ok(()) => MavResult::MAV_RESULT_ACCEPTED,
        Err(why) => {
            crate::text_fmt(
                MavSeverity::MAV_SEVERITY_WARNING,
                format_args!("reload failed: {}", why),
            );
            MavResult::MAV_RESULT_FAILED
        }
    };
    end(now(), Action::StorageReload, result);
}

fn now() -> u64 {
    embassy_time::Instant::now().as_millis()
}

fn end(now_ms: u64, action: Action, result: MavResult) {
    if BOOK.lock(|b| b.borrow_mut().finish(now_ms, result)) {
        defmt::info!(
            "[INFO] action {=str} ended: {=str}",
            action_name(action),
            result_name(result)
        );
    }
}

/// Every supervisor tick: the books' time limit, the running calibration, and the end of the
/// running restart action.
pub fn tick(now_ms: u64) {
    let (timed_out, running) = BOOK.lock(|b| {
        let mut book = b.borrow_mut();
        (book.tick(now_ms), book.running())
    });
    if let Some(action) = timed_out {
        defmt::warn!(
            "[INFO] action {=str} ended: FAILED (time limit, {=u64} ms)",
            action_name(action),
            action.max_duration_ms()
        );
    }
    match running {
        Some(
            action @ (Action::GyroCalibration | Action::AccelCalibration | Action::MagCalibration),
        ) => match crate::calibration::step() {
            Step::Progress(percent) => {
                BOOK.lock(|b| b.borrow_mut().set_progress(now_ms, percent));
            }
            Step::End(result) => end(now_ms, action, result),
        },
        _ => crate::calibration::idle(),
    }
    if let Some(action @ Action::DeviceRestart(id)) = running
        && let Some(dev) = find(id)
    {
        let number = RESTART_NUMBER.lock(Cell::get);
        let result = match dev.restart_outcome(number) {
            Some(true) => MavResult::MAV_RESULT_ACCEPTED,
            Some(false) => MavResult::MAV_RESULT_FAILED,
            None => return,
        };
        end(now_ms, action, result);
    }
}
