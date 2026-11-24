//! The station's five flight commands (link-contract section 10, Amendments A and C) between the links on core 0 and
//! the hands-off state machine in the flight step on core 1, through one `fw_core::flight_cmd::Mailbox`.
//!
//! Core 0 ([`command`], from `actions::command` on either link): the decode table; a flight command is posted and its
//! answer awaited inside the node's command hook, so the one COMMAND_ACK the contract allows ("at once", ACCEPTED or
//! DENIED, never IN_PROGRESS, TEMPORARILY_REJECTED or FAILED) carries the flight step's answer. mavlink-device dfdb11a
//! sends the hook's outcome as the ACK at once and has no way to send none, so the answer cannot come later as a
//! `LateAck` without a second ACK. The wait is bounded: [`BOUND_US`] for the flight step to take the request, then as
//! long again for its answer. No answer: DENIED, code 0. Another flight command outstanding (the other link, which can
//! preempt this one on core 0): DENIED, code 0, at once.
//!
//! Core 1 ([`before_step`], [`after_step`], from `flight::Stepper::sample`): the request, taken at most once, given to
//! `Flight::set_command` for the step that follows; that step's answer back, tagged with the request's number.

use fw_core::flight_cmd::{Decoded, Exchange, Mailbox, decode, exchange, request_byte};
use fw_flight::gnc::{Allocator, Controller, Estimator};
use fw_flight::step::Flight;
use mavlink_device::marv::MavResult;
use mavlink_device::{CommandOutcome, CommandRequest};

/// Core 0 -> flight step and back: the one flight command in flight.
pub static FLIGHT_CMD: Mailbox = Mailbox::new();

/// How long core 0 waits for the flight step to take a command, us (then as long again for its answer): 3.2 IMU1 steps
/// at 800 Hz.
pub const BOUND_US: u64 = 4_000;

/// Core 0: the COMMAND_ACK of one of the five flight commands; `None` for every other command (it goes on as before).
pub fn command(req: &CommandRequest, link: u8) -> Option<CommandOutcome> {
    let c = match decode(req.command, &req.params) {
        Decoded::Other => return None,
        Decoded::BadParams => {
            defmt::info!(
                "[INFO] flight command {=u32} on link {=u8}: DENIED, parameters not in the table",
                req.command as u32,
                link
            );
            return Some(outcome(0));
        }
        Decoded::Command(c) => c,
    };
    let radio = link == crate::LINK_RADIO as u8;
    let byte = match exchange(&FLIGHT_CMD, request_byte(c, radio), BOUND_US, crate::now_us) {
        Exchange::Answered(b) => b,
        Exchange::Busy => {
            defmt::warn!("[INFO] flight command {=u32} on link {=u8}: DENIED, another one outstanding", req.command as u32, link);
            0
        }
        Exchange::Timeout { taken } => {
            defmt::warn!(
                "[INFO] flight command {=u32} on link {=u8}: DENIED, no answer from the flight step (taken {=bool})",
                req.command as u32,
                link,
                taken
            );
            0
        }
    };
    defmt::info!(
        "[INFO] flight command {=u32} on link {=u8} from {=u8}/{=u8}: answer {=u8}",
        req.command as u32,
        link,
        req.sender.system_id,
        req.sender.component_id,
        byte
    );
    Some(outcome(byte))
}

fn outcome(byte: u8) -> CommandOutcome {
    let (accepted, code) = fw_flight::station::ack_of(byte);
    CommandOutcome {
        result: if accepted {
            MavResult::MAV_RESULT_ACCEPTED
        } else {
            MavResult::MAV_RESULT_DENIED
        },
        progress: 0,
        result_param2: code,
    }
}

/// Core 1, before `Flight::step`: the waiting request, given to the flight for that step; its number.
pub fn before_step<E: Estimator, C: Controller, A: Allocator>(flight: &mut Flight<E, C, A>) -> Option<u32> {
    let (seq, c) = fw_flight::station::take(&FLIGHT_CMD)?;
    flight.set_command(c);
    Some(seq)
}

/// Core 1, after that step: its answer to request `seq`.
pub fn after_step<E: Estimator, C: Controller, A: Allocator>(flight: &mut Flight<E, C, A>, seq: Option<u32>) {
    if let Some(seq) = seq {
        fw_flight::station::answer(&FLIGHT_CMD, seq, flight.take_command_result());
    }
}
