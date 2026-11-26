//! The flight step's side of the station's flight commands (`fw_core::flight_cmd`): mailbox request byte to state-machine
//! command, and the answer back as the mailbox answer byte and the COMMAND_ACK result and `result_param2`.

use fw_core::flight_cmd::{ACCEPTED, FlightCommand, Mailbox, request_of};

use crate::hands_off::{Command, CommandResult, Link, StationCommand};

/// The state machine's command for a mailbox request byte; `None` for a byte that is not one.
pub fn command_of(byte: u8) -> Option<StationCommand> {
    let (c, radio) = request_of(byte)?;
    Some(StationCommand {
        command: match c {
            FlightCommand::Start => Command::Start,
            FlightCommand::Pause => Command::Pause,
            FlightCommand::Abort => Command::Abort,
            FlightCommand::Land => Command::Land,
            FlightCommand::Return => Command::Return,
        },
        link: if radio { Link::Radio } else { Link::Usb },
    })
}

/// The mailbox's answer byte: [`ACCEPTED`], or the reason code (`DenyReason::code`); `None` (a step that gave no
/// answer) is code 0, unknown.
pub fn answer_byte(result: Option<CommandResult>) -> u8 {
    match result {
        Some(CommandResult::Accepted) => ACCEPTED,
        Some(CommandResult::Denied(r)) => r.code(),
        None => 0,
    }
}

/// The COMMAND_ACK of an answer byte: (ACCEPTED, 0) or (DENIED, the reason code in `result_param2`).
pub fn ack_of(byte: u8) -> (bool, i32) {
    if byte == ACCEPTED { (true, 0) } else { (false, i32::from(byte)) }
}

/// The state machine's plan id of a plan id from core 0; 0 is the empty list, so no list.
pub fn plan_id_of(plan_id: u32) -> Option<u32> {
    (plan_id != 0).then_some(plan_id)
}

/// The two calls around `Flight::step`: before, the waiting request if any; a byte that is no command is answered code 0 at once.
pub fn take(mailbox: &Mailbox) -> Option<(u32, StationCommand)> {
    let (seq, byte) = mailbox.take()?;
    match command_of(byte) {
        Some(c) => Some((seq, c)),
        None => {
            mailbox.answer(seq, 0);
            None
        }
    }
}

/// After the step that had request `seq`: its answer, tagged with `seq`.
pub fn answer(mailbox: &Mailbox, seq: u32, result: Option<CommandResult>) {
    mailbox.answer(seq, answer_byte(result));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hands_off::{DenyReason, State};
    use fw_core::flight_cmd::request_byte;

    #[test]
    fn every_request_byte_maps_to_its_command_and_link() {
        let all = [
            (FlightCommand::Start, Command::Start),
            (FlightCommand::Pause, Command::Pause),
            (FlightCommand::Abort, Command::Abort),
            (FlightCommand::Land, Command::Land),
            (FlightCommand::Return, Command::Return),
        ];
        for (f, c) in all {
            assert_eq!(command_of(request_byte(f, true)), Some(StationCommand { command: c, link: Link::Radio }));
            assert_eq!(command_of(request_byte(f, false)), Some(StationCommand { command: c, link: Link::Usb }));
        }
        assert_eq!(command_of(0), None);
        let m = Mailbox::new();
        let s = m.post(0x07).unwrap();
        assert_eq!(take(&m), None, "not a command: not applied");
        assert_eq!(m.poll(s), Some(0), "answered code 0");
    }

    #[test]
    fn the_ack_carries_accepted_or_denied_with_every_reason_code() {
        assert_eq!(ack_of(answer_byte(Some(CommandResult::Accepted))), (true, 0));
        assert_eq!(ack_of(answer_byte(None)), (false, 0));
        let reasons = [
            (DenyReason::NotRadioLink, 1),
            (DenyReason::HandsOffDisabled, 3),
            (DenyReason::NotInThisState, 4),
            (DenyReason::StationAbsent, 5),
            (DenyReason::NoWaypoints, 6),
            (DenyReason::HeightOutsideLimits, 7),
            (DenyReason::NoHome, 8),
            (DenyReason::VehicleBeyondRange, 9),
            (DenyReason::WaypointDistanceUnknown, 10),
            (DenyReason::WaypointBeyondRange, 11),
            (DenyReason::HeadingNotOk, 12),
            (DenyReason::NotT3, 13),
            (DenyReason::BatteryNotOk, 14),
            (DenyReason::ArmSwitchOff, 15),
            (DenyReason::SwitchNotHigh, 16),
            (DenyReason::BatteryCritical, 17),
            (DenyReason::LinkLost, 18),
            (DenyReason::MissionSwitchOff, 19),
        ];
        for (r, code) in reasons {
            assert_eq!(ack_of(answer_byte(Some(CommandResult::Denied(r)))), (false, code), "{r:?}");
        }
    }

    #[test]
    fn plan_id_of_0_is_no_list_and_any_other_is_itself() {
        assert_eq!(plan_id_of(0), None);
        assert_eq!(plan_id_of(0x1862_F4A2), Some(0x1862_F4A2));
        assert_eq!(plan_id_of(u32::MAX), Some(u32::MAX));
    }

    #[test]
    fn mode_codes_of_the_contract_for_all_nine_values() {
        let states = [
            (State::Pilot, 1),
            (State::Takeover, 2),
            (State::Hold, 3),
            (State::Auto, 4),
            (State::Return, 5),
            (State::Land, 6),
            (State::Stop, 7),
            (State::Blind, 8),
        ];
        for (s, code) in states {
            assert_eq!(s.code(), code, "{s:?}");
            assert_ne!(s.code(), 0, "0 is reserved for not reported");
        }
    }
}
