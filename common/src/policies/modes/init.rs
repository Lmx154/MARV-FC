//! Shared `INIT` transition rules.

use crate::messages::runtime::FlightPhase;
use crate::services::hil::model::{
    HilCommandAck, HilCommandAckResult, HilControlAction, HilControlCommand,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InitHilCommandDecision {
    pub ack_result: Option<HilCommandAckResult>,
    pub enter_hil: bool,
}

impl InitHilCommandDecision {
    pub const fn ignored() -> Self {
        Self {
            ack_result: None,
            enter_hil: false,
        }
    }

    pub const fn into_ack(self, command: HilControlCommand) -> Option<HilCommandAck> {
        match self.ack_result {
            Some(result) => Some(HilCommandAck::new(
                command.command_id,
                result,
                command.source_system,
                command.source_component,
            )),
            None => None,
        }
    }
}

pub fn evaluate_init_hil_command(
    command: HilControlCommand,
    system_id: u8,
    component_id: u8,
    boot_elapsed_ms: u32,
    hil_boot_window_ms: u32,
) -> InitHilCommandDecision {
    let system_matches = command.target_system == 0 || command.target_system == system_id;
    let component_matches =
        command.target_component == 0 || command.target_component == component_id;

    if !(system_matches && component_matches) {
        return InitHilCommandDecision::ignored();
    }

    match command.action {
        HilControlAction::EnterHilMode => {
            if boot_elapsed_ms <= hil_boot_window_ms {
                InitHilCommandDecision {
                    ack_result: Some(HilCommandAckResult::Accepted),
                    enter_hil: true,
                }
            } else {
                InitHilCommandDecision {
                    ack_result: Some(HilCommandAckResult::TemporarilyRejected),
                    enter_hil: false,
                }
            }
        }
        HilControlAction::SelectSubmode(_) => InitHilCommandDecision {
            ack_result: Some(HilCommandAckResult::TemporarilyRejected),
            enter_hil: false,
        },
        HilControlAction::InvalidPayload => InitHilCommandDecision {
            ack_result: Some(HilCommandAckResult::Unsupported),
            enter_hil: false,
        },
    }
}

pub const fn phase_after_init(feed_critical_ready: bool) -> FlightPhase {
    if feed_critical_ready {
        FlightPhase::Ready
    } else {
        FlightPhase::Fault
    }
}

#[cfg(test)]
mod tests {
    use super::{evaluate_init_hil_command, phase_after_init};
    use crate::messages::runtime::FlightPhase;
    use crate::services::hil::model::HilCommandAckResult;
    use crate::services::hil::{HilControlCommand, HilSubmode};

    #[test]
    fn accepts_targeted_hil_request_during_boot_window() {
        let decision = evaluate_init_hil_command(
            HilControlCommand::enter_hil_mode(31_010, 7, 9, 42, 3, 0),
            42,
            3,
            500,
            1_500,
        );

        assert_eq!(decision.ack_result, Some(HilCommandAckResult::Accepted));
        assert!(decision.enter_hil);
    }

    #[test]
    fn ignores_commands_for_other_targets() {
        let decision = evaluate_init_hil_command(
            HilControlCommand::enter_hil_mode(31_010, 7, 9, 24, 3, 0),
            42,
            3,
            500,
            1_500,
        );

        assert_eq!(decision.ack_result, None);
        assert!(!decision.enter_hil);
    }

    #[test]
    fn rejects_late_hil_request_after_init_window() {
        let decision = evaluate_init_hil_command(
            HilControlCommand::enter_hil_mode(31_010, 7, 9, 42, 3, 0),
            42,
            3,
            1_501,
            1_500,
        );

        assert_eq!(
            decision.ack_result,
            Some(HilCommandAckResult::TemporarilyRejected)
        );
        assert!(!decision.enter_hil);
    }

    #[test]
    fn rejects_submode_selection_before_hil_phase() {
        let decision = evaluate_init_hil_command(
            HilControlCommand::select_submode(31_011, HilSubmode::FullRun, 7, 9, 42, 3, 0),
            42,
            3,
            500,
            1_500,
        );

        assert_eq!(
            decision.ack_result,
            Some(HilCommandAckResult::TemporarilyRejected)
        );
        assert!(!decision.enter_hil);
    }

    #[test]
    fn init_health_maps_to_ready_or_fault() {
        assert_eq!(phase_after_init(true), FlightPhase::Ready);
        assert_eq!(phase_after_init(false), FlightPhase::Fault);
    }
}
