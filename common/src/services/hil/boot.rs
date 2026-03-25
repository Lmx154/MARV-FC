//! Shared HIL boot and USB-mode coordination.

use crate::messages::runtime::FlightPhase;
use crate::policies::modes::evaluate_init_hil_command;
use crate::services::hil::model::{HilCommandAck, HilControlCommand};

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum UsbHilMode {
    InitProbe,
    HilActive,
    Passive,
}

impl UsbHilMode {
    pub const fn is_hil_active(self) -> bool {
        matches!(self, Self::HilActive)
    }
}

pub const fn usb_hil_mode_for_phase(phase: FlightPhase) -> UsbHilMode {
    match phase {
        FlightPhase::Init => UsbHilMode::InitProbe,
        FlightPhase::Hil => UsbHilMode::HilActive,
        FlightPhase::Ready | FlightPhase::Active | FlightPhase::Fault => UsbHilMode::Passive,
    }
}

pub const fn should_report_init_probe_liveness(mode: UsbHilMode) -> bool {
    matches!(mode, UsbHilMode::InitProbe)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InitControlCommandOutcome {
    pub ack: Option<HilCommandAck>,
    pub enter_hil: bool,
}

pub fn evaluate_usb_init_control_command(
    command: HilControlCommand,
    system_id: u8,
    component_id: u8,
    mode: UsbHilMode,
    host_connected: bool,
    boot_elapsed_ms: u32,
    hil_boot_window_ms: u32,
) -> InitControlCommandOutcome {
    let elapsed_ms = match mode {
        UsbHilMode::InitProbe if host_connected => 0,
        UsbHilMode::InitProbe => boot_elapsed_ms,
        UsbHilMode::HilActive | UsbHilMode::Passive => hil_boot_window_ms.saturating_add(1),
    };
    let decision = evaluate_init_hil_command(
        command,
        system_id,
        component_id,
        elapsed_ms,
        hil_boot_window_ms,
    );

    InitControlCommandOutcome {
        ack: decision.into_ack(command),
        enter_hil: decision.enter_hil,
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HilBootDecision {
    Continue,
    EnterHil,
    SelectReal,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HilBootSelector {
    waiting_on_usb_host: bool,
}

impl HilBootSelector {
    pub const fn new() -> Self {
        Self {
            waiting_on_usb_host: false,
        }
    }

    pub fn update(
        &mut self,
        hil_requested: bool,
        usb_host_connected: bool,
        boot_elapsed_ms: u32,
        hil_boot_window_ms: u32,
    ) -> HilBootDecision {
        if hil_requested {
            return HilBootDecision::EnterHil;
        }

        if usb_host_connected {
            self.waiting_on_usb_host = true;
        }

        if self.waiting_on_usb_host && !usb_host_connected {
            HilBootDecision::SelectReal
        } else if !self.waiting_on_usb_host && boot_elapsed_ms >= hil_boot_window_ms {
            HilBootDecision::SelectReal
        } else {
            HilBootDecision::Continue
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        HilBootDecision, HilBootSelector, UsbHilMode, evaluate_usb_init_control_command,
        should_report_init_probe_liveness, usb_hil_mode_for_phase,
    };
    use crate::messages::runtime::FlightPhase;
    use crate::services::hil::{HilControlCommand, SensorBackend};

    #[test]
    fn flight_phase_maps_to_usb_mode() {
        assert_eq!(
            usb_hil_mode_for_phase(FlightPhase::Init),
            UsbHilMode::InitProbe
        );
        assert_eq!(
            usb_hil_mode_for_phase(FlightPhase::Hil),
            UsbHilMode::HilActive
        );
        assert_eq!(
            usb_hil_mode_for_phase(FlightPhase::Ready),
            UsbHilMode::Passive
        );
    }

    #[test]
    fn init_probe_liveness_only_reports_during_init_probe() {
        assert!(should_report_init_probe_liveness(UsbHilMode::InitProbe));
        assert!(!should_report_init_probe_liveness(UsbHilMode::HilActive));
        assert!(!should_report_init_probe_liveness(UsbHilMode::Passive));
    }

    #[test]
    fn connected_host_keeps_init_command_inside_boot_window() {
        let outcome = evaluate_usb_init_control_command(
            HilControlCommand::request_backend(31_010, SensorBackend::Hil, 7, 9, 42, 3, 0),
            42,
            3,
            UsbHilMode::InitProbe,
            true,
            5_000,
            1_500,
        );

        assert!(outcome.enter_hil);
        assert!(outcome.ack.is_some());
    }

    #[test]
    fn boot_selector_exits_hil_window_without_usb_host() {
        let mut selector = HilBootSelector::new();
        assert_eq!(
            selector.update(false, false, 1_499, 1_500),
            HilBootDecision::Continue
        );
        assert_eq!(
            selector.update(false, false, 1_500, 1_500),
            HilBootDecision::SelectReal
        );
    }

    #[test]
    fn boot_selector_waits_for_host_disconnect_once_connected() {
        let mut selector = HilBootSelector::new();
        assert_eq!(
            selector.update(false, true, 10, 1_500),
            HilBootDecision::Continue
        );
        assert_eq!(
            selector.update(false, true, 5_000, 1_500),
            HilBootDecision::Continue
        );
        assert_eq!(
            selector.update(false, false, 5_100, 1_500),
            HilBootDecision::SelectReal
        );
    }

    #[test]
    fn boot_selector_prioritizes_hil_request() {
        let mut selector = HilBootSelector::new();
        assert_eq!(
            selector.update(true, false, 0, 1_500),
            HilBootDecision::EnterHil
        );
    }
}
