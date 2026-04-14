//! Shared HIL event helpers for portable runtime state.

use crate::messages::runtime::FlightPhase;
use crate::services::hil::model::{HilEgressMessage, HilMissionEvent};
use crate::utilities::time::MeasurementTimestamp;

pub const MAV_CMD_MARV_RUNTIME_PHASE_EVENT: u16 = 31_100;

pub const fn runtime_phase_event(
    timestamp: MeasurementTimestamp,
    phase: FlightPhase,
) -> HilEgressMessage {
    HilEgressMessage::MissionEvent(HilMissionEvent {
        timestamp,
        command_id: MAV_CMD_MARV_RUNTIME_PHASE_EVENT,
        params: [phase.wire_code() as f32, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    })
}

#[cfg(test)]
mod tests {
    use super::{MAV_CMD_MARV_RUNTIME_PHASE_EVENT, runtime_phase_event};
    use crate::messages::runtime::FlightPhase;
    use crate::services::hil::model::HilEgressMessage;
    use crate::utilities::time::MeasurementTimestamp;

    #[test]
    fn runtime_phase_event_encodes_phase_wire_code() {
        let timestamp = MeasurementTimestamp::from_micros(42);
        let event = runtime_phase_event(timestamp, FlightPhase::Ready);

        match event {
            HilEgressMessage::MissionEvent(event) => {
                assert_eq!(event.timestamp, timestamp);
                assert_eq!(event.command_id, MAV_CMD_MARV_RUNTIME_PHASE_EVENT);
                assert_eq!(event.params[0], FlightPhase::Ready.wire_code() as f32);
            }
            other => panic!("unexpected event: {other:?}"),
        }
    }
}
