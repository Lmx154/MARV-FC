use common::protocol::hilink;

use crate::channels::HilinkBridgePriority;

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub enum RadioPriority {
    P0Critical,
    P1Command,
    P2Event,
    P3Snapshot,
    P4Background,
    #[allow(dead_code)]
    P5Debug,
}

impl RadioPriority {
    pub const fn into_bridge_priority(self) -> HilinkBridgePriority {
        match self {
            Self::P0Critical => HilinkBridgePriority::P0Critical,
            Self::P1Command => HilinkBridgePriority::P1Command,
            Self::P2Event => HilinkBridgePriority::P2Event,
            Self::P3Snapshot => HilinkBridgePriority::P3Snapshot,
            Self::P4Background | Self::P5Debug => HilinkBridgePriority::P4Background,
        }
    }
}

pub fn classify_normal_msg(packet: &hilink::DecodedPacket<'_>) -> RadioPriority {
    let Ok(msg_type) = packet.header.message_type() else {
        return RadioPriority::P3Snapshot;
    };

    match msg_type {
        hilink::MsgType::Disarm | hilink::MsgType::MotorStop => RadioPriority::P0Critical,
        hilink::MsgType::LoRaCommand => classify_legacy_lora_command(packet),
        hilink::MsgType::Ack
        | hilink::MsgType::Nack
        | hilink::MsgType::Arm
        | hilink::MsgType::Rtl
        | hilink::MsgType::BenchEnable
        | hilink::MsgType::BenchDisable
        | hilink::MsgType::MotorTest
        | hilink::MsgType::MotorSweep
        | hilink::MsgType::DshotCommand
        | hilink::MsgType::ActuatorStatusRequest
        | hilink::MsgType::LoRaCommandAck
        | hilink::MsgType::LoRaSetProfile
        | hilink::MsgType::LoRaRequestSnapshot => RadioPriority::P1Command,
        hilink::MsgType::LoRaFaults | hilink::MsgType::LoRaEvent => RadioPriority::P2Event,
        hilink::MsgType::LoRaFlightSnapshot
        | hilink::MsgType::TelemetrySnapshot
        | hilink::MsgType::HilSensorFrame
        | hilink::MsgType::HilResponseFrame
        | hilink::MsgType::Heartbeat
        | hilink::MsgType::SystemState
        | hilink::MsgType::MotorState
        | hilink::MsgType::EstimatorState => RadioPriority::P3Snapshot,
        hilink::MsgType::LoRaGpsSnapshot
        | hilink::MsgType::LoRaLinkStatus
        | hilink::MsgType::Gps
        | hilink::MsgType::RadioStatus => RadioPriority::P4Background,
        _ => RadioPriority::P3Snapshot,
    }
}

fn classify_legacy_lora_command(packet: &hilink::DecodedPacket<'_>) -> RadioPriority {
    match hilink::decode_payload::<hilink::LoRaCommandPayload>(packet) {
        Ok(command)
            if command.command_id == hilink::lora_command_id::ABORT
                || command.command_id == hilink::lora_command_id::DISARM
                || command.command_id == hilink::lora_command_id::MOTOR_STOP =>
        {
            RadioPriority::P0Critical
        }
        Ok(_) | Err(_) => RadioPriority::P1Command,
    }
}
