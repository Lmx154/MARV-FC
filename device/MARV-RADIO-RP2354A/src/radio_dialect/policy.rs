use common::protocol::hilink;

use crate::channels::HilinkBridgePriority;

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub enum RadioPriority {
    P0Critical,
    P1Command,
    #[allow(dead_code)]
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

pub const DEFAULT_COMMAND_EXPIRES_MS: u16 = 2_000;

pub fn classify_normal_msg(packet: &hilink::DecodedPacket<'_>) -> RadioPriority {
    let Ok(msg_type) = packet.header.message_type() else {
        return RadioPriority::P3Snapshot;
    };

    match msg_type {
        hilink::MsgType::Disarm | hilink::MsgType::MotorStop => RadioPriority::P0Critical,
        hilink::MsgType::Ack
        | hilink::MsgType::Nack
        | hilink::MsgType::Arm
        | hilink::MsgType::Rtl
        | hilink::MsgType::BenchEnable
        | hilink::MsgType::BenchDisable
        | hilink::MsgType::MotorTest
        | hilink::MsgType::MotorSweep
        | hilink::MsgType::DshotCommand
        | hilink::MsgType::ActuatorStatusRequest => RadioPriority::P1Command,
        hilink::MsgType::TelemetrySnapshot
        | hilink::MsgType::HilSensorFrame
        | hilink::MsgType::HilResponseFrame
        | hilink::MsgType::Heartbeat
        | hilink::MsgType::SystemState
        | hilink::MsgType::MotorState
        | hilink::MsgType::EstimatorState => RadioPriority::P3Snapshot,
        hilink::MsgType::Gps | hilink::MsgType::RadioStatus => RadioPriority::P4Background,
        _ => RadioPriority::P3Snapshot,
    }
}

pub fn normal_command_to_lora_command_id(msg_type: hilink::MsgType) -> Option<u16> {
    match msg_type {
        hilink::MsgType::Arm => Some(hilink::lora_command_id::ARM),
        hilink::MsgType::Disarm => Some(hilink::lora_command_id::DISARM),
        hilink::MsgType::MotorStop => Some(hilink::lora_command_id::MOTOR_STOP),
        hilink::MsgType::Ping => Some(hilink::lora_command_id::PING),
        _ => None,
    }
}

pub const fn command_flags(command_id: u16) -> u16 {
    match command_id {
        hilink::lora_command_id::DISARM | hilink::lora_command_id::MOTOR_STOP => {
            hilink::lora_command_flags::URGENT | hilink::lora_command_flags::ALLOW_WHILE_FAILSAFE
        }
        _ => 0,
    }
}

pub const fn lora_ack_status_is_ack(status: u8) -> bool {
    status == hilink::lora_command_status::ACCEPTED
        || status == hilink::lora_command_status::DUPLICATE_ACCEPTED
}
