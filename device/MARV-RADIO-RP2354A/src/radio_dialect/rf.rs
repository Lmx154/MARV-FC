use common::protocol::hilink::{self, WirePayload};

use crate::channels::{HILINK_BRIDGE_FRAME_BYTES, HilinkBridgeFrame};

pub const RF_TYPE_LEN: usize = 1;
pub const RF_CRC_LEN: usize = 2;
pub const RF_PACKET_OVERHEAD_LEN: usize = RF_TYPE_LEN + RF_CRC_LEN;
pub const MAX_RF_PAYLOAD_LEN: usize = hilink::LoRaFlightSnapshotPayload::WIRE_LEN;
pub const MAX_RF_PACKET_LEN: usize = encoded_rf_len(MAX_RF_PAYLOAD_LEN);

const _: () = assert!(MAX_RF_PACKET_LEN <= HILINK_BRIDGE_FRAME_BYTES);

#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RfMsgType {
    LoRaFlightSnapshot = 1,
    LoRaGpsSnapshot = 2,
    LoRaEvent = 3,
    LoRaFaults = 4,
    LoRaLinkStatus = 5,
    LoRaCommand = 16,
    LoRaCommandAck = 17,
    LoRaSetProfile = 18,
    LoRaRequestSnapshot = 19,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RfDialectError {
    BufferTooSmall,
    CrcMismatch,
    InvalidPayloadLength,
    UnknownRfMsgType,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DecodedRfPacket<'a> {
    pub msg_type: RfMsgType,
    pub payload: &'a [u8],
}

pub trait RfPayload: WirePayload {
    const RF_MSG_TYPE: RfMsgType;
}

impl RfPayload for hilink::LoRaFlightSnapshotPayload {
    const RF_MSG_TYPE: RfMsgType = RfMsgType::LoRaFlightSnapshot;
}

impl RfPayload for hilink::LoRaGpsSnapshotPayload {
    const RF_MSG_TYPE: RfMsgType = RfMsgType::LoRaGpsSnapshot;
}

impl RfPayload for hilink::LoRaEventPayload {
    const RF_MSG_TYPE: RfMsgType = RfMsgType::LoRaEvent;
}

impl RfPayload for hilink::LoRaFaultsPayload {
    const RF_MSG_TYPE: RfMsgType = RfMsgType::LoRaFaults;
}

impl RfPayload for hilink::LoRaLinkStatusPayload {
    const RF_MSG_TYPE: RfMsgType = RfMsgType::LoRaLinkStatus;
}

impl RfPayload for hilink::LoRaCommandPayload {
    const RF_MSG_TYPE: RfMsgType = RfMsgType::LoRaCommand;
}

impl RfPayload for hilink::LoRaCommandAckPayload {
    const RF_MSG_TYPE: RfMsgType = RfMsgType::LoRaCommandAck;
}

impl RfPayload for hilink::LoRaSetProfilePayload {
    const RF_MSG_TYPE: RfMsgType = RfMsgType::LoRaSetProfile;
}

impl RfPayload for hilink::LoRaRequestSnapshotPayload {
    const RF_MSG_TYPE: RfMsgType = RfMsgType::LoRaRequestSnapshot;
}

impl TryFrom<u8> for RfMsgType {
    type Error = RfDialectError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::LoRaFlightSnapshot),
            2 => Ok(Self::LoRaGpsSnapshot),
            3 => Ok(Self::LoRaEvent),
            4 => Ok(Self::LoRaFaults),
            5 => Ok(Self::LoRaLinkStatus),
            16 => Ok(Self::LoRaCommand),
            17 => Ok(Self::LoRaCommandAck),
            18 => Ok(Self::LoRaSetProfile),
            19 => Ok(Self::LoRaRequestSnapshot),
            _ => Err(RfDialectError::UnknownRfMsgType),
        }
    }
}

impl RfMsgType {
    pub const fn payload_len(self) -> usize {
        match self {
            Self::LoRaFlightSnapshot => hilink::LoRaFlightSnapshotPayload::WIRE_LEN,
            Self::LoRaGpsSnapshot => hilink::LoRaGpsSnapshotPayload::WIRE_LEN,
            Self::LoRaEvent => hilink::LoRaEventPayload::WIRE_LEN,
            Self::LoRaFaults => hilink::LoRaFaultsPayload::WIRE_LEN,
            Self::LoRaLinkStatus => hilink::LoRaLinkStatusPayload::WIRE_LEN,
            Self::LoRaCommand => hilink::LoRaCommandPayload::WIRE_LEN,
            Self::LoRaCommandAck => hilink::LoRaCommandAckPayload::WIRE_LEN,
            Self::LoRaSetProfile => hilink::LoRaSetProfilePayload::WIRE_LEN,
            Self::LoRaRequestSnapshot => hilink::LoRaRequestSnapshotPayload::WIRE_LEN,
        }
    }
}

pub const fn encoded_rf_len(payload_len: usize) -> usize {
    RF_PACKET_OVERHEAD_LEN + payload_len
}

pub fn encode_rf_packet<P: RfPayload>(
    payload: &P,
    out: &mut [u8],
) -> Result<usize, RfDialectError> {
    let len = encoded_rf_len(P::WIRE_LEN);
    if out.len() < len {
        return Err(RfDialectError::BufferTooSmall);
    }

    out[0] = P::RF_MSG_TYPE as u8;
    let written = payload
        .encode_payload(&mut out[1..1 + P::WIRE_LEN])
        .map_err(|_| RfDialectError::InvalidPayloadLength)?;
    if written != P::WIRE_LEN {
        return Err(RfDialectError::InvalidPayloadLength);
    }

    let crc = hilink::crc16_ccitt_false(&out[..1 + P::WIRE_LEN]);
    out[1 + P::WIRE_LEN..len].copy_from_slice(&crc.to_le_bytes());
    Ok(len)
}

pub fn decode_rf_packet(input: &[u8]) -> Result<DecodedRfPacket<'_>, RfDialectError> {
    if input.len() < RF_PACKET_OVERHEAD_LEN {
        return Err(RfDialectError::InvalidPayloadLength);
    }

    let msg_type = RfMsgType::try_from(input[0])?;
    let expected_len = encoded_rf_len(msg_type.payload_len());
    if input.len() != expected_len || input.len() > MAX_RF_PACKET_LEN {
        return Err(RfDialectError::InvalidPayloadLength);
    }

    let packet_len = input.len() - RF_CRC_LEN;
    let stored_crc = u16::from_le_bytes([input[packet_len], input[packet_len + 1]]);
    let computed_crc = hilink::crc16_ccitt_false(&input[..packet_len]);
    if stored_crc != computed_crc {
        return Err(RfDialectError::CrcMismatch);
    }

    Ok(DecodedRfPacket {
        msg_type,
        payload: &input[1..packet_len],
    })
}

pub fn decode_rf_payload<P: RfPayload>(packet: &DecodedRfPacket<'_>) -> hilink::Result<P> {
    if packet.msg_type != P::RF_MSG_TYPE {
        return Err(hilink::Error::InvalidPayloadLength);
    }
    if packet.payload.len() != P::WIRE_LEN {
        return Err(hilink::Error::InvalidPayloadLength);
    }

    P::decode_payload(packet.payload)
}

pub fn encode_rf_frame<P: RfPayload>(payload: &P) -> Result<HilinkBridgeFrame, RfDialectError> {
    let mut frame = HilinkBridgeFrame::new();
    let len = encode_rf_packet(payload, &mut frame.bytes[..HILINK_BRIDGE_FRAME_BYTES])?;
    frame.len = len;
    Ok(frame)
}
