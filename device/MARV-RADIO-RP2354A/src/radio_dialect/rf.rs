use common::protocol::hilink::{self, rf as common_rf};

use crate::channels::{HILINK_BRIDGE_FRAME_BYTES, HilinkBridgeFrame};

pub use common_rf::{DecodedRfPacket, RfError as RfDialectError, RfMsgType, RfWirePayload};

pub const MAX_RF_PAYLOAD_LEN: usize = hilink::LoRaFlightSnapshotPayload::WIRE_LEN;
pub const MAX_RF_PACKET_LEN: usize = encoded_rf_len(MAX_RF_PAYLOAD_LEN);

const _: () = assert!(MAX_RF_PACKET_LEN <= HILINK_BRIDGE_FRAME_BYTES);

pub const fn encoded_rf_len(payload_len: usize) -> usize {
    common_rf::encoded_rf_len(payload_len)
}

pub fn encode_rf_packet<P: RfWirePayload>(
    payload: &P,
    out: &mut [u8],
) -> Result<usize, RfDialectError> {
    common_rf::encode_rf_packet(payload, out)
}

pub fn decode_rf_packet(input: &[u8]) -> Result<DecodedRfPacket<'_>, RfDialectError> {
    if input.len() > MAX_RF_PACKET_LEN {
        return Err(RfDialectError::InvalidPayloadLength);
    }
    common_rf::decode_rf_packet(input)
}

pub fn decode_rf_payload<P: RfWirePayload>(packet: &DecodedRfPacket<'_>) -> hilink::Result<P> {
    common_rf::decode_rf_payload(packet).map_err(|_| hilink::Error::InvalidPayloadLength)
}

pub fn encode_rf_frame<P: RfWirePayload>(payload: &P) -> Result<HilinkBridgeFrame, RfDialectError> {
    let mut frame = HilinkBridgeFrame::new();
    let len = encode_rf_packet(payload, &mut frame.bytes[..HILINK_BRIDGE_FRAME_BYTES])?;
    frame.len = len;
    Ok(frame)
}
