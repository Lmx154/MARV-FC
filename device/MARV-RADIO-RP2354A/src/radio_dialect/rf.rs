use crate::channels::HilinkBridgeFrame;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RfDialectError {
    PayloadTooLarge,
}

pub fn copy_legacy_payload_to_hilink_frame(
    payload: &[u8],
    frame: &mut HilinkBridgeFrame,
) -> Result<(), RfDialectError> {
    if payload.len() > frame.bytes.len() {
        return Err(RfDialectError::PayloadTooLarge);
    }

    frame.bytes[..payload.len()].copy_from_slice(payload);
    frame.len = payload.len();
    Ok(())
}

pub fn legacy_payload_from_hilink_frame(frame: &HilinkBridgeFrame) -> &[u8] {
    frame.as_slice()
}
