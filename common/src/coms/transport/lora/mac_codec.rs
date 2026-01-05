//! MAC layer on-air frame codec and constants.
use heapless::Vec;

use super::phy_service::MAX_PHY_PAYLOAD;

// On-air header uses little-endian encoding for multi-byte fields.
pub const HEADER_LEN: usize = 4;
pub const MAX_PAYLOAD_LEN: usize = MAX_PHY_PAYLOAD - HEADER_LEN;

pub type FrameBytes = Vec<u8, MAX_PHY_PAYLOAD>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameType {
    AcqPing = 0x01,
    AcqPong = 0x02,
    ControlUp = 0x10,
    ControlDown = 0x11,
}

impl FrameType {
    pub const fn name(self) -> &'static str {
        match self {
            FrameType::AcqPing => "ACQ_PING",
            FrameType::AcqPong => "ACQ_PONG",
            FrameType::ControlUp => "CONTROL_UP",
            FrameType::ControlDown => "CONTROL_DOWN",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameHeader {
    pub tick_seq: u16,
    pub frame_type: FrameType,
    pub flags: u8,
}

impl FrameHeader {
    pub const fn new(frame_type: FrameType, tick_seq: u16) -> Self {
        Self {
            tick_seq,
            frame_type,
            flags: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    TooShort,
    UnknownFrameType(u8),
}

pub fn encode_frame(header: &FrameHeader, payload: &[u8]) -> FrameBytes {
    let mut out = FrameBytes::new();
    if payload.len() > MAX_PAYLOAD_LEN {
        return out;
    }

    let _ = out.extend_from_slice(&header.tick_seq.to_le_bytes());
    let _ = out.push(header.frame_type as u8);
    let _ = out.push(header.flags);
    let _ = out.extend_from_slice(payload);
    out
}

pub fn decode_frame(bytes: &[u8]) -> Result<(FrameHeader, &[u8]), DecodeError> {
    if bytes.len() < HEADER_LEN {
        return Err(DecodeError::TooShort);
    }

    let tick_seq = u16::from_le_bytes([bytes[0], bytes[1]]);
    let frame_type = match bytes[2] {
        x if x == FrameType::AcqPing as u8 => FrameType::AcqPing,
        x if x == FrameType::AcqPong as u8 => FrameType::AcqPong,
        x if x == FrameType::ControlUp as u8 => FrameType::ControlUp,
        x if x == FrameType::ControlDown as u8 => FrameType::ControlDown,
        other => return Err(DecodeError::UnknownFrameType(other)),
    };
    let flags = bytes[3];

    let header = FrameHeader {
        tick_seq,
        frame_type,
        flags,
    };

    Ok((header, &bytes[HEADER_LEN..]))
}
