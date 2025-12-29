use heapless::Vec;

use super::phy::MAX_PHY_PAYLOAD;

// On-air header uses little-endian encoding for multi-byte fields.
pub const MAGIC: u16 = 0x4D52;
pub const VERSION: u8 = 1;
pub const NET_ID: u8 = 0x01;

pub const HEADER_LEN: usize = 8;
pub const MAX_PAYLOAD_LEN: usize = MAX_PHY_PAYLOAD - HEADER_LEN;

pub type FrameBytes = Vec<u8, MAX_PHY_PAYLOAD>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum FrameType {
    AcqPing = 0x01,
    AcqPong = 0x02,
}

impl FrameType {
    pub const fn name(self) -> &'static str {
        match self {
            FrameType::AcqPing => "ACQ_PING",
            FrameType::AcqPong => "ACQ_PONG",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameHeader {
    pub magic: u16,
    pub version: u8,
    pub net_id: u8,
    pub tick_seq: u16,
    pub frame_type: FrameType,
    pub flags: u8,
}

impl FrameHeader {
    pub const fn new(frame_type: FrameType, tick_seq: u16) -> Self {
        Self {
            magic: MAGIC,
            version: VERSION,
            net_id: NET_ID,
            tick_seq,
            frame_type,
            flags: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    TooShort,
    BadMagic(u16),
    BadVersion(u8),
    BadNetId(u8),
    UnknownFrameType(u8),
}

pub fn encode_frame(header: &FrameHeader, payload: &[u8]) -> FrameBytes {
    let mut out = FrameBytes::new();
    if payload.len() > MAX_PAYLOAD_LEN {
        return out;
    }

    let _ = out.extend_from_slice(&header.magic.to_le_bytes());
    let _ = out.push(header.version);
    let _ = out.push(header.net_id);
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

    let magic = u16::from_le_bytes([bytes[0], bytes[1]]);
    if magic != MAGIC {
        return Err(DecodeError::BadMagic(magic));
    }

    let version = bytes[2];
    if version != VERSION {
        return Err(DecodeError::BadVersion(version));
    }

    let net_id = bytes[3];
    if net_id != NET_ID {
        return Err(DecodeError::BadNetId(net_id));
    }

    let tick_seq = u16::from_le_bytes([bytes[4], bytes[5]]);
    let frame_type = match bytes[6] {
        x if x == FrameType::AcqPing as u8 => FrameType::AcqPing,
        x if x == FrameType::AcqPong as u8 => FrameType::AcqPong,
        other => return Err(DecodeError::UnknownFrameType(other)),
    };
    let flags = bytes[7];

    let header = FrameHeader {
        magic,
        version,
        net_id,
        tick_seq,
        frame_type,
        flags,
    };

    Ok((header, &bytes[HEADER_LEN..]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codec_round_trip() {
        let header = FrameHeader::new(FrameType::AcqPing, 42);
        let payload = [0xA5, 0x5A, 0x01];

        let frame = encode_frame(&header, &payload);
        assert_eq!(frame.len(), HEADER_LEN + payload.len());

        let (decoded, decoded_payload) = decode_frame(frame.as_slice()).unwrap();
        assert_eq!(decoded, header);
        assert_eq!(decoded_payload, payload.as_ref());
    }

    #[test]
    fn codec_rejects_bad_net_id() {
        let mut header = FrameHeader::new(FrameType::AcqPing, 1);
        header.net_id = NET_ID.wrapping_add(1);

        let frame = encode_frame(&header, &[]);
        assert!(matches!(
            decode_frame(frame.as_slice()),
            Err(DecodeError::BadNetId(_))
        ));
    }
}
