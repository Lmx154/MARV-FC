pub const FRAME_MAGIC: [u8; 2] = *b"MV";
pub const FRAME_VERSION: u8 = 1;
pub const FRAME_HEADER_LEN: usize = 6;
pub const MAX_FRAME_LEN: usize = u8::MAX as usize;
pub const MAX_FRAME_PAYLOAD_LEN: usize = MAX_FRAME_LEN - FRAME_HEADER_LEN;

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
#[repr(u8)]
pub enum LoraNodeRole {
    Radio = 1,
    GroundStation = 2,
}

impl LoraNodeRole {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::Radio),
            2 => Some(Self::GroundStation),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
#[repr(u8)]
pub enum LoraFrameKind {
    Ping = 1,
    Pong = 2,
    Beacon = 3,
    Data = 4,
    LinkStatus = 5,
    Heartbeat = 6,
}

impl LoraFrameKind {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::Ping),
            2 => Some(Self::Pong),
            3 => Some(Self::Beacon),
            4 => Some(Self::Data),
            5 => Some(Self::LinkStatus),
            6 => Some(Self::Heartbeat),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoraFrame<'a> {
    pub source: LoraNodeRole,
    pub kind: LoraFrameKind,
    pub payload: &'a [u8],
}

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub enum LoraFrameError {
    BufferTooSmall,
    PayloadTooLarge,
    TooShort,
    BadMagic,
    UnsupportedVersion(u8),
    UnknownSource(u8),
    UnknownKind(u8),
    LengthMismatch { expected: usize, actual: usize },
}

pub fn encode_frame(frame: LoraFrame<'_>, out: &mut [u8]) -> Result<usize, LoraFrameError> {
    if frame.payload.len() > MAX_FRAME_PAYLOAD_LEN {
        return Err(LoraFrameError::PayloadTooLarge);
    }

    let len = FRAME_HEADER_LEN + frame.payload.len();
    if out.len() < len {
        return Err(LoraFrameError::BufferTooSmall);
    }

    out[0] = FRAME_MAGIC[0];
    out[1] = FRAME_MAGIC[1];
    out[2] = FRAME_VERSION;
    out[3] = frame.source as u8;
    out[4] = frame.kind as u8;
    out[5] = frame.payload.len() as u8;
    out[FRAME_HEADER_LEN..len].copy_from_slice(frame.payload);

    Ok(len)
}

pub fn decode_frame(buf: &[u8]) -> Result<LoraFrame<'_>, LoraFrameError> {
    if buf.len() < FRAME_HEADER_LEN {
        return Err(LoraFrameError::TooShort);
    }

    if buf[0] != FRAME_MAGIC[0] || buf[1] != FRAME_MAGIC[1] {
        return Err(LoraFrameError::BadMagic);
    }

    if buf[2] != FRAME_VERSION {
        return Err(LoraFrameError::UnsupportedVersion(buf[2]));
    }

    let source = LoraNodeRole::from_u8(buf[3]).ok_or(LoraFrameError::UnknownSource(buf[3]))?;
    let kind = LoraFrameKind::from_u8(buf[4]).ok_or(LoraFrameError::UnknownKind(buf[4]))?;
    let payload_len = buf[5] as usize;
    let expected_len = FRAME_HEADER_LEN + payload_len;

    if buf.len() != expected_len {
        return Err(LoraFrameError::LengthMismatch {
            expected: expected_len,
            actual: buf.len(),
        });
    }

    Ok(LoraFrame {
        source,
        kind,
        payload: &buf[FRAME_HEADER_LEN..],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_round_trips() {
        let mut buf = [0; 32];
        let frame = LoraFrame {
            source: LoraNodeRole::Radio,
            kind: LoraFrameKind::Data,
            payload: b"hello",
        };

        let len = encode_frame(frame, &mut buf).unwrap();
        let decoded = decode_frame(&buf[..len]).unwrap();

        assert_eq!(decoded.source, LoraNodeRole::Radio);
        assert_eq!(decoded.kind, LoraFrameKind::Data);
        assert_eq!(decoded.payload, b"hello");
    }

    #[test]
    fn rejects_bad_length() {
        let packet = [FRAME_MAGIC[0], FRAME_MAGIC[1], FRAME_VERSION, 1, 1, 2, 0];

        assert_eq!(
            decode_frame(&packet),
            Err(LoraFrameError::LengthMismatch {
                expected: 8,
                actual: 7
            })
        );
    }
}
