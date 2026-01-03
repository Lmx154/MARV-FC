#![allow(dead_code)]

use heapless::Vec;

pub const MAX_PACKET_BYTES: usize = 255;
pub const MAX_PACKET_PAYLOAD: usize = MAX_PACKET_BYTES.saturating_sub(1);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PacketType {
    KeepAlive = 0x00,
    RcData = 0x01,
    Command = 0x02,
    LinkStats = 0x10,
    Ack = 0x11,
    Telemetry = 0x12,
}

impl PacketType {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x00 => Some(PacketType::KeepAlive),
            0x01 => Some(PacketType::RcData),
            0x02 => Some(PacketType::Command),
            0x10 => Some(PacketType::LinkStats),
            0x11 => Some(PacketType::Ack),
            0x12 => Some(PacketType::Telemetry),
            _ => None,
        }
    }

    pub const fn name(self) -> &'static str {
        match self {
            PacketType::KeepAlive => "KEEPALIVE",
            PacketType::RcData => "RC_DATA",
            PacketType::Command => "COMMAND",
            PacketType::LinkStats => "LINK_STATS",
            PacketType::Ack => "ACK",
            PacketType::Telemetry => "TELEMETRY",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Packet {
    pub packet_type: PacketType,
    pub payload: Vec<u8, MAX_PACKET_PAYLOAD>,
}

impl Packet {
    pub fn new(packet_type: PacketType) -> Self {
        Self {
            packet_type,
            payload: Vec::new(),
        }
    }

    pub fn with_payload(
        packet_type: PacketType,
        payload: Vec<u8, MAX_PACKET_PAYLOAD>,
    ) -> Self {
        Self { packet_type, payload }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandFrame {
    pub seq: u8,
    pub cmd_id: u8,
    pub payload: u32,
}

impl CommandFrame {
    pub fn encode(&self) -> Vec<u8, MAX_PACKET_PAYLOAD> {
        let mut out = Vec::new();
        let _ = out.push(self.seq);
        let _ = out.push(self.cmd_id);
        let _ = out.extend_from_slice(&self.payload.to_le_bytes());
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() < 6 {
            return Err(DecodeError::BadLength);
        }
        Ok(CommandFrame {
            seq: bytes[0],
            cmd_id: bytes[1],
            payload: u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]),
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkStats {
    pub rssi: i16,
    pub snr: i16,
    pub lq: u8,
}

impl LinkStats {
    pub fn encode(&self) -> Vec<u8, MAX_PACKET_PAYLOAD> {
        let mut out = Vec::new();
        let _ = out.extend_from_slice(&self.rssi.to_le_bytes());
        let _ = out.extend_from_slice(&self.snr.to_le_bytes());
        let _ = out.push(self.lq);
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() < 5 {
            return Err(DecodeError::BadLength);
        }
        Ok(LinkStats {
            rssi: i16::from_le_bytes([bytes[0], bytes[1]]),
            snr: i16::from_le_bytes([bytes[2], bytes[3]]),
            lq: bytes[4],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    PayloadTooLarge,
    PayloadTooShort,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    TooShort,
    UnknownType(u8),
    BadLength,
}

pub fn encode_packet_fixed(
    packet: &Packet,
    len: usize,
) -> Result<Vec<u8, MAX_PACKET_BYTES>, EncodeError> {
    if len > MAX_PACKET_BYTES {
        return Err(EncodeError::PayloadTooLarge);
    }

    let mut out = Vec::new();
    if len == 0 {
        if packet.packet_type == PacketType::KeepAlive && packet.payload.is_empty() {
            return Ok(out);
        }
        return Err(EncodeError::PayloadTooShort);
    }

    if packet.payload.len() > MAX_PACKET_PAYLOAD {
        return Err(EncodeError::PayloadTooLarge);
    }

    let needed = 1usize.saturating_add(packet.payload.len());
    if needed > len {
        return Err(EncodeError::PayloadTooShort);
    }

    out.push(packet.packet_type as u8)
        .map_err(|_| EncodeError::PayloadTooLarge)?;
    out.extend_from_slice(packet.payload.as_slice())
        .map_err(|_| EncodeError::PayloadTooLarge)?;

    while out.len() < len {
        out.push(0).map_err(|_| EncodeError::PayloadTooLarge)?;
    }

    Ok(out)
}

pub fn decode_packet(bytes: &[u8]) -> Result<Packet, DecodeError> {
    if bytes.is_empty() {
        return Ok(Packet::new(PacketType::KeepAlive));
    }

    let packet_type = PacketType::from_u8(bytes[0]).ok_or(DecodeError::UnknownType(bytes[0]))?;
    let payload_len = bytes.len().saturating_sub(1);
    if payload_len > MAX_PACKET_PAYLOAD {
        return Err(DecodeError::BadLength);
    }

    let mut payload = Vec::new();
    payload
        .extend_from_slice(&bytes[1..])
        .map_err(|_| DecodeError::BadLength)?;
    Ok(Packet::with_payload(packet_type, payload))
}
