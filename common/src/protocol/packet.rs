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
    TelemetryImu = 0x12,
    TelemetryBaro = 0x13,
    TelemetryMag = 0x14,
    TelemetryGps = 0x15,
    TelemetrySystem = 0x16,
    TelemetryBurst = 0x17,
}

impl PacketType {
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0x00 => Some(PacketType::KeepAlive),
            0x01 => Some(PacketType::RcData),
            0x02 => Some(PacketType::Command),
            0x10 => Some(PacketType::LinkStats),
            0x11 => Some(PacketType::Ack),
            0x12 => Some(PacketType::TelemetryImu),
            0x13 => Some(PacketType::TelemetryBaro),
            0x14 => Some(PacketType::TelemetryMag),
            0x15 => Some(PacketType::TelemetryGps),
            0x16 => Some(PacketType::TelemetrySystem),
            0x17 => Some(PacketType::TelemetryBurst),
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
            PacketType::TelemetryImu => "TELEMETRY_IMU",
            PacketType::TelemetryBaro => "TELEMETRY_BARO",
            PacketType::TelemetryMag => "TELEMETRY_MAG",
            PacketType::TelemetryGps => "TELEMETRY_GPS",
            PacketType::TelemetrySystem => "TELEMETRY_SYSTEM",
            PacketType::TelemetryBurst => "TELEMETRY_BURST",
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
pub struct TelemetryImu {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
}

impl TelemetryImu {
    pub fn encode(&self) -> Vec<u8, MAX_PACKET_PAYLOAD> {
        let mut out = Vec::new();
        for v in self.accel {
            let _ = out.extend_from_slice(&v.to_le_bytes());
        }
        for v in self.gyro {
            let _ = out.extend_from_slice(&v.to_le_bytes());
        }
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() < 12 {
            return Err(DecodeError::BadLength);
        }
        Ok(TelemetryImu {
            accel: [
                i16::from_le_bytes([bytes[0], bytes[1]]),
                i16::from_le_bytes([bytes[2], bytes[3]]),
                i16::from_le_bytes([bytes[4], bytes[5]]),
            ],
            gyro: [
                i16::from_le_bytes([bytes[6], bytes[7]]),
                i16::from_le_bytes([bytes[8], bytes[9]]),
                i16::from_le_bytes([bytes[10], bytes[11]]),
            ],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TelemetryBaro {
    pub pressure_pa: i32,
    pub temp_c_x10: i16,
}

impl TelemetryBaro {
    pub fn encode(&self) -> Vec<u8, MAX_PACKET_PAYLOAD> {
        let mut out = Vec::new();
        let _ = out.extend_from_slice(&self.pressure_pa.to_le_bytes());
        let _ = out.extend_from_slice(&self.temp_c_x10.to_le_bytes());
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() < 6 {
            return Err(DecodeError::BadLength);
        }
        Ok(TelemetryBaro {
            pressure_pa: i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            temp_c_x10: i16::from_le_bytes([bytes[4], bytes[5]]),
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TelemetryMag {
    pub mag: [i16; 3],
}

impl TelemetryMag {
    pub fn encode(&self) -> Vec<u8, MAX_PACKET_PAYLOAD> {
        let mut out = Vec::new();
        for v in self.mag {
            let _ = out.extend_from_slice(&v.to_le_bytes());
        }
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() < 6 {
            return Err(DecodeError::BadLength);
        }
        Ok(TelemetryMag {
            mag: [
                i16::from_le_bytes([bytes[0], bytes[1]]),
                i16::from_le_bytes([bytes[2], bytes[3]]),
                i16::from_le_bytes([bytes[4], bytes[5]]),
            ],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TelemetryGps {
    pub lat: i32,
    pub lon: i32,
    pub alt_mm: i32,
    pub sats: u8,
    pub fix: u8,
}

impl TelemetryGps {
    pub fn encode(&self) -> Vec<u8, MAX_PACKET_PAYLOAD> {
        let mut out = Vec::new();
        let _ = out.extend_from_slice(&self.lat.to_le_bytes());
        let _ = out.extend_from_slice(&self.lon.to_le_bytes());
        let _ = out.extend_from_slice(&self.alt_mm.to_le_bytes());
        let _ = out.push(self.sats);
        let _ = out.push(self.fix);
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() < 14 {
            return Err(DecodeError::BadLength);
        }
        Ok(TelemetryGps {
            lat: i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            lon: i32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            alt_mm: i32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
            sats: bytes[12],
            fix: bytes[13],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TelemetrySystem {
    pub vbat_mv: u16,
    pub temp_c: i8,
    pub arm_status: u8,
    pub rssi_uplink: i8,
}

impl TelemetrySystem {
    pub fn encode(&self) -> Vec<u8, MAX_PACKET_PAYLOAD> {
        let mut out = Vec::new();
        let _ = out.extend_from_slice(&self.vbat_mv.to_le_bytes());
        let _ = out.push(self.temp_c as u8);
        let _ = out.push(self.arm_status);
        let _ = out.push(self.rssi_uplink as u8);
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, DecodeError> {
        if bytes.len() < 5 {
            return Err(DecodeError::BadLength);
        }
        Ok(TelemetrySystem {
            vbat_mv: u16::from_le_bytes([bytes[0], bytes[1]]),
            temp_c: bytes[2] as i8,
            arm_status: bytes[3],
            rssi_uplink: bytes[4] as i8,
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
