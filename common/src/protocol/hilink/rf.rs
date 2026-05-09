//! Compact RF dialect payloads and framing.
//!
//! RF frames are transport-internal packets. They do not use the normal HILink
//! header, message IDs, COBS delimiter, or `WirePayload` trait.

use super::crc16_ccitt_false;

pub const RF_TYPE_LEN: usize = 1;
pub const RF_CRC_LEN: usize = 2;
pub const RF_PACKET_OVERHEAD_LEN: usize = RF_TYPE_LEN + RF_CRC_LEN;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RfError {
    UnknownRfMsgType,
    UnsupportedTranslation,
    CorrelationMismatch,
    BufferTooSmall,
    InvalidPayloadLength,
    CrcMismatch,
}

pub type Result<T> = core::result::Result<T, RfError>;

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

impl core::convert::TryFrom<u8> for RfMsgType {
    type Error = RfError;

    fn try_from(value: u8) -> Result<Self> {
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
            _ => Err(RfError::UnknownRfMsgType),
        }
    }
}

impl RfMsgType {
    pub const fn payload_len(self) -> usize {
        match self {
            Self::LoRaFlightSnapshot => LoRaFlightSnapshotPayload::WIRE_LEN,
            Self::LoRaGpsSnapshot => LoRaGpsSnapshotPayload::WIRE_LEN,
            Self::LoRaEvent => LoRaEventPayload::WIRE_LEN,
            Self::LoRaFaults => LoRaFaultsPayload::WIRE_LEN,
            Self::LoRaLinkStatus => LoRaLinkStatusPayload::WIRE_LEN,
            Self::LoRaCommand => LoRaCommandPayload::WIRE_LEN,
            Self::LoRaCommandAck => LoRaCommandAckPayload::WIRE_LEN,
            Self::LoRaSetProfile => LoRaSetProfilePayload::WIRE_LEN,
            Self::LoRaRequestSnapshot => LoRaRequestSnapshotPayload::WIRE_LEN,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DecodedRfPacket<'a> {
    pub msg_type: RfMsgType,
    pub payload: &'a [u8],
}

pub trait RfWirePayload: Sized {
    const MSG_TYPE: RfMsgType;
    const WIRE_LEN: usize;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize>;
    fn decode_payload(input: &[u8]) -> Result<Self>;
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoRaFlightSnapshotPayload {
    pub time_ms: u32,
    pub state: u8,
    pub mode: u8,
    pub flags: u16,
    pub altitude_dm: i32,
    pub vertical_velocity_cms: i16,
    pub accel_mag_cms2: u16,
    pub battery_mv: u16,
    pub pyro_or_actuator_flags: u16,
    pub fault_summary: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoRaGpsSnapshotPayload {
    pub time_ms: u32,
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_msl_dm: i32,
    pub ground_speed_cms: u16,
    pub heading_cdeg: u16,
    pub sats: u8,
    pub fix_type: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoRaEventPayload {
    pub time_ms: u32,
    pub event_id: u16,
    pub severity: u8,
    pub arg0: i32,
    pub arg1: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoRaFaultsPayload {
    pub time_ms: u32,
    pub active_faults: u32,
    pub latched_faults: u32,
    pub inhibit_flags: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoRaLinkStatusPayload {
    pub time_ms: u32,
    pub uplink_rssi_dbm: i8,
    pub uplink_snr_x4: i8,
    pub downlink_rssi_dbm: i8,
    pub downlink_snr_x4: i8,
    pub rx_packets_delta: u16,
    pub tx_packets_delta: u16,
    pub lost_packets_delta: u16,
    pub active_profile: u8,
    pub telemetry_rate_hz: u8,
    pub reserved: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoRaCommandPayload {
    pub command_id: u16,
    pub command_seq: u16,
    pub expires_ms: u16,
    pub flags: u16,
    pub arg0: i32,
    pub arg1: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoRaCommandAckPayload {
    pub command_id: u16,
    pub command_seq: u16,
    pub status: u8,
    pub reason: u8,
    pub state: u8,
    pub reserved: u8,
    pub detail: i32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoRaSetProfilePayload {
    pub command_seq: u16,
    pub profile: u8,
    pub telemetry_rate_hz: u8,
    pub gps_rate_hz: u8,
    pub link_status_rate_hz: u8,
    pub flags: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct LoRaRequestSnapshotPayload {
    pub command_seq: u16,
    pub request_flags: u16,
}

pub mod lora_state {
    pub const UNKNOWN: u8 = 0;
    pub const BOOT: u8 = 1;
    pub const STANDBY: u8 = 2;
    pub const ARMED: u8 = 3;
    pub const ASCENT: u8 = 4;
    pub const COAST: u8 = 5;
    pub const DESCENT: u8 = 6;
    pub const LANDED: u8 = 7;
    pub const ABORT: u8 = 8;
    pub const FAILSAFE: u8 = 9;
}

pub mod lora_mode {
    pub const UNKNOWN: u8 = 0;
    pub const MANUAL: u8 = 1;
    pub const AUTO: u8 = 2;
    pub const GUIDED: u8 = 3;
    pub const HIL: u8 = 4;
    pub const RECOVERY: u8 = 5;
}

pub mod lora_flags {
    pub const ARMED: u16 = 1 << 0;
    pub const FAILSAFE: u16 = 1 << 1;
    pub const GPS_VALID: u16 = 1 << 2;
    pub const ESTIMATOR_VALID: u16 = 1 << 3;
    pub const BATTERY_LOW: u16 = 1 << 4;
    pub const PYRO_SAFE: u16 = 1 << 5;
    pub const RADIO_DEGRADED: u16 = 1 << 6;
    pub const RECOVERY_ACTIVE: u16 = 1 << 7;
}

pub mod lora_fault {
    pub const IMU: u32 = 1 << 0;
    pub const MAG: u32 = 1 << 1;
    pub const BARO: u32 = 1 << 2;
    pub const GPS: u32 = 1 << 3;
    pub const ESTIMATOR: u32 = 1 << 4;
    pub const BATTERY: u32 = 1 << 5;
    pub const RADIO: u32 = 1 << 6;
    pub const STORAGE: u32 = 1 << 7;
    pub const ACTUATOR: u32 = 1 << 8;
    pub const PYRO: u32 = 1 << 9;
    pub const SAFETY_INHIBIT: u32 = 1 << 10;
}

pub mod lora_pyro_actuator_flags {
    pub const MOTOR_OUTPUT_ACTIVE: u16 = 1 << 0;
    pub const MOTOR_OUTPUT_CLAMPED: u16 = 1 << 1;
    pub const PYRO_CONTINUITY_1: u16 = 1 << 2;
    pub const PYRO_CONTINUITY_2: u16 = 1 << 3;
    pub const PYRO_FIRED_1: u16 = 1 << 4;
    pub const PYRO_FIRED_2: u16 = 1 << 5;
    pub const PYRO_INHIBITED: u16 = 1 << 6;
}

pub mod lora_event_id {
    pub const BOOT: u16 = 1;
    pub const STATE_CHANGE: u16 = 2;
    pub const MODE_CHANGE: u16 = 3;
    pub const ARM_ACCEPTED: u16 = 4;
    pub const DISARMED: u16 = 5;
    pub const ABORT_TRIGGERED: u16 = 6;
    pub const GPS_FIX_CHANGED: u16 = 7;
    pub const RADIO_PROFILE_CHANGED: u16 = 8;
    pub const FAULT_ASSERTED: u16 = 9;
    pub const FAULT_CLEARED: u16 = 10;
    pub const PYRO_FIRED: u16 = 11;
    pub const RECOVERY_BEACON_ENTERED: u16 = 12;
}

pub mod lora_event_severity {
    pub const INFO: u8 = 0;
    pub const WARNING: u8 = 1;
    pub const ERROR: u8 = 2;
    pub const CRITICAL: u8 = 3;
}

pub mod lora_command_id {
    pub const ARM: u16 = 1;
    pub const DISARM: u16 = 2;
    pub const ABORT: u16 = 3;
    pub const MOTOR_STOP: u16 = 4;
    pub const SET_MODE: u16 = 5;
    pub const SET_TELEMETRY_RATE: u16 = 6;
    pub const SET_RADIO_PROFILE: u16 = 7;
    pub const REQUEST_SNAPSHOT: u16 = 8;
    pub const REQUEST_GPS: u16 = 9;
    pub const REQUEST_FAULTS: u16 = 10;
    pub const ENTER_RECOVERY_BEACON: u16 = 11;
    pub const PING: u16 = 12;
}

pub mod lora_command_status {
    pub const ACCEPTED: u8 = 0;
    pub const REJECTED: u8 = 1;
    pub const DENIED_STATE: u8 = 2;
    pub const DENIED_SAFETY: u8 = 3;
    pub const INVALID_ARG: u8 = 4;
    pub const EXPIRED: u8 = 5;
    pub const DUPLICATE_ACCEPTED: u8 = 6;
    pub const DUPLICATE_REJECTED: u8 = 7;
    pub const BUSY: u8 = 8;
}

pub mod lora_command_reason {
    pub const NONE: u8 = 0;
    pub const BAD_STATE: u8 = 1;
    pub const SAFETY_INHIBIT: u8 = 2;
    pub const AUTH_REQUIRED: u8 = 3;
    pub const UNSUPPORTED: u8 = 4;
    pub const BAD_ARGUMENT: u8 = 5;
    pub const EXPIRED: u8 = 6;
    pub const DUPLICATE: u8 = 7;
    pub const RADIO_BUSY: u8 = 8;
}

pub mod lora_command_flags {
    pub const URGENT: u16 = 1 << 0;
    pub const REQUIRE_ARMED: u16 = 1 << 1;
    pub const ALLOW_WHILE_FAILSAFE: u16 = 1 << 2;
    pub const QUEUE_IF_BUSY: u16 = 1 << 3;
}

pub mod lora_profile_flags {
    pub const TEMPORARY: u16 = 1 << 0;
    pub const SAVE_DEFAULT: u16 = 1 << 1;
    pub const ENTER_RECOVERY_RX_WINDOWS: u16 = 1 << 2;
}

pub mod lora_profile {
    pub const SF7_500: u8 = 1;
    pub const SF8_500: u8 = 2;
    pub const SF8_250: u8 = 3;
    pub const RECOVERY_BEACON: u8 = 4;
}

pub mod lora_request_flags {
    pub const FLIGHT_SNAPSHOT: u16 = 1 << 0;
    pub const GPS_SNAPSHOT: u16 = 1 << 1;
    pub const LINK_STATUS: u16 = 1 << 2;
    pub const FAULTS: u16 = 1 << 3;
}

pub mod lora_scaling {
    pub const ALTITUDE_INVALID_DM: i32 = i32::MIN;
    pub const ALT_MSL_INVALID_DM: i32 = i32::MIN;
    pub const LAT_LON_INVALID_E7: i32 = i32::MIN;
    pub const HEADING_INVALID_CDEG: u16 = u16::MAX;
    pub const RSSI_MIN_DBM: i8 = -127;
    pub const RSSI_MAX_DBM: i8 = 20;
    pub const SNR_MIN_X4: i8 = -80;
    pub const SNR_MAX_X4: i8 = 80;
}

macro_rules! impl_payload {
    ($ty:ty, $msg_type:expr, $wire_len:expr, |$self_var:ident, $writer_var:ident| $encode:block, |$reader_var:ident| $decode:block) => {
        impl RfWirePayload for $ty {
            const MSG_TYPE: RfMsgType = $msg_type;
            const WIRE_LEN: usize = $wire_len;

            fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
                let $self_var = self;
                let mut $writer_var = Writer::new(out);
                $encode
                Ok($writer_var.len())
            }

            fn decode_payload(input: &[u8]) -> Result<Self> {
                expect_len(input, Self::WIRE_LEN)?;
                let mut $reader_var = Reader::new(input);
                Ok($decode)
            }
        }
    };
}

impl_payload!(
    LoRaFlightSnapshotPayload,
    RfMsgType::LoRaFlightSnapshot,
    22,
    |this, w| {
        w.u32(this.time_ms)?;
        w.u8(this.state)?;
        w.u8(this.mode)?;
        w.u16(this.flags)?;
        w.i32(this.altitude_dm)?;
        w.i16(this.vertical_velocity_cms)?;
        w.u16(this.accel_mag_cms2)?;
        w.u16(this.battery_mv)?;
        w.u16(this.pyro_or_actuator_flags)?;
        w.u16(this.fault_summary)?;
    },
    |r| {
        Self {
            time_ms: r.u32()?,
            state: r.u8()?,
            mode: r.u8()?,
            flags: r.u16()?,
            altitude_dm: r.i32()?,
            vertical_velocity_cms: r.i16()?,
            accel_mag_cms2: r.u16()?,
            battery_mv: r.u16()?,
            pyro_or_actuator_flags: r.u16()?,
            fault_summary: r.u16()?,
        }
    }
);

impl_payload!(
    LoRaGpsSnapshotPayload,
    RfMsgType::LoRaGpsSnapshot,
    22,
    |this, w| {
        w.u32(this.time_ms)?;
        w.i32(this.lat_e7)?;
        w.i32(this.lon_e7)?;
        w.i32(this.alt_msl_dm)?;
        w.u16(this.ground_speed_cms)?;
        w.u16(this.heading_cdeg)?;
        w.u8(this.sats)?;
        w.u8(this.fix_type)?;
    },
    |r| {
        Self {
            time_ms: r.u32()?,
            lat_e7: r.i32()?,
            lon_e7: r.i32()?,
            alt_msl_dm: r.i32()?,
            ground_speed_cms: r.u16()?,
            heading_cdeg: r.u16()?,
            sats: r.u8()?,
            fix_type: r.u8()?,
        }
    }
);

impl_payload!(
    LoRaEventPayload,
    RfMsgType::LoRaEvent,
    15,
    |this, w| {
        w.u32(this.time_ms)?;
        w.u16(this.event_id)?;
        w.u8(this.severity)?;
        w.i32(this.arg0)?;
        w.i32(this.arg1)?;
    },
    |r| {
        Self {
            time_ms: r.u32()?,
            event_id: r.u16()?,
            severity: r.u8()?,
            arg0: r.i32()?,
            arg1: r.i32()?,
        }
    }
);

impl_payload!(
    LoRaFaultsPayload,
    RfMsgType::LoRaFaults,
    14,
    |this, w| {
        w.u32(this.time_ms)?;
        w.u32(this.active_faults)?;
        w.u32(this.latched_faults)?;
        w.u16(this.inhibit_flags)?;
    },
    |r| {
        Self {
            time_ms: r.u32()?,
            active_faults: r.u32()?,
            latched_faults: r.u32()?,
            inhibit_flags: r.u16()?,
        }
    }
);

impl_payload!(
    LoRaLinkStatusPayload,
    RfMsgType::LoRaLinkStatus,
    18,
    |this, w| {
        w.u32(this.time_ms)?;
        w.i8(this.uplink_rssi_dbm)?;
        w.i8(this.uplink_snr_x4)?;
        w.i8(this.downlink_rssi_dbm)?;
        w.i8(this.downlink_snr_x4)?;
        w.u16(this.rx_packets_delta)?;
        w.u16(this.tx_packets_delta)?;
        w.u16(this.lost_packets_delta)?;
        w.u8(this.active_profile)?;
        w.u8(this.telemetry_rate_hz)?;
        w.u16(this.reserved)?;
    },
    |r| {
        Self {
            time_ms: r.u32()?,
            uplink_rssi_dbm: r.i8()?,
            uplink_snr_x4: r.i8()?,
            downlink_rssi_dbm: r.i8()?,
            downlink_snr_x4: r.i8()?,
            rx_packets_delta: r.u16()?,
            tx_packets_delta: r.u16()?,
            lost_packets_delta: r.u16()?,
            active_profile: r.u8()?,
            telemetry_rate_hz: r.u8()?,
            reserved: r.u16()?,
        }
    }
);

impl_payload!(
    LoRaCommandPayload,
    RfMsgType::LoRaCommand,
    16,
    |this, w| {
        w.u16(this.command_id)?;
        w.u16(this.command_seq)?;
        w.u16(this.expires_ms)?;
        w.u16(this.flags)?;
        w.i32(this.arg0)?;
        w.i32(this.arg1)?;
    },
    |r| {
        Self {
            command_id: r.u16()?,
            command_seq: r.u16()?,
            expires_ms: r.u16()?,
            flags: r.u16()?,
            arg0: r.i32()?,
            arg1: r.i32()?,
        }
    }
);

impl_payload!(
    LoRaCommandAckPayload,
    RfMsgType::LoRaCommandAck,
    12,
    |this, w| {
        w.u16(this.command_id)?;
        w.u16(this.command_seq)?;
        w.u8(this.status)?;
        w.u8(this.reason)?;
        w.u8(this.state)?;
        w.u8(this.reserved)?;
        w.i32(this.detail)?;
    },
    |r| {
        Self {
            command_id: r.u16()?,
            command_seq: r.u16()?,
            status: r.u8()?,
            reason: r.u8()?,
            state: r.u8()?,
            reserved: r.u8()?,
            detail: r.i32()?,
        }
    }
);

impl_payload!(
    LoRaSetProfilePayload,
    RfMsgType::LoRaSetProfile,
    8,
    |this, w| {
        w.u16(this.command_seq)?;
        w.u8(this.profile)?;
        w.u8(this.telemetry_rate_hz)?;
        w.u8(this.gps_rate_hz)?;
        w.u8(this.link_status_rate_hz)?;
        w.u16(this.flags)?;
    },
    |r| {
        Self {
            command_seq: r.u16()?,
            profile: r.u8()?,
            telemetry_rate_hz: r.u8()?,
            gps_rate_hz: r.u8()?,
            link_status_rate_hz: r.u8()?,
            flags: r.u16()?,
        }
    }
);

impl_payload!(
    LoRaRequestSnapshotPayload,
    RfMsgType::LoRaRequestSnapshot,
    4,
    |this, w| {
        w.u16(this.command_seq)?;
        w.u16(this.request_flags)?;
    },
    |r| {
        Self {
            command_seq: r.u16()?,
            request_flags: r.u16()?,
        }
    }
);

pub const fn encoded_rf_len(payload_len: usize) -> usize {
    RF_PACKET_OVERHEAD_LEN + payload_len
}

pub fn encode_rf_packet<P: RfWirePayload>(payload: &P, out: &mut [u8]) -> Result<usize> {
    let len = encoded_rf_len(P::WIRE_LEN);
    if out.len() < len {
        return Err(RfError::BufferTooSmall);
    }

    out[0] = P::MSG_TYPE as u8;
    let written = payload.encode_payload(&mut out[1..1 + P::WIRE_LEN])?;
    if written != P::WIRE_LEN {
        return Err(RfError::InvalidPayloadLength);
    }

    let crc = crc16_ccitt_false(&out[..1 + P::WIRE_LEN]);
    out[1 + P::WIRE_LEN..len].copy_from_slice(&crc.to_le_bytes());
    Ok(len)
}

pub fn decode_rf_packet(input: &[u8]) -> Result<DecodedRfPacket<'_>> {
    if input.len() < RF_PACKET_OVERHEAD_LEN {
        return Err(RfError::InvalidPayloadLength);
    }

    let msg_type = RfMsgType::try_from(input[0])?;
    let expected_len = encoded_rf_len(msg_type.payload_len());
    if input.len() != expected_len {
        return Err(RfError::InvalidPayloadLength);
    }

    let packet_len = input.len() - RF_CRC_LEN;
    let stored_crc = u16::from_le_bytes([input[packet_len], input[packet_len + 1]]);
    let computed_crc = crc16_ccitt_false(&input[..packet_len]);
    if stored_crc != computed_crc {
        return Err(RfError::CrcMismatch);
    }

    Ok(DecodedRfPacket {
        msg_type,
        payload: &input[1..packet_len],
    })
}

pub fn decode_rf_payload<P: RfWirePayload>(packet: &DecodedRfPacket<'_>) -> Result<P> {
    if packet.msg_type != P::MSG_TYPE || packet.payload.len() != P::WIRE_LEN {
        return Err(RfError::InvalidPayloadLength);
    }

    P::decode_payload(packet.payload)
}

fn expect_len(input: &[u8], expected: usize) -> Result<()> {
    if input.len() == expected {
        Ok(())
    } else {
        Err(RfError::InvalidPayloadLength)
    }
}

struct Writer<'a> {
    out: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    const fn new(out: &'a mut [u8]) -> Self {
        Self { out, pos: 0 }
    }

    const fn len(&self) -> usize {
        self.pos
    }

    fn bytes(&mut self, bytes: &[u8]) -> Result<()> {
        if self.pos + bytes.len() > self.out.len() {
            return Err(RfError::BufferTooSmall);
        }
        self.out[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
        Ok(())
    }

    fn u8(&mut self, value: u8) -> Result<()> {
        self.bytes(&[value])
    }

    fn i8(&mut self, value: i8) -> Result<()> {
        self.bytes(&value.to_le_bytes())
    }

    fn u16(&mut self, value: u16) -> Result<()> {
        self.bytes(&value.to_le_bytes())
    }

    fn i16(&mut self, value: i16) -> Result<()> {
        self.bytes(&value.to_le_bytes())
    }

    fn u32(&mut self, value: u32) -> Result<()> {
        self.bytes(&value.to_le_bytes())
    }

    fn i32(&mut self, value: i32) -> Result<()> {
        self.bytes(&value.to_le_bytes())
    }
}

struct Reader<'a> {
    input: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    const fn new(input: &'a [u8]) -> Self {
        Self { input, pos: 0 }
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N]> {
        if self.pos + N > self.input.len() {
            return Err(RfError::InvalidPayloadLength);
        }
        let mut out = [0u8; N];
        out.copy_from_slice(&self.input[self.pos..self.pos + N]);
        self.pos += N;
        Ok(out)
    }

    fn u8(&mut self) -> Result<u8> {
        Ok(self.take::<1>()?[0])
    }

    fn i8(&mut self) -> Result<i8> {
        Ok(i8::from_le_bytes(self.take::<1>()?))
    }

    fn u16(&mut self) -> Result<u16> {
        Ok(u16::from_le_bytes(self.take::<2>()?))
    }

    fn i16(&mut self) -> Result<i16> {
        Ok(i16::from_le_bytes(self.take::<2>()?))
    }

    fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.take::<4>()?))
    }

    fn i32(&mut self) -> Result<i32> {
        Ok(i32::from_le_bytes(self.take::<4>()?))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lora_payload_sizes_match_v1_wire_contract() {
        assert_eq!(LoRaFlightSnapshotPayload::WIRE_LEN, 22);
        assert_eq!(LoRaGpsSnapshotPayload::WIRE_LEN, 22);
        assert_eq!(LoRaEventPayload::WIRE_LEN, 15);
        assert_eq!(LoRaFaultsPayload::WIRE_LEN, 14);
        assert_eq!(LoRaLinkStatusPayload::WIRE_LEN, 18);
        assert_eq!(LoRaCommandPayload::WIRE_LEN, 16);
        assert_eq!(LoRaCommandAckPayload::WIRE_LEN, 12);
        assert_eq!(LoRaSetProfilePayload::WIRE_LEN, 8);
        assert_eq!(LoRaRequestSnapshotPayload::WIRE_LEN, 4);
    }

    #[test]
    fn rf_packet_round_trips_with_compact_type_and_crc() {
        let payload = LoRaCommandPayload {
            command_id: lora_command_id::ARM,
            command_seq: 42,
            expires_ms: 250,
            flags: lora_command_flags::URGENT,
            arg0: 1,
            arg1: -1,
        };
        let mut encoded = [0u8; encoded_rf_len(LoRaCommandPayload::WIRE_LEN)];

        let len = encode_rf_packet(&payload, &mut encoded).unwrap();
        let packet = decode_rf_packet(&encoded[..len]).unwrap();
        let decoded = decode_rf_payload::<LoRaCommandPayload>(&packet).unwrap();

        assert_eq!(packet.msg_type, RfMsgType::LoRaCommand);
        assert_eq!(decoded, payload);
    }

    #[test]
    fn rf_packet_rejects_bad_crc() {
        let payload = LoRaRequestSnapshotPayload {
            command_seq: 7,
            request_flags: lora_request_flags::FLIGHT_SNAPSHOT,
        };
        let mut encoded = [0u8; encoded_rf_len(LoRaRequestSnapshotPayload::WIRE_LEN)];
        let len = encode_rf_packet(&payload, &mut encoded).unwrap();
        encoded[len - 1] ^= 0x55;

        assert_eq!(decode_rf_packet(&encoded[..len]), Err(RfError::CrcMismatch));
    }
}
