//! MAVLink framing and message support belongs here.

use core::fmt;

use heapless::Vec;

use crate::services::hil::backend::SensorBackend;
use crate::services::hil::egress::HilEgressProtocol;
use crate::services::hil::ingress::HilIngressProtocol;
use crate::services::hil::model::{
    HilActuatorCommand, HilBarometerSample, HilCommandAck, HilCommandAckResult, HilControlAction,
    HilControlCommand, HilEgressMessage, HilGpsSample, HilImuSample, HilIngressMessage, HilSubmode,
    HilTick,
};
use crate::utilities::time::MeasurementTimestamp;

pub const MAVLINK_V2_STX: u8 = 0xFD;
pub const MAVLINK_V1_STX: u8 = 0xFE;
const MAVLINK_V1_HEADER_LEN: usize = 5;
const MAVLINK_V2_HEADER_LEN: usize = 9;
const MAVLINK_CHECKSUM_LEN: usize = 2;
const MAVLINK_SIGNATURE_LEN: usize = 13;
const MAVLINK_IFLAG_SIGNED: u8 = 0x01;
const MAX_MAVLINK_FRAME_LEN: usize = 64;
pub const MIN_MAVLINK_FRAME_LEN: usize = 1 + MAVLINK_V1_HEADER_LEN + MAVLINK_CHECKSUM_LEN;

pub const SYSTEM_TIME_ID: u32 = 2;
pub const HIL_SENSOR_ID: u32 = 107;
pub const HIL_GPS_ID: u32 = 113;
pub const ACTUATOR_CONTROL_TARGET_ID: u32 = 140;
pub const COMMAND_LONG_ID: u32 = 76;
pub const COMMAND_ACK_ID: u32 = 77;
pub const MAV_CMD_MARV_ENTER_HIL_MODE: u16 = 31_010;
pub const MAV_CMD_MARV_SET_HIL_BACKEND: u16 = MAV_CMD_MARV_ENTER_HIL_MODE;
pub const MAV_CMD_MARV_SELECT_HIL_SUBMODE: u16 = 31_011;

const CRC_EXTRA_SYSTEM_TIME: u8 = 137;
const CRC_EXTRA_HIL_SENSOR: u8 = 108;
const CRC_EXTRA_HIL_GPS: u8 = 124;
const CRC_EXTRA_ACTUATOR_CONTROL_TARGET: u8 = 181;
const CRC_EXTRA_COMMAND_LONG: u8 = 152;
const CRC_EXTRA_COMMAND_ACK: u8 = 143;

const HIL_SENSOR_LEN: usize = 64;
const HIL_GPS_BASE_LEN: usize = 36;
const SYSTEM_TIME_LEN: usize = 12;
const COMMAND_LONG_LEN: usize = 33;
const COMMAND_ACK_LEN: usize = 10;
const IMU_UPDATED_MASK: u32 = 0x003F;
const BARO_UPDATED_MASK: u32 = 0x1A00;

#[derive(Clone, Copy, Debug, Default)]
pub struct SystemTimeMessage {
    pub time_boot_ms: u32,
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub struct HilSensorMessage {
    pub time_usec: u64,
    pub accel_mps2: [f32; 3],
    pub gyro_rad_s: [f32; 3],
    pub abs_pressure_hpa: f32,
    pub pressure_alt_m: f32,
    pub temperature_c: f32,
    pub fields_updated: u32,
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug)]
pub struct HilGpsMessage {
    pub time_usec: u64,
    pub fix_type: u8,
    pub lat_deg_e7: i32,
    pub lon_deg_e7: i32,
    pub alt_mm: i32,
    pub eph: u16,
    pub epv: u16,
    pub vel_cm_s: u16,
    pub vn_cm_s: i16,
    pub ve_cm_s: i16,
    pub vd_cm_s: i16,
    pub cog_cdeg: u16,
    pub satellites_visible: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct ActuatorControlTargetMessage {
    pub time_usec: u64,
    pub group_mlx: u8,
    pub controls: [f32; 8],
}

#[derive(Clone, Copy, Debug)]
pub struct CommandLongMessage {
    pub params: [f32; 7],
    pub command: u16,
    pub target_system: u8,
    pub target_component: u8,
    pub confirmation: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct CommandAckMessage {
    pub command: u16,
    pub result: u8,
    pub progress: u8,
    pub result_param2: i32,
    pub target_system: u8,
    pub target_component: u8,
}

#[derive(Clone, Copy, Debug)]
pub enum DecodedMessage {
    SystemTime(SystemTimeMessage),
    HilSensor(HilSensorMessage),
    HilGps(HilGpsMessage),
    CommandLong(CommandLongMessage),
}

#[derive(Clone, Copy, Debug)]
pub struct MavlinkFrame {
    pub source_system: u8,
    pub source_component: u8,
    pub message: DecodedMessage,
}

pub struct MavlinkHilMessagePump<const CAPACITY: usize, const PENDING: usize = 4> {
    buffer: Vec<u8, CAPACITY>,
    pending: Vec<HilIngressMessage, PENDING>,
}

impl<const CAPACITY: usize, const PENDING: usize> Default
    for MavlinkHilMessagePump<CAPACITY, PENDING>
{
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAPACITY: usize, const PENDING: usize> MavlinkHilMessagePump<CAPACITY, PENDING> {
    pub const fn new() -> Self {
        Self {
            buffer: Vec::new(),
            pending: Vec::new(),
        }
    }

    pub fn reset(&mut self) {
        self.buffer.clear();
        self.pending.clear();
    }

    pub fn ingest_bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            if self.buffer.is_full() {
                self.buffer.remove(0);
            }
            let _ = self.buffer.push(byte);
        }
    }

    pub fn try_next_message(&mut self) -> Option<Result<HilIngressMessage, DecodeError>> {
        if !self.pending.is_empty() {
            return Some(Ok(self.pending.remove(0)));
        }

        loop {
            let sync_offset = self
                .buffer
                .iter()
                .position(|byte| *byte == MAVLINK_V2_STX || *byte == MAVLINK_V1_STX)?;
            if sync_offset > 0 {
                self.drop_front(sync_offset);
            }

            if self.buffer.len() < MIN_MAVLINK_FRAME_LEN {
                return None;
            }

            match try_consume_frame(self.buffer.as_slice()) {
                Ok(Some((frame, consumed))) => {
                    self.drop_front(consumed);
                    frame_to_hil_messages(frame, &mut self.pending);
                    if self.pending.is_empty() {
                        continue;
                    }
                    return Some(Ok(self.pending.remove(0)));
                }
                Ok(None) => return None,
                Err(DecodeError::BadMagic(_)) => {
                    self.drop_front(1);
                }
                Err(error) => {
                    self.drop_front(1);
                    return Some(Err(error));
                }
            }
        }
    }

    fn drop_front(&mut self, count: usize) {
        for _ in 0..count.min(self.buffer.len()) {
            self.buffer.remove(0);
        }
    }
}

impl<const CAPACITY: usize, const PENDING: usize> HilIngressProtocol
    for MavlinkHilMessagePump<CAPACITY, PENDING>
{
    type Error = DecodeError;

    fn reset(&mut self) {
        self.reset();
    }

    fn ingest_bytes(&mut self, bytes: &[u8]) {
        self.ingest_bytes(bytes);
    }

    fn try_next_message(&mut self) -> Option<Result<HilIngressMessage, Self::Error>> {
        self.try_next_message()
    }
}

pub struct MavlinkHilEgressEncoder {
    sequence: u8,
    system_id: u8,
    component_id: u8,
}

impl MavlinkHilEgressEncoder {
    pub const fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            sequence: 0,
            system_id,
            component_id,
        }
    }

    fn encode_actuator_command(
        &mut self,
        command: HilActuatorCommand,
    ) -> Vec<u8, MAX_MAVLINK_FRAME_LEN> {
        let frame = encode_actuator_control_target(
            &ActuatorControlTargetMessage {
                time_usec: command.timestamp.as_micros(),
                group_mlx: command.group,
                controls: command.controls,
            },
            self.sequence,
            self.system_id,
            self.component_id,
        );
        self.sequence = self.sequence.wrapping_add(1);
        frame
    }

    fn encode_mission_event(
        &mut self,
        event: crate::services::hil::model::HilMissionEvent,
    ) -> Vec<u8, MAX_MAVLINK_FRAME_LEN> {
        let frame = encode_command_long(
            &CommandLongMessage {
                params: event.params,
                command: event.command_id,
                target_system: 0,
                target_component: 0,
                confirmation: 0,
            },
            self.sequence,
            self.system_id,
            self.component_id,
        );
        self.sequence = self.sequence.wrapping_add(1);
        frame
    }

    fn encode_command_ack(&mut self, ack: HilCommandAck) -> Vec<u8, MAX_MAVLINK_FRAME_LEN> {
        let frame = encode_command_ack(
            &CommandAckMessage {
                command: ack.command_id,
                result: mav_result_wire_code(ack.result),
                progress: ack.progress,
                result_param2: ack.result_param2,
                target_system: ack.target_system,
                target_component: ack.target_component,
            },
            self.sequence,
            self.system_id,
            self.component_id,
        );
        self.sequence = self.sequence.wrapping_add(1);
        frame
    }
}

impl HilEgressProtocol for MavlinkHilEgressEncoder {
    type EncodedFrame = Vec<u8, MAX_MAVLINK_FRAME_LEN>;
    type Error = ();

    fn encode_message(
        &mut self,
        message: HilEgressMessage,
    ) -> Result<Option<Self::EncodedFrame>, Self::Error> {
        match message {
            HilEgressMessage::ActuatorCommand(command) => {
                Ok(Some(self.encode_actuator_command(command)))
            }
            HilEgressMessage::MissionEvent(event) => Ok(Some(self.encode_mission_event(event))),
            HilEgressMessage::CommandAck(ack) => Ok(Some(self.encode_command_ack(ack))),
        }
    }
}

#[derive(Debug)]
pub enum DecodeError {
    BadMagic(u8),
    FrameTooShort,
    UnsupportedVersionFlags(u8),
    ChecksumMismatch { expected: u16, actual: u16 },
    UnsupportedMessage { id: u32, payload_len: usize },
}

pub fn frame_to_hil_messages<const N: usize>(
    frame: MavlinkFrame,
    out: &mut Vec<HilIngressMessage, N>,
) {
    match frame.message {
        DecodedMessage::SystemTime(message) => {
            let _ = out.push(HilIngressMessage::Tick(HilTick {
                timestamp: MeasurementTimestamp::from_micros(
                    u64::from(message.time_boot_ms) * 1_000,
                ),
                time_boot_ms: message.time_boot_ms,
            }));
        }
        DecodedMessage::HilSensor(message) => {
            let timestamp = MeasurementTimestamp::from_micros(message.time_usec);

            if message.fields_updated & IMU_UPDATED_MASK != 0 {
                let _ = out.push(HilIngressMessage::ImuSample(HilImuSample {
                    timestamp,
                    accel_mps2: message.accel_mps2,
                    gyro_rad_s: message.gyro_rad_s,
                }));
            }

            if message.fields_updated & BARO_UPDATED_MASK != 0 {
                let _ = out.push(HilIngressMessage::BarometerSample(HilBarometerSample {
                    timestamp,
                    pressure_pa: message.abs_pressure_hpa * 100.0,
                    temperature_c: message.temperature_c,
                }));
            }
        }
        DecodedMessage::HilGps(message) => {
            if message.fix_type >= 2 {
                let _ = out.push(HilIngressMessage::GpsSample(HilGpsSample {
                    timestamp: MeasurementTimestamp::from_micros(message.time_usec),
                    lat_deg: f64::from(message.lat_deg_e7) / 1.0e7,
                    lon_deg: f64::from(message.lon_deg_e7) / 1.0e7,
                    alt_m: (message.alt_mm as f32) / 1_000.0,
                    vel_ned_mps: [
                        (message.vn_cm_s as f32) / 100.0,
                        (message.ve_cm_s as f32) / 100.0,
                        (message.vd_cm_s as f32) / 100.0,
                    ],
                    sats: message.satellites_visible,
                }));
            }
        }
        DecodedMessage::CommandLong(message) => {
            if let Some(command) =
                decode_hil_control_command(message, frame.source_system, frame.source_component)
            {
                let _ = out.push(HilIngressMessage::ControlCommand(command));
            }
        }
    }
}

fn decode_hil_control_command(
    message: CommandLongMessage,
    source_system: u8,
    source_component: u8,
) -> Option<HilControlCommand> {
    match message.command {
        MAV_CMD_MARV_ENTER_HIL_MODE => {
            let action = if matches!(
                SensorBackend::from_wire_param(message.params[0]),
                Some(SensorBackend::Hil)
            ) {
                HilControlAction::EnterHilMode
            } else {
                HilControlAction::InvalidPayload
            };

            Some(HilControlCommand {
                command_id: message.command,
                action,
                source_system,
                source_component,
                target_system: message.target_system,
                target_component: message.target_component,
                confirmation: message.confirmation,
            })
        }
        MAV_CMD_MARV_SELECT_HIL_SUBMODE => {
            let action = if let Some(submode) = HilSubmode::from_wire_param(message.params[0]) {
                HilControlAction::SelectSubmode(submode)
            } else {
                HilControlAction::InvalidPayload
            };

            Some(HilControlCommand {
                command_id: message.command,
                action,
                source_system,
                source_component,
                target_system: message.target_system,
                target_component: message.target_component,
                confirmation: message.confirmation,
            })
        }
        _ => None,
    }
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadMagic(value) => write!(f, "invalid MAVLink magic byte: {value:#x}"),
            Self::FrameTooShort => write!(f, "MAVLink frame too short"),
            Self::UnsupportedVersionFlags(flags) => {
                write!(f, "unsupported MAVLink incompat flags: {flags:#x}")
            }
            Self::ChecksumMismatch { expected, actual } => {
                write!(
                    f,
                    "MAVLink checksum mismatch: expected {expected:#06x}, got {actual:#06x}"
                )
            }
            Self::UnsupportedMessage { id, payload_len } => {
                write!(
                    f,
                    "unsupported MAVLink message id {id} with payload len {payload_len}"
                )
            }
        }
    }
}

pub fn decode_frame(bytes: &[u8]) -> Result<MavlinkFrame, DecodeError> {
    if bytes.is_empty() {
        return Err(DecodeError::FrameTooShort);
    }

    match bytes[0] {
        MAVLINK_V2_STX => decode_v2_frame(bytes),
        MAVLINK_V1_STX => decode_v1_frame(bytes),
        other => Err(DecodeError::BadMagic(other)),
    }
}

pub fn try_consume_frame(bytes: &[u8]) -> Result<Option<(MavlinkFrame, usize)>, DecodeError> {
    if bytes.is_empty() {
        return Ok(None);
    }

    let frame_len = match bytes[0] {
        MAVLINK_V2_STX => v2_frame_len(bytes)?,
        MAVLINK_V1_STX => v1_frame_len(bytes),
        other => return Err(DecodeError::BadMagic(other)),
    };
    let Some(frame_len) = frame_len else {
        return Ok(None);
    };
    if bytes.len() < frame_len {
        return Ok(None);
    }

    let frame = decode_frame(&bytes[..frame_len])?;
    Ok(Some((frame, frame_len)))
}

fn decode_v2_frame(bytes: &[u8]) -> Result<MavlinkFrame, DecodeError> {
    if bytes.len() < 1 + MAVLINK_V2_HEADER_LEN + MAVLINK_CHECKSUM_LEN {
        return Err(DecodeError::FrameTooShort);
    }

    let payload_len = bytes[1] as usize;
    let incompat_flags = bytes[2];
    if incompat_flags & !MAVLINK_IFLAG_SIGNED != 0 {
        return Err(DecodeError::UnsupportedVersionFlags(incompat_flags));
    }

    let signed_len = if incompat_flags & MAVLINK_IFLAG_SIGNED != 0 {
        MAVLINK_SIGNATURE_LEN
    } else {
        0
    };
    let frame_len = 1 + MAVLINK_V2_HEADER_LEN + payload_len + MAVLINK_CHECKSUM_LEN + signed_len;
    if bytes.len() < frame_len {
        return Err(DecodeError::FrameTooShort);
    }

    let message_id = u32::from(bytes[7]) | (u32::from(bytes[8]) << 8) | (u32::from(bytes[9]) << 16);
    let payload = &bytes[10..10 + payload_len];
    let checksum_offset = 10 + payload_len;
    let actual_checksum = read_u16_le(bytes, checksum_offset);
    let crc_extra = crc_extra_for(message_id).ok_or(DecodeError::UnsupportedMessage {
        id: message_id,
        payload_len,
    })?;
    let expected_checksum = checksum_v2(&bytes[1..10], payload, crc_extra);
    if expected_checksum != actual_checksum {
        return Err(DecodeError::ChecksumMismatch {
            expected: expected_checksum,
            actual: actual_checksum,
        });
    }

    let mut frame = decode_payload(message_id, payload_len, payload)?;
    frame.source_system = bytes[5];
    frame.source_component = bytes[6];
    Ok(frame)
}

fn decode_v1_frame(bytes: &[u8]) -> Result<MavlinkFrame, DecodeError> {
    if bytes.len() < 1 + MAVLINK_V1_HEADER_LEN + MAVLINK_CHECKSUM_LEN {
        return Err(DecodeError::FrameTooShort);
    }

    let payload_len = bytes[1] as usize;
    let frame_len = 1 + MAVLINK_V1_HEADER_LEN + payload_len + MAVLINK_CHECKSUM_LEN;
    if bytes.len() < frame_len {
        return Err(DecodeError::FrameTooShort);
    }

    let message_id = u32::from(bytes[5]);
    let payload = &bytes[6..6 + payload_len];
    let checksum_offset = 6 + payload_len;
    let actual_checksum = read_u16_le(bytes, checksum_offset);
    let crc_extra = crc_extra_for(message_id).ok_or(DecodeError::UnsupportedMessage {
        id: message_id,
        payload_len,
    })?;
    let expected_checksum = checksum_v1(&bytes[1..6], payload, crc_extra);
    if expected_checksum != actual_checksum {
        return Err(DecodeError::ChecksumMismatch {
            expected: expected_checksum,
            actual: actual_checksum,
        });
    }

    let mut frame = decode_payload(message_id, payload_len, payload)?;
    frame.source_system = bytes[3];
    frame.source_component = bytes[4];
    Ok(frame)
}

fn decode_payload(
    message_id: u32,
    payload_len: usize,
    payload: &[u8],
) -> Result<MavlinkFrame, DecodeError> {
    let message = match message_id {
        SYSTEM_TIME_ID if payload_len <= SYSTEM_TIME_LEN => {
            DecodedMessage::SystemTime(SystemTimeMessage {
                time_boot_ms: read_u32_padded(payload, 8),
            })
        }
        HIL_SENSOR_ID if payload_len <= HIL_SENSOR_LEN => {
            DecodedMessage::HilSensor(HilSensorMessage {
                time_usec: read_u64_padded(payload, 0),
                accel_mps2: [
                    read_f32_padded(payload, 8),
                    read_f32_padded(payload, 12),
                    read_f32_padded(payload, 16),
                ],
                gyro_rad_s: [
                    read_f32_padded(payload, 20),
                    read_f32_padded(payload, 24),
                    read_f32_padded(payload, 28),
                ],
                abs_pressure_hpa: read_f32_padded(payload, 44),
                pressure_alt_m: read_f32_padded(payload, 52),
                temperature_c: read_f32_padded(payload, 56),
                fields_updated: read_u32_padded(payload, 60),
            })
        }
        HIL_GPS_ID if payload_len <= HIL_GPS_BASE_LEN => DecodedMessage::HilGps(HilGpsMessage {
            time_usec: read_u64_padded(payload, 0),
            lat_deg_e7: read_i32_padded(payload, 8),
            lon_deg_e7: read_i32_padded(payload, 12),
            alt_mm: read_i32_padded(payload, 16),
            eph: read_u16_padded(payload, 20),
            epv: read_u16_padded(payload, 22),
            vel_cm_s: read_u16_padded(payload, 24),
            vn_cm_s: read_i16_padded(payload, 26),
            ve_cm_s: read_i16_padded(payload, 28),
            vd_cm_s: read_i16_padded(payload, 30),
            cog_cdeg: read_u16_padded(payload, 32),
            fix_type: read_u8_padded(payload, 34),
            satellites_visible: read_u8_padded(payload, 35),
        }),
        COMMAND_LONG_ID if payload_len <= COMMAND_LONG_LEN => {
            let mut params = [0.0; 7];
            for (index, param) in params.iter_mut().enumerate() {
                *param = read_f32_padded(payload, index * 4);
            }

            DecodedMessage::CommandLong(CommandLongMessage {
                params,
                command: read_u16_padded(payload, 28),
                target_system: read_u8_padded(payload, 30),
                target_component: read_u8_padded(payload, 31),
                confirmation: read_u8_padded(payload, 32),
            })
        }
        _ => {
            return Err(DecodeError::UnsupportedMessage {
                id: message_id,
                payload_len,
            });
        }
    };

    Ok(MavlinkFrame {
        source_system: 0,
        source_component: 0,
        message,
    })
}

fn v2_frame_len(bytes: &[u8]) -> Result<Option<usize>, DecodeError> {
    if bytes.len() < 3 {
        return Ok(None);
    }

    let payload_len = bytes[1] as usize;
    let incompat_flags = bytes[2];
    if incompat_flags & !MAVLINK_IFLAG_SIGNED != 0 {
        return Err(DecodeError::UnsupportedVersionFlags(incompat_flags));
    }

    let signed_len = if incompat_flags & MAVLINK_IFLAG_SIGNED != 0 {
        MAVLINK_SIGNATURE_LEN
    } else {
        0
    };

    Ok(Some(
        1 + MAVLINK_V2_HEADER_LEN + payload_len + MAVLINK_CHECKSUM_LEN + signed_len,
    ))
}

fn v1_frame_len(bytes: &[u8]) -> Option<usize> {
    if bytes.len() < 2 {
        return None;
    }

    Some(1 + MAVLINK_V1_HEADER_LEN + usize::from(bytes[1]) + MAVLINK_CHECKSUM_LEN)
}

pub fn encode_actuator_control_target(
    message: &ActuatorControlTargetMessage,
    sequence: u8,
    system_id: u8,
    component_id: u8,
) -> Vec<u8, MAX_MAVLINK_FRAME_LEN> {
    let mut frame = Vec::<u8, MAX_MAVLINK_FRAME_LEN>::new();
    let payload_len = 41u8;

    let _ = frame.push(MAVLINK_V2_STX);
    let _ = frame.push(payload_len);
    let _ = frame.push(0);
    let _ = frame.push(0);
    let _ = frame.push(sequence);
    let _ = frame.push(system_id);
    let _ = frame.push(component_id);
    let _ = frame.push((ACTUATOR_CONTROL_TARGET_ID & 0xFF) as u8);
    let _ = frame.push(((ACTUATOR_CONTROL_TARGET_ID >> 8) & 0xFF) as u8);
    let _ = frame.push(((ACTUATOR_CONTROL_TARGET_ID >> 16) & 0xFF) as u8);
    extend_slice(&mut frame, &message.time_usec.to_le_bytes());
    for control in message.controls {
        extend_slice(&mut frame, &control.to_le_bytes());
    }
    let _ = frame.push(message.group_mlx);
    let checksum = checksum_v2(
        &frame[1..10],
        &frame[10..51],
        CRC_EXTRA_ACTUATOR_CONTROL_TARGET,
    );
    extend_slice(&mut frame, &checksum.to_le_bytes());
    frame
}

pub fn encode_command_long(
    message: &CommandLongMessage,
    sequence: u8,
    system_id: u8,
    component_id: u8,
) -> Vec<u8, MAX_MAVLINK_FRAME_LEN> {
    let mut frame = Vec::<u8, MAX_MAVLINK_FRAME_LEN>::new();

    let _ = frame.push(MAVLINK_V2_STX);
    let _ = frame.push(COMMAND_LONG_LEN as u8);
    let _ = frame.push(0);
    let _ = frame.push(0);
    let _ = frame.push(sequence);
    let _ = frame.push(system_id);
    let _ = frame.push(component_id);
    let _ = frame.push((COMMAND_LONG_ID & 0xFF) as u8);
    let _ = frame.push(((COMMAND_LONG_ID >> 8) & 0xFF) as u8);
    let _ = frame.push(((COMMAND_LONG_ID >> 16) & 0xFF) as u8);
    for param in message.params {
        extend_slice(&mut frame, &param.to_le_bytes());
    }
    extend_slice(&mut frame, &message.command.to_le_bytes());
    let _ = frame.push(message.target_system);
    let _ = frame.push(message.target_component);
    let _ = frame.push(message.confirmation);
    let checksum = checksum_v2(
        &frame[1..10],
        &frame[10..10 + COMMAND_LONG_LEN],
        CRC_EXTRA_COMMAND_LONG,
    );
    extend_slice(&mut frame, &checksum.to_le_bytes());
    frame
}

pub fn encode_command_ack(
    message: &CommandAckMessage,
    sequence: u8,
    system_id: u8,
    component_id: u8,
) -> Vec<u8, MAX_MAVLINK_FRAME_LEN> {
    let mut frame = Vec::<u8, MAX_MAVLINK_FRAME_LEN>::new();

    let _ = frame.push(MAVLINK_V2_STX);
    let _ = frame.push(COMMAND_ACK_LEN as u8);
    let _ = frame.push(0);
    let _ = frame.push(0);
    let _ = frame.push(sequence);
    let _ = frame.push(system_id);
    let _ = frame.push(component_id);
    let _ = frame.push((COMMAND_ACK_ID & 0xFF) as u8);
    let _ = frame.push(((COMMAND_ACK_ID >> 8) & 0xFF) as u8);
    let _ = frame.push(((COMMAND_ACK_ID >> 16) & 0xFF) as u8);
    extend_slice(&mut frame, &message.command.to_le_bytes());
    let _ = frame.push(message.result);
    let _ = frame.push(message.progress);
    extend_slice(&mut frame, &message.result_param2.to_le_bytes());
    let _ = frame.push(message.target_system);
    let _ = frame.push(message.target_component);
    let checksum = checksum_v2(
        &frame[1..10],
        &frame[10..10 + COMMAND_ACK_LEN],
        CRC_EXTRA_COMMAND_ACK,
    );
    extend_slice(&mut frame, &checksum.to_le_bytes());
    frame
}

pub fn encode_enter_hil_mode_command(
    target_system: u8,
    target_component: u8,
    confirmation: u8,
    sequence: u8,
    system_id: u8,
    component_id: u8,
) -> Vec<u8, MAX_MAVLINK_FRAME_LEN> {
    let mut params = [0.0; 7];
    params[0] = SensorBackend::Hil.wire_code() as f32;

    encode_command_long(
        &CommandLongMessage {
            params,
            command: MAV_CMD_MARV_ENTER_HIL_MODE,
            target_system,
            target_component,
            confirmation,
        },
        sequence,
        system_id,
        component_id,
    )
}

pub fn encode_hil_backend_command(
    backend: SensorBackend,
    target_system: u8,
    target_component: u8,
    confirmation: u8,
    sequence: u8,
    system_id: u8,
    component_id: u8,
) -> Vec<u8, MAX_MAVLINK_FRAME_LEN> {
    let mut params = [0.0; 7];
    params[0] = backend.wire_code() as f32;

    encode_command_long(
        &CommandLongMessage {
            params,
            command: MAV_CMD_MARV_ENTER_HIL_MODE,
            target_system,
            target_component,
            confirmation,
        },
        sequence,
        system_id,
        component_id,
    )
}

pub fn encode_hil_submode_command(
    submode: HilSubmode,
    target_system: u8,
    target_component: u8,
    confirmation: u8,
    sequence: u8,
    system_id: u8,
    component_id: u8,
) -> Vec<u8, MAX_MAVLINK_FRAME_LEN> {
    let mut params = [0.0; 7];
    params[0] = submode.wire_code() as f32;

    encode_command_long(
        &CommandLongMessage {
            params,
            command: MAV_CMD_MARV_SELECT_HIL_SUBMODE,
            target_system,
            target_component,
            confirmation,
        },
        sequence,
        system_id,
        component_id,
    )
}

fn extend_slice<const N: usize>(frame: &mut Vec<u8, N>, bytes: &[u8]) {
    for byte in bytes {
        let _ = frame.push(*byte);
    }
}

fn crc_extra_for(message_id: u32) -> Option<u8> {
    match message_id {
        SYSTEM_TIME_ID => Some(CRC_EXTRA_SYSTEM_TIME),
        HIL_SENSOR_ID => Some(CRC_EXTRA_HIL_SENSOR),
        HIL_GPS_ID => Some(CRC_EXTRA_HIL_GPS),
        ACTUATOR_CONTROL_TARGET_ID => Some(CRC_EXTRA_ACTUATOR_CONTROL_TARGET),
        COMMAND_LONG_ID => Some(CRC_EXTRA_COMMAND_LONG),
        COMMAND_ACK_ID => Some(CRC_EXTRA_COMMAND_ACK),
        _ => None,
    }
}

fn mav_result_wire_code(result: HilCommandAckResult) -> u8 {
    match result {
        HilCommandAckResult::Accepted => 0,
        HilCommandAckResult::TemporarilyRejected => 1,
        HilCommandAckResult::Denied => 2,
        HilCommandAckResult::Unsupported => 3,
        HilCommandAckResult::Failed => 4,
        HilCommandAckResult::InProgress => 5,
        HilCommandAckResult::Cancelled => 6,
        HilCommandAckResult::CommandLongOnly => 7,
        HilCommandAckResult::CommandIntOnly => 8,
        HilCommandAckResult::UnsupportedMavFrame => 9,
    }
}

fn checksum_v2(header_without_magic: &[u8], payload: &[u8], crc_extra: u8) -> u16 {
    let mut crc = 0xFFFFu16;
    for byte in header_without_magic
        .iter()
        .chain(payload.iter())
        .chain([crc_extra].iter())
    {
        x25_accumulate(*byte, &mut crc);
    }
    crc
}

fn checksum_v1(header_without_magic: &[u8], payload: &[u8], crc_extra: u8) -> u16 {
    let mut crc = 0xFFFFu16;
    for byte in header_without_magic
        .iter()
        .chain(payload.iter())
        .chain([crc_extra].iter())
    {
        x25_accumulate(*byte, &mut crc);
    }
    crc
}

fn x25_accumulate(byte: u8, crc: &mut u16) {
    let tmp = byte ^ (*crc as u8);
    let tmp = tmp ^ (tmp << 4);
    *crc = (*crc >> 8) ^ (u16::from(tmp) << 8) ^ (u16::from(tmp) << 3) ^ (u16::from(tmp) >> 4);
}

fn read_u16_le(bytes: &[u8], offset: usize) -> u16 {
    let mut buf = [0u8; 2];
    buf.copy_from_slice(&bytes[offset..offset + 2]);
    u16::from_le_bytes(buf)
}

fn read_u8_padded(bytes: &[u8], offset: usize) -> u8 {
    bytes.get(offset).copied().unwrap_or(0)
}

fn read_u64_padded(bytes: &[u8], offset: usize) -> u64 {
    let mut buf = [0u8; 8];
    copy_padded(bytes, offset, &mut buf);
    u64::from_le_bytes(buf)
}

fn read_u32_padded(bytes: &[u8], offset: usize) -> u32 {
    let mut buf = [0u8; 4];
    copy_padded(bytes, offset, &mut buf);
    u32::from_le_bytes(buf)
}

fn read_i32_padded(bytes: &[u8], offset: usize) -> i32 {
    let mut buf = [0u8; 4];
    copy_padded(bytes, offset, &mut buf);
    i32::from_le_bytes(buf)
}

fn read_u16_padded(bytes: &[u8], offset: usize) -> u16 {
    let mut buf = [0u8; 2];
    copy_padded(bytes, offset, &mut buf);
    u16::from_le_bytes(buf)
}

fn read_i16_padded(bytes: &[u8], offset: usize) -> i16 {
    let mut buf = [0u8; 2];
    copy_padded(bytes, offset, &mut buf);
    i16::from_le_bytes(buf)
}

fn read_f32_padded(bytes: &[u8], offset: usize) -> f32 {
    let mut buf = [0u8; 4];
    copy_padded(bytes, offset, &mut buf);
    f32::from_le_bytes(buf)
}

fn copy_padded(bytes: &[u8], offset: usize, out: &mut [u8]) {
    if offset >= bytes.len() {
        return;
    }

    let count = (bytes.len() - offset).min(out.len());
    out[..count].copy_from_slice(&bytes[offset..offset + count]);
}

#[cfg(test)]
mod tests {
    use heapless::Vec;

    use super::{
        ACTUATOR_CONTROL_TARGET_ID, ActuatorControlTargetMessage, COMMAND_ACK_ID, COMMAND_LONG_ID,
        CRC_EXTRA_SYSTEM_TIME, CommandAckMessage, CommandLongMessage, DecodeError, DecodedMessage,
        MAV_CMD_MARV_ENTER_HIL_MODE, MAV_CMD_MARV_SELECT_HIL_SUBMODE, MAV_CMD_MARV_SET_HIL_BACKEND,
        MAVLINK_V1_STX, MAVLINK_V2_STX, SYSTEM_TIME_ID, checksum_v1, checksum_v2, decode_frame,
        encode_actuator_control_target, encode_command_ack, encode_command_long,
        encode_hil_backend_command, encode_hil_submode_command, extend_slice,
        frame_to_hil_messages, mav_result_wire_code, try_consume_frame,
    };
    use crate::services::hil::backend::SensorBackend;
    use crate::services::hil::model::{
        HilCommandAckResult, HilControlAction, HilIngressMessage, HilSubmode,
    };

    #[test]
    fn actuator_encoding_places_group_after_controls() {
        let frame = encode_actuator_control_target(
            &ActuatorControlTargetMessage {
                time_usec: 42,
                group_mlx: 1,
                controls: [0.1, -0.2, 0.3, 0.4, 0.0, 1.0, -1.0, 0.5],
            },
            7,
            42,
            1,
        );

        assert_eq!(frame[0], MAVLINK_V2_STX);
        assert_eq!(frame[1], 41);
        assert_eq!(
            u32::from(frame[7]) | (u32::from(frame[8]) << 8) | (u32::from(frame[9]) << 16),
            ACTUATOR_CONTROL_TARGET_ID
        );
        assert_eq!(frame[50], 1);
    }

    #[test]
    fn command_long_encoding_places_command_after_params() {
        let frame = encode_command_long(
            &CommandLongMessage {
                params: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
                command: 31_000,
                target_system: 0,
                target_component: 0,
                confirmation: 0,
            },
            9,
            42,
            7,
        );

        assert_eq!(frame[0], MAVLINK_V2_STX);
        assert_eq!(frame[1], 33);
        assert_eq!(
            u32::from(frame[7]) | (u32::from(frame[8]) << 8) | (u32::from(frame[9]) << 16),
            COMMAND_LONG_ID
        );
        assert_eq!(u16::from_le_bytes([frame[38], frame[39]]), 31_000);
    }

    #[test]
    fn decode_maps_enter_hil_command_long_into_hil_control_command() {
        let frame = encode_hil_backend_command(SensorBackend::Hil, 42, 7, 1, 9, 12, 3);
        let decoded = decode_frame(&frame).unwrap();

        let mut messages = Vec::<HilIngressMessage, 4>::new();
        frame_to_hil_messages(decoded, &mut messages);

        assert_eq!(messages.len(), 1);
        assert!(matches!(
            messages[0],
            HilIngressMessage::ControlCommand(command)
                if command.source_system == 12
                    && command.source_component == 3
                    && command.target_system == 42
                    && command.target_component == 7
                    && command.confirmation == 1
                    && matches!(command.action, HilControlAction::EnterHilMode)
        ));
    }

    #[test]
    fn decode_maps_submode_command_long_into_hil_control_command() {
        let frame = encode_hil_submode_command(HilSubmode::StateEstimation, 42, 7, 1, 9, 12, 3);
        let decoded = decode_frame(&frame).unwrap();

        let mut messages = Vec::<HilIngressMessage, 4>::new();
        frame_to_hil_messages(decoded, &mut messages);

        assert_eq!(messages.len(), 1);
        assert!(matches!(
            messages[0],
            HilIngressMessage::ControlCommand(command)
                if command.source_system == 12
                    && command.source_component == 3
                    && command.target_system == 42
                    && command.target_component == 7
                    && command.confirmation == 1
                    && matches!(
                        command.action,
                        HilControlAction::SelectSubmode(HilSubmode::StateEstimation)
                    )
        ));
    }

    #[test]
    fn decode_maps_invalid_enter_hil_payload_into_invalid_control_command() {
        let frame = encode_hil_backend_command(SensorBackend::Real, 42, 7, 1, 9, 12, 3);
        let decoded = decode_frame(&frame).unwrap();

        let mut messages = Vec::<HilIngressMessage, 4>::new();
        frame_to_hil_messages(decoded, &mut messages);

        assert_eq!(messages.len(), 1);
        assert!(matches!(
            messages[0],
            HilIngressMessage::ControlCommand(command)
                if matches!(command.action, HilControlAction::InvalidPayload)
        ));
    }

    #[test]
    fn decode_maps_invalid_submode_payload_into_invalid_control_command() {
        let mut params = [0.0; 7];
        params[0] = 99.0;
        let frame = encode_command_long(
            &CommandLongMessage {
                params,
                command: MAV_CMD_MARV_SELECT_HIL_SUBMODE,
                target_system: 42,
                target_component: 7,
                confirmation: 1,
            },
            9,
            12,
            3,
        );
        let decoded = decode_frame(&frame).unwrap();

        let mut messages = Vec::<HilIngressMessage, 4>::new();
        frame_to_hil_messages(decoded, &mut messages);

        assert_eq!(messages.len(), 1);
        assert!(matches!(
            messages[0],
            HilIngressMessage::ControlCommand(command)
                if matches!(command.action, HilControlAction::InvalidPayload)
        ));
    }

    #[test]
    fn command_ack_encoding_emits_real_mavlink_command_ack_frame() {
        let frame = encode_command_ack(
            &CommandAckMessage {
                command: MAV_CMD_MARV_SET_HIL_BACKEND,
                result: 2,
                progress: 0,
                result_param2: 0,
                target_system: 12,
                target_component: 3,
            },
            9,
            42,
            7,
        );

        assert_eq!(frame[0], MAVLINK_V2_STX);
        assert_eq!(frame[1], 10);
        assert_eq!(
            u32::from(frame[7]) | (u32::from(frame[8]) << 8) | (u32::from(frame[9]) << 16),
            COMMAND_ACK_ID
        );
        assert_eq!(
            u16::from_le_bytes([frame[10], frame[11]]),
            MAV_CMD_MARV_SET_HIL_BACKEND
        );
        assert_eq!(frame[12], 2);
        assert_eq!(frame[18], 12);
        assert_eq!(frame[19], 3);
    }

    #[test]
    fn command_ack_result_codes_match_mav_result_wire_values() {
        assert_eq!(mav_result_wire_code(HilCommandAckResult::Accepted), 0);
        assert_eq!(mav_result_wire_code(HilCommandAckResult::Denied), 2);
        assert_eq!(mav_result_wire_code(HilCommandAckResult::Unsupported), 3);
        assert_eq!(mav_result_wire_code(HilCommandAckResult::Failed), 4);
        assert_eq!(mav_result_wire_code(HilCommandAckResult::InProgress), 5);
        assert_eq!(mav_result_wire_code(HilCommandAckResult::Cancelled), 6);
        assert_eq!(
            mav_result_wire_code(HilCommandAckResult::CommandLongOnly),
            7
        );
        assert_eq!(mav_result_wire_code(HilCommandAckResult::CommandIntOnly), 8);
        assert_eq!(
            mav_result_wire_code(HilCommandAckResult::UnsupportedMavFrame),
            9
        );
    }

    #[test]
    fn command_long_decode_preserves_backend_switch_command_id() {
        let frame = encode_hil_backend_command(SensorBackend::Replay, 1, 2, 0, 3, 4, 5);
        let decoded = decode_frame(&frame).unwrap();

        assert!(matches!(
            decoded.message,
            DecodedMessage::CommandLong(CommandLongMessage {
                command: MAV_CMD_MARV_ENTER_HIL_MODE,
                ..
            })
        ));
    }

    #[test]
    fn command_long_decode_preserves_select_submode_command_id() {
        let frame = encode_hil_submode_command(HilSubmode::FullRun, 1, 2, 0, 3, 4, 5);
        let decoded = decode_frame(&frame).unwrap();

        assert!(matches!(
            decoded.message,
            DecodedMessage::CommandLong(CommandLongMessage {
                command: MAV_CMD_MARV_SELECT_HIL_SUBMODE,
                ..
            })
        ));
    }

    #[test]
    fn decode_rejects_non_mavlink_data() {
        let error = decode_frame(&[0x00, 0x01]).unwrap_err();
        assert!(matches!(error, DecodeError::BadMagic(0x00)));
    }

    #[test]
    fn try_consume_frame_waits_for_complete_v2_frame() {
        let frame = encode_system_time_v2(123, 7, 42, 1);
        let partial = try_consume_frame(&frame[..5]).unwrap();
        assert!(partial.is_none());

        let consumed = try_consume_frame(&frame).unwrap().unwrap();
        assert_eq!(consumed.1, frame.len());
        assert!(matches!(
            consumed.0.message,
            DecodedMessage::SystemTime(message) if message.time_boot_ms == 123
        ));
    }

    #[test]
    fn decode_accepts_v1_system_time_frame() {
        let frame = encode_system_time_v1(456, 7, 42, 1);
        let decoded = decode_frame(&frame).unwrap();
        assert!(matches!(
            decoded.message,
            DecodedMessage::SystemTime(message) if message.time_boot_ms == 456
        ));
    }

    fn encode_system_time_v2(
        time_boot_ms: u32,
        sequence: u8,
        system_id: u8,
        component_id: u8,
    ) -> Vec<u8, 32> {
        let mut frame = Vec::<u8, 32>::new();
        let _ = frame.push(MAVLINK_V2_STX);
        let _ = frame.push(12);
        let _ = frame.push(0);
        let _ = frame.push(0);
        let _ = frame.push(sequence);
        let _ = frame.push(system_id);
        let _ = frame.push(component_id);
        let _ = frame.push((SYSTEM_TIME_ID & 0xFF) as u8);
        let _ = frame.push(((SYSTEM_TIME_ID >> 8) & 0xFF) as u8);
        let _ = frame.push(((SYSTEM_TIME_ID >> 16) & 0xFF) as u8);
        extend_slice(&mut frame, &0u64.to_le_bytes());
        extend_slice(&mut frame, &time_boot_ms.to_le_bytes());
        let checksum = checksum_v2(&frame[1..10], &frame[10..22], CRC_EXTRA_SYSTEM_TIME);
        extend_slice(&mut frame, &checksum.to_le_bytes());
        frame
    }

    fn encode_system_time_v1(
        time_boot_ms: u32,
        sequence: u8,
        system_id: u8,
        component_id: u8,
    ) -> Vec<u8, 24> {
        let mut frame = Vec::<u8, 24>::new();
        let _ = frame.push(MAVLINK_V1_STX);
        let _ = frame.push(12);
        let _ = frame.push(sequence);
        let _ = frame.push(system_id);
        let _ = frame.push(component_id);
        let _ = frame.push(SYSTEM_TIME_ID as u8);
        extend_slice(&mut frame, &0u64.to_le_bytes());
        extend_slice(&mut frame, &time_boot_ms.to_le_bytes());
        let checksum = checksum_v1(&frame[1..6], &frame[6..18], CRC_EXTRA_SYSTEM_TIME);
        extend_slice(&mut frame, &checksum.to_le_bytes());
        frame
    }
}
