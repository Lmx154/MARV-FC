//! MAVLink framing and message support belongs here.

use core::fmt;

use heapless::Vec;

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

const CRC_EXTRA_SYSTEM_TIME: u8 = 137;
const CRC_EXTRA_HIL_SENSOR: u8 = 108;
const CRC_EXTRA_HIL_GPS: u8 = 124;
const CRC_EXTRA_ACTUATOR_CONTROL_TARGET: u8 = 181;

const HIL_SENSOR_LEN: usize = 64;
const HIL_GPS_BASE_LEN: usize = 36;
const SYSTEM_TIME_LEN: usize = 12;

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
pub enum DecodedMessage {
    SystemTime(SystemTimeMessage),
    HilSensor(HilSensorMessage),
    HilGps(HilGpsMessage),
}

#[derive(Clone, Copy, Debug)]
pub struct MavlinkFrame {
    pub message: DecodedMessage,
}

#[derive(Debug)]
pub enum DecodeError {
    BadMagic(u8),
    FrameTooShort,
    UnsupportedVersionFlags(u8),
    ChecksumMismatch { expected: u16, actual: u16 },
    UnsupportedMessage { id: u32, payload_len: usize },
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

    decode_payload(message_id, payload_len, payload)
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

    decode_payload(message_id, payload_len, payload)
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
        _ => {
            return Err(DecodeError::UnsupportedMessage {
                id: message_id,
                payload_len,
            });
        }
    };

    Ok(MavlinkFrame { message })
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
        _ => None,
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
        ACTUATOR_CONTROL_TARGET_ID, ActuatorControlTargetMessage, CRC_EXTRA_SYSTEM_TIME,
        DecodeError, DecodedMessage, MAVLINK_V1_STX, MAVLINK_V2_STX, SYSTEM_TIME_ID, checksum_v1,
        checksum_v2, decode_frame, encode_actuator_control_target, extend_slice, try_consume_frame,
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
