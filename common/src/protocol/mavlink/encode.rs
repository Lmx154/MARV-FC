//! MAVLink message builders and lightweight helpers.

#![allow(async_fn_in_trait)]

use core::str;

use mavio::dialects::ardupilotmega::messages as ap_messages;
use mavio::dialects::common::enums as common_enums;
use mavio::dialects::common::messages as common_messages;
use mavio::protocol::{Message, V2};
use mavio::Frame;

use crate::params::{ParamDef, ParamRegistry, ParamType, ParamValue, PARAM_DEFS, PARAM_NAME_MAX};
use crate::policies::fc_state::FcStateSnapshot;

/// MAVLink endpoint identifiers.
#[derive(Copy, Clone, Debug)]
pub struct MavEndpointConfig {
    pub sys_id: u8,
    pub comp_id: u8,
}

/// Heartbeat identity configuration.
#[derive(Copy, Clone, Debug)]
pub struct HeartbeatConfig {
    pub mav_type: common_enums::MavType,
    pub autopilot: common_enums::MavAutopilot,
}

impl Default for HeartbeatConfig {
    fn default() -> Self {
        Self {
            mav_type: common_enums::MavType::Generic,
            autopilot: common_enums::MavAutopilot::Generic,
        }
    }
}

/// Maximum STATUSTEXT payload (MAVLink spec).
pub const STATUSTEXT_CAP: usize = 50;

/// Compact telemetry sample used for fast-lane transport.
#[derive(Copy, Clone, Debug, Default)]
pub struct TelemetrySample {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
    pub mag: [i16; 3],
    pub baro_p_pa: i32,
    pub baro_t_cx10: i16,
    pub gps_lat_e7: i32,
    pub gps_lon_e7: i32,
    pub gps_alt_mm: i32,
    pub gps_sat: u8,
    pub gps_fix: u8,
}

/// Source of compact telemetry samples.
pub trait TelemetrySource {
    async fn latest(&self) -> TelemetrySample;
}

/// Build a MAVLink2 frame from a dialect message.
pub fn build_frame_from_msg<M: Message>(
    cfg: MavEndpointConfig,
    seq: u8,
    msg: &M,
) -> Frame<V2> {
    Frame::builder()
        .version(V2)
        .sequence(seq)
        .system_id(cfg.sys_id)
        .component_id(cfg.comp_id)
        .message(msg)
        .expect("mavlink: frame build failed")
        .build()
}

/// Build a STATUSTEXT message with default severity.
pub fn build_statustext(text: &str) -> common_messages::Statustext {
    let mut buf = [0u8; STATUSTEXT_CAP];
    let bytes = text.as_bytes();
    let len = bytes.len().min(STATUSTEXT_CAP);
    buf[..len].copy_from_slice(&bytes[..len]);
    common_messages::Statustext {
        severity: common_enums::MavSeverity::Info,
        text: buf,
        id: 0,
        chunk_seq: 0,
    }
}

/// Convert STATUSTEXT payload to a trimmed string slice.
pub fn statustext_to_str(msg: &common_messages::Statustext) -> Option<&str> {
    let len = msg
        .text
        .iter()
        .position(|&b| b == 0)
        .unwrap_or(msg.text.len());
    str::from_utf8(&msg.text[..len]).ok()
}

/// Build a STATUSTEXT frame.
pub fn build_statustext_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    msg: common_messages::Statustext,
) -> Frame<V2> {
    build_frame_from_msg(cfg, seq, &msg)
}

/// Build a HEARTBEAT frame from the FC state.
pub fn build_heartbeat_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    snapshot: &FcStateSnapshot,
    hb_cfg: HeartbeatConfig,
) -> Frame<V2> {
    let msg = common_messages::Heartbeat {
        type_: hb_cfg.mav_type,
        autopilot: hb_cfg.autopilot,
        base_mode: snapshot.base_mode,
        custom_mode: snapshot.custom_mode,
        system_status: snapshot.system_status,
        mavlink_version: 3,
    };
    build_frame_from_msg(cfg, seq, &msg)
}

/// Build a PARAM_VALUE frame for a raw parameter index (returns None for hidden params).
pub fn build_param_value_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    params: &ParamRegistry,
    raw_idx: u16,
) -> Option<Frame<V2>> {
    let def = params.def_by_index(raw_idx)?;
    if !is_param_visible(def) {
        return None;
    }
    let val = params.get_by_index(raw_idx)?;
    let mav_idx = visible_index_from_raw(raw_idx)?;
    let msg = common_messages::ParamValue {
        param_id: param_name_bytes(def.name),
        param_value: val.as_mavlink_f32(),
        param_type: mav_param_type(def.ty),
        param_count: visible_param_count(),
        param_index: mav_idx,
    };
    Some(build_frame_from_msg(cfg, seq, &msg))
}

/// Build DEVICE_OP_READ_REPLY.
pub fn build_device_op_read_reply_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    request_id: u32,
    result: u8,
    regstart: u8,
    data: &[u8],
) -> Frame<V2> {
    let mut payload = [0u8; 128];
    let count = data.len().min(payload.len());
    payload[..count].copy_from_slice(&data[..count]);
    let msg = ap_messages::DeviceOpReadReply {
        request_id,
        result,
        regstart,
        count: count as u8,
        data: payload,
        bank: 0,
    };
    build_frame_from_msg(cfg, seq, &msg)
}

/// Build DEVICE_OP_WRITE_REPLY.
pub fn build_device_op_write_reply_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    request_id: u32,
    result: u8,
) -> Frame<V2> {
    let msg = ap_messages::DeviceOpWriteReply { request_id, result };
    build_frame_from_msg(cfg, seq, &msg)
}

/// Build SYS_STATUS.
pub fn build_sys_status(_now_ms: u32, sample: &TelemetrySample) -> common_messages::SysStatus {
    let mut sensors = common_enums::MavSysStatusSensor::empty();
    sensors.insert(common_enums::MavSysStatusSensor::_3D_GYRO);
    sensors.insert(common_enums::MavSysStatusSensor::_3D_ACCEL);
    sensors.insert(common_enums::MavSysStatusSensor::_3D_MAG);
    sensors.insert(common_enums::MavSysStatusSensor::ABSOLUTE_PRESSURE);
    if sample.gps_fix != 0 {
        sensors.insert(common_enums::MavSysStatusSensor::GPS);
    }

    common_messages::SysStatus {
        onboard_control_sensors_present: sensors,
        onboard_control_sensors_enabled: sensors,
        onboard_control_sensors_health: sensors,
        load: 0,
        voltage_battery: u16::MAX,
        current_battery: -1,
        battery_remaining: -1,
        drop_rate_comm: 0,
        errors_comm: 0,
        errors_count1: 0,
        errors_count2: 0,
        errors_count3: 0,
        errors_count4: 0,
        onboard_control_sensors_present_extended:
            common_enums::MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_enabled_extended:
            common_enums::MavSysStatusSensorExtended::empty(),
        onboard_control_sensors_health_extended:
            common_enums::MavSysStatusSensorExtended::empty(),
    }
}

/// Build RAW_IMU.
pub fn build_raw_imu(now_us: u64, sample: &TelemetrySample) -> common_messages::RawImu {
    common_messages::RawImu {
        time_usec: now_us,
        xacc: sample.accel[0],
        yacc: sample.accel[1],
        zacc: sample.accel[2],
        xgyro: sample.gyro[0],
        ygyro: sample.gyro[1],
        zgyro: sample.gyro[2],
        xmag: sample.mag[0],
        ymag: sample.mag[1],
        zmag: sample.mag[2],
        id: 0,
        temperature: 0,
    }
}

/// Build SCALED_PRESSURE.
pub fn build_scaled_pressure(
    now_ms: u32,
    sample: &TelemetrySample,
) -> common_messages::ScaledPressure {
    common_messages::ScaledPressure {
        time_boot_ms: now_ms,
        press_abs: (sample.baro_p_pa as f32) / 100.0,
        press_diff: 0.0,
        temperature: (sample.baro_t_cx10 as i32 * 10) as i16,
        temperature_press_diff: 0,
    }
}

/// Build GPS_RAW_INT.
pub fn build_gps_raw_int(
    now_us: u64,
    sample: &TelemetrySample,
) -> common_messages::GpsRawInt {
    common_messages::GpsRawInt {
        time_usec: now_us,
        fix_type: gps_fix_type(sample.gps_fix),
        lat: sample.gps_lat_e7,
        lon: sample.gps_lon_e7,
        alt: sample.gps_alt_mm,
        eph: u16::MAX,
        epv: u16::MAX,
        vel: u16::MAX,
        cog: u16::MAX,
        satellites_visible: sample.gps_sat,
        alt_ellipsoid: 0,
        h_acc: 0,
        v_acc: 0,
        vel_acc: 0,
        hdg_acc: 0,
        yaw: u16::MAX,
    }
}

/// Build ATTITUDE (placeholder zeros).
pub fn build_attitude(now_ms: u32) -> common_messages::Attitude {
    common_messages::Attitude {
        time_boot_ms: now_ms,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        rollspeed: 0.0,
        pitchspeed: 0.0,
        yawspeed: 0.0,
    }
}

/// Build compact telemetry into ENCAPSULATED_DATA.
pub fn build_telemetry_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    _now_us: u64,
    sample: &TelemetrySample,
) -> Option<Frame<V2>> {
    let mut data = [0u8; 253];
    let mut offset: usize = 0;

    data[offset] = 1; // version
    offset += 1;

    write_i16(&mut data, &mut offset, sample.accel[0]);
    write_i16(&mut data, &mut offset, sample.accel[1]);
    write_i16(&mut data, &mut offset, sample.accel[2]);
    write_i16(&mut data, &mut offset, sample.gyro[0]);
    write_i16(&mut data, &mut offset, sample.gyro[1]);
    write_i16(&mut data, &mut offset, sample.gyro[2]);
    write_i16(&mut data, &mut offset, sample.mag[0]);
    write_i16(&mut data, &mut offset, sample.mag[1]);
    write_i16(&mut data, &mut offset, sample.mag[2]);
    write_i32(&mut data, &mut offset, sample.baro_p_pa);
    write_i16(&mut data, &mut offset, sample.baro_t_cx10);
    write_i32(&mut data, &mut offset, sample.gps_lat_e7);
    write_i32(&mut data, &mut offset, sample.gps_lon_e7);
    write_i32(&mut data, &mut offset, sample.gps_alt_mm);
    write_u8(&mut data, &mut offset, sample.gps_sat);
    write_u8(&mut data, &mut offset, sample.gps_fix);

    let msg = common_messages::EncapsulatedData {
        seqnr: seq as u16,
        data,
    };
    Some(build_frame_from_msg(cfg, seq, &msg))
}

fn write_i16(buf: &mut [u8], offset: &mut usize, v: i16) {
    let bytes = v.to_le_bytes();
    buf[*offset..*offset + bytes.len()].copy_from_slice(&bytes);
    *offset += bytes.len();
}

fn write_i32(buf: &mut [u8], offset: &mut usize, v: i32) {
    let bytes = v.to_le_bytes();
    buf[*offset..*offset + bytes.len()].copy_from_slice(&bytes);
    *offset += bytes.len();
}

fn write_u8(buf: &mut [u8], offset: &mut usize, v: u8) {
    buf[*offset] = v;
    *offset += 1;
}

fn gps_fix_type(fix: u8) -> common_enums::GpsFixType {
    match fix {
        2 => common_enums::GpsFixType::_2dFix,
        3 => common_enums::GpsFixType::_3dFix,
        4 => common_enums::GpsFixType::Dgps,
        5 => common_enums::GpsFixType::RtkFloat,
        6 => common_enums::GpsFixType::RtkFixed,
        7 => common_enums::GpsFixType::Static,
        8 => common_enums::GpsFixType::Ppp,
        0 | 1 | _ => common_enums::GpsFixType::NoFix,
    }
}

fn mav_param_type(ty: ParamType) -> common_enums::MavParamType {
    match ty {
        ParamType::Bool => common_enums::MavParamType::Uint8,
        ParamType::I32 => common_enums::MavParamType::Int32,
        ParamType::U32 => common_enums::MavParamType::Uint32,
        ParamType::F32 => common_enums::MavParamType::Real32,
    }
}

fn param_name_bytes(name: &str) -> [u8; PARAM_NAME_MAX] {
    let mut out = [0u8; PARAM_NAME_MAX];
    let bytes = name.as_bytes();
    let len = bytes.len().min(PARAM_NAME_MAX);
    out[..len].copy_from_slice(&bytes[..len]);
    out
}

fn is_param_visible(def: &ParamDef) -> bool {
    match def.name {
        "RAD_TEL_HZ"
        | "TEL_FAST_EN"
        | "TEL_FAST_HZ"
        | "TEL_FAST_MAXB"
        | "TEL_NORM_EN"
        | "TEL_NORM_MAXHZ"
        | "TEL_NORM_QMAX"
        | "TEL_ACK_TMO_MS"
        | "TEL_ACK_RTRY_MAX"
        | "TEL_QOS"
        | "TEL_ADAPT_EN"
        | "TEL_SNR_BAD"
        | "TEL_SNR_GOOD"
        | "TEL_ADAPT_HOLD"
        | "LINK_TICK_HZ"
        | "LINK_SLOT_MD"
        | "LINK_FAST_MB"
        | "LAS_MAX_AGE_MS"
        | "LORA_F_KHZ"
        | "LORA_SF"
        | "LORA_BW"
        | "LORA_CR"
        | "LORA_SW" => false,
        _ => true,
    }
}

fn visible_param_count() -> u16 {
    let mut count = 0u16;
    for def in PARAM_DEFS.iter() {
        if is_param_visible(def) {
            count = count.saturating_add(1);
        }
    }
    count
}

fn visible_index_from_raw(raw_idx: u16) -> Option<u16> {
    let mut visible = 0u16;
    for (idx, def) in PARAM_DEFS.iter().enumerate() {
        if !is_param_visible(def) {
            continue;
        }
        if idx as u16 == raw_idx {
            return Some(visible);
        }
        visible = visible.saturating_add(1);
    }
    None
}

pub(crate) fn raw_index_from_visible(visible_idx: u16) -> Option<u16> {
    let mut visible = 0u16;
    for (idx, def) in PARAM_DEFS.iter().enumerate() {
        if !is_param_visible(def) {
            continue;
        }
        if visible == visible_idx {
            return Some(idx as u16);
        }
        visible = visible.saturating_add(1);
    }
    None
}

pub(crate) fn find_visible_param_by_name(name: &str) -> Option<u16> {
    let raw_idx = PARAM_DEFS
        .iter()
        .position(|def| def.name == name)
        .map(|idx| idx as u16)?;
    let def = PARAM_DEFS.get(raw_idx as usize)?;
    if is_param_visible(def) {
        Some(raw_idx)
    } else {
        None
    }
}

pub(crate) fn parse_param_value(def: &ParamDef, v: f32) -> ParamValue {
    match def.ty {
        ParamType::Bool => ParamValue::Bool(v != 0.0),
        ParamType::I32 => ParamValue::I32(v.to_bits() as i32),
        ParamType::U32 => ParamValue::U32(v.to_bits()),
        ParamType::F32 => ParamValue::F32(v),
    }
}
