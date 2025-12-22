//! MAVLink2 message helpers and light-weight telemetry packing.
//! Keep all MAVLink framing and payload building in this module for reuse.
#![allow(async_fn_in_trait)]

use mavio::dialects::common::enums::{
    GpsFixType, MavAutopilot, MavCmd, MavParamType, MavResult, MavSysStatusSensor, MavType,
};
use mavio::dialects::common::messages;
use mavio::dialects::ardupilotmega as apm;
use mavio::Frame;
use mavio::Message;
use mavio::protocol::V2;

use crate::params::{ParamType, ParamValue, ParamRegistry, PARAM_NAME_MAX};
use crate::policies::fc_state::FcStateSnapshot;

/// MAVLink endpoint identity (system + component IDs).
#[derive(Clone, Copy)]
pub struct MavEndpointConfig {
    pub sys_id: u8,
    pub comp_id: u8,
}

/// Build a MAVLink2 frame from an already-constructed message payload.
///
/// This is used by higher layers that want to decouple message *content* generation
/// (raw sensors vs estimator) from message *transport*.
pub fn build_frame_from_msg<M: Message>(cfg: MavEndpointConfig, seq: u8, msg: &M) -> Frame<V2> {
    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(msg)
        .expect("MAV: build frame")
        .build()
}

// ---------------------------------------------------------------------------
// RADIO_STATUS helpers (link-quality reporting)
// ---------------------------------------------------------------------------

fn clamp_i8(v: i16) -> i8 {
    v.clamp(i8::MIN as i16, i8::MAX as i16) as i8
}

/// Build a MAVLink2 `RADIO_STATUS` frame.
///
/// Notes:
/// - MAVLink `RADIO_STATUS` does not have a dedicated SNR field.
/// - We encode SNR (in dB, derived from `snr_x4/4`) into the `noise` field as a pragmatic
///   way to surface link quality in common GCS tooling.
/// - The remaining fields (`remrssi`, `remnoise`, `rxerrors`, `fixed`) are intentionally
///   provided by the caller so firmwares can pack additional link/MAC metrics without
///   introducing a custom dialect message.
pub fn build_radio_status_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    last_rssi_dbm: Option<i16>,
    last_snr_x4: Option<i16>,
    remrssi: u8,
    txbuf_pct: u8,
    remnoise: u8,
    rxerrors: u16,
    fixed: u16,
) -> Frame<V2> {
    // `mavio` represents MAVLink int8_t fields as `u8` (raw byte storage).
    // Convert signed values to their two's-complement byte form.
    let rssi = clamp_i8(last_rssi_dbm.unwrap_or(0)) as u8;
    let snr_db = clamp_i8(last_snr_x4.unwrap_or(0) / 4) as u8;

    let msg = messages::RadioStatus {
        rssi,
        remrssi,
        txbuf: txbuf_pct,
        // Encoded SNR (dB) for visibility (see note above).
        noise: snr_db,
        remnoise,
        rxerrors,
        fixed,
    };

    build_frame_from_msg(cfg, seq, &msg)
}

// ---------------------------------------------------------------------------
// STATUSTEXT helpers
// ---------------------------------------------------------------------------

/// Maximum text length for `STATUSTEXT`.
pub const STATUSTEXT_CAP: usize = 50;

/// Build a `STATUSTEXT` payload from &str, truncating to the MAVLink limit.
pub fn build_statustext(text: &str) -> messages::Statustext {
    use mavio::dialects::common::enums::MavSeverity;

    let mut text_bytes = [0u8; STATUSTEXT_CAP];
    let raw = text.as_bytes();
    let copy_len = core::cmp::min(raw.len(), text_bytes.len());
    text_bytes[..copy_len].copy_from_slice(&raw[..copy_len]);

    messages::Statustext {
        severity: MavSeverity::Info,
        text: text_bytes,
        id: 0,
        chunk_seq: 0,
    }
}

/// Convert STATUSTEXT payload back into &str-like view for logging.
pub fn statustext_to_str(msg: &messages::Statustext) -> &str {
    let end = msg
        .text
        .iter()
        .position(|&b| b == 0)
        .unwrap_or(msg.text.len());
    core::str::from_utf8(&msg.text[..end]).unwrap_or("<non-utf8>")
}

/// Build a MAVLink2 `Frame<V2>` carrying a `Common::Statustext`.
pub fn build_statustext_frame(cfg: MavEndpointConfig, seq: u8, msg: messages::Statustext) -> Frame<V2> {
    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build Statustext frame")
        .build()
}

// ---------------------------------------------------------------------------
// HEARTBEAT helpers
// ---------------------------------------------------------------------------

/// Static identification fields for HEARTBEAT.
#[derive(Clone, Copy, Debug)]
pub struct HeartbeatConfig {
    pub vehicle_type: MavType,
    pub autopilot: MavAutopilot,
}

impl Default for HeartbeatConfig {
    fn default() -> Self {
        Self {
            vehicle_type: MavType::Quadrotor,
            // Mission Planner UI compatibility is better when we identify as ArduPilot.
            autopilot: MavAutopilot::Ardupilotmega,
        }
    }
}

/// Build a HEARTBEAT message from the current FC state.
pub fn build_heartbeat(state: &FcStateSnapshot, hb_cfg: HeartbeatConfig) -> messages::Heartbeat {
    messages::Heartbeat {
        custom_mode: state.custom_mode,
        type_: hb_cfg.vehicle_type,
        autopilot: hb_cfg.autopilot,
        base_mode: state.base_mode,
        system_status: state.system_status,
        mavlink_version: 3,
    }
}

// ---------------------------------------------------------------------------
// Standard MAVLink telemetry for Mission Planner sensor/status panels
// ---------------------------------------------------------------------------

const SYS_SENSOR_3D_GYRO: u32 = 1 << 0;
const SYS_SENSOR_3D_ACCEL: u32 = 1 << 1;
const SYS_SENSOR_3D_MAG: u32 = 1 << 2;
const SYS_SENSOR_ABS_PRESSURE: u32 = 1 << 3;
const SYS_SENSOR_GPS: u32 = 1 << 5;

fn sensor_present_enabled_mask() -> MavSysStatusSensor {
    let bits = SYS_SENSOR_3D_GYRO
        | SYS_SENSOR_3D_ACCEL
        | SYS_SENSOR_3D_MAG
        | SYS_SENSOR_ABS_PRESSURE
        | SYS_SENSOR_GPS;
    MavSysStatusSensor::from_bits_truncate(bits)
}

/// Build `SYS_STATUS` from currently-available raw sensors.
pub fn build_sys_status(_now_ms: u32, sample: &TelemetrySample) -> messages::SysStatus {
    let present = sensor_present_enabled_mask();
    let enabled = present;

    // Health: mark GPS unhealthy when there is no fix.
    let mut health_bits = present.bits();
    if sample.gps_fix == 0 {
        health_bits &= !SYS_SENSOR_GPS;
    }
    let health = MavSysStatusSensor::from_bits_truncate(health_bits);

    messages::SysStatus {
        onboard_control_sensors_present: present,
        onboard_control_sensors_enabled: enabled,
        onboard_control_sensors_health: health,
        ..Default::default()
    }
}

pub fn build_sys_status_frame(cfg: MavEndpointConfig, seq: u8, now_ms: u32, sample: &TelemetrySample) -> Frame<V2> {
    let msg = build_sys_status(now_ms, sample);
    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build SysStatus frame")
        .build()
}

/// Convert BMI088 accel raw counts (±6g) to milli-g.
fn bmi088_acc_counts_to_mg(counts: i16) -> i16 {
    // 6g full-scale over 16-bit signed range => 6/32768 g/LSB => 6000/32768 mg/LSB.
    // mg ≈ counts * 6000 / 32768.
    let mg = (counts as i32 * 6000) / 32768;
    mg.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// Convert BMM350 raw counts to milli-gauss (approx; trim not applied in driver).
fn bmm350_counts_to_milligauss(counts: i16) -> i16 {
    // Driver scale: 0.001907 uT/LSB. 1 uT = 10 mG => 0.01907 mG/LSB.
    // mG ≈ counts * 1907 / 100000.
    let mg = (counts as i32 * 1907) / 100_000;
    mg.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// Build `RAW_IMU`.
pub fn build_raw_imu(now_us: u64, sample: &TelemetrySample) -> messages::RawImu {
    messages::RawImu {
        time_usec: now_us,
        xacc: bmi088_acc_counts_to_mg(sample.accel[0]),
        yacc: bmi088_acc_counts_to_mg(sample.accel[1]),
        zacc: bmi088_acc_counts_to_mg(sample.accel[2]),
        // Gyro: we currently forward raw counts (range scaling TBD).
        xgyro: sample.gyro[0],
        ygyro: sample.gyro[1],
        zgyro: sample.gyro[2],
        xmag: bmm350_counts_to_milligauss(sample.mag[0]),
        ymag: bmm350_counts_to_milligauss(sample.mag[1]),
        zmag: bmm350_counts_to_milligauss(sample.mag[2]),
        ..Default::default()
    }
}

pub fn build_raw_imu_frame(cfg: MavEndpointConfig, seq: u8, now_us: u64, sample: &TelemetrySample) -> Frame<V2> {
    let msg = build_raw_imu(now_us, sample);
    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build RawImu frame")
        .build()
}

/// Build `SCALED_PRESSURE`.
pub fn build_scaled_pressure(now_ms: u32, sample: &TelemetrySample) -> messages::ScaledPressure {
    let press_hpa = (sample.baro_p_pa as f32) / 100.0;
    let temp_cdeg = (sample.baro_t_cx10 as i32 * 10)
        .clamp(i16::MIN as i32, i16::MAX as i32) as i16;
    messages::ScaledPressure {
        time_boot_ms: now_ms,
        press_abs: press_hpa,
        press_diff: 0.0,
        temperature: temp_cdeg,
        ..Default::default()
    }
}

pub fn build_scaled_pressure_frame(cfg: MavEndpointConfig, seq: u8, now_ms: u32, sample: &TelemetrySample) -> Frame<V2> {
    let msg = build_scaled_pressure(now_ms, sample);
    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build ScaledPressure frame")
        .build()
}

/// Build `GPS_RAW_INT`.
pub fn build_gps_raw_int(now_us: u64, sample: &TelemetrySample) -> messages::GpsRawInt {
    let (eph, epv) = if sample.gps_fix == 0 { (u16::MAX, u16::MAX) } else { (250, 250) };
    let fix_type = gps_fix_type_from_u8(sample.gps_fix);
    messages::GpsRawInt {
        time_usec: now_us,
        fix_type,
        lat: sample.gps_lat_e7,
        lon: sample.gps_lon_e7,
        alt: sample.gps_alt_mm,
        eph,
        epv,
        satellites_visible: sample.gps_sat,
        ..Default::default()
    }
}

fn gps_fix_type_from_u8(v: u8) -> GpsFixType {
    // MAVLink GPS_FIX_TYPE is specified for 0..=8.
    // Clamp to the known range so the transmute stays valid.
    let v = v.min(8);
    // SAFETY: `GpsFixType` is a MAVLink enum represented as u8 with discriminants 0..=8.
    unsafe { core::mem::transmute::<u8, GpsFixType>(v) }
}

pub fn build_gps_raw_int_frame(cfg: MavEndpointConfig, seq: u8, now_us: u64, sample: &TelemetrySample) -> Frame<V2> {
    let msg = build_gps_raw_int(now_us, sample);
    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build GpsRawInt frame")
        .build()
}

/// Build `ATTITUDE`.
///
/// For now we publish neutral attitude. This keeps the message schedule stable
/// for Mission Planner; later we can swap in EKF outputs without changing GCS.
pub fn build_attitude(now_ms: u32) -> messages::Attitude {
    messages::Attitude {
        time_boot_ms: now_ms,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
        rollspeed: 0.0,
        pitchspeed: 0.0,
        yawspeed: 0.0,
    }
}

pub fn build_attitude_frame(cfg: MavEndpointConfig, seq: u8, now_ms: u32) -> Frame<V2> {
    let msg = build_attitude(now_ms);
    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build Attitude frame")
        .build()
}

/// Build a MAVLink2 `Frame<V2>` carrying a `Common::Heartbeat`.
pub fn build_heartbeat_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    state: &FcStateSnapshot,
    hb_cfg: HeartbeatConfig,
) -> Frame<V2> {
    let msg = build_heartbeat(state, hb_cfg);
    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build Heartbeat frame")
        .build()
}

// ---------------------------------------------------------------------------
// COMMAND_ACK helpers
// ---------------------------------------------------------------------------

pub fn build_command_ack_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    command: MavCmd,
    result: MavResult,
    target_system: u8,
    target_component: u8,
) -> Frame<V2> {
    let msg = messages::CommandAck {
        command,
        result,
        progress: 0,
        result_param2: 0,
        target_system,
        target_component,
    };

    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build CommandAck frame")
        .build()
}

// ---------------------------------------------------------------------------
// Device operation helpers (Mission Planner: DEVICE_OP_READ/WRITE)
// ---------------------------------------------------------------------------

/// Build a MAVLink2 `DEVICE_OP_READ_REPLY` frame.
pub fn build_device_op_read_reply_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    request_id: u32,
    result: u8,
    regstart: u8,
    data: &[u8],
) -> Frame<V2> {
    let mut payload = [0u8; 128];
    let count = core::cmp::min(data.len(), payload.len());
    payload[..count].copy_from_slice(&data[..count]);

    let msg = apm::messages::DeviceOpReadReply {
        request_id,
        result,
        regstart,
        count: count as u8,
        data: payload,
        bank: 0,
    };

    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build DeviceOpReadReply frame")
        .build()
}

/// Build a MAVLink2 `DEVICE_OP_WRITE_REPLY` frame.
pub fn build_device_op_write_reply_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    request_id: u32,
    result: u8,
) -> Frame<V2> {
    let msg = apm::messages::DeviceOpWriteReply { request_id, result };

    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id.into())
        .component_id(cfg.comp_id.into())
        .sequence(seq.into())
        .message(&msg)
        .expect("MAV: build DeviceOpWriteReply frame")
        .build()
}

// ---------------------------------------------------------------------------
// Light-weight telemetry payload (EncapsulatedData-based)
// ---------------------------------------------------------------------------

/// Versioned, compact telemetry snapshot meant for UART/LoRa forwarding.
#[derive(Clone, Copy, Default)]
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

impl TelemetrySample {
    pub const FORMAT_VERSION: u8 = 1;
    pub const WIRE_LEN: usize = 39;

    /// Encode into little-endian bytes; returns length used.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < Self::WIRE_LEN {
            return None;
        }
        out[0] = Self::FORMAT_VERSION;
        let mut idx = 1;

        for v in self.accel {
            out[idx..idx + 2].copy_from_slice(&v.to_le_bytes());
            idx += 2;
        }
        for v in self.gyro {
            out[idx..idx + 2].copy_from_slice(&v.to_le_bytes());
            idx += 2;
        }
        for v in self.mag {
            out[idx..idx + 2].copy_from_slice(&v.to_le_bytes());
            idx += 2;
        }

        out[idx..idx + 4].copy_from_slice(&self.baro_p_pa.to_le_bytes());
        idx += 4;
        out[idx..idx + 2].copy_from_slice(&self.baro_t_cx10.to_le_bytes());
        idx += 2;
        out[idx..idx + 4].copy_from_slice(&self.gps_lat_e7.to_le_bytes());
        idx += 4;
        out[idx..idx + 4].copy_from_slice(&self.gps_lon_e7.to_le_bytes());
        idx += 4;
        out[idx..idx + 4].copy_from_slice(&self.gps_alt_mm.to_le_bytes());
        idx += 4;
        out[idx] = self.gps_sat;
        out[idx + 1] = self.gps_fix;

        Some(Self::WIRE_LEN)
    }

    /// Decode from bytes produced by `encode`.
    pub fn decode(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::WIRE_LEN || buf[0] != Self::FORMAT_VERSION {
            return None;
        }
        let mut idx = 1;
        let read_i16 = |b: &[u8], i: &mut usize| {
            let val = i16::from_le_bytes([b[*i], b[*i + 1]]);
            *i += 2;
            val
        };
        let read_i32 = |b: &[u8], i: &mut usize| {
            let val = i32::from_le_bytes([b[*i], b[*i + 1], b[*i + 2], b[*i + 3]]);
            *i += 4;
            val
        };

        let mut accel = [0i16; 3];
        let mut gyro = [0i16; 3];
        let mut mag = [0i16; 3];
        for v in accel.iter_mut() {
            *v = read_i16(buf, &mut idx);
        }
        for v in gyro.iter_mut() {
            *v = read_i16(buf, &mut idx);
        }
        for v in mag.iter_mut() {
            *v = read_i16(buf, &mut idx);
        }

        let baro_p_pa = read_i32(buf, &mut idx);
        let baro_t_cx10 = read_i16(buf, &mut idx);
        let gps_lat_e7 = read_i32(buf, &mut idx);
        let gps_lon_e7 = read_i32(buf, &mut idx);
        let gps_alt_mm = read_i32(buf, &mut idx);
        let gps_sat = buf[idx];
        let gps_fix = buf[idx + 1];

        Some(Self {
            accel,
            gyro,
            mag,
            baro_p_pa,
            baro_t_cx10,
            gps_lat_e7,
            gps_lon_e7,
            gps_alt_mm,
            gps_sat,
            gps_fix,
        })
    }
}

/// Trait for pluggable telemetry sources (raw sensors, estimators, etc.).
pub trait TelemetrySource {
    async fn latest(&self) -> TelemetrySample;
}

/// Build a "FAST telemetry" MAVLink frame.
///
/// Previously this used `ENCAPSULATED_DATA`, but that cannot fit inside the
/// wireless inner MTU (236 bytes) once the MAC header is present.
///
/// For now we use a standard `RAW_IMU` message as a compact carrier.
pub fn build_telemetry_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    now_us: u64,
    sample: &TelemetrySample,
) -> Option<Frame<V2>> {
    Some(build_raw_imu_frame(cfg, seq, now_us, sample))
}

// ---------------------------------------------------------------------------
// Parameter Protocol (PARAM_REQUEST_LIST, PARAM_REQUEST_READ, PARAM_SET, PARAM_VALUE)
// ---------------------------------------------------------------------------

/// Map our `ParamType` to MAVLink's `MavParamType`.
pub fn param_type_to_mav(ty: ParamType) -> MavParamType {
    match ty {
        ParamType::Bool => MavParamType::Uint8,
        ParamType::I32 => MavParamType::Int32,
        ParamType::U32 => MavParamType::Uint32,
        ParamType::F32 => MavParamType::Real32,
    }
}

/// Encode a parameter name into MAVLink's fixed-size [u8; 16] array.
/// Null-terminates and pads with zeros.
pub fn encode_param_id(name: &str) -> [u8; PARAM_NAME_MAX] {
    let mut out = [0u8; PARAM_NAME_MAX];
    let bytes = name.as_bytes();
    let len = core::cmp::min(bytes.len(), PARAM_NAME_MAX - 1);
    out[..len].copy_from_slice(&bytes[..len]);
    out
}

/// Decode a MAVLink param_id[16] back into a &str (up to first null or end of array).
pub fn decode_param_id(param_id: &[u8; PARAM_NAME_MAX]) -> &str {
    let end = param_id.iter().position(|&b| b == 0).unwrap_or(PARAM_NAME_MAX);
    core::str::from_utf8(&param_id[..end]).unwrap_or("")
}

/// Build a PARAM_VALUE message from the registry at a given index.
pub fn build_param_value_msg(registry: &ParamRegistry, idx: u16) -> Option<messages::ParamValue> {
    let def = registry.def_by_index(idx)?;
    let val = registry.get_by_index(idx)?;

    Some(messages::ParamValue {
        param_value: val.as_mavlink_f32(),
        param_count: registry.count(),
        param_index: idx,
        param_id: encode_param_id(def.name),
        param_type: param_type_to_mav(def.ty),
    })
}

/// Build a PARAM_VALUE frame for a single parameter.
pub fn build_param_value_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    registry: &ParamRegistry,
    idx: u16,
) -> Option<Frame<V2>> {
    let msg = build_param_value_msg(registry, idx)?;

    Some(
        Frame::builder()
            .version(V2)
            .system_id(cfg.sys_id.into())
            .component_id(cfg.comp_id.into())
            .sequence(seq.into())
            .message(&msg)
            .expect("MAV: build ParamValue frame")
            .build(),
    )
}

/// Parse a PARAM_SET message and attempt to set the parameter in the registry.
/// Returns the parameter index if successful.
pub fn handle_param_set(registry: &mut ParamRegistry, msg: &messages::ParamSet) -> Option<u16> {
    let name = decode_param_id(&msg.param_id);

    let idx = registry.find_index_by_name(name)?;
    let def = registry.def_by_index(idx)?;

    let new_value = match def.ty {
        ParamType::Bool => ParamValue::Bool(msg.param_value != 0.0),
        ParamType::I32 => ParamValue::I32(msg.param_value.to_bits() as i32),
        ParamType::U32 => ParamValue::U32(msg.param_value.to_bits()),
        ParamType::F32 => ParamValue::F32(msg.param_value),
    };

    registry.set_by_index(idx, new_value).ok()?;

    Some(idx)
}

/// Handle PARAM_REQUEST_READ - returns the index to send back.
/// Supports both index-based and name-based lookups.
pub fn handle_param_request_read(registry: &ParamRegistry, msg: &messages::ParamRequestRead) -> Option<u16> {
    if msg.param_index >= 0 {
        let idx = msg.param_index as u16;
        if idx < registry.count() { Some(idx) } else { None }
    } else {
        let name = decode_param_id(&msg.param_id);
        registry.find_index_by_name(name)
    }
}

#[allow(dead_code)]
pub fn parameter_protocol_example() {
    // This function exists only for documentation purposes.
}
