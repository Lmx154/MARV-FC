//! MAVLink2 message helpers and light-weight telemetry packing.
//! Keep all MAVLink framing and payload building in this module for reuse.
#![allow(async_fn_in_trait)]

use mavio::Frame;
use mavio::protocol::V2;
use mavio::dialects::common::messages;

/// MAVLink endpoint identity (system + component IDs).
#[derive(Clone, Copy)]
pub struct MavEndpointConfig {
    pub sys_id: u8,
    pub comp_id: u8,
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
pub fn build_statustext_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    msg: messages::Statustext,
) -> Frame<V2> {
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
// Light-weight telemetry payload (EncapsulatedData-based)
// ---------------------------------------------------------------------------

/// Versioned, compact telemetry snapshot meant for UART/LoRa forwarding.
///
/// Keep this small so it fits well within MAVLink MTU and is easy to decode.
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
    pub const WIRE_LEN: usize = 39; // bytes encoded below

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

/// Build an EncapsulatedData MAVLink frame carrying the compact telemetry payload.
pub fn build_telemetry_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    sample: &TelemetrySample,
) -> Option<Frame<V2>> {
    let mut payload = [0u8; 253];
    let Some(_len) = sample.encode(&mut payload) else {
        return None;
    };
    // Zero-fill remaining bytes; receiver uses fixed size decode.
    let msg = messages::EncapsulatedData {
        seqnr: seq as u16,
        data: payload,
    };

    Some(
        Frame::builder()
            .version(V2)
            .system_id(cfg.sys_id.into())
            .component_id(cfg.comp_id.into())
            .sequence(seq.into())
            .message(&msg)
            .expect("MAV: build EncapsulatedData frame")
            .build(),
    )
}
