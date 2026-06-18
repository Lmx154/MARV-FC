//! Serde models that mirror the frontend contract in `src/types.ts`.
//!
//! The React UI consumes a single `TelemetryPacket` plus `DebugLine`s. These structs serialize
//! to the exact camelCase JSON the UI expects, so the views need no changes when switching from
//! the demo data source to the live serial backend.

use serde::Serialize;

use marv_hilink::flight_state;

/// cm/s² per g (standard gravity, ×100). Used to convert the wire accel magnitude to g.
const CMS2_PER_G: f32 = 980.665;

/// Mirrors `TelemetryPacket` in `src/types.ts` (camelCase fields).
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct TelemetryPacket {
    pub seq: u32,
    pub mission_time_ms: u64,
    pub stage: &'static str,
    pub altitude_m: f32,
    pub max_altitude_m: f32,
    pub vertical_speed_mps: f32,
    pub accel_g: f32,
    pub battery_v: f32,
    pub gps_lat: f64,
    pub gps_lon: f64,
    pub gps_alt_m: f32,
    pub gps_valid: bool,
    pub gps_sats: u32,
    pub rssi_dbm: i32,
    pub snr_db: f32,
    pub packet_age_ms: u64,
    pub packet_rate_hz: f32,
    pub packet_loss_pct: f32,
    // Ground-station radio's active RF profile (from RadioStatus), for the operator to confirm a
    // link-wide profile switch took effect. `radio_active_known` is false until the first report.
    pub radio_active_known: bool,
    pub radio_active_preset: u8,
    pub radio_active_tx_power_dbm: i8,
    pub radio_active_frequency_hz: u32,
}

impl Default for TelemetryPacket {
    fn default() -> Self {
        Self {
            seq: 0,
            mission_time_ms: 0,
            stage: flight_stage(flight_state::PAD),
            altitude_m: 0.0,
            max_altitude_m: 0.0,
            vertical_speed_mps: 0.0,
            accel_g: 0.0,
            battery_v: 0.0,
            gps_lat: 0.0,
            gps_lon: 0.0,
            gps_alt_m: 0.0,
            gps_valid: false,
            gps_sats: 0,
            rssi_dbm: 0,
            snr_db: 0.0,
            packet_age_ms: 0,
            packet_rate_hz: 0.0,
            packet_loss_pct: 0.0,
            radio_active_known: false,
            radio_active_preset: 0xFF,
            radio_active_tx_power_dbm: 0,
            radio_active_frequency_hz: 0,
        }
    }
}

/// Mirrors `DebugKind` in `src/types.ts`. The serial worker emits one of each: `Raw` (frame hex
/// dump), `Parsed` (decoded HILink summary), plus `System`/`Link`/`Error` status lines.
#[derive(Clone, Copy, Debug, Serialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum DebugKind {
    Raw,
    Parsed,
    Error,
    System,
    Link,
}

/// Mirrors `DebugLine` in `src/types.ts`.
#[derive(Clone, Debug, Serialize)]
pub struct DebugLine {
    pub time: String,
    pub kind: DebugKind,
    pub text: String,
}

impl DebugLine {
    pub fn new(kind: DebugKind, text: impl Into<String>) -> Self {
        Self {
            time: chrono::Local::now().format("%H:%M:%S%.3f").to_string(),
            kind,
            text: text.into(),
        }
    }
}

/// Pushed on the `link-status` event whenever the connection state changes.
#[derive(Clone, Debug, Serialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct LinkStatus {
    pub connected: bool,
    pub port: Option<String>,
    pub baud: u32,
    pub last_error: Option<String>,
}

/// One serial port, returned by the `list_serial_ports` command.
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PortInfo {
    pub port_name: String,
    pub display_name: String,
}

/// Map the rocket `system_state` byte to the UI's `FlightStage` string.
/// The single place to adjust if the protocol's `flight_state` codes change.
pub fn flight_stage(system_state: u8) -> &'static str {
    match system_state {
        flight_state::PAD => "PAD",
        flight_state::BOOST => "BOOST",
        flight_state::BURNOUT => "BURNOUT",
        flight_state::COAST => "COAST",
        flight_state::APOGEE => "APOGEE",
        flight_state::DROGUE_DESCENT => "DROGUE_DESCENT",
        flight_state::MAIN_DESCENT => "MAIN_DESCENT",
        flight_state::LANDED => "LANDED",
        _ => "PAD",
    }
}

/// Wire altitude is the NED down-component negated (up positive).
pub fn altitude_from_ned(position_ned_m: [f32; 3]) -> f32 {
    -position_ned_m[2]
}

/// Wire vertical speed is the NED down-velocity negated (up positive).
pub fn vertical_speed_from_ned(velocity_ned_mps: [f32; 3]) -> f32 {
    -velocity_ned_mps[2]
}

/// Convert the wire accel magnitude (cm/s², ×1) to g. 0 stays 0.
pub fn accel_g_from_cms2(accel_mag_cms2: u16) -> f32 {
    f32::from(accel_mag_cms2) / CMS2_PER_G
}

/// A 3D GPS fix (`fix_type >= 3`) with enough satellites is treated as valid.
pub fn gps_is_valid(fix_type: u8, sats: u8) -> bool {
    fix_type >= 3 && sats >= 4
}
