use crate::backend::{HilinkActuatorCommand, HilinkTelemetryState, UartPortInfo};

use serde::Serialize;

#[derive(Clone, Serialize)]
pub struct AppState {
    pub current_time: String,
    pub gazebo_bridge: GazeboBridgeStats,
    pub gazebo_bridge_process: GazeboBridgeProcessState,
    pub uart: UartState,
    pub hil_comparison: HilComparisonState,
    pub hilink: HilinkTelemetryState,
    pub serial_monitor: SerialMonitorState,
    pub hilink_commands: HilinkCommandState,
}

#[derive(Clone, Default, Serialize)]
pub struct GazeboBridgeProcessState {
    pub launch_path: String,
    pub running: bool,
    pub pid: Option<u32>,
    pub last_error: Option<String>,
    pub last_exit_status: Option<String>,
}

#[derive(Clone, Serialize)]
pub struct GazeboBridgeStats {
    pub endpoint: String,
    pub connected: bool,
    pub connected_for_secs: Option<u64>,
    pub connection_attempts: u64,
    pub actuator_frames_sent: u64,
    pub sensor_frames_received: u64,
    pub last_sensor_sequence: Option<u64>,
    pub last_sensor_time_us: Option<u64>,
    pub last_sensor_clock_source: Option<String>,
    pub last_error: Option<String>,
}

impl Default for GazeboBridgeStats {
    fn default() -> Self {
        Self {
            endpoint: "127.0.0.1:9000".to_string(),
            connected: false,
            connected_for_secs: None,
            connection_attempts: 0,
            actuator_frames_sent: 0,
            sensor_frames_received: 0,
            last_sensor_sequence: None,
            last_sensor_time_us: None,
            last_sensor_clock_source: None,
            last_error: None,
        }
    }
}

#[derive(Clone, Default, Serialize)]
pub struct HilComparisonState {
    pub source_active: bool,
    pub hil_ready: bool,
    pub outstanding_source: Option<HilOutstandingSourceFrame>,
    pub latest_source: Option<HilSourceFrame>,
    pub latest_response: Option<HilResponseFrame>,
    pub latest_forwarded_actuator: Option<HilForwardedActuatorFrame>,
    pub matched: bool,
    pub sim_time_delta_us: Option<i64>,
    pub protocol_fault_count: u64,
    pub latest_protocol_fault: Option<String>,
}

#[derive(Clone, Serialize)]
pub struct HilSourceFrame {
    pub sequence: u64,
    pub sim_time_us: u64,
    pub clock_source: Option<String>,
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
    pub orientation_quat: Option<[f32; 4]>,
    pub mag_ut: [f32; 3],
    pub pressure_pa: f32,
    pub baro_altitude_m: f32,
    pub temperature_c: f32,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
    pub vel_ned_mps: [f32; 3],
    pub sats: u8,
    pub fix_type: u8,
}

#[derive(Clone, Serialize)]
pub struct HilResponseFrame {
    pub sim_tick: u64,
    pub sim_time_us: u64,
    pub flags: u32,
    pub motor_cmd: [u16; 4],
}

impl From<HilinkActuatorCommand> for HilResponseFrame {
    fn from(command: HilinkActuatorCommand) -> Self {
        Self {
            sim_tick: command.sim_tick,
            sim_time_us: command.sim_time_us,
            flags: command.flags,
            motor_cmd: command.motor_cmd,
        }
    }
}

#[derive(Clone, Serialize)]
pub struct HilOutstandingSourceFrame {
    pub sequence: u64,
    pub sim_time_us: u64,
}

#[derive(Clone, Serialize)]
pub struct HilForwardedActuatorFrame {
    pub sequence: u64,
    pub sim_time_us: u64,
    pub motor_cmd: [u16; 4],
}

#[derive(Clone, Default, Serialize)]
pub struct UartState {
    pub available_ports: Vec<UartPortInfo>,
    pub connected: bool,
    pub selected_port: Option<String>,
    pub baud_rate: u32,
    pub line_coding: String,
    pub last_error: Option<String>,
}

#[derive(Clone, Default, Serialize)]
pub struct SerialMonitorState {
    pub lines: Vec<String>,
    pub dropped_lines: u64,
    pub parse_enabled: bool,
    pub parsed_lines: Vec<String>,
    pub dropped_parsed_lines: u64,
}

#[derive(Clone, Default, Serialize)]
pub struct HilinkCommandState {
    pub pending: Vec<PendingHilinkCommand>,
    pub last_event: Option<String>,
    pub pong_count: u64,
    pub ack_count: u64,
    pub nack_count: u64,
    pub gps_count: u64,
    pub telemetry_snapshot_count: u64,
}

#[derive(Clone, Serialize)]
pub struct PendingHilinkCommand {
    pub seq: u16,
    pub msg_type: u8,
    pub label: String,
    pub sent_elapsed_ms: u128,
    pub status: PendingCommandStatus,
}

#[derive(Clone, Serialize)]
pub enum PendingCommandStatus {
    Waiting,
    Pong,
    Ack { status: u8 },
    Nack { reason: u8 },
}

impl PendingCommandStatus {
    #[allow(dead_code)]
    pub fn label(&self) -> String {
        match self {
            Self::Waiting => "waiting".to_string(),
            Self::Pong => "Pong".to_string(),
            Self::Ack { status } => format!("Ack status {status}"),
            Self::Nack { reason } => format!("Nack reason {reason}"),
        }
    }
}
