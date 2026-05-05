use crate::backend::{HilinkTelemetryState, UartPortInfo};

#[derive(Clone)]
pub struct AppState {
    pub current_time: String,
    pub operation_mode: OperationMode,
    pub gazebo_bridge: GazeboBridgeStats,
    pub uart: UartState,
    pub hilink: HilinkTelemetryState,
    pub serial_monitor: SerialMonitorState,
    pub hilink_commands: HilinkCommandState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OperationMode {
    Hil,
    FieldRadio,
    FieldProgramming,
}

impl OperationMode {
    pub fn label(self) -> &'static str {
        match self {
            Self::Hil => "HIL operation",
            Self::FieldRadio => "Field operation",
            Self::FieldProgramming => "Field programming",
        }
    }

    pub fn link_label(self) -> &'static str {
        match self {
            Self::Hil => "Direct FC USB + Gazebo bridge",
            Self::FieldRadio => "CP2102 GS radio link",
            Self::FieldProgramming => "Direct FC USB",
        }
    }

    pub fn uses_gazebo_bridge(self) -> bool {
        self == Self::Hil
    }

    pub fn default_baud_rate(self) -> u32 {
        match self {
            Self::Hil | Self::FieldRadio => 115_200,
            Self::FieldProgramming => 921_600,
        }
    }
}

#[derive(Clone)]
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

#[derive(Clone, Default)]
pub struct UartState {
    pub available_ports: Vec<UartPortInfo>,
    pub connected: bool,
    pub selected_port: Option<String>,
    pub baud_rate: u32,
    pub line_coding: String,
    pub last_error: Option<String>,
}

#[derive(Clone, Default)]
pub struct SerialMonitorState {
    pub lines: Vec<String>,
    pub dropped_lines: u64,
    pub parse_enabled: bool,
    pub parsed_lines: Vec<String>,
    pub dropped_parsed_lines: u64,
}

#[derive(Clone, Default)]
pub struct HilinkCommandState {
    pub pending: Vec<PendingHilinkCommand>,
    pub last_event: Option<String>,
    pub pong_count: u64,
    pub ack_count: u64,
    pub nack_count: u64,
    pub gps_count: u64,
    pub telemetry_snapshot_count: u64,
}

#[derive(Clone)]
pub struct PendingHilinkCommand {
    pub seq: u16,
    pub msg_type: u8,
    pub label: String,
    pub sent_elapsed_ms: u128,
    pub status: PendingCommandStatus,
}

#[derive(Clone)]
pub enum PendingCommandStatus {
    Waiting,
    Pong,
    Ack { status: u8 },
    Nack { reason: u8 },
}

impl PendingCommandStatus {
    pub fn label(&self) -> String {
        match self {
            Self::Waiting => "waiting".to_string(),
            Self::Pong => "Pong".to_string(),
            Self::Ack { status } => format!("Ack status {status}"),
            Self::Nack { reason } => format!("Nack reason {reason}"),
        }
    }
}
