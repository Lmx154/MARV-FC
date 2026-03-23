//! Protocol-neutral HIL semantic message model.

use crate::services::hil::backend::SensorBackend;
use crate::utilities::time::MeasurementTimestamp;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HilTick {
    pub timestamp: MeasurementTimestamp,
    pub time_boot_ms: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilImuSample {
    pub timestamp: MeasurementTimestamp,
    pub accel_mps2: [f32; 3],
    pub gyro_rad_s: [f32; 3],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilBarometerSample {
    pub timestamp: MeasurementTimestamp,
    pub pressure_pa: f32,
    pub temperature_c: f32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilGpsSample {
    pub timestamp: MeasurementTimestamp,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_m: f32,
    pub vel_ned_mps: [f32; 3],
    pub sats: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilMagSample {
    pub timestamp: MeasurementTimestamp,
    pub field_ut: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HilIngressMessage {
    Tick(HilTick),
    ImuSample(HilImuSample),
    BarometerSample(HilBarometerSample),
    GpsSample(HilGpsSample),
    MagnetometerSample(HilMagSample),
    ControlCommand(HilControlCommand),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HilControlAction {
    RequestBackend(SensorBackend),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HilControlCommand {
    pub command_id: u16,
    pub action: HilControlAction,
    pub source_system: u8,
    pub source_component: u8,
    pub target_system: u8,
    pub target_component: u8,
    pub confirmation: u8,
}

impl HilControlCommand {
    pub const fn request_backend(
        command_id: u16,
        backend: SensorBackend,
        source_system: u8,
        source_component: u8,
        target_system: u8,
        target_component: u8,
        confirmation: u8,
    ) -> Self {
        Self {
            command_id,
            action: HilControlAction::RequestBackend(backend),
            source_system,
            source_component,
            target_system,
            target_component,
            confirmation,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HilCommandAckResult {
    Accepted,
    TemporarilyRejected,
    Denied,
    Unsupported,
    Failed,
    InProgress,
    Cancelled,
    CommandLongOnly,
    CommandIntOnly,
    UnsupportedMavFrame,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HilCommandAck {
    pub command_id: u16,
    pub result: HilCommandAckResult,
    pub progress: u8,
    pub result_param2: i32,
    pub target_system: u8,
    pub target_component: u8,
}

impl HilCommandAck {
    pub const fn new(
        command_id: u16,
        result: HilCommandAckResult,
        target_system: u8,
        target_component: u8,
    ) -> Self {
        Self {
            command_id,
            result,
            progress: 0,
            result_param2: 0,
            target_system,
            target_component,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilActuatorCommand {
    pub timestamp: MeasurementTimestamp,
    pub group: u8,
    pub controls: [f32; 8],
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilMissionEvent {
    pub timestamp: MeasurementTimestamp,
    pub command_id: u16,
    pub params: [f32; 7],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HilEgressMessage {
    ActuatorCommand(HilActuatorCommand),
    MissionEvent(HilMissionEvent),
    CommandAck(HilCommandAck),
}
