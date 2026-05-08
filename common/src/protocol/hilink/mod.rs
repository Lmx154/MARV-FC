//! HILink wire protocol.
//!
//! Wire layout:
//! `COBS(Header + Payload + CRC16-CCITT-FALSE little-endian) + 0x00 delimiter`.
//! Public packet structs are `repr(C)` for sharing shape, but the wire format is
//! explicit little-endian serialization.

use core::convert::TryFrom;

pub const PROTOCOL_VERSION: u8 = 1;
pub const HEADER_LEN: usize = 12;
pub const CRC_LEN: usize = 2;
pub const FRAME_DELIMITER: u8 = 0;

pub type Result<T> = core::result::Result<T, Error>;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    BufferTooSmall,
    PayloadTooLarge,
    HeaderTooShort,
    PayloadLenMismatch,
    BadDelimiter,
    CobsDecodeZero,
    CobsDecodeOverrun,
    CrcMismatch,
    UnknownMsgType,
    InvalidPayloadLength,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MsgType {
    Ping = 1,
    Pong = 2,
    Ack = 3,
    Nack = 4,
    Heartbeat = 5,

    HilSensorFrame = 10,
    HilResponseFrame = 11,
    HilReady = 12,

    Imu = 20,
    Mag = 21,
    Baro = 22,
    Gps = 23,
    Battery = 24,
    SystemState = 25,
    MotorState = 26,
    EstimatorState = 27,
    RadioStatus = 28,
    TelemetrySnapshot = 29,

    Arm = 40,
    Disarm = 41,
    ControlWaypoint = 42,
    CvWaypoint = 43,
    TofWaypoint = 44,
    MissionWaypoint = 45,
    Rtl = 46,

    BenchEnable = 60,
    BenchDisable = 61,
    MotorTest = 62,
    MotorSweep = 63,
    MotorStop = 64,
    DshotCommand = 65,
    ActuatorStatusRequest = 66,
    ActuatorStatus = 67,

    LoRaFlightSnapshot = 81,
    LoRaGpsSnapshot = 83,
    LoRaEvent = 85,
    LoRaFaults = 86,
    LoRaLinkStatus = 87,

    LoRaCommand = 100,
    LoRaCommandAck = 101,
    LoRaSetProfile = 102,
    LoRaRequestSnapshot = 103,
}

impl TryFrom<u8> for MsgType {
    type Error = Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            1 => Ok(Self::Ping),
            2 => Ok(Self::Pong),
            3 => Ok(Self::Ack),
            4 => Ok(Self::Nack),
            5 => Ok(Self::Heartbeat),
            10 => Ok(Self::HilSensorFrame),
            11 => Ok(Self::HilResponseFrame),
            12 => Ok(Self::HilReady),
            20 => Ok(Self::Imu),
            21 => Ok(Self::Mag),
            22 => Ok(Self::Baro),
            23 => Ok(Self::Gps),
            24 => Ok(Self::Battery),
            25 => Ok(Self::SystemState),
            26 => Ok(Self::MotorState),
            27 => Ok(Self::EstimatorState),
            28 => Ok(Self::RadioStatus),
            29 => Ok(Self::TelemetrySnapshot),
            40 => Ok(Self::Arm),
            41 => Ok(Self::Disarm),
            42 => Ok(Self::ControlWaypoint),
            43 => Ok(Self::CvWaypoint),
            44 => Ok(Self::TofWaypoint),
            45 => Ok(Self::MissionWaypoint),
            46 => Ok(Self::Rtl),
            60 => Ok(Self::BenchEnable),
            61 => Ok(Self::BenchDisable),
            62 => Ok(Self::MotorTest),
            63 => Ok(Self::MotorSweep),
            64 => Ok(Self::MotorStop),
            65 => Ok(Self::DshotCommand),
            66 => Ok(Self::ActuatorStatusRequest),
            67 => Ok(Self::ActuatorStatus),
            81 => Ok(Self::LoRaFlightSnapshot),
            83 => Ok(Self::LoRaGpsSnapshot),
            85 => Ok(Self::LoRaEvent),
            86 => Ok(Self::LoRaFaults),
            87 => Ok(Self::LoRaLinkStatus),
            100 => Ok(Self::LoRaCommand),
            101 => Ok(Self::LoRaCommandAck),
            102 => Ok(Self::LoRaSetProfile),
            103 => Ok(Self::LoRaRequestSnapshot),
            _ => Err(Error::UnknownMsgType),
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Header {
    pub version: u8,
    pub msg_type: u8,
    pub flags: u8,
    pub reserved: u8,
    pub seq: u16,
    pub send_time_ms: u32,
    pub payload_len: u16,
}

impl Header {
    pub const WIRE_LEN: usize = HEADER_LEN;

    pub const fn new(msg_type: MsgType, seq: u16, send_time_ms: u32, payload_len: u16) -> Self {
        Self {
            version: PROTOCOL_VERSION,
            msg_type: msg_type as u8,
            flags: 0,
            reserved: 0,
            seq,
            send_time_ms,
            payload_len,
        }
    }

    pub fn message_type(&self) -> Result<MsgType> {
        MsgType::try_from(self.msg_type)
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.version)?;
        w.u8(self.msg_type)?;
        w.u8(self.flags)?;
        w.u8(self.reserved)?;
        w.u16(self.seq)?;
        w.u32(self.send_time_ms)?;
        w.u16(self.payload_len)?;
        Ok(w.len())
    }

    pub fn decode(input: &[u8]) -> Result<Self> {
        if input.len() < HEADER_LEN {
            return Err(Error::HeaderTooShort);
        }

        let mut r = Reader::new(&input[..HEADER_LEN]);
        Ok(Self {
            version: r.u8()?,
            msg_type: r.u8()?,
            flags: r.u8()?,
            reserved: r.u8()?,
            seq: r.u16()?,
            send_time_ms: r.u32()?,
            payload_len: r.u16()?,
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SimStamp {
    pub sim_tick: u64,
    pub sim_time_us: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilSensorFrame {
    pub stamp: SimStamp,
    pub valid_flags: u32,

    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
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
    pub reserved0: [u8; 3],

    pub battery_voltage_v: f32,
    pub rssi_dbm: i16,
    pub snr_db_x100: i16,
    pub loss_pct_x100: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilResponseFrame {
    pub stamp: SimStamp,
    pub system_state: u8,
    pub reserved0: [u8; 3],
    pub flags: u32,

    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub attitude_quat: [f32; 4],

    pub motor_cmd: [u16; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuPayload {
    pub stamp: SimStamp,
    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MagPayload {
    pub stamp: SimStamp,
    pub field_ut: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BaroPayload {
    pub stamp: SimStamp,
    pub pressure_pa: f32,
    pub altitude_m: f32,
    pub temperature_c: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GpsPayload {
    pub stamp: SimStamp,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
    pub vel_ned_mps: [f32; 3],
    pub sats: u8,
    pub fix_type: u8,
    pub reserved0: [u8; 2],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SystemStatePayload {
    pub stamp: SimStamp,
    pub system_state: u8,
    pub reserved0: [u8; 3],
    pub flags: u32,
    pub battery_voltage_v: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TelemetrySnapshotPayload {
    pub stamp: SimStamp,
    pub system_state: u8,
    pub reserved0: [u8; 3],
    pub flags: u32,
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub attitude_quat: [f32; 4],
    pub battery_voltage_v: f32,
    pub rssi_dbm: i16,
    pub snr_db_x100: i16,
    pub loss_pct_x100: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HeartbeatPayload {
    pub stamp: SimStamp,
    pub system_state: u8,
    pub reserved0: [u8; 3],
    pub flags: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AckPayload {
    pub acked_seq: u16,
    pub acked_msg_type: u8,
    pub status: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct NackPayload {
    pub rejected_seq: u16,
    pub rejected_msg_type: u8,
    pub reason: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PingPayload;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PongPayload;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct HilReadyPayload;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ArmPayload;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DisarmPayload;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GlobalWaypointPayload {
    pub ref_stamp: SimStamp,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
    pub yaw_deg: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CvWaypointPayload {
    pub ref_stamp: SimStamp,
    pub dir_body: [f32; 3],
    pub confidence: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TofWaypointPayload {
    pub ref_stamp: SimStamp,
    pub distance_m: f32,
    pub bearing_deg: f32,
    pub elevation_deg: f32,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RtlPayload;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BenchEnablePayload {
    pub magic: u32,
    pub timeout_ms: u16,
    pub reserved0: [u8; 2],
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BenchDisablePayload;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct MotorTestPayload {
    pub motor_mask: u8,
    pub mode: u8,
    pub reserved0: [u8; 2],
    pub value: u16,
    pub duration_ms: u16,
    pub ramp_ms: u16,
    pub reserved1: u16,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct MotorSweepPayload {
    pub motor_mask: u8,
    pub mode: u8,
    pub reserved0: [u8; 2],
    pub start_value: u16,
    pub end_value: u16,
    pub step_value: u16,
    pub step_duration_ms: u16,
    pub zero_between_ms: u16,
    pub repeat_count: u8,
    pub reserved1: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct MotorStopPayload;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DshotCommandPayload {
    pub motor_mask: u8,
    pub command: u8,
    pub repeat_count: u8,
    pub reserved0: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ActuatorStatusRequestPayload;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ActuatorStatusPayload {
    pub armed: u8,
    pub bench_enabled: u8,
    pub active_motor_mask: u8,
    pub mode: u8,
    pub commanded_dshot: [u16; 4],
    pub last_command_age_ms: u16,
    pub bench_timeout_ms: u16,
    pub flags: u32,
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

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DecodedPacket<'a> {
    pub header: Header,
    pub payload: &'a [u8],
}

pub mod valid {
    pub const ACCEL: u32 = 1 << 0;
    pub const GYRO: u32 = 1 << 1;
    pub const MAG: u32 = 1 << 2;
    pub const BARO: u32 = 1 << 3;
    pub const GPS: u32 = 1 << 4;
    pub const BATTERY: u32 = 1 << 5;
    pub const RADIO: u32 = 1 << 6;
}

pub mod response_flags {
    pub const ARMED: u32 = 1 << 0;
    pub const FAILSAFE: u32 = 1 << 1;
    pub const ESTIMATOR_VALID: u32 = 1 << 2;
    pub const MOTORS_VALID: u32 = 1 << 3;
}

pub mod bench {
    pub const ENABLE_MAGIC: u32 = 0x4D4F544F;
    pub const MOTOR_MASK_M1: u8 = 0x01;
    pub const MOTOR_MASK_M2: u8 = 0x02;
    pub const MOTOR_MASK_M3: u8 = 0x04;
    pub const MOTOR_MASK_M4: u8 = 0x08;
    pub const MOTOR_MASK_ALL: u8 = 0x0F;
    pub const MAX_TEST_DURATION_MS: u16 = 5000;
}

pub mod motor_test_mode {
    pub const STOP: u8 = 0;
    pub const RAW_DSHOT: u8 = 1;
    pub const NORMALIZED: u8 = 2;
}

pub mod actuator_flags {
    pub const OUTPUT_ACTIVE: u32 = 1 << 0;
    pub const COMMAND_TIMEOUT: u32 = 1 << 1;
    pub const CLAMPED: u32 = 1 << 2;
    pub const REJECTED_WHILE_DISARMED: u32 = 1 << 3;
    pub const BENCH_MODE_ENABLED: u32 = 1 << 4;
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

pub trait WirePayload: Sized {
    const MSG_TYPE: MsgType;
    const WIRE_LEN: usize;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize>;
    fn decode_payload(input: &[u8]) -> Result<Self>;
}

pub const fn raw_frame_len(payload_len: usize) -> usize {
    HEADER_LEN + payload_len + CRC_LEN
}

pub const fn encoded_frame_len(payload_len: usize) -> usize {
    let raw_len = raw_frame_len(payload_len);
    raw_len + (raw_len / 254) + 2
}

pub fn encode_packet<P: WirePayload>(
    payload: &P,
    seq: u16,
    send_time_ms: u32,
    raw_scratch: &mut [u8],
    out: &mut [u8],
) -> Result<usize> {
    if P::WIRE_LEN > u16::MAX as usize {
        return Err(Error::PayloadTooLarge);
    }

    let header = Header::new(P::MSG_TYPE, seq, send_time_ms, P::WIRE_LEN as u16);
    encode_packet_raw(&header, payload, raw_scratch, out)
}

pub fn encode_packet_raw<P: WirePayload>(
    header: &Header,
    payload: &P,
    raw_scratch: &mut [u8],
    out: &mut [u8],
) -> Result<usize> {
    let payload_len = usize::from(header.payload_len);
    if payload_len != P::WIRE_LEN {
        return Err(Error::PayloadLenMismatch);
    }

    let raw_len = raw_frame_len(payload_len);
    if raw_scratch.len() < raw_len {
        return Err(Error::BufferTooSmall);
    }

    let mut pos = header.encode(raw_scratch)?;
    let written = payload.encode_payload(&mut raw_scratch[pos..pos + payload_len])?;
    if written != payload_len {
        return Err(Error::InvalidPayloadLength);
    }
    pos += written;

    let crc = crc16_ccitt_false(&raw_scratch[..pos]);
    raw_scratch[pos..pos + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
    pos += CRC_LEN;

    let encoded_len = cobs_encode(&raw_scratch[..pos], out)?;
    if out.len() <= encoded_len {
        return Err(Error::BufferTooSmall);
    }
    out[encoded_len] = FRAME_DELIMITER;
    Ok(encoded_len + 1)
}

pub fn decode_packet<'a>(input: &[u8], raw_scratch: &'a mut [u8]) -> Result<DecodedPacket<'a>> {
    let encoded = match input.last() {
        Some(&FRAME_DELIMITER) => &input[..input.len() - 1],
        Some(_) | None => return Err(Error::BadDelimiter),
    };

    let raw_len = cobs_decode(encoded, raw_scratch)?;
    if raw_len < HEADER_LEN + CRC_LEN {
        return Err(Error::HeaderTooShort);
    }

    let packet_len = raw_len - CRC_LEN;
    let stored_crc = u16::from_le_bytes([raw_scratch[packet_len], raw_scratch[packet_len + 1]]);
    let computed_crc = crc16_ccitt_false(&raw_scratch[..packet_len]);
    if stored_crc != computed_crc {
        return Err(Error::CrcMismatch);
    }

    let header = Header::decode(&raw_scratch[..HEADER_LEN])?;
    let payload_end = HEADER_LEN + usize::from(header.payload_len);
    if payload_end != packet_len {
        return Err(Error::PayloadLenMismatch);
    }

    Ok(DecodedPacket {
        header,
        payload: &raw_scratch[HEADER_LEN..payload_end],
    })
}

pub fn decode_payload<P: WirePayload>(packet: &DecodedPacket<'_>) -> Result<P> {
    if packet.header.message_type()? != P::MSG_TYPE {
        return Err(Error::UnknownMsgType);
    }

    P::decode_payload(packet.payload)
}

pub fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in bytes {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

pub fn cobs_encode(input: &[u8], out: &mut [u8]) -> Result<usize> {
    if out.is_empty() {
        return Err(Error::BufferTooSmall);
    }

    let mut read_index = 0;
    let mut write_index = 1;
    let mut code_index = 0;
    let mut code = 1u8;

    while read_index < input.len() {
        if write_index >= out.len() {
            return Err(Error::BufferTooSmall);
        }

        if input[read_index] == 0 {
            out[code_index] = code;
            code_index = write_index;
            write_index += 1;
            code = 1;
        } else {
            out[write_index] = input[read_index];
            write_index += 1;
            code += 1;

            if code == 0xFF {
                out[code_index] = code;
                code_index = write_index;
                write_index += 1;
                code = 1;
            }
        }

        read_index += 1;
    }

    if code_index >= out.len() {
        return Err(Error::BufferTooSmall);
    }
    out[code_index] = code;
    Ok(write_index)
}

pub fn cobs_decode(input: &[u8], out: &mut [u8]) -> Result<usize> {
    let mut read_index = 0;
    let mut write_index = 0;

    while read_index < input.len() {
        let code = input[read_index];
        if code == 0 {
            return Err(Error::CobsDecodeZero);
        }
        read_index += 1;

        let copy_len = usize::from(code - 1);
        if read_index + copy_len > input.len() {
            return Err(Error::CobsDecodeOverrun);
        }
        if write_index + copy_len > out.len() {
            return Err(Error::BufferTooSmall);
        }

        out[write_index..write_index + copy_len]
            .copy_from_slice(&input[read_index..read_index + copy_len]);
        write_index += copy_len;
        read_index += copy_len;

        if code != 0xFF && read_index < input.len() {
            if write_index >= out.len() {
                return Err(Error::BufferTooSmall);
            }
            out[write_index] = 0;
            write_index += 1;
        }
    }

    Ok(write_index)
}

macro_rules! impl_empty_payload {
    ($ty:ty, $msg_type:expr) => {
        impl WirePayload for $ty {
            const MSG_TYPE: MsgType = $msg_type;
            const WIRE_LEN: usize = 0;

            fn encode_payload(&self, _out: &mut [u8]) -> Result<usize> {
                Ok(0)
            }

            fn decode_payload(input: &[u8]) -> Result<Self> {
                expect_len(input, Self::WIRE_LEN)?;
                Ok(Self)
            }
        }
    };
}

impl_empty_payload!(PingPayload, MsgType::Ping);
impl_empty_payload!(PongPayload, MsgType::Pong);
impl_empty_payload!(HilReadyPayload, MsgType::HilReady);
impl_empty_payload!(ArmPayload, MsgType::Arm);
impl_empty_payload!(DisarmPayload, MsgType::Disarm);
impl_empty_payload!(RtlPayload, MsgType::Rtl);
impl_empty_payload!(BenchDisablePayload, MsgType::BenchDisable);
impl_empty_payload!(MotorStopPayload, MsgType::MotorStop);
impl_empty_payload!(ActuatorStatusRequestPayload, MsgType::ActuatorStatusRequest);

impl WirePayload for GlobalWaypointPayload {
    const MSG_TYPE: MsgType = MsgType::ControlWaypoint;
    const WIRE_LEN: usize = 40;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.ref_stamp)?;
        w.f64(self.lat_deg)?;
        w.f64(self.lon_deg)?;
        w.f32(self.alt_msl_m)?;
        w.f32(self.yaw_deg)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            ref_stamp: r.sim_stamp()?,
            lat_deg: r.f64()?,
            lon_deg: r.f64()?,
            alt_msl_m: r.f32()?,
            yaw_deg: r.f32()?,
        })
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MissionWaypointPayload(pub GlobalWaypointPayload);

impl WirePayload for MissionWaypointPayload {
    const MSG_TYPE: MsgType = MsgType::MissionWaypoint;
    const WIRE_LEN: usize = <GlobalWaypointPayload as WirePayload>::WIRE_LEN;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        self.0.encode_payload(out)
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        Ok(Self(GlobalWaypointPayload::decode_payload(input)?))
    }
}

impl WirePayload for CvWaypointPayload {
    const MSG_TYPE: MsgType = MsgType::CvWaypoint;
    const WIRE_LEN: usize = 32;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.ref_stamp)?;
        w.f32x3(self.dir_body)?;
        w.f32(self.confidence)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            ref_stamp: r.sim_stamp()?,
            dir_body: r.f32x3()?,
            confidence: r.f32()?,
        })
    }
}

impl WirePayload for TofWaypointPayload {
    const MSG_TYPE: MsgType = MsgType::TofWaypoint;
    const WIRE_LEN: usize = 28;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.ref_stamp)?;
        w.f32(self.distance_m)?;
        w.f32(self.bearing_deg)?;
        w.f32(self.elevation_deg)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            ref_stamp: r.sim_stamp()?,
            distance_m: r.f32()?,
            bearing_deg: r.f32()?,
            elevation_deg: r.f32()?,
        })
    }
}

impl WirePayload for BenchEnablePayload {
    const MSG_TYPE: MsgType = MsgType::BenchEnable;
    const WIRE_LEN: usize = 8;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u32(self.magic)?;
        w.u16(self.timeout_ms)?;
        w.bytes(&self.reserved0)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            magic: r.u32()?,
            timeout_ms: r.u16()?,
            reserved0: r.take::<2>()?,
        })
    }
}

impl WirePayload for MotorTestPayload {
    const MSG_TYPE: MsgType = MsgType::MotorTest;
    const WIRE_LEN: usize = 12;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.motor_mask)?;
        w.u8(self.mode)?;
        w.bytes(&self.reserved0)?;
        w.u16(self.value)?;
        w.u16(self.duration_ms)?;
        w.u16(self.ramp_ms)?;
        w.u16(self.reserved1)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            motor_mask: r.u8()?,
            mode: r.u8()?,
            reserved0: r.take::<2>()?,
            value: r.u16()?,
            duration_ms: r.u16()?,
            ramp_ms: r.u16()?,
            reserved1: r.u16()?,
        })
    }
}

impl WirePayload for MotorSweepPayload {
    const MSG_TYPE: MsgType = MsgType::MotorSweep;
    const WIRE_LEN: usize = 16;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.motor_mask)?;
        w.u8(self.mode)?;
        w.bytes(&self.reserved0)?;
        w.u16(self.start_value)?;
        w.u16(self.end_value)?;
        w.u16(self.step_value)?;
        w.u16(self.step_duration_ms)?;
        w.u16(self.zero_between_ms)?;
        w.u8(self.repeat_count)?;
        w.u8(self.reserved1)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            motor_mask: r.u8()?,
            mode: r.u8()?,
            reserved0: r.take::<2>()?,
            start_value: r.u16()?,
            end_value: r.u16()?,
            step_value: r.u16()?,
            step_duration_ms: r.u16()?,
            zero_between_ms: r.u16()?,
            repeat_count: r.u8()?,
            reserved1: r.u8()?,
        })
    }
}

impl WirePayload for DshotCommandPayload {
    const MSG_TYPE: MsgType = MsgType::DshotCommand;
    const WIRE_LEN: usize = 4;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.motor_mask)?;
        w.u8(self.command)?;
        w.u8(self.repeat_count)?;
        w.u8(self.reserved0)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            motor_mask: r.u8()?,
            command: r.u8()?,
            repeat_count: r.u8()?,
            reserved0: r.u8()?,
        })
    }
}

impl WirePayload for ActuatorStatusPayload {
    const MSG_TYPE: MsgType = MsgType::ActuatorStatus;
    const WIRE_LEN: usize = 20;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u8(self.armed)?;
        w.u8(self.bench_enabled)?;
        w.u8(self.active_motor_mask)?;
        w.u8(self.mode)?;
        w.u16x4(self.commanded_dshot)?;
        w.u16(self.last_command_age_ms)?;
        w.u16(self.bench_timeout_ms)?;
        w.u32(self.flags)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            armed: r.u8()?,
            bench_enabled: r.u8()?,
            active_motor_mask: r.u8()?,
            mode: r.u8()?,
            commanded_dshot: r.u16x4()?,
            last_command_age_ms: r.u16()?,
            bench_timeout_ms: r.u16()?,
            flags: r.u32()?,
        })
    }
}

impl WirePayload for LoRaFlightSnapshotPayload {
    const MSG_TYPE: MsgType = MsgType::LoRaFlightSnapshot;
    const WIRE_LEN: usize = 22;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u32(self.time_ms)?;
        w.u8(self.state)?;
        w.u8(self.mode)?;
        w.u16(self.flags)?;
        w.i32(self.altitude_dm)?;
        w.i16(self.vertical_velocity_cms)?;
        w.u16(self.accel_mag_cms2)?;
        w.u16(self.battery_mv)?;
        w.u16(self.pyro_or_actuator_flags)?;
        w.u16(self.fault_summary)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
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
        })
    }
}

impl WirePayload for LoRaGpsSnapshotPayload {
    const MSG_TYPE: MsgType = MsgType::LoRaGpsSnapshot;
    const WIRE_LEN: usize = 22;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u32(self.time_ms)?;
        w.i32(self.lat_e7)?;
        w.i32(self.lon_e7)?;
        w.i32(self.alt_msl_dm)?;
        w.u16(self.ground_speed_cms)?;
        w.u16(self.heading_cdeg)?;
        w.u8(self.sats)?;
        w.u8(self.fix_type)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            time_ms: r.u32()?,
            lat_e7: r.i32()?,
            lon_e7: r.i32()?,
            alt_msl_dm: r.i32()?,
            ground_speed_cms: r.u16()?,
            heading_cdeg: r.u16()?,
            sats: r.u8()?,
            fix_type: r.u8()?,
        })
    }
}

impl WirePayload for LoRaEventPayload {
    const MSG_TYPE: MsgType = MsgType::LoRaEvent;
    const WIRE_LEN: usize = 15;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u32(self.time_ms)?;
        w.u16(self.event_id)?;
        w.u8(self.severity)?;
        w.i32(self.arg0)?;
        w.i32(self.arg1)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            time_ms: r.u32()?,
            event_id: r.u16()?,
            severity: r.u8()?,
            arg0: r.i32()?,
            arg1: r.i32()?,
        })
    }
}

impl WirePayload for LoRaFaultsPayload {
    const MSG_TYPE: MsgType = MsgType::LoRaFaults;
    const WIRE_LEN: usize = 14;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u32(self.time_ms)?;
        w.u32(self.active_faults)?;
        w.u32(self.latched_faults)?;
        w.u16(self.inhibit_flags)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            time_ms: r.u32()?,
            active_faults: r.u32()?,
            latched_faults: r.u32()?,
            inhibit_flags: r.u16()?,
        })
    }
}

impl WirePayload for LoRaLinkStatusPayload {
    const MSG_TYPE: MsgType = MsgType::LoRaLinkStatus;
    const WIRE_LEN: usize = 18;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u32(self.time_ms)?;
        w.i8(self.uplink_rssi_dbm)?;
        w.i8(self.uplink_snr_x4)?;
        w.i8(self.downlink_rssi_dbm)?;
        w.i8(self.downlink_snr_x4)?;
        w.u16(self.rx_packets_delta)?;
        w.u16(self.tx_packets_delta)?;
        w.u16(self.lost_packets_delta)?;
        w.u8(self.active_profile)?;
        w.u8(self.telemetry_rate_hz)?;
        w.u16(self.reserved)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
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
        })
    }
}

impl WirePayload for LoRaCommandPayload {
    const MSG_TYPE: MsgType = MsgType::LoRaCommand;
    const WIRE_LEN: usize = 16;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u16(self.command_id)?;
        w.u16(self.command_seq)?;
        w.u16(self.expires_ms)?;
        w.u16(self.flags)?;
        w.i32(self.arg0)?;
        w.i32(self.arg1)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            command_id: r.u16()?,
            command_seq: r.u16()?,
            expires_ms: r.u16()?,
            flags: r.u16()?,
            arg0: r.i32()?,
            arg1: r.i32()?,
        })
    }
}

impl WirePayload for LoRaCommandAckPayload {
    const MSG_TYPE: MsgType = MsgType::LoRaCommandAck;
    const WIRE_LEN: usize = 12;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u16(self.command_id)?;
        w.u16(self.command_seq)?;
        w.u8(self.status)?;
        w.u8(self.reason)?;
        w.u8(self.state)?;
        w.u8(self.reserved)?;
        w.i32(self.detail)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            command_id: r.u16()?,
            command_seq: r.u16()?,
            status: r.u8()?,
            reason: r.u8()?,
            state: r.u8()?,
            reserved: r.u8()?,
            detail: r.i32()?,
        })
    }
}

impl WirePayload for LoRaSetProfilePayload {
    const MSG_TYPE: MsgType = MsgType::LoRaSetProfile;
    const WIRE_LEN: usize = 8;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u16(self.command_seq)?;
        w.u8(self.profile)?;
        w.u8(self.telemetry_rate_hz)?;
        w.u8(self.gps_rate_hz)?;
        w.u8(self.link_status_rate_hz)?;
        w.u16(self.flags)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            command_seq: r.u16()?,
            profile: r.u8()?,
            telemetry_rate_hz: r.u8()?,
            gps_rate_hz: r.u8()?,
            link_status_rate_hz: r.u8()?,
            flags: r.u16()?,
        })
    }
}

impl WirePayload for LoRaRequestSnapshotPayload {
    const MSG_TYPE: MsgType = MsgType::LoRaRequestSnapshot;
    const WIRE_LEN: usize = 4;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u16(self.command_seq)?;
        w.u16(self.request_flags)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            command_seq: r.u16()?,
            request_flags: r.u16()?,
        })
    }
}

impl WirePayload for HilSensorFrame {
    const MSG_TYPE: MsgType = MsgType::HilSensorFrame;
    const WIRE_LEN: usize = 115;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.stamp)?;
        w.u32(self.valid_flags)?;
        w.f32x3(self.accel_mps2)?;
        w.f32x3(self.gyro_rps)?;
        w.f32x3(self.mag_ut)?;
        w.f32(self.pressure_pa)?;
        w.f32(self.baro_altitude_m)?;
        w.f32(self.temperature_c)?;
        w.f64(self.lat_deg)?;
        w.f64(self.lon_deg)?;
        w.f32(self.alt_msl_m)?;
        w.f32x3(self.vel_ned_mps)?;
        w.u8(self.sats)?;
        w.u8(self.fix_type)?;
        w.bytes(&self.reserved0)?;
        w.f32(self.battery_voltage_v)?;
        w.i16(self.rssi_dbm)?;
        w.i16(self.snr_db_x100)?;
        w.u16(self.loss_pct_x100)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            stamp: r.sim_stamp()?,
            valid_flags: r.u32()?,
            accel_mps2: r.f32x3()?,
            gyro_rps: r.f32x3()?,
            mag_ut: r.f32x3()?,
            pressure_pa: r.f32()?,
            baro_altitude_m: r.f32()?,
            temperature_c: r.f32()?,
            lat_deg: r.f64()?,
            lon_deg: r.f64()?,
            alt_msl_m: r.f32()?,
            vel_ned_mps: r.f32x3()?,
            sats: r.u8()?,
            fix_type: r.u8()?,
            reserved0: r.bytes3()?,
            battery_voltage_v: r.f32()?,
            rssi_dbm: r.i16()?,
            snr_db_x100: r.i16()?,
            loss_pct_x100: r.u16()?,
        })
    }
}

impl WirePayload for HilResponseFrame {
    const MSG_TYPE: MsgType = MsgType::HilResponseFrame;
    const WIRE_LEN: usize = 72;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.stamp)?;
        w.u8(self.system_state)?;
        w.bytes(&self.reserved0)?;
        w.u32(self.flags)?;
        w.f32x3(self.position_ned_m)?;
        w.f32x3(self.velocity_ned_mps)?;
        w.f32x4(self.attitude_quat)?;
        w.u16x4(self.motor_cmd)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            stamp: r.sim_stamp()?,
            system_state: r.u8()?,
            reserved0: r.bytes3()?,
            flags: r.u32()?,
            position_ned_m: r.f32x3()?,
            velocity_ned_mps: r.f32x3()?,
            attitude_quat: r.f32x4()?,
            motor_cmd: r.u16x4()?,
        })
    }
}

impl WirePayload for ImuPayload {
    const MSG_TYPE: MsgType = MsgType::Imu;
    const WIRE_LEN: usize = 40;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.stamp)?;
        w.f32x3(self.accel_mps2)?;
        w.f32x3(self.gyro_rps)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            stamp: r.sim_stamp()?,
            accel_mps2: r.f32x3()?,
            gyro_rps: r.f32x3()?,
        })
    }
}

impl WirePayload for MagPayload {
    const MSG_TYPE: MsgType = MsgType::Mag;
    const WIRE_LEN: usize = 28;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.stamp)?;
        w.f32x3(self.field_ut)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            stamp: r.sim_stamp()?,
            field_ut: r.f32x3()?,
        })
    }
}

impl WirePayload for BaroPayload {
    const MSG_TYPE: MsgType = MsgType::Baro;
    const WIRE_LEN: usize = 28;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.stamp)?;
        w.f32(self.pressure_pa)?;
        w.f32(self.altitude_m)?;
        w.f32(self.temperature_c)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            stamp: r.sim_stamp()?,
            pressure_pa: r.f32()?,
            altitude_m: r.f32()?,
            temperature_c: r.f32()?,
        })
    }
}

impl WirePayload for GpsPayload {
    const MSG_TYPE: MsgType = MsgType::Gps;
    const WIRE_LEN: usize = 52;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.stamp)?;
        w.f64(self.lat_deg)?;
        w.f64(self.lon_deg)?;
        w.f32(self.alt_msl_m)?;
        w.f32x3(self.vel_ned_mps)?;
        w.u8(self.sats)?;
        w.u8(self.fix_type)?;
        w.bytes(&self.reserved0)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            stamp: r.sim_stamp()?,
            lat_deg: r.f64()?,
            lon_deg: r.f64()?,
            alt_msl_m: r.f32()?,
            vel_ned_mps: r.f32x3()?,
            sats: r.u8()?,
            fix_type: r.u8()?,
            reserved0: r.take::<2>()?,
        })
    }
}

impl WirePayload for SystemStatePayload {
    const MSG_TYPE: MsgType = MsgType::SystemState;
    const WIRE_LEN: usize = 28;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.stamp)?;
        w.u8(self.system_state)?;
        w.bytes(&self.reserved0)?;
        w.u32(self.flags)?;
        w.f32(self.battery_voltage_v)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            stamp: r.sim_stamp()?,
            system_state: r.u8()?,
            reserved0: r.bytes3()?,
            flags: r.u32()?,
            battery_voltage_v: r.f32()?,
        })
    }
}

impl WirePayload for TelemetrySnapshotPayload {
    const MSG_TYPE: MsgType = MsgType::TelemetrySnapshot;
    const WIRE_LEN: usize = 74;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.stamp)?;
        w.u8(self.system_state)?;
        w.bytes(&self.reserved0)?;
        w.u32(self.flags)?;
        w.f32x3(self.position_ned_m)?;
        w.f32x3(self.velocity_ned_mps)?;
        w.f32x4(self.attitude_quat)?;
        w.f32(self.battery_voltage_v)?;
        w.i16(self.rssi_dbm)?;
        w.i16(self.snr_db_x100)?;
        w.u16(self.loss_pct_x100)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            stamp: r.sim_stamp()?,
            system_state: r.u8()?,
            reserved0: r.bytes3()?,
            flags: r.u32()?,
            position_ned_m: r.f32x3()?,
            velocity_ned_mps: r.f32x3()?,
            attitude_quat: r.f32x4()?,
            battery_voltage_v: r.f32()?,
            rssi_dbm: r.i16()?,
            snr_db_x100: r.i16()?,
            loss_pct_x100: r.u16()?,
        })
    }
}

impl WirePayload for HeartbeatPayload {
    const MSG_TYPE: MsgType = MsgType::Heartbeat;
    const WIRE_LEN: usize = 24;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.sim_stamp(self.stamp)?;
        w.u8(self.system_state)?;
        w.bytes(&self.reserved0)?;
        w.u32(self.flags)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            stamp: r.sim_stamp()?,
            system_state: r.u8()?,
            reserved0: r.bytes3()?,
            flags: r.u32()?,
        })
    }
}

impl WirePayload for AckPayload {
    const MSG_TYPE: MsgType = MsgType::Ack;
    const WIRE_LEN: usize = 4;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u16(self.acked_seq)?;
        w.u8(self.acked_msg_type)?;
        w.u8(self.status)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            acked_seq: r.u16()?,
            acked_msg_type: r.u8()?,
            status: r.u8()?,
        })
    }
}

impl WirePayload for NackPayload {
    const MSG_TYPE: MsgType = MsgType::Nack;
    const WIRE_LEN: usize = 4;

    fn encode_payload(&self, out: &mut [u8]) -> Result<usize> {
        let mut w = Writer::new(out);
        w.u16(self.rejected_seq)?;
        w.u8(self.rejected_msg_type)?;
        w.u8(self.reason)?;
        Ok(w.len())
    }

    fn decode_payload(input: &[u8]) -> Result<Self> {
        expect_len(input, Self::WIRE_LEN)?;
        let mut r = Reader::new(input);
        Ok(Self {
            rejected_seq: r.u16()?,
            rejected_msg_type: r.u8()?,
            reason: r.u8()?,
        })
    }
}

fn expect_len(input: &[u8], expected: usize) -> Result<()> {
    if input.len() == expected {
        Ok(())
    } else {
        Err(Error::InvalidPayloadLength)
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
            return Err(Error::BufferTooSmall);
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

    fn i32(&mut self, value: i32) -> Result<()> {
        self.bytes(&value.to_le_bytes())
    }

    fn u32(&mut self, value: u32) -> Result<()> {
        self.bytes(&value.to_le_bytes())
    }

    fn u64(&mut self, value: u64) -> Result<()> {
        self.bytes(&value.to_le_bytes())
    }

    fn f32(&mut self, value: f32) -> Result<()> {
        self.u32(value.to_bits())
    }

    fn f64(&mut self, value: f64) -> Result<()> {
        self.u64(value.to_bits())
    }

    fn sim_stamp(&mut self, stamp: SimStamp) -> Result<()> {
        self.u64(stamp.sim_tick)?;
        self.u64(stamp.sim_time_us)
    }

    fn f32x3(&mut self, values: [f32; 3]) -> Result<()> {
        for value in values {
            self.f32(value)?;
        }
        Ok(())
    }

    fn f32x4(&mut self, values: [f32; 4]) -> Result<()> {
        for value in values {
            self.f32(value)?;
        }
        Ok(())
    }

    fn u16x4(&mut self, values: [u16; 4]) -> Result<()> {
        for value in values {
            self.u16(value)?;
        }
        Ok(())
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
            return Err(Error::InvalidPayloadLength);
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

    fn i32(&mut self) -> Result<i32> {
        Ok(i32::from_le_bytes(self.take::<4>()?))
    }

    fn u32(&mut self) -> Result<u32> {
        Ok(u32::from_le_bytes(self.take::<4>()?))
    }

    fn u64(&mut self) -> Result<u64> {
        Ok(u64::from_le_bytes(self.take::<8>()?))
    }

    fn f32(&mut self) -> Result<f32> {
        Ok(f32::from_bits(self.u32()?))
    }

    fn f64(&mut self) -> Result<f64> {
        Ok(f64::from_bits(self.u64()?))
    }

    fn sim_stamp(&mut self) -> Result<SimStamp> {
        Ok(SimStamp {
            sim_tick: self.u64()?,
            sim_time_us: self.u64()?,
        })
    }

    fn bytes3(&mut self) -> Result<[u8; 3]> {
        self.take::<3>()
    }

    fn f32x3(&mut self) -> Result<[f32; 3]> {
        Ok([self.f32()?, self.f32()?, self.f32()?])
    }

    fn f32x4(&mut self) -> Result<[f32; 4]> {
        Ok([self.f32()?, self.f32()?, self.f32()?, self.f32()?])
    }

    fn u16x4(&mut self) -> Result<[u16; 4]> {
        Ok([self.u16()?, self.u16()?, self.u16()?, self.u16()?])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_known_check_value() {
        assert_eq!(crc16_ccitt_false(b"123456789"), 0x29B1);
    }

    #[test]
    fn cobs_round_trips_zeroes() {
        let input = [0, 1, 2, 0, 3, 4, 5, 0];
        let mut encoded = [0u8; 16];
        let mut decoded = [0u8; 16];

        let encoded_len = cobs_encode(&input, &mut encoded).unwrap();
        let decoded_len = cobs_decode(&encoded[..encoded_len], &mut decoded).unwrap();

        assert_eq!(&decoded[..decoded_len], input);
    }

    #[test]
    fn hil_ready_round_trips() {
        let ready = HilReadyPayload;
        let mut raw = [0u8; raw_frame_len(HilReadyPayload::WIRE_LEN)];
        let mut encoded = [0u8; encoded_frame_len(HilReadyPayload::WIRE_LEN)];
        let encoded_len = encode_packet(&ready, 9, 2222, &mut raw, &mut encoded).unwrap();

        let mut decoded_raw = [0u8; raw_frame_len(HilReadyPayload::WIRE_LEN)];
        let packet = decode_packet(&encoded[..encoded_len], &mut decoded_raw).unwrap();
        let decoded = decode_payload::<HilReadyPayload>(&packet).unwrap();

        assert_eq!(packet.header.message_type().unwrap(), MsgType::HilReady);
        assert_eq!(packet.header.seq, 9);
        assert_eq!(packet.header.send_time_ms, 2222);
        assert_eq!(decoded, ready);
    }

    #[test]
    fn ping_pong_round_trips() {
        let ping = PingPayload;
        let mut raw = [0u8; raw_frame_len(PingPayload::WIRE_LEN)];
        let mut encoded = [0u8; encoded_frame_len(PingPayload::WIRE_LEN)];
        let encoded_len = encode_packet(&ping, 1, 100, &mut raw, &mut encoded).unwrap();

        let mut decoded_raw = [0u8; raw_frame_len(PingPayload::WIRE_LEN)];
        let packet = decode_packet(&encoded[..encoded_len], &mut decoded_raw).unwrap();
        let decoded = decode_payload::<PingPayload>(&packet).unwrap();

        assert_eq!(packet.header.message_type().unwrap(), MsgType::Ping);
        assert_eq!(decoded, ping);

        let pong = PongPayload;
        let encoded_len = encode_packet(&pong, 2, 101, &mut raw, &mut encoded).unwrap();
        let packet = decode_packet(&encoded[..encoded_len], &mut decoded_raw).unwrap();
        let decoded = decode_payload::<PongPayload>(&packet).unwrap();

        assert_eq!(packet.header.message_type().unwrap(), MsgType::Pong);
        assert_eq!(decoded, pong);
    }

    #[test]
    fn command_ack_nack_and_heartbeat_payloads_round_trip() {
        let heartbeat = HeartbeatPayload {
            stamp: SimStamp {
                sim_tick: 100,
                sim_time_us: 50_000,
            },
            system_state: 2,
            reserved0: [0; 3],
            flags: response_flags::ARMED | response_flags::MOTORS_VALID,
        };
        round_trip_payload(&heartbeat, 10, 1_000);

        let ack = AckPayload {
            acked_seq: 4,
            acked_msg_type: MsgType::Arm as u8,
            status: 0,
        };
        round_trip_payload(&ack, 11, 1_001);

        let nack = NackPayload {
            rejected_seq: 5,
            rejected_msg_type: MsgType::Disarm as u8,
            reason: 1,
        };
        round_trip_payload(&nack, 12, 1_002);

        round_trip_payload(&ArmPayload, 13, 1_003);
        round_trip_payload(&DisarmPayload, 14, 1_004);
        round_trip_payload(
            &GlobalWaypointPayload {
                ref_stamp: SimStamp {
                    sim_tick: 15,
                    sim_time_us: 30_000,
                },
                lat_deg: 30.2672,
                lon_deg: -97.7431,
                alt_msl_m: 171.0,
                yaw_deg: 45.0,
            },
            15,
            1_005,
        );
        round_trip_payload(
            &MissionWaypointPayload(GlobalWaypointPayload {
                ref_stamp: SimStamp {
                    sim_tick: 16,
                    sim_time_us: 32_000,
                },
                lat_deg: 30.2673,
                lon_deg: -97.7432,
                alt_msl_m: 172.0,
                yaw_deg: 90.0,
            }),
            16,
            1_006,
        );
        round_trip_payload(
            &CvWaypointPayload {
                ref_stamp: SimStamp {
                    sim_tick: 17,
                    sim_time_us: 34_000,
                },
                dir_body: [1.0, 0.0, 0.0],
                confidence: 0.75,
            },
            17,
            1_007,
        );
        round_trip_payload(
            &TofWaypointPayload {
                ref_stamp: SimStamp {
                    sim_tick: 18,
                    sim_time_us: 36_000,
                },
                distance_m: 1.5,
                bearing_deg: 10.0,
                elevation_deg: -2.0,
            },
            18,
            1_008,
        );
        round_trip_payload(&RtlPayload, 19, 1_009);
    }

    #[test]
    fn bench_payloads_round_trip() {
        round_trip_payload(
            &BenchEnablePayload {
                magic: bench::ENABLE_MAGIC,
                timeout_ms: 30_000,
                reserved0: [0; 2],
            },
            20,
            2_000,
        );
        round_trip_payload(&BenchDisablePayload, 21, 2_001);
        round_trip_payload(
            &MotorTestPayload {
                motor_mask: bench::MOTOR_MASK_M1,
                mode: motor_test_mode::RAW_DSHOT,
                reserved0: [0; 2],
                value: 150,
                duration_ms: 1000,
                ramp_ms: 0,
                reserved1: 0,
            },
            22,
            2_002,
        );
        round_trip_payload(
            &MotorSweepPayload {
                motor_mask: bench::MOTOR_MASK_ALL,
                mode: motor_test_mode::RAW_DSHOT,
                reserved0: [0; 2],
                start_value: 150,
                end_value: 400,
                step_value: 50,
                step_duration_ms: 750,
                zero_between_ms: 500,
                repeat_count: 1,
                reserved1: 0,
            },
            23,
            2_003,
        );
        round_trip_payload(&MotorStopPayload, 24, 2_004);
        round_trip_payload(
            &DshotCommandPayload {
                motor_mask: bench::MOTOR_MASK_M1,
                command: 1,
                repeat_count: 6,
                reserved0: 0,
            },
            25,
            2_005,
        );
        round_trip_payload(&ActuatorStatusRequestPayload, 26, 2_006);
        round_trip_payload(
            &ActuatorStatusPayload {
                armed: 1,
                bench_enabled: 1,
                active_motor_mask: bench::MOTOR_MASK_ALL,
                mode: motor_test_mode::RAW_DSHOT,
                commanded_dshot: [120, 121, 122, 123],
                last_command_age_ms: 10,
                bench_timeout_ms: 2000,
                flags: actuator_flags::OUTPUT_ACTIVE | actuator_flags::BENCH_MODE_ENABLED,
            },
            27,
            2_007,
        );
    }

    #[test]
    fn lora_v1_message_ids_are_active_and_reserved_ids_are_inactive() {
        assert_eq!(MsgType::try_from(81), Ok(MsgType::LoRaFlightSnapshot));
        assert_eq!(MsgType::try_from(83), Ok(MsgType::LoRaGpsSnapshot));
        assert_eq!(MsgType::try_from(85), Ok(MsgType::LoRaEvent));
        assert_eq!(MsgType::try_from(86), Ok(MsgType::LoRaFaults));
        assert_eq!(MsgType::try_from(87), Ok(MsgType::LoRaLinkStatus));
        assert_eq!(MsgType::try_from(100), Ok(MsgType::LoRaCommand));
        assert_eq!(MsgType::try_from(101), Ok(MsgType::LoRaCommandAck));
        assert_eq!(MsgType::try_from(102), Ok(MsgType::LoRaSetProfile));
        assert_eq!(MsgType::try_from(103), Ok(MsgType::LoRaRequestSnapshot));

        assert_eq!(MsgType::try_from(80), Err(Error::UnknownMsgType));
        assert_eq!(MsgType::try_from(82), Err(Error::UnknownMsgType));
        assert_eq!(MsgType::try_from(84), Err(Error::UnknownMsgType));
    }

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
    fn lora_payloads_round_trip() {
        round_trip_payload(
            &LoRaFlightSnapshotPayload {
                time_ms: 123_456,
                state: lora_state::ASCENT,
                mode: lora_mode::AUTO,
                flags: lora_flags::ARMED | lora_flags::GPS_VALID,
                altitude_dm: 12_345,
                vertical_velocity_cms: 678,
                accel_mag_cms2: 1_520,
                battery_mv: 16_200,
                pyro_or_actuator_flags: lora_pyro_actuator_flags::MOTOR_OUTPUT_ACTIVE,
                fault_summary: lora_fault::BATTERY as u16,
            },
            40,
            4_000,
        );
        round_trip_payload(
            &LoRaGpsSnapshotPayload {
                time_ms: 123_500,
                lat_e7: 302_672_000,
                lon_e7: -977_431_000,
                alt_msl_dm: 1_710,
                ground_speed_cms: 550,
                heading_cdeg: 9_000,
                sats: 12,
                fix_type: 3,
            },
            41,
            4_001,
        );
        round_trip_payload(
            &LoRaLinkStatusPayload {
                time_ms: 124_000,
                uplink_rssi_dbm: -72,
                uplink_snr_x4: 28,
                downlink_rssi_dbm: -70,
                downlink_snr_x4: 30,
                rx_packets_delta: 98,
                tx_packets_delta: 101,
                lost_packets_delta: 2,
                active_profile: lora_profile::SF7_500,
                telemetry_rate_hz: 10,
                reserved: 0,
            },
            42,
            4_002,
        );
        round_trip_payload(
            &LoRaEventPayload {
                time_ms: 124_100,
                event_id: lora_event_id::STATE_CHANGE,
                severity: lora_event_severity::INFO,
                arg0: lora_state::ARMED as i32,
                arg1: lora_state::ASCENT as i32,
            },
            43,
            4_003,
        );
        round_trip_payload(
            &LoRaFaultsPayload {
                time_ms: 124_200,
                active_faults: lora_fault::GPS | lora_fault::RADIO,
                latched_faults: lora_fault::GPS,
                inhibit_flags: 0,
            },
            44,
            4_004,
        );
        round_trip_payload(
            &LoRaCommandPayload {
                command_id: lora_command_id::ABORT,
                command_seq: 77,
                expires_ms: 250,
                flags: lora_command_flags::URGENT | lora_command_flags::ALLOW_WHILE_FAILSAFE,
                arg0: 0,
                arg1: 0,
            },
            45,
            4_005,
        );
        round_trip_payload(
            &LoRaCommandAckPayload {
                command_id: lora_command_id::ABORT,
                command_seq: 77,
                status: lora_command_status::ACCEPTED,
                reason: lora_command_reason::NONE,
                state: lora_state::ABORT,
                reserved: 0,
                detail: 0,
            },
            46,
            4_006,
        );
        round_trip_payload(
            &LoRaSetProfilePayload {
                command_seq: 78,
                profile: lora_profile::SF8_500,
                telemetry_rate_hz: 5,
                gps_rate_hz: 1,
                link_status_rate_hz: 1,
                flags: lora_profile_flags::TEMPORARY,
            },
            47,
            4_007,
        );
        round_trip_payload(
            &LoRaRequestSnapshotPayload {
                command_seq: 79,
                request_flags: lora_request_flags::FLIGHT_SNAPSHOT | lora_request_flags::FAULTS,
            },
            48,
            4_008,
        );
    }

    #[test]
    fn lora_event_payload_explicit_layout_has_no_padding() {
        let payload = LoRaEventPayload {
            time_ms: 0x1122_3344,
            event_id: 0x5566,
            severity: 0x77,
            arg0: 0x0102_0304,
            arg1: -2,
        };
        let mut encoded = [0u8; LoRaEventPayload::WIRE_LEN];

        assert_eq!(payload.encode_payload(&mut encoded), Ok(15));
        assert_eq!(
            encoded,
            [
                0x44, 0x33, 0x22, 0x11, 0x66, 0x55, 0x77, 0x04, 0x03, 0x02, 0x01, 0xFE, 0xFF, 0xFF,
                0xFF,
            ]
        );
    }

    #[test]
    fn lora_faults_payload_explicit_layout_has_no_padding() {
        let payload = LoRaFaultsPayload {
            time_ms: 0x1122_3344,
            active_faults: 0x5566_7788,
            latched_faults: 0x99AA_BBCC,
            inhibit_flags: 0xDDEE,
        };
        let mut encoded = [0u8; LoRaFaultsPayload::WIRE_LEN];

        assert_eq!(payload.encode_payload(&mut encoded), Ok(14));
        assert_eq!(
            encoded,
            [
                0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 0xCC, 0xBB, 0xAA, 0x99, 0xEE, 0xDD,
            ]
        );
    }

    #[test]
    fn hil_sensor_frame_round_trips() {
        let frame = HilSensorFrame {
            stamp: SimStamp {
                sim_tick: 15_234,
                sim_time_us: 76_170_000,
            },
            valid_flags: valid::ACCEL | valid::GYRO | valid::GPS,
            accel_mps2: [1.0, 2.0, 3.0],
            gyro_rps: [0.1, 0.2, 0.3],
            mag_ut: [10.0, 11.0, 12.0],
            pressure_pa: 101_325.0,
            baro_altitude_m: 12.5,
            temperature_c: 25.0,
            lat_deg: 30.2672,
            lon_deg: -97.7431,
            alt_msl_m: 171.0,
            vel_ned_mps: [4.0, 5.0, -0.5],
            sats: 12,
            fix_type: 3,
            reserved0: [0; 3],
            battery_voltage_v: 16.2,
            rssi_dbm: -48,
            snr_db_x100: 1_150,
            loss_pct_x100: 25,
        };

        let mut raw = [0u8; 256];
        let mut encoded = [0u8; 256];
        let encoded_len = encode_packet(&frame, 7, 1234, &mut raw, &mut encoded).unwrap();

        let mut decoded_raw = [0u8; 256];
        let packet = decode_packet(&encoded[..encoded_len], &mut decoded_raw).unwrap();
        let decoded = decode_payload::<HilSensorFrame>(&packet).unwrap();

        assert_eq!(packet.header.seq, 7);
        assert_eq!(packet.header.send_time_ms, 1234);
        assert_eq!(decoded, frame);
    }

    #[test]
    fn hil_response_frame_round_trips() {
        let frame = HilResponseFrame {
            stamp: SimStamp {
                sim_tick: 42,
                sim_time_us: 210_000,
            },
            system_state: 2,
            reserved0: [0; 3],
            flags: response_flags::ARMED | response_flags::ESTIMATOR_VALID,
            position_ned_m: [1.0, 2.0, -3.0],
            velocity_ned_mps: [0.1, 0.2, 0.3],
            attitude_quat: [1.0, 0.0, 0.0, 0.0],
            motor_cmd: [1000, 1100, 1200, 1300],
        };

        let mut raw = [0u8; raw_frame_len(HilSensorFrame::WIRE_LEN)];
        let mut encoded = [0u8; encoded_frame_len(HilSensorFrame::WIRE_LEN)];
        let encoded_len = encode_packet(&frame, 8, 1235, &mut raw, &mut encoded).unwrap();

        let mut decoded_raw = [0u8; raw_frame_len(HilSensorFrame::WIRE_LEN)];
        let packet = decode_packet(&encoded[..encoded_len], &mut decoded_raw).unwrap();
        let decoded = decode_payload::<HilResponseFrame>(&packet).unwrap();

        assert_eq!(
            packet.header.message_type().unwrap(),
            MsgType::HilResponseFrame
        );
        assert_eq!(decoded, frame);
    }

    #[test]
    fn telemetry_payloads_round_trip() {
        let stamp = SimStamp {
            sim_tick: 101,
            sim_time_us: 2_020_000,
        };

        round_trip_payload(
            &ImuPayload {
                stamp,
                accel_mps2: [1.0, 2.0, -9.81],
                gyro_rps: [0.1, 0.2, 0.3],
            },
            30,
            3_000,
        );
        round_trip_payload(
            &BaroPayload {
                stamp,
                pressure_pa: 101_325.0,
                altitude_m: 171.0,
                temperature_c: 25.0,
            },
            31,
            3_001,
        );
        round_trip_payload(
            &MagPayload {
                stamp,
                field_ut: [10.0, -20.0, 30.0],
            },
            32,
            3_002,
        );
        round_trip_payload(
            &GpsPayload {
                stamp,
                lat_deg: 30.2672,
                lon_deg: -97.7431,
                alt_msl_m: 171.0,
                vel_ned_mps: [0.0, 0.0, 0.0],
                sats: 12,
                fix_type: 3,
                reserved0: [0; 2],
            },
            33,
            3_003,
        );
        round_trip_payload(
            &SystemStatePayload {
                stamp,
                system_state: 2,
                reserved0: [0; 3],
                flags: response_flags::ARMED | response_flags::MOTORS_VALID,
                battery_voltage_v: 16.2,
            },
            34,
            3_004,
        );
        round_trip_payload(
            &TelemetrySnapshotPayload {
                stamp,
                system_state: 2,
                reserved0: [0; 3],
                flags: response_flags::ARMED | response_flags::ESTIMATOR_VALID,
                position_ned_m: [1.0, 2.0, -3.0],
                velocity_ned_mps: [0.1, 0.2, 0.3],
                attitude_quat: [1.0, 0.0, 0.0, 0.0],
                battery_voltage_v: 16.2,
                rssi_dbm: -48,
                snr_db_x100: 1_150,
                loss_pct_x100: 25,
            },
            35,
            3_005,
        );
    }

    #[test]
    fn corrupt_crc_is_rejected() {
        let mut raw = [0u8; raw_frame_len(HilReadyPayload::WIRE_LEN)];
        let mut encoded = [0u8; encoded_frame_len(HilReadyPayload::WIRE_LEN)];

        let header = Header::new(MsgType::HilReady, 9, 2222, 0);
        let mut raw_len = header.encode(&mut raw).unwrap();
        raw[raw_len..raw_len + CRC_LEN].copy_from_slice(&0x1234u16.to_le_bytes());
        raw_len += CRC_LEN;

        let cobs_len = cobs_encode(&raw[..raw_len], &mut encoded).unwrap();
        encoded[cobs_len] = FRAME_DELIMITER;
        let encoded_len = cobs_len + 1;

        let mut decoded_raw = [0u8; raw_frame_len(HilReadyPayload::WIRE_LEN)];
        assert_eq!(
            decode_packet(&encoded[..encoded_len], &mut decoded_raw),
            Err(Error::CrcMismatch)
        );
    }

    #[test]
    fn missing_delimiter_is_rejected() {
        let ready = HilReadyPayload;
        let mut raw = [0u8; raw_frame_len(HilReadyPayload::WIRE_LEN)];
        let mut encoded = [0u8; encoded_frame_len(HilReadyPayload::WIRE_LEN)];
        let encoded_len = encode_packet(&ready, 9, 2222, &mut raw, &mut encoded).unwrap();

        let mut decoded_raw = [0u8; raw_frame_len(HilReadyPayload::WIRE_LEN)];
        assert_eq!(
            decode_packet(&encoded[..encoded_len - 1], &mut decoded_raw),
            Err(Error::BadDelimiter)
        );
    }

    fn round_trip_payload<P>(payload: &P, seq: u16, send_time_ms: u32)
    where
        P: WirePayload + Copy + PartialEq + core::fmt::Debug,
    {
        let mut raw = [0u8; 256];
        let mut encoded = [0u8; 256];
        let encoded_len =
            encode_packet(payload, seq, send_time_ms, &mut raw, &mut encoded).unwrap();

        let mut decoded_raw = [0u8; 256];
        let packet = decode_packet(&encoded[..encoded_len], &mut decoded_raw).unwrap();
        let decoded = decode_payload::<P>(&packet).unwrap();

        assert_eq!(packet.header.seq, seq);
        assert_eq!(packet.header.send_time_ms, send_time_ms);
        assert_eq!(decoded, *payload);
    }
}
