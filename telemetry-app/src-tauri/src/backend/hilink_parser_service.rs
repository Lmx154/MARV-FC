use serde::{Deserialize, Serialize};

const PROTOCOL_VERSION: u8 = 1;
const HEADER_LEN: usize = 12;
const CRC_LEN: usize = 2;
const FRAME_DELIMITER: u8 = 0;
const MAX_FRAME_LEN: usize = 512;
const HEADER_FLAG_REQUEST_ACK: u8 = 1 << 0;
const HEADER_FLAG_URGENT_CONTROL: u8 = 1 << 3;

const BENCH_ENABLE_MAGIC: u32 = 0x4D4F_544F;
const MOTOR_MASK_M1: u8 = 0x01;
const MOTOR_MASK_M2: u8 = 0x02;
const MOTOR_MASK_M3: u8 = 0x04;
const MOTOR_MASK_M4: u8 = 0x08;
const MOTOR_MASK_ALL: u8 = 0x0F;
const MOTOR_TEST_MODE_STOP: u8 = 0;
const MOTOR_TEST_MODE_RAW_DSHOT: u8 = 1;
const MOTOR_TEST_MODE_NORMALIZED: u8 = 2;

#[derive(Clone, Debug)]
pub enum HilinkCommand {
    Ping,
    Arm,
    Disarm,
    Rtl,
    BenchEnable(BenchEnableCommand),
    BenchDisable,
    MotorTest(MotorTestCommand),
    MotorSweep(MotorSweepCommand),
    MotorStop,
    DshotCommand(DshotCommand),
    ActuatorStatusRequest,
    MixerMotorOrder(MixerMotorOrderCommand),
    ControlWaypoint(GlobalWaypointCommand),
    MissionWaypoint(GlobalWaypointCommand),
    CvWaypoint(CvWaypointCommand),
    TofWaypoint(TofWaypointCommand),
    HilSensorFrame(HilSensorFrameCommand),
}

impl HilinkCommand {
    pub fn msg_type_id(&self) -> u8 {
        self.msg_type()
    }

    pub fn label(&self) -> &'static str {
        match self {
            Self::Ping => "Ping",
            Self::Arm => "Arm",
            Self::Disarm => "Disarm",
            Self::Rtl => "Rtl",
            Self::BenchEnable(_) => "BenchEnable",
            Self::BenchDisable => "BenchDisable",
            Self::MotorTest(_) => "MotorTest",
            Self::MotorSweep(_) => "MotorSweep",
            Self::MotorStop => "MotorStop",
            Self::DshotCommand(_) => "DshotCommand",
            Self::ActuatorStatusRequest => "ActuatorStatusRequest",
            Self::MixerMotorOrder(_) => "MixerMotorOrder",
            Self::ControlWaypoint(_) => "ControlWaypoint",
            Self::MissionWaypoint(_) => "MissionWaypoint",
            Self::CvWaypoint(_) => "CvWaypoint",
            Self::TofWaypoint(_) => "TofWaypoint",
            Self::HilSensorFrame(_) => "HilSensorFrame",
        }
    }

    fn msg_type(&self) -> u8 {
        match self {
            Self::Ping => 1,
            Self::HilSensorFrame(_) => 10,
            Self::Arm => 40,
            Self::Disarm => 41,
            Self::ControlWaypoint(_) => 42,
            Self::CvWaypoint(_) => 43,
            Self::TofWaypoint(_) => 44,
            Self::MissionWaypoint(_) => 45,
            Self::Rtl => 46,
            Self::BenchEnable(_) => 60,
            Self::BenchDisable => 61,
            Self::MotorTest(_) => 62,
            Self::MotorSweep(_) => 63,
            Self::MotorStop => 64,
            Self::DshotCommand(_) => 65,
            Self::ActuatorStatusRequest => 66,
            Self::MixerMotorOrder(_) => 68,
        }
    }

    fn flags(&self) -> u8 {
        match self {
            Self::Ping | Self::HilSensorFrame(_) => 0,
            Self::MotorStop => HEADER_FLAG_REQUEST_ACK | HEADER_FLAG_URGENT_CONTROL,
            Self::Arm
            | Self::Disarm
            | Self::Rtl
            | Self::BenchEnable(_)
            | Self::BenchDisable
            | Self::MotorTest(_)
            | Self::MotorSweep(_)
            | Self::DshotCommand(_)
            | Self::ActuatorStatusRequest
            | Self::MixerMotorOrder(_)
            | Self::ControlWaypoint(_)
            | Self::MissionWaypoint(_)
            | Self::CvWaypoint(_)
            | Self::TofWaypoint(_) => HEADER_FLAG_REQUEST_ACK,
        }
    }

    fn payload(&self) -> Vec<u8> {
        match self {
            Self::Ping
            | Self::Arm
            | Self::Disarm
            | Self::Rtl
            | Self::BenchDisable
            | Self::MotorStop
            | Self::ActuatorStatusRequest => Vec::new(),
            Self::BenchEnable(command) => {
                let mut payload = Vec::with_capacity(8);
                push_u32(&mut payload, BENCH_ENABLE_MAGIC);
                payload.extend(command.timeout_ms.to_le_bytes());
                payload.extend([0, 0]);
                payload
            }
            Self::MotorTest(command) => {
                let mut payload = Vec::with_capacity(12);
                payload.push(command.motor_mask);
                payload.push(command.mode);
                payload.extend([0, 0]);
                payload.extend(command.value.to_le_bytes());
                payload.extend(command.duration_ms.to_le_bytes());
                payload.extend(command.ramp_ms.to_le_bytes());
                payload.extend(0u16.to_le_bytes());
                payload
            }
            Self::MotorSweep(command) => {
                let mut payload = Vec::with_capacity(16);
                payload.push(command.motor_mask);
                payload.push(command.mode);
                payload.extend([0, 0]);
                payload.extend(command.start_value.to_le_bytes());
                payload.extend(command.end_value.to_le_bytes());
                payload.extend(command.step_value.to_le_bytes());
                payload.extend(command.step_duration_ms.to_le_bytes());
                payload.extend(command.zero_between_ms.to_le_bytes());
                payload.push(command.repeat_count);
                payload.push(0);
                payload
            }
            Self::DshotCommand(command) => {
                let mut payload = Vec::with_capacity(4);
                payload.push(command.motor_mask);
                payload.push(command.command);
                payload.push(command.repeat_count);
                payload.push(0);
                payload
            }
            Self::MixerMotorOrder(command) => command.output_for_motor.to_vec(),
            Self::ControlWaypoint(command) | Self::MissionWaypoint(command) => {
                let mut payload = Vec::with_capacity(40);
                push_stamp(&mut payload, command.ref_sim_tick, command.ref_sim_time_us);
                push_f64(&mut payload, command.lat_deg);
                push_f64(&mut payload, command.lon_deg);
                push_f32(&mut payload, command.alt_msl_m);
                push_f32(&mut payload, command.yaw_deg);
                payload
            }
            Self::CvWaypoint(command) => {
                let mut payload = Vec::with_capacity(32);
                push_stamp(&mut payload, command.ref_sim_tick, command.ref_sim_time_us);
                push_f32x3(&mut payload, command.dir_body);
                push_f32(&mut payload, command.confidence);
                payload
            }
            Self::TofWaypoint(command) => {
                let mut payload = Vec::with_capacity(28);
                push_stamp(&mut payload, command.ref_sim_tick, command.ref_sim_time_us);
                push_f32(&mut payload, command.distance_m);
                push_f32(&mut payload, command.bearing_deg);
                push_f32(&mut payload, command.elevation_deg);
                payload
            }
            Self::HilSensorFrame(frame) => {
                let mut payload = Vec::with_capacity(115);
                push_stamp(&mut payload, frame.sim_tick, frame.sim_time_us);
                push_u32(&mut payload, frame.valid_flags);
                push_f32x3(&mut payload, frame.accel_mps2);
                push_f32x3(&mut payload, frame.gyro_rps);
                push_f32x3(&mut payload, frame.mag_ut);
                push_f32(&mut payload, frame.pressure_pa);
                push_f32(&mut payload, frame.baro_altitude_m);
                push_f32(&mut payload, frame.temperature_c);
                push_f64(&mut payload, frame.lat_deg);
                push_f64(&mut payload, frame.lon_deg);
                push_f32(&mut payload, frame.alt_msl_m);
                push_f32x3(&mut payload, frame.vel_ned_mps);
                payload.push(frame.sats);
                payload.push(frame.fix_type);
                payload.extend([0, 0, 0]);
                push_f32(&mut payload, frame.battery_voltage_v);
                payload.extend(frame.rssi_dbm.to_le_bytes());
                payload.extend(frame.snr_db_x100.to_le_bytes());
                payload.extend(frame.loss_pct_x100.to_le_bytes());
                payload
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct BenchEnableCommand {
    pub timeout_ms: u16,
}

impl Default for BenchEnableCommand {
    fn default() -> Self {
        Self { timeout_ms: 0 }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct MotorTestCommand {
    pub motor_mask: u8,
    pub mode: u8,
    pub value: u16,
    pub duration_ms: u16,
    pub ramp_ms: u16,
}

impl Default for MotorTestCommand {
    fn default() -> Self {
        Self {
            motor_mask: MOTOR_MASK_ALL,
            mode: MOTOR_TEST_MODE_RAW_DSHOT,
            value: 120,
            duration_ms: 1_000,
            ramp_ms: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct MotorSweepCommand {
    pub motor_mask: u8,
    pub mode: u8,
    pub start_value: u16,
    pub end_value: u16,
    pub step_value: u16,
    pub step_duration_ms: u16,
    pub zero_between_ms: u16,
    pub repeat_count: u8,
}

impl Default for MotorSweepCommand {
    fn default() -> Self {
        Self {
            motor_mask: MOTOR_MASK_ALL,
            mode: MOTOR_TEST_MODE_RAW_DSHOT,
            start_value: 120,
            end_value: 200,
            step_value: 20,
            step_duration_ms: 500,
            zero_between_ms: 150,
            repeat_count: 1,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct DshotCommand {
    pub motor_mask: u8,
    pub command: u8,
    pub repeat_count: u8,
}

impl Default for DshotCommand {
    fn default() -> Self {
        Self {
            motor_mask: MOTOR_MASK_ALL,
            command: 0,
            repeat_count: 1,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct MixerMotorOrderCommand {
    pub output_for_motor: [u8; 4],
}

impl Default for MixerMotorOrderCommand {
    fn default() -> Self {
        Self {
            output_for_motor: [1, 2, 3, 4],
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct GlobalWaypointCommand {
    pub ref_sim_tick: u64,
    pub ref_sim_time_us: u64,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
    pub yaw_deg: f32,
}

impl Default for GlobalWaypointCommand {
    fn default() -> Self {
        Self {
            ref_sim_tick: 0,
            ref_sim_time_us: 0,
            lat_deg: 26.310942,
            lon_deg: -98.174728,
            alt_msl_m: 28.711437,
            yaw_deg: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct CvWaypointCommand {
    pub ref_sim_tick: u64,
    pub ref_sim_time_us: u64,
    pub dir_body: [f32; 3],
    pub confidence: f32,
}

impl Default for CvWaypointCommand {
    fn default() -> Self {
        Self {
            ref_sim_tick: 0,
            ref_sim_time_us: 0,
            dir_body: [1.0, 0.0, 0.0],
            confidence: 1.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct TofWaypointCommand {
    pub ref_sim_tick: u64,
    pub ref_sim_time_us: u64,
    pub distance_m: f32,
    pub bearing_deg: f32,
    pub elevation_deg: f32,
}

impl Default for TofWaypointCommand {
    fn default() -> Self {
        Self {
            ref_sim_tick: 0,
            ref_sim_time_us: 0,
            distance_m: 1.0,
            bearing_deg: 0.0,
            elevation_deg: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct HilSensorFrameCommand {
    pub sim_tick: u64,
    pub sim_time_us: u64,
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
    pub battery_voltage_v: f32,
    pub rssi_dbm: i16,
    pub snr_db_x100: i16,
    pub loss_pct_x100: u16,
}

impl Default for HilSensorFrameCommand {
    fn default() -> Self {
        Self {
            sim_tick: 1,
            sim_time_us: 20_000,
            valid_flags: (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6),
            accel_mps2: [0.0, 0.0, -9.81],
            gyro_rps: [0.0, 0.0, 0.0],
            mag_ut: [0.0, 0.0, 0.0],
            pressure_pa: 101_325.0,
            baro_altitude_m: 28.711437,
            temperature_c: 25.0,
            lat_deg: 26.310942,
            lon_deg: -98.174728,
            alt_msl_m: 28.711437,
            vel_ned_mps: [0.0, 0.0, 0.0],
            sats: 12,
            fix_type: 3,
            battery_voltage_v: 16.2,
            rssi_dbm: -48,
            snr_db_x100: 1150,
            loss_pct_x100: 0,
        }
    }
}

pub fn encode_hilink_command(
    command: &HilinkCommand,
    seq: u16,
    send_time_ms: u32,
) -> Result<Vec<u8>, String> {
    encode_packet(
        command.msg_type(),
        command.flags(),
        seq,
        send_time_ms,
        &command.payload(),
    )
    .map_err(|error| error.label().to_string())
}

pub fn parse_hilink_frames_for_display(bytes: &[u8], direction: &str) -> Vec<String> {
    let mut summaries = Vec::new();
    let mut frame = Vec::new();

    for &byte in bytes {
        frame.push(byte);
        if byte != FRAME_DELIMITER {
            continue;
        }

        if frame.len() > 1 {
            summaries.push(parse_hilink_frame_for_display(&frame, direction));
        }
        frame.clear();
    }

    if !frame.is_empty() {
        summaries.push(format!(
            "{direction} partial frame: {} bytes without 0x00 delimiter",
            frame.len()
        ));
    }

    summaries
}

pub fn hilink_msg_type_label(value: u8) -> String {
    msg_type_label(value)
}

fn parse_hilink_frame_for_display(frame: &[u8], direction: &str) -> String {
    match decode_message(
        frame,
        if direction == "TX" {
            FrameDirection::Tx
        } else {
            FrameDirection::Rx
        },
    ) {
        Ok(message) => format_hilink_message_summary(direction, &message),
        Err(error) => format!(
            "{direction} parse error: {} ({} bytes)",
            error.label(),
            frame.len()
        ),
    }
}

fn format_hilink_message_summary(direction: &str, message: &HilinkMessage) -> String {
    let header = &message.header;
    let details = match &message.payload {
        HilinkPayload::Empty => "payload=empty".to_string(),
        HilinkPayload::Ack(payload) => format!(
            "acked_seq={} acked_msg={} status={}",
            payload.acked_seq,
            msg_type_label(payload.acked_msg_type),
            payload.status
        ),
        HilinkPayload::Nack(payload) => format!(
            "rejected_seq={} rejected_msg={} reason={}",
            payload.rejected_seq,
            msg_type_label(payload.rejected_msg_type),
            payload.reason
        ),
        HilinkPayload::HilResponseFrame(payload) => format!(
            "tick={} time={}us state={} flags={} motors=[{}, {}, {}, {}]",
            payload.stamp.sim_tick,
            payload.stamp.sim_time_us,
            payload.system_state,
            response_flags(payload.flags),
            payload.motor_cmd[0],
            payload.motor_cmd[1],
            payload.motor_cmd[2],
            payload.motor_cmd[3]
        ),
        HilinkPayload::Heartbeat(payload) => format!(
            "tick={} time={}us state={} flags={}",
            payload.stamp.sim_tick,
            payload.stamp.sim_time_us,
            payload.system_state,
            response_flags(payload.flags)
        ),
        HilinkPayload::HilSensorFrame(payload) => format!(
            "tick={} time={}us valid={} accel=[{}, {}, {}] gyro=[{}, {}, {}] gps=({}, {}, {})",
            payload.stamp.sim_tick,
            payload.stamp.sim_time_us,
            valid_flags(payload.valid_flags),
            fmt_f32(payload.accel_mps2[0]),
            fmt_f32(payload.accel_mps2[1]),
            fmt_f32(payload.accel_mps2[2]),
            fmt_f32(payload.gyro_rps[0]),
            fmt_f32(payload.gyro_rps[1]),
            fmt_f32(payload.gyro_rps[2]),
            fmt_f64(payload.lat_deg),
            fmt_f64(payload.lon_deg),
            fmt_f32(payload.alt_msl_m)
        ),
        HilinkPayload::MotorState(payload) => format!(
            "tick={} time={}us motors=[{}, {}, {}, {}]",
            payload.stamp.sim_tick,
            payload.stamp.sim_time_us,
            payload.motor_cmd[0],
            payload.motor_cmd[1],
            payload.motor_cmd[2],
            payload.motor_cmd[3]
        ),
        HilinkPayload::GlobalWaypoint(payload) => format!(
            "ref_tick={} ref_time={}us lat={} lon={} alt={} yaw={}",
            payload.ref_stamp.sim_tick,
            payload.ref_stamp.sim_time_us,
            fmt_f64(payload.lat_deg),
            fmt_f64(payload.lon_deg),
            fmt_f32(payload.alt_msl_m),
            fmt_f32(payload.yaw_deg)
        ),
        HilinkPayload::CvWaypoint(payload) => format!(
            "ref_tick={} dir=[{}, {}, {}] confidence={}",
            payload.ref_stamp.sim_tick,
            fmt_f32(payload.dir_body[0]),
            fmt_f32(payload.dir_body[1]),
            fmt_f32(payload.dir_body[2]),
            fmt_f32(payload.confidence)
        ),
        HilinkPayload::TofWaypoint(payload) => format!(
            "ref_tick={} distance={}m bearing={}deg elevation={}deg",
            payload.ref_stamp.sim_tick,
            fmt_f32(payload.distance_m),
            fmt_f32(payload.bearing_deg),
            fmt_f32(payload.elevation_deg)
        ),
        HilinkPayload::Imu(payload) => format!(
            "tick={} accel=[{}, {}, {}] gyro=[{}, {}, {}]",
            payload.stamp.sim_tick,
            fmt_f32(payload.accel_mps2[0]),
            fmt_f32(payload.accel_mps2[1]),
            fmt_f32(payload.accel_mps2[2]),
            fmt_f32(payload.gyro_rps[0]),
            fmt_f32(payload.gyro_rps[1]),
            fmt_f32(payload.gyro_rps[2])
        ),
        HilinkPayload::Mag(payload) => format!(
            "tick={} mag=[{}, {}, {}]",
            payload.stamp.sim_tick,
            fmt_f32(payload.mag_ut[0]),
            fmt_f32(payload.mag_ut[1]),
            fmt_f32(payload.mag_ut[2])
        ),
        HilinkPayload::Baro(payload) => format!(
            "tick={} pressure={}Pa altitude={}m temp={}C",
            payload.stamp.sim_tick,
            fmt_f32(payload.pressure_pa),
            fmt_f32(payload.altitude_m),
            fmt_f32(payload.temperature_c)
        ),
        HilinkPayload::Gps(payload) => format!(
            "tick={} lat={} lon={} alt={}m vel=[{}, {}, {}] sats={} fix={}",
            payload.stamp.sim_tick,
            fmt_f64(payload.lat_deg),
            fmt_f64(payload.lon_deg),
            fmt_f32(payload.alt_msl_m),
            fmt_f32(payload.vel_ned_mps[0]),
            fmt_f32(payload.vel_ned_mps[1]),
            fmt_f32(payload.vel_ned_mps[2]),
            payload.sats,
            sats_label(payload.fix_type)
        ),
        HilinkPayload::EstimatorState(payload) => format!(
            "tick={} pos=[{}, {}, {}] vel=[{}, {}, {}] quat=[{}, {}, {}, {}]",
            payload.stamp.sim_tick,
            fmt_f32(payload.position_ned_m[0]),
            fmt_f32(payload.position_ned_m[1]),
            fmt_f32(payload.position_ned_m[2]),
            fmt_f32(payload.velocity_ned_mps[0]),
            fmt_f32(payload.velocity_ned_mps[1]),
            fmt_f32(payload.velocity_ned_mps[2]),
            fmt_f32(payload.attitude_quat[0]),
            fmt_f32(payload.attitude_quat[1]),
            fmt_f32(payload.attitude_quat[2]),
            fmt_f32(payload.attitude_quat[3])
        ),
        HilinkPayload::SystemState(payload) => format!(
            "tick={} state={} flags={} battery={}V",
            payload.stamp.sim_tick,
            payload.system_state,
            response_flags(payload.flags),
            fmt_f32(payload.battery_voltage_v)
        ),
        HilinkPayload::TelemetrySnapshot(payload) => format!(
            "tick={} state={} flags={} pos=[{}, {}, {}] battery={}V",
            payload.stamp.sim_tick,
            payload.system_state,
            response_flags(payload.flags),
            fmt_f32(payload.position_ned_m[0]),
            fmt_f32(payload.position_ned_m[1]),
            fmt_f32(payload.position_ned_m[2]),
            fmt_f32(payload.battery_voltage_v)
        ),
        HilinkPayload::BenchEnable(payload) => format!(
            "magic={} timeout={}ms",
            flag_bits(payload.magic),
            payload.timeout_ms
        ),
        HilinkPayload::MotorTest(payload) => format!(
            "mask={} mode={} value={} duration={}ms ramp={}ms",
            motor_mask_label(payload.motor_mask),
            motor_test_mode_label(payload.mode),
            payload.value,
            payload.duration_ms,
            payload.ramp_ms
        ),
        HilinkPayload::MotorSweep(payload) => format!(
            "mask={} mode={} start={} end={} step={} step_ms={} zero_ms={} repeat={}",
            motor_mask_label(payload.motor_mask),
            motor_test_mode_label(payload.mode),
            payload.start_value,
            payload.end_value,
            payload.step_value,
            payload.step_duration_ms,
            payload.zero_between_ms,
            payload.repeat_count
        ),
        HilinkPayload::DshotCommand(payload) => format!(
            "mask={} command={} repeat={}",
            motor_mask_label(payload.motor_mask),
            payload.command,
            payload.repeat_count
        ),
        HilinkPayload::MixerMotorOrder(payload) => format!(
            "m1->out{} m2->out{} m3->out{} m4->out{}",
            payload.output_for_motor[0],
            payload.output_for_motor[1],
            payload.output_for_motor[2],
            payload.output_for_motor[3]
        ),
        HilinkPayload::ActuatorStatus(payload) => format!(
            "armed={} bench={} active={} mode={} dshot=[{}, {}, {}, {}] age={}ms timeout={}ms order=[{}, {}, {}, {}] flags={}",
            payload.armed,
            payload.bench_enabled,
            motor_mask_label(payload.active_motor_mask),
            motor_test_mode_label(payload.mode),
            payload.commanded_dshot[0],
            payload.commanded_dshot[1],
            payload.commanded_dshot[2],
            payload.commanded_dshot[3],
            payload.last_command_age_ms,
            payload.bench_timeout_ms,
            payload.mixer_motor_order[0],
            payload.mixer_motor_order[1],
            payload.mixer_motor_order[2],
            payload.mixer_motor_order[3],
            actuator_flags(payload.flags)
        ),
        HilinkPayload::Raw(bytes) => format!("raw_payload={} bytes", bytes.len()),
    };

    format!(
        "{direction} {} seq={} send={}ms len={} flags={} {}",
        header.msg_type.label(),
        header.seq,
        header.send_time_ms,
        header.payload_len,
        flag_bits(header.flags),
        details
    )
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameDirection {
    Rx,
    #[allow(dead_code)]
    Tx,
}

impl FrameDirection {
    fn label(self) -> &'static str {
        match self {
            Self::Rx => "RX",
            Self::Tx => "TX",
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct HilinkParserService {
    rx_frame: Vec<u8>,
    #[allow(dead_code)]
    tx_frame: Vec<u8>,
    telemetry: HilinkTelemetryState,
    pending_actuator_commands: Vec<HilinkActuatorCommand>,
    pending_events: Vec<HilinkEvent>,
}

impl HilinkParserService {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push_rx_bytes(&mut self, bytes: &[u8]) {
        Self::push_bytes(
            bytes,
            FrameDirection::Rx,
            &mut self.rx_frame,
            &mut self.telemetry,
            &mut self.pending_actuator_commands,
            &mut self.pending_events,
        );
    }

    #[allow(dead_code)]
    pub fn push_tx_bytes(&mut self, bytes: &[u8]) {
        Self::push_bytes(
            bytes,
            FrameDirection::Tx,
            &mut self.tx_frame,
            &mut self.telemetry,
            &mut self.pending_actuator_commands,
            &mut self.pending_events,
        );
    }

    pub fn telemetry(&self) -> HilinkTelemetryState {
        self.telemetry.clone()
    }

    pub fn drain_actuator_commands(&mut self) -> Vec<HilinkActuatorCommand> {
        self.pending_actuator_commands.drain(..).collect()
    }

    pub fn drain_events(&mut self) -> Vec<HilinkEvent> {
        self.pending_events.drain(..).collect()
    }

    fn push_bytes(
        bytes: &[u8],
        direction: FrameDirection,
        frame: &mut Vec<u8>,
        telemetry: &mut HilinkTelemetryState,
        pending_actuator_commands: &mut Vec<HilinkActuatorCommand>,
        pending_events: &mut Vec<HilinkEvent>,
    ) {
        for &byte in bytes {
            frame.push(byte);

            if frame.len() > MAX_FRAME_LEN {
                frame.clear();
                telemetry.record_error("frame exceeded maximum parser length");
                continue;
            }

            if byte != FRAME_DELIMITER {
                continue;
            }

            if frame.len() == 1 {
                frame.clear();
                continue;
            }

            match decode_message(frame, direction) {
                Ok(message) => {
                    if let Some(event) = HilinkEvent::from_message(&message) {
                        pending_events.push(event);
                    }
                    if let Some(command) = telemetry.apply_message(message) {
                        pending_actuator_commands.push(command);
                    }
                }
                Err(error) => telemetry.record_error(error.label()),
            }
            frame.clear();
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct HilinkActuatorCommand {
    pub sim_tick: u64,
    pub sim_time_us: u64,
    pub flags: u32,
    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub attitude_quat: [f32; 4],
    pub motor_cmd: [u16; 4],
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HilinkEvent {
    Pong {
        seq: u16,
    },
    Ack {
        seq: u16,
        acked_seq: u16,
        acked_msg_type: u8,
        status: u8,
    },
    Nack {
        seq: u16,
        rejected_seq: u16,
        rejected_msg_type: u8,
        reason: u8,
    },
    Gps {
        seq: u16,
    },
    TelemetrySnapshot {
        seq: u16,
    },
    HilReady {
        seq: u16,
    },
    Heartbeat {
        seq: u16,
    },
}

impl HilinkEvent {
    fn from_message(message: &HilinkMessage) -> Option<Self> {
        if message.direction != FrameDirection::Rx {
            return None;
        }

        match &message.payload {
            HilinkPayload::Empty if message.header.msg_type == MsgType::Pong => Some(Self::Pong {
                seq: message.header.seq,
            }),
            HilinkPayload::Ack(payload) => Some(Self::Ack {
                seq: message.header.seq,
                acked_seq: payload.acked_seq,
                acked_msg_type: payload.acked_msg_type,
                status: payload.status,
            }),
            HilinkPayload::Nack(payload) => Some(Self::Nack {
                seq: message.header.seq,
                rejected_seq: payload.rejected_seq,
                rejected_msg_type: payload.rejected_msg_type,
                reason: payload.reason,
            }),
            HilinkPayload::Gps(_) => Some(Self::Gps {
                seq: message.header.seq,
            }),
            HilinkPayload::TelemetrySnapshot(_) => Some(Self::TelemetrySnapshot {
                seq: message.header.seq,
            }),
            HilinkPayload::Heartbeat(_) => Some(Self::Heartbeat {
                seq: message.header.seq,
            }),
            HilinkPayload::Empty if message.header.msg_type == MsgType::HilReady => {
                Some(Self::HilReady {
                    seq: message.header.seq,
                })
            }
            _ => None,
        }
    }
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct HilinkTelemetryState {
    pub fields: Vec<HilinkTelemetryField>,
    pub rx_frames: u64,
    pub tx_frames: u64,
    pub parse_errors: u64,
    pub last_error: Option<String>,
    pub last_message: Option<String>,
}

impl HilinkTelemetryState {
    fn apply_message(&mut self, message: HilinkMessage) -> Option<HilinkActuatorCommand> {
        match message.direction {
            FrameDirection::Rx => self.rx_frames += 1,
            FrameDirection::Tx => self.tx_frames += 1,
        }

        self.last_error = None;
        self.last_message = Some(format!(
            "{} {} seq {}",
            message.direction.label(),
            message.header.msg_type.label(),
            message.header.seq
        ));

        self.upsert("Protocol", "LAST_DIRECTION", message.direction.label(), "");
        self.upsert(
            "Protocol",
            "LAST_MSG_TYPE",
            message.header.msg_type.label(),
            "",
        );
        self.upsert("Protocol", "LAST_VERSION", message.header.version, "");
        self.upsert(
            "Protocol",
            "LAST_HEADER_FLAGS",
            flag_bits(message.header.flags),
            "",
        );
        self.upsert("Protocol", "LAST_SEQ", message.header.seq, "");
        self.upsert(
            "Protocol",
            "LAST_SEND_TIME_MS",
            message.header.send_time_ms,
            "ms",
        );
        self.upsert(
            "Protocol",
            "LAST_PAYLOAD_LEN",
            message.header.payload_len,
            "bytes",
        );
        self.upsert("Protocol", "RX_FRAMES", self.rx_frames, "frames");
        self.upsert("Protocol", "TX_FRAMES", self.tx_frames, "frames");
        self.upsert("Protocol", "PARSE_ERRORS", self.parse_errors, "errors");

        let mut actuator_command = None;
        match message.payload {
            HilinkPayload::Empty => match message.header.msg_type {
                MsgType::BenchDisable => {
                    self.upsert("Commands", "BENCH_DISABLED", "true", "");
                }
                MsgType::MotorStop => {
                    self.upsert("Commands", "MOTOR_STOP", "true", "");
                }
                MsgType::ActuatorStatusRequest => {
                    self.upsert("Commands", "ACTUATOR_STATUS_REQUEST", "sent", "");
                }
                _ => {}
            },
            HilinkPayload::Ack(payload) => {
                self.upsert("Commands", "ACKED_SEQ", payload.acked_seq, "");
                self.upsert(
                    "Commands",
                    "ACKED_MSG_TYPE",
                    msg_type_label(payload.acked_msg_type),
                    "",
                );
                self.upsert("Commands", "ACK_STATUS", payload.status, "");
            }
            HilinkPayload::Nack(payload) => {
                self.upsert("Commands", "REJECTED_SEQ", payload.rejected_seq, "");
                self.upsert(
                    "Commands",
                    "REJECTED_MSG_TYPE",
                    msg_type_label(payload.rejected_msg_type),
                    "",
                );
                self.upsert("Commands", "NACK_REASON", payload.reason, "");
            }
            HilinkPayload::Imu(payload) => {
                self.apply_stamp(payload.stamp);
                self.apply_vec3("Sensors", "IMU_ACCEL", payload.accel_mps2, "m/s^2");
                self.apply_vec3("Sensors", "IMU_GYRO", payload.gyro_rps, "rad/s");
            }
            HilinkPayload::Mag(payload) => {
                self.apply_stamp(payload.stamp);
                self.apply_vec3("Sensors", "MAG_FIELD", payload.mag_ut, "uT");
            }
            HilinkPayload::Baro(payload) => {
                self.apply_stamp(payload.stamp);
                self.upsert(
                    "Sensors",
                    "BARO_PRESSURE",
                    fmt_f32(payload.pressure_pa),
                    "Pa",
                );
                self.upsert("Sensors", "BARO_ALTITUDE", fmt_f32(payload.altitude_m), "m");
                self.upsert(
                    "Sensors",
                    "TEMPERATURE",
                    fmt_f32(payload.temperature_c),
                    "C",
                );
            }
            HilinkPayload::Gps(payload) => {
                self.apply_stamp(payload.stamp);
                self.apply_gps(
                    payload.lat_deg,
                    payload.lon_deg,
                    payload.alt_msl_m,
                    payload.vel_ned_mps,
                    payload.sats,
                    payload.fix_type,
                );
            }
            HilinkPayload::EstimatorState(payload) => {
                self.apply_stamp(payload.stamp);
                self.apply_vec3("Navigation", "POSITION_NED", payload.position_ned_m, "m");
                self.apply_vec3(
                    "Navigation",
                    "VELOCITY_NED",
                    payload.velocity_ned_mps,
                    "m/s",
                );
                self.apply_quat(payload.attitude_quat);
                self.apply_vec3("System", "GYRO_BIAS", payload.gyro_bias, "rad/s");
                self.apply_vec3("System", "ACCEL_BIAS", payload.accel_bias, "m/s^2");
            }
            HilinkPayload::MotorState(payload) => {
                self.apply_stamp(payload.stamp);
                self.apply_motors(payload.motor_cmd);
            }
            HilinkPayload::HilSensorFrame(payload) => {
                self.apply_stamp(payload.stamp);
                self.upsert(
                    "Protocol",
                    "SENSOR_VALID_FLAGS",
                    valid_flags(payload.valid_flags),
                    "",
                );
                self.apply_vec3("Sensors", "IMU_ACCEL", payload.accel_mps2, "m/s^2");
                self.apply_vec3("Sensors", "IMU_GYRO", payload.gyro_rps, "rad/s");
                self.apply_vec3("Sensors", "MAG_FIELD", payload.mag_ut, "uT");
                self.upsert(
                    "Sensors",
                    "BARO_PRESSURE",
                    fmt_f32(payload.pressure_pa),
                    "Pa",
                );
                self.upsert(
                    "Sensors",
                    "BARO_ALTITUDE",
                    fmt_f32(payload.baro_altitude_m),
                    "m",
                );
                self.upsert(
                    "Sensors",
                    "TEMPERATURE",
                    fmt_f32(payload.temperature_c),
                    "C",
                );
                self.apply_gps(
                    payload.lat_deg,
                    payload.lon_deg,
                    payload.alt_msl_m,
                    payload.vel_ned_mps,
                    payload.sats,
                    payload.fix_type,
                );
                self.apply_power_radio(
                    payload.battery_voltage_v,
                    payload.rssi_dbm,
                    payload.snr_db_x100,
                    payload.loss_pct_x100,
                );
            }
            HilinkPayload::HilResponseFrame(payload) => {
                self.apply_stamp(payload.stamp);
                self.apply_system_state(payload.system_state, payload.flags, None);
                self.apply_vec3("Navigation", "POSITION_NED", payload.position_ned_m, "m");
                self.apply_vec3(
                    "Navigation",
                    "VELOCITY_NED",
                    payload.velocity_ned_mps,
                    "m/s",
                );
                self.apply_quat(payload.attitude_quat);
                self.apply_motors(payload.motor_cmd);
                if message.direction == FrameDirection::Rx {
                    actuator_command = Some(HilinkActuatorCommand {
                        sim_tick: payload.stamp.sim_tick,
                        sim_time_us: payload.stamp.sim_time_us,
                        flags: payload.flags,
                        position_ned_m: payload.position_ned_m,
                        velocity_ned_mps: payload.velocity_ned_mps,
                        attitude_quat: payload.attitude_quat,
                        motor_cmd: payload.motor_cmd,
                    });
                }
            }
            HilinkPayload::Heartbeat(payload) => {
                self.apply_stamp(payload.stamp);
                self.apply_system_state(payload.system_state, payload.flags, None);
            }
            HilinkPayload::SystemState(payload) => {
                self.apply_stamp(payload.stamp);
                self.apply_system_state(
                    payload.system_state,
                    payload.flags,
                    Some(payload.battery_voltage_v),
                );
            }
            HilinkPayload::TelemetrySnapshot(payload) => {
                self.apply_stamp(payload.stamp);
                self.apply_system_state(
                    payload.system_state,
                    payload.flags,
                    Some(payload.battery_voltage_v),
                );
                self.apply_vec3("Navigation", "POSITION_NED", payload.position_ned_m, "m");
                self.apply_vec3(
                    "Navigation",
                    "VELOCITY_NED",
                    payload.velocity_ned_mps,
                    "m/s",
                );
                self.apply_quat(payload.attitude_quat);
                self.apply_power_radio(
                    payload.battery_voltage_v,
                    payload.rssi_dbm,
                    payload.snr_db_x100,
                    payload.loss_pct_x100,
                );
            }
            HilinkPayload::GlobalWaypoint(payload) => {
                self.apply_stamp(payload.ref_stamp);
                self.upsert("Commands", "WAYPOINT_LAT", fmt_f64(payload.lat_deg), "deg");
                self.upsert("Commands", "WAYPOINT_LON", fmt_f64(payload.lon_deg), "deg");
                self.upsert(
                    "Commands",
                    "WAYPOINT_ALT_MSL",
                    fmt_f32(payload.alt_msl_m),
                    "m",
                );
                self.upsert("Commands", "WAYPOINT_YAW", fmt_f32(payload.yaw_deg), "deg");
            }
            HilinkPayload::CvWaypoint(payload) => {
                self.apply_stamp(payload.ref_stamp);
                self.apply_vec3("Commands", "CV_DIR_BODY", payload.dir_body, "");
                self.upsert("Commands", "CV_CONFIDENCE", fmt_f32(payload.confidence), "");
            }
            HilinkPayload::TofWaypoint(payload) => {
                self.apply_stamp(payload.ref_stamp);
                self.upsert("Commands", "TOF_DISTANCE", fmt_f32(payload.distance_m), "m");
                self.upsert(
                    "Commands",
                    "TOF_BEARING",
                    fmt_f32(payload.bearing_deg),
                    "deg",
                );
                self.upsert(
                    "Commands",
                    "TOF_ELEVATION",
                    fmt_f32(payload.elevation_deg),
                    "deg",
                );
            }
            HilinkPayload::BenchEnable(payload) => {
                self.upsert("Commands", "BENCH_MAGIC", flag_bits(payload.magic), "");
                self.upsert("Commands", "BENCH_TIMEOUT_MS", payload.timeout_ms, "ms");
            }
            HilinkPayload::MotorTest(payload) => {
                self.upsert(
                    "Commands",
                    "BENCH_MOTOR_MASK",
                    motor_mask_label(payload.motor_mask),
                    "",
                );
                self.upsert(
                    "Commands",
                    "BENCH_MOTOR_MODE",
                    motor_test_mode_label(payload.mode),
                    "",
                );
                self.upsert("Commands", "BENCH_MOTOR_VALUE", payload.value, "dshot");
                self.upsert(
                    "Commands",
                    "BENCH_MOTOR_DURATION_MS",
                    payload.duration_ms,
                    "ms",
                );
                self.upsert("Commands", "BENCH_MOTOR_RAMP_MS", payload.ramp_ms, "ms");
            }
            HilinkPayload::MotorSweep(payload) => {
                self.upsert(
                    "Commands",
                    "BENCH_SWEEP_MASK",
                    motor_mask_label(payload.motor_mask),
                    "",
                );
                self.upsert(
                    "Commands",
                    "BENCH_SWEEP_MODE",
                    motor_test_mode_label(payload.mode),
                    "",
                );
                self.upsert(
                    "Commands",
                    "BENCH_SWEEP_RANGE",
                    format!("{}->{}", payload.start_value, payload.end_value),
                    "dshot",
                );
                self.upsert("Commands", "BENCH_SWEEP_STEP", payload.step_value, "dshot");
                self.upsert(
                    "Commands",
                    "BENCH_SWEEP_STEP_MS",
                    payload.step_duration_ms,
                    "ms",
                );
                self.upsert(
                    "Commands",
                    "BENCH_SWEEP_ZERO_MS",
                    payload.zero_between_ms,
                    "ms",
                );
                self.upsert(
                    "Commands",
                    "BENCH_SWEEP_REPEAT",
                    payload.repeat_count,
                    "count",
                );
            }
            HilinkPayload::DshotCommand(payload) => {
                self.upsert(
                    "Commands",
                    "BENCH_DSHOT_MASK",
                    motor_mask_label(payload.motor_mask),
                    "",
                );
                self.upsert("Commands", "BENCH_DSHOT_COMMAND", payload.command, "code");
                self.upsert(
                    "Commands",
                    "BENCH_DSHOT_REPEAT",
                    payload.repeat_count,
                    "count",
                );
            }
            HilinkPayload::MixerMotorOrder(payload) => {
                self.apply_mixer_motor_order(payload.output_for_motor);
            }
            HilinkPayload::ActuatorStatus(payload) => {
                self.upsert("Actuators", "BENCH_ARMED", payload.armed, "bool");
                self.upsert("Actuators", "BENCH_ENABLED", payload.bench_enabled, "bool");
                self.upsert(
                    "Actuators",
                    "ACTIVE_MOTOR_MASK",
                    motor_mask_label(payload.active_motor_mask),
                    "",
                );
                self.upsert(
                    "Actuators",
                    "BENCH_MODE",
                    motor_test_mode_label(payload.mode),
                    "",
                );
                for (index, value) in payload.commanded_dshot.into_iter().enumerate() {
                    self.upsert(
                        "Actuators",
                        format!("COMMAND_DSHOT_{}", index + 1),
                        value,
                        "dshot",
                    );
                }
                self.upsert(
                    "Actuators",
                    "LAST_COMMAND_AGE_MS",
                    payload.last_command_age_ms,
                    "ms",
                );
                self.upsert(
                    "Actuators",
                    "BENCH_TIMEOUT_MS",
                    payload.bench_timeout_ms,
                    "ms",
                );
                self.apply_mixer_motor_order(payload.mixer_motor_order);
                self.upsert(
                    "Actuators",
                    "ACTUATOR_FLAGS",
                    actuator_flags(payload.flags),
                    "",
                );
            }
            HilinkPayload::Raw(bytes) => {
                self.upsert("Protocol", "LAST_RAW_PAYLOAD_BYTES", bytes.len(), "bytes");
            }
        }

        actuator_command
    }

    fn record_error(&mut self, error: &str) {
        self.parse_errors += 1;
        self.last_error = Some(error.to_string());
        self.upsert("Protocol", "PARSE_ERRORS", self.parse_errors, "errors");
        self.upsert("Protocol", "LAST_ERROR", error, "");
    }

    fn apply_stamp(&mut self, stamp: SimStamp) {
        self.upsert("Timing", "SIM_TICK", stamp.sim_tick, "tick");
        self.upsert("Timing", "SIM_TIME", stamp.sim_time_us, "us");
    }

    fn apply_vec3(
        &mut self,
        group: &'static str,
        prefix: &str,
        values: [f32; 3],
        unit: &'static str,
    ) {
        for (axis, value) in ["X", "Y", "Z"].into_iter().zip(values) {
            self.upsert(group, format!("{prefix}_{axis}"), fmt_f32(value), unit);
        }
    }

    fn apply_quat(&mut self, attitude_quat: [f32; 4]) {
        for (axis, value) in ["W", "X", "Y", "Z"].into_iter().zip(attitude_quat) {
            self.upsert(
                "Navigation",
                format!("ATTITUDE_QUAT_{axis}"),
                fmt_f32(value),
                "",
            );
        }
    }

    fn apply_motors(&mut self, motor_cmd: [u16; 4]) {
        for (index, value) in motor_cmd.into_iter().enumerate() {
            self.upsert(
                "Actuators",
                format!("MOTOR_CMD_{}", index + 1),
                value,
                "normalized",
            );
        }
    }

    fn apply_mixer_motor_order(&mut self, output_for_motor: [u8; 4]) {
        self.upsert(
            "Actuators",
            "MIXER_MOTOR_ORDER",
            format!(
                "M1->O{} M2->O{} M3->O{} M4->O{}",
                output_for_motor[0], output_for_motor[1], output_for_motor[2], output_for_motor[3]
            ),
            "",
        );
        for (index, output) in output_for_motor.into_iter().enumerate() {
            self.upsert(
                "Actuators",
                format!("MIXER_MOTOR_{}_OUTPUT", index + 1),
                output,
                "output",
            );
        }
    }

    fn apply_gps(
        &mut self,
        lat_deg: f64,
        lon_deg: f64,
        alt_msl_m: f32,
        vel_ned_mps: [f32; 3],
        sats: u8,
        fix_type: u8,
    ) {
        self.upsert("Navigation", "GPS_LAT", fmt_f64(lat_deg), "deg");
        self.upsert("Navigation", "GPS_LON", fmt_f64(lon_deg), "deg");
        self.upsert("Navigation", "GPS_ALT_MSL", fmt_f32(alt_msl_m), "m");
        self.apply_vec3("Navigation", "GPS_VEL_NED", vel_ned_mps, "m/s");
        self.upsert("Navigation", "GPS_SAT_COUNT", sats, "count");
        self.upsert("Navigation", "GPS_FIX_TYPE", sats_label(fix_type), "");
    }

    fn apply_power_radio(
        &mut self,
        battery_voltage_v: f32,
        rssi_dbm: i16,
        snr_db_x100: i16,
        loss_pct_x100: u16,
    ) {
        self.upsert("System", "BAT_VOLTAGE", fmt_f32(battery_voltage_v), "V");
        self.upsert("System", "RADIO_RSSI", rssi_dbm, "dBm");
        self.upsert("System", "RADIO_SNR", fmt_x100(snr_db_x100), "dB");
        self.upsert("System", "RADIO_LOSS", fmt_x100(loss_pct_x100), "%");
    }

    fn apply_system_state(&mut self, system_state: u8, flags: u32, battery_voltage_v: Option<f32>) {
        self.upsert("System", "SYS_STATE", system_state, "state");
        self.upsert("System", "RESPONSE_FLAGS", response_flags(flags), "");
        if let Some(battery_voltage_v) = battery_voltage_v {
            self.upsert("System", "BAT_VOLTAGE", fmt_f32(battery_voltage_v), "V");
        }
    }

    fn upsert(
        &mut self,
        group: &'static str,
        parameter: impl Into<String>,
        value: impl ToString,
        unit: &'static str,
    ) {
        let parameter = parameter.into();
        let value = value.to_string();

        if let Some(field) = self
            .fields
            .iter_mut()
            .find(|field| field.group == group && field.parameter == parameter)
        {
            field.value = value;
            field.unit = unit;
            return;
        }

        self.fields.push(HilinkTelemetryField {
            group,
            parameter,
            value,
            unit,
        });
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct HilinkTelemetryField {
    pub group: &'static str,
    pub parameter: String,
    pub value: String,
    pub unit: &'static str,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Header {
    version: u8,
    msg_type: MsgType,
    flags: u8,
    reserved: u8,
    seq: u16,
    send_time_ms: u32,
    payload_len: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MsgType {
    Ping,
    Pong,
    Ack,
    Nack,
    Heartbeat,
    HilSensorFrame,
    HilResponseFrame,
    HilReady,
    Imu,
    Mag,
    Baro,
    Gps,
    Battery,
    SystemState,
    MotorState,
    EstimatorState,
    RadioStatus,
    TelemetrySnapshot,
    Arm,
    Disarm,
    ControlWaypoint,
    CvWaypoint,
    TofWaypoint,
    MissionWaypoint,
    Rtl,
    BenchEnable,
    BenchDisable,
    MotorTest,
    MotorSweep,
    MotorStop,
    DshotCommand,
    ActuatorStatusRequest,
    ActuatorStatus,
    MixerMotorOrder,
}

impl MsgType {
    fn from_u8(value: u8) -> Result<Self, ParseError> {
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
            68 => Ok(Self::MixerMotorOrder),
            _ => Err(ParseError::UnknownMsgType),
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Ping => "Ping",
            Self::Pong => "Pong",
            Self::Ack => "Ack",
            Self::Nack => "Nack",
            Self::Heartbeat => "Heartbeat",
            Self::HilSensorFrame => "HilSensorFrame",
            Self::HilResponseFrame => "HilResponseFrame",
            Self::HilReady => "HilReady",
            Self::Imu => "Imu",
            Self::Mag => "Mag",
            Self::Baro => "Baro",
            Self::Gps => "Gps",
            Self::Battery => "Battery",
            Self::SystemState => "SystemState",
            Self::MotorState => "MotorState",
            Self::EstimatorState => "EstimatorState",
            Self::RadioStatus => "RadioStatus",
            Self::TelemetrySnapshot => "TelemetrySnapshot",
            Self::Arm => "Arm",
            Self::Disarm => "Disarm",
            Self::ControlWaypoint => "ControlWaypoint",
            Self::CvWaypoint => "CvWaypoint",
            Self::TofWaypoint => "TofWaypoint",
            Self::MissionWaypoint => "MissionWaypoint",
            Self::Rtl => "Rtl",
            Self::BenchEnable => "BenchEnable",
            Self::BenchDisable => "BenchDisable",
            Self::MotorTest => "MotorTest",
            Self::MotorSweep => "MotorSweep",
            Self::MotorStop => "MotorStop",
            Self::DshotCommand => "DshotCommand",
            Self::ActuatorStatusRequest => "ActuatorStatusRequest",
            Self::ActuatorStatus => "ActuatorStatus",
            Self::MixerMotorOrder => "MixerMotorOrder",
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct HilinkMessage {
    direction: FrameDirection,
    header: Header,
    payload: HilinkPayload,
}

#[derive(Clone, Debug, PartialEq)]
enum HilinkPayload {
    Empty,
    Ack(AckPayload),
    Nack(NackPayload),
    Imu(ImuPayload),
    Mag(MagPayload),
    Baro(BaroPayload),
    Gps(GpsPayload),
    EstimatorState(EstimatorStatePayload),
    MotorState(MotorStatePayload),
    HilSensorFrame(HilSensorFramePayload),
    HilResponseFrame(HilResponseFramePayload),
    Heartbeat(HeartbeatPayload),
    SystemState(SystemStatePayload),
    TelemetrySnapshot(TelemetrySnapshotPayload),
    GlobalWaypoint(GlobalWaypointPayload),
    CvWaypoint(CvWaypointPayload),
    TofWaypoint(TofWaypointPayload),
    BenchEnable(BenchEnablePayload),
    MotorTest(MotorTestPayload),
    MotorSweep(MotorSweepPayload),
    DshotCommand(DshotCommandPayload),
    MixerMotorOrder(MixerMotorOrderPayload),
    ActuatorStatus(ActuatorStatusPayload),
    Raw(Vec<u8>),
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct SimStamp {
    sim_tick: u64,
    sim_time_us: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct AckPayload {
    acked_seq: u16,
    acked_msg_type: u8,
    status: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct NackPayload {
    rejected_seq: u16,
    rejected_msg_type: u8,
    reason: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ImuPayload {
    stamp: SimStamp,
    accel_mps2: [f32; 3],
    gyro_rps: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct MagPayload {
    stamp: SimStamp,
    mag_ut: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct BaroPayload {
    stamp: SimStamp,
    pressure_pa: f32,
    altitude_m: f32,
    temperature_c: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct GpsPayload {
    stamp: SimStamp,
    lat_deg: f64,
    lon_deg: f64,
    alt_msl_m: f32,
    vel_ned_mps: [f32; 3],
    sats: u8,
    fix_type: u8,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct EstimatorStatePayload {
    stamp: SimStamp,
    position_ned_m: [f32; 3],
    velocity_ned_mps: [f32; 3],
    attitude_quat: [f32; 4],
    gyro_bias: [f32; 3],
    accel_bias: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct MotorStatePayload {
    stamp: SimStamp,
    motor_cmd: [u16; 4],
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct HilSensorFramePayload {
    stamp: SimStamp,
    valid_flags: u32,
    accel_mps2: [f32; 3],
    gyro_rps: [f32; 3],
    mag_ut: [f32; 3],
    pressure_pa: f32,
    baro_altitude_m: f32,
    temperature_c: f32,
    lat_deg: f64,
    lon_deg: f64,
    alt_msl_m: f32,
    vel_ned_mps: [f32; 3],
    sats: u8,
    fix_type: u8,
    battery_voltage_v: f32,
    rssi_dbm: i16,
    snr_db_x100: i16,
    loss_pct_x100: u16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct HilResponseFramePayload {
    stamp: SimStamp,
    system_state: u8,
    flags: u32,
    position_ned_m: [f32; 3],
    velocity_ned_mps: [f32; 3],
    attitude_quat: [f32; 4],
    motor_cmd: [u16; 4],
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct HeartbeatPayload {
    stamp: SimStamp,
    system_state: u8,
    flags: u32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct SystemStatePayload {
    stamp: SimStamp,
    system_state: u8,
    flags: u32,
    battery_voltage_v: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct TelemetrySnapshotPayload {
    stamp: SimStamp,
    system_state: u8,
    flags: u32,
    position_ned_m: [f32; 3],
    velocity_ned_mps: [f32; 3],
    attitude_quat: [f32; 4],
    battery_voltage_v: f32,
    rssi_dbm: i16,
    snr_db_x100: i16,
    loss_pct_x100: u16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct GlobalWaypointPayload {
    ref_stamp: SimStamp,
    lat_deg: f64,
    lon_deg: f64,
    alt_msl_m: f32,
    yaw_deg: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct CvWaypointPayload {
    ref_stamp: SimStamp,
    dir_body: [f32; 3],
    confidence: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct TofWaypointPayload {
    ref_stamp: SimStamp,
    distance_m: f32,
    bearing_deg: f32,
    elevation_deg: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct BenchEnablePayload {
    magic: u32,
    timeout_ms: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MotorTestPayload {
    motor_mask: u8,
    mode: u8,
    value: u16,
    duration_ms: u16,
    ramp_ms: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MotorSweepPayload {
    motor_mask: u8,
    mode: u8,
    start_value: u16,
    end_value: u16,
    step_value: u16,
    step_duration_ms: u16,
    zero_between_ms: u16,
    repeat_count: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct DshotCommandPayload {
    motor_mask: u8,
    command: u8,
    repeat_count: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MixerMotorOrderPayload {
    output_for_motor: [u8; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ActuatorStatusPayload {
    armed: u8,
    bench_enabled: u8,
    active_motor_mask: u8,
    mode: u8,
    commanded_dshot: [u16; 4],
    last_command_age_ms: u16,
    bench_timeout_ms: u16,
    mixer_motor_order: [u8; 4],
    flags: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ParseError {
    BufferTooSmall,
    HeaderTooShort,
    PayloadLenMismatch,
    BadDelimiter,
    CobsDecodeZero,
    CobsDecodeOverrun,
    CrcMismatch,
    UnknownMsgType,
    InvalidPayloadLength,
    UnsupportedProtocolVersion,
    NonZeroReserved,
}

impl ParseError {
    fn label(self) -> &'static str {
        match self {
            Self::BufferTooSmall => "parser buffer too small",
            Self::HeaderTooShort => "decoded frame shorter than HILink header",
            Self::PayloadLenMismatch => "header payload length did not match decoded body length",
            Self::BadDelimiter => "frame did not end with the HILink delimiter",
            Self::CobsDecodeZero => "invalid zero byte inside COBS frame",
            Self::CobsDecodeOverrun => "COBS code overran frame input",
            Self::CrcMismatch => "CRC16-CCITT-FALSE mismatch",
            Self::UnknownMsgType => "unknown HILink message type",
            Self::InvalidPayloadLength => "payload length was invalid for message type",
            Self::UnsupportedProtocolVersion => "unsupported HILink protocol version",
            Self::NonZeroReserved => "reserved HILink header byte was nonzero",
        }
    }
}

fn decode_message(input: &[u8], direction: FrameDirection) -> Result<HilinkMessage, ParseError> {
    if input.last() != Some(&FRAME_DELIMITER) {
        return Err(ParseError::BadDelimiter);
    }

    let mut raw = [0u8; MAX_FRAME_LEN];
    let raw_len = cobs_decode(&input[..input.len() - 1], &mut raw)?;
    if raw_len < HEADER_LEN + CRC_LEN {
        return Err(ParseError::HeaderTooShort);
    }

    let packet_len = raw_len - CRC_LEN;
    let stored_crc = u16::from_le_bytes([raw[packet_len], raw[packet_len + 1]]);
    let computed_crc = crc16_ccitt_false(&raw[..packet_len]);
    if stored_crc != computed_crc {
        return Err(ParseError::CrcMismatch);
    }

    let header = decode_header(&raw[..HEADER_LEN])?;
    if header.version != PROTOCOL_VERSION {
        return Err(ParseError::UnsupportedProtocolVersion);
    }
    if header.reserved != 0 {
        return Err(ParseError::NonZeroReserved);
    }

    let payload_end = HEADER_LEN + usize::from(header.payload_len);
    if payload_end != packet_len {
        return Err(ParseError::PayloadLenMismatch);
    }

    let payload = decode_payload(header.msg_type, &raw[HEADER_LEN..payload_end])?;
    Ok(HilinkMessage {
        direction,
        header,
        payload,
    })
}

fn decode_header(input: &[u8]) -> Result<Header, ParseError> {
    let mut reader = Reader::new(input);
    Ok(Header {
        version: reader.u8()?,
        msg_type: MsgType::from_u8(reader.u8()?)?,
        flags: reader.u8()?,
        reserved: reader.u8()?,
        seq: reader.u16()?,
        send_time_ms: reader.u32()?,
        payload_len: reader.u16()?,
    })
}

fn decode_payload(msg_type: MsgType, input: &[u8]) -> Result<HilinkPayload, ParseError> {
    let mut reader = Reader::new(input);
    match msg_type {
        MsgType::Ping
        | MsgType::Pong
        | MsgType::HilReady
        | MsgType::Arm
        | MsgType::Disarm
        | MsgType::Rtl
        | MsgType::BenchDisable
        | MsgType::MotorStop
        | MsgType::ActuatorStatusRequest => {
            expect_len(input, 0)?;
            Ok(HilinkPayload::Empty)
        }
        MsgType::Ack => {
            expect_len(input, 4)?;
            Ok(HilinkPayload::Ack(AckPayload {
                acked_seq: reader.u16()?,
                acked_msg_type: reader.u8()?,
                status: reader.u8()?,
            }))
        }
        MsgType::Nack => {
            expect_len(input, 4)?;
            Ok(HilinkPayload::Nack(NackPayload {
                rejected_seq: reader.u16()?,
                rejected_msg_type: reader.u8()?,
                reason: reader.u8()?,
            }))
        }
        MsgType::Imu => {
            expect_len(input, 40)?;
            Ok(HilinkPayload::Imu(ImuPayload {
                stamp: reader.sim_stamp()?,
                accel_mps2: reader.f32x3()?,
                gyro_rps: reader.f32x3()?,
            }))
        }
        MsgType::Mag => {
            expect_len(input, 28)?;
            Ok(HilinkPayload::Mag(MagPayload {
                stamp: reader.sim_stamp()?,
                mag_ut: reader.f32x3()?,
            }))
        }
        MsgType::Baro => {
            expect_len(input, 28)?;
            Ok(HilinkPayload::Baro(BaroPayload {
                stamp: reader.sim_stamp()?,
                pressure_pa: reader.f32()?,
                altitude_m: reader.f32()?,
                temperature_c: reader.f32()?,
            }))
        }
        MsgType::Gps => {
            expect_len(input, 52)?;
            let payload = GpsPayload {
                stamp: reader.sim_stamp()?,
                lat_deg: reader.f64()?,
                lon_deg: reader.f64()?,
                alt_msl_m: reader.f32()?,
                vel_ned_mps: reader.f32x3()?,
                sats: reader.u8()?,
                fix_type: reader.u8()?,
            };
            reader.bytes::<2>()?;
            Ok(HilinkPayload::Gps(payload))
        }
        MsgType::EstimatorState => {
            expect_len(input, 80)?;
            Ok(HilinkPayload::EstimatorState(EstimatorStatePayload {
                stamp: reader.sim_stamp()?,
                position_ned_m: reader.f32x3()?,
                velocity_ned_mps: reader.f32x3()?,
                attitude_quat: reader.f32x4()?,
                gyro_bias: reader.f32x3()?,
                accel_bias: reader.f32x3()?,
            }))
        }
        MsgType::MotorState => {
            expect_len(input, 24)?;
            Ok(HilinkPayload::MotorState(MotorStatePayload {
                stamp: reader.sim_stamp()?,
                motor_cmd: reader.u16x4()?,
            }))
        }
        MsgType::HilSensorFrame => {
            expect_len(input, 115)?;
            let payload = HilSensorFramePayload {
                stamp: reader.sim_stamp()?,
                valid_flags: reader.u32()?,
                accel_mps2: reader.f32x3()?,
                gyro_rps: reader.f32x3()?,
                mag_ut: reader.f32x3()?,
                pressure_pa: reader.f32()?,
                baro_altitude_m: reader.f32()?,
                temperature_c: reader.f32()?,
                lat_deg: reader.f64()?,
                lon_deg: reader.f64()?,
                alt_msl_m: reader.f32()?,
                vel_ned_mps: reader.f32x3()?,
                sats: reader.u8()?,
                fix_type: reader.u8()?,
                battery_voltage_v: {
                    reader.bytes::<3>()?;
                    reader.f32()?
                },
                rssi_dbm: reader.i16()?,
                snr_db_x100: reader.i16()?,
                loss_pct_x100: reader.u16()?,
            };
            Ok(HilinkPayload::HilSensorFrame(payload))
        }
        MsgType::HilResponseFrame => {
            expect_len(input, 72)?;
            let payload = HilResponseFramePayload {
                stamp: reader.sim_stamp()?,
                system_state: reader.u8()?,
                flags: {
                    reader.bytes::<3>()?;
                    reader.u32()?
                },
                position_ned_m: reader.f32x3()?,
                velocity_ned_mps: reader.f32x3()?,
                attitude_quat: reader.f32x4()?,
                motor_cmd: reader.u16x4()?,
            };
            Ok(HilinkPayload::HilResponseFrame(payload))
        }
        MsgType::Heartbeat => {
            expect_len(input, 24)?;
            let payload = HeartbeatPayload {
                stamp: reader.sim_stamp()?,
                system_state: reader.u8()?,
                flags: {
                    reader.bytes::<3>()?;
                    reader.u32()?
                },
            };
            Ok(HilinkPayload::Heartbeat(payload))
        }
        MsgType::SystemState => {
            expect_len(input, 28)?;
            let payload = SystemStatePayload {
                stamp: reader.sim_stamp()?,
                system_state: reader.u8()?,
                flags: {
                    reader.bytes::<3>()?;
                    reader.u32()?
                },
                battery_voltage_v: reader.f32()?,
            };
            Ok(HilinkPayload::SystemState(payload))
        }
        MsgType::TelemetrySnapshot => {
            expect_len(input, 74)?;
            let payload = TelemetrySnapshotPayload {
                stamp: reader.sim_stamp()?,
                system_state: reader.u8()?,
                flags: {
                    reader.bytes::<3>()?;
                    reader.u32()?
                },
                position_ned_m: reader.f32x3()?,
                velocity_ned_mps: reader.f32x3()?,
                attitude_quat: reader.f32x4()?,
                battery_voltage_v: reader.f32()?,
                rssi_dbm: reader.i16()?,
                snr_db_x100: reader.i16()?,
                loss_pct_x100: reader.u16()?,
            };
            Ok(HilinkPayload::TelemetrySnapshot(payload))
        }
        MsgType::ControlWaypoint | MsgType::MissionWaypoint => {
            expect_len(input, 40)?;
            Ok(HilinkPayload::GlobalWaypoint(GlobalWaypointPayload {
                ref_stamp: reader.sim_stamp()?,
                lat_deg: reader.f64()?,
                lon_deg: reader.f64()?,
                alt_msl_m: reader.f32()?,
                yaw_deg: reader.f32()?,
            }))
        }
        MsgType::CvWaypoint => {
            expect_len(input, 32)?;
            Ok(HilinkPayload::CvWaypoint(CvWaypointPayload {
                ref_stamp: reader.sim_stamp()?,
                dir_body: reader.f32x3()?,
                confidence: reader.f32()?,
            }))
        }
        MsgType::TofWaypoint => {
            expect_len(input, 28)?;
            Ok(HilinkPayload::TofWaypoint(TofWaypointPayload {
                ref_stamp: reader.sim_stamp()?,
                distance_m: reader.f32()?,
                bearing_deg: reader.f32()?,
                elevation_deg: reader.f32()?,
            }))
        }
        MsgType::BenchEnable => {
            expect_len(input, 8)?;
            let payload = BenchEnablePayload {
                magic: reader.u32()?,
                timeout_ms: reader.u16()?,
            };
            reader.bytes::<2>()?;
            Ok(HilinkPayload::BenchEnable(payload))
        }
        MsgType::MotorTest => {
            expect_len(input, 12)?;
            let payload = MotorTestPayload {
                motor_mask: reader.u8()?,
                mode: reader.u8()?,
                value: {
                    reader.bytes::<2>()?;
                    reader.u16()?
                },
                duration_ms: reader.u16()?,
                ramp_ms: reader.u16()?,
            };
            reader.bytes::<2>()?;
            Ok(HilinkPayload::MotorTest(payload))
        }
        MsgType::MotorSweep => {
            expect_len(input, 16)?;
            let payload = MotorSweepPayload {
                motor_mask: reader.u8()?,
                mode: reader.u8()?,
                start_value: {
                    reader.bytes::<2>()?;
                    reader.u16()?
                },
                end_value: reader.u16()?,
                step_value: reader.u16()?,
                step_duration_ms: reader.u16()?,
                zero_between_ms: reader.u16()?,
                repeat_count: reader.u8()?,
            };
            reader.bytes::<1>()?;
            Ok(HilinkPayload::MotorSweep(payload))
        }
        MsgType::DshotCommand => {
            expect_len(input, 4)?;
            let payload = DshotCommandPayload {
                motor_mask: reader.u8()?,
                command: reader.u8()?,
                repeat_count: reader.u8()?,
            };
            reader.bytes::<1>()?;
            Ok(HilinkPayload::DshotCommand(payload))
        }
        MsgType::MixerMotorOrder => {
            expect_len(input, 4)?;
            Ok(HilinkPayload::MixerMotorOrder(MixerMotorOrderPayload {
                output_for_motor: reader.bytes::<4>()?,
            }))
        }
        MsgType::ActuatorStatus => {
            expect_len(input, 24)?;
            Ok(HilinkPayload::ActuatorStatus(ActuatorStatusPayload {
                armed: reader.u8()?,
                bench_enabled: reader.u8()?,
                active_motor_mask: reader.u8()?,
                mode: reader.u8()?,
                commanded_dshot: reader.u16x4()?,
                last_command_age_ms: reader.u16()?,
                bench_timeout_ms: reader.u16()?,
                mixer_motor_order: reader.bytes::<4>()?,
                flags: reader.u32()?,
            }))
        }
        MsgType::Battery | MsgType::RadioStatus => Ok(HilinkPayload::Raw(input.to_vec())),
    }
}

fn encode_packet(
    msg_type: u8,
    flags: u8,
    seq: u16,
    send_time_ms: u32,
    payload: &[u8],
) -> Result<Vec<u8>, ParseError> {
    if payload.len() > u16::MAX as usize {
        return Err(ParseError::InvalidPayloadLength);
    }

    let mut raw = Vec::with_capacity(HEADER_LEN + payload.len() + CRC_LEN);
    raw.push(PROTOCOL_VERSION);
    raw.push(msg_type);
    raw.push(flags);
    raw.push(0);
    raw.extend(seq.to_le_bytes());
    raw.extend(send_time_ms.to_le_bytes());
    raw.extend((payload.len() as u16).to_le_bytes());
    raw.extend(payload);

    let crc = crc16_ccitt_false(&raw);
    raw.extend(crc.to_le_bytes());

    let mut encoded = Vec::with_capacity(raw.len() + (raw.len() / 254) + 2);
    cobs_encode(&raw, &mut encoded);
    encoded.push(FRAME_DELIMITER);
    Ok(encoded)
}

fn cobs_encode(input: &[u8], out: &mut Vec<u8>) {
    out.clear();
    out.push(0);

    let mut code_index = 0;
    let mut code = 1u8;

    for &byte in input {
        if byte == 0 {
            out[code_index] = code;
            code_index = out.len();
            out.push(0);
            code = 1;
        } else {
            out.push(byte);
            code += 1;

            if code == 0xFF {
                out[code_index] = code;
                code_index = out.len();
                out.push(0);
                code = 1;
            }
        }
    }

    out[code_index] = code;
}

fn cobs_decode(input: &[u8], out: &mut [u8]) -> Result<usize, ParseError> {
    let mut read_index = 0;
    let mut write_index = 0;

    while read_index < input.len() {
        let code = input[read_index];
        if code == 0 {
            return Err(ParseError::CobsDecodeZero);
        }
        read_index += 1;

        let copy_len = usize::from(code - 1);
        if read_index + copy_len > input.len() {
            return Err(ParseError::CobsDecodeOverrun);
        }
        if write_index + copy_len > out.len() {
            return Err(ParseError::BufferTooSmall);
        }

        out[write_index..write_index + copy_len]
            .copy_from_slice(&input[read_index..read_index + copy_len]);
        write_index += copy_len;
        read_index += copy_len;

        if code != 0xFF && read_index < input.len() {
            if write_index >= out.len() {
                return Err(ParseError::BufferTooSmall);
            }
            out[write_index] = 0;
            write_index += 1;
        }
    }

    Ok(write_index)
}

fn push_stamp(out: &mut Vec<u8>, sim_tick: u64, sim_time_us: u64) {
    out.extend(sim_tick.to_le_bytes());
    out.extend(sim_time_us.to_le_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend(value.to_le_bytes());
}

fn push_f32(out: &mut Vec<u8>, value: f32) {
    out.extend(value.to_bits().to_le_bytes());
}

fn push_f64(out: &mut Vec<u8>, value: f64) {
    out.extend(value.to_bits().to_le_bytes());
}

fn push_f32x3(out: &mut Vec<u8>, values: [f32; 3]) {
    for value in values {
        push_f32(out, value);
    }
}

fn crc16_ccitt_false(bytes: &[u8]) -> u16 {
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

fn expect_len(input: &[u8], expected: usize) -> Result<(), ParseError> {
    if input.len() == expected {
        Ok(())
    } else {
        Err(ParseError::InvalidPayloadLength)
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

    fn bytes<const N: usize>(&mut self) -> Result<[u8; N], ParseError> {
        if self.pos + N > self.input.len() {
            return Err(ParseError::InvalidPayloadLength);
        }
        let mut out = [0u8; N];
        out.copy_from_slice(&self.input[self.pos..self.pos + N]);
        self.pos += N;
        Ok(out)
    }

    fn u8(&mut self) -> Result<u8, ParseError> {
        Ok(self.bytes::<1>()?[0])
    }

    fn u16(&mut self) -> Result<u16, ParseError> {
        Ok(u16::from_le_bytes(self.bytes::<2>()?))
    }

    fn i16(&mut self) -> Result<i16, ParseError> {
        Ok(i16::from_le_bytes(self.bytes::<2>()?))
    }

    fn u32(&mut self) -> Result<u32, ParseError> {
        Ok(u32::from_le_bytes(self.bytes::<4>()?))
    }

    fn u64(&mut self) -> Result<u64, ParseError> {
        Ok(u64::from_le_bytes(self.bytes::<8>()?))
    }

    fn f32(&mut self) -> Result<f32, ParseError> {
        Ok(f32::from_bits(self.u32()?))
    }

    fn f64(&mut self) -> Result<f64, ParseError> {
        Ok(f64::from_bits(self.u64()?))
    }

    fn sim_stamp(&mut self) -> Result<SimStamp, ParseError> {
        Ok(SimStamp {
            sim_tick: self.u64()?,
            sim_time_us: self.u64()?,
        })
    }

    fn f32x3(&mut self) -> Result<[f32; 3], ParseError> {
        Ok([self.f32()?, self.f32()?, self.f32()?])
    }

    fn f32x4(&mut self) -> Result<[f32; 4], ParseError> {
        Ok([self.f32()?, self.f32()?, self.f32()?, self.f32()?])
    }

    fn u16x4(&mut self) -> Result<[u16; 4], ParseError> {
        Ok([self.u16()?, self.u16()?, self.u16()?, self.u16()?])
    }
}

fn msg_type_label(value: u8) -> String {
    MsgType::from_u8(value)
        .map(|msg_type| msg_type.label().to_string())
        .unwrap_or_else(|_| format!("Unknown({value})"))
}

fn fmt_f32(value: f32) -> String {
    if value.is_finite() {
        format!("{value:.3}")
    } else {
        "invalid".to_string()
    }
}

fn fmt_f64(value: f64) -> String {
    if value.is_finite() {
        format!("{value:.7}")
    } else {
        "invalid".to_string()
    }
}

fn fmt_x100(value: impl Into<i64>) -> String {
    let value = value.into();
    format!("{:.2}", value as f64 / 100.0)
}

fn flag_bits<T>(value: T) -> String
where
    T: Into<u64>,
{
    format!("0x{:X}", value.into())
}

fn labels_from_flags(value: u32, labels: &[(u32, &str)]) -> String {
    let names: Vec<&str> = labels
        .iter()
        .filter_map(|(flag, label)| ((value & *flag) != 0).then_some(*label))
        .collect();

    if names.is_empty() {
        "none".to_string()
    } else {
        names.join("|")
    }
}

fn valid_flags(value: u32) -> String {
    labels_from_flags(
        value,
        &[
            (1 << 0, "ACCEL"),
            (1 << 1, "GYRO"),
            (1 << 2, "MAG"),
            (1 << 3, "BARO"),
            (1 << 4, "GPS"),
            (1 << 5, "BATTERY"),
            (1 << 6, "RADIO"),
        ],
    )
}

fn response_flags(value: u32) -> String {
    labels_from_flags(
        value,
        &[
            (1 << 0, "ARMED"),
            (1 << 1, "FAILSAFE"),
            (1 << 2, "ESTIMATOR_VALID"),
            (1 << 3, "MOTORS_VALID"),
            (1 << 4, "CONTROL_VALID"),
            (1 << 5, "CONTROL_CLAMPED"),
            (1 << 6, "SENSOR_INPUT_VALID"),
            (1 << 7, "BENCH_OVERRIDE"),
        ],
    )
}

fn motor_test_mode_label(value: u8) -> &'static str {
    match value {
        MOTOR_TEST_MODE_STOP => "STOP",
        MOTOR_TEST_MODE_RAW_DSHOT => "RAW_DSHOT",
        MOTOR_TEST_MODE_NORMALIZED => "NORMALIZED",
        _ => "UNKNOWN",
    }
}

fn motor_mask_label(value: u8) -> String {
    if value == 0 {
        return "none".to_string();
    }
    if value == MOTOR_MASK_ALL {
        return "ALL".to_string();
    }

    let mut labels = Vec::new();
    if (value & MOTOR_MASK_M1) != 0 {
        labels.push("M1");
    }
    if (value & MOTOR_MASK_M2) != 0 {
        labels.push("M2");
    }
    if (value & MOTOR_MASK_M3) != 0 {
        labels.push("M3");
    }
    if (value & MOTOR_MASK_M4) != 0 {
        labels.push("M4");
    }

    let known_mask = MOTOR_MASK_ALL;
    let unknown = value & !known_mask;
    if unknown != 0 {
        labels.push("EXTRA");
    }

    labels.join("|")
}

fn actuator_flags(value: u32) -> String {
    labels_from_flags(
        value,
        &[
            (1 << 0, "OUTPUT_ACTIVE"),
            (1 << 1, "COMMAND_TIMEOUT"),
            (1 << 2, "CLAMPED"),
            (1 << 3, "REJECTED_WHILE_DISARMED"),
            (1 << 4, "BENCH_MODE_ENABLED"),
        ],
    )
}

fn sats_label(fix_type: u8) -> String {
    match fix_type {
        0 => "0 no fix".to_string(),
        1 => "1 dead reckoning".to_string(),
        2 => "2 2D fix".to_string(),
        3 => "3 3D fix".to_string(),
        4 => "4 GNSS + dead reckoning".to_string(),
        5 => "5 time only".to_string(),
        _ => fix_type.to_string(),
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
    fn parses_hil_sensor_frame_across_uart_chunks() {
        let mut payload = Vec::new();
        push_stamp(&mut payload, 15_234, 76_170_000);
        push_u32(&mut payload, (1 << 0) | (1 << 1) | (1 << 4));
        push_f32x3(&mut payload, [1.0, 2.0, 3.0]);
        push_f32x3(&mut payload, [0.1, 0.2, 0.3]);
        push_f32x3(&mut payload, [10.0, 11.0, 12.0]);
        push_f32(&mut payload, 101_325.0);
        push_f32(&mut payload, 12.5);
        push_f32(&mut payload, 25.0);
        push_f64(&mut payload, 30.2672);
        push_f64(&mut payload, -97.7431);
        push_f32(&mut payload, 171.0);
        push_f32x3(&mut payload, [4.0, 5.0, -0.5]);
        payload.push(12);
        payload.push(3);
        payload.extend([0, 0, 0]);
        push_f32(&mut payload, 16.2);
        payload.extend((-48i16).to_le_bytes());
        payload.extend(1150i16.to_le_bytes());
        payload.extend(25u16.to_le_bytes());

        let encoded = encode_test_packet(10, 7, 1234, &payload);
        let split = encoded.len() / 2;
        let mut parser = HilinkParserService::new();
        parser.push_rx_bytes(&encoded[..split]);
        parser.push_rx_bytes(&encoded[split..]);

        let telemetry = parser.telemetry();
        assert_eq!(telemetry.rx_frames, 1);
        assert_field(&telemetry, "Protocol", "LAST_MSG_TYPE", "HilSensorFrame");
        assert_field(&telemetry, "Timing", "SIM_TICK", "15234");
        assert_field(&telemetry, "Navigation", "GPS_LAT", "30.2672000");
        assert_field(&telemetry, "System", "RADIO_SNR", "11.50");
    }

    #[test]
    fn parses_gps_frame_from_uart() {
        let mut payload = Vec::new();
        push_stamp(&mut payload, 15_235, 76_175_000);
        push_f64(&mut payload, 30.2672345);
        push_f64(&mut payload, -97.7431567);
        push_f32(&mut payload, 172.5);
        push_f32x3(&mut payload, [1.25, -2.5, 0.75]);
        payload.push(14);
        payload.push(3);
        payload.extend([0, 0]);

        let encoded = encode_test_packet(23, 8, 1235, &payload);
        let mut parser = HilinkParserService::new();
        parser.push_rx_bytes(&encoded);

        let telemetry = parser.telemetry();
        assert_eq!(telemetry.rx_frames, 1);
        assert_field(&telemetry, "Protocol", "LAST_MSG_TYPE", "Gps");
        assert_field(&telemetry, "Timing", "SIM_TICK", "15235");
        assert_field(&telemetry, "Timing", "SIM_TIME", "76175000");
        assert_field(&telemetry, "Navigation", "GPS_LAT", "30.2672345");
        assert_field(&telemetry, "Navigation", "GPS_LON", "-97.7431567");
        assert_field(&telemetry, "Navigation", "GPS_ALT_MSL", "172.500");
        assert_field(&telemetry, "Navigation", "GPS_VEL_NED_X", "1.250");
        assert_field(&telemetry, "Navigation", "GPS_VEL_NED_Y", "-2.500");
        assert_field(&telemetry, "Navigation", "GPS_VEL_NED_Z", "0.750");
        assert_field(&telemetry, "Navigation", "GPS_SAT_COUNT", "14");
        assert_field(&telemetry, "Navigation", "GPS_FIX_TYPE", "3 3D fix");
    }

    #[test]
    fn rejects_bad_crc() {
        let mut encoded = encode_test_packet(12, 9, 2222, &[]);
        let index = encoded.len() - 2;
        encoded[index] ^= 0x01;

        let mut parser = HilinkParserService::new();
        parser.push_rx_bytes(&encoded);

        let telemetry = parser.telemetry();
        assert_eq!(telemetry.rx_frames, 0);
        assert_eq!(telemetry.parse_errors, 1);
        assert_eq!(
            telemetry.last_error.as_deref(),
            Some("CRC16-CCITT-FALSE mismatch")
        );
    }

    #[test]
    fn encodes_zero_length_arm_command() {
        let encoded = encode_hilink_command(&HilinkCommand::Arm, 12, 3456).unwrap();
        let message = decode_message(&encoded, FrameDirection::Tx).unwrap();

        assert_eq!(message.header.msg_type, MsgType::Arm);
        assert_eq!(message.header.flags, HEADER_FLAG_REQUEST_ACK);
        assert_eq!(message.header.seq, 12);
        assert_eq!(message.header.send_time_ms, 3456);
        assert_eq!(message.header.payload_len, 0);
        assert!(matches!(message.payload, HilinkPayload::Empty));
    }

    #[test]
    fn queues_rx_command_response_events() {
        let pong = encode_test_packet(2, 50, 2000, &[]);
        let hil_ready = encode_test_packet(12, 53, 2003, &[]);
        let mut heartbeat_payload = Vec::new();
        push_stamp(&mut heartbeat_payload, 100, 200_000);
        heartbeat_payload.push(2);
        heartbeat_payload.extend([0, 0, 0]);
        push_u32(&mut heartbeat_payload, 1);
        let heartbeat = encode_test_packet(5, 54, 2004, &heartbeat_payload);

        let mut ack_payload = Vec::new();
        ack_payload.extend(12u16.to_le_bytes());
        ack_payload.push(40);
        ack_payload.push(0);
        let ack = encode_test_packet(3, 51, 2001, &ack_payload);

        let mut nack_payload = Vec::new();
        nack_payload.extend(13u16.to_le_bytes());
        nack_payload.push(41);
        nack_payload.push(2);
        let nack = encode_test_packet(4, 52, 2002, &nack_payload);

        let mut parser = HilinkParserService::new();
        parser.push_rx_bytes(&pong);
        parser.push_rx_bytes(&ack);
        parser.push_rx_bytes(&nack);
        parser.push_rx_bytes(&hil_ready);
        parser.push_rx_bytes(&heartbeat);

        assert_eq!(
            parser.drain_events(),
            vec![
                HilinkEvent::Pong { seq: 50 },
                HilinkEvent::Ack {
                    seq: 51,
                    acked_seq: 12,
                    acked_msg_type: 40,
                    status: 0,
                },
                HilinkEvent::Nack {
                    seq: 52,
                    rejected_seq: 13,
                    rejected_msg_type: 41,
                    reason: 2,
                },
                HilinkEvent::HilReady { seq: 53 },
                HilinkEvent::Heartbeat { seq: 54 },
            ]
        );
    }

    #[test]
    fn encodes_mission_waypoint_command() {
        let command = HilinkCommand::MissionWaypoint(GlobalWaypointCommand {
            ref_sim_tick: 10,
            ref_sim_time_us: 20_000,
            lat_deg: 30.2672,
            lon_deg: -97.7431,
            alt_msl_m: 171.0,
            yaw_deg: 45.0,
        });
        let encoded = encode_hilink_command(&command, 13, 3457).unwrap();
        let message = decode_message(&encoded, FrameDirection::Tx).unwrap();

        assert_eq!(message.header.msg_type, MsgType::MissionWaypoint);
        assert_eq!(message.header.flags, HEADER_FLAG_REQUEST_ACK);
        assert_eq!(message.header.payload_len, 40);
        let HilinkPayload::GlobalWaypoint(payload) = message.payload else {
            panic!("expected waypoint payload");
        };
        assert_eq!(payload.ref_stamp.sim_tick, 10);
        assert_eq!(payload.ref_stamp.sim_time_us, 20_000);
        assert_eq!(payload.yaw_deg, 45.0);
    }

    #[test]
    fn encodes_motor_test_command() {
        let command = HilinkCommand::MotorTest(MotorTestCommand {
            motor_mask: MOTOR_MASK_M1 | MOTOR_MASK_M3,
            mode: MOTOR_TEST_MODE_RAW_DSHOT,
            value: 150,
            duration_ms: 1_000,
            ramp_ms: 75,
        });
        let encoded = encode_hilink_command(&command, 18, 9000).unwrap();
        let message = decode_message(&encoded, FrameDirection::Tx).unwrap();

        assert_eq!(message.header.msg_type, MsgType::MotorTest);
        assert_eq!(message.header.flags, HEADER_FLAG_REQUEST_ACK);
        assert_eq!(message.header.payload_len, 12);

        let HilinkPayload::MotorTest(payload) = message.payload else {
            panic!("expected motor test payload");
        };
        assert_eq!(payload.motor_mask, MOTOR_MASK_M1 | MOTOR_MASK_M3);
        assert_eq!(payload.mode, MOTOR_TEST_MODE_RAW_DSHOT);
        assert_eq!(payload.value, 150);
        assert_eq!(payload.duration_ms, 1_000);
        assert_eq!(payload.ramp_ms, 75);
    }

    #[test]
    fn encodes_mixer_motor_order_command() {
        let command = HilinkCommand::MixerMotorOrder(MixerMotorOrderCommand {
            output_for_motor: [3, 1, 4, 2],
        });
        let encoded = encode_hilink_command(&command, 19, 9_100).unwrap();
        let message = decode_message(&encoded, FrameDirection::Tx).unwrap();

        assert_eq!(message.header.msg_type, MsgType::MixerMotorOrder);
        assert_eq!(message.header.flags, HEADER_FLAG_REQUEST_ACK);
        assert_eq!(message.header.payload_len, 4);

        let HilinkPayload::MixerMotorOrder(payload) = message.payload else {
            panic!("expected mixer motor order payload");
        };
        assert_eq!(payload.output_for_motor, [3, 1, 4, 2]);
    }

    #[test]
    fn parses_actuator_status_payload() {
        let mut payload = Vec::new();
        payload.push(1);
        payload.push(1);
        payload.push(MOTOR_MASK_ALL);
        payload.push(MOTOR_TEST_MODE_RAW_DSHOT);
        for value in [120u16, 121, 122, 123] {
            payload.extend(value.to_le_bytes());
        }
        payload.extend(300u16.to_le_bytes());
        payload.extend(2_000u16.to_le_bytes());
        payload.extend([3, 1, 4, 2]);
        payload.extend(((1u32 << 0) | (1u32 << 4)).to_le_bytes());

        let encoded = encode_test_packet(67, 5, 100, &payload);
        let mut parser = HilinkParserService::new();
        parser.push_rx_bytes(&encoded);

        let telemetry = parser.telemetry();
        assert_field(&telemetry, "Actuators", "BENCH_ARMED", "1");
        assert_field(&telemetry, "Actuators", "ACTIVE_MOTOR_MASK", "ALL");
        assert_field(&telemetry, "Actuators", "COMMAND_DSHOT_4", "123");
        assert_field(
            &telemetry,
            "Actuators",
            "MIXER_MOTOR_ORDER",
            "M1->O3 M2->O1 M3->O4 M4->O2",
        );
        assert_field(
            &telemetry,
            "Actuators",
            "ACTUATOR_FLAGS",
            "OUTPUT_ACTIVE|BENCH_MODE_ENABLED",
        );
    }

    #[test]
    fn queues_actuator_command_from_rx_hil_response_frame() {
        let mut payload = Vec::new();
        push_stamp(&mut payload, 42, 840_000);
        payload.push(2);
        payload.extend([0, 0, 0]);
        push_u32(&mut payload, (1 << 0) | (1 << 3));
        push_f32x3(&mut payload, [1.0, 2.0, -3.0]);
        push_f32x3(&mut payload, [0.1, 0.2, 0.3]);
        push_f32x4(&mut payload, [1.0, 0.0, 0.0, 0.0]);
        for motor in [12000_u16, 12001, 12002, 12003] {
            payload.extend(motor.to_le_bytes());
        }

        let encoded = encode_test_packet(11, 8, 1235, &payload);
        let mut parser = HilinkParserService::new();
        parser.push_rx_bytes(&encoded);

        let commands = parser.drain_actuator_commands();
        assert_eq!(
            commands,
            vec![HilinkActuatorCommand {
                sim_tick: 42,
                sim_time_us: 840_000,
                flags: (1 << 0) | (1 << 3),
                position_ned_m: [1.0, 2.0, -3.0],
                velocity_ned_mps: [0.1, 0.2, 0.3],
                attitude_quat: [1.0, 0.0, 0.0, 0.0],
                motor_cmd: [12000, 12001, 12002, 12003],
            }]
        );
    }

    fn assert_field(
        telemetry: &HilinkTelemetryState,
        group: &str,
        parameter: &str,
        expected_value: &str,
    ) {
        let field = telemetry
            .fields
            .iter()
            .find(|field| field.group == group && field.parameter == parameter)
            .expect("field should exist");
        assert_eq!(field.value, expected_value);
    }

    fn encode_test_packet(msg_type: u8, seq: u16, send_time_ms: u32, payload: &[u8]) -> Vec<u8> {
        let mut raw = Vec::new();
        raw.push(PROTOCOL_VERSION);
        raw.push(msg_type);
        raw.push(0);
        raw.push(0);
        raw.extend(seq.to_le_bytes());
        raw.extend(send_time_ms.to_le_bytes());
        raw.extend((payload.len() as u16).to_le_bytes());
        raw.extend(payload);
        let crc = crc16_ccitt_false(&raw);
        raw.extend(crc.to_le_bytes());

        let mut encoded = vec![0u8; raw.len() + (raw.len() / 254) + 2];
        let encoded_len = cobs_encode_for_test(&raw, &mut encoded);
        encoded.truncate(encoded_len);
        encoded.push(FRAME_DELIMITER);
        encoded
    }

    fn cobs_encode_for_test(input: &[u8], out: &mut [u8]) -> usize {
        let mut read_index = 0;
        let mut write_index = 1;
        let mut code_index = 0;
        let mut code = 1u8;

        while read_index < input.len() {
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

        out[code_index] = code;
        write_index
    }

    fn push_stamp(out: &mut Vec<u8>, sim_tick: u64, sim_time_us: u64) {
        out.extend(sim_tick.to_le_bytes());
        out.extend(sim_time_us.to_le_bytes());
    }

    fn push_u32(out: &mut Vec<u8>, value: u32) {
        out.extend(value.to_le_bytes());
    }

    fn push_f32(out: &mut Vec<u8>, value: f32) {
        out.extend(value.to_bits().to_le_bytes());
    }

    fn push_f64(out: &mut Vec<u8>, value: f64) {
        out.extend(value.to_bits().to_le_bytes());
    }

    fn push_f32x3(out: &mut Vec<u8>, values: [f32; 3]) {
        for value in values {
            push_f32(out, value);
        }
    }

    fn push_f32x4(out: &mut Vec<u8>, values: [f32; 4]) {
        for value in values {
            push_f32(out, value);
        }
    }
}
