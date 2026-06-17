//! Decode one HILink frame into a typed message we care about, using the shared `marv-hilink`
//! crate (the exact same encode/decode source the firmware uses).

use marv_hilink::{
    decode_packet, decode_payload, raw_frame_len, AckPayload, AuxImuPayload, BaroPayload,
    GpsPayload, HeartbeatPayload, ImuPayload, MagPayload, MsgType, NackPayload, RadioStatusPayload,
    SystemStatePayload, TelemetrySnapshotPayload,
};

use crate::model::{accel_g_from_cms2, altitude_from_ned, flight_stage, vertical_speed_from_ned};

/// Scratch buffer big enough for the largest payload's pre-COBS frame.
const SCRATCH_LEN: usize = raw_frame_len(256);

/// A decoded HILink message, narrowed to the variants the telemetry UI uses.
pub enum ParsedMessage {
    Telemetry(TelemetrySnapshotPayload),
    Gps(GpsPayload),
    Radio(RadioStatusPayload),
    Heartbeat(HeartbeatPayload),
    SystemState(SystemStatePayload),
    Imu(ImuPayload),
    AuxImu(AuxImuPayload),
    Mag(MagPayload),
    Baro(BaroPayload),
    Ack(AckPayload),
    Nack(NackPayload),
    /// A valid frame of a type the UI does not visualize.
    Other(MsgType),
}

pub struct DecodedFrame {
    pub seq: u16,
    pub message: ParsedMessage,
}

/// Number of distinct `ParsedMessage` categories, for per-class console throttling.
pub const PARSED_CLASS_COUNT: usize = 12;

impl ParsedMessage {
    /// A stable small index per message category so the debug console can throttle each class
    /// independently (otherwise the high-rate `Telemetry` frames starve the low-rate ones).
    pub fn class_key(&self) -> usize {
        match self {
            ParsedMessage::Telemetry(_) => 0,
            ParsedMessage::Gps(_) => 1,
            ParsedMessage::Radio(_) => 2,
            ParsedMessage::Heartbeat(_) => 3,
            ParsedMessage::SystemState(_) => 4,
            ParsedMessage::Imu(_) => 5,
            ParsedMessage::AuxImu(_) => 6,
            ParsedMessage::Mag(_) => 7,
            ParsedMessage::Baro(_) => 8,
            ParsedMessage::Ack(_) => 9,
            ParsedMessage::Nack(_) => 10,
            ParsedMessage::Other(_) => 11,
        }
    }
}

/// Decode a complete frame (COBS bytes terminated by `0x00`).
pub fn decode_frame(frame: &[u8]) -> Result<DecodedFrame, marv_hilink::Error> {
    let mut scratch = [0u8; SCRATCH_LEN];
    let packet = decode_packet(frame, &mut scratch)?;
    let msg_type = packet.header.message_type()?;

    let message = match msg_type {
        MsgType::TelemetrySnapshot => ParsedMessage::Telemetry(decode_payload(&packet)?),
        MsgType::Gps => ParsedMessage::Gps(decode_payload(&packet)?),
        MsgType::RadioStatus => ParsedMessage::Radio(decode_payload(&packet)?),
        MsgType::Heartbeat => ParsedMessage::Heartbeat(decode_payload(&packet)?),
        MsgType::SystemState => ParsedMessage::SystemState(decode_payload(&packet)?),
        MsgType::Imu => ParsedMessage::Imu(decode_payload(&packet)?),
        MsgType::AuxImu => ParsedMessage::AuxImu(decode_payload(&packet)?),
        MsgType::Mag => ParsedMessage::Mag(decode_payload(&packet)?),
        MsgType::Baro => ParsedMessage::Baro(decode_payload(&packet)?),
        MsgType::Ack => ParsedMessage::Ack(decode_payload(&packet)?),
        MsgType::Nack => ParsedMessage::Nack(decode_payload(&packet)?),
        other => ParsedMessage::Other(other),
    };

    Ok(DecodedFrame {
        seq: packet.header.seq,
        message,
    })
}

impl ParsedMessage {
    /// A concise one-line summary for the debug console (PARSED lines).
    pub fn summary(&self) -> String {
        match self {
            ParsedMessage::Telemetry(t) => format!(
                "TELEM stage={} alt={:.1}m vspd={:.1}m/s accel={:.2}g batt={:.2}V",
                flight_stage(t.system_state),
                altitude_from_ned(t.position_ned_m),
                vertical_speed_from_ned(t.velocity_ned_mps),
                accel_g_from_cms2(t.accel_mag_cms2),
                t.battery_voltage_v,
            ),
            ParsedMessage::Gps(g) => format!(
                "GPS {:.5},{:.5} alt={:.1}m sats={} fix={}",
                g.lat_deg, g.lon_deg, g.alt_msl_m, g.sats, g.fix_type,
            ),
            ParsedMessage::Radio(r) => format!(
                "RADIO rssi={}dBm snr={:.1}dB loss={:.1}% rate={}Hz",
                r.rssi_dbm,
                f32::from(r.snr_db_x100) / 100.0,
                f32::from(r.loss_pct_x100) / 100.0,
                r.packet_rate_hz,
            ),
            ParsedMessage::Heartbeat(h) => {
                format!("HEARTBEAT stage={} flags={:#06x}", flight_stage(h.system_state), h.flags)
            }
            ParsedMessage::SystemState(s) => format!(
                "SYSTEM stage={} batt={:.2}V flags={:#06x}",
                flight_stage(s.system_state),
                s.battery_voltage_v,
                s.flags,
            ),
            ParsedMessage::Imu(i) => format!("IMU1 {}", imu_summary(i.accel_mps2, i.gyro_rps)),
            ParsedMessage::AuxImu(i) => format!("IMU2 {}", imu_summary(i.accel_mps2, i.gyro_rps)),
            ParsedMessage::Mag(m) => format!(
                "MAG [{:.1},{:.1},{:.1}]uT",
                m.field_ut[0], m.field_ut[1], m.field_ut[2],
            ),
            ParsedMessage::Baro(b) => format!(
                "BARO p={:.0}Pa alt={:.1}m T={:.1}C",
                b.pressure_pa, b.altitude_m, b.temperature_c,
            ),
            ParsedMessage::Ack(a) => format!(
                "ACK seq={} msg={} status={}",
                a.acked_seq, a.acked_msg_type, a.status,
            ),
            ParsedMessage::Nack(n) => format!(
                "NACK seq={} msg={} reason={}",
                n.rejected_seq, n.rejected_msg_type, n.reason,
            ),
            ParsedMessage::Other(t) => format!("{t:?} (not visualized)"),
        }
    }
}

/// Format an IMU sample as accel in g and gyro in deg/s for the debug console.
fn imu_summary(accel_mps2: [f32; 3], gyro_rps: [f32; 3]) -> String {
    const G: f32 = 9.806_65;
    const DEG_PER_RAD: f32 = 57.295_78;
    format!(
        "a=[{:.2},{:.2},{:.2}]g g=[{:.1},{:.1},{:.1}]dps",
        accel_mps2[0] / G,
        accel_mps2[1] / G,
        accel_mps2[2] / G,
        gyro_rps[0] * DEG_PER_RAD,
        gyro_rps[1] * DEG_PER_RAD,
        gyro_rps[2] * DEG_PER_RAD,
    )
}
