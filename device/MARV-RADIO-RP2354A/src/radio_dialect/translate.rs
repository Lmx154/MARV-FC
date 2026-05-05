use common::protocol::hilink::{self, WirePayload};

use crate::channels::{HILINK_BRIDGE_FRAME_BYTES, HilinkBridgeFrame};
use crate::config::FirmwareRole;
use crate::radio_dialect::{normal, policy, rf, state_cache::RadioStateCache};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HostToRfDecision {
    Translated(HilinkBridgeFrame),
    Drop,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RfToHostDecision {
    Translated(HilinkBridgeFrame),
    Drop,
}

pub fn host_to_rf(
    role: FirmwareRole,
    frame: &HilinkBridgeFrame,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> HostToRfDecision {
    let mut raw = [0u8; HILINK_BRIDGE_FRAME_BYTES];
    let Ok(packet) = normal::decode_frame(frame.as_slice(), &mut raw) else {
        return HostToRfDecision::Drop;
    };

    match role {
        FirmwareRole::GroundStation => ground_station_normal_to_rf(&packet, cache, now_ms)
            .map_or(HostToRfDecision::Drop, HostToRfDecision::Translated),
        FirmwareRole::Radio => vehicle_normal_to_rf(&packet, cache)
            .map_or(HostToRfDecision::Drop, HostToRfDecision::Translated),
    }
}

pub fn rf_to_host(
    role: FirmwareRole,
    payload: &[u8],
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> RfToHostDecision {
    let Ok(packet) = rf::decode_rf_packet(payload) else {
        return RfToHostDecision::Drop;
    };

    match role {
        FirmwareRole::GroundStation => ground_station_rf_to_normal(&packet, cache, now_ms)
            .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated),
        FirmwareRole::Radio => vehicle_rf_to_normal(&packet, cache, now_ms)
            .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated),
    }
}

fn ground_station_normal_to_rf(
    packet: &hilink::DecodedPacket<'_>,
    cache: &mut RadioStateCache,
    _now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    let msg_type = packet.header.message_type().ok()?;
    let command_id = policy::normal_command_to_lora_command_id(msg_type)?;
    normal_command_payload_is_valid(msg_type, packet)?;

    let command_seq = cache.next_rf_command_seq();
    let command = hilink::LoRaCommandPayload {
        command_id,
        command_seq,
        expires_ms: policy::DEFAULT_COMMAND_EXPIRES_MS,
        flags: policy::command_flags(command_id),
        arg0: 0,
        arg1: 0,
    };

    let frame = rf::encode_rf_frame(&command).ok()?;
    cache.store_command_correlation(packet.header, command_seq, command_id);
    Some(frame)
}

fn ground_station_rf_to_normal(
    packet: &rf::DecodedRfPacket<'_>,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    match packet.msg_type {
        rf::RfMsgType::LoRaCommandAck => {
            translate_lora_command_ack_to_normal(packet, cache, now_ms)
        }
        rf::RfMsgType::LoRaFlightSnapshot => {
            let snapshot =
                rf::decode_rf_payload::<hilink::LoRaFlightSnapshotPayload>(packet).ok()?;
            encode_normal_frame(
                &lora_flight_to_telemetry_snapshot(snapshot),
                cache.next_normal_tx_seq(),
                now_ms,
            )
        }
        rf::RfMsgType::LoRaGpsSnapshot => {
            let gps = rf::decode_rf_payload::<hilink::LoRaGpsSnapshotPayload>(packet).ok()?;
            encode_normal_frame(
                &lora_gps_to_normal_gps(gps),
                cache.next_normal_tx_seq(),
                now_ms,
            )
        }
        _ => None,
    }
}

fn vehicle_rf_to_normal(
    packet: &rf::DecodedRfPacket<'_>,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    match packet.msg_type {
        rf::RfMsgType::LoRaCommand => {
            let command = rf::decode_rf_payload::<hilink::LoRaCommandPayload>(packet).ok()?;
            let normal_seq = cache.next_normal_tx_seq();
            let frame = lora_command_to_normal_command(command, normal_seq, now_ms)?;
            let normal_msg_type = normal_msg_type_for_lora_command(command.command_id)?;
            cache.store_command_correlation(
                hilink::Header::new(normal_msg_type, normal_seq, now_ms, 0),
                command.command_seq,
                command.command_id,
            );
            Some(frame)
        }
        _ => None,
    }
}

fn vehicle_normal_to_rf(
    packet: &hilink::DecodedPacket<'_>,
    cache: &mut RadioStateCache,
) -> Option<HilinkBridgeFrame> {
    let msg_type = packet.header.message_type().ok()?;
    match msg_type {
        hilink::MsgType::Ack => {
            let ack = hilink::decode_payload::<hilink::AckPayload>(packet).ok()?;
            let correlation = cache.take_normal_correlation(ack.acked_seq, ack.acked_msg_type)?;
            let rf_ack = hilink::LoRaCommandAckPayload {
                command_id: correlation.command_id,
                command_seq: correlation.command_seq,
                status: hilink::lora_command_status::ACCEPTED,
                reason: hilink::lora_command_reason::NONE,
                state: 0,
                reserved: 0,
                detail: 0,
            };
            rf::encode_rf_frame(&rf_ack).ok()
        }
        hilink::MsgType::Nack => {
            let nack = hilink::decode_payload::<hilink::NackPayload>(packet).ok()?;
            let correlation =
                cache.take_normal_correlation(nack.rejected_seq, nack.rejected_msg_type)?;
            let rf_ack = hilink::LoRaCommandAckPayload {
                command_id: correlation.command_id,
                command_seq: correlation.command_seq,
                status: hilink::lora_command_status::REJECTED,
                reason: nack.reason,
                state: 0,
                reserved: 0,
                detail: 0,
            };
            rf::encode_rf_frame(&rf_ack).ok()
        }
        hilink::MsgType::TelemetrySnapshot => {
            let telemetry =
                hilink::decode_payload::<hilink::TelemetrySnapshotPayload>(packet).ok()?;
            rf::encode_rf_frame(&telemetry_to_lora_flight(
                telemetry,
                packet.header.send_time_ms,
            ))
            .ok()
        }
        hilink::MsgType::Gps => {
            let gps = hilink::decode_payload::<hilink::GpsPayload>(packet).ok()?;
            rf::encode_rf_frame(&gps_to_lora_gps(gps, packet.header.send_time_ms)).ok()
        }
        _ => None,
    }
}

fn translate_lora_command_ack_to_normal(
    packet: &rf::DecodedRfPacket<'_>,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    let ack = rf::decode_rf_payload::<hilink::LoRaCommandAckPayload>(packet).ok()?;
    let correlation = cache.take_command_correlation(ack.command_id, ack.command_seq)?;

    if policy::lora_ack_status_is_ack(ack.status) {
        let normal_ack = hilink::AckPayload {
            acked_seq: correlation.normal_seq,
            acked_msg_type: correlation.normal_msg_type,
            status: ack.status,
        };
        encode_normal_frame(&normal_ack, cache.next_normal_tx_seq(), now_ms)
    } else {
        let normal_nack = hilink::NackPayload {
            rejected_seq: correlation.normal_seq,
            rejected_msg_type: correlation.normal_msg_type,
            reason: ack.reason,
        };
        encode_normal_frame(&normal_nack, cache.next_normal_tx_seq(), now_ms)
    }
}

fn normal_command_payload_is_valid(
    msg_type: hilink::MsgType,
    packet: &hilink::DecodedPacket<'_>,
) -> Option<()> {
    let valid = match msg_type {
        hilink::MsgType::Arm => hilink::decode_payload::<hilink::ArmPayload>(packet).is_ok(),
        hilink::MsgType::Disarm => hilink::decode_payload::<hilink::DisarmPayload>(packet).is_ok(),
        hilink::MsgType::MotorStop => {
            hilink::decode_payload::<hilink::MotorStopPayload>(packet).is_ok()
        }
        hilink::MsgType::Ping => hilink::decode_payload::<hilink::PingPayload>(packet).is_ok(),
        _ => return None,
    };
    valid.then_some(())
}

fn lora_command_to_normal_command(
    command: hilink::LoRaCommandPayload,
    seq: u16,
    now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    match command.command_id {
        hilink::lora_command_id::ARM => encode_normal_frame(&hilink::ArmPayload, seq, now_ms),
        hilink::lora_command_id::DISARM => encode_normal_frame(&hilink::DisarmPayload, seq, now_ms),
        hilink::lora_command_id::MOTOR_STOP => {
            encode_normal_frame(&hilink::MotorStopPayload, seq, now_ms)
        }
        hilink::lora_command_id::PING => encode_normal_frame(&hilink::PingPayload, seq, now_ms),
        _ => None,
    }
}

fn normal_msg_type_for_lora_command(command_id: u16) -> Option<hilink::MsgType> {
    match command_id {
        hilink::lora_command_id::ARM => Some(hilink::MsgType::Arm),
        hilink::lora_command_id::DISARM => Some(hilink::MsgType::Disarm),
        hilink::lora_command_id::MOTOR_STOP => Some(hilink::MsgType::MotorStop),
        hilink::lora_command_id::PING => Some(hilink::MsgType::Ping),
        _ => None,
    }
}

fn telemetry_to_lora_flight(
    telemetry: hilink::TelemetrySnapshotPayload,
    time_ms: u32,
) -> hilink::LoRaFlightSnapshotPayload {
    hilink::LoRaFlightSnapshotPayload {
        time_ms,
        state: telemetry.system_state,
        mode: hilink::lora_mode::UNKNOWN,
        flags: telemetry_flags_to_lora_flags(telemetry.flags),
        altitude_dm: saturating_f32_to_i32(-telemetry.position_ned_m[2] * 10.0),
        vertical_velocity_cms: saturating_f32_to_i16(-telemetry.velocity_ned_mps[2] * 100.0),
        accel_mag_cms2: 0,
        battery_mv: saturating_f32_to_u16(telemetry.battery_voltage_v * 1_000.0),
        pyro_or_actuator_flags: 0,
        fault_summary: 0,
    }
}

fn gps_to_lora_gps(gps: hilink::GpsPayload, time_ms: u32) -> hilink::LoRaGpsSnapshotPayload {
    let gps_valid = gps.fix_type > 0;
    hilink::LoRaGpsSnapshotPayload {
        time_ms,
        lat_e7: if gps_valid {
            saturating_f64_to_i32(gps.lat_deg * 10_000_000.0)
        } else {
            hilink::lora_scaling::LAT_LON_INVALID_E7
        },
        lon_e7: if gps_valid {
            saturating_f64_to_i32(gps.lon_deg * 10_000_000.0)
        } else {
            hilink::lora_scaling::LAT_LON_INVALID_E7
        },
        alt_msl_dm: if gps_valid {
            saturating_f32_to_i32(gps.alt_msl_m * 10.0)
        } else {
            hilink::lora_scaling::ALT_MSL_INVALID_DM
        },
        ground_speed_cms: saturating_f32_to_u16(
            sqrt_f32(
                gps.vel_ned_mps[0] * gps.vel_ned_mps[0] + gps.vel_ned_mps[1] * gps.vel_ned_mps[1],
            ) * 100.0,
        ),
        heading_cdeg: hilink::lora_scaling::HEADING_INVALID_CDEG,
        sats: gps.sats,
        fix_type: gps.fix_type,
    }
}

fn lora_flight_to_telemetry_snapshot(
    snapshot: hilink::LoRaFlightSnapshotPayload,
) -> hilink::TelemetrySnapshotPayload {
    hilink::TelemetrySnapshotPayload {
        stamp: hilink::SimStamp {
            sim_tick: 0,
            sim_time_us: u64::from(snapshot.time_ms) * 1_000,
        },
        system_state: snapshot.state,
        reserved0: [0; 3],
        flags: 0,
        position_ned_m: [0.0, 0.0, -(snapshot.altitude_dm as f32) / 10.0],
        velocity_ned_mps: [0.0, 0.0, -(snapshot.vertical_velocity_cms as f32) / 100.0],
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        battery_voltage_v: (snapshot.battery_mv as f32) / 1_000.0,
        rssi_dbm: 0,
        snr_db_x100: 0,
        loss_pct_x100: 0,
    }
}

fn lora_gps_to_normal_gps(snapshot: hilink::LoRaGpsSnapshotPayload) -> hilink::GpsPayload {
    hilink::GpsPayload {
        stamp: hilink::SimStamp {
            sim_tick: 0,
            sim_time_us: u64::from(snapshot.time_ms) * 1_000,
        },
        lat_deg: if snapshot.lat_e7 == hilink::lora_scaling::LAT_LON_INVALID_E7 {
            0.0
        } else {
            snapshot.lat_e7 as f64 / 10_000_000.0
        },
        lon_deg: if snapshot.lon_e7 == hilink::lora_scaling::LAT_LON_INVALID_E7 {
            0.0
        } else {
            snapshot.lon_e7 as f64 / 10_000_000.0
        },
        alt_msl_m: if snapshot.alt_msl_dm == hilink::lora_scaling::ALT_MSL_INVALID_DM {
            0.0
        } else {
            snapshot.alt_msl_dm as f32 / 10.0
        },
        vel_ned_mps: [0.0, 0.0, 0.0],
        sats: snapshot.sats,
        fix_type: snapshot.fix_type,
        reserved0: [0; 2],
    }
}

fn telemetry_flags_to_lora_flags(flags: u32) -> u16 {
    let mut lora_flags = 0;
    if (flags & hilink::response_flags::ARMED) != 0 {
        lora_flags |= 1 << hilink::lora_flags::ARMED;
    }
    if (flags & hilink::response_flags::FAILSAFE) != 0 {
        lora_flags |= 1 << hilink::lora_flags::FAILSAFE;
    }
    if (flags & hilink::response_flags::ESTIMATOR_VALID) != 0 {
        lora_flags |= 1 << hilink::lora_flags::ESTIMATOR_VALID;
    }
    lora_flags
}

fn encode_normal_frame<P: WirePayload>(
    payload: &P,
    seq: u16,
    now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    let mut raw = [0u8; HILINK_BRIDGE_FRAME_BYTES];
    let mut encoded = [0u8; HILINK_BRIDGE_FRAME_BYTES];
    let len = hilink::encode_packet(payload, seq, now_ms, &mut raw, &mut encoded).ok()?;

    let mut frame = HilinkBridgeFrame::new();
    frame.bytes[..len].copy_from_slice(&encoded[..len]);
    frame.len = len;
    Some(frame)
}

fn saturating_f32_to_i16(value: f32) -> i16 {
    if !value.is_finite() {
        0
    } else {
        round_f32(value).clamp(i16::MIN as f32, i16::MAX as f32) as i16
    }
}

fn saturating_f32_to_i32(value: f32) -> i32 {
    if !value.is_finite() {
        hilink::lora_scaling::ALTITUDE_INVALID_DM
    } else {
        round_f32(value).clamp((i32::MIN + 1) as f32, i32::MAX as f32) as i32
    }
}

fn saturating_f64_to_i32(value: f64) -> i32 {
    if !value.is_finite() {
        hilink::lora_scaling::LAT_LON_INVALID_E7
    } else {
        round_f64(value).clamp((i32::MIN + 1) as f64, i32::MAX as f64) as i32
    }
}

fn saturating_f32_to_u16(value: f32) -> u16 {
    if !value.is_finite() {
        0
    } else {
        round_f32(value).clamp(0.0, u16::MAX as f32) as u16
    }
}

fn round_f32(value: f32) -> f32 {
    if value >= 0.0 {
        value + 0.5
    } else {
        value - 0.5
    }
}

fn round_f64(value: f64) -> f64 {
    if value >= 0.0 {
        value + 0.5
    } else {
        value - 0.5
    }
}

fn sqrt_f32(value: f32) -> f32 {
    if !value.is_finite() || value <= 0.0 {
        return 0.0;
    }

    let mut estimate = if value >= 1.0 { value } else { 1.0 };
    for _ in 0..6 {
        estimate = 0.5 * (estimate + value / estimate);
    }
    estimate
}
