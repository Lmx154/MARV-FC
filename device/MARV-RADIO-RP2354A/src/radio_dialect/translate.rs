use common::protocol::hilink::{self, WirePayload};

use crate::channels::{HILINK_BRIDGE_FRAME_BYTES, HilinkBridgeFrame};
use crate::config::FirmwareRole;
use crate::radio_dialect::profile_switch::{ProfileChange, VehicleArm};
use crate::radio_dialect::{normal, policy, rf, state_cache::RadioStateCache};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HostToRfDecision {
    Translated(HilinkBridgeFrame),
    Cached,
    Drop,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RfToHostDecision {
    Translated(HilinkBridgeFrame),
    Handled,
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
        FirmwareRole::GroundStation => ground_station_normal_to_rf(&packet, cache, now_ms),
        FirmwareRole::Radio => vehicle_normal_to_rf(&packet, cache, now_ms),
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
        FirmwareRole::GroundStation => ground_station_rf_to_normal(&packet, cache, now_ms),
        FirmwareRole::Radio => vehicle_rf_to_normal(&packet, cache, now_ms),
    }
}

fn ground_station_normal_to_rf(
    packet: &hilink::DecodedPacket<'_>,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> HostToRfDecision {
    let Ok(msg_type) = packet.header.message_type() else {
        return HostToRfDecision::Drop;
    };

    if msg_type == hilink::MsgType::SetRadioProfile {
        return ground_station_initiate_profile_switch(packet, cache, now_ms);
    }

    if msg_type == hilink::MsgType::SetIdleFallback {
        return ground_station_set_idle_fallback(packet, cache);
    }

    let Some(command_id) = policy::normal_command_to_lora_command_id(msg_type) else {
        return HostToRfDecision::Drop;
    };
    if normal_command_payload_is_valid(msg_type, packet).is_none() {
        return HostToRfDecision::Drop;
    }

    let command_seq = cache.next_rf_command_seq();
    let command = hilink::LoRaCommandPayload {
        command_id,
        command_seq,
        expires_ms: policy::DEFAULT_COMMAND_EXPIRES_MS,
        flags: policy::command_flags(command_id),
        arg0: 0,
        arg1: 0,
    };

    let Ok(frame) = rf::encode_rf_frame(&command) else {
        return HostToRfDecision::Drop;
    };
    cache.store_command_correlation(packet.header, command_seq, command_id);
    HostToRfDecision::Translated(frame)
}

/// Begin the GS side of a coordinated, link-wide RF profile switch on a host `SetRadioProfile`.
///
/// Validates the request against the band plan and refuses if a switch is already in flight or
/// the request is illegal (dropped → the host command times out without an ack). On success it
/// correlates the command for the eventual host ack and arms the switch FSM; the actual
/// `LoRaSetProfile` frame (first send + retransmits) is emitted by the scheduler while the GS
/// awaits the vehicle's ack, so nothing is transmitted inline here (`Cached`).
fn ground_station_initiate_profile_switch(
    packet: &hilink::DecodedPacket<'_>,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> HostToRfDecision {
    let Ok(request) = hilink::decode_payload::<hilink::SetRadioProfilePayload>(packet) else {
        return HostToRfDecision::Drop;
    };
    let change = ProfileChange {
        command_seq: 0,
        preset: request.preset,
        tx_power_dbm: request.tx_power_dbm,
        frequency_hz: request.frequency_hz,
        flags: request.flags,
    };
    if change.to_profile().is_none() || cache.profile_switch_active() {
        return HostToRfDecision::Drop;
    }

    let command_seq = cache.next_rf_command_seq();
    let change = ProfileChange {
        command_seq,
        ..change
    };
    cache.store_command_correlation(
        packet.header,
        command_seq,
        hilink::lora_command_id::SET_RADIO_PROFILE,
    );
    cache.begin_gs_profile_switch(change, now_ms);
    HostToRfDecision::Cached
}

/// Apply a host `SetIdleFallback` to the ground-station radio and relay it to the vehicle.
///
/// The GS adopts the (clamped) window immediately for its own idle-fallback watchdog, then emits a
/// `SET_IDLE_FALLBACK` LoRa command carrying the window in `arg0` so the vehicle radio adopts the
/// same value — both ends re-home to the setup profile on the operator's schedule. The relayed
/// command is correlated so the vehicle's ack becomes the host's Ack/Nack.
fn ground_station_set_idle_fallback(
    packet: &hilink::DecodedPacket<'_>,
    cache: &mut RadioStateCache,
) -> HostToRfDecision {
    let Ok(request) = hilink::decode_payload::<hilink::SetIdleFallbackPayload>(packet) else {
        return HostToRfDecision::Drop;
    };
    let idle_fallback_ms = hilink::idle_fallback::clamp(request.idle_fallback_ms);
    cache.set_idle_fallback_ms(idle_fallback_ms);

    let command_seq = cache.next_rf_command_seq();
    let command = hilink::LoRaCommandPayload {
        command_id: hilink::lora_command_id::SET_IDLE_FALLBACK,
        command_seq,
        expires_ms: policy::DEFAULT_COMMAND_EXPIRES_MS,
        flags: policy::command_flags(hilink::lora_command_id::SET_IDLE_FALLBACK),
        arg0: idle_fallback_ms as i32,
        arg1: 0,
    };

    let Ok(frame) = rf::encode_rf_frame(&command) else {
        return HostToRfDecision::Drop;
    };
    cache.store_command_correlation(
        packet.header,
        command_seq,
        hilink::lora_command_id::SET_IDLE_FALLBACK,
    );
    HostToRfDecision::Translated(frame)
}

fn ground_station_rf_to_normal(
    packet: &rf::DecodedRfPacket<'_>,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> RfToHostDecision {
    match packet.msg_type {
        rf::RfMsgType::LoRaCommandAck => {
            translate_lora_command_ack_to_normal(packet, cache, now_ms)
                .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated)
        }
        rf::RfMsgType::LoRaFlightSnapshot => {
            let Ok(snapshot) = rf::decode_rf_payload::<hilink::LoRaFlightSnapshotPayload>(packet)
            else {
                return RfToHostDecision::Drop;
            };
            encode_normal_frame(
                &lora_flight_to_telemetry_snapshot(snapshot),
                cache.next_normal_tx_seq(),
                now_ms,
            )
            .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated)
        }
        rf::RfMsgType::LoRaGpsSnapshot => {
            let Ok(gps) = rf::decode_rf_payload::<hilink::LoRaGpsSnapshotPayload>(packet) else {
                return RfToHostDecision::Drop;
            };
            encode_normal_frame(
                &lora_gps_to_normal_gps(gps),
                cache.next_normal_tx_seq(),
                now_ms,
            )
            .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated)
        }
        rf::RfMsgType::LoRaLinkStatus => {
            let Ok(link) = rf::decode_rf_payload::<hilink::LoRaLinkStatusPayload>(packet) else {
                return RfToHostDecision::Drop;
            };
            // Stamp the ground-station radio's own active profile so the host can verify the link
            // settled on the commanded settings (a decoded peer frame proves the vehicle matches).
            let status = lora_link_status_to_radio_status(
                link,
                cache.local_active_preset(),
                cache.local_active_frequency_hz(),
                cache.local_active_tx_power_dbm(),
            );
            let seq = cache.next_normal_tx_seq();
            encode_normal_frame(&status, seq, now_ms)
                .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated)
        }
        rf::RfMsgType::LoRaImu1Snapshot => {
            let Ok(imu) = rf::decode_rf_payload::<hilink::LoRaImu1SnapshotPayload>(packet) else {
                return RfToHostDecision::Drop;
            };
            encode_normal_frame(
                &lora_imu1_to_normal_imu(imu),
                cache.next_normal_tx_seq(),
                now_ms,
            )
            .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated)
        }
        rf::RfMsgType::LoRaImu2Snapshot => {
            let Ok(imu) = rf::decode_rf_payload::<hilink::LoRaImu2SnapshotPayload>(packet) else {
                return RfToHostDecision::Drop;
            };
            encode_normal_frame(
                &lora_imu2_to_normal_aux_imu(imu),
                cache.next_normal_tx_seq(),
                now_ms,
            )
            .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated)
        }
        rf::RfMsgType::LoRaMagSnapshot => {
            let Ok(mag) = rf::decode_rf_payload::<hilink::LoRaMagSnapshotPayload>(packet) else {
                return RfToHostDecision::Drop;
            };
            encode_normal_frame(
                &lora_mag_to_normal_mag(mag),
                cache.next_normal_tx_seq(),
                now_ms,
            )
            .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated)
        }
        rf::RfMsgType::LoRaBaroSnapshot => {
            let Ok(baro) = rf::decode_rf_payload::<hilink::LoRaBaroSnapshotPayload>(packet) else {
                return RfToHostDecision::Drop;
            };
            encode_normal_frame(
                &lora_baro_to_normal_baro(baro),
                cache.next_normal_tx_seq(),
                now_ms,
            )
            .map_or(RfToHostDecision::Drop, RfToHostDecision::Translated)
        }
        rf::RfMsgType::LoRaEvent | rf::RfMsgType::LoRaFaults => RfToHostDecision::Handled,
        _ => RfToHostDecision::Drop,
    }
}

fn vehicle_rf_to_normal(
    packet: &rf::DecodedRfPacket<'_>,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> RfToHostDecision {
    match packet.msg_type {
        rf::RfMsgType::LoRaCommand => {
            let Ok(command) = rf::decode_rf_payload::<hilink::LoRaCommandPayload>(packet) else {
                return RfToHostDecision::Drop;
            };

            // SET_IDLE_FALLBACK is consumed by the vehicle radio itself (not forwarded to the flight
            // controller): adopt the operator's window so both ends re-home on the same schedule.
            if command.command_id == hilink::lora_command_id::SET_IDLE_FALLBACK {
                let (status, reason) = if cache
                    .vehicle_command_is_duplicate(command.command_id, command.command_seq)
                {
                    (
                        hilink::lora_command_status::DUPLICATE_ACCEPTED,
                        hilink::lora_command_reason::DUPLICATE,
                    )
                } else {
                    cache.set_idle_fallback_ms(command.arg0.max(0) as u32);
                    cache.note_vehicle_command_forwarded(command.command_id, command.command_seq);
                    (
                        hilink::lora_command_status::ACCEPTED,
                        hilink::lora_command_reason::NONE,
                    )
                };
                cache.queue_pending_lora_command_ack(lora_command_ack(command, status, reason));
                return RfToHostDecision::Handled;
            }

            let Some(normal_msg_type) = normal_msg_type_for_lora_command(command.command_id) else {
                cache.queue_pending_lora_command_ack(lora_command_ack(
                    command,
                    hilink::lora_command_status::REJECTED,
                    hilink::lora_command_reason::UNSUPPORTED,
                ));
                return RfToHostDecision::Handled;
            };

            if cache.vehicle_command_is_duplicate(command.command_id, command.command_seq) {
                cache.queue_pending_lora_command_ack(lora_command_ack(
                    command,
                    hilink::lora_command_status::DUPLICATE_ACCEPTED,
                    hilink::lora_command_reason::DUPLICATE,
                ));
                return RfToHostDecision::Handled;
            }

            let normal_seq = cache.next_normal_tx_seq();
            let Some(frame) = lora_command_to_normal_command(command, normal_seq, now_ms) else {
                cache.queue_pending_lora_command_ack(lora_command_ack(
                    command,
                    hilink::lora_command_status::REJECTED,
                    hilink::lora_command_reason::UNSUPPORTED,
                ));
                return RfToHostDecision::Handled;
            };
            cache.note_vehicle_command_forwarded(command.command_id, command.command_seq);
            cache.store_command_correlation(
                hilink::Header::new(normal_msg_type, normal_seq, now_ms, 0),
                command.command_seq,
                command.command_id,
            );
            RfToHostDecision::Translated(frame)
        }
        rf::RfMsgType::LoRaSetProfile => {
            let Ok(request) = rf::decode_rf_payload::<hilink::LoRaSetProfilePayload>(packet) else {
                return RfToHostDecision::Drop;
            };
            let change = ProfileChange {
                command_seq: request.command_seq,
                preset: request.preset,
                tx_power_dbm: request.tx_power_dbm,
                frequency_hz: request.frequency_hz,
                flags: request.flags,
            };
            let (status, reason) = match cache.begin_vehicle_profile_switch(change, now_ms) {
                VehicleArm::Accepted => (
                    hilink::lora_command_status::ACCEPTED,
                    hilink::lora_command_reason::NONE,
                ),
                VehicleArm::DuplicateAccepted => (
                    hilink::lora_command_status::DUPLICATE_ACCEPTED,
                    hilink::lora_command_reason::DUPLICATE,
                ),
                VehicleArm::Rejected => (
                    hilink::lora_command_status::REJECTED,
                    hilink::lora_command_reason::BAD_ARGUMENT,
                ),
                VehicleArm::Busy => (
                    hilink::lora_command_status::BUSY,
                    hilink::lora_command_reason::RADIO_BUSY,
                ),
            };
            // The vehicle radio consumes the profile change itself (it is not a flight-controller
            // command), acking back over the current link so the GS can complete the switch.
            cache.queue_pending_lora_command_ack(set_profile_command_ack(
                change.command_seq,
                status,
                reason,
            ));
            RfToHostDecision::Handled
        }
        rf::RfMsgType::LoRaEvent | rf::RfMsgType::LoRaFaults | rf::RfMsgType::LoRaLinkStatus => {
            RfToHostDecision::Handled
        }
        _ => RfToHostDecision::Drop,
    }
}

fn set_profile_command_ack(
    command_seq: u16,
    status: u8,
    reason: u8,
) -> hilink::LoRaCommandAckPayload {
    hilink::LoRaCommandAckPayload {
        command_id: hilink::lora_command_id::SET_RADIO_PROFILE,
        command_seq,
        status,
        reason,
        state: 0,
        reserved: 0,
        detail: 0,
    }
}

fn vehicle_normal_to_rf(
    packet: &hilink::DecodedPacket<'_>,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> HostToRfDecision {
    let Ok(msg_type) = packet.header.message_type() else {
        return HostToRfDecision::Drop;
    };
    match msg_type {
        hilink::MsgType::Ack => {
            let Ok(ack) = hilink::decode_payload::<hilink::AckPayload>(packet) else {
                return HostToRfDecision::Drop;
            };
            let Some(correlation) =
                cache.take_normal_correlation(ack.acked_seq, ack.acked_msg_type)
            else {
                return HostToRfDecision::Drop;
            };
            let rf_ack = hilink::LoRaCommandAckPayload {
                command_id: correlation.command_id,
                command_seq: correlation.command_seq,
                status: hilink::lora_command_status::ACCEPTED,
                reason: hilink::lora_command_reason::NONE,
                state: 0,
                reserved: 0,
                detail: 0,
            };
            rf::encode_rf_frame(&rf_ack)
                .map_or(HostToRfDecision::Drop, HostToRfDecision::Translated)
        }
        hilink::MsgType::Nack => {
            let Ok(nack) = hilink::decode_payload::<hilink::NackPayload>(packet) else {
                return HostToRfDecision::Drop;
            };
            let Some(correlation) =
                cache.take_normal_correlation(nack.rejected_seq, nack.rejected_msg_type)
            else {
                return HostToRfDecision::Drop;
            };
            let rf_ack = hilink::LoRaCommandAckPayload {
                command_id: correlation.command_id,
                command_seq: correlation.command_seq,
                status: hilink::lora_command_status::REJECTED,
                reason: nack.reason,
                state: 0,
                reserved: 0,
                detail: 0,
            };
            rf::encode_rf_frame(&rf_ack)
                .map_or(HostToRfDecision::Drop, HostToRfDecision::Translated)
        }
        hilink::MsgType::Pong => {
            if hilink::decode_payload::<hilink::PongPayload>(packet).is_err() {
                return HostToRfDecision::Drop;
            }
            let Some(correlation) =
                cache.take_first_normal_correlation_for_msg_type(hilink::MsgType::Ping)
            else {
                return HostToRfDecision::Drop;
            };
            let rf_ack = hilink::LoRaCommandAckPayload {
                command_id: correlation.command_id,
                command_seq: correlation.command_seq,
                status: hilink::lora_command_status::ACCEPTED,
                reason: hilink::lora_command_reason::NONE,
                state: 0,
                reserved: 0,
                detail: 0,
            };
            rf::encode_rf_frame(&rf_ack)
                .map_or(HostToRfDecision::Drop, HostToRfDecision::Translated)
        }
        hilink::MsgType::TelemetrySnapshot => {
            let Ok(telemetry) = hilink::decode_payload::<hilink::TelemetrySnapshotPayload>(packet)
            else {
                return HostToRfDecision::Drop;
            };
            cache.note_vehicle_system_state(
                telemetry.system_state,
                telemetry_flags_to_fault_summary(telemetry.flags),
                packet.header.send_time_ms,
            );
            cache.store_vehicle_flight_snapshot(telemetry_to_lora_flight(
                telemetry,
                packet.header.send_time_ms,
            ));
            HostToRfDecision::Cached
        }
        hilink::MsgType::Imu => {
            let Ok(imu) = hilink::decode_payload::<hilink::ImuPayload>(packet) else {
                return HostToRfDecision::Drop;
            };
            let time_ms = packet.header.send_time_ms;
            // Keep folding accel magnitude into the flight snapshot for the legacy summary path,
            // and also cache the full IMU sample as its own scheduled downlink class.
            let mut snapshot = cache.vehicle_flight_snapshot_template(time_ms);
            snapshot.accel_mag_cms2 = accel_magnitude_cms2(imu.accel_mps2);
            cache.store_vehicle_flight_snapshot(snapshot);
            cache.store_vehicle_imu1_snapshot(imu_to_lora_imu1(imu.accel_mps2, imu.gyro_rps, time_ms));
            HostToRfDecision::Cached
        }
        hilink::MsgType::AuxImu => {
            let Ok(imu) = hilink::decode_payload::<hilink::AuxImuPayload>(packet) else {
                return HostToRfDecision::Drop;
            };
            cache.store_vehicle_imu2_snapshot(imu_to_lora_imu2(
                imu.accel_mps2,
                imu.gyro_rps,
                packet.header.send_time_ms,
            ));
            HostToRfDecision::Cached
        }
        hilink::MsgType::Baro => {
            let Ok(baro) = hilink::decode_payload::<hilink::BaroPayload>(packet) else {
                return HostToRfDecision::Drop;
            };
            let time_ms = packet.header.send_time_ms;
            let mut snapshot = cache.vehicle_flight_snapshot_template(time_ms);
            snapshot.altitude_dm = altitude_m_to_dm(baro.altitude_m);
            cache.store_vehicle_flight_snapshot(snapshot);
            cache.store_vehicle_baro_snapshot(baro_to_lora_baro(baro, time_ms));
            HostToRfDecision::Cached
        }
        hilink::MsgType::Mag => {
            let Ok(mag) = hilink::decode_payload::<hilink::MagPayload>(packet) else {
                return HostToRfDecision::Drop;
            };
            cache.store_vehicle_mag_snapshot(mag_to_lora_mag(mag, packet.header.send_time_ms));
            HostToRfDecision::Cached
        }
        hilink::MsgType::SystemState => {
            let Ok(state) = hilink::decode_payload::<hilink::SystemStatePayload>(packet) else {
                return HostToRfDecision::Drop;
            };
            cache.note_vehicle_system_state(
                state.system_state,
                telemetry_flags_to_fault_summary(state.flags),
                now_ms,
            );
            HostToRfDecision::Cached
        }
        hilink::MsgType::Gps => {
            let Ok(gps) = hilink::decode_payload::<hilink::GpsPayload>(packet) else {
                return HostToRfDecision::Drop;
            };
            let mut snapshot = cache.vehicle_flight_snapshot_template(packet.header.send_time_ms);
            snapshot.flags = lora_flags_with_gps_validity(snapshot.flags, gps.fix_type > 0);
            cache.store_vehicle_flight_snapshot(snapshot);
            cache.store_vehicle_gps_snapshot(gps_to_lora_gps(gps, packet.header.send_time_ms));
            HostToRfDecision::Cached
        }
        _ => HostToRfDecision::Drop,
    }
}

fn lora_command_ack(
    command: hilink::LoRaCommandPayload,
    status: u8,
    reason: u8,
) -> hilink::LoRaCommandAckPayload {
    hilink::LoRaCommandAckPayload {
        command_id: command.command_id,
        command_seq: command.command_seq,
        status,
        reason,
        state: 0,
        reserved: 0,
        detail: 0,
    }
}

fn translate_lora_command_ack_to_normal(
    packet: &rf::DecodedRfPacket<'_>,
    cache: &mut RadioStateCache,
    now_ms: u32,
) -> Option<HilinkBridgeFrame> {
    let ack = rf::decode_rf_payload::<hilink::LoRaCommandAckPayload>(packet).ok()?;

    // A SET_RADIO_PROFILE ack also drives the GS switch FSM: accept advances to the coordinated
    // apply, reject abandons it. (Retransmit-triggered duplicate acks are no-ops past the await.)
    if ack.command_id == hilink::lora_command_id::SET_RADIO_PROFILE {
        if policy::lora_ack_status_is_ack(ack.status) {
            cache.profile_switch_peer_accepted(ack.command_seq, now_ms);
        } else {
            cache.profile_switch_peer_rejected(ack.command_seq);
        }
    }

    let correlation = cache.take_command_correlation(ack.command_id, ack.command_seq)?;

    if policy::lora_ack_status_is_ack(ack.status) {
        if correlation.normal_msg_type == hilink::MsgType::Ping as u8 {
            return encode_normal_frame(&hilink::PongPayload, cache.next_normal_tx_seq(), now_ms);
        }

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
        accel_mag_cms2: telemetry.accel_mag_cms2,
        battery_mv: saturating_f32_to_u16(telemetry.battery_voltage_v * 1_000.0),
        pyro_or_actuator_flags: 0,
        fault_summary: saturating_u32_to_u16(telemetry_flags_to_fault_summary(telemetry.flags)),
    }
}

fn altitude_m_to_dm(altitude_m: f32) -> i32 {
    if !altitude_m.is_finite() {
        hilink::lora_scaling::ALTITUDE_INVALID_DM
    } else {
        saturating_f32_to_i32(altitude_m * 10.0)
    }
}

fn accel_magnitude_cms2(accel_mps2: [f32; 3]) -> u16 {
    let magnitude_mps2 = sqrt_f32(
        accel_mps2[0] * accel_mps2[0]
            + accel_mps2[1] * accel_mps2[1]
            + accel_mps2[2] * accel_mps2[2],
    );
    saturating_f32_to_u16(magnitude_mps2 * 100.0)
}

fn lora_flags_with_gps_validity(mut flags: u16, gps_valid: bool) -> u16 {
    if gps_valid {
        flags |= hilink::lora_flags::GPS_VALID;
    } else {
        flags &= !hilink::lora_flags::GPS_VALID;
    }
    flags
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
        flags: lora_flags_to_telemetry_flags(snapshot.flags),
        position_ned_m: [0.0, 0.0, -(snapshot.altitude_dm as f32) / 10.0],
        velocity_ned_mps: [0.0, 0.0, -(snapshot.vertical_velocity_cms as f32) / 100.0],
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        battery_voltage_v: (snapshot.battery_mv as f32) / 1_000.0,
        rssi_dbm: 0,
        snr_db_x100: 0,
        loss_pct_x100: 0,
        accel_mag_cms2: snapshot.accel_mag_cms2,
    }
}

/// Convert the radio's LoRa link metrics into a normal-HILink `RadioStatus` message so
/// the ground-station host (telemetry UI) sees RSSI / SNR / loss / rate. Previously these
/// were consumed locally and never reached the host.
fn lora_link_status_to_radio_status(
    link: hilink::LoRaLinkStatusPayload,
    active_preset: u8,
    active_frequency_hz: u32,
    active_tx_power_dbm: i8,
) -> hilink::RadioStatusPayload {
    let total = u32::from(link.rx_packets_delta) + u32::from(link.lost_packets_delta);
    let loss_pct_x100 = if total == 0 {
        0
    } else {
        saturating_u32_to_u16(u32::from(link.lost_packets_delta) * 10_000 / total)
    };
    hilink::RadioStatusPayload {
        rssi_dbm: i16::from(link.downlink_rssi_dbm),
        // RF carries SNR in 0.25 dB units (x4); RadioStatus uses 0.01 dB units (x100).
        snr_db_x100: i16::from(link.downlink_snr_x4) * 25,
        loss_pct_x100,
        packet_rate_hz: link.telemetry_rate_hz,
        reserved0: 0,
        active_preset,
        active_tx_power_dbm,
        active_frequency_hz,
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

const GRAVITY_MPS2: f32 = 9.806_65;
const RAD_PER_DEG: f32 = 0.017_453_292;
const DEG_PER_RAD: f32 = 57.295_78;

fn sim_stamp_from_ms(time_ms: u32) -> hilink::SimStamp {
    hilink::SimStamp {
        sim_tick: 0,
        sim_time_us: u64::from(time_ms) * 1_000,
    }
}

fn accel_mps2_to_cg(accel_mps2: [f32; 3]) -> [i16; 3] {
    [
        saturating_f32_to_i16(accel_mps2[0] / GRAVITY_MPS2 * 100.0),
        saturating_f32_to_i16(accel_mps2[1] / GRAVITY_MPS2 * 100.0),
        saturating_f32_to_i16(accel_mps2[2] / GRAVITY_MPS2 * 100.0),
    ]
}

fn cg_to_accel_mps2(accel_cg: [i16; 3]) -> [f32; 3] {
    [
        accel_cg[0] as f32 / 100.0 * GRAVITY_MPS2,
        accel_cg[1] as f32 / 100.0 * GRAVITY_MPS2,
        accel_cg[2] as f32 / 100.0 * GRAVITY_MPS2,
    ]
}

fn gyro_rps_to_ddps(gyro_rps: [f32; 3]) -> [i16; 3] {
    [
        saturating_f32_to_i16(gyro_rps[0] * DEG_PER_RAD * 10.0),
        saturating_f32_to_i16(gyro_rps[1] * DEG_PER_RAD * 10.0),
        saturating_f32_to_i16(gyro_rps[2] * DEG_PER_RAD * 10.0),
    ]
}

fn ddps_to_gyro_rps(gyro_ddps: [i16; 3]) -> [f32; 3] {
    [
        gyro_ddps[0] as f32 / 10.0 * RAD_PER_DEG,
        gyro_ddps[1] as f32 / 10.0 * RAD_PER_DEG,
        gyro_ddps[2] as f32 / 10.0 * RAD_PER_DEG,
    ]
}

fn field_ut_to_mgauss(field_ut: [f32; 3]) -> [i16; 3] {
    [
        saturating_f32_to_i16(field_ut[0] * 10.0),
        saturating_f32_to_i16(field_ut[1] * 10.0),
        saturating_f32_to_i16(field_ut[2] * 10.0),
    ]
}

fn mgauss_to_field_ut(field_mgauss: [i16; 3]) -> [f32; 3] {
    [
        field_mgauss[0] as f32 / 10.0,
        field_mgauss[1] as f32 / 10.0,
        field_mgauss[2] as f32 / 10.0,
    ]
}

fn imu_to_lora_imu1(
    accel_mps2: [f32; 3],
    gyro_rps: [f32; 3],
    time_ms: u32,
) -> hilink::LoRaImu1SnapshotPayload {
    hilink::LoRaImu1SnapshotPayload {
        time_ms,
        accel_cg: accel_mps2_to_cg(accel_mps2),
        gyro_ddps: gyro_rps_to_ddps(gyro_rps),
    }
}

fn imu_to_lora_imu2(
    accel_mps2: [f32; 3],
    gyro_rps: [f32; 3],
    time_ms: u32,
) -> hilink::LoRaImu2SnapshotPayload {
    hilink::LoRaImu2SnapshotPayload {
        time_ms,
        accel_cg: accel_mps2_to_cg(accel_mps2),
        gyro_ddps: gyro_rps_to_ddps(gyro_rps),
    }
}

fn mag_to_lora_mag(mag: hilink::MagPayload, time_ms: u32) -> hilink::LoRaMagSnapshotPayload {
    hilink::LoRaMagSnapshotPayload {
        time_ms,
        field_mgauss: field_ut_to_mgauss(mag.field_ut),
    }
}

fn baro_to_lora_baro(baro: hilink::BaroPayload, time_ms: u32) -> hilink::LoRaBaroSnapshotPayload {
    hilink::LoRaBaroSnapshotPayload {
        time_ms,
        pressure_pa: saturating_f32_to_u32(baro.pressure_pa),
        altitude_dm: altitude_m_to_dm(baro.altitude_m),
        temp_cc: saturating_f32_to_i16(baro.temperature_c * 100.0),
    }
}

fn lora_imu1_to_normal_imu(snapshot: hilink::LoRaImu1SnapshotPayload) -> hilink::ImuPayload {
    hilink::ImuPayload {
        stamp: sim_stamp_from_ms(snapshot.time_ms),
        accel_mps2: cg_to_accel_mps2(snapshot.accel_cg),
        gyro_rps: ddps_to_gyro_rps(snapshot.gyro_ddps),
    }
}

fn lora_imu2_to_normal_aux_imu(snapshot: hilink::LoRaImu2SnapshotPayload) -> hilink::AuxImuPayload {
    hilink::AuxImuPayload {
        stamp: sim_stamp_from_ms(snapshot.time_ms),
        accel_mps2: cg_to_accel_mps2(snapshot.accel_cg),
        gyro_rps: ddps_to_gyro_rps(snapshot.gyro_ddps),
    }
}

fn lora_mag_to_normal_mag(snapshot: hilink::LoRaMagSnapshotPayload) -> hilink::MagPayload {
    hilink::MagPayload {
        stamp: sim_stamp_from_ms(snapshot.time_ms),
        field_ut: mgauss_to_field_ut(snapshot.field_mgauss),
    }
}

fn lora_baro_to_normal_baro(snapshot: hilink::LoRaBaroSnapshotPayload) -> hilink::BaroPayload {
    let altitude_m = if snapshot.altitude_dm == hilink::lora_scaling::ALTITUDE_INVALID_DM {
        0.0
    } else {
        snapshot.altitude_dm as f32 / 10.0
    };
    hilink::BaroPayload {
        stamp: sim_stamp_from_ms(snapshot.time_ms),
        pressure_pa: snapshot.pressure_pa as f32,
        altitude_m,
        temperature_c: snapshot.temp_cc as f32 / 100.0,
    }
}

fn saturating_f32_to_u32(value: f32) -> u32 {
    if !value.is_finite() {
        0
    } else {
        round_f32(value).clamp(0.0, u32::MAX as f32) as u32
    }
}

fn telemetry_flags_to_lora_flags(flags: u32) -> u16 {
    let mut lora_flags = 0;
    if (flags & hilink::response_flags::ARMED) != 0 {
        lora_flags |= hilink::lora_flags::ARMED;
    }
    if (flags & hilink::response_flags::FAILSAFE) != 0 {
        lora_flags |= hilink::lora_flags::FAILSAFE;
    }
    if (flags & hilink::response_flags::ESTIMATOR_VALID) != 0 {
        lora_flags |= hilink::lora_flags::ESTIMATOR_VALID;
    }
    lora_flags
}

fn lora_flags_to_telemetry_flags(lora_flags: u16) -> u32 {
    let mut flags = 0;
    if (lora_flags & hilink::lora_flags::ARMED) != 0 {
        flags |= hilink::response_flags::ARMED;
    }
    if (lora_flags & hilink::lora_flags::FAILSAFE) != 0 {
        flags |= hilink::response_flags::FAILSAFE;
    }
    if (lora_flags & hilink::lora_flags::ESTIMATOR_VALID) != 0 {
        flags |= hilink::response_flags::ESTIMATOR_VALID;
    }
    flags
}

fn telemetry_flags_to_fault_summary(flags: u32) -> u32 {
    let mut faults = 0;
    if (flags & hilink::response_flags::FAILSAFE) != 0 {
        faults |= hilink::lora_fault::SAFETY_INHIBIT;
    }
    if (flags & hilink::response_flags::ESTIMATOR_VALID) == 0 {
        faults |= hilink::lora_fault::ESTIMATOR;
    }
    faults
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

fn saturating_u32_to_u16(value: u32) -> u16 {
    value.min(u16::MAX as u32) as u16
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
