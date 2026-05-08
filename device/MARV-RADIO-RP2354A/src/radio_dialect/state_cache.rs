use common::comms::links::lora::stats::LoraLinkStats;
use common::protocol::hilink;

pub const COMMAND_CORRELATION_DEPTH: usize = 16;
pub const VEHICLE_COMMAND_HISTORY_DEPTH: usize = 8;
pub const PENDING_LORA_COMMAND_ACK_DEPTH: usize = 4;
pub const PENDING_LORA_EVENT_DEPTH: usize = crate::radio_dialect::policy::PENDING_LORA_EVENT_DEPTH;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandCorrelation {
    pub normal_seq: u16,
    pub normal_msg_type: u8,
    pub command_seq: u16,
    pub command_id: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct VehicleCommandRecord {
    pub command_seq: u16,
    pub command_id: u16,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RadioStateCache {
    next_rf_command_seq: u16,
    next_normal_tx_seq: u16,
    correlation_cursor: usize,
    command_correlations: [Option<CommandCorrelation>; COMMAND_CORRELATION_DEPTH],
    vehicle_command_cursor: usize,
    vehicle_command_history: [Option<VehicleCommandRecord>; VEHICLE_COMMAND_HISTORY_DEPTH],
    pending_lora_command_ack_cursor: usize,
    pending_lora_command_acks:
        [Option<hilink::LoRaCommandAckPayload>; PENDING_LORA_COMMAND_ACK_DEPTH],
    pending_lora_event_cursor: usize,
    pending_lora_events: [Option<hilink::LoRaEventPayload>; PENDING_LORA_EVENT_DEPTH],
    latest_lora_faults: Option<hilink::LoRaFaultsPayload>,
    lora_faults_dirty: bool,
    latest_vehicle_flight_snapshot: Option<hilink::LoRaFlightSnapshotPayload>,
    vehicle_flight_snapshot_dirty: bool,
    vehicle_flight_snapshot_sent_once: bool,
    vehicle_flight_snapshot_last_sent_ms: u32,
    latest_vehicle_gps_snapshot: Option<hilink::LoRaGpsSnapshotPayload>,
    vehicle_gps_snapshot_dirty: bool,
    vehicle_gps_snapshot_sent_once: bool,
    vehicle_gps_snapshot_last_sent_ms: u32,
    latest_lora_link_status: Option<hilink::LoRaLinkStatusPayload>,
    lora_link_status_dirty: bool,
    lora_link_status_sent_once: bool,
    lora_link_status_last_sent_ms: u32,
    last_link_status_tx_packets: u32,
    last_link_status_rx_packets: u32,
    last_link_status_lost_packets: u32,
    vehicle_system_state: Option<u8>,
    vehicle_fault_summary: Option<u32>,
}

impl RadioStateCache {
    pub const fn new() -> Self {
        Self {
            next_rf_command_seq: 0,
            next_normal_tx_seq: 0,
            correlation_cursor: 0,
            command_correlations: [None; COMMAND_CORRELATION_DEPTH],
            vehicle_command_cursor: 0,
            vehicle_command_history: [None; VEHICLE_COMMAND_HISTORY_DEPTH],
            pending_lora_command_ack_cursor: 0,
            pending_lora_command_acks: [None; PENDING_LORA_COMMAND_ACK_DEPTH],
            pending_lora_event_cursor: 0,
            pending_lora_events: [None; PENDING_LORA_EVENT_DEPTH],
            latest_lora_faults: None,
            lora_faults_dirty: false,
            latest_vehicle_flight_snapshot: None,
            vehicle_flight_snapshot_dirty: false,
            vehicle_flight_snapshot_sent_once: false,
            vehicle_flight_snapshot_last_sent_ms: 0,
            latest_vehicle_gps_snapshot: None,
            vehicle_gps_snapshot_dirty: false,
            vehicle_gps_snapshot_sent_once: false,
            vehicle_gps_snapshot_last_sent_ms: 0,
            latest_lora_link_status: None,
            lora_link_status_dirty: false,
            lora_link_status_sent_once: false,
            lora_link_status_last_sent_ms: 0,
            last_link_status_tx_packets: 0,
            last_link_status_rx_packets: 0,
            last_link_status_lost_packets: 0,
            vehicle_system_state: None,
            vehicle_fault_summary: None,
        }
    }

    pub fn next_rf_command_seq(&mut self) -> u16 {
        let seq = self.next_rf_command_seq;
        self.next_rf_command_seq = self.next_rf_command_seq.wrapping_add(1);
        seq
    }

    pub fn next_normal_tx_seq(&mut self) -> u16 {
        let seq = self.next_normal_tx_seq;
        self.next_normal_tx_seq = self.next_normal_tx_seq.wrapping_add(1);
        seq
    }

    pub fn store_command_correlation(
        &mut self,
        normal_header: hilink::Header,
        command_seq: u16,
        command_id: u16,
    ) {
        self.command_correlations[self.correlation_cursor] = Some(CommandCorrelation {
            normal_seq: normal_header.seq,
            normal_msg_type: normal_header.msg_type,
            command_seq,
            command_id,
        });
        self.correlation_cursor = (self.correlation_cursor + 1) % COMMAND_CORRELATION_DEPTH;
    }

    pub fn take_command_correlation(
        &mut self,
        command_id: u16,
        command_seq: u16,
    ) -> Option<CommandCorrelation> {
        for correlation in self.command_correlations.iter_mut() {
            let Some(stored) = *correlation else {
                continue;
            };
            if stored.command_id == command_id && stored.command_seq == command_seq {
                *correlation = None;
                return Some(stored);
            }
        }

        None
    }

    pub fn take_normal_correlation(
        &mut self,
        normal_seq: u16,
        normal_msg_type: u8,
    ) -> Option<CommandCorrelation> {
        for correlation in self.command_correlations.iter_mut() {
            let Some(stored) = *correlation else {
                continue;
            };
            if stored.normal_seq == normal_seq && stored.normal_msg_type == normal_msg_type {
                *correlation = None;
                return Some(stored);
            }
        }

        None
    }

    pub fn take_first_normal_correlation_for_msg_type(
        &mut self,
        normal_msg_type: hilink::MsgType,
    ) -> Option<CommandCorrelation> {
        let normal_msg_type = normal_msg_type as u8;
        for correlation in self.command_correlations.iter_mut() {
            let Some(stored) = *correlation else {
                continue;
            };
            if stored.normal_msg_type == normal_msg_type {
                *correlation = None;
                return Some(stored);
            }
        }

        None
    }

    pub fn vehicle_command_is_duplicate(&self, command_id: u16, command_seq: u16) -> bool {
        self.vehicle_command_history.iter().any(|record| {
            record.is_some_and(|stored| {
                stored.command_id == command_id && stored.command_seq == command_seq
            })
        })
    }

    pub fn note_vehicle_command_forwarded(&mut self, command_id: u16, command_seq: u16) {
        self.vehicle_command_history[self.vehicle_command_cursor] = Some(VehicleCommandRecord {
            command_id,
            command_seq,
        });
        self.vehicle_command_cursor =
            (self.vehicle_command_cursor + 1) % VEHICLE_COMMAND_HISTORY_DEPTH;
    }

    pub fn queue_pending_lora_command_ack(&mut self, ack: hilink::LoRaCommandAckPayload) {
        if let Some(slot) = self
            .pending_lora_command_acks
            .iter_mut()
            .find(|slot| slot.is_none())
        {
            *slot = Some(ack);
            return;
        }

        self.pending_lora_command_acks[self.pending_lora_command_ack_cursor] = Some(ack);
        self.pending_lora_command_ack_cursor =
            (self.pending_lora_command_ack_cursor + 1) % PENDING_LORA_COMMAND_ACK_DEPTH;
    }

    pub fn pop_pending_lora_command_ack(&mut self) -> Option<hilink::LoRaCommandAckPayload> {
        for slot in self.pending_lora_command_acks.iter_mut() {
            if slot.is_some() {
                return slot.take();
            }
        }

        None
    }

    pub fn queue_pending_lora_event(&mut self, event: hilink::LoRaEventPayload) {
        if let Some(slot) = self
            .pending_lora_events
            .iter_mut()
            .find(|slot| slot.is_none())
        {
            *slot = Some(event);
            return;
        }

        self.pending_lora_events[self.pending_lora_event_cursor] = Some(event);
        self.pending_lora_event_cursor =
            (self.pending_lora_event_cursor + 1) % PENDING_LORA_EVENT_DEPTH;
    }

    pub fn pop_pending_lora_event(&mut self) -> Option<hilink::LoRaEventPayload> {
        for slot in self.pending_lora_events.iter_mut() {
            if slot.is_some() {
                return slot.take();
            }
        }

        None
    }

    pub fn store_lora_faults(&mut self, faults: hilink::LoRaFaultsPayload) {
        if self.latest_lora_faults != Some(faults) {
            self.latest_lora_faults = Some(faults);
            self.lora_faults_dirty = true;
        }
    }

    pub fn take_dirty_lora_faults(&mut self) -> Option<hilink::LoRaFaultsPayload> {
        if !self.lora_faults_dirty {
            return None;
        }

        self.lora_faults_dirty = false;
        self.latest_lora_faults
    }

    pub fn note_vehicle_system_state(&mut self, state: u8, fault_summary: u32, now_ms: u32) {
        if let Some(previous) = self.vehicle_system_state {
            if previous != state {
                self.queue_pending_lora_event(hilink::LoRaEventPayload {
                    time_ms: now_ms,
                    event_id: hilink::lora_event_id::STATE_CHANGE,
                    severity: hilink::lora_event_severity::INFO,
                    arg0: previous as i32,
                    arg1: state as i32,
                });
            }
        }
        self.vehicle_system_state = Some(state);

        if self.vehicle_fault_summary != Some(fault_summary) {
            self.store_lora_faults(hilink::LoRaFaultsPayload {
                time_ms: now_ms,
                active_faults: fault_summary,
                latched_faults: fault_summary,
                inhibit_flags: 0,
            });
        }
        self.vehicle_fault_summary = Some(fault_summary);
    }

    pub fn store_vehicle_flight_snapshot(&mut self, snapshot: hilink::LoRaFlightSnapshotPayload) {
        self.latest_vehicle_flight_snapshot = Some(snapshot);
        self.vehicle_flight_snapshot_dirty = true;
    }

    pub fn due_vehicle_flight_snapshot(
        &self,
        now_ms: u32,
        period_ms: u32,
    ) -> Option<hilink::LoRaFlightSnapshotPayload> {
        if !self.vehicle_flight_snapshot_dirty {
            return None;
        }

        if self.vehicle_flight_snapshot_sent_once
            && now_ms.wrapping_sub(self.vehicle_flight_snapshot_last_sent_ms) < period_ms
        {
            return None;
        }

        self.latest_vehicle_flight_snapshot
    }

    pub fn note_vehicle_flight_snapshot_sent(&mut self, now_ms: u32) {
        self.vehicle_flight_snapshot_dirty = false;
        self.vehicle_flight_snapshot_sent_once = true;
        self.vehicle_flight_snapshot_last_sent_ms = now_ms;
    }

    pub fn store_vehicle_gps_snapshot(&mut self, snapshot: hilink::LoRaGpsSnapshotPayload) {
        self.latest_vehicle_gps_snapshot = Some(snapshot);
        self.vehicle_gps_snapshot_dirty = true;
    }

    pub fn due_vehicle_gps_snapshot(
        &self,
        now_ms: u32,
        period_ms: u32,
    ) -> Option<hilink::LoRaGpsSnapshotPayload> {
        if !self.vehicle_gps_snapshot_dirty {
            return None;
        }

        if self.vehicle_gps_snapshot_sent_once
            && now_ms.wrapping_sub(self.vehicle_gps_snapshot_last_sent_ms) < period_ms
        {
            return None;
        }

        self.latest_vehicle_gps_snapshot
    }

    pub fn note_vehicle_gps_snapshot_sent(&mut self, now_ms: u32) {
        self.vehicle_gps_snapshot_dirty = false;
        self.vehicle_gps_snapshot_sent_once = true;
        self.vehicle_gps_snapshot_last_sent_ms = now_ms;
    }

    pub fn refresh_lora_link_status(
        &mut self,
        now_ms: u32,
        period_ms: u32,
        stats: &LoraLinkStats,
        active_profile: u8,
        telemetry_rate_hz: u8,
    ) {
        if self.lora_link_status_sent_once
            && now_ms.wrapping_sub(self.lora_link_status_last_sent_ms) < period_ms
        {
            return;
        }

        let lost_packets = stats
            .missed_peer_packets
            .wrapping_add(stats.rx_errors)
            .wrapping_add(stats.malformed_frames);
        self.latest_lora_link_status = Some(hilink::LoRaLinkStatusPayload {
            time_ms: now_ms,
            uplink_rssi_dbm: saturating_i16_to_i8(stats.last_rssi),
            uplink_snr_x4: saturating_i16_to_i8(stats.last_snr_x4),
            downlink_rssi_dbm: 0,
            downlink_snr_x4: 0,
            rx_packets_delta: saturating_u32_to_u16(
                stats
                    .rx_packets
                    .wrapping_sub(self.last_link_status_rx_packets),
            ),
            tx_packets_delta: saturating_u32_to_u16(
                stats
                    .tx_packets
                    .wrapping_sub(self.last_link_status_tx_packets),
            ),
            lost_packets_delta: saturating_u32_to_u16(
                lost_packets.wrapping_sub(self.last_link_status_lost_packets),
            ),
            active_profile,
            telemetry_rate_hz,
            reserved: 0,
        });
        self.last_link_status_rx_packets = stats.rx_packets;
        self.last_link_status_tx_packets = stats.tx_packets;
        self.last_link_status_lost_packets = lost_packets;
        self.lora_link_status_dirty = true;
    }

    pub fn due_lora_link_status(
        &self,
        now_ms: u32,
        period_ms: u32,
    ) -> Option<hilink::LoRaLinkStatusPayload> {
        if !self.lora_link_status_dirty {
            return None;
        }

        if self.lora_link_status_sent_once
            && now_ms.wrapping_sub(self.lora_link_status_last_sent_ms) < period_ms
        {
            return None;
        }

        self.latest_lora_link_status
    }

    pub fn note_lora_link_status_sent(&mut self, now_ms: u32) {
        self.lora_link_status_dirty = false;
        self.lora_link_status_sent_once = true;
        self.lora_link_status_last_sent_ms = now_ms;
    }
}

fn saturating_i16_to_i8(value: i16) -> i8 {
    value.clamp(i8::MIN as i16, i8::MAX as i16) as i8
}

fn saturating_u32_to_u16(value: u32) -> u16 {
    value.min(u16::MAX as u32) as u16
}

impl Default for RadioStateCache {
    fn default() -> Self {
        Self::new()
    }
}
