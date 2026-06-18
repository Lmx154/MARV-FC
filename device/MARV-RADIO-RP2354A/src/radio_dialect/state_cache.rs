use common::comms::links::lora::stats::LoraLinkStats;
use common::protocol::hilink;

use crate::config;
use crate::radio_dialect::profile_switch::{ProfileChange, ProfileSwitch, SwitchAction, VehicleArm};

/// Generate the latest-cache `store` / `due` / `note_sent` accessor trio for a periodic
/// vehicle sensor snapshot, mirroring the hand-written flight/gps snapshot methods.
macro_rules! vehicle_snapshot_accessors {
    ($ty:ty, $latest:ident, $dirty:ident, $sent_once:ident, $last_sent:ident,
     $store:ident, $due:ident, $note:ident) => {
        pub fn $store(&mut self, snapshot: $ty) {
            self.$latest = Some(snapshot);
            self.$dirty = true;
        }

        pub fn $due(&self, now_ms: u32, period_ms: u32) -> Option<$ty> {
            if !self.$dirty {
                return None;
            }
            if self.$sent_once && now_ms.wrapping_sub(self.$last_sent) < period_ms {
                return None;
            }
            self.$latest
        }

        pub fn $note(&mut self, now_ms: u32) {
            self.$dirty = false;
            self.$sent_once = true;
            self.$last_sent = now_ms;
        }
    };
}

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
    latest_vehicle_imu1_snapshot: Option<hilink::LoRaImu1SnapshotPayload>,
    vehicle_imu1_snapshot_dirty: bool,
    vehicle_imu1_snapshot_sent_once: bool,
    vehicle_imu1_snapshot_last_sent_ms: u32,
    latest_vehicle_imu2_snapshot: Option<hilink::LoRaImu2SnapshotPayload>,
    vehicle_imu2_snapshot_dirty: bool,
    vehicle_imu2_snapshot_sent_once: bool,
    vehicle_imu2_snapshot_last_sent_ms: u32,
    latest_vehicle_mag_snapshot: Option<hilink::LoRaMagSnapshotPayload>,
    vehicle_mag_snapshot_dirty: bool,
    vehicle_mag_snapshot_sent_once: bool,
    vehicle_mag_snapshot_last_sent_ms: u32,
    latest_vehicle_baro_snapshot: Option<hilink::LoRaBaroSnapshotPayload>,
    vehicle_baro_snapshot_dirty: bool,
    vehicle_baro_snapshot_sent_once: bool,
    vehicle_baro_snapshot_last_sent_ms: u32,
    latest_lora_link_status: Option<hilink::LoRaLinkStatusPayload>,
    lora_link_status_dirty: bool,
    lora_link_status_sent_once: bool,
    lora_link_status_last_sent_ms: u32,
    last_link_status_tx_packets: u32,
    last_link_status_rx_packets: u32,
    last_link_status_lost_packets: u32,
    vehicle_system_state: Option<u8>,
    vehicle_fault_summary: Option<u32>,
    profile_switch: ProfileSwitch,
    local_active_preset: u8,
    local_active_frequency_hz: u32,
    local_active_tx_power_dbm: i8,
    idle_fallback_ms: u32,
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
            latest_vehicle_imu1_snapshot: None,
            vehicle_imu1_snapshot_dirty: false,
            vehicle_imu1_snapshot_sent_once: false,
            vehicle_imu1_snapshot_last_sent_ms: 0,
            latest_vehicle_imu2_snapshot: None,
            vehicle_imu2_snapshot_dirty: false,
            vehicle_imu2_snapshot_sent_once: false,
            vehicle_imu2_snapshot_last_sent_ms: 0,
            latest_vehicle_mag_snapshot: None,
            vehicle_mag_snapshot_dirty: false,
            vehicle_mag_snapshot_sent_once: false,
            vehicle_mag_snapshot_last_sent_ms: 0,
            latest_vehicle_baro_snapshot: None,
            vehicle_baro_snapshot_dirty: false,
            vehicle_baro_snapshot_sent_once: false,
            vehicle_baro_snapshot_last_sent_ms: 0,
            latest_lora_link_status: None,
            lora_link_status_dirty: false,
            lora_link_status_sent_once: false,
            lora_link_status_last_sent_ms: 0,
            last_link_status_tx_packets: 0,
            last_link_status_rx_packets: 0,
            last_link_status_lost_packets: 0,
            vehicle_system_state: None,
            vehicle_fault_summary: None,
            profile_switch: ProfileSwitch::new(),
            local_active_preset: hilink::band_plan::UNKNOWN_PRESET,
            local_active_frequency_hz: 0,
            local_active_tx_power_dbm: 0,
            idle_fallback_ms: config::DEFAULT_IDLE_FALLBACK_MS,
        }
    }

    // --- Local active RF profile (reported to the host via RadioStatus for verification) ---

    /// Record the radio's live RF profile. Set at link bring-up and after every profile switch so
    /// the host's RadioStatus reflects exactly what this radio is transmitting on.
    pub fn set_local_active_profile(&mut self, preset: u8, frequency_hz: u32, tx_power_dbm: i8) {
        self.local_active_preset = preset;
        self.local_active_frequency_hz = frequency_hz;
        self.local_active_tx_power_dbm = tx_power_dbm;
    }

    pub fn local_active_preset(&self) -> u8 {
        self.local_active_preset
    }

    pub fn local_active_frequency_hz(&self) -> u32 {
        self.local_active_frequency_hz
    }

    pub fn local_active_tx_power_dbm(&self) -> i8 {
        self.local_active_tx_power_dbm
    }

    // --- Idle fallback to the boot/"setup" profile (see `hilink::idle_fallback`) ---

    /// Window (ms) this radio waits, hearing nothing from its peer, before re-homing to the setup
    /// profile. Read each bridge loop so an operator change takes effect without a restart.
    pub fn idle_fallback_ms(&self) -> u32 {
        self.idle_fallback_ms
    }

    /// Set the idle-fallback window, clamped to the legal range. Driven by the host `SetIdleFallback`
    /// command on the GS, and by the relayed `SET_IDLE_FALLBACK` LoRa command on the vehicle, so
    /// both ends share the operator's value.
    pub fn set_idle_fallback_ms(&mut self, idle_fallback_ms: u32) {
        self.idle_fallback_ms = hilink::idle_fallback::clamp(idle_fallback_ms);
    }

    // --- Coordinated RF profile switch (see `radio_dialect::profile_switch`) ---

    pub fn profile_switch_active(&self) -> bool {
        self.profile_switch.is_active()
    }

    /// GS: arm a switch after a validated host request. Returns `false` if one is already running.
    pub fn begin_gs_profile_switch(&mut self, change: ProfileChange, now_ms: u32) -> bool {
        self.profile_switch.begin_gs(change, now_ms)
    }

    pub fn profile_switch_peer_accepted(&mut self, command_seq: u16, now_ms: u32) {
        self.profile_switch.peer_accepted(command_seq, now_ms);
    }

    pub fn profile_switch_peer_rejected(&mut self, command_seq: u16) {
        self.profile_switch.peer_rejected(command_seq);
    }

    /// Vehicle: validate + de-duplicate a `LoRaSetProfile`, arming the switch when fresh.
    pub fn begin_vehicle_profile_switch(
        &mut self,
        change: ProfileChange,
        now_ms: u32,
    ) -> VehicleArm {
        if change.to_profile().is_none() {
            return VehicleArm::Rejected;
        }
        if self.vehicle_command_is_duplicate(
            hilink::lora_command_id::SET_RADIO_PROFILE,
            change.command_seq,
        ) {
            return VehicleArm::DuplicateAccepted;
        }
        if self.profile_switch.is_active() {
            return VehicleArm::Busy;
        }
        self.note_vehicle_command_forwarded(
            hilink::lora_command_id::SET_RADIO_PROFILE,
            change.command_seq,
        );
        self.profile_switch.arm_vehicle(change, now_ms);
        VehicleArm::Accepted
    }

    pub fn take_profile_switch_retransmit(&mut self, now_ms: u32) -> Option<ProfileChange> {
        self.profile_switch.take_retransmit(now_ms)
    }

    pub fn note_switch_peer_seen(&mut self) {
        self.profile_switch.note_peer_seen();
    }

    pub fn poll_profile_switch(&mut self, now_ms: u32) -> SwitchAction {
        self.profile_switch.poll(now_ms)
    }

    pub fn abort_profile_switch(&mut self) {
        self.profile_switch.abort();
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

        if let Some(mut snapshot) = self.latest_vehicle_flight_snapshot {
            snapshot.time_ms = now_ms;
            snapshot.state = state;
            snapshot.fault_summary = saturating_u32_to_u16(fault_summary);
            self.store_vehicle_flight_snapshot(snapshot);
        }
    }

    pub fn vehicle_flight_snapshot_template(
        &self,
        time_ms: u32,
    ) -> hilink::LoRaFlightSnapshotPayload {
        let mut snapshot =
            self.latest_vehicle_flight_snapshot
                .unwrap_or(hilink::LoRaFlightSnapshotPayload {
                    time_ms,
                    state: hilink::lora_state::UNKNOWN,
                    mode: hilink::lora_mode::UNKNOWN,
                    flags: 0,
                    altitude_dm: hilink::lora_scaling::ALTITUDE_INVALID_DM,
                    vertical_velocity_cms: 0,
                    accel_mag_cms2: 0,
                    battery_mv: 0,
                    pyro_or_actuator_flags: 0,
                    fault_summary: 0,
                });

        snapshot.time_ms = time_ms;
        if let Some(state) = self.vehicle_system_state {
            snapshot.state = state;
        }
        if let Some(fault_summary) = self.vehicle_fault_summary {
            snapshot.fault_summary = saturating_u32_to_u16(fault_summary);
        }
        snapshot
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

    vehicle_snapshot_accessors!(
        hilink::LoRaImu1SnapshotPayload,
        latest_vehicle_imu1_snapshot,
        vehicle_imu1_snapshot_dirty,
        vehicle_imu1_snapshot_sent_once,
        vehicle_imu1_snapshot_last_sent_ms,
        store_vehicle_imu1_snapshot,
        due_vehicle_imu1_snapshot,
        note_vehicle_imu1_snapshot_sent
    );

    vehicle_snapshot_accessors!(
        hilink::LoRaImu2SnapshotPayload,
        latest_vehicle_imu2_snapshot,
        vehicle_imu2_snapshot_dirty,
        vehicle_imu2_snapshot_sent_once,
        vehicle_imu2_snapshot_last_sent_ms,
        store_vehicle_imu2_snapshot,
        due_vehicle_imu2_snapshot,
        note_vehicle_imu2_snapshot_sent
    );

    vehicle_snapshot_accessors!(
        hilink::LoRaMagSnapshotPayload,
        latest_vehicle_mag_snapshot,
        vehicle_mag_snapshot_dirty,
        vehicle_mag_snapshot_sent_once,
        vehicle_mag_snapshot_last_sent_ms,
        store_vehicle_mag_snapshot,
        due_vehicle_mag_snapshot,
        note_vehicle_mag_snapshot_sent
    );

    vehicle_snapshot_accessors!(
        hilink::LoRaBaroSnapshotPayload,
        latest_vehicle_baro_snapshot,
        vehicle_baro_snapshot_dirty,
        vehicle_baro_snapshot_sent_once,
        vehicle_baro_snapshot_last_sent_ms,
        store_vehicle_baro_snapshot,
        due_vehicle_baro_snapshot,
        note_vehicle_baro_snapshot_sent
    );

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
