//! Merges the differently-rated HILink messages (TelemetrySnapshot, Gps, RadioStatus,
//! Heartbeat, SystemState) into one coherent `TelemetryPacket`, and owns the fields that have
//! no direct wire source (max altitude, packet age/rate/loss, mission clock).

use std::time::Instant;

use crate::decode::{DecodedFrame, ParsedMessage};
use crate::model::{
    accel_g_from_cms2, altitude_from_ned, flight_stage, gps_is_valid, vertical_speed_from_ned,
    TelemetryPacket,
};

/// RadioStatus link metrics are authoritative for this long after arrival; afterwards we fall
/// back to whatever the telemetry snapshot carried.
const RADIO_STATUS_FRESH_MS: u128 = 3_000;
/// EWMA smoothing for the measured inter-arrival packet rate.
const RATE_EWMA_ALPHA: f32 = 0.2;
/// A header-seq jump larger than this means the stream reset (reconnect / new device); reset the
/// loss counters instead of reporting a huge spurious loss.
const SEQ_RESET_GAP: u16 = 1_000;

#[derive(Default)]
pub struct TelemetryAggregator {
    // Latest decoded values.
    system_state: u8,
    altitude_m: f32,
    max_altitude_m: f32,
    vertical_speed_mps: f32,
    accel_g: f32,
    battery_v: f32,
    gps_lat: f64,
    gps_lon: f64,
    gps_alt_m: f32,
    gps_valid: bool,
    gps_sats: u32,

    // Link metrics carried inside the telemetry snapshot (fallback source).
    snap_rssi_dbm: i32,
    snap_snr_db: f32,
    snap_loss_pct: f32,

    // Authoritative link metrics from the dedicated RadioStatus message.
    radio_rssi_dbm: i32,
    radio_snr_db: f32,
    radio_loss_pct: f32,
    radio_rate_hz: f32,
    radio_at: Option<Instant>,

    // Ground-station radio's active RF profile (from RadioStatus), for switch verification.
    radio_active_known: bool,
    radio_active_preset: u8,
    radio_active_tx_power_dbm: i8,
    radio_active_frequency_hz: u32,

    // Mission clock (from the FC stamp when present).
    mission_time_ms: u64,
    have_mission_time: bool,

    // Derived / stateful.
    seq_counter: u32,
    first_at: Option<Instant>,
    last_at: Option<Instant>,
    rate_ewma_hz: f32,
    last_header_seq: Option<u16>,
    received: u64,
    lost: u64,
}

impl TelemetryAggregator {
    pub fn new() -> Self {
        Self::default()
    }

    /// Clear all derived/stateful tracking (call on a fresh connection). Latest values are also
    /// reset so a new flight does not show stale numbers.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Have we received any telemetry-bearing frame yet?
    pub fn has_data(&self) -> bool {
        self.last_at.is_some()
    }

    /// Fold one decoded frame into the running state.
    pub fn ingest(&mut self, frame: &DecodedFrame, now: Instant) {
        self.update_timing(now);
        self.update_loss(frame.seq);
        self.seq_counter = self.seq_counter.wrapping_add(1);

        match &frame.message {
            ParsedMessage::Telemetry(t) => {
                self.system_state = t.system_state;
                self.altitude_m = altitude_from_ned(t.position_ned_m);
                self.max_altitude_m = self.max_altitude_m.max(self.altitude_m);
                self.vertical_speed_mps = vertical_speed_from_ned(t.velocity_ned_mps);
                self.accel_g = accel_g_from_cms2(t.accel_mag_cms2);
                self.battery_v = t.battery_voltage_v;
                self.snap_rssi_dbm = i32::from(t.rssi_dbm);
                self.snap_snr_db = f32::from(t.snr_db_x100) / 100.0;
                self.snap_loss_pct = f32::from(t.loss_pct_x100) / 100.0;
                self.update_mission_time(t.stamp.sim_time_us);
            }
            ParsedMessage::Gps(g) => {
                self.gps_lat = g.lat_deg;
                self.gps_lon = g.lon_deg;
                self.gps_alt_m = g.alt_msl_m;
                self.gps_sats = u32::from(g.sats);
                self.gps_valid = gps_is_valid(g.fix_type, g.sats);
                self.update_mission_time(g.stamp.sim_time_us);
            }
            ParsedMessage::Radio(r) => {
                self.radio_rssi_dbm = i32::from(r.rssi_dbm);
                self.radio_snr_db = f32::from(r.snr_db_x100) / 100.0;
                self.radio_loss_pct = f32::from(r.loss_pct_x100) / 100.0;
                self.radio_rate_hz = f32::from(r.packet_rate_hz);
                self.radio_at = Some(now);
                self.radio_active_preset = r.active_preset;
                self.radio_active_tx_power_dbm = r.active_tx_power_dbm;
                self.radio_active_frequency_hz = r.active_frequency_hz;
                self.radio_active_known = true;
            }
            ParsedMessage::Heartbeat(h) => {
                self.system_state = h.system_state;
                self.update_mission_time(h.stamp.sim_time_us);
            }
            ParsedMessage::SystemState(s) => {
                self.system_state = s.system_state;
                self.battery_v = s.battery_voltage_v;
                self.update_mission_time(s.stamp.sim_time_us);
            }
            // Raw sensor frames are surfaced in the PARSED debug console via `summary()`; they
            // are not yet folded into the structured dashboard model.
            ParsedMessage::Imu(_)
            | ParsedMessage::AuxImu(_)
            | ParsedMessage::Mag(_)
            | ParsedMessage::Baro(_)
            | ParsedMessage::Ack(_)
            | ParsedMessage::Nack(_)
            | ParsedMessage::Other(_) => {}
        }
    }

    fn update_timing(&mut self, now: Instant) {
        if let Some(last) = self.last_at {
            let dt_ms = now.duration_since(last).as_secs_f32() * 1000.0;
            if dt_ms > 0.0 {
                let inst_hz = 1000.0 / dt_ms;
                self.rate_ewma_hz = if self.rate_ewma_hz == 0.0 {
                    inst_hz
                } else {
                    RATE_EWMA_ALPHA * inst_hz + (1.0 - RATE_EWMA_ALPHA) * self.rate_ewma_hz
                };
            }
        }
        self.first_at.get_or_insert(now);
        self.last_at = Some(now);
    }

    fn update_loss(&mut self, header_seq: u16) {
        match self.last_header_seq {
            Some(prev) => {
                let gap = header_seq.wrapping_sub(prev);
                if gap == 0 {
                    // Duplicate seq — ignore.
                } else if gap > SEQ_RESET_GAP {
                    // Stream reset: restart counting from here.
                    self.received = 1;
                    self.lost = 0;
                } else {
                    self.received += 1;
                    self.lost += u64::from(gap - 1);
                }
            }
            None => self.received = 1,
        }
        self.last_header_seq = Some(header_seq);
    }

    fn update_mission_time(&mut self, sim_time_us: u64) {
        if sim_time_us > 0 {
            self.mission_time_ms = sim_time_us / 1_000;
            self.have_mission_time = true;
        }
    }

    fn radio_fresh(&self, now: Instant) -> bool {
        self.radio_at
            .is_some_and(|t| now.duration_since(t).as_millis() <= RADIO_STATUS_FRESH_MS)
    }

    /// Build the packet to push to the UI at the current instant.
    pub fn snapshot(&self, now: Instant) -> TelemetryPacket {
        let packet_age_ms = self
            .last_at
            .map(|t| now.duration_since(t).as_millis() as u64)
            .unwrap_or(0);

        let mission_time_ms = if self.have_mission_time {
            self.mission_time_ms
        } else {
            self.first_at
                .map(|t| now.duration_since(t).as_millis() as u64)
                .unwrap_or(0)
        };

        let radio_fresh = self.radio_fresh(now);
        let (rssi_dbm, snr_db) = if radio_fresh {
            (self.radio_rssi_dbm, self.radio_snr_db)
        } else {
            (self.snap_rssi_dbm, self.snap_snr_db)
        };

        let packet_loss_pct = if radio_fresh {
            self.radio_loss_pct
        } else {
            let total = self.received + self.lost;
            if total == 0 {
                self.snap_loss_pct
            } else {
                (self.lost as f32 / total as f32) * 100.0
            }
        };

        let packet_rate_hz = if radio_fresh && self.radio_rate_hz > 0.0 {
            self.radio_rate_hz
        } else {
            self.rate_ewma_hz
        };

        TelemetryPacket {
            seq: self.seq_counter,
            mission_time_ms,
            stage: flight_stage(self.system_state),
            altitude_m: self.altitude_m,
            max_altitude_m: self.max_altitude_m,
            vertical_speed_mps: self.vertical_speed_mps,
            accel_g: self.accel_g,
            battery_v: self.battery_v,
            gps_lat: self.gps_lat,
            gps_lon: self.gps_lon,
            gps_alt_m: self.gps_alt_m,
            gps_valid: self.gps_valid,
            gps_sats: self.gps_sats,
            rssi_dbm,
            snr_db,
            packet_age_ms,
            packet_rate_hz,
            packet_loss_pct,
            radio_active_known: self.radio_active_known,
            radio_active_preset: self.radio_active_preset,
            radio_active_tx_power_dbm: self.radio_active_tx_power_dbm,
            radio_active_frequency_hz: self.radio_active_frequency_hz,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decode::DecodedFrame;
    use marv_hilink::{flight_state, SimStamp, TelemetrySnapshotPayload};
    use std::time::Duration;

    fn telem_frame(seq: u16, altitude_m: f32, accel_cms2: u16) -> DecodedFrame {
        DecodedFrame {
            seq,
            message: ParsedMessage::Telemetry(TelemetrySnapshotPayload {
                stamp: SimStamp { sim_tick: 0, sim_time_us: 0 },
                system_state: flight_state::BOOST,
                reserved0: [0; 3],
                flags: 0,
                position_ned_m: [0.0, 0.0, -altitude_m],
                velocity_ned_mps: [0.0, 0.0, -5.0],
                attitude_quat: [1.0, 0.0, 0.0, 0.0],
                battery_voltage_v: 16.0,
                rssi_dbm: -50,
                snr_db_x100: 900,
                loss_pct_x100: 0,
                accel_mag_cms2: accel_cms2,
            }),
        }
    }

    #[test]
    fn tracks_max_altitude_and_converts_units() {
        let mut agg = TelemetryAggregator::new();
        let t0 = Instant::now();
        agg.ingest(&telem_frame(1, 100.0, 1962), t0);
        agg.ingest(&telem_frame(2, 250.0, 0), t0 + Duration::from_millis(100));
        agg.ingest(&telem_frame(3, 180.0, 0), t0 + Duration::from_millis(200));

        let snap = agg.snapshot(t0 + Duration::from_millis(200));
        assert_eq!(snap.stage, "BOOST");
        assert!((snap.altitude_m - 180.0).abs() < 0.01);
        assert!((snap.max_altitude_m - 250.0).abs() < 0.01);
        assert!((snap.vertical_speed_mps - 5.0).abs() < 0.01);
    }

    #[test]
    fn accel_cms2_converts_to_g() {
        let mut agg = TelemetryAggregator::new();
        // 1962 cm/s² ≈ 2 g.
        agg.ingest(&telem_frame(1, 0.0, 1962), Instant::now());
        let snap = agg.snapshot(Instant::now());
        assert!((snap.accel_g - 2.0).abs() < 0.02, "got {}", snap.accel_g);
    }

    #[test]
    fn computes_loss_from_seq_gaps() {
        let mut agg = TelemetryAggregator::new();
        let t0 = Instant::now();
        agg.ingest(&telem_frame(10, 0.0, 0), t0);
        // Skip seq 11 (one lost), arrive at 12.
        agg.ingest(&telem_frame(12, 0.0, 0), t0 + Duration::from_millis(100));
        agg.ingest(&telem_frame(13, 0.0, 0), t0 + Duration::from_millis(200));

        let snap = agg.snapshot(t0 + Duration::from_millis(200));
        // received 3 (12,13 increments... ) lost 1 over total -> 25%.
        assert!(snap.packet_loss_pct > 24.0 && snap.packet_loss_pct < 26.0, "got {}", snap.packet_loss_pct);
    }

    #[test]
    fn seq_wrap_does_not_spike_loss() {
        let mut agg = TelemetryAggregator::new();
        let t0 = Instant::now();
        agg.ingest(&telem_frame(u16::MAX - 1, 0.0, 0), t0);
        agg.ingest(&telem_frame(u16::MAX, 0.0, 0), t0 + Duration::from_millis(100));
        agg.ingest(&telem_frame(0, 0.0, 0), t0 + Duration::from_millis(200));
        agg.ingest(&telem_frame(1, 0.0, 0), t0 + Duration::from_millis(300));

        let snap = agg.snapshot(t0 + Duration::from_millis(300));
        assert!(snap.packet_loss_pct < 1.0, "wrap should not register loss, got {}", snap.packet_loss_pct);
    }
}
