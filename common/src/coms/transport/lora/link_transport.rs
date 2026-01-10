//! Link-level packet selection and ARQ; independent of RF/MAC timing.
use heapless::Vec;

use crate::protocol::packet::{
    CommandFrame, LinkStats, Packet, PacketType, TelemetryBaro, TelemetryGps, TelemetryImu,
    TelemetryMag, TelemetrySystem, MAX_PACKET_PAYLOAD,
};

pub const DEFAULT_ARQ_TIMEOUT_MS: u64 = 250;
pub const MAX_RC_BYTES: usize = 16;

const IMU_PERIOD_US: u64 = 1_000_000 / 150;
const BARO_PERIOD_US: u64 = 1_000_000 / 100;
const MAG_PERIOD_US: u64 = 1_000_000 / 50;
const GPS_PERIOD_US: u64 = 1_000_000 / 10;
const SYSTEM_PERIOD_US: u64 = 1_000_000 / 1;

const IMU_PAYLOAD_LEN: usize = 12;
const BARO_PAYLOAD_LEN: usize = 6;
const MAG_PAYLOAD_LEN: usize = 6;
const GPS_PAYLOAD_LEN: usize = 14;
const SYSTEM_PAYLOAD_LEN: usize = 5;

#[derive(Clone, Copy, Debug)]
enum TelemetryKind {
    Imu,
    Baro,
    Mag,
    Gps,
    System,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RxMeta {
    pub rssi: i16,
    pub snr: i16,
}

#[derive(Clone, Debug, Default)]
struct TelemetryState {
    last_imu_us: u64,
    last_baro_us: u64,
    last_mag_us: u64,
    last_gps_us: u64,
    last_system_us: u64,
    mock_seq: u32,
    last_imu: Option<TelemetryImu>,
    last_baro: Option<TelemetryBaro>,
    last_mag: Option<TelemetryMag>,
    last_gps: Option<TelemetryGps>,
    last_system: Option<TelemetrySystem>,
}

pub struct LoraTransport {
    pending_cmd: Option<CommandFrame>,
    awaiting_ack: bool,
    last_cmd_tx_ms: u64,
    next_cmd_seq: u8,
    arq_timeout_ms: u64,
    rc_latest: Option<Vec<u8, MAX_RC_BYTES>>,
    pending_ack: Option<u8>,
    last_stats: LinkStats,
    telemetry: TelemetryState,
}

impl LoraTransport {
    pub fn new(arq_timeout_ms: u64) -> Self {
        Self {
            pending_cmd: None,
            awaiting_ack: false,
            last_cmd_tx_ms: 0,
            next_cmd_seq: 0,
            arq_timeout_ms,
            rc_latest: None,
            pending_ack: None,
            last_stats: LinkStats {
                rssi: 0,
                snr: 0,
                lq: 0,
            },
            telemetry: TelemetryState::default(),
        }
    }

    pub fn new_default() -> Self {
        Self::new(DEFAULT_ARQ_TIMEOUT_MS)
    }

    pub fn note_link_stats(&mut self, meta: RxMeta) {
        self.last_stats = LinkStats {
            rssi: meta.rssi,
            snr: meta.snr,
            lq: self.last_stats.lq,
        };
    }

    pub fn set_lq(&mut self, lq: u8) {
        self.last_stats.lq = lq;
    }

    pub fn enqueue_command(&mut self, cmd_id: u8, payload: u32) -> bool {
        if self.pending_cmd.is_some() {
            return false;
        }
        let cmd = CommandFrame {
            seq: self.next_cmd_seq,
            cmd_id,
            payload,
        };
        self.next_cmd_seq = self.next_cmd_seq.wrapping_add(1);
        self.pending_cmd = Some(cmd);
        true
    }

    pub fn update_rc(&mut self, data: &[u8]) -> bool {
        let mut buf = Vec::new();
        if buf.extend_from_slice(data).is_err() {
            return false;
        }
        self.rc_latest = Some(buf);
        true
    }

    pub fn next_uplink(&mut self, now_ms: u64, max_len: usize) -> Option<Packet> {
        if let Some(cmd) = self.pending_cmd {
            if self.awaiting_ack {
                if now_ms.saturating_sub(self.last_cmd_tx_ms) >= self.arq_timeout_ms {
                    let packet = Packet::with_payload(PacketType::Command, cmd.encode());
                    if packet_fits(&packet, max_len) {
                        self.last_cmd_tx_ms = now_ms;
                        return Some(packet);
                    }
                }
            } else {
                let packet = Packet::with_payload(PacketType::Command, cmd.encode());
                if packet_fits(&packet, max_len) {
                    self.awaiting_ack = true;
                    self.last_cmd_tx_ms = now_ms;
                    return Some(packet);
                }
            }
        }

        if let Some(rc) = self.rc_latest.take() {
            let mut payload = Vec::<u8, MAX_PACKET_PAYLOAD>::new();
            if payload.extend_from_slice(rc.as_slice()).is_ok() {
                let packet = Packet::with_payload(PacketType::RcData, payload);
                if packet_fits(&packet, max_len) {
                    return Some(packet);
                }
            }
        }

        let packet = Packet::new(PacketType::KeepAlive);
        if packet_fits(&packet, max_len) {
            Some(packet)
        } else {
            None
        }
    }

    pub fn next_downlink(&mut self, now_us: u64, max_len: usize) -> Option<Packet> {
        if let Some(seq) = self.pending_ack.take() {
            let mut payload = Vec::new();
            let _ = payload.push(seq);
            let packet = Packet::with_payload(PacketType::Ack, payload);
            if packet_fits(&packet, max_len) {
                return Some(packet);
            }
            self.pending_ack = Some(seq);
        }

        if let Some(packet) = self.next_telemetry(now_us, max_len) {
            return Some(packet);
        }

        let packet = Packet::with_payload(PacketType::LinkStats, self.last_stats.encode());
        if packet_fits(&packet, max_len) {
            return Some(packet);
        }

        let packet = Packet::new(PacketType::KeepAlive);
        if packet_fits(&packet, max_len) {
            Some(packet)
        } else {
            None
        }
    }

    pub fn on_uplink_rx(&mut self, packet: &Packet, meta: RxMeta) {
        self.note_link_stats(meta);
        if packet.packet_type != PacketType::Command {
            return;
        }

        if let Ok(cmd) = CommandFrame::decode(packet.payload.as_slice()) {
            self.pending_ack = Some(cmd.seq);
        }
    }

    pub fn on_downlink_rx(&mut self, packet: &Packet, _meta: RxMeta) {
        match packet.packet_type {
            PacketType::Ack => {
                let Some(seq) = packet.payload.first().copied() else {
                    return;
                };
                let Some(pending) = self.pending_cmd else {
                    return;
                };
                if pending.seq == seq {
                    self.pending_cmd = None;
                    self.awaiting_ack = false;
                }
            }
            PacketType::LinkStats => {
                if let Ok(stats) = LinkStats::decode(packet.payload.as_slice()) {
                    self.last_stats = stats;
                }
            }
            PacketType::TelemetryImu => {
                if let Ok(imu) = TelemetryImu::decode(packet.payload.as_slice()) {
                    self.telemetry.last_imu = Some(imu);
                }
            }
            PacketType::TelemetryBaro => {
                if let Ok(baro) = TelemetryBaro::decode(packet.payload.as_slice()) {
                    self.telemetry.last_baro = Some(baro);
                }
            }
            PacketType::TelemetryMag => {
                if let Ok(mag) = TelemetryMag::decode(packet.payload.as_slice()) {
                    self.telemetry.last_mag = Some(mag);
                }
            }
            PacketType::TelemetryGps => {
                if let Ok(gps) = TelemetryGps::decode(packet.payload.as_slice()) {
                    self.telemetry.last_gps = Some(gps);
                }
            }
            PacketType::TelemetrySystem => {
                if let Ok(system) = TelemetrySystem::decode(packet.payload.as_slice()) {
                    self.telemetry.last_system = Some(system);
                }
            }
            _ => {}
        }
    }
}

impl LoraTransport {
    fn next_telemetry(&mut self, now_us: u64, max_len: usize) -> Option<Packet> {
        let kind = self.select_telemetry_kind(now_us, max_len)?;
        let packet = match kind {
            TelemetryKind::Imu => Packet::with_payload(
                PacketType::TelemetryImu,
                self.mock_imu().encode(),
            ),
            TelemetryKind::Baro => Packet::with_payload(
                PacketType::TelemetryBaro,
                self.mock_baro().encode(),
            ),
            TelemetryKind::Mag => Packet::with_payload(
                PacketType::TelemetryMag,
                self.mock_mag().encode(),
            ),
            TelemetryKind::Gps => Packet::with_payload(
                PacketType::TelemetryGps,
                self.mock_gps().encode(),
            ),
            TelemetryKind::System => Packet::with_payload(
                PacketType::TelemetrySystem,
                self.mock_system().encode(),
            ),
        };
        if packet_fits(&packet, max_len) {
            self.mark_telemetry_sent(kind, now_us);
            Some(packet)
        } else {
            None
        }
    }

    fn select_telemetry_kind(&self, now_us: u64, max_len: usize) -> Option<TelemetryKind> {
        let best: Option<(TelemetryKind, u64, u64)> = None;

        let best = pick_due(
            best,
            TelemetryKind::Imu,
            now_us,
            self.telemetry.last_imu_us,
            IMU_PERIOD_US,
            IMU_PAYLOAD_LEN,
            max_len,
        );
        let best = pick_due(
            best,
            TelemetryKind::Baro,
            now_us,
            self.telemetry.last_baro_us,
            BARO_PERIOD_US,
            BARO_PAYLOAD_LEN,
            max_len,
        );
        let best = pick_due(
            best,
            TelemetryKind::Mag,
            now_us,
            self.telemetry.last_mag_us,
            MAG_PERIOD_US,
            MAG_PAYLOAD_LEN,
            max_len,
        );
        let best = pick_due(
            best,
            TelemetryKind::Gps,
            now_us,
            self.telemetry.last_gps_us,
            GPS_PERIOD_US,
            GPS_PAYLOAD_LEN,
            max_len,
        );
        let best = pick_due(
            best,
            TelemetryKind::System,
            now_us,
            self.telemetry.last_system_us,
            SYSTEM_PERIOD_US,
            SYSTEM_PAYLOAD_LEN,
            max_len,
        );

        best.map(|(kind, _, _)| kind)
    }

    fn mark_telemetry_sent(&mut self, kind: TelemetryKind, now_us: u64) {
        match kind {
            TelemetryKind::Imu => self.telemetry.last_imu_us = now_us,
            TelemetryKind::Baro => self.telemetry.last_baro_us = now_us,
            TelemetryKind::Mag => self.telemetry.last_mag_us = now_us,
            TelemetryKind::Gps => self.telemetry.last_gps_us = now_us,
            TelemetryKind::System => self.telemetry.last_system_us = now_us,
        }
    }

    fn next_mock_seed(&mut self) -> u32 {
        let next = self.telemetry.mock_seq.wrapping_add(1);
        self.telemetry.mock_seq = next;
        next
    }

    fn mock_imu(&mut self) -> TelemetryImu {
        let seed = self.next_mock_seed() as i16;
        TelemetryImu {
            accel: [seed, seed.wrapping_add(100), seed.wrapping_sub(100)],
            gyro: [
                seed.wrapping_add(200),
                seed.wrapping_add(300),
                seed.wrapping_add(400),
            ],
        }
    }

    fn mock_baro(&mut self) -> TelemetryBaro {
        let seed = self.next_mock_seed();
        TelemetryBaro {
            pressure_pa: 101_325 + (seed as i32 % 2000),
            temp_c_x10: 250 + (seed as i16 % 50),
        }
    }

    fn mock_mag(&mut self) -> TelemetryMag {
        let seed = self.next_mock_seed() as i16;
        TelemetryMag {
            mag: [seed, seed.wrapping_add(50), seed.wrapping_sub(50)],
        }
    }

    fn mock_gps(&mut self) -> TelemetryGps {
        let seed = self.next_mock_seed() as i32;
        TelemetryGps {
            lat: 37_774_9000 + (seed % 1000),
            lon: -122_419_4000 + (seed % 1000),
            alt_mm: 15_000 + (seed % 500),
            sats: 10,
            fix: 3,
        }
    }

    fn mock_system(&mut self) -> TelemetrySystem {
        let seed = self.next_mock_seed();
        TelemetrySystem {
            vbat_mv: 12_000 + (seed as u16 % 500),
            temp_c: (25 + (seed as i8 % 5)),
            arm_status: (seed as u8) & 0x01,
            rssi_uplink: clamp_i16_to_i8(self.last_stats.rssi),
        }
    }
}

fn pick_due(
    best: Option<(TelemetryKind, u64, u64)>,
    kind: TelemetryKind,
    now_us: u64,
    last_us: u64,
    period_us: u64,
    payload_len: usize,
    max_len: usize,
) -> Option<(TelemetryKind, u64, u64)> {
    if !payload_fits(max_len, payload_len) {
        return best;
    }
    let overdue = now_us.saturating_sub(last_us);
    if overdue < period_us {
        return best;
    }
    match best {
        None => Some((kind, overdue, period_us)),
        Some((best_kind, best_overdue, best_period)) => {
            if overdue > best_overdue
                || (overdue == best_overdue && period_us > best_period)
            {
                Some((kind, overdue, period_us))
            } else {
                Some((best_kind, best_overdue, best_period))
            }
        }
    }
}

fn payload_fits(max_len: usize, payload_len: usize) -> bool {
    if max_len == 0 {
        return false;
    }
    1usize.saturating_add(payload_len) <= max_len
}

fn clamp_i16_to_i8(value: i16) -> i8 {
    if value > i8::MAX as i16 {
        i8::MAX
    } else if value < i8::MIN as i16 {
        i8::MIN
    } else {
        value as i8
    }
}

fn packet_fits(packet: &Packet, max_len: usize) -> bool {
    if max_len == 0 {
        return packet.packet_type == PacketType::KeepAlive;
    }

    let needed = 1usize.saturating_add(packet.payload.len());
    needed <= max_len
}

impl Default for LoraTransport {
    fn default() -> Self {
        Self::new(DEFAULT_ARQ_TIMEOUT_MS)
    }
}
