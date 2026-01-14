//! Link-level packet selection and ARQ; independent of RF/MAC timing.
use heapless::{Deque, Vec};

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
const TELEMETRY_QUEUE_LEN: usize = 32;
const TELEMETRY_BURST_HEADER_LEN: usize = 2;

#[derive(Clone, Copy, Debug)]
enum TelemetryKind {
    Imu,
    Baro,
    Mag,
    Gps,
    System,
}

impl TelemetryKind {
    const fn as_u8(self) -> u8 {
        match self {
            TelemetryKind::Imu => 0,
            TelemetryKind::Baro => 1,
            TelemetryKind::Mag => 2,
            TelemetryKind::Gps => 3,
            TelemetryKind::System => 4,
        }
    }

    const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(TelemetryKind::Imu),
            1 => Some(TelemetryKind::Baro),
            2 => Some(TelemetryKind::Mag),
            3 => Some(TelemetryKind::Gps),
            4 => Some(TelemetryKind::System),
            _ => None,
        }
    }

    const fn sample_len(self) -> usize {
        match self {
            TelemetryKind::Imu => IMU_PAYLOAD_LEN,
            TelemetryKind::Baro => BARO_PAYLOAD_LEN,
            TelemetryKind::Mag => MAG_PAYLOAD_LEN,
            TelemetryKind::Gps => GPS_PAYLOAD_LEN,
            TelemetryKind::System => SYSTEM_PAYLOAD_LEN,
        }
    }

}

#[derive(Debug)]
struct TelemetryQueues {
    imu: Deque<TelemetryImu, TELEMETRY_QUEUE_LEN>,
    baro: Deque<TelemetryBaro, TELEMETRY_QUEUE_LEN>,
    mag: Deque<TelemetryMag, TELEMETRY_QUEUE_LEN>,
    gps: Deque<TelemetryGps, TELEMETRY_QUEUE_LEN>,
    system: Deque<TelemetrySystem, TELEMETRY_QUEUE_LEN>,
    drops_imu: u32,
    drops_baro: u32,
    drops_mag: u32,
    drops_gps: u32,
    drops_system: u32,
}

impl Default for TelemetryQueues {
    fn default() -> Self {
        Self {
            imu: Deque::new(),
            baro: Deque::new(),
            mag: Deque::new(),
            gps: Deque::new(),
            system: Deque::new(),
            drops_imu: 0,
            drops_baro: 0,
            drops_mag: 0,
            drops_gps: 0,
            drops_system: 0,
        }
    }
}

impl TelemetryQueues {
    fn push_imu(&mut self, sample: TelemetryImu) {
        push_with_drop(&mut self.imu, sample, &mut self.drops_imu);
    }

    fn push_baro(&mut self, sample: TelemetryBaro) {
        push_with_drop(&mut self.baro, sample, &mut self.drops_baro);
    }

    fn push_mag(&mut self, sample: TelemetryMag) {
        push_with_drop(&mut self.mag, sample, &mut self.drops_mag);
    }

    fn push_gps(&mut self, sample: TelemetryGps) {
        push_with_drop(&mut self.gps, sample, &mut self.drops_gps);
    }

    fn push_system(&mut self, sample: TelemetrySystem) {
        push_with_drop(&mut self.system, sample, &mut self.drops_system);
    }

    fn pop_imu(&mut self) -> Option<TelemetryImu> {
        self.imu.pop_front()
    }

    fn pop_baro(&mut self) -> Option<TelemetryBaro> {
        self.baro.pop_front()
    }

    fn pop_mag(&mut self) -> Option<TelemetryMag> {
        self.mag.pop_front()
    }

    fn pop_gps(&mut self) -> Option<TelemetryGps> {
        self.gps.pop_front()
    }

    fn pop_system(&mut self) -> Option<TelemetrySystem> {
        self.system.pop_front()
    }

    fn len_imu(&self) -> usize {
        self.imu.len()
    }

    fn len_baro(&self) -> usize {
        self.baro.len()
    }

    fn len_mag(&self) -> usize {
        self.mag.len()
    }

    fn len_gps(&self) -> usize {
        self.gps.len()
    }

    fn len_system(&self) -> usize {
        self.system.len()
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RxMeta {
    pub rssi: i16,
    pub snr: i16,
}

#[derive(Debug)]
struct TelemetryState {
    last_sent_imu_us: u64,
    last_sent_baro_us: u64,
    last_sent_mag_us: u64,
    last_sent_gps_us: u64,
    last_sent_system_us: u64,
    last_mock_imu_us: u64,
    last_mock_baro_us: u64,
    last_mock_mag_us: u64,
    last_mock_gps_us: u64,
    last_mock_system_us: u64,
    mock_seq: u32,
    last_imu: Option<TelemetryImu>,
    last_baro: Option<TelemetryBaro>,
    last_mag: Option<TelemetryMag>,
    last_gps: Option<TelemetryGps>,
    last_system: Option<TelemetrySystem>,
    queues: TelemetryQueues,
    external_telemetry: bool,
}

impl Default for TelemetryState {
    fn default() -> Self {
        Self {
            last_sent_imu_us: 0,
            last_sent_baro_us: 0,
            last_sent_mag_us: 0,
            last_sent_gps_us: 0,
            last_sent_system_us: 0,
            last_mock_imu_us: 0,
            last_mock_baro_us: 0,
            last_mock_mag_us: 0,
            last_mock_gps_us: 0,
            last_mock_system_us: 0,
            mock_seq: 0,
            last_imu: None,
            last_baro: None,
            last_mag: None,
            last_gps: None,
            last_system: None,
            queues: TelemetryQueues::default(),
            external_telemetry: false,
        }
    }
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

    pub fn poll_telemetry(&mut self, now_us: u64) {
        if self.telemetry.external_telemetry {
            return;
        }
        self.enqueue_mock_telemetry(now_us);
    }

    pub fn enqueue_imu(&mut self, sample: TelemetryImu) {
        self.telemetry.external_telemetry = true;
        self.telemetry.queues.push_imu(sample);
    }

    pub fn enqueue_baro(&mut self, sample: TelemetryBaro) {
        self.telemetry.external_telemetry = true;
        self.telemetry.queues.push_baro(sample);
    }

    pub fn enqueue_mag(&mut self, sample: TelemetryMag) {
        self.telemetry.external_telemetry = true;
        self.telemetry.queues.push_mag(sample);
    }

    pub fn enqueue_gps(&mut self, sample: TelemetryGps) {
        self.telemetry.external_telemetry = true;
        self.telemetry.queues.push_gps(sample);
    }

    pub fn enqueue_system(&mut self, sample: TelemetrySystem) {
        self.telemetry.external_telemetry = true;
        self.telemetry.queues.push_system(sample);
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
        self.poll_telemetry(now_us);

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
            PacketType::TelemetryBurst => {
                self.on_burst_rx(packet.payload.as_slice());
            }
            _ => {}
        }
    }
}

impl LoraTransport {
    fn next_telemetry(&mut self, now_us: u64, max_len: usize) -> Option<Packet> {
        let kind = self.select_telemetry_kind(now_us, max_len)?;
        let packet = self
            .pop_burst(kind, max_len)
            .or_else(|| self.pop_single(kind));
        let Some(packet) = packet else {
            return None;
        };
        if packet_fits(&packet, max_len) {
            self.mark_telemetry_sent(kind, now_us);
            Some(packet)
        } else {
            None
        }
    }

    fn on_burst_rx(&mut self, payload: &[u8]) {
        if payload.len() < TELEMETRY_BURST_HEADER_LEN {
            return;
        }
        let Some(kind) = TelemetryKind::from_u8(payload[0]) else {
            return;
        };
        let count = payload[1] as usize;
        if count == 0 {
            return;
        }
        let sample_len = kind.sample_len();
        if sample_len == 0 {
            return;
        }
        let needed = TELEMETRY_BURST_HEADER_LEN + count.saturating_mul(sample_len);
        if payload.len() < needed {
            return;
        }
        let mut offset = TELEMETRY_BURST_HEADER_LEN;
        for _ in 0..count {
            let end = offset + sample_len;
            let chunk = &payload[offset..end];
            match kind {
                TelemetryKind::Imu => {
                    if let Ok(imu) = TelemetryImu::decode(chunk) {
                        self.telemetry.last_imu = Some(imu);
                    }
                }
                TelemetryKind::Baro => {
                    if let Ok(baro) = TelemetryBaro::decode(chunk) {
                        self.telemetry.last_baro = Some(baro);
                    }
                }
                TelemetryKind::Mag => {
                    if let Ok(mag) = TelemetryMag::decode(chunk) {
                        self.telemetry.last_mag = Some(mag);
                    }
                }
                TelemetryKind::Gps => {
                    if let Ok(gps) = TelemetryGps::decode(chunk) {
                        self.telemetry.last_gps = Some(gps);
                    }
                }
                TelemetryKind::System => {
                    if let Ok(system) = TelemetrySystem::decode(chunk) {
                        self.telemetry.last_system = Some(system);
                    }
                }
            }
            offset = end;
        }
    }

    fn pop_burst(&mut self, kind: TelemetryKind, max_len: usize) -> Option<Packet> {
        match kind {
            TelemetryKind::Imu => self.build_burst(
                kind,
                max_len,
                self.telemetry.queues.len_imu(),
                |queues| queues.pop_imu(),
                |sample| sample.encode(),
            ),
            TelemetryKind::Baro => self.build_burst(
                kind,
                max_len,
                self.telemetry.queues.len_baro(),
                |queues| queues.pop_baro(),
                |sample| sample.encode(),
            ),
            TelemetryKind::Mag => self.build_burst(
                kind,
                max_len,
                self.telemetry.queues.len_mag(),
                |queues| queues.pop_mag(),
                |sample| sample.encode(),
            ),
            TelemetryKind::Gps => self.build_burst(
                kind,
                max_len,
                self.telemetry.queues.len_gps(),
                |queues| queues.pop_gps(),
                |sample| sample.encode(),
            ),
            TelemetryKind::System => self.build_burst(
                kind,
                max_len,
                self.telemetry.queues.len_system(),
                |queues| queues.pop_system(),
                |sample| sample.encode(),
            ),
        }
    }

    fn build_burst<T, F, G>(
        &mut self,
        kind: TelemetryKind,
        max_len: usize,
        available: usize,
        mut pop: F,
        mut encode: G,
    ) -> Option<Packet>
    where
        F: FnMut(&mut TelemetryQueues) -> Option<T>,
        G: FnMut(T) -> Vec<u8, MAX_PACKET_PAYLOAD>,
    {
        let max_samples = max_burst_samples(max_len, kind.sample_len());
        if max_samples < 2 {
            return None;
        }
        let count = max_samples.min(available);
        if count < 2 {
            return None;
        }

        let mut payload = Vec::<u8, MAX_PACKET_PAYLOAD>::new();
        if payload.push(kind.as_u8()).is_err() {
            return None;
        }
        if payload.push(count as u8).is_err() {
            return None;
        }
        for _ in 0..count {
            let sample = pop(&mut self.telemetry.queues)?;
            let bytes = encode(sample);
            if payload.extend_from_slice(bytes.as_slice()).is_err() {
                return None;
            }
        }
        Some(Packet::with_payload(PacketType::TelemetryBurst, payload))
    }

    fn pop_single(&mut self, kind: TelemetryKind) -> Option<Packet> {
        match kind {
            TelemetryKind::Imu => self
                .telemetry
                .queues
                .pop_imu()
                .map(|imu| Packet::with_payload(PacketType::TelemetryImu, imu.encode())),
            TelemetryKind::Baro => self
                .telemetry
                .queues
                .pop_baro()
                .map(|baro| Packet::with_payload(PacketType::TelemetryBaro, baro.encode())),
            TelemetryKind::Mag => self
                .telemetry
                .queues
                .pop_mag()
                .map(|mag| Packet::with_payload(PacketType::TelemetryMag, mag.encode())),
            TelemetryKind::Gps => self
                .telemetry
                .queues
                .pop_gps()
                .map(|gps| Packet::with_payload(PacketType::TelemetryGps, gps.encode())),
            TelemetryKind::System => self
                .telemetry
                .queues
                .pop_system()
                .map(|system| Packet::with_payload(PacketType::TelemetrySystem, system.encode())),
        }
    }

    fn select_telemetry_kind(&self, now_us: u64, max_len: usize) -> Option<TelemetryKind> {
        let best: Option<(TelemetryKind, u64, u64)> = None;

        let best = pick_due(
            best,
            TelemetryKind::Imu,
            now_us,
            self.telemetry.last_sent_imu_us,
            IMU_PERIOD_US,
            IMU_PAYLOAD_LEN,
            max_len,
            self.telemetry.queues.len_imu(),
        );
        let best = pick_due(
            best,
            TelemetryKind::Baro,
            now_us,
            self.telemetry.last_sent_baro_us,
            BARO_PERIOD_US,
            BARO_PAYLOAD_LEN,
            max_len,
            self.telemetry.queues.len_baro(),
        );
        let best = pick_due(
            best,
            TelemetryKind::Mag,
            now_us,
            self.telemetry.last_sent_mag_us,
            MAG_PERIOD_US,
            MAG_PAYLOAD_LEN,
            max_len,
            self.telemetry.queues.len_mag(),
        );
        let best = pick_due(
            best,
            TelemetryKind::Gps,
            now_us,
            self.telemetry.last_sent_gps_us,
            GPS_PERIOD_US,
            GPS_PAYLOAD_LEN,
            max_len,
            self.telemetry.queues.len_gps(),
        );
        let best = pick_due(
            best,
            TelemetryKind::System,
            now_us,
            self.telemetry.last_sent_system_us,
            SYSTEM_PERIOD_US,
            SYSTEM_PAYLOAD_LEN,
            max_len,
            self.telemetry.queues.len_system(),
        );

        best.map(|(kind, _, _)| kind)
    }

    fn mark_telemetry_sent(&mut self, kind: TelemetryKind, now_us: u64) {
        match kind {
            TelemetryKind::Imu => self.telemetry.last_sent_imu_us = now_us,
            TelemetryKind::Baro => self.telemetry.last_sent_baro_us = now_us,
            TelemetryKind::Mag => self.telemetry.last_sent_mag_us = now_us,
            TelemetryKind::Gps => self.telemetry.last_sent_gps_us = now_us,
            TelemetryKind::System => self.telemetry.last_sent_system_us = now_us,
        }
    }

    fn enqueue_mock_telemetry(&mut self, now_us: u64) {
        let imu_period = IMU_PERIOD_US.max(1);
        let mut last_imu = self.telemetry.last_mock_imu_us;
        let mut produced = 0usize;
        while now_us.saturating_sub(last_imu) >= imu_period
            && produced < TELEMETRY_QUEUE_LEN
        {
            let sample = self.mock_imu();
            self.telemetry.queues.push_imu(sample);
            last_imu = last_imu.wrapping_add(imu_period);
            produced += 1;
        }
        self.telemetry.last_mock_imu_us = last_imu;

        let baro_period = BARO_PERIOD_US.max(1);
        let mut last_baro = self.telemetry.last_mock_baro_us;
        produced = 0;
        while now_us.saturating_sub(last_baro) >= baro_period
            && produced < TELEMETRY_QUEUE_LEN
        {
            let sample = self.mock_baro();
            self.telemetry.queues.push_baro(sample);
            last_baro = last_baro.wrapping_add(baro_period);
            produced += 1;
        }
        self.telemetry.last_mock_baro_us = last_baro;

        let mag_period = MAG_PERIOD_US.max(1);
        let mut last_mag = self.telemetry.last_mock_mag_us;
        produced = 0;
        while now_us.saturating_sub(last_mag) >= mag_period
            && produced < TELEMETRY_QUEUE_LEN
        {
            let sample = self.mock_mag();
            self.telemetry.queues.push_mag(sample);
            last_mag = last_mag.wrapping_add(mag_period);
            produced += 1;
        }
        self.telemetry.last_mock_mag_us = last_mag;

        let gps_period = GPS_PERIOD_US.max(1);
        let mut last_gps = self.telemetry.last_mock_gps_us;
        produced = 0;
        while now_us.saturating_sub(last_gps) >= gps_period
            && produced < TELEMETRY_QUEUE_LEN
        {
            let sample = self.mock_gps();
            self.telemetry.queues.push_gps(sample);
            last_gps = last_gps.wrapping_add(gps_period);
            produced += 1;
        }
        self.telemetry.last_mock_gps_us = last_gps;

        let system_period = SYSTEM_PERIOD_US.max(1);
        let mut last_system = self.telemetry.last_mock_system_us;
        produced = 0;
        while now_us.saturating_sub(last_system) >= system_period
            && produced < TELEMETRY_QUEUE_LEN
        {
            let sample = self.mock_system();
            self.telemetry.queues.push_system(sample);
            last_system = last_system.wrapping_add(system_period);
            produced += 1;
        }
        self.telemetry.last_mock_system_us = last_system;
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
    available: usize,
) -> Option<(TelemetryKind, u64, u64)> {
    if available == 0 {
        return best;
    }
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

fn max_burst_samples(max_len: usize, sample_len: usize) -> usize {
    if sample_len == 0 {
        return 0;
    }
    let max_payload = max_len.saturating_sub(1);
    if max_payload <= TELEMETRY_BURST_HEADER_LEN {
        return 0;
    }
    (max_payload - TELEMETRY_BURST_HEADER_LEN) / sample_len
}

fn push_with_drop<T, const N: usize>(queue: &mut Deque<T, N>, sample: T, drops: &mut u32) {
    match queue.push_back(sample) {
        Ok(()) => {}
        Err(sample) => {
            let _ = queue.pop_front();
            let _ = queue.push_back(sample);
            *drops = drops.wrapping_add(1);
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
