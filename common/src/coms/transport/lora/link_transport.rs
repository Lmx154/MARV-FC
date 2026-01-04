//! Link-level packet selection and ARQ; independent of RF/MAC timing.
use heapless::Vec;

use crate::protocol::packet::{
    CommandFrame, LinkStats, Packet, PacketType, MAX_PACKET_PAYLOAD,
};

pub const DEFAULT_ARQ_TIMEOUT_MS: u64 = 250;
pub const MAX_RC_BYTES: usize = 16;

#[derive(Clone, Copy, Debug, Default)]
pub struct RxMeta {
    pub rssi: i16,
    pub snr: i16,
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

    pub fn next_downlink(&mut self, max_len: usize) -> Option<Packet> {
        if let Some(seq) = self.pending_ack.take() {
            let mut payload = Vec::new();
            let _ = payload.push(seq);
            let packet = Packet::with_payload(PacketType::Ack, payload);
            if packet_fits(&packet, max_len) {
                return Some(packet);
            }
            self.pending_ack = Some(seq);
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
            _ => {}
        }
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
