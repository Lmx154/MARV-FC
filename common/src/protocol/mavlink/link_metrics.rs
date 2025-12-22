//! Full link metrics message helpers (custom dialect).

use mavio::protocol::V2;
use mavio::Frame;

use crate::protocol::mavlink::encode::MavEndpointConfig;

/// MAVLink message id for `MARV_LINK_METRICS` (custom dialect).
pub const MAVLINK_MSG_ID_MARV_LINK_METRICS: u32 = 42_003;
/// CRC extra for `MARV_LINK_METRICS` (per MAVLink mavparse).
pub const MARV_LINK_METRICS_CRC_EXTRA: u8 = 119;

pub const MARV_LINK_METRICS_FIELD_COUNT: usize = 24;
pub const MARV_LINK_METRICS_PAYLOAD_LEN: usize = MARV_LINK_METRICS_FIELD_COUNT * 4;

#[derive(Clone, Copy, Debug, Default)]
pub struct LinkMetrics {
    pub rx_packets: u32,
    pub tx_packets: u32,
    pub q_fast: u32,
    pub q_reliable: u32,
    pub q_normal: u32,
    pub enq_fast: u32,
    pub enq_reliable: u32,
    pub enq_normal: u32,
    pub tx_fast: u32,
    pub tx_reliable: u32,
    pub tx_normal: u32,
    pub drop_fast_replaced: u32,
    pub drop_reliable_overflow: u32,
    pub drop_normal_overflow: u32,
    pub drop_too_large: u32,
    pub missed_master_ticks: u32,
    pub missed_fast_seq_rx: u32,
    pub missed_fast_seq_tx: u32,
    pub tx_inhibit_desync: u32,
    pub rf_txn_state: u32,
    pub rf_last_result: u32,
    pub rf_apply_at_tick: u32,
    pub tick_hz: u32,
    pub slot_mode: u32,
}

impl LinkMetrics {
    pub fn encode(&self, out: &mut [u8; MARV_LINK_METRICS_PAYLOAD_LEN]) {
        let mut i = 0usize;
        write_u32(out, &mut i, self.rx_packets);
        write_u32(out, &mut i, self.tx_packets);
        write_u32(out, &mut i, self.q_fast);
        write_u32(out, &mut i, self.q_reliable);
        write_u32(out, &mut i, self.q_normal);
        write_u32(out, &mut i, self.enq_fast);
        write_u32(out, &mut i, self.enq_reliable);
        write_u32(out, &mut i, self.enq_normal);
        write_u32(out, &mut i, self.tx_fast);
        write_u32(out, &mut i, self.tx_reliable);
        write_u32(out, &mut i, self.tx_normal);
        write_u32(out, &mut i, self.drop_fast_replaced);
        write_u32(out, &mut i, self.drop_reliable_overflow);
        write_u32(out, &mut i, self.drop_normal_overflow);
        write_u32(out, &mut i, self.drop_too_large);
        write_u32(out, &mut i, self.missed_master_ticks);
        write_u32(out, &mut i, self.missed_fast_seq_rx);
        write_u32(out, &mut i, self.missed_fast_seq_tx);
        write_u32(out, &mut i, self.tx_inhibit_desync);
        write_u32(out, &mut i, self.rf_txn_state);
        write_u32(out, &mut i, self.rf_last_result);
        write_u32(out, &mut i, self.rf_apply_at_tick);
        write_u32(out, &mut i, self.tick_hz);
        write_u32(out, &mut i, self.slot_mode);
    }
}

fn write_u32(out: &mut [u8; MARV_LINK_METRICS_PAYLOAD_LEN], idx: &mut usize, value: u32) {
    let bytes = value.to_le_bytes();
    out[*idx..*idx + 4].copy_from_slice(&bytes);
    *idx += 4;
}

/// Build a `MARV_LINK_METRICS` frame (manual payload/CRC).
pub fn build_link_metrics_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    metrics: &LinkMetrics,
) -> Frame<V2> {
    let mut payload = [0u8; MARV_LINK_METRICS_PAYLOAD_LEN];
    metrics.encode(&mut payload);

    Frame::builder()
        .version(V2)
        .system_id(cfg.sys_id)
        .component_id(cfg.comp_id)
        .sequence(seq)
        .message_id(MAVLINK_MSG_ID_MARV_LINK_METRICS)
        .payload(&payload)
        .crc_extra(MARV_LINK_METRICS_CRC_EXTRA)
        .build()
}
