//! Link-level derived timing profile built from RF + MAC for MAC engines.
use super::mac_codec::HEADER_LEN;
use super::mac_config::MacConfig;
use super::rf_config::RfConfig;

#[derive(Clone, Copy, Debug)]
pub struct LinkProfile {
    pub tick_hz: u32,
    pub slot_ratio_r: u16,
    pub tx_guard_us: u64,
    pub dl_tx_offset_us: u64,
    pub rx_ready_guard_us: u64,
    pub uplink_payload_len: usize,
    pub downlink_payload_len: usize,
    pub uplink_toa_us: u64,
    pub downlink_toa_us: u64,
}

impl LinkProfile {
    pub const fn tick_period_us(&self) -> u64 {
        let hz = if self.tick_hz == 0 { 1 } else { self.tick_hz };
        1_000_000u64 / hz as u64
    }

    pub const fn tick_period_ms(&self) -> u64 {
        let hz = if self.tick_hz == 0 { 1 } else { self.tick_hz };
        1000u64 / hz as u64
    }

    pub fn from_configs(rf: RfConfig, mac: MacConfig) -> Self {
        let uplink_len = frame_len(mac.uplink_payload_len);
        let downlink_len = frame_len(mac.downlink_payload_len);
        Self {
            tick_hz: mac.tick_hz,
            slot_ratio_r: mac.slot_ratio_r,
            tx_guard_us: mac.tx_guard_us,
            dl_tx_offset_us: mac.dl_tx_offset_us,
            rx_ready_guard_us: mac.rx_ready_guard_us,
            uplink_payload_len: mac.uplink_payload_len,
            downlink_payload_len: mac.downlink_payload_len,
            uplink_toa_us: rf.toa_us(uplink_len),
            downlink_toa_us: rf.toa_us(downlink_len),
        }
    }
}

fn frame_len(payload_len: usize) -> usize {
    HEADER_LEN + payload_len
}
