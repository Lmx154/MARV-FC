use super::link_test_config::LinkTestConfig;
use super::lora_config::LoRaConfig;

#[derive(Clone, Copy, Debug)]
pub struct LinkProfile {
    pub lora: LoRaConfig,
    pub tick_hz: u32,
    pub slot_ratio_r: u16,
    pub tx_guard_us: u64,
    pub dl_tx_offset_us: u64,
    pub rx_ready_guard_us: u64,
    pub rx_timeout_symbols: u16,
    pub rx_timeout_auto: bool,
    pub uplink_payload_len: usize,
    pub downlink_payload_len: usize,
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

    pub const fn from_test(cfg: LinkTestConfig) -> Self {
        Self {
            lora: cfg.lora,
            tick_hz: cfg.tick_hz,
            slot_ratio_r: cfg.slot_ratio_r,
            tx_guard_us: cfg.tx_guard_us,
            dl_tx_offset_us: cfg.dl_tx_offset_us,
            rx_ready_guard_us: cfg.rx_ready_guard_us,
            rx_timeout_symbols: cfg.rx_timeout_symbols,
            rx_timeout_auto: cfg.rx_timeout_auto,
            uplink_payload_len: cfg.uplink_payload_len,
            downlink_payload_len: cfg.downlink_payload_len,
        }
    }

    pub fn as_test_config(&self) -> LinkTestConfig {
        LinkTestConfig {
            lora: self.lora,
            tick_hz: self.tick_hz,
            slot_ratio_r: self.slot_ratio_r,
            tx_guard_us: self.tx_guard_us,
            dl_tx_offset_us: self.dl_tx_offset_us,
            rx_ready_guard_us: self.rx_ready_guard_us,
            rx_timeout_symbols: self.rx_timeout_symbols,
            rx_timeout_auto: self.rx_timeout_auto,
            uplink_payload_len: self.uplink_payload_len,
            downlink_payload_len: self.downlink_payload_len,
        }
    }
}

impl From<LinkTestConfig> for LinkProfile {
    fn from(cfg: LinkTestConfig) -> Self {
        Self::from_test(cfg)
    }
}
