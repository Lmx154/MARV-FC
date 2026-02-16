//! Single MAC preset for 915 MHz long-range CMD + telemetry link.

use super::mac_config::MacConfig;

pub const LORA_CMD_TELEM_LONG_RANGE_915: MacConfig = MacConfig {
    tick_hz: 38,
    slot_ratio_r: 2,
    tx_guard_us: 3_000,
    dl_tx_offset_us: 5_000,
    rx_ready_guard_us: 1_500,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 15,
};
