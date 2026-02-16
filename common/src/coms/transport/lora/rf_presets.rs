//! RF preset for single-mode 915 MHz long-range operation.
use super::rf_config::{LoRaConfig, RfConfig};

pub const LORA_CMD_TELEM_LONG_RANGE_915: RfConfig = LoRaConfig {
    sf: 7,
    bw: 0x06,
    cr: 0x01,
    tx_power: 14,
    preamble_len: 8,
    ..LoRaConfig::preset_fast()
};
