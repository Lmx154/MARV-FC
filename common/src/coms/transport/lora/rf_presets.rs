//! RF layer presets and active profile selection.
use super::rf_config::{LoRaConfig, RfConfig};

// Fast-mid profile: better margin than SF6, still high throughput.
pub const LORA_SF7_BW500_CR45: RfConfig = LoRaConfig {
    sf: 7,
    bw: 0x06,
    cr: 0x01,
    ..LoRaConfig::preset_fast()
};

// Fast profile: default bench throughput preset (SF6/BW500/CR4/5).
pub const LORA_SF6_BW500_CR45: RfConfig = LoRaConfig::preset_fast();

// Max bandwidth LoRa (short-range / stress testing).
pub const LORA_SF5_BW500_CR45: RfConfig = LoRaConfig {
    sf: 5,
    bw: 0x06,
    cr: 0x01,
    ..LoRaConfig::preset_fast()
};

// Robust mid-range profile.
pub const LORA_SF9_BW250_CR46: RfConfig = LoRaConfig {
    sf: 9,
    bw: 0x05,
    cr: 0x02,
    ..LoRaConfig::preset_fast()
};

// Long-range-ish profile (slower, larger link budget).
pub const LORA_SF10_BW125_CR46: RfConfig = LoRaConfig {
    sf: 10,
    bw: 0x04,
    cr: 0x02,
    ..LoRaConfig::preset_fast()
};

// Long-range preset for maximum link margin.
pub const LORA_SF11_BW125_CR48: RfConfig = LoRaConfig::preset_long_range();

// Toggle this to switch RF profiles at runtime.
pub const ACTIVE: RfConfig = LORA_SF6_BW500_CR45;
