//! RF layer presets and active profile selection.
use super::rf_config::{LoRaConfig, RfConfig};

// ToA guide (timing constraint):
// - total_len_bytes = HEADER_LEN (3) + payload_len
// - toa_us = rf.toa_us(total_len_bytes)
// - max_tick_hz ~= floor(1_000_000 / (toa_us + tx_guard_us))
// Assumptions for the per-preset notes below:
// - ELRS payload size = 8 bytes (total_len_bytes = 11)
// - tx_guard_us = 1000
// Update the notes if payload size or guard changes.

// Fast-mid profile: better margin than SF6, still high throughput.
// ELRS(8B) ToA ~ 10.304 ms, max_tick_hz ~ 88 (guard 1000us).
pub const LORA_SF7_BW500_CR45: RfConfig = LoRaConfig {
    sf: 7,
    bw: 0x06,
    cr: 0x01,
    ..LoRaConfig::preset_fast()
};

// Mid profile: step down from SF7 while keeping BW500 for good range/latency balance.
// ELRS(8B) ToA ~ 20.608 ms, max_tick_hz ~ 46 (guard 1000us).
pub const LORA_SF8_BW500_CR45: RfConfig = LoRaConfig {
    sf: 8,
    bw: 0x06,
    cr: 0x01,
    ..LoRaConfig::preset_fast()
};

// Fast profile: default bench throughput preset (SF6/BW500/CR4/5).
// ELRS(8B) ToA ~ 6.304 ms, max_tick_hz ~ 136 (guard 1000us).
pub const LORA_SF6_BW500_CR45: RfConfig = LoRaConfig::preset_fast();

// Max bandwidth LoRa (short-range / stress testing).
// ELRS(8B) ToA ~ 3.472 ms, max_tick_hz ~ 223 (guard 1000us).
pub const LORA_SF5_BW500_CR45: RfConfig = LoRaConfig {
    sf: 5,
    bw: 0x06,
    cr: 0x01,
    ..LoRaConfig::preset_fast()
};

// Robust mid-range profile.
// ELRS(8B) ToA ~ 78.336 ms, max_tick_hz ~ 12 (guard 1000us).
pub const LORA_SF9_BW250_CR46: RfConfig = LoRaConfig {
    sf: 9,
    bw: 0x05,
    cr: 0x02,
    ..LoRaConfig::preset_fast()
};

// Long-range-ish profile (slower, larger link budget).
// ELRS(8B) ToA ~ 313.344 ms, max_tick_hz ~ 3 (guard 1000us).
pub const LORA_SF10_BW125_CR46: RfConfig = LoRaConfig {
    sf: 10,
    bw: 0x04,
    cr: 0x02,
    ..LoRaConfig::preset_fast()
};

// Long-range preset for maximum link margin.
// ELRS(8B) ToA ~ 790.528 ms, max_tick_hz ~ 1 (guard 1000us).
pub const LORA_SF11_BW125_CR48: RfConfig = LoRaConfig::preset_long_range();
