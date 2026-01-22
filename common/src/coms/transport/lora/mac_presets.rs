//! Vehicle and rate-specific MAC presets live here.
//!
//! These presets encode packet sizes, lane priorities, and timing targets.
//! Values are kept per-vehicle to allow divergence even when numbers match.

use super::mac_config::MacConfig;

// Drone presets (fast/tiny payloads).
pub const DRONE_200: MacConfig = MacConfig {
    tick_hz: 200,
    slot_ratio_r: 5,
    tx_guard_us: 500,
    dl_tx_offset_us: 500,
    rx_ready_guard_us: 300,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 0,
    downlink_payload_len: 0,
};

pub const DRONE_150: MacConfig = MacConfig {
    tick_hz: 150,
    slot_ratio_r: 5,
    tx_guard_us: 500,
    dl_tx_offset_us: 1_500,
    rx_ready_guard_us: 400,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 4,
    downlink_payload_len: 4,
};

pub const DRONE_100: MacConfig = MacConfig {
    tick_hz: 100,
    slot_ratio_r: 5,
    tx_guard_us: 500,
    dl_tx_offset_us: 6_000,
    rx_ready_guard_us: 400,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 4,
    downlink_payload_len: 4,
};

// Drone presets (standard payloads).
pub const DRONE_50: MacConfig = MacConfig {
    tick_hz: 50,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

pub const DRONE_25: MacConfig = MacConfig {
    tick_hz: 25,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

pub const DRONE_10: MacConfig = MacConfig {
    tick_hz: 10,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

pub const DRONE_5: MacConfig = MacConfig {
    tick_hz: 5,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

pub const DRONE_1: MacConfig = MacConfig {
    tick_hz: 1,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

// Rocket presets (fast/tiny payloads).
pub const ROCKET_200: MacConfig = MacConfig {
    tick_hz: 200,
    slot_ratio_r: 5,
    tx_guard_us: 500,
    dl_tx_offset_us: 500,
    rx_ready_guard_us: 300,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 0,
    downlink_payload_len: 0,
};

pub const ROCKET_150: MacConfig = MacConfig {
    tick_hz: 150,
    slot_ratio_r: 5,
    tx_guard_us: 500,
    dl_tx_offset_us: 1_500,
    rx_ready_guard_us: 400,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 4,
    downlink_payload_len: 4,
};

pub const ROCKET_100: MacConfig = MacConfig {
    tick_hz: 100,
    slot_ratio_r: 5,
    tx_guard_us: 500,
    dl_tx_offset_us: 6_000,
    rx_ready_guard_us: 400,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 4,
    downlink_payload_len: 4,
};

// Rocket presets (standard payloads).
pub const ROCKET_50: MacConfig = MacConfig {
    tick_hz: 50,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

pub const ROCKET_25: MacConfig = MacConfig {
    tick_hz: 25,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

pub const ROCKET_10: MacConfig = MacConfig {
    tick_hz: 10,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

pub const ROCKET_5: MacConfig = MacConfig {
    tick_hz: 5,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

pub const ROCKET_1: MacConfig = MacConfig {
    tick_hz: 1,
    slot_ratio_r: 10,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 16,
};

// LoRa telemetry-first presets (SX1262).
// These are tuned to maximize downlink telemetry while staying inside ToA budgets.
pub const LORA_TELEM_200: MacConfig = MacConfig {
    tick_hz: 200,
    slot_ratio_r: 2,
    tx_guard_us: 400,
    dl_tx_offset_us: 400,
    rx_ready_guard_us: 250,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 14,
};

pub const LORA_TELEM_100: MacConfig = MacConfig {
    tick_hz: 100,
    slot_ratio_r: 2,
    tx_guard_us: 500,
    dl_tx_offset_us: 500,
    rx_ready_guard_us: 300,
    rx_timeout_symbols: 64,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 27,
};

pub const LORA_TELEM_50: MacConfig = MacConfig {
    tick_hz: 50,
    slot_ratio_r: 2,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 1_000,
    rx_ready_guard_us: 300,
    rx_timeout_symbols: 32,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 27,
};

pub const LORA_TELEM_20: MacConfig = MacConfig {
    tick_hz: 20,
    slot_ratio_r: 2,
    tx_guard_us: 1_000,
    dl_tx_offset_us: 2_500,
    rx_ready_guard_us: 800,
    rx_timeout_symbols: 32,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 27,
};

pub const LORA_TELEM_5: MacConfig = MacConfig {
    tick_hz: 5,
    slot_ratio_r: 2,
    tx_guard_us: 5_000,
    dl_tx_offset_us: 10_000,
    rx_ready_guard_us: 3_000,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 27,
};

pub const LORA_TELEM_1: MacConfig = MacConfig {
    tick_hz: 1,
    slot_ratio_r: 2,
    tx_guard_us: 10_000,
    dl_tx_offset_us: 20_000,
    rx_ready_guard_us: 5_000,
    rx_timeout_symbols: 16,
    rx_timeout_auto: true,
    uplink_payload_len: 8,
    downlink_payload_len: 15,
};
