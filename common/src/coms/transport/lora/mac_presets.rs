//! Vehicle and test MAC presets live here to avoid implicit sharing.
//!
//! These presets encode packet sizes, lane priorities, and timing targets.
//! Those inputs are vehicle- or use-case-specific, so the presets are kept
//! separate even when numeric values happen to match.
/// Non-shareable inputs baked into this preset:
/// - uplink/downlink payload sizes 
/// - tick rate and slot ratio (control latency vs telemetry budget)
/// - guard/offset/ready timings (slot scheduling policy)
/// - RX timeout policy (derived from the above schedule)
/// These feed rx_timeout_symbols and slot_rx_symbols, so they stay per-vehicle.
/// 

use super::mac_config::MacConfig;

/// Drone MAC preset.
///
pub const DRONE_MAC: MacConfig = MacConfig {
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

/// Rocket MAC preset.
///
pub const ROCKET_MAC: MacConfig = MacConfig {
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

/// Test MAC preset (bench and bring-up).
///
pub const TEST_MAC: MacConfig = MacConfig {
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
