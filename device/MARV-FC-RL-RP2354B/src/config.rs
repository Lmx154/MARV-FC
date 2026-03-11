#![allow(dead_code)]

pub const XOSC_HZ: u32 = 12_000_000;
pub const FAST_LOOP_HZ: u32 = 1_000;
pub const WATCHDOG_TIMEOUT_MS: u32 = 250;
pub const STATUS_HEARTBEAT_PERIOD_MS: u64 = 1_000;

#[derive(Clone, Copy, Debug)]
pub struct DeviceConfig {
    pub fast_loop_hz: u32,
    pub watchdog_timeout_ms: u32,
    pub status_heartbeat_period_ms: u64,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            fast_loop_hz: FAST_LOOP_HZ,
            watchdog_timeout_ms: WATCHDOG_TIMEOUT_MS,
            status_heartbeat_period_ms: STATUS_HEARTBEAT_PERIOD_MS,
        }
    }
}
