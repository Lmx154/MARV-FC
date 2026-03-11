#![allow(dead_code)]

use embassy_rp::{Peri, peripherals};

pub const FEED_AUTHORITY: &str = "core0 watchdog supervisor";

pub struct FeedContract {
    pub requires_fast_path: bool,
    pub requires_actuator_path: bool,
    pub requires_core1_heartbeat: bool,
}

impl FeedContract {
    pub const fn marv_fc_default() -> Self {
        Self {
            requires_fast_path: true,
            requires_actuator_path: true,
            requires_core1_heartbeat: false,
        }
    }
}

pub struct WatchdogResources {
    pub peripheral: Peri<'static, peripherals::WATCHDOG>,
    pub timeout_ms: u32,
    pub contract: FeedContract,
}
