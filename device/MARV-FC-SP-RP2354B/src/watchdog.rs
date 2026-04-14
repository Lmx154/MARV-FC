#![allow(dead_code)]

use common::messages::fault::ResetReason;
use common::messages::runtime::FlightPhase;
use common::services::health::{WatchdogContract, WatchdogSource};
use embassy_rp::watchdog::{ResetReason as RpResetReason, Watchdog};
use embassy_rp::{Peri, peripherals};
use embassy_time::Duration;

pub const FEED_AUTHORITY: &str = "core0 watchdog supervisor";

pub const SOURCE_BOOT_COORDINATOR: u32 = 1 << 0;
pub const SOURCE_HIL_TIME: u32 = 1 << 1;
pub const SOURCE_PRIMARY_IMU: u32 = 1 << 2;
pub const SOURCE_BAROMETER: u32 = 1 << 3;
pub const SOURCE_AUX_IMU: u32 = 1 << 4;

pub const SOURCE_COUNT: usize = 5;
pub const SOURCES: [WatchdogSource; SOURCE_COUNT] = [
    WatchdogSource::new(SOURCE_BOOT_COORDINATOR, 500),
    WatchdogSource::new(SOURCE_HIL_TIME, 300),
    WatchdogSource::new(SOURCE_PRIMARY_IMU, 200),
    WatchdogSource::new(SOURCE_BAROMETER, 400),
    WatchdogSource::new(SOURCE_AUX_IMU, 400),
];

pub const INIT_CONTRACT: WatchdogContract = WatchdogContract::new(SOURCE_BOOT_COORDINATOR, 0);
pub const HIL_CONTRACT: WatchdogContract = WatchdogContract::new(SOURCE_HIL_TIME, 0);
pub const READY_CONTRACT: WatchdogContract =
    WatchdogContract::new(SOURCE_PRIMARY_IMU | SOURCE_BAROMETER, SOURCE_AUX_IMU);

pub struct WatchdogResources {
    pub peripheral: Peri<'static, peripherals::WATCHDOG>,
    pub timeout_ms: u32,
}

pub struct HardwareWatchdog {
    inner: Watchdog,
    timeout_ms: u32,
}

impl HardwareWatchdog {
    pub fn new(peripheral: Peri<'static, peripherals::WATCHDOG>, timeout_ms: u32) -> Self {
        Self {
            inner: Watchdog::new(peripheral),
            timeout_ms,
        }
    }

    pub fn start(&mut self) {
        self.inner.pause_on_debug(true);
        self.inner
            .start(Duration::from_millis(u64::from(self.timeout_ms.max(1))));
    }

    pub fn feed(&mut self) {
        self.inner.feed();
    }

    pub fn reset_reason(&self) -> ResetReason {
        match self.inner.reset_reason() {
            Some(RpResetReason::Forced) => ResetReason::Software,
            Some(RpResetReason::TimedOut) => ResetReason::Watchdog,
            None => ResetReason::Unknown,
        }
    }
}

pub const fn contract_for_phase(phase: FlightPhase) -> WatchdogContract {
    match phase {
        FlightPhase::Init => INIT_CONTRACT,
        FlightPhase::Hil => HIL_CONTRACT,
        FlightPhase::Ready | FlightPhase::Active | FlightPhase::Fault => READY_CONTRACT,
    }
}
