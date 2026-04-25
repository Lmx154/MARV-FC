use common::messages::fault::ResetReason;
use common::services::health::WatchdogDriver;
use embassy_rp::watchdog::{ResetReason as RpResetReason, Watchdog};
use embassy_rp::{Peri, peripherals};
use embassy_time::Duration;

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

impl WatchdogDriver for HardwareWatchdog {
    fn start(&mut self) {
        HardwareWatchdog::start(self);
    }

    fn feed(&mut self) {
        HardwareWatchdog::feed(self);
    }
}
