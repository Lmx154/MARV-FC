#![allow(dead_code)]

use common::messages::runtime::FlightPhase;
use common::services::health::{WatchdogContract, WatchdogSource};
pub use rp235x_base::watchdog::{HardwareWatchdog, WatchdogResources};

pub const FEED_AUTHORITY: &str = "core0 watchdog supervisor";

pub const SOURCE_BOOT_COORDINATOR: u32 = 1 << 0;
pub const SOURCE_USB_PROBE: u32 = 1 << 1;
pub const SOURCE_HIL_TIME: u32 = 1 << 2;
pub const SOURCE_IMU: u32 = 1 << 3;
pub const SOURCE_BAROMETER: u32 = 1 << 4;

pub const SOURCE_COUNT: usize = 5;
pub const SOURCES: [WatchdogSource; SOURCE_COUNT] = [
    WatchdogSource::new(SOURCE_BOOT_COORDINATOR, 500),
    WatchdogSource::new(SOURCE_USB_PROBE, 500),
    WatchdogSource::new(SOURCE_HIL_TIME, 300),
    WatchdogSource::new(SOURCE_IMU, 200),
    WatchdogSource::new(SOURCE_BAROMETER, 400),
];

pub const INIT_CONTRACT: WatchdogContract =
    WatchdogContract::new(SOURCE_BOOT_COORDINATOR | SOURCE_USB_PROBE, 0);
pub const HIL_CONTRACT: WatchdogContract = WatchdogContract::new(SOURCE_HIL_TIME, 0);
pub const READY_CONTRACT: WatchdogContract =
    WatchdogContract::new(SOURCE_IMU | SOURCE_BAROMETER, 0);

pub const fn contract_for_phase(phase: FlightPhase) -> WatchdogContract {
    match phase {
        FlightPhase::Init => INIT_CONTRACT,
        FlightPhase::Hil => HIL_CONTRACT,
        FlightPhase::Ready | FlightPhase::Active | FlightPhase::Fault => READY_CONTRACT,
    }
}
