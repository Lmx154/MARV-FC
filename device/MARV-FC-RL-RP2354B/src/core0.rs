use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use crate::channels;
use crate::config::{DeviceConfig, STATUS_HEARTBEAT_PERIOD_MS};
use crate::core1;
use crate::pinmap;
use crate::resources::DeviceResources;
use crate::storage;
use crate::watchdog;

pub async fn run(_spawner: Spawner, resources: DeviceResources) -> ! {
    let DeviceResources {
        pins,
        buses,
        watchdog: _watchdog,
        system: _system,
    } = resources;
    let config = DeviceConfig::default();

    match storage::run_startup_smoke_test(pins.storage, buses.storage) {
        Ok(path) => info!("sd smoke test complete: {}", path.as_str()),
        Err(error) => info!("sd smoke test failed: {:?}", error),
    }

    info!("MARV-FC-RL-RP2354B resource graph initialized");
    info!("core0 owns SPI1 sensors, I2C domains, actuator outputs, and watchdog feed");
    info!("core1 planned ownership: {}", core1::ROLE_SUMMARY);
    info!("watchdog authority: {}", watchdog::FEED_AUTHORITY);
    info!(
        "channel plan: core0={} core1={}",
        channels::TOPOLOGY.core0_feed_critical.len(),
        channels::TOPOLOGY.core1_background.len()
    );
    info!(
        "key pins: radio=GP{=u8}/GP{=u8} sensor-spi=GP{=u8}/GP{=u8}/GP{=u8} sd-cs=GP{=u8}",
        pinmap::FC_RADIO_TX,
        pinmap::FC_RADIO_RX,
        pinmap::SENSOR_SPI_SCK,
        pinmap::SENSOR_SPI_MOSI,
        pinmap::SENSOR_SPI_MISO,
        pinmap::STORAGE_SPI_CS,
    );
    info!(
        "fast_loop_hz={} watchdog_timeout_ms={}",
        config.fast_loop_hz, config.watchdog_timeout_ms
    );

    loop {
        Timer::after(Duration::from_millis(STATUS_HEARTBEAT_PERIOD_MS)).await;
        info!("core0 heartbeat");
    }
}
