//! Portable periodic magnetometer owner service.

use embassy_sync::blocking_mutex::raw::RawMutex;

use crate::interfaces::sensors::MagnetometerSource;
use crate::interfaces::timing::MonotonicClock;
use crate::messages::sensor::MagnetometerSampleStamped;
use crate::utils::delay::DelayMs;

use super::channels::MagnetometerSampleChannel;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MagnetometerServiceConfig {
    pub enabled: bool,
    pub period_ms: u32,
}

impl MagnetometerServiceConfig {
    pub const fn new(enabled: bool, period_ms: u32) -> Self {
        Self { enabled, period_ms }
    }
}

impl Default for MagnetometerServiceConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            period_ms: 40,
        }
    }
}

pub async fn run_magnetometer_service<
    M,
    S,
    C,
    D,
    F,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    channel: &'static MagnetometerSampleChannel<M, DEPTH, SUBS, PUBS>,
    source: &mut S,
    clock: &C,
    delay: &mut D,
    config: MagnetometerServiceConfig,
    mut on_error: F,
) -> !
where
    M: RawMutex,
    S: MagnetometerSource,
    C: MonotonicClock,
    D: DelayMs,
    F: FnMut(S::Error),
{
    let period_ms = config.period_ms.max(1);

    loop {
        if config.enabled {
            let timestamp = clock.now();
            match source.read_magnetometer_sample().await {
                Ok(sample) => {
                    channel
                        .immediate_publisher()
                        .publish_immediate(MagnetometerSampleStamped { timestamp, sample });
                }
                Err(error) => on_error(error),
            }
        }

        delay.delay_ms(period_ms).await;
    }
}
