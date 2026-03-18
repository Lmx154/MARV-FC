//! Portable periodic pressure transducer owner service.

use embassy_sync::blocking_mutex::raw::RawMutex;

use crate::interfaces::sensors::PressureTransducerSource;
use crate::interfaces::timing::MonotonicClock;
use crate::messages::sensor::PressureTransducerSampleStamped;
use crate::utils::delay::DelayMs;

use super::sample_channels::PressureTransducerSampleChannel;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PressureTransducerServiceConfig {
    pub enabled: bool,
    pub period_ms: u32,
}

impl PressureTransducerServiceConfig {
    pub const fn new(enabled: bool, period_ms: u32) -> Self {
        Self { enabled, period_ms }
    }
}

impl Default for PressureTransducerServiceConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            period_ms: 20,
        }
    }
}

pub async fn run_pressure_transducer_service<
    M,
    S,
    C,
    D,
    F,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    channel: &'static PressureTransducerSampleChannel<M, DEPTH, SUBS, PUBS>,
    source: &mut S,
    clock: &C,
    delay: &mut D,
    config: PressureTransducerServiceConfig,
    mut on_error: F,
) -> !
where
    M: RawMutex,
    S: PressureTransducerSource,
    C: MonotonicClock,
    D: DelayMs,
    F: FnMut(S::Error),
{
    let period_ms = config.period_ms.max(1);

    loop {
        if config.enabled {
            let timestamp = clock.now();
            match source.read_pressure_transducer_sample().await {
                Ok(sample) => {
                    channel
                        .immediate_publisher()
                        .publish_immediate(PressureTransducerSampleStamped { timestamp, sample });
                }
                Err(error) => on_error(error),
            }
        }

        delay.delay_ms(period_ms).await;
    }
}
