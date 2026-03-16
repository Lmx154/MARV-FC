//! Portable fast-loop task body for IMU sample production.

use embassy_sync::blocking_mutex::raw::RawMutex;

use crate::interfaces::sensors::ImuSource;
use crate::interfaces::timing::MonotonicClock;
use crate::services::acquisition::{
    ImuProducerConfig, ImuProducerError, ImuSampleChannel, run_imu_producer_task,
};
use crate::utils::delay::DelayMs;

pub async fn run_fast_imu_acquisition_task<
    M,
    S,
    C,
    D,
    F,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    channel: &'static ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
    source: &mut S,
    clock: &C,
    delay: &mut D,
    config: ImuProducerConfig,
    on_error: F,
) -> !
where
    M: RawMutex,
    S: ImuSource,
    C: MonotonicClock,
    D: DelayMs,
    F: FnMut(ImuProducerError<S::Error>),
{
    run_imu_producer_task(channel, source, clock, delay, config, on_error).await
}
