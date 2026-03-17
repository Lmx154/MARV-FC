//! Portable orchestration body for a tightly coordinated multi-IMU service.

use embassy_sync::blocking_mutex::raw::RawMutex;

use crate::interfaces::sensors::ImuSource;
use crate::interfaces::timing::MonotonicClock;
use crate::utils::delay::DelayMs;

use super::imu_producer::{
    ImuProducerConfig, ImuProducerError, ImuSampleChannel, produce_one_imu_sample,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum DualImuServiceError<PrimaryError, AuxiliaryError> {
    Primary(ImuProducerError<PrimaryError>),
    Auxiliary(ImuProducerError<AuxiliaryError>),
}

pub async fn run_dual_imu_service<
    M,
    Primary,
    Auxiliary,
    C,
    D,
    F,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    primary_channel: &'static ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
    primary_source: &mut Primary,
    primary_enabled: bool,
    auxiliary_channel: &'static ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
    auxiliary_source: &mut Auxiliary,
    auxiliary_enabled: bool,
    clock: &C,
    delay: &mut D,
    config: ImuProducerConfig,
    mut on_error: F,
) -> !
where
    M: RawMutex,
    Primary: ImuSource,
    Auxiliary: ImuSource,
    C: MonotonicClock,
    D: DelayMs,
    F: FnMut(DualImuServiceError<Primary::Error, Auxiliary::Error>),
{
    let period_ms = config.period_ms.max(1);

    loop {
        if primary_enabled {
            if let Err(error) = produce_one_imu_sample(primary_channel, primary_source, clock).await
            {
                on_error(DualImuServiceError::Primary(error));
            }
        }

        if auxiliary_enabled {
            if let Err(error) =
                produce_one_imu_sample(auxiliary_channel, auxiliary_source, clock).await
            {
                on_error(DualImuServiceError::Auxiliary(error));
            }
        }

        delay.delay_ms(period_ms).await;
    }
}
