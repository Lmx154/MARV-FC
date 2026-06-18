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
    let mut consecutive_failures: u32 = 0;

    loop {
        if config.enabled {
            poll_magnetometer_service_once(
                channel,
                source,
                clock,
                delay,
                &mut consecutive_failures,
                &mut on_error,
            )
            .await;
        }

        delay.delay_ms(period_ms).await;
    }
}

/// Consecutive read failures tolerated before the service replays the source's
/// hardware bring-up.
const REINIT_AFTER_CONSECUTIVE_FAILURES: u32 = 3;

/// One acquisition step: read + publish, or report the fault and — after enough
/// consecutive failures — replay the source's bring-up so a wedged sensor can
/// recover. Split out from the `-> !` loop so the recovery logic is unit-testable.
async fn poll_magnetometer_service_once<
    M,
    S,
    C,
    D,
    F,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    channel: &MagnetometerSampleChannel<M, DEPTH, SUBS, PUBS>,
    source: &mut S,
    clock: &C,
    delay: &mut D,
    consecutive_failures: &mut u32,
    on_error: &mut F,
) where
    M: RawMutex,
    S: MagnetometerSource,
    C: MonotonicClock,
    D: DelayMs,
    F: FnMut(S::Error),
{
    let timestamp = clock.now();
    match source.read_magnetometer_sample().await {
        Ok(sample) => {
            *consecutive_failures = 0;
            channel
                .immediate_publisher()
                .publish_immediate(MagnetometerSampleStamped { timestamp, sample });
        }
        Err(error) => {
            on_error(error);
            *consecutive_failures = consecutive_failures.saturating_add(1);
            // A wedged sensor won't recover from a bare read retry, so replay the
            // bring-up sequence after a few consecutive failures. Reset the
            // counter whether or not bring-up succeeds, so failures must
            // re-accumulate before the next attempt instead of re-initializing on
            // every iteration. Faults stay visible via `on_error` until a read
            // succeeds again.
            if *consecutive_failures >= REINIT_AFTER_CONSECUTIVE_FAILURES {
                *consecutive_failures = 0;
                if let Err(error) = source.reinitialize(delay).await {
                    on_error(error);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use core::future::Future;

    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embassy_sync::pubsub::WaitResult;
    use futures::executor::block_on;

    use super::{MagnetometerSampleChannel, REINIT_AFTER_CONSECUTIVE_FAILURES,
        poll_magnetometer_service_once};
    use crate::interfaces::sensors::MagnetometerSource;
    use crate::interfaces::timing::MonotonicClock;
    use crate::messages::sensor::MagnetometerSample;
    use crate::utilities::time::MeasurementTimestamp;
    use crate::utils::delay::DelayMs;

    struct FakeClock;
    impl MonotonicClock for FakeClock {
        fn now(&self) -> MeasurementTimestamp {
            MeasurementTimestamp::from_micros(0)
        }
    }

    struct NoopDelay;
    impl DelayMs for NoopDelay {
        async fn delay_ms(&mut self, _ms: u32) {}
    }

    /// Fails its first `fail_reads` reads, then succeeds; counts reinit calls.
    struct FlakyMag {
        fail_reads: u32,
        reinit_calls: u32,
        sample: MagnetometerSample,
    }

    impl MagnetometerSource for FlakyMag {
        type Error = ();

        fn read_magnetometer_sample(
            &mut self,
        ) -> impl Future<Output = Result<MagnetometerSample, Self::Error>> + '_ {
            async move {
                if self.fail_reads > 0 {
                    self.fail_reads -= 1;
                    Err(())
                } else {
                    Ok(self.sample)
                }
            }
        }

        fn reinitialize<D: DelayMs>(
            &mut self,
            _delay: &mut D,
        ) -> impl Future<Output = Result<(), Self::Error>> {
            async move {
                self.reinit_calls += 1;
                Ok(())
            }
        }
    }

    #[test]
    fn reinitializes_after_consecutive_failures_then_recovers() {
        let channel = MagnetometerSampleChannel::<NoopRawMutex, 8, 1, 1>::new();
        let mut subscriber = channel.subscriber().unwrap();
        let clock = FakeClock;
        let mut delay = NoopDelay;
        let mut source = FlakyMag {
            fail_reads: 5,
            reinit_calls: 0,
            sample: MagnetometerSample {
                field_ut: [1.0, 2.0, 3.0],
            },
        };
        let mut consecutive_failures = 0u32;
        let mut error_count = 0u32;

        for _ in 0..8 {
            block_on(poll_magnetometer_service_once(
                &channel,
                &mut source,
                &clock,
                &mut delay,
                &mut consecutive_failures,
                &mut |_err: ()| error_count += 1,
            ));
        }

        // The 5 failed reads are each reported as faults.
        assert_eq!(error_count, 5);
        // Hitting the threshold (3) re-inits exactly once; the counter resets, so
        // the next two failures do not trigger a second re-init before recovery.
        assert_eq!(REINIT_AFTER_CONSECUTIVE_FAILURES, 3);
        assert_eq!(source.reinit_calls, 1);
        // Once reads succeed, the failure counter is cleared and samples publish.
        assert_eq!(consecutive_failures, 0);
        let first = match subscriber.try_next_message().unwrap() {
            WaitResult::Message(message) => message,
            WaitResult::Lagged(_) => panic!("unexpected lag"),
        };
        assert_eq!(first.sample.field_ut, [1.0, 2.0, 3.0]);
    }
}
