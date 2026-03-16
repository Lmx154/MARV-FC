//! Generic IMU producer service using a bounded embassy pub-sub channel.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::pubsub::{ImmediatePublisher, PubSubChannel, Subscriber, WaitResult};

use crate::interfaces::sensors::ImuSource;
use crate::interfaces::timing::MonotonicClock;
use crate::messages::sensor::ImuSampleStamped;
use crate::utils::delay::DelayMs;

pub type ImuSampleChannel<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    PubSubChannel<M, ImuSampleStamped, DEPTH, SUBS, PUBS>;
pub type ImuSamplePublisher<'a, M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    ImmediatePublisher<'a, M, ImuSampleStamped, DEPTH, SUBS, PUBS>;
pub type ImuSampleSubscriber<'a, M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> =
    Subscriber<'a, M, ImuSampleStamped, DEPTH, SUBS, PUBS>;
pub type ImuSampleWaitResult = WaitResult<ImuSampleStamped>;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImuProducerConfig {
    pub period_ms: u32,
}

impl Default for ImuProducerConfig {
    fn default() -> Self {
        Self { period_ms: 1 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ImuProducerError<E> {
    Source(E),
}

pub async fn produce_one_imu_sample<
    M,
    S,
    C,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    channel: &ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
    source: &mut S,
    clock: &C,
) -> Result<(), ImuProducerError<S::Error>>
where
    M: RawMutex,
    S: ImuSource,
    C: MonotonicClock,
{
    let sample = source
        .read_imu_sample()
        .await
        .map_err(ImuProducerError::Source)?;
    let stamped = ImuSampleStamped::new(clock.now(), sample);

    // Slow subscribers must never backpressure the feed-critical acquisition path.
    channel.immediate_publisher().publish_immediate(stamped);
    Ok(())
}

pub async fn run_imu_producer_task<
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
    mut on_error: F,
) -> !
where
    M: RawMutex,
    S: ImuSource,
    C: MonotonicClock,
    D: DelayMs,
    F: FnMut(ImuProducerError<S::Error>),
{
    let period_ms = if config.period_ms == 0 {
        1
    } else {
        config.period_ms
    };

    loop {
        if let Err(error) = produce_one_imu_sample(channel, source, clock).await {
            on_error(error);
        }

        delay.delay_ms(period_ms).await;
    }
}

#[cfg(test)]
mod tests {
    use core::cell::Cell;
    use core::convert::Infallible;

    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embassy_sync::pubsub::WaitResult;
    use futures::executor::block_on;

    use super::{ImuSampleChannel, produce_one_imu_sample};
    use crate::interfaces::sensors::ImuSource;
    use crate::interfaces::timing::MonotonicClock;
    use crate::messages::sensor::{ImuSample, ImuSampleStamped};
    use crate::utilities::time::MeasurementTimestamp;

    struct FakeClock {
        times_micros: [u64; 2],
        index: Cell<usize>,
    }

    impl FakeClock {
        fn new(times_micros: [u64; 2]) -> Self {
            Self {
                times_micros,
                index: Cell::new(0),
            }
        }
    }

    impl MonotonicClock for FakeClock {
        fn now(&self) -> MeasurementTimestamp {
            let index = self.index.get();
            self.index.set(index.saturating_add(1));
            MeasurementTimestamp::from_micros(self.times_micros[index])
        }
    }

    struct FakeSource {
        samples: [ImuSample; 2],
        index: usize,
    }

    impl FakeSource {
        fn new(samples: [ImuSample; 2]) -> Self {
            Self { samples, index: 0 }
        }
    }

    impl ImuSource for FakeSource {
        type Error = Infallible;

        fn read_imu_sample(
            &mut self,
        ) -> impl core::future::Future<Output = Result<ImuSample, Self::Error>> + '_ {
            async move {
                let sample = self.samples[self.index];
                self.index += 1;
                Ok(sample)
            }
        }
    }

    #[test]
    fn publishes_one_sample_to_multiple_subscribers() {
        let channel = ImuSampleChannel::<NoopRawMutex, 4, 2, 1>::new();
        let mut estimator = channel.subscriber().unwrap();
        let mut logger = channel.subscriber().unwrap();
        let expected_sample = ImuSample {
            accel_mps2: [1.0, 2.0, 3.0],
            gyro_rad_s: [4.0, 5.0, 6.0],
        };
        let expected_stamped =
            ImuSampleStamped::new(MeasurementTimestamp::from_micros(100), expected_sample);
        let mut source = FakeSource::new([expected_sample, ImuSample::default()]);
        let clock = FakeClock::new([100, 200]);

        block_on(produce_one_imu_sample(&channel, &mut source, &clock)).unwrap();

        assert_eq!(estimator.try_next_message_pure(), Some(expected_stamped));
        assert_eq!(logger.try_next_message_pure(), Some(expected_stamped));
    }

    #[test]
    fn lagging_subscriber_does_not_block_fast_publisher() {
        let channel = ImuSampleChannel::<NoopRawMutex, 1, 2, 1>::new();
        let mut estimator = channel.subscriber().unwrap();
        let mut logger = channel.subscriber().unwrap();
        let first_sample = ImuSample {
            accel_mps2: [1.0, 0.0, 0.0],
            gyro_rad_s: [0.0, 1.0, 0.0],
        };
        let second_sample = ImuSample {
            accel_mps2: [2.0, 0.0, 0.0],
            gyro_rad_s: [0.0, 2.0, 0.0],
        };
        let mut source = FakeSource::new([first_sample, second_sample]);
        let clock = FakeClock::new([100, 200]);

        block_on(produce_one_imu_sample(&channel, &mut source, &clock)).unwrap();
        assert_eq!(
            estimator.try_next_message_pure(),
            Some(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(100),
                first_sample,
            ))
        );

        block_on(produce_one_imu_sample(&channel, &mut source, &clock)).unwrap();

        assert_eq!(logger.try_next_message(), Some(WaitResult::Lagged(1)));
        assert_eq!(
            logger.try_next_message_pure(),
            Some(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(200),
                second_sample,
            ))
        );
        assert_eq!(
            estimator.try_next_message_pure(),
            Some(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(200),
                second_sample,
            ))
        );
    }
}
