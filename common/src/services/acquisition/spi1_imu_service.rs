//! Portable SPI1 IMU owner service with explicit per-source scheduling.

use embassy_sync::blocking_mutex::raw::RawMutex;

use crate::interfaces::sensors::ImuSource;
use crate::interfaces::timing::MonotonicClock;
use crate::messages::sensor::{ImuSample, ImuSampleStamped};
use crate::utilities::time::MeasurementTimestamp;
use crate::utils::delay::DelayMs;

use super::sample_channels::ImuSampleChannel;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Spi1ImuSourceSchedule {
    pub enabled: bool,
    pub period_ms: u32,
}

impl Spi1ImuSourceSchedule {
    pub const fn new(enabled: bool, period_ms: u32) -> Self {
        Self { enabled, period_ms }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Spi1ImuServiceConfig {
    pub primary: Spi1ImuSourceSchedule,
    pub auxiliary: Spi1ImuSourceSchedule,
}

impl Spi1ImuServiceConfig {
    pub const fn same_rate(enabled: bool, period_ms: u32) -> Self {
        let schedule = Spi1ImuSourceSchedule::new(enabled, period_ms);
        Self {
            primary: schedule,
            auxiliary: schedule,
        }
    }
}

impl Default for Spi1ImuServiceConfig {
    fn default() -> Self {
        Self::same_rate(true, 1)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Spi1ImuServiceError<PrimaryError, AuxiliaryError> {
    Primary(PrimaryError),
    Auxiliary(AuxiliaryError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ScheduledSource {
    enabled: bool,
    period_ms: u32,
    next_due: MeasurementTimestamp,
}

impl ScheduledSource {
    fn new(schedule: Spi1ImuSourceSchedule, start: MeasurementTimestamp) -> Self {
        Self {
            enabled: schedule.enabled,
            period_ms: schedule.period_ms.max(1),
            next_due: start,
        }
    }

    fn is_due(self, now: MeasurementTimestamp) -> bool {
        self.enabled && now >= self.next_due
    }

    fn advance(&mut self, after: MeasurementTimestamp) {
        if !self.enabled {
            return;
        }

        let period_micros = u64::from(self.period_ms) * 1_000;

        loop {
            let advanced = MeasurementTimestamp::from_micros(
                self.next_due.as_micros().saturating_add(period_micros),
            );
            self.next_due = advanced;

            if self.next_due > after {
                break;
            }
        }
    }
}

fn next_due_delay_ms(
    primary: ScheduledSource,
    auxiliary: ScheduledSource,
    now: MeasurementTimestamp,
) -> u32 {
    let next_due = match (primary.enabled, auxiliary.enabled) {
        (true, true) => primary.next_due.min(auxiliary.next_due),
        (true, false) => primary.next_due,
        (false, true) => auxiliary.next_due,
        (false, false) => {
            return 1;
        }
    };

    let remaining_micros = next_due.saturating_delta_since(now).as_micros();
    remaining_micros.div_ceil(1_000).max(1) as u32
}

fn publish_imu_sample<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
    channel: &ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
    timestamp: MeasurementTimestamp,
    sample: ImuSample,
) where
    M: RawMutex,
{
    // Slow subscribers must never backpressure the feed-critical acquisition path.
    channel
        .immediate_publisher()
        .publish_immediate(ImuSampleStamped::new(timestamp, sample));
}

async fn read_and_publish_imu_sample<
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
) -> Result<(), S::Error>
where
    M: RawMutex,
    S: ImuSource,
    C: MonotonicClock,
{
    let timestamp = clock.now();
    let sample = source.read_imu_sample().await?;
    publish_imu_sample(channel, timestamp, sample);
    Ok(())
}

async fn poll_spi1_imu_service_once<
    'a,
    M,
    Primary,
    Auxiliary,
    C,
    F,
    const DEPTH: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    primary_channel: &'a ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
    primary_source: &mut Primary,
    primary_schedule: &mut ScheduledSource,
    auxiliary_channel: &'a ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
    auxiliary_source: &mut Auxiliary,
    auxiliary_schedule: &mut ScheduledSource,
    clock: &C,
    on_error: &mut F,
) -> bool
where
    M: RawMutex,
    Primary: ImuSource,
    Auxiliary: ImuSource,
    C: MonotonicClock,
    F: FnMut(Spi1ImuServiceError<Primary::Error, Auxiliary::Error>),
{
    let mut did_work = false;
    let now = clock.now();

    if primary_schedule.is_due(now) {
        did_work = true;
        if let Err(error) =
            read_and_publish_imu_sample(primary_channel, primary_source, clock).await
        {
            on_error(Spi1ImuServiceError::Primary(error));
        }
        primary_schedule.advance(clock.now());
    }

    let now = clock.now();
    if auxiliary_schedule.is_due(now) {
        did_work = true;
        if let Err(error) =
            read_and_publish_imu_sample(auxiliary_channel, auxiliary_source, clock).await
        {
            on_error(Spi1ImuServiceError::Auxiliary(error));
        }
        auxiliary_schedule.advance(clock.now());
    }

    did_work
}

pub async fn run_spi1_imu_service<
    'a,
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
    primary_channel: &'a ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
    primary_source: &mut Primary,
    auxiliary_channel: &'a ImuSampleChannel<M, DEPTH, SUBS, PUBS>,
    auxiliary_source: &mut Auxiliary,
    clock: &C,
    delay: &mut D,
    config: Spi1ImuServiceConfig,
    mut on_error: F,
) -> !
where
    M: RawMutex,
    Primary: ImuSource,
    Auxiliary: ImuSource,
    C: MonotonicClock,
    D: DelayMs,
    F: FnMut(Spi1ImuServiceError<Primary::Error, Auxiliary::Error>),
{
    let start = clock.now();
    let mut primary_schedule = ScheduledSource::new(config.primary, start);
    let mut auxiliary_schedule = ScheduledSource::new(config.auxiliary, start);

    loop {
        if !poll_spi1_imu_service_once(
            primary_channel,
            primary_source,
            &mut primary_schedule,
            auxiliary_channel,
            auxiliary_source,
            &mut auxiliary_schedule,
            clock,
            &mut on_error,
        )
        .await
        {
            let delay_ms = next_due_delay_ms(primary_schedule, auxiliary_schedule, clock.now());
            delay.delay_ms(delay_ms).await;
        }
    }
}

#[cfg(test)]
mod tests {
    use core::cell::{Cell, RefCell};
    use core::convert::Infallible;

    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use futures::executor::block_on;

    use super::{
        ScheduledSource, Spi1ImuServiceError, Spi1ImuSourceSchedule, next_due_delay_ms,
        poll_spi1_imu_service_once, read_and_publish_imu_sample,
    };
    use crate::interfaces::sensors::ImuSource;
    use crate::interfaces::timing::MonotonicClock;
    use crate::messages::sensor::{ImuSample, ImuSampleStamped};
    use crate::services::acquisition::ImuSampleChannel;
    use crate::utilities::time::MeasurementTimestamp;

    struct SharedClock {
        now_micros: Cell<u64>,
    }

    impl SharedClock {
        fn new(now_micros: u64) -> Self {
            Self {
                now_micros: Cell::new(now_micros),
            }
        }

        fn advance_micros(&self, delta_micros: u64) {
            self.now_micros
                .set(self.now_micros.get().saturating_add(delta_micros));
        }
    }

    impl MonotonicClock for SharedClock {
        fn now(&self) -> MeasurementTimestamp {
            MeasurementTimestamp::from_micros(self.now_micros.get())
        }
    }

    struct RecordingSource<'a> {
        sample: ImuSample,
        clock: &'a SharedClock,
        read_times: &'a RefCell<Vec<u64>>,
        transaction_time_micros: u64,
    }

    struct TimeAdvancingSource<'a> {
        sample: ImuSample,
        clock: &'a SharedClock,
        read_started_at: &'a RefCell<Vec<u64>>,
        read_finished_at: &'a RefCell<Vec<u64>>,
        finish_time_micros: u64,
    }

    impl ImuSource for RecordingSource<'_> {
        type Error = Infallible;

        fn read_imu_sample(
            &mut self,
        ) -> impl core::future::Future<Output = Result<ImuSample, Self::Error>> + '_ {
            async move {
                self.read_times
                    .borrow_mut()
                    .push(self.clock.now().as_micros());
                self.clock.advance_micros(self.transaction_time_micros);
                Ok(self.sample)
            }
        }
    }

    impl ImuSource for TimeAdvancingSource<'_> {
        type Error = Infallible;

        fn read_imu_sample(
            &mut self,
        ) -> impl core::future::Future<Output = Result<ImuSample, Self::Error>> + '_ {
            async move {
                self.read_started_at
                    .borrow_mut()
                    .push(self.clock.now().as_micros());
                self.clock.now_micros.set(self.finish_time_micros);
                self.read_finished_at
                    .borrow_mut()
                    .push(self.clock.now().as_micros());
                Ok(self.sample)
            }
        }
    }

    #[test]
    fn scheduler_applies_independent_per_source_cadence() {
        let channel_a = ImuSampleChannel::<NoopRawMutex, 8, 1, 1>::new();
        let channel_b = ImuSampleChannel::<NoopRawMutex, 8, 1, 1>::new();
        let mut subscriber_a = channel_a.subscriber().unwrap();
        let mut subscriber_b = channel_b.subscriber().unwrap();
        let clock = SharedClock::new(0);
        let primary_reads = RefCell::new(Vec::new());
        let auxiliary_reads = RefCell::new(Vec::new());
        let mut primary = RecordingSource {
            sample: ImuSample {
                accel_mps2: [1.0, 0.0, 0.0],
                gyro_rad_s: [0.0, 1.0, 0.0],
            },
            clock: &clock,
            read_times: &primary_reads,
            transaction_time_micros: 100,
        };
        let mut auxiliary = RecordingSource {
            sample: ImuSample {
                accel_mps2: [2.0, 0.0, 0.0],
                gyro_rad_s: [0.0, 2.0, 0.0],
            },
            clock: &clock,
            read_times: &auxiliary_reads,
            transaction_time_micros: 100,
        };
        let mut primary_schedule = ScheduledSource::new(
            Spi1ImuSourceSchedule::new(true, 1),
            MeasurementTimestamp::from_micros(0),
        );
        let mut auxiliary_schedule = ScheduledSource::new(
            Spi1ImuSourceSchedule::new(true, 2),
            MeasurementTimestamp::from_micros(0),
        );
        let mut errors = Vec::<Spi1ImuServiceError<Infallible, Infallible>>::new();

        block_on(poll_spi1_imu_service_once(
            &channel_a,
            &mut primary,
            &mut primary_schedule,
            &channel_b,
            &mut auxiliary,
            &mut auxiliary_schedule,
            &clock,
            &mut |error| errors.push(error),
        ));
        clock.advance_micros(900);

        block_on(poll_spi1_imu_service_once(
            &channel_a,
            &mut primary,
            &mut primary_schedule,
            &channel_b,
            &mut auxiliary,
            &mut auxiliary_schedule,
            &clock,
            &mut |error| errors.push(error),
        ));
        clock.advance_micros(1_000);

        block_on(poll_spi1_imu_service_once(
            &channel_a,
            &mut primary,
            &mut primary_schedule,
            &channel_b,
            &mut auxiliary,
            &mut auxiliary_schedule,
            &clock,
            &mut |error| errors.push(error),
        ));

        assert!(errors.is_empty());
        assert_eq!(primary_reads.borrow().as_slice(), &[0, 1_100, 2_200]);
        assert_eq!(auxiliary_reads.borrow().as_slice(), &[100, 2_300]);
        assert_eq!(
            subscriber_a.try_next_message_pure(),
            Some(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(0),
                primary.sample,
            ))
        );
        assert_eq!(
            subscriber_a.try_next_message_pure(),
            Some(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(1_100),
                primary.sample,
            ))
        );
        assert_eq!(
            subscriber_a.try_next_message_pure(),
            Some(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(2_200),
                primary.sample,
            ))
        );
        assert_eq!(
            subscriber_b.try_next_message_pure(),
            Some(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(100),
                auxiliary.sample,
            ))
        );
        assert_eq!(
            subscriber_b.try_next_message_pure(),
            Some(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(2_300),
                auxiliary.sample,
            ))
        );
    }

    #[test]
    fn next_due_delay_rounds_up_to_next_millisecond() {
        let primary = ScheduledSource::new(
            Spi1ImuSourceSchedule::new(true, 2),
            MeasurementTimestamp::from_micros(1_500),
        );
        let auxiliary = ScheduledSource::new(
            Spi1ImuSourceSchedule::new(false, 1),
            MeasurementTimestamp::from_micros(0),
        );

        assert_eq!(
            next_due_delay_ms(primary, auxiliary, MeasurementTimestamp::from_micros(600)),
            1
        );
        assert_eq!(
            next_due_delay_ms(primary, auxiliary, MeasurementTimestamp::from_micros(0)),
            2
        );
    }

    #[test]
    fn sample_timestamp_is_captured_before_source_read_completes() {
        let channel = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let mut subscriber = channel.subscriber().unwrap();
        let clock = SharedClock::new(100);
        let read_started_at = RefCell::new(Vec::new());
        let read_finished_at = RefCell::new(Vec::new());
        let expected_sample = ImuSample {
            accel_mps2: [1.0, 2.0, 3.0],
            gyro_rad_s: [4.0, 5.0, 6.0],
        };
        let mut source = TimeAdvancingSource {
            sample: expected_sample,
            clock: &clock,
            read_started_at: &read_started_at,
            read_finished_at: &read_finished_at,
            finish_time_micros: 250,
        };

        block_on(read_and_publish_imu_sample(&channel, &mut source, &clock)).unwrap();

        assert_eq!(read_started_at.borrow().as_slice(), &[100]);
        assert_eq!(read_finished_at.borrow().as_slice(), &[250]);
        assert_eq!(
            subscriber.try_next_message_pure(),
            Some(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(100),
                expected_sample,
            ))
        );
    }
}
