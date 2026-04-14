use common::interfaces::sensors::ImuSource;
use common::interfaces::timing::MonotonicClock;
use common::messages::sensor::{ImuSample, ImuSampleStamped};
use common::services::acquisition::ImuSampleChannel;
use common::utilities::time::MeasurementTimestamp;
use common::utils::delay::DelayMs;
use embassy_sync::blocking_mutex::raw::RawMutex;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImuSchedule {
    pub enabled: bool,
    pub period_ms: u32,
}

impl ImuSchedule {
    pub const fn new(enabled: bool, period_ms: u32) -> Self {
        Self { enabled, period_ms }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorSpiClusterConfig {
    pub primary_imu: ImuSchedule,
    pub auxiliary_imu: ImuSchedule,
}

impl SensorSpiClusterConfig {
    pub const fn same_rate(enabled: bool, period_ms: u32) -> Self {
        let schedule = ImuSchedule::new(enabled, period_ms);
        Self {
            primary_imu: schedule,
            auxiliary_imu: schedule,
        }
    }
}

impl Default for SensorSpiClusterConfig {
    fn default() -> Self {
        Self::same_rate(true, 1)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum SensorSpiClusterError<PrimaryError, AuxiliaryError> {
    PrimaryImu(PrimaryError),
    AuxiliaryImu(AuxiliaryError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ScheduledSource {
    enabled: bool,
    period_ms: u32,
    next_due: MeasurementTimestamp,
}

impl ScheduledSource {
    fn new(schedule: ImuSchedule, start: MeasurementTimestamp) -> Self {
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
        (false, false) => return 1,
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

async fn poll_sensor_spi_cluster_once<
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
    F: FnMut(SensorSpiClusterError<Primary::Error, Auxiliary::Error>),
{
    let mut did_work = false;
    let now = clock.now();

    if primary_schedule.is_due(now) {
        did_work = true;
        if let Err(error) =
            read_and_publish_imu_sample(primary_channel, primary_source, clock).await
        {
            on_error(SensorSpiClusterError::PrimaryImu(error));
        }
        primary_schedule.advance(clock.now());
    }

    let now = clock.now();
    if auxiliary_schedule.is_due(now) {
        did_work = true;
        if let Err(error) =
            read_and_publish_imu_sample(auxiliary_channel, auxiliary_source, clock).await
        {
            on_error(SensorSpiClusterError::AuxiliaryImu(error));
        }
        auxiliary_schedule.advance(clock.now());
    }

    did_work
}

pub async fn run_spi1_sensor_cluster<
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
    config: SensorSpiClusterConfig,
    mut on_error: F,
) -> !
where
    M: RawMutex,
    Primary: ImuSource,
    Auxiliary: ImuSource,
    C: MonotonicClock,
    D: DelayMs,
    F: FnMut(SensorSpiClusterError<Primary::Error, Auxiliary::Error>),
{
    let start = clock.now();
    let mut primary_schedule = ScheduledSource::new(config.primary_imu, start);
    let mut auxiliary_schedule = ScheduledSource::new(config.auxiliary_imu, start);

    loop {
        if !poll_sensor_spi_cluster_once(
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
    use std::vec::Vec;

    use super::{
        ImuSchedule, ScheduledSource, SensorSpiClusterConfig, SensorSpiClusterError,
        next_due_delay_ms, poll_sensor_spi_cluster_once, read_and_publish_imu_sample,
    };
    use common::interfaces::sensors::ImuSource;
    use common::interfaces::timing::MonotonicClock;
    use common::messages::sensor::{ImuSample, ImuSampleStamped};
    use common::services::acquisition::ImuSampleChannel;
    use common::utilities::time::MeasurementTimestamp;

    struct SharedClock {
        now_micros: Cell<u64>,
    }

    impl SharedClock {
        fn new(now_micros: u64) -> Self {
            Self {
                now_micros: Cell::new(now_micros),
            }
        }

        fn set(&self, now_micros: u64) {
            self.now_micros.set(now_micros);
        }
    }

    impl MonotonicClock for SharedClock {
        fn now(&self) -> MeasurementTimestamp {
            MeasurementTimestamp::from_micros(self.now_micros.get())
        }
    }

    struct RecordingSource {
        sample: ImuSample,
        reads: RefCell<Vec<u64>>,
        clock: &'static SharedClock,
    }

    impl RecordingSource {
        fn new(sample: ImuSample, clock: &'static SharedClock) -> Self {
            Self {
                sample,
                reads: RefCell::new(Vec::new()),
                clock,
            }
        }
    }

    impl ImuSource for RecordingSource {
        type Error = Infallible;

        fn read_imu_sample(
            &mut self,
        ) -> impl core::future::Future<Output = Result<ImuSample, Self::Error>> + '_ {
            async move {
                self.reads.borrow_mut().push(self.clock.now().as_micros());
                Ok(self.sample)
            }
        }
    }

    #[test]
    fn next_due_delay_prefers_earliest_enabled_source() {
        let now = MeasurementTimestamp::from_micros(5_000);
        let primary = ScheduledSource {
            enabled: true,
            period_ms: 1,
            next_due: MeasurementTimestamp::from_micros(7_500),
        };
        let auxiliary = ScheduledSource {
            enabled: true,
            period_ms: 2,
            next_due: MeasurementTimestamp::from_micros(6_100),
        };

        assert_eq!(next_due_delay_ms(primary, auxiliary, now), 2);

        let disabled = ScheduledSource {
            enabled: false,
            period_ms: 1,
            next_due: now,
        };
        assert_eq!(next_due_delay_ms(disabled, disabled, now), 1);
    }

    #[test]
    fn poll_sensor_spi_cluster_respects_individual_schedules() {
        static CLOCK: SharedClock = SharedClock {
            now_micros: Cell::new(0),
        };

        let channel_a = ImuSampleChannel::<NoopRawMutex, 8, 1, 1>::new();
        let channel_b = ImuSampleChannel::<NoopRawMutex, 8, 1, 1>::new();
        let mut subscriber_a = channel_a.subscriber().unwrap();
        let mut subscriber_b = channel_b.subscriber().unwrap();

        let mut source_a = RecordingSource::new(
            ImuSample {
                accel_mps2: [1.0, 0.0, 0.0],
                gyro_rad_s: [0.1, 0.0, 0.0],
            },
            &CLOCK,
        );
        let mut source_b = RecordingSource::new(
            ImuSample {
                accel_mps2: [2.0, 0.0, 0.0],
                gyro_rad_s: [0.2, 0.0, 0.0],
            },
            &CLOCK,
        );

        CLOCK.set(0);
        let mut schedule_a = ScheduledSource::new(ImuSchedule::new(true, 1), CLOCK.now());
        let mut schedule_b = ScheduledSource::new(ImuSchedule::new(true, 2), CLOCK.now());
        let mut errors = Vec::<SensorSpiClusterError<Infallible, Infallible>>::new();

        block_on(poll_sensor_spi_cluster_once(
            &channel_a,
            &mut source_a,
            &mut schedule_a,
            &channel_b,
            &mut source_b,
            &mut schedule_b,
            &CLOCK,
            &mut |error| errors.push(error),
        ));

        assert!(errors.is_empty());
        let first_a = subscriber_a.try_next_message().unwrap().unwrap_message();
        let first_b = subscriber_b.try_next_message().unwrap().unwrap_message();
        assert_eq!(first_a.timestamp, MeasurementTimestamp::from_micros(0));
        assert_eq!(first_b.timestamp, MeasurementTimestamp::from_micros(0));

        CLOCK.set(1_100);
        block_on(poll_sensor_spi_cluster_once(
            &channel_a,
            &mut source_a,
            &mut schedule_a,
            &channel_b,
            &mut source_b,
            &mut schedule_b,
            &CLOCK,
            &mut |error| errors.push(error),
        ));

        let second_a = subscriber_a.try_next_message().unwrap().unwrap_message();
        assert_eq!(second_a.timestamp, MeasurementTimestamp::from_micros(1_100));
        assert!(subscriber_b.try_next_message().is_none());

        CLOCK.set(2_200);
        block_on(poll_sensor_spi_cluster_once(
            &channel_a,
            &mut source_a,
            &mut schedule_a,
            &channel_b,
            &mut source_b,
            &mut schedule_b,
            &CLOCK,
            &mut |error| errors.push(error),
        ));

        let third_a = subscriber_a.try_next_message().unwrap().unwrap_message();
        let second_b = subscriber_b.try_next_message().unwrap().unwrap_message();
        assert_eq!(third_a.timestamp, MeasurementTimestamp::from_micros(2_200));
        assert_eq!(second_b.timestamp, MeasurementTimestamp::from_micros(2_200));
    }

    #[test]
    fn sensor_spi_cluster_config_same_rate_sets_both_imus() {
        let config = SensorSpiClusterConfig::same_rate(true, 4);
        assert_eq!(config.primary_imu, ImuSchedule::new(true, 4));
        assert_eq!(config.auxiliary_imu, ImuSchedule::new(true, 4));
    }

    #[test]
    fn read_and_publish_imu_sample_uses_measurement_timestamp() {
        static CLOCK: SharedClock = SharedClock {
            now_micros: Cell::new(123_456),
        };

        let channel = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let mut subscriber = channel.subscriber().unwrap();
        let sample = ImuSample {
            accel_mps2: [3.0, 2.0, 1.0],
            gyro_rad_s: [0.3, 0.2, 0.1],
        };
        let mut source = RecordingSource::new(sample, &CLOCK);

        block_on(read_and_publish_imu_sample(&channel, &mut source, &CLOCK)).unwrap();

        let message: ImuSampleStamped = subscriber.try_next_message().unwrap().unwrap_message();
        assert_eq!(
            message.timestamp,
            MeasurementTimestamp::from_micros(123_456)
        );
        assert_eq!(message.sample, sample);
    }
}
