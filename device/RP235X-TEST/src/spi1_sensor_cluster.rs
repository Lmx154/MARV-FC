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
            self.next_due = MeasurementTimestamp::from_micros(
                self.next_due.as_micros().saturating_add(period_micros),
            );
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

        if !did_work {
            let delay_ms = next_due_delay_ms(primary_schedule, auxiliary_schedule, clock.now());
            delay.delay_ms(delay_ms).await;
        }
    }
}
