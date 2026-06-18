//! Portable Core 0 sensor logging task body.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::pubsub::{Subscriber, WaitResult};

use crate::messages::logging::{LogSinkState, LoggedSensor};
use crate::messages::sensor::{
    BarometerSampleStamped, GpsFixSampleStamped, ImuSampleStamped, MagnetometerSampleStamped,
    PressureTransducerSampleStamped, TimeSample,
};
use crate::services::logging::{
    LogChannel, SensorSnapshotLogger, SensorSnapshotLoggerError, TryEnqueueLogError,
};
use crate::utilities::time::MeasurementTimestamp;
use crate::utils::delay::DelayMs;

pub async fn run_core0_sensor_logging_task<
    M,
    D,
    F,
    const LOG_DEPTH: usize,
    const IMU_DEPTH: usize,
    const IMU_SUBS: usize,
    const IMU_PUBS: usize,
    const BARO_DEPTH: usize,
    const BARO_SUBS: usize,
    const BARO_PUBS: usize,
    const PRESSURE_DEPTH: usize,
    const PRESSURE_SUBS: usize,
    const PRESSURE_PUBS: usize,
    const MAG_DEPTH: usize,
    const MAG_SUBS: usize,
    const MAG_PUBS: usize,
    const GPS_DEPTH: usize,
    const GPS_SUBS: usize,
    const GPS_PUBS: usize,
    const TIME_DEPTH: usize,
    const TIME_SUBS: usize,
    const TIME_PUBS: usize,
    const STATUS_DEPTH: usize,
    const SENSOR_FAULT_DEPTH: usize,
>(
    logger: &mut SensorSnapshotLogger,
    log_channel: &'static LogChannel<M, LOG_DEPTH>,
    imu: Option<&mut Subscriber<'_, M, ImuSampleStamped, IMU_DEPTH, IMU_SUBS, IMU_PUBS>>,
    aux_imu: Option<&mut Subscriber<'_, M, ImuSampleStamped, IMU_DEPTH, IMU_SUBS, IMU_PUBS>>,
    barometer: Option<
        &mut Subscriber<'_, M, BarometerSampleStamped, BARO_DEPTH, BARO_SUBS, BARO_PUBS>,
    >,
    pressure_transducer: Option<
        &mut Subscriber<
            '_,
            M,
            PressureTransducerSampleStamped,
            PRESSURE_DEPTH,
            PRESSURE_SUBS,
            PRESSURE_PUBS,
        >,
    >,
    magnetometer: Option<
        &mut Subscriber<'_, M, MagnetometerSampleStamped, MAG_DEPTH, MAG_SUBS, MAG_PUBS>,
    >,
    gps: Option<&mut Subscriber<'_, M, GpsFixSampleStamped, GPS_DEPTH, GPS_SUBS, GPS_PUBS>>,
    time: Option<&mut Subscriber<'_, M, TimeSample, TIME_DEPTH, TIME_SUBS, TIME_PUBS>>,
    sink_states: Option<&Receiver<'_, M, LogSinkState, STATUS_DEPTH>>,
    sensor_faults: Option<&Receiver<'_, M, LoggedSensor, SENSOR_FAULT_DEPTH>>,
    delay: &mut D,
    mut time_source: impl FnMut() -> MeasurementTimestamp,
    mut on_error: F,
) -> !
where
    M: RawMutex,
    D: DelayMs,
    F: FnMut(SensorSnapshotLoggerError),
{
    let mut imu = imu;
    let mut aux_imu = aux_imu;
    let mut barometer = barometer;
    let mut pressure_transducer = pressure_transducer;
    let mut magnetometer = magnetometer;
    let mut gps = gps;
    let mut time = time;
    let mut latest_authoritative_time = None;
    let mut last_emitted_timestamp = None;

    // Hold an absolute, fixed-rate cadence so per-iteration work (draining,
    // CSV formatting, enqueue) does not stretch the logging period. Sleeping a
    // full `period_ms` after every pass makes the effective period
    // `period_ms + work`, which drifts the rate below the configured target
    // (e.g. ~99 Hz instead of 100 Hz) and never recovers. Scheduling against an
    // absolute deadline grid keeps the long-run average exactly at the target
    // rate. Delay granularity is whole milliseconds, so individual wakeups jitter
    // by <1 ms, but the grid is anchored to absolute time so that error does not
    // accumulate. The logged `log_us` column still records the true emit time.
    let period_us = (logger.period_ms() as u64).saturating_mul(1_000).max(1_000);
    let mut next_wake_us: Option<u64> = None;

    loop {
        if let Some(subscriber) = imu.as_deref_mut() {
            logger.drain_imu(subscriber);
        }
        if let Some(subscriber) = aux_imu.as_deref_mut() {
            logger.drain_aux_imu(subscriber);
        }
        if let Some(subscriber) = barometer.as_deref_mut() {
            logger.drain_barometer(subscriber);
        }
        if let Some(subscriber) = pressure_transducer.as_deref_mut() {
            logger.drain_pressure_transducer(subscriber);
        }
        if let Some(subscriber) = magnetometer.as_deref_mut() {
            logger.drain_magnetometer(subscriber);
        }
        if let Some(subscriber) = gps.as_deref_mut() {
            logger.drain_gps(subscriber);
        }
        if let Some(subscriber) = time.as_deref_mut() {
            while let Some(message) = subscriber.try_next_message() {
                match message {
                    WaitResult::Lagged(_) => {}
                    WaitResult::Message(sample) => {
                        latest_authoritative_time = Some(sample.timestamp);
                    }
                }
            }
        }
        if let Some(receiver) = sink_states {
            logger.drain_sink_states(receiver);
        }
        if let Some(receiver) = sensor_faults {
            logger.drain_sensor_faults(receiver);
        }

        let snapshot_time = latest_authoritative_time.unwrap_or_else(|| time_source());
        // In authoritative-time (HIL) mode the loop wakes faster than sim time
        // advances, so skip emitting a duplicate row for an unchanged timestamp.
        let is_duplicate_authoritative_sample =
            latest_authoritative_time.is_some() && Some(snapshot_time) == last_emitted_timestamp;

        if !is_duplicate_authoritative_sample {
            if let Err(error) = logger.emit_snapshot(log_channel, snapshot_time) {
                if !matches!(
                    error,
                    SensorSnapshotLoggerError::Queue(TryEnqueueLogError::ChannelFull)
                ) {
                    on_error(error);
                }
            } else {
                last_emitted_timestamp = Some(snapshot_time);
            }
        }

        // Sleep until the next deadline on the absolute period grid.
        let now_us = time_source().as_micros();
        let mut target_us = next_wake_us.unwrap_or(now_us).saturating_add(period_us);
        if target_us <= now_us {
            // Fell behind (work plus any SD back-pressure exceeded the period).
            // Re-anchor to "now" instead of bursting back-to-back rows to catch up.
            target_us = now_us.saturating_add(period_us);
        }
        next_wake_us = Some(target_us);
        // Floor to whole milliseconds; the absolute grid corrects the rounding.
        // `max(1)` only engages in the rare catch-up window, guaranteeing the
        // task always yields to the lower-priority SD sink.
        let remaining_ms = ((target_us - now_us) / 1_000).max(1) as u32;
        delay.delay_ms(remaining_ms).await;
    }
}
