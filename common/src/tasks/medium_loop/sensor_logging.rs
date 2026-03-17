//! Portable Core 0 sensor logging task body.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::pubsub::Subscriber;

use crate::messages::logging::{LogSinkState, LoggedSensor};
use crate::messages::sensor::{
    BarometerSampleStamped, GpsFixSampleStamped, ImuSampleStamped, MagnetometerSampleStamped,
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
    const MAG_DEPTH: usize,
    const MAG_SUBS: usize,
    const MAG_PUBS: usize,
    const GPS_DEPTH: usize,
    const GPS_SUBS: usize,
    const GPS_PUBS: usize,
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
    magnetometer: Option<
        &mut Subscriber<'_, M, MagnetometerSampleStamped, MAG_DEPTH, MAG_SUBS, MAG_PUBS>,
    >,
    gps: Option<&mut Subscriber<'_, M, GpsFixSampleStamped, GPS_DEPTH, GPS_SUBS, GPS_PUBS>>,
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
    let mut magnetometer = magnetometer;
    let mut gps = gps;

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
        if let Some(subscriber) = magnetometer.as_deref_mut() {
            logger.drain_magnetometer(subscriber);
        }
        if let Some(subscriber) = gps.as_deref_mut() {
            logger.drain_gps(subscriber);
        }
        if let Some(receiver) = sink_states {
            logger.drain_sink_states(receiver);
        }
        if let Some(receiver) = sensor_faults {
            logger.drain_sensor_faults(receiver);
        }

        if let Err(error) = logger.emit_snapshot(log_channel, time_source()) {
            if !matches!(
                error,
                SensorSnapshotLoggerError::Queue(TryEnqueueLogError::ChannelFull)
            ) {
                on_error(error);
            }
        }

        delay.delay_ms(logger.period_ms()).await;
    }
}
