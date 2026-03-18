//! Same-core sensor snapshot logging with freshness and sink-state tracking.

use core::fmt::Write;

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Receiver;
use embassy_sync::pubsub::{Subscriber, WaitResult};
use heapless::String;

use crate::interfaces::storage::{LogError, LogLine, LogPath, LoggerEngine};
use crate::messages::logging::{
    LogSinkState, LoggedSensor, SensorLogField, SensorLogSnapshot, SensorLogState,
};
use crate::messages::sensor::{
    BarometerSample, BarometerSampleStamped, GpsFixSample, GpsFixSampleStamped, ImuSample,
    ImuSampleStamped, MagnetometerSample, MagnetometerSampleStamped, PressureTransducerSample,
    PressureTransducerSampleStamped,
};
use crate::services::logging::{LogChannel, TryEnqueueLogError, try_enqueue_line};
use crate::utilities::time::MeasurementTimestamp;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorSnapshotLogFlags {
    pub include_sink_state: bool,
    pub include_sensor_state: bool,
    pub include_sample_timestamps: bool,
    pub include_lag_counters: bool,
    pub mark_stale_data: bool,
}

impl Default for SensorSnapshotLogFlags {
    fn default() -> Self {
        Self {
            include_sink_state: true,
            include_sensor_state: true,
            include_sample_timestamps: true,
            include_lag_counters: true,
            mark_stale_data: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorSnapshotSensorConfig {
    pub imu: bool,
    pub aux_imu: bool,
    pub barometer: bool,
    pub pressure_transducer: bool,
    pub magnetometer: bool,
    pub gps: bool,
}

impl Default for SensorSnapshotSensorConfig {
    fn default() -> Self {
        Self {
            imu: true,
            aux_imu: false,
            barometer: true,
            pressure_transducer: false,
            magnetometer: true,
            gps: true,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorSnapshotLoggerConfig {
    pub period_ms: u32,
    pub emit_header: bool,
    pub sensors: SensorSnapshotSensorConfig,
    pub flags: SensorSnapshotLogFlags,
}

impl Default for SensorSnapshotLoggerConfig {
    fn default() -> Self {
        Self {
            period_ms: 10,
            emit_header: true,
            sensors: SensorSnapshotSensorConfig::default(),
            flags: SensorSnapshotLogFlags::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum SensorSnapshotLoggerError {
    Path(LogError),
    Queue(TryEnqueueLogError),
    Format,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct SensorSlot<T> {
    last_sample: Option<T>,
    fresh_since_emit: bool,
    faulted: bool,
    lagged_since_emit: u32,
}

impl<T> Default for SensorSlot<T> {
    fn default() -> Self {
        Self {
            last_sample: None,
            fresh_since_emit: false,
            faulted: false,
            lagged_since_emit: 0,
        }
    }
}

pub struct SensorSnapshotLogger {
    path: LogPath,
    config: SensorSnapshotLoggerConfig,
    header_emitted: bool,
    sink_state: LogSinkState,
    imu: SensorSlot<ImuSampleStamped>,
    aux_imu: SensorSlot<ImuSampleStamped>,
    barometer: SensorSlot<BarometerSampleStamped>,
    pressure_transducer: SensorSlot<PressureTransducerSampleStamped>,
    magnetometer: SensorSlot<MagnetometerSampleStamped>,
    gps: SensorSlot<GpsFixSampleStamped>,
}

impl SensorSnapshotLogger {
    pub fn new(path: &str, config: SensorSnapshotLoggerConfig) -> Result<Self, LogError> {
        let mut owned_path = LogPath::new();
        owned_path
            .push_str(path)
            .map_err(|_| LogError::PathTooLong)?;

        Ok(Self {
            path: owned_path,
            config: SensorSnapshotLoggerConfig {
                period_ms: config.period_ms.max(1),
                ..config
            },
            header_emitted: false,
            sink_state: LogSinkState::Healthy,
            imu: SensorSlot::default(),
            aux_imu: SensorSlot::default(),
            barometer: SensorSlot::default(),
            pressure_transducer: SensorSlot::default(),
            magnetometer: SensorSlot::default(),
            gps: SensorSlot::default(),
        })
    }

    pub fn path(&self) -> &str {
        self.path.as_str()
    }

    pub fn config(&self) -> SensorSnapshotLoggerConfig {
        self.config
    }

    pub fn period_ms(&self) -> u32 {
        self.config.period_ms
    }

    pub fn sensor_enabled(&self, sensor: LoggedSensor) -> bool {
        match sensor {
            LoggedSensor::Imu => self.config.sensors.imu,
            LoggedSensor::AuxImu => self.config.sensors.aux_imu,
            LoggedSensor::Barometer => self.config.sensors.barometer,
            LoggedSensor::PressureTransducer => self.config.sensors.pressure_transducer,
            LoggedSensor::Magnetometer => self.config.sensors.magnetometer,
            LoggedSensor::Gps => self.config.sensors.gps,
        }
    }

    pub fn sink_state(&self) -> LogSinkState {
        self.sink_state
    }

    pub fn note_sink_state(&mut self, state: LogSinkState) {
        self.sink_state = state;
    }

    pub fn note_sensor_fault(&mut self, sensor: LoggedSensor) {
        self.slot_mut(sensor).set_faulted(true);
    }

    pub fn note_sensor_recovered(&mut self, sensor: LoggedSensor) {
        self.slot_mut(sensor).set_faulted(false);
    }

    pub fn note_sink_error(&mut self, error: LogError) {
        self.note_sink_state(error.into());
    }

    pub fn note_sink_backpressured(&mut self) {
        self.note_sink_state(LogSinkState::Backpressured);
    }

    pub fn note_sink_recovered(&mut self) {
        self.note_sink_state(LogSinkState::Healthy);
    }

    pub fn drain_sink_states<M, const DEPTH: usize>(
        &mut self,
        receiver: &Receiver<'_, M, LogSinkState, DEPTH>,
    ) where
        M: RawMutex,
    {
        while let Ok(state) = receiver.try_receive() {
            self.note_sink_state(state);
        }
    }

    pub fn drain_sensor_faults<M, const DEPTH: usize>(
        &mut self,
        receiver: &Receiver<'_, M, LoggedSensor, DEPTH>,
    ) where
        M: RawMutex,
    {
        while let Ok(sensor) = receiver.try_receive() {
            self.note_sensor_fault(sensor);
        }
    }

    pub fn drain_imu<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
        &mut self,
        subscriber: &mut Subscriber<'_, M, ImuSampleStamped, DEPTH, SUBS, PUBS>,
    ) where
        M: RawMutex,
    {
        if self.config.sensors.imu {
            drain_subscriber(subscriber, &mut self.imu);
        }
    }

    pub fn drain_aux_imu<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
        &mut self,
        subscriber: &mut Subscriber<'_, M, ImuSampleStamped, DEPTH, SUBS, PUBS>,
    ) where
        M: RawMutex,
    {
        if self.config.sensors.aux_imu {
            drain_subscriber(subscriber, &mut self.aux_imu);
        }
    }

    pub fn drain_barometer<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
        &mut self,
        subscriber: &mut Subscriber<'_, M, BarometerSampleStamped, DEPTH, SUBS, PUBS>,
    ) where
        M: RawMutex,
    {
        if self.config.sensors.barometer {
            drain_subscriber(subscriber, &mut self.barometer);
        }
    }

    pub fn drain_pressure_transducer<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
        &mut self,
        subscriber: &mut Subscriber<'_, M, PressureTransducerSampleStamped, DEPTH, SUBS, PUBS>,
    ) where
        M: RawMutex,
    {
        if self.config.sensors.pressure_transducer {
            drain_subscriber(subscriber, &mut self.pressure_transducer);
        }
    }

    pub fn drain_magnetometer<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
        &mut self,
        subscriber: &mut Subscriber<'_, M, MagnetometerSampleStamped, DEPTH, SUBS, PUBS>,
    ) where
        M: RawMutex,
    {
        if self.config.sensors.magnetometer {
            drain_subscriber(subscriber, &mut self.magnetometer);
        }
    }

    pub fn drain_gps<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
        &mut self,
        subscriber: &mut Subscriber<'_, M, GpsFixSampleStamped, DEPTH, SUBS, PUBS>,
    ) where
        M: RawMutex,
    {
        if self.config.sensors.gps {
            drain_subscriber(subscriber, &mut self.gps);
        }
    }

    pub fn snapshot(&self, timestamp: MeasurementTimestamp) -> SensorLogSnapshot {
        SensorLogSnapshot {
            timestamp,
            sink_state: self.sink_state,
            imu: build_field(&self.imu, self.config.flags.mark_stale_data, |sample| {
                (sample.timestamp, sample.sample)
            }),
            aux_imu: build_field(&self.aux_imu, self.config.flags.mark_stale_data, |sample| {
                (sample.timestamp, sample.sample)
            }),
            barometer: build_field(
                &self.barometer,
                self.config.flags.mark_stale_data,
                |sample| (sample.timestamp, sample.sample),
            ),
            pressure_transducer: build_field(
                &self.pressure_transducer,
                self.config.flags.mark_stale_data,
                |sample| (sample.timestamp, sample.sample),
            ),
            magnetometer: build_field(
                &self.magnetometer,
                self.config.flags.mark_stale_data,
                |sample| (sample.timestamp, sample.sample),
            ),
            gps: build_field(&self.gps, self.config.flags.mark_stale_data, |sample| {
                (sample.timestamp, sample.sample)
            }),
        }
    }

    pub fn mark_snapshot_emitted(&mut self) {
        reset_slot_for_next_emit(&mut self.imu);
        reset_slot_for_next_emit(&mut self.aux_imu);
        reset_slot_for_next_emit(&mut self.barometer);
        reset_slot_for_next_emit(&mut self.pressure_transducer);
        reset_slot_for_next_emit(&mut self.magnetometer);
        reset_slot_for_next_emit(&mut self.gps);
    }

    pub fn format_header_line(&self) -> Result<LogLine, SensorSnapshotLoggerError> {
        let mut line = LogLine::new();
        write!(&mut line, "log_us").map_err(|_| SensorSnapshotLoggerError::Format)?;

        if self.config.flags.include_sink_state {
            write!(&mut line, ",sink_state").map_err(|_| SensorSnapshotLoggerError::Format)?;
        }

        if self.config.sensors.imu {
            write_sensor_header(&mut line, "imu", &self.config.flags)?;
            write!(
                &mut line,
                ",imu_ax_mps2,imu_ay_mps2,imu_az_mps2,imu_gx_rad_s,imu_gy_rad_s,imu_gz_rad_s"
            )
            .map_err(|_| SensorSnapshotLoggerError::Format)?;
        }
        if self.config.sensors.aux_imu {
            write_sensor_header(&mut line, "aux_imu", &self.config.flags)?;
            write!(
                &mut line,
                ",aux_imu_ax_mps2,aux_imu_ay_mps2,aux_imu_az_mps2,aux_imu_gx_rad_s,aux_imu_gy_rad_s,aux_imu_gz_rad_s"
            )
            .map_err(|_| SensorSnapshotLoggerError::Format)?;
        }
        if self.config.sensors.barometer {
            write_sensor_header(&mut line, "baro", &self.config.flags)?;
            write!(&mut line, ",baro_pressure_pa,baro_temp_c")
                .map_err(|_| SensorSnapshotLoggerError::Format)?;
        }
        if self.config.sensors.pressure_transducer {
            write_sensor_header(&mut line, "pressure_transducer", &self.config.flags)?;
            write!(
                &mut line,
                ",pressure_transducer_psi,pressure_transducer_kpa"
            )
            .map_err(|_| SensorSnapshotLoggerError::Format)?;
        }
        if self.config.sensors.magnetometer {
            write_sensor_header(&mut line, "mag", &self.config.flags)?;
            write!(&mut line, ",mag_x_ut,mag_y_ut,mag_z_ut")
                .map_err(|_| SensorSnapshotLoggerError::Format)?;
        }
        if self.config.sensors.gps {
            write_sensor_header(&mut line, "gps", &self.config.flags)?;
            write!(
                &mut line,
                ",gps_lat_deg,gps_lon_deg,gps_alt_m,gps_vn_mps,gps_ve_mps,gps_vd_mps,gps_sats"
            )
            .map_err(|_| SensorSnapshotLoggerError::Format)?;
        }

        Ok(line)
    }

    pub fn format_snapshot_line(
        &self,
        snapshot: &SensorLogSnapshot,
    ) -> Result<LogLine, SensorSnapshotLoggerError> {
        let mut line = LogLine::new();
        write!(&mut line, "{}", snapshot.timestamp.as_micros())
            .map_err(|_| SensorSnapshotLoggerError::Format)?;

        if self.config.flags.include_sink_state {
            write!(&mut line, ",{}", snapshot.sink_state.as_str())
                .map_err(|_| SensorSnapshotLoggerError::Format)?;
        }

        if self.config.sensors.imu {
            write_imu_field(&mut line, &snapshot.imu, &self.config.flags)?;
        }
        if self.config.sensors.aux_imu {
            write_imu_field(&mut line, &snapshot.aux_imu, &self.config.flags)?;
        }
        if self.config.sensors.barometer {
            write_barometer_field(&mut line, &snapshot.barometer, &self.config.flags)?;
        }
        if self.config.sensors.pressure_transducer {
            write_pressure_transducer_field(
                &mut line,
                &snapshot.pressure_transducer,
                &self.config.flags,
            )?;
        }
        if self.config.sensors.magnetometer {
            write_magnetometer_field(&mut line, &snapshot.magnetometer, &self.config.flags)?;
        }
        if self.config.sensors.gps {
            write_gps_field(&mut line, &snapshot.gps, &self.config.flags)?;
        }

        Ok(line)
    }

    pub fn emit_snapshot<M, const DEPTH: usize>(
        &mut self,
        channel: &LogChannel<M, DEPTH>,
        timestamp: MeasurementTimestamp,
    ) -> Result<(), SensorSnapshotLoggerError>
    where
        M: RawMutex,
    {
        if self.config.emit_header && !self.header_emitted {
            let header = self.format_header_line()?;
            try_enqueue_line(channel, self.path.as_str(), header.as_str()).map_err(|error| {
                self.update_sink_state_from_queue_error(error);
                SensorSnapshotLoggerError::Queue(error)
            })?;
            self.header_emitted = true;
        }

        let snapshot = self.snapshot(timestamp);
        let line = self.format_snapshot_line(&snapshot)?;
        try_enqueue_line(channel, self.path.as_str(), line.as_str()).map_err(|error| {
            self.update_sink_state_from_queue_error(error);
            SensorSnapshotLoggerError::Queue(error)
        })?;
        self.mark_snapshot_emitted();
        Ok(())
    }

    pub fn append_snapshot<E>(
        &mut self,
        engine: &mut E,
        timestamp: MeasurementTimestamp,
    ) -> Result<(), SensorSnapshotLoggerError>
    where
        E: LoggerEngine,
    {
        if self.config.emit_header && !self.header_emitted {
            let header = self.format_header_line()?;
            engine
                .append_line(self.path.as_str(), header.as_str())
                .map_err(|error| {
                    self.note_sink_error(error);
                    SensorSnapshotLoggerError::Path(error)
                })?;
            self.header_emitted = true;
        }

        let snapshot = self.snapshot(timestamp);
        let line = self.format_snapshot_line(&snapshot)?;
        engine
            .append_line(self.path.as_str(), line.as_str())
            .map_err(|error| {
                self.note_sink_error(error);
                SensorSnapshotLoggerError::Path(error)
            })?;
        self.mark_snapshot_emitted();
        Ok(())
    }

    fn update_sink_state_from_queue_error(&mut self, error: TryEnqueueLogError) {
        match error {
            TryEnqueueLogError::Build(build_error) => self.note_sink_error(build_error),
            TryEnqueueLogError::ChannelFull => self.note_sink_backpressured(),
        }
    }

    fn slot_mut(&mut self, sensor: LoggedSensor) -> &mut dyn SensorSlotControl {
        match sensor {
            LoggedSensor::Imu => &mut self.imu,
            LoggedSensor::AuxImu => &mut self.aux_imu,
            LoggedSensor::Barometer => &mut self.barometer,
            LoggedSensor::PressureTransducer => &mut self.pressure_transducer,
            LoggedSensor::Magnetometer => &mut self.magnetometer,
            LoggedSensor::Gps => &mut self.gps,
        }
    }
}

trait SensorSlotControl {
    fn set_faulted(&mut self, faulted: bool);
}

impl<T> SensorSlotControl for SensorSlot<T> {
    fn set_faulted(&mut self, faulted: bool) {
        self.faulted = faulted;
    }
}

fn drain_subscriber<M, T, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
    subscriber: &mut Subscriber<'_, M, T, DEPTH, SUBS, PUBS>,
    slot: &mut SensorSlot<T>,
) where
    M: RawMutex,
    T: Copy,
{
    while let Some(message) = subscriber.try_next_message() {
        match message {
            WaitResult::Lagged(skipped) => {
                slot.lagged_since_emit = slot
                    .lagged_since_emit
                    .saturating_add(skipped.min(u32::MAX as u64) as u32);
            }
            WaitResult::Message(sample) => {
                slot.last_sample = Some(sample);
                slot.fresh_since_emit = true;
                slot.faulted = false;
            }
        }
    }
}

fn build_field<TStamped, TSample>(
    slot: &SensorSlot<TStamped>,
    mark_stale_data: bool,
    unpack: impl Fn(TStamped) -> (MeasurementTimestamp, TSample),
) -> SensorLogField<TSample>
where
    TStamped: Copy,
    TSample: Copy,
{
    let (sample_timestamp, sample) = match slot.last_sample {
        Some(last) => {
            let (timestamp, sample) = unpack(last);
            (Some(timestamp), Some(sample))
        }
        None => (None, None),
    };

    let state = if slot.faulted {
        SensorLogState::Fault
    } else if slot.fresh_since_emit {
        SensorLogState::Fresh
    } else if slot.last_sample.is_some() {
        if mark_stale_data {
            SensorLogState::Stale
        } else {
            SensorLogState::Fresh
        }
    } else {
        SensorLogState::Missing
    };

    SensorLogField {
        state,
        sample_timestamp,
        lagged_samples: slot.lagged_since_emit,
        sample,
    }
}

fn reset_slot_for_next_emit<T>(slot: &mut SensorSlot<T>) {
    slot.fresh_since_emit = false;
    slot.lagged_since_emit = 0;
}

fn write_sensor_preamble<T>(
    line: &mut String<{ crate::interfaces::storage::MAX_LOG_LINE_LEN }>,
    field: &SensorLogField<T>,
    flags: &SensorSnapshotLogFlags,
) -> Result<(), SensorSnapshotLoggerError> {
    if flags.include_sensor_state {
        write!(line, ",{}", field.state.as_str()).map_err(|_| SensorSnapshotLoggerError::Format)?;
    }

    if flags.include_sample_timestamps {
        write!(line, ",").map_err(|_| SensorSnapshotLoggerError::Format)?;
        if let Some(timestamp) = field.sample_timestamp {
            write!(line, "{}", timestamp.as_micros())
                .map_err(|_| SensorSnapshotLoggerError::Format)?;
        }
    }

    if flags.include_lag_counters {
        write!(line, ",{}", field.lagged_samples).map_err(|_| SensorSnapshotLoggerError::Format)?;
    }

    Ok(())
}

fn write_sensor_header(
    line: &mut String<{ crate::interfaces::storage::MAX_LOG_LINE_LEN }>,
    prefix: &str,
    flags: &SensorSnapshotLogFlags,
) -> Result<(), SensorSnapshotLoggerError> {
    if flags.include_sensor_state {
        write!(line, ",{}_state", prefix).map_err(|_| SensorSnapshotLoggerError::Format)?;
    }
    if flags.include_sample_timestamps {
        write!(line, ",{}_sample_us", prefix).map_err(|_| SensorSnapshotLoggerError::Format)?;
    }
    if flags.include_lag_counters {
        write!(line, ",{}_lagged", prefix).map_err(|_| SensorSnapshotLoggerError::Format)?;
    }
    Ok(())
}

fn write_imu_field(
    line: &mut String<{ crate::interfaces::storage::MAX_LOG_LINE_LEN }>,
    field: &SensorLogField<ImuSample>,
    flags: &SensorSnapshotLogFlags,
) -> Result<(), SensorSnapshotLoggerError> {
    write_sensor_preamble(line, field, flags)?;
    match field.sample {
        Some(sample) => write!(
            line,
            ",{},{},{},{},{},{}",
            sample.accel_mps2[0],
            sample.accel_mps2[1],
            sample.accel_mps2[2],
            sample.gyro_rad_s[0],
            sample.gyro_rad_s[1],
            sample.gyro_rad_s[2]
        )
        .map_err(|_| SensorSnapshotLoggerError::Format),
        None => write!(line, ",,,,,,").map_err(|_| SensorSnapshotLoggerError::Format),
    }
}

fn write_barometer_field(
    line: &mut String<{ crate::interfaces::storage::MAX_LOG_LINE_LEN }>,
    field: &SensorLogField<BarometerSample>,
    flags: &SensorSnapshotLogFlags,
) -> Result<(), SensorSnapshotLoggerError> {
    write_sensor_preamble(line, field, flags)?;
    match field.sample {
        Some(sample) => write!(line, ",{},{}", sample.pressure_pa, sample.temperature_c)
            .map_err(|_| SensorSnapshotLoggerError::Format),
        None => write!(line, ",,").map_err(|_| SensorSnapshotLoggerError::Format),
    }
}

fn write_pressure_transducer_field(
    line: &mut String<{ crate::interfaces::storage::MAX_LOG_LINE_LEN }>,
    field: &SensorLogField<PressureTransducerSample>,
    flags: &SensorSnapshotLogFlags,
) -> Result<(), SensorSnapshotLoggerError> {
    write_sensor_preamble(line, field, flags)?;
    match field.sample {
        Some(sample) => write!(line, ",{},{}", sample.pressure_psi, sample.pressure_kpa())
            .map_err(|_| SensorSnapshotLoggerError::Format),
        None => write!(line, ",,").map_err(|_| SensorSnapshotLoggerError::Format),
    }
}

fn write_magnetometer_field(
    line: &mut String<{ crate::interfaces::storage::MAX_LOG_LINE_LEN }>,
    field: &SensorLogField<MagnetometerSample>,
    flags: &SensorSnapshotLogFlags,
) -> Result<(), SensorSnapshotLoggerError> {
    write_sensor_preamble(line, field, flags)?;
    match field.sample {
        Some(sample) => write!(
            line,
            ",{},{},{}",
            sample.field_ut[0], sample.field_ut[1], sample.field_ut[2]
        )
        .map_err(|_| SensorSnapshotLoggerError::Format),
        None => write!(line, ",,,").map_err(|_| SensorSnapshotLoggerError::Format),
    }
}

fn write_gps_field(
    line: &mut String<{ crate::interfaces::storage::MAX_LOG_LINE_LEN }>,
    field: &SensorLogField<GpsFixSample>,
    flags: &SensorSnapshotLogFlags,
) -> Result<(), SensorSnapshotLoggerError> {
    write_sensor_preamble(line, field, flags)?;
    match field.sample {
        Some(sample) => write!(
            line,
            ",{},{},{},{},{},{},{}",
            sample.lat_deg,
            sample.lon_deg,
            sample.alt_m,
            sample.vel_ned_mps[0],
            sample.vel_ned_mps[1],
            sample.vel_ned_mps[2],
            sample.sats
        )
        .map_err(|_| SensorSnapshotLoggerError::Format),
        None => write!(line, ",,,,,,,").map_err(|_| SensorSnapshotLoggerError::Format),
    }
}

#[cfg(test)]
mod tests {
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embassy_sync::pubsub::PubSubChannel;

    use super::{
        SensorSnapshotLogFlags, SensorSnapshotLogger, SensorSnapshotLoggerConfig,
        SensorSnapshotSensorConfig,
    };
    use crate::interfaces::storage::LogError;
    use crate::messages::logging::{LogSinkState, LoggedSensor, SensorLogState};
    use crate::messages::sensor::{
        BarometerSample, BarometerSampleStamped, GpsFixSample, GpsFixSampleStamped, ImuSample,
        ImuSampleStamped, PressureTransducerSample, PressureTransducerSampleStamped,
    };
    use crate::services::logging::LogChannel;
    use crate::utilities::time::MeasurementTimestamp;

    #[test]
    fn marks_stale_fields_when_sensor_did_not_update_for_this_emit() {
        let imu_channel = PubSubChannel::<NoopRawMutex, ImuSampleStamped, 4, 1, 1>::new();
        let gps_channel = PubSubChannel::<NoopRawMutex, GpsFixSampleStamped, 4, 1, 1>::new();
        let mut imu_subscriber = imu_channel.subscriber().unwrap();
        let mut gps_subscriber = gps_channel.subscriber().unwrap();
        let mut logger = SensorSnapshotLogger::new(
            "flight.csv",
            SensorSnapshotLoggerConfig {
                period_ms: 10,
                emit_header: false,
                ..Default::default()
            },
        )
        .unwrap();

        imu_channel
            .immediate_publisher()
            .publish_immediate(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(10_000),
                ImuSample {
                    accel_mps2: [1.0, 2.0, 3.0],
                    gyro_rad_s: [4.0, 5.0, 6.0],
                },
            ));
        gps_channel
            .immediate_publisher()
            .publish_immediate(GpsFixSampleStamped {
                timestamp: MeasurementTimestamp::from_micros(10_000),
                sample: GpsFixSample {
                    lat_deg: 30.0,
                    lon_deg: -97.0,
                    alt_m: 100.0,
                    vel_ned_mps: [1.0, 2.0, 3.0],
                    sats: 10,
                },
            });

        logger.drain_imu(&mut imu_subscriber);
        logger.drain_gps(&mut gps_subscriber);
        let first = logger.snapshot(MeasurementTimestamp::from_micros(10_000));
        assert_eq!(first.imu.state, SensorLogState::Fresh);
        assert_eq!(first.gps.state, SensorLogState::Fresh);
        logger.mark_snapshot_emitted();

        imu_channel
            .immediate_publisher()
            .publish_immediate(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(20_000),
                ImuSample {
                    accel_mps2: [7.0, 8.0, 9.0],
                    gyro_rad_s: [10.0, 11.0, 12.0],
                },
            ));

        logger.drain_imu(&mut imu_subscriber);
        logger.drain_gps(&mut gps_subscriber);
        let second = logger.snapshot(MeasurementTimestamp::from_micros(20_000));
        assert_eq!(second.imu.state, SensorLogState::Fresh);
        assert_eq!(second.gps.state, SensorLogState::Stale);
        assert_eq!(second.gps.sample.unwrap().sats, 10);
    }

    #[test]
    fn sensor_fault_clears_after_new_sample_arrives() {
        let imu_channel = PubSubChannel::<NoopRawMutex, ImuSampleStamped, 4, 1, 1>::new();
        let mut imu_subscriber = imu_channel.subscriber().unwrap();
        let mut logger = SensorSnapshotLogger::new("flight.csv", Default::default()).unwrap();

        logger.note_sensor_fault(LoggedSensor::Imu);
        let faulted = logger.snapshot(MeasurementTimestamp::from_micros(1));
        assert_eq!(faulted.imu.state, SensorLogState::Fault);

        imu_channel
            .immediate_publisher()
            .publish_immediate(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(2),
                ImuSample {
                    accel_mps2: [1.0, 0.0, 0.0],
                    gyro_rad_s: [0.0, 1.0, 0.0],
                },
            ));
        logger.drain_imu(&mut imu_subscriber);
        let recovered = logger.snapshot(MeasurementTimestamp::from_micros(2));
        assert_eq!(recovered.imu.state, SensorLogState::Fresh);
    }

    #[test]
    fn queue_backpressure_and_sink_recovery_update_sink_state() {
        let log_channel = LogChannel::<NoopRawMutex, 1>::new();
        let mut logger = SensorSnapshotLogger::new(
            "flight.csv",
            SensorSnapshotLoggerConfig {
                period_ms: 10,
                emit_header: false,
                ..Default::default()
            },
        )
        .unwrap();

        logger
            .emit_snapshot(&log_channel, MeasurementTimestamp::from_micros(1))
            .unwrap();

        let error = logger
            .emit_snapshot(&log_channel, MeasurementTimestamp::from_micros(2))
            .unwrap_err();
        assert!(matches!(
            error,
            super::SensorSnapshotLoggerError::Queue(
                crate::services::logging::TryEnqueueLogError::ChannelFull
            )
        ));
        assert_eq!(logger.sink_state(), LogSinkState::Backpressured);

        let receiver = log_channel.receiver();
        let _ = receiver.try_receive().unwrap();

        logger.note_sink_recovered();
        assert_eq!(logger.sink_state(), LogSinkState::Healthy);
    }

    #[test]
    fn sink_errors_map_to_degraded_sink_state() {
        let mut logger = SensorSnapshotLogger::new("flight.csv", Default::default()).unwrap();
        logger.note_sink_error(LogError::Filesystem);
        assert_eq!(logger.sink_state(), LogSinkState::FilesystemError);
    }

    #[test]
    fn formats_csv_snapshot_line_with_expected_markers() {
        let mut logger = SensorSnapshotLogger::new(
            "flight.csv",
            SensorSnapshotLoggerConfig {
                period_ms: 10,
                emit_header: false,
                ..Default::default()
            },
        )
        .unwrap();
        logger.barometer.last_sample = Some(BarometerSampleStamped {
            timestamp: MeasurementTimestamp::from_micros(99),
            sample: BarometerSample {
                pressure_pa: 101_325.0,
                temperature_c: 22.5,
            },
        });
        logger.barometer.fresh_since_emit = true;
        let snapshot = logger.snapshot(MeasurementTimestamp::from_micros(100));
        let line = logger.format_snapshot_line(&snapshot).unwrap();

        assert!(line.contains("healthy"));
        assert!(line.contains("fresh"));
        assert!(line.contains("101325"));
    }

    #[test]
    fn config_can_disable_metadata_columns() {
        let logger = SensorSnapshotLogger::new(
            "flight.csv",
            SensorSnapshotLoggerConfig {
                emit_header: false,
                flags: SensorSnapshotLogFlags {
                    include_sink_state: false,
                    include_sensor_state: false,
                    include_sample_timestamps: false,
                    include_lag_counters: false,
                    mark_stale_data: true,
                },
                ..Default::default()
            },
        )
        .unwrap();

        let header = logger.format_header_line().unwrap();
        assert!(!header.contains("sink_state"));
        assert!(!header.contains("imu_state"));
        assert!(!header.contains("aux_imu_state"));
        assert!(header.contains("imu_ax_mps2"));
    }

    #[test]
    fn config_can_disable_sensor_sections() {
        let logger = SensorSnapshotLogger::new(
            "flight.csv",
            SensorSnapshotLoggerConfig {
                emit_header: false,
                sensors: SensorSnapshotSensorConfig {
                    imu: true,
                    aux_imu: false,
                    barometer: false,
                    pressure_transducer: false,
                    magnetometer: false,
                    gps: false,
                },
                ..Default::default()
            },
        )
        .unwrap();

        let header = logger.format_header_line().unwrap();
        assert!(header.contains("imu_ax_mps2"));
        assert!(!header.contains("baro_pressure_pa"));
        assert!(!header.contains("pressure_transducer_psi"));
        assert!(!header.contains("gps_lat_deg"));
    }

    #[test]
    fn pressure_transducer_section_formats_expected_values() {
        let pressure_channel =
            PubSubChannel::<NoopRawMutex, PressureTransducerSampleStamped, 4, 1, 1>::new();
        let mut pressure_subscriber = pressure_channel.subscriber().unwrap();
        let mut logger = SensorSnapshotLogger::new(
            "flight.csv",
            SensorSnapshotLoggerConfig {
                emit_header: true,
                sensors: SensorSnapshotSensorConfig {
                    imu: false,
                    aux_imu: false,
                    barometer: false,
                    pressure_transducer: true,
                    magnetometer: false,
                    gps: false,
                },
                ..Default::default()
            },
        )
        .unwrap();

        pressure_channel
            .immediate_publisher()
            .publish_immediate(PressureTransducerSampleStamped {
                timestamp: MeasurementTimestamp::from_micros(42),
                sample: PressureTransducerSample { pressure_psi: 73.5 },
            });
        logger.drain_pressure_transducer(&mut pressure_subscriber);

        let header = logger.format_header_line().unwrap();
        let line = logger
            .format_snapshot_line(&logger.snapshot(MeasurementTimestamp::from_micros(100)))
            .unwrap();

        assert!(header.contains("pressure_transducer_psi"));
        assert!(header.contains("pressure_transducer_kpa"));
        assert!(line.contains(",73.5,"));
    }

    #[test]
    fn stale_marking_can_be_disabled() {
        let imu_channel = PubSubChannel::<NoopRawMutex, ImuSampleStamped, 4, 1, 1>::new();
        let mut imu_subscriber = imu_channel.subscriber().unwrap();
        let mut logger = SensorSnapshotLogger::new(
            "flight.csv",
            SensorSnapshotLoggerConfig {
                flags: SensorSnapshotLogFlags {
                    mark_stale_data: false,
                    ..Default::default()
                },
                ..Default::default()
            },
        )
        .unwrap();

        imu_channel
            .immediate_publisher()
            .publish_immediate(ImuSampleStamped::new(
                MeasurementTimestamp::from_micros(10_000),
                ImuSample {
                    accel_mps2: [1.0, 2.0, 3.0],
                    gyro_rad_s: [4.0, 5.0, 6.0],
                },
            ));
        logger.drain_imu(&mut imu_subscriber);
        logger.mark_snapshot_emitted();

        let snapshot = logger.snapshot(MeasurementTimestamp::from_micros(20_000));
        assert_eq!(snapshot.imu.state, SensorLogState::Fresh);
    }
}
