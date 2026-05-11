//! HILink frame boundary adapter for canonical sensor publication.

use crate::protocol::hilink::{HilSensorFrame, SimStamp, valid};
use crate::services::hil::model::{
    HilBarometerSample, HilGpsSample, HilImuSample, HilIngressMessage, HilMagSample, HilTick,
};
use crate::services::hil::routing::{
    HilBarometerRoute, HilControlCommandRoute, HilGpsRoute, HilImuRoute, HilIngressRoutes,
    HilMagnetometerRoute, HilTimeRoute,
};
use crate::services::hil::runtime::{HilDispatch, HilRuntime};
use crate::utilities::time::MeasurementTimestamp;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HilValidationCounters {
    pub duplicate_tick: u32,
    pub out_of_order_tick: u32,
    pub response_mismatch: u32,
    pub missed_response: u32,
    pub invalid_non_finite_sample: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HilCorrelationState {
    pub latest_accepted_stamp: Option<SimStamp>,
    pub last_processed_tick: Option<u64>,
    pub current_in_flight_tick: Option<u64>,
    pub counters: HilValidationCounters,
    pub input_active: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum HilSensorFrameRejection {
    #[default]
    Inactive,
    DuplicateTick,
    OutOfOrderTick,
    InvalidSample,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilSensorFrameDispatch {
    pub stamp: SimStamp,
    pub accepted: bool,
    pub rejection: Option<HilSensorFrameRejection>,
    pub dispatch: HilDispatch,
    pub counters: HilValidationCounters,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct HilSensorFrameAdapter {
    runtime: HilRuntime,
    correlation: HilCorrelationState,
}

impl HilSensorFrameAdapter {
    pub const fn new() -> Self {
        Self {
            runtime: HilRuntime::new(),
            correlation: HilCorrelationState {
                latest_accepted_stamp: None,
                last_processed_tick: None,
                current_in_flight_tick: None,
                counters: HilValidationCounters {
                    duplicate_tick: 0,
                    out_of_order_tick: 0,
                    response_mismatch: 0,
                    missed_response: 0,
                    invalid_non_finite_sample: 0,
                },
                input_active: false,
            },
        }
    }

    pub fn active() -> Self {
        let mut adapter = Self::new();
        adapter.set_input_active(true);
        adapter
    }

    pub fn set_input_active(&mut self, active: bool) {
        if self.correlation.input_active == active {
            return;
        }

        self.correlation.input_active = active;
        self.correlation.current_in_flight_tick = None;
        if active {
            self.runtime.enable_virtual_sensor_streaming();
        } else {
            self.runtime.disable_virtual_sensor_streaming();
        }
    }

    pub const fn correlation(&self) -> HilCorrelationState {
        self.correlation
    }

    pub fn reset_stream_correlation(&mut self) {
        self.correlation.latest_accepted_stamp = None;
        self.correlation.last_processed_tick = None;
        self.correlation.current_in_flight_tick = None;
    }

    pub fn accept_frame<Time, Imu, Barometer, Gps, Magnetometer, Control>(
        &mut self,
        frame: HilSensorFrame,
        routes: &HilIngressRoutes<'_, Time, Imu, Barometer, Gps, Magnetometer, Control>,
    ) -> HilSensorFrameDispatch
    where
        Time: HilTimeRoute,
        Imu: HilImuRoute,
        Barometer: HilBarometerRoute,
        Gps: HilGpsRoute,
        Magnetometer: HilMagnetometerRoute,
        Control: HilControlCommandRoute,
    {
        if !self.correlation.input_active {
            return self.rejected(frame.stamp, HilSensorFrameRejection::Inactive);
        }

        if let Some(last_tick) = self.correlation.last_processed_tick {
            if frame.stamp.sim_tick == last_tick {
                self.correlation.counters.duplicate_tick =
                    self.correlation.counters.duplicate_tick.saturating_add(1);
                return self.rejected(frame.stamp, HilSensorFrameRejection::DuplicateTick);
            }

            if frame.stamp.sim_tick < last_tick
                || self
                    .correlation
                    .latest_accepted_stamp
                    .is_some_and(|stamp| frame.stamp.sim_time_us <= stamp.sim_time_us)
            {
                self.correlation.counters.out_of_order_tick = self
                    .correlation
                    .counters
                    .out_of_order_tick
                    .saturating_add(1);
                return self.rejected(frame.stamp, HilSensorFrameRejection::OutOfOrderTick);
            }
        }

        if self.has_invalid_asserted_sample(frame) {
            return self.rejected(frame.stamp, HilSensorFrameRejection::InvalidSample);
        }

        let timestamp = MeasurementTimestamp::from_micros(frame.stamp.sim_time_us);
        let time_boot_ms = (frame.stamp.sim_time_us / 1_000).min(u32::MAX as u64) as u32;
        let mut dispatch = self.runtime.accept(
            HilIngressMessage::Tick(HilTick {
                timestamp,
                time_boot_ms,
            }),
            routes,
        );

        if self.publish_imu(frame, timestamp, routes) {
            dispatch.imu_published = true;
        }
        if self.publish_barometer(frame, timestamp, routes) {
            dispatch.barometer_published = true;
        }
        if self.publish_gps(frame, timestamp, routes) {
            dispatch.gps_published = true;
        }
        if self.publish_magnetometer(frame, timestamp, routes) {
            dispatch.magnetometer_published = true;
        }

        self.correlation.latest_accepted_stamp = Some(frame.stamp);
        self.correlation.last_processed_tick = Some(frame.stamp.sim_tick);
        self.correlation.current_in_flight_tick = Some(frame.stamp.sim_tick);

        HilSensorFrameDispatch {
            stamp: frame.stamp,
            accepted: true,
            rejection: None,
            dispatch,
            counters: self.correlation.counters,
        }
    }

    pub fn mark_response_sent(&mut self, stamp: SimStamp) {
        if self.correlation.current_in_flight_tick == Some(stamp.sim_tick) {
            self.correlation.current_in_flight_tick = None;
        } else {
            self.correlation.counters.response_mismatch = self
                .correlation
                .counters
                .response_mismatch
                .saturating_add(1);
        }
    }

    pub fn mark_missed_response(&mut self) {
        if self.correlation.current_in_flight_tick.take().is_some() {
            self.correlation.counters.missed_response =
                self.correlation.counters.missed_response.saturating_add(1);
        }
    }

    fn rejected(
        &self,
        stamp: SimStamp,
        rejection: HilSensorFrameRejection,
    ) -> HilSensorFrameDispatch {
        HilSensorFrameDispatch {
            stamp,
            accepted: false,
            rejection: Some(rejection),
            dispatch: HilDispatch::default(),
            counters: self.correlation.counters,
        }
    }

    fn publish_imu<Time, Imu, Barometer, Gps, Magnetometer, Control>(
        &mut self,
        frame: HilSensorFrame,
        timestamp: MeasurementTimestamp,
        routes: &HilIngressRoutes<'_, Time, Imu, Barometer, Gps, Magnetometer, Control>,
    ) -> bool
    where
        Imu: HilImuRoute,
    {
        if frame.valid_flags & (valid::ACCEL | valid::GYRO) != (valid::ACCEL | valid::GYRO) {
            return false;
        }
        if !finite_f32x3(frame.accel_mps2) || !finite_f32x3(frame.gyro_rps) {
            self.count_invalid_sample();
            return false;
        }

        routes.imu.publish_imu(
            HilImuSample {
                timestamp,
                accel_mps2: frame.accel_mps2,
                gyro_rad_s: frame.gyro_rps,
            }
            .into(),
        );
        true
    }

    fn publish_barometer<Time, Imu, Barometer, Gps, Magnetometer, Control>(
        &mut self,
        frame: HilSensorFrame,
        timestamp: MeasurementTimestamp,
        routes: &HilIngressRoutes<'_, Time, Imu, Barometer, Gps, Magnetometer, Control>,
    ) -> bool
    where
        Barometer: HilBarometerRoute,
    {
        if frame.valid_flags & valid::BARO == 0 {
            return false;
        }
        if !frame.pressure_pa.is_finite()
            || !frame.baro_altitude_m.is_finite()
            || !frame.temperature_c.is_finite()
        {
            self.count_invalid_sample();
            return false;
        }

        routes.barometer.publish_barometer(
            HilBarometerSample {
                timestamp,
                pressure_pa: frame.pressure_pa,
                temperature_c: frame.temperature_c,
            }
            .into(),
        );
        true
    }

    fn publish_gps<Time, Imu, Barometer, Gps, Magnetometer, Control>(
        &mut self,
        frame: HilSensorFrame,
        timestamp: MeasurementTimestamp,
        routes: &HilIngressRoutes<'_, Time, Imu, Barometer, Gps, Magnetometer, Control>,
    ) -> bool
    where
        Gps: HilGpsRoute,
    {
        if frame.valid_flags & valid::GPS == 0 {
            return false;
        }
        if !frame.lat_deg.is_finite()
            || !frame.lon_deg.is_finite()
            || !frame.alt_msl_m.is_finite()
            || !finite_f32x3(frame.vel_ned_mps)
        {
            self.count_invalid_sample();
            return false;
        }

        routes.gps.publish_gps(
            HilGpsSample {
                timestamp,
                lat_deg: frame.lat_deg,
                lon_deg: frame.lon_deg,
                alt_m: frame.alt_msl_m,
                vel_ned_mps: frame.vel_ned_mps,
                sats: frame.sats,
                fix_type: frame.fix_type,
            }
            .into(),
        );
        true
    }

    fn publish_magnetometer<Time, Imu, Barometer, Gps, Magnetometer, Control>(
        &mut self,
        frame: HilSensorFrame,
        timestamp: MeasurementTimestamp,
        routes: &HilIngressRoutes<'_, Time, Imu, Barometer, Gps, Magnetometer, Control>,
    ) -> bool
    where
        Magnetometer: HilMagnetometerRoute,
    {
        if frame.valid_flags & valid::MAG == 0 {
            return false;
        }
        if !finite_f32x3(frame.mag_ut) {
            self.count_invalid_sample();
            return false;
        }

        routes.magnetometer.publish_magnetometer(
            HilMagSample {
                timestamp,
                field_ut: frame.mag_ut,
            }
            .into(),
        );
        true
    }

    fn count_invalid_sample(&mut self) {
        self.correlation.counters.invalid_non_finite_sample = self
            .correlation
            .counters
            .invalid_non_finite_sample
            .saturating_add(1);
    }

    fn has_invalid_asserted_sample(&mut self, frame: HilSensorFrame) -> bool {
        let invalid_imu = (frame.valid_flags & valid::ACCEL != 0
            && !finite_f32x3(frame.accel_mps2))
            || (frame.valid_flags & valid::GYRO != 0 && !finite_f32x3(frame.gyro_rps));
        let invalid_baro = frame.valid_flags & valid::BARO != 0
            && (!frame.pressure_pa.is_finite()
                || !frame.baro_altitude_m.is_finite()
                || !frame.temperature_c.is_finite());
        let invalid_gps = frame.valid_flags & valid::GPS != 0
            && (!frame.lat_deg.is_finite()
                || !frame.lon_deg.is_finite()
                || !frame.alt_msl_m.is_finite()
                || !finite_f32x3(frame.vel_ned_mps));
        let invalid_mag = frame.valid_flags & valid::MAG != 0 && !finite_f32x3(frame.mag_ut);

        if invalid_imu || invalid_baro || invalid_gps || invalid_mag {
            self.count_invalid_sample();
            true
        } else {
            false
        }
    }
}

impl From<HilImuSample> for crate::messages::sensor::ImuSampleStamped {
    fn from(sample: HilImuSample) -> Self {
        Self::new(
            sample.timestamp,
            crate::messages::sensor::ImuSample {
                accel_mps2: sample.accel_mps2,
                gyro_rad_s: sample.gyro_rad_s,
            },
        )
    }
}

impl From<HilBarometerSample> for crate::messages::sensor::BarometerSampleStamped {
    fn from(sample: HilBarometerSample) -> Self {
        Self {
            timestamp: sample.timestamp,
            sample: crate::messages::sensor::BarometerSample {
                pressure_pa: sample.pressure_pa,
                temperature_c: sample.temperature_c,
            },
        }
    }
}

impl From<HilGpsSample> for crate::messages::sensor::GpsFixSampleStamped {
    fn from(sample: HilGpsSample) -> Self {
        Self {
            timestamp: sample.timestamp,
            sample: crate::messages::sensor::GpsFixSample {
                lat_deg: sample.lat_deg,
                lon_deg: sample.lon_deg,
                alt_m: sample.alt_m,
                vel_ned_mps: sample.vel_ned_mps,
                sats: sample.sats,
                fix_type: sample.fix_type,
            },
        }
    }
}

impl From<HilMagSample> for crate::messages::sensor::MagnetometerSampleStamped {
    fn from(sample: HilMagSample) -> Self {
        Self {
            timestamp: sample.timestamp,
            sample: crate::messages::sensor::MagnetometerSample {
                field_ut: sample.field_ut,
            },
        }
    }
}

fn finite_f32x3(values: [f32; 3]) -> bool {
    values[0].is_finite() && values[1].is_finite() && values[2].is_finite()
}

#[cfg(test)]
mod tests {
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;
    use embassy_sync::pubsub::WaitResult;

    use super::{HilSensorFrameAdapter, HilSensorFrameRejection};
    use crate::protocol::hilink::{HilSensorFrame, SimStamp, valid};
    use crate::services::acquisition::{
        BarometerSampleChannel, GpsFixSampleChannel, ImuSampleChannel, MagnetometerSampleChannel,
        TimeSampleChannel,
    };
    use crate::services::hil::routing::HilIngressRoutes;
    use crate::test_helpers::assert_vec3_near;
    use crate::utilities::time::MeasurementTimestamp;

    fn frame(tick: u64, time_us: u64) -> HilSensorFrame {
        HilSensorFrame {
            stamp: SimStamp {
                sim_tick: tick,
                sim_time_us: time_us,
            },
            valid_flags: valid::ACCEL | valid::GYRO | valid::MAG | valid::BARO | valid::GPS,
            accel_mps2: [1.0, 2.0, 3.0],
            gyro_rps: [0.1, 0.2, 0.3],
            mag_ut: [10.0, 11.0, 12.0],
            pressure_pa: 101_325.0,
            baro_altitude_m: 0.0,
            temperature_c: 25.0,
            lat_deg: 26.0,
            lon_deg: -98.0,
            alt_msl_m: 120.0,
            vel_ned_mps: [1.0, 0.0, 0.0],
            sats: 10,
            fix_type: 3,
            ..HilSensorFrame::default()
        }
    }

    #[test]
    fn adapter_publishes_hilink_frame_to_canonical_channels() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let baro = BarometerSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let gps = GpsFixSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let mag = MagnetometerSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &baro, &gps, &mag, &());
        let mut time_sub = time.subscriber().unwrap();
        let mut imu_sub = imu.subscriber().unwrap();
        let mut baro_sub = baro.subscriber().unwrap();
        let mut gps_sub = gps.subscriber().unwrap();
        let mut mag_sub = mag.subscriber().unwrap();
        let mut adapter = HilSensorFrameAdapter::active();

        let dispatch = adapter.accept_frame(frame(7, 42_000), &routes);

        assert!(dispatch.accepted);
        assert!(dispatch.dispatch.tick.is_some());
        assert!(dispatch.dispatch.imu_published);
        assert!(dispatch.dispatch.barometer_published);
        assert!(dispatch.dispatch.gps_published);
        assert!(dispatch.dispatch.magnetometer_published);

        match time_sub.try_next_message().unwrap() {
            WaitResult::Message(sample) => {
                assert_eq!(sample.timestamp, MeasurementTimestamp::from_micros(42_000));
                assert_eq!(sample.time_boot_ms, 42);
            }
            WaitResult::Lagged(skipped) => panic!("unexpected lagged time sample: {skipped}"),
        }
        match imu_sub.try_next_message().unwrap() {
            WaitResult::Message(sample) => {
                assert_eq!(sample.timestamp, MeasurementTimestamp::from_micros(42_000));
            }
            WaitResult::Lagged(skipped) => panic!("unexpected lagged imu sample: {skipped}"),
        }
        assert!(baro_sub.try_next_message().is_some());
        assert!(gps_sub.try_next_message().is_some());
        assert!(mag_sub.try_next_message().is_some());
    }

    #[test]
    fn adapter_rejects_duplicate_and_out_of_order_ticks() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut adapter = HilSensorFrameAdapter::active();

        assert!(adapter.accept_frame(frame(2, 2_000), &routes).accepted);

        let duplicate = adapter.accept_frame(frame(2, 3_000), &routes);
        assert!(!duplicate.accepted);
        assert_eq!(
            duplicate.rejection,
            Some(HilSensorFrameRejection::DuplicateTick)
        );
        assert_eq!(duplicate.counters.duplicate_tick, 1);

        let out_of_order = adapter.accept_frame(frame(1, 1_000), &routes);
        assert!(!out_of_order.accepted);
        assert_eq!(
            out_of_order.rejection,
            Some(HilSensorFrameRejection::OutOfOrderTick)
        );
        assert_eq!(out_of_order.counters.out_of_order_tick, 1);
    }

    #[test]
    fn inactive_adapter_rejects_frame_without_publishing() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut time_sub = time.subscriber().unwrap();
        let mut imu_sub = imu.subscriber().unwrap();
        let mut adapter = HilSensorFrameAdapter::new();

        let dispatch = adapter.accept_frame(frame(1, 1_000), &routes);

        assert!(!dispatch.accepted);
        assert_eq!(dispatch.rejection, Some(HilSensorFrameRejection::Inactive));
        assert!(!dispatch.dispatch.imu_published);
        assert!(time_sub.try_next_message().is_none());
        assert!(imu_sub.try_next_message().is_none());
    }

    #[test]
    fn adapter_publishes_only_asserted_valid_sensor_groups() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let baro = BarometerSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let gps = GpsFixSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let mag = MagnetometerSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &baro, &gps, &mag, &());
        let mut imu_sub = imu.subscriber().unwrap();
        let mut baro_sub = baro.subscriber().unwrap();
        let mut gps_sub = gps.subscriber().unwrap();
        let mut mag_sub = mag.subscriber().unwrap();
        let mut adapter = HilSensorFrameAdapter::active();
        let mut partial = frame(1, 1_000);
        partial.valid_flags = valid::ACCEL | valid::GYRO;

        let dispatch = adapter.accept_frame(partial, &routes);

        assert!(dispatch.accepted);
        assert!(dispatch.dispatch.imu_published);
        assert!(!dispatch.dispatch.barometer_published);
        assert!(!dispatch.dispatch.gps_published);
        assert!(!dispatch.dispatch.magnetometer_published);
        match imu_sub.try_next_message().unwrap() {
            WaitResult::Message(sample) => {
                assert_vec3_near(sample.sample.accel_mps2, [1.0, 2.0, 3.0], 0.000_001);
                assert_vec3_near(sample.sample.gyro_rad_s, [0.1, 0.2, 0.3], 0.000_001);
            }
            WaitResult::Lagged(skipped) => panic!("unexpected lagged imu sample: {skipped}"),
        }
        assert!(baro_sub.try_next_message().is_none());
        assert!(gps_sub.try_next_message().is_none());
        assert!(mag_sub.try_next_message().is_none());
    }

    #[test]
    fn adapter_ignores_non_finite_unasserted_fields() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut adapter = HilSensorFrameAdapter::active();
        let mut partial = frame(1, 1_000);
        partial.valid_flags = valid::ACCEL | valid::GYRO;
        partial.pressure_pa = f32::NAN;
        partial.baro_altitude_m = f32::NAN;
        partial.temperature_c = f32::NAN;
        partial.lat_deg = f64::NAN;
        partial.lon_deg = f64::NAN;
        partial.alt_msl_m = f32::NAN;
        partial.vel_ned_mps = [f32::NAN, 0.0, 0.0];
        partial.mag_ut = [f32::NAN, 0.0, 0.0];

        let dispatch = adapter.accept_frame(partial, &routes);

        assert!(dispatch.accepted);
        assert!(dispatch.dispatch.imu_published);
        assert_eq!(dispatch.counters.invalid_non_finite_sample, 0);
    }

    #[test]
    fn adapter_rejects_non_finite_asserted_samples_without_forking_channels() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut imu_sub = imu.subscriber().unwrap();
        let mut adapter = HilSensorFrameAdapter::active();
        let mut bad = frame(1, 1_000);
        bad.gyro_rps[0] = f32::NAN;

        let dispatch = adapter.accept_frame(bad, &routes);

        assert!(!dispatch.accepted);
        assert_eq!(
            dispatch.rejection,
            Some(HilSensorFrameRejection::InvalidSample)
        );
        assert!(!dispatch.dispatch.imu_published);
        assert_eq!(dispatch.counters.invalid_non_finite_sample, 1);
        assert!(imu_sub.try_next_message().is_none());
    }

    #[test]
    fn adapter_tracks_response_sent_missed_and_mismatch_counters() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut adapter = HilSensorFrameAdapter::active();

        let first = frame(1, 1_000);
        assert!(adapter.accept_frame(first, &routes).accepted);
        adapter.mark_response_sent(first.stamp);
        assert_eq!(adapter.correlation().current_in_flight_tick, None);
        assert_eq!(adapter.correlation().counters.response_mismatch, 0);
        assert_eq!(adapter.correlation().counters.missed_response, 0);

        let second = frame(2, 2_000);
        assert!(adapter.accept_frame(second, &routes).accepted);
        adapter.mark_response_sent(SimStamp {
            sim_tick: 99,
            sim_time_us: 99_000,
        });
        assert_eq!(adapter.correlation().current_in_flight_tick, Some(2));
        assert_eq!(adapter.correlation().counters.response_mismatch, 1);

        adapter.mark_missed_response();
        assert_eq!(adapter.correlation().current_in_flight_tick, None);
        assert_eq!(adapter.correlation().counters.missed_response, 1);

        adapter.mark_missed_response();
        assert_eq!(adapter.correlation().counters.missed_response, 1);
    }

    #[test]
    fn stream_correlation_reset_allows_new_tick_epoch() {
        let time = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let routes = HilIngressRoutes::new(&time, &imu, &(), &(), &(), &());
        let mut adapter = HilSensorFrameAdapter::active();

        assert!(adapter.accept_frame(frame(10, 10_000), &routes).accepted);
        adapter.reset_stream_correlation();

        let restarted = adapter.accept_frame(frame(1, 1_000), &routes);

        assert!(restarted.accepted);
        assert_eq!(restarted.rejection, None);
        assert_eq!(
            adapter.correlation().latest_accepted_stamp,
            Some(SimStamp {
                sim_tick: 1,
                sim_time_us: 1_000,
            })
        );
        assert_eq!(adapter.correlation().last_processed_tick, Some(1));
    }
}
