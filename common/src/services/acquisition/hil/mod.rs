//! Compatibility shim while HIL migrates from acquisition into the shared HIL framework.

use embassy_sync::blocking_mutex::raw::RawMutex;
use heapless::Vec;

use crate::protocol::mavlink::{MavlinkFrame, frame_to_hil_messages};
use crate::services::acquisition::{
    BarometerSampleChannel, GpsFixSampleChannel, ImuSampleChannel, TimeSampleChannel,
};
use crate::services::hil::{HilIngressRoutes, HilRuntime};

pub type HilMavlinkDispatch = crate::services::hil::HilDispatch;

#[derive(Clone, Copy, Debug, Default)]
pub struct MavlinkHilSensorBridge {
    runtime: HilRuntime,
}

impl MavlinkHilSensorBridge {
    pub fn handle_frame<
        M,
        const TIME_DEPTH: usize,
        const TIME_SUBS: usize,
        const TIME_PUBS: usize,
        const IMU_DEPTH: usize,
        const IMU_SUBS: usize,
        const IMU_PUBS: usize,
        const BARO_DEPTH: usize,
        const BARO_SUBS: usize,
        const BARO_PUBS: usize,
        const GPS_DEPTH: usize,
        const GPS_SUBS: usize,
        const GPS_PUBS: usize,
    >(
        &mut self,
        frame: MavlinkFrame,
        time_channel: &TimeSampleChannel<M, TIME_DEPTH, TIME_SUBS, TIME_PUBS>,
        imu_channel: &ImuSampleChannel<M, IMU_DEPTH, IMU_SUBS, IMU_PUBS>,
        barometer_channel: &BarometerSampleChannel<M, BARO_DEPTH, BARO_SUBS, BARO_PUBS>,
        gps_channel: &GpsFixSampleChannel<M, GPS_DEPTH, GPS_SUBS, GPS_PUBS>,
    ) -> HilMavlinkDispatch
    where
        M: RawMutex,
    {
        let routes = HilIngressRoutes::new(
            time_channel,
            imu_channel,
            barometer_channel,
            gps_channel,
            &(),
            &(),
        );
        let mut messages = Vec::<_, 4>::new();
        frame_to_hil_messages(frame, &mut messages);

        let mut dispatch = HilMavlinkDispatch::default();
        for message in messages {
            dispatch.merge(self.runtime.accept(message, &routes));
        }
        dispatch
    }
}

#[cfg(test)]
mod tests {
    use embassy_sync::blocking_mutex::raw::NoopRawMutex;

    use super::MavlinkHilSensorBridge;
    use crate::protocol::mavlink::{
        DecodedMessage, HilGpsMessage, HilSensorMessage, MavlinkFrame, SystemTimeMessage,
    };
    use crate::services::acquisition::{
        BarometerSampleChannel, GpsFixSampleChannel, ImuSampleChannel, TimeSampleChannel,
    };

    #[test]
    fn bridge_publishes_samples_and_deduplicates_ticks() {
        let time_channel = TimeSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let imu_channel = ImuSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let baro_channel = BarometerSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let gps_channel = GpsFixSampleChannel::<NoopRawMutex, 4, 1, 1>::new();
        let mut time_subscriber = time_channel.subscriber().unwrap();
        let mut imu_subscriber = imu_channel.subscriber().unwrap();
        let mut baro_subscriber = baro_channel.subscriber().unwrap();
        let mut gps_subscriber = gps_channel.subscriber().unwrap();
        let mut bridge = MavlinkHilSensorBridge::default();

        let sensor_dispatch = bridge.handle_frame(
            MavlinkFrame {
                source_system: 0,
                source_component: 0,
                message: DecodedMessage::HilSensor(HilSensorMessage {
                    time_usec: 10_000,
                    accel_mps2: [1.0, 2.0, 3.0],
                    gyro_rad_s: [0.1, 0.2, 0.3],
                    abs_pressure_hpa: 1013.25,
                    pressure_alt_m: 123.0,
                    temperature_c: 20.0,
                    fields_updated: 0x003F | 0x1A00,
                }),
            },
            &time_channel,
            &imu_channel,
            &baro_channel,
            &gps_channel,
        );
        assert!(sensor_dispatch.imu_published);
        assert!(sensor_dispatch.barometer_published);
        assert!(imu_subscriber.try_next_message().is_some());
        assert!(baro_subscriber.try_next_message().is_some());

        let gps_dispatch = bridge.handle_frame(
            MavlinkFrame {
                source_system: 0,
                source_component: 0,
                message: DecodedMessage::HilGps(HilGpsMessage {
                    time_usec: 10_000,
                    fix_type: 3,
                    lat_deg_e7: 321000000,
                    lon_deg_e7: -972000000,
                    alt_mm: 120_000,
                    eph: 0,
                    epv: 0,
                    vel_cm_s: 120,
                    vn_cm_s: 100,
                    ve_cm_s: 0,
                    vd_cm_s: -50,
                    cog_cdeg: 0,
                    satellites_visible: 9,
                }),
            },
            &time_channel,
            &imu_channel,
            &baro_channel,
            &gps_channel,
        );
        assert!(gps_dispatch.gps_published);
        assert!(gps_subscriber.try_next_message().is_some());

        let first_tick = bridge.handle_frame(
            MavlinkFrame {
                source_system: 0,
                source_component: 0,
                message: DecodedMessage::SystemTime(SystemTimeMessage { time_boot_ms: 123 }),
            },
            &time_channel,
            &imu_channel,
            &baro_channel,
            &gps_channel,
        );
        assert_eq!(first_tick.tick.unwrap().time_boot_ms, 123);
        match time_subscriber.try_next_message().unwrap() {
            embassy_sync::pubsub::WaitResult::Message(sample) => {
                assert_eq!(sample.time_boot_ms, 123);
            }
            embassy_sync::pubsub::WaitResult::Lagged(skipped) => {
                panic!("unexpected lagged tick sample: {skipped}");
            }
        }

        let duplicate_tick = bridge.handle_frame(
            MavlinkFrame {
                source_system: 0,
                source_component: 0,
                message: DecodedMessage::SystemTime(SystemTimeMessage { time_boot_ms: 123 }),
            },
            &time_channel,
            &imu_channel,
            &baro_channel,
            &gps_channel,
        );
        assert!(duplicate_tick.tick.is_none());
        assert!(time_subscriber.try_next_message().is_none());
    }
}
