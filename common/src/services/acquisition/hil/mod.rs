//! Portable bridge from MAVLink HIL messages into shared sample channels.

use embassy_sync::blocking_mutex::raw::RawMutex;

use crate::messages::sensor::{
    BarometerSample, BarometerSampleStamped, GpsFixSample, GpsFixSampleStamped, ImuSample,
    ImuSampleStamped, TimeSample,
};
use crate::protocol::mavlink::{
    DecodedMessage, HilGpsMessage, HilSensorMessage, MavlinkFrame, SystemTimeMessage,
};
use crate::utilities::time::MeasurementTimestamp;

use super::channels::{
    BarometerSampleChannel, GpsFixSampleChannel, ImuSampleChannel, TimeSampleChannel,
};

const IMU_UPDATED_MASK: u32 = 0x003F;
const BARO_UPDATED_MASK: u32 = 0x1A00;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HilMavlinkDispatch {
    pub tick: Option<TimeSample>,
    pub imu_published: bool,
    pub barometer_published: bool,
    pub gps_published: bool,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct MavlinkHilSensorBridge {
    last_time_boot_ms: Option<u32>,
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
        match frame.message {
            DecodedMessage::SystemTime(message) => self.handle_system_time(message, time_channel),
            DecodedMessage::HilSensor(message) => {
                self.handle_hil_sensor(message, imu_channel, barometer_channel)
            }
            DecodedMessage::HilGps(message) => self.handle_hil_gps(message, gps_channel),
        }
    }

    fn handle_system_time<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
        &mut self,
        message: SystemTimeMessage,
        channel: &TimeSampleChannel<M, DEPTH, SUBS, PUBS>,
    ) -> HilMavlinkDispatch
    where
        M: RawMutex,
    {
        if self
            .last_time_boot_ms
            .is_some_and(|previous| message.time_boot_ms <= previous)
        {
            return HilMavlinkDispatch::default();
        }

        self.last_time_boot_ms = Some(message.time_boot_ms);
        let tick = TimeSample {
            timestamp: MeasurementTimestamp::from_micros(u64::from(message.time_boot_ms) * 1_000),
            time_boot_ms: message.time_boot_ms,
        };
        channel.immediate_publisher().publish_immediate(tick);
        HilMavlinkDispatch {
            tick: Some(tick),
            ..Default::default()
        }
    }

    fn handle_hil_sensor<
        M,
        const IMU_DEPTH: usize,
        const IMU_SUBS: usize,
        const IMU_PUBS: usize,
        const BARO_DEPTH: usize,
        const BARO_SUBS: usize,
        const BARO_PUBS: usize,
    >(
        &mut self,
        message: HilSensorMessage,
        imu_channel: &ImuSampleChannel<M, IMU_DEPTH, IMU_SUBS, IMU_PUBS>,
        barometer_channel: &BarometerSampleChannel<M, BARO_DEPTH, BARO_SUBS, BARO_PUBS>,
    ) -> HilMavlinkDispatch
    where
        M: RawMutex,
    {
        let timestamp = MeasurementTimestamp::from_micros(message.time_usec);
        let mut dispatch = HilMavlinkDispatch::default();

        if message.fields_updated & IMU_UPDATED_MASK != 0 {
            imu_channel
                .immediate_publisher()
                .publish_immediate(ImuSampleStamped::new(
                    timestamp,
                    ImuSample {
                        accel_mps2: message.accel_mps2,
                        gyro_rad_s: message.gyro_rad_s,
                    },
                ));
            dispatch.imu_published = true;
        }

        if message.fields_updated & BARO_UPDATED_MASK != 0 {
            barometer_channel
                .immediate_publisher()
                .publish_immediate(BarometerSampleStamped {
                    timestamp,
                    sample: BarometerSample {
                        pressure_pa: message.abs_pressure_hpa * 100.0,
                        temperature_c: message.temperature_c,
                    },
                });
            dispatch.barometer_published = true;
        }

        dispatch
    }

    fn handle_hil_gps<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize>(
        &mut self,
        message: HilGpsMessage,
        channel: &GpsFixSampleChannel<M, DEPTH, SUBS, PUBS>,
    ) -> HilMavlinkDispatch
    where
        M: RawMutex,
    {
        if message.fix_type < 2 {
            return HilMavlinkDispatch::default();
        }

        channel
            .immediate_publisher()
            .publish_immediate(GpsFixSampleStamped {
                timestamp: MeasurementTimestamp::from_micros(message.time_usec),
                sample: GpsFixSample {
                    lat_deg: f64::from(message.lat_deg_e7) / 1.0e7,
                    lon_deg: f64::from(message.lon_deg_e7) / 1.0e7,
                    alt_m: (message.alt_mm as f32) / 1_000.0,
                    vel_ned_mps: [
                        (message.vn_cm_s as f32) / 100.0,
                        (message.ve_cm_s as f32) / 100.0,
                        (message.vd_cm_s as f32) / 100.0,
                    ],
                    sats: message.satellites_visible,
                },
            });

        HilMavlinkDispatch {
            gps_published: true,
            ..Default::default()
        }
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
                    satellites_visible: 12,
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
                message: DecodedMessage::SystemTime(SystemTimeMessage { time_boot_ms: 10 }),
            },
            &time_channel,
            &imu_channel,
            &baro_channel,
            &gps_channel,
        );
        assert!(first_tick.tick.is_some());
        assert!(time_subscriber.try_next_message().is_some());

        let duplicate_tick = bridge.handle_frame(
            MavlinkFrame {
                message: DecodedMessage::SystemTime(SystemTimeMessage { time_boot_ms: 10 }),
            },
            &time_channel,
            &imu_channel,
            &baro_channel,
            &gps_channel,
        );
        assert!(duplicate_tick.tick.is_none());
    }
}
