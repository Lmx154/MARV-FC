//! Routing boundary between HIL semantics and canonical portable message contracts.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::Channel;

use crate::messages::sensor::{
    BarometerSampleStamped, GpsFixSampleStamped, ImuSampleStamped, MagnetometerSampleStamped,
    TimeSample,
};
use crate::services::acquisition::{
    BarometerSampleChannel, GpsFixSampleChannel, ImuSampleChannel, MagnetometerSampleChannel,
    TimeSampleChannel,
};
use crate::services::hil::model::HilControlCommand;

pub trait HilTimeRoute {
    fn publish_time(&self, sample: TimeSample);
}

pub trait HilImuRoute {
    fn publish_imu(&self, sample: ImuSampleStamped);
}

pub trait HilBarometerRoute {
    fn publish_barometer(&self, sample: BarometerSampleStamped);
}

pub trait HilGpsRoute {
    fn publish_gps(&self, sample: GpsFixSampleStamped);
}

pub trait HilMagnetometerRoute {
    fn publish_magnetometer(&self, sample: MagnetometerSampleStamped);
}

pub trait HilControlCommandRoute {
    fn publish_control_command(&self, command: HilControlCommand);
}

impl HilTimeRoute for () {
    fn publish_time(&self, _sample: TimeSample) {}
}

impl HilImuRoute for () {
    fn publish_imu(&self, _sample: ImuSampleStamped) {}
}

impl HilBarometerRoute for () {
    fn publish_barometer(&self, _sample: BarometerSampleStamped) {}
}

impl HilGpsRoute for () {
    fn publish_gps(&self, _sample: GpsFixSampleStamped) {}
}

impl HilControlCommandRoute for () {
    fn publish_control_command(&self, _command: HilControlCommand) {}
}

impl<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> HilTimeRoute
    for TimeSampleChannel<M, DEPTH, SUBS, PUBS>
where
    M: RawMutex,
{
    fn publish_time(&self, sample: TimeSample) {
        self.immediate_publisher().publish_immediate(sample);
    }
}

impl<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> HilImuRoute
    for ImuSampleChannel<M, DEPTH, SUBS, PUBS>
where
    M: RawMutex,
{
    fn publish_imu(&self, sample: ImuSampleStamped) {
        self.immediate_publisher().publish_immediate(sample);
    }
}

impl<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> HilBarometerRoute
    for BarometerSampleChannel<M, DEPTH, SUBS, PUBS>
where
    M: RawMutex,
{
    fn publish_barometer(&self, sample: BarometerSampleStamped) {
        self.immediate_publisher().publish_immediate(sample);
    }
}

impl<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> HilGpsRoute
    for GpsFixSampleChannel<M, DEPTH, SUBS, PUBS>
where
    M: RawMutex,
{
    fn publish_gps(&self, sample: GpsFixSampleStamped) {
        self.immediate_publisher().publish_immediate(sample);
    }
}

impl<M, const DEPTH: usize, const SUBS: usize, const PUBS: usize> HilMagnetometerRoute
    for MagnetometerSampleChannel<M, DEPTH, SUBS, PUBS>
where
    M: RawMutex,
{
    fn publish_magnetometer(&self, sample: MagnetometerSampleStamped) {
        self.immediate_publisher().publish_immediate(sample);
    }
}

impl HilMagnetometerRoute for () {
    fn publish_magnetometer(&self, _sample: MagnetometerSampleStamped) {}
}

impl<M, const DEPTH: usize> HilControlCommandRoute for Channel<M, HilControlCommand, DEPTH>
where
    M: RawMutex,
{
    fn publish_control_command(&self, command: HilControlCommand) {
        let _ = self.try_send(command);
    }
}

pub struct HilIngressRoutes<'a, Time, Imu, Barometer, Gps, Magnetometer = (), Control = ()> {
    pub time: &'a Time,
    pub imu: &'a Imu,
    pub barometer: &'a Barometer,
    pub gps: &'a Gps,
    pub magnetometer: &'a Magnetometer,
    pub control: &'a Control,
}

impl<'a, Time, Imu, Barometer, Gps, Magnetometer, Control>
    HilIngressRoutes<'a, Time, Imu, Barometer, Gps, Magnetometer, Control>
{
    pub const fn new(
        time: &'a Time,
        imu: &'a Imu,
        barometer: &'a Barometer,
        gps: &'a Gps,
        magnetometer: &'a Magnetometer,
        control: &'a Control,
    ) -> Self {
        Self {
            time,
            imu,
            barometer,
            gps,
            magnetometer,
            control,
        }
    }
}
