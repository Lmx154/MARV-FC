//! Acquisition services belong here.

pub mod bmi088_source;
pub mod lsm6dsv32x_source;
pub mod mavlink_hil;
pub mod sample_channels;
pub mod spi1_imu_service;

pub use bmi088_source::Bmi088ImuSource;
pub use lsm6dsv32x_source::Lsm6dsv32xImuSource;
pub use mavlink_hil::{HilMavlinkDispatch, MavlinkHilSensorBridge};
pub use sample_channels::{
    BarometerSampleChannel, BarometerSamplePublisher, BarometerSampleSubscriber,
    BarometerSampleWaitResult, GpsFixSampleChannel, GpsFixSamplePublisher, GpsFixSampleSubscriber,
    GpsFixSampleWaitResult, ImuSampleChannel, ImuSamplePublisher, ImuSampleSubscriber,
    ImuSampleWaitResult, MagnetometerSampleChannel, MagnetometerSamplePublisher,
    MagnetometerSampleSubscriber, MagnetometerSampleWaitResult, SampleChannel, SamplePublisher,
    SampleSubscriber, SampleWaitResult, TimeSampleChannel, TimeSamplePublisher,
    TimeSampleSubscriber, TimeSampleWaitResult,
};
pub use spi1_imu_service::{
    Spi1ImuServiceConfig, Spi1ImuServiceError, Spi1ImuSourceSchedule, run_spi1_imu_service,
};
