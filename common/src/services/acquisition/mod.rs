//! Acquisition services belong here.

pub mod barometer_service;
pub mod bmi088_source;
pub mod bmp388_source;
pub mod lsm6dsv32x_source;
pub mod mavlink_hil;
pub mod pressure_transducer_service;
pub mod sample_channels;
pub mod spi1_imu_service;

pub use barometer_service::{BarometerServiceConfig, run_barometer_service};
pub use bmi088_source::Bmi088ImuSource;
pub use bmp388_source::Bmp388BarometerSource;
pub use lsm6dsv32x_source::Lsm6dsv32xImuSource;
pub use mavlink_hil::{HilMavlinkDispatch, MavlinkHilSensorBridge};
pub use pressure_transducer_service::{
    PressureTransducerServiceConfig, run_pressure_transducer_service,
};
pub use sample_channels::{
    BarometerSampleChannel, BarometerSamplePublisher, BarometerSampleSubscriber,
    BarometerSampleWaitResult, GpsFixSampleChannel, GpsFixSamplePublisher, GpsFixSampleSubscriber,
    GpsFixSampleWaitResult, ImuSampleChannel, ImuSamplePublisher, ImuSampleSubscriber,
    ImuSampleWaitResult, MagnetometerSampleChannel, MagnetometerSamplePublisher,
    MagnetometerSampleSubscriber, MagnetometerSampleWaitResult, PressureTransducerSampleChannel,
    PressureTransducerSamplePublisher, PressureTransducerSampleSubscriber,
    PressureTransducerSampleWaitResult, SampleChannel, SamplePublisher, SampleSubscriber,
    SampleWaitResult, TimeSampleChannel, TimeSamplePublisher, TimeSampleSubscriber,
    TimeSampleWaitResult,
};
pub use spi1_imu_service::{
    Spi1ImuServiceConfig, Spi1ImuServiceError, Spi1ImuSourceSchedule, run_spi1_imu_service,
};
