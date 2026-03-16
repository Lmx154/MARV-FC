//! Acquisition services belong here.

pub mod bmi088_source;
pub mod imu_producer;
pub mod sample_channels;

pub use bmi088_source::Bmi088ImuSource;
pub use imu_producer::{
    ImuProducerConfig, ImuProducerError, ImuSampleChannel, ImuSamplePublisher, ImuSampleSubscriber,
    ImuSampleWaitResult, run_imu_producer_task,
};
pub use sample_channels::{
    BarometerSampleChannel, BarometerSamplePublisher, BarometerSampleSubscriber,
    BarometerSampleWaitResult, GpsFixSampleChannel, GpsFixSamplePublisher, GpsFixSampleSubscriber,
    GpsFixSampleWaitResult, MagnetometerSampleChannel, MagnetometerSamplePublisher,
    MagnetometerSampleSubscriber, MagnetometerSampleWaitResult, SampleChannel, SamplePublisher,
    SampleSubscriber, SampleWaitResult,
};
