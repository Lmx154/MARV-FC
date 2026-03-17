//! Acquisition services belong here.

pub mod bmi088_source;
pub mod imu_cluster_service;
pub mod imu_producer;
pub mod lsm6dsv32x_source;
pub mod sample_channels;

pub use bmi088_source::Bmi088ImuSource;
pub use imu_cluster_service::{DualImuServiceError, run_dual_imu_service};
pub use imu_producer::{
    ImuProducerConfig, ImuProducerError, ImuSampleChannel, ImuSamplePublisher, ImuSampleSubscriber,
    ImuSampleWaitResult, produce_one_imu_sample, run_imu_producer_task,
};
pub use lsm6dsv32x_source::Lsm6dsv32xImuSource;
pub use sample_channels::{
    BarometerSampleChannel, BarometerSamplePublisher, BarometerSampleSubscriber,
    BarometerSampleWaitResult, GpsFixSampleChannel, GpsFixSamplePublisher, GpsFixSampleSubscriber,
    GpsFixSampleWaitResult, MagnetometerSampleChannel, MagnetometerSamplePublisher,
    MagnetometerSampleSubscriber, MagnetometerSampleWaitResult, SampleChannel, SamplePublisher,
    SampleSubscriber, SampleWaitResult,
};
