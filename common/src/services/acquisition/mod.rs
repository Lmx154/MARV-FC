//! Acquisition services belong here.

pub mod barometer_service;
pub mod channels;
pub mod hil;
pub mod magnetometer_service;
pub mod pressure_transducer_service;
pub mod sources;

pub use barometer_service::{BarometerServiceConfig, run_barometer_service};
pub use channels::{
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
pub use hil::{HilMavlinkDispatch, MavlinkHilSensorBridge};
pub use magnetometer_service::{MagnetometerServiceConfig, run_magnetometer_service};
pub use pressure_transducer_service::{
    PressureTransducerServiceConfig, run_pressure_transducer_service,
};
pub use sources::{
    Bmi088ImuSource, Bmm350MagnetometerSource, Bmp388BarometerSource, Bmp390BarometerSource,
    Bmp581BarometerSource, Lsm6dsv32xImuSource, Mpu6050ImuSource,
};
