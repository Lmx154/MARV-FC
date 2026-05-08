//! Small re-export surface for common architectural primitives.

pub use crate::drivers::storage::{MicrosdLogger, MicrosdLoggerConfig};
pub use crate::interfaces::storage::{LogError, LogPath, LoggerEngine};
pub use crate::messages::control::RgbLedCommand;
pub use crate::messages::fault::{HealthReport, ResetReason, WatchdogStatus};
pub use crate::messages::logging::{
    LogCommand, LogSinkState, LoggedSensor, SensorLogField, SensorLogSnapshot, SensorLogState,
};
pub use crate::messages::runtime::FlightPhase;
pub use crate::messages::sensor::{
    BarometerSample, BarometerSampleStamped, GpsFixSample, GpsFixSampleStamped, ImuSample,
    ImuSampleStamped, MagnetometerSample, MagnetometerSampleStamped, PressureTransducerSample,
    PressureTransducerSampleStamped,
};
pub use crate::policies::mission::{
    BarometerRgbLedMission, BarometerRgbLedMissionConfig, Mission, MissionUpdate,
};
pub use crate::protocol::packet::{Packet, PacketType};
pub use crate::services::acquisition::{
    BarometerSampleChannel, BarometerSamplePublisher, BarometerSampleSubscriber,
    BarometerSampleWaitResult, BarometerServiceConfig, Bmi088ImuSource, Bmp388BarometerSource,
    Bmp390BarometerSource, ImuSampleChannel, ImuSamplePublisher, ImuSampleSubscriber,
    ImuSampleWaitResult, Lsm6dsv32xImuSource, Mpu6050ImuSource, PressureTransducerSampleChannel,
    PressureTransducerSamplePublisher, PressureTransducerSampleSubscriber,
    PressureTransducerSampleWaitResult, PressureTransducerServiceConfig,
};
pub use crate::services::health::{
    DeadlineWindow, FaultAggregation, FeedDecision, LivenessClass, LivenessEvidence,
    LivenessUpdate, WatchdogContract, WatchdogEvaluation, WatchdogSource, WatchdogSupervisor,
};
pub use crate::services::logging::{LogChannel, TryEnqueueLogError, try_enqueue_line};
pub use crate::services::logging::{
    SensorSnapshotLogFlags, SensorSnapshotLogger, SensorSnapshotLoggerConfig,
    SensorSnapshotLoggerError, SensorSnapshotSensorConfig,
};
pub use crate::tasks::background::mission::run_barometer_rgb_led_mission_task;
pub use crate::utilities::time::{MeasurementDelta, MeasurementTimestamp};
