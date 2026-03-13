//! Small re-export surface for common architectural primitives.

pub use crate::messages::fault::{HealthReport, ResetReason, WatchdogStatus};
pub use crate::messages::logging::LogCommand;
pub use crate::protocol::packet::{Packet, PacketType};
pub use crate::interfaces::storage::{LogError, LogPath, LoggerEngine};
pub use crate::drivers::storage::{MicrosdLogger, MicrosdLoggerConfig};
pub use crate::services::health::{
    DeadlineWindow, FaultAggregation, FeedDecision, LivenessClass, LivenessEvidence,
    WatchdogSupervisor,
};
pub use crate::services::logging::{LogChannel, TryEnqueueLogError, try_enqueue_line};
