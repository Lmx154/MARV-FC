//! Summary health message for startup and supervisory reporting.

use super::{ResetReason, WatchdogStatus};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HealthReport {
    pub reset_reason: ResetReason,
    pub watchdog_status: WatchdogStatus,
    pub degraded: bool,
}
