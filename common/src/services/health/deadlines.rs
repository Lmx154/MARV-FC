//! Deadline metadata shared by health supervision.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DeadlineWindow {
    pub period_ms: u32,
    pub max_lag_ms: u32,
}
