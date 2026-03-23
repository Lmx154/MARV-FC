//! Shared liveness evidence types.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LivenessClass {
    FeedCritical,
    DegradeCritical,
    NonCritical,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LivenessEvidence {
    pub class: LivenessClass,
    pub last_progress_ms: u64,
    pub max_age_ms: u32,
}

impl LivenessEvidence {
    pub const fn is_stale(self, now_ms: u64) -> bool {
        now_ms.saturating_sub(self.last_progress_ms) > self.max_age_ms as u64
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LivenessUpdate {
    pub mask: u32,
    pub now_ms: u64,
}

impl LivenessUpdate {
    pub const fn new(mask: u32, now_ms: u64) -> Self {
        Self { mask, now_ms }
    }
}
