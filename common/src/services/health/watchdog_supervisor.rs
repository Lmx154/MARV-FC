//! Tier-2 watchdog supervision primitives.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeedDecision {
    FeedAllowed,
    DegradedButFeedable,
    DoNotFeed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WatchdogSupervisor {
    required_mask: u32,
}

impl WatchdogSupervisor {
    pub const fn new(required_mask: u32) -> Self {
        Self { required_mask }
    }

    pub const fn required_mask(self) -> u32 {
        self.required_mask
    }
}
