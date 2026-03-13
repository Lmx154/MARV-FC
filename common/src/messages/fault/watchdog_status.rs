//! Watchdog supervisor decisions published as typed messages.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WatchdogStatus {
    FeedAllowed,
    DegradedButFeedable,
    DoNotFeed,
}
