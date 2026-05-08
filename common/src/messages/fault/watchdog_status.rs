//! Watchdog supervisor decisions published as typed messages.

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum WatchdogStatus {
    FeedAllowed,
    DegradedButFeedable,
    DoNotFeed,
}
