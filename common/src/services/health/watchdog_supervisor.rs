//! Tier-2 watchdog supervision primitives.

use crate::messages::fault::WatchdogStatus;
use crate::services::health::liveness::LivenessUpdate;

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum FeedDecision {
    FeedAllowed,
    DegradedButFeedable,
    DoNotFeed,
}

impl FeedDecision {
    pub const fn watchdog_status(self) -> WatchdogStatus {
        match self {
            Self::FeedAllowed => WatchdogStatus::FeedAllowed,
            Self::DegradedButFeedable => WatchdogStatus::DegradedButFeedable,
            Self::DoNotFeed => WatchdogStatus::DoNotFeed,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WatchdogContract {
    required_mask: u32,
    degrade_mask: u32,
}

impl WatchdogContract {
    pub const fn new(required_mask: u32, degrade_mask: u32) -> Self {
        Self {
            required_mask,
            degrade_mask,
        }
    }

    pub const fn required_mask(self) -> u32 {
        self.required_mask
    }

    pub const fn degrade_mask(self) -> u32 {
        self.degrade_mask
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WatchdogSource {
    pub mask: u32,
    pub max_age_ms: u32,
}

impl WatchdogSource {
    pub const fn new(mask: u32, max_age_ms: u32) -> Self {
        Self { mask, max_age_ms }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WatchdogEvaluation {
    pub decision: FeedDecision,
    pub stale_required_mask: u32,
    pub stale_degrade_mask: u32,
}

impl WatchdogEvaluation {
    pub const fn watchdog_status(self) -> WatchdogStatus {
        self.decision.watchdog_status()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WatchdogSupervisor<const N: usize> {
    sources: [WatchdogSource; N],
    last_progress_ms: [u64; N],
    contract: WatchdogContract,
}

impl<const N: usize> WatchdogSupervisor<N> {
    pub const fn new(
        sources: [WatchdogSource; N],
        contract: WatchdogContract,
        now_ms: u64,
    ) -> Self {
        Self {
            sources,
            last_progress_ms: [now_ms; N],
            contract,
        }
    }

    pub const fn required_mask(self) -> u32 {
        self.contract.required_mask()
    }

    pub const fn contract(self) -> WatchdogContract {
        self.contract
    }

    pub fn set_contract(&mut self, contract: WatchdogContract, now_ms: u64) {
        self.contract = contract;
        self.last_progress_ms.fill(now_ms);
    }

    pub fn record(&mut self, update: LivenessUpdate) {
        if update.mask == 0 {
            return;
        }

        for (index, source) in self.sources.iter().enumerate() {
            if update.mask & source.mask != 0 {
                self.last_progress_ms[index] = update.now_ms;
            }
        }
    }

    pub fn evaluate(&self, now_ms: u64) -> WatchdogEvaluation {
        let mut stale_required_mask = 0u32;
        let mut stale_degrade_mask = 0u32;

        for (index, source) in self.sources.iter().enumerate() {
            let stale =
                now_ms.saturating_sub(self.last_progress_ms[index]) > u64::from(source.max_age_ms);

            if !stale {
                continue;
            }

            if self.contract.required_mask() & source.mask != 0 {
                stale_required_mask |= source.mask;
            }
            if self.contract.degrade_mask() & source.mask != 0 {
                stale_degrade_mask |= source.mask;
            }
        }

        let decision = if stale_required_mask != 0 {
            FeedDecision::DoNotFeed
        } else if stale_degrade_mask != 0 {
            FeedDecision::DegradedButFeedable
        } else {
            FeedDecision::FeedAllowed
        };

        WatchdogEvaluation {
            decision,
            stale_required_mask,
            stale_degrade_mask,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        FeedDecision, WatchdogContract, WatchdogSource, WatchdogSupervisor,
    };
    use crate::services::health::LivenessUpdate;

    #[test]
    fn required_source_staleness_blocks_feed() {
        let mut supervisor = WatchdogSupervisor::new(
            [
                WatchdogSource::new(1 << 0, 50),
                WatchdogSource::new(1 << 1, 100),
            ],
            WatchdogContract::new(1 << 0, 1 << 1),
            0,
        );
        supervisor.record(LivenessUpdate::new((1 << 0) | (1 << 1), 10));

        let evaluation = supervisor.evaluate(70);
        assert_eq!(evaluation.decision, FeedDecision::DoNotFeed);
        assert_eq!(evaluation.stale_required_mask, 1 << 0);
    }

    #[test]
    fn degrade_source_staleness_keeps_feed_allowed_but_degraded() {
        let mut supervisor = WatchdogSupervisor::new(
            [
                WatchdogSource::new(1 << 0, 100),
                WatchdogSource::new(1 << 1, 50),
            ],
            WatchdogContract::new(1 << 0, 1 << 1),
            0,
        );
        supervisor.record(LivenessUpdate::new(1 << 0, 10));
        supervisor.record(LivenessUpdate::new(1 << 1, 10));

        let evaluation = supervisor.evaluate(70);
        assert_eq!(evaluation.decision, FeedDecision::DegradedButFeedable);
        assert_eq!(evaluation.stale_required_mask, 0);
        assert_eq!(evaluation.stale_degrade_mask, 1 << 1);
    }

    #[test]
    fn contract_switch_reseeds_deadlines_for_new_phase() {
        let mut supervisor = WatchdogSupervisor::new(
            [WatchdogSource::new(1 << 0, 50), WatchdogSource::new(1 << 1, 50)],
            WatchdogContract::new(1 << 0, 0),
            0,
        );
        supervisor.record(LivenessUpdate::new(1 << 0, 10));

        supervisor.set_contract(WatchdogContract::new(1 << 1, 0), 100);

        let evaluation = supervisor.evaluate(120);
        assert_eq!(evaluation.decision, FeedDecision::FeedAllowed);
    }
}
