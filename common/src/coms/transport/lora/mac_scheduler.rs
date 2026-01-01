pub const DEFAULT_LOCK_TIMEOUT_MS: u64 = 1000;
pub const DEFAULT_TICK_SLOP_TICKS: u16 = 2;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkState {
    Search,
    Locked,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkEvent {
    None,
    LockAcquired,
    LockLost,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickUpdate {
    pub event: LinkEvent,
    pub missed: u16,
    pub seq_anomaly: u16,
}

impl Default for TickUpdate {
    fn default() -> Self {
        Self {
            event: LinkEvent::None,
            missed: 0,
            seq_anomaly: 0,
        }
    }
}

pub struct TickTracker {
    state: LinkState,
    lock_timeout_ms: u64,
    tick_period_ms: u64,
    max_tick_slop: u16,
    last_tick_seq: Option<u16>,
    last_rx_ms: Option<u64>,
    rx_count: u32,
    missed_ticks: u32,
    seq_anomaly_count: u32,
    dup_count: u32,
    lock_count: u32,
    timeout_count: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct TickClock {
    period_us: u64,
    next_tick_us: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct FitDecision {
    pub fits: bool,
    pub deadline_us: u64,
    pub interval_us: u64,
}

#[derive(Clone, Copy, Debug)]
pub struct TickWindow {
    pub tick_start_us: u64,
    pub tick_end_us: u64,
    pub tx_deadline_us: u64,
}

impl TickWindow {
    pub fn fits_tx(&self, tx_start_us: u64, duration_us: u64) -> bool {
        tx_start_us.saturating_add(duration_us) <= self.tx_deadline_us
    }

    pub fn budget_us_from(&self, tx_start_us: u64) -> u64 {
        self.tx_deadline_us.saturating_sub(tx_start_us)
    }
}

impl TickClock {
    pub fn new(now_us: u64, period_us: u64) -> Self {
        let period_us = period_us.max(1);
        Self {
            period_us,
            next_tick_us: now_us.wrapping_add(period_us),
        }
    }

    pub fn period_us(&self) -> u64 {
        self.period_us
    }

    pub fn next_tick_boundary_us(&self) -> u64 {
        self.next_tick_us
    }

    pub fn check_fit(&self, now_us: u64, duration_us: u64, guard_us: u64) -> FitDecision {
        let deadline_us = self.next_tick_us;
        let interval_us = deadline_us.saturating_sub(now_us);
        let total_us = duration_us.saturating_add(guard_us);
        let fits = now_us.saturating_add(total_us) <= deadline_us;
        FitDecision {
            fits,
            deadline_us,
            interval_us,
        }
    }

    pub fn window(&self, tick_start_us: u64, guard_us: u64) -> TickWindow {
        let tick_end_us = tick_start_us.wrapping_add(self.period_us);
        let tx_deadline_us = tick_end_us.saturating_sub(guard_us);
        TickWindow {
            tick_start_us,
            tick_end_us,
            tx_deadline_us,
        }
    }

    pub fn align(&mut self, tick_start_us: u64) {
        self.next_tick_us = tick_start_us.wrapping_add(self.period_us);
    }

    pub fn poll(&mut self, now_us: u64) -> Option<u64> {
        if now_us < self.next_tick_us {
            return None;
        }

        let tick_start_us = self.next_tick_us;
        loop {
            self.next_tick_us = self.next_tick_us.wrapping_add(self.period_us);
            if now_us < self.next_tick_us {
                break;
            }
        }
        Some(tick_start_us)
    }
}

impl TickTracker {
    pub const fn new(lock_timeout_ms: u64, tick_period_ms: u64) -> Self {
        let tick_period_ms = if tick_period_ms == 0 { 1 } else { tick_period_ms };
        Self {
            state: LinkState::Search,
            lock_timeout_ms,
            tick_period_ms,
            max_tick_slop: DEFAULT_TICK_SLOP_TICKS,
            last_tick_seq: None,
            last_rx_ms: None,
            rx_count: 0,
            missed_ticks: 0,
            seq_anomaly_count: 0,
            dup_count: 0,
            lock_count: 0,
            timeout_count: 0,
        }
    }

    pub const fn state(&self) -> LinkState {
        self.state
    }

    pub const fn last_tick_seq(&self) -> Option<u16> {
        self.last_tick_seq
    }

    pub const fn last_rx_ms(&self) -> Option<u64> {
        self.last_rx_ms
    }

    pub const fn rx_count(&self) -> u32 {
        self.rx_count
    }

    pub const fn missed_ticks(&self) -> u32 {
        self.missed_ticks
    }

    pub const fn seq_anomaly_count(&self) -> u32 {
        self.seq_anomaly_count
    }

    pub const fn dup_count(&self) -> u32 {
        self.dup_count
    }

    pub const fn lock_count(&self) -> u32 {
        self.lock_count
    }

    pub const fn timeout_count(&self) -> u32 {
        self.timeout_count
    }

    pub fn on_uplink(&mut self, now_ms: u64, tick_seq: u16) -> TickUpdate {
        let mut update = TickUpdate::default();

        if let (Some(prev_seq), Some(last_ms)) = (self.last_tick_seq, self.last_rx_ms) {
            let delta = tick_seq.wrapping_sub(prev_seq);
            if delta == 0 {
                self.dup_count = self.dup_count.wrapping_add(1);
            } else {
                let max_plausible = self.max_plausible_delta(now_ms, last_ms);
                if delta > max_plausible {
                    update.seq_anomaly = delta;
                    self.seq_anomaly_count = self.seq_anomaly_count.wrapping_add(1);
                } else if delta > 1 {
                    let missed = delta - 1;
                    update.missed = missed;
                    self.missed_ticks = self.missed_ticks.wrapping_add(missed as u32);
                }
            }
        }

        self.last_tick_seq = Some(tick_seq);
        self.last_rx_ms = Some(now_ms);
        self.rx_count = self.rx_count.wrapping_add(1);

        if self.state == LinkState::Search {
            self.state = LinkState::Locked;
            self.lock_count = self.lock_count.wrapping_add(1);
            update.event = LinkEvent::LockAcquired;
        }

        update
    }

    pub fn poll(&mut self, now_ms: u64) -> LinkEvent {
        if self.state != LinkState::Locked {
            return LinkEvent::None;
        }

        let Some(last_ms) = self.last_rx_ms else {
            self.state = LinkState::Search;
            self.timeout_count = self.timeout_count.wrapping_add(1);
            return LinkEvent::LockLost;
        };

        if now_ms.saturating_sub(last_ms) >= self.lock_timeout_ms {
            self.state = LinkState::Search;
            self.timeout_count = self.timeout_count.wrapping_add(1);
            return LinkEvent::LockLost;
        }

        LinkEvent::None
    }

    fn max_plausible_delta(&self, now_ms: u64, last_ms: u64) -> u16 {
        let elapsed_ms = now_ms.saturating_sub(last_ms);
        let period_ms = self.tick_period_ms.max(1);
        let mut max_expected = (elapsed_ms + period_ms - 1) / period_ms;
        if max_expected == 0 {
            max_expected = 1;
        }
        let max_with_slop = max_expected.saturating_add(self.max_tick_slop as u64);
        if max_with_slop > u16::MAX as u64 {
            u16::MAX
        } else {
            max_with_slop as u16
        }
    }
}
