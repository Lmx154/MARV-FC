pub const DEFAULT_LOCK_TIMEOUT_MS: u64 = 1000;

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
}

impl Default for TickUpdate {
    fn default() -> Self {
        Self {
            event: LinkEvent::None,
            missed: 0,
        }
    }
}

pub struct TickTracker {
    state: LinkState,
    lock_timeout_ms: u64,
    last_tick_seq: Option<u16>,
    last_rx_ms: Option<u64>,
    rx_count: u32,
    missed_ticks: u32,
    lock_count: u32,
    timeout_count: u32,
}

impl TickTracker {
    pub const fn new(lock_timeout_ms: u64) -> Self {
        Self {
            state: LinkState::Search,
            lock_timeout_ms,
            last_tick_seq: None,
            last_rx_ms: None,
            rx_count: 0,
            missed_ticks: 0,
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

    pub const fn lock_count(&self) -> u32 {
        self.lock_count
    }

    pub const fn timeout_count(&self) -> u32 {
        self.timeout_count
    }

    pub fn on_uplink(&mut self, now_ms: u64, tick_seq: u16) -> TickUpdate {
        let mut update = TickUpdate::default();

        if let Some(prev_seq) = self.last_tick_seq {
            let delta = tick_seq.wrapping_sub(prev_seq);
            if delta > 1 {
                let missed = delta - 1;
                update.missed = missed;
                self.missed_ticks = self.missed_ticks.wrapping_add(missed as u32);
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
}
