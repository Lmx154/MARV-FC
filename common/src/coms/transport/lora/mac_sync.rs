//phase 4
use super::mac_codec::FrameType;

pub const DEFAULT_LOCK_TIMEOUT_MS: u64 = 1000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SyncRole {
    GroundStation,
    Radio,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SyncState {
    Search,
    Locked,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SyncEvent {
    None,
    LockAcquired,
    LockLost,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SyncResult {
    pub event: SyncEvent,
    pub respond_pong: bool,
}

impl Default for SyncResult {
    fn default() -> Self {
        Self {
            event: SyncEvent::None,
            respond_pong: false,
        }
    }
}

pub struct MacSync {
    role: SyncRole,
    state: SyncState,
    lock_timeout_ms: u64,
    last_tick_seq: Option<u16>,
    last_rx_ms: Option<u64>,
    ping_rx: u32,
    pong_rx: u32,
    lock_count: u32,
    timeout_count: u32,
}

impl MacSync {
    pub const fn new(role: SyncRole, lock_timeout_ms: u64) -> Self {
        Self {
            role,
            state: SyncState::Search,
            lock_timeout_ms,
            last_tick_seq: None,
            last_rx_ms: None,
            ping_rx: 0,
            pong_rx: 0,
            lock_count: 0,
            timeout_count: 0,
        }
    }

    pub const fn state(&self) -> SyncState {
        self.state
    }

    pub const fn locked(&self) -> bool {
        matches!(self.state, SyncState::Locked)
    }

    pub const fn last_tick_seq(&self) -> Option<u16> {
        self.last_tick_seq
    }

    pub const fn last_rx_ms(&self) -> Option<u64> {
        self.last_rx_ms
    }

    pub const fn ping_rx(&self) -> u32 {
        self.ping_rx
    }

    pub const fn pong_rx(&self) -> u32 {
        self.pong_rx
    }

    pub const fn lock_count(&self) -> u32 {
        self.lock_count
    }

    pub const fn timeout_count(&self) -> u32 {
        self.timeout_count
    }

    pub fn on_frame(&mut self, now_ms: u64, frame_type: FrameType, tick_seq: u16) -> SyncResult {
        let mut result = SyncResult::default();

        match frame_type {
            FrameType::AcqPing => {
                self.ping_rx = self.ping_rx.wrapping_add(1);
                if matches!(self.role, SyncRole::Radio) {
                    result.respond_pong = true;
                    result.event = self.update_lock(now_ms, tick_seq);
                }
            }
            FrameType::AcqPong => {
                self.pong_rx = self.pong_rx.wrapping_add(1);
                if matches!(self.role, SyncRole::GroundStation) {
                    result.event = self.update_lock(now_ms, tick_seq);
                }
            }
        }

        result
    }

    pub fn poll(&mut self, now_ms: u64) -> SyncEvent {
        if self.state != SyncState::Locked {
            return SyncEvent::None;
        }

        let Some(last_ms) = self.last_rx_ms else {
            self.state = SyncState::Search;
            self.timeout_count = self.timeout_count.wrapping_add(1);
            return SyncEvent::LockLost;
        };

        if now_ms.saturating_sub(last_ms) >= self.lock_timeout_ms {
            self.state = SyncState::Search;
            self.timeout_count = self.timeout_count.wrapping_add(1);
            return SyncEvent::LockLost;
        }

        SyncEvent::None
    }

    fn update_lock(&mut self, now_ms: u64, tick_seq: u16) -> SyncEvent {
        self.last_tick_seq = Some(tick_seq);
        self.last_rx_ms = Some(now_ms);
        if self.state == SyncState::Search {
            self.state = SyncState::Locked;
            self.lock_count = self.lock_count.wrapping_add(1);
            return SyncEvent::LockAcquired;
        }
        SyncEvent::None
    }
}
