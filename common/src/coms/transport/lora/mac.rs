//! Deterministic tick-based MAC for LoRa links.
//!
//! Design goals (ELRS-like feel, link-only):
//! - Fixed tick cadence: at most one TX per tick.
//! - Slot schedule (U/D) to reserve airtime per direction.
//! - Three traffic classes per node (outbound): FAST (newest-wins), RELIABLE, NORMAL.
//! - No link-level ARQ here; reliability happens above the link.
//!
//! This module is `no_std` and allocation-free.

use heapless::{Deque, Vec};

use super::aggregate::AggBuilder;
use crate::params::{ParamId, ParamRegistry};

/// Max bytes carried as a single MAC payload (LoRaLink payload).
///
/// Matches `LoRaLink::MTU` (payload only, excluding LoRaLink header).
pub const LORA_MAC_MTU: usize = 240;

/// On-air MAC header length (inside the LoRaLink payload).
///
/// We prepend this header to every transmitted packet to carry tick/slot sync.
pub const MAC_HDR_LEN: usize = 4;

/// Max bytes available for an inner payload (e.g., a MAVLink frame).
pub const INNER_MTU: usize = LORA_MAC_MTU - MAC_HDR_LEN;

pub type MacPayload = Vec<u8, INNER_MTU>;

/// Magic/version for the MAC header.
pub const MAC_MAGIC: u8 = 0x4D; // 'M'
pub const MAC_VERSION: u8 = 0x01;

/// Flags for MAC header.
pub const MAC_FLAG_HAS_INNER: u8 = 0x01;

/// Default link tick frequency (Hz) used by GS/Radio firmware when no configuration is provided.
pub const LINK_TICK_HZ_DEFAULT: u16 = 50;

/// Default directional slot mode used by GS/Radio firmware when no configuration is provided.
pub const LINK_SLOT_MODE_DEFAULT: SlotMode = SlotMode::UdUd;

/// Firmware-facing configuration for the deterministic tick MAC.
///
/// This is intentionally small and `Copy` so it can be passed into embassy tasks.
#[derive(Clone, Copy, Debug)]
pub struct LinkMacConfig {
    /// Tick cadence in Hz.
    pub tick_hz: u16,
    /// Directional slot schedule.
    pub slot_mode: SlotMode,
    /// Maximum bytes allowed for FAST-lane payloads.
    ///
    /// Note: this is an application/link policy limit; the MAC still enforces `INNER_MTU`.
    pub fast_max_bytes: u16,
}

impl Default for LinkMacConfig {
    fn default() -> Self {
        Self {
            tick_hz: LINK_TICK_HZ_DEFAULT,
            slot_mode: LINK_SLOT_MODE_DEFAULT,
            // Default to the maximum that can fit inside the MAC inner payload.
            fast_max_bytes: INNER_MTU as u16,
        }
    }
}

impl LinkMacConfig {
    /// Build link MAC config from the firmware parameter registry.
    ///
    /// This is intended to run on the FC (rp235x) as the single source of truth.
    /// The resulting config can then be propagated to GS/Radio over MAVLink.
    pub fn from_params(params: &ParamRegistry) -> Self {
        let tick_hz = params.u32(ParamId::LinkTickHz).clamp(1, u16::MAX as u32) as u16;

        let slot_mode = match params.u32(ParamId::LinkSlotMode) {
            0 => SlotMode::UuuD,
            1 => SlotMode::UdUd,
            2 => SlotMode::Uddd,
            _ => LINK_SLOT_MODE_DEFAULT,
        };

        // 0 disables FAST (everything gets demoted by callers).
        let fast_max_bytes = params
            .u32(ParamId::LinkFastMaxB)
            .min(INNER_MTU as u32)
            .min(u16::MAX as u32) as u16;

        Self {
            tick_hz,
            slot_mode,
            fast_max_bytes,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlotDir {
    U,
    D,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlotMode {
    /// Control-centric: U U U D
    UuuD,
    /// Balanced: U D U D
    UdUd,
    /// Telemetry-centric: U D D D
    Uddd,
}

impl SlotMode {
    pub fn pattern(&self) -> &'static [SlotDir] {
        // Keep these as simple static patterns (no allocation, no const-generic fuss).
        static UUUD: [SlotDir; 4] = [SlotDir::U, SlotDir::U, SlotDir::U, SlotDir::D];
        static UDUD: [SlotDir; 4] = [SlotDir::U, SlotDir::D, SlotDir::U, SlotDir::D];
        static UDDD: [SlotDir; 4] = [SlotDir::U, SlotDir::D, SlotDir::D, SlotDir::D];

        match self {
            SlotMode::UuuD => &UUUD,
            SlotMode::UdUd => &UDUD,
            SlotMode::Uddd => &UDDD,
        }
    }
}

/// Which side of the directional schedule this node occupies.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MacRole {
    /// Transmit on `U` slots, receive on `D` slots.
    Uplink,
    /// Transmit on `D` slots, receive on `U` slots.
    Downlink,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct MacCounters {
    pub enqueued_fast: u32,
    pub enqueued_reliable: u32,
    pub enqueued_normal: u32,

    pub tx_fast: u32,
    pub tx_reliable: u32,
    pub tx_normal: u32,

    pub dropped_too_large: u32,
    pub dropped_reliable_overflow: u32,
    pub dropped_normal_overflow: u32,
    pub fast_replaced: u32,

    pub tx_inhibit_desync: u32,
}

#[derive(Clone, Copy, Debug)]
pub enum EnqueueClass {
    Fast,
    Reliable,
    Normal,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EnqueueError {
    TooLarge,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TickResult {
    /// Not yet time for the next tick.
    NotDue,
    /// Tick occurred, but this node is not allowed to transmit in this slot.
    RxSlot { tick_seq: u8, slot: SlotDir },
    /// Tick occurred and this node is allowed to transmit.
    /// `payload` may be None (caller may choose to send a SYNC-only packet).
    TxSlot {
        tick_seq: u8,
        slot: SlotDir,
        payload: Option<MacPayload>,
    },
}

/// Deterministic tick MAC for outbound traffic.
///
/// This owns only the *outbound* queues for a node.
pub struct LinkMac<const REL_CAP: usize, const NORM_CAP: usize> {
    role: MacRole,
    slot_mode: SlotMode,

    tick_interval_ms: u32,
    next_tick_ms: u32,
    tick_seq: u8,

    // Only meaningful on the non-master node.
    last_master_sync_ms: Option<u32>,

    fast: Option<MacPayload>,
    reliable: Deque<MacPayload, REL_CAP>,
    normal: Deque<MacPayload, NORM_CAP>,

    counters: MacCounters,
}

impl<const REL_CAP: usize, const NORM_CAP: usize> LinkMac<REL_CAP, NORM_CAP> {
    pub fn new(now_ms: u32, tick_hz: u16, slot_mode: SlotMode, role: MacRole) -> Self {
        let tick_interval_ms = hz_to_interval_ms(tick_hz);
        Self {
            role,
            slot_mode,
            tick_interval_ms,
            next_tick_ms: now_ms.wrapping_add(tick_interval_ms),
            tick_seq: 0,
            last_master_sync_ms: None,
            fast: None,
            reliable: Deque::new(),
            normal: Deque::new(),
            counters: MacCounters::default(),
        }
    }

    pub fn tick_interval_ms(&self) -> u32 {
        self.tick_interval_ms
    }

    /// Current tick sequence number.
    ///
    /// Note: this is the tick sequence that will be returned next time `poll_tick()` fires.
    pub fn next_tick_seq(&self) -> u8 {
        self.tick_seq
    }

    pub fn slot_mode(&self) -> SlotMode {
        self.slot_mode
    }

    /// Returns true iff we have seen a master sync packet recently.
    ///
    /// Intended for the non-master node to inhibit TX when desynced.
    pub fn master_synced_within_ms(&self, now_ms: u32, max_age_ms: u32) -> bool {
        let Some(last) = self.last_master_sync_ms else {
            return false;
        };
        let age = now_ms.wrapping_sub(last);
        // Wrap-safe: treat large ages as "in the past" across wrap.
        if age >= 0x8000_0000 {
            return false;
        }
        age <= max_age_ms
    }

    pub fn record_tx_inhibit_desync(&mut self) {
        self.counters.tx_inhibit_desync += 1;
    }

    pub fn counters(&self) -> MacCounters {
        self.counters
    }

    pub fn queue_depths(&self) -> (bool, usize, usize) {
        (self.fast.is_some(), self.reliable.len(), self.normal.len())
    }

    pub fn set_tick_hz(&mut self, now_ms: u32, tick_hz: u16) {
        let tick_interval_ms = hz_to_interval_ms(tick_hz);
        if tick_interval_ms == self.tick_interval_ms {
            return;
        }
        self.tick_interval_ms = tick_interval_ms;
        // Reset cadence relative to now to avoid bursts.
        self.next_tick_ms = now_ms.wrapping_add(tick_interval_ms);
    }

    pub fn set_slot_mode(&mut self, slot_mode: SlotMode) {
        self.slot_mode = slot_mode;
        self.tick_seq = 0;
    }

    /// Align local tick cadence/phase to the master.
    ///
    /// Call this on the **non-master** node whenever a packet is received from
    /// the master that carries `master_tick_seq`.
    ///
    /// We treat the receive time as the current tick boundary and schedule the
    /// next tick one interval later.
    pub fn on_master_sync(&mut self, now_ms: u32, master_tick_seq: u8) {
        // Next local tick should correspond to the master's next tick.
        self.tick_seq = master_tick_seq.wrapping_add(1);
        self.next_tick_ms = now_ms.wrapping_add(self.tick_interval_ms);
        self.last_master_sync_ms = Some(now_ms);
    }

    pub fn time_until_tick_ms(&self, now_ms: u32) -> u32 {
        let elapsed = now_ms.wrapping_sub(self.next_tick_ms);
        // If now_ms >= next_tick_ms (wrap-safe), we're due.
        if elapsed < 0x8000_0000 {
            0
        } else {
            self.next_tick_ms.wrapping_sub(now_ms)
        }
    }

    pub fn enqueue(&mut self, class: EnqueueClass, bytes: &[u8]) -> Result<(), EnqueueError> {
        // bytes are the *inner* payload; we reserve space for the MAC header.
        if bytes.len() > INNER_MTU {
            self.counters.dropped_too_large += 1;
            return Err(EnqueueError::TooLarge);
        }

        let mut payload: MacPayload = MacPayload::new();
        // Infallible due to the size check above.
        let _ = payload.extend_from_slice(bytes);

        match class {
            EnqueueClass::Fast => {
                if self.fast.is_some() {
                    self.counters.fast_replaced += 1;
                }
                self.fast = Some(payload);
                self.counters.enqueued_fast += 1;
            }
            EnqueueClass::Reliable => {
                if self.reliable.is_full() {
                    let _ = self.reliable.pop_front();
                    self.counters.dropped_reliable_overflow += 1;
                }
                let _ = self.reliable.push_back(payload);
                self.counters.enqueued_reliable += 1;
            }
            EnqueueClass::Normal => {
                if self.normal.is_full() {
                    let _ = self.normal.pop_front();
                    self.counters.dropped_normal_overflow += 1;
                }
                let _ = self.normal.push_back(payload);
                self.counters.enqueued_normal += 1;
            }
        }

        Ok(())
    }

    /// Advance the MAC by one tick if due.
    ///
    /// The master can choose to send a SYNC-only packet when `payload` is None.
    pub fn poll_tick(&mut self, now_ms: u32) -> TickResult {
        if self.time_until_tick_ms(now_ms) != 0 {
            return TickResult::NotDue;
        }

        // Move next tick forward by one interval (no catch-up bursts).
        self.next_tick_ms = now_ms.wrapping_add(self.tick_interval_ms);

        let tick_seq = self.tick_seq;
        let slot = self.slot_for_tick(tick_seq);
        self.tick_seq = self.tick_seq.wrapping_add(1);

        if !self.can_tx_in_slot(slot) {
            return TickResult::RxSlot { tick_seq, slot };
        }

        let payload = self.build_aggregate_payload();

        TickResult::TxSlot {
            tick_seq,
            slot,
            payload,
        }
    }

    fn build_aggregate_payload(&mut self) -> Option<MacPayload> {
        // Build a Link Aggregate Packet inside the inner payload.
        let mut out: MacPayload = MacPayload::new();
        let mut b = AggBuilder::new(&mut out);

        if let Some(fast) = self.fast.take() {
            if b.push_fast(&fast) {
                self.counters.tx_fast += 1;
            } else {
                // FAST must never stall behind aggregate overhead.
                // If it can't be inserted into an aggregate packet, fall back to sending it
                // as a legacy single-frame inner payload for this tick.
                self.counters.tx_fast += 1;
                return Some(fast);
            }
        }

        // Pack as many RELIABLE frames as fit.
        loop {
            let front_len = match self.reliable.front() {
                Some(p) => p.len(),
                None => break,
            };
            if !b.can_push(front_len) {
                // If we haven't packed anything yet, fall back to legacy single-frame send.
                // This prevents a frame that fits `INNER_MTU` from stalling forever due to
                // aggregate wrapper overhead.
                if b.is_empty() {
                    if let Some(p) = self.reliable.pop_front() {
                        self.counters.tx_reliable += 1;
                        return Some(p);
                    }
                }
                break;
            }
            if let Some(p) = self.reliable.pop_front() {
                if b.push_mav(&p) {
                    self.counters.tx_reliable += 1;
                } else {
                    // Should not happen if can_push was true; requeue at front.
                    let _ = self.reliable.push_front(p);
                    break;
                }
            }
        }

        // Pack as many NORMAL frames as fit.
        loop {
            let front_len = match self.normal.front() {
                Some(p) => p.len(),
                None => break,
            };
            if !b.can_push(front_len) {
                // Same legacy fallback as RELIABLE, but for NORMAL.
                if b.is_empty() {
                    if let Some(p) = self.normal.pop_front() {
                        self.counters.tx_normal += 1;
                        return Some(p);
                    }
                }
                break;
            }
            if let Some(p) = self.normal.pop_front() {
                if b.push_mav(&p) {
                    self.counters.tx_normal += 1;
                } else {
                    let _ = self.normal.push_front(p);
                    break;
                }
            }
        }

        if b.is_empty() {
            return None;
        }

        b.finish();
        Some(out)
    }

    fn slot_for_tick(&self, tick_seq: u8) -> SlotDir {
        let pat = self.slot_mode.pattern();
        let idx = (tick_seq as usize) % pat.len();
        pat[idx]
    }

    fn can_tx_in_slot(&self, slot: SlotDir) -> bool {
        match (self.role, slot) {
            (MacRole::Uplink, SlotDir::U) => true,
            (MacRole::Downlink, SlotDir::D) => true,
            _ => false,
        }
    }
}

fn hz_to_interval_ms(hz: u16) -> u32 {
    let hz = hz.max(1) as u32;
    (1000 / hz).max(1)
}
