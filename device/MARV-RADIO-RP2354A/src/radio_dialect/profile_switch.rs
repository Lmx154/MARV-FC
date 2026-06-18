//! Coordinated, link-wide RF profile switch with verification and symmetric rollback.
//!
//! Changing frequency/bandwidth on only one radio kills the link, so a profile change is a
//! *make-before-break* handshake driven by the ground station (master) over the **current** link:
//!
//! 1. GS validates the request, then transmits `LoRaSetProfile` to the vehicle (with retransmits)
//!    while staying on the current profile (`AwaitingPeerAck`).
//! 2. Vehicle validates, ACKs over the current link, and arms its own switch (`PendingApply`).
//! 3. Both sides apply the new profile on a short guard delay, then enter `Verifying`.
//! 4. Verification is PHY-level: a peer frame can only be decoded if both radios are on the same
//!    profile, so *hearing the peer after switching* proves the link came up on the new settings.
//! 5. If a side does not hear the peer before `VERIFY_TIMEOUT_MS`, it reverts to the previous
//!    profile. Because both sides store the same previous profile and run the same timer, every
//!    failure mode (command lost, ack lost, new link dead) converges back to the old profile.
//!
//! This module is the pure state machine; the bridge owns the radio and performs the actual
//! `apply_runtime_profile` in response to [`SwitchAction`]s, and feeds peer-frame events back via
//! [`ProfileSwitch::note_peer_seen`]. No hardware or channel access lives here, so it stays
//! testable in isolation.

use common::comms::links::lora::{LoraProfile, lora_profile_for_preset};
use common::protocol::hilink;

/// GS: total time to wait for the vehicle's ack (across retransmits) before giving up.
const GS_ACK_TIMEOUT_MS: u32 = 4_000;
/// GS: interval between `LoRaSetProfile` (re)transmissions while awaiting the ack.
const GS_RETRY_INTERVAL_MS: u32 = 600;
/// GS: maximum `LoRaSetProfile` transmissions before declaring the peer unreachable.
const GS_MAX_ATTEMPTS: u8 = 5;
/// GS: delay from receiving the ack to applying the new profile. Kept short — the ack already
/// crossed the link, so the GS can switch and start listening on the new profile promptly.
const GS_APPLY_GUARD_MS: u32 = 150;
/// Vehicle: delay from acking to applying the new profile. Must exceed the time for the ack to
/// actually leave on the old profile (one loop + airtime) so the GS reliably receives it before
/// the vehicle changes frequency. Bench-tunable; sized for a switch initiated on a fast profile.
const VEHICLE_APPLY_GUARD_MS: u32 = 1_000;
/// Both: time to hear the peer on the new profile before rolling back.
const VERIFY_TIMEOUT_MS: u32 = 8_000;

/// Requested RF profile change in wire terms (before building a typed [`LoraProfile`]). Small and
/// `Copy`/`Eq` so it lives in the radio state cache without pulling a `LoraProfile` (and its
/// non-`Eq` float modulation params) into it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProfileChange {
    pub command_seq: u16,
    pub preset: u8,
    pub tx_power_dbm: i8,
    pub frequency_hz: u32,
    pub flags: u16,
}

impl ProfileChange {
    pub const fn zeroed() -> Self {
        Self {
            command_seq: 0,
            preset: 0,
            tx_power_dbm: 0,
            frequency_hz: 0,
            flags: 0,
        }
    }

    pub const fn power_override(&self) -> bool {
        self.flags & hilink::lora_profile_flags::POWER_OVERRIDE != 0
    }

    /// Validate against the band plan and build the target profile, or `None` if the request is
    /// not a legal SRAD Mode C channel / power for its preset.
    pub fn to_profile(self) -> Option<LoraProfile> {
        hilink::band_plan::validate(
            self.preset,
            self.frequency_hz,
            self.tx_power_dbm,
            self.power_override(),
        )
        .ok()?;
        lora_profile_for_preset(self.preset, self.frequency_hz, self.tx_power_dbm)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SwitchPhase {
    Idle,
    /// GS only: `LoRaSetProfile` sent, waiting for the vehicle's ack.
    AwaitingPeerAck,
    /// Both: change agreed; apply the new profile once `deadline_ms` is reached.
    PendingApply,
    /// Both: new profile applied; waiting to hear the peer on it before `deadline_ms`.
    Verifying,
}

/// Outcome of [`ProfileSwitch::begin_vehicle`], mapped by the caller to a `LoRaCommandAck`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum VehicleArm {
    Accepted,
    DuplicateAccepted,
    Rejected,
    Busy,
}

/// Work for the bridge to perform after [`ProfileSwitch::poll`]. The state machine has already
/// advanced; the bridge just carries out the hardware effect.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SwitchAction {
    None,
    /// Apply the target profile now (build via [`ProfileChange::to_profile`]); capture the current
    /// profile as the rollback target first.
    Apply(ProfileChange),
    /// Verification succeeded — the peer was heard on the new profile (carries the command seq).
    Committed(u16),
    /// Verification timed out — re-apply the previously captured profile (carries the command seq).
    Revert(u16),
    /// GS: the vehicle never acked — abort; nothing was switched (carries the command seq).
    Failed(u16),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProfileSwitch {
    phase: SwitchPhase,
    change: ProfileChange,
    /// Phase-specific absolute deadline (ms): apply-at for `PendingApply`, timeout for the others.
    deadline_ms: u32,
    /// GS `AwaitingPeerAck`: next (re)transmit time.
    next_retry_ms: u32,
    attempts: u8,
    peer_seen: bool,
}

impl ProfileSwitch {
    pub const fn new() -> Self {
        Self {
            phase: SwitchPhase::Idle,
            change: ProfileChange::zeroed(),
            deadline_ms: 0,
            next_retry_ms: 0,
            attempts: 0,
            peer_seen: false,
        }
    }

    pub fn is_active(&self) -> bool {
        self.phase != SwitchPhase::Idle
    }

    /// GS: arm the switch after a host request. Refuses (returns `false`) if one is already in
    /// flight, so the caller can reject the new request as busy.
    pub fn begin_gs(&mut self, change: ProfileChange, now_ms: u32) -> bool {
        if self.phase != SwitchPhase::Idle {
            return false;
        }
        self.phase = SwitchPhase::AwaitingPeerAck;
        self.change = change;
        self.attempts = 0;
        self.next_retry_ms = now_ms; // transmit on the next scheduler pass
        self.deadline_ms = now_ms.wrapping_add(GS_ACK_TIMEOUT_MS);
        self.peer_seen = false;
        true
    }

    /// GS scheduler hook: return the `LoRaSetProfile` payload to (re)transmit, if due.
    pub fn take_retransmit(&mut self, now_ms: u32) -> Option<ProfileChange> {
        if self.phase != SwitchPhase::AwaitingPeerAck
            || self.attempts >= GS_MAX_ATTEMPTS
            || !reached(now_ms, self.next_retry_ms)
        {
            return None;
        }
        self.attempts += 1;
        self.next_retry_ms = now_ms.wrapping_add(GS_RETRY_INTERVAL_MS);
        Some(self.change)
    }

    /// GS: the vehicle accepted — schedule the coordinated apply.
    pub fn peer_accepted(&mut self, command_seq: u16, now_ms: u32) {
        if self.phase == SwitchPhase::AwaitingPeerAck && self.change.command_seq == command_seq {
            self.phase = SwitchPhase::PendingApply;
            self.deadline_ms = now_ms.wrapping_add(GS_APPLY_GUARD_MS);
        }
    }

    /// GS: the vehicle rejected — abandon, staying on the current profile.
    pub fn peer_rejected(&mut self, command_seq: u16) {
        if self.phase == SwitchPhase::AwaitingPeerAck && self.change.command_seq == command_seq {
            self.phase = SwitchPhase::Idle;
        }
    }

    /// Vehicle: accept a validated, non-duplicate change and schedule the apply.
    pub fn arm_vehicle(&mut self, change: ProfileChange, now_ms: u32) {
        self.phase = SwitchPhase::PendingApply;
        self.change = change;
        self.deadline_ms = now_ms.wrapping_add(VEHICLE_APPLY_GUARD_MS);
        self.peer_seen = false;
        self.attempts = 0;
    }

    /// Bridge: a valid peer frame arrived (proof the peer is on our current profile).
    pub fn note_peer_seen(&mut self) {
        if self.phase == SwitchPhase::Verifying {
            self.peer_seen = true;
        }
    }

    /// Bridge: abort the switch (e.g. the radio failed to apply the target).
    pub fn abort(&mut self) {
        self.phase = SwitchPhase::Idle;
    }

    /// Bridge: drive time-based transitions and return the next hardware action.
    pub fn poll(&mut self, now_ms: u32) -> SwitchAction {
        match self.phase {
            SwitchPhase::Idle => SwitchAction::None,
            SwitchPhase::AwaitingPeerAck => {
                if reached(now_ms, self.deadline_ms) {
                    let seq = self.change.command_seq;
                    self.phase = SwitchPhase::Idle;
                    SwitchAction::Failed(seq)
                } else {
                    SwitchAction::None
                }
            }
            SwitchPhase::PendingApply => {
                if reached(now_ms, self.deadline_ms) {
                    self.phase = SwitchPhase::Verifying;
                    self.deadline_ms = now_ms.wrapping_add(VERIFY_TIMEOUT_MS);
                    self.peer_seen = false;
                    SwitchAction::Apply(self.change)
                } else {
                    SwitchAction::None
                }
            }
            SwitchPhase::Verifying => {
                if self.peer_seen {
                    let seq = self.change.command_seq;
                    self.phase = SwitchPhase::Idle;
                    SwitchAction::Committed(seq)
                } else if reached(now_ms, self.deadline_ms) {
                    let seq = self.change.command_seq;
                    self.phase = SwitchPhase::Idle;
                    SwitchAction::Revert(seq)
                } else {
                    SwitchAction::None
                }
            }
        }
    }
}

/// `now >= deadline` in wrapping-u32 millisecond time (treats the difference as signed). Safe for
/// our short timeouts versus the ~49-day `u32` ms wrap.
const fn reached(now_ms: u32, deadline_ms: u32) -> bool {
    now_ms.wrapping_sub(deadline_ms) < 0x8000_0000
}
