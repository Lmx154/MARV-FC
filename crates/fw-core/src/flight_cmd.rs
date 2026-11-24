//! The station's five flight commands (link-contract section 10, Amendments A and C): the COMMAND_LONG decode table, and
//! the single-slot hand-off of one command from core 0 to the flight step and of its answer back.
//!
//! The hand-off ([`Mailbox`]) is two 32-bit words, each a 24-bit sequence number and an 8-bit payload; sequence 0 means
//! empty. Core 0 claims the mailbox (one command at a time, across both links and both of core 0's executors), posts the
//! request with `compare_exchange(0, word)`; the flight step takes it with `swap(0)`, so a request is consumed at most
//! once and cannot be read again after a restart of either side (the word is empty once taken; a board reset clears
//! every static). The flight step answers with one store of the answer word tagged with the request's sequence number;
//! core 0 reads only the answer with its own number, so a late answer to an older command is never taken for a newer
//! one. Core 0 withdraws a request that was not taken within its bound with `compare_exchange(word, 0)`: success
//! proves the flight step never saw it. Core 0 keeps the command as a [`Pending`] and checks it from its normal task. Nothing here allocates, takes a critical section or loops on the flight
//! step's side ([`Mailbox::take`], [`Mailbox::answer`]); `thumbv8m.main` has the 32-bit compare-and-swap.

use core::sync::atomic::{AtomicU32, Ordering};

use mavlink_device::marv::MavCmd;

/// One of the five flight commands, its parameters checked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlightCommand {
    /// 300 `MAV_CMD_MISSION_START`, parameters ignored.
    Start,
    /// 193 `MAV_CMD_DO_PAUSE_CONTINUE`, param1 0.
    Pause,
    /// 176 `MAV_CMD_DO_SET_MODE`, param1 1, param2 3 (HOLD).
    Abort,
    /// 176 `MAV_CMD_DO_SET_MODE`, param1 1, param2 6 (LAND).
    Land,
    /// 20 `MAV_CMD_NAV_RETURN_TO_LAUNCH`.
    Return,
}

/// What [`decode`] made of a COMMAND_LONG.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decoded {
    /// Not one of the five: the command goes where it went before.
    Other,
    /// One of the five with parameters the table does not give: DENIED (section 10, DENIED item 3), reason code 0.
    BadParams,
    /// A flight command for the state machine.
    Command(FlightCommand),
}

/// The decode table of section 10. Only the parameters the contract's DENIED item 3 names are checked: Pause's param1,
/// DO_SET_MODE's param1 and param2; Start's are ignored ("the vehicle ignores them"), and Return's are not checked
/// (item 3 does not list them).
pub fn decode(command: MavCmd, params: &[f32; 7]) -> Decoded {
    match command {
        MavCmd::MAV_CMD_MISSION_START => Decoded::Command(FlightCommand::Start),
        MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH => Decoded::Command(FlightCommand::Return),
        MavCmd::MAV_CMD_DO_PAUSE_CONTINUE if params[0] == 0.0 => Decoded::Command(FlightCommand::Pause),
        MavCmd::MAV_CMD_DO_SET_MODE if params[0] == 1.0 && params[1] == 3.0 => Decoded::Command(FlightCommand::Abort),
        MavCmd::MAV_CMD_DO_SET_MODE if params[0] == 1.0 && params[1] == 6.0 => Decoded::Command(FlightCommand::Land),
        MavCmd::MAV_CMD_DO_PAUSE_CONTINUE | MavCmd::MAV_CMD_DO_SET_MODE => Decoded::BadParams,
        _ => Decoded::Other,
    }
}

/// The ground station's (system, component): link-contract section 2, "Ground station: system 255, component 190".
pub const STATION: (u8, u8) = (255, 190);

/// The station's own latest HEARTBEAT time, from `kept` (the value so far) and the RC port's latest ground-station
/// HEARTBEAT (`last_gcs_us`, `sender`; any sender of a HEARTBEAT of type GCS sets both): that time when the sender is
/// exactly [`STATION`], else `kept` (another sender never moves it and never clears it).
pub fn station_heartbeat_us(kept: Option<u64>, last_gcs_us: Option<u64>, sender: Option<(u8, u8)>) -> Option<u64> {
    match (last_gcs_us, sender) {
        (Some(t), Some(s)) if s == STATION => Some(t),
        _ => kept,
    }
}

/// The request payload's radio-link bit (clear: USB).
const RADIO: u8 = 0x10;

/// The request payload: the command (1 to 5) and the link it came on.
pub fn request_byte(command: FlightCommand, radio: bool) -> u8 {
    let c = match command {
        FlightCommand::Start => 1,
        FlightCommand::Pause => 2,
        FlightCommand::Abort => 3,
        FlightCommand::Land => 4,
        FlightCommand::Return => 5,
    };
    c | if radio { RADIO } else { 0 }
}

/// The inverse of [`request_byte`]: the command and whether it came on the radio link; `None` for any other byte.
pub fn request_of(byte: u8) -> Option<(FlightCommand, bool)> {
    let c = match byte & !RADIO {
        1 => FlightCommand::Start,
        2 => FlightCommand::Pause,
        3 => FlightCommand::Abort,
        4 => FlightCommand::Land,
        5 => FlightCommand::Return,
        _ => return None,
    };
    Some((c, byte & RADIO != 0))
}

/// The answer payload of an ACCEPTED command; any other value is DENIED with that value as the reason code.
pub const ACCEPTED: u8 = 0xFF;

/// The sequence numbers' range: 24 bits, 0 excluded.
const SEQ_MASK: u32 = 0x00FF_FFFF;

fn word(seq: u32, payload: u8) -> u32 {
    (seq << 8) | u32::from(payload)
}

/// The single-slot hand-off, in a `static`.
pub struct Mailbox {
    /// Core 0 -> flight step: the request word; 0 when empty.
    req: AtomicU32,
    /// Flight step -> core 0: the latest answer word; 0 before the first.
    ans: AtomicU32,
    /// Core 0 only: the sequence number of the command outstanding (claimed, not yet ended); 0 when none.
    claim: AtomicU32,
    /// Core 0 only: the last sequence number given out.
    last: AtomicU32,
}

impl Default for Mailbox {
    fn default() -> Self {
        Self::new()
    }
}

impl Mailbox {
    /// Empty.
    pub const fn new() -> Self {
        Self {
            req: AtomicU32::new(0),
            ans: AtomicU32::new(0),
            claim: AtomicU32::new(0),
            last: AtomicU32::new(0),
        }
    }

    /// Core 0: claims the mailbox and posts `payload` with a new sequence number, returned; `None` when another command
    /// is outstanding (claimed and not yet [`Mailbox::end`]ed).
    pub fn post(&self, payload: u8) -> Option<u32> {
        let mut seq = (self.last.load(Ordering::Relaxed).wrapping_add(1)) & SEQ_MASK;
        if seq == 0 {
            seq = 1;
        }
        self.claim.compare_exchange(0, seq, Ordering::Acquire, Ordering::Relaxed).ok()?;
        self.last.store(seq, Ordering::Relaxed);
        // The claim makes this the only poster; the slot is empty unless a request was neither taken nor withdrawn,
        // which `end` rules out. Fail closed all the same.
        if self.req.compare_exchange(0, word(seq, payload), Ordering::Release, Ordering::Relaxed).is_err() {
            self.claim.store(0, Ordering::Release);
            return None;
        }
        Some(seq)
    }

    /// Core 0: the answer to `seq`, if the flight step has given it.
    pub fn poll(&self, seq: u32) -> Option<u8> {
        let a = self.ans.load(Ordering::Acquire);
        (seq != 0 && a >> 8 == seq).then_some(a as u8)
    }

    /// Core 0: withdraws the request `seq` (`payload` as posted). True: it was never taken, the flight step will not see
    /// it. False: the flight step has taken it (its answer follows in the same step).
    pub fn withdraw(&self, seq: u32, payload: u8) -> bool {
        self.req
            .compare_exchange(word(seq, payload), 0, Ordering::AcqRel, Ordering::Relaxed)
            .is_ok()
    }

    /// Core 0: ends command `seq` (answered, or withdrawn, or given up), so the next can be posted. A request not yet
    /// taken is withdrawn first.
    pub fn end(&self, seq: u32, payload: u8) {
        let _ = self.withdraw(seq, payload);
        let _ = self.claim.compare_exchange(seq, 0, Ordering::Release, Ordering::Relaxed);
    }

    /// Flight step: takes the waiting request, at most once: (sequence number, payload). An empty slot is seen with a
    /// plain load and left alone; a request posted after that load is taken by the next call.
    pub fn take(&self) -> Option<(u32, u8)> {
        if self.req.load(Ordering::Relaxed) == 0 {
            return None;
        }
        let w = self.req.swap(0, Ordering::AcqRel);
        (w != 0).then_some((w >> 8, w as u8))
    }

    /// Flight step: the answer to request `seq`.
    pub fn answer(&self, seq: u32, payload: u8) {
        self.ans.store(word(seq & SEQ_MASK, payload), Ordering::Release);
    }
}

/// How a command [`Pending::check`] ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exchange {
    /// The flight step answered: [`ACCEPTED`] or a reason code.
    Answered(u8),
    /// Not taken within the bound: withdrawn untaken, never applied.
    Timeout,
}

/// Core 0: a command posted and not yet ended. Core 0 checks it from its normal task (no wait anywhere).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pending {
    seq: u32,
    payload: u8,
    posted_us: u64,
    taken: bool,
}

impl Pending {
    /// Posts `payload` at `now_us`; `None` when another command is outstanding (nothing posted).
    pub fn post(mailbox: &Mailbox, payload: u8, now_us: u64) -> Option<Pending> {
        let seq = mailbox.post(payload)?;
        Some(Pending { seq, payload, posted_us: now_us, taken: false })
    }

    /// The answer, if it has come, or the timeout: the flight step has `bound_us` from the post to take the request
    /// (then it is withdrawn untaken). A request it took was applied: it is never timed out, its answer (given in the
    /// step that took it) is waited for until it comes or [`Pending::abandon`] (its link went down). `Some` ends the
    /// command (the mailbox is free for the next); `None`: still waiting.
    pub fn check(&mut self, mailbox: &Mailbox, now_us: u64, bound_us: u64) -> Option<Exchange> {
        if let Some(a) = mailbox.poll(self.seq) {
            mailbox.end(self.seq, self.payload);
            return Some(Exchange::Answered(a));
        }
        if !self.taken && now_us.saturating_sub(self.posted_us) >= bound_us {
            if mailbox.withdraw(self.seq, self.payload) {
                mailbox.end(self.seq, self.payload);
                return Some(Exchange::Timeout);
            }
            self.taken = true;
        }
        None
    }

    /// Ends the command unanswered (its link went down): withdrawn if not taken yet.
    pub fn abandon(self, mailbox: &Mailbox) {
        mailbox.end(self.seq, self.payload);
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    const P: [f32; 7] = [0.0; 7];

    fn with(i: usize, v: f32, j: usize, w: f32) -> [f32; 7] {
        let mut p = P;
        p[i] = v;
        p[j] = w;
        p
    }

    #[test]
    fn decode_table_of_the_contract() {
        use FlightCommand::*;
        assert_eq!(decode(MavCmd::MAV_CMD_MISSION_START, &P), Decoded::Command(Start));
        assert_eq!(decode(MavCmd::MAV_CMD_MISSION_START, &[1.0; 7]), Decoded::Command(Start), "ignored");
        assert_eq!(decode(MavCmd::MAV_CMD_DO_PAUSE_CONTINUE, &P), Decoded::Command(Pause));
        assert_eq!(decode(MavCmd::MAV_CMD_DO_PAUSE_CONTINUE, &with(0, 1.0, 1, 0.0)), Decoded::BadParams, "continue");
        assert_eq!(decode(MavCmd::MAV_CMD_DO_PAUSE_CONTINUE, &with(0, f32::NAN, 1, 0.0)), Decoded::BadParams);
        assert_eq!(decode(MavCmd::MAV_CMD_DO_SET_MODE, &with(0, 1.0, 1, 3.0)), Decoded::Command(Abort));
        assert_eq!(decode(MavCmd::MAV_CMD_DO_SET_MODE, &with(0, 1.0, 1, 6.0)), Decoded::Command(Land));
        for (p1, p2) in [(0.0, 3.0), (0.0, 6.0), (1.0, 4.0), (1.0, 0.0), (2.0, 3.0), (1.0, 3.5), (1.0, f32::NAN)] {
            assert_eq!(decode(MavCmd::MAV_CMD_DO_SET_MODE, &with(0, p1, 1, p2)), Decoded::BadParams, "{p1} {p2}");
        }
        assert_eq!(decode(MavCmd::MAV_CMD_NAV_RETURN_TO_LAUNCH, &P), Decoded::Command(Return));
        for c in [
            MavCmd::MAV_CMD_PREFLIGHT_CALIBRATION,
            MavCmd::MAV_CMD_PREFLIGHT_STORAGE,
            MavCmd::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            MavCmd::MAV_CMD_DO_MOTOR_TEST,
            MavCmd::MAV_CMD_COMPONENT_ARM_DISARM,
            MavCmd::MAV_CMD_NAV_LAND,
        ] {
            assert_eq!(decode(c, &P), Decoded::Other);
        }
    }

    #[test]
    fn only_the_station_sender_moves_the_station_heartbeat_time() {
        assert_eq!(station_heartbeat_us(None, None, None), None, "no heartbeat: none");
        let t = station_heartbeat_us(None, Some(1_000), Some(STATION));
        assert_eq!(t, Some(1_000), "the station at t1");
        let t = station_heartbeat_us(t, Some(2_000), Some((254, 190)));
        assert_eq!(t, Some(1_000), "another GCS-type sender at t2 does not move it");
        for other in [(255, 191), (0, 190), (1, 1), (255, 0)] {
            assert_eq!(station_heartbeat_us(t, Some(2_500), Some(other)), Some(1_000), "{other:?}");
        }
        assert_eq!(station_heartbeat_us(t, None, None), Some(1_000), "never cleared");
        let t = station_heartbeat_us(t, Some(3_000), Some(STATION));
        assert_eq!(t, Some(3_000), "the station at t3");
        assert_eq!(station_heartbeat_us(None, Some(2_000), Some((254, 190))), None, "only others: none");
        assert_eq!(STATION, (255, 190));
    }

    #[test]
    fn request_byte_round_trips_and_rejects_others() {
        use FlightCommand::*;
        for c in [Start, Pause, Abort, Land, Return] {
            for radio in [false, true] {
                assert_eq!(request_of(request_byte(c, radio)), Some((c, radio)));
            }
        }
        for b in [0u8, 6, 0x10, 0x16, 0x20, 0xFF] {
            assert_eq!(request_of(b), None, "{b:#x}");
        }
    }

    #[test]
    fn a_request_is_taken_exactly_once_and_answered_with_its_sequence_number() {
        let m = Mailbox::new();
        let s = m.post(0x11).unwrap();
        assert_eq!(m.post(0x12), None, "one command at a time");
        assert_eq!(m.take(), Some((s, 0x11)));
        assert_eq!(m.take(), None, "consumed once");
        assert_eq!(m.poll(s), None);
        m.answer(s, ACCEPTED);
        assert_eq!(m.poll(s), Some(ACCEPTED));
        assert!(!m.withdraw(s, 0x11), "already taken");
        m.end(s, 0x11);
        let s2 = m.post(0x13).unwrap();
        assert_ne!(s2, s);
        assert_eq!(m.poll(s2), None, "the old answer is not the new one's");
    }

    #[test]
    fn take_on_an_empty_slot_is_none_and_leaves_it_empty_and_postable() {
        let m = Mailbox::new();
        for _ in 0..3 {
            assert_eq!(m.take(), None);
            assert_eq!(m.req.load(Ordering::Relaxed), 0);
        }
        let s = m.post(0x12).unwrap();
        assert_eq!(m.take(), Some((s, 0x12)), "a request posted after empty takes is taken once");
        assert_eq!(m.take(), None);
        assert_eq!(m.poll(s), None, "the empty takes answered nothing");
    }

    #[test]
    fn no_replay_after_a_restart_of_the_flight_side() {
        // The flight side keeps no state: a restarted reader finds the slot empty once the request was taken.
        let m = Mailbox::new();
        let s = m.post(0x14).unwrap();
        assert_eq!(m.take(), Some((s, 0x14)));
        // ... the flight side restarts here, before it answered ...
        for _ in 0..3 {
            assert_eq!(m.take(), None);
        }
        // A board reset restarts both sides: every static is new and empty.
        let fresh = Mailbox::new();
        assert_eq!(fresh.take(), None);
        assert_eq!(fresh.poll(0), None, "sequence 0 is never an answer");
    }

    #[test]
    fn sequence_numbers_wrap_past_zero_and_a_stale_answer_is_never_taken() {
        let m = Mailbox::new();
        m.last.store(SEQ_MASK - 1, Ordering::Relaxed);
        let a = m.post(1).unwrap();
        assert_eq!(a, SEQ_MASK);
        assert_eq!(m.take(), Some((a, 1)));
        m.answer(a, 4);
        m.end(a, 1);
        let b = m.post(2).unwrap();
        assert_eq!(b, 1, "0 is skipped");
        assert_eq!(m.poll(b), None, "the answer to {a:#x} is not {b}'s");
        assert_eq!(m.take(), Some((1, 2)));
        m.answer(b, 8);
        assert_eq!(m.poll(b), Some(8));
        m.end(b, 2);
    }

    #[test]
    fn pending_check_answers_and_times_out_both_ways_and_abandon_withdraws() {
        let m = Mailbox::new();
        // Answered at the third check.
        let mut p = Pending::post(&m, 0x13, 0).unwrap();
        assert_eq!(p.check(&m, 10, 100), None);
        let (s, b) = m.take().unwrap();
        assert_eq!(b, 0x13);
        assert_eq!(p.check(&m, 20, 100), None);
        m.answer(s, 13);
        assert_eq!(p.check(&m, 30, 100), Some(Exchange::Answered(13)));
        // Never taken: withdrawn at the bound; the flight step then finds nothing.
        let mut p = Pending::post(&m, 0x11, 1_000).unwrap();
        assert_eq!(Pending::post(&m, 0x12, 1_000), None, "one at a time");
        assert_eq!(p.check(&m, 1_099, 100), None);
        assert_eq!(p.check(&m, 1_100, 100), Some(Exchange::Timeout));
        assert_eq!(m.take(), None, "a withdrawn request is never applied");
        // Taken before the bound, answered after it: the real answer.
        let mut p = Pending::post(&m, 0x14, 3_000).unwrap();
        let (s, _) = m.take().unwrap();
        assert_eq!(p.check(&m, 3_150, 100), None);
        m.answer(s, ACCEPTED);
        assert_eq!(p.check(&m, 3_160, 100), Some(Exchange::Answered(ACCEPTED)));
        // Abandoned untaken: withdrawn, and the mailbox is free.
        let p = Pending::post(&m, 0x15, 4_000).unwrap();
        p.abandon(&m);
        assert_eq!(m.take(), None);
        assert!(Pending::post(&m, 0x15, 4_001).is_some());
    }

    #[test]
    fn a_taken_command_is_never_timed_out_only_answered_or_dropped_with_its_link() {
        let m = Mailbox::new();
        // Taken, answered long after every bound: the real answer, never a timeout.
        let mut p = Pending::post(&m, 0x13, 0).unwrap();
        let (s, _) = m.take().unwrap();
        assert_eq!(p.check(&m, 100, 100), None, "taken: wait for its answer");
        for t in [200, 10_000, 60_000_000] {
            assert_eq!(p.check(&m, t, 100), None, "still waiting at {t}");
        }
        m.answer(s, ACCEPTED);
        assert_eq!(p.check(&m, 60_000_001, 100), Some(Exchange::Answered(ACCEPTED)));
        // Taken, never answered: waiting until the link goes down, then dropped; the mailbox is free again.
        let mut p = Pending::post(&m, 0x14, 1_000).unwrap();
        let (old, _) = m.take().unwrap();
        assert_eq!(p.check(&m, 1_000_000, 100), None);
        assert_eq!(Pending::post(&m, 0x15, 1_000_000), None, "still outstanding");
        p.abandon(&m);
        // The next command gets its own answer, not the old one's (late or not).
        let mut q = Pending::post(&m, 0x15, 2_000_000).unwrap();
        m.answer(old, 4);
        assert_eq!(q.check(&m, 2_000_010, 100), None, "the answer to {old} is not the next command's");
        let (s, b) = m.take().unwrap();
        assert_eq!((b, s != old), (0x15, true));
        m.answer(s, 9);
        assert_eq!(q.check(&m, 2_000_020, 100), Some(Exchange::Answered(9)));
        assert!(Pending::post(&m, 0x11, 2_000_030).is_some(), "ended: free");
    }
}
