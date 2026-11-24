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
//! proves the flight step never saw it. Nothing here allocates, takes a critical section or loops on the flight
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

    /// Flight step: takes the waiting request, at most once: (sequence number, payload).
    pub fn take(&self) -> Option<(u32, u8)> {
        let w = self.req.swap(0, Ordering::AcqRel);
        (w != 0).then_some((w >> 8, w as u8))
    }

    /// Flight step: the answer to request `seq`.
    pub fn answer(&self, seq: u32, payload: u8) {
        self.ans.store(word(seq & SEQ_MASK, payload), Ordering::Release);
    }
}

/// How [`exchange`] ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Exchange {
    /// The flight step answered: [`ACCEPTED`] or a reason code.
    Answered(u8),
    /// Another command was outstanding: nothing was posted.
    Busy,
    /// No answer within the bound. `taken`: the flight step had taken the request (so it was applied, and its answer
    /// was lost or late); false: it was withdrawn untaken and never applied.
    Timeout {
        /// The flight step had taken it.
        taken: bool,
    },
}

/// Core 0: posts `payload`, waits for its answer up to `bound_us` (by `now_us`, which is called once per poll), and
/// ends the command. A request taken but not answered by the bound gets a second `bound_us` (its answer comes in the
/// step that took it).
pub fn exchange(mailbox: &Mailbox, payload: u8, bound_us: u64, mut now_us: impl FnMut() -> u64) -> Exchange {
    let Some(seq) = mailbox.post(payload) else {
        return Exchange::Busy;
    };
    let start = now_us();
    let mut taken = false;
    let result = loop {
        if let Some(a) = mailbox.poll(seq) {
            break Exchange::Answered(a);
        }
        let waited = now_us().saturating_sub(start);
        if !taken && waited >= bound_us {
            if mailbox.withdraw(seq, payload) {
                break Exchange::Timeout { taken: false };
            }
            taken = true;
        }
        if waited >= bound_us.saturating_mul(2) {
            break Exchange::Timeout { taken: true };
        }
        core::hint::spin_loop();
    };
    mailbox.end(seq, payload);
    result
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::cell::Cell;

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
    fn exchange_answers_and_times_out_both_ways() {
        let m = Mailbox::new();
        // Answered: the flight step runs inside the clock.
        let t = Cell::new(0u64);
        let r = exchange(&m, 0x13, 100, || {
            t.set(t.get() + 10);
            if t.get() == 30
                && let Some((s, p)) = m.take()
            {
                assert_eq!(p, 0x13);
                m.answer(s, 13);
            }
            t.get()
        });
        assert_eq!(r, Exchange::Answered(13));
        // Never taken: withdrawn, and the flight step then finds nothing.
        let t = Cell::new(0u64);
        let r = exchange(&m, 0x11, 100, || {
            t.set(t.get() + 10);
            t.get()
        });
        assert_eq!(r, Exchange::Timeout { taken: false });
        assert_eq!(m.take(), None, "a withdrawn request is never applied");
        // Taken, never answered: the second bound runs out.
        let t = Cell::new(0u64);
        let r = exchange(&m, 0x12, 100, || {
            t.set(t.get() + 10);
            if t.get() == 50 {
                assert!(m.take().is_some());
            }
            t.get()
        });
        assert_eq!(r, Exchange::Timeout { taken: true });
        // Taken late (after the first bound), answered in the second: the real answer.
        let t = Cell::new(0u64);
        let taken = Cell::new(None);
        let r = exchange(&m, 0x12, 100, || {
            t.set(t.get() + 10);
            if t.get() == 90 {
                taken.set(m.take());
            }
            if t.get() == 150
                && let Some((s, _)) = taken.get()
            {
                m.answer(s, ACCEPTED);
            }
            t.get()
        });
        assert_eq!(r, Exchange::Answered(ACCEPTED));
        // Busy: a command outstanding on the other link.
        let s = m.post(0x15).unwrap();
        assert_eq!(exchange(&m, 0x11, 100, || 0), Exchange::Busy);
        m.end(s, 0x15);
        assert_eq!(m.take(), None, "ended untaken: withdrawn");
    }
}
