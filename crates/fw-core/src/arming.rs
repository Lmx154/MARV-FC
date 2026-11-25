//! The pre-arm rule: the reasons core 0 does not permit
//! arming. Core 0's supervisor tick stores `Handshake::set_permit(reasons(..).is_empty())`; the
//! gate arms only with the permit set. The RC hook names the first reason in the refusal text.

use crate::health::State;
use crate::led::ROLL_CALL_END_MS;

/// The reasons arming is not permitted, a bitset; empty = permitted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Reasons(u8);

impl Reasons {
    /// P1: the boot's roll call has not ended.
    pub const BOOTING: Reasons = Reasons(1);
    /// P2: IMU1's row is not OK.
    pub const IMU1: Reasons = Reasons(1 << 1);
    /// P3: the ESC row is not OK.
    pub const ESC: Reasons = Reasons(1 << 2);
    /// P4: a calibration runs, the action book is busy or a motor-test spin is active.
    pub const BUSY: Reasons = Reasons(1 << 3);

    /// The bits, for a static.
    pub const fn bits(self) -> u8 {
        self.0
    }

    /// From [`Reasons::bits`].
    pub const fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    /// No reason: arming is permitted.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// Whether every bit of `other` is set.
    pub const fn contains(self, other: Reasons) -> bool {
        self.0 & other.0 == other.0
    }

    /// The WARNING STATUSTEXT of a refused arm: the first reason in the order P1..P4, or "busy" when there is none (the permit was clear because a command was being admitted).
    pub const fn refusal(self) -> &'static str {
        if self.contains(Self::BOOTING) {
            "Arming refused: booting"
        } else if self.contains(Self::IMU1) {
            "Arming refused: IMU1 not OK"
        } else if self.contains(Self::ESC) {
            "Arming refused: ESC not OK"
        } else {
            "Arming refused: busy"
        }
    }
}

/// The reasons at `uptime_ms` (the clock the status LED reads): P1 uptime below [`ROLL_CALL_END_MS`]; P2 IMU1's row state not OK; P3 the ESC row state not OK; P4 `calibrating`, `book_busy` or `test_spin`.
pub fn reasons(
    uptime_ms: u64,
    imu1: State,
    esc: State,
    calibrating: bool,
    book_busy: bool,
    test_spin: bool,
) -> Reasons {
    let mut bits = 0;
    if uptime_ms < u64::from(ROLL_CALL_END_MS) {
        bits |= Reasons::BOOTING.0;
    }
    if imu1 != State::Ok {
        bits |= Reasons::IMU1.0;
    }
    if esc != State::Ok {
        bits |= Reasons::ESC.0;
    }
    if calibrating || book_busy || test_spin {
        bits |= Reasons::BUSY.0;
    }
    Reasons(bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOT_OK: [State; 5] = [
        State::Starting,
        State::Degraded,
        State::Recovering,
        State::Failed,
        State::Absent,
    ];

    /// Empty only when P1 to P4 hold; each reason gives its text.
    #[test]
    fn reasons_empty_only_when_all_four_hold() {
        let ready = |uptime_ms| reasons(uptime_ms, State::Ok, State::Ok, false, false, false);
        assert_eq!(ROLL_CALL_END_MS, 9150);
        assert_eq!(ready(9149), Reasons::BOOTING);
        assert_eq!(ready(9149).refusal(), "Arming refused: booting");
        assert!(ready(9150).is_empty());
        assert!(ready(0).contains(Reasons::BOOTING));

        for state in NOT_OK {
            let r = reasons(9150, state, State::Ok, false, false, false);
            assert_eq!(r, Reasons::IMU1, "IMU1 {state:?}");
            assert_eq!(r.refusal(), "Arming refused: IMU1 not OK", "IMU1 {state:?}");
            let r = reasons(9150, State::Ok, state, false, false, false);
            assert_eq!(r, Reasons::ESC, "ESC {state:?}");
            assert_eq!(r.refusal(), "Arming refused: ESC not OK", "ESC {state:?}");
        }

        for (calibrating, book_busy, test_spin) in [
            (true, false, false),
            (false, true, false),
            (false, false, true),
        ] {
            let r = reasons(
                9150,
                State::Ok,
                State::Ok,
                calibrating,
                book_busy,
                test_spin,
            );
            assert_eq!(r, Reasons::BUSY);
            assert_eq!(r.refusal(), "Arming refused: busy");
        }

        let all = reasons(0, State::Failed, State::Absent, true, true, true);
        assert_eq!(all.bits(), 0b1111);
        assert_eq!(all.refusal(), "Arming refused: booting");
        assert_eq!(
            Reasons::from_bits(all.bits() & !Reasons::BOOTING.bits()).refusal(),
            "Arming refused: IMU1 not OK"
        );
        assert_eq!(Reasons::default().refusal(), "Arming refused: busy");
    }
}
