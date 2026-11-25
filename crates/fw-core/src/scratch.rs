//! The codec for the record kept in the watchdog scratch registers (layout version 2).
//!
//! Four 32-bit words survive a watchdog reset. Layout:
//!
//! | Word | Content                                                                              |
//! |------|--------------------------------------------------------------------------------------|
//! | S0   | magic `0x4D525702`; the low byte is the layout version (2); `0x4D525701` is refused  |
//! | S1   | reason (bits 0 to 7: 1 stall core 0, 2 stall flight, 3 requested) \| task id << 8 \| stall age in units of 10 ms << 16 (saturating at 255); bits 24 to 31 zero |
//! | S2   | uptime in ms                                                                         |
//! | S3   | `!(S0 ^ S1 ^ S2)`                                                                    |
//!
//! Power-on garbage is refused: anything that is not exactly an encoded record decodes to `None`.

/// S0 of a layout-version-2 record.
pub const MAGIC: u32 = 0x4D52_5702;

/// Why the watchdog reset the board.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    /// Core 0's executor stalled (code 1).
    StallCore0,
    /// The flight core stalled (code 2).
    StallFlight,
    /// The reset was requested by the firmware (code 3).
    Requested,
}

impl Reason {
    const fn code(self) -> u32 {
        match self {
            Self::StallCore0 => 1,
            Self::StallFlight => 2,
            Self::Requested => 3,
        }
    }

    const fn from_code(code: u32) -> Option<Self> {
        match code {
            1 => Some(Self::StallCore0),
            2 => Some(Self::StallFlight),
            3 => Some(Self::Requested),
            _ => None,
        }
    }
}

/// The content of a scratch record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Record {
    /// Why the reset happened.
    pub reason: Reason,
    /// The task that stalled (0 when not applicable).
    pub task_id: u8,
    /// The stall age in units of 10 ms, saturated at 255.
    pub age_10ms: u8,
    /// Uptime in ms when the record was written.
    pub uptime_ms: u32,
}

impl Record {
    /// The stall age in ms (a multiple of 10, at most 2550).
    pub const fn age_ms(&self) -> u32 {
        self.age_10ms as u32 * 10
    }
}

const fn check_word(s0: u32, s1: u32, s2: u32) -> u32 {
    !(s0 ^ s1 ^ s2)
}

/// Encodes a record into the four scratch words S0..S3. The age is divided by 10 and saturates
/// at 255 units (2550 ms).
pub const fn encode(reason: Reason, task_id: u8, age_ms: u32, uptime_ms: u32) -> [u32; 4] {
    let units = age_ms / 10;
    let units = if units > 255 { 255 } else { units };
    let s1 = reason.code() | (task_id as u32) << 8 | units << 16;
    [MAGIC, s1, uptime_ms, check_word(MAGIC, s1, uptime_ms)]
}

/// Decodes the four scratch words; `None` for anything that is not an encoded record: a wrong magic (including version 1), a wrong check word, an unknown reason, a non-zero top byte of S1.
pub const fn decode(words: [u32; 4]) -> Option<Record> {
    let [s0, s1, s2, s3] = words;
    if s0 != MAGIC || s3 != check_word(s0, s1, s2) || s1 >> 24 != 0 {
        return None;
    }
    let Some(reason) = Reason::from_code(s1 & 0xFF) else {
        return None;
    };
    Some(Record {
        reason,
        task_id: (s1 >> 8) as u8,
        age_10ms: (s1 >> 16) as u8,
        uptime_ms: s2,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const REASONS: [Reason; 3] = [Reason::StallCore0, Reason::StallFlight, Reason::Requested];

    #[test]
    fn round_trip_each_reason() {
        for (i, reason) in REASONS.into_iter().enumerate() {
            for (task, age, up) in [(0u8, 0u32, 0u32), (7, 120, 123_456), (255, 2550, u32::MAX)] {
                let words = encode(reason, task, age, up);
                let r = decode(words).unwrap();
                assert_eq!(r.reason, reason);
                assert_eq!(r.task_id, task);
                assert_eq!(r.age_ms(), age);
                assert_eq!(r.uptime_ms, up);
                assert_eq!(words[1] & 0xFF, i as u32 + 1);
            }
        }
    }

    #[test]
    fn exact_words_hand_computed() {
        let words = encode(Reason::StallFlight, 5, 125, 0x0001_2345);
        assert_eq!(words[0], 0x4D52_5702);
        assert_eq!(words[1], 0x000C_0502);
        assert_eq!(words[2], 0x0001_2345);
        assert_eq!(words[3], 0xB2A0_8EBA);
        assert_eq!(decode(words).unwrap().age_10ms, 12);
    }

    #[test]
    fn age_saturates_at_255_units() {
        for age in [2550u32, 2559, 2560, 10_000, u32::MAX] {
            let r = decode(encode(Reason::StallCore0, 1, age, 0)).unwrap();
            assert_eq!(r.age_10ms, 255);
            assert_eq!(r.age_ms(), 2550);
        }
        assert_eq!(
            decode(encode(Reason::StallCore0, 1, 9, 0))
                .unwrap()
                .age_10ms,
            0
        );
        assert_eq!(
            decode(encode(Reason::StallCore0, 1, 10, 0))
                .unwrap()
                .age_10ms,
            1
        );
    }

    #[test]
    fn every_single_bit_flip_is_rejected() {
        for reason in REASONS {
            let good = encode(reason, 9, 340, 987_654);
            let mut cases = 0;
            for word in 0..4 {
                for bit in 0..32 {
                    let mut bad = good;
                    bad[word] ^= 1 << bit;
                    assert_eq!(decode(bad), None, "word {word} bit {bit}");
                    cases += 1;
                }
            }
            assert_eq!(cases, 128);
        }
    }

    #[test]
    fn rejects_unknown_reason_and_top_byte_even_with_a_good_check_word() {
        let make = |s1: u32| [MAGIC, s1, 5, check_word(MAGIC, s1, 5)];
        assert!(decode(make(0x0001_0501)).is_some());
        for reason in [0u32, 4, 5, 255] {
            assert_eq!(decode(make(reason | 0x0001_0500)), None);
        }
        for top in [1u32, 0x80, 0xFF] {
            assert_eq!(decode(make(1 | top << 24)), None);
        }
    }

    #[test]
    fn rejects_power_on_garbage() {
        assert_eq!(decode([0; 4]), None);
        assert_eq!(decode([u32::MAX; 4]), None);
        let s1 = 1 | 5 << 8 | 3 << 16;
        let v1 = [0x4D52_5701, s1, 77, check_word(0x4D52_5701, s1, 77)];
        assert_eq!(decode(v1), None);
        let mut w = encode(Reason::Requested, 0, 0, 1);
        w[3] ^= 0x10;
        assert_eq!(decode(w), None);
        let mut x: u64 = 0x2545_F491_4F6C_DD1D;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            (x >> 16) as u32
        };
        for _ in 0..100_000 {
            let q = [next(), next(), next(), next()];
            assert_eq!(decode(q), None);
        }
    }
}
