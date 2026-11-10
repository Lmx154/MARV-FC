//! Bit timing of the PIO transport, computed from the system clock. Pure code: the PIO program
//! itself lives in `pio` (feature `embassy-rp`).
//!
//! # The program shape
//!
//! Every bit is three phases of `t3`, `t1` and `t2` PIO cycles, each between 1 and 16 (the delay
//! field of an instruction with one side-set bit is 4 bits wide):
//!
//! ```text
//! phase A: t3 cycles, line low             (out x, 1)
//! phase B: t1 cycles, line high            (jmp !x)
//! phase C: t2 cycles, line high for a 1, low for a 0
//! ```
//!
//! So T0H = t1, T0L = t2 + t3, T1H = t1 + t2 and T1L = t3, each times the cycle time. The cycle
//! time is `divider / clk_sys`; the divider is an integer, so there is no fractional-divider
//! jitter.

use crate::limits::*;

/// The largest divider tried.
const MAX_DIVIDER: u32 = 65_535;
/// Longest phase: a delay of 15 plus the instruction's own cycle.
const MAX_PHASE: u8 = 16;
/// Data bits per word on the wire.
const BITS_PER_WORD: u64 = 24;

/// Cycle counts and divider of the PIO program.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PioTiming {
    /// `clk_sys` the timing was computed for, in hertz.
    pub clk_sys_hz: u32,
    /// Integer clock divider of the state machine.
    pub divider: u16,
    /// Cycles of phase B: the high time of a 0.
    pub t1: u8,
    /// Cycles of phase C: the extra high time of a 1, the extra low time of a 0.
    pub t2: u8,
    /// Cycles of phase A: the low time of a 1.
    pub t3: u8,
}

/// The four times of a bit, in nanoseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BitTimes {
    /// "0" code, high.
    pub t0h_ns: u32,
    /// "1" code, high.
    pub t1h_ns: u32,
    /// "0" code, low.
    pub t0l_ns: u32,
    /// "1" code, low.
    pub t1l_ns: u32,
}

/// Why no timing was found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimingError {
    /// No integer divider and phase lengths put all four times inside the datasheet limits at
    /// this clock (too slow to resolve 220-380 ns, or too fast for a 16-bit divider).
    ClockOutOfRange,
}

impl PioTiming {
    /// PIO cycles in one bit.
    pub const fn cycles_per_bit(&self) -> u32 {
        self.t1 as u32 + self.t2 as u32 + self.t3 as u32
    }

    const fn ps(&self, cycles: u32) -> u64 {
        cycles as u64 * self.divider as u64 * 1_000_000_000_000 / self.clk_sys_hz as u64
    }

    /// The four times of a bit, rounded to the nearest nanosecond.
    pub const fn bit_times(&self) -> BitTimes {
        let t1 = self.t1 as u32;
        let t2 = self.t2 as u32;
        let t3 = self.t3 as u32;
        BitTimes {
            t0h_ns: ns(self.ps(t1)),
            t1h_ns: ns(self.ps(t1 + t2)),
            t0l_ns: ns(self.ps(t2 + t3)),
            t1l_ns: ns(self.ps(t3)),
        }
    }

    /// Time to send one 24-bit word, rounded up to a whole nanosecond.
    pub const fn word_ns(&self) -> u32 {
        (BITS_PER_WORD * self.ps(self.cycles_per_bit())).div_ceil(1_000) as u32
    }
}

const fn ns(ps: u64) -> u32 {
    ((ps + 500) / 1_000) as u32
}

/// Distance of `value` from the nearest of `[lo, hi]` (nanoseconds as picoseconds); negative when
/// outside.
const fn margin(value_ps: u64, lo_ns: u32, hi_ns: u32) -> i64 {
    let lo = lo_ns as i64 * 1_000;
    let hi = hi_ns as i64 * 1_000;
    let v = value_ps as i64;
    let below = v - lo;
    let above = hi - v;
    if below < above { below } else { above }
}

/// Finds the divider and cycle counts for `clk_sys_hz` that put all four times of a bit inside the
/// datasheet limits ([`crate::limits`]) with the largest margin to the nearest limit.
///
/// Returns [`TimingError::ClockOutOfRange`] when no combination fits: below about 8 MHz the
/// cycle time is too coarse to put all four times in their windows, and a clock of 0 has no
/// cycle time at all.
pub fn pio_timing(clk_sys_hz: u32) -> Result<PioTiming, TimingError> {
    if clk_sys_hz == 0 {
        return Err(TimingError::ClockOutOfRange);
    }
    let mut best: Option<(i64, PioTiming)> = None;
    let mut divider = 1u32;
    while divider <= MAX_DIVIDER {
        // Cycle time in picoseconds; one cycle must fit in the shortest high time.
        let cycle_ps = divider as u64 * 1_000_000_000_000 / clk_sys_hz as u64;
        if cycle_ps > T0H_MAX_NS as u64 * 1_000 {
            break;
        }
        let mut t1 = 1u8;
        while t1 <= MAX_PHASE {
            let m0h = margin(t1 as u64 * cycle_ps, T0H_MIN_NS, T0H_MAX_NS);
            let mut t2 = 1u8;
            while m0h >= 0 && t2 <= MAX_PHASE {
                let m1h = margin((t1 + t2) as u64 * cycle_ps, T1H_MIN_NS, T1H_MAX_NS);
                let mut t3 = 1u8;
                while m1h >= 0 && t3 <= MAX_PHASE {
                    let m1l = margin(t3 as u64 * cycle_ps, T1L_MIN_NS, T1L_MAX_NS);
                    let m0l = margin((t2 + t3) as u64 * cycle_ps, T0L_MIN_NS, T0L_MAX_NS);
                    let score = min(min(m0h, m1h), min(m1l, m0l));
                    if score >= 0 && best.is_none_or(|(s, _)| score > s) {
                        best = Some((
                            score,
                            PioTiming {
                                clk_sys_hz,
                                divider: divider as u16,
                                t1,
                                t2,
                                t3,
                            },
                        ));
                    }
                    t3 += 1;
                }
                t2 += 1;
            }
            t1 += 1;
        }
        divider += 1;
    }
    best.map(|(_, t)| t).ok_or(TimingError::ClockOutOfRange)
}

const fn min(a: i64, b: i64) -> i64 {
    if a < b { a } else { b }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_inside(t: &PioTiming) {
        let b = t.bit_times();
        assert!((T0H_MIN_NS..=T0H_MAX_NS).contains(&b.t0h_ns), "{t:?} {b:?}");
        assert!((T1H_MIN_NS..=T1H_MAX_NS).contains(&b.t1h_ns), "{t:?} {b:?}");
        assert!((T0L_MIN_NS..=T0L_MAX_NS).contains(&b.t0l_ns), "{t:?} {b:?}");
        assert!((T1L_MIN_NS..=T1L_MAX_NS).contains(&b.t1l_ns), "{t:?} {b:?}");
        for phase in [t.t1, t.t2, t.t3] {
            assert!((1..=16).contains(&phase));
        }
    }

    #[test]
    fn rp2350_clocks_fit_inside_the_limits() {
        for clk in [150_000_000, 125_000_000, 48_000_000] {
            let t = pio_timing(clk).unwrap();
            assert_eq!(t.clk_sys_hz, clk);
            assert_inside(&t);
        }
    }

    #[test]
    fn the_150_mhz_choice_keeps_a_margin_from_every_limit() {
        let t = pio_timing(150_000_000).unwrap();
        let b = t.bit_times();
        assert_eq!((t.divider, t.t1, t.t2, t.t3), (47, 1, 1, 2), "{t:?} {b:?}");
        assert_eq!(b.t0h_ns, 313);
        assert_eq!(b.t1h_ns, 627);
        assert_eq!(b.t0l_ns, 940);
        assert_eq!(b.t1l_ns, 627);
        assert_eq!(t.word_ns(), 30_080);
    }

    #[test]
    fn clocks_that_cannot_meet_the_limits_are_an_error() {
        // One cycle of a 1 MHz clock is 1 us, longer than the whole T0H window.
        assert_eq!(pio_timing(1_000_000), Err(TimingError::ClockOutOfRange));
        assert_eq!(pio_timing(0), Err(TimingError::ClockOutOfRange));
    }

    #[test]
    fn every_clock_from_8_to_400_mhz_is_inside_the_limits() {
        for mhz in 8..=400 {
            assert_inside(&pio_timing(mhz * 1_000_000).unwrap());
        }
    }
}
