//! The LED line on one RP235x PIO state machine (feature `embassy-rp`), on any GPIO.
//!
//! The state machine makes every edge from `clk_sys`; the CPU only pushes 24-bit words into the
//! TX FIFO. No SPI controller, no DMA and no timer interrupt is needed for one LED.
//!
//! # The program
//!
//! Four instructions, one side-set bit (the data pin). `t1`, `t2` and `t3` are the cycle counts
//! of [`PioTiming`]; each instruction's delay is its count minus one. OSR shifts left with
//! autopull at 24 bits, so the first bit out is bit 31 of the word and bits 7..0 are dropped at
//! the refill.
//!
//! ```text
//! .side_set 1
//! .wrap_target
//!         out x, 1        side 0 [t3-1]   ; low for t3; the bit
//!         jmp !x zero     side 1 [t1-1]   ; high for t1
//!         jmp bit_start   side 1 [t2-1]   ; a 1: high t2 more
//! zero:   nop             side 0 [t2-1]   ; a 0: low t2
//! .wrap
//! ```
//!
//! With an empty FIFO the `out` stalls with the pin low, which is the idle state. At 150 MHz
//! `clk_sys` [`pio_timing`] gives divider 47, `t1` = 1, `t2` = 1, `t3` = 2: T0H 313 ns, T1H 627 ns,
//! T0L 940 ns, T1L 627 ns, 1.25 us per bit and 30.08 us per LED.
//!
//! # Reset gap
//!
//! The transport itself keeps the line low for [`RESET_US`] between frames. After the last word
//! is in the FIFO it reads the FIFO level and sleeps until the frame is certainly out (at most
//! one word time per queued word, about 30 us for one LED); the sleep is an `embassy_time`
//! timer, not a busy wait. The reset time is counted from there, and the next `send` waits for
//! it before it pushes a word. A `send` that is dropped half way is covered: the reset time
//! then starts after the longest frame the words could have been.
//!
//! # Chains without DMA
//!
//! The TX FIFO is joined to 8 words and the shift register holds a ninth, so a chain of up to 9
//! LEDs is queued by the CPU in microseconds and then runs with no help: it cannot be disturbed
//! by interrupts or a busy executor. In a longer chain the CPU must keep pushing: a word is
//! consumed every 30 us, and the FIFO holds 8 of them, so the executor may be late by up to
//! about 240 us. A late CPU leaves the line low in the middle of the frame for longer than the
//! datasheet allows (T0L and T1L at most 1 us) and, beyond 280 us, resets the chain half way. For
//! chains that long use DMA instead; this transport does not.
//!
//! # Constraints
//!
//! - Pins 32 and above: the PIO block's GPIOBASE must be 16. `set_config` sets it from the pins of
//!   the state machine; **another state machine in the same PIO block configured with all its
//!   pins below 32 sets GPIOBASE back to 0 and breaks this one.** Keep every state machine of
//!   the block on pins 16 to 47.
//! - The pin gets embassy-rp's default pad settings (4 mA, slow slew, no pulls).
//! - The blocking send spins on the TX FIFO and on the clock; `embassy_time` needs its time
//!   driver (the application's HAL initialisation).

use core::convert::Infallible;

use embassy_rp::Peri;
use embassy_rp::clocks::clk_sys_freq;
use embassy_rp::gpio::Level;
use embassy_rp::pio::program::{Assembler, JmpCondition, OutDestination, SideSet};
use embassy_rp::pio::{
    Common, Config, Direction, FifoJoin, Instance, PioPin, ShiftConfig, ShiftDirection,
    StateMachine,
};
use embassy_time::{Duration, Instant, Timer};

use crate::limits::RESET_US;
use crate::timing::{PioTiming, TimingError, pio_timing};

/// Added to the time a frame takes: the embassy-time tick is 1 us and `Instant::now()` rounds down.
const SLACK_NS: u64 = 2_000;
/// Words that can be in flight beyond the FIFO's: the one in the shift register and one the
/// state machine is about to pull.
const IN_FLIGHT: u64 = 2;

/// The [`Bus`](crate::Bus) on a PIO state machine: async, and blocking
/// ([`crate::blocking::Bus`]). See the module documentation.
pub struct PioTransport<'d, PIO: Instance, const SM: usize> {
    sm: StateMachine<'d, PIO, SM>,
    timing: PioTiming,
    /// The line has been low since before this instant; the next frame may start then.
    free_at: Instant,
}

impl<'d, PIO: Instance, const SM: usize> PioTransport<'d, PIO, SM> {
    /// Loads the program into `common`, takes over `sm` and `pin`, and leaves the line low.
    /// Timing is computed from the current `clk_sys`; fails if no timing fits the limits at that
    /// clock.
    pub fn new(
        common: &mut Common<'d, PIO>,
        sm: StateMachine<'d, PIO, SM>,
        pin: Peri<'d, impl PioPin + 'd>,
    ) -> Result<Self, TimingError> {
        let timing = pio_timing(clk_sys_freq())?;
        Ok(Self::with_timing(common, sm, pin, timing))
    }

    /// As [`Self::new`], with a timing from [`pio_timing`] for the clock the state machine really
    /// runs at.
    pub fn with_timing(
        common: &mut Common<'d, PIO>,
        mut sm: StateMachine<'d, PIO, SM>,
        pin: Peri<'d, impl PioPin + 'd>,
        timing: PioTiming,
    ) -> Self {
        let mut a: Assembler<32> = Assembler::new_with_side_set(SideSet::new(false, 1, false));
        let mut wrap_target = a.label();
        let mut wrap_source = a.label();
        let mut zero = a.label();
        a.bind(&mut wrap_target);
        a.out_with_delay_and_side_set(OutDestination::X, 1, timing.t3 - 1, 0);
        a.jmp_with_delay_and_side_set(JmpCondition::XIsZero, &mut zero, timing.t1 - 1, 1);
        a.jmp_with_delay_and_side_set(JmpCondition::Always, &mut wrap_target, timing.t2 - 1, 1);
        a.bind(&mut zero);
        a.nop_with_delay_and_side_set(timing.t2 - 1, 0);
        a.bind(&mut wrap_source);
        let program = common.load_program(&a.assemble_with_wrap(wrap_source, wrap_target));

        let pin = common.make_pio_pin(pin);
        let mut cfg = Config::default();
        cfg.use_program(&program, &[&pin]);
        cfg.clock_divider = timing.divider.into();
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg.shift_out = ShiftConfig {
            threshold: 24,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        sm.set_enable(false);
        // set_config also writes GPIOBASE; the pin latch and direction come after it.
        sm.set_config(&cfg);
        sm.set_pins(Level::Low, &[&pin]);
        sm.set_pin_dirs(Direction::Out, &[&pin]);
        sm.set_enable(true);

        Self {
            sm,
            timing,
            free_at: Instant::now() + Duration::from_micros(RESET_US as u64),
        }
    }

    /// The timing the state machine runs with.
    pub fn timing(&self) -> PioTiming {
        self.timing
    }

    /// Time to send `words` words, with everything that can be in flight, plus the slack.
    fn frame_bound(&self, words: u64) -> Duration {
        Duration::from_nanos(words * self.timing.word_ns() as u64 + SLACK_NS)
    }

    /// Called with the last word queued: when the frame is certainly out and, after the reset
    /// time, the next may start.
    fn frame_end(&mut self) -> Instant {
        let queued = self.sm.tx().level() as u64 + 1;
        Instant::now() + self.frame_bound(queued)
    }
}

impl<PIO: Instance, const SM: usize> crate::Bus for PioTransport<'_, PIO, SM> {
    type Error = Infallible;

    async fn send(&mut self, words: &[u32]) -> Result<(), Infallible> {
        if words.is_empty() {
            return Ok(());
        }
        Timer::at(self.free_at).await;
        // Until the end is known, assume the worst, so a dropped future leaves a safe gap.
        let worst = Instant::now() + self.frame_bound(words.len() as u64 + IN_FLIGHT + 8);
        self.free_at = worst + Duration::from_micros(RESET_US as u64);
        for &word in words {
            self.sm.tx().wait_push(word).await;
        }
        let end = self.frame_end();
        Timer::at(end).await;
        self.free_at = end + Duration::from_micros(RESET_US as u64);
        Ok(())
    }
}

impl<PIO: Instance, const SM: usize> crate::blocking::Bus for PioTransport<'_, PIO, SM> {
    type Error = Infallible;

    fn send(&mut self, words: &[u32]) -> Result<(), Infallible> {
        if words.is_empty() {
            return Ok(());
        }
        spin_until(self.free_at);
        let worst = Instant::now() + self.frame_bound(words.len() as u64 + IN_FLIGHT + 8);
        self.free_at = worst + Duration::from_micros(RESET_US as u64);
        for &word in words {
            while !self.sm.tx().try_push(word) {}
        }
        let end = self.frame_end();
        spin_until(end);
        self.free_at = end + Duration::from_micros(RESET_US as u64);
        Ok(())
    }
}

fn spin_until(t: Instant) {
    while Instant::now() < t {}
}
