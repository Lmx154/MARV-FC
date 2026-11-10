//! The LED line on a plain SPI bus: no timer, no HAL-specific code.
//!
//! # Encoding
//!
//! Each data bit becomes four SPI bits, most significant first: a 0 is `1000`, a 1 is `1100`. The
//! SPI clock `f` sets the four times of a bit:
//!
//! | Code | High | Low |
//! |---|---|---|
//! | 0 (`1000`) | 1 / f | 3 / f |
//! | 1 (`1100`) | 2 / f | 2 / f |
//!
//! Against the limits of [`crate::limits`] (T0H 220-380 ns; T1H, T0L, T1L 580-1000 ns):
//!
//! - T0H = 1 / f: 2.63 MHz <= f <= 4.55 MHz
//! - T1H = T1L = 2 / f: 2.00 MHz <= f <= 3.448 MHz (the upper end from the 580 ns minimum)
//! - T0L = 3 / f: 3.00 MHz <= f <= 5.17 MHz (the lower end from the 1000 ns maximum)
//!
//! So the clock must be 3.0 to 3.448 MHz: [`SPI_MIN_HZ`] to [`SPI_MAX_HZ`], computed from the
//! limits by [`clock_window`]. [`SPI_NOMINAL_HZ`] is the middle where T1H and T0L have equal
//! margin. A HAL that rounds the divider down gives a lower rate than asked: check the rate it
//! really makes, for example 3.125 MHz from a 150 MHz peripheral clock (T0H 320 ns, T1H 640 ns,
//! T0L 960 ns).
//!
//! # What the SPI bus must do
//!
//! - **Bytes with no gap between them.** A gap shows as extra time on the line (the line is
//!   MOSI, so it keeps the level of the last bit sent, which is always 0: both codes end in
//!   0). A stream fed by DMA and an SPI controller that shifts back to back is gapless; a
//!   blocking loop that waits for each byte to come back is not.
//! - **MOSI idles low**, and the data is 8-bit frames, most significant bit first. Clock polarity
//!   and phase do not matter to the LED, which has no clock input; the mode that gives a gapless
//!   stream is the controller's business.
//! - **Chip select is not used.** The LED has no select line; leave it to the controller to
//!   ignore it or to toggle a pin nobody listens to.
//!
//! # Reset gap
//!
//! [`RESET_BYTES`] zero bytes follow the last data byte of every frame: at the highest allowed
//! clock they last [`crate::limits::RESET_US`], more than the 280 us the LED needs to latch. The
//! gap is part of the SPI stream, so no timer is needed. A frame that was cut short (the future
//! dropped, or an error from the bus) never sent those bytes, so the next frame starts with them.
//!
//! # Long chains
//!
//! One frame of up to [`CHUNK_WORDS`] LEDs is encoded in a single stack buffer and written with
//! one SPI call. A longer frame is written in several calls, every one of them ending after a
//! whole LED; the line is low for the time the HAL takes between calls (microseconds with DMA),
//! longer than the datasheet's 1 us low time but far below the 280 us reset.

use crate::limits::*;

/// SPI bits per data bit.
pub const SPI_BITS_PER_BIT: u32 = 4;
/// SPI code of a 0 bit, most significant of the four bits first.
pub const CODE_0: u8 = 0b1000;
/// SPI code of a 1 bit, most significant of the four bits first.
pub const CODE_1: u8 = 0b1100;
/// SPI bytes per LED: 24 data bits, four SPI bits each.
pub const BYTES_PER_WORD: usize = 12;

/// The SPI clock range, in hertz, that puts all four times of both codes inside the limits.
///
/// A code with `high` SPI bits high and `low` low has high time `high / f` and low time `low / f`;
/// each time gives a lower and an upper bound on `f`. Rounded inwards to whole hertz.
pub const fn clock_window() -> (u32, u32) {
    // (SPI bits, minimum ns, maximum ns) of T0H, T1H, T0L, T1L.
    const TIMES: [(u64, u32, u32); 4] = [
        (1, T0H_MIN_NS, T0H_MAX_NS),
        (2, T1H_MIN_NS, T1H_MAX_NS),
        (3, T0L_MIN_NS, T0L_MAX_NS),
        (2, T1L_MIN_NS, T1L_MAX_NS),
    ];
    let mut lo = 0u64;
    let mut hi = u64::MAX;
    let mut i = 0;
    while i < TIMES.len() {
        let (bits, min_ns, max_ns) = TIMES[i];
        // bits / f <= max  =>  f >= bits / max (rounded up);  bits / f >= min  =>  f <= bits / min.
        let from = (bits * 1_000_000_000).div_ceil(max_ns as u64);
        let to = bits * 1_000_000_000 / min_ns as u64;
        if from > lo {
            lo = from;
        }
        if to < hi {
            hi = to;
        }
        i += 1;
    }
    (lo as u32, hi as u32)
}

/// Lowest SPI clock the encoding allows, in hertz.
pub const SPI_MIN_HZ: u32 = clock_window().0;
/// Highest SPI clock the encoding allows, in hertz.
pub const SPI_MAX_HZ: u32 = clock_window().1;
/// The clock to ask the HAL for, in hertz. A HAL that rounds down yields a rate inside the window.
pub const SPI_NOMINAL_HZ: u32 = 3_200_000;

/// Zero bytes after each frame: [`crate::limits::RESET_US`] at [`SPI_MAX_HZ`], rounded up.
pub const RESET_BYTES: usize =
    (RESET_US as u64 * SPI_MAX_HZ as u64).div_ceil(8 * 1_000_000) as usize;

/// LEDs per SPI write; see the module documentation.
pub const CHUNK_WORDS: usize = 16;

const BUFFER: usize = CHUNK_WORDS * BYTES_PER_WORD + RESET_BYTES;

/// SPI bytes of the 24 data bits of `word` (bits 31..8; see [`crate::Rgb::wire`]).
pub const fn encode_word(word: u32) -> [u8; BYTES_PER_WORD] {
    let mut out = [0u8; BYTES_PER_WORD];
    let mut i = 0;
    while i < BYTES_PER_WORD {
        let first = (word >> (31 - 2 * i)) & 1;
        let second = (word >> (30 - 2 * i)) & 1;
        out[i] = code(first) << 4 | code(second);
        i += 1;
    }
    out
}

const fn code(bit: u32) -> u8 {
    if bit == 1 { CODE_1 } else { CODE_0 }
}

/// Encodes `words` into `out`, 12 bytes each, and returns the number of bytes written.
///
/// # Panics
///
/// If `out` is shorter than `12 * words.len()`.
pub fn encode(words: &[u32], out: &mut [u8]) -> usize {
    assert!(out.len() >= words.len() * BYTES_PER_WORD);
    let (chunks, _) = out.as_chunks_mut::<BYTES_PER_WORD>();
    for (word, bytes) in words.iter().zip(chunks) {
        *bytes = encode_word(*word);
    }
    words.len() * BYTES_PER_WORD
}

/// Encodes one chunk of a frame into `buf`; the last chunk of the frame also gets the reset bytes.
/// Returns the number of bytes to write.
fn fill(chunk: &[u32], last: bool, buf: &mut [u8; BUFFER]) -> usize {
    let mut n = encode(chunk, buf);
    if last {
        buf[n..n + RESET_BYTES].fill(0);
        n += RESET_BYTES;
    }
    n
}

/// The [`Bus`](crate::Bus) on an SPI bus: async on `embedded_hal_async::spi::SpiBus`, blocking
/// ([`crate::blocking::Bus`]) on `embedded_hal::spi::SpiBus`.
///
/// Configure the bus to a clock between [`SPI_MIN_HZ`] and [`SPI_MAX_HZ`] before wrapping it. See
/// the module documentation for what the controller has to do.
pub struct SpiTransport<S> {
    spi: S,
    /// A frame was started and did not reach its reset bytes.
    cut_short: bool,
}

impl<S> SpiTransport<S> {
    /// Wraps the SPI bus. Nothing is sent until the first frame.
    pub const fn new(spi: S) -> Self {
        Self {
            spi,
            cut_short: false,
        }
    }

    /// Returns the SPI bus.
    pub fn release(self) -> S {
        self.spi
    }
}

impl<S: embedded_hal_async::spi::SpiBus<u8>> crate::Bus for SpiTransport<S> {
    type Error = S::Error;

    async fn send(&mut self, words: &[u32]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        if self.cut_short {
            self.spi.write(&[0u8; RESET_BYTES]).await?;
        }
        self.cut_short = true;
        let mut buf = [0u8; BUFFER];
        let chunks = words.chunks(CHUNK_WORDS);
        let count = chunks.len();
        for (i, chunk) in chunks.enumerate() {
            let n = fill(chunk, i + 1 == count, &mut buf);
            self.spi.write(&buf[..n]).await?;
        }
        self.spi.flush().await?;
        self.cut_short = false;
        Ok(())
    }
}

impl<S: embedded_hal::spi::SpiBus<u8>> crate::blocking::Bus for SpiTransport<S> {
    type Error = S::Error;

    fn send(&mut self, words: &[u32]) -> Result<(), Self::Error> {
        if words.is_empty() {
            return Ok(());
        }
        if self.cut_short {
            self.spi.write(&[0u8; RESET_BYTES])?;
        }
        self.cut_short = true;
        let mut buf = [0u8; BUFFER];
        let chunks = words.chunks(CHUNK_WORDS);
        let count = chunks.len();
        for (i, chunk) in chunks.enumerate() {
            let n = fill(chunk, i + 1 == count, &mut buf);
            self.spi.write(&buf[..n])?;
        }
        self.spi.flush()?;
        self.cut_short = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tests::run;
    use crate::{Rgb, Ws2812c, blocking};
    use core::convert::Infallible;
    use std::vec::Vec;

    #[test]
    fn clock_window_comes_from_the_limits() {
        assert_eq!(clock_window(), (3_000_000, 3_448_275));
        assert_eq!((SPI_MIN_HZ, SPI_MAX_HZ), (3_000_000, 3_448_275));
        assert!((SPI_MIN_HZ..=SPI_MAX_HZ).contains(&SPI_NOMINAL_HZ));
        // 150 MHz / 48, the rate embassy-rp makes for a request of SPI_NOMINAL_HZ.
        assert!((SPI_MIN_HZ..=SPI_MAX_HZ).contains(&3_125_000));
    }

    #[test]
    fn bit_codes_encode_in_four_spi_bits() {
        assert_eq!(encode_word(0), [0x88; 12]);
        assert_eq!(encode_word(0xFFFF_FF00), [0xCC; 12]);
        // The low byte of the word is not sent.
        assert_eq!(encode_word(0x0000_00FF), [0x88; 12]);
        assert_eq!(
            encode_word(0xA500_0000),
            [
                0xC8, 0xC8, 0x8C, 0x8C, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88
            ]
        );
    }

    #[test]
    fn a_full_pixel_encodes_green_then_red_then_blue() {
        // G = 0xA5 (1010 0101), R = 0x3C (0011 1100), B = 0x0F (0000 1111).
        let word = Rgb::new(0x3C, 0xA5, 0x0F).wire();
        assert_eq!(
            encode_word(word),
            [
                0xC8, 0xC8, 0x8C, 0x8C, // G
                0x88, 0xCC, 0xCC, 0x88, // R
                0x88, 0x88, 0xCC, 0xCC, // B
            ]
        );
    }

    #[test]
    fn frame_length_is_twelve_bytes_per_led_plus_the_reset() {
        let words = [1u32 << 8; 3];
        let mut out = [0xEE; 64];
        assert_eq!(encode(&words, &mut out), 36);
        let mut buf = [0xEE; BUFFER];
        assert_eq!(fill(&words, true, &mut buf), 36 + RESET_BYTES);
        assert!(buf[36..36 + RESET_BYTES].iter().all(|b| *b == 0));
        assert_eq!(fill(&words, false, &mut buf), 36);
    }

    #[test]
    fn reset_bytes_last_longer_than_the_limit_at_the_highest_clock() {
        let ns_at_max = RESET_BYTES as u64 * 8 * 1_000_000_000 / SPI_MAX_HZ as u64;
        assert!(ns_at_max >= RESET_MIN_US as u64 * 1_000, "{ns_at_max} ns");
        assert!(ns_at_max >= RESET_US as u64 * 1_000, "{ns_at_max} ns");
        assert!(
            (RESET_BYTES - 1) as u64 * 8 * 1_000_000_000 / (SPI_MAX_HZ as u64)
                < RESET_US as u64 * 1_000
        );
    }

    /// Run lengths of the SPI bit stream as (level, SPI bits).
    fn runs(bytes: &[u8]) -> Vec<(bool, u32)> {
        let mut out: Vec<(bool, u32)> = Vec::new();
        for byte in bytes {
            for i in (0..8).rev() {
                let level = byte >> i & 1 == 1;
                match out.last_mut() {
                    Some((l, n)) if *l == level => *n += 1,
                    _ => out.push((level, 1)),
                }
            }
        }
        out
    }

    fn within(bits: u32, hz: u64, min_ns: u32, max_ns: u32) -> bool {
        let ps = bits as u64 * 1_000_000_000_000 / hz;
        ps >= min_ns as u64 * 1_000 && ps <= max_ns as u64 * 1_000
    }

    #[test]
    fn every_bit_time_is_inside_the_limits_across_the_clock_window() {
        let word = Rgb::new(0x3C, 0xA5, 0x0F).wire();
        let mut bytes = encode_word(word).to_vec();
        bytes.extend_from_slice(&encode_word(0));
        let runs = runs(&bytes);
        // 48 bits: each data bit is one high run followed by one low run.
        assert_eq!(runs.len(), 96);
        for hz in [SPI_MIN_HZ as u64, SPI_NOMINAL_HZ as u64, SPI_MAX_HZ as u64] {
            for (i, pair) in runs.chunks(2).enumerate() {
                let (high, low) = (pair[0], pair[1]);
                assert!(high.0 && !low.0);
                let bit = if i < 24 { word >> (31 - i) & 1 } else { 0 };
                if bit == 1 {
                    assert!(within(high.1, hz, T1H_MIN_NS, T1H_MAX_NS), "T1H at {hz}");
                    assert!(within(low.1, hz, T1L_MIN_NS, T1L_MAX_NS), "T1L at {hz}");
                } else {
                    assert!(within(high.1, hz, T0H_MIN_NS, T0H_MAX_NS), "T0H at {hz}");
                    assert!(within(low.1, hz, T0L_MIN_NS, T0L_MAX_NS), "T0L at {hz}");
                }
            }
        }
        // A kilohertz outside the window on either side breaks a limit.
        let (hi, lo) = (SPI_MAX_HZ as u64 + 1_000, SPI_MIN_HZ as u64 - 1_000);
        assert!(!within(2, hi, T1H_MIN_NS, T1H_MAX_NS));
        assert!(!within(3, lo, T0L_MIN_NS, T0L_MAX_NS));
    }

    #[derive(Default)]
    struct FakeSpi {
        writes: Vec<Vec<u8>>,
        flushes: usize,
        /// An async write records its bytes and then never completes.
        stall: bool,
    }

    impl embedded_hal::spi::ErrorType for FakeSpi {
        type Error = Infallible;
    }

    impl embedded_hal::spi::SpiBus<u8> for FakeSpi {
        fn read(&mut self, _: &mut [u8]) -> Result<(), Infallible> {
            unreachable!()
        }
        fn write(&mut self, words: &[u8]) -> Result<(), Infallible> {
            self.writes.push(words.to_vec());
            Ok(())
        }
        fn transfer(&mut self, _: &mut [u8], _: &[u8]) -> Result<(), Infallible> {
            unreachable!()
        }
        fn transfer_in_place(&mut self, _: &mut [u8]) -> Result<(), Infallible> {
            unreachable!()
        }
        fn flush(&mut self) -> Result<(), Infallible> {
            self.flushes += 1;
            Ok(())
        }
    }

    impl embedded_hal_async::spi::SpiBus<u8> for FakeSpi {
        async fn read(&mut self, _: &mut [u8]) -> Result<(), Infallible> {
            unreachable!()
        }
        async fn write(&mut self, words: &[u8]) -> Result<(), Infallible> {
            embedded_hal::spi::SpiBus::write(self, words)?;
            if self.stall {
                core::future::pending::<()>().await;
            }
            Ok(())
        }
        async fn transfer(&mut self, _: &mut [u8], _: &[u8]) -> Result<(), Infallible> {
            unreachable!()
        }
        async fn transfer_in_place(&mut self, _: &mut [u8]) -> Result<(), Infallible> {
            unreachable!()
        }
        async fn flush(&mut self) -> Result<(), Infallible> {
            embedded_hal::spi::SpiBus::flush(self)
        }
    }

    fn expected(words: &[u32]) -> Vec<u8> {
        let mut v = Vec::new();
        for w in words {
            v.extend_from_slice(&encode_word(*w));
        }
        v.extend(core::iter::repeat_n(0, RESET_BYTES));
        v
    }

    #[test]
    fn async_transport_writes_one_buffer_per_frame_and_flushes() {
        let mut led = Ws2812c::new(SpiTransport::new(FakeSpi::default()));
        run(led.set(Rgb::new(0x3C, 0xA5, 0x0F))).unwrap();
        let spi = led.release().release();
        assert_eq!(spi.writes, [expected(&[0xA53C_0F00])]);
        assert_eq!(spi.flushes, 1);
    }

    #[test]
    fn blocking_transport_writes_the_same_bytes() {
        let mut led = blocking::Ws2812c::new(SpiTransport::new(FakeSpi::default()));
        led.set(Rgb::new(0x3C, 0xA5, 0x0F)).unwrap();
        led.off().unwrap();
        let spi = led.release().release();
        assert_eq!(spi.writes, [expected(&[0xA53C_0F00]), expected(&[0])]);
        assert_eq!(spi.flushes, 2);
    }

    #[test]
    fn the_frame_after_a_dropped_send_starts_with_the_reset_bytes() {
        use core::future::Future;
        use core::pin::pin;
        use core::task::{Context, Waker};

        let mut bus = SpiTransport::new(FakeSpi {
            stall: true,
            ..FakeSpi::default()
        });
        {
            let mut send = pin!(crate::Bus::send(&mut bus, &[0xA53C_0F00]));
            let mut cx = Context::from_waker(Waker::noop());
            assert!(send.as_mut().poll(&mut cx).is_pending());
            // Dropped here, in the middle of its SPI write.
        }
        bus.spi.stall = false;
        run(crate::Bus::send(&mut bus, &[0])).unwrap();
        run(crate::Bus::send(&mut bus, &[0])).unwrap();
        let spi = bus.release();
        // The write that was cut short; the reset bytes; the next frame; a frame with no extra gap.
        assert_eq!(spi.writes.len(), 4);
        assert_eq!(spi.writes[1], [0u8; RESET_BYTES]);
        assert_eq!(spi.writes[2], expected(&[0]));
        assert_eq!(spi.writes[3], expected(&[0]));
    }

    #[test]
    fn a_long_chain_is_split_after_whole_leds_with_the_reset_on_the_last_write_only() {
        let colors: Vec<Rgb> = (0..20).map(|i| Rgb::new(i, 2 * i, 3 * i)).collect();
        let words: Vec<u32> = colors.iter().map(|c| c.wire()).collect();
        let mut led = Ws2812c::new(SpiTransport::new(FakeSpi::default()));
        run(led.write(&colors)).unwrap();
        let spi = led.release().release();
        assert_eq!(spi.writes.len(), 2);
        assert_eq!(spi.writes[0].len(), CHUNK_WORDS * BYTES_PER_WORD);
        assert_eq!(spi.writes[1].len(), 4 * BYTES_PER_WORD + RESET_BYTES);
        let all: Vec<u8> = spi.writes.concat();
        assert_eq!(all, expected(&words));
        assert_eq!(spi.flushes, 1);
    }
}
