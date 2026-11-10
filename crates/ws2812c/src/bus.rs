//! The seam between the LED driver and the hardware that moves the bits.
//!
//! A [`Bus`] knows nothing about colours. It sends 24-bit words (see [`crate::Rgb::wire`]) onto
//! the one data line and keeps the line's timing: that is the whole job of a transport, and the
//! driver above it is only colour handling. [`crate::spi::SpiTransport`] and
//! `pio::PioTransport` (feature `embassy-rp`) are the two transports of this crate; a test can use any type
//! that implements the trait.
//!
//! # Rules every implementation follows
//!
//! - A call to `send` is one frame: the words go out back to back, one LED taking the first word
//!   of the frame, the next LED the second, and so on. Between the data bits of a frame the line
//!   timing is inside the limits of [`crate::limits`], with no gap. A transport that has to split
//!   a long frame into several hardware transfers says so in its own documentation.
//! - Each word is sent from bit 31 down to bit 8, 24 bits. Bits 7..0 are ignored.
//! - The line is low whenever no frame is going out, and it stays low for at least
//!   [`crate::limits::RESET_MIN_US`] between the last bit of one frame and the first bit of the
//!   next, so the LEDs latch the data. The transport guarantees this itself; the caller never
//!   waits. The gap before the very first frame is the transport's own start-up.
//! - `send` of an empty slice sends nothing and still returns `Ok`.
//! - When `send` returns, every word has been handed to the hardware. Whether the last bits have
//!   left the pin is the transport's to say: the PIO transport waits for them, the SPI transport
//!   leaves that to `flush` of the SPI bus. Dropping the future before it completes may cut a
//!   frame short; the following frame is still separated by the reset time.
#![allow(async_fn_in_trait)]

/// Async data line of the LED chain.
pub trait Bus {
    /// What the hardware reports when a transfer fails.
    type Error;

    /// Sends `words` as one frame. See the module documentation for the rules.
    async fn send(&mut self, words: &[u32]) -> Result<(), Self::Error>;
}
