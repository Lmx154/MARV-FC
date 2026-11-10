//! Driver for the Worldsemi WS2812C-2020 RGB LED (one wire, 24 bits per LED, GRB order).
//!
//! Built on the `embedded-hal` 1.0 traits only: it works with any executor or none. The crate root
//! is the async driver ([`Ws2812c`], `embedded-hal-async`); [`blocking`] is the same driver on the
//! blocking traits. The LED knowledge (colours, the wire word, the datasheet timing limits, the
//! SPI encoding and clock window, the PIO timing calculation) is pure code that runs on the host.
//! The driver itself does colour handling only; patterns, blinking and fading belong to the
//! application.
//!
//! # Transports
//!
//! The driver sends through a [`Bus`], which keeps the line timing and the reset gap between
//! frames:
//!
//! - [`spi::SpiTransport`] works with any HAL. The data line is the MOSI pin of an SPI bus run at
//!   3.0 to 3.45 MHz ([`spi::SPI_MIN_HZ`], [`spi::SPI_MAX_HZ`]); the clock pin is not used by the
//!   LED. The SPI controller must send bytes with no gap between them.
//! - `pio::PioTransport` (feature `embassy-rp`) runs one PIO state machine on any GPIO and needs
//!   no SPI controller and no DMA for a single LED.
//!
//! # Brightness
//!
//! An LED at full level is very bright and draws about 15 mA. [`Ws2812c::set_brightness`] scales
//! every colour of the following writes; the default is 255 (no scaling).
//!
//! Async, SPI:
//! ```ignore
//! let mut spi_cfg = embassy_rp::spi::Config::default();
//! spi_cfg.frequency = ws2812c::spi::SPI_NOMINAL_HZ;
//! let spi = Spi::new_txonly(p.SPI1, p.PIN_26, p.PIN_43, p.DMA_CH0, Irqs, spi_cfg);
//! let mut led = ws2812c::Ws2812c::new(ws2812c::spi::SpiTransport::new(spi));
//! led.set_brightness(32);
//! led.set(ws2812c::Rgb::GREEN).await?;
//! ```
//!
//! Async, PIO (feature `embassy-rp`):
//! ```ignore
//! let Pio { mut common, sm0, .. } = Pio::new(p.PIO1, Irqs);
//! let bus = ws2812c::pio::PioTransport::new(&mut common, sm0, p.PIN_43)?;
//! let mut led = ws2812c::Ws2812c::new(bus);
//! led.write(&[ws2812c::Rgb::RED, ws2812c::Rgb::BLUE]).await?;
//! led.off().await?;
//! ```
//!
//! Blocking:
//! ```ignore
//! let mut led = ws2812c::blocking::Ws2812c::new(ws2812c::spi::SpiTransport::new(spi_bus));
//! led.set(ws2812c::Rgb::WHITE)?;
//! ```
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod blocking;
pub mod bus;
mod color;
pub mod limits;
#[cfg(feature = "embassy-rp")]
pub mod pio;
pub mod spi;
pub mod timing;

pub use bus::Bus;
pub use color::Rgb;

/// Longest chain [`Ws2812c::write`] sends in one frame.
pub const MAX_CHAIN: usize = 64;

/// Driver errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// The transport reported an error.
    Bus(E),
    /// `write` was given more than [`MAX_CHAIN`] colours. Nothing was sent.
    ChainTooLong,
}

/// WS2812C-2020 LEDs on an async [`Bus`]: one LED, or a short chain of them.
pub struct Ws2812c<B> {
    bus: B,
    brightness: u8,
}

impl<B: Bus> Ws2812c<B> {
    /// Wraps the transport. Nothing is sent until the first write; brightness is 255.
    pub const fn new(bus: B) -> Self {
        Self {
            bus,
            brightness: 255,
        }
    }

    /// Returns the transport.
    pub fn release(self) -> B {
        self.bus
    }

    /// Sets the brightness applied to every later write, 0 (all off) to 255 (unchanged colours).
    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness;
    }

    /// Shows `color` on the first (or only) LED.
    pub async fn set(&mut self, color: Rgb) -> Result<(), Error<B::Error>> {
        self.write(&[color]).await
    }

    /// Turns the first LED off.
    pub async fn off(&mut self) -> Result<(), Error<B::Error>> {
        self.set(Rgb::OFF).await
    }

    /// Sends one frame for a chain: `colors[0]` to the LED on the driver's pin, `colors[1]` to
    /// the next, and so on. An empty slice sends nothing; more than [`MAX_CHAIN`] colours is
    /// [`Error::ChainTooLong`].
    pub async fn write(&mut self, colors: &[Rgb]) -> Result<(), Error<B::Error>> {
        if colors.len() > MAX_CHAIN {
            return Err(Error::ChainTooLong);
        }
        if colors.is_empty() {
            return Ok(());
        }
        let mut words = [0u32; MAX_CHAIN];
        for (word, color) in words.iter_mut().zip(colors) {
            *word = color.scaled(self.brightness).wire();
        }
        self.bus
            .send(&words[..colors.len()])
            .await
            .map_err(Error::Bus)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, Poll, Waker};
    use std::vec::Vec;

    /// Records every frame; fails when told to.
    #[derive(Default)]
    pub(crate) struct FakeBus {
        pub frames: Vec<Vec<u32>>,
        pub fail: bool,
    }

    impl Bus for FakeBus {
        type Error = u8;

        async fn send(&mut self, words: &[u32]) -> Result<(), u8> {
            if self.fail {
                return Err(7);
            }
            self.frames.push(words.to_vec());
            Ok(())
        }
    }

    pub(crate) fn run<F: Future>(future: F) -> F::Output {
        let mut future = pin!(future);
        let mut cx = Context::from_waker(Waker::noop());
        match future.as_mut().poll(&mut cx) {
            Poll::Ready(v) => v,
            Poll::Pending => panic!("future pended"),
        }
    }

    #[test]
    fn set_sends_one_frame_with_one_wire_word() {
        let mut led = Ws2812c::new(FakeBus::default());
        run(led.set(Rgb::new(0x3C, 0xA5, 0x0F))).unwrap();
        assert_eq!(led.release().frames, [[0xA53C_0F00]]);
    }

    #[test]
    fn off_sends_a_zero_word() {
        let mut led = Ws2812c::new(FakeBus::default());
        run(led.off()).unwrap();
        assert_eq!(led.release().frames, [[0]]);
    }

    #[test]
    fn brightness_applies_to_later_writes_only() {
        let mut led = Ws2812c::new(FakeBus::default());
        run(led.set(Rgb::WHITE)).unwrap();
        led.set_brightness(32);
        run(led.set(Rgb::WHITE)).unwrap();
        led.set_brightness(0);
        run(led.set(Rgb::WHITE)).unwrap();
        led.set_brightness(255);
        run(led.set(Rgb::GREEN)).unwrap();
        assert_eq!(
            led.release().frames,
            [[0xFFFF_FF00], [0x2020_2000], [0], [0xFF00_0000]]
        );
    }

    #[test]
    fn a_chain_is_one_frame_in_order() {
        let mut led = Ws2812c::new(FakeBus::default());
        led.set_brightness(128);
        run(led.write(&[Rgb::RED, Rgb::GREEN, Rgb::BLUE])).unwrap();
        assert_eq!(
            led.release().frames,
            [[0x0080_0000, 0x8000_0000, 0x0000_8000]]
        );
    }

    #[test]
    fn an_empty_chain_sends_nothing_and_the_longest_chain_fits() {
        let mut led = Ws2812c::new(FakeBus::default());
        run(led.write(&[])).unwrap();
        run(led.write(&[Rgb::BLUE; MAX_CHAIN])).unwrap();
        let frames = led.release().frames;
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].len(), MAX_CHAIN);
    }

    #[test]
    fn a_chain_that_is_too_long_sends_nothing() {
        let mut led = Ws2812c::new(FakeBus::default());
        let r = run(led.write(&[Rgb::RED; MAX_CHAIN + 1]));
        assert_eq!(r, Err(Error::ChainTooLong));
        assert!(led.release().frames.is_empty());
    }

    #[test]
    fn a_bus_error_is_passed_up() {
        let mut led = Ws2812c::new(FakeBus {
            fail: true,
            ..Default::default()
        });
        assert_eq!(run(led.set(Rgb::RED)), Err(Error::Bus(7)));
    }
}
