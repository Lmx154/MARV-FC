//! Blocking front-end: the async driver run to completion over an adapter whose futures never pend.

use core::future::Future;
use core::pin::pin;
use core::task::{Context, Poll, Waker};

use crate::{Error, Rgb};

/// Blocking data line of the LED chain: [`crate::Bus`] without `async`. The rules in
/// [`crate::bus`] apply unchanged.
pub trait Bus {
    /// What the hardware reports when a transfer fails.
    type Error;

    /// Sends `words` as one frame.
    fn send(&mut self, words: &[u32]) -> Result<(), Self::Error>;
}

/// Blocking bus presented as an async one.
struct Adapter<B>(B);

impl<B: Bus> crate::Bus for Adapter<B> {
    type Error = B::Error;

    async fn send(&mut self, words: &[u32]) -> Result<(), Self::Error> {
        self.0.send(words)
    }
}

fn block_on<F: Future>(future: F) -> F::Output {
    let mut future = pin!(future);
    let mut cx = Context::from_waker(Waker::noop());
    loop {
        if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
            return value;
        }
    }
}

/// WS2812C-2020 LEDs on a blocking [`Bus`]. Same methods as [`crate::Ws2812c`], without `async`.
pub struct Ws2812c<B> {
    inner: crate::Ws2812c<Adapter<B>>,
}

impl<B: Bus> Ws2812c<B> {
    /// Wraps the transport. Nothing is sent until the first write; brightness is 255.
    pub const fn new(bus: B) -> Self {
        Self {
            inner: crate::Ws2812c::new(Adapter(bus)),
        }
    }

    /// Returns the transport.
    pub fn release(self) -> B {
        self.inner.release().0
    }

    /// See [`crate::Ws2812c::set_brightness`].
    pub fn set_brightness(&mut self, brightness: u8) {
        self.inner.set_brightness(brightness);
    }

    /// See [`crate::Ws2812c::set`].
    pub fn set(&mut self, color: Rgb) -> Result<(), Error<B::Error>> {
        block_on(self.inner.set(color))
    }

    /// See [`crate::Ws2812c::off`].
    pub fn off(&mut self) -> Result<(), Error<B::Error>> {
        block_on(self.inner.off())
    }

    /// See [`crate::Ws2812c::write`].
    pub fn write(&mut self, colors: &[Rgb]) -> Result<(), Error<B::Error>> {
        block_on(self.inner.write(colors))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MAX_CHAIN;
    use std::vec::Vec;

    #[derive(Default)]
    struct FakeBus {
        frames: Vec<Vec<u32>>,
        fail: bool,
    }

    impl Bus for FakeBus {
        type Error = u8;

        fn send(&mut self, words: &[u32]) -> Result<(), u8> {
            if self.fail {
                return Err(9);
            }
            self.frames.push(words.to_vec());
            Ok(())
        }
    }

    #[test]
    fn blocking_driver_sends_the_same_frames_as_the_async_one() {
        let mut led = Ws2812c::new(FakeBus::default());
        led.set(Rgb::new(0x3C, 0xA5, 0x0F)).unwrap();
        led.set_brightness(32);
        led.set(Rgb::WHITE).unwrap();
        led.write(&[Rgb::RED, Rgb::BLUE]).unwrap();
        led.off().unwrap();
        assert_eq!(
            led.release().frames,
            [
                &[0xA53C_0F00][..],
                &[0x2020_2000],
                &[0x0020_0000, 0x0000_2000],
                &[0]
            ]
        );
    }

    #[test]
    fn blocking_errors_are_passed_up() {
        let mut led = Ws2812c::new(FakeBus {
            fail: true,
            ..Default::default()
        });
        assert_eq!(led.set(Rgb::RED), Err(Error::Bus(9)));
        assert_eq!(
            led.write(&[Rgb::RED; MAX_CHAIN + 1]),
            Err(Error::ChainTooLong)
        );
    }
}
