//! The flight core's yield rule: no flight task holds the executor longer than one bus transaction
//! or one short busy wait. [`YieldSpi`] wraps the SPI device handed to an IMU driver and yields to
//! the executor after every transaction, so another flight task whose data is ready runs in
//! between; [`measured`] keeps a task's longest poll (the longest stretch it ran without yielding)
//! and its poll count in a [`Stretch`]. No timer, no lock, no log.

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_futures::yield_now;
use embedded_hal_async::spi::{ErrorType, Operation, SpiDevice};

/// An SPI device that yields once after each transaction.
pub struct YieldSpi<S>(pub S);

impl<S: ErrorType> ErrorType for YieldSpi<S> {
    type Error = S::Error;
}

impl<S: SpiDevice> SpiDevice for YieldSpi<S> {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        let result = self.0.transaction(operations).await;
        yield_now().await;
        result
    }
}

/// A flight task's longest poll (us) and its poll count (every poll but the last ends in a yield).
/// Written by the task, read by its adapter.
pub struct Stretch {
    max_us: AtomicU32,
    polls: AtomicU32,
}

impl Stretch {
    pub const fn new() -> Self {
        Self {
            max_us: AtomicU32::new(0),
            polls: AtomicU32::new(0),
        }
    }

    /// The longest poll (us) and the poll count.
    pub fn get(&self) -> (u32, u32) {
        (
            self.max_us.load(Ordering::Relaxed),
            self.polls.load(Ordering::Relaxed),
        )
    }
}

/// Runs `fut`, taking the clock before and after each poll into `stretch`.
pub async fn measured<F: Future>(stretch: &Stretch, fut: F) -> F::Output {
    let mut fut = pin!(fut);
    poll_fn(|cx| {
        let begun = crate::now_us();
        let poll = fut.as_mut().poll(cx);
        let took = u32::try_from(crate::now_us().saturating_sub(begun)).unwrap_or(u32::MAX);
        if took > stretch.max_us.load(Ordering::Relaxed) {
            stretch.max_us.store(took, Ordering::Relaxed);
        }
        stretch.polls.store(
            stretch.polls.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
        poll
    })
    .await
}
