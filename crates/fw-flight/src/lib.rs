//! Flight-core (core 1) tasks of the MARV V2 firmware.
//! Today: the liveness answer [`liveness`] and the IMU device loop [`imu`] (IMU1 runs it), and
//! [`tagged`], which names the task that ran last for the watchdog (`RUNNING1`).
//! Rule: no dependency on `embassy-time`, `embassy-sync` or `defmt`.
#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};
use fw_core::xcore::Kick;

pub mod imu;

/// The flight core's liveness answer (`fbeat`): waits on `kick`, then copies `ping` into `pong`,
/// forever. Core 0 bumps `ping` and kicks; a `pong` that stops following `ping` means the flight
/// core makes no progress. Any kick bits wake it. No timer, no lock, no log.
pub async fn liveness(kick: &Kick, ping: &AtomicU32, pong: &AtomicU32) -> ! {
    loop {
        kick.wait().await;
        pong.store(ping.load(Ordering::Acquire), Ordering::Release);
    }
}

/// Runs `fut`, storing `id` into `running` each time the executor polls it, that is each time the
/// task resumes after an await: the watchdog's record of the task that ran last. A plain atomic
/// store; no timer, no lock, no log.
pub async fn tagged<F: Future>(running: &AtomicU8, id: u8, fut: F) -> F::Output {
    let mut fut = pin!(fut);
    poll_fn(|cx| {
        running.store(id, Ordering::Relaxed);
        fut.as_mut().poll(cx)
    })
    .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::task::{Context, Poll, Waker};

    #[test]
    fn tagged_stores_the_id_at_every_poll() {
        let running = AtomicU8::new(0);
        let mut polls = 0;
        let inner = poll_fn(|_| {
            polls += 1;
            if polls < 3 {
                Poll::Pending
            } else {
                Poll::Ready(polls)
            }
        });
        let mut fut = pin!(tagged(&running, 17, inner));
        let mut cx = Context::from_waker(Waker::noop());
        for _ in 0..2 {
            running.store(16, Ordering::Relaxed);
            assert_eq!(fut.as_mut().poll(&mut cx), Poll::Pending);
            assert_eq!(running.load(Ordering::Relaxed), 17);
        }
        running.store(16, Ordering::Relaxed);
        assert_eq!(fut.as_mut().poll(&mut cx), Poll::Ready(3));
        assert_eq!(running.load(Ordering::Relaxed), 17);
    }
}
