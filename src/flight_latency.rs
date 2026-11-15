//! The flight core's latency figure: [`measured`] takes the clock before and after every poll of a
//! flight task and keeps, in a [`Stretch`], the longest poll (the longest stretch the task ran
//! without yielding) since boot and over the last completed 10 s window, and the poll count. No
//! timer, no lock, no log.

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};

use fw_flight::imu::Latency;

const WINDOW_MS: u32 = 10_000;

/// Written by the task (one writer), read by its adapter.
pub struct Stretch {
    max_us: AtomicU32,
    window_us: AtomicU32,
    last_window_us: AtomicU32,
    window_start_ms: AtomicU32,
    polls: AtomicU32,
}

impl Stretch {
    pub const fn new() -> Self {
        Self {
            max_us: AtomicU32::new(0),
            window_us: AtomicU32::new(0),
            last_window_us: AtomicU32::new(0),
            window_start_ms: AtomicU32::new(0),
            polls: AtomicU32::new(0),
        }
    }

    pub fn get(&self) -> Latency {
        Latency {
            max_us: self.max_us.load(Ordering::Relaxed),
            window_us: self.last_window_us.load(Ordering::Relaxed),
            polls: self.polls.load(Ordering::Relaxed),
        }
    }

    fn note(&self, took_us: u32, now_us: u64) {
        let now_ms = (now_us / 1000) as u32;
        if now_ms.wrapping_sub(self.window_start_ms.load(Ordering::Relaxed)) >= WINDOW_MS {
            self.last_window_us
                .store(self.window_us.load(Ordering::Relaxed), Ordering::Relaxed);
            self.window_us.store(0, Ordering::Relaxed);
            self.window_start_ms.store(now_ms, Ordering::Relaxed);
        }
        if took_us > self.window_us.load(Ordering::Relaxed) {
            self.window_us.store(took_us, Ordering::Relaxed);
        }
        if took_us > self.max_us.load(Ordering::Relaxed) {
            self.max_us.store(took_us, Ordering::Relaxed);
        }
        self.polls.store(
            self.polls.load(Ordering::Relaxed).wrapping_add(1),
            Ordering::Relaxed,
        );
    }
}

/// Runs `fut`, taking the clock before and after each poll into `stretch`.
pub async fn measured<F: Future>(stretch: &Stretch, fut: F) -> F::Output {
    let mut fut = pin!(fut);
    poll_fn(|cx| {
        let begun = crate::now_us();
        let poll = fut.as_mut().poll(cx);
        let ended = crate::now_us();
        stretch.note(
            u32::try_from(ended.saturating_sub(begun)).unwrap_or(u32::MAX),
            ended,
        );
        poll
    })
    .await
}
