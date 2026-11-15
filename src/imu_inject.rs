//! Fault injection around a flight-core IMU adapter (features `inject-imu1-persistent`,
//! `inject-imu1-missed-edge`, `inject-imu2-persistent`), 5 s after the first good sample. One
//! wrapper for both IMUs: `imu1::adapter` and `imu2::adapter` wrap their adapter in [`Inject`]
//! with their own cells and [`Mode`]. Runs on the flight core: no log, its state goes out through
//! `injection()` and core 0 logs it.

use embedded_hal_async::delay::DelayNs;
use fw_core::health::ErrorKind;
use fw_flight::imu::{ImuAdapter, ImuCells, ImuReading, Pending, StartFailure, VerifyError};

/// Injection state words published through the cells (`ImuStatus::inject`).
pub mod inject_state {
    /// Armed, waiting for 5 s of good samples.
    pub const ARMED: u32 = 1;
    /// Active: failing (persistent) or swallowing edges (missed-edge).
    pub const ACTIVE: u32 = 2;
    /// Over.
    pub const DONE: u32 = 3;
}

/// What the wrapper injects.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Every acquire, recover and restart fails until a RESTART request has been raised after the
    /// failure began.
    #[cfg(any(feature = "inject-imu1-persistent", feature = "inject-imu2-persistent"))]
    Persistent,
    /// The data-ready signal is not seen for 200 ms while the chip keeps producing.
    #[cfg(feature = "inject-imu1-missed-edge")]
    MissedEdge,
}

pub struct Inject<A> {
    inner: A,
    cells: &'static ImuCells,
    mode: Mode,
    first_good_us: Option<u64>,
    state: u32,
    since_us: u64,
    restarts_at_start: u32,
}

impl<A> Inject<A> {
    const AFTER_US: u64 = 5_000_000;
    #[cfg(feature = "inject-imu1-missed-edge")]
    const SWALLOW_US: u64 = 200_000;

    /// Wraps `inner`, whose task owns `cells` (the RESTART requests are read there).
    pub fn new(inner: A, cells: &'static ImuCells, mode: Mode) -> Self {
        Self {
            inner,
            cells,
            mode,
            first_good_us: None,
            state: inject_state::ARMED,
            since_us: 0,
            restarts_at_start: 0,
        }
    }

    fn set(&mut self, state: u32, now: u64) {
        self.state = state;
        self.since_us = now;
    }

    fn due(&self, now: u64) -> bool {
        self.state == inject_state::ARMED
            && self
                .first_good_us
                .is_some_and(|t| now.saturating_sub(t) >= Self::AFTER_US)
    }

    /// Persistent: true while every acquire, recover and restart must fail (until a RESTART
    /// request has been raised after the failure began).
    fn failing(&mut self) -> bool {
        #[cfg(any(feature = "inject-imu1-persistent", feature = "inject-imu2-persistent"))]
        if self.mode == Mode::Persistent {
            let now = crate::now_us();
            if self.due(now) {
                self.restarts_at_start = self.cells.restart_number();
                self.set(inject_state::ACTIVE, now);
            }
            if self.state == inject_state::ACTIVE
                && self.cells.restart_number() != self.restarts_at_start
            {
                self.set(inject_state::DONE, now);
            }
            return self.state == inject_state::ACTIVE;
        }
        let _ = (self.cells, self.restarts_at_start);
        false
    }
}

impl<A: ImuAdapter> ImuAdapter for Inject<A> {
    async fn data_ready(&mut self) {
        #[cfg(feature = "inject-imu1-missed-edge")]
        if self.mode == Mode::MissedEdge {
            let now = crate::now_us();
            if self.due(now) {
                self.set(inject_state::ACTIVE, now);
            }
            if self.state == inject_state::ACTIVE {
                if now.saturating_sub(self.since_us) < Self::SWALLOW_US {
                    // The signal is not seen: the wait never ends (core 0's CHECK kick drops it).
                    return core::future::pending::<()>().await;
                }
                self.set(inject_state::DONE, now);
            }
        }
        self.inner.data_ready().await;
    }

    async fn start<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), StartFailure> {
        self.inner.start(delay).await
    }

    async fn acquire(&mut self) -> Result<ImuReading, ErrorKind> {
        if self.failing() {
            return Err(ErrorKind::Bus);
        }
        let reading = self.inner.acquire().await?;
        self.first_good_us.get_or_insert_with(crate::now_us);
        Ok(reading)
    }

    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        self.inner.status().await
    }

    async fn verify(&mut self) -> Result<(), VerifyError> {
        self.inner.verify().await
    }

    async fn apply<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        self.inner.apply(delay).await
    }

    async fn recover<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        if self.failing() {
            return Err(ErrorKind::Bus);
        }
        self.inner.recover(delay).await
    }

    async fn restart<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind> {
        if self.failing() {
            return Err(ErrorKind::Bus);
        }
        self.inner.restart(delay).await
    }

    fn calls(&self) -> u32 {
        self.inner.calls()
    }

    fn injection(&self) -> (u32, u64) {
        (self.state, self.since_us)
    }

    fn latency(&self) -> (u32, u32) {
        self.inner.latency()
    }
}
