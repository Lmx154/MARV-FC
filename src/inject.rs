//! Fault injection around a device adapter, for the board checks (cargo features, off by default).
//! The inner adapter is always called, so the bus traffic is real; the wrapper then replaces the
//! result with a bus error.
//! - `inject-baro-transient`: after 5 s of good samples, `acquire` fails 4 times in a row, once.
//! - `inject-baro-persistent`: after 5 s of good samples, every `acquire`, `recover` and `restart`
//!   fails (it wins when both features are on).
//! - `inject-baro-missed-edge`: after 5 s of good samples, the data-ready edges are swallowed for
//!   200 ms while the chip keeps producing data, once.
//! - `inject-baro-silent`: the same for 1 s, and `status` answers "nothing pending" meanwhile.
//! - `inject-baro-por`: after 5 s of good samples, one `acquire` answers a power-on reset, once.
//! - `inject-baro-verify`: after 5 s of good samples, one `verify` answers "mismatch", once.
//! - `inject-baro-stuck`: after 5 s of good samples, `acquire` never returns.
//!
//! Each logs an `[INFO] inject:` line when it starts and when it ends acting.

use embassy_time::{Duration, Instant};
use fw_core::health::ErrorKind;

use crate::device::{AdapterCounters, DeviceAdapter, Pending, Reading, StartFailure, VerifyError};

const AFTER: Duration = Duration::from_secs(5);
const TRANSIENT_FAILURES: u8 = 4;
const PERSISTENT: bool = cfg!(feature = "inject-baro-persistent");
const TRANSIENT: bool = cfg!(feature = "inject-baro-transient");
const MISSED_EDGE: bool = cfg!(feature = "inject-baro-missed-edge");
const SILENT: bool = cfg!(feature = "inject-baro-silent");
const VERIFY: bool = cfg!(feature = "inject-baro-verify");
const POR: bool = cfg!(feature = "inject-baro-por");
const STUCK: bool = cfg!(feature = "inject-baro-stuck");
/// How long the edges are swallowed (`silent` wins over `missed-edge`).
const WINDOW: Option<Duration> = if SILENT {
    Some(Duration::from_secs(1))
} else if MISSED_EDGE {
    Some(Duration::from_millis(200))
} else {
    None
};

#[derive(Clone, Copy)]
enum Phase {
    Idle,
    Active(Instant),
    Done,
}

pub struct Inject<A> {
    inner: A,
    first_good: Option<Instant>,
    injected: u8,
    phase: Phase,
    verify_injected: bool,
    por_injected: bool,
    applied: bool,
}

impl<A: DeviceAdapter> Inject<A> {
    pub fn new(inner: A) -> Self {
        defmt::info!(
            "[INFO] inject: {=str} armed",
            if PERSISTENT {
                "persistent"
            } else if SILENT {
                "silent"
            } else if MISSED_EDGE {
                "missed-edge"
            } else if VERIFY {
                "verify"
            } else if POR {
                "por"
            } else if STUCK {
                "stuck"
            } else {
                "transient"
            }
        );
        Self {
            inner,
            first_good: None,
            injected: 0,
            phase: Phase::Idle,
            verify_injected: false,
            por_injected: false,
            applied: false,
        }
    }

    fn due(&self) -> bool {
        self.first_good.is_some_and(|t| t.elapsed() >= AFTER)
    }

    /// True while the edges are swallowed; starts and ends the window.
    fn swallowing(&mut self) -> bool {
        let Some(len) = WINDOW else {
            return false;
        };
        match self.phase {
            Phase::Idle if self.due() => {
                self.phase = Phase::Active(Instant::now());
                defmt::info!(
                    "[INFO] inject: edges swallowed from {=u64} ms for {=u64} ms",
                    Instant::now().as_millis(),
                    len.as_millis()
                );
                true
            }
            Phase::Active(start) if start.elapsed() >= len => {
                self.phase = Phase::Done;
                defmt::info!(
                    "[INFO] inject: edges pass again at {=u64} ms",
                    Instant::now().as_millis()
                );
                false
            }
            Phase::Active(_) => true,
            Phase::Idle | Phase::Done => false,
        }
    }

    fn window_open(&self) -> bool {
        match (self.phase, WINDOW) {
            (Phase::Active(start), Some(len)) => start.elapsed() < len,
            _ => false,
        }
    }

    fn fail(&mut self, what: &str) -> bool {
        let fail = (PERSISTENT || TRANSIENT)
            && self.due()
            && (PERSISTENT || self.injected < TRANSIENT_FAILURES);
        if fail {
            self.injected = self.injected.saturating_add(1);
            defmt::info!(
                "[INFO] inject: {=str} fails at {=u64} ms (injected failure {=u8})",
                what,
                Instant::now().as_millis(),
                self.injected
            );
        }
        fail
    }
}

impl<A: DeviceAdapter> DeviceAdapter for Inject<A> {
    async fn data_ready(&mut self) {
        loop {
            self.inner.data_ready().await;
            if !self.swallowing() {
                return;
            }
        }
    }

    async fn start(&mut self) -> Result<(), StartFailure> {
        self.inner.start().await
    }

    async fn acquire(&mut self) -> Result<Reading, ErrorKind> {
        let result = self.inner.acquire().await;
        if result.is_ok() && self.first_good.is_none() {
            self.first_good = Some(Instant::now());
        }
        if STUCK && self.due() {
            defmt::info!(
                "[INFO] inject: acquire never returns from {=u64} ms",
                Instant::now().as_millis()
            );
            match core::future::pending::<core::convert::Infallible>().await {}
        }
        if POR && !self.por_injected && self.due() {
            self.por_injected = true;
            defmt::info!(
                "[INFO] inject: acquire answers a power-on reset at {=u64} ms",
                Instant::now().as_millis()
            );
            return Err(ErrorKind::Por);
        }
        if self.fail("acquire") {
            return Err(ErrorKind::Bus);
        }
        result
    }

    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        let result = self.inner.status().await;
        if SILENT && self.window_open() && matches!(result, Ok(Pending::Data)) {
            return Ok(Pending::Nothing);
        }
        result
    }

    async fn verify(&mut self) -> Result<(), VerifyError> {
        let result = self.inner.verify().await;
        if VERIFY && !self.verify_injected && self.due() {
            self.verify_injected = true;
            defmt::info!(
                "[INFO] inject: verify answers mismatch at {=u64} ms",
                Instant::now().as_millis()
            );
            return Err(VerifyError::Mismatch);
        }
        result
    }

    async fn apply(&mut self) -> Result<(), ErrorKind> {
        let result = self.inner.apply().await;
        if VERIFY && self.verify_injected && !self.applied {
            self.applied = true;
            defmt::info!(
                "[INFO] inject: apply reached the chip at {=u64} ms, injection over",
                Instant::now().as_millis()
            );
        }
        result
    }

    async fn recover(&mut self) -> Result<(), ErrorKind> {
        let result = self.inner.recover().await;
        if PERSISTENT && self.fail("recover") {
            return Err(ErrorKind::Bus);
        }
        result
    }

    async fn restart(&mut self) -> Result<(), ErrorKind> {
        let result = self.inner.restart().await;
        if PERSISTENT && self.fail("restart") {
            return Err(ErrorKind::Bus);
        }
        result
    }

    fn counters(&self) -> AdapterCounters {
        self.inner.counters()
    }
}
