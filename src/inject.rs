//! Fault injection around a device adapter, for the board checks (cargo features, off by default).
//! The inner adapter is always called, so the bus traffic is real; the wrapper then replaces the
//! result with a bus error.
//! - `inject-baro-transient`: after 5 s of good samples, `acquire` fails 4 times in a row, once.
//! - `inject-baro-persistent`: after 5 s of good samples, every `acquire`, `recover` and `restart`
//!   fails (it wins when both features are on).

use embassy_time::{Duration, Instant};
use fw_core::health::ErrorKind;

use crate::device::{AdapterCounters, DeviceAdapter, Pending, Reading, StartFailure, VerifyError};

const AFTER: Duration = Duration::from_secs(5);
const TRANSIENT_FAILURES: u8 = 4;
const PERSISTENT: bool = cfg!(feature = "inject-baro-persistent");

pub struct Inject<A> {
    inner: A,
    first_good: Option<Instant>,
    injected: u8,
}

impl<A: DeviceAdapter> Inject<A> {
    pub fn new(inner: A) -> Self {
        defmt::info!(
            "[INFO] inject: {=str} armed",
            if PERSISTENT {
                "persistent"
            } else {
                "transient"
            }
        );
        Self {
            inner,
            first_good: None,
            injected: 0,
        }
    }

    fn due(&self) -> bool {
        self.first_good.is_some_and(|t| t.elapsed() >= AFTER)
    }

    fn fail(&mut self, what: &str) -> bool {
        let fail = self.due() && (PERSISTENT || self.injected < TRANSIENT_FAILURES);
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
        self.inner.data_ready().await;
    }

    async fn start(&mut self) -> Result<(), StartFailure> {
        self.inner.start().await
    }

    async fn acquire(&mut self) -> Result<Reading, ErrorKind> {
        let result = self.inner.acquire().await;
        if result.is_ok() && self.first_good.is_none() {
            self.first_good = Some(Instant::now());
        }
        if self.fail("acquire") {
            return Err(ErrorKind::Bus);
        }
        result
    }

    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        self.inner.status().await
    }

    async fn verify(&mut self) -> Result<(), VerifyError> {
        self.inner.verify().await
    }

    async fn apply(&mut self) -> Result<(), ErrorKind> {
        self.inner.apply().await
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
