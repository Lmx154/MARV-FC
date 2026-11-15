//! The flight-core IMU device loop (design: decision-foundation, items 1 and 2 in the form of the
//! two-core amendment). One task per IMU runs [`ImuTask::run`] over an [`ImuAdapter`]: it waits
//! on `select(data-ready edge, kick)` and never on a timer. The data-ready deadline comes from
//! core 0 as a CHECK kick ([`ImuCells::request_check`]); the woken task compares the clock with its
//! last sample and, only if it is late by more than T = max(3 P, 10 ms), makes the one status read.
//! Everything core 0 needs is published through one [`Seqlock`] ([`ImuStatus`]), written after
//! every step. Waits only through the [`DelayNs`] passed in, on the start and recovery paths,
//! counted by [`CountingDelay`]. No log, no lock, no timer.

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Poll;

use embedded_hal_async::delay::DelayNs;
use fw_core::health::{Action, Counters, ErrorKind, Event, Health, Limits, Report, State};
use fw_core::xcore::{Kick, Seqlock, SeqlockWriter};

/// Kick bit: core 0 sees the last sample older than T.
pub const REQ_CHECK: u8 = 1;
/// Kick bit: restart the device, from any state; the request's number is in [`ImuCells`].
pub const REQ_RESTART: u8 = 2;
/// Kick bit: apply a new configuration. Reserved: taken and ignored (not built).
pub const REQ_APPLY: u8 = 4;
/// Transitions kept in [`ImuStatus::ring`].
pub const TRANSITIONS: usize = 4;

/// Floor of the data-ready deadline T = max(3 P, 10 ms).
const DEADLINE_FLOOR_US: u64 = 10_000;
/// `verify()` between samples, about once per second.
const VERIFY_EVERY_US: u64 = 1_000_000;
/// Longest single back-off wait: progress moves and a RESTART is seen between slices.
const SLICE_MS: u64 = 100;
/// Copies a core-0 reader attempts before it gives up for this tick.
const READ_ATTEMPTS: u32 = 64;

/// The physical values of one sample.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuReading {
    /// Angular rate, rad/s, chip axes.
    pub gyro_rads: [f32; 3],
    /// Acceleration, m/s^2, chip axes.
    pub accel_ms2: [f32; 3],
    /// Die temperature, degrees Celsius.
    pub temperature_c: f32,
}

/// One published sample: values, board time of the read in us, `seq` (+1 per sample).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    /// The values.
    pub reading: ImuReading,
    /// Board time (`now_us`) when the read began.
    pub t_us: u64,
    /// +1 per published sample.
    pub seq: u32,
}

/// What the one status read after a late CHECK found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pending {
    /// A sample is waiting: the edge was missed.
    Data,
    /// No sample: a deadline miss.
    Nothing,
    /// The device has been reset.
    PowerOnReset,
}

/// A failed start.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StartFailure {
    /// The device answered its identity before the start failed.
    pub identified: bool,
}

/// A failed `verify()`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifyError {
    /// A register differs from the configuration.
    Mismatch,
    /// The check itself failed.
    Failed(ErrorKind),
}

/// The flight-core side of one IMU, driven by [`ImuTask`]. The contract of the core-0
/// `DeviceAdapter`: every method except [`data_ready`](Self::data_ready) may use the bus and is
/// always awaited to completion; `Ok` from [`start`](Self::start), [`recover`](Self::recover) or
/// [`restart`](Self::restart) only says the bus and the device answer: the loop then waits for one
/// fresh sample (a data-ready edge and one good [`acquire`](Self::acquire)) before it reports
/// success; no edge within T (seen at a kick) or a failed `acquire` counts as a failure.
/// The `delay` passed in is the task's [`CountingDelay`]; [`verify`](Self::verify) runs between
/// samples and gets none: it must not wait on a timer.
#[allow(async_fn_in_trait)]
pub trait ImuAdapter {
    /// Waits for the data-ready edge. No bus traffic: it may be dropped at any await.
    async fn data_ready(&mut self);
    /// Init and configure. Used for a boot and for the start retries; the first start attempt after an
    /// operator RESTART request uses [`restart`](Self::restart) instead.
    async fn start<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), StartFailure>;
    /// Reads one sample. `Err(ErrorKind::Por)`: the device reported a power-on reset.
    async fn acquire(&mut self) -> Result<ImuReading, ErrorKind>;
    /// The one status read after a late CHECK.
    async fn status(&mut self) -> Result<Pending, ErrorKind>;
    /// Checks the configuration registers, changing nothing.
    async fn verify(&mut self) -> Result<(), VerifyError>;
    /// Configures the running device with the adapter's configuration.
    async fn apply<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind>;
    /// Brings the bus and the device back in step without a reset.
    async fn recover<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind>;
    /// Bus clear where the bus has one, then init and configure. Used by the automatic recovery and
    /// for the first start attempt after an operator RESTART request.
    async fn restart<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind>;
    /// Adapter operations that went to the bus.
    fn calls(&self) -> u32;
    /// A fault injection's state word and the time it last changed (us); `(0, 0)` without one.
    fn injection(&self) -> (u32, u64) {
        (0, 0)
    }
}

/// A [`DelayNs`] that counts its waits (one per call).
pub struct CountingDelay<D> {
    inner: D,
    count: u32,
}

impl<D> CountingDelay<D> {
    /// Wraps `inner`; the count starts at 0.
    pub const fn new(inner: D) -> Self {
        Self { inner, count: 0 }
    }

    /// Waits so far.
    pub fn count(&self) -> u32 {
        self.count
    }
}

impl<D: DelayNs> DelayNs for CountingDelay<D> {
    async fn delay_ns(&mut self, ns: u32) {
        self.count = self.count.wrapping_add(1);
        self.inner.delay_ns(ns).await;
    }

    async fn delay_us(&mut self, us: u32) {
        self.count = self.count.wrapping_add(1);
        self.inner.delay_us(us).await;
    }

    async fn delay_ms(&mut self, ms: u32) {
        self.count = self.count.wrapping_add(1);
        self.inner.delay_ms(ms).await;
    }
}

/// One health transition (or report) as the task stepped it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Transition {
    /// State before the step.
    pub before: State,
    /// State after the step.
    pub after: State,
    /// Board time of the step, ms.
    pub at_ms: u64,
    /// The machine's last error kind then.
    pub error: ErrorKind,
    /// The report the step returned.
    pub report: Option<Report>,
}

/// Everything the IMU task publishes to core 0, after every step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuStatus {
    /// Health state.
    pub state: State,
    /// Health counters (errors, timeouts, recoveries, attempts, overflows, last error, present).
    pub health: Counters,
    /// Adapter operations that went to the bus.
    pub calls: u32,
    /// +1 after every await of the loop.
    pub progress: u32,
    /// The latest sample.
    pub latest: Option<ImuSample>,
    /// Samples published.
    pub samples: u32,
    /// Board time of the latest sample, us.
    pub last_sample_us: Option<u64>,
    /// Largest interval between two published samples, us.
    pub max_gap_us: u64,
    /// CHECK kicks that found a sample waiting.
    pub missed_edges: u32,
    /// CHECK kicks taken.
    pub checks: u32,
    /// Board time of the latest missed-edge detection, us (0: none).
    pub missed_at_us: u64,
    /// The sample's age at that detection, us.
    pub missed_late_us: u64,
    /// Waits through the task's [`CountingDelay`] since boot.
    pub delays: u32,
    /// Of those, waits since the device first reached Ok (design invariant F2: 0 in a clean run).
    pub delays_after_start: u32,
    /// The latest finished restart request: number << 1, bit 0 set when the device reached Ok,
    /// clear when Failed or Absent (0: none yet; numbers start at 1).
    pub restart_outcome: u32,
    /// Number of the latest restart request taken.
    pub restart_taken: u32,
    /// Board time the restart request was taken, ms.
    pub restart_taken_ms: u64,
    /// Board time the latest recovery attempt began, ms (0: none).
    pub attempt_ms: u64,
    /// Board time of the latest failure that a recovery back-off counts from (entering
    /// Recovering, or a failed attempt), ms.
    pub failure_ms: u64,
    /// Board time the latest recovery attempt failed, ms (0: none).
    pub attempt_failed_ms: u64,
    /// Transitions recorded since boot.
    pub transitions: u32,
    /// The latest transitions: number `n` is at index `n % TRANSITIONS`.
    pub ring: [Option<Transition>; TRANSITIONS],
    /// The adapter's fault-injection state word (0: none).
    pub inject: u32,
    /// Board time the injection state last changed, us.
    pub inject_since_us: u64,
    /// `verify()` calls between samples.
    pub verifies: u32,
    /// Duration of the latest, shortest and longest `verify()`, us.
    pub verify_last_us: u32,
    /// Shortest `verify()`, us.
    pub verify_min_us: u32,
    /// Longest `verify()`, us.
    pub verify_max_us: u32,
    /// Verifies after which a sample was already waiting (the status read right after).
    pub verify_pending: u32,
    /// Intervals between two samples longer than 1.5 P, counted in Ok and Degraded.
    pub long_gaps: u32,
    /// Of those, intervals that contained a `verify()`.
    pub long_gaps_after_verify: u32,
}

impl ImuStatus {
    /// Before the task's first step.
    pub const INITIAL: Self = Self {
        state: State::Starting,
        health: Counters {
            errors: 0,
            timeouts: 0,
            recoveries: 0,
            attempts: 0,
            config_restored: 0,
            overflows: 0,
            last_error: ErrorKind::None,
            present: false,
        },
        calls: 0,
        progress: 0,
        latest: None,
        samples: 0,
        last_sample_us: None,
        max_gap_us: 0,
        missed_edges: 0,
        checks: 0,
        missed_at_us: 0,
        missed_late_us: 0,
        delays: 0,
        delays_after_start: 0,
        restart_outcome: 0,
        restart_taken: 0,
        restart_taken_ms: 0,
        attempt_ms: 0,
        failure_ms: 0,
        attempt_failed_ms: 0,
        transitions: 0,
        ring: [None; TRANSITIONS],
        inject: 0,
        inject_since_us: 0,
        verifies: 0,
        verify_last_us: 0,
        verify_min_us: 0,
        verify_max_us: 0,
        verify_pending: 0,
        long_gaps: 0,
        long_gaps_after_verify: 0,
    };
}

/// The cells between core 0 and one IMU task: the kick (requests and wakes), the number of the
/// latest restart request, and the published status.
pub struct ImuCells {
    kick: Kick,
    restart_number: AtomicU32,
    status: Seqlock<ImuStatus>,
}

impl ImuCells {
    /// No request, the initial status, the writer not yet taken.
    pub const fn new() -> Self {
        Self {
            kick: Kick::new(),
            restart_number: AtomicU32::new(0),
            status: Seqlock::new(ImuStatus::INITIAL),
        }
    }

    /// Core 0: the last sample is older than T.
    pub fn request_check(&self) {
        self.kick.kick(REQ_CHECK);
    }

    /// Core 0: restart request `number` (stored before the kick, which publishes it).
    pub fn request_restart(&self, number: u32) {
        self.restart_number.store(number, Ordering::Release);
        self.kick.kick(REQ_RESTART);
    }

    /// The number of the latest restart request.
    pub fn restart_number(&self) -> u32 {
        self.restart_number.load(Ordering::Acquire)
    }

    /// Core 0: a consistent copy of the status, or `None` if every bounded attempt overlapped a
    /// write (try again at the next tick).
    pub fn read(&self) -> Option<ImuStatus> {
        self.status.try_read(READ_ATTEMPTS)
    }
}

impl Default for ImuCells {
    fn default() -> Self {
        Self::new()
    }
}

enum Woken {
    Edge,
    Kick(u8),
}

/// The edge first: if both are ready the kick's bits stay in the kick for the next wait.
async fn edge_or_kick<F: Future<Output = ()>>(edge: F, kick: &Kick) -> Woken {
    let mut edge = pin!(edge);
    let mut bits = pin!(kick.wait());
    poll_fn(|cx| {
        if edge.as_mut().poll(cx).is_ready() {
            return Poll::Ready(Woken::Edge);
        }
        match bits.as_mut().poll(cx) {
            Poll::Ready(b) => Poll::Ready(Woken::Kick(b)),
            Poll::Pending => Poll::Pending,
        }
    })
    .await
}

/// One IMU's flight-core task: its `Health`, its adapter, its published status.
pub struct ImuTask<'a, A, D> {
    cells: &'a ImuCells,
    writer: SeqlockWriter<'a, ImuStatus>,
    adapter: A,
    delay: CountingDelay<D>,
    now_us: fn() -> u64,
    deadline_us: u64,
    long_gap_us: u64,
    verified: bool,
    health: Health,
    action: Action,
    status: ImuStatus,
    seq: u32,
    pending: u8,
    last_verify_us: u64,
    last_status_us: u64,
    restart_run: Option<u32>,
    strong_first: bool,
    delays_at_start: Option<u32>,
}

impl<'a, A: ImuAdapter, D: DelayNs> ImuTask<'a, A, D> {
    /// The task for `cells` (`None` if their writer was taken already), with the nominal sample
    /// period `period_us` (T = max(3 P, 10 ms)).
    pub fn new(
        cells: &'a ImuCells,
        adapter: A,
        delay: D,
        now_us: fn() -> u64,
        period_us: u32,
        limits: Limits,
    ) -> Option<Self> {
        let writer = cells.status.take_writer()?;
        let mut health = Health::new(limits);
        let now = now_us();
        let action = health.step(Event::Tick, now / 1000).action;
        Some(Self {
            cells,
            writer,
            adapter,
            delay: CountingDelay::new(delay),
            now_us,
            deadline_us: (3 * u64::from(period_us)).max(DEADLINE_FLOOR_US),
            long_gap_us: 3 * u64::from(period_us) / 2,
            verified: false,
            health,
            action,
            status: ImuStatus::INITIAL,
            seq: 0,
            pending: 0,
            last_verify_us: now,
            last_status_us: 0,
            restart_run: None,
            strong_first: false,
            delays_at_start: None,
        })
    }

    /// The loop, forever.
    pub async fn run(mut self) -> ! {
        loop {
            self.once().await;
        }
    }

    /// One step: takes the requests, carries out the machine's action, feeds the result back,
    /// publishes.
    pub async fn once(&mut self) {
        self.pending |= self.cells.kick.take();
        if self.take(REQ_RESTART) {
            let number = self.cells.restart_number();
            self.restart_run = Some(number);
            self.strong_first = true;
            let (before, now) = (self.health.state(), self.now_ms());
            self.action = self.health.step(Event::Restart, now).action;
            self.status.restart_taken = number;
            self.status.restart_taken_ms = now;
            self.record(before, None, now);
            self.publish();
        }
        if matches!(self.action, Action::Start | Action::Restart) {
            self.take(REQ_APPLY);
        }
        let event = match self.action {
            Action::Start if core::mem::take(&mut self.strong_first) => {
                match self.adapter.restart(&mut self.delay).await {
                    Ok(()) if self.fresh_sample().await => Event::StartOk,
                    Ok(()) => Event::StartFailed { identified: true },
                    Err(_) => Event::StartFailed { identified: false },
                }
            }
            Action::Start => match self.adapter.start(&mut self.delay).await {
                Ok(()) if self.fresh_sample().await => Event::StartOk,
                Ok(()) => Event::StartFailed { identified: true },
                Err(f) => Event::StartFailed {
                    identified: f.identified,
                },
            },
            Action::Reconfigure => Event::ReconfigureDone {
                ok: self.adapter.apply(&mut self.delay).await.is_ok(),
            },
            Action::Recover => {
                self.status.attempt_ms = self.now_ms();
                self.publish();
                let ok = self.adapter.recover(&mut self.delay).await.is_ok()
                    && self.fresh_sample().await;
                Event::RecoverDone { ok }
            }
            Action::Restart => {
                let ok = self.adapter.restart(&mut self.delay).await.is_ok()
                    && self.fresh_sample().await;
                Event::RestartDone { ok }
            }
            Action::WaitUntil(until) => {
                self.wait_until(until).await;
                Event::Tick
            }
            Action::None if matches!(self.health.state(), State::Ok | State::Degraded) => {
                let now = (self.now_us)();
                if self.take(REQ_APPLY) {
                    Event::Tick
                } else if now.saturating_sub(self.last_verify_us) >= VERIFY_EVERY_US {
                    self.last_verify_us = now;
                    let result = self.adapter.verify().await;
                    self.bump();
                    self.note_verify(now);
                    match result {
                        Ok(()) => self.after_verify().await,
                        Err(VerifyError::Mismatch) => Event::VerifyMismatch,
                        Err(VerifyError::Failed(kind)) => Event::Error(kind),
                    }
                } else {
                    self.sample().await
                }
            }
            // Park, and (not reached) None outside Ok and Degraded: no bus traffic; a request
            // or core 0's next CHECK wakes the task.
            Action::Park | Action::None => {
                if !self.take(REQ_CHECK) {
                    self.pending |= self.cells.kick.wait().await & !REQ_CHECK;
                }
                Event::Tick
            }
        };
        self.bump();

        let (before, now) = (self.health.state(), self.now_ms());
        let step = self.health.step(event, now);
        self.action = step.action;
        if step.report == Some(Report::EnteredRecovering) {
            self.status.failure_ms = now;
        }
        if event == (Event::RestartDone { ok: false }) {
            self.status.failure_ms = now;
            self.status.attempt_failed_ms = now;
        }
        self.record(before, step.report, now);
        let after = self.health.state();
        if self.delays_at_start.is_none() && after == State::Ok {
            self.delays_at_start = Some(self.delay.count());
        }
        self.pending |= self.cells.kick.take();
        if let Some(number) = self.restart_run
            && self.pending & REQ_RESTART == 0
        {
            let ok = match after {
                State::Ok => Some(1),
                State::Failed | State::Absent => Some(0),
                _ => None,
            };
            if let Some(ok) = ok {
                self.status.restart_outcome = number << 1 | ok;
                self.restart_run = None;
            }
        }
        self.publish();
    }

    fn now_ms(&self) -> u64 {
        (self.now_us)() / 1000
    }

    fn bump(&mut self) {
        self.status.progress = self.status.progress.wrapping_add(1);
    }

    fn take(&mut self, bit: u8) -> bool {
        let taken = self.pending & bit != 0;
        self.pending &= !bit;
        taken
    }

    fn record(&mut self, before: State, report: Option<Report>, now_ms: u64) {
        let after = self.health.state();
        if after == before && report.is_none() {
            return;
        }
        let index = self.status.transitions as usize % TRANSITIONS;
        if let Some(slot) = self.status.ring.get_mut(index) {
            *slot = Some(Transition {
                before,
                after,
                at_ms: now_ms,
                error: self.health.counters().last_error,
                report,
            });
        }
        self.status.transitions = self.status.transitions.wrapping_add(1);
    }

    fn publish(&mut self) {
        self.status.state = self.health.state();
        self.status.health = *self.health.counters();
        self.status.calls = self.adapter.calls();
        let count = self.delay.count();
        self.status.delays = count;
        self.status.delays_after_start =
            self.delays_at_start.map_or(0, |at| count.wrapping_sub(at));
        let (inject, since) = self.adapter.injection();
        self.status.inject = inject;
        self.status.inject_since_us = since;
        self.writer.write(self.status);
    }

    /// One wait for data-ready or a kick, and what follows it.
    async fn sample(&mut self) -> Event {
        if self.take(REQ_CHECK) {
            return self.check().await;
        }
        match edge_or_kick(self.adapter.data_ready(), &self.cells.kick).await {
            Woken::Edge => {
                self.bump();
                self.read().await
            }
            Woken::Kick(bits) => {
                self.pending |= bits;
                if self.take(REQ_CHECK) {
                    self.check().await
                } else {
                    Event::Tick
                }
            }
        }
    }

    /// Core 0 thinks the task is late: one status read if the last sample (or the last status
    /// read) is older than T.
    async fn check(&mut self) -> Event {
        self.status.checks = self.status.checks.wrapping_add(1);
        let now = (self.now_us)();
        let last = self.status.last_sample_us.unwrap_or(0);
        if now.saturating_sub(last.max(self.last_status_us)) <= self.deadline_us {
            return Event::Tick;
        }
        self.last_status_us = now;
        let pending = self.adapter.status().await;
        self.bump();
        match pending {
            Ok(Pending::Data) => {
                self.status.missed_edges = self.status.missed_edges.wrapping_add(1);
                self.status.missed_at_us = now;
                self.status.missed_late_us = now.saturating_sub(last);
                self.read().await
            }
            Ok(Pending::Nothing) => Event::Error(ErrorKind::Timeout),
            Ok(Pending::PowerOnReset) => Event::PowerOnReset,
            Err(kind) => Event::Error(kind),
        }
    }

    /// After a start, `recover` or `restart` that returned `Ok`: one fresh sample, or a failure
    /// when a kick finds no edge for longer than T.
    async fn fresh_sample(&mut self) -> bool {
        let begun = (self.now_us)();
        loop {
            match edge_or_kick(self.adapter.data_ready(), &self.cells.kick).await {
                Woken::Edge => {
                    self.bump();
                    return matches!(self.read().await, Event::Good);
                }
                Woken::Kick(bits) => {
                    self.bump();
                    self.pending |= bits & !REQ_CHECK;
                    if (self.now_us)().saturating_sub(begun) > self.deadline_us {
                        return false;
                    }
                }
            }
        }
    }

    /// Records the duration of the `verify()` that began at `begun` (us).
    fn note_verify(&mut self, begun: u64) {
        let took = u32::try_from((self.now_us)().saturating_sub(begun)).unwrap_or(u32::MAX);
        let s = &mut self.status;
        s.verify_min_us = if s.verifies == 0 {
            took
        } else {
            s.verify_min_us.min(took)
        };
        s.verify_max_us = s.verify_max_us.max(took);
        s.verify_last_us = took;
        s.verifies = s.verifies.wrapping_add(1);
        self.verified = true;
    }

    /// After a `verify()` between samples: one status read, and a sample already waiting is read
    /// at once (its data-ready signal may have come and gone while `verify` ran: waiting for the
    /// next one would lose it).
    async fn after_verify(&mut self) -> Event {
        let pending = self.adapter.status().await;
        self.bump();
        match pending {
            Ok(Pending::Data) => {
                self.status.verify_pending = self.status.verify_pending.wrapping_add(1);
                self.read().await
            }
            Ok(Pending::Nothing) => Event::Tick,
            Ok(Pending::PowerOnReset) => Event::PowerOnReset,
            Err(kind) => Event::Error(kind),
        }
    }

    /// Reads and publishes one sample.
    async fn read(&mut self) -> Event {
        let t_us = (self.now_us)();
        match self.adapter.acquire().await {
            Ok(reading) => {
                if let Some(last) = self.status.last_sample_us {
                    let gap = t_us.saturating_sub(last);
                    self.status.max_gap_us = self.status.max_gap_us.max(gap);
                    if gap > self.long_gap_us
                        && matches!(self.health.state(), State::Ok | State::Degraded)
                    {
                        self.status.long_gaps = self.status.long_gaps.wrapping_add(1);
                        if self.verified {
                            self.status.long_gaps_after_verify =
                                self.status.long_gaps_after_verify.wrapping_add(1);
                        }
                    }
                }
                self.verified = false;
                self.status.last_sample_us = Some(t_us);
                self.status.latest = Some(ImuSample {
                    reading,
                    t_us,
                    seq: self.seq,
                });
                self.seq = self.seq.wrapping_add(1);
                self.status.samples = self.status.samples.wrapping_add(1);
                Event::Good
            }
            Err(ErrorKind::Por) => Event::PowerOnReset,
            Err(kind) => Event::Error(kind),
        }
    }

    /// A back-off wait through the delay, in slices; ends early on a RESTART request.
    async fn wait_until(&mut self, until_ms: u64) {
        loop {
            let now = self.now_ms();
            if now >= until_ms {
                return;
            }
            let slice = (until_ms - now).min(SLICE_MS) as u32;
            self.delay.delay_ms(slice).await;
            self.bump();
            self.pending |= self.cells.kick.take();
            if self.pending & REQ_RESTART != 0 {
                return;
            }
            self.publish();
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::boxed::Box;
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;
    use std::task::{Context, Waker};
    use std::vec::Vec;

    const P_US: u64 = 1000;

    std::thread_local! {
        static NOW: Cell<u64> = const { Cell::new(0) };
    }

    fn now() -> u64 {
        NOW.with(Cell::get)
    }

    fn advance(us: u64) {
        NOW.with(|n| n.set(n.get() + us));
    }

    struct Fake {
        producing: bool,
        pending: Pending,
        acquire_fails: bool,
        recover_ok: bool,
        restart_ok: bool,
        slow_verify: bool,
        calls: u32,
        starts: u32,
        restarts: u32,
    }

    struct FakeAdapter(Rc<RefCell<Fake>>);

    impl FakeAdapter {
        fn call(&self) {
            self.0.borrow_mut().calls += 1;
        }
    }

    impl ImuAdapter for FakeAdapter {
        async fn data_ready(&mut self) {
            poll_fn(|_| {
                if self.0.borrow().producing {
                    advance(P_US);
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await
        }

        async fn start<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), StartFailure> {
            self.call();
            self.0.borrow_mut().starts += 1;
            delay.delay_ms(2).await;
            Ok(())
        }

        async fn acquire(&mut self) -> Result<ImuReading, ErrorKind> {
            self.call();
            let mut fake = self.0.borrow_mut();
            if fake.slow_verify && fake.pending == Pending::Data {
                fake.pending = Pending::Nothing;
                fake.producing = true;
            }
            drop(fake);
            if self.0.borrow().acquire_fails {
                Err(ErrorKind::Bus)
            } else {
                Ok(ImuReading {
                    accel_ms2: [0.0, 0.0, 9.81],
                    ..ImuReading::default()
                })
            }
        }

        async fn status(&mut self) -> Result<Pending, ErrorKind> {
            self.call();
            Ok(self.0.borrow().pending)
        }

        async fn verify(&mut self) -> Result<(), VerifyError> {
            self.call();
            let mut fake = self.0.borrow_mut();
            if fake.slow_verify {
                // Longer than one period: the next sample's data-ready signal came and went
                // while it ran; the sample is waiting in the device.
                advance(P_US * 8 / 5);
                fake.producing = false;
                fake.pending = Pending::Data;
            }
            Ok(())
        }

        async fn apply<D: DelayNs>(&mut self, _delay: &mut D) -> Result<(), ErrorKind> {
            self.call();
            Ok(())
        }

        async fn recover<D: DelayNs>(&mut self, _delay: &mut D) -> Result<(), ErrorKind> {
            self.call();
            if self.0.borrow().recover_ok {
                Ok(())
            } else {
                Err(ErrorKind::Bus)
            }
        }

        async fn restart<D: DelayNs>(&mut self, _delay: &mut D) -> Result<(), ErrorKind> {
            self.call();
            self.0.borrow_mut().restarts += 1;
            if self.0.borrow().restart_ok {
                Ok(())
            } else {
                Err(ErrorKind::Bus)
            }
        }

        fn calls(&self) -> u32 {
            self.0.borrow().calls
        }
    }

    /// Advances the fake clock by the requested time.
    struct FakeDelay;

    impl DelayNs for FakeDelay {
        async fn delay_ns(&mut self, ns: u32) {
            advance(u64::from(ns) / 1000);
        }
    }

    type Task = ImuTask<'static, FakeAdapter, FakeDelay>;

    fn setup() -> (&'static ImuCells, Task, Rc<RefCell<Fake>>) {
        NOW.with(|n| n.set(1_000_000));
        let cells: &'static ImuCells = Box::leak(Box::new(ImuCells::new()));
        let fake = Rc::new(RefCell::new(Fake {
            producing: true,
            pending: Pending::Nothing,
            acquire_fails: false,
            recover_ok: true,
            restart_ok: true,
            slow_verify: false,
            calls: 0,
            starts: 0,
            restarts: 0,
        }));
        let task = ImuTask::new(
            cells,
            FakeAdapter(fake.clone()),
            FakeDelay,
            now,
            P_US as u32,
            Limits::DESIGN,
        )
        .expect("first writer");
        (cells, task, fake)
    }

    /// Polls one step once; `true` if it completed.
    fn step(task: &mut Task) -> bool {
        let mut fut = pin!(task.once());
        let mut cx = Context::from_waker(Waker::noop());
        fut.as_mut().poll(&mut cx).is_ready()
    }

    fn status(cells: &ImuCells) -> ImuStatus {
        cells.read().expect("no writer in flight")
    }

    fn started() -> (&'static ImuCells, Task, Rc<RefCell<Fake>>) {
        let (cells, mut task, fake) = setup();
        assert!(step(&mut task));
        assert_eq!(status(cells).state, State::Ok);
        for _ in 0..5 {
            assert!(step(&mut task));
        }
        (cells, task, fake)
    }

    /// The stream stops; a CHECK kick after `late_us`.
    fn late_check(cells: &ImuCells, task: &mut Task, fake: &Rc<RefCell<Fake>>, late_us: u64) {
        fake.borrow_mut().producing = false;
        advance(late_us);
        cells.request_check();
        assert!(step(task));
    }

    fn missed_three(cells: &ImuCells, task: &mut Task, fake: &Rc<RefCell<Fake>>) {
        fake.borrow_mut().pending = Pending::Nothing;
        for _ in 0..3 {
            late_check(cells, task, fake, 11_000);
        }
        assert_eq!(status(cells).state, State::Recovering);
    }

    #[test]
    fn clean_stream_keeps_ok_seq_continuous_and_no_delay_after_start() {
        let (cells, mut task, _fake) = setup();
        let mut last_seq: Option<u32> = None;
        for _ in 0..3000 {
            assert!(step(&mut task));
            let s = status(cells);
            assert_eq!(s.state, State::Ok);
            if let Some(sample) = s.latest {
                if let Some(prev) = last_seq {
                    assert!(sample.seq == prev || sample.seq == prev + 1);
                }
                last_seq = Some(sample.seq);
            }
        }
        let s = status(cells);
        assert_eq!(s.latest.map(|l| l.seq + 1), Some(s.samples));
        assert!(s.samples > 2900);
        assert_eq!(s.health.errors, 0);
        assert_eq!(s.missed_edges, 0);
        assert_eq!(s.max_gap_us, P_US);
        assert!(s.delays > 0, "the start waits through the delay");
        assert_eq!(s.delays_after_start, 0);
    }

    #[test]
    fn a_sample_waiting_after_a_long_verify_is_read_at_once() {
        let (cells, mut task, fake) = started();
        fake.borrow_mut().slow_verify = true;
        let mut last = status(cells);
        for _ in 0..2500 {
            assert!(
                step(&mut task),
                "the loop waited for a data-ready signal with a sample waiting"
            );
            let s = status(cells);
            if let (Some(a), Some(b)) = (last.latest, s.latest) {
                assert!(b.seq == a.seq || b.seq == a.seq + 1);
            }
            last = s;
        }
        let s = status(cells);
        assert!(s.verifies >= 2);
        assert_eq!(s.verify_pending, s.verifies);
        assert_eq!(s.verify_last_us, 1600);
        assert_eq!(s.latest.map(|l| l.seq + 1), Some(s.samples));
        assert_eq!(s.missed_edges, 0);
        assert_eq!(s.health.errors, 0);
        assert_eq!(s.state, State::Ok);
        assert!(s.max_gap_us < 2 * P_US);
    }

    #[test]
    fn check_when_not_late_reads_nothing() {
        let (cells, mut task, fake) = started();
        let calls = status(cells).calls;
        late_check(cells, &mut task, &fake, 5_000);
        let s = status(cells);
        assert_eq!(s.calls, calls);
        assert_eq!(s.checks, 1);
        assert_eq!(s.health.errors, 0);
    }

    #[test]
    fn late_check_with_data_is_a_missed_edge_and_no_health_event() {
        let (cells, mut task, fake) = started();
        let before = status(cells);
        fake.borrow_mut().pending = Pending::Data;
        late_check(cells, &mut task, &fake, 15_000);
        let s = status(cells);
        assert_eq!(s.missed_edges, 1);
        assert_eq!(s.samples, before.samples + 1);
        assert_eq!(s.health.errors, 0);
        assert_eq!(s.state, State::Ok);
        assert_eq!(s.missed_late_us, 15_000);
    }

    #[test]
    fn late_check_with_nothing_is_a_deadline_miss() {
        let (cells, mut task, fake) = started();
        late_check(cells, &mut task, &fake, 15_000);
        let s = status(cells);
        assert_eq!(s.health.timeouts, 1);
        assert_eq!(s.health.errors, 1);
        assert_eq!(s.health.last_error, ErrorKind::Timeout);
        assert_eq!(s.state, State::Degraded);
    }

    #[test]
    fn three_deadline_misses_reach_recovering() {
        let (cells, mut task, fake) = started();
        missed_three(cells, &mut task, &fake);
        let s = status(cells);
        assert_eq!(s.health.timeouts, 3);
        let last = (s.transitions as usize - 1) % TRANSITIONS;
        assert_eq!(
            s.ring[last].and_then(|t| t.report),
            Some(Report::EnteredRecovering)
        );
    }

    #[test]
    fn recovery_succeeds_only_after_a_fresh_sample() {
        let (cells, mut task, fake) = started();
        missed_three(cells, &mut task, &fake);
        // `recover` and `restart` answer Ok, but no sample can be read: no success.
        fake.borrow_mut().producing = true;
        fake.borrow_mut().acquire_fails = true;
        for _ in 0..4 {
            assert!(step(&mut task));
        }
        let s = status(cells);
        assert_eq!(s.state, State::Recovering);
        assert_eq!(s.health.recoveries, 0);
        // Samples can be read again: the next attempt succeeds on its fresh sample.
        fake.borrow_mut().acquire_fails = false;
        let samples = s.samples;
        for _ in 0..8 {
            assert!(step(&mut task));
            if status(cells).state == State::Ok {
                break;
            }
        }
        let s = status(cells);
        assert_eq!(s.state, State::Ok);
        assert_eq!(s.health.recoveries, 1);
        assert_eq!(s.samples, samples + 1);
    }

    fn failed_after_four(cells: &ImuCells, task: &mut Task, fake: &Rc<RefCell<Fake>>) {
        missed_three(cells, task, fake);
        fake.borrow_mut().producing = true;
        fake.borrow_mut().acquire_fails = true;
        let mut gaps = Vec::new();
        let mut seen = status(cells);
        for _ in 0..200 {
            assert!(step(task));
            let s = status(cells);
            if s.attempt_ms != seen.attempt_ms {
                gaps.push(s.attempt_ms - seen.failure_ms);
            }
            seen = s;
            if s.state == State::Failed {
                break;
            }
        }
        assert_eq!(seen.state, State::Failed);
        assert_eq!(seen.health.attempts, 4);
        assert_eq!(gaps, [100, 300, 1000, 3000]);
    }

    #[test]
    fn exactly_four_failed_attempts_end_failed() {
        let (cells, mut task, fake) = started();
        failed_after_four(cells, &mut task, &fake);
        // Parked: no bus traffic on a kick.
        let calls = status(cells).calls;
        cells.request_check();
        assert!(step(&mut task));
        let s = status(cells);
        assert_eq!(s.state, State::Failed);
        assert_eq!(s.calls, calls);
    }

    #[test]
    fn restart_from_failed_returns_to_ok_and_publishes_its_number() {
        let (cells, mut task, fake) = started();
        failed_after_four(cells, &mut task, &fake);
        fake.borrow_mut().acquire_fails = false;
        cells.request_restart(7);
        for _ in 0..8 {
            assert!(step(&mut task));
            if status(cells).state == State::Ok {
                break;
            }
        }
        let s = status(cells);
        assert_eq!(s.state, State::Ok);
        assert_eq!(s.restart_taken, 7);
        assert_eq!(s.restart_outcome, 7 << 1 | 1);
    }

    #[test]
    fn a_boot_calls_start_and_an_operator_restart_calls_restart_for_its_first_attempt() {
        let (cells, mut task, fake) = setup();
        assert!(step(&mut task));
        assert_eq!(status(cells).state, State::Ok);
        assert_eq!((fake.borrow().starts, fake.borrow().restarts), (1, 0));
        failed_after_four(cells, &mut task, &fake);
        fake.borrow_mut().acquire_fails = false;
        let (starts, restarts) = (fake.borrow().starts, fake.borrow().restarts);
        cells.request_restart(1);
        for _ in 0..8 {
            assert!(step(&mut task));
            if status(cells).state == State::Ok {
                break;
            }
        }
        assert_eq!(status(cells).state, State::Ok);
        assert_eq!(fake.borrow().restarts, restarts + 1);
        assert_eq!(fake.borrow().starts, starts);
    }

    #[test]
    fn a_failed_operator_restart_goes_on_to_the_start_retries() {
        let (cells, mut task, fake) = started();
        failed_after_four(cells, &mut task, &fake);
        fake.borrow_mut().acquire_fails = false;
        fake.borrow_mut().restart_ok = false;
        let (starts, restarts) = (fake.borrow().starts, fake.borrow().restarts);
        cells.request_restart(1);
        for _ in 0..40 {
            assert!(step(&mut task));
            if status(cells).state == State::Ok {
                break;
            }
        }
        assert_eq!(status(cells).state, State::Ok);
        assert_eq!(fake.borrow().restarts, restarts + 1);
        assert_eq!(fake.borrow().starts, starts + 1);
    }
}
