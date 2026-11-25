//! The flight-core IMU device loop. One task per IMU runs [`ImuTask::run`] over an [`ImuAdapter`]: it waits
//! on `select(data-ready edge, kick)` and never on a timer. The data-ready deadline comes from
//! core 0 as a CHECK kick ([`ImuCells::request_check`]); the woken task compares the clock with its
//! last sample and, only if it is late by more than T = max(3 P, 10 ms), makes the one status read.
//! Everything core 0 needs is published through one [`Seqlock`] ([`ImuStatus`]), written after
//! every step. Waits only through the [`DelayNs`] passed in, on the start and recovery paths,
//! counted by [`CountingDelay`]. The configuration is checked incrementally: after every K-th
//! sample one [`ImuAdapter::verify_step`] (one register), never while a sample is waiting; the loop
//! never runs a whole-configuration check. No log, no lock, no timer.

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Poll;

use embedded_hal_async::delay::DelayNs;
use fw_core::health::{Action, Counters, ErrorKind, Event, Health, Limits, Report, State};
use fw_core::selftest::{Outcome as SelfTestOutcome, Verdict};
use fw_core::xcore::{Kick, Seqlock, SeqlockWriter};

/// Kick bit: core 0 sees the last sample older than T.
pub const REQ_CHECK: u8 = 1;
/// Kick bit: restart the device, from any state; the request's number is in [`ImuCells`].
pub const REQ_RESTART: u8 = 2;
/// Kick bit: apply a new configuration. Reserved: taken and ignored (not built).
pub const REQ_APPLY: u8 = 4;
/// Kick bit: self-test the device, then restart it as for [`REQ_RESTART`] under the same number;
pub const REQ_SELF_TEST: u8 = 8;
/// Transitions kept in [`ImuStatus::ring`].
pub const TRANSITIONS: usize = 4;

/// Floor of the data-ready deadline T = max(3 P, 10 ms).
const DEADLINE_FLOOR_US: u64 = 10_000;
/// Longest single back-off wait: progress moves and a RESTART is seen between slices.
const SLICE_MS: u64 = 100;
/// Copies a core-0 reader attempts before it gives up for this tick.
const READ_ATTEMPTS: u32 = 64;

/// The physical values of one sample.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuReading {
    /// Angular rate, rad/s, body frame (FRD), offsets removed.
    pub gyro_rads: [f32; 3],
    /// Specific force, m/s^2, body frame (FRD), offsets removed: level and at rest z is about -9.81.
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

/// What a successful verify step found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifyStep {
    /// The register matched; more registers in this pass.
    More,
    /// The register matched and it was the last of the pass.
    PassComplete,
}

/// The flight task's latency figures (measured around every poll by the binary).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Latency {
    /// The longest stretch the task ran without yielding, us, since boot.
    pub max_us: u32,
    /// The same over the last completed 10 s window, us.
    pub window_us: u32,
    /// Polls of the task (every poll but the last ends in a yield).
    pub polls: u32,
}

/// A failed verify step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VerifyError {
    /// A register differs from the configuration.
    Mismatch,
    /// The check itself failed.
    Failed(ErrorKind),
}

/// The flight-core side of one IMU, driven by [`ImuTask`]. The contract of the core-0
/// `DeviceAdapter`: every method except [`data_ready`](Self::data_ready) may use the bus and is.
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
    /// One step of the incremental configuration check, changing nothing: at most one register (one
    /// or two bus transactions, a short busy wait at most); [`VerifyStep::PassComplete`] after the.
    async fn verify_step(&mut self) -> Result<VerifyStep, VerifyError>;
    /// K: one verify step after every K-th sample (at least 1).
    fn verify_every(&self) -> u32;
    /// True while a sample is waiting (the latched data-ready level). No bus traffic.
    fn data_waiting(&mut self) -> bool;
    /// Configures the running device with the adapter's configuration.
    async fn apply<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind>;
    /// Brings the bus and the device back in step without a reset.
    async fn recover<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind>;
    /// Bus clear where the bus has one, then init and configure. Used by the automatic recovery and
    /// for the first start attempt after an operator RESTART request.
    async fn restart<D: DelayNs>(&mut self, delay: &mut D) -> Result<(), ErrorKind>;
    /// The device's built-in self-test with the driver's default settings (the device must lie
    /// still). The loop then restarts the device as for a RESTART request, whatever this returned.
    async fn self_test<D: DelayNs>(&mut self, delay: &mut D) -> Verdict;
    /// Adapter operations that went to the bus.
    fn calls(&self) -> u32;
    /// A fault injection's state word and the time it last changed (us); `(0, 0)` without one.
    fn injection(&self) -> (u32, u64) {
        (0, 0)
    }
    /// The task's latency figures; all 0 when unmeasured.
    fn latency(&self) -> Latency {
        Latency::default()
    }
}

/// What the task does with each good sample right after publishing it, synchronously: IMU1's task runs
/// the flight step, IMU2's passes `()`, which does nothing.
pub trait SampleSink {
    /// One good sample, just published. Must not wait.
    fn sample(&mut self, sample: &ImuSample);
}

impl SampleSink for () {
    fn sample(&mut self, _: &ImuSample) {}
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
    /// Of those, waits since the device first reached Ok (0 in a clean run).
    pub delays_after_start: u32,
    /// The latest finished restart request: number << 1, bit 0 set when the device reached Ok,
    /// clear when Failed or Absent (0: none yet; numbers start at 1).
    pub restart_outcome: u32,
    /// The latest finished self-test (`None` before the first); the restart that follows it
    /// publishes its outcome in `restart_outcome` under the same number.
    pub self_test: Option<SelfTestOutcome>,
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
    /// Verify steps taken between samples.
    pub verifies: u32,
    /// Verify passes completed (every owned register checked once).
    pub verify_passes: u32,
    /// Duration of the latest verify step, us.
    pub verify_last_us: u32,
    /// Shortest verify step, us.
    pub verify_min_us: u32,
    /// Longest verify step, us.
    pub verify_max_us: u32,
    /// Verify steps after which a sample was already waiting (the data-ready level; no bus read).
    pub verify_pending: u32,
    /// Intervals between two samples longer than 1.5 P, counted in Ok and Degraded.
    pub long_gaps: u32,
    /// Of those, intervals that contained a verify step.
    pub long_gaps_after_verify: u32,
    /// The task's latency figures (`ImuAdapter::latency`).
    pub latency: Latency,
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
        self_test: None,
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
        verify_passes: 0,
        verify_last_us: 0,
        verify_min_us: 0,
        verify_max_us: 0,
        verify_pending: 0,
        long_gaps: 0,
        long_gaps_after_verify: 0,
        latency: Latency {
            max_us: 0,
            window_us: 0,
            polls: 0,
        },
    };
}

/// The cells between core 0 and one IMU task: the kick (requests and wakes), the number of the
/// latest restart request, and the published status.
pub struct ImuCells {
    kick: Kick,
    restart_number: AtomicU32,
    self_test_number: AtomicU32,
    status: Seqlock<ImuStatus>,
}

impl ImuCells {
    /// No request, the initial status, the writer not yet taken.
    pub const fn new() -> Self {
        Self {
            kick: Kick::new(),
            restart_number: AtomicU32::new(0),
            self_test_number: AtomicU32::new(0),
            status: Seqlock::new(ImuStatus::INITIAL),
        }
    }

    /// Core 0: self-test request `number` (stored before the kick, which publishes it).
    pub fn request_self_test(&self, number: u32) {
        self.self_test_number.store(number, Ordering::Release);
        self.kick.kick(REQ_SELF_TEST);
    }

    /// The number of the latest self-test request.
    pub fn self_test_number(&self) -> u32 {
        self.self_test_number.load(Ordering::Acquire)
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

/// One IMU's flight-core task: its `Health`, its adapter, its published status, its sample sink.
pub struct ImuTask<'a, A, D, S> {
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
    since_step: u32,
    last_status_us: u64,
    restart_run: Option<u32>,
    strong_first: bool,
    delays_at_start: Option<u32>,
    sink: S,
}

impl<'a, A: ImuAdapter, D: DelayNs, S: SampleSink> ImuTask<'a, A, D, S> {
    /// The task for `cells` (`None` if their writer was taken already), with the nominal sample
    /// period `period_us` (T = max(3 P, 10 ms)); every good sample goes to `sink` once published.
    pub fn new(
        cells: &'a ImuCells,
        adapter: A,
        delay: D,
        now_us: fn() -> u64,
        period_us: u32,
        limits: Limits,
        sink: S,
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
            since_step: 0,
            last_status_us: 0,
            restart_run: None,
            strong_first: false,
            delays_at_start: None,
            sink,
        })
    }

    /// The loop, forever.
    pub async fn run(mut self) -> ! {
        loop {
            self.once().await;
        }
    }

    /// One step: takes the requests, carries out the machine's action, feeds the result back,
    /// publishes, then hands a new sample to the sink.
    pub async fn once(&mut self) {
        let seq = self.seq;
        self.pending |= self.cells.kick.take();
        let mut restart = None;
        if self.take(REQ_SELF_TEST) {
            let number = self.cells.self_test_number();
            self.self_test(number).await;
            restart = Some(number);
        }
        if self.take(REQ_RESTART) {
            restart = Some(self.cells.restart_number());
        }
        if let Some(number) = restart {
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
                } else if self.since_step >= self.adapter.verify_every()
                    && !self.adapter.data_waiting()
                {
                    // A sample waiting is read first (the branch below); then the step.
                    self.since_step = 0;
                    let result = self.adapter.verify_step().await;
                    self.bump();
                    self.note_verify(now);
                    if self.adapter.data_waiting() {
                        self.status.verify_pending = self.status.verify_pending.wrapping_add(1);
                    }
                    match result {
                        Ok(VerifyStep::More) => Event::Tick,
                        Ok(VerifyStep::PassComplete) => {
                            self.status.verify_passes = self.status.verify_passes.wrapping_add(1);
                            Event::Tick
                        }
                        Err(VerifyError::Mismatch) => Event::VerifyMismatch,
                        Err(VerifyError::Failed(kind)) => Event::Error(kind),
                    }
                } else {
                    self.sample().await
                }
            }
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
        if self.seq != seq
            && let Some(sample) = self.status.latest
        {
            self.sink.sample(&sample);
        }
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
        self.status.latency = self.adapter.latency();
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

    /// A SELF-TEST request: the adapter's test (no sample is read meanwhile), its outcome published
    /// under request `number`.
    async fn self_test(&mut self, number: u32) {
        let begun = (self.now_us)();
        let verdict = self.adapter.self_test(&mut self.delay).await;
        self.bump();
        let took_us = u32::try_from((self.now_us)().saturating_sub(begun)).unwrap_or(u32::MAX);
        self.status.self_test = Some(SelfTestOutcome {
            number,
            verdict,
            took_us,
        });
        self.publish();
    }

    /// Records the duration of the verify step that began at `begun` (us).
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
                self.since_step = self.since_step.saturating_add(1);
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
            if self.pending & (REQ_RESTART | REQ_SELF_TEST) != 0 {
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
        /// Registers per verify pass (N) and samples per step (K).
        steps_n: u32,
        every: u32,
        step_index: u32,
        steps: u32,
        /// The step index that answers "mismatch" (once) or a bus error (once).
        mismatch_at: Option<u32>,
        bus_error_at: Option<u32>,
        applies: u32,
        /// The data-ready level: a sample is waiting.
        waiting: bool,
        /// Each good acquire leaves the next sample waiting, every other time.
        toggle_waiting: bool,
        steps_while_waiting: u32,
        /// A step longer than one period: the next sample is waiting when it returns.
        slow_step: bool,
        calls: u32,
        starts: u32,
        restarts: u32,
        /// What the self-test answers and how long it takes (us); like the ICM-45686's test it.
        self_test_verdict: Verdict,
        self_test_us: u64,
        self_tests: u32,
        stopped: bool,
        /// Core 0's CHECK kick while the test runs.
        check_during_test: Option<&'static ImuCells>,
        status_calls: u32,
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
                if self.0.borrow().stopped {
                    Poll::Pending
                } else if self.0.borrow().waiting {
                    Poll::Ready(())
                } else if self.0.borrow().producing {
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
            self.0.borrow_mut().stopped = false;
            delay.delay_ms(2).await;
            Ok(())
        }

        async fn acquire(&mut self) -> Result<ImuReading, ErrorKind> {
            self.call();
            let mut fake = self.0.borrow_mut();
            if fake.acquire_fails {
                Err(ErrorKind::Bus)
            } else {
                fake.waiting = fake.toggle_waiting && !fake.waiting;
                Ok(ImuReading {
                    accel_ms2: [0.0, 0.0, 9.81],
                    ..ImuReading::default()
                })
            }
        }

        async fn status(&mut self) -> Result<Pending, ErrorKind> {
            self.call();
            self.0.borrow_mut().status_calls += 1;
            Ok(self.0.borrow().pending)
        }

        async fn verify_step(&mut self) -> Result<VerifyStep, VerifyError> {
            self.call();
            let mut fake = self.0.borrow_mut();
            fake.steps += 1;
            if fake.waiting {
                fake.steps_while_waiting += 1;
            }
            if fake.slow_step {
                advance(P_US * 8 / 5);
                fake.waiting = true;
            }
            let index = fake.step_index;
            fake.step_index = (index + 1) % fake.steps_n;
            if fake.mismatch_at == Some(index) {
                fake.mismatch_at = None;
                return Err(VerifyError::Mismatch);
            }
            if fake.bus_error_at == Some(index) {
                fake.bus_error_at = None;
                return Err(VerifyError::Failed(ErrorKind::Bus));
            }
            Ok(if fake.step_index == 0 {
                VerifyStep::PassComplete
            } else {
                VerifyStep::More
            })
        }

        fn verify_every(&self) -> u32 {
            self.0.borrow().every
        }

        fn data_waiting(&mut self) -> bool {
            self.0.borrow().waiting
        }

        async fn apply<D: DelayNs>(&mut self, _delay: &mut D) -> Result<(), ErrorKind> {
            self.call();
            let mut fake = self.0.borrow_mut();
            fake.applies += 1;
            fake.step_index = 0;
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
                self.0.borrow_mut().stopped = false;
                Ok(())
            } else {
                Err(ErrorKind::Bus)
            }
        }

        async fn self_test<D: DelayNs>(&mut self, delay: &mut D) -> Verdict {
            self.call();
            let (us, cells) = {
                let mut fake = self.0.borrow_mut();
                fake.self_tests += 1;
                fake.stopped = true;
                (fake.self_test_us, fake.check_during_test)
            };
            delay.delay_us(us as u32).await;
            if let Some(cells) = cells {
                cells.request_check();
            }
            self.0.borrow().self_test_verdict
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

    type Task = ImuTask<'static, FakeAdapter, FakeDelay, ()>;

    fn setup() -> (&'static ImuCells, Task, Rc<RefCell<Fake>>) {
        NOW.with(|n| n.set(1_000_000));
        let cells: &'static ImuCells = Box::leak(Box::new(ImuCells::new()));
        let fake = Rc::new(RefCell::new(Fake {
            producing: true,
            pending: Pending::Nothing,
            acquire_fails: false,
            recover_ok: true,
            restart_ok: true,
            steps_n: 5,
            every: 4,
            step_index: 0,
            steps: 0,
            mismatch_at: None,
            bus_error_at: None,
            applies: 0,
            waiting: false,
            toggle_waiting: false,
            steps_while_waiting: 0,
            slow_step: false,
            calls: 0,
            starts: 0,
            restarts: 0,
            self_test_verdict: Verdict::Pass,
            self_test_us: 800_000,
            self_tests: 0,
            stopped: false,
            check_during_test: None,
            status_calls: 0,
        }));
        let task = ImuTask::new(
            cells,
            FakeAdapter(fake.clone()),
            FakeDelay,
            now,
            P_US as u32,
            Limits::DESIGN,
            (),
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
        assert!(s.samples + s.verifies >= 2999);
        assert!(s.verifies > 500);
        assert_eq!(s.health.errors, 0);
        assert_eq!(s.missed_edges, 0);
        assert_eq!(s.max_gap_us, P_US);
        assert!(s.delays > 0, "the start waits through the delay");
        assert_eq!(s.delays_after_start, 0);
    }

    #[test]
    fn a_sample_waiting_after_a_long_step_is_read_at_once() {
        let (cells, mut task, fake) = started();
        fake.borrow_mut().slow_step = true;
        let mut last = status(cells);
        let before = last.verifies;
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
        assert!(s.verifies >= before + 2);
        assert_eq!(s.verify_pending, s.verifies - before);
        assert_eq!(s.verify_last_us, 1600);
        assert_eq!(s.latest.map(|l| l.seq + 1), Some(s.samples));
        assert_eq!(s.missed_edges, 0);
        assert_eq!(s.health.errors, 0);
        assert_eq!(s.state, State::Ok);
        assert!(s.max_gap_us < 2 * P_US);
    }

    #[test]
    fn a_full_verify_pass_takes_n_steps_over_n_times_k_samples_with_no_gap() {
        let (cells, mut task, fake) = setup();
        let (n, k) = (fake.borrow().steps_n, fake.borrow().every);
        let mut last: Option<u32> = None;
        for _ in 0..1000 {
            assert!(step(&mut task));
            let s = status(cells);
            if let (Some(a), Some(b)) = (last, s.latest.map(|l| l.seq)) {
                assert!(b == a || b == a + 1);
            }
            last = s.latest.map(|l| l.seq);
            if s.verify_passes == 1 {
                break;
            }
        }
        let s = status(cells);
        assert_eq!(s.verify_passes, 1);
        assert_eq!(s.verifies, n);
        assert_eq!(s.samples, n * k);
        assert_eq!(fake.borrow().steps, n);
        assert_eq!(s.max_gap_us, P_US);
        assert_eq!(s.long_gaps, 0);
        assert_eq!(s.state, State::Ok);
        assert_eq!(s.health.errors, 0);
    }

    #[test]
    fn a_mismatch_at_step_j_is_the_mismatch_event_and_reconfigures() {
        let (cells, mut task, fake) = started();
        fake.borrow_mut().mismatch_at = Some(3);
        let mut seen = false;
        for _ in 0..200 {
            assert!(step(&mut task));
            let s = status(cells);
            seen |= (0..TRANSITIONS)
                .any(|i| s.ring[i].is_some_and(|t| t.error == ErrorKind::ConfigMismatch));
            if s.health.config_restored == 1 {
                break;
            }
        }
        let s = status(cells);
        assert_eq!(s.health.config_restored, 1);
        assert_eq!(fake.borrow().applies, 1);
        assert!(seen || s.health.last_error == ErrorKind::ConfigMismatch);
        for _ in 0..50 {
            assert!(step(&mut task));
        }
        let s = status(cells);
        assert!(matches!(s.state, State::Ok | State::Degraded));
        assert_eq!(s.health.config_restored, 1);
    }

    #[test]
    fn a_bus_error_at_a_step_is_an_error_event() {
        let (cells, mut task, fake) = started();
        fake.borrow_mut().bus_error_at = Some(2);
        for _ in 0..100 {
            assert!(step(&mut task));
            if status(cells).health.errors == 1 {
                break;
            }
        }
        let s = status(cells);
        assert_eq!(s.health.errors, 1);
        assert_eq!(s.health.last_error, ErrorKind::Bus);
        assert_eq!(s.state, State::Degraded);
        assert_eq!(fake.borrow().applies, 0);
    }

    #[test]
    fn a_step_is_never_taken_while_a_sample_is_waiting() {
        let (cells, mut task, fake) = started();
        fake.borrow_mut().toggle_waiting = true;
        let mut last = status(cells);
        for _ in 0..3000 {
            assert!(step(&mut task));
            let s = status(cells);
            if let (Some(a), Some(b)) = (last.latest, s.latest) {
                assert!(b.seq == a.seq || b.seq == a.seq + 1);
            }
            last = s;
        }
        let s = status(cells);
        assert!(fake.borrow().steps > 10);
        assert_eq!(fake.borrow().steps_while_waiting, 0);
        assert_eq!(s.health.errors, 0);
        assert_eq!(s.state, State::Ok);
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
        fake.borrow_mut().producing = true;
        fake.borrow_mut().acquire_fails = true;
        for _ in 0..4 {
            assert!(step(&mut task));
        }
        let s = status(cells);
        assert_eq!(s.state, State::Recovering);
        assert_eq!(s.health.recoveries, 0);
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

    use fw_core::selftest::axes;

    /// The action's final result is FAILED (as for a pass whose device did not come back) and the.
    fn ends_failed_with_the_device_ok(done: &ImuStatus, number: u32) {
        let outcome = done.self_test.expect("the test's outcome");
        assert_eq!(
            done.restart_outcome,
            number << 1 | 1,
            "the device came back Ok"
        );
        let pass = SelfTestOutcome {
            verdict: Verdict::Pass,
            ..outcome
        };
        assert_eq!(outcome.result(true), pass.result(false));
        assert_ne!(outcome.result(true), pass.result(true));
    }

    /// Self-test request `number` on a running device, stepped (bounded) until the restart that.
    fn self_test_run(
        cells: &ImuCells,
        task: &mut Task,
        fake: &Rc<RefCell<Fake>>,
        number: u32,
    ) -> (ImuStatus, ImuStatus) {
        let before = status(cells);
        let restarts = fake.borrow().restarts;
        cells.request_self_test(number);
        let mut done = None;
        for _ in 0..10 {
            assert!(step(task));
            let s = status(cells);
            if s.restart_outcome >> 1 == number {
                done = Some(s);
                break;
            }
        }
        let done = done.expect("the restart after the test published its outcome");
        assert_eq!(fake.borrow().self_tests, 1);
        assert_eq!(
            fake.borrow().restarts,
            restarts + 1,
            "the strong restart path"
        );
        assert_eq!(done.restart_taken, number);
        assert_eq!(done.state, State::Ok);
        assert_eq!(done.self_test.map(|o| o.number), Some(number));
        for _ in 0..20 {
            assert!(step(task));
        }
        let end = status(cells);
        assert_eq!(end.state, State::Ok);
        assert!(end.samples > done.samples, "samples flow again");
        assert_eq!(end.health.errors, before.health.errors);
        assert_eq!(end.health.timeouts, before.health.timeouts);
        (done, end)
    }

    #[test]
    fn a_passing_self_test_restarts_the_device_and_is_accepted() {
        let (cells, mut task, fake) = started();
        let (done, _) = self_test_run(cells, &mut task, &fake, 5);
        let outcome = done.self_test.expect("the test's outcome");
        assert_eq!(outcome.verdict, Verdict::Pass);
        assert_eq!(outcome.took_us, 800_000);
        assert_eq!(done.restart_outcome, 5 << 1 | 1);
        assert_ne!(outcome.result(true), outcome.result(false), "ACCEPTED");
    }

    #[test]
    fn a_failed_self_test_still_restarts_the_device_and_ends_failed_with_its_axes() {
        let (cells, mut task, fake) = started();
        let mask = axes(0, true, false, false) | axes(1, false, true, true);
        fake.borrow_mut().self_test_verdict = Verdict::Fail(mask);
        let (done, _) = self_test_run(cells, &mut task, &fake, 2);
        assert_eq!(done.self_test.map(|o| o.verdict), Some(Verdict::Fail(mask)));
        ends_failed_with_the_device_ok(&done, 2);
    }

    #[test]
    fn a_self_test_error_still_restarts_the_device_and_ends_failed() {
        let (cells, mut task, fake) = started();
        fake.borrow_mut().self_test_verdict = Verdict::Error(ErrorKind::Bus);
        let (done, _) = self_test_run(cells, &mut task, &fake, 9);
        assert_eq!(
            done.self_test.map(|o| o.verdict),
            Some(Verdict::Error(ErrorKind::Bus))
        );
        ends_failed_with_the_device_ok(&done, 9);
    }

    #[test]
    fn a_check_kicked_during_the_self_test_does_not_score() {
        let (cells, mut task, fake) = started();
        fake.borrow_mut().check_during_test = Some(cells);
        cells.request_check();
        let (done, end) = self_test_run(cells, &mut task, &fake, 4);
        assert_eq!(done.self_test.map(|o| o.verdict), Some(Verdict::Pass));
        assert_eq!(end.health.timeouts, 0);
        assert_eq!(end.health.errors, 0);
        assert_eq!(
            fake.borrow().status_calls,
            0,
            "no status read: nothing was late"
        );
        assert!(
            end.checks >= 1,
            "the stale CHECK was taken after the restart"
        );
    }
}
