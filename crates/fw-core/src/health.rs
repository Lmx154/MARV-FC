//! The per-device health state machine.
//!
//! One [`Health`] per device, stepped by the device's own task. It performs no I/O and reads no
//! clock: the task passes each [`Event`] with the current time in milliseconds and carries out the
//! [`Action`] that comes back, then feeds the result in as the next event.
//!
//! # Driving protocol
//!
//! 1. Create the machine with [`Health::new`]; it is in `Starting` and wants a start.
//! 2. Call [`Health::step`] with [`Event::Tick`] to get the first action ([`Action::Start`]).
//! 3. Carry the action out. For [`Action::WaitUntil`] sleep until the deadline, then step with
//!    [`Event::Tick`]. For `Start`, `Reconfigure`, `Recover` and `Restart` run the operation and
//!    step with its result event. [`Action::None`] means nothing is due (sampling goes on in `Ok` and
//!    `Degraded`, an operation is in flight in the other states). [`Action::Park`] means no bus
//!    traffic at all.
//! 4. Any event may be fed at any time; the returned action is always the current instruction.
//!    Results that do not match the operation in flight are ignored.
//!
//! # Time
//!
//! Time is `u64` milliseconds from any monotonic origin (the caller's boot clock). A `u64` does not
//! wrap in practice, so a back-off deadline is a plain `now + wait` (saturating) and is compared
//! with `>=`; there is no wrap-around to handle. Time that steps backwards only makes a wait
//! longer, never shorter.
//!
//! # Not in this module
//!
//! The caller or the supervisor owns: the `stuck` flag (reported as Failed with error STUCK),
//! discarding a timeout whose wait overlapped a flash save, the one-text-per-second rate limit and
//! the sending of the texts (this module only reports the transitions), and the deadlines
//! `T = max(3 P, 10 ms)`. They are inputs: the task raises [`Event::Error`] with
//! [`ErrorKind::Timeout`] only for a timeout that counts.

/// The health state of a device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    /// Not yet started, or retrying the start.
    Starting,
    /// Running, error score 0.
    Ok,
    /// Running with a non-zero error score.
    Degraded,
    /// Being recovered (back-off, `recover`, `restart`).
    Recovering,
    /// Given up; parked until RESTART or reboot.
    Failed,
    /// Never started; parked until RESTART or reboot.
    Absent,
}

/// The kind of the last error, as in the dialect's error enum.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ErrorKind {
    /// No error yet.
    #[default]
    None,
    /// Bus error, checksum or decode error.
    Bus,
    /// Deadline miss.
    Timeout,
    /// The device answered with the wrong identity.
    IdMismatch,
    /// `verify` found a register that differs from the configuration.
    ConfigMismatch,
    /// Power-on reset seen.
    Por,
    /// Stuck task (set by the supervisor, never by this machine).
    Stuck,
}

/// The numbers of the machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
    /// Score added per error event.
    pub error_weight: u8,
    /// Score removed per good sample (floor 0).
    pub good_weight: u8,
    /// Score at which Degraded becomes Recovering.
    pub threshold: u8,
    /// Wait before start retry 1, 2, 3 (the last entry is reused for more retries).
    pub start_backoff_ms: [u32; 3],
    /// Number of start retries; the failure of the last retry makes the device Absent.
    pub start_retries: u8,
    /// Back-off before recovery attempt 1, 2, 3, 4 (the last entry is reused beyond four).
    pub recovery_backoff_ms: [u32; 4],
    /// Recovery attempts per episode; a failed last attempt makes the device Failed.
    pub max_attempts: u8,
    /// Entering Recovering this many times within the window makes the device Failed (below 2: off).
    pub flap_entries: u8,
    /// The flap window in milliseconds.
    pub flap_window_ms: u32,
}

impl Limits {
    /// The values of the design.
    pub const DESIGN: Limits = Limits {
        error_weight: 3,
        good_weight: 1,
        threshold: 9,
        start_backoff_ms: [100, 300, 1000],
        start_retries: 3,
        recovery_backoff_ms: [100, 300, 1000, 3000],
        max_attempts: 4,
        flap_entries: 4,
        flap_window_ms: 60_000,
    };
}

impl Default for Limits {
    fn default() -> Self {
        Limits::DESIGN
    }
}

/// An input to the machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    /// The start succeeded; the device answered its identity.
    StartOk,
    /// The start failed; `identified` says whether the device answered its identity anyway.
    StartFailed {
        /// The device answered its identity before the start failed.
        identified: bool,
    },
    /// A good sample.
    Good,
    /// An error event: deadline miss ([`ErrorKind::Timeout`]), bus, checksum or decode error
    /// ([`ErrorKind::Bus`]), wrong identity ([`ErrorKind::IdMismatch`]). Use
    /// [`Event::VerifyMismatch`] for a `verify` mismatch.
    Error(ErrorKind),
    /// `verify` found a mismatch: an error event, and the task must reconfigure at once.
    VerifyMismatch,
    /// A power-on reset of the device was seen.
    PowerOnReset,
    /// The reconfigure after a `verify` mismatch finished.
    ReconfigureDone {
        /// It succeeded.
        ok: bool,
    },
    /// `recover()` finished.
    RecoverDone {
        /// It succeeded.
        ok: bool,
    },
    /// `restart()` finished.
    RestartDone {
        /// It succeeded.
        ok: bool,
    },
    /// A FIFO overflow or UART overrun: counted, never an error event.
    Overflow,
    /// Time has passed (a wait may have ended).
    Tick,
    /// The ground station's RESTART request: allowed in every state; behaves like boot.
    Restart,
}

/// What the device's task must do next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action {
    /// Nothing new.
    None,
    /// Run the device's start (init, identity, configure).
    Start,
    /// Rewrite the configuration after a `verify` mismatch.
    Reconfigure,
    /// Call `recover()`.
    Recover,
    /// Call `restart()`.
    Restart,
    /// Wait until this time (ms), then step with [`Event::Tick`].
    WaitUntil(u64),
    /// No bus traffic; nothing is retried automatically.
    Park,
}

/// A transition the supervisor reports (one text each; Ok/Degraded flapping has none).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Report {
    /// Entered Recovering.
    EnteredRecovering,
    /// Recovering -> Ok.
    Recovered,
    /// Became Failed.
    Failed,
    /// Became Absent.
    Absent,
}

/// The result of one [`Health::step`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Step {
    /// What the task must do next.
    pub action: Action,
    /// A transition that happened in this step, if one is to be reported.
    pub report: Option<Report>,
}

/// The counters and report data the machine owns (they feed the device's status row).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    /// Error events counted while Ok or Degraded.
    pub errors: u32,
    /// Of those, deadline misses.
    pub timeouts: u32,
    /// Recoveries completed (Recovering -> Ok).
    pub recoveries: u32,
    /// Recovery attempts started in the current or last episode.
    pub attempts: u8,
    /// `verify` mismatches repaired by a successful reconfigure.
    pub config_restored: u32,
    /// FIFO overflows and UART overruns.
    pub overflows: u32,
    /// Kind of the last error event (or power-on reset).
    pub last_error: ErrorKind,
    /// The device answered its identity at least once this boot (sticky).
    pub present: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Op {
    Start,
    Reconfigure,
    Recover,
    Restart,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Idle,
    Due(Op),
    Busy(Op),
    Wait { until: u64, then: Op },
    Parked,
}

const FLAP_CAP: usize = 8;

/// The health state machine of one device.
#[derive(Clone, Debug)]
pub struct Health {
    limits: Limits,
    state: State,
    phase: Phase,
    score: u8,
    counters: Counters,
    start_failures: u8,
    flap: [u64; FLAP_CAP],
    flap_len: usize,
}

fn backoff(table: &[u32], index: usize) -> u64 {
    u64::from(table.get(index).or(table.last()).copied().unwrap_or(0))
}

impl Health {
    /// A machine in `Starting` that wants a start (get it with a first [`Event::Tick`]).
    pub fn new(limits: Limits) -> Self {
        Health {
            limits,
            state: State::Starting,
            phase: Phase::Due(Op::Start),
            score: 0,
            counters: Counters::default(),
            start_failures: 0,
            flap: [0; FLAP_CAP],
            flap_len: 0,
        }
    }

    /// The current state.
    pub fn state(&self) -> State {
        self.state
    }

    /// The current error score.
    pub fn score(&self) -> u8 {
        self.score
    }

    /// The counters and report data.
    pub fn counters(&self) -> &Counters {
        &self.counters
    }

    /// The limits in use.
    pub fn limits(&self) -> &Limits {
        &self.limits
    }

    /// Feed one event at time `now_ms`; returns the next action and any transition to report.
    pub fn step(&mut self, event: Event, now_ms: u64) -> Step {
        let report = self.apply(event, now_ms);
        let action = self.poll(now_ms);
        Step { action, report }
    }

    fn apply(&mut self, event: Event, now: u64) -> Option<Report> {
        if event == Event::Restart {
            self.restart();
            return None;
        }
        if matches!(self.state, State::Failed | State::Absent) {
            return None;
        }
        if event == Event::Overflow {
            self.counters.overflows = self.counters.overflows.saturating_add(1);
            return None;
        }
        match self.state {
            State::Starting => self.on_starting(event, now),
            State::Ok | State::Degraded => self.on_service(event, now),
            State::Recovering => self.on_recovering(event, now),
            State::Failed | State::Absent => None,
        }
    }

    fn restart(&mut self) {
        self.state = State::Starting;
        self.phase = Phase::Due(Op::Start);
        self.score = 0;
        self.start_failures = 0;
        self.flap_len = 0;
    }

    fn on_starting(&mut self, event: Event, now: u64) -> Option<Report> {
        if self.phase != Phase::Busy(Op::Start) {
            return None;
        }
        match event {
            Event::StartOk => {
                self.counters.present = true;
                self.state = State::Ok;
                self.score = 0;
                self.start_failures = 0;
                self.phase = Phase::Idle;
                None
            }
            Event::StartFailed { identified } => {
                self.counters.present |= identified;
                self.start_failures = self.start_failures.saturating_add(1);
                if self.start_failures > self.limits.start_retries {
                    self.state = State::Absent;
                    self.phase = Phase::Parked;
                    Some(Report::Absent)
                } else {
                    let index = usize::from(self.start_failures).saturating_sub(1);
                    let wait = backoff(&self.limits.start_backoff_ms, index);
                    self.phase = Phase::Wait {
                        until: now.saturating_add(wait),
                        then: Op::Start,
                    };
                    None
                }
            }
            _ => None,
        }
    }

    fn on_service(&mut self, event: Event, now: u64) -> Option<Report> {
        match event {
            Event::Good => {
                if self.state == State::Degraded {
                    self.score = self.score.saturating_sub(self.limits.good_weight);
                    if self.score == 0 {
                        self.state = State::Ok;
                    }
                }
                None
            }
            Event::Error(kind) => self.error_event(kind, now),
            Event::VerifyMismatch => {
                let reconfiguring = self.phase == Phase::Busy(Op::Reconfigure);
                let report = self.error_event(ErrorKind::ConfigMismatch, now);
                if !reconfiguring && matches!(self.state, State::Ok | State::Degraded) {
                    self.phase = Phase::Due(Op::Reconfigure);
                }
                report
            }
            Event::PowerOnReset => {
                self.counters.last_error = ErrorKind::Por;
                self.enter_recovering(now)
            }
            Event::ReconfigureDone { ok } if self.phase == Phase::Busy(Op::Reconfigure) => {
                if ok {
                    self.counters.config_restored = self.counters.config_restored.saturating_add(1);
                    self.phase = Phase::Idle;
                    None
                } else {
                    self.enter_recovering(now)
                }
            }
            _ => None,
        }
    }

    fn error_event(&mut self, kind: ErrorKind, now: u64) -> Option<Report> {
        self.counters.errors = self.counters.errors.saturating_add(1);
        if kind == ErrorKind::Timeout {
            self.counters.timeouts = self.counters.timeouts.saturating_add(1);
        }
        self.counters.last_error = kind;
        self.score = self.score.saturating_add(self.limits.error_weight);
        if self.score >= self.limits.threshold {
            self.enter_recovering(now)
        } else {
            self.state = State::Degraded;
            None
        }
    }

    fn enter_recovering(&mut self, now: u64) -> Option<Report> {
        if self.flapping(now) {
            self.state = State::Failed;
            self.phase = Phase::Parked;
            return Some(Report::Failed);
        }
        self.flap.rotate_left(1);
        if let Some(last) = self.flap.last_mut() {
            *last = now;
        }
        self.flap_len = (self.flap_len + 1).min(FLAP_CAP);
        self.state = State::Recovering;
        self.counters.attempts = 0;
        self.phase = Phase::Wait {
            until: now.saturating_add(backoff(&self.limits.recovery_backoff_ms, 0)),
            then: Op::Recover,
        };
        Some(Report::EnteredRecovering)
    }

    fn flapping(&self, now: u64) -> bool {
        let entries = usize::from(self.limits.flap_entries).min(FLAP_CAP + 1);
        let Some(back) = entries.checked_sub(2) else {
            return false;
        };
        match self.flap.iter().rev().take(self.flap_len).nth(back) {
            Some(&first) => now.saturating_sub(first) <= u64::from(self.limits.flap_window_ms),
            None => false,
        }
    }

    fn on_recovering(&mut self, event: Event, now: u64) -> Option<Report> {
        match event {
            Event::RecoverDone { ok } if self.phase == Phase::Busy(Op::Recover) => {
                if ok {
                    Some(self.recovered())
                } else {
                    self.phase = Phase::Due(Op::Restart);
                    None
                }
            }
            Event::RestartDone { ok } if self.phase == Phase::Busy(Op::Restart) => {
                if ok {
                    Some(self.recovered())
                } else {
                    self.attempt_failed(now)
                }
            }
            _ => None,
        }
    }

    fn recovered(&mut self) -> Report {
        self.state = State::Ok;
        self.score = 0;
        self.counters.recoveries = self.counters.recoveries.saturating_add(1);
        self.phase = Phase::Idle;
        Report::Recovered
    }

    fn attempt_failed(&mut self, now: u64) -> Option<Report> {
        if self.counters.attempts >= self.limits.max_attempts {
            self.state = State::Failed;
            self.phase = Phase::Parked;
            return Some(Report::Failed);
        }
        let wait = backoff(
            &self.limits.recovery_backoff_ms,
            usize::from(self.counters.attempts),
        );
        self.phase = Phase::Wait {
            until: now.saturating_add(wait),
            then: Op::Recover,
        };
        None
    }

    fn poll(&mut self, now: u64) -> Action {
        match self.phase {
            Phase::Idle | Phase::Busy(_) => Action::None,
            Phase::Parked => Action::Park,
            Phase::Due(op) => self.issue(op),
            Phase::Wait { until, then } => {
                if now >= until {
                    self.issue(then)
                } else {
                    Action::WaitUntil(until)
                }
            }
        }
    }

    fn issue(&mut self, op: Op) -> Action {
        self.phase = Phase::Busy(op);
        match op {
            Op::Start => Action::Start,
            Op::Reconfigure => Action::Reconfigure,
            Op::Recover => {
                self.counters.attempts = self.counters.attempts.saturating_add(1);
                Action::Recover
            }
            Op::Restart => Action::Restart,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn st(action: Action, report: Option<Report>) -> Step {
        Step { action, report }
    }

    fn quiet(action: Action) -> Step {
        st(action, None)
    }

    struct Rig {
        h: Health,
    }

    impl Rig {
        fn new() -> Self {
            Self::with(Limits::DESIGN)
        }

        fn with(limits: Limits) -> Self {
            Rig {
                h: Health::new(limits),
            }
        }

        fn at(&mut self, t: u64, e: Event) -> Step {
            self.h.step(e, t)
        }

        /// Boot to Ok at time `t`.
        fn boot(&mut self, t: u64) {
            assert_eq!(self.at(t, Event::Tick), quiet(Action::Start));
            assert_eq!(self.at(t, Event::StartOk), quiet(Action::None));
            assert_eq!(self.h.state(), State::Ok);
        }

        fn booted() -> Self {
            let mut r = Self::new();
            r.boot(0);
            r
        }

        /// Run recovery attempts that all fail; `t` is the time of entry, each operation takes 10 ms.
        /// Returns the times at which `Recover` was issued and the final step.
        fn fail_all(&mut self, t: u64, attempts: usize) -> (std::vec::Vec<u64>, Step) {
            let mut issued = std::vec::Vec::new();
            let mut now = t;
            let mut last = quiet(Action::None);
            let mut next = self.at(now, Event::Tick);
            for _ in 0..attempts {
                while let Action::WaitUntil(u) = next.action {
                    now = u;
                    next = self.at(now, Event::Tick);
                }
                assert_eq!(next.action, Action::Recover);
                issued.push(now);
                now += 10;
                next = self.at(now, Event::RecoverDone { ok: false });
                assert_eq!(next, quiet(Action::Restart));
                now += 10;
                last = self.at(now, Event::RestartDone { ok: false });
                next = last;
            }
            (issued, last)
        }
    }

    #[test]
    fn boot_starts_and_reaches_ok() {
        let mut r = Rig::new();
        assert_eq!(r.h.state(), State::Starting);
        assert_eq!(r.at(5, Event::Tick), quiet(Action::Start));
        assert_eq!(r.at(6, Event::Tick), quiet(Action::None));
        assert_eq!(r.at(7, Event::StartOk), quiet(Action::None));
        assert_eq!(r.h.state(), State::Ok);
        assert!(r.h.counters().present);
        assert_eq!(r.h.score(), 0);
    }

    #[test]
    fn start_retries_at_100_300_1000_then_absent() {
        let mut r = Rig::new();
        assert_eq!(r.at(0, Event::Tick), quiet(Action::Start));
        let f = Event::StartFailed { identified: false };
        assert_eq!(r.at(10, f), quiet(Action::WaitUntil(110)));
        assert_eq!(r.at(109, Event::Tick), quiet(Action::WaitUntil(110)));
        assert_eq!(r.at(110, Event::Tick), quiet(Action::Start));
        assert_eq!(r.at(120, f), quiet(Action::WaitUntil(420)));
        assert_eq!(r.at(419, Event::Tick), quiet(Action::WaitUntil(420)));
        assert_eq!(r.at(420, Event::Tick), quiet(Action::Start));
        assert_eq!(r.at(430, f), quiet(Action::WaitUntil(1430)));
        assert_eq!(r.at(1429, Event::Tick), quiet(Action::WaitUntil(1430)));
        assert_eq!(r.at(1430, Event::Tick), quiet(Action::Start));
        assert_eq!(r.at(1440, f), st(Action::Park, Some(Report::Absent)));
        assert_eq!(r.h.state(), State::Absent);
        assert!(!r.h.counters().present);
    }

    #[test]
    fn start_literal_reading_three_failed_starts_is_absent() {
        let limits = Limits {
            start_retries: 2,
            ..Limits::DESIGN
        };
        let mut r = Rig::with(limits);
        let f = Event::StartFailed { identified: false };
        r.at(0, Event::Tick);
        assert_eq!(r.at(0, f), quiet(Action::WaitUntil(100)));
        r.at(100, Event::Tick);
        assert_eq!(r.at(100, f), quiet(Action::WaitUntil(400)));
        r.at(400, Event::Tick);
        assert_eq!(r.at(400, f), st(Action::Park, Some(Report::Absent)));
    }

    #[test]
    fn start_ok_after_a_retry_and_present_is_sticky() {
        let mut r = Rig::new();
        r.at(0, Event::Tick);
        assert_eq!(
            r.at(0, Event::StartFailed { identified: true }),
            quiet(Action::WaitUntil(100))
        );
        assert!(r.h.counters().present);
        assert_eq!(r.at(100, Event::Tick), quiet(Action::Start));
        assert_eq!(
            r.at(100, Event::StartFailed { identified: false }),
            quiet(Action::WaitUntil(400))
        );
        assert!(r.h.counters().present);
        r.at(400, Event::Tick);
        r.at(400, Event::StartOk);
        assert_eq!(r.h.state(), State::Ok);
        assert!(r.h.counters().present);
    }

    #[test]
    fn start_failures_restart_count_after_success() {
        let mut r = Rig::new();
        r.at(0, Event::Tick);
        r.at(0, Event::StartFailed { identified: false });
        r.at(100, Event::Tick);
        r.at(100, Event::StartOk);
        r.at(200, Event::Restart);
        assert_eq!(r.at(200, Event::Tick), quiet(Action::None));
        assert_eq!(r.h.state(), State::Starting);
    }

    #[test]
    fn ok_to_degraded_on_first_error_and_back_after_three_good() {
        let mut r = Rig::booted();
        assert_eq!(r.at(1, Event::Good), quiet(Action::None));
        assert_eq!(r.h.score(), 0);
        assert_eq!(r.at(2, Event::Error(ErrorKind::Bus)), quiet(Action::None));
        assert_eq!(r.h.state(), State::Degraded);
        assert_eq!(r.h.score(), 3);
        r.at(3, Event::Good);
        r.at(4, Event::Good);
        assert_eq!(r.h.state(), State::Degraded);
        assert_eq!(r.h.score(), 1);
        assert_eq!(r.at(5, Event::Good), quiet(Action::None));
        assert_eq!(r.h.state(), State::Ok);
        assert_eq!(r.h.score(), 0);
        assert_eq!(r.h.counters().errors, 1);
        assert_eq!(r.h.counters().last_error, ErrorKind::Bus);
    }

    #[test]
    fn three_errors_in_a_row_enter_recovering() {
        let mut r = Rig::booted();
        assert_eq!(
            r.at(100, Event::Error(ErrorKind::Timeout)),
            quiet(Action::None)
        );
        assert_eq!(
            r.at(101, Event::Error(ErrorKind::Timeout)),
            quiet(Action::None)
        );
        assert_eq!(r.h.score(), 6);
        assert_eq!(
            r.at(102, Event::Error(ErrorKind::Timeout)),
            st(Action::WaitUntil(202), Some(Report::EnteredRecovering))
        );
        assert_eq!(r.h.state(), State::Recovering);
        assert_eq!(r.h.counters().errors, 3);
        assert_eq!(r.h.counters().timeouts, 3);
    }

    /// Feed `pattern` repeatedly for `periods` periods; true if Recovering was entered.
    fn reaches_recovering(pattern: &[Event], periods: usize) -> bool {
        let mut r = Rig::booted();
        let mut t = 1;
        for _ in 0..periods {
            for &e in pattern {
                t += 1;
                r.at(t, e);
                if r.h.state() == State::Recovering {
                    return true;
                }
            }
        }
        false
    }

    #[test]
    fn sustained_error_rate_just_above_25_percent_recovers_just_below_does_not() {
        let e = Event::Error(ErrorKind::Bus);
        let g = Event::Good;
        // 2 errors in 7 samples = 28.6 %: net +1 per period.
        assert!(reaches_recovering(&[e, g, e, g, g, g, g], 20));
        // exactly 25 % (1 in 4): net 0, bounded.
        assert!(!reaches_recovering(&[e, g, g, g], 1000));
        // 2 errors in 9 samples = 22.2 %: net -1 per period.
        assert!(!reaches_recovering(&[e, g, e, g, g, g, g, g, g], 1000));
    }

    #[test]
    fn power_on_reset_enters_recovering_at_once() {
        let mut r = Rig::booted();
        assert_eq!(
            r.at(50, Event::PowerOnReset),
            st(Action::WaitUntil(150), Some(Report::EnteredRecovering))
        );
        assert_eq!(r.h.state(), State::Recovering);
        assert_eq!(r.h.counters().last_error, ErrorKind::Por);
        let mut r = Rig::booted();
        r.at(1, Event::Error(ErrorKind::Bus));
        assert_eq!(r.h.state(), State::Degraded);
        assert_eq!(
            r.at(2, Event::PowerOnReset),
            st(Action::WaitUntil(102), Some(Report::EnteredRecovering))
        );
    }

    #[test]
    fn power_on_reset_while_starting_or_recovering_is_ignored() {
        let mut r = Rig::new();
        r.at(0, Event::Tick);
        assert_eq!(r.at(1, Event::PowerOnReset), quiet(Action::None));
        assert_eq!(r.h.state(), State::Starting);
        let mut r = Rig::booted();
        r.at(0, Event::PowerOnReset);
        assert_eq!(r.at(10, Event::PowerOnReset), quiet(Action::WaitUntil(100)));
    }

    #[test]
    fn verify_mismatch_reconfigure_ok() {
        let mut r = Rig::booted();
        assert_eq!(r.at(10, Event::VerifyMismatch), quiet(Action::Reconfigure));
        assert_eq!(r.h.state(), State::Degraded);
        assert_eq!(r.h.score(), 3);
        assert_eq!(r.h.counters().errors, 1);
        assert_eq!(r.h.counters().last_error, ErrorKind::ConfigMismatch);
        assert_eq!(r.at(11, Event::Tick), quiet(Action::None));
        assert_eq!(
            r.at(12, Event::ReconfigureDone { ok: true }),
            quiet(Action::None)
        );
        assert_eq!(r.h.counters().config_restored, 1);
        assert_eq!(r.h.state(), State::Degraded);
        r.at(13, Event::Good);
        r.at(14, Event::Good);
        r.at(15, Event::Good);
        assert_eq!(r.h.state(), State::Ok);
        // A reconfigure result nobody asked for is ignored.
        r.at(16, Event::ReconfigureDone { ok: true });
        assert_eq!(r.h.counters().config_restored, 1);
    }

    #[test]
    fn verify_mismatch_reconfigure_failed_enters_recovering() {
        let mut r = Rig::booted();
        assert_eq!(r.at(10, Event::VerifyMismatch), quiet(Action::Reconfigure));
        assert_eq!(
            r.at(20, Event::ReconfigureDone { ok: false }),
            st(Action::WaitUntil(120), Some(Report::EnteredRecovering))
        );
        assert_eq!(r.h.counters().config_restored, 0);
        assert_eq!(r.h.state(), State::Recovering);
    }

    #[test]
    fn verify_mismatch_that_reaches_the_threshold_goes_to_recovering() {
        let mut r = Rig::booted();
        r.at(1, Event::Error(ErrorKind::Bus));
        r.at(2, Event::Error(ErrorKind::Bus));
        assert_eq!(
            r.at(3, Event::VerifyMismatch),
            st(Action::WaitUntil(103), Some(Report::EnteredRecovering))
        );
    }

    #[test]
    fn recovery_back_off_is_100_300_1000_3000_then_failed() {
        let mut r = Rig::booted();
        assert_eq!(
            r.at(1000, Event::PowerOnReset),
            st(Action::WaitUntil(1100), Some(Report::EnteredRecovering))
        );
        assert_eq!(r.at(1099, Event::Tick), quiet(Action::WaitUntil(1100)));
        // Each operation takes 10 ms: the back-off counts from the failure of the previous attempt.
        let (issued, last) = r.fail_all(1100, 4);
        // attempt 1 at 1100; its failure at 1120; attempt 2 at 1120+300; failure 1440; +1000 = 2440; failure 2460; +3000 = 5460
        assert_eq!(issued, [1100, 1420, 2440, 5460]);
        assert_eq!(last, st(Action::Park, Some(Report::Failed)));
        assert_eq!(r.h.state(), State::Failed);
        assert_eq!(r.h.counters().attempts, 4);
        assert_eq!(r.h.counters().recoveries, 0);
    }

    #[test]
    fn recovery_waits_exactly_the_back_off_after_a_failure() {
        let mut r = Rig::booted();
        r.at(0, Event::PowerOnReset);
        r.at(100, Event::Tick);
        r.at(100, Event::RecoverDone { ok: false });
        assert_eq!(
            r.at(100, Event::RestartDone { ok: false }),
            quiet(Action::WaitUntil(400))
        );
        assert_eq!(r.at(399, Event::Tick), quiet(Action::WaitUntil(400)));
        assert_eq!(r.at(400, Event::Tick), quiet(Action::Recover));
        r.at(400, Event::RecoverDone { ok: false });
        assert_eq!(
            r.at(400, Event::RestartDone { ok: false }),
            quiet(Action::WaitUntil(1400))
        );
        assert_eq!(r.at(1400, Event::Tick), quiet(Action::Recover));
        r.at(1400, Event::RecoverDone { ok: false });
        assert_eq!(
            r.at(1400, Event::RestartDone { ok: false }),
            quiet(Action::WaitUntil(4400))
        );
        assert_eq!(r.at(4399, Event::Tick), quiet(Action::WaitUntil(4400)));
        assert_eq!(r.at(4400, Event::Tick), quiet(Action::Recover));
        r.at(4400, Event::RecoverDone { ok: false });
        assert_eq!(
            r.at(4400, Event::RestartDone { ok: false }),
            st(Action::Park, Some(Report::Failed))
        );
    }

    #[test]
    fn recover_ok_returns_to_ok() {
        let mut r = Rig::booted();
        r.at(0, Event::PowerOnReset);
        assert_eq!(r.at(100, Event::Tick), quiet(Action::Recover));
        assert_eq!(
            r.at(105, Event::RecoverDone { ok: true }),
            st(Action::None, Some(Report::Recovered))
        );
        assert_eq!(r.h.state(), State::Ok);
        assert_eq!(r.h.score(), 0);
        assert_eq!(r.h.counters().recoveries, 1);
        assert_eq!(r.h.counters().attempts, 1);
    }

    #[test]
    fn recover_fails_then_restart_succeeds() {
        let mut r = Rig::booted();
        r.at(0, Event::Error(ErrorKind::Bus));
        r.at(0, Event::Error(ErrorKind::Bus));
        r.at(0, Event::Error(ErrorKind::Bus));
        assert_eq!(r.at(100, Event::Tick), quiet(Action::Recover));
        assert_eq!(
            r.at(110, Event::RecoverDone { ok: false }),
            quiet(Action::Restart)
        );
        assert_eq!(r.at(111, Event::Tick), quiet(Action::None));
        assert_eq!(
            r.at(120, Event::RestartDone { ok: true }),
            st(Action::None, Some(Report::Recovered))
        );
        assert_eq!(r.h.state(), State::Ok);
        assert_eq!(r.h.score(), 0);
        assert_eq!(r.h.counters().recoveries, 1);
    }

    #[test]
    fn exactly_four_attempts_then_failed() {
        let mut r = Rig::booted();
        r.at(0, Event::PowerOnReset);
        let (issued, last) = r.fail_all(0, 4);
        assert_eq!(issued.len(), 4);
        assert_eq!(last.report, Some(Report::Failed));
        // No fifth attempt, whatever happens.
        for t in [10_000, 20_000, 1_000_000] {
            assert_eq!(r.at(t, Event::Tick), quiet(Action::Park));
        }
    }

    #[test]
    fn errors_and_samples_are_ignored_while_recovering() {
        let mut r = Rig::booted();
        r.at(0, Event::PowerOnReset);
        let before = *r.h.counters();
        assert_eq!(
            r.at(10, Event::Error(ErrorKind::Bus)),
            quiet(Action::WaitUntil(100))
        );
        assert_eq!(r.at(11, Event::Good), quiet(Action::WaitUntil(100)));
        assert_eq!(
            r.at(12, Event::VerifyMismatch),
            quiet(Action::WaitUntil(100))
        );
        assert_eq!(*r.h.counters(), before);
        assert_eq!(r.h.score(), 0);
    }

    fn por_cycle(r: &mut Rig, t: u64) -> Step {
        let s = r.at(t, Event::PowerOnReset);
        if s.report == Some(Report::EnteredRecovering) {
            r.at(t + 100, Event::Tick);
            r.at(t + 101, Event::RecoverDone { ok: true });
        }
        s
    }

    #[test]
    fn flap_limit_fourth_entry_within_60_s_is_failed() {
        let mut r = Rig::booted();
        assert_eq!(por_cycle(&mut r, 0).report, Some(Report::EnteredRecovering));
        assert_eq!(
            por_cycle(&mut r, 20_000).report,
            Some(Report::EnteredRecovering)
        );
        assert_eq!(
            por_cycle(&mut r, 40_000).report,
            Some(Report::EnteredRecovering)
        );
        assert_eq!(
            r.at(60_000, Event::PowerOnReset),
            st(Action::Park, Some(Report::Failed))
        );
        assert_eq!(r.h.state(), State::Failed);
        assert_eq!(r.h.counters().attempts, 1);
    }

    #[test]
    fn flap_limit_fourth_entry_after_more_than_60_s_is_not_failed() {
        let mut r = Rig::booted();
        por_cycle(&mut r, 0);
        por_cycle(&mut r, 20_000);
        por_cycle(&mut r, 40_000);
        assert_eq!(
            por_cycle(&mut r, 60_001).report,
            Some(Report::EnteredRecovering)
        );
        assert_eq!(r.h.state(), State::Ok);
        // The window slides: the next entry compares against the one at 20 000.
        assert_eq!(por_cycle(&mut r, 70_000).report, Some(Report::Failed));
    }

    #[test]
    fn flap_counts_entries_not_recoveries_and_three_entries_are_fine() {
        let mut r = Rig::booted();
        for i in 0..3 {
            assert_eq!(
                por_cycle(&mut r, i * 1000).report,
                Some(Report::EnteredRecovering)
            );
        }
        assert_eq!(r.h.state(), State::Ok);
        assert_eq!(r.h.counters().recoveries, 3);
    }

    #[test]
    fn failed_and_absent_park_and_ignore_everything_but_restart() {
        let events = [
            Event::StartOk,
            Event::StartFailed { identified: true },
            Event::Good,
            Event::Error(ErrorKind::Bus),
            Event::VerifyMismatch,
            Event::PowerOnReset,
            Event::ReconfigureDone { ok: true },
            Event::RecoverDone { ok: true },
            Event::RestartDone { ok: true },
            Event::Overflow,
            Event::Tick,
        ];
        let failed = || {
            let mut r = Rig::booted();
            r.at(0, Event::PowerOnReset);
            r.fail_all(0, 4);
            r
        };
        let absent = || {
            let mut r = Rig::new();
            r.at(0, Event::Tick);
            for t in [0, 100, 400, 1400] {
                r.at(t, Event::Tick);
                r.at(t, Event::StartFailed { identified: false });
            }
            r
        };
        for (mut r, state) in [(failed(), State::Failed), (absent(), State::Absent)] {
            assert_eq!(r.h.state(), state);
            let before = *r.h.counters();
            for (i, &e) in events.iter().enumerate() {
                assert_eq!(r.at(100_000 + i as u64, e), quiet(Action::Park));
                assert_eq!(r.h.state(), state);
            }
            assert_eq!(*r.h.counters(), before);
        }
    }

    #[test]
    fn restart_from_every_state_behaves_like_boot() {
        // Starting
        let mut r = Rig::new();
        r.at(0, Event::Tick);
        r.at(0, Event::StartFailed { identified: true });
        assert_eq!(r.at(5, Event::Restart), quiet(Action::Start));
        assert_eq!(r.h.state(), State::Starting);
        // The start retries are fresh: 3 retries again, from 100 ms.
        assert_eq!(
            r.at(6, Event::StartFailed { identified: false }),
            quiet(Action::WaitUntil(106))
        );
        // Ok
        let mut r = Rig::booted();
        assert_eq!(r.at(5, Event::Restart), quiet(Action::Start));
        assert_eq!(r.h.state(), State::Starting);
        // Degraded
        let mut r = Rig::booted();
        r.at(1, Event::Error(ErrorKind::Bus));
        assert_eq!(r.h.score(), 3);
        assert_eq!(r.at(5, Event::Restart), quiet(Action::Start));
        assert_eq!(r.h.state(), State::Starting);
        assert_eq!(r.h.score(), 0);
        // Recovering
        let mut r = Rig::booted();
        r.at(0, Event::PowerOnReset);
        assert_eq!(r.at(5, Event::Restart), quiet(Action::Start));
        assert_eq!(r.h.state(), State::Starting);
        // a late result of the abandoned recovery is ignored
        assert_eq!(
            r.at(6, Event::RecoverDone { ok: true }),
            quiet(Action::None)
        );
        assert_eq!(r.h.state(), State::Starting);
        // Failed
        let mut r = Rig::booted();
        r.at(0, Event::PowerOnReset);
        r.fail_all(0, 4);
        assert_eq!(r.h.state(), State::Failed);
        assert_eq!(r.at(900_000, Event::Restart), quiet(Action::Start));
        assert_eq!(r.at(900_001, Event::StartOk), quiet(Action::None));
        assert_eq!(r.h.state(), State::Ok);
        assert_eq!(r.h.score(), 0);
        // Absent
        let mut r = Rig::new();
        r.at(0, Event::Tick);
        for t in [0, 100, 400, 1400] {
            r.at(t, Event::Tick);
            r.at(t, Event::StartFailed { identified: false });
        }
        assert_eq!(r.h.state(), State::Absent);
        assert_eq!(r.at(5000, Event::Restart), quiet(Action::Start));
        assert_eq!(r.h.state(), State::Starting);
        assert_eq!(
            r.at(5000, Event::StartFailed { identified: false }),
            quiet(Action::WaitUntil(5100))
        );
    }

    #[test]
    fn restart_clears_flap_history_and_keeps_counters() {
        let mut r = Rig::booted();
        por_cycle(&mut r, 0);
        por_cycle(&mut r, 1000);
        por_cycle(&mut r, 2000);
        let counters = *r.h.counters();
        r.at(3000, Event::Restart);
        r.at(3000, Event::Tick);
        r.at(3000, Event::StartOk);
        assert_eq!(
            por_cycle(&mut r, 4000).report,
            Some(Report::EnteredRecovering)
        );
        assert_eq!(r.h.counters().recoveries, counters.recoveries + 1);
        assert!(r.h.counters().errors >= counters.errors);
    }

    #[test]
    fn overflow_is_a_counter_only() {
        let mut r = Rig::booted();
        for t in 0..5 {
            assert_eq!(r.at(t, Event::Overflow), quiet(Action::None));
        }
        assert_eq!(r.h.counters().overflows, 5);
        assert_eq!(r.h.counters().errors, 0);
        assert_eq!(r.h.score(), 0);
        assert_eq!(r.h.state(), State::Ok);
    }

    #[test]
    fn error_kinds_are_recorded_and_timeouts_counted() {
        let mut r = Rig::booted();
        r.at(0, Event::Error(ErrorKind::IdMismatch));
        assert_eq!(r.h.counters().last_error, ErrorKind::IdMismatch);
        r.at(0, Event::Error(ErrorKind::Timeout));
        assert_eq!(r.h.counters().last_error, ErrorKind::Timeout);
        assert_eq!(r.h.counters().errors, 2);
        assert_eq!(r.h.counters().timeouts, 1);
    }

    #[test]
    fn deadlines_survive_huge_times() {
        let mut r = Rig::new();
        r.at(u64::MAX - 50, Event::Tick);
        assert_eq!(
            r.at(u64::MAX - 50, Event::StartFailed { identified: false }),
            quiet(Action::WaitUntil(u64::MAX))
        );
        assert_eq!(r.at(u64::MAX, Event::Tick), quiet(Action::Start));
    }

    // ----- the property test (invariants I2.1 and I2.2) -----

    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x
        }

        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
    }

    #[derive(Debug, PartialEq, Eq)]
    enum Violation {
        /// I2.1
        TooManyAttempts {
            seq: u64,
            step: usize,
            recovers: u32,
        },
        /// I2.2
        LeftParked {
            seq: u64,
            step: usize,
            from: State,
            to: State,
        },
        NotParkedAction {
            seq: u64,
            step: usize,
            action: Action,
        },
        ScoreOutOfRange {
            seq: u64,
            step: usize,
            score: u8,
        },
        CounterDecreased {
            seq: u64,
            step: usize,
        },
        Inconsistent {
            seq: u64,
            step: usize,
            what: &'static str,
        },
    }

    #[derive(Default, Debug)]
    struct Coverage {
        failed: u64,
        absent: u64,
        recovered: u64,
        flap_failed: u64,
        restarts_from_parked: u64,
        steps: u64,
    }

    fn random_event(rng: &mut Rng, last: Action) -> Event {
        let kinds = [
            ErrorKind::None,
            ErrorKind::Bus,
            ErrorKind::Timeout,
            ErrorKind::IdMismatch,
            ErrorKind::ConfigMismatch,
            ErrorKind::Por,
            ErrorKind::Stuck,
        ];
        // Mostly answer the last action, so that episodes run to their end.
        if rng.below(100) < 70 {
            let ok = rng.below(100) < 30;
            match last {
                Action::Start => {
                    return if ok {
                        Event::StartOk
                    } else {
                        Event::StartFailed {
                            identified: rng.below(2) == 0,
                        }
                    };
                }
                Action::Reconfigure => return Event::ReconfigureDone { ok },
                Action::Recover => return Event::RecoverDone { ok },
                Action::Restart => return Event::RestartDone { ok },
                Action::WaitUntil(_) => return Event::Tick,
                Action::None | Action::Park => {}
            }
        }
        let ok = rng.below(2) == 0;
        match rng.below(200) {
            0..=2 => Event::Restart,
            3..=70 => Event::Good,
            71..=120 => Event::Error(kinds[rng.below(7) as usize]),
            121..=130 => Event::VerifyMismatch,
            131..=138 => Event::PowerOnReset,
            139..=146 => Event::Overflow,
            147..=160 => Event::Tick,
            161..=165 => Event::StartOk,
            166..=170 => Event::StartFailed { identified: ok },
            171..=180 => Event::ReconfigureDone { ok },
            181..=190 => Event::RecoverDone { ok },
            _ => Event::RestartDone { ok },
        }
    }

    fn time_step(rng: &mut Rng) -> u64 {
        match rng.below(100) {
            0..=49 => rng.below(3),
            50..=79 => rng.below(200),
            80..=94 => rng.below(2000),
            _ => rng.below(100_000),
        }
    }

    fn counters_array(c: &Counters) -> [u32; 5] {
        [
            c.errors,
            c.timeouts,
            c.recoveries,
            c.config_restored,
            c.overflows,
        ]
    }

    /// Run `seqs` random sequences of `len` events through `limits` and check the design's
    /// invariants (against the design's numbers, not against `limits`).
    fn check(limits: Limits, seed: u64, seqs: u64, len: usize) -> Result<Coverage, Violation> {
        let d = Limits::DESIGN;
        let max_score = d.threshold - 1 + d.error_weight;
        let mut cov = Coverage::default();
        for seq in 0..seqs {
            let mut rng = Rng(seed ^ seq.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
            let mut h = Health::new(limits);
            let mut now = rng.below(1_000_000);
            let mut last = Action::None;
            let mut recovers = 0u32;
            let mut restarts = 0u32;
            for step in 0..len {
                now += time_step(&mut rng);
                let event = random_event(&mut rng, last);
                let prev_state = h.state();
                let prev = counters_array(h.counters());
                let out = h.step(event, now);
                cov.steps += 1;
                let state = h.state();
                let was_parked = matches!(prev_state, State::Failed | State::Absent);
                if was_parked {
                    if event == Event::Restart {
                        cov.restarts_from_parked += 1;
                        if state != State::Starting || out.action != Action::Start {
                            return Err(Violation::LeftParked {
                                seq,
                                step,
                                from: prev_state,
                                to: state,
                            });
                        }
                    } else {
                        // I2.2
                        if state != prev_state {
                            return Err(Violation::LeftParked {
                                seq,
                                step,
                                from: prev_state,
                                to: state,
                            });
                        }
                        if out.action != Action::Park || out.report.is_some() {
                            return Err(Violation::NotParkedAction {
                                seq,
                                step,
                                action: out.action,
                            });
                        }
                    }
                }
                if out.action == Action::Park && !matches!(state, State::Failed | State::Absent) {
                    return Err(Violation::Inconsistent {
                        seq,
                        step,
                        what: "park outside Failed/Absent",
                    });
                }
                if matches!(state, State::Failed | State::Absent) && out.action != Action::Park {
                    return Err(Violation::NotParkedAction {
                        seq,
                        step,
                        action: out.action,
                    });
                }
                // Episodes: counted from entering Recovering; a RESTART ends them.
                if event == Event::Restart
                    || (prev_state != State::Recovering && state == State::Recovering)
                {
                    recovers = 0;
                    restarts = 0;
                }
                match out.action {
                    Action::Recover => recovers += 1,
                    Action::Restart if state == State::Recovering => restarts += 1,
                    _ => {}
                }
                // I2.1
                if recovers > u32::from(d.max_attempts) || restarts > u32::from(d.max_attempts) {
                    return Err(Violation::TooManyAttempts {
                        seq,
                        step,
                        recovers,
                    });
                }
                if matches!(out.action, Action::Recover | Action::Restart)
                    && state != State::Recovering
                {
                    return Err(Violation::Inconsistent {
                        seq,
                        step,
                        what: "recover/restart outside Recovering",
                    });
                }
                if h.score() > max_score {
                    return Err(Violation::ScoreOutOfRange {
                        seq,
                        step,
                        score: h.score(),
                    });
                }
                let consistent = match state {
                    State::Ok => h.score() == 0,
                    State::Degraded => h.score() > 0 && h.score() < d.threshold,
                    _ => true,
                };
                if !consistent {
                    return Err(Violation::Inconsistent {
                        seq,
                        step,
                        what: "score does not fit the state",
                    });
                }
                let after = counters_array(h.counters());
                if after.iter().zip(prev.iter()).any(|(a, b)| a < b) {
                    return Err(Violation::CounterDecreased { seq, step });
                }
                if h.counters().timeouts > h.counters().errors {
                    return Err(Violation::Inconsistent {
                        seq,
                        step,
                        what: "timeouts > errors",
                    });
                }
                if h.counters().attempts > d.max_attempts {
                    return Err(Violation::TooManyAttempts {
                        seq,
                        step,
                        recovers: u32::from(h.counters().attempts),
                    });
                }
                if let Action::WaitUntil(t) = out.action
                    && t < now
                {
                    return Err(Violation::Inconsistent {
                        seq,
                        step,
                        what: "deadline in the past",
                    });
                }
                match out.report {
                    Some(Report::Failed) => {
                        cov.failed += 1;
                        if prev_state != State::Recovering {
                            cov.flap_failed += 1;
                        }
                    }
                    Some(Report::Absent) => cov.absent += 1,
                    Some(Report::Recovered) => cov.recovered += 1,
                    _ => {}
                }
                last = out.action;
            }
        }
        Ok(cov)
    }

    #[test]
    fn property_i2_1_and_i2_2_over_100000_random_sequences() {
        let t = std::time::Instant::now();
        let cov = check(Limits::DESIGN, 0x00C0_FFEE_D00D_1234, 100_000, 80).unwrap();
        std::eprintln!("property test: {:?} in {:?}", cov, t.elapsed());
        // The run must have reached the interesting states, or it proves nothing.
        assert!(cov.failed > 1000, "{cov:?}");
        assert!(cov.absent > 1000, "{cov:?}");
        assert!(cov.recovered > 1000, "{cov:?}");
        assert!(cov.flap_failed > 10, "{cov:?}");
        assert!(cov.restarts_from_parked > 1000, "{cov:?}");
    }

    #[test]
    fn negative_control_without_the_retry_cap_violates_i2_1() {
        let limits = Limits {
            max_attempts: 255,
            ..Limits::DESIGN
        };
        let result = check(limits, 0x00C0_FFEE_D00D_1234, 100_000, 80);
        assert!(
            matches!(result, Err(Violation::TooManyAttempts { .. })),
            "{result:?}"
        );
    }
}
