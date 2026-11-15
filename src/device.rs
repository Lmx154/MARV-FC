//! The device layer (design: decision-foundation, item 1; core 0): one task per device runs
//! [`run_device`] over its [`DeviceAdapter`]; one static registry row per device ([`Device`]) is
//! what the supervisor, the telemetry and SYS_STATUS read. One row so far: [`BARO`].

use core::cell::Cell;
use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_sync::watch::{Sender, Watch};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use fw_core::health::{Action, Counters, ErrorKind, Event, Health, Limits, Report, State};
use fw_core::sys_status::{DeviceRow, StatusInput, device_status};
use mavlink_device::dialect::MavSeverity;
use mavlink_device::marv::{MARV_DEVICE_STATUS_DATA, MarvDevice};

/// Floor of the data-ready deadline T = max(3 P, 10 ms).
const DEADLINE_FLOOR: Duration = Duration::from_millis(10);
/// Floor of the stuck threshold max(1 s, 20 P).
const STUCK_FLOOR: Duration = Duration::from_secs(1);
/// `verify()` between reads, about once per second.
const VERIFY_EVERY: Duration = Duration::from_secs(1);
/// Longest single wait of a device task: back-off and park waits are cut into slices so that
/// `progress` keeps moving and only a task held inside an await is seen as stuck.
const SLICE: Duration = Duration::from_millis(100);
/// At most one text per device per second.
const TEXT_EVERY: Duration = Duration::from_secs(1);
/// Readers of a device's latest sample.
const READERS: usize = 2;
const NOTICE_DEPTH: usize = 4;

/// Where a sample came from.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Source {
    Hardware,
    /// Reserved: HIL is not built (design item 7).
    #[allow(dead_code)]
    Hil,
}

/// The physical values of one sample (one variant per kind of device).
#[derive(Clone, Copy, PartialEq, defmt::Format)]
pub enum Reading {
    Baro {
        pressure_pa: f32,
        temperature_c: f32,
    },
}

/// One published sample: values, board time of the read in us, `seq` (+1 per publish), source.
#[derive(Clone, Copy, PartialEq, defmt::Format)]
pub struct Sample {
    pub reading: Reading,
    pub t_us: u64,
    pub seq: u32,
    pub source: Source,
}

/// What the adapter counts itself.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct AdapterCounters {
    /// Adapter operations that went to the bus.
    pub calls: u32,
}

/// What the one status read after a data-ready timeout found.
pub enum Pending {
    /// A sample is waiting: the edge was missed.
    Data,
    /// No sample: a deadline miss.
    Nothing,
    /// The device has been reset.
    PowerOnReset,
}

/// A failed start.
pub struct StartFailure {
    /// The device answered its identity before the start failed.
    pub identified: bool,
}

/// A failed `verify()`.
pub enum VerifyError {
    /// A register differs from the configuration.
    Mismatch,
    /// The check itself failed.
    Failed(ErrorKind),
}

/// The firmware side of one device, driven by [`run_device`]. Every method except
/// [`data_ready`](Self::data_ready) may use the bus and is always awaited to completion.
pub trait DeviceAdapter {
    /// Waits for the device's data-ready signal. No bus traffic: it may be dropped at any await.
    async fn data_ready(&mut self);
    /// Init and configure.
    async fn start(&mut self) -> Result<(), StartFailure>;
    /// Reads one sample.
    async fn acquire(&mut self) -> Result<Reading, ErrorKind>;
    /// The one status read after a data-ready timeout.
    async fn status(&mut self) -> Result<Pending, ErrorKind>;
    /// Checks the configuration registers, changing nothing.
    async fn verify(&mut self) -> Result<(), VerifyError>;
    /// Configures the running device with the adapter's configuration.
    async fn apply(&mut self) -> Result<(), ErrorKind>;
    /// Brings the bus and the device back in step without a reset.
    async fn recover(&mut self) -> Result<(), ErrorKind>;
    /// Bus clear where the bus has one, then init and configure.
    async fn restart(&mut self) -> Result<(), ErrorKind>;
    /// The adapter's own counters.
    fn counters(&self) -> AdapterCounters;
}

/// A device's health and counters as its task last wrote them.
#[derive(Clone, Copy)]
pub struct DeviceStatus {
    pub state: State,
    pub health: Counters,
    pub adapter: AdapterCounters,
    pub samples: u32,
    pub missed_edges: u32,
    pub last_sample_us: Option<u64>,
    pub max_gap_us: u64,
}

impl DeviceStatus {
    const INITIAL: Self = Self {
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
        adapter: AdapterCounters { calls: 0 },
        samples: 0,
        missed_edges: 0,
        last_sample_us: None,
        max_gap_us: 0,
    };
}

/// One registry row. `status` and `latest` are written only by the device's own task, `stuck` only
/// by the supervisor; `progress` is bumped by the task after every await.
pub struct Device {
    pub id: MarvDevice,
    pub name: &'static str,
    /// Nominal sample period P.
    pub period_us: u32,
    pub latest: Watch<CriticalSectionRawMutex, Sample, READERS>,
    status: Mutex<CriticalSectionRawMutex, Cell<DeviceStatus>>,
    /// Wakes the task for a request (no request exists yet).
    pub wake: Signal<CriticalSectionRawMutex, ()>,
    pub progress: AtomicU32,
    pub stuck: AtomicBool,
}

impl Device {
    pub const fn new(id: MarvDevice, name: &'static str, period_us: u32) -> Self {
        Self {
            id,
            name,
            period_us,
            latest: Watch::new(),
            status: Mutex::new(Cell::new(DeviceStatus::INITIAL)),
            wake: Signal::new(),
            progress: AtomicU32::new(0),
            stuck: AtomicBool::new(false),
        }
    }

    pub fn status(&self) -> DeviceStatus {
        self.status.lock(Cell::get)
    }

    fn set_status(&self, status: DeviceStatus) {
        self.status.lock(|cell| cell.set(status));
    }

    fn period(&self) -> Duration {
        Duration::from_micros(u64::from(self.period_us))
    }

    /// T = max(3 P, 10 ms).
    pub fn deadline(&self) -> Duration {
        (self.period() * 3).max(DEADLINE_FLOOR)
    }

    /// max(1 s, 20 P).
    pub fn stuck_after(&self) -> Duration {
        (self.period() * 20).max(STUCK_FLOOR)
    }

    fn mark(&self) {
        self.progress.fetch_add(1, Ordering::Relaxed);
        RUNNING.store(self.id as u8, Ordering::Relaxed);
    }

    /// The row for SYS_STATUS and the HEARTBEAT state.
    pub fn row(&self) -> DeviceRow {
        let status = self.status();
        DeviceRow {
            device: self.id,
            present: status.health.present,
            state: status.state,
            stuck: self.stuck.load(Ordering::Relaxed),
            locked: false,
        }
    }

    /// The device's `MARV_DEVICE_STATUS`.
    pub fn message(&self, now_us: u64) -> MARV_DEVICE_STATUS_DATA {
        let status = self.status();
        let ms = |us: u64| u32::try_from(us / 1000).unwrap_or(u32::MAX);
        device_status(&StatusInput {
            device: self.id,
            state: status.state,
            counters: status.health,
            stuck: self.stuck.load(Ordering::Relaxed),
            samples: status.samples,
            last_sample_age_ms: status.last_sample_us.map(|t| ms(now_us.saturating_sub(t))),
            hil: false,
            self_test: false,
            time_boot_ms: ms(now_us),
        })
    }
}

/// The number of registry rows.
pub const DEVICE_COUNT: usize = 1;
/// The barometer's row.
pub static BARO: Device = Device::new(MarvDevice::MARV_DEVICE_BARO, "BARO", crate::baro::PERIOD_US);
/// The registry.
pub static DEVICES: [&Device; DEVICE_COUNT] = [&BARO];
/// The id (`MARV_DEVICE`) of the device task that ran last.
pub static RUNNING: AtomicU8 = AtomicU8::new(0);

/// What the supervisor turns into a text.
#[derive(Clone, Copy)]
enum Notice {
    Report(Report),
    Stuck,
}

/// Device tasks -> supervisor: the health transitions to report.
static NOTICES: Channel<CriticalSectionRawMutex, (&'static Device, Notice), 8> = Channel::new();

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

pub fn state_name(state: State) -> &'static str {
    match state {
        State::Starting => "Starting",
        State::Ok => "Ok",
        State::Degraded => "Degraded",
        State::Recovering => "Recovering",
        State::Failed => "Failed",
        State::Absent => "Absent",
    }
}

pub fn error_name(error: ErrorKind) -> &'static str {
    match error {
        ErrorKind::None => "none",
        ErrorKind::Bus => "bus",
        ErrorKind::Timeout => "timeout",
        ErrorKind::IdMismatch => "id mismatch",
        ErrorKind::ConfigMismatch => "config mismatch",
        ErrorKind::Por => "power-on reset",
        ErrorKind::Stuck => "stuck",
    }
}

/// The device task's loop: steps the device's `Health` and carries out the action it returns.
/// Sampling is `select(with_timeout(T, data ready), wake)`; neither branch is a bus call, and every
/// bus call is awaited to completion outside any `select` or timeout (I1.1).
pub async fn run_device<A: DeviceAdapter>(
    dev: &'static Device,
    adapter: &mut A,
    limits: Limits,
) -> ! {
    let mut health = Health::new(limits);
    let mut status = DeviceStatus::INITIAL;
    let mut seq: u32 = 0;
    let mut last_verify = Instant::now();
    let mut last_failure_ms: u64 = 0;
    let sender = dev.latest.sender();
    let mut step = health.step(Event::Tick, now_ms());
    loop {
        let event = match step.action {
            Action::Start => match adapter.start().await {
                Ok(()) => Event::StartOk,
                Err(f) => Event::StartFailed {
                    identified: f.identified,
                },
            },
            Action::Reconfigure => Event::ReconfigureDone {
                ok: adapter.apply().await.is_ok(),
            },
            Action::Recover => {
                let now = now_ms();
                defmt::info!(
                    "[INFO] {=str}: recovery attempt {=u8} starts at {=u64} ms, {=u64} ms after the previous failure",
                    dev.name,
                    health.counters().attempts,
                    now,
                    now.saturating_sub(last_failure_ms)
                );
                Event::RecoverDone {
                    ok: adapter.recover().await.is_ok(),
                }
            }
            Action::Restart => Event::RestartDone {
                ok: adapter.restart().await.is_ok(),
            },
            Action::WaitUntil(until) => {
                sleep_until(dev, until).await;
                Event::Tick
            }
            Action::Park => {
                // No bus traffic; a request would arrive through `wake` (none exists yet).
                let _ = select(Timer::after(SLICE), dev.wake.wait()).await;
                Event::Tick
            }
            Action::None if matches!(health.state(), State::Ok | State::Degraded) => {
                if last_verify.elapsed() >= VERIFY_EVERY {
                    last_verify = Instant::now();
                    match adapter.verify().await {
                        Ok(()) => Event::Tick,
                        Err(VerifyError::Mismatch) => Event::VerifyMismatch,
                        Err(VerifyError::Failed(kind)) => Event::Error(kind),
                    }
                } else {
                    sample(dev, adapter, &mut status, &mut seq, &sender).await
                }
            }
            Action::None => {
                // Not reached: every operation's result is fed back at once.
                Timer::after(SLICE).await;
                Event::Tick
            }
        };
        dev.mark();

        let before = health.state();
        let now = now_ms();
        step = health.step(event, now);
        let after = health.state();
        if after != before {
            defmt::info!(
                "[INFO] {=str}: {=str} -> {=str} at {=u64} ms (last error {=str})",
                dev.name,
                state_name(before),
                state_name(after),
                now,
                error_name(health.counters().last_error)
            );
        }
        if let Some(report) = step.report {
            if report == Report::EnteredRecovering {
                last_failure_ms = now;
            }
            if NOTICES.try_send((dev, Notice::Report(report))).is_err() {
                defmt::warn!("[INFO] {=str}: notice queue full", dev.name);
            }
        }
        if event == (Event::RestartDone { ok: false }) {
            last_failure_ms = now;
            defmt::info!(
                "[INFO] {=str}: recovery attempt {=u8} failed at {=u64} ms",
                dev.name,
                health.counters().attempts,
                now
            );
        }
        status.state = after;
        status.health = *health.counters();
        status.adapter = adapter.counters();
        dev.set_status(status);
    }
}

/// One data-ready wait and what follows it.
async fn sample<A: DeviceAdapter>(
    dev: &'static Device,
    adapter: &mut A,
    status: &mut DeviceStatus,
    seq: &mut u32,
    sender: &Sender<'static, CriticalSectionRawMutex, Sample, READERS>,
) -> Event {
    match select(
        with_timeout(dev.deadline(), adapter.data_ready()),
        dev.wake.wait(),
    )
    .await
    {
        Either::First(Ok(())) => {
            dev.mark();
            read(adapter, status, seq, sender).await
        }
        Either::First(Err(_)) => {
            dev.mark();
            let pending = adapter.status().await;
            dev.mark();
            match pending {
                Ok(Pending::Data) => {
                    status.missed_edges = status.missed_edges.wrapping_add(1);
                    read(adapter, status, seq, sender).await
                }
                Ok(Pending::Nothing) => Event::Error(ErrorKind::Timeout),
                Ok(Pending::PowerOnReset) => Event::PowerOnReset,
                Err(kind) => Event::Error(kind),
            }
        }
        // Requests (APPLY, RESTART, SELF_TEST) are not built yet.
        Either::Second(()) => Event::Tick,
    }
}

/// Reads and publishes one sample.
async fn read<A: DeviceAdapter>(
    adapter: &mut A,
    status: &mut DeviceStatus,
    seq: &mut u32,
    sender: &Sender<'static, CriticalSectionRawMutex, Sample, READERS>,
) -> Event {
    let t_us = Instant::now().as_micros();
    match adapter.acquire().await {
        Ok(reading) => {
            if let Some(last) = status.last_sample_us {
                status.max_gap_us = status.max_gap_us.max(t_us.saturating_sub(last));
            }
            status.last_sample_us = Some(t_us);
            sender.send(Sample {
                reading,
                t_us,
                seq: *seq,
                source: Source::Hardware,
            });
            *seq = seq.wrapping_add(1);
            status.samples = status.samples.wrapping_add(1);
            Event::Good
        }
        Err(kind) => Event::Error(kind),
    }
}

async fn sleep_until(dev: &Device, until_ms: u64) {
    let until = Instant::from_millis(until_ms);
    loop {
        let now = Instant::now();
        if now >= until {
            return;
        }
        Timer::at(until.min(now + SLICE)).await;
        dev.mark();
    }
}

/// The supervisor's view of one device.
struct Watcher {
    progress: u32,
    progress_at: Instant,
    notices: [Option<Notice>; NOTICE_DEPTH],
    last_text: Option<Instant>,
    stat_samples: u32,
}

/// The device part of the supervisor: stuck detection, the transition texts (at most one per
/// device per second, none for Ok/Degraded) and the `[STAT]` lines.
pub struct Supervision {
    watchers: [Watcher; DEVICE_COUNT],
    stat_at: Instant,
}

impl Supervision {
    pub fn new(now: Instant) -> Self {
        Self {
            watchers: core::array::from_fn(|_| Watcher {
                progress: 0,
                progress_at: now,
                notices: [None; NOTICE_DEPTH],
                last_text: None,
                stat_samples: 0,
            }),
            stat_at: now,
        }
    }

    /// Every supervisor tick.
    pub fn tick(&mut self, now: Instant) {
        while let Ok((dev, notice)) = NOTICES.try_receive() {
            if let Some(w) = DEVICES
                .iter()
                .zip(self.watchers.iter_mut())
                .find_map(|(d, w)| core::ptr::eq(*d, dev).then_some(w))
            {
                queue(w, dev, notice);
            }
        }
        for (dev, w) in DEVICES.iter().zip(self.watchers.iter_mut()) {
            let progress = dev.progress.load(Ordering::Relaxed);
            let stuck = dev.stuck.load(Ordering::Relaxed);
            if progress != w.progress {
                w.progress = progress;
                w.progress_at = now;
                if stuck {
                    dev.stuck.store(false, Ordering::Relaxed);
                    defmt::info!(
                        "[INFO] {=str}: progress again at {=u64} ms",
                        dev.name,
                        now.as_millis()
                    );
                }
            } else if !stuck && now - w.progress_at >= dev.stuck_after() {
                dev.stuck.store(true, Ordering::Relaxed);
                defmt::error!(
                    "[INFO] {=str}: stuck at {=u64} ms (no progress for {=u64} ms)",
                    dev.name,
                    now.as_millis(),
                    (now - w.progress_at).as_millis()
                );
                queue(w, dev, Notice::Stuck);
            }
            let due = w.last_text.is_none_or(|t| now - t >= TEXT_EVERY);
            if due && let Some(notice) = w.notices[0] {
                w.notices.rotate_left(1);
                w.notices[NOTICE_DEPTH - 1] = None;
                w.last_text = Some(now);
                send_text(dev, notice);
            }
        }
    }

    /// Every 10 s: one `[STAT]` line per device.
    pub fn stat(&mut self, now: Instant) {
        let window_us = (now - self.stat_at).as_micros().max(1);
        self.stat_at = now;
        for (dev, w) in DEVICES.iter().zip(self.watchers.iter_mut()) {
            let s = dev.status();
            let rate = s.samples.wrapping_sub(w.stat_samples) as f32 * 1e6 / window_us as f32;
            w.stat_samples = s.samples;
            let latest = dev.latest.try_get();
            let (seq, age_ms) = latest.map_or((u32::MAX, u64::MAX), |l| {
                (l.seq, now.as_micros().saturating_sub(l.t_us) / 1000)
            });
            defmt::info!(
                "[STAT] {=str}: {=str}{=str}, samples {=u32} ({=f32}/s over the last 10 s), seq {=u32}, errors {=u32}, timeouts {=u32}, missed edges {=u32}, recoveries {=u32}, config restored {=u32}, attempts {=u8}, largest gap {=u64} us, adapter calls {=u32}, last {} age {=u64} ms",
                dev.name,
                state_name(s.state),
                if dev.stuck.load(Ordering::Relaxed) {
                    " STUCK"
                } else {
                    ""
                },
                s.samples,
                rate,
                seq,
                s.health.errors,
                s.health.timeouts,
                s.missed_edges,
                s.health.recoveries,
                s.health.config_restored,
                s.health.attempts,
                s.max_gap_us,
                s.adapter.calls,
                latest.map(|l| l.reading),
                age_ms
            );
        }
    }
}

fn queue(w: &mut Watcher, dev: &Device, notice: Notice) {
    match w.notices.iter_mut().find(|n| n.is_none()) {
        Some(slot) => *slot = Some(notice),
        None => defmt::warn!("[INFO] {=str}: text backlog full, dropped one", dev.name),
    }
}

fn send_text(dev: &Device, notice: Notice) {
    let (severity, what) = match notice {
        Notice::Report(Report::EnteredRecovering) => {
            (MavSeverity::MAV_SEVERITY_WARNING, "recovering")
        }
        Notice::Report(Report::Recovered) => (MavSeverity::MAV_SEVERITY_NOTICE, "recovered"),
        Notice::Report(Report::Failed) => (MavSeverity::MAV_SEVERITY_ERROR, "failed"),
        Notice::Report(Report::Absent) => (MavSeverity::MAV_SEVERITY_ERROR, "absent"),
        Notice::Stuck => (MavSeverity::MAV_SEVERITY_ERROR, "stuck"),
    };
    let error = error_name(dev.status().health.last_error);
    crate::text_fmt(severity, format_args!("{}: {} ({})", dev.name, what, error));
    defmt::info!(
        "[INFO] text {=str}: {=str}: {=str} ({=str})",
        severity_name(severity),
        dev.name,
        what,
        error
    );
}

fn severity_name(severity: MavSeverity) -> &'static str {
    match severity {
        MavSeverity::MAV_SEVERITY_WARNING => "WARNING",
        MavSeverity::MAV_SEVERITY_NOTICE => "NOTICE",
        MavSeverity::MAV_SEVERITY_ERROR => "ERROR",
        _ => "other",
    }
}

/// A STATUSTEXT body of at most 50 bytes; longer text is cut.
#[derive(Clone, Copy)]
pub struct TextLine {
    len: usize,
    bytes: [u8; 50],
}

impl TextLine {
    pub fn new(args: fmt::Arguments<'_>) -> Self {
        let mut line = Self {
            len: 0,
            bytes: [0; 50],
        };
        let _ = fmt::write(&mut line, args);
        line
    }

    pub fn as_str(&self) -> &str {
        let bytes = self.bytes.get(..self.len).unwrap_or(&[]);
        core::str::from_utf8(bytes).unwrap_or("")
    }
}

impl fmt::Write for TextLine {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for c in s.chars() {
            let mut buf = [0; 4];
            let encoded = c.encode_utf8(&mut buf).as_bytes();
            let end = self.len + encoded.len();
            let Some(dst) = self.bytes.get_mut(self.len..end) else {
                return Ok(());
            };
            dst.copy_from_slice(encoded);
            self.len = end;
        }
        Ok(())
    }
}
