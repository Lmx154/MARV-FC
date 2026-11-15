//! The device layer (design: decision-foundation, item 1; core 0): one task per device runs
//! [`run_device`] over its [`DeviceAdapter`]; one static registry row per device ([`Device`]) is
//! what the supervisor, the telemetry and SYS_STATUS read. A row is backed either by a core-0 task
//! ([`BARO`]) or by a flight-core task's cells ([`IMU1`], `fw_flight::imu`): the supervisor copies
//! those into the row every tick, kicks CHECK when the row's last sample is older than T, and logs
//! the flight core's events, so the rows look the same to every reader.

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
use fw_flight::imu::{ImuCells, TRANSITIONS};
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
    Imu {
        gyro_rads: [f32; 3],
        accel_ms2: [f32; 3],
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
    ///
    /// The loop does not take `Ok` as "started": it then waits for one fresh sample (see
    /// [`recover`](Self::recover)); if none comes the start has failed, with `identified: true`.
    async fn start(&mut self) -> Result<(), StartFailure>;
    /// Reads one sample. `Err(ErrorKind::Por)` means the device reported a power-on reset: the loop
    /// feeds it to `Health` as a power-on-reset event, not as an error event.
    async fn acquire(&mut self) -> Result<Reading, ErrorKind>;
    /// The one status read after a data-ready timeout.
    async fn status(&mut self) -> Result<Pending, ErrorKind>;
    /// Checks the configuration registers, changing nothing.
    async fn verify(&mut self) -> Result<(), VerifyError>;
    /// Configures the running device with the adapter's configuration.
    async fn apply(&mut self) -> Result<(), ErrorKind>;
    /// Brings the bus and the device back in step without a reset.
    ///
    /// Contract (also of [`restart`](Self::restart)): `Ok` only says the bus and the device answer. The
    /// loop then waits for ONE fresh sample, `with_timeout(T, data_ready())` and one `acquire` awaited
    /// to completion, and only if that works does it report success to `Health`. A timeout or a failed
    /// `acquire` counts as if the adapter had returned an error: a failed `recover` goes on to
    /// `restart`, a failed `restart` ends the attempt.
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

/// One registry row. `status` and `latest` are written only by the device's own task (by the
/// supervisor from the cells for a flight-core row), `stuck` only by the supervisor; `progress` is
/// bumped by the task after every await.
pub struct Device {
    pub id: MarvDevice,
    pub name: &'static str,
    /// Nominal sample period P in microseconds; it follows the configured output data rate.
    period_us: AtomicU32,
    pub latest: Watch<CriticalSectionRawMutex, Sample, READERS>,
    status: Mutex<CriticalSectionRawMutex, Cell<DeviceStatus>>,
    /// Wakes the task for a request.
    pub wake: Signal<CriticalSectionRawMutex, ()>,
    pub progress: AtomicU32,
    pub stuck: AtomicBool,
    /// The requests raised for the task (`REQ_APPLY`, `REQ_RESTART`); the task clears each as it takes it.
    request: AtomicU8,
    /// Restart requests raised since boot: the number of the latest request.
    restarts: AtomicU32,
    /// The outcome of the latest restart the task has finished: request number << 1, bit 0 set when
    /// the device reached Ok, clear when it reached Failed or Absent (0 = none yet; numbers start at 1).
    restart_outcome: AtomicU32,
    /// The flight-core task's cells, for a row backed by the flight core.
    flight: Option<&'static ImuCells>,
}

/// Request bit: apply the stored settings to the running device.
const REQ_APPLY: u8 = 1;
/// Request bit: restart the device, from any state.
const REQ_RESTART: u8 = 2;

impl Device {
    pub const fn new(id: MarvDevice, name: &'static str, period_us: u32) -> Self {
        Self {
            id,
            name,
            period_us: AtomicU32::new(period_us),
            latest: Watch::new(),
            status: Mutex::new(Cell::new(DeviceStatus::INITIAL)),
            wake: Signal::new(),
            progress: AtomicU32::new(0),
            stuck: AtomicBool::new(false),
            request: AtomicU8::new(0),
            restarts: AtomicU32::new(0),
            restart_outcome: AtomicU32::new(0),
            flight: None,
        }
    }

    /// A row backed by a flight-core task's `cells`.
    pub const fn flight(
        id: MarvDevice,
        name: &'static str,
        period_us: u32,
        cells: &'static ImuCells,
    ) -> Self {
        let mut row = Self::new(id, name, period_us);
        row.flight = Some(cells);
        row
    }

    /// Sets the nominal period P (the task does, when it applies a configuration).
    pub fn set_period_us(&self, us: u32) {
        self.period_us.store(us, Ordering::Relaxed);
    }

    pub fn period_us(&self) -> u32 {
        self.period_us.load(Ordering::Relaxed)
    }

    /// Raises the device's APPLY request and wakes its task (a flight-core row has no APPLY yet).
    pub fn request_apply(&self) {
        if self.flight.is_some() {
            return;
        }
        self.request.fetch_or(REQ_APPLY, Ordering::AcqRel);
        self.wake.signal(());
    }

    /// Raises the device's RESTART request and wakes its task. Returns the request's number
    /// (1, 2, ...): [`restart_outcome`](Self::restart_outcome) reports the outcome of that request.
    pub fn request_restart(&self) -> u32 {
        let number = self.restarts.fetch_add(1, Ordering::AcqRel).wrapping_add(1);
        if let Some(cells) = self.flight {
            cells.request_restart(number);
            return number;
        }
        self.request.fetch_or(REQ_RESTART, Ordering::AcqRel);
        self.wake.signal(());
        number
    }

    /// Restart requests raised since boot.
    pub fn restarts(&self) -> u32 {
        self.restarts.load(Ordering::Acquire)
    }

    /// The outcome of restart request `number`, once its restart has finished: `Some(true)` the
    /// device reached Ok, `Some(false)` Failed or Absent; `None` while it is not decided. The outcome
    /// of any other request is never reported as this one's.
    pub fn restart_outcome(&self, number: u32) -> Option<bool> {
        let outcome = self.restart_outcome.load(Ordering::Acquire);
        (outcome >> 1 == number).then_some(outcome & 1 == 1)
    }

    fn take_request(&self, bit: u8) -> bool {
        self.request.fetch_and(!bit, Ordering::AcqRel) & bit != 0
    }

    fn requested(&self, bit: u8) -> bool {
        self.request.load(Ordering::Acquire) & bit != 0
    }

    pub fn status(&self) -> DeviceStatus {
        self.status.lock(Cell::get)
    }

    fn set_status(&self, status: DeviceStatus) {
        self.status.lock(|cell| cell.set(status));
    }

    fn period(&self) -> Duration {
        Duration::from_micros(u64::from(self.period_us()))
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
pub const DEVICE_COUNT: usize = 2;
/// IMU1's row, backed by the flight core.
pub static IMU1: Device = Device::flight(
    MarvDevice::MARV_DEVICE_IMU1,
    "IMU1",
    crate::imu1::PERIOD_US,
    &crate::imu1::CELLS,
);
/// The barometer's row.
pub static BARO: Device = Device::new(MarvDevice::MARV_DEVICE_BARO, "BARO", crate::baro::PERIOD_US);
/// The registry.
pub static DEVICES: [&Device; DEVICE_COUNT] = [&IMU1, &BARO];

/// The row of device `id`.
pub fn find(id: MarvDevice) -> Option<&'static Device> {
    DEVICES.iter().copied().find(|d| d.id == id)
}

/// Raises the APPLY request of every device whose bit (bit = `MARV_DEVICE` id) is set in `mask`.
pub fn raise_apply(mask: u32) {
    for dev in DEVICES {
        if mask & (1 << (dev.id as u8)) != 0 {
            dev.request_apply();
        }
    }
}

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
    // The number of the restart request being carried out, until its outcome is published.
    let mut restart_run: Option<u32> = None;
    let sender = dev.latest.sender();
    let mut step = health.step(Event::Tick, now_ms());
    loop {
        if dev.take_request(REQ_RESTART) {
            restart_run = Some(dev.restarts());
            let (before, now) = (health.state(), now_ms());
            step = health.step(Event::Restart, now);
            defmt::info!(
                "[INFO] {=str}: restart request {=u32} taken at {=u64} ms: {=str} -> {=str}",
                dev.name,
                dev.restarts(),
                now,
                state_name(before),
                state_name(health.state())
            );
            status.state = health.state();
            status.health = *health.counters();
            dev.set_status(status);
        }
        // A start or restart reads the stored settings itself: it carries out an APPLY request.
        if matches!(step.action, Action::Start | Action::Restart) {
            dev.take_request(REQ_APPLY);
        }
        let event = match step.action {
            Action::Start => match adapter.start().await {
                Ok(()) if fresh_sample(dev, adapter, &mut status, &mut seq, &sender).await => {
                    Event::StartOk
                }
                Ok(()) => Event::StartFailed { identified: true },
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
                let ok = adapter.recover().await.is_ok()
                    && fresh_sample(dev, adapter, &mut status, &mut seq, &sender).await;
                Event::RecoverDone { ok }
            }
            Action::Restart => {
                let ok = adapter.restart().await.is_ok()
                    && fresh_sample(dev, adapter, &mut status, &mut seq, &sender).await;
                Event::RestartDone { ok }
            }
            Action::WaitUntil(until) => {
                sleep_until(dev, until).await;
                Event::Tick
            }
            Action::Park => {
                // No bus traffic; a request arrives through `wake`.
                let _ = select(Timer::after(SLICE), dev.wake.wait()).await;
                Event::Tick
            }
            Action::None if matches!(health.state(), State::Ok | State::Degraded) => {
                if dev.take_request(REQ_APPLY) {
                    match adapter.apply().await {
                        Ok(()) => Event::Tick,
                        Err(kind) => Event::Error(kind),
                    }
                } else if last_verify.elapsed() >= VERIFY_EVERY {
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
        if let Some(number) = restart_run
            && !dev.requested(REQ_RESTART)
        {
            let ok = match after {
                State::Ok => Some(1),
                State::Failed | State::Absent => Some(0),
                _ => None,
            };
            if let Some(ok) = ok {
                dev.restart_outcome
                    .store(number << 1 | ok, Ordering::Release);
                restart_run = None;
            }
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
        // A request (APPLY, RESTART) is taken at the top of the loop.
        Either::Second(()) => Event::Tick,
    }
}

/// After a start, `recover` or `restart` that returned `Ok`: waits for one fresh sample (a timeout
/// around the data-ready wait, which is no bus call, then one `acquire` awaited to completion).
async fn fresh_sample<A: DeviceAdapter>(
    dev: &'static Device,
    adapter: &mut A,
    status: &mut DeviceStatus,
    seq: &mut u32,
    sender: &Sender<'static, CriticalSectionRawMutex, Sample, READERS>,
) -> bool {
    let ready = with_timeout(dev.deadline(), adapter.data_ready())
        .await
        .is_ok();
    dev.mark();
    ready && matches!(read(adapter, status, seq, sender).await, Event::Good)
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
        Err(ErrorKind::Por) => Event::PowerOnReset,
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
        if dev.requested(REQ_RESTART) {
            return;
        }
    }
}

/// The supervisor's view of one device.
struct Watcher {
    progress: u32,
    progress_at: Instant,
    notices: [Option<Notice>; NOTICE_DEPTH],
    last_text: Option<Instant>,
    stat_samples: u32,
    flight: FlightSeen,
}

/// What the supervisor last saw in a flight-core row's cells.
#[derive(Clone, Copy, Default)]
struct FlightSeen {
    seq: Option<u32>,
    transitions: u32,
    restart_taken: u32,
    attempt_ms: u64,
    attempt_failed_ms: u64,
    missed_edges: u32,
    inject: u32,
    delays_after_start: u32,
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
                flight: FlightSeen::default(),
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
            mirror(dev, w, now);
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
                "[STAT] {=str}: {=str}{=str}, samples {=u32} ({=f32}/s over the last 10 s), seq {=u32}, errors {=u32}, timeouts {=u32}, missed edges {=u32}, overflows {=u32}, recoveries {=u32}, config restored {=u32}, attempts {=u8}, largest gap {=u64} us, adapter calls {=u32}, last {} age {=u64} ms",
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
                s.health.overflows,
                s.health.recoveries,
                s.health.config_restored,
                s.health.attempts,
                s.max_gap_us,
                s.adapter.calls,
                latest.map(|l| l.reading),
                age_ms
            );
            if let Some(f) = dev.flight.and_then(ImuCells::read) {
                let (accel, gyro, temperature) = f.latest.map_or(([0.0; 3], [0.0; 3], 0.0), |l| {
                    (
                        l.reading.accel_ms2,
                        l.reading.gyro_rads,
                        l.reading.temperature_c,
                    )
                });
                defmt::info!(
                    "[STAT] {=str} flight core: delay count {=u32} ({=u32} after the start), CHECK kicks taken {=u32}, |accel| {=f32} m/s^2, accel {} m/s^2, gyro {} rad/s, temperature {=f32} C, progress {=u32}",
                    dev.name,
                    f.delays,
                    f.delays_after_start,
                    f.checks,
                    norm(accel),
                    accel,
                    gyro,
                    temperature,
                    f.progress
                );
            }
        }
    }
}

/// The length of a vector (Newton's method: `core` has no `f32::sqrt`).
fn norm(v: [f32; 3]) -> f32 {
    let s = v.iter().map(|x| x * x).sum::<f32>();
    if s <= 0.0 {
        return 0.0;
    }
    let mut x = s.max(1.0);
    for _ in 0..24 {
        x = 0.5 * (x + s / x);
    }
    x
}

/// A flight-core row: copies the cells into the row, logs the flight core's events (it logs
/// nothing itself), queues the transition texts, and kicks CHECK when the last sample is older
/// than T.
fn mirror(dev: &'static Device, w: &mut Watcher, now: Instant) {
    let Some(cells) = dev.flight else {
        return;
    };
    let Some(s) = cells.read() else {
        return;
    };
    let now_us = now.as_micros();
    let mut status = dev.status();
    status.state = s.state;
    status.health = s.health;
    status.adapter = AdapterCounters { calls: s.calls };
    status.samples = s.samples;
    status.missed_edges = s.missed_edges;
    status.last_sample_us = s.last_sample_us;
    status.max_gap_us = s.max_gap_us;
    dev.set_status(status);
    dev.progress.store(s.progress, Ordering::Relaxed);
    dev.restart_outcome
        .store(s.restart_outcome, Ordering::Release);
    if let Some(l) = s.latest
        && w.flight.seq != Some(l.seq)
    {
        w.flight.seq = Some(l.seq);
        dev.latest.sender().send(Sample {
            reading: Reading::Imu {
                gyro_rads: l.reading.gyro_rads,
                accel_ms2: l.reading.accel_ms2,
                temperature_c: l.reading.temperature_c,
            },
            t_us: l.t_us,
            seq: l.seq,
            source: Source::Hardware,
        });
    }

    let new = s.transitions.wrapping_sub(w.flight.transitions);
    if new as usize > TRANSITIONS {
        defmt::warn!(
            "[INFO] {=str}: {=u32} transitions since the last tick, the oldest {=u32} not seen",
            dev.name,
            new,
            new - TRANSITIONS as u32
        );
    }
    for n in s.transitions.wrapping_sub(new.min(TRANSITIONS as u32))..s.transitions {
        let Some(Some(t)) = s.ring.get(n as usize % TRANSITIONS).copied() else {
            continue;
        };
        if t.before != t.after {
            defmt::info!(
                "[INFO] {=str}: {=str} -> {=str} at {=u64} ms (last error {=str})",
                dev.name,
                state_name(t.before),
                state_name(t.after),
                t.at_ms,
                error_name(t.error)
            );
        }
        if let Some(report) = t.report {
            queue(w, dev, Notice::Report(report));
        }
    }
    w.flight.transitions = s.transitions;

    if s.restart_taken != w.flight.restart_taken {
        w.flight.restart_taken = s.restart_taken;
        defmt::info!(
            "[INFO] {=str}: restart request {=u32} taken at {=u64} ms",
            dev.name,
            s.restart_taken,
            s.restart_taken_ms
        );
    }
    if s.attempt_ms != w.flight.attempt_ms {
        w.flight.attempt_ms = s.attempt_ms;
        defmt::info!(
            "[INFO] {=str}: recovery attempt {=u8} starts at {=u64} ms, {=u64} ms after the previous failure",
            dev.name,
            s.health.attempts,
            s.attempt_ms,
            s.attempt_ms.saturating_sub(s.failure_ms)
        );
    }
    if s.attempt_failed_ms != w.flight.attempt_failed_ms {
        w.flight.attempt_failed_ms = s.attempt_failed_ms;
        defmt::info!(
            "[INFO] {=str}: recovery attempt {=u8} failed at {=u64} ms",
            dev.name,
            s.health.attempts,
            s.attempt_failed_ms
        );
    }
    if s.missed_edges != w.flight.missed_edges {
        defmt::info!(
            "[INFO] {=str}: missed edges {=u32} (+{=u32}), the latest found at {=u64} us with the last sample {=u64} us old; injection state {=u32} since {=u64} us",
            dev.name,
            s.missed_edges,
            s.missed_edges.wrapping_sub(w.flight.missed_edges),
            s.missed_at_us,
            s.missed_late_us,
            s.inject,
            s.inject_since_us
        );
        w.flight.missed_edges = s.missed_edges;
    }
    if s.inject != w.flight.inject {
        w.flight.inject = s.inject;
        defmt::info!(
            "[INFO] {=str}: injection state {=u32} since {=u64} us",
            dev.name,
            s.inject,
            s.inject_since_us
        );
    }
    if s.delays_after_start != w.flight.delays_after_start {
        w.flight.delays_after_start = s.delays_after_start;
        defmt::info!(
            "[INFO] {=str}: delay count after the start {=u32} at {=u64} ms",
            dev.name,
            s.delays_after_start,
            now.as_millis()
        );
    }

    let late = s
        .last_sample_us
        .is_none_or(|t| now_us.saturating_sub(t) > dev.deadline().as_micros());
    if late {
        cells.request_check();
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
    if matches!(notice, Notice::Stuck) {
        crate::text_fmt(severity, format_args!("{}: {}", dev.name, what));
        defmt::info!(
            "[INFO] text {=str}: {=str}: {=str}",
            severity_name(severity),
            dev.name,
            what
        );
    } else {
        crate::text_fmt(severity, format_args!("{}: {} ({})", dev.name, what, error));
        defmt::info!(
            "[INFO] text {=str}: {=str}: {=str} ({=str})",
            severity_name(severity),
            dev.name,
            what,
            error
        );
    }
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
