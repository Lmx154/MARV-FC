//! The device layer (core 0): one task per device runs [`run_device`] over its [`DeviceAdapter`]; one static registry row per device ([`Device`])
//! is what the supervisor, the telemetry and SYS_STATUS read. A row is backed either by a core-0 task ([`BARO`]) or by a flight-core task's cells
//! ([`IMU1`], [`IMU2`], `fw_flight::imu`): the supervisor copies those into the row every tick, kicks CHECK when the row's last sample is older than T, and logs the flight core's events.

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
use fw_core::sample::{BaroReading, GnssReading, MagReading, UtcDate, UtcTime};
use fw_core::selftest::{Outcome as SelfTestOutcome, Verdict};
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
const TEXT_EVERY: Duration = Duration::from_secs(1);
const READERS: usize = 2;
const NOTICE_DEPTH: usize = 4;

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Source {
    Hardware,
    #[allow(dead_code)]
    Hil,
}

/// The physical values of one sample, one variant per kind of device; barometer, compass and GNSS carry `fw_core::sample` types.
#[derive(Clone, Copy, PartialEq)]
pub enum Reading {
    Baro(BaroReading),
    Mag {
        /// The field in microtesla with the calibration applied, m = S (r - o), and the die temperature: what the flight core gets.
        reading: MagReading,
        /// The raw field r in microtesla, body frame (after `MAG_ORIENT`, before the calibration): what the compass calibration measures. Core 0 only.
        raw_ut: [f32; 3],
    },
    Batt {
        voltage_v: f32,
        current_a: f32,
    },
    Imu {
        gyro_rads: [f32; 3],
        accel_ms2: [f32; 3],
        temperature_c: f32,
    },
    /// One fix of the receiver, in the driver's integer units; position, speed and course are `None` without a fix.
    Gnss(GnssReading),
}

/// The `[STAT]` text of a reading, field for field as the derived format prints it (fw-core has no defmt).
impl defmt::Format for Reading {
    fn format(&self, f: defmt::Formatter<'_>) {
        match *self {
            Reading::Baro(BaroReading {
                pressure_pa,
                temperature_c,
            }) => defmt::write!(
                f,
                "Baro {{ pressure_pa: {=f32}, temperature_c: {=f32} }}",
                pressure_pa,
                temperature_c
            ),
            Reading::Mag {
                reading:
                    MagReading {
                        field_ut,
                        temperature_c,
                    },
                raw_ut,
            } => defmt::write!(
                f,
                "Mag {{ field_ut: {}, raw_ut: {}, temperature_c: {=f32} }}",
                field_ut,
                raw_ut,
                temperature_c
            ),
            Reading::Batt {
                voltage_v,
                current_a,
            } => defmt::write!(
                f,
                "Batt {{ voltage_v: {=f32}, current_a: {=f32} }}",
                voltage_v,
                current_a
            ),
            Reading::Imu {
                gyro_rads,
                accel_ms2,
                temperature_c,
            } => defmt::write!(
                f,
                "Imu {{ gyro_rads: {}, accel_ms2: {}, temperature_c: {=f32} }}",
                gyro_rads,
                accel_ms2,
                temperature_c
            ),
            Reading::Gnss(g) => defmt::write!(
                f,
                "Gnss {{ fix_type: {=u8}, lat_e7: {}, lon_e7: {}, alt_mm: {}, speed_mm_s: {}, course_cdeg: {}, sats_used: {}, hdop_x100: {}, vdop_x100: {}, time: {}, date: {}, lat_std_mm: {}, lon_std_mm: {}, alt_std_mm: {} }}",
                g.fix_type,
                g.lat_e7,
                g.lon_e7,
                g.alt_mm,
                g.speed_mm_s,
                g.course_cdeg,
                g.sats_used,
                g.hdop_x100,
                g.vdop_x100,
                Utc(g.time),
                UtcDay(g.date),
                g.lat_std_mm,
                g.lon_std_mm,
                g.alt_std_mm
            ),
        }
    }
}

/// An epoch's UTC time in the `[STAT]` text, in the derived-Debug layout.
pub(crate) struct Utc(pub Option<UtcTime>);

impl defmt::Format for Utc {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self.0 {
            Some(t) => defmt::write!(
                f,
                "Some(Time {{ hour: {=u8}, minute: {=u8}, second: {=u8}, millis: {=u16} }})",
                t.hour,
                t.minute,
                t.second,
                t.millis
            ),
            None => defmt::write!(f, "None"),
        }
    }
}

/// An epoch's UTC date in the `[STAT]` text, in the derived-Debug layout.
struct UtcDay(Option<UtcDate>);

impl defmt::Format for UtcDay {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self.0 {
            Some(d) => defmt::write!(
                f,
                "Some(Date {{ year: {=u16}, month: {=u8}, day: {=u8} }})",
                d.year,
                d.month,
                d.day
            ),
            None => defmt::write!(f, "None"),
        }
    }
}

/// One published sample: values, board time of the read in us, `seq` (+1 per publish), source.
#[derive(Clone, Copy, PartialEq, defmt::Format)]
pub struct Sample {
    pub reading: Reading,
    pub t_us: u64,
    pub seq: u32,
    pub source: Source,
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct AdapterCounters {
    pub calls: u32,
}

/// What the one status read after a data-ready timeout found.
pub enum Pending {
    /// A sample is waiting: the edge was missed.
    Data,
    /// No sample: a deadline miss.
    Nothing,
    PowerOnReset,
}

pub struct StartFailure {
    pub identified: bool,
}

pub enum VerifyError {
    Mismatch,
    Failed(ErrorKind),
}

/// The firmware side of one device, driven by [`run_device`]. Every method except
/// [`data_ready`](Self::data_ready) may use the bus and is always awaited to completion.
pub trait DeviceAdapter {
    /// Waits for the device's data-ready signal. No bus traffic: it may be dropped at any await.
    async fn data_ready(&mut self);
    /// Init and configure, for a boot and the start retries (the first attempt after an operator RESTART request uses [`restart`](Self::restart)).
    /// The loop then waits for one fresh sample (see [`recover`](Self::recover)); if none comes the start has failed, with `identified: true`.
    async fn start(&mut self) -> Result<(), StartFailure>;
    /// Reads one sample. `Err(ErrorKind::Por)` means the device reported a power-on reset: the loop
    /// feeds it to `Health` as a power-on-reset event, not as an error event.
    async fn acquire(&mut self) -> Result<Reading, ErrorKind>;
    async fn status(&mut self) -> Result<Pending, ErrorKind>;
    async fn verify(&mut self) -> Result<(), VerifyError>;
    async fn apply(&mut self) -> Result<(), ErrorKind>;
    /// Brings the bus and the device back in step without a reset.
    /// `Ok` (also of [`restart`](Self::restart)) only says the bus and the device answer; the loop then waits for one fresh sample
    /// (`with_timeout(T, data_ready())` and one `acquire`) before reporting success to `Health`.
    /// A timeout or a failed `acquire` counts as an adapter error: a failed `recover` goes on to `restart`, a failed `restart` ends the attempt.
    async fn recover(&mut self) -> Result<(), ErrorKind>;
    /// Bus clear where the bus has one, then init and configure; used by the automatic recovery and the first start attempt after an operator RESTART request.
    /// A failed attempt after a RESTART request goes on to the normal start retries.
    async fn restart(&mut self) -> Result<(), ErrorKind>;
    /// The device's built-in self-test with the driver's default settings. The loop then restarts
    /// the device as for an operator RESTART request, whatever this returned.
    async fn self_test(&mut self) -> Verdict {
        Verdict::Unsupported
    }
    fn counters(&self) -> AdapterCounters;
    /// Called once for every sample published to the row's `Watch`, right after it: barometer, compass and GNSS hand the sample to the flight core here (`crate::flight`). Never waits.
    fn to_flight(&mut self, sample: &Sample);
}

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
    /// Nominal sample period P in microseconds; it follows the configured output data rate, except
    /// GNSS: fixed 1 s.
    period_us: AtomicU32,
    pub latest: Watch<CriticalSectionRawMutex, Sample, READERS>,
    status: Mutex<CriticalSectionRawMutex, Cell<DeviceStatus>>,
    pub wake: Signal<CriticalSectionRawMutex, ()>,
    pub progress: AtomicU32,
    pub stuck: AtomicBool,
    /// The requests raised for the task (`REQ_APPLY`, `REQ_RESTART`, `REQ_SELF_TEST`); the task clears each as it takes it.
    request: AtomicU8,
    restarts: AtomicU32,
    /// The outcome of the latest restart the task has finished: request number << 1, bit 0 set when
    /// the device reached Ok, clear when it reached Failed or Absent (0 = none yet; numbers start at 1).
    restart_outcome: AtomicU32,
    self_test: Mutex<CriticalSectionRawMutex, Cell<Option<SelfTestOutcome>>>,
    self_testing: AtomicBool,
    self_test_rows: AtomicU32,
    flight: Option<&'static ImuCells>,
    /// The data-ready deadline T in microseconds when it is not 3 P (0: 3 P).
    deadline_us: u32,
}

const REQ_APPLY: u8 = 1;
pub(crate) const REQ_RESTART: u8 = 2;
/// Request bit: self-test the device, then restart it as for `REQ_RESTART`, from any state.
const REQ_SELF_TEST: u8 = 4;

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
            self_test: Mutex::new(Cell::new(None)),
            self_testing: AtomicBool::new(false),
            self_test_rows: AtomicU32::new(0),
            flight: None,
            deadline_us: 0,
        }
    }

    pub const fn with_deadline_us(mut self, us: u32) -> Self {
        self.deadline_us = us;
        self
    }

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

    pub fn set_period_us(&self, us: u32) {
        self.period_us.store(us, Ordering::Relaxed);
    }

    pub fn period_us(&self) -> u32 {
        self.period_us.load(Ordering::Relaxed)
    }

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

    /// Raises the device's SELF-TEST request, wakes its task and sets the row's self-test flag.
    /// Numbered with the restart requests: the restart that follows reports under the same number ([`restart_outcome`](Self::restart_outcome)), the test through [`self_test_outcome`](Self::self_test_outcome).
    pub fn request_self_test(&self) -> u32 {
        self.self_testing.store(true, Ordering::Relaxed);
        let number = self.restarts.fetch_add(1, Ordering::AcqRel).wrapping_add(1);
        if let Some(cells) = self.flight {
            cells.request_self_test(number);
            return number;
        }
        self.request.fetch_or(REQ_SELF_TEST, Ordering::AcqRel);
        self.wake.signal(());
        number
    }

    /// The outcome of self-test request `number` once its test has finished (`None` before, and
    /// for any other request).
    pub fn self_test_outcome(&self, number: u32) -> Option<SelfTestOutcome> {
        self.self_test
            .lock(Cell::get)
            .filter(|o| o.number == number)
    }

    /// The self-test action has ended: clears the row's flag. Returns the MARV_DEVICE_STATUS rows
    /// built with the flag since boot.
    pub fn self_test_ended(&self) -> u32 {
        self.self_testing.store(false, Ordering::Relaxed);
        self.self_test_rows.load(Ordering::Relaxed)
    }

    pub(crate) fn take_request(&self, bit: u8) -> bool {
        self.request.fetch_and(!bit, Ordering::AcqRel) & bit != 0
    }

    pub(crate) fn requested(&self, bit: u8) -> bool {
        self.request.load(Ordering::Acquire) & bit != 0
    }

    pub fn status(&self) -> DeviceStatus {
        self.status.lock(Cell::get)
    }

    pub(crate) fn set_status(&self, status: DeviceStatus) {
        self.status.lock(|cell| cell.set(status));
    }

    pub(crate) fn finish_restart(&self, number: u32, ok: bool) {
        self.restart_outcome
            .store(number << 1 | u32::from(ok), Ordering::Release);
    }

    fn period(&self) -> Duration {
        Duration::from_micros(u64::from(self.period_us()))
    }

    pub fn deadline(&self) -> Duration {
        if self.deadline_us != 0 {
            return Duration::from_micros(u64::from(self.deadline_us));
        }
        (self.period() * 3).max(DEADLINE_FLOOR)
    }

    pub fn stuck_after(&self) -> Duration {
        (self.period() * 20).max(STUCK_FLOOR)
    }

    pub(crate) fn mark(&self) {
        self.progress.fetch_add(1, Ordering::Relaxed);
    }

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

    pub fn message(&self, now_us: u64) -> MARV_DEVICE_STATUS_DATA {
        let status = self.status();
        let ms = |us: u64| u32::try_from(us / 1000).unwrap_or(u32::MAX);
        let self_test = self.self_testing.load(Ordering::Relaxed);
        if self_test {
            self.self_test_rows.fetch_add(1, Ordering::Relaxed);
        }
        device_status(&StatusInput {
            device: self.id,
            state: status.state,
            counters: status.health,
            stuck: self.stuck.load(Ordering::Relaxed),
            samples: status.samples,
            last_sample_age_ms: status.last_sample_us.map(|t| ms(now_us.saturating_sub(t))),
            hil: false,
            self_test,
            time_boot_ms: ms(now_us),
        })
    }
}

pub const DEVICE_COUNT: usize = 7;
/// The IMU rows' period before `main` sets it from the boot setup: 800 Hz, the default of `IMUn_ODR_HZ`.
const IMU_BOOT_PERIOD_US: u32 = 1250;
/// IMU1's row, backed by the flight core; its P is set at boot from `IMU1_ODR_HZ`.
pub static IMU1: Device = Device::flight(
    MarvDevice::MARV_DEVICE_IMU1,
    "IMU1",
    IMU_BOOT_PERIOD_US,
    &crate::imu1::CELLS,
);
/// IMU2's row, backed by the flight core; its P is set at boot from `IMU2_ODR_HZ`.
pub static IMU2: Device = Device::flight(
    MarvDevice::MARV_DEVICE_IMU2,
    "IMU2",
    IMU_BOOT_PERIOD_US,
    &crate::imu2::CELLS,
);
pub static BARO: Device = Device::new(MarvDevice::MARV_DEVICE_BARO, "BARO", crate::baro::PERIOD_US);
pub static MAG: Device = Device::new(MarvDevice::MARV_DEVICE_MAG, "MAG", crate::mag::PERIOD_US);
/// The GNSS receiver's row: P = 1 s, T = 2.5 s.
pub static GNSS: Device = Device::new(MarvDevice::MARV_DEVICE_GNSS, "GNSS", crate::gnss::PERIOD_US)
    .with_deadline_us(crate::gnss::DEADLINE_US);
pub static BATT: Device = Device::new(MarvDevice::MARV_DEVICE_BATT, "BATT", crate::batt::PERIOD_US);
/// The SD card's row (`sd`): P = 1 s, so stuck after 20 s.
pub static SD: Device = Device::new(MarvDevice::MARV_DEVICE_SD, "SD", crate::sd::PERIOD_US);
pub static DEVICES: [&Device; DEVICE_COUNT] = [
    &IMU1,
    &IMU2,
    &BARO,
    &MAG,
    &GNSS,
    &BATT,
    &SD,
];

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

/// Device state changes since boot (core-0 rows and flight rows), for the `save-loop` summary.
pub static STATE_CHANGES: AtomicU32 = AtomicU32::new(0);
/// Deadline misses and stuck windows discarded because a flash save overlapped them.
pub static SAVE_DISCARDS: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy)]
enum Notice {
    Report(Report),
    Stuck,
}

static NOTICES: Channel<CriticalSectionRawMutex, (&'static Device, Notice), 8> = Channel::new();

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// Hands a health transition to the supervisor for its text (a task with its own loop, `sd`).
pub(crate) fn notify(dev: &'static Device, report: Report) {
    if NOTICES.try_send((dev, Notice::Report(report))).is_err() {
        defmt::warn!("[INFO] {=str}: notice queue full", dev.name);
    }
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
/// Sampling is `select(with_timeout(T, data ready), wake)`; every bus call is awaited to completion outside any `select` or timeout.
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
    // Set by an operator RESTART request: its first start attempt goes through `restart()`.
    let mut strong_first = false;
    let sender = dev.latest.sender();
    let mut step = health.step(Event::Tick, now_ms());
    loop {
        // A self-test request: the test, then the operator restart under the test's number.
        let mut restart = None;
        if dev.take_request(REQ_SELF_TEST) {
            let number = dev.restarts();
            self_test(dev, adapter, number).await;
            restart = Some(number);
        }
        if dev.take_request(REQ_RESTART) {
            restart = Some(dev.restarts());
        }
        if let Some(number) = restart {
            restart_run = Some(number);
            strong_first = true;
            let (before, now) = (health.state(), now_ms());
            step = health.step(Event::Restart, now);
            defmt::info!(
                "[INFO] {=str}: restart request {=u32} taken at {=u64} ms: {=str} -> {=str}",
                dev.name,
                number,
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
            Action::Start if core::mem::take(&mut strong_first) => match adapter.restart().await {
                Ok(()) if fresh_sample(dev, adapter, &mut status, &mut seq, &sender).await => {
                    Event::StartOk
                }
                Ok(()) => Event::StartFailed { identified: true },
                Err(_) => Event::StartFailed { identified: false },
            },
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
            STATE_CHANGES.fetch_add(1, Ordering::Relaxed);
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

async fn self_test<A: DeviceAdapter>(dev: &'static Device, adapter: &mut A, number: u32) {
    let begun = Instant::now();
    let verdict = adapter.self_test().await;
    dev.mark();
    let took_us = u32::try_from(begun.elapsed().as_micros()).unwrap_or(u32::MAX);
    dev.self_test.lock(|cell| {
        cell.set(Some(SelfTestOutcome {
            number,
            verdict,
            took_us,
        }))
    });
}

async fn sample<A: DeviceAdapter>(
    dev: &'static Device,
    adapter: &mut A,
    status: &mut DeviceStatus,
    seq: &mut u32,
    sender: &Sender<'static, CriticalSectionRawMutex, Sample, READERS>,
) -> Event {
    let waited_from = Instant::now();
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
                Ok(Pending::Nothing)
                    if crate::settings::overlaps_save(waited_from, Instant::now()) =>
                {
                    discard_for_save(dev, "deadline miss");
                    Event::Tick
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
    let waited_from = Instant::now();
    let mut ready = with_timeout(dev.deadline(), adapter.data_ready())
        .await
        .is_ok();
    if !ready && crate::settings::overlaps_save(waited_from, Instant::now()) {
        discard_for_save(dev, "deadline miss after a start");
        ready = with_timeout(dev.deadline(), adapter.data_ready())
            .await
            .is_ok();
    }
    dev.mark();
    ready && matches!(read(adapter, status, seq, sender).await, Event::Good)
}

/// A timeout or stuck window that a flash save overlapped: not a health event.
pub(crate) fn discard_for_save(dev: &Device, what: &str) {
    SAVE_DISCARDS.fetch_add(1, Ordering::Relaxed);
    defmt::info!(
        "[INFO] {=str}: {=str} discarded, a flash save overlapped it",
        dev.name,
        what
    );
}

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
            let sample = Sample {
                reading,
                t_us,
                seq: *seq,
                source: Source::Hardware,
            };
            sender.send(sample);
            adapter.to_flight(&sample);
            *seq = seq.wrapping_add(1);
            status.samples = status.samples.wrapping_add(1);
            Event::Good
        }
        Err(ErrorKind::Por) => Event::PowerOnReset,
        Err(kind) => Event::Error(kind),
    }
}

pub(crate) async fn sleep_until(dev: &Device, until_ms: u64) {
    let until = Instant::from_millis(until_ms);
    loop {
        let now = Instant::now();
        if now >= until {
            return;
        }
        Timer::at(until.min(now + SLICE)).await;
        dev.mark();
        if dev.requested(REQ_RESTART | REQ_SELF_TEST) {
            return;
        }
    }
}

struct Watcher {
    progress: u32,
    progress_at: Instant,
    notices: [Option<Notice>; NOTICE_DEPTH],
    last_text: Option<Instant>,
    stat_samples: u32,
    flight: FlightSeen,
}

#[derive(Clone, Copy, Default)]
struct FlightSeen {
    seq: Option<u32>,
    transitions: u32,
    restart_taken: u32,
    attempt_ms: u64,
    attempt_failed_ms: u64,
    failure_ms: u64,
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
            // A parked flight-core row (Failed, Absent) makes no progress by design: it is not
            // kicked and waits only for RESTART, so it is not stuck. Nor is a row while its
            // self-test flag is set (the test is one long adapter call).
            let parked = (dev.flight.is_some()
                && matches!(dev.status().state, State::Failed | State::Absent))
                || dev.self_testing.load(Ordering::Relaxed);
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
            } else if parked {
                w.progress_at = now;
            } else if !stuck
                && dev.flight.is_none()
                && now - w.progress_at >= dev.stuck_after()
                && crate::settings::overlaps_save(w.progress_at, now)
            {
                w.progress_at = now;
                discard_for_save(dev, "stuck window");
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

    pub fn stat(&mut self, now: Instant) {
        let window_us = (now - self.stat_at).as_micros().max(1);
        self.stat_at = now;
        for (dev, w) in DEVICES.iter().zip(self.watchers.iter_mut()) {
            let s = dev.status();
            let rate = s.samples.wrapping_sub(w.stat_samples) as f32 * 1e6 / window_us as f32;
            w.stat_samples = s.samples;
            if cfg!(feature = "quiet-stat") {
                continue;
            }
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
            if let Some(Sample {
                reading:
                    Reading::Mag {
                        reading: MagReading { field_ut, .. },
                        ..
                    },
                ..
            }) = latest
            {
                defmt::info!(
                    "[STAT] {=str}: |B| {=f32} uT, B {} uT",
                    dev.name,
                    norm(field_ut),
                    field_ut
                );
            }
            if core::ptr::eq(*dev, &GNSS) {
                crate::gnss::stat(now, window_us, latest);
            }
            if let Some(f) = dev.flight.and_then(ImuCells::read) {
                let (accel, gyro, temperature) = f.latest.map_or(([0.0; 3], [0.0; 3], 0.0), |l| {
                    (
                        l.reading.accel_ms2,
                        l.reading.gyro_rads,
                        l.reading.temperature_c,
                    )
                });
                defmt::info!(
                    "[STAT] {=str} flight core: delay count {=u32} ({=u32} after the start), CHECK kicks taken {=u32}, verify steps {=u32} took last {=u32} min {=u32} max {=u32} us, passes {=u32}, sample already waiting after a step {=u32}, gaps over 1.5 P {=u32} ({=u32} across a step), longest run {=u32} us (last 10 s window {=u32} us), polls {=u32}, |accel| {=f32} m/s^2, accel {} m/s^2, gyro {} rad/s, temperature {=f32} C, progress {=u32}",
                    dev.name,
                    f.delays,
                    f.delays_after_start,
                    f.checks,
                    f.verifies,
                    f.verify_last_us,
                    f.verify_min_us,
                    f.verify_max_us,
                    f.verify_passes,
                    f.verify_pending,
                    f.long_gaps,
                    f.long_gaps_after_verify,
                    f.latency.max_us,
                    f.latency.window_us,
                    f.latency.polls,
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

/// A flight-core row: copies the cells into the row, logs the flight core's events, queues the transition texts and kicks CHECK when the last sample is older
/// than T, except in Failed and Absent (parked: only RESTART wakes it). The delay count is logged when it changed and no start or recovery is under way.
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
    dev.self_test.lock(|cell| cell.set(s.self_test));
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
    STATE_CHANGES.fetch_add(new, Ordering::Relaxed);
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
        // The attempt may have failed within the same tick: then its back-off counted from the failure seen before.
        let previous = if s.failure_ms >= s.attempt_ms {
            w.flight.failure_ms
        } else {
            s.failure_ms
        };
        defmt::info!(
            "[INFO] {=str}: recovery attempt {=u8} starts at {=u64} ms, {=u64} ms after the previous failure",
            dev.name,
            s.health.attempts,
            s.attempt_ms,
            s.attempt_ms.saturating_sub(previous)
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
    w.flight.failure_ms = s.failure_ms;
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
    if s.delays_after_start != w.flight.delays_after_start
        && !matches!(s.state, State::Recovering | State::Starting)
    {
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
    if late && !matches!(s.state, State::Failed | State::Absent) {
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
