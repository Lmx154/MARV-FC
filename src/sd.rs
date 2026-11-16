//! The SD card as a device row (decision-sd-logging Q1, Q2, Q4 first bullet, Q6; milestone SD2): one
//! core-0 thread-mode task owns the bus (PIO0 SM0, DMA channel 4 completing on `DMA_IRQ_1` on core 0),
//! the card and the mounted FAT32 volume, and drives row 9 through the shared `Health` machine with
//! its own loop. Nothing is written to the card except `LOGS/` when it is missing (the log writer is
//! SD3).
//!
//! A start, a recovery and a restart are the same: a new bus, `init`, block 0 through
//! `fw_core::mbr` (partition 1 of type 0x0B/0x0C), the mount over `BufStream` + `StreamSlice`,
//! `LOGS/` opened (created if missing) and listed once for the notice text. While mounted, every
//! second, one liveness: the FAT's status flags and the root directory's `LOGS` entry, two
//! different blocks, so that `BufStream`'s one-block cache serves neither and each is one block read
//! from the card; a good liveness is the row's sample. The waits (`select` of the 1 s timer and
//! the row's wake) are no card calls; every card and file-system call is awaited to its end and a
//! RESTART request is taken between them (Q2-I1). A recovery or a restart un-mounts first.
//!
//! Row (Q6): `NoCard` on every start try: Absent, TIMEOUT, "SD: no card"; block 0 not an MBR with a
//! FAT32 partition 1, or a volume that does not mount as FAT32: Failed, ID_MISMATCH, "SD: no FAT32
//! volume, reformat as FAT32" (the machine parks the row after the start retries; this task shows
//! why); mounted: Ok with one NOTICE; `LOG_BACKEND_TYPE` 0 at boot: Absent, NONE, "SD: logging
//! off", the card untouched. Errors in service go through the machine and its ladder (BUS or
//! TIMEOUT, the supervisor's texts).

use core::sync::atomic::Ordering;

use atomic_waker::AtomicWaker;
use block_device_adapters::{BufStream, BufStreamError, StreamSlice, StreamSliceError};
use embassy_futures::select::select;
use embassy_rp::Peri;
use embassy_rp::interrupt::{self, InterruptExt};
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH4, PIN_28, PIN_29, PIN_30, PIN_31, PIN_32, PIN_33, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_time::{Duration, Instant, Timer};
use embedded_fatfs::{DefaultTimeProvider, FatType, FileSystem, FsOptions, LossyOemCpConverter};
use embedded_io_async::Write;
use fw_core::health::{Action, Counters, ErrorKind, Event, Health, Limits, Report, State};
use fw_core::mbr::{Partition, partition1};
use fw_core::watchdog::task;
use fw_flight::tagged;
use mavlink_device::ParamValue;
use mavlink_device::dialect::MavSeverity;
use pio_sdmmc::pio::{DmaIrqHandler, DmaWaker, PioBus, PioBusConfig};
use pio_sdmmc::{Block, BusError, Config, Error as CardError, SdCard};

use crate::device::{self, Device, DeviceStatus, REQ_RESTART, SD, error_name, state_name};

/// The row's nominal period P: one liveness per second.
pub const PERIOD_US: u32 = 1_000_000;
const LIVENESS_EVERY: Duration = Duration::from_secs(1);
/// Longest wait while parked, as in `device`.
const SLICE: Duration = Duration::from_millis(100);
/// The SD card's channel (resources.md DMA row 4).
const CH: usize = 4;
const BIT: u32 = 1 << CH;
/// INTE0 as the flight core leaves it: IMU1's channels 0 and 1, IMU2's 2 and 3.
const FLIGHT_CHANNELS: u32 = 0x0f;
/// At most this many entries of `LOGS/` are read for the notice.
const LIST_MAX: usize = 1024;
/// 512-byte blocks per GiB.
const GIB_BLOCKS: u64 = 1 << 21;
const NO_CARD: &str = "SD: no card";
const WRONG_FORMAT: &str = "SD: no FAT32 volume, reformat as FAT32";

type Sd<'a> = SdCard<PioBus<'a, PIO0, 0>>;
type Volume<'a> = StreamSlice<BufStream<Sd<'a>, 512>>;
type Fs<'v, 'a> = FileSystem<&'v mut Volume<'a>, DefaultTimeProvider, LossyOemCpConverter>;
type FsError = embedded_fatfs::Error<StreamSliceError<BufStreamError<CardError>>>;

/// The waker of the SD bus's DMA completion (the driver keeps no static state, CONTRACT A6).
static SD_WAKER: AtomicWaker = AtomicWaker::new();

pub struct SdWaker;

impl DmaWaker for SdWaker {
    fn waker() -> &'static AtomicWaker {
        &SD_WAKER
    }
}

embassy_rp::bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    DMA_IRQ_1 => DmaIrqHandler<SdWaker>;
});

/// The SD card's peripherals (docs/pinout.md: CLK 32, CMD 33, DAT0..3 28..31).
pub struct Parts {
    pub pio: Peri<'static, PIO0>,
    pub dma: Peri<'static, DMA_CH4>,
    pub clk: Peri<'static, PIN_32>,
    pub cmd: Peri<'static, PIN_33>,
    pub dat0: Peri<'static, PIN_28>,
    pub dat1: Peri<'static, PIN_29>,
    pub dat2: Peri<'static, PIN_30>,
    pub dat3: Peri<'static, PIN_31>,
}

/// Why a start, recovery or restart failed.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Fault {
    NoCard,
    WrongFormat,
    Error(ErrorKind),
}

/// Q6's error of a driver error: TIMEOUT for time-outs, BUS for the rest.
fn card_kind(e: &CardError) -> ErrorKind {
    match e {
        CardError::InitTimeout { .. }
        | CardError::Bus {
            error: BusError::Timeout,
            ..
        } => ErrorKind::Timeout,
        _ => ErrorKind::Bus,
    }
}

/// The error of a file-system call: the driver's through the adapters, BUS (a decode error) else.
fn fs_kind(e: &FsError) -> ErrorKind {
    match e {
        embedded_fatfs::Error::Io(StreamSliceError::Other(BufStreamError::Io(e))) => card_kind(e),
        _ => ErrorKind::Bus,
    }
}

fn now_ms() -> u64 {
    Instant::now().as_millis()
}

/// Row 9 as this task drives it.
struct Row {
    dev: &'static Device,
    health: Health,
    status: DeviceStatus,
    action: Action,
    /// The fault of the latest failed start: what the row shows once the start retries have ended.
    start_fault: Option<Fault>,
    /// The restart request being carried out, until its outcome is published.
    restart_run: Option<u32>,
    checked: bool,
}

impl Row {
    /// The state and counters shown: the machine's, except that a row parked Absent by its start
    /// retries shows why (Q6).
    fn shown(&self) -> (State, Counters) {
        let mut counters = *self.health.counters();
        let mut state = self.health.state();
        if state == State::Absent {
            match self.start_fault {
                Some(Fault::NoCard) => counters.last_error = ErrorKind::Timeout,
                Some(Fault::WrongFormat) => {
                    state = State::Failed;
                    counters.last_error = ErrorKind::IdMismatch;
                }
                Some(Fault::Error(kind)) => counters.last_error = kind,
                None => {}
            }
        }
        (state, counters)
    }

    fn publish(&mut self) {
        (self.status.state, self.status.health) = self.shown();
        self.dev.set_status(self.status);
    }

    fn feed(&mut self, event: Event) {
        let before = self.status.state;
        let now = now_ms();
        let step = self.health.step(event, now);
        self.action = step.action;
        self.publish();
        let after = self.status.state;
        if after != before {
            device::STATE_CHANGES.fetch_add(1, Ordering::Relaxed);
            defmt::info!(
                "[INFO] SD: {=str} -> {=str} at {=u64} ms (last error {=str})",
                state_name(before),
                state_name(after),
                now,
                error_name(self.status.health.last_error)
            );
        }
        if let Some(report) = step.report {
            match (report, self.start_fault) {
                (Report::Absent, Some(Fault::NoCard)) => {
                    text(MavSeverity::MAV_SEVERITY_ERROR, NO_CARD)
                }
                (Report::Absent, Some(Fault::WrongFormat)) => {
                    text(MavSeverity::MAV_SEVERITY_ERROR, WRONG_FORMAT)
                }
                _ => device::notify(self.dev, report),
            }
        }
        if let Some(number) = self.restart_run
            && !self.dev.requested(REQ_RESTART)
        {
            let ok = match after {
                State::Ok => Some(true),
                State::Failed | State::Absent => Some(false),
                _ => None,
            };
            if let Some(ok) = ok {
                self.dev.finish_restart(number, ok);
                self.restart_run = None;
            }
        }
    }

    /// A RESTART request: taken here, between operations.
    fn take_restart(&mut self) {
        if !self.dev.take_request(REQ_RESTART) {
            return;
        }
        let number = self.dev.restarts();
        let before = self.status.state;
        self.restart_run = Some(number);
        self.start_fault = None;
        self.action = self.health.step(Event::Restart, now_ms()).action;
        self.publish();
        defmt::info!(
            "[INFO] SD: restart request {=u32} taken at {=u64} ms: {=str} -> {=str}",
            number,
            now_ms(),
            state_name(before),
            state_name(self.status.state)
        );
    }

    /// The operation `op` failed with `fault`.
    fn failed(&mut self, op: Action, fault: Fault, began: Instant) {
        defmt::info!(
            "[INFO] SD: {=str} failed after {=u64} ms: {=str}",
            op_name(op),
            began.elapsed().as_millis(),
            match fault {
                Fault::NoCard => "no card",
                Fault::WrongFormat => "no FAT32 volume",
                Fault::Error(kind) => error_name(kind),
            }
        );
        let event = match op {
            Action::Start => {
                self.start_fault = Some(fault);
                Event::StartFailed {
                    identified: fault != Fault::NoCard,
                }
            }
            Action::Recover => Event::RecoverDone { ok: false },
            _ => Event::RestartDone { ok: false },
        };
        self.feed(event);
    }

    fn succeeded(&mut self, op: Action) {
        self.start_fault = None;
        self.feed(match op {
            Action::Start => Event::StartOk,
            Action::Recover => Event::RecoverDone { ok: true },
            _ => Event::RestartDone { ok: true },
        });
    }
}

fn op_name(op: Action) -> &'static str {
    match op {
        Action::Start => "start",
        Action::Recover => "recovery",
        _ => "restart",
    }
}

fn text(severity: MavSeverity, line: &str) {
    defmt::info!("[INFO] SD text: {=str}", line);
    crate::text(severity, line);
}

#[embassy_executor::task]
pub async fn task(parts: Parts) -> ! {
    tagged(&crate::watchdog::RUNNING0, task::SD, run(parts)).await
}

async fn run(mut parts: Parts) -> ! {
    let dev = &SD;
    let backend = crate::store::with(|p| match p.get_named(fw_settings::sd::BACKEND_TYPE) {
        Some(ParamValue::I32(v)) => Some(v),
        _ => None,
    })
    .flatten();
    if !fw_settings::sd::file_backend(backend) {
        off(dev).await
    }
    let mut row = Row {
        dev,
        health: Health::new(Limits::DESIGN),
        status: dev.status(),
        action: Action::None,
        start_fault: None,
        restart_run: None,
        checked: false,
    };
    row.feed(Event::Tick);
    loop {
        row.take_restart();
        match row.action {
            Action::Start | Action::Recover | Action::Restart => {
                session(&mut parts, &mut row).await
            }
            Action::WaitUntil(until) => {
                device::sleep_until(dev, until).await;
                row.feed(Event::Tick);
            }
            Action::Park => {
                // No card traffic; a RESTART request arrives through `wake`.
                let _ = select(Timer::after(SLICE), dev.wake.wait()).await;
                dev.mark();
                row.feed(Event::Tick);
            }
            // Not reached: every operation's result is fed back at once, and nothing here asks
            // for a reconfigure.
            Action::None | Action::Reconfigure => {
                Timer::after(SLICE).await;
                dev.mark();
                row.feed(Event::Tick);
            }
        }
    }
}

/// `LOG_BACKEND_TYPE` = 0 at boot: Absent, NONE, one text; the card is never touched. A RESTART
/// request ends at once, not Ok (the parameter takes effect at the next boot).
async fn off(dev: &'static Device) -> ! {
    let mut status = dev.status();
    status.state = State::Absent;
    dev.set_status(status);
    device::STATE_CHANGES.fetch_add(1, Ordering::Relaxed);
    defmt::info!(
        "[INFO] SD: LOG_BACKEND_TYPE 0: Starting -> Absent at {=u64} ms (last error none), the card is not touched",
        now_ms()
    );
    text(MavSeverity::MAV_SEVERITY_INFO, "SD: logging off");
    loop {
        let _ = select(Timer::after(SLICE), dev.wake.wait()).await;
        dev.mark();
        if dev.take_request(REQ_RESTART) {
            let number = dev.restarts();
            dev.finish_restart(number, false);
            defmt::info!(
                "[INFO] SD: restart request {=u32}: logging off until the next boot",
                number
            );
        }
    }
}

/// One bus: built, the start (recovery, restart) the machine asked for, the service while mounted,
/// the un-mount. Returns when the machine wants another operation or a RESTART request waits.
async fn session(parts: &mut Parts, row: &mut Row) {
    let op = row.action;
    let began = Instant::now();
    let Pio {
        mut common, sm0, ..
    } = Pio::new(parts.pio.reborrow(), Irqs);
    let bus = PioBus::new(
        &mut common,
        sm0,
        parts.dma.reborrow(),
        Irqs,
        parts.clk.reborrow(),
        parts.cmd.reborrow(),
        parts.dat0.reborrow(),
        parts.dat1.reborrow(),
        parts.dat2.reborrow(),
        parts.dat3.reborrow(),
        PioBusConfig::default(),
    );
    if !core::mem::replace(&mut row.checked, true) {
        register_checks();
    }
    let mut card = SdCard::new(bus, Config::default());
    let opened = open(&mut card).await;
    row.dev.mark();
    let part = match opened {
        Ok(part) => part,
        Err(fault) => return row.failed(op, fault, began),
    };
    let start = u64::from(part.first_block) * 512;
    let end = start + u64::from(part.blocks) * 512;
    let sliced = StreamSlice::new(BufStream::<_, 512>::new(card), start, end).await;
    row.dev.mark();
    let mut volume = match sliced {
        Ok(volume) => volume,
        Err(StreamSliceError::Other(BufStreamError::Io(e))) => {
            return row.failed(op, Fault::Error(card_kind(&e)), began);
        }
        Err(_) => return row.failed(op, Fault::Error(ErrorKind::Bus), began),
    };
    mounted(&mut volume, part, row, op, began).await;
    let flushed = volume.flush().await;
    row.dev.mark();
    let card = volume.into_inner().into_inner();
    defmt::info!(
        "[INFO] SD: session ended at {=u64} ms, the stream's flush ok {=bool}; driver {}",
        now_ms(),
        flushed.is_ok(),
        card.stats()
    );
}

/// `init` and block 0: the partition, or why not.
async fn open(card: &mut Sd<'_>) -> Result<Partition, Fault> {
    let t = Instant::now();
    let info = card.init().await;
    #[cfg(feature = "inject-sd-no-card")]
    let info: Result<pio_sdmmc::CardInfo, CardError> = {
        let _ = info;
        Err(CardError::NoCard)
    };
    let info = match info {
        Ok(info) => info,
        Err(CardError::NoCard) => return Err(Fault::NoCard),
        Err(e) => {
            defmt::info!("[INFO] SD: init failed: {}", e);
            return Err(Fault::Error(card_kind(&e)));
        }
    };
    defmt::info!(
        "[INFO] SD: card init in {=u64} us: {=u64} blocks, {=u32} Hz",
        t.elapsed().as_micros(),
        info.capacity_blocks,
        info.clock_hz
    );
    let mut block0 = [Block::new(); 1];
    if let Err(e) = card.read_blocks(0, &mut block0).await {
        defmt::info!("[INFO] SD: block 0 not read: {}", e);
        return Err(Fault::Error(card_kind(&e)));
    }
    let [mut block0] = block0;
    #[cfg(feature = "inject-sd-wrong-format")]
    if let Some(kind) = block0.as_bytes_mut().get_mut(0x1be + 4) {
        *kind = 0x07;
    }
    let bytes: &[u8; 512] = block0
        .as_bytes_mut()
        .first_chunk()
        .ok_or(Fault::WrongFormat)?;
    match partition1(bytes, info.capacity_blocks) {
        Ok(part) => Ok(part),
        Err(why) => {
            defmt::info!(
                "[INFO] SD: block 0 is not an MBR with a FAT32 partition 1: {}, partition entry 1 {=[u8]:02x}",
                defmt::Debug2Format(&why),
                bytes.get(0x1be..0x1ce).unwrap_or(&[])
            );
            Err(Fault::WrongFormat)
        }
    }
}

/// The mount, the notice, the service, the un-mount.
async fn mounted(
    volume: &mut Volume<'_>,
    part: Partition,
    row: &mut Row,
    op: Action,
    began: Instant,
) {
    let t = Instant::now();
    let fs = match FileSystem::new(volume, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(embedded_fatfs::Error::Io(StreamSliceError::Other(BufStreamError::Io(e)))) => {
            row.dev.mark();
            return row.failed(op, Fault::Error(card_kind(&e)), began);
        }
        Err(e) => {
            row.dev.mark();
            defmt::info!("[INFO] SD: mount: {}", defmt::Debug2Format(&e));
            return row.failed(op, Fault::WrongFormat, began);
        }
    };
    let mount_us = t.elapsed().as_micros();
    row.dev.mark();
    if fs.fat_type() != FatType::Fat32 {
        let _ = fs.unmount().await;
        row.dev.mark();
        return row.failed(op, Fault::WrongFormat, began);
    }
    let listed = logs(&fs).await;
    row.dev.mark();
    let (files, bytes) = match listed {
        Ok(v) => v,
        Err(e) => {
            let kind = fs_kind(&e);
            let _ = fs.unmount().await;
            row.dev.mark();
            return row.failed(op, Fault::Error(kind), began);
        }
    };
    let tenths = (u64::from(part.blocks) * 10 + GIB_BLOCKS / 2) / GIB_BLOCKS;
    let used_mib = (bytes + (1 << 19)) >> 20;
    defmt::info!(
        "[INFO] SD: {=str} done in {=u64} ms: partition 1 type {=u8:#04x} from block {=u32}, {=u32} blocks; mounted in {=u64} us, cluster {=u32} B, label {=[u8]:a}; LOGS/ holds {=u32} files, {=u64} B",
        op_name(op),
        began.elapsed().as_millis(),
        part.kind,
        part.first_block,
        part.blocks,
        mount_us,
        fs.cluster_size(),
        fs.volume_label_as_bytes(),
        files,
        bytes
    );
    let line = device::TextLine::new(format_args!(
        "SD: {}.{} GiB FAT32, {} MiB in {} logs",
        tenths / 10,
        tenths % 10,
        used_mib,
        files
    ));
    text(MavSeverity::MAV_SEVERITY_NOTICE, line.as_str());
    row.succeeded(op);
    serve(&fs, row).await;
    let t = Instant::now();
    let unmounted = fs.unmount().await;
    row.dev.mark();
    defmt::info!(
        "[INFO] SD: un-mounted in {=u64} us, ok {=bool}",
        t.elapsed().as_micros(),
        unmounted.is_ok()
    );
}

/// `LOGS/`, created if missing, and its files: (count, bytes). One listing, bounded.
async fn logs(fs: &Fs<'_, '_>) -> Result<(u32, u64), FsError> {
    let dir = fs.root_dir().create_dir("LOGS").await?;
    let mut entries = dir.iter();
    let (mut files, mut bytes) = (0u32, 0u64);
    for _ in 0..LIST_MAX {
        let Some(entry) = entries.next().await else {
            break;
        };
        let entry = entry?;
        if entry.is_file() {
            files += 1;
            bytes += entry.len();
        }
    }
    Ok((files, bytes))
}

/// While the machine wants nothing and no RESTART request waits: one liveness per second.
async fn serve(fs: &Fs<'_, '_>, row: &mut Row) {
    let dev = row.dev;
    while row.action == Action::None && !dev.requested(REQ_RESTART) {
        let _ = select(Timer::after(LIVENESS_EVERY), dev.wake.wait()).await;
        dev.mark();
        if dev.requested(REQ_RESTART) {
            return;
        }
        let began = Instant::now();
        let alive = liveness(fs).await;
        dev.mark();
        let event = match alive {
            Ok(()) => {
                let t_us = began.as_micros();
                let status = &mut row.status;
                if let Some(last) = status.last_sample_us {
                    status.max_gap_us = status.max_gap_us.max(t_us.saturating_sub(last));
                }
                status.last_sample_us = Some(t_us);
                status.samples = status.samples.wrapping_add(1);
                Event::Good
            }
            Err(e) => {
                let kind = fs_kind(&e);
                defmt::info!("[INFO] SD: liveness failed: {}", defmt::Debug2Format(&e));
                if kind == ErrorKind::Timeout
                    && crate::settings::overlaps_save(began, Instant::now())
                {
                    device::discard_for_save(dev, "liveness time-out");
                    Event::Tick
                } else {
                    Event::Error(kind)
                }
            }
        };
        row.feed(event);
    }
}

/// The FAT's status flags (its first block) and the root directory's `LOGS` entry (the root's
/// first block): two blocks, so each is read from the card.
async fn liveness(fs: &Fs<'_, '_>) -> Result<(), FsError> {
    fs.read_status_flags().await?;
    fs.root_dir().open_dir("LOGS").await?;
    Ok(())
}

/// Q1-I1 and Q1-I2, once, after the first bus was built. `sd-irq-negative` sets INTE0 bit 4 first
/// (and clears it again after the checks): the INTE0 check must fail.
fn register_checks() {
    #[cfg(feature = "sd-irq-negative")]
    alias_write(pac::DMA.inte(0), BIT, 0x2000);
    let inte = [0, 1, 2, 3].map(|n| pac::DMA.inte(n).read());
    #[cfg(feature = "sd-irq-negative")]
    alias_write(pac::DMA.inte(0), BIT, 0x3000);
    defmt::info!(
        "[INFO] SD checks after the bus was built: INTE0 {=u32:#010x} INTE1 {=u32:#010x} INTE2 {=u32:#010x} INTE3 {=u32:#010x}",
        inte[0],
        inte[1],
        inte[2],
        inte[3]
    );
    let irq0 = interrupt::DMA_IRQ_0.is_enabled();
    let irq1 = interrupt::DMA_IRQ_1.is_enabled();
    let core1 = crate::CORE1_DMA_IRQ1.load(Ordering::Acquire);
    crate::check(
        inte[0] == FLIGHT_CHANNELS,
        "SD: INTE0 = exactly the flight core's channels 0-3, INTE0",
        inte[0],
    );
    crate::check(inte[1] == BIT, "SD: INTE1 = 1 << 4, INTE1", inte[1]);
    crate::check(
        inte[2] == 0 && inte[3] == 0,
        "SD: INTE2 = INTE3 = 0, INTE2 | INTE3",
        inte[2] | inte[3],
    );
    crate::check(
        !irq0 && irq1,
        "SD: on core 0 DMA_IRQ_1 enabled and DMA_IRQ_0 not (bit 1 IRQ_1, bit 0 IRQ_0)",
        u32::from(irq1) << 1 | u32::from(irq0),
    );
    crate::check(
        core1 == 1,
        "SD: DMA_IRQ_1 not enabled on core 1 (0 not read, 1 off, 2 on)",
        u32::from(core1),
    );
}

/// `sd-irq-negative`: writes `bits` to the SET (+0x2000) or CLR (+0x3000) alias of `reg`.
#[cfg(feature = "sd-irq-negative")]
fn alias_write(reg: pac::common::Reg<u32, pac::common::RW>, bits: u32, alias: usize) {
    // SAFETY: RP2350 datasheet 2.1.3 "Atomic register access": the DMA block has the atomic SET
    // alias at +0x2000 and the CLR alias at +0x3000 (pio-sdmmc's `write_set`/`write_clear` do the
    // same); `reg` comes from the PAC, so its alias is a register too.
    unsafe {
        pac::common::Reg::<u32, pac::common::RW>::from_ptr(
            reg.as_ptr().cast::<u8>().wrapping_add(alias).cast(),
        )
    }
    .write_value(bits);
}
