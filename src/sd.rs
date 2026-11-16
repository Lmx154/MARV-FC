//! The SD card as a device row and the log writer (decision-sd-logging Q1, Q2, Q4, Q6; milestones
//! SD2, SD3): one core-0 thread-mode task owns the bus (PIO0 SM0, DMA channel 4 completing on
//! `DMA_IRQ_1` on core 0), the card, the mounted FAT32 volume and the open log, drives row 9
//! through the shared `Health` machine with its own loop, and is the one consumer of the log ring
//! (`logger::RING`; `logger`'s collector is its one producer).
//!
//! A start, a recovery and a restart are the same: a new bus, `init`, block 0 through
//! `fw_core::mbr` (partition 1 of type 0x0B/0x0C), the mount over `BufStream` + `StreamSlice`,
//! `LOGS/` opened (created if missing) and listed once; then the new log (`fw_core::logfile`): the
//! number after the one `LOGS/LASTLOG.TXT` holds (or the listing's largest), the clean-up
//! (`plan_cleanup` over the listing, as ArduPilot's `Prep_MinSpace`: one text and one deletion per
//! step until nothing is left to delete, or "SD: card full": Failed, no log), a stale file of that
//! number deleted, `LASTLOG.TXT` written BEFORE the log is opened (Q4-I3), `LOGS/NNNNNNNN.BIN`
//! created and the header (the FMT records) written; then the ring is emptied, the collector is
//! told the number and the filler length (`logger::OPEN`, `logger::PAD`) and the row is Ok with
//! one NOTICE ("SD: 59.5 GiB FAT32, 17 MiB in 2 logs, log 3"). A deletion is one awaited call of
//! about 10 ms per 32 KiB cluster (50 s for 161 MB): it runs only here, never while a log is
//! written, and the row is kept out of the stuck check by a progress mark every second.
//!
//! The service: wait (no card call) for a piece in the ring, 1 s or the row's wake; write whole
//! 4096-byte pieces in place from the ring and release them after the write returned; after 1 s
//! without a write the bytes that wait; `flush` once per second, then the DSF figures for the
//! collector. The file offset and the ring's read index are kept equal modulo 4096 (the filler
//! before the log's first record makes the read index equal to the header's length), so a short
//! piece is followed by one that ends at the next 4096-byte boundary of both, and later pieces
//! are whole and in place again. A failed write is retried once at once; a second failure is one error event and the
//! piece is discarded. A time-out that overlapped a settings save is retried with no event. A
//! good write is the row's sample (`samples` = 512-byte blocks written, `overflows` = records the
//! ring dropped). Every card and file-system call is awaited to its end and a RESTART request is
//! taken between them (Q2-I1); a recovery or a restart closes the log, un-mounts and opens the
//! next log number.
//!
//! Row (Q6): `NoCard` on every start try: Absent, TIMEOUT, "SD: no card"; block 0 not an MBR with a
//! FAT32 partition 1, or a volume that does not mount as FAT32: Failed, ID_MISMATCH, "SD: no FAT32
//! volume, reformat as FAT32" (the machine parks the row after the start retries; this task shows
//! why); logging: Ok with one NOTICE; card full: Failed, "SD: card full"; `LOG_BACKEND_TYPE` 0 at
//! boot: Absent, NONE, "SD: logging off", the card untouched, nothing collected. Errors in service
//! go through the machine and its ladder (BUS or TIMEOUT, the supervisor's texts).
//!
//! Test hooks: `inject-sd-write-fail` (60 s after the log opened the next six write calls fail
//! without card traffic: three pieces, each with its retry); `log-verify` (after the mount, before
//! the new log, the previous log is read through `dflog::walk::Walker` and summarised);
//! `inject-log-small-volume` (the first open's clean-up is told a volume of the reserve plus the
//! listed bytes minus one, so the oldest log is deleted before the log opens); `inject-log-delete-big`
//! (the first open's clean-up is told a volume of the reserve plus the bytes of the logs after the
//! largest one in ring order, so every log up to and including the largest is deleted).

use core::sync::atomic::Ordering;

use atomic_waker::AtomicWaker;
use block_device_adapters::{BufStream, BufStreamError, StreamSlice, StreamSliceError};
use embassy_futures::select::{Either, select, select4};
use embassy_rp::Peri;
use embassy_rp::interrupt::{self, InterruptExt};
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH4, PIN_28, PIN_29, PIN_30, PIN_31, PIN_32, PIN_33, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_time::{Duration, Instant, Timer};
use embedded_fatfs::{
    DefaultTimeProvider, FatType, File, FileSystem, FsOptions, LossyOemCpConverter,
};
use embedded_io_async::{Read, Seek, SeekFrom, Write};
use fw_core::dflog::{Dsf, HEADER_SEQUENCE_LEN, write_header};
use fw_core::health::{Action, Counters, ErrorKind, Event, Health, Limits, Report, State};
use fw_core::logfile::{
    CleanUp, LASTLOG_NAME, LOG_DIR, MAX_LOG_NUMBER, RESERVE_BYTES, file_name, format_lastlog,
    last_number, next_number, parse_file_name, parse_lastlog, plan_cleanup,
};
use fw_core::mbr::{Partition, partition1};
use fw_core::watchdog::task;
use fw_core::xcore::{LogConsumer, LogProducer};
use fw_flight::tagged;
use mavlink_device::ParamValue;
use mavlink_device::dialect::MavSeverity;
use pio_sdmmc::pio::{DmaIrqHandler, DmaWaker, PioBus, PioBusConfig};
use pio_sdmmc::{Block, BusError, Config, Error as CardError, SdCard};
use static_cell::ConstStaticCell;

use crate::device::{self, Device, DeviceStatus, REQ_RESTART, SD, error_name, state_name};
use crate::logdl::{self, CHUNK, DATA_HEADER, ENTRY_LEN, HANDOVER, HANDOVER_BYTES, Request};
use crate::logger::{DATA, DSF_FIGURES, OPEN, PAD, PIECE, RING, RING_BYTES};

/// The row's nominal period P: the service's 1 s (the flush, the longest wait).
pub const PERIOD_US: u32 = 1_000_000;
const SECOND: Duration = Duration::from_secs(1);
/// Longest wait while parked, as in `device`.
const SLICE: Duration = Duration::from_millis(100);
/// The SD card's channel (resources.md DMA row 4).
const CH: usize = 4;
const BIT: u32 = 1 << CH;
/// INTE0 as the flight core leaves it: IMU1's channels 0 and 1, IMU2's 2 and 3.
const FLIGHT_CHANNELS: u32 = 0x0f;
/// At most this many entries of `LOGS/` are listed.
const LIST_MAX: usize = 1024;
/// One slot per log number (slot 0 unused).
const LOG_SLOTS: usize = MAX_LOG_NUMBER as usize + 1;
/// A listing slot with no file.
const ABSENT: u32 = u32::MAX;
/// Attempts of one write or flush whose time-outs overlapped a settings save.
const SAVE_RETRIES: u32 = 3;
/// 512-byte blocks per GiB.
const GIB_BLOCKS: u64 = 1 << 21;
const NO_CARD: &str = "SD: no card";
const WRONG_FORMAT: &str = "SD: no FAT32 volume, reformat as FAT32";
const CARD_FULL: &str = "SD: card full";

type Sd<'a> = SdCard<PioBus<'a, PIO0, 0>>;
type Volume<'a> = StreamSlice<BufStream<Sd<'a>, 512>>;
type Fs<'v, 'a> = FileSystem<&'v mut Volume<'a>, DefaultTimeProvider, LossyOemCpConverter>;
type FsError = embedded_fatfs::Error<StreamSliceError<BufStreamError<CardError>>>;
type LogFile<'f, 'v, 'a> = File<'f, &'v mut Volume<'a>, DefaultTimeProvider, LossyOemCpConverter>;

/// The `LOGS/` listing: per log number its file size (`ABSENT`: none).
static SIZES: ConstStaticCell<[u32; LOG_SLOTS]> = ConstStaticCell::new([ABSENT; LOG_SLOTS]);
/// The header of every log (the FMT records), built once.
static HEADER: ConstStaticCell<[u8; HEADER_SEQUENCE_LEN]> =
    ConstStaticCell::new([0; HEADER_SEQUENCE_LEN]);
/// `log-verify`: the read buffer, 4-byte aligned so that whole blocks are read in place.
#[cfg(feature = "log-verify")]
#[repr(C, align(4))]
struct VerifyBuf([u8; PIECE]);
#[cfg(feature = "log-verify")]
static VERIFY_BUF: ConstStaticCell<VerifyBuf> = ConstStaticCell::new(VerifyBuf([0; PIECE]));

/// Log bytes a download step reads at most (10 LOG_DATA).
const READ_CHUNK: usize = 10 * CHUNK;
/// Room a whole read step needs in the hand-over ring.
const READ_ROOM: usize = READ_CHUNK + 10 * DATA_HEADER;
/// The download's read buffer.
static READ_BUF: ConstStaticCell<[u8; READ_CHUNK]> = ConstStaticCell::new([0; READ_CHUNK]);

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
    /// The card is full (`plan_cleanup`): Failed, no log, parked until a RESTART request.
    full: bool,
}

impl Row {
    /// The state and counters shown: the machine's, except that a row parked Absent by its start
    /// retries shows why (Q6).
    fn shown(&self) -> (State, Counters) {
        let mut counters = *self.health.counters();
        counters.overflows = RING.dropped();
        let mut state = self.health.state();
        if self.full {
            state = State::Failed;
        }
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
        self.full = false;
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

    /// The clean-up planner said the card is full: Failed, no log, parked until a RESTART request.
    fn card_full(&mut self) {
        self.full = true;
        let before = self.status.state;
        self.publish();
        device::STATE_CHANGES.fetch_add(1, Ordering::Relaxed);
        defmt::info!(
            "[INFO] SD: {=str} -> Failed at {=u64} ms: the card is full, no log",
            state_name(before),
            now_ms()
        );
        text(MavSeverity::MAV_SEVERITY_ERROR, CARD_FULL);
        if let Some(number) = self.restart_run.take() {
            self.dev.finish_restart(number, false);
        }
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
    let Some(consumer) = RING.take_consumer() else {
        defmt::panic!("[FAIL] log ring: consumer taken twice");
    };
    let Some(handover) = HANDOVER.take_producer() else {
        defmt::panic!("[FAIL] hand-over ring: producer taken twice");
    };
    let header = HEADER.take();
    write_header(header);
    let mut w = LogState {
        consumer,
        tail: 0,
        pad_left: 0,
        sizes: SIZES.take(),
        header,
        written: 0,
        inject_calls: 0,
        handover,
        read_buf: READ_BUF.take(),
        #[cfg(feature = "inject-sd-write-fail")]
        injected: false,
        #[cfg(any(feature = "inject-log-small-volume", feature = "inject-log-delete-big"))]
        pretend_told: false,
        #[cfg(feature = "log-verify")]
        verify_buf: &mut VERIFY_BUF.take().0,
    };
    let mut row = Row {
        dev,
        health: Health::new(Limits::DESIGN),
        status: dev.status(),
        action: Action::None,
        start_fault: None,
        restart_run: None,
        checked: false,
        full: false,
    };
    row.feed(Event::Tick);
    loop {
        row.take_restart();
        if row.full {
            // No card traffic; a RESTART request arrives through `wake`.
            let _ = select(Timer::after(SLICE), dev.wake.wait()).await;
            dev.mark();
            continue;
        }
        match row.action {
            Action::Start | Action::Recover | Action::Restart => {
                session(&mut parts, &mut row, &mut w).await
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
async fn session(parts: &mut Parts, row: &mut Row, w: &mut LogState) {
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
    mounted(&mut volume, part, row, op, began, w).await;
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

/// What the writer keeps across sessions.
struct LogState {
    consumer: LogConsumer<'static, RING_BYTES>,
    /// The ring's read index: bytes consumed since boot (wrapping).
    tail: u32,
    /// Filler bytes of the open log still to consume unwritten.
    pad_left: usize,
    /// The `LOGS/` listing: per log number its size, `ABSENT` for none.
    sizes: &'static mut [u32; LOG_SLOTS],
    header: &'static mut [u8; HEADER_SEQUENCE_LEN],
    /// Bytes written to logs since boot (the row's samples are these in 512-byte blocks).
    written: u64,
    /// Write calls still to fail (`inject-sd-write-fail`; 0 otherwise).
    inject_calls: u8,
    /// The log download's answers to the USB task.
    handover: LogProducer<'static, HANDOVER_BYTES>,
    read_buf: &'static mut [u8; READ_CHUNK],
    #[cfg(feature = "inject-sd-write-fail")]
    injected: bool,
    /// The clean-up hooks' pretended volume was used (once per boot).
    #[cfg(any(feature = "inject-log-small-volume", feature = "inject-log-delete-big"))]
    pretend_told: bool,
    #[cfg(feature = "log-verify")]
    verify_buf: &'static mut [u8; PIECE],
}

impl LogState {
    fn consume(&mut self, n: usize) {
        let n = self.consumer.consume(n);
        self.tail = self.tail.wrapping_add(n as u32);
    }

    /// Empties the ring (nothing is pushed while no log is open).
    fn drain(&mut self) {
        let n = self.consumer.readable();
        self.consume(n);
    }

    /// The listing as `(number, size)`.
    fn listing(&self) -> impl Iterator<Item = (u32, u64)> + '_ {
        (0u32..)
            .zip(self.sizes.iter())
            .filter(|&(_, &size)| size != ABSENT)
            .map(|(n, &size)| (n, u64::from(size)))
    }
}

/// The open log.
struct Open {
    number: u32,
    /// The file offset: bytes written to the file.
    offset: u64,
    opened_at: Instant,
}

/// Why the service ended.
#[derive(Clone, Copy, PartialEq, Eq)]
enum End {
    /// The machine wants another operation (recovery after errors).
    Machine,
    /// A RESTART request waits.
    Restart,
}

/// The path of log `n`, `LOGS/NNNNNNNN.BIN`.
fn log_path(n: u32) -> [u8; 17] {
    let mut path = *b"LOGS/00000000.BIN";
    if let Some(name) = path.get_mut(5..) {
        name.copy_from_slice(&file_name(n));
    }
    path
}

fn as_str(bytes: &[u8]) -> &str {
    core::str::from_utf8(bytes).unwrap_or("?")
}

/// The mount, the new log, the notice, the service, the log closed, the un-mount.
async fn mounted(
    volume: &mut Volume<'_>,
    part: Partition,
    row: &mut Row,
    op: Action,
    began: Instant,
    w: &mut LogState,
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
    let listed = list(&fs, w.sizes).await;
    row.dev.mark();
    let listing = match listed {
        Ok(v) => v,
        Err(e) => {
            let kind = fs_kind(&e);
            let _ = fs.unmount().await;
            row.dev.mark();
            return row.failed(op, Fault::Error(kind), began);
        }
    };
    let lastlog = read_lastlog(&fs).await;
    row.dev.mark();
    let lastlog = match lastlog {
        Ok(v) => v,
        Err(e) => {
            let kind = fs_kind(&e);
            let _ = fs.unmount().await;
            row.dev.mark();
            return row.failed(op, Fault::Error(kind), began);
        }
    };
    #[cfg(feature = "log-verify")]
    verify(&fs, lastlog.or(listing.max), row.dev, w.verify_buf).await;
    let logged = logging(&fs, part, row, op, began, mount_us, &listing, lastlog, w).await;
    if let Err(kind) = logged {
        let _ = fs.unmount().await;
        row.dev.mark();
        return row.failed(op, Fault::Error(kind), began);
    }
    let t = Instant::now();
    let unmounted = fs.unmount().await;
    row.dev.mark();
    defmt::info!(
        "[INFO] SD: un-mounted in {=u64} us, ok {=bool}",
        t.elapsed().as_micros(),
        unmounted.is_ok()
    );
}

/// The new log, the notice, the service, the log closed. `Err` if the log could not be opened.
#[allow(clippy::too_many_arguments)]
async fn logging(
    fs: &Fs<'_, '_>,
    part: Partition,
    row: &mut Row,
    op: Action,
    began: Instant,
    mount_us: u64,
    listing: &Listing,
    lastlog: Option<u32>,
    w: &mut LogState,
) -> Result<(), ErrorKind> {
    let number = next_number(Some(last_number(lastlog, listing.max)));
    match clean_up(fs, number, part, row, w).await {
        Ok(true) => {
            row.card_full();
            return Ok(());
        }
        Ok(false) => {}
        Err(kind) => return Err(kind),
    }
    let t_open = Instant::now();
    let opened = open_log(fs, number, w).await;
    row.dev.mark();
    let mut file = match opened {
        Ok(file) => file,
        Err(e) => {
            let kind = fs_kind(&e);
            defmt::info!(
                "[INFO] SD: log {=u32} not opened: {}",
                number,
                defmt::Debug2Format(&e)
            );
            return Err(kind);
        }
    };
    // The hand-off: the ring is empty (nothing is pushed while `OPEN` is 0); the filler makes the
    // read index equal to the file offset (the header's length) modulo one piece.
    w.drain();
    let pad = (HEADER_SEQUENCE_LEN as u32).wrapping_sub(w.tail) as usize % PIECE;
    w.pad_left = pad;
    PAD.store(pad as u32, Ordering::Relaxed);
    OPEN.store(number, Ordering::Release);
    let tenths = (u64::from(part.blocks) * 10 + GIB_BLOCKS / 2) / GIB_BLOCKS;
    let used_mib = (listing.log_bytes + (1 << 19)) >> 20;
    defmt::info!(
        "[INFO] SD: {=str} done in {=u64} ms: partition 1 type {=u8:#04x} from block {=u32}, {=u32} blocks; mounted in {=u64} us, cluster {=u32} B, label {=[u8]:a}; LOGS/ holds {=u32} logs ({=u64} B) and {=u32} other files, LASTLOG.TXT {=?}; log {=u32} opened in {=u64} us, filler {=usize} B",
        op_name(op),
        began.elapsed().as_millis(),
        part.kind,
        part.first_block,
        part.blocks,
        mount_us,
        fs.cluster_size(),
        fs.volume_label_as_bytes(),
        listing.logs,
        listing.log_bytes,
        listing.others,
        lastlog,
        number,
        t_open.elapsed().as_micros(),
        pad
    );
    let line = device::TextLine::new(format_args!(
        "SD: {}.{} GiB FAT32, {} MiB in {} logs, log {}",
        tenths / 10,
        tenths % 10,
        used_mib,
        listing.logs,
        number
    ));
    text(MavSeverity::MAV_SEVERITY_NOTICE, line.as_str());
    row.succeeded(op);
    let mut log = Open {
        number,
        offset: HEADER_SEQUENCE_LEN as u64,
        opened_at: Instant::now(),
    };
    let end = serve(fs, &mut file, &mut log, row, w).await;
    OPEN.store(0, Ordering::Release);
    if end == End::Restart {
        // What waits in the ring goes into this log first (the collector has stopped).
        for _ in 0..RING_BYTES / PIECE + 1 {
            if w.pad_left > 0 {
                let n = w.pad_left.min(w.consumer.readable());
                w.consume(n);
                w.pad_left -= n;
            }
            let readable = w.consumer.readable();
            if readable == 0 {
                break;
            }
            let len = (PIECE - w.tail as usize % PIECE).min(readable);
            let (piece, _) = w.consumer.peek_up_to(len);
            let written = write_piece(&mut file, piece, row.dev, &mut w.inject_calls).await;
            row.dev.mark();
            let len = piece.len();
            w.consume(len);
            match written {
                Ok(n) => {
                    log.offset += n as u64;
                    w.written += n as u64;
                }
                Err(_) => break,
            }
        }
    }
    w.pad_left = 0;
    let t = Instant::now();
    let closed = file.close().await;
    row.dev.mark();
    defmt::info!(
        "[INFO] SD: log {=u32} closed after {=u64} s in {=u64} us, ok {=bool}: {=u64} B, {=u32} records dropped in all",
        log.number,
        log.opened_at.elapsed().as_secs(),
        t.elapsed().as_micros(),
        closed.is_ok(),
        log.offset,
        RING.dropped()
    );
    Ok(())
}

/// What one listing of `LOGS/` found.
struct Listing {
    logs: u32,
    log_bytes: u64,
    others: u32,
    /// The largest log number.
    max: Option<u32>,
}

/// `LOGS/`, created if missing, listed once (bounded) into `sizes`.
async fn list(fs: &Fs<'_, '_>, sizes: &mut [u32; LOG_SLOTS]) -> Result<Listing, FsError> {
    sizes.fill(ABSENT);
    let dir = fs.root_dir().create_dir(LOG_DIR).await?;
    let mut entries = dir.iter();
    let mut listing = Listing {
        logs: 0,
        log_bytes: 0,
        others: 0,
        max: None,
    };
    for _ in 0..LIST_MAX {
        let Some(entry) = entries.next().await else {
            break;
        };
        let entry = entry?;
        if !entry.is_file() {
            continue;
        }
        match parse_file_name(entry.short_file_name_as_bytes()) {
            Some(n) => {
                if let Some(slot) = sizes.get_mut(n as usize) {
                    *slot = u32::try_from(entry.len()).unwrap_or(ABSENT).min(ABSENT - 1);
                }
                listing.logs += 1;
                listing.log_bytes += entry.len();
                listing.max = listing.max.max(Some(n));
            }
            None => listing.others += 1,
        }
    }
    Ok(listing)
}

/// The number `LOGS/LASTLOG.TXT` holds; `None` if the file is missing or does not parse.
async fn read_lastlog(fs: &Fs<'_, '_>) -> Result<Option<u32>, FsError> {
    let dir = fs.root_dir().open_dir(LOG_DIR).await?;
    let mut file = match dir.open_file(LASTLOG_NAME).await {
        Ok(file) => file,
        Err(embedded_fatfs::Error::NotFound) => return Ok(None),
        Err(e) => return Err(e),
    };
    let mut buf = [0u8; 16];
    let mut n = 0;
    while let Some(rest) = buf.get_mut(n..)
        && !rest.is_empty()
    {
        match file.read(rest).await {
            Ok(0) => break,
            Ok(k) => n += k,
            Err(e) => {
                let _ = file.close().await;
                return Err(e);
            }
        }
    }
    file.close().await?;
    Ok(parse_lastlog(buf.get(..n).unwrap_or(&[])))
}

/// Log `number`: a stale file of that number deleted, `LASTLOG.TXT` written (Q4-I3), the log
/// created and its header written.
async fn open_log<'f, 'v, 'a>(
    fs: &'f Fs<'v, 'a>,
    number: u32,
    w: &mut LogState,
) -> Result<LogFile<'f, 'v, 'a>, FsError> {
    let dir = fs.root_dir().open_dir(LOG_DIR).await?;
    let name = file_name(number);
    let name = as_str(&name);
    if let Some(slot) = w.sizes.get_mut(number as usize)
        && *slot != ABSENT
    {
        let t = Instant::now();
        dir.remove(name).await?;
        defmt::info!(
            "[INFO] SD: stale LOGS/{=str} ({=u32} B) deleted in {=u64} us",
            name,
            *slot,
            t.elapsed().as_micros()
        );
        *slot = ABSENT;
    }
    let mut digits = [0u8; 12];
    let len = format_lastlog(number, &mut digits);
    let mut lastlog = dir.create_file(LASTLOG_NAME).await?;
    let written = async {
        lastlog.write_all(digits.get(..len).unwrap_or(&[])).await?;
        lastlog.truncate().await
    }
    .await;
    let closed = lastlog.close().await;
    written?;
    closed?;
    let mut file = dir.create_file(name).await?;
    if let Err(e) = file.write_all(&w.header[..]).await {
        let _ = file.close().await;
        return Err(e);
    }
    if let Some(slot) = w.sizes.get_mut(number as usize) {
        *slot = HEADER_SEQUENCE_LEN as u32;
    }
    w.written += HEADER_SEQUENCE_LEN as u64;
    Ok(file)
}

/// Writes `piece` to the log: one immediate retry of a failed write call, and a time-out that
/// overlapped a settings save retried with no event. `Ok(bytes)`, or the error and the bytes
/// written before it.
async fn write_piece(
    file: &mut LogFile<'_, '_, '_>,
    piece: &[u8],
    dev: &'static Device,
    inject_calls: &mut u8,
) -> Result<usize, (ErrorKind, usize)> {
    let mut done = 0;
    let mut retried = false;
    let mut save_retries = 0;
    while let Some(rest) = piece.get(done..)
        && !rest.is_empty()
    {
        let began = Instant::now();
        let result = if *inject_calls > 0 {
            *inject_calls -= 1;
            Err(ErrorKind::Bus)
        } else {
            match file.write(rest).await {
                // No byte written: the file is at its size limit or the card is full.
                Ok(0) => Err(ErrorKind::Bus),
                Ok(n) => Ok(n),
                Err(e) => {
                    defmt::info!(
                        "[INFO] SD: write of {=usize} B failed after {=u64} us: {}",
                        rest.len(),
                        began.elapsed().as_micros(),
                        defmt::Debug2Format(&e)
                    );
                    Err(fs_kind(&e))
                }
            }
        };
        match result {
            Ok(n) => done += n,
            Err(kind) => {
                if kind == ErrorKind::Timeout
                    && save_retries < SAVE_RETRIES
                    && crate::settings::overlaps_save(began, Instant::now())
                {
                    save_retries += 1;
                    device::discard_for_save(dev, "write time-out");
                } else if !retried {
                    retried = true;
                    defmt::info!(
                        "[INFO] SD: write failed ({=str}), retrying",
                        error_name(kind)
                    );
                } else {
                    return Err((kind, done));
                }
            }
        }
    }
    Ok(done)
}

/// `flush` with the same retries as a write.
async fn flush(file: &mut LogFile<'_, '_, '_>, dev: &'static Device) -> Result<(), ErrorKind> {
    let mut retried = false;
    let mut save_retries = 0;
    loop {
        let began = Instant::now();
        let Err(e) = file.flush().await else {
            return Ok(());
        };
        let kind = fs_kind(&e);
        defmt::info!(
            "[INFO] SD: flush failed after {=u64} us: {}",
            began.elapsed().as_micros(),
            defmt::Debug2Format(&e)
        );
        if kind == ErrorKind::Timeout
            && save_retries < SAVE_RETRIES
            && crate::settings::overlaps_save(began, Instant::now())
        {
            save_retries += 1;
            device::discard_for_save(dev, "flush time-out");
        } else if !retried {
            retried = true;
        } else {
            return Err(kind);
        }
    }
}

/// The writer's loop while the machine wants nothing and no RESTART request waits.
async fn serve<'f, 'v, 'a>(
    fs: &'f Fs<'v, 'a>,
    file: &mut LogFile<'f, 'v, 'a>,
    log: &mut Open,
    row: &mut Row,
    w: &mut LogState,
) -> End {
    let dev = row.dev;
    let mut last_write = Instant::now();
    let mut last_flush = Instant::now();
    let mut download = Download::new();
    // When the last download step began, until the loop has looked at what is due after it.
    let mut step_began: Option<Instant> = None;
    loop {
        if dev.requested(REQ_RESTART) {
            download.close().await;
            return End::Restart;
        }
        if row.action != Action::None {
            download.close().await;
            return End::Machine;
        }
        #[cfg(feature = "inject-sd-write-fail")]
        if !w.injected && log.opened_at.elapsed() >= Duration::from_secs(60) {
            w.injected = true;
            w.inject_calls = 6;
            defmt::info!(
                "[INFO] SD: inject-sd-write-fail: the next 6 write calls of log {=u32} fail",
                log.number
            );
        }
        if w.pad_left > 0 {
            let n = w.pad_left.min(w.consumer.readable());
            w.consume(n);
            w.pad_left -= n;
        }
        let now = Instant::now();
        let readable = if w.pad_left > 0 {
            0
        } else {
            w.consumer.readable()
        };
        if readable == 0 {
            last_write = now;
        }
        let need = PIECE - w.tail as usize % PIECE;
        let write_due = readable >= need || now >= last_write + SECOND;
        let flush_due = now >= last_flush + SECOND;
        if let Some(began) = step_began.take()
            && (write_due || flush_due)
        {
            let late = u32::try_from(began.elapsed().as_micros()).unwrap_or(u32::MAX);
            logdl::LATE_MAX_US.fetch_max(late, Ordering::Relaxed);
        }
        if !write_due && !flush_due {
            // The log download only between pieces, one bounded step at a time, while Ok.
            if row.status.state == State::Ok {
                let began = Instant::now();
                if download.step(fs, w, log.number).await {
                    dev.mark();
                    let took = u32::try_from(began.elapsed().as_micros()).unwrap_or(u32::MAX);
                    logdl::STEP_MAX_US.fetch_max(took, Ordering::Relaxed);
                    step_began = Some(began);
                    continue;
                }
            }
            let until = (last_write + SECOND).min(last_flush + SECOND);
            let _ = select4(
                DATA.wait(),
                Timer::at(until),
                dev.wake.wait(),
                logdl::WAKE.wait(),
            )
            .await;
            dev.mark();
            continue;
        }
        if write_due {
            let (piece, _) = w.consumer.peek_up_to(need.min(readable));
            let began = Instant::now();
            let written = write_piece(file, piece, dev, &mut w.inject_calls).await;
            dev.mark();
            let len = piece.len();
            w.consume(len);
            last_write = Instant::now();
            let event = match written {
                Ok(n) => {
                    log.offset += n as u64;
                    w.written += n as u64;
                    let t_us = began.as_micros();
                    let status = &mut row.status;
                    if let Some(last) = status.last_sample_us {
                        status.max_gap_us = status.max_gap_us.max(t_us.saturating_sub(last));
                    }
                    status.last_sample_us = Some(t_us);
                    status.samples = u32::try_from(w.written / 512).unwrap_or(u32::MAX);
                    Event::Good
                }
                Err((kind, n)) => {
                    log.offset += n as u64;
                    w.written += n as u64;
                    defmt::info!(
                        "[INFO] SD: log {=u32}: a piece of {=usize} B discarded after its retry failed ({=str}), {=usize} B of it written",
                        log.number,
                        len,
                        error_name(kind),
                        n
                    );
                    Event::Error(kind)
                }
            };
            row.feed(event);
        }
        if flush_due {
            let flushed = flush(file, dev).await;
            dev.mark();
            last_flush = Instant::now();
            if let Err(kind) = flushed {
                row.feed(Event::Error(kind));
                continue;
            }
            let stats = w.consumer.take_stats();
            DSF_FIGURES.lock(|c| {
                c.set(Some(Dsf {
                    dropped: 0,
                    blocks: u32::try_from(log.offset / 512).unwrap_or(u32::MAX),
                    bytes: u32::try_from(log.offset).unwrap_or(u32::MAX),
                    free_min: stats.free_min,
                    free_max: stats.free_max,
                    free_avg: stats.free_avg,
                }))
            });
            if let Some(slot) = w.sizes.get_mut(log.number as usize) {
                *slot = u32::try_from(log.offset)
                    .unwrap_or(ABSENT - 1)
                    .min(ABSENT - 1);
            }
        }
    }
}

/// What the SD task does for the log download (`logdl`).
#[derive(Clone, Copy)]
enum Job {
    /// Nothing; a read handle is closed.
    Idle,
    /// Nothing more for this request; a read handle stays open for the next request.
    Done,
    /// The LOG_ENTRY of the logs from `next` to `end`.
    List {
        next: u32,
        end: u32,
        num_logs: u16,
        last: u16,
    },
    /// The one LOG_ENTRY with id 0: no logs.
    Empty,
    /// LOG_DATA for the bytes `pos..end` of log `id`.
    Data { id: u32, pos: u64, end: u64 },
    /// The one LOG_DATA with count 0: `ofs` at or past the end of log `id`.
    End { id: u32, ofs: u32 },
}

/// The download's read handle: a second file beside the log being written.
struct Reader<'f, 'v, 'a> {
    number: u32,
    /// The listed size when it was opened (the file's directory entry holds at least this).
    size: u64,
    /// The file offset.
    at: u64,
    file: LogFile<'f, 'v, 'a>,
}

/// The SD task's side of the log download.
struct Download<'f, 'v, 'a> {
    /// The generation of the request taken last.
    generation: u32,
    job: Job,
    reader: Option<Reader<'f, 'v, 'a>>,
}

impl<'f, 'v, 'a> Download<'f, 'v, 'a> {
    /// A request made before the service began is not answered.
    fn new() -> Self {
        Self {
            generation: logdl::request().0,
            job: Job::Idle,
            reader: None,
        }
    }

    async fn close(&mut self) {
        if let Some(reader) = self.reader.take() {
            let _ = reader.file.close().await;
        }
    }

    /// One bounded step: a new request taken, the read handle closed, opened or moved, at most
    /// `READ_CHUNK` bytes read and handed over, or LOG_ENTRY records handed over. `false`: nothing
    /// to do now (no request, or no room in the hand-over ring).
    async fn step(&mut self, fs: &'f Fs<'v, 'a>, w: &mut LogState, open: u32) -> bool {
        let (generation, request) = logdl::request();
        if generation != self.generation {
            self.generation = generation;
            self.job = job_of(request, w, open);
            return true;
        }
        if let Some(reader) = &self.reader {
            let keep = match self.job {
                Job::Data { id, end, .. } => id == reader.number && end <= reader.size,
                Job::Done => true,
                _ => false,
            };
            if !keep {
                self.close().await;
                return true;
            }
        }
        let room = w.handover.free();
        match self.job {
            Job::Idle | Job::Done => false,
            Job::Empty => {
                if room < ENTRY_LEN {
                    return false;
                }
                logdl::push_entry(&mut w.handover, generation, 0, 0, 0, 0);
                self.job = Job::Done;
                true
            }
            Job::End { id, ofs } => {
                if room < DATA_HEADER {
                    return false;
                }
                logdl::push_data(&mut w.handover, generation, id as u16, ofs, &[]);
                self.job = Job::Done;
                true
            }
            Job::List {
                mut next,
                end,
                num_logs,
                last,
            } => {
                if room < ENTRY_LEN {
                    return false;
                }
                while next <= end && w.handover.free() >= ENTRY_LEN {
                    if let Some(&size) = w.sizes.get(next as usize)
                        && size != ABSENT
                    {
                        logdl::push_entry(
                            &mut w.handover,
                            generation,
                            next as u16,
                            num_logs,
                            last,
                            size,
                        );
                    }
                    next += 1;
                }
                self.job = if next > end {
                    Job::Done
                } else {
                    Job::List {
                        next,
                        end,
                        num_logs,
                        last,
                    }
                };
                true
            }
            Job::Data { id, pos, end } => {
                let Some(reader) = self.reader.as_mut() else {
                    let size = w.sizes.get(id as usize).copied().unwrap_or(0);
                    let path = log_path(id);
                    match fs.root_dir().open_file(as_str(&path)).await {
                        Ok(file) => {
                            self.reader = Some(Reader {
                                number: id,
                                size: u64::from(size),
                                at: 0,
                                file,
                            })
                        }
                        Err(e) => {
                            defmt::info!(
                                "[INFO] SD: download: {=str} not opened: {}",
                                as_str(&path),
                                defmt::Debug2Format(&e)
                            );
                            self.job = Job::Done;
                        }
                    }
                    return true;
                };
                if reader.at != pos {
                    let sought = reader.file.seek(SeekFrom::Start(pos)).await;
                    match sought {
                        Ok(at) if at == pos => reader.at = at,
                        r => {
                            defmt::info!(
                                "[INFO] SD: download: log {=u32}: seek to {=u64} B ended at {}",
                                id,
                                pos,
                                defmt::Debug2Format(&r)
                            );
                            self.job = Job::Done;
                            self.close().await;
                        }
                    }
                    return true;
                }
                if room < READ_ROOM {
                    return false;
                }
                let n = usize::try_from(end - pos)
                    .unwrap_or(READ_CHUNK)
                    .min(READ_CHUNK);
                let buf = w.read_buf.get_mut(..n).unwrap_or(&mut []);
                let read = reader.file.read(buf).await;
                match read {
                    Ok(k) if k > 0 => {
                        let mut ofs = pos;
                        for chunk in buf.get(..k).unwrap_or(&[]).chunks(CHUNK) {
                            logdl::push_data(
                                &mut w.handover,
                                generation,
                                id as u16,
                                ofs as u32,
                                chunk,
                            );
                            ofs += chunk.len() as u64;
                        }
                        reader.at = ofs;
                        self.job = if ofs >= end {
                            Job::Done
                        } else {
                            Job::Data { id, pos: ofs, end }
                        };
                    }
                    r => {
                        defmt::info!(
                            "[INFO] SD: download: log {=u32}: read at {=u64} B (end {=u64} B) gave {}",
                            id,
                            pos,
                            end,
                            defmt::Debug2Format(&r)
                        );
                        self.job = Job::Done;
                        self.close().await;
                    }
                }
                true
            }
        }
    }
}

/// The job of a request: answered from the listing; nothing for a log that does not exist.
fn job_of(request: Request, w: &LogState, open: u32) -> Job {
    match request {
        Request::Stop => Job::Idle,
        Request::List { start, end } => {
            let num_logs = u16::try_from(w.listing().count()).unwrap_or(u16::MAX);
            if num_logs == 0 {
                Job::Empty
            } else {
                Job::List {
                    next: u32::from(start).max(1),
                    end: u32::from(end).min(MAX_LOG_NUMBER),
                    num_logs,
                    last: u16::try_from(open).unwrap_or(0),
                }
            }
        }
        Request::Data { id, ofs, count } => {
            let size = match w.sizes.get(usize::from(id)) {
                Some(&size) if id != 0 && size != ABSENT => u64::from(size),
                _ => return Job::Idle,
            };
            let pos = u64::from(ofs);
            if pos >= size {
                return Job::End {
                    id: u32::from(id),
                    ofs,
                };
            }
            let end = pos.saturating_add(u64::from(count)).min(size);
            if end == pos {
                Job::Done
            } else {
                Job::Data {
                    id: u32::from(id),
                    pos,
                    end,
                }
            }
        }
    }
}

/// The clean-up before log `number` is opened (ArduPilot's `Prep_MinSpace`): `plan_cleanup` over the
/// listing until it says `Nothing`; per `Delete`, one text, the deletion (one awaited call, about
/// 10 ms per 32 KiB cluster: 50 s for 161 MB) and the listing updated. `Ok(true)`: the card is full.
/// While a deletion runs the row is marked every second (the supervisor's stuck check needs no
/// progress mark but the task's own) and the deletion future is polled by reference, never
/// dropped (Q2-I1). An error ends the open like any other error of the open.
async fn clean_up(
    fs: &Fs<'_, '_>,
    number: u32,
    part: Partition,
    row: &mut Row,
    w: &mut LogState,
) -> Result<bool, ErrorKind> {
    #[allow(unused_mut)]
    let mut volume_bytes = u64::from(part.blocks) * 512;
    #[cfg(any(feature = "inject-log-small-volume", feature = "inject-log-delete-big"))]
    if !core::mem::replace(&mut w.pretend_told, true) {
        volume_bytes = pretend_volume(w, number);
    }
    loop {
        let n = match plan_cleanup(volume_bytes, RESERVE_BYTES, number, w.listing()) {
            CleanUp::Nothing => return Ok(false),
            CleanUp::Full => return Ok(true),
            CleanUp::Delete(n) => n,
        };
        let path = log_path(n);
        let size = w.sizes.get(n as usize).copied().unwrap_or(0);
        let line = device::TextLine::new(format_args!("SD: clean-up, deleting log {}", n));
        text(MavSeverity::MAV_SEVERITY_NOTICE, line.as_str());
        let t = Instant::now();
        row.dev.mark();
        let root = fs.root_dir();
        let removed = {
            let mut remove = core::pin::pin!(root.remove(as_str(&path)));
            loop {
                match select(remove.as_mut(), Timer::after(SECOND)).await {
                    Either::First(r) => break r,
                    Either::Second(()) => row.dev.mark(),
                }
            }
        };
        row.dev.mark();
        defmt::info!(
            "[INFO] SD: clean-up: {=str} ({=u32} B) deleted in {=u64} ms, ok {=bool}",
            as_str(&path),
            size,
            t.elapsed().as_millis(),
            removed.is_ok()
        );
        match removed {
            Ok(()) => {
                if let Some(slot) = w.sizes.get_mut(n as usize) {
                    *slot = ABSENT;
                }
            }
            Err(e) => return Err(fs_kind(&e)),
        }
    }
}

/// The test hooks' pretended volume for the first open's clean-up (the whole loop), once per boot.
/// `inject-log-small-volume`: the reserve plus the listed bytes minus one, so the oldest log goes.
/// `inject-log-delete-big`: the reserve plus the bytes of the logs after the largest log other than
/// `number` in ring order (`number` itself, a stale file, counts as the newest), so the planner
/// deletes oldest first until the largest is gone and stops.
#[cfg(any(feature = "inject-log-small-volume", feature = "inject-log-delete-big"))]
fn pretend_volume(w: &LogState, number: u32) -> u64 {
    #[cfg(feature = "inject-log-delete-big")]
    {
        let rank = |n: u32| (n + MAX_LOG_NUMBER - 1 - number) % MAX_LOG_NUMBER;
        let largest = w
            .listing()
            .filter(|&(n, _)| n != number)
            .max_by_key(|&(n, size)| (size, rank(n)));
        let (volume, big) = match largest {
            Some((big, size)) => {
                let after: u64 = w
                    .listing()
                    .filter(|&(n, _)| rank(n) > rank(big))
                    .map(|(_, size)| size)
                    .sum();
                (RESERVE_BYTES + after, Some((big, size)))
            }
            None => (
                RESERVE_BYTES + w.listing().map(|(_, size)| size).sum::<u64>(),
                None,
            ),
        };
        defmt::info!(
            "[INFO] SD: inject-log-delete-big: the planner is told a volume of {=u64} B, so every log up to the largest {} goes",
            volume,
            big
        );
        volume
    }
    #[cfg(not(feature = "inject-log-delete-big"))]
    {
        let _ = number;
        let used: u64 = w.listing().map(|(_, size)| size).sum();
        let volume = RESERVE_BYTES + used - 1;
        defmt::info!(
            "[INFO] SD: inject-log-small-volume: the planner is told a volume of {=u64} B (the reserve + the {=u64} B listed - 1) once",
            volume,
            used
        );
        volume
    }
}

/// `log-verify`: the previous log read through the walker in 4 KiB pieces and summarised.
#[cfg(feature = "log-verify")]
async fn verify(fs: &Fs<'_, '_>, number: Option<u32>, dev: &'static Device, buf: &mut [u8; PIECE]) {
    use fw_core::dflog::TYPES;
    use fw_core::dflog::walk::Walker;
    let Some(n) = number else {
        defmt::info!("[INFO] log-verify: no previous log");
        return;
    };
    let path = log_path(n);
    let path = as_str(&path);
    let mut file = match fs.root_dir().open_file(path).await {
        Ok(file) => file,
        Err(e) => {
            dev.mark();
            defmt::info!(
                "[INFO] log-verify: {=str} not opened: {}",
                path,
                defmt::Debug2Format(&e)
            );
            return;
        }
    };
    let began = Instant::now();
    let mut walker = Walker::new();
    let mut size = 0u64;
    loop {
        let r = file.read(&mut buf[..]).await;
        dev.mark();
        match r {
            Ok(0) => break,
            Ok(k) => {
                walker.feed(buf.get(..k).unwrap_or(&[]));
                size += k as u64;
            }
            Err(e) => {
                defmt::info!(
                    "[INFO] log-verify: read failed at {=u64} B: {}",
                    size,
                    defmt::Debug2Format(&e)
                );
                break;
            }
        }
    }
    let _ = file.close().await;
    dev.mark();
    let s = walker.summary();
    defmt::info!(
        "[INFO] log-verify: {=str}: {=u64} B read in {=u64} ms; records {=u32}, bad bytes {=u32}, tail bytes {=usize}, began with FMT {=bool}, FMT matched {=u32} mismatched {=u32}",
        path,
        size,
        began.elapsed().as_millis(),
        s.records,
        s.bad_bytes,
        s.tail_bytes,
        s.began_with_fmt,
        s.fmt_matched,
        s.fmt_mismatched
    );
    for (def, t) in TYPES.iter().zip(s.types.iter()) {
        let span_us = t.last_time_us.saturating_sub(t.first_time_us);
        let rate = if span_us > 0 && t.count > 1 {
            (t.count - 1) as f32 * 1.0e6 / span_us as f32
        } else {
            0.0
        };
        defmt::info!(
            "[INFO] log-verify: {=str}: count {=u32}, TimeUS first {=u64} last {=u64}, rate {=f32}/s",
            def.name,
            t.count,
            t.first_time_us,
            t.last_time_us,
            rate
        );
    }
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
