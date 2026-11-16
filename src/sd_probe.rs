//! Probe P3 of the SD-card logging design (features `sd-fat-probe`, `sd-fat-format`;
//! decision-sd-logging Q1, Q2): the SD card through `pio-sdmmc` (PIO0 SM0, DMA channel 4 completing
//! on `DMA_IRQ_1` on core 0) and a FAT32 volume through `embedded-fatfs`, in one core-0 task, while
//! the flight core runs its IMUs.
//!
//! From 20.5 s: the bus and its INTEn/NVIC checks (Q1-I1, Q1-I2), `init`, with `sd-fat-format` a new
//! MBR (one partition of type 0x0C from block 2048 to the end) and a FAT32 format of it (32 KiB
//! clusters, label MARV), then the MBR check, the mount through `BufStream` + `StreamSlice`, the
//! free-space query, `LOGS/P3TEST.BIN` written (16 MiB in 4 KiB writes, a flush each second, the
//! file truncated first if it exists), read back and compared against the same generator, `LOGS/`
//! listed, and the un-mount. Every call is awaited to its end (Q2-I1); every file is closed, which
//! flushes it, before it is dropped (Q2-I2).

use core::sync::atomic::Ordering;

use atomic_waker::AtomicWaker;
use block_device_adapters::{BufStream, StreamSlice};
use embassy_rp::Peri;
use embassy_rp::interrupt::{self, InterruptExt};
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH4, PIN_28, PIN_29, PIN_30, PIN_31, PIN_32, PIN_33, PIO0};
use embassy_rp::pio::{self, Pio};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_fatfs::{DefaultTimeProvider, FatType, FileSystem, FsOptions, LossyOemCpConverter};
use embedded_io_async::{Read, Write};
use fw_core::watchdog::task;
use fw_flight::tagged;
use pio_sdmmc::pio::{DmaIrqHandler, DmaWaker, PioBus, PioBusConfig};
use pio_sdmmc::{Block, Config, SdCard};

use crate::{CORE1_DMA_IRQ1, check};

/// The SD card's channel (resources.md DMA row 4).
const CH: usize = 4;
const BIT: u32 = 1 << CH;
/// INTE0 as the flight core leaves it: IMU1's channels 0 and 1, IMU2's 2 and 3.
const FLIGHT_CHANNELS: u32 = 0x0f;
/// The probe starts at this uptime (just after the 20 s `[STAT]` lines).
const START_MS: u64 = 20_500;
const PIECE: usize = 4096;
const PIECES: usize = 4096;
const FILE_BYTES: u32 = (PIECE * PIECES) as u32;
const FLUSH_EVERY: Duration = Duration::from_secs(1);
const SEED: u32 = 0x4d41_5256;
const PATH: &str = "LOGS/P3TEST.BIN";
/// The first block of the partition `sd-fat-format` writes.
#[cfg(feature = "sd-fat-format")]
const PART_START: u32 = 2048;
/// At most this many directory entries are listed.
const LIST_MAX: usize = 64;
/// `sd-fat-paced`: one piece every 7 ticks (700 ms) and a flush every 10 (1 s), for 1500 ticks (150 s).
const PACED_PATH: &str = "LOGS/P3PACED.BIN";
const PACED_TICK: Duration = Duration::from_millis(100);
const PACED_TICKS: u32 = 1500;
const PACED_WRITE_TICKS: u32 = 7;
const PACED_FLUSH_TICKS: u32 = 10;
const PACED_PIECES: usize = PACED_TICKS.div_ceil(PACED_WRITE_TICKS) as usize;

type Sd = SdCard<PioBus<'static, PIO0, 0>>;
type Volume = StreamSlice<BufStream<Sd, 512>>;
type Fs<'a> = FileSystem<&'a mut Volume, DefaultTimeProvider, LossyOemCpConverter>;

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

/// A 4 KiB piece, word aligned, so that `BufStream` hands it to the card without a copy.
#[repr(C, align(4))]
struct Piece([u8; PIECE]);

/// Fills `buf` from the 32-bit LCG `x` (Numerical Recipes constants), one little-endian word each.
fn fill(buf: &mut [u8; PIECE], x: &mut u32) {
    for word in buf.as_chunks_mut::<4>().0 {
        *x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *word = x.to_le_bytes();
    }
}

/// Bytes of `buf` that differ from the generator's next piece.
fn mismatches(buf: &[u8; PIECE], x: &mut u32) -> u32 {
    let mut wrong = 0;
    for word in buf.as_chunks::<4>().0 {
        *x = x.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        wrong += word
            .iter()
            .zip(x.to_le_bytes())
            .filter(|&(a, b)| *a != b)
            .count() as u32;
    }
    wrong
}

fn checked(pass: bool, what: &str, value: u32) -> u32 {
    check(pass, what, value);
    u32::from(!pass)
}

/// Q1-I1 and Q1-I2; returns the number of failed checks.
fn register_checks(after: &str) -> u32 {
    let inte = [0, 1, 2, 3].map(|n| pac::DMA.inte(n).read());
    defmt::info!(
        "[INFO] sd-probe checks after {=str}: INTE0 {=u32:#010x} INTE1 {=u32:#010x} INTE2 {=u32:#010x} INTE3 {=u32:#010x}",
        after,
        inte[0],
        inte[1],
        inte[2],
        inte[3]
    );
    let irq0 = interrupt::DMA_IRQ_0.is_enabled();
    let irq1 = interrupt::DMA_IRQ_1.is_enabled();
    let core1 = CORE1_DMA_IRQ1.load(Ordering::Acquire);
    checked(
        inte[0] == FLIGHT_CHANNELS,
        "sd-probe: INTE0 = exactly the flight core's channels 0-3, INTE0",
        inte[0],
    ) + checked(inte[1] == BIT, "sd-probe: INTE1 = 1 << 4, INTE1", inte[1])
        + checked(inte[2] == 0, "sd-probe: INTE2 = 0, INTE2", inte[2])
        + checked(inte[3] == 0, "sd-probe: INTE3 = 0, INTE3", inte[3])
        + checked(
            !irq0,
            "sd-probe: DMA_IRQ_0 not enabled on core 0, enabled",
            u32::from(irq0),
        )
        + checked(
            irq1,
            "sd-probe: DMA_IRQ_1 enabled on core 0, enabled",
            u32::from(irq1),
        )
        + checked(
            core1 == 1,
            "sd-probe: DMA_IRQ_1 not enabled on core 1 (0 not read, 1 off, 2 on)",
            u32::from(core1),
        )
}

/// MBR partition 1: (type, first block, blocks), or why block 0 is not an MBR with a FAT32 one.
fn partition1(b: &[u8]) -> Result<(u8, u32, u32), &'static str> {
    let word = |at: usize| {
        b.get(at..at + 4)
            .and_then(|s| s.try_into().ok())
            .map_or(0, u32::from_le_bytes)
    };
    if b.get(510..512) != Some(&[0x55, 0xaa][..]) {
        return Err("no 0x55AA at byte 510");
    }
    let kind = b.get(0x1be + 4).copied().unwrap_or(0);
    if kind != 0x0b && kind != 0x0c {
        return Err("partition 1 is not of type 0x0B or 0x0C");
    }
    Ok((kind, word(0x1be + 8), word(0x1be + 12)))
}

/// The MBR `sd-fat-format` writes: partition 1 of type 0x0C (FAT32, LBA) from `PART_START` over
/// `blocks` blocks, CHS fields at their "use LBA" value.
#[cfg(feature = "sd-fat-format")]
fn mbr(blocks: u32) -> Block {
    let mut block = Block::new();
    let b = block.as_bytes_mut();
    let mut entry = [0u8; 16];
    entry[1..4].copy_from_slice(&[0xfe, 0xff, 0xff]);
    entry[4] = 0x0c;
    entry[5..8].copy_from_slice(&[0xfe, 0xff, 0xff]);
    entry[8..12].copy_from_slice(&PART_START.to_le_bytes());
    entry[12..16].copy_from_slice(&blocks.to_le_bytes());
    if let Some(s) = b.get_mut(0x1b8..0x1bc) {
        s.copy_from_slice(&SEED.to_le_bytes());
    }
    if let Some(s) = b.get_mut(0x1be..0x1ce) {
        s.copy_from_slice(&entry);
    }
    if let Some(s) = b.get_mut(510..512) {
        s.copy_from_slice(&[0x55, 0xaa]);
    }
    block
}

fn us(t: Instant) -> u64 {
    t.elapsed().as_micros()
}

#[embassy_executor::task]
pub async fn task(parts: Parts) {
    tagged(&crate::watchdog::RUNNING0, task::TEST, run(parts)).await
}

async fn run(parts: Parts) {
    Timer::at(Instant::from_millis(START_MS)).await;
    let began = Instant::now();
    defmt::info!(
        "[INFO] sd-probe: start at {=u64} ms, format {=bool}",
        began.as_millis(),
        cfg!(feature = "sd-fat-format")
    );
    let Pio {
        mut common, sm0, ..
    } = Pio::new(parts.pio, Irqs);
    let bus = PioBus::new(
        &mut common,
        sm0,
        parts.dma,
        Irqs,
        parts.clk,
        parts.cmd,
        parts.dat0,
        parts.dat1,
        parts.dat2,
        parts.dat3,
        PioBusConfig::default(),
    );
    let mut failures = register_checks("the SD bus was built");
    let card = SdCard::new(bus, Config::default());
    failures += probe(card).await;
    failures += register_checks("the probe");
    if failures == 0 {
        defmt::info!(
            "[PASS] sd-probe: every step passed, {=u64} ms in all",
            began.elapsed().as_millis()
        );
    } else {
        defmt::error!(
            "[FAIL] sd-probe: {=u32} failed checks, {=u64} ms in all",
            failures,
            began.elapsed().as_millis()
        );
    }
}

/// `step!(failures, "what", expr)`: the value, or a `[FAIL]` line with the error and `return`.
macro_rules! step {
    ($failures:ident, $what:literal, $e:expr) => {
        match $e {
            Ok(v) => v,
            Err(e) => {
                defmt::error!(
                    "[FAIL] sd-probe: {=str}: {}",
                    $what,
                    defmt::Debug2Format(&e)
                );
                return $failures + 1;
            }
        }
    };
}

/// Everything after the bus; returns the number of failed checks.
async fn probe(mut card: Sd) -> u32 {
    let failures = 0;
    let t = Instant::now();
    let info = step!(failures, "card init", card.init().await);
    defmt::info!("[INFO] sd-probe: card init in {=u64} us: {}", us(t), info);
    let mut block0 = [Block::new(); 1];

    #[cfg(feature = "sd-fat-format")]
    {
        step!(
            failures,
            "read block 0",
            card.read_blocks(0, &mut block0).await
        );
        let old = block0[0].as_bytes();
        defmt::info!(
            "[INFO] sd-probe format: block 0 before: first 16 bytes {=[u8]:02x}, partition entry 1 {=[u8]:02x}, signature {=[u8]:02x}",
            old.get(..16).unwrap_or(&[]),
            old.get(0x1be..0x1ce).unwrap_or(&[]),
            old.get(510..512).unwrap_or(&[])
        );
        let blocks = u32::try_from(info.capacity_blocks)
            .unwrap_or(u32::MAX)
            .saturating_sub(PART_START);
        let t = Instant::now();
        step!(
            failures,
            "write the MBR",
            card.write_blocks(0, &[mbr(blocks)]).await
        );
        let mbr_us = us(t);
        let t = Instant::now();
        let start = u64::from(PART_START) * 512;
        let end = start + u64::from(blocks) * 512;
        let mut volume = step!(
            failures,
            "partition slice for the format",
            StreamSlice::new(BufStream::<_, 512>::new(card), start, end).await
        );
        let options = embedded_fatfs::FormatVolumeOptions::new()
            .fat_type(FatType::Fat32)
            .bytes_per_cluster(32 * 1024)
            .volume_label(*b"MARV       ");
        step!(
            failures,
            "format FAT32",
            embedded_fatfs::format_volume(&mut volume, options).await
        );
        defmt::info!(
            "[PASS] sd-probe format: MBR written in {=u64} us (partition 1 type 0x0C, blocks {=u32} from {=u32}); FAT32 format (32 KiB clusters, label MARV) in {=u64} ms",
            mbr_us,
            blocks,
            PART_START,
            t.elapsed().as_millis()
        );
        card = volume.into_inner().into_inner();
    }

    let t = Instant::now();
    step!(
        failures,
        "read block 0",
        card.read_blocks(0, &mut block0).await
    );
    let (kind, start, blocks) = step!(failures, "MBR", partition1(block0[0].as_bytes()));
    defmt::info!(
        "[PASS] sd-probe: MBR read and checked in {=u64} us: 0x55AA, partition 1 type {=u8:#04x}, first block {=u32}, blocks {=u32} ({=u64} MiB)",
        us(t),
        kind,
        start,
        blocks,
        u64::from(blocks) / 2048
    );
    let (start, end) = (
        u64::from(start) * 512,
        (u64::from(start) + u64::from(blocks)) * 512,
    );
    let mut volume: Volume = step!(
        failures,
        "partition slice",
        StreamSlice::new(BufStream::<_, 512>::new(card), start, end).await
    );
    let mounted = session(&mut volume).await;
    let failures = failures + mounted.unwrap_or(1);
    let t = Instant::now();
    step!(failures, "the stream's flush", volume.flush().await);
    let flush_us = us(t);
    let mut card = volume.into_inner().into_inner();
    defmt::info!(
        "[INFO] sd-probe: the stream's own flush after the session {=u64} us; driver {}",
        flush_us,
        card.stats()
    );
    if mounted.is_none() {
        defmt::error!(
            "[INFO] sd-probe: block 0 starts {=[u8]:02x}",
            block0[0].as_bytes().get(..32).unwrap_or(&[])
        );
        let lba = (start / 512) as u32;
        if card.read_blocks(lba, &mut block0).await.is_ok() {
            defmt::error!(
                "[INFO] sd-probe: the partition's first block ({=u32}) starts {=[u8]:02x}",
                lba,
                block0[0].as_bytes().get(..32).unwrap_or(&[])
            );
        }
    }
    failures
}

/// Mount, free space, the files, un-mount; the number of failed checks, or None if the mount failed.
async fn session(volume: &mut Volume) -> Option<u32> {
    let t = Instant::now();
    let mounted = FileSystem::new(volume, FsOptions::new()).await;
    let mount_us = us(t);
    let fs = match mounted {
        Ok(fs) => fs,
        Err(e) => {
            defmt::error!(
                "[FAIL] sd-probe: mount after {=u64} us: {}",
                mount_us,
                defmt::Debug2Format(&e)
            );
            return None;
        }
    };
    Some(mounted_session(fs, mount_us).await)
}

async fn mounted_session(fs: Fs<'_>, mount_us: u64) -> u32 {
    let failures = 0;
    let flags = fs.read_status_flags().await;
    defmt::info!(
        "[PASS] sd-probe: mounted in {=u64} us: FAT32 {=bool}, cluster {=u32} B, label {=[u8]:a}, dirty at mount {}",
        mount_us,
        fs.fat_type() == FatType::Fat32,
        fs.cluster_size(),
        fs.volume_label_as_bytes(),
        flags.as_ref().map(|f| f.dirty()).ok()
    );
    let failures = failures + u32::from(fs.fat_type() != FatType::Fat32);

    let t = Instant::now();
    let stats = step!(failures, "free space", fs.stats().await);
    let stats_us = us(t);
    let cluster = u64::from(stats.cluster_size());
    defmt::info!(
        "[PASS] sd-probe: free space {=u64} B ({=u64} MiB) of {=u64} B ({=u64} MiB), {=u32} of {=u32} clusters of {=u64} B; the query took {=u64} us",
        u64::from(stats.free_clusters()) * cluster,
        (u64::from(stats.free_clusters()) * cluster) >> 20,
        u64::from(stats.total_clusters()) * cluster,
        (u64::from(stats.total_clusters()) * cluster) >> 20,
        stats.free_clusters(),
        stats.total_clusters(),
        cluster,
        stats_us
    );
    let t = Instant::now();
    let stats = step!(failures, "free space again", fs.stats().await);
    defmt::info!(
        "[INFO] sd-probe: free space asked again: {=u32} clusters in {=u64} us",
        stats.free_clusters(),
        us(t)
    );

    let failures = failures
        + if cfg!(feature = "sd-fat-paced") {
            paced(&fs).await
        } else {
            files(&fs).await
        };

    let t = Instant::now();
    step!(failures, "un-mount", fs.unmount().await);
    defmt::info!("[PASS] sd-probe: un-mounted in {=u64} us", us(t));
    failures
}

/// `LOGS/`, the 16 MiB write, the read-back and the listing; returns the number of failed checks.
async fn files(fs: &Fs<'_>) -> u32 {
    let failures = 0;
    let root = fs.root_dir();
    let t = Instant::now();
    let existed = step!(failures, "LOGS/ exists", root.dir_exists("LOGS").await);
    step!(failures, "create LOGS/", root.create_dir("LOGS").await);
    let t_create = Instant::now();
    let mut file = step!(failures, "create the file", root.create_file(PATH).await);
    if let Err(e) = file.truncate().await {
        defmt::error!(
            "[FAIL] sd-probe: truncate the file: {}",
            defmt::Debug2Format(&e)
        );
        let _ = file.close().await;
        return failures + 1;
    }
    defmt::info!(
        "[INFO] sd-probe: LOGS/ existed {=bool}, opened in {=u64} us; {=str} created or opened and truncated in {=u64} us",
        existed,
        t_create.duration_since(t).as_micros(),
        PATH,
        us(t_create)
    );

    let mut piece = Piece([0; PIECE]);
    let aligned = (piece.0.as_ptr() as usize).is_multiple_of(4);
    let mut durations = [0u32; PIECES];
    let (mut flushes, mut flush_max, mut flush_sum) = (0u32, 0u64, 0u64);
    let mut write_error = false;
    let mut x = SEED;
    let t_write = Instant::now();
    let mut last_flush = t_write;
    for d in durations.iter_mut() {
        fill(&mut piece.0, &mut x);
        let t = Instant::now();
        let r = file.write(&piece.0).await;
        *d = us(t) as u32;
        match r {
            Ok(PIECE) => {}
            other => {
                defmt::error!("[FAIL] sd-probe: write: {}", defmt::Debug2Format(&other));
                write_error = true;
                break;
            }
        }
        if last_flush.elapsed() >= FLUSH_EVERY {
            let t = Instant::now();
            let r = file.flush().await;
            let f = us(t);
            flushes += 1;
            flush_max = flush_max.max(f);
            flush_sum += f;
            last_flush = Instant::now();
            if let Err(e) = r {
                defmt::error!("[FAIL] sd-probe: flush: {}", defmt::Debug2Format(&e));
                write_error = true;
                break;
            }
        }
    }
    let t = Instant::now();
    let closed = file.close().await;
    let close_us = us(t);
    let write_ms = t_write.elapsed().as_millis();
    let failures = failures + u32::from(write_error);
    step!(failures, "close after writing", closed);
    let slowest = durations
        .iter()
        .enumerate()
        .max_by_key(|&(_, &d)| d)
        .map_or(0, |(i, _)| i);
    durations.sort_unstable();
    let sum: u64 = durations.iter().map(|&d| u64::from(d)).sum();
    let at = |q: usize| durations.get(PIECES * q / 100).copied().unwrap_or(0);
    let over = |limit: u32| durations.iter().filter(|&&d| d > limit).count() as u32;
    defmt::info!(
        "[INFO] sd-probe write: {=u32} B in {=usize} calls of {=usize} B in {=u64} ms ({=u64} kB/s with flushes and close); per call: mean {=u64} us, median {=u32} us, 99th percentile {=u32} us, longest {=u32} us, over 5 ms {=u32}, over 30 ms {=u32}; piece word aligned {=bool}",
        FILE_BYTES,
        PIECES,
        PIECE,
        write_ms,
        u64::from(FILE_BYTES) / write_ms.max(1),
        sum / PIECES as u64,
        at(50),
        at(99),
        durations.last().copied().unwrap_or(0),
        over(5_000),
        over(30_000),
        aligned
    );
    defmt::info!(
        "[INFO] sd-probe write: {=u32} flushes (one a second), mean {=u64} us, longest {=u64} us; close {=u64} us; the longest write was call {=usize}",
        flushes,
        flush_sum / u64::from(flushes.max(1)),
        flush_max,
        close_us,
        slowest
    );

    let t = Instant::now();
    let mut file = step!(failures, "re-open the file", root.open_file(PATH).await);
    let open_us = us(t);
    let mut x = SEED;
    let (mut compared, mut wrong) = (0u32, 0u32);
    let mut read_error = false;
    let t_read = Instant::now();
    for _ in 0..PIECES {
        match file.read(&mut piece.0).await {
            Ok(PIECE) => {
                compared += PIECE as u32;
                wrong += mismatches(&piece.0, &mut x);
            }
            other => {
                defmt::error!(
                    "[FAIL] sd-probe: read after {=u32} B: {}",
                    compared,
                    defmt::Debug2Format(&other)
                );
                read_error = true;
                break;
            }
        }
    }
    let read_ms = t_read.elapsed().as_millis();
    let tail = file.read(&mut piece.0).await;
    step!(failures, "close after reading", file.close().await);
    let at_end = matches!(tail, Ok(0));
    let pass = !read_error && compared == FILE_BYTES && wrong == 0 && at_end;
    let failures = failures + u32::from(!pass);
    let line = if pass { "[PASS]" } else { "[FAIL]" };
    defmt::info!(
        "{=str} sd-probe read-back: re-opened in {=u64} us; {=u32} B compared, {=u32} B wrong, end of file after them {=bool}; {=u64} ms ({=u64} kB/s)",
        line,
        open_us,
        compared,
        wrong,
        at_end,
        read_ms,
        u64::from(compared) / read_ms.max(1)
    );

    let logs = step!(failures, "open LOGS/", root.open_dir("LOGS").await);
    let mut entries = logs.iter();
    let mut listed = false;
    for _ in 0..LIST_MAX {
        let Some(entry) = entries.next().await else {
            break;
        };
        let entry = step!(failures, "list LOGS/", entry);
        let name = entry.short_file_name_as_bytes();
        listed |= name == b"P3TEST.BIN" && entry.len() == u64::from(FILE_BYTES);
        defmt::info!(
            "[INFO] sd-probe: LOGS/ {=[u8]:a} {=u64} B dir {=bool}",
            name,
            entry.len(),
            entry.is_dir()
        );
    }
    let failures = failures + u32::from(!listed);
    let line = if listed { "[PASS]" } else { "[FAIL]" };
    defmt::info!(
        "{=str} sd-probe: LOGS/ lists P3TEST.BIN with {=u32} B",
        line,
        FILE_BYTES
    );
    failures
}

/// `sd-fat-paced`: the logger's duty instead of the burst. `LOGS/P3PACED.BIN` created (truncated if it
/// exists), then for 150 s one 4 KiB piece every 700 ms and a flush every second, nothing else; closed.
async fn paced(fs: &Fs<'_>) -> u32 {
    let failures = 0;
    let root = fs.root_dir();
    step!(failures, "create LOGS/", root.create_dir("LOGS").await);
    let t = Instant::now();
    let mut file = step!(
        failures,
        "create the file",
        root.create_file(PACED_PATH).await
    );
    if let Err(e) = file.truncate().await {
        defmt::error!(
            "[FAIL] sd-probe paced: truncate the file: {}",
            defmt::Debug2Format(&e)
        );
        let _ = file.close().await;
        return failures + 1;
    }
    defmt::info!(
        "[INFO] sd-probe paced: {=str} created or opened and truncated in {=u64} us; from {=u64} ms, {=u32} s of one 4 KiB write every {=u64} ms and a flush every {=u64} ms",
        PACED_PATH,
        us(t),
        Instant::now().as_millis(),
        PACED_TICKS / 10,
        PACED_TICK.as_millis() * u64::from(PACED_WRITE_TICKS),
        PACED_TICK.as_millis() * u64::from(PACED_FLUSH_TICKS)
    );
    let mut piece = Piece([0; PIECE]);
    let mut durations = [0u32; PACED_PIECES];
    let mut written = 0usize;
    let (mut flushes, mut flush_max, mut flush_sum) = (0u32, 0u64, 0u64);
    let mut error = false;
    let mut x = SEED;
    let mut ticker = Ticker::every(PACED_TICK);
    for tick in 0..PACED_TICKS {
        ticker.next().await;
        if tick.is_multiple_of(PACED_WRITE_TICKS) {
            let Some(d) = durations.get_mut(written) else {
                break;
            };
            fill(&mut piece.0, &mut x);
            let t = Instant::now();
            let r = file.write(&piece.0).await;
            *d = us(t) as u32;
            written += 1;
            if !matches!(r, Ok(PIECE)) {
                defmt::error!("[FAIL] sd-probe paced: write: {}", defmt::Debug2Format(&r));
                error = true;
                break;
            }
        }
        if tick % PACED_FLUSH_TICKS == PACED_FLUSH_TICKS - 1 {
            let t = Instant::now();
            let r = file.flush().await;
            let f = us(t);
            flushes += 1;
            flush_max = flush_max.max(f);
            flush_sum += f;
            if let Err(e) = r {
                defmt::error!("[FAIL] sd-probe paced: flush: {}", defmt::Debug2Format(&e));
                error = true;
                break;
            }
        }
    }
    let t = Instant::now();
    let closed = file.close().await;
    let close_us = us(t);
    let failures = failures + u32::from(error);
    step!(failures, "close the paced file", closed);
    let calls = durations.get_mut(..written).unwrap_or(&mut []);
    let slowest = calls
        .iter()
        .enumerate()
        .max_by_key(|&(_, &d)| d)
        .map_or(0, |(i, _)| i);
    calls.sort_unstable();
    let n = calls.len().max(1);
    let sum: u64 = calls.iter().map(|&d| u64::from(d)).sum();
    let at = |q: usize| calls.get(calls.len() * q / 100).copied().unwrap_or(0);
    let over = |limit: u32| calls.iter().filter(|&&d| d > limit).count() as u32;
    defmt::info!(
        "[INFO] sd-probe paced: {=usize} writes of {=usize} B, ended at {=u64} ms; per call: mean {=u64} us, median {=u32} us, 99th percentile {=u32} us, longest {=u32} us (call {=usize}), over 5 ms {=u32}, over 30 ms {=u32}; {=u32} flushes, mean {=u64} us, longest {=u64} us; close {=u64} us",
        written,
        PIECE,
        Instant::now().as_millis(),
        sum / n as u64,
        at(50),
        at(99),
        calls.last().copied().unwrap_or(0),
        slowest,
        over(5_000),
        over(30_000),
        flushes,
        flush_sum / u64::from(flushes.max(1)),
        flush_max,
        close_us
    );
    failures
}
