//! The 16 MiB PSRAM in socket U24 (decision-psram-offload-20261121 in MARV-V2-drivers, D2/D3, work units
//! W2, W3 and W4). It starts the chip at boot, keeps a 64-byte canary in the memory, re-maps the window after
//! every flash erase or program, and holds the storage of the two log rings, `logger::RING` (1 MiB) and
//! `logdl::HANDOVER` (4 KiB), which [`init`] binds once the canary has passed. Without a working chip the rings
//! stay unbound: the collector drains the flight ring into nothing, the SD task is off ("no PSRAM") and the log
//! download is refused; arming is not blocked, and there is no SRAM fallback ring.
//!
//! The memory is `qmi-psram`'s uncached alias at 0x15000000 (`memory.x`, region PSRAM, section `.psram`).
//! Rules this module keeps (invariants P1 to P6 of the decision):
//! - Core 1 never touches it (P1): nothing here is reachable from core 1; `tools/check-ram-code.py` fails
//!   otherwise (negative control: the feature `psram-negative`).
//! - Only core 0 uses it, in thread mode (P2): the collector, the SD task and the USB link task, all on core 0's
//!   thread-mode executor (`main`'s). The one exception is [`after_flash_op`], which runs wherever a settings
//!   save runs: in thread mode, or on `SWI_IRQ_0` when the save came over the radio.
//! - Nothing in it is atomic and nothing has an initial value (P3): [`PsramBuf`] is `MaybeUninit` bytes, and
//!   `psram_static!` below is the one place a static is put in `.psram`.
//! - No access before `init` and its canary, and none after a flash operation before the remap and the
//!   canary (P4): [`ready`] says whether the memory may be used. The rings' handles exist only after the bind
//!   in [`init`]; at run time each producer and consumer checks [`ready`] before a batch of PSRAM accesses
//!   (the collector once per tick, the SD task once per loop turn, the USB link per sample), so a FAULT after a
//!   flash operation stops them.
//! - No DMA to or from it (P5), uncached alias only (P6).
//! A missing or failing chip never panics: the state reads FAULT, the log says why, and the board goes on.

use core::cell::Cell;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use cortex_m::peripheral::SCB;
use cortex_m::peripheral::scb::VectActive;
use embassy_rp::Peri;
use embassy_rp::pac::Interrupt;
use embassy_rp::peripherals::{PIN_0, QMI_CS1};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Instant};
use fw_core::extring::{PsramBuf, PsramReady};
use qmi_psram::Psram;
use qmi_psram::rp2350::{Cs1Pin, Rp2350Qmi};

/// Bytes of the canary.
const CANARY_LEN: usize = 64;
/// Words of the canary.
const CANARY_WORDS: usize = CANARY_LEN / 4;

/// Not started (before [`init`]), or the chip was never found: nothing is mapped.
const OFF: u8 = 0;
/// Started, canary verified, no flash operation since without a verified remap.
const READY: u8 = 1;
/// A fault, sticky until the next boot: the memory must not be used. See `REASON`.
const FAULT: u8 = 2;

const R_NONE: u8 = 0;
const R_INIT: u8 = 1;
const R_CANARY: u8 = 2;
const R_REMAP: u8 = 3;
const R_AFTER_FLASH: u8 = 4;
const R_BIND: u8 = 5;

static STATE: AtomicU8 = AtomicU8::new(OFF);
static REASON: AtomicU8 = AtomicU8::new(R_NONE);
/// Canary checks made (boot, periodic, after a flash operation) and those that failed.
static CHECKS: AtomicU32 = AtomicU32::new(0);
static CHECK_FAILS: AtomicU32 = AtomicU32::new(0);
/// Remaps made after a flash operation.
static REMAPS: AtomicU32 = AtomicU32::new(0);
/// Entry points called outside their context (P2): not on core 0, or in an interrupt where only thread mode
/// is allowed. A release counter, shown in the `[STAT]` line; a debug build also panics.
static VIOLATIONS: AtomicU32 = AtomicU32::new(0);

/// The driver, kept for `remap`. Taken out and put back (as `settings::with_storage` does) so that no lock is
/// held while the window is re-programmed.
static DRIVER: Mutex<CriticalSectionRawMutex, Cell<Option<Psram<Rp2350Qmi>>>> =
    Mutex::new(Cell::new(None));

/// The one place a static is put in the `.psram` section (invariant P3: the attribute appears once): a [`PsramBuf`] of
/// `$n` bytes, `static mut` so that [`init`] can hand the ring its one `&'static mut`. Nothing else may name it.
macro_rules! psram_static {
    ($name:ident, $n:expr) => {
        #[unsafe(link_section = ".psram")]
        static mut $name: PsramBuf<{ $n }> = PsramBuf::new();
    };
}

psram_static!(CANARY, CANARY_LEN);
// `logger::RING`'s storage (the collector writes it, the SD task reads it).
psram_static!(LOG_BUF, crate::logger::RING_BYTES);
// `logdl::HANDOVER`'s storage (the SD task writes it, the USB task reads it).
psram_static!(HANDOVER_BUF, crate::logdl::HANDOVER_BYTES);

/// The canary's first byte.
fn canary_base() -> *mut u8 {
    // The newtype is `repr(C)` around one `UnsafeCell` of bytes, so a pointer to it is a pointer to its bytes.
    (&raw mut CANARY).cast()
}

/// Word `k` of the canary pattern: xorshift32 from a fixed seed, so that no two neighbouring words, and no
/// byte lane, repeat a simple pattern (a stuck data line or a wrong window shows).
fn pattern(k: usize) -> u32 {
    let mut x: u32 = 0xC0DE_5A5A;
    for _ in 0..=k {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
    }
    x
}

/// Writes the canary (the memory must be mapped and idle).
fn canary_write() {
    let base = canary_base().cast::<u32>();
    for k in 0..CANARY_WORDS {
        // SAFETY: `base` is 4-aligned (`PsramBuf`), `k` is inside the 64 bytes; the caller has just started the
        // memory and nothing else uses the canary.
        unsafe { base.add(k).write_volatile(pattern(k)) };
    }
}

/// Words of the canary that do not read back as written; 0 means the memory is mapped and intact.
fn canary_mismatches() -> u32 {
    let base = canary_base().cast::<u32>();
    let mut bad = 0;
    for k in 0..CANARY_WORDS {
        // SAFETY: as in `canary_write`; read-only.
        if unsafe { base.add(k).read_volatile() } != pattern(k) {
            bad += 1;
        }
    }
    bad
}

/// The access-context check (invariant P2): core 0, and thread mode, or `SWI_IRQ_0` when `allow_swi` (the
/// flash hook, because a radio save runs there). A violation is counted (release) and asserted (debug); the
/// call goes on, because skipping a remap would be worse.
fn context_ok(allow_swi: bool) -> bool {
    let core0 = embassy_rp::pac::SIO.cpuid().read() == 0;
    let context = match SCB::vect_active() {
        VectActive::ThreadMode => true,
        VectActive::Interrupt { irqn } => {
            allow_swi && u16::from(irqn) == Interrupt::SWI_IRQ_0 as u16
        }
        _ => false,
    };
    let ok = core0 && context;
    if !ok {
        VIOLATIONS.fetch_add(1, Ordering::Relaxed);
    }
    debug_assert!(
        ok,
        "psram: entry point called outside core 0 / its allowed context"
    );
    ok
}

fn fault(reason: u8) {
    REASON.store(reason, Ordering::Relaxed);
    STATE.store(FAULT, Ordering::Release);
}

/// True when the memory has been started, its canary verified, and no flash operation has left it unmapped.
/// The ring users check it before each batch of PSRAM accesses: it turns false for good at a FAULT.
pub fn ready() -> bool {
    STATE.load(Ordering::Acquire) == READY
}

/// One word for the `[STAT]` line.
pub fn status_text() -> &'static str {
    match (
        STATE.load(Ordering::Relaxed),
        REASON.load(Ordering::Relaxed),
    ) {
        (READY, _) => "ok",
        (FAULT, R_INIT) => "FAULT init",
        (FAULT, R_CANARY) => "FAULT canary",
        (FAULT, R_REMAP) => "FAULT remap",
        (FAULT, R_AFTER_FLASH) => "FAULT canary after flash op",
        (FAULT, R_BIND) => "FAULT bind",
        (FAULT, _) => "FAULT",
        _ => "off",
    }
}

pub fn checks() -> u32 {
    CHECKS.load(Ordering::Relaxed)
}

pub fn check_fails() -> u32 {
    CHECK_FAILS.load(Ordering::Relaxed)
}

pub fn remaps() -> u32 {
    REMAPS.load(Ordering::Relaxed)
}

pub fn violations() -> u32 {
    VIOLATIONS.load(Ordering::Relaxed)
}

/// Starts the PSRAM and checks the canary. At boot, on core 0, after the boot-time flash writes (the
/// settings are read before) and before `spawn_core1` (the driver's contract: nothing else runs from
/// flash while it talks to the chip). Takes the chip-select peripheral and its pin so that nothing else can.
/// A missing or failing chip leaves the state at FAULT with a `[FAIL]` line; it never panics.
pub fn init(qmi_cs1: Peri<'static, QMI_CS1>, pin0: Peri<'static, PIN_0>) {
    // Consumed here for good: QMI CS1 and GPIO0 belong to the driver from now on.
    let _ = (qmi_cs1, pin0);
    context_ok(false);
    let started = Instant::now();
    if cfg!(feature = "psram-absent") {
        fault(R_INIT);
        defmt::error!(
            "[FAIL] psram: init treated as failed (feature psram-absent): the chip is not touched, the board goes on without it"
        );
        return;
    }
    // SAFETY: QMI CS1 and GPIO0 are owned (the two peripherals were consumed above); core 1 has not been
    // started, so nothing else executes from flash or touches the XIP window while the driver works, and no
    // DMA channel reads either.
    let qmi = unsafe { Rp2350Qmi::new(Cs1Pin::Gpio0) };
    let mut psram = Psram::new(qmi);
    let clk_sys_hz = embassy_rp::clocks::clk_sys_freq();
    let info = match psram.init(&mut Delay, clk_sys_hz) {
        Ok(info) => info,
        Err(e) => {
            fault(R_INIT);
            defmt::error!(
                "[FAIL] psram: init failed ({}): no PSRAM, the board goes on without it, logging to it is off",
                e
            );
            return;
        }
    };
    let init_us = started.elapsed().as_micros() as u32;
    let canary_started = Instant::now();
    canary_write();
    let bad = canary_mismatches();
    CHECKS.fetch_add(1, Ordering::Relaxed);
    let canary_us = canary_started.elapsed().as_micros() as u32;
    if bad != 0 {
        CHECK_FAILS.fetch_add(1, Ordering::Relaxed);
        fault(R_CANARY);
        defmt::error!(
            "[FAIL] psram: ID {=u8:#04x} {=u8:#04x} {=u8:#04x}, {=u32} KiB, but the {=usize}-byte canary read back wrong in {=u32} of {=usize} words: not used",
            info.id.manufacturer,
            info.id.kgd,
            info.id.eid_high,
            (info.size / 1024) as u32,
            CANARY_LEN,
            bad,
            CANARY_WORDS
        );
        return;
    }
    let (clkdiv, rx_delay) = psram.timing().map_or((0, 0), |t| (t.clkdiv, t.rx_delay));
    DRIVER.lock(|c| c.set(Some(psram)));
    // The memory is started and the canary verified, on core 0 in thread mode, before any task and before core 1:
    // the rings may be bound, which zeroes their storage (about 3 ms for the 68 KiB).
    let bind_started = Instant::now();
    if !bind_rings() {
        fault(R_BIND);
        defmt::error!(
            "[FAIL] psram: a log ring was bound twice: the PSRAM is not used, logging to it is off"
        );
        return;
    }
    let bind_us = bind_started.elapsed().as_micros() as u32;
    REASON.store(R_NONE, Ordering::Relaxed);
    STATE.store(READY, Ordering::Release);
    defmt::info!(
        "[PASS] psram: ID {=u8:#04x} {=u8:#04x} {=u8:#04x}, {=u32} KiB at 0x15000000 (uncached), clk_sys {=u32} Hz, SCK divisor {=u16}, RXDELAY {=u8}, init {=u32} us, canary {=usize} B written and verified in {=u32} us, log rings bound and zeroed in {=u32} us",
        info.id.manufacturer,
        info.id.kgd,
        info.id.eid_high,
        (info.size / 1024) as u32,
        clk_sys_hz,
        clkdiv,
        rx_delay,
        init_us,
        CANARY_LEN,
        canary_us,
        bind_us
    );
}

/// Binds the log rings to their storage, once, from [`init`] only. `false` if a ring was already bound.
fn bind_rings() -> bool {
    // SAFETY: called after the chip started and the canary read back right, on core 0 (the only caller, `init`).
    let ready = unsafe { PsramReady::new() };
    // SAFETY: each `&'static mut` below is made exactly once, here: `init` runs once (it consumes `QMI_CS1`, a
    // singleton), nothing else names these statics, and no ring is bound yet, so no other reference exists.
    let log = crate::logger::RING.bind(unsafe { &mut *(&raw mut LOG_BUF) }, &ready);
    // SAFETY: as above.
    let handover = crate::logdl::HANDOVER.bind(unsafe { &mut *(&raw mut HANDOVER_BUF) }, &ready);
    log.is_ok() && handover.is_ok()
}

/// Verifies the canary; a failure makes the state FAULT (sticky) with the reason `reason`. Only while READY.
fn check(reason: u8) {
    if STATE.load(Ordering::Acquire) != READY {
        return;
    }
    CHECKS.fetch_add(1, Ordering::Relaxed);
    let bad = canary_mismatches();
    if bad != 0 {
        CHECK_FAILS.fetch_add(1, Ordering::Relaxed);
        // One log line: only the call that turns READY into FAULT prints it.
        if STATE.swap(FAULT, Ordering::AcqRel) == READY {
            REASON.store(reason, Ordering::Relaxed);
            defmt::error!(
                "[FAIL] psram: canary read back wrong in {=u32} of {=usize} words ({=str}): the PSRAM is not used from now on",
                bad,
                CANARY_WORDS,
                if reason == R_AFTER_FLASH {
                    "after a flash operation"
                } else {
                    "periodic check"
                }
            );
        }
    }
}

/// The periodic canary check: every 10 s from the supervisor's `[STAT]` tick (core 0, thread mode).
pub fn periodic() {
    context_ok(false);
    check(R_CANARY);
}

/// The hook after a flash erase or program (`settings::guarded`, on success or failure, synchronously):
/// programming the in-package flash through the boot ROM resets the QMI window registers, so the window is
/// re-programmed (`remap`, contents kept) and the canary read back. A failure makes the state FAULT (P4).
///
/// Before [`init`] (the boot-time flash writes) nothing is mapped and it does nothing; `init` runs after them.
///
/// Why no access can fall between the flash operation and this remap (P2): this runs synchronously, right
/// after `blocking_erase`/`blocking_write` return, on the context that called them (a thread-mode task, or
/// `SWI_IRQ_0`, where the radio's save runs: `rc.rs`). Another thread-mode task cannot run before it returns, a
/// preempting interrupt cannot return into one, and the interrupt handlers that can run in between (the
/// hardware interrupts at P0, `SWI_IRQ_1` at P1, and `SWI_IRQ_0` itself when this runs in thread mode) do not
/// touch the PSRAM: `psram::` is called only from `main` (boot, the supervisor task) and from `settings`.
/// Another save cannot interleave either: `with_storage` hands the flash out once.
pub fn after_flash_op() {
    if STATE.load(Ordering::Acquire) != READY {
        return;
    }
    context_ok(true);
    if !cfg!(feature = "psram-skip-remap") {
        let remapped = match DRIVER.lock(Cell::take) {
            Some(mut psram) => {
                let r = psram.remap();
                DRIVER.lock(|c| c.set(Some(psram)));
                r.map_err(|e| {
                    defmt::error!("[FAIL] psram: remap after a flash operation failed ({})", e)
                })
                .is_ok()
            }
            None => false,
        };
        REMAPS.fetch_add(1, Ordering::Relaxed);
        if !remapped {
            fault(R_REMAP);
            return;
        }
    }
    check(R_AFTER_FLASH);
}

/// The feature `psram-negative`: a read of the canary from code that core 1 reaches (`now_us`), which
/// `tools/check-ram-code.py` must refuse (a function and a data object in the XIP window). Never flashed.
#[cfg(feature = "psram-negative")]
#[inline(never)]
pub fn negative_touch() -> u8 {
    // SAFETY: a volatile byte read of the canary's first byte; this feature builds a binary that is only
    // checked, never run.
    unsafe { canary_base().read_volatile() }
}
