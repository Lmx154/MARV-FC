//! The hardware watchdog (design: decision-foundation, two-core amendment, item 6).
//!
//! [`boot`] reads the reset reason and the scratch record once, clears the record's magic and
//! queues the boot report; `main` then starts the watchdog (1600 ms, paused while a core is halted
//! by the debugger) and [`start_monitor`] puts the monitor alone on an `InterruptExecutor` on
//! `SWI_IRQ_1`, above core 0's thread-mode executor. Every 100 ms the monitor applies
//! `fw_core::watchdog::Monitor` to `BEAT0`, `PONG` and `SAVE_EPOCH`: it feeds only while both
//! heartbeats are younger than 200 ms; at 200 ms of stall it writes the scratch record once (the
//! core and the task in `RUNNING0` / `RUNNING1`) and stops feeding, so the board resets about
//! 1.7 s after the stall began; a stall that ends first clears the record and is logged by the
//! supervisor ([`report`]), with one text when it reached 500 ms; while a reboot is pending the
//! monitor neither writes nor clears the record. The monitor uses atomics and the watchdog
//! registers only. A requested reboot ([`request_reboot`]) is carried out by the monitor
//! (`trigger_reset`) 100 ms after the final ACK was handed to the link. No device task feeds it.
//! Known and accepted: a reboot request that races a stall's start within a few microseconds can
//! leave a scratch record that decodes as none, so the boot report then says "reset: forced, no
//! record".
//!
//! Features `hang-core0`, `spin-1s`, `hang-irqoff` ([`hang_point_core0`], in the barometer's
//! `acquire`) and `hang-flight` ([`hang_point_flight`], in IMU1's `acquire`) act once, 20 s after
//! a start that the watchdog did not cause.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};

use embassy_executor::InterruptExecutor;
use embassy_rp::interrupt::{self, InterruptExt, Priority};
use embassy_rp::watchdog::{ResetReason, Watchdog};
use embassy_time::{Duration, Instant, Ticker};
use fw_core::scratch::{self, Reason};
use fw_core::watchdog::{
    self as wdg, CHECK_MS, Core, Ended, Inputs, Monitor, PERIOD_MS, ResetKind, Severity, StallText,
    task,
};
use mavlink_device::dialect::MavSeverity;

use crate::device::TextLine;

/// The id of core 0's thread-mode task that ran last (`fw_core::watchdog::task`).
pub static RUNNING0: AtomicU8 = AtomicU8::new(task::NONE);
/// The id of the flight core's task that ran last, written by `fw_flight::tagged` on core 1.
pub static RUNNING1: AtomicU8 = AtomicU8::new(task::NONE);
/// A reboot was requested; the monitor resets at `REBOOT_AT_MS` (its clock, ms).
static REBOOT_REQ: AtomicBool = AtomicBool::new(false);
static REBOOT_AT_MS: AtomicU32 = AtomicU32::new(0);
/// Feeds since start.
pub static FEEDS: AtomicU32 = AtomicU32::new(0);
/// The check features may act: the last reset was not the watchdog's. Cleared when one acts.
pub static HANG_ARMED: AtomicBool = AtomicBool::new(false);
/// Stalls the monitor saw begin and end, and the last one of each: core | task << 8 | flag << 16
/// | age in 10 ms << 24 (flag: recorded for a begin, text for an end).
static BEGUN: AtomicU32 = AtomicU32::new(0);
static BEGUN_INFO: AtomicU32 = AtomicU32::new(0);
static ENDED: AtomicU32 = AtomicU32::new(0);
static ENDED_INFO: AtomicU32 = AtomicU32::new(0);
/// Checks that saw `SAVE_EPOCH` changed (a flash save since the check before).
static SAVE_CHECKS: AtomicU32 = AtomicU32::new(0);

static EXECUTOR_WDG: InterruptExecutor = InterruptExecutor::new();

#[cortex_m_rt::interrupt]
unsafe fn SWI_IRQ_1() {
    // SAFETY: the one handler of the interrupt `EXECUTOR_WDG` was started on.
    unsafe { EXECUTOR_WDG.on_interrupt() }
}

fn now_ms() -> u32 {
    Instant::now().as_millis() as u32
}

fn pack(core: Core, task: u8, flag: bool, age_ms: u32) -> u32 {
    u32::from(core == Core::Flight)
        | u32::from(task) << 8
        | u32::from(flag) << 16
        | (age_ms / 10).min(255) << 24
}

fn unpack(info: u32) -> (Core, u8, bool, u32) {
    let core = if info & 1 != 0 {
        Core::Flight
    } else {
        Core::Core0
    };
    (
        core,
        (info >> 8) as u8,
        info >> 16 & 1 != 0,
        (info >> 24) * 10,
    )
}

fn write_record(words: [u32; 4]) {
    let w = embassy_rp::pac::WATCHDOG;
    w.scratch0().write(|r| *r = words[0]);
    w.scratch1().write(|r| *r = words[1]);
    w.scratch2().write(|r| *r = words[2]);
    w.scratch3().write(|r| *r = words[3]);
}

/// The four scratch words S0..S3.
pub fn read_scratch(wd: &mut Watchdog) -> [u32; 4] {
    [0, 1, 2, 3].map(|i| wd.get_scratch(i))
}

pub fn reason_name(reason: Option<ResetReason>) -> &'static str {
    match reason {
        Some(ResetReason::TimedOut) => "TimedOut",
        Some(ResetReason::Forced) => "Forced",
        None => "none",
    }
}

fn severity(s: Severity) -> MavSeverity {
    match s {
        Severity::Critical => MavSeverity::MAV_SEVERITY_CRITICAL,
        Severity::Warning => MavSeverity::MAV_SEVERITY_WARNING,
        Severity::Info => MavSeverity::MAV_SEVERITY_INFO,
    }
}

/// The reset cause for the boot text line ("power-on" also for a debugger or RUN-pin reset).
static CAUSE: AtomicU8 = AtomicU8::new(0);

/// "timeout", "forced" or "power-on".
pub fn cause() -> &'static str {
    match CAUSE.load(Ordering::Relaxed) {
        1 => "timeout",
        2 => "forced",
        _ => "power-on",
    }
}

/// At boot, before the watchdog starts: the reset reason and the scratch record, read once; the
/// record's magic is cleared; the boot report as a text (held until a station's node runs) and
/// one log line. Arms the check features when the watchdog did not cause the reset.
pub fn boot(wd: &mut Watchdog) {
    let reason = wd.reset_reason();
    let words = read_scratch(wd);
    let record = scratch::decode(words);
    wd.set_scratch(0, 0);
    HANG_ARMED.store(reason.is_none(), Ordering::Relaxed);
    let kind = reason.map(|r| match r {
        ResetReason::TimedOut => ResetKind::TimedOut,
        ResetReason::Forced => ResetKind::Forced,
    });
    CAUSE.store(
        match kind {
            Some(ResetKind::TimedOut) => 1,
            Some(ResetKind::Forced) => 2,
            None => 0,
        },
        Ordering::Relaxed,
    );
    let report = wdg::boot_report(kind, record);
    let line = report.map_or(TextLine::new(format_args!("nothing")), |r| {
        TextLine::new(format_args!("{}", r))
    });
    let (code, task_id, age_ms, uptime_ms) = record.map_or((0, 0, 0, 0), |r| {
        (r.reason as u8 + 1, r.task_id, r.age_ms(), r.uptime_ms)
    });
    defmt::info!(
        "[INFO] boot report: reset reason {=str}, record {=bool} (reason {=u8}, task {=str}, age {=u32} ms, uptime {=u32} ms): {=str}",
        reason_name(reason),
        record.is_some(),
        code,
        wdg::task_name(task_id),
        age_ms,
        uptime_ms,
        line.as_str()
    );
    if let Some(r) = report {
        crate::text_sticky(severity(r.severity()), line.as_str());
    }
}

fn inputs() -> Inputs {
    Inputs {
        now_ms: now_ms(),
        beat0: crate::BEAT0.load(Ordering::Relaxed),
        pong: crate::PONG.load(Ordering::Relaxed),
        save_epoch: crate::settings::SAVE_EPOCH.load(Ordering::Relaxed),
        reboot_at_ms: REBOOT_REQ
            .load(Ordering::Acquire)
            .then(|| REBOOT_AT_MS.load(Ordering::Relaxed)),
    }
}

#[embassy_executor::task]
async fn monitor(mut wd: Watchdog) -> ! {
    let mut m = Monitor::new(inputs());
    let mut ticker = Ticker::every(Duration::from_millis(u64::from(CHECK_MS)));
    let mut stall_task = task::NONE;
    loop {
        ticker.next().await;
        let i = inputs();
        let step = m.step(i);
        if step.save_check {
            SAVE_CHECKS.fetch_add(1, Ordering::Relaxed);
        }
        if step.reset {
            wd.trigger_reset();
        }
        if let Some((core, age_ms)) = step.began {
            stall_task = match core {
                Core::Core0 => RUNNING0.load(Ordering::Relaxed),
                Core::Flight => RUNNING1.load(Ordering::Relaxed),
            };
            if step.record {
                write_record(scratch::encode(core.reason(), stall_task, age_ms, i.now_ms));
            }
            BEGUN_INFO.store(
                pack(core, stall_task, step.record, age_ms),
                Ordering::Relaxed,
            );
            BEGUN.fetch_add(1, Ordering::Release);
        }
        if step.clear {
            wd.set_scratch(0, 0);
        }
        if let Some(e) = step.ended {
            ENDED_INFO.store(
                pack(e.core, stall_task, e.text, e.age_ms),
                Ordering::Relaxed,
            );
            ENDED.fetch_add(1, Ordering::Release);
        }
        if step.feed {
            wd.feed(Duration::from_millis(u64::from(PERIOD_MS)));
            FEEDS.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Starts the monitor on `SWI_IRQ_1` (priority 1, above core 0's thread mode). Call on core 0
/// after the watchdog was started.
pub fn start_monitor(wd: Watchdog) {
    interrupt::SWI_IRQ_1.set_priority(Priority::P1);
    let spawner = EXECUTOR_WDG.start(interrupt::SWI_IRQ_1);
    spawner.spawn(defmt::unwrap!(monitor(wd)));
}

/// The supervisor, every tick: logs each stall the monitor saw begin or end (after the fact: a
/// core-0 stall is logged only if core 0 resumed), and one text for a stall that reached 500 ms.
pub fn report(begun_seen: &mut u32, ended_seen: &mut u32) {
    let begun = BEGUN.load(Ordering::Acquire);
    if begun != *begun_seen {
        *begun_seen = begun;
        let (core, task_id, recorded, age_ms) = unpack(BEGUN_INFO.load(Ordering::Relaxed));
        defmt::warn!(
            "[INFO] watchdog: {=str} stall noticed at {=u32} ms of age ({=str} ran last), record {=bool}, feeding stopped",
            if core == Core::Core0 {
                "core 0"
            } else {
                "flight core"
            },
            age_ms,
            wdg::task_name(task_id),
            recorded
        );
    }
    let ended = ENDED.load(Ordering::Acquire);
    if ended != *ended_seen {
        *ended_seen = ended;
        let (core, task_id, text, age_ms) = unpack(ENDED_INFO.load(Ordering::Relaxed));
        let t = StallText {
            ended: Ended { core, age_ms, text },
            task_id,
        };
        let line = TextLine::new(format_args!("{}", t));
        defmt::warn!(
            "[INFO] watchdog: stall over, no reset: {=str} (at least {=u32} ms), record cleared, feeding again",
            line.as_str(),
            age_ms
        );
        if text {
            crate::text(MavSeverity::MAV_SEVERITY_WARNING, line.as_str());
        }
    }
}

/// The supervisor, every 10 s: the monitor's counters.
pub fn stat() {
    if cfg!(feature = "quiet-stat") {
        return;
    }
    if !CORE0_REPAINTED.swap(true, Ordering::Relaxed) {
        CORE0_BOOT_USED.store(stack_use(core0_stack()).0, Ordering::Relaxed);
        repaint_core0();
    }
    let boot0 = CORE0_BOOT_USED.load(Ordering::Relaxed);
    let (used0, free0) = stack_use(core0_stack());
    let (used1, free1) = stack_use((
        CORE1_STACK_LO.load(Ordering::Relaxed),
        CORE1_STACK_HI.load(Ordering::Relaxed),
    ));
    defmt::info!(
        "[STAT] watchdog: feeds {=u32}, stalls begun {=u32}, ended {=u32}, checks after a save {=u32}; stack high-water: core 0 boot peak {=u32} B, since start-up used {=u32} B free {=u32} B, core 1 used {=u32} B free {=u32} B",
        FEEDS.load(Ordering::Relaxed),
        BEGUN.load(Ordering::Relaxed),
        ENDED.load(Ordering::Relaxed),
        SAVE_CHECKS.load(Ordering::Relaxed),
        boot0,
        used0,
        free0,
        used1,
        free1
    );
}

/// The word both stacks are painted with at start-up (stack S1): a word of a stack that still holds
/// it was, in all likelihood, never written since. The `[STAT] watchdog:` line counts the painted
/// words from each stack's low end: free = those bytes, used = the rest of the stack.
const STACK_PAINT: u32 = 0x5354_4B21;
/// Core 1's stack, `[lo, hi)`, recorded by [`paint_core1`]; 0 until then.
static CORE1_STACK_LO: AtomicU32 = AtomicU32::new(0);
static CORE1_STACK_HI: AtomicU32 = AtomicU32::new(0);

/// Core 0's stack, `[_stack_end, _stack_start)` (memory.x).
fn core0_stack() -> (u32, u32) {
    unsafe extern "C" {
        static _stack_end: u32;
        static _stack_start: u32;
    }
    (
        core::ptr::addr_of!(_stack_end) as u32,
        core::ptr::addr_of!(_stack_start) as u32,
    )
}

/// Paints core 0's stack from `_stack_end` up to 256 B below the stack pointer. Call first thing
/// in `main`, before any interrupt is enabled: nothing below the stack pointer is live then.
#[inline(never)]
pub fn paint_core0() {
    let (lo, _) = core0_stack();
    let hi = cortex_m::register::msp::read().saturating_sub(256) & !3;
    let mut p = lo;
    while p < hi {
        // SAFETY: `[_stack_end, hi)` is core 0's stack below the stack pointer, which nothing uses
        // yet (no interrupt enabled, the only code running is this function, whose frame is above).
        unsafe { (p as *mut u32).write_volatile(STACK_PAINT) };
        p += 4;
    }
}

/// Core 0's peak since boot (bytes used), scanned just before the re-paint; 0 until then.
static CORE0_BOOT_USED: AtomicU32 = AtomicU32::new(0);
/// Set by the one call of [`repaint_core0`] (the first status report): it never runs twice.
static CORE0_REPAINTED: AtomicBool = AtomicBool::new(false);

/// Re-paints the unused part of core 0's stack once, so the mark shows the depth after start-up
/// (the boot peak, one 16.7 KB frame of `flight::steer_core()` or `flight::vert()`, built one after the other, hides it otherwise).
/// Only called from [`stat`], which runs in the supervisor task on core 0's thread-mode executor
/// (`#[embassy_executor::main]`), never from core 1. It runs with interrupts enabled. That is safe
/// because the loop writes only below `MSP - 256`: an interrupt that preempts it pushes its
/// exception frame (8 words, 26 with a lazily stacked FPU frame: at most 104 B) and its handler's
/// frames below the current stack pointer, which is above `hi`'s 256 B allowance or at worst
/// inside the region being painted, and all of that is dead once the handler returns, before the
/// loop's next write; nothing live lies below the stack pointer. The only effect is that a deep
/// handler's use of the range the loop has not yet reached is overwritten by the paint (an
/// under-count of the since-start-up peak, bounded by the handler's one-off depth).
#[inline(never)]
fn repaint_core0() {
    let (lo, _) = core0_stack();
    let hi = cortex_m::register::msp::read().saturating_sub(256) & !3;
    let mut p = lo;
    while p < hi {
        // SAFETY: `[_stack_end, hi)` is core 0's dead stack below the stack pointer (see above).
        unsafe { (p as *mut u32).write_volatile(STACK_PAINT) };
        p += 4;
    }
}

/// Paints core 1's stack (all of it) and records where it is. Call on core 0 before `spawn_core1`.
pub fn paint_core1(mem: &mut [u8]) {
    for word in mem.chunks_exact_mut(4) {
        word.copy_from_slice(&STACK_PAINT.to_ne_bytes());
    }
    let lo = mem.as_ptr() as u32;
    CORE1_STACK_LO.store(lo, Ordering::Relaxed);
    CORE1_STACK_HI.store(lo + (mem.len() as u32 & !3), Ordering::Relaxed);
}

/// (used, free) bytes of the stack `[lo, hi)`: free = the painted words from `lo` up.
fn stack_use((lo, hi): (u32, u32)) -> (u32, u32) {
    let mut p = lo;
    // SAFETY: word reads of a stack's own memory (core 1's is a static it runs on: a word read
    // races its writes harmlessly, the value only feeds a count); `lo` and `hi` are word aligned.
    while p < hi && unsafe { (p as *const u32).read_volatile() } == STACK_PAINT {
        p += 4;
    }
    (hi - p, p - lo)
}

/// The REBOOT action: writes the REQUESTED record and asks the monitor to reset; the reset comes
/// `REBOOT_AFTER_ACK_MS` after the final ACK was handed to the link ([`reboot_ack_handed`]), or
/// `REBOOT_NO_ACK_MS` after this call when no ACK was handed (link gone). Nothing is saved.
pub fn request_reboot(now_ms: u32) {
    REBOOT_AT_MS.store(
        now_ms.wrapping_add(wdg::REBOOT_NO_ACK_MS),
        Ordering::Relaxed,
    );
    REBOOT_REQ.store(true, Ordering::Release);
    write_record(scratch::encode(Reason::Requested, task::ACTION, 0, now_ms));
}

/// The reboot's final ACK was handed to the link now.
pub fn reboot_ack_handed(now_ms: u32) {
    if REBOOT_REQ.load(Ordering::Acquire) {
        REBOOT_AT_MS.store(
            now_ms.wrapping_add(wdg::REBOOT_AFTER_ACK_MS),
            Ordering::Relaxed,
        );
        defmt::info!(
            "[INFO] reboot: final ACK handed to the link at {=u32} ms, reset due at {=u32} ms",
            now_ms,
            now_ms.wrapping_add(wdg::REBOOT_AFTER_ACK_MS)
        );
    }
}

/// Features `hang-core0`, `spin-1s`, `hang-irqoff`: called in the barometer's `acquire`; once,
/// 20 s after a start the watchdog did not cause, spins forever (the executor stalls), 1.0 s, or
/// forever with interrupts off (the monitor cannot run).
#[cfg(any(feature = "hang-core0", feature = "spin-1s", feature = "hang-irqoff"))]
pub fn hang_point_core0() {
    if Instant::now() < Instant::from_secs(20) || !HANG_ARMED.swap(false, Ordering::Relaxed) {
        return;
    }
    let begun = Instant::now();
    #[cfg(feature = "hang-core0")]
    {
        defmt::info!(
            "[INFO] hang-core0: BARO spins forever from {=u64} ms",
            begun.as_millis()
        );
        loop {
            core::hint::spin_loop();
        }
    }
    #[cfg(feature = "hang-irqoff")]
    {
        defmt::info!(
            "[INFO] hang-irqoff: interrupts off, spinning forever from {=u64} ms",
            begun.as_millis()
        );
        cortex_m::interrupt::disable();
        loop {
            core::hint::spin_loop();
        }
    }
    #[cfg(feature = "spin-1s")]
    {
        defmt::info!(
            "[INFO] spin-1s: BARO spins 1.0 s from {=u64} ms",
            begun.as_millis()
        );
        while begun.elapsed() < Duration::from_millis(1000) {
            core::hint::spin_loop();
        }
        defmt::info!(
            "[INFO] spin-1s: spun from {=u64} to {=u64} ms",
            begun.as_millis(),
            Instant::now().as_millis()
        );
    }
}

/// Feature `hang-flight`: set by core 0 ([`hang_flight_arm`]); IMU1's `acquire` on the flight core
/// then spins forever (a plain atomic load there; no timer, no log).
#[cfg(feature = "hang-flight")]
static HANG_FLIGHT_GO: AtomicBool = AtomicBool::new(false);

#[cfg(feature = "hang-flight")]
pub fn hang_point_flight() {
    if HANG_FLIGHT_GO.load(Ordering::Relaxed) {
        loop {
            core::hint::spin_loop();
        }
    }
}

#[cfg(feature = "hang-flight")]
#[embassy_executor::task]
pub async fn hang_flight_arm() {
    fw_flight::tagged(&RUNNING0, task::TEST, async {
        embassy_time::Timer::at(Instant::from_secs(20)).await;
        if HANG_ARMED.swap(false, Ordering::Relaxed) {
            defmt::info!(
                "[INFO] hang-flight: IMU1 spins forever from about {=u64} ms",
                Instant::now().as_millis()
            );
            HANG_FLIGHT_GO.store(true, Ordering::Relaxed);
        }
    })
    .await
}
