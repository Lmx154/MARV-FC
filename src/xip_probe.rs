//! Feature `xip-stat`: a read-only measurement of the flash read mode and of the cost of one XIP cache
//! miss. It reads QMI and XIP_CTRL registers and writes only `XIP_CTRL.ctr_hit()` and `.ctr_acc()` (any
//! write clears a counter). Measurement only; the normal build does not contain it.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_rp::pac::{QMI, XIP_CTRL};
use embassy_time::Instant;

/// Three 32 kB areas far beyond the program image and below the settings region (0x101F_E000).
const AREAS: [usize; 3] = [0x1010_0000, 0x1011_0000, 0x1012_0000];
const AREA_BYTES: usize = 32 * 1024;
const STRIDE: usize = 8;
const CONTROL_BYTES: usize = 4 * 1024;

/// Time of the last counter clear, in milliseconds since start (wrapping 32 bits).
static LAST_MS: AtomicU32 = AtomicU32::new(0);

/// Reads one byte every `STRIDE` bytes of `bytes` bytes from `base`.
fn sweep(base: usize, bytes: usize) {
    let mut at = base;
    let end = base + bytes;
    while at < end {
        // SAFETY: `at` lies in the memory-mapped, readable XIP flash window (0x1010_0000 to 0x1013_0000,
        // inside the 2 MiB flash and outside the settings region); a one-byte volatile read has no
        // side effect and nothing is written.
        let byte = unsafe { core::ptr::read_volatile(at as *const u8) };
        core::hint::black_box(byte);
        at += STRIDE;
    }
}

fn clear() {
    XIP_CTRL.ctr_hit().write_value(0);
    XIP_CTRL.ctr_acc().write_value(0);
}

fn counters() -> (u32, u32) {
    (XIP_CTRL.ctr_acc().read(), XIP_CTRL.ctr_hit().read())
}

/// Prints the flash read mode, then the three miss-cost passes and the control pass.
pub fn run() {
    let timing = QMI.mem(0).timing().read();
    let rfmt = QMI.mem(0).rfmt().read();
    let rcmd = QMI.mem(0).rcmd().read();
    let ctrl = XIP_CTRL.ctrl().read();
    defmt::info!(
        "[XIP] QMI M0 timing {=u32:#010x} rfmt {=u32:#010x} rcmd {=u32:#010x}, XIP_CTRL.ctrl {=u32:#010x}",
        timing.0,
        rfmt.0,
        rcmd.0,
        ctrl.0
    );
    defmt::info!(
        "[XIP] clkdiv {=u8} rxdelay {=u8} cooldown {=u8} pagebreak {=u8}, read prefix {=u8:#04x} (prefix_len {=u8}, prefix_width {=u8}), addr_width {=u8}, suffix_len {=u8}, dummy_len {=u8} (dummy_width {=u8}), data_width {=u8} (0 single, 1 dual, 2 quad), dtr {=bool}, clk_sys {=u32} Hz",
        timing.clkdiv(),
        timing.rxdelay(),
        timing.cooldown(),
        timing.pagebreak() as u8,
        rcmd.prefix(),
        rfmt.prefix_len() as u8,
        rfmt.prefix_width() as u8,
        rfmt.addr_width() as u8,
        rfmt.suffix_len() as u8,
        rfmt.dummy_len() as u8,
        rfmt.dummy_width() as u8,
        rfmt.data_width() as u8,
        rfmt.dtr(),
        embassy_rp::clocks::clk_sys_freq()
    );

    for (pass, base) in AREAS.into_iter().enumerate() {
        clear();
        let begun = Instant::now();
        sweep(base, AREA_BYTES);
        let us = begun.elapsed().as_micros();
        let (acc, hit) = counters();
        let miss = acc.wrapping_sub(hit);
        let ns_per_miss = if miss == 0 {
            0
        } else {
            us * 1000 / u64::from(miss)
        };
        defmt::info!(
            "[XIP] pass {=usize} at {=usize:#010x}: {=u64} us, {=usize} reads, ACC {=u32} HIT {=u32}, {=u64} ns per miss",
            pass,
            base,
            us,
            AREA_BYTES / STRIDE,
            acc,
            hit,
            ns_per_miss
        );
    }

    sweep(AREAS[0], CONTROL_BYTES);
    clear();
    let begun = Instant::now();
    sweep(AREAS[0], CONTROL_BYTES);
    let us = begun.elapsed().as_micros();
    let (acc, hit) = counters();
    defmt::info!(
        "[XIP] control, same {=usize} B read twice, second reading: {=u64} us, {=usize} reads, ACC {=u32} HIT {=u32}",
        CONTROL_BYTES,
        us,
        CONTROL_BYTES / STRIDE,
        acc,
        hit
    );
    clear();
    LAST_MS.store(Instant::now().as_millis() as u32, Ordering::Relaxed);
}

/// One `[STAT]` line: XIP accesses, hits and misses since the previous call (or the end of `run`), then
/// both counters are cleared. The counters count both cores together and saturate at 32 bits.
pub fn stat(now: Instant) {
    let (acc, hit) = counters();
    clear();
    let now_ms = now.as_millis() as u32;
    let last = LAST_MS.swap(now_ms, Ordering::Relaxed);
    defmt::info!(
        "[STAT] XIP accesses {=u32} hits {=u32} misses {=u32} in {=u32} ms",
        acc,
        hit,
        acc.wrapping_sub(hit),
        now_ms.wrapping_sub(last)
    );
}
