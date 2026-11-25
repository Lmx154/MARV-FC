//! Feature `xip-park`: the board check of invariant R1 (decision-ram-code-20261115 Q5 in
//! MARV-V2-drivers): after `core1_main` starts, the flight core makes no XIP access. 30 s after start,
//! a 1 s window with both cores running must count more than 100 000 XIP accesses: it only proves the
//! counter runs (with core 1 out of XIP it counts core 0 alone, 1.5 to 1.8 million per second idle).
//! Then core 0 parks itself for 1.0 s in SRAM with its interrupts off (`park_core0`, a root of
//! tools/check-ram-code.py), so every XIP access counted in that window is core 1's: there must be
//! none, while IMU1 and IMU2 each deliver at least 790 samples and no error. With `ram-negative`
//! (`now_us` in flash) the parked window must count at least 1000. The watchdog survives (1.0 s plus
//! its feed period is under 1.6 s); the "core 0 stalled" and flight-core texts after it are expected.
//! Once, and not after a watchdog reset.

use core::arch::asm;
use core::sync::atomic::Ordering;

use embassy_rp::pac::{TIMER0, XIP_CTRL};
use embassy_time::{Duration, Timer};
use fw_core::watchdog::task;

use crate::{imu1, imu2, watchdog};

const START: Duration = Duration::from_secs(30);
const PARK_US: u32 = 1_000_000;
const MIN_SAMPLES: u32 = 790;
/// The unparked window proves only that the counter runs; it counts core 0 alone (1.5 to 1.8 million
/// per second idle, measured 2026-11-16).
const UNPARKED_MIN: u32 = 100_000;

/// Core 0 parked in SRAM with its interrupts off for `PARK_US`: clears the XIP counters, spins on
/// TIMER0 (1 MHz), reads them. Returns (accesses, hits, microseconds parked).
#[inline(never)]
#[unsafe(link_section = ".data.xip_park")]
fn park_core0() -> (u32, u32, u32) {
    // SAFETY: masks this core's interrupts until the `cpsie` below; nothing in between waits for one.
    unsafe { asm!("cpsid i", options(nomem, nostack, preserves_flags)) };
    XIP_CTRL.ctr_hit().write_value(0);
    XIP_CTRL.ctr_acc().write_value(0);
    let begun = TIMER0.timerawl().read();
    let mut now = begun;
    while now.wrapping_sub(begun) < PARK_US {
        now = TIMER0.timerawl().read();
    }
    let acc = XIP_CTRL.ctr_acc().read();
    let hit = XIP_CTRL.ctr_hit().read();
    // SAFETY: unmasks the interrupts masked above.
    unsafe { asm!("cpsie i", options(nomem, nostack, preserves_flags)) };
    (acc, hit, now.wrapping_sub(begun))
}

#[embassy_executor::task]
pub async fn task() {
    watchdog::RUNNING0.store(task::TEST, Ordering::Relaxed);
    Timer::after(START).await;
    watchdog::RUNNING0.store(task::TEST, Ordering::Relaxed);
    if !watchdog::HANG_ARMED.swap(false, Ordering::Relaxed) {
        return;
    }
    XIP_CTRL.ctr_hit().write_value(0);
    XIP_CTRL.ctr_acc().write_value(0);
    Timer::after(Duration::from_secs(1)).await;
    let acc = XIP_CTRL.ctr_acc().read();
    if acc > UNPARKED_MIN {
        defmt::info!(
            "[PASS] xip-park: unparked 1 s window, both cores running: CTR_ACC {=u32} (over 100 000: the counter runs)",
            acc
        );
    } else {
        defmt::error!(
            "[FAIL] xip-park: unparked 1 s window, both cores running: CTR_ACC {=u32} (over 100 000: the counter runs)",
            acc
        );
    }

    let cells = [&imu1::CELLS, &imu2::CELLS];
    let before = cells.map(|c| c.read().map_or((0, 0), |s| (s.samples, s.health.errors)));
    let (acc, hit, us) = park_core0();
    let after = cells.map(|c| c.read().map_or((0, 0), |s| (s.samples, s.health.errors)));
    let [(s1, e1), (s2, e2)] = [0, 1].map(|i| {
        (
            after[i].0.wrapping_sub(before[i].0),
            after[i].1.wrapping_sub(before[i].1),
        )
    });
    let imus_ok = s1 >= MIN_SAMPLES && s2 >= MIN_SAMPLES && e1 == 0 && e2 == 0;
    let (pass, wanted) = if cfg!(feature = "ram-negative") {
        (
            acc >= 1000,
            "NEGATIVE CONTROL, now_us in flash: CTR_ACC at least 1000",
        )
    } else {
        (
            acc == 0 && imus_ok,
            "CTR_ACC exactly 0, each IMU at least +790 samples and +0 errors",
        )
    };
    if pass {
        defmt::info!(
            "[PASS] xip-park: parked 1 s window ({=u32} us, core 0 in SRAM, interrupts off): CTR_ACC {=u32} (HIT {=u32}), IMU1 +{=u32} samples +{=u32} errors, IMU2 +{=u32} samples +{=u32} errors; {=str}",
            us,
            acc,
            hit,
            s1,
            e1,
            s2,
            e2,
            wanted
        );
    } else {
        defmt::error!(
            "[FAIL] xip-park: parked 1 s window ({=u32} us, core 0 in SRAM, interrupts off): CTR_ACC {=u32} (HIT {=u32}), IMU1 +{=u32} samples +{=u32} errors, IMU2 +{=u32} samples +{=u32} errors; {=str}",
            us,
            acc,
            hit,
            s1,
            e1,
            s2,
            e2,
            wanted
        );
    }
}
