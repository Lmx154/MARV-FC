//! The hardware watchdog (design: decision-foundation, two-core amendment, item 6).
//!
//! Feature `wdg-scratch-probe`: the check that the scratch registers 0 to 3 survive a watchdog
//! time-out reset and a forced reset (`trigger_reset`); no monitor runs in that build.

use embassy_rp::watchdog::{ResetReason, Watchdog};

/// The four scratch words S0..S3.
pub fn read_scratch(wd: &mut Watchdog) -> [u32; 4] {
    [0, 1, 2, 3].map(|i| wd.get_scratch(i))
}

/// Writes the four scratch words S0..S3.
pub fn write_scratch(wd: &mut Watchdog, words: [u32; 4]) {
    for (i, w) in words.into_iter().enumerate() {
        wd.set_scratch(i, w);
    }
}

pub fn reason_name(reason: Option<ResetReason>) -> &'static str {
    match reason {
        Some(ResetReason::TimedOut) => "TimedOut",
        Some(ResetReason::Forced) => "Forced",
        None => "none",
    }
}

#[cfg(feature = "wdg-scratch-probe")]
pub mod probe {
    use embassy_rp::watchdog::Watchdog;
    use embassy_time::{Duration, Instant, Timer};
    use fw_core::scratch::{Reason, decode, encode};
    use mavlink_device::dialect::MavSeverity;

    use super::{read_scratch, reason_name, write_scratch};

    const TASK_TIMEOUT: u8 = 0xA5;
    const TASK_FORCED: u8 = 0x5A;

    /// What the probe does 5 s after start.
    #[derive(Clone, Copy)]
    pub enum Next {
        /// Start the watchdog and never feed it.
        TimeOut,
        /// `trigger_reset`.
        Force,
        /// Nothing more.
        Done,
    }

    /// At boot: reads and logs the reset reason and the scratch words, then decides the next step.
    pub fn boot(wd: &mut Watchdog) -> Next {
        let reason = wd.reset_reason();
        let words = read_scratch(wd);
        let record = decode(words);
        defmt::info!(
            "[INFO] wdg-scratch-probe: reset reason {=str}, scratch {=u32:#010x} {=u32:#010x} {=u32:#010x} {=u32:#010x}, record {=bool}",
            reason_name(reason),
            words[0],
            words[1],
            words[2],
            words[3],
            record.is_some()
        );
        match record {
            Some(r) if r.reason == Reason::StallCore0 && r.task_id == TASK_TIMEOUT => {
                defmt::info!(
                    "[PASS] scratch 0-3 survived a watchdog reset (reason {=str}, uptime word {=u32})",
                    reason_name(reason),
                    r.uptime_ms
                );
                crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_INFO,
                    format_args!("probe: kept after {} reset", reason_name(reason)),
                );
                write_scratch(wd, encode(Reason::Requested, TASK_FORCED, 0, 54_321));
                Next::Force
            }
            Some(r) if r.reason == Reason::Requested && r.task_id == TASK_FORCED => {
                defmt::info!(
                    "[PASS] scratch 0-3 survived a forced reset (reason {=str}, uptime word {=u32})",
                    reason_name(reason),
                    r.uptime_ms
                );
                crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_INFO,
                    format_args!("probe: kept after {} reset", reason_name(reason)),
                );
                wd.set_scratch(0, 0);
                Next::Done
            }
            _ if reason.is_none() => {
                write_scratch(wd, encode(Reason::StallCore0, TASK_TIMEOUT, 200, 12_345));
                Next::TimeOut
            }
            _ => {
                defmt::error!(
                    "[FAIL] scratch 0-3 lost after a {=str} reset",
                    reason_name(reason)
                );
                crate::text_fmt(
                    MavSeverity::MAV_SEVERITY_CRITICAL,
                    format_args!("probe: LOST after {} reset", reason_name(reason)),
                );
                Next::Done
            }
        }
    }

    #[embassy_executor::task]
    pub async fn act(mut wd: Watchdog, next: Next) {
        Timer::after(Duration::from_secs(5)).await;
        match next {
            Next::TimeOut => {
                wd.pause_on_debug(true);
                defmt::info!(
                    "[INFO] wdg-scratch-probe: watchdog started at {=u64} ms (1600 ms), never fed",
                    Instant::now().as_millis()
                );
                wd.start(Duration::from_millis(1600));
                Timer::after(Duration::from_secs(10)).await;
                defmt::error!("[FAIL] wdg-scratch-probe: no reset 10 s after the start");
            }
            Next::Force => {
                defmt::info!(
                    "[INFO] wdg-scratch-probe: trigger_reset at {=u64} ms",
                    Instant::now().as_millis()
                );
                Timer::after(Duration::from_millis(100)).await;
                wd.trigger_reset();
            }
            Next::Done => {}
        }
    }
}
