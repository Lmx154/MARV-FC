//! Flight-core (core 1) tasks of the MARV V2 firmware.
//! Today: the liveness answer [`liveness`]; the IMU tasks come with the device milestones.
//! Rule: no dependency on `embassy-time`, `embassy-sync` or `defmt`.
#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![cfg_attr(
    not(test),
    deny(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::panic,
        clippy::unreachable,
        clippy::todo,
        clippy::unimplemented,
        clippy::indexing_slicing,
        clippy::undocumented_unsafe_blocks
    )
)]

use core::sync::atomic::{AtomicU32, Ordering};
use fw_core::xcore::Kick;

pub mod imu;

/// The flight core's liveness answer (`fbeat`): waits on `kick`, then copies `ping` into `pong`,
/// forever. Core 0 bumps `ping` and kicks; a `pong` that stops following `ping` means the flight
/// core makes no progress. Any kick bits wake it. No timer, no lock, no log.
pub async fn liveness(kick: &Kick, ping: &AtomicU32, pong: &AtomicU32) -> ! {
    loop {
        kick.wait().await;
        pong.store(ping.load(Ordering::Acquire), Ordering::Release);
    }
}
