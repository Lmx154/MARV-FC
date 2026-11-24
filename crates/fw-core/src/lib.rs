//! Pure `no_std` core of the MARV V2 firmware: the logic that is tested on the host.
//! Device health, parameters, record codecs, command admission, calibration maths and the lock-free
//! cells between the two cores.
#![no_std]
// `deny`, not `forbid`: the `xcore` module (lock-free cells) uses `unsafe`, and `forbid` cannot be
// relaxed for a single module.
#![deny(unsafe_code)]
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

#[cfg(test)]
extern crate std;

pub mod admission;
pub mod arming;
pub mod calib;
pub mod dflog;
pub mod esc;
pub mod extring;
pub mod flight_cmd;
pub mod handshake;
pub mod health;
pub mod i2c;
pub mod led;
pub mod logfile;
pub mod mbr;
pub mod mission;
pub mod params;
pub mod rc;
pub mod record;
pub mod relay;
#[cfg(test)]
mod ring_contract;
pub mod rotation;
pub mod sample;
pub mod scratch;
pub mod selftest;
pub mod sys_status;
pub mod text_ring;
pub mod watchdog;
pub mod xcore;
