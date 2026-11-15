//! Device-specific pure logic of the MARV V2 firmware: per device, the parameter table and the pure
//! `settings_to_config` with its validator. No hardware, no clock; tested on the host. Generic
//! logic (the store, the health machine, admission) is in `fw-core`.
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

#[cfg(test)]
extern crate std;

pub mod baro;
pub mod mag;
