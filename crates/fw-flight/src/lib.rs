//! Flight-core (core 1) tasks of the MARV V2 firmware.
//! Empty skeleton for the foundation layer: nothing is implemented yet.
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

pub mod imu;
