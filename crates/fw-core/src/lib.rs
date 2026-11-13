//! Pure `no_std` core of the MARV V2 firmware: the logic that is tested on the host.
//! Empty skeleton for the foundation layer: nothing is implemented yet.
#![no_std]
// `deny`, not `forbid`: the `xcore` module (lock-free cells) will need atomics and possibly
// `unsafe`, and `forbid` cannot be relaxed for a single module.
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
pub mod handshake;
pub mod health;
pub mod params;
pub mod record;
pub mod scratch;
pub mod sys_status;
pub mod xcore;

#[cfg(test)]
mod tests {
    #[test]
    fn skeleton_builds() {
        assert_eq!(1 + 1, 2);
    }
}
