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
pub mod batt;
pub mod imu;
pub mod mag;
pub mod sd;

use fw_core::params::ParamDef;

/// The number of parameters of the whole firmware.
pub const COUNT: usize = baro::COUNT + mag::COUNT + batt::COUNT + imu::COUNT + sd::COUNT;

/// The firmware's one parameter table: every device's table in the order BARO, MAG, BATT, IMU, SD.
pub static TABLE: [ParamDef; COUNT] = concat();

#[allow(clippy::indexing_slicing)] // the loops run exactly over each source table's length
const fn concat() -> [ParamDef; COUNT] {
    let mut out = [baro::TABLE[0]; COUNT];
    let mut n = 0;
    let mut i = 0;
    while i < baro::COUNT {
        out[n] = baro::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < mag::COUNT {
        out[n] = mag::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < batt::COUNT {
        out[n] = batt::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < imu::COUNT {
        out[n] = imu::TABLE[i];
        n += 1;
        i += 1;
    }
    i = 0;
    while i < sd::COUNT {
        out[n] = sd::TABLE[i];
        n += 1;
        i += 1;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::ParamStore;

    #[test]
    fn the_whole_table_is_accepted_with_every_validator() {
        let mut params = Params::<COUNT>::new(&TABLE).unwrap();
        assert!(params.set_validator(baro::TABLE[0].device, baro::validate));
        assert!(params.set_validator(mag::TABLE[0].device, mag::validate));
        assert_eq!(params.count() as usize, COUNT);
        assert_eq!(COUNT, 26);
        let mut expected = baro::TABLE
            .iter()
            .chain(mag::TABLE.iter())
            .chain(batt::TABLE.iter())
            .chain(imu::TABLE.iter())
            .chain(sd::TABLE.iter());
        for def in &TABLE {
            assert_eq!(def.name, expected.next().unwrap().name);
        }
        assert!(expected.next().is_none());
    }
}
