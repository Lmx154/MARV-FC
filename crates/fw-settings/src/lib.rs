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
pub mod rc;
pub mod sd;

use fw_core::params::ParamDef;

/// The number of parameters of the whole firmware.
pub const COUNT: usize = baro::COUNT + mag::COUNT + batt::COUNT + imu::COUNT + sd::COUNT + rc::COUNT;

/// The firmware's one parameter table: every device's table in the order BARO, MAG, BATT, IMU, SD, RC.
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
    i = 0;
    while i < rc::COUNT {
        out[n] = rc::TABLE[i];
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
        assert_eq!(COUNT, 70);
        let mut expected = baro::TABLE
            .iter()
            .chain(mag::TABLE.iter())
            .chain(batt::TABLE.iter())
            .chain(imu::TABLE.iter())
            .chain(sd::TABLE.iter())
            .chain(rc::TABLE.iter());
        for def in &TABLE {
            assert_eq!(def.name, expected.next().unwrap().name);
        }
        assert!(expected.next().is_none());
    }

    /// decision-sd-logging Q4: the start burst of a log (the header, a PARM for every parameter,
    /// the boot texts that wait for the log's text cursor: the ring's depth and its sticky slots)
    /// is at most 16 KiB, also with the writer's alignment filler (under one 4 KiB piece) in the
    /// ring before it. Encoded from the table, not counted from literals.
    #[test]
    fn the_start_burst_is_at_most_16_kib() {
        use fw_core::dflog::{HEADER_SEQUENCE_LEN, encode_msg, encode_parm, write_header};
        use fw_core::text_ring::{DEPTH, STICKY};
        use mavlink_device::ParamValue;
        let as_f32 = |v: ParamValue| match v {
            ParamValue::I32(i) => i as f32,
            ParamValue::F32(f) => f,
        };
        let mut buf = [0u8; 256];
        let mut header = [0u8; HEADER_SEQUENCE_LEN];
        let mut burst = write_header(&mut header);
        assert_eq!(burst, HEADER_SEQUENCE_LEN);
        for def in &TABLE {
            let d = as_f32(def.default);
            let n = encode_parm(&mut buf, u64::MAX, def.name.as_bytes(), d, d);
            assert!(n > 0, "{}", def.name);
            burst += n;
        }
        let longest = [b'x'; 64];
        for _ in 0..DEPTH + STICKY {
            let n = encode_msg(&mut buf, u64::MAX, &longest);
            assert!(n > 0);
            burst += n;
        }
        const FILLER_MAX: usize = 4096 - 1;
        assert!(burst <= 16 * 1024, "start burst {burst} B");
        assert!(
            burst + FILLER_MAX <= 16 * 1024,
            "with the filler {} B",
            burst + FILLER_MAX
        );
    }
}
