//! The settings record codec for the flash settings region.
//!
//! The region is two 4096-byte flash sectors, A and B, with one record per sector. A save writes
//! the sector that does not hold the current record; at boot the valid record with the highest
//! sequence wins. This module only turns bytes into records and back: it never touches flash,
//! and it does not know which parameters exist (the caller decides which values are stored).
//!
//! # Layout (version 1, all integers little-endian)
//!
//! A record starts at offset 0 of its sector. Its length is `16 + 24 * count + 4` bytes, at most
//! 4076 for 169 entries.
//!
//! | Offset                  | Size | Field                                                       |
//! |-------------------------|------|-------------------------------------------------------------|
//! | 0                       | 4    | magic, the ASCII bytes `MRVS`                               |
//! | 4                       | 2    | layout version `u16`, 1                                     |
//! | 6                       | 2    | entry count `u16`, 0 to 169                                 |
//! | 8                       | 4    | sequence number `u32`                                       |
//! | 12                      | 4    | record length `u32` in bytes, CRC included (exactly the formula above) |
//! | 16 + 24 * i             | 24   | entry `i`                                                   |
//! | 16 + 24 * count         | 4    | CRC-32 `u32`                                                |
//! | 20 + 24 * count to 4095 | rest | unused tail, left 0xFF                                      |
//!
//! Header size: 16 bytes. Entry (24 bytes):
//!
//! | Offset | Size | Field                                                                    |
//! |--------|------|--------------------------------------------------------------------------|
//! | 0      | 16   | name: ASCII `A-Z0-9_`, zero-padded to 16 bytes                           |
//! | 16     | 1    | type code: 1 = `I32`, 2 = `F32`                                          |
//! | 17     | 3    | padding, MUST be zero                                                    |
//! | 20     | 4    | value `u32`: the `i32` as is, or the `f32` as `to_bits`                  |
//!
//! The CRC is the IEEE CRC-32 (reflected polynomial 0xEDB88320, initial value and final XOR
//! 0xFFFFFFFF; the check value of `"123456789"` is 0xCBF43926). It covers bytes
//! `0 .. length - 4`: the header and every entry. It sits in the last four bytes of the record.
//!
//! The unused tail (bytes from `length` to 4095) is NOT covered by the CRC and is ignored by the
//! decoder: it is not part of the record. A corruption there does not change which record is
//! chosen. An erased sector reads 0xFF everywhere.
//!
//! # Decoding rules
//!
//! A sector is `Erased` when all 4096 bytes are 0xFF. It is `Invalid` when the magic, the layout
//! version, the count (above 169), the length (not exactly the formula) or the CRC is wrong. A
//! record with a good CRC is valid; inside it an entry is skipped (and counted in
//! [`Record::skipped`]) when its name is not a valid [`ParamName`] or not zero-padded, its type
//! code is not 1 or 2, or its padding is not zero.
//!
//! # Choosing
//!
//! [`choose`] takes the valid record with the highest sequence, compared as a plain `u32`
//! (no wrap-around handling). With equal sequences sector A wins. The next save writes the sector
//! that does not hold the current record (A when there is none) with sequence `current + 1`,
//! saturating at `u32::MAX` (unreachable in practice: flash endurance is far below 2^32 saves).

use mavlink_device::{ParamName, ParamValue};

/// Size of one settings sector, in bytes.
pub const SECTOR_LEN: usize = 4096;
/// Most entries one record holds.
pub const MAX_ENTRIES: usize = 169;
/// Size of the record header, in bytes.
pub const HEADER_LEN: usize = 16;
/// Size of one entry, in bytes.
pub const ENTRY_LEN: usize = 24;
/// The layout version this module reads and writes.
pub const LAYOUT_VERSION: u16 = 1;
/// The record magic.
pub const MAGIC: [u8; 4] = *b"MRVS";

const CRC_LEN: usize = 4;
const TYPE_I32: u8 = 1;
const TYPE_F32: u8 = 2;

const _: () = assert!(HEADER_LEN + MAX_ENTRIES * ENTRY_LEN + CRC_LEN <= SECTOR_LEN);

/// One stored parameter value.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Entry {
    /// The parameter's name.
    pub name: ParamName,
    /// The parameter's value.
    pub value: ParamValue,
}

/// Why [`encode`] refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EncodeError {
    /// More than [`MAX_ENTRIES`] entries were given.
    TooManyEntries,
}

/// One of the two settings sectors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sector {
    /// The first sector.
    A,
    /// The second sector.
    B,
}

impl Sector {
    /// The other sector.
    pub const fn other(self) -> Self {
        match self {
            Self::A => Self::B,
            Self::B => Self::A,
        }
    }
}

/// The IEEE CRC-32 of `data` (bitwise, no table).
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

fn put(dst: &mut [u8], offset: usize, bytes: &[u8]) {
    if let Some(slot) = dst.get_mut(offset..offset.saturating_add(bytes.len())) {
        slot.copy_from_slice(bytes);
    }
}

fn u16_at(bytes: &[u8], offset: usize) -> Option<u16> {
    let raw = bytes.get(offset..offset.checked_add(2)?)?;
    Some(u16::from_le_bytes(<[u8; 2]>::try_from(raw).ok()?))
}

fn u32_at(bytes: &[u8], offset: usize) -> Option<u32> {
    let raw = bytes.get(offset..offset.checked_add(4)?)?;
    Some(u32::from_le_bytes(<[u8; 4]>::try_from(raw).ok()?))
}

fn write_entry(out: &mut [u8], offset: usize, entry: &Entry) {
    put(out, offset, &[0u8; ENTRY_LEN]);
    put(out, offset, entry.name.as_bytes());
    let (code, bits) = match entry.value {
        ParamValue::I32(v) => (TYPE_I32, v as u32),
        ParamValue::F32(v) => (TYPE_F32, v.to_bits()),
    };
    put(out, offset + 16, &[code]);
    put(out, offset + 20, &bits.to_le_bytes());
}

/// Encodes a record with `sequence` and `entries` into `out`, and returns the record's length.
///
/// The whole buffer is first set to 0xFF, so the unused tail stays erased and the buffer can be
/// programmed as it is. When more than [`MAX_ENTRIES`] entries are given, `out` is left all 0xFF
/// (no half record) and the error is returned.
pub fn encode<I>(
    sequence: u32,
    entries: I,
    out: &mut [u8; SECTOR_LEN],
) -> Result<usize, EncodeError>
where
    I: IntoIterator<Item = Entry>,
{
    out.fill(0xFF);
    let mut count = 0usize;
    for entry in entries {
        if count >= MAX_ENTRIES {
            out.fill(0xFF);
            return Err(EncodeError::TooManyEntries);
        }
        write_entry(out, HEADER_LEN + count * ENTRY_LEN, &entry);
        count += 1;
    }
    let length = HEADER_LEN + count * ENTRY_LEN + CRC_LEN;
    put(out, 0, &MAGIC);
    put(out, 4, &LAYOUT_VERSION.to_le_bytes());
    put(out, 6, &(count as u16).to_le_bytes());
    put(out, 8, &sequence.to_le_bytes());
    put(out, 12, &(length as u32).to_le_bytes());
    let crc = crc32(out.get(..length - CRC_LEN).unwrap_or(&[]));
    put(out, length - CRC_LEN, &crc.to_le_bytes());
    Ok(length)
}

fn parse_entry(chunk: &[u8]) -> Option<Entry> {
    let name = chunk.get(..16)?;
    let end = name.iter().position(|&b| b == 0).unwrap_or(16);
    if !name.get(end..)?.iter().all(|&b| b == 0) {
        return None;
    }
    let name = ParamName::from_bytes(name.get(..end)?).ok()?;
    if !chunk.get(17..20)?.iter().all(|&b| b == 0) {
        return None;
    }
    let bits = u32_at(chunk, 20)?;
    let value = match *chunk.get(16)? {
        TYPE_I32 => ParamValue::I32(bits as i32),
        TYPE_F32 => ParamValue::F32(f32::from_bits(bits)),
        _ => return None,
    };
    Some(Entry { name, value })
}

/// A record that passed the magic, version, length and CRC checks.
#[derive(Clone, Copy, Debug)]
pub struct Record<'a> {
    bytes: &'a [u8; SECTOR_LEN],
    sequence: u32,
    count: usize,
    skipped: usize,
}

impl<'a> Record<'a> {
    fn entry_bytes(&self) -> &'a [u8] {
        self.bytes
            .get(HEADER_LEN..HEADER_LEN + self.count * ENTRY_LEN)
            .unwrap_or(&[])
    }

    /// The record's sequence number.
    pub fn sequence(&self) -> u32 {
        self.sequence
    }

    /// The entry count the record declares (valid and skipped entries together).
    pub fn declared_count(&self) -> usize {
        self.count
    }

    /// How many entries were skipped: invalid name, type code or padding.
    pub fn skipped(&self) -> usize {
        self.skipped
    }

    /// The valid entries, in stored order (at most [`MAX_ENTRIES`]).
    pub fn entries(&self) -> impl Iterator<Item = Entry> + use<'a> {
        self.entry_bytes()
            .as_chunks::<ENTRY_LEN>()
            .0
            .iter()
            .filter_map(|chunk| parse_entry(chunk))
    }
}

/// The result of decoding one sector.
#[derive(Clone, Copy, Debug)]
pub enum Decoded<'a> {
    /// Every byte is 0xFF.
    Erased,
    /// Not erased and not a valid record.
    Invalid,
    /// A valid record.
    Valid(Record<'a>),
}

/// Decodes one sector's bytes.
pub fn decode(bytes: &[u8; SECTOR_LEN]) -> Decoded<'_> {
    decode_impl(bytes, true)
}

fn decode_impl(bytes: &[u8; SECTOR_LEN], check_crc: bool) -> Decoded<'_> {
    if bytes.iter().all(|&b| b == 0xFF) {
        return Decoded::Erased;
    }
    parse_record(bytes, check_crc).map_or(Decoded::Invalid, Decoded::Valid)
}

fn parse_record(bytes: &[u8; SECTOR_LEN], check_crc: bool) -> Option<Record<'_>> {
    if bytes.get(..4)? != MAGIC {
        return None;
    }
    if u16_at(bytes, 4)? != LAYOUT_VERSION {
        return None;
    }
    let count = usize::from(u16_at(bytes, 6)?);
    if count > MAX_ENTRIES {
        return None;
    }
    let sequence = u32_at(bytes, 8)?;
    let length = HEADER_LEN + count * ENTRY_LEN + CRC_LEN;
    if u32_at(bytes, 12)? as usize != length {
        return None;
    }
    if check_crc {
        let stored = u32_at(bytes, length - CRC_LEN)?;
        if crc32(bytes.get(..length - CRC_LEN)?) != stored {
            return None;
        }
    }
    let mut record = Record {
        bytes,
        sequence,
        count,
        skipped: 0,
    };
    record.skipped = record
        .entry_bytes()
        .as_chunks::<ENTRY_LEN>()
        .0
        .iter()
        .filter(|chunk| parse_entry(*chunk).is_none())
        .count();
    Some(record)
}

/// What the boot found in the settings region.
#[derive(Clone, Copy, Debug)]
pub enum Boot<'a> {
    /// A current record, in `sector`.
    Current {
        /// The sector holding it.
        sector: Sector,
        /// The record.
        record: Record<'a>,
    },
    /// No valid record and both sectors erased: run on defaults (INFO).
    DefaultsErased,
    /// No valid record and at least one sector not erased: run on defaults (WARNING); nothing
    /// is erased.
    DefaultsInvalid,
}

/// The decision of [`choose`].
#[derive(Clone, Copy, Debug)]
pub struct Choice<'a> {
    /// What the boot found.
    pub boot: Boot<'a>,
    /// The sector the next save must write: the one not holding the current record, A if none.
    pub write_sector: Sector,
    /// The sequence number of the next save: current + 1 (saturating), 1 when there is no record.
    pub next_sequence: u32,
}

/// Picks the current record from both sectors' bytes.
///
/// The valid record with the highest sequence (plain `u32` comparison) wins; with equal
/// sequences sector A wins.
pub fn choose<'a>(a: &'a [u8; SECTOR_LEN], b: &'a [u8; SECTOR_LEN]) -> Choice<'a> {
    choose_impl(a, b, true)
}

fn choose_impl<'a>(
    a: &'a [u8; SECTOR_LEN],
    b: &'a [u8; SECTOR_LEN],
    check_crc: bool,
) -> Choice<'a> {
    let da = decode_impl(a, check_crc);
    let db = decode_impl(b, check_crc);
    let current = match (da, db) {
        (Decoded::Valid(ra), Decoded::Valid(rb)) => {
            if rb.sequence > ra.sequence {
                Some((Sector::B, rb))
            } else {
                Some((Sector::A, ra))
            }
        }
        (Decoded::Valid(ra), _) => Some((Sector::A, ra)),
        (_, Decoded::Valid(rb)) => Some((Sector::B, rb)),
        _ => None,
    };
    match current {
        Some((sector, record)) => Choice {
            boot: Boot::Current { sector, record },
            write_sector: sector.other(),
            next_sequence: record.sequence.saturating_add(1),
        },
        None => {
            let both_erased = matches!((da, db), (Decoded::Erased, Decoded::Erased));
            Choice {
                boot: if both_erased {
                    Boot::DefaultsErased
                } else {
                    Boot::DefaultsInvalid
                },
                write_sector: Sector::A,
                next_sequence: 1,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn name(s: &str) -> ParamName {
        ParamName::new(s).unwrap()
    }

    fn entries(n: usize) -> Vec<Entry> {
        (0..n)
            .map(|i| Entry {
                name: name(&std::format!("P{i}")),
                value: if i % 2 == 0 {
                    ParamValue::I32((i as i32) * 1000 - 77)
                } else {
                    ParamValue::F32(i as f32 * 0.25 + 0.1)
                },
            })
            .collect()
    }

    fn sector(seq: u32, n: usize) -> ([u8; SECTOR_LEN], usize) {
        let mut buf = [0u8; SECTOR_LEN];
        let len = encode(seq, entries(n), &mut buf).unwrap();
        (buf, len)
    }

    const ERASED: [u8; SECTOR_LEN] = [0xFF; SECTOR_LEN];

    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 << 13;
            self.0 ^= self.0 >> 7;
            self.0 ^= self.0 << 17;
            self.0
        }
        fn fill(&mut self, buf: &mut [u8]) {
            for b in buf {
                *b = self.next() as u8;
            }
        }
    }

    fn current(c: &Choice) -> Option<(Sector, u32)> {
        match c.boot {
            Boot::Current { sector, record } => Some((sector, record.sequence())),
            _ => None,
        }
    }

    #[test]
    fn crc32_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32(b""), 0);
    }

    #[test]
    fn layout_hand_built() {
        let list = [
            Entry {
                name: name("A_B"),
                value: ParamValue::I32(-2),
            },
            Entry {
                name: name("PI"),
                value: ParamValue::F32(1.0),
            },
        ];
        let mut buf = [0u8; SECTOR_LEN];
        let len = encode(0x0102_0304, list, &mut buf).unwrap();
        assert_eq!(len, 68);
        let mut expected: Vec<u8> = Vec::new();
        expected.extend_from_slice(&[0x4D, 0x52, 0x56, 0x53]); // "MRVS"
        expected.extend_from_slice(&[0x01, 0x00]); // version 1
        expected.extend_from_slice(&[0x02, 0x00]); // count 2
        expected.extend_from_slice(&[0x04, 0x03, 0x02, 0x01]); // sequence
        expected.extend_from_slice(&[0x44, 0x00, 0x00, 0x00]); // length 68
        expected.extend_from_slice(&[0x41, 0x5F, 0x42]); // "A_B"
        expected.extend_from_slice(&[0u8; 13]);
        expected.extend_from_slice(&[0x01, 0, 0, 0]); // I32, padding
        expected.extend_from_slice(&[0xFE, 0xFF, 0xFF, 0xFF]); // -2
        expected.extend_from_slice(&[0x50, 0x49]); // "PI"
        expected.extend_from_slice(&[0u8; 14]);
        expected.extend_from_slice(&[0x02, 0, 0, 0]); // F32, padding
        expected.extend_from_slice(&[0x00, 0x00, 0x80, 0x3F]); // 1.0
        expected.extend_from_slice(&[0x22, 0xAD, 0xE4, 0x43]); // CRC-32 0x43E4AD22
        expected.resize(SECTOR_LEN, 0xFF);
        assert_eq!(expected.len(), SECTOR_LEN);
        assert_eq!(buf.as_slice(), expected.as_slice());
        assert_eq!(crc32(&buf[..64]), 0x43E4_AD22);
        let Decoded::Valid(r) = decode(&buf) else {
            panic!("not valid")
        };
        assert_eq!(r.sequence(), 0x0102_0304);
        assert_eq!(r.entries().collect::<Vec<_>>(), list);
    }

    #[test]
    fn round_trip_0_1_169() {
        for n in [0usize, 1, 2, 100, 168, 169] {
            let (buf, len) = sector(7, n);
            assert_eq!(len, 20 + 24 * n);
            assert!(buf[len..].iter().all(|&b| b == 0xFF), "tail erased, n={n}");
            let Decoded::Valid(r) = decode(&buf) else {
                panic!("n={n} not valid")
            };
            assert_eq!(r.sequence(), 7);
            assert_eq!(r.declared_count(), n);
            assert_eq!(r.skipped(), 0);
            assert_eq!(r.entries().collect::<Vec<_>>(), entries(n));
        }
        assert_eq!(20 + 24 * MAX_ENTRIES, 4076);
    }

    #[test]
    fn nan_and_extremes_round_trip_bitwise() {
        let list = [
            Entry {
                name: name("N"),
                value: ParamValue::F32(f32::from_bits(0x7FC1_2345)),
            },
            Entry {
                name: name("M"),
                value: ParamValue::I32(i32::MIN),
            },
            Entry {
                name: name("SIXTEEN_CHARS_OK"),
                value: ParamValue::F32(-0.0),
            },
            Entry {
                name: name("ABCDEFGHIJKLMNOP"),
                value: ParamValue::I32(i32::MAX),
            },
        ];
        let mut buf = [0u8; SECTOR_LEN];
        encode(1, list, &mut buf).unwrap();
        let Decoded::Valid(r) = decode(&buf) else {
            panic!()
        };
        let got: Vec<Entry> = r.entries().collect();
        assert_eq!(got.len(), 4);
        match (got[0].value, got[2].value) {
            (ParamValue::F32(a), ParamValue::F32(b)) => {
                assert_eq!(a.to_bits(), 0x7FC1_2345);
                assert_eq!(b.to_bits(), (-0.0f32).to_bits());
            }
            _ => panic!(),
        }
        assert_eq!(got[1], list[1]);
        assert_eq!(got[3], list[3]);
    }

    #[test]
    fn too_many_entries_refused() {
        let mut buf = [0u8; SECTOR_LEN];
        assert_eq!(
            encode(1, entries(170), &mut buf),
            Err(EncodeError::TooManyEntries)
        );
        assert!(buf.iter().all(|&b| b == 0xFF), "no half record left");
        assert!(encode(1, entries(169), &mut buf).is_ok());
    }

    /// Corrupts every position of the newest record with each of `xors`, in both sector orders,
    /// and returns (cases, cases where the older record was NOT chosen).
    fn corruption_run(check_crc: bool, n: usize, xors: &[u8]) -> (usize, usize) {
        let (old, _) = sector(5, 3);
        let (new, len) = sector(6, n);
        let mut cases = 0;
        let mut escapes = 0;
        for pos in 0..len {
            for &x in xors {
                let mut bad = new;
                bad[pos] ^= x;
                cases += 1;
                let c = choose_impl(&old, &bad, check_crc);
                if current(&c) != Some((Sector::A, 5)) || !matches!(c.write_sector, Sector::B) {
                    escapes += 1;
                }
                let c = choose_impl(&bad, &old, check_crc);
                cases += 1;
                if current(&c) != Some((Sector::B, 5)) || !matches!(c.write_sector, Sector::A) {
                    escapes += 1;
                }
            }
        }
        (cases, escapes)
    }

    #[test]
    fn i3_2_every_single_byte_corruption_falls_back() {
        let mut total = 0;
        for n in [0usize, 1, 2, 3, 7, 169] {
            let (cases, escapes) = corruption_run(true, n, &[0x01, 0x80, 0xFF]);
            assert_eq!(escapes, 0, "n={n}");
            total += cases;
        }
        let all: Vec<u8> = (1..=255).collect();
        let (cases, escapes) = corruption_run(true, 1, &all);
        assert_eq!(escapes, 0);
        total += cases;
        assert!(total > 20_000, "{total}");
    }

    #[test]
    fn corruption_in_unused_tail_is_not_covered_and_harmless() {
        let (old, _) = sector(5, 3);
        let (new, len) = sector(6, 3);
        for pos in len..SECTOR_LEN {
            for x in [0x01u8, 0x80, 0xFF] {
                let mut bad = new;
                bad[pos] ^= x;
                let c = choose(&old, &bad);
                assert_eq!(current(&c), Some((Sector::B, 6)), "pos={pos}");
            }
        }
    }

    #[test]
    fn negative_control_without_crc_lets_corruption_through() {
        let all: Vec<u8> = (1..=255).collect();
        let (cases, escapes) = corruption_run(false, 3, &[0x01, 0x80, 0xFF]);
        assert!(escapes > 0, "{escapes} of {cases}");
        let (_, escapes) = corruption_run(false, 1, &all);
        assert!(escapes > 0);
        // the same harness with the CRC: none
        assert_eq!(corruption_run(true, 3, &[0x01, 0x80, 0xFF]).1, 0);
    }

    #[test]
    fn i3_3_cut_write_leaves_previous_record() {
        let (old, _) = sector(5, 4);
        let mut checked = 0;
        for n in [0usize, 1, 4, 30, 169] {
            for seq in 6..40u32 {
                let (new, len) = sector(seq, n);
                if new[len - 1] == 0xFF {
                    // a cut would leave the same bytes as the whole record
                    continue;
                }
                for k in 0..len {
                    let mut cut = ERASED;
                    cut[..k].copy_from_slice(&new[..k]);
                    let c = choose(&old, &cut);
                    assert_eq!(current(&c), Some((Sector::A, 5)), "n={n} seq={seq} k={k}");
                    let c = choose(&cut, &old);
                    assert_eq!(current(&c), Some((Sector::B, 5)), "n={n} seq={seq} k={k}");
                    checked += 1;
                }
            }
        }
        assert!(checked > 5000, "{checked}");
    }

    #[test]
    fn boot_classification() {
        let (valid, _) = sector(3, 2);
        let c = choose(&ERASED, &ERASED);
        assert!(matches!(c.boot, Boot::DefaultsErased));
        assert_eq!((c.write_sector, c.next_sequence), (Sector::A, 1));

        let c = choose(&ERASED, &valid);
        assert_eq!(current(&c), Some((Sector::B, 3)));
        assert_eq!((c.write_sector, c.next_sequence), (Sector::A, 4));
        let c = choose(&valid, &ERASED);
        assert_eq!(current(&c), Some((Sector::A, 3)));
        assert_eq!((c.write_sector, c.next_sequence), (Sector::B, 4));

        let mut junk = [0u8; SECTOR_LEN];
        Rng(99).fill(&mut junk);
        let c = choose(&junk, &junk);
        assert!(matches!(c.boot, Boot::DefaultsInvalid));
        assert_eq!((c.write_sector, c.next_sequence), (Sector::A, 1));
        let c = choose(&junk, &ERASED);
        assert!(matches!(c.boot, Boot::DefaultsInvalid));
        let c = choose(&ERASED, &junk);
        assert!(matches!(c.boot, Boot::DefaultsInvalid));
        let mut one_byte = ERASED;
        one_byte[4000] = 0;
        assert!(matches!(decode(&one_byte), Decoded::Invalid));
        let c = choose(&one_byte, &ERASED);
        assert!(matches!(c.boot, Boot::DefaultsInvalid));
        let zeros = [0u8; SECTOR_LEN];
        assert!(matches!(decode(&zeros), Decoded::Invalid));

        // valid + invalid: the valid one is current, the invalid one is written next
        let c = choose(&junk, &valid);
        assert_eq!(current(&c), Some((Sector::B, 3)));
        assert_eq!(c.write_sector, Sector::A);
    }

    #[test]
    fn highest_sequence_wins_plain_u32_and_ties_go_to_a() {
        let (lo, _) = sector(10, 1);
        let (hi, _) = sector(11, 2);
        assert_eq!(current(&choose(&lo, &hi)), Some((Sector::B, 11)));
        assert_eq!(current(&choose(&hi, &lo)), Some((Sector::A, 11)));

        let (x, _) = sector(7, 1);
        let (y, _) = sector(7, 2);
        let c = choose(&x, &y);
        assert_eq!(current(&c), Some((Sector::A, 7)));
        assert_eq!((c.write_sector, c.next_sequence), (Sector::B, 8));
        let c = choose(&y, &x);
        assert_eq!(current(&c), Some((Sector::A, 7)));

        // plain comparison: 0 does not beat u32::MAX
        let (zero, _) = sector(0, 1);
        let (max, _) = sector(u32::MAX, 1);
        assert_eq!(current(&choose(&zero, &max)), Some((Sector::B, u32::MAX)));
        let c = choose(&max, &zero);
        assert_eq!(current(&c), Some((Sector::A, u32::MAX)));
        assert_eq!(c.next_sequence, u32::MAX);
        assert_eq!(current(&choose(&zero, &ERASED)), Some((Sector::A, 0)));
    }

    #[test]
    fn invalid_entries_are_skipped_and_counted() {
        let list = [
            Entry {
                name: name("GOOD1"),
                value: ParamValue::I32(1),
            },
            Entry {
                name: name("BAD_TYPE"),
                value: ParamValue::I32(2),
            },
            Entry {
                name: name("BAD_PAD"),
                value: ParamValue::I32(3),
            },
            Entry {
                name: name("BAD_NAME"),
                value: ParamValue::I32(4),
            },
            Entry {
                name: name("GOOD2"),
                value: ParamValue::F32(2.5),
            },
            Entry {
                name: name("BAD_TAIL"),
                value: ParamValue::I32(5),
            },
        ];
        let mut buf = [0u8; SECTOR_LEN];
        let len = encode(1, list, &mut buf).unwrap();
        let base = |i: usize| HEADER_LEN + i * ENTRY_LEN;
        buf[base(1) + 16] = 9; // unknown type code
        buf[base(2) + 18] = 1; // padding not zero
        buf[base(3)] = b'a'; // lower case name
        buf[base(5) + 12] = b'X'; // non-zero byte after the name's first zero
        let crc = crc32(&buf[..len - 4]);
        buf[len - 4..len].copy_from_slice(&crc.to_le_bytes());
        let Decoded::Valid(r) = decode(&buf) else {
            panic!()
        };
        assert_eq!(r.declared_count(), 6);
        assert_eq!(r.skipped(), 4);
        let got: Vec<Entry> = r.entries().collect();
        assert_eq!(got, [list[0], list[4]]);
        // an empty name is also invalid
        buf[base(0)..base(0) + 16].fill(0);
        let crc = crc32(&buf[..len - 4]);
        buf[len - 4..len].copy_from_slice(&crc.to_le_bytes());
        let Decoded::Valid(r) = decode(&buf) else {
            panic!()
        };
        assert_eq!(r.skipped(), 5);
        assert_eq!(r.entries().count(), 1);
    }

    #[test]
    fn header_fields_are_checked() {
        let (good, len) = sector(3, 2);
        let refix = |buf: &mut [u8; SECTOR_LEN]| {
            let crc = crc32(&buf[..len - 4]);
            buf[len - 4..len].copy_from_slice(&crc.to_le_bytes());
        };
        // each field wrong while the CRC is made right again: still Invalid
        for (pos, val) in [(0usize, b'X'), (4, 2), (6, 3), (12, 0x45), (13, 1)] {
            let mut b = good;
            b[pos] = val;
            refix(&mut b);
            assert!(matches!(decode(&b), Decoded::Invalid), "pos={pos}");
        }
        // a count above 169 with a matching length
        let mut b = ERASED;
        b[..4].copy_from_slice(&MAGIC);
        b[4..6].copy_from_slice(&1u16.to_le_bytes());
        b[6..8].copy_from_slice(&170u16.to_le_bytes());
        b[12..16].copy_from_slice(&(20u32 + 24 * 170).to_le_bytes());
        assert!(matches!(decode(&b), Decoded::Invalid));
    }

    #[test]
    fn decode_never_panics_and_never_yields_more_than_169() {
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
        let mut buf = [0u8; SECTOR_LEN];
        for _ in 0..10_000 {
            rng.fill(&mut buf);
            if let Decoded::Valid(r) = decode(&buf) {
                assert!(r.entries().count() <= MAX_ENTRIES);
            }
            let _ = choose(&buf, &ERASED);
        }
        // a random body behind a valid header and a right CRC: entries are skipped, not trusted
        for i in 0..10_000u32 {
            rng.fill(&mut buf);
            let count = (rng.next() % 170) as usize;
            let len = 20 + 24 * count;
            buf[..4].copy_from_slice(&MAGIC);
            buf[4..6].copy_from_slice(&1u16.to_le_bytes());
            buf[6..8].copy_from_slice(&(count as u16).to_le_bytes());
            buf[8..12].copy_from_slice(&i.to_le_bytes());
            buf[12..16].copy_from_slice(&(len as u32).to_le_bytes());
            // half of the cases get plausible names so that some entries survive
            if i % 2 == 0 {
                for e in 0..count {
                    let o = HEADER_LEN + e * ENTRY_LEN;
                    buf[o..o + 16].fill(0);
                    buf[o] = b'A' + (rng.next() % 26) as u8;
                    buf[o + 16] = (rng.next() % 4) as u8;
                    buf[o + 17..o + 20].fill(0);
                }
            }
            let crc = crc32(&buf[..len - 4]);
            buf[len - 4..len].copy_from_slice(&crc.to_le_bytes());
            let Decoded::Valid(r) = decode(&buf) else {
                panic!("valid header and CRC must decode")
            };
            let n = r.entries().count();
            assert!(n <= MAX_ENTRIES);
            assert_eq!(n + r.skipped(), count);
        }
        // valid records followed by a random tail stay valid and unchanged
        for i in 0..2_000usize {
            let n = i % 170;
            let (mut buf, len) = sector(i as u32, n);
            rng.fill(&mut buf[len..]);
            let Decoded::Valid(r) = decode(&buf) else {
                panic!("tail must not matter")
            };
            assert_eq!(r.entries().collect::<Vec<_>>(), entries(n));
        }
    }
}
