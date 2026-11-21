//! A reader for our own `.BIN` files: counts, times and damage (design: decision-sd-logging, Q3
//! and SD3). For a host test of the writer and, later, an on-board check of a file read back.
//!
//! [`Walker`] is fed byte slices of any size, in order (a record may span two slices) and keeps a
//! [`Summary`]: per record type the count and the first and last `TimeUS`; the bytes skipped as
//! bad; whether the stream began with the FMT of FMT; how many FMT records match the table
//! [`TYPES`](super::TYPES) byte for byte and how many do not; and the bytes of an incomplete last
//! record. State of a fixed size, no allocation.
//!
//! Resynchronisation is ArduPilot's: a record starts with `A3 95 <type>` where `<type>` is a
//! type of our table; any byte that cannot continue such a prefix is skipped and counted as bad,
//! one at a time, so a damaged or unknown record costs its bytes (all of them, unless a
//! following byte pair looks like a header) and the next good header is found. A record is
//! counted when its whole length (from the table) has arrived; its content is not checked
//! beyond the header, except FMT records. A file cut by a power loss ends in the middle of a
//! record: the bytes of that last, incomplete record are reported as [`Summary::tail_bytes`]
//! and not as bad bytes.

use super::{FMT, HEAD1, HEAD2, HEADER_LEN, ID_FMT, TYPES, fmt_record};

/// Number of record types in the table, and the length of [`Summary::types`].
pub const NUM_TYPES: usize = TYPES.len();

/// The longest record of the table (the FMT, 89 bytes).
const MAX_RECORD: usize = {
    let mut max = 0;
    let mut rest = TYPES.as_slice();
    while let [def, tail @ ..] = rest {
        if def.length as usize > max {
            max = def.length as usize;
        }
        rest = tail;
    }
    max
};

/// What was seen of one record type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TypeStats {
    /// Records of this type.
    pub count: u32,
    /// `TimeUS` of the first record of this type (0 if none, and for FMT, which has none).
    pub first_time_us: u64,
    /// `TimeUS` of the last record of this type (0 if none, and for FMT).
    pub last_time_us: u64,
}

/// The result of a walk so far.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Summary {
    /// Bytes fed.
    pub bytes: u64,
    /// Whole records counted, of all types.
    pub records: u32,
    /// Per type, in the order of [`TYPES`](super::TYPES) (FMT first).
    pub types: [TypeStats; NUM_TYPES],
    /// Bytes skipped while looking for a header (saturating).
    pub bad_bytes: u32,
    /// Bytes of an incomplete record at the end of what was fed (0 to 88): the tail of a cut
    /// file. A header prefix of one or two bytes counts too.
    pub tail_bytes: usize,
    /// The very first bytes were the FMT of FMT, as the table writes it.
    pub began_with_fmt: bool,
    /// FMT records identical to the table's, byte for byte (the FMT of FMT included).
    pub fmt_matched: u32,
    /// FMT records that differ from the table's, or describe a type it does not have.
    pub fmt_mismatched: u32,
}

/// An incremental parser; see the module documentation.
///
/// ```
/// use fw_core::dflog::{write_header, HEADER_SEQUENCE_LEN};
/// use fw_core::dflog::walk::Walker;
///
/// let mut header = [0u8; HEADER_SEQUENCE_LEN];
/// assert_eq!(write_header(&mut header), HEADER_SEQUENCE_LEN);
/// let mut walker = Walker::new();
/// for piece in header.chunks(7) {
///     walker.feed(piece);
/// }
/// let summary = walker.summary();
/// assert!(summary.began_with_fmt);
/// assert_eq!((summary.fmt_matched, summary.fmt_mismatched), (23, 0));
/// assert_eq!((summary.bad_bytes, summary.tail_bytes), (0, 0));
/// ```
#[derive(Clone, Debug)]
pub struct Walker {
    /// The record being assembled: `have` bytes of it are valid.
    buf: [u8; MAX_RECORD],
    have: usize,
    /// Length of the record being assembled, 0 until its three header bytes are in.
    need: usize,
    /// `None` until the first record is complete or the first byte is skipped.
    began_with_fmt: Option<bool>,
    summary: Summary,
}

impl Walker {
    /// A walker at the start of a stream.
    pub const fn new() -> Self {
        Self {
            buf: [0; MAX_RECORD],
            have: 0,
            need: 0,
            began_with_fmt: None,
            summary: Summary {
                bytes: 0,
                records: 0,
                types: [TypeStats {
                    count: 0,
                    first_time_us: 0,
                    last_time_us: 0,
                }; NUM_TYPES],
                bad_bytes: 0,
                tail_bytes: 0,
                began_with_fmt: false,
                fmt_matched: 0,
                fmt_mismatched: 0,
            },
        }
    }

    /// The result so far; `tail_bytes` is the incomplete record at the end of what was fed.
    pub fn summary(&self) -> Summary {
        Summary {
            tail_bytes: self.have,
            began_with_fmt: self.began_with_fmt == Some(true),
            ..self.summary
        }
    }

    /// Feeds the next bytes of the stream; any length, empty included.
    pub fn feed(&mut self, mut data: &[u8]) {
        self.summary.bytes = self.summary.bytes.saturating_add(data.len() as u64);
        while let Some((&first, after_first)) = data.split_first() {
            if self.need == 0 {
                data = after_first;
                self.header_byte(first);
                continue;
            }
            let take = (self.need - self.have).min(data.len());
            let Some((now, rest)) = data.split_at_checked(take) else {
                return;
            };
            data = rest;
            if let Some(dst) = self.buf.get_mut(self.have..self.have + take) {
                dst.copy_from_slice(now);
            }
            self.have += take;
            if self.have == self.need {
                self.complete();
            }
        }
    }

    /// The next byte of a header (the first three of a record).
    fn header_byte(&mut self, byte: u8) {
        if let Some(slot) = self.buf.get_mut(self.have) {
            *slot = byte;
        }
        self.have += 1;
        while self.have > 0 && !self.prefix_ok() {
            self.skip_first();
        }
        if self.have == HEADER_LEN {
            let id = self.buf.get(2).copied().unwrap_or(0);
            self.need = TYPES
                .iter()
                .find(|def| def.type_id == id)
                .map_or(0, |def| usize::from(def.length));
        }
    }

    /// Whether the bytes held are the start of a record: `A3`, `95`, a known type.
    fn prefix_ok(&self) -> bool {
        let want = [HEAD1, HEAD2];
        self.buf
            .iter()
            .take(self.have.min(2))
            .eq(want.iter().take(self.have))
            && (self.have < HEADER_LEN
                || self
                    .buf
                    .get(2)
                    .is_some_and(|&id| TYPES.iter().any(|def| def.type_id == id)))
    }

    /// Drops the first byte held as bad and keeps the rest (which may start a header).
    fn skip_first(&mut self) {
        self.summary.bad_bytes = self.summary.bad_bytes.saturating_add(1);
        if self.began_with_fmt.is_none() {
            self.began_with_fmt = Some(false);
        }
        if let Some(held) = self.buf.get_mut(..self.have) {
            held.rotate_left(1);
        }
        self.have -= 1;
    }

    /// A whole record is in `buf`.
    fn complete(&mut self) {
        let id = self.buf.get(2).copied().unwrap_or(0);
        let index = TYPES.iter().position(|def| def.type_id == id);
        let fmt_matches = id == ID_FMT && self.fmt_matches();
        if let Some(stats) = index.and_then(|i| self.summary.types.get_mut(i)) {
            stats.count = stats.count.saturating_add(1);
            if id != ID_FMT {
                let time = self
                    .buf
                    .get(HEADER_LEN..HEADER_LEN + 8)
                    .and_then(|b| <[u8; 8]>::try_from(b).ok())
                    .map_or(0, u64::from_le_bytes);
                if stats.count == 1 {
                    stats.first_time_us = time;
                }
                stats.last_time_us = time;
            }
        }
        if id == ID_FMT {
            let counter = if fmt_matches {
                &mut self.summary.fmt_matched
            } else {
                &mut self.summary.fmt_mismatched
            };
            *counter = counter.saturating_add(1);
        }
        if self.began_with_fmt.is_none() {
            let described = self.buf.get(HEADER_LEN).copied();
            self.began_with_fmt = Some(fmt_matches && described == Some(ID_FMT));
        }
        self.summary.records = self.summary.records.saturating_add(1);
        self.have = 0;
        self.need = 0;
    }

    /// The FMT record in `buf` against the one the table writes for the type it describes.
    fn fmt_matches(&self) -> bool {
        let Some(described) = self.buf.get(HEADER_LEN).copied() else {
            return false;
        };
        let Some(def) = TYPES.iter().find(|def| def.type_id == described) else {
            return false;
        };
        let mut want = [0u8; MAX_RECORD];
        let n = fmt_record(def, &mut want);
        n == usize::from(FMT.length) && want.get(..n) == self.buf.get(..n) && self.have == n
    }
}

impl Default for Walker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::super::*;
    use super::*;
    use std::vec;
    use std::vec::Vec;

    /// A stream and what is known of it by construction.
    struct Sample {
        bytes: Vec<u8>,
        /// Offset and type of every record, in order.
        records: Vec<(usize, u8)>,
        expected: [TypeStats; NUM_TYPES],
    }

    fn index_of(id: u8) -> usize {
        TYPES.iter().position(|d| d.type_id == id).unwrap()
    }

    impl Sample {
        fn push(&mut self, id: u8, time_us: u64, f: impl FnOnce(&mut [u8]) -> usize) {
            let mut b = [0u8; 128];
            let n = f(&mut b);
            assert_eq!(n, usize::from(TYPES[index_of(id)].length));
            self.records.push((self.bytes.len(), id));
            self.bytes.extend_from_slice(&b[..n]);
            let stats = &mut self.expected[index_of(id)];
            stats.count += 1;
            if id != ID_FMT {
                if stats.count == 1 {
                    stats.first_time_us = time_us;
                }
                stats.last_time_us = time_us;
            }
        }
    }

    /// The example's file: header, 25 PARM, 3 MSG, 10 s of data at the design's rates.
    fn sample() -> Sample {
        let mut s = Sample {
            bytes: Vec::new(),
            records: Vec::new(),
            expected: [TypeStats::default(); NUM_TYPES],
        };
        for def in &TYPES {
            s.push(ID_FMT, 0, |b| fmt_record(def, b));
        }
        for i in 0..25u64 {
            s.push(ID_PARM, 1_000 + i * 10, |b| {
                encode_parm(b, 1_000 + i * 10, b"PARAM", i as f32, 0.0)
            });
        }
        for i in 0..3u64 {
            s.push(ID_MSG, 2_000 + i * 100, |b| {
                encode_msg(b, 2_000 + i * 100, b"text")
            });
        }
        for ms in 0..10_000u64 {
            let t = 3_000_000 + ms * 1_000;
            if ms % 40 == 0 {
                for inst in 0..2u8 {
                    s.push(ID_IMU, t, |b| {
                        encode_imu(b, t, inst, [0.1; 3], [0.2, 0.1, -9.8], 31.0)
                    });
                }
            }
            if ms % 20 == 0 {
                s.push(ID_BARO, t, |b| encode_baro(b, t, 0, 101_325.0, 24.5));
            }
            if ms % 10 == 0 {
                s.push(ID_MAG, t, |b| encode_mag(b, t, 0, [250, -80, 420]));
            }
            if ms % 100 == 0 {
                s.push(ID_BAT, t, |b| encode_bat(b, t, 0, 12.4, 3.2));
                s.push(ID_RCIN, t, |b| encode_rcin(b, t, &[1500; 14]));
            }
            if ms % 1_000 == 0 {
                let g = Gps {
                    status: 3,
                    nsats: 14,
                    lat_e7: 411_234_567,
                    lng_e7: -86_543_210,
                    ..Gps::default()
                };
                s.push(ID_GPS, t, |b| encode_gps(b, t, &g));
                for id in 0..7u8 {
                    let d = Dev {
                        id,
                        state: 1,
                        ..Dev::default()
                    };
                    s.push(ID_DEV, t, |b| encode_dev(b, t, &d));
                }
                s.push(ID_DSF, t, |b| encode_dsf(b, t, &Dsf::default()));
            }
        }
        s
    }

    fn walk(data: &[u8], piece: usize) -> Summary {
        let mut walker = Walker::new();
        let mut calls = 0;
        for chunk in data.chunks(piece) {
            walker.feed(chunk);
            calls += 1;
            assert!(calls <= data.len() + 1, "bounded");
        }
        walker.summary()
    }

    fn expect_clean(s: &Sample, got: &Summary) {
        assert_eq!(got.types, s.expected);
        assert_eq!(got.records as usize, s.records.len());
        assert_eq!(got.bytes as usize, s.bytes.len());
        assert_eq!((got.bad_bytes, got.tail_bytes), (0, 0));
        assert!(got.began_with_fmt);
        assert_eq!((got.fmt_matched, got.fmt_mismatched), (23, 0));
    }

    #[test]
    fn max_record_is_the_fmt() {
        assert_eq!(MAX_RECORD, 89);
        assert_eq!(NUM_TYPES, 23);
    }

    #[test]
    fn the_whole_sample_reads_clean_with_the_right_counts_and_times() {
        let s = sample();
        let got = walk(&s.bytes, s.bytes.len());
        expect_clean(&s, &got);
        let count = |id| got.types[index_of(id)].count;
        assert_eq!(
            [
                ID_FMT, ID_PARM, ID_MSG, ID_IMU, ID_BARO, ID_MAG, ID_GPS, ID_BAT, ID_RCIN, ID_DEV,
                ID_DSF
            ]
            .map(count),
            [23, 25, 3, 500, 500, 1000, 10, 100, 100, 70, 10]
        );
        let imu = got.types[index_of(ID_IMU)];
        assert_eq!(
            (imu.first_time_us, imu.last_time_us),
            (3_000_000, 3_000_000 + 9_960_000)
        );
        let parm = got.types[index_of(ID_PARM)];
        assert_eq!((parm.first_time_us, parm.last_time_us), (1_000, 1_240));
    }

    #[test]
    fn pieces_of_any_size_give_the_same_result_as_the_whole() {
        let s = sample();
        let whole = walk(&s.bytes, s.bytes.len());
        expect_clean(&s, &whole);
        for piece in [1, 7, 512, 4096] {
            assert_eq!(walk(&s.bytes, piece), whole, "pieces of {piece}");
        }
        // Empty slices change nothing.
        let mut walker = Walker::new();
        for chunk in s.bytes.chunks(100) {
            walker.feed(&[]);
            walker.feed(chunk);
        }
        assert_eq!(walker.summary(), whole);
    }

    #[test]
    fn a_flipped_header_byte_costs_that_record_and_resynchronises() {
        let s = sample();
        let victim = 23 + 25 + 3 + 100;
        let (offset, id) = s.records[victim];
        let len = usize::from(TYPES[index_of(id)].length);
        for (byte, value) in [(0, 0xA2), (1, 0x94), (2, 0x77)] {
            let mut data = s.bytes.clone();
            data[offset + byte] = value;
            for piece in [1, 7, 4096] {
                let got = walk(&data, piece);
                assert_eq!(
                    got.bad_bytes as usize, len,
                    "byte {byte}, pieces of {piece}"
                );
                assert_eq!(got.records as usize, s.records.len() - 1);
                assert_eq!(got.tail_bytes, 0);
                assert!(got.began_with_fmt);
                let mut want = s.expected;
                want[index_of(id)].count -= 1;
                assert_eq!(got.types, want, "everything else is read");
                assert_eq!((got.fmt_matched, got.fmt_mismatched), (23, 0));
            }
        }
    }

    #[test]
    fn garbage_before_the_first_record_is_counted_and_skipped() {
        let s = sample();
        let mut data = vec![0x00, 0xA3, 0x12, 0x95, 0xA3, 0xA3];
        data.extend_from_slice(&s.bytes);
        let got = walk(&data, 5);
        assert_eq!(got.bad_bytes, 6);
        assert!(!got.began_with_fmt, "the stream did not begin with it");
        assert_eq!(got.types, s.expected);
        assert_eq!((got.fmt_matched, got.fmt_mismatched), (23, 0));
        assert_eq!(walk(&[7; 100], 9).bad_bytes, 100);
    }

    #[test]
    fn a_stream_cut_in_the_middle_of_a_record_reports_the_tail_not_bad_bytes() {
        let s = sample();
        let mid = 23 + 25 + 3 + 777;
        let (offset, id) = s.records[mid];
        let len = usize::from(TYPES[index_of(id)].length);
        for tail in 0..len {
            let cut = &s.bytes[..offset + tail];
            for piece in [1, 512] {
                let got = walk(cut, piece);
                assert_eq!(
                    got.tail_bytes, tail,
                    "cut {tail} bytes into a {len}-byte record"
                );
                assert_eq!(got.bad_bytes, 0);
                assert_eq!(got.records as usize, mid);
                assert!(got.began_with_fmt);
            }
        }
        // Cut inside the header of the first record, and an empty stream.
        assert_eq!(walk(&s.bytes[..2], 1).tail_bytes, 2);
        assert_eq!(walk(&s.bytes[..2], 1).bad_bytes, 0);
        let empty = walk(&[], 1);
        assert_eq!((empty.bytes, empty.records, empty.tail_bytes), (0, 0, 0));
        assert!(!empty.began_with_fmt);
        // The whole file ends at a record boundary: no tail.
        assert_eq!(walk(&s.bytes, 4096).tail_bytes, 0);
    }

    #[test]
    fn an_unknown_type_record_is_skipped_as_bad_bytes() {
        let s = sample();
        let at = s.records[23 + 25 + 3 + 50].0;
        let mut data = s.bytes[..at].to_vec();
        data.extend_from_slice(&[0xA3, 0x95, 0x77]);
        data.extend_from_slice(&[0x11; 10]);
        data.extend_from_slice(&s.bytes[at..]);
        for piece in [1, 7, 512] {
            let got = walk(&data, piece);
            assert_eq!(got.bad_bytes, 13);
            assert_eq!(got.types, s.expected, "every real record is read");
            assert_eq!(got.records as usize, s.records.len());
            assert_eq!(got.tail_bytes, 0);
        }
    }

    #[test]
    fn a_header_prefix_inside_garbage_is_not_lost() {
        // A3 95 A3 95 83 ...: the first pair is not followed by a type, the second is a header.
        let mut data = vec![0xA3, 0x95];
        let mut b = [0u8; 128];
        let n = fmt_record(&FMT, &mut b);
        data.extend_from_slice(&b[..n]);
        let got = walk(&data, 1);
        assert_eq!(got.bad_bytes, 2);
        assert_eq!(got.types[0].count, 1);
        assert_eq!(got.fmt_matched, 1);
    }

    #[test]
    fn fmt_records_that_differ_from_the_table_are_counted_apart() {
        let s = sample();
        // One column letter of BARO's FMT changed: matched 22, mismatched 1, nothing skipped.
        let (offset, _) = s.records[index_of(ID_BARO)];
        let mut data = s.bytes.clone();
        data[offset + 9] = b'Z';
        let got = walk(&data, 64);
        assert_eq!((got.fmt_matched, got.fmt_mismatched), (22, 1));
        assert_eq!((got.bad_bytes, got.tail_bytes), (0, 0));
        assert!(got.began_with_fmt);
        assert_eq!(got.types, s.expected);
        // An FMT of a type the table does not have.
        let mut data = s.bytes.clone();
        data[offset + 3] = 200;
        assert_eq!(walk(&data, 64).fmt_mismatched, 1);
        // The FMT of FMT altered: the stream did not begin with it.
        let mut data = s.bytes.clone();
        data[9] = b'C';
        let got = walk(&data, 64);
        assert!(!got.began_with_fmt);
        assert_eq!((got.fmt_matched, got.fmt_mismatched), (22, 1));
    }

    #[test]
    fn a_stream_that_starts_with_another_record_did_not_begin_with_the_fmt_of_fmt() {
        let s = sample();
        let (offset, _) = s.records[23];
        let got = walk(&s.bytes[offset..], 512);
        assert!(!got.began_with_fmt);
        assert_eq!(got.bad_bytes, 0);
        // An FMT of another type first is not the FMT of FMT either.
        let (second, _) = s.records[1];
        assert!(!walk(&s.bytes[second..], 512).began_with_fmt);
    }

    #[test]
    fn one_record_of_each_of_the_23_types_reads_clean() {
        let mut s = Sample {
            bytes: Vec::new(),
            records: Vec::new(),
            expected: [TypeStats::default(); NUM_TYPES],
        };
        for def in &TYPES {
            s.push(ID_FMT, 0, |b| fmt_record(def, b));
        }
        s.push(ID_PARM, 1, |b| encode_parm(b, 1, b"P", 1.0, 2.0));
        s.push(ID_MSG, 2, |b| encode_msg(b, 2, b"m"));
        s.push(ID_IMU, 3, |b| encode_imu(b, 3, 0, [0.0; 3], [0.0; 3], 0.0));
        s.push(ID_BARO, 4, |b| encode_baro(b, 4, 0, 1.0, 2.0));
        s.push(ID_MAG, 5, |b| encode_mag(b, 5, 0, [1, 2, 3]));
        s.push(ID_GPS, 6, |b| encode_gps(b, 6, &Gps::default()));
        s.push(ID_BAT, 7, |b| encode_bat(b, 7, 0, 1.0, 2.0));
        s.push(ID_RCIN, 8, |b| encode_rcin(b, 8, &[1500; 14]));
        s.push(ID_DEV, 9, |b| encode_dev(b, 9, &Dev::default()));
        s.push(ID_DSF, 10, |b| encode_dsf(b, 10, &Dsf::default()));
        s.push(ID_NAV, 11, |b| encode_nav(b, 11, [1.0; 4], [0.0; 3], 1));
        s.push(ID_SETP, 12, |b| {
            encode_setp(b, 12, 1, [1.0; 4], [0.0; 3], 1.0)
        });
        s.push(ID_DMD, 13, |b| encode_dmd(b, 13, true, 1.0, [0.0; 3], 0));
        s.push(ID_MOT, 14, |b| encode_mot(b, 14, true, 1, [1; 4], [2; 4]));
        s.push(ID_ESC, 15, |b| encode_esc(b, 15, [1; 4], [2; 4], [3; 4], 5));
        s.push(ID_RCS, 16, |b| encode_rcs(b, 16, [1.0; 4], true));
        s.push(ID_FDS, 17, |b| encode_fds(b, 17, 1, 2));
        s.push(ID_GPA, 18, |b| {
            encode_gpa(b, 18, 150, Some(1), Some(2), Some(3), 4)
        });
        s.push(ID_GPV, 19, |b| encode_gpv(b, 19, [1, -2, 3], 4, 5, 6));
        s.push(ID_NKF1, 20, |b| {
            encode_nkf1(b, 20, 1, [1.0; 4], [0.0; 3], [0.0; 3], 1)
        });
        s.push(ID_NKF2, 21, |b| encode_nkf2(b, 21, 1, [0.0; 3], [0.0; 3], [1.0; 6]));
        s.push(ID_NKI, 22, |b| encode_nki(b, 22, 1, 4, 0.5, 0.25, 1));
        let got = walk(&s.bytes, s.bytes.len());
        expect_clean(&s, &got);
        let counts: Vec<u32> = got.types.iter().map(|t| t.count).collect();
        let mut want = vec![1u32; 23];
        want[0] = 23;
        assert_eq!(counts, want);
        assert_eq!(got.records, 45);
    }
}
