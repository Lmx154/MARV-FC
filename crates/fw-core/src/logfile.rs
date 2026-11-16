//! Log file rules of the SD logger, pure: the numbers, the names, `LASTLOG.TXT` and the clean-up
//! plan (design: decision-sd-logging, Q4 "Files" and "Free space", with the lead's amendment of
//! facts-sd-logging C1).
//!
//! Logs are `LOGS/NNNNNNNN.BIN`, numbered 1 to [`MAX_LOG_NUMBER`] (500) and then 1 again (the
//! file that held the number is deleted first). `LOGS/LASTLOG.TXT` holds the last number used, in
//! decimal. The order of a new log's start is:
//!
//! 1. [`parse_lastlog`] the file (if it exists), take the largest [`parse_file_name`] number of the
//!    `LOGS/` listing, and [`last_number`] gives the last number used (0 if nothing).
//! 2. [`next_number`] gives the number of the new log.
//! 3. `LASTLOG.TXT` is written with [`format_lastlog`] BEFORE the log is opened (Q4-I3).
//! 4. [`plan_cleanup`] is called with the listing, one deletion per call, until it says
//!    [`CleanUp::Nothing`] (go on) or [`CleanUp::Full`] (no log: "SD: card full"). The firmware
//!    never asks the file system for free space (a full FAT scan takes 67 s after a power cut): the
//!    space used is the sum of the sizes of the log files.

/// The largest log number: numbering goes 1 to this, then 1 again.
pub const MAX_LOG_NUMBER: u32 = 500;

/// Length of a log's file name, `NNNNNNNN.BIN`.
pub const FILE_NAME_LEN: usize = 12;

/// The directory of the logs.
pub const LOG_DIR: &str = "LOGS";

/// The name of the file that holds the last log number.
pub const LASTLOG_NAME: &str = "LASTLOG.TXT";

/// Space that must stay free when a log starts: 500 MiB.
pub const RESERVE_BYTES: u64 = 500 * 1024 * 1024;

/// Free space below which there is no log ("card full"): 8 MiB.
pub const FULL_BYTES: u64 = 8 * 1024 * 1024;

/// The number of the log after `last`: `last + 1`, and 1 after 500. `None`, 0 and any number
/// above 500 (not a log number) give 1.
pub const fn next_number(last: Option<u32>) -> u32 {
    match last {
        Some(n @ 1..MAX_LOG_NUMBER) => n + 1,
        _ => 1,
    }
}

/// The decimal digit of `n` worth `scale` (1, 10, 100, ...), as ASCII.
const fn digit(n: u32, scale: u32) -> u8 {
    b'0' + (n / scale % 10) as u8
}

/// The file name of log `n`: 8 decimal digits, zero padded, then `.BIN` (`00000007.BIN`). A number
/// of more than 8 digits keeps its low 8 (log numbers never exceed 500).
pub const fn file_name(n: u32) -> [u8; FILE_NAME_LEN] {
    let n = n % 100_000_000;
    [
        digit(n, 10_000_000),
        digit(n, 1_000_000),
        digit(n, 100_000),
        digit(n, 10_000),
        digit(n, 1_000),
        digit(n, 100),
        digit(n, 10),
        digit(n, 1),
        b'.',
        b'B',
        b'I',
        b'N',
    ]
}

/// The log number a file name carries: exactly 8 ASCII digits, a dot and `BIN` in any case
/// (an 8.3 name; no spaces, no long name), with a number from 1 to 500. Anything else
/// (`P3TEST.BIN`, `LASTLOG.TXT`, `00000000.BIN`, `00000501.BIN`) is not a log: `None`.
pub fn parse_file_name(name: &[u8]) -> Option<u32> {
    let [d @ .., b'.', e1, e2, e3] = name else {
        return None;
    };
    if d.len() != 8
        || !(e1.eq_ignore_ascii_case(&b'B')
            && e2.eq_ignore_ascii_case(&b'I')
            && e3.eq_ignore_ascii_case(&b'N'))
    {
        return None;
    }
    let mut n = 0u32;
    for &c in d {
        if !c.is_ascii_digit() {
            return None;
        }
        n = n * 10 + u32::from(c - b'0');
    }
    (1..=MAX_LOG_NUMBER).contains(&n).then_some(n)
}

/// Writes the content of `LASTLOG.TXT` for log `n` into `out`: the decimal digits, then CR LF
/// (as ArduPilot writes it). Returns the length (3 to 12), or 0, with nothing written, if `out`
/// is too short.
pub fn format_lastlog(n: u32, out: &mut [u8]) -> usize {
    let mut digits = [0u8; 10];
    let mut count = 0;
    let mut rest = n;
    loop {
        if let Some(slot) = digits.get_mut(count) {
            *slot = b'0' + (rest % 10) as u8;
        }
        count += 1;
        rest /= 10;
        if rest == 0 {
            break;
        }
    }
    let total = count + 2;
    let Some(dst) = out.get_mut(..total) else {
        return 0;
    };
    for (slot, &digit) in dst.iter_mut().zip(digits.iter().take(count).rev()) {
        *slot = digit;
    }
    if let Some([cr, lf]) = dst.get_mut(count..) {
        *cr = b'\r';
        *lf = b'\n';
    }
    total
}

/// The log number in the content of `LASTLOG.TXT`: one or more decimal digits, then anything
/// of spaces, tabs, CR, LF, NUL and `D` (ArduPilot may append one). Empty, garbage (any other
/// byte, including before the digits), 0 and numbers above 500 give `None`.
pub fn parse_lastlog(content: &[u8]) -> Option<u32> {
    let digits = content
        .iter()
        .position(|c| !c.is_ascii_digit())
        .unwrap_or(content.len());
    let (number, rest) = content.split_at(digits);
    if number.is_empty()
        || !rest
            .iter()
            .all(|c| matches!(c, b' ' | b'\t' | b'\r' | b'\n' | 0 | b'D'))
    {
        return None;
    }
    let mut n = 0u32;
    for &c in number {
        n = n * 10 + u32::from(c - b'0');
        if n > MAX_LOG_NUMBER {
            return None;
        }
    }
    (n >= 1).then_some(n)
}

/// The last log number used: the number `LASTLOG.TXT` holds if it parsed (`lastlog`, from
/// [`parse_lastlog`]), else the largest log number of the `LOGS/` listing (`listing_max`, from
/// [`parse_file_name`]), else 0. A value that is not a log number (0 or above 500) counts as
/// absent.
pub fn last_number(lastlog: Option<u32>, listing_max: Option<u32>) -> u32 {
    let valid = |n: &u32| (1..=MAX_LOG_NUMBER).contains(n);
    lastlog
        .filter(valid)
        .or_else(|| listing_max.filter(valid))
        .unwrap_or(0)
}

/// What the clean-up planner decided.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CleanUp {
    /// Enough space: open the log.
    Nothing,
    /// Delete this log (`LOGS/NNNNNNNN.BIN`), then ask again with the new listing.
    Delete(u32),
    /// No log: even with every other log deleted less than [`FULL_BYTES`] would be free ("SD:
    /// card full").
    Full,
}

/// Plans the clean-up before log `current` is opened, ONE step per call (so the firmware can
/// delete between chunks and re-list).
///
/// `logs` is the `LOGS/` listing as `(number, size in bytes)`, numbers from [`parse_file_name`]
/// (each file once; an entry with a number outside 1 to 500 is ignored). The space used is the
/// sum of the sizes of all of them, `current` included if it is listed (an old file of that
/// number that the new log replaces: delete it first, or leave it out of the listing, if it
/// should not count). `volume_bytes` is the size of the partition.
///
/// - [`CleanUp::Full`] if `volume_bytes` minus the size of `current` (listed or not) is under
///   [`FULL_BYTES`]: deleting every other log would not help.
/// - [`CleanUp::Nothing`] if the used space is at most `volume_bytes` minus `reserve_bytes`
///   ([`RESERVE_BYTES`]; a reserve under [`FULL_BYTES`] counts as [`FULL_BYTES`]), or if it is
///   more but only `current` is listed.
/// - else [`CleanUp::Delete`] of the OLDEST log other than `current`: ring order, the first
///   number after `current` going up, wrapping from 500 to 1 (after writing 7 the oldest is 8,
///   then 9, ... 500, 1, ... 6). `current` outside 1 to 500 (0 for "none yet") counts as 0, so
///   the oldest is the lowest number.
///
/// One pass over `logs`, no allocation.
pub fn plan_cleanup(
    volume_bytes: u64,
    reserve_bytes: u64,
    current: u32,
    logs: impl IntoIterator<Item = (u32, u64)>,
) -> CleanUp {
    let current = if (1..=MAX_LOG_NUMBER).contains(&current) {
        current
    } else {
        0
    };
    let mut used = 0u64;
    let mut current_size = 0u64;
    let mut oldest: Option<(u32, u32)> = None;
    for (number, size) in logs {
        if !(1..=MAX_LOG_NUMBER).contains(&number) {
            continue;
        }
        used = used.saturating_add(size);
        if number == current {
            current_size = current_size.saturating_add(size);
            continue;
        }
        let age_rank = (number + MAX_LOG_NUMBER - 1 - current) % MAX_LOG_NUMBER;
        if oldest.is_none_or(|(rank, _)| age_rank < rank) {
            oldest = Some((age_rank, number));
        }
    }
    if volume_bytes.saturating_sub(current_size) < FULL_BYTES {
        return CleanUp::Full;
    }
    let limit = volume_bytes.saturating_sub(reserve_bytes.max(FULL_BYTES));
    if used <= limit {
        return CleanUp::Nothing;
    }
    match oldest {
        Some((_, number)) => CleanUp::Delete(number),
        None => CleanUp::Nothing,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    const MIB: u64 = 1024 * 1024;
    const GIB: u64 = 1024 * MIB;
    /// A 64 GB card's partition, about 59.5 GiB.
    const VOLUME: u64 = 124_735_488 * 512;

    #[test]
    fn next_number_counts_up_and_wraps_500_to_1() {
        assert_eq!(next_number(None), 1);
        assert_eq!(next_number(Some(0)), 1);
        assert_eq!(next_number(Some(1)), 2);
        assert_eq!(next_number(Some(499)), 500);
        assert_eq!(next_number(Some(500)), 1);
        assert_eq!(next_number(Some(501)), 1);
        assert_eq!(next_number(Some(u32::MAX)), 1);
        let mut n = 0;
        for _ in 0..1_000 {
            n = next_number(Some(n));
            assert!((1..=500).contains(&n));
        }
    }

    #[test]
    fn file_names() {
        assert_eq!(&file_name(1), b"00000001.BIN");
        assert_eq!(&file_name(500), b"00000500.BIN");
        assert_eq!(&file_name(0), b"00000000.BIN");
        assert_eq!(&file_name(12_345_678), b"12345678.BIN");
        assert_eq!(&file_name(123_456_789), b"23456789.BIN");
        for n in 1..=500 {
            assert_eq!(parse_file_name(&file_name(n)), Some(n));
        }
    }

    #[test]
    fn parse_file_name_accepts_exactly_the_8_3_shape() {
        assert_eq!(parse_file_name(b"00000001.BIN"), Some(1));
        assert_eq!(parse_file_name(b"00000500.BIN"), Some(500));
        assert_eq!(parse_file_name(b"00000042.bin"), Some(42));
        assert_eq!(parse_file_name(b"00000042.Bin"), Some(42));
        for bad in [
            &b""[..],
            b"P3TEST.BIN",
            b"LASTLOG.TXT",
            b"00000000.BIN",
            b"00000501.BIN",
            b"99999999.BIN",
            b"0000001.BIN",
            b"000000001.BIN",
            b"00000001.BI",
            b"00000001.BINX",
            b"00000001.TXT",
            b"00000001 BIN",
            b"0000000A.BIN",
            b"0000 001.BIN",
            b"-0000001.BIN",
            b"+0000001.BIN",
            b"00000001.BIN\0",
            b"\xFF0000001.BIN",
            b".BIN",
        ] {
            assert_eq!(parse_file_name(bad), None, "{bad:?}");
        }
    }

    #[test]
    fn lastlog_format() {
        let mut out = [0xEEu8; 16];
        assert_eq!(format_lastlog(7, &mut out), 3);
        assert_eq!(&out[..3], b"7\r\n");
        assert_eq!(out[3], 0xEE, "nothing beyond is touched");
        assert_eq!(format_lastlog(500, &mut out), 5);
        assert_eq!(&out[..5], b"500\r\n");
        assert_eq!(format_lastlog(0, &mut out), 3);
        assert_eq!(&out[..3], b"0\r\n");
        assert_eq!(format_lastlog(u32::MAX, &mut out), 12);
        assert_eq!(&out[..12], b"4294967295\r\n");
        let mut small = [0xEEu8; 4];
        assert_eq!(format_lastlog(500, &mut small), 0);
        assert_eq!(small, [0xEE; 4], "too short: nothing written");
        assert_eq!(format_lastlog(7, &mut small[..3]), 3);
        assert_eq!(format_lastlog(7, &mut small[..2]), 0);
        assert_eq!(format_lastlog(7, &mut []), 0);
    }

    #[test]
    fn lastlog_round_trip() {
        let mut out = [0u8; 12];
        for n in 1..=500 {
            let len = format_lastlog(n, &mut out);
            assert_eq!(parse_lastlog(&out[..len]), Some(n));
        }
    }

    /// Q4-I3: missing, garbage, 500 (and the wrap after it).
    #[test]
    fn q4_i3_lastlog_parse_and_the_number_rule() {
        assert_eq!(parse_lastlog(b"7"), Some(7));
        assert_eq!(parse_lastlog(b"7\n"), Some(7));
        assert_eq!(parse_lastlog(b"7\r\n"), Some(7));
        assert_eq!(parse_lastlog(b"500\r\n"), Some(500));
        assert_eq!(parse_lastlog(b"12D"), Some(12));
        assert_eq!(parse_lastlog(b"12 D\r\n"), Some(12));
        assert_eq!(parse_lastlog(b"0012"), Some(12));
        assert_eq!(parse_lastlog(b"3\0\0\0"), Some(3));
        for bad in [
            &b""[..],
            b"\r\n",
            b"0",
            b"0\r\n",
            b"501",
            b"99999999999999999999",
            b"4294967297",
            b"abc",
            b"7x",
            b"x7",
            b" 7",
            b"-7",
            b"7 8",
            b"7.0",
            b"\xFF\xFE",
            b"D",
        ] {
            assert_eq!(parse_lastlog(bad), None, "{bad:?}");
        }
        // The rule: LASTLOG.TXT, else the largest number of the listing, else 0.
        assert_eq!(last_number(Some(7), Some(20)), 7);
        assert_eq!(last_number(None, Some(20)), 20, "missing or garbage file");
        assert_eq!(last_number(parse_lastlog(b"garbage"), Some(20)), 20);
        assert_eq!(last_number(None, None), 0);
        assert_eq!(last_number(Some(0), None), 0);
        assert_eq!(last_number(Some(501), Some(9)), 9);
        assert_eq!(last_number(Some(500), Some(20)), 500);
        // 500 wraps to 1.
        assert_eq!(next_number(Some(last_number(Some(500), None))), 1);
        assert_eq!(next_number(Some(last_number(None, None))), 1);
        assert_eq!(next_number(Some(last_number(None, Some(499)))), 500);
    }

    fn plan(current: u32, logs: &[(u32, u64)]) -> CleanUp {
        plan_cleanup(VOLUME, RESERVE_BYTES, current, logs.iter().copied())
    }

    #[test]
    fn nothing_to_delete_when_within_the_limit() {
        assert_eq!(plan(1, &[]), CleanUp::Nothing);
        assert_eq!(plan(3, &[(1, GIB), (2, GIB)]), CleanUp::Nothing);
        let limit = VOLUME - RESERVE_BYTES;
        assert_eq!(
            plan(3, &[(1, limit - 5), (2, 5)]),
            CleanUp::Nothing,
            "at the limit"
        );
        assert_eq!(
            plan(3, &[(1, limit - 5), (2, 6)]),
            CleanUp::Delete(1),
            "one byte over"
        );
    }

    #[test]
    fn deletes_the_oldest_in_ring_order_with_wrap_around() {
        let big = 20 * GIB;
        let logs = [(5, big), (6, big), (498, big), (500, big), (1, big)];
        // Current 6 (a lap later): 7.. up to 500, then 1.. : the first present is 498.
        assert_eq!(plan(6, &logs), CleanUp::Delete(498));
        assert_eq!(plan(3, &logs), CleanUp::Delete(5));
        assert_eq!(plan(499, &logs), CleanUp::Delete(500));
        assert_eq!(plan(500, &logs), CleanUp::Delete(1));
        assert_eq!(plan(498, &logs), CleanUp::Delete(500));
        assert_eq!(plan(1, &logs), CleanUp::Delete(5));
        assert_eq!(plan(0, &logs), CleanUp::Delete(1), "no current: the lowest");
        assert_eq!(
            plan(900, &logs),
            CleanUp::Delete(1),
            "not a log number: as none"
        );
        // The listing order does not matter.
        let mut shuffled = logs;
        shuffled.reverse();
        assert_eq!(plan(6, &shuffled), CleanUp::Delete(498));
    }

    #[test]
    fn deleting_step_by_step_ends_nothing() {
        // 40 logs of 2 GiB, numbered 481 to 500 and 1 to 20, current 21; limit about 59 GiB.
        let mut logs: Vec<(u32, u64)> = (481..=500).chain(1..=20).map(|n| (n, 2 * GIB)).collect();
        let mut deleted = Vec::new();
        for _ in 0..100 {
            match plan_cleanup(VOLUME, RESERVE_BYTES, 21, logs.iter().copied()) {
                CleanUp::Delete(n) => {
                    deleted.push(n);
                    logs.retain(|&(m, _)| m != n);
                }
                CleanUp::Nothing => break,
                CleanUp::Full => panic!("not full"),
            }
        }
        // 40 x 2 GiB = 80 GiB over 59.0 GiB: 11 deleted (the oldest after 21 first: 481...).
        assert_eq!(deleted, (481..=491).collect::<Vec<_>>());
        assert_eq!(logs.len(), 29);
        assert_eq!(
            plan_cleanup(VOLUME, RESERVE_BYTES, 21, logs.iter().copied()),
            CleanUp::Nothing
        );
    }

    #[test]
    fn a_single_huge_log() {
        let huge = VOLUME - 100 * MIB;
        // Not the current one: delete it.
        assert_eq!(plan(2, &[(1, huge)]), CleanUp::Delete(1));
        // The current one (being written, or the old file of that number): never chosen, and
        // there is room beyond the 8 MiB: go on.
        assert_eq!(plan(1, &[(1, huge)]), CleanUp::Nothing);
        // The current one fills the card: full.
        assert_eq!(plan(1, &[(1, VOLUME - 7 * MIB)]), CleanUp::Full);
        assert_eq!(
            plan(1, &[(1, VOLUME - 8 * MIB)]),
            CleanUp::Nothing,
            "exactly 8 MiB"
        );
        assert_eq!(plan(1, &[(1, VOLUME - 8 * MIB + 1)]), CleanUp::Full);
        assert_eq!(
            plan(2, &[(1, VOLUME - 7 * MIB)]),
            CleanUp::Delete(1),
            "deleting helps"
        );
    }

    #[test]
    fn the_current_log_among_the_listing_counts_but_is_never_deleted() {
        let limit = VOLUME - RESERVE_BYTES;
        // Used = the current one plus another one: over the limit, the other one goes.
        assert_eq!(plan(7, &[(7, limit), (8, 1)]), CleanUp::Delete(8));
        assert_eq!(plan(7, &[(8, 1), (7, limit)]), CleanUp::Delete(8));
        // Only the current one, over the limit: nothing more can be deleted.
        assert_eq!(plan(7, &[(7, limit + 1)]), CleanUp::Nothing);
        // Never 7, whatever the other numbers: 6 is the newest, so last.
        assert_eq!(plan(7, &[(7, limit), (6, 1), (8, 1)]), CleanUp::Delete(8));
        assert_eq!(plan(7, &[(7, limit), (6, 1)]), CleanUp::Delete(6));
    }

    #[test]
    fn full_when_even_deleting_everything_would_not_help() {
        assert_eq!(plan_cleanup(7 * MIB, RESERVE_BYTES, 1, []), CleanUp::Full);
        assert_eq!(
            plan_cleanup(8 * MIB, RESERVE_BYTES, 1, []),
            CleanUp::Nothing
        );
        assert_eq!(plan_cleanup(0, RESERVE_BYTES, 1, []), CleanUp::Full);
        // A volume under the reserve but over 8 MiB: delete everything else, then go on.
        let small = 300 * MIB;
        assert_eq!(
            plan_cleanup(small, RESERVE_BYTES, 3, [(1, MIB), (2, MIB)]),
            CleanUp::Delete(1)
        );
        assert_eq!(
            plan_cleanup(small, RESERVE_BYTES, 3, [(3, MIB)]),
            CleanUp::Nothing
        );
        // A reserve under 8 MiB counts as 8 MiB.
        assert_eq!(
            plan_cleanup(100 * MIB, 0, 3, [(1, 100 * MIB - 8 * MIB + 1)]),
            CleanUp::Delete(1)
        );
        assert_eq!(
            plan_cleanup(100 * MIB, 0, 3, [(1, 100 * MIB - 8 * MIB)]),
            CleanUp::Nothing
        );
    }

    #[test]
    fn entries_that_are_not_logs_are_ignored() {
        assert_eq!(
            plan(3, &[(0, 100 * GIB), (501, 100 * GIB)]),
            CleanUp::Nothing
        );
        assert_eq!(
            plan(3, &[(0, 100 * GIB), (501, 100 * GIB), (9, 100 * GIB)]),
            CleanUp::Delete(9)
        );
    }

    #[test]
    fn five_hundred_logs_in_one_pass() {
        let logs: Vec<(u32, u64)> = (1..=500).map(|n| (n, 200 * MIB)).collect();
        // 100 000 MiB over a 59.5 GiB volume.
        assert_eq!(plan(250, &logs), CleanUp::Delete(251));
        assert_eq!(plan(500, &logs), CleanUp::Delete(1));
        assert_eq!(plan(1, &logs), CleanUp::Delete(2));
    }
}
