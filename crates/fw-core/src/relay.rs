//! The relay of the flight log ring into the main log ring.
//!
//! [`relay`] is the pure body of core 0's collector tick: it walks the readable records of the flight ring,
//! takes each length from [`dflog::TYPES`] by type id and copies the record into the destination ring.
//! A record the destination has no room for is dropped (counted there) but still consumed from the source,
//! so a stalled card never backs up into the flight core; with no destination everything is consumed.
//! Bytes that are not a record header (`A3 95 <type>`) are skipped one at a time ([`Relayed::skipped`]);
//! a record cut short by the readable end is skipped with the rest. Only bytes readable at the call start are walked.

use crate::dflog::{self, HEAD1, HEAD2, HEADER_LEN};
use crate::xcore::{LogConsumer, LogProducer};

/// What one call of [`relay`] did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Relayed {
    /// Records copied into the destination.
    pub copied: u32,
    /// Records the destination had no room for: dropped (counted by the destination), consumed.
    pub dropped: u32,
    /// Records consumed with no destination.
    pub discarded: u32,
    /// Bytes skipped because they were not a record header, or a record cut short.
    pub skipped: u32,
}

/// Moves the records readable in `src` into `dst`; see the module documentation.
pub fn relay<const S: usize, const D: usize>(
    src: &mut LogConsumer<'_, S>,
    dst: Option<&mut LogProducer<'_, D>>,
) -> Relayed {
    relay_with(src, dst.map(|ring| |record: &[u8]| ring.push(record)))
}

/// [`relay`] into any destination: `push` takes one whole record and says whether it fitted
/// (`false`: dropped, and the destination counts it). The board's collector relays into an
/// [`ExtLogRing`](crate::extring::ExtLogRing) with it; `None` consumes and discards.
pub fn relay_with<const S: usize, F: FnMut(&[u8]) -> bool>(
    src: &mut LogConsumer<'_, S>,
    mut push: Option<F>,
) -> Relayed {
    let mut out = Relayed::default();
    let mut left = src.readable();
    let mut scratch = [0u8; 255];
    while left > 0 {
        let (a, b) = src.peek_up_to(left);
        let byte = |i: usize| a.get(i).or_else(|| b.get(i - a.len())).copied();
        let len = match (byte(0), byte(1), byte(2)) {
            (Some(HEAD1), Some(HEAD2), Some(id)) => dflog::TYPES
                .iter()
                .find(|def| def.type_id == id)
                .map(|def| usize::from(def.length)),
            _ => None,
        };
        let Some(len) = len else {
            src.consume(1);
            out.skipped = out.skipped.saturating_add(1);
            left -= 1;
            continue;
        };
        if len > left || len < HEADER_LEN {
            src.consume(left);
            out.skipped = out.skipped.saturating_add(left as u32);
            break;
        }
        let first = len.min(a.len());
        if let (Some(to), Some(from)) = (scratch.get_mut(..first), a.get(..first)) {
            to.copy_from_slice(from);
        }
        if let (Some(to), Some(from)) = (scratch.get_mut(first..len), b.get(..len - first)) {
            to.copy_from_slice(from);
        }
        match push.as_mut() {
            None => out.discarded += 1,
            Some(push) => match scratch.get(..len).is_some_and(push) {
                true => out.copied += 1,
                false => out.dropped += 1,
            },
        }
        src.consume(len);
        left -= len;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xcore::LogRing;
    use std::vec::Vec;

    /// A record of type `id` whose `TimeUS` is `t` and whose other bytes are `t as u8`.
    fn record(id: u8, t: u64) -> Vec<u8> {
        let len = dflog::TYPES
            .iter()
            .find(|d| d.type_id == id)
            .unwrap()
            .length;
        let mut v = std::vec![t as u8; usize::from(len)];
        v[..3].copy_from_slice(&[HEAD1, HEAD2, id]);
        v[3..11].copy_from_slice(&t.to_le_bytes());
        v
    }

    fn drain<const N: usize>(c: &mut LogConsumer<'_, N>) -> Vec<u8> {
        let (a, b) = c.peek();
        let all = [a, b].concat();
        c.consume(all.len());
        all
    }

    /// The mix used by the tests: ESC (40), RCS (28), FDS (19).
    fn mix(n: u64) -> Vec<Vec<u8>> {
        (0..n)
            .map(|t| {
                record(
                    [dflog::ID_ESC, dflog::ID_RCS, dflog::ID_FDS][t as usize % 3],
                    t,
                )
            })
            .collect()
    }

    #[test]
    fn records_arrive_whole_and_in_order_also_across_the_wrap() {
        let src_ring: LogRing<256> = LogRing::new();
        let dst_ring: LogRing<1024> = LogRing::new();
        let (mut sp, mut sc) = (
            src_ring.take_producer().unwrap(),
            src_ring.take_consumer().unwrap(),
        );
        let (mut dp, mut dc) = (
            dst_ring.take_producer().unwrap(),
            dst_ring.take_consumer().unwrap(),
        );
        let mut want = Vec::new();
        let mut t = 0;
        // Rounds that leave the source's indexes at every offset: the records wrap around its end.
        for round in 0..20u64 {
            for rec in mix(3 + round % 4) {
                let rec = record(rec[2], t);
                t += 1;
                assert!(sp.push(&rec));
                want.extend_from_slice(&rec);
            }
            let got = relay(&mut sc, Some(&mut dp));
            assert_eq!((got.dropped, got.skipped, got.discarded), (0, 0, 0));
            assert_eq!(sc.readable(), 0);
            assert_eq!(drain(&mut dc), want);
            want.clear();
        }
        assert_eq!(src_ring.dropped() + dst_ring.dropped(), 0);
    }

    #[test]
    fn a_full_destination_counts_each_dropped_record_and_still_consumes_the_source() {
        let src_ring: LogRing<256> = LogRing::new();
        let dst_ring: LogRing<64> = LogRing::new();
        let (mut sp, mut sc) = (
            src_ring.take_producer().unwrap(),
            src_ring.take_consumer().unwrap(),
        );
        let (mut dp, mut dc) = (
            dst_ring.take_producer().unwrap(),
            dst_ring.take_consumer().unwrap(),
        );
        // 5 RCS of 28 bytes: two fit in 64, three do not.
        let recs: Vec<Vec<u8>> = (0..5).map(|t| record(dflog::ID_RCS, t)).collect();
        for r in &recs {
            assert!(sp.push(r));
        }
        let got = relay(&mut sc, Some(&mut dp));
        assert_eq!((got.copied, got.dropped), (2, 3));
        assert_eq!(dst_ring.dropped(), 3);
        assert_eq!(
            sc.readable(),
            0,
            "the source is consumed whatever the destination did"
        );
        assert_eq!(drain(&mut dc), [recs[0].clone(), recs[1].clone()].concat());
        // The next tick: room again, the source's later records get through; no backlog.
        assert!(sp.push(&record(dflog::ID_FDS, 9)));
        let got = relay(&mut sc, Some(&mut dp));
        assert_eq!((got.copied, got.dropped), (1, 0));
        assert_eq!(drain(&mut dc), record(dflog::ID_FDS, 9));
        assert_eq!(src_ring.dropped(), 0);
    }

    #[test]
    fn with_no_log_open_everything_is_consumed_and_nothing_pushed() {
        let src_ring: LogRing<256> = LogRing::new();
        let (mut sp, mut sc) = (
            src_ring.take_producer().unwrap(),
            src_ring.take_consumer().unwrap(),
        );
        for r in mix(4) {
            assert!(sp.push(&r));
        }
        let got = relay::<256, 64>(&mut sc, None);
        assert_eq!((got.discarded, got.copied, got.dropped), (4, 0, 0));
        assert_eq!(sc.readable(), 0);
    }

    #[test]
    fn bytes_that_are_not_a_record_are_skipped_and_never_stall_the_source() {
        let src_ring: LogRing<256> = LogRing::new();
        let dst_ring: LogRing<256> = LogRing::new();
        let (mut sp, mut sc) = (
            src_ring.take_producer().unwrap(),
            src_ring.take_consumer().unwrap(),
        );
        let (mut dp, mut dc) = (
            dst_ring.take_producer().unwrap(),
            dst_ring.take_consumer().unwrap(),
        );
        let good = record(dflog::ID_FDS, 1);
        // Junk, a header of an unknown type, a good record, then the head of a record cut short.
        assert!(sp.push(&[1, 2, HEAD1]));
        assert!(sp.push(&[HEAD1, HEAD2, 200]));
        assert!(sp.push(&good));
        assert!(sp.push(&record(dflog::ID_ESC, 2)[..10]));
        let got = relay(&mut sc, Some(&mut dp));
        assert_eq!(got.copied, 1);
        assert_eq!(got.skipped, 3 + 3 + 10);
        assert_eq!(sc.readable(), 0);
        assert_eq!(drain(&mut dc), good);
    }

    /// The board's collector relays into an `ExtLogRing` (PSRAM): same records, same order, the
    /// destination counts what does not fit, and the source is consumed all the same.
    #[test]
    #[allow(unsafe_code)] // `PsramReady::new`: a host test has no PSRAM to wait for.
    fn relay_with_an_external_ring_destination_copies_whole_records_and_counts_drops() {
        use crate::extring::{ExtLogRing, PsramBuf, PsramReady};
        let src_ring: LogRing<256> = LogRing::new();
        let dst_ring: ExtLogRing<64> = ExtLogRing::new();
        let buf: &'static mut PsramBuf<64> =
            std::boxed::Box::leak(std::boxed::Box::new(PsramBuf::new()));
        // SAFETY: a host test, there is no PSRAM to wait for.
        let ready = unsafe { PsramReady::new() };
        dst_ring.bind(buf, &ready).unwrap();
        let (mut sp, mut sc) = (
            src_ring.take_producer().unwrap(),
            src_ring.take_consumer().unwrap(),
        );
        let (mut dp, dc) = (
            dst_ring.take_producer().unwrap(),
            dst_ring.take_consumer().unwrap(),
        );
        let recs = mix(3);
        for r in &recs {
            assert!(sp.push(r));
        }
        let got = relay_with(&mut sc, Some(|rec: &[u8]| dp.push(rec)));
        // ESC 40 B fits in 64, RCS 28 B does not (24 left), FDS 19 B does.
        assert_eq!((got.copied, got.dropped), (2, 1));
        assert_eq!(dst_ring.dropped(), 1);
        assert_eq!(sc.readable(), 0);
        let (a, b) = dc.peek();
        assert_eq!([a, b].concat(), [recs[0].clone(), recs[2].clone()].concat());
        let got = relay_with::<256, fn(&[u8]) -> bool>(&mut sc, None);
        assert_eq!((got.copied, got.discarded), (0, 0));
    }
}
