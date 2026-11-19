//! The relay of the flight log ring into the main log ring (design: decision-flight-logging, section 3).
//!
//! [`relay`] is the pure body of what core 0's collector does every tick: it walks the readable
//! bytes of the flight ring (the consumer side of a [`LogRing`](crate::xcore::LogRing) that core 1
//! fills with whole records), takes the length of each record from [`dflog::TYPES`] by its type
//! id, reserves that length in the destination ring and copies the record. When the destination
//! has no room the record is dropped (the destination counts it, see
//! [`LogProducer::reserve`]) and it is STILL consumed from the source, so a stalled card never
//! backs up into the flight core. With no destination ("no log open") everything readable is
//! consumed and nothing is pushed.
//!
//! A byte sequence that is not a record header (`A3 95 <type of the table>`) is skipped one byte
//! at a time and counted in [`Relayed::skipped`], as the walker of [`dflog::walk`] does (PROVISIONAL:
//! the design does not say). Every step consumes at least one byte, so the flight ring cannot
//! stall on bad bytes. A record whose length runs past the readable bytes (the producer commits
//! whole records only, so this means damage) is skipped with all the bytes that remain.
//! Only the bytes readable when the call starts are walked.

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
    mut dst: Option<&mut LogProducer<'_, D>>,
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
        match dst.as_deref_mut() {
            None => out.discarded += 1,
            Some(ring) => match scratch.get(..len).is_some_and(|rec| ring.push(rec)) {
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
}
