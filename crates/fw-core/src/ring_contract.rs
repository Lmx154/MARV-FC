//! Test-only: ONE set of behavioural tests for the byte-ring protocol, instantiated for the SRAM
//! ring ([`crate::xcore::LogRing`]) and the external-memory ring ([`crate::extring::ExtLogRing`]
//! bound to a [`crate::extring::PsramBuf`]), so the protocol written twice cannot drift apart
//! unnoticed.
//!
//! `ring_contract_tests!(name, setup)` expands to a module `name` of tests; `setup!(ring, N)` is a
//! macro that declares `ring`, a ready ring of `N` bytes whose `take_producer` and
//! `take_consumer` return the handles (the external ring is bound there). The tests use only the
//! API the two rings share.
#![allow(unsafe_code)]

use std::collections::VecDeque;
use std::vec::Vec;

use crate::xcore::LogStats;

/// Small deterministic generator (xorshift64*), so that a failure repeats.
pub(crate) struct Rng(pub(crate) u64);

impl Rng {
    pub(crate) fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    pub(crate) fn below(&mut self, n: usize) -> usize {
        (self.next() >> 11) as usize % n
    }
}

/// A reference ring: bytes in a `VecDeque`, a drop count and the free-byte samples of the
/// current statistics window.
pub(crate) struct Model {
    bytes: VecDeque<u8>,
    dropped: u32,
    window: Vec<u32>,
    capacity: usize,
}

impl Model {
    fn sample(&mut self) -> usize {
        let free = self.capacity - self.bytes.len();
        self.window.push(free as u32);
        free
    }

    fn stats(&mut self) -> LogStats {
        let free = (self.capacity - self.bytes.len()) as u32;
        let window = core::mem::take(&mut self.window);
        if window.is_empty() {
            return LogStats {
                samples: 0,
                free_min: free,
                free_max: free,
                free_avg: free,
            };
        }
        let sum: u64 = window.iter().map(|&f| u64::from(f)).sum();
        LogStats {
            samples: window.len() as u32,
            free_min: *window.iter().min().unwrap(),
            free_max: *window.iter().max().unwrap(),
            free_avg: (sum / window.len() as u64) as u32,
        }
    }
}

/// The ring's committed bytes, as one `Vec`.
macro_rules! read_all {
    ($consumer:expr) => {{
        let (a, b) = $consumer.peek();
        [a, b].concat()
    }};
}

macro_rules! ring_contract_tests {
    ($name:ident, $setup:ident) => {
        mod $name {
            use super::{Model, Rng};
            use crate::xcore::{LogStats, PUSH_WITH_MAX};
            use core::sync::atomic::{AtomicBool, Ordering};
            use std::collections::VecDeque;
            use std::thread;
            use std::time::{Duration, Instant};
            use std::vec::Vec;

        #[test]
        fn handles_are_taken_once() {
            $setup!(ring, 16);
            assert_eq!(ring.capacity(), 16);
            let producer = ring.take_producer();
            assert!(producer.is_some());
            assert!(ring.take_producer().is_none());
            let consumer = ring.take_consumer();
            assert!(consumer.is_some());
            assert!(ring.take_consumer().is_none());
        }

        #[test]
        fn empty_ring() {
            $setup!(ring, 16);
            let consumer = ring.take_consumer().unwrap();
            assert_eq!(consumer.readable(), 0);
            assert_eq!(consumer.peek(), (&[][..], &[][..]));
            assert_eq!(consumer.free(), 16);
            assert_eq!(ring.free(), 16);
            assert_eq!(ring.readable(), 0);
            assert_eq!(ring.dropped(), 0);
        }

        #[test]
        fn exact_fit_then_one_byte_too_many() {
            $setup!(ring, 16);
            let mut producer = ring.take_producer().unwrap();
            let mut consumer = ring.take_consumer().unwrap();
            let record: Vec<u8> = (0..16).collect();
            assert!(producer.push(&record), "exactly the free space fits");
            assert_eq!(ring.dropped(), 0);
            assert_eq!(consumer.readable(), 16);
            assert_eq!(producer.free(), 0);
            assert!(!producer.push(&[1]), "one byte over");
            assert_eq!(ring.dropped(), 1);
            assert_eq!(read_all!(consumer), record, "the drop wrote nothing");
            assert_eq!(consumer.consume(16), 16);
            assert!(!producer.push(&[0; 17]), "one byte more than the ring");
            assert_eq!(ring.dropped(), 2);
            assert_eq!(consumer.readable(), 0);
            assert!(producer.push(&[7; 16]));
            assert_eq!(read_all!(consumer), [7; 16]);
        }

        #[test]
        fn one_byte_too_many_after_partial_fill() {
            $setup!(ring, 16);
            let mut producer = ring.take_producer().unwrap();
            let consumer = ring.take_consumer().unwrap();
            assert!(producer.push(&[1; 10]));
            assert!(producer.push(&[2; 6]), "the 6 bytes left");
            assert!(!producer.push(&[3]));
            assert_eq!(consumer.readable(), 16);
            let mut ring2 = [0u8; 16];
            ring2[..10].fill(1);
            ring2[10..].fill(2);
            assert_eq!(read_all!(consumer), ring2);
        }

        #[test]
        fn zero_length_record() {
            $setup!(ring, 8);
            let mut producer = ring.take_producer().unwrap();
            let consumer = ring.take_consumer().unwrap();
            assert!(producer.push(&[]));
            assert_eq!(consumer.readable(), 0);
            assert_eq!(ring.dropped(), 0);
            assert!(producer.push(&[9; 8]));
            assert!(producer.push(&[]), "an empty record fits a full ring");
            assert_eq!(ring.dropped(), 0);
            let grant = producer.reserve(0).unwrap();
            assert!(grant.is_empty());
            assert_eq!(grant.len(), 0);
            grant.commit();
            assert_eq!(consumer.readable(), 8);
        }

        #[test]
        fn record_larger_than_ring_is_a_drop_not_a_panic() {
            $setup!(ring, 8);
            let mut producer = ring.take_producer().unwrap();
            assert!(!producer.push(&[0; 9]));
            assert!(!producer.push(&[0; 4096]));
            assert!(producer.reserve(usize::MAX).is_none());
            assert!(producer.reserve(1 << 33).is_none());
            assert!(!producer.push_with(9, |_| unreachable!()));
            assert!(!producer.push_with(PUSH_WITH_MAX + 1, |_| unreachable!()));
            assert_eq!(ring.dropped(), 6);
            assert_eq!(ring.readable(), 0);
        }

        #[test]
        fn wrap_around_and_two_slice_peek() {
            $setup!(ring, 8);
            let mut producer = ring.take_producer().unwrap();
            let mut consumer = ring.take_consumer().unwrap();
            assert!(producer.push(&[1, 2, 3, 4, 5, 6]));
            assert_eq!(consumer.consume(5), 5);
            assert!(
                producer.push(&[7, 8, 9, 10, 11, 12]),
                "wraps: 2 bytes, then 4"
            );
            assert_eq!(consumer.readable(), 7);
            let (a, b) = consumer.peek();
            assert_eq!((a, b), (&[6, 7, 8][..], &[9, 10, 11, 12][..]));
            let (a, b) = consumer.peek_up_to(2);
            assert_eq!((a, b), (&[6, 7][..], &[][..]));
            let (a, b) = consumer.peek_up_to(5);
            assert_eq!((a, b), (&[6, 7, 8][..], &[9, 10][..]));
            let (a, b) = consumer.peek_up_to(0);
            assert_eq!((a.len(), b.len()), (0, 0));
            assert_eq!(consumer.consume(4), 4);
            let (a, b) = consumer.peek();
            assert_eq!((a, b), (&[10, 11, 12][..], &[][..]));
            assert_eq!(
                consumer.consume(100),
                3,
                "consume is clamped to what is readable"
            );
            assert_eq!(consumer.readable(), 0);
        }

        #[test]
        fn reserve_across_the_end_gives_two_slices_and_push_with_copes() {
            $setup!(ring, 8);
            let mut producer = ring.take_producer().unwrap();
            let mut consumer = ring.take_consumer().unwrap();
            assert!(producer.push(&[0; 6]));
            consumer.consume(6);
            {
                let mut grant = producer.reserve(5).unwrap();
                let (a, b) = grant.slices();
                assert_eq!((a.len(), b.len()), (2, 3));
                a.copy_from_slice(&[1, 2]);
                b.copy_from_slice(&[3, 4, 5]);
                grant.commit();
            }
            assert_eq!(read_all!(consumer), [1, 2, 3, 4, 5]);
            consumer.consume(5);
            assert!(producer.push_with(4, |buf| {
                assert_eq!(buf.len(), 4);
                buf.copy_from_slice(&[10, 11, 12, 13]);
                4
            }));
            consumer.consume(4);
            assert!(producer.push_with(3, |buf| {
                assert_eq!(buf.len(), 3, "a contiguous buffer also across the end");
                buf.copy_from_slice(&[20, 21, 22]);
                3
            }));
            assert_eq!(read_all!(consumer), [20, 21, 22]);
            assert_eq!(ring.dropped(), 0);
        }

        #[test]
        fn uncommitted_grant_is_invisible_and_partial_commit_gives_the_rest_back() {
            $setup!(ring, 8);
            let mut producer = ring.take_producer().unwrap();
            let consumer = ring.take_consumer().unwrap();
            {
                let mut grant = producer.reserve(8).unwrap();
                let (a, _) = grant.slices();
                a.fill(0xEE);
            }
            assert_eq!(consumer.readable(), 0, "dropped grant publishes nothing");
            assert_eq!(ring.dropped(), 0);
            let mut grant = producer.reserve(6).unwrap();
            let (a, _) = grant.slices();
            a.copy_from_slice(&[1, 2, 3, 4, 5, 6]);
            grant.commit_up_to(4);
            assert_eq!(read_all!(consumer), [1, 2, 3, 4]);
            assert_eq!(producer.free(), 4);
            assert!(producer.push_with(4, |b| {
                b[..2].copy_from_slice(&[7, 8]);
                2
            }));
            assert!(producer.push_with(2, |_| 0));
            assert!(producer.push_with(2, |_| 99), "a count over len is clamped");
            assert_eq!(read_all!(consumer)[..6], [1, 2, 3, 4, 7, 8]);
            assert_eq!(consumer.readable(), 8);
        }

        #[test]
        fn stats_windows() {
            $setup!(ring, 64);
            let mut producer = ring.take_producer().unwrap();
            let mut consumer = ring.take_consumer().unwrap();
            let empty = consumer.take_stats();
            assert_eq!(
                empty,
                LogStats {
                    samples: 0,
                    free_min: 64,
                    free_max: 64,
                    free_avg: 64
                }
            );
            assert!(producer.push(&[0; 10]));
            assert!(producer.push(&[0; 10]));
            assert!(producer.push(&[0; 10]));
            assert!(!producer.push(&[0; 100]));
            let stats = consumer.take_stats();
            assert_eq!(
                stats,
                LogStats {
                    samples: 4,
                    free_min: 34,
                    free_max: 64,
                    free_avg: (64 + 54 + 44 + 34) / 4
                }
            );
            let stats = consumer.take_stats();
            assert_eq!(stats.samples, 0);
            assert_eq!(stats.free_avg, 34);
            consumer.consume(30);
            assert!(producer.push(&[0; 4]));
            assert!(producer.push(&[0; 4]));
            let stats = consumer.take_stats();
            assert_eq!(
                stats,
                LogStats {
                    samples: 2,
                    free_min: 60,
                    free_max: 64,
                    free_avg: 62
                }
            );
        }

        /// `ops` random operations against the model; every byte read equals the model's, in order, `dropped` equals the model's count at every step, and the statistics windows equal the model's.
        fn model_run<const N: usize>(ops: u32, seed: u64) -> (u64, u64) {
            $setup!(ring, N);
            let mut producer = ring.take_producer().unwrap();
            let mut consumer = ring.take_consumer().unwrap();
            let mut model = Model {
                bytes: VecDeque::new(),
                dropped: 0,
                window: Vec::new(),
                capacity: N,
            };
            let mut rng = Rng(seed);
            let mut counter = 0u8;
            let (mut accepted, mut dropped_records) = (0u64, 0u64);
            let next_len = |rng: &mut Rng| {
                if rng.below(2) == 0 {
                    rng.below(9)
                } else {
                    rng.below(N + 4)
                }
            };
            for step in 0..ops {
                match rng.below(100) {
                    0..35 => {
                        let len = next_len(&mut rng);
                        let record: Vec<u8> = (0..len)
                            .map(|_| {
                                counter = counter.wrapping_add(1);
                                counter
                            })
                            .collect();
                        let free = model.sample();
                        let fits = len <= free;
                        assert_eq!(producer.push(&record), fits, "step {step}");
                        if fits {
                            model.bytes.extend(record);
                            accepted += 1;
                        } else {
                            model.dropped += 1;
                            dropped_records += 1;
                        }
                    }
                    35..50 => {
                        let len = next_len(&mut rng).min(PUSH_WITH_MAX + 3);
                        let write = rng.below(len + 2);
                        let free = model.sample();
                        let fits = len <= free && len <= PUSH_WITH_MAX;
                        let mut produced = Vec::new();
                        let ok = producer.push_with(len, |buf| {
                            assert_eq!(buf.len(), len);
                            for b in buf.iter_mut() {
                                counter = counter.wrapping_add(1);
                                *b = counter;
                            }
                            produced = buf.to_vec();
                            write
                        });
                        assert_eq!(ok, fits, "step {step}");
                        if fits {
                            let n = write.min(len);
                            model.bytes.extend(&produced[..n]);
                            accepted += 1;
                        } else {
                            model.dropped += 1;
                            dropped_records += 1;
                        }
                    }
                    50..65 => {
                        let len = next_len(&mut rng);
                        let free = model.sample();
                        let fits = len <= free;
                        let keep = rng.below(len + 2);
                        let commit = rng.below(2) == 0;
                        match producer.reserve(len) {
                            Some(mut grant) => {
                                assert!(fits, "step {step}");
                                assert_eq!(grant.len(), len);
                                let (a, b) = grant.slices();
                                assert_eq!(a.len() + b.len(), len);
                                let mut data = Vec::new();
                                for slot in a.iter_mut().chain(b.iter_mut()) {
                                    counter = counter.wrapping_add(1);
                                    *slot = counter;
                                    data.push(counter);
                                }
                                if commit {
                                    grant.commit_up_to(keep);
                                    model.bytes.extend(&data[..keep.min(len)]);
                                    accepted += 1;
                                }
                            }
                            None => {
                                assert!(!fits, "step {step}");
                                model.dropped += 1;
                                dropped_records += 1;
                            }
                        }
                    }
                    65..85 => {
                        let want = rng.below(N + 3);
                        let got = consumer.consume(want);
                        assert_eq!(got, want.min(model.bytes.len()), "step {step}");
                        model.bytes.drain(..got);
                    }
                    85..97 => {
                        let max = if rng.below(4) == 0 {
                            usize::MAX
                        } else {
                            rng.below(N + 3)
                        };
                        let (a, b) = consumer.peek_up_to(max);
                        let n = max.min(model.bytes.len());
                        assert_eq!(a.len() + b.len(), n, "step {step}");
                        assert!(!a.is_empty() || b.is_empty(), "second only after a first");
                        let got: Vec<u8> = a.iter().chain(b).copied().collect();
                        let want: Vec<u8> = model.bytes.iter().take(n).copied().collect();
                        assert_eq!(got, want, "step {step}");
                    }
                    _ => {
                        let want = model.stats();
                        assert_eq!(consumer.take_stats(), want, "step {step}");
                    }
                }
                assert_eq!(ring.dropped(), model.dropped, "step {step}");
                assert_eq!(consumer.readable(), model.bytes.len(), "step {step}");
                assert_eq!(producer.free(), N - model.bytes.len(), "step {step}");
            }
            (accepted, dropped_records)
        }

        #[test]
        fn q4_i2_property_ten_to_the_six_operations_against_a_model() {
            let start = Instant::now();
            let (accepted, dropped) = model_run::<128>(1_000_000, 0x1234_5678_9ABC_DEF1);
            std::println!(
                "q4-i2 model, N = 128: 10^6 operations in {:?}: {accepted} records accepted, {dropped} dropped",
                start.elapsed()
            );
            assert!(
                accepted > 10_000 && dropped > 10_000,
                "both paths are exercised"
            );
            for (seed, n) in [(1, 1usize), (2, 2), (3, 16), (4, 4096)] {
                let (accepted, dropped) = match n {
                    1 => model_run::<1>(100_000, seed),
                    2 => model_run::<2>(100_000, seed),
                    16 => model_run::<16>(100_000, seed),
                    _ => model_run::<4096>(100_000, seed),
                };
                assert!(accepted > 0, "N = {n}");
                std::println!("q4-i2 model, N = {n}: {accepted} accepted, {dropped} dropped");
            }
            std::println!("q4-i2 total: {:?}", start.elapsed());
        }

        const MAX_RECORD: usize = 120;
        const PUSHES: u32 = 400_000;
        const DEADLINE: Duration = Duration::from_secs(30);

        /// A record of `len` bytes (6 to `MAX_RECORD`): the length as `u16`, the sequence number as
        /// `u32`, then the bytes `seq + i`.
        fn record_byte(len: usize, seq: u32, i: usize) -> u8 {
            match i {
                0 => len as u8,
                1 => (len >> 8) as u8,
                2..6 => seq.to_le_bytes()[i - 2],
                _ => (seq as u8).wrapping_add(i as u8),
            }
        }

        #[test]
        fn two_threads_no_torn_record_no_lost_byte_drops_plus_delivered_equals_pushed() {
            $setup!(ring, 1024);
            let mut producer = ring.take_producer().unwrap();
            let mut consumer = ring.take_consumer().unwrap();
            let done = AtomicBool::new(false);
            let start = Instant::now();
            let (pushed, delivered, missing) = thread::scope(|s| {
                let producing = s.spawn(|| {
                    let mut rng = Rng(77);
                    let mut pushed = 0u32;
                    for seq in 0..PUSHES {
                        assert!(start.elapsed() < DEADLINE, "producer: time limit");
                        let len = 6 + rng.below(MAX_RECORD - 5);
                        if seq % 2 == 0 {
                            let mut buf = [0u8; MAX_RECORD];
                            for (i, b) in buf.iter_mut().take(len).enumerate() {
                                *b = record_byte(len, seq, i);
                            }
                            producer.push(&buf[..len]);
                        } else if let Some(mut grant) = producer.reserve(len) {
                            let (a, b) = grant.slices();
                            for (i, slot) in a.iter_mut().chain(b.iter_mut()).enumerate() {
                                *slot = record_byte(len, seq, i);
                            }
                            grant.commit();
                        }
                        pushed += 1;
                    }
                    done.store(true, Ordering::Release);
                    pushed
                });
                let mut rng = Rng(99);
                let mut pending: Vec<u8> = Vec::new();
                let (mut delivered, mut missing, mut next_seq) = (0u32, 0u32, 0u32);
                let mut rounds = 0u64;
                loop {
                    rounds += 1;
                    assert!(rounds < 2_000_000_000, "consumer: iteration limit");
                    assert!(start.elapsed() < DEADLINE, "consumer: time limit");
                    let finished = done.load(Ordering::Acquire);
                    let take = 1 + rng.below(300);
                    let n = {
                        let (a, b) = consumer.peek_up_to(take);
                        pending.extend_from_slice(a);
                        pending.extend_from_slice(b);
                        a.len() + b.len()
                    };
                    assert_eq!(consumer.consume(n), n);
                    while pending.len() >= 2 {
                        let len = usize::from(pending[0]) | usize::from(pending[1]) << 8;
                        assert!((6..=MAX_RECORD).contains(&len), "torn: length {len}");
                        if pending.len() < len {
                            break;
                        }
                        let seq = u32::from_le_bytes(pending[2..6].try_into().unwrap());
                        assert!(
                            seq >= next_seq && seq < PUSHES,
                            "seq {seq}, expected {next_seq}"
                        );
                        for (i, &b) in pending[..len].iter().enumerate() {
                            assert_eq!(b, record_byte(len, seq, i), "torn record {seq} byte {i}");
                        }
                        missing += seq - next_seq;
                        next_seq = seq + 1;
                        delivered += 1;
                        pending.drain(..len);
                    }
                    if finished && n == 0 {
                        break;
                    }
                }
                assert!(
                    pending.is_empty(),
                    "{} bytes of a partial record left",
                    pending.len()
                );
                missing += PUSHES - next_seq;
                (producing.join().unwrap(), delivered, missing)
            });
            let dropped = ring.dropped();
            std::println!(
                "two threads: {pushed} pushed, {delivered} delivered, {dropped} dropped in {:?}",
                start.elapsed()
            );
            assert_eq!(pushed, PUSHES);
            assert_eq!(missing, dropped, "every missing record is a counted drop");
            assert_eq!(u64::from(delivered) + u64::from(dropped), u64::from(pushed));
            assert!(delivered > 0);
            assert_eq!(ring.readable(), 0);
        }
        }
    };
}

/// `setup!(ring, N)` for the SRAM ring.
macro_rules! setup_xcore {
    ($ring:ident, $n:tt) => {
        let $ring = crate::xcore::LogRing::<$n>::new();
    };
}

/// `setup!(ring, N)` for the external ring: bound to a leaked heap buffer (a `'static`, like the
/// board's).
macro_rules! setup_extring {
    ($ring:ident, $n:tt) => {
        let $ring = crate::extring::ExtLogRing::<$n>::new();
        {
            let buf =
                std::boxed::Box::leak(std::boxed::Box::new(crate::extring::PsramBuf::<$n>::new()));
            // SAFETY: a host test, there is no PSRAM to wait for.
            let ready = unsafe { crate::extring::PsramReady::new() };
            $ring.bind(buf, &ready).unwrap();
        }
    };
}

ring_contract_tests!(xcore_log_ring, setup_xcore);
ring_contract_tests!(ext_log_ring, setup_extring);
