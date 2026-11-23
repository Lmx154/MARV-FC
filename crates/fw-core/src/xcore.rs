//! Lock-free cells between the two cores: `Seqlock` (flight core to core 0, the writer never waits), `TripleBuffer` (core 0 to flight core, wait-free on both sides), `Kick` (cross-core wake and requests) and `LogRing` (byte ring for whole log records, wait-free on both sides).
//!
//! Design: decision-foundation, two-core amendment, "Rules" (SHARING) and invariant F3.
//!
//! Every cell is `const`-constructible so it can live in a `static`, holds `T: Copy` payloads,
//! allocates nothing and uses only 8- and 32-bit atomics: `thumbv8m.main-none-eabihf` has no
//! 64-bit atomics. No cell takes a critical section, and no operation on the flight core's side
//! (the [`Seqlock`] writer, the [`TripleBuffer`] reader, [`Kick::kick`], [`Kick::take`]) loops.
//!
//! # Memory orderings and why each is enough
//!
//! [`Seqlock`] (the Linux `seqcount` protocol with one writer):
//! - writer: `seq.load(Relaxed)`: only the writer stores `seq`, so it reads its own last store.
//! - writer: `seq.store(odd, Relaxed)` then `fence(Release)`: the fence orders the odd store
//!   before every later store, so no reader can see a byte of the new payload without the odd
//!   value being visible to its second sequence load (it pairs with the reader's acquire fence).
//! - writer: payload stored with one volatile write.
//! - writer: `seq.store(even, Release)`: the payload stores are visible before the new even value.
//! - reader: `seq.load(Acquire)`: pairs with that release store, so a reader that sees the even
//!   value copies at least that write's payload, and the copy cannot be hoisted above the load.
//! - reader: payload copied into a `MaybeUninit<T>` with one volatile read, then `fence(Acquire)`:
//!   the copy's loads complete before the second sequence load.
//! - reader: `seq.load(Relaxed)`: ordered by the fence above; equal to the first value means no
//!   write began before the copy ended, and only then is the copy turned into a `T`.
//!
//! On ARMv8-M the release and acquire operations and the fences compile to `stl`/`lda` or `dmb`, which
//! order accesses across both cores. The payload accesses are volatile, not atomic: in the Rust
//! abstract machine an overlapping read and write is a data race (the same approach as the
//! `seqlock` crate and crossbeam's `AtomicCell` fallback); the protocol never interprets the bytes
//! of an overlapped copy, which is why the copy stays a `MaybeUninit<T>` until the second check.
//! The sequence is 32 bits: a reader would accept a torn copy only if it stalled inside one copy
//! for exactly a multiple of 2^31 writes (about 3 days at 8 kHz).
//!
//! [`TripleBuffer`] (one writer, one reader, three slots, one `AtomicU8` holding the middle slot's
//! index and a fresh bit): each slot belongs to exactly one of the writer, the reader or the
//! middle word, and changes hands only through a `swap(AcqRel)` of that word. Release publishes the
//! slot just written (writer) or hands back a slot whose reads are complete (reader); acquire makes
//! the received slot's contents, or the end of the other side's reads, visible before it is used.
//! The reader's first `load(Relaxed)` only tests the fresh bit: if it sees the bit, the `swap`
//! that follows carries the synchronisation. Slot accesses are plain reads and writes: ownership
//! makes them race-free.
//!
//! [`Kick`]: `fetch_or(Release)` in [`Kick::kick`] and `swap(0, Acquire)` in [`Kick::take`] and
//! [`Kick::wait`]: whatever the kicker wrote before kicking is visible to the task that receives
//! the bits. No lost wake: `wait` registers its waker, then takes the bits again. Either that
//! `swap` sees the kick's bits, or the kick's `fetch_or` comes later in `bits`' order and the
//! kick's `AtomicWaker::wake` (an `AcqRel` read-modify-write of the waker's state) then finds
//! the registered waker, or wakes the waker being registered.
//!
//! [`LogRing`] (one producer, one consumer, a byte ring of `N` bytes, free-running 32-bit write
//! index `head` and read index `tail`, both stored by one side only): the bytes in `[tail, head)`
//! belong to the consumer, the other `N - (head - tail)` bytes to the producer.
//! - producer: `head.load(Relaxed)` (its own last store), `tail.load(Acquire)`: pairs with the
//!   consumer's `tail.store(Release)`, so every read the consumer made of bytes it released has
//!   completed before the producer writes over them (a producer never overwrites an unread byte:
//!   it grants only `len <= N - (head - tail)` bytes, and `tail` never runs ahead of the reads).
//! - producer: the record's bytes are plain stores into the granted area, then `commit` does
//!   `head.store(head + len, Release)`: the bytes are visible before the new `head`.
//! - consumer: `head.load(Acquire)` pairs with that store, so every byte below the loaded `head`
//!   is complete and visible; a record is granted whole and published by one store of `head`, so
//!   the consumer sees none of it or all of it (an uncommitted or dropped grant never moves
//!   `head`). It reads in place, then `consume` does `tail.store(tail + n, Release)`.
//! - no read-modify-write and no loop on either side: both are wait-free. The producer's
//!   statistics go through a [`Seqlock`] written by the producer alone; the consumer asks for a
//!   new window with a counter only it stores (`stat_req`, `Relaxed`: no data is published
//!   through it); `dropped` is stored by the producer alone (load, saturating add, store).
//!
//! Read-modify-writes on ARMv8-M Mainline (`swap`, `fetch_or`, `TripleBuffer` and `Kick`) compile
//! to a load/store-exclusive pair (`ldaexb`/`stlexb`) that retries only when the exclusive monitor
//! was cleared between the two (the other core wrote the same word, or an exception was taken):
//! no algorithmic loop, and retries bounded by the other side's writes to that one word in a
//! window of a few cycles. The
//! [`Seqlock`] writer uses no read-modify-write at all. The handle takes (`take_writer`,
//! `take_reader`) use `swap(true, Relaxed)`: only the uniqueness of the winner matters, no data
//! is published through those flags.
#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use core::future::poll_fn;
use core::mem::MaybeUninit;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering, fence};
use core::task::Poll;

use atomic_waker::AtomicWaker;

/// One-writer, many-reader cell, flight core to core 0.
///
/// The one writer ([`SeqlockWriter`], taken once) never waits and never retries; readers copy the
/// value and retry until the copy is consistent ([`Seqlock::read`]), or give up after a bounded
/// number of attempts ([`Seqlock::try_read`]).
pub struct Seqlock<T> {
    seq: AtomicU32,
    data: UnsafeCell<T>,
    writer_taken: AtomicBool,
}

// SAFETY: `data` is stored only by the one `SeqlockWriter` (unique: `take_writer` hands it out
// once, and `write` takes `&mut self`), and readers copy it into a `MaybeUninit<T>` that becomes a
// `T` only when the sequence shows that no write overlapped the copy. `T: Send` because a reader
// on another core receives a copy of a value written on the writer's core.
unsafe impl<T: Copy + Send> Sync for Seqlock<T> {}

impl<T: Copy> Seqlock<T> {
    /// A cell holding `initial`, with its writer not yet taken.
    pub const fn new(initial: T) -> Self {
        Self {
            seq: AtomicU32::new(0),
            data: UnsafeCell::new(initial),
            writer_taken: AtomicBool::new(false),
        }
    }

    /// The cell's one writer: `Some` on the first call, `None` on every later call.
    ///
    /// ```
    /// use fw_core::xcore::Seqlock;
    ///
    /// static CELL: Seqlock<u32> = Seqlock::new(0);
    ///
    /// let mut writer = CELL.take_writer().expect("first take");
    /// assert!(CELL.take_writer().is_none());
    /// writer.write(7);
    /// assert_eq!(CELL.read(), 7);
    /// assert_eq!(CELL.try_read(4), Some(7));
    /// ```
    pub fn take_writer(&self) -> Option<SeqlockWriter<'_, T>> {
        if self.writer_taken.swap(true, Ordering::Relaxed) {
            None
        } else {
            Some(SeqlockWriter { cell: self })
        }
    }

    /// A consistent copy of the latest value, retrying while a write overlaps the copy.
    ///
    /// Spins as long as the writer is inside a write (a few hundred cycles, or the length of a
    /// pause of the writer's core in mid-write): for core 0 only, never the flight core.
    pub fn read(&self) -> T {
        loop {
            if let Some(value) = self.read_once() {
                return value;
            }
            core::hint::spin_loop();
        }
    }

    /// A consistent copy of the latest value, or `None` if each of `max_attempts` copies overlapped
    /// a write (for example the writer's core was paused in mid-write). `max_attempts == 0` gives
    /// `None`.
    pub fn try_read(&self, max_attempts: u32) -> Option<T> {
        for _ in 0..max_attempts {
            if let Some(value) = self.read_once() {
                return Some(value);
            }
            core::hint::spin_loop();
        }
        None
    }

    fn read_once(&self) -> Option<T> {
        let before = self.seq.load(Ordering::Acquire);
        if before & 1 != 0 {
            return None;
        }
        // SAFETY: `data.get()` is valid and aligned for the life of `self`. The copy is read as a
        // `MaybeUninit<T>` (same layout as `T`), so bytes torn by a concurrent write are never
        // a `T`; the volatile read is one access the compiler can neither drop nor merge.
        let copy = unsafe { ptr::read_volatile(self.data.get().cast::<MaybeUninit<T>>()) };
        fence(Ordering::Acquire);
        let after = self.seq.load(Ordering::Relaxed);
        if before == after {
            // SAFETY: the sequence was even before the copy and unchanged after it (the acquire
            // fence orders the copy before the second load), so no write overlapped the copy: it
            // holds the bytes of one complete `T` stored by `new` or `SeqlockWriter::write`.
            Some(unsafe { copy.assume_init() })
        } else {
            None
        }
    }
}

/// The one writer of a [`Seqlock`], from [`Seqlock::take_writer`]: `Send`, not `Clone`.
pub struct SeqlockWriter<'a, T> {
    cell: &'a Seqlock<T>,
}

impl<T: Copy> SeqlockWriter<'_, T> {
    /// Stores `value`. Wait-free: two stores, one fence, one copy; no loop and no read-modify-write.
    pub fn write(&mut self, value: T) {
        let cell = self.cell;
        let seq = cell.seq.load(Ordering::Relaxed);
        cell.seq.store(seq.wrapping_add(1), Ordering::Relaxed);
        fence(Ordering::Release);
        // SAFETY: `data.get()` is valid and aligned for the life of `cell`. This handle is the
        // only writer (`take_writer` hands out one, `write` takes `&mut self`); readers may copy
        // concurrently, and the odd sequence written above makes them discard such a copy.
        unsafe { ptr::write_volatile(cell.data.get(), value) };
        cell.seq.store(seq.wrapping_add(2), Ordering::Release);
    }
}

const SLOT_MASK: u8 = 0b011;
const FRESH: u8 = 0b100;

/// One-writer, one-reader cell, core 0 to the flight core: wait-free on both sides.
///
/// Three slots and one atomic word holding the middle slot's index and a fresh bit. The writer
/// fills its own slot and swaps it into the middle; the reader swaps the middle out only when the
/// fresh bit is set. Both handles are taken once.
///
/// ```
/// use fw_core::xcore::TripleBuffer;
///
/// static CELL: TripleBuffer<u32> = TripleBuffer::new(0);
///
/// let mut writer = CELL.take_writer().expect("first take");
/// let mut reader = CELL.take_reader().expect("first take");
/// assert!(CELL.take_writer().is_none());
/// assert!(CELL.take_reader().is_none());
/// assert_eq!(reader.read(), (0, false));
/// writer.write(5);
/// assert_eq!(reader.read(), (5, true));
/// assert_eq!(reader.read(), (5, false));
/// ```
pub struct TripleBuffer<T> {
    slots: [UnsafeCell<T>; 3],
    middle: AtomicU8,
    writer_taken: AtomicBool,
    reader_taken: AtomicBool,
}

// SAFETY: each slot is owned by exactly one of the writer handle, the reader handle and the
// middle word, and changes owner only through an `AcqRel` swap of `middle`; each handle is unique
// (taken once, methods take `&mut self`), so no slot is ever accessed by two cores at once.
// `T: Send` because values written on one core are read on the other.
unsafe impl<T: Copy + Send> Sync for TripleBuffer<T> {}

impl<T: Copy> TripleBuffer<T> {
    /// A buffer whose three slots hold `initial`: the reader's first read gives `(initial, false)`.
    pub const fn new(initial: T) -> Self {
        Self {
            slots: [
                UnsafeCell::new(initial),
                UnsafeCell::new(initial),
                UnsafeCell::new(initial),
            ],
            middle: AtomicU8::new(1),
            writer_taken: AtomicBool::new(false),
            reader_taken: AtomicBool::new(false),
        }
    }

    /// The buffer's one writer: `Some` on the first call, `None` on every later call.
    pub fn take_writer(&self) -> Option<TripleWriter<'_, T>> {
        if self.writer_taken.swap(true, Ordering::Relaxed) {
            None
        } else {
            Some(TripleWriter { buf: self, slot: 0 })
        }
    }

    /// The buffer's one reader: `Some` on the first call, `None` on every later call.
    pub fn take_reader(&self) -> Option<TripleReader<'_, T>> {
        if self.reader_taken.swap(true, Ordering::Relaxed) {
            None
        } else {
            Some(TripleReader { buf: self, slot: 2 })
        }
    }

    fn slot(&self, index: u8) -> *mut T {
        let [first, second, third] = &self.slots;
        match index & SLOT_MASK {
            0 => first.get(),
            1 => second.get(),
            _ => third.get(),
        }
    }
}

/// The one writer of a [`TripleBuffer`], from [`TripleBuffer::take_writer`]: `Send`, not `Clone`.
pub struct TripleWriter<'a, T> {
    buf: &'a TripleBuffer<T>,
    slot: u8,
}

impl<T: Copy> TripleWriter<'_, T> {
    /// Publishes `value`. Wait-free: one slot write and one swap.
    pub fn write(&mut self, value: T) {
        // SAFETY: `self.slot` is owned by this writer (see the `Sync` impl), so nothing else
        // accesses it until the swap below hands it over; the pointer is valid and aligned.
        unsafe { self.buf.slot(self.slot).write(value) };
        let previous = self.buf.middle.swap(self.slot | FRESH, Ordering::AcqRel);
        self.slot = previous & SLOT_MASK;
    }
}

/// The one reader of a [`TripleBuffer`], from [`TripleBuffer::take_reader`]: `Send`, not `Clone`.
pub struct TripleReader<'a, T> {
    buf: &'a TripleBuffer<T>,
    slot: u8,
}

impl<T: Copy> TripleReader<'_, T> {
    /// The latest published value, and `true` if at least one write was published since the
    /// previous read. Wait-free: one load, at most one swap, one slot read.
    pub fn read(&mut self) -> (T, bool) {
        let fresh = self.buf.middle.load(Ordering::Relaxed) & FRESH != 0;
        if fresh {
            let previous = self.buf.middle.swap(self.slot, Ordering::AcqRel);
            self.slot = previous & SLOT_MASK;
        }
        // SAFETY: `self.slot` is owned by this reader (see the `Sync` impl): the writer never
        // writes it while the reader holds it; the pointer is valid and aligned.
        let value = unsafe { self.buf.slot(self.slot).read() };
        (value, fresh)
    }
}

/// The one cross-core wake: request bits plus one waiting task.
///
/// [`Kick::kick`] may be called from either core and from interrupt context (lock-free). One task
/// waits with [`Kick::wait`], or polls with [`Kick::take`]. Two concurrent waiters are not
/// supported: the second registration replaces the first task's waker, so the first can sleep
/// through a kick, and the bits go to whichever task polls first.
pub struct Kick {
    bits: AtomicU8,
    waker: AtomicWaker,
}

impl Kick {
    /// No bits pending, no task registered.
    pub const fn new() -> Self {
        Self {
            bits: AtomicU8::new(0),
            waker: AtomicWaker::new(),
        }
    }

    /// Sets `bits` (OR-ed into those already pending) and wakes the waiting task. `bits == 0`
    /// does nothing.
    pub fn kick(&self, bits: u8) {
        if bits == 0 {
            return;
        }
        self.bits.fetch_or(bits, Ordering::Release);
        self.waker.wake();
    }

    /// The pending bits, cleared atomically; 0 if none. Never waits.
    pub fn take(&self) -> u8 {
        self.bits.swap(0, Ordering::Acquire)
    }

    /// Waits until bits are pending, then returns them (non-zero) and clears them atomically.
    pub async fn wait(&self) -> u8 {
        poll_fn(|cx| {
            let pending = self.take();
            if pending != 0 {
                return Poll::Ready(pending);
            }
            self.waker.register(cx.waker());
            let pending = self.take();
            if pending != 0 {
                Poll::Ready(pending)
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

impl Default for Kick {
    fn default() -> Self {
        Self::new()
    }
}

/// Statistics of the free space of a [`LogRing`] over one window, from [`LogConsumer::take_stats`]
/// (the DSF record's FMn, FMx and FAv).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogStats {
    /// Samples in the window: one per [`LogProducer::reserve`] (so per push, dropped ones too).
    /// 0 for an empty window; the three values below are then the free bytes now.
    pub samples: u32,
    /// Smallest free-byte count seen by a sample.
    pub free_min: u32,
    /// Largest free-byte count seen by a sample.
    pub free_max: u32,
    /// Integer mean of the free-byte counts of the samples (their sum divided by `samples`,
    /// rounded down; the sum is 64-bit, so it cannot overflow).
    pub free_avg: u32,
}

/// The producer's running statistics of one window; `epoch` is the consumer's request counter
/// at the time the window began.
#[derive(Clone, Copy)]
struct Window {
    epoch: u32,
    samples: u32,
    min: u32,
    max: u32,
    sum: u64,
}

impl Window {
    const fn empty(epoch: u32) -> Self {
        Self {
            epoch,
            samples: 0,
            min: u32::MAX,
            max: 0,
            sum: 0,
        }
    }

    fn add(&mut self, free: u32) {
        self.samples = self.samples.saturating_add(1);
        self.min = self.min.min(free);
        self.max = self.max.max(free);
        self.sum = self.sum.saturating_add(u64::from(free));
    }
}

/// Attempts [`LogConsumer::take_stats`] makes to read the producer's statistics before it gives
/// up for this call (a try fails only if the producer was inside its few-instruction write).
const STAT_READ_ATTEMPTS: u32 = 32;

/// The ring's storage, 4-byte aligned so that a 4-byte-aligned file write can read from it.
#[repr(C, align(4))]
struct Storage<const N: usize>([u8; N]);

/// Lock-free single-producer single-consumer byte ring for whole log records: `N` bytes (a power
/// of two, at most 2^31; 32768 in the firmware), wait-free on both sides, usable between two
/// cores, no critical section.
///
/// The producer ([`LogProducer`], taken once) puts a record in WHOLE or not at all: when the free
/// space is smaller than the record, nothing is written and the drop counter
/// ([`LogRing::dropped`]) is incremented. The consumer ([`LogConsumer`], taken once) reads BYTES,
/// in place, as at most two slices ([`LogConsumer::peek`]), and releases them with
/// [`LogConsumer::consume`]; it never sees a byte of a record that is not committed. The memory
/// ordering is argued in the module documentation.
///
/// The consumer's bytes keep the record order, so a consumer that consumes whole 4096-byte
/// pieces reads from addresses that are 4-byte aligned (the storage is) whenever the ring is
/// at least 4096 bytes and it has consumed multiples of 4096 since the start.
///
/// ```
/// use fw_core::xcore::LogRing;
///
/// static RING: LogRing<64> = LogRing::new();
///
/// let mut producer = RING.take_producer().expect("first take");
/// let mut consumer = RING.take_consumer().expect("first take");
/// assert!(RING.take_producer().is_none());
/// assert!(producer.push(b"hello"));
/// assert!(!producer.push(&[0; 100]), "does not fit: dropped, counted");
/// assert_eq!(RING.dropped(), 1);
/// // Encode in place: `reserve`, fill the slices, `commit`.
/// let mut grant = producer.reserve(3).expect("fits");
/// let (first, _second) = grant.slices();
/// first.copy_from_slice(b"abc");
/// grant.commit();
/// let (a, b) = consumer.peek();
/// assert_eq!((a, b), (&b"helloabc"[..], &b""[..]));
/// assert_eq!(consumer.consume(8), 8);
/// assert_eq!(consumer.readable(), 0);
/// ```
pub struct LogRing<const N: usize> {
    storage: UnsafeCell<Storage<N>>,
    /// Free-running write index: stored by the producer only.
    head: AtomicU32,
    /// Free-running read index: stored by the consumer only.
    tail: AtomicU32,
    /// Records dropped (saturating): stored by the producer only.
    dropped: AtomicU32,
    /// Number of windows the consumer has taken: stored by the consumer only.
    stat_req: AtomicU32,
    /// The producer's current statistics window; its writer is the producer's taken-once flag.
    stats: Seqlock<Window>,
    consumer_taken: AtomicBool,
}

// SAFETY: the bytes of `storage` are accessed under the ring protocol: the producer writes only
// the free area `[head, tail + N)` and the consumer reads only the committed area `[tail, head)`;
// the two areas are disjoint, each index is stored by one side only (each handle is unique:
// taken once, methods that move an index take `&mut self`), and the `Release` stores of an
// index and `Acquire` loads of it on the other side order every access to a byte before the byte
// changes owner. All other fields are atomics or a `Seqlock`, which is `Sync` itself.
unsafe impl<const N: usize> Sync for LogRing<N> {}

impl<const N: usize> LogRing<N> {
    /// An empty ring with both handles not yet taken. A compile error unless `N` is a power of
    /// two and at most 2^31.
    pub const fn new() -> Self {
        const {
            assert!(
                N.is_power_of_two() && N <= 1 << 31,
                "LogRing: N must be a power of two, at most 2^31"
            );
        }
        Self {
            storage: UnsafeCell::new(Storage([0; N])),
            head: AtomicU32::new(0),
            tail: AtomicU32::new(0),
            dropped: AtomicU32::new(0),
            stat_req: AtomicU32::new(0),
            stats: Seqlock::new(Window::empty(0)),
            consumer_taken: AtomicBool::new(false),
        }
    }

    /// The ring's one producer: `Some` on the first call, `None` on every later call.
    pub fn take_producer(&self) -> Option<LogProducer<'_, N>> {
        let stats = self.stats.take_writer()?;
        Some(LogProducer {
            ring: self,
            stats,
            window: Window::empty(0),
        })
    }

    /// The ring's one consumer: `Some` on the first call, `None` on every later call.
    pub fn take_consumer(&self) -> Option<LogConsumer<'_, N>> {
        if self.consumer_taken.swap(true, Ordering::Relaxed) {
            None
        } else {
            Some(LogConsumer {
                ring: self,
                epoch: 0,
            })
        }
    }

    /// The ring's size in bytes.
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Records dropped since the start (saturates at `u32::MAX`). Readable from any core.
    pub fn dropped(&self) -> u32 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Committed bytes not yet consumed. Readable from any core; a snapshot (the two indices
    /// are read one after the other, `tail` first: `head` only grows, so the `head` read second
    /// is never behind the `tail` read first and the difference cannot wrap).
    pub fn readable(&self) -> usize {
        self.readable_with(|| {})
    }

    /// [`readable`](Self::readable) with `between` run after the first load and before the second
    /// (the tests' hook for the other side's work between them).
    fn readable_with(&self, between: impl FnOnce()) -> usize {
        let tail = self.tail.load(Ordering::Acquire);
        between();
        let head = self.head.load(Ordering::Acquire);
        head.wrapping_sub(tail) as usize
    }

    /// Free bytes (`capacity - readable`). Readable from any core, same caveat as `readable`.
    pub fn free(&self) -> usize {
        N - self.readable().min(N)
    }

    fn base(&self) -> *mut u8 {
        self.storage.get().cast::<u8>()
    }
}

impl<const N: usize> Default for LogRing<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// The one producer of a [`LogRing`], from [`LogRing::take_producer`]: `Send`, not `Clone`.
/// Every method is wait-free.
pub struct LogProducer<'a, const N: usize> {
    ring: &'a LogRing<N>,
    stats: SeqlockWriter<'a, Window>,
    window: Window,
}

impl<const N: usize> LogProducer<'_, N> {
    /// Free bytes now: at least this many, because only the consumer can change it, upwards.
    pub fn free(&self) -> usize {
        let head = self.ring.head.load(Ordering::Relaxed);
        let tail = self.ring.tail.load(Ordering::Acquire);
        N - (head.wrapping_sub(tail) as usize).min(N)
    }

    /// Records dropped since the start (saturates at `u32::MAX`).
    pub fn dropped(&self) -> u32 {
        self.ring.dropped()
    }

    /// Puts `record` in whole, or not at all: `false` (nothing written, the drop counter
    /// incremented) if fewer than `record.len()` bytes are free, which includes a record longer
    /// than the ring. An empty record succeeds and writes nothing.
    pub fn push(&mut self, record: &[u8]) -> bool {
        match self.reserve(record.len()) {
            Some(mut grant) => {
                grant.copy_from(record);
                grant.commit();
                true
            }
            None => false,
        }
    }

    /// Encodes one record straight into the ring: `encode` gets a CONTIGUOUS buffer of exactly
    /// `len` bytes (the maximum the record can need) and returns how many bytes it wrote
    /// (`dflog`'s `encode_*` functions have this shape: `|b| encode_bat(b, t, 0, v, a)`); those
    /// bytes, at most `len`, are committed. Returns `false`, with the drop counted and `encode`
    /// not called, if `len` bytes are not free or `len` is over [`PUSH_WITH_MAX`]; otherwise
    /// `true`, also when `encode` returns 0 (nothing is committed then).
    ///
    /// The buffer is the ring itself unless the free area wraps around the end of the storage
    /// at that point; then `encode` writes a stack buffer of [`PUSH_WITH_MAX`] bytes that is
    /// copied in (that is the reason for the limit). Use [`LogProducer::reserve`] to handle the
    /// two slices yourself.
    pub fn push_with(&mut self, len: usize, encode: impl FnOnce(&mut [u8]) -> usize) -> bool {
        if len > PUSH_WITH_MAX {
            self.sample(self.free());
            return self.drop_one();
        }
        let Some(mut grant) = self.reserve(len) else {
            return false;
        };
        let (first, second) = grant.slices();
        if second.is_empty() {
            let written = encode(first).min(len);
            grant.commit_up_to(written);
        } else {
            let mut scratch = [0u8; PUSH_WITH_MAX];
            let written = scratch.get_mut(..len).map_or(0, encode).min(len);
            grant.copy_from(scratch.get(..written).unwrap_or(&[]));
            grant.commit_up_to(written);
        }
        true
    }

    /// Reserves `len` bytes for one record: `Some(grant)` to fill and [`Grant::commit`], or
    /// `None` (nothing written, the drop counter incremented) if fewer than `len` bytes are
    /// free. A grant dropped without `commit` publishes nothing (the bytes written into it are
    /// never seen by the consumer). One grant at a time (it borrows the producer).
    ///
    /// Every call samples the free space for the statistics (see [`LogConsumer::take_stats`]),
    /// whether it succeeds or not.
    pub fn reserve(&mut self, len: usize) -> Option<Grant<'_, N>> {
        let head = self.ring.head.load(Ordering::Relaxed);
        let tail = self.ring.tail.load(Ordering::Acquire);
        let free = N - (head.wrapping_sub(tail) as usize).min(N);
        self.sample(free);
        match u32::try_from(len) {
            Ok(len) if len as usize <= free => Some(Grant {
                ring: self.ring,
                head,
                len,
            }),
            _ => {
                self.drop_one();
                None
            }
        }
    }

    fn drop_one(&mut self) -> bool {
        let count = self.ring.dropped.load(Ordering::Relaxed);
        self.ring
            .dropped
            .store(count.saturating_add(1), Ordering::Relaxed);
        false
    }

    fn sample(&mut self, free: usize) {
        let epoch = self.ring.stat_req.load(Ordering::Relaxed);
        if epoch != self.window.epoch {
            self.window = Window::empty(epoch);
        }
        self.window.add(u32::try_from(free).unwrap_or(u32::MAX));
        self.stats.write(self.window);
    }
}

/// The longest record [`LogProducer::push_with`] takes (it holds a stack buffer of this size for
/// the case of a free area that wraps around the end of the storage).
pub const PUSH_WITH_MAX: usize = 320;

/// Space reserved in a [`LogRing`] for one record, from [`LogProducer::reserve`]: one or two
/// mutable slices (two when the area wraps around the end of the storage), `len` bytes in all.
/// Nothing is visible to the consumer until [`Grant::commit`].
pub struct Grant<'a, const N: usize> {
    ring: &'a LogRing<N>,
    head: u32,
    len: u32,
}

impl<const N: usize> Grant<'_, N> {
    /// The reserved bytes in all.
    pub fn len(&self) -> usize {
        self.len as usize
    }

    /// `true` for a grant of 0 bytes.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The reserved area as two slices, in record order: the second is empty unless the area
    /// wraps around the end of the storage. Their contents are not defined until written.
    pub fn slices(&mut self) -> (&mut [u8], &mut [u8]) {
        let start = self.head as usize & (N - 1);
        let len = self.len as usize;
        let first = len.min(N - start);
        let base = self.ring.base();
        // SAFETY: `start < N` and `first <= N - start`, so the first slice lies inside the
        // storage; the second starts at its beginning and has `len - first <= start` bytes, so
        // the two do not overlap each other, and both lie in the free area, which only this grant
        // (it borrows the unique producer) may access until `commit` publishes it.
        unsafe {
            (
                core::slice::from_raw_parts_mut(base.add(start), first),
                core::slice::from_raw_parts_mut(base, len - first),
            )
        }
    }

    /// Copies `src` to the start of the grant, as many bytes as `src` and the grant have in
    /// common, across the wrap if there is one; returns how many.
    pub fn copy_from(&mut self, src: &[u8]) -> usize {
        let (first, second) = self.slices();
        let n = src.len().min(first.len() + second.len());
        let k = n.min(first.len());
        if let (Some(dst), Some(from)) = (first.get_mut(..k), src.get(..k)) {
            dst.copy_from_slice(from);
        }
        if let (Some(dst), Some(from)) = (second.get_mut(..n - k), src.get(k..n)) {
            dst.copy_from_slice(from);
        }
        n
    }

    /// Publishes the whole grant to the consumer.
    pub fn commit(self) {
        let len = self.len;
        self.commit_len(len);
    }

    /// Publishes the first `n` bytes of the grant (`n` over its length counts as its length)
    /// and gives the rest back.
    pub fn commit_up_to(self, n: usize) {
        let n = u32::try_from(n).unwrap_or(u32::MAX).min(self.len);
        self.commit_len(n);
    }

    fn commit_len(self, n: u32) {
        self.ring
            .head
            .store(self.head.wrapping_add(n), Ordering::Release);
    }
}

/// The one consumer of a [`LogRing`], from [`LogRing::take_consumer`]: `Send`, not `Clone`.
/// Every method is wait-free except [`LogConsumer::take_stats`], which retries a bounded number
/// of times.
pub struct LogConsumer<'a, const N: usize> {
    ring: &'a LogRing<N>,
    /// Windows taken so far; a window the producer began after the last take has this epoch.
    epoch: u32,
}

impl<const N: usize> LogConsumer<'_, N> {
    /// Committed bytes not yet consumed (the producer may commit more at any time).
    pub fn readable(&self) -> usize {
        let tail = self.ring.tail.load(Ordering::Relaxed);
        let head = self.ring.head.load(Ordering::Acquire);
        head.wrapping_sub(tail) as usize
    }

    /// All committed bytes as at most two slices, in order, in place: the second is empty unless
    /// they wrap around the end of the storage. They stay valid until `consume` (which needs
    /// `&mut self`, so they cannot be held across it); bytes committed after the call are not
    /// in them.
    pub fn peek(&self) -> (&[u8], &[u8]) {
        self.peek_up_to(usize::MAX)
    }

    /// The first `max` committed bytes (or all, if fewer) as at most two slices; see `peek`.
    pub fn peek_up_to(&self, max: usize) -> (&[u8], &[u8]) {
        let tail = self.ring.tail.load(Ordering::Relaxed);
        let head = self.ring.head.load(Ordering::Acquire);
        let n = (head.wrapping_sub(tail) as usize).min(max);
        let start = tail as usize & (N - 1);
        let first = n.min(N - start);
        let base = self.ring.base();
        // SAFETY: `n` is at most the committed bytes `[tail, head)`, which the `Acquire` load of
        // `head` made visible and which the producer does not write until `consume` releases
        // them (and `consume` needs `&mut self`, so not while these slices live). `start < N`
        // and `first <= N - start`; the second slice starts at the beginning of the storage and
        // has `n - first <= start` bytes: the two slices lie inside the storage and do not
        // overlap.
        unsafe {
            (
                core::slice::from_raw_parts(base.add(start), first),
                core::slice::from_raw_parts(base, n - first),
            )
        }
    }

    /// Releases the first `n` committed bytes to the producer; returns how many were released
    /// (`n` over `readable()` counts as `readable()`).
    pub fn consume(&mut self, n: usize) -> usize {
        let tail = self.ring.tail.load(Ordering::Relaxed);
        let head = self.ring.head.load(Ordering::Acquire);
        let n = (head.wrapping_sub(tail) as usize).min(n);
        // `n` is at most `N <= 2^31`.
        self.ring
            .tail
            .store(tail.wrapping_add(n as u32), Ordering::Release);
        n
    }

    /// Records dropped since the start (saturates at `u32::MAX`).
    pub fn dropped(&self) -> u32 {
        self.ring.dropped()
    }

    /// Free bytes now (`capacity - readable`; the producer may use them at any time).
    pub fn free(&self) -> usize {
        N - self.readable().min(N)
    }

    /// The statistics of the free space since the previous call (or the start), and a new
    /// window begins.
    ///
    /// The producer samples the free bytes at the start of every [`LogProducer::reserve`] (so
    /// of every push, dropped ones included, before the record goes in) and keeps the minimum,
    /// the maximum, the count and the 64-bit sum of the samples; `free_avg` is sum / count. The
    /// window ends at the producer's last sample before this call reads the producer's record of
    /// it, and the next window begins with the producer's first sample after it sees the
    /// request. A sample made between this call's read of the window and the producer seeing the
    /// request for a new one goes into the window already read, which is not read again: it is
    /// in neither window, so a sample can be missed at a window boundary. It is never in both
    /// and never half in each: `free_min`, `free_max` and `free_avg` always come from one
    /// consistent window, the same samples. Returns `samples == 0` and the free bytes now when
    /// the producer made no sample since the previous call, or (very rarely, the producer's
    /// few-instruction update of the record overlapping every one of 32 reads) could not be
    /// read: the window is then not closed, and the next call returns it.
    pub fn take_stats(&mut self) -> LogStats {
        if let Some(window) = self.ring.stats.try_read(STAT_READ_ATTEMPTS)
            && window.epoch == self.epoch
            && window.samples > 0
        {
            self.epoch = self.epoch.wrapping_add(1);
            self.ring.stat_req.store(self.epoch, Ordering::Relaxed);
            let avg = window.sum / u64::from(window.samples);
            return LogStats {
                samples: window.samples,
                free_min: window.min,
                free_max: window.max,
                free_avg: u32::try_from(avg).unwrap_or(u32::MAX),
            };
        }
        let free = u32::try_from(self.free()).unwrap_or(u32::MAX);
        LogStats {
            samples: 0,
            free_min: free,
            free_max: free,
            free_avg: free,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::future::Future;
    use core::pin::pin;
    use core::task::{Context, RawWaker, RawWakerVTable, Waker};
    use std::sync::Arc;
    use std::sync::atomic::AtomicUsize;
    use std::task::Wake;
    use std::thread;
    use std::time::{Duration, Instant};

    const ITERATIONS: u32 = 10_000_000;
    const WORDS: usize = 16;
    type Payload = [u32; WORDS];

    #[derive(Debug, Default)]
    struct Outcome {
        reads: u64,
        torn: u64,
        backwards: u64,
        fresh_wrong: u64,
    }

    fn is_torn(value: &Payload) -> bool {
        value.iter().any(|&word| word != value[0])
    }

    fn seqlock_harness(read: fn(&Seqlock<Payload>) -> Payload) -> Outcome {
        let cell = Seqlock::new([0; WORDS]);
        let mut writer = cell.take_writer().unwrap();
        thread::scope(|s| {
            s.spawn(move || {
                for i in 1..=ITERATIONS {
                    writer.write([i; WORDS]);
                }
            });
            let mut out = Outcome::default();
            let mut last = 0;
            loop {
                let value = read(&cell);
                out.reads += 1;
                if is_torn(&value) {
                    out.torn += 1;
                    continue;
                }
                if value[0] < last {
                    out.backwards += 1;
                }
                last = value[0];
                if last == ITERATIONS {
                    return out;
                }
            }
        })
    }

    /// The negative control: the real reader minus the second sequence check.
    fn read_without_second_check(cell: &Seqlock<Payload>) -> Payload {
        loop {
            if cell.seq.load(Ordering::Acquire) & 1 == 0 {
                // SAFETY: valid, aligned pointer; any bit pattern is a valid `[u32; 16]`, so a
                // torn copy is still a value (the point of this control is to count them).
                return unsafe { ptr::read_volatile(cell.data.get()) };
            }
            core::hint::spin_loop();
        }
    }

    #[test]
    fn seqlock_f3_never_torn_never_backwards() {
        let out = seqlock_harness(Seqlock::read);
        std::println!("seqlock F3: {out:?}");
        assert_eq!(out.torn, 0);
        assert_eq!(out.backwards, 0);
    }

    #[test]
    fn seqlock_negative_control_without_second_check_tears() {
        let out = seqlock_harness(read_without_second_check);
        std::println!("seqlock negative control: {out:?}");
        assert!(out.torn > 0);
    }

    #[test]
    fn triple_buffer_f3_never_torn_fresh_exact() {
        let buf = TripleBuffer::new([0; WORDS]);
        let mut writer = buf.take_writer().unwrap();
        let mut reader = buf.take_reader().unwrap();
        let out = thread::scope(|s| {
            s.spawn(move || {
                for i in 1..=ITERATIONS {
                    writer.write([i; WORDS]);
                }
            });
            let mut out = Outcome::default();
            let mut last = 0;
            loop {
                let (value, fresh) = reader.read();
                out.reads += 1;
                if is_torn(&value) {
                    out.torn += 1;
                    continue;
                }
                if value[0] < last {
                    out.backwards += 1;
                }
                if fresh != (value[0] != last) {
                    out.fresh_wrong += 1;
                }
                last = value[0];
                if last == ITERATIONS {
                    return out;
                }
            }
        });
        std::println!("triple buffer F3: {out:?}");
        assert_eq!(out.torn, 0);
        assert_eq!(out.backwards, 0);
        assert_eq!(out.fresh_wrong, 0);
    }

    #[test]
    fn triple_buffer_negative_control_single_buffer_tears() {
        let shared: [AtomicU32; WORDS] = [const { AtomicU32::new(0) }; WORDS];
        let out = thread::scope(|s| {
            s.spawn(|| {
                for i in 1..=ITERATIONS {
                    for word in &shared {
                        word.store(i, Ordering::Relaxed);
                    }
                }
            });
            let mut out = Outcome::default();
            loop {
                let value: Payload = core::array::from_fn(|k| shared[k].load(Ordering::Relaxed));
                out.reads += 1;
                if is_torn(&value) {
                    out.torn += 1;
                } else if value[0] == ITERATIONS {
                    return out;
                }
            }
        });
        std::println!("single buffer negative control: {out:?}");
        assert!(out.torn > 0);
    }

    #[derive(Default)]
    struct CountingWaker(AtomicUsize);

    impl Wake for CountingWaker {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn counting_waker() -> (Arc<CountingWaker>, Waker) {
        let counter = Arc::new(CountingWaker::default());
        let waker = Waker::from(Arc::clone(&counter));
        (counter, waker)
    }

    #[test]
    fn kick_pending_without_bits_then_ready_with_exact_bits() {
        let kick = Kick::new();
        let (_, waker) = counting_waker();
        let mut cx = Context::from_waker(&waker);
        let mut fut = pin!(kick.wait());
        assert_eq!(fut.as_mut().poll(&mut cx), Poll::Pending);
        kick.kick(0b0000_0101);
        assert_eq!(fut.as_mut().poll(&mut cx), Poll::Ready(0b0000_0101));
        assert_eq!(kick.take(), 0);
    }

    #[test]
    fn kick_bits_or_together_and_take_clears() {
        let kick = Kick::new();
        assert_eq!(kick.take(), 0);
        kick.kick(0b0001);
        kick.kick(0b0100);
        kick.kick(0);
        assert_eq!(kick.take(), 0b0101);
        assert_eq!(kick.take(), 0);
        kick.kick(0b1000_0010);
        let (_, waker) = counting_waker();
        let mut cx = Context::from_waker(&waker);
        assert_eq!(pin!(kick.wait()).poll(&mut cx), Poll::Ready(0b1000_0010));
        assert_eq!(kick.take(), 0);
        assert_eq!(pin!(kick.wait()).poll(&mut cx), Poll::Pending);
    }

    #[test]
    fn kick_after_registration_wakes_and_next_poll_is_ready() {
        let kick = Kick::new();
        let (counter, waker) = counting_waker();
        let mut cx = Context::from_waker(&waker);
        let mut fut = pin!(kick.wait());
        assert_eq!(fut.as_mut().poll(&mut cx), Poll::Pending);
        assert_eq!(counter.0.load(Ordering::SeqCst), 0);
        kick.kick(0b1000);
        assert_eq!(counter.0.load(Ordering::SeqCst), 1);
        assert_eq!(fut.as_mut().poll(&mut cx), Poll::Ready(0b1000));
    }

    /// The cell under test plus a waker whose first `clone` kicks it. `AtomicWaker::register`
    /// clones the waker while its state is `REGISTERING`, so the kick lands after `wait`'s first
    /// `take` and before its second. `register` then wakes the clone itself (it sees `WAKING`),
    /// so only the same poll returning `Ready` shows that the second `take` caught the kick.
    struct KickOnClone {
        kick: Kick,
        bits: u8,
        fired: AtomicBool,
    }

    static KICK_ON_CLONE: RawWakerVTable =
        RawWakerVTable::new(kick_on_clone, ignore_waker, ignore_waker, ignore_waker);

    fn kick_on_clone(data: *const ()) -> RawWaker {
        // SAFETY: `data` comes from the `&KickOnClone` the test built the waker from, which
        // outlives the waker and every clone of it.
        let hook = unsafe { &*data.cast::<KickOnClone>() };
        if !hook.fired.swap(true, Ordering::SeqCst) {
            hook.kick.kick(hook.bits);
        }
        RawWaker::new(data, &KICK_ON_CLONE)
    }

    fn ignore_waker(_: *const ()) {}

    #[test]
    fn kick_between_first_take_and_registration_is_returned_by_same_poll() {
        let hook = KickOnClone {
            kick: Kick::new(),
            bits: 0b0010_0000,
            fired: AtomicBool::new(false),
        };
        let raw = RawWaker::new(ptr::from_ref(&hook).cast(), &KICK_ON_CLONE);
        // SAFETY: the vtable keeps the `RawWaker` contract: `clone` returns the same data and
        // vtable, `wake`, `wake_by_ref` and `drop` free nothing, and `hook` outlives the waker.
        let waker = unsafe { Waker::from_raw(raw) };
        let mut cx = Context::from_waker(&waker);
        let mut fut = pin!(hook.kick.wait());
        assert_eq!(fut.as_mut().poll(&mut cx), Poll::Ready(0b0010_0000));
        assert!(hook.fired.load(Ordering::SeqCst));
        assert_eq!(hook.kick.take(), 0);
    }

    struct ThreadWaker(thread::Thread);

    impl Wake for ThreadWaker {
        fn wake(self: Arc<Self>) {
            self.0.unpark();
        }
    }

    /// Minimal thread-park executor; `None` if `timeout` passes with the future still pending,
    /// without polling it again after the deadline (so a lost wake is a failure, not a delay).
    fn block_on_timeout<F: Future>(fut: F, timeout: Duration) -> Option<F::Output> {
        let waker = Waker::from(Arc::new(ThreadWaker(thread::current())));
        let mut cx = Context::from_waker(&waker);
        let mut fut = pin!(fut);
        let deadline = Instant::now() + timeout;
        loop {
            if let Poll::Ready(value) = fut.as_mut().poll(&mut cx) {
                return Some(value);
            }
            thread::park_timeout(deadline.saturating_duration_since(Instant::now()));
            if Instant::now() >= deadline {
                return None;
            }
        }
    }

    #[test]
    fn kick_two_threads_every_kick_received() {
        const KICKS: u32 = 100_000;
        const TIMEOUT: Duration = Duration::from_secs(5);
        let kick = Kick::new();
        let received = AtomicU32::new(0);
        let failed = AtomicBool::new(false);
        thread::scope(|s| {
            s.spawn(|| {
                for i in 0..KICKS {
                    let expected = 1u8 << (i % 8);
                    if block_on_timeout(kick.wait(), TIMEOUT) != Some(expected) {
                        failed.store(true, Ordering::SeqCst);
                        return;
                    }
                    received.store(i + 1, Ordering::SeqCst);
                }
            });
            for i in 0..KICKS {
                kick.kick(1u8 << (i % 8));
                let deadline = Instant::now() + TIMEOUT;
                while received.load(Ordering::SeqCst) != i + 1 {
                    assert!(!failed.load(Ordering::SeqCst), "waiter failed at kick {i}");
                    assert!(Instant::now() < deadline, "kick {i} not received");
                    thread::yield_now();
                }
            }
        });
        assert_eq!(received.load(Ordering::SeqCst), KICKS);
        assert_eq!(kick.take(), 0);
    }
}

#[cfg(test)]
mod log_ring_tests {
    use super::*;
    use std::collections::VecDeque;
    use std::thread;
    use std::time::{Duration, Instant};
    use std::vec::Vec;

    /// Small deterministic generator (xorshift64*), so that a failure repeats.
    struct Rng(u64);

    impl Rng {
        fn next(&mut self) -> u64 {
            self.0 ^= self.0 >> 12;
            self.0 ^= self.0 << 25;
            self.0 ^= self.0 >> 27;
            self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
        }

        fn below(&mut self, n: usize) -> usize {
            (self.next() >> 11) as usize % n
        }
    }

    fn read_all<const N: usize>(consumer: &LogConsumer<'_, N>) -> Vec<u8> {
        let (a, b) = consumer.peek();
        [a, b].concat()
    }

    #[test]
    fn readable_from_a_third_party_reads_tail_first_so_it_cannot_wrap() {
        // The other side moves both indices past the first load: with `head` read first the
        // difference would be 8 - 16 = about 2^32; with `tail` first it is 16 - 0.
        let ring = LogRing::<64>::new();
        let mut producer = ring.take_producer().unwrap();
        let mut consumer = ring.take_consumer().unwrap();
        assert!(producer.push(&[1; 8]));
        let readable = ring.readable_with(|| {
            assert!(producer.push(&[2; 8]));
            assert_eq!(consumer.consume(16), 16);
        });
        assert!(readable <= ring.capacity(), "wrapped: {readable}");
        assert_eq!(ring.readable(), 0);
        assert_eq!(ring.free(), 64);
    }

    #[test]
    fn handles_are_taken_once() {
        let ring = LogRing::<16>::new();
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
        let ring = LogRing::<16>::new();
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
        let ring = LogRing::<16>::new();
        let mut producer = ring.take_producer().unwrap();
        let mut consumer = ring.take_consumer().unwrap();
        let record: Vec<u8> = (0..16).collect();
        assert!(producer.push(&record), "exactly the free space fits");
        assert_eq!(ring.dropped(), 0);
        assert_eq!(consumer.readable(), 16);
        assert_eq!(producer.free(), 0);
        assert!(!producer.push(&[1]), "one byte over");
        assert_eq!(ring.dropped(), 1);
        assert_eq!(read_all(&consumer), record, "the drop wrote nothing");
        assert_eq!(consumer.consume(16), 16);
        assert!(!producer.push(&[0; 17]), "one byte more than the ring");
        assert_eq!(ring.dropped(), 2);
        assert_eq!(consumer.readable(), 0);
        assert!(producer.push(&[7; 16]));
        assert_eq!(read_all(&consumer), [7; 16]);
    }

    #[test]
    fn one_byte_too_many_after_partial_fill() {
        let ring = LogRing::<16>::new();
        let mut producer = ring.take_producer().unwrap();
        let consumer = ring.take_consumer().unwrap();
        assert!(producer.push(&[1; 10]));
        assert!(producer.push(&[2; 6]), "the 6 bytes left");
        assert!(!producer.push(&[3]));
        assert_eq!(consumer.readable(), 16);
        let mut ring2 = [0u8; 16];
        ring2[..10].fill(1);
        ring2[10..].fill(2);
        assert_eq!(read_all(&consumer), ring2);
    }

    #[test]
    fn zero_length_record() {
        let ring = LogRing::<8>::new();
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
        let ring = LogRing::<8>::new();
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
        let ring = LogRing::<8>::new();
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
        let ring = LogRing::<8>::new();
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
        assert_eq!(read_all(&consumer), [1, 2, 3, 4, 5]);
        consumer.consume(5);
        // The head is at 3 now: a contiguous record, then one that wraps (6 bytes from index 3).
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
        assert_eq!(read_all(&consumer), [20, 21, 22]);
        assert_eq!(ring.dropped(), 0);
    }

    #[test]
    fn uncommitted_grant_is_invisible_and_partial_commit_gives_the_rest_back() {
        let ring = LogRing::<8>::new();
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
        assert_eq!(read_all(&consumer), [1, 2, 3, 4]);
        assert_eq!(producer.free(), 4);
        // `encode` writing less than `len`, and 0.
        assert!(producer.push_with(4, |b| {
            b[..2].copy_from_slice(&[7, 8]);
            2
        }));
        assert!(producer.push_with(2, |_| 0));
        assert!(producer.push_with(2, |_| 99), "a count over len is clamped");
        assert_eq!(read_all(&consumer)[..6], [1, 2, 3, 4, 7, 8]);
        assert_eq!(consumer.readable(), 8);
    }

    #[test]
    fn indices_wrap_around_u32() {
        let ring = LogRing::<8>::new();
        ring.head.store(u32::MAX - 2, Ordering::Relaxed);
        ring.tail.store(u32::MAX - 2, Ordering::Relaxed);
        let mut producer = ring.take_producer().unwrap();
        let mut consumer = ring.take_consumer().unwrap();
        for round in 0..8u8 {
            assert!(producer.push(&[round; 5]));
            assert_eq!(read_all(&consumer), [round; 5]);
            assert_eq!(consumer.consume(5), 5);
        }
        assert_eq!(ring.dropped(), 0);
    }

    #[test]
    fn dropped_counter_saturates() {
        let ring = LogRing::<4>::new();
        ring.dropped.store(u32::MAX - 1, Ordering::Relaxed);
        let mut producer = ring.take_producer().unwrap();
        for _ in 0..4 {
            assert!(!producer.push(&[0; 5]));
        }
        assert_eq!(ring.dropped(), u32::MAX);
    }

    #[test]
    fn stats_windows() {
        let ring = LogRing::<64>::new();
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
        // Samples are the free bytes before each push: 64, 54, 44, then a dropped 100-byte push
        // sees 34.
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
        // No push since: an empty window, and the free bytes now.
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

    /// A reference ring: bytes in a `VecDeque`, a drop count and the free-byte samples of the
    /// current statistics window.
    struct Model {
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

    /// Q4-I2: `ops` random operations against the model; every byte read equals the model's, in
    /// order, `dropped` equals the model's count at every step, and the statistics windows
    /// equal the model's.
    fn model_run<const N: usize>(ops: u32, seed: u64) -> (u64, u64) {
        let ring = LogRing::<N>::new();
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
        let ring = LogRing::<1024>::new();
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
