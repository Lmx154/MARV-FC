//! Lock-free cells between the two cores: `Seqlock` (flight core to core 0, the writer never waits), `TripleBuffer` (core 0 to flight core, wait-free on both sides) and `Kick` (cross-core wake and requests).
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
