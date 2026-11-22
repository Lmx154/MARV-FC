//! `ExtLogRing`: the [`LogRing`](crate::xcore::LogRing) protocol over a byte buffer that is not in
//! the ring's own memory: the control block (indices, statistics, a bound pointer) lives in the
//! struct, in SRAM, and the `N` bytes of storage are a [`PsramBuf`] bound at run time.
//!
//! Design: decision-psram-offload-20261121, D2. The log buffers (`logger::RING`, `logdl::HANDOVER`)
//! are the only users; the board places its [`PsramBuf`] statics in the `.psram` section and binds
//! them once the PSRAM is initialised and verified.
//!
//! # The four rules of the external memory (P1 to P4 of the decision)
//!
//! - P1: core 1 never touches it. Core 1 makes no XIP/QMI access; this module's code is not in the
//!   flight core's RAM-code selector (the module is `extring`, not `xcore`) and only core-0 tasks
//!   call it.
//! - P2: core-0 thread mode only. Never from an interrupt handler or an interrupt executor, never
//!   during a flash operation (the QSPI bus is then not mapped as PSRAM).
//! - P3: no atomic and no initial value in the external memory. The bytes are plain loads and
//!   stores (through the slices of [`Grant::slices`] and [`LogConsumer::peek`]); a load/store
//!   exclusive loop outside SRAM never completes on the RP2350, so every atomic (the indices, the
//!   drop counter, the statistics, the taken flags, the bound pointer) is in the control block.
//!   [`PsramBuf`] holds `MaybeUninit` bytes (nothing is programmed into `.psram`: the image carries
//!   no initial value), [`ExtLogRing::bind`] zeroes the `N` bytes once, at run time, and the ring
//!   reads only `[tail, head)`, which the producer wrote after the bind.
//! - P4: no access before the bind. [`ExtLogRing::take_producer`] and
//!   [`ExtLogRing::take_consumer`] return `None` until [`ExtLogRing::bind`] has succeeded, and
//!   `bind` needs a [`PsramReady`], which the board makes only after the PSRAM is initialised and
//!   its canary verified. After a flash operation the board must not use the handles until it has
//!   remapped and verified the PSRAM again (the board's rule; a ring does not know).
//!
//! # Protocol
//!
//! Identical to `LogRing`, whose module documentation ([`crate::xcore`]) argues each ordering:
//! free-running 32-bit `head` (stored by the producer only, `Release` after the bytes) and `tail`
//! (stored by the consumer only, `Release` after the reads), `Acquire` loads on the other side, no
//! read-modify-write and no loop on either side. The bound pointer is stored once, with
//! `compare_exchange(AcqRel)`, and every access to the bytes loads it with `Acquire`, so a handle
//! passed to another thread still sees the bytes of a buffer bound before.
//!
//! The statistics use `xcore`'s `Seqlock` (a plain struct of the control block, in SRAM). Its
//! generic code, instantiated for this module's window type, carries `xcore` in its mangled name,
//! so the RAM-code selector `*7fw_core5xcore*` places that small instance in SRAM on core 0's
//! behalf; the ring's own code stays in flash.
//!
//! The byte slices: [`Grant::slices`] and [`LogProducer::push_with`]'s `encode` get `&mut [u8]`
//! over the buffer, which must never cover uninitialised memory (a `&mut [u8]` over it is
//! undefined behaviour in Rust, whatever the bytes are used for). [`ExtLogRing::bind`] therefore
//! writes zeroes over all `N` bytes before it publishes the buffer (64 KiB of PSRAM, about 3 ms
//! once at boot), as `LogRing`'s storage is zeroed. The contents of a grant are still not defined
//! until written (stale bytes of an earlier lap): callers treat them as write-only, as
//! `LogRing`'s documentation says; the encoders in `dflog` do.
#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};

use crate::xcore::{LogStats, PUSH_WITH_MAX, Seqlock, SeqlockWriter};

/// `N` bytes of external memory for one [`ExtLogRing`]: the one type the board places in the
/// `.psram` section. No initial value (`MaybeUninit`), so nothing is stored in the image and the
/// bytes are undefined until the ring's producer writes them. 4-byte aligned, like `LogRing`'s
/// storage, so a 4-byte-aligned file write can read from it.
#[repr(C, align(4))]
pub struct PsramBuf<const N: usize>(UnsafeCell<MaybeUninit<[u8; N]>>);

// SAFETY: the buffer has no operations of its own; its bytes are reached only through the one
// `ExtLogRing` it is bound to (`bind` takes it by `&'static mut`, so nothing else holds it), under
// the ring protocol, which gives each byte to one side at a time. A bare `&PsramBuf` can do
// nothing with the bytes, so sharing it between threads is sound.
unsafe impl<const N: usize> Sync for PsramBuf<N> {}

impl<const N: usize> PsramBuf<N> {
    /// An uninitialised buffer (`static BUF: PsramBuf<N> = PsramBuf::new()`).
    pub const fn new() -> Self {
        Self(UnsafeCell::new(MaybeUninit::uninit()))
    }
}

impl<const N: usize> Default for PsramBuf<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Proof that the PSRAM is initialised and its canary verified. The board makes one after that
/// and passes it to [`ExtLogRing::bind`]; it carries no data.
#[derive(Debug)]
pub struct PsramReady(());

impl PsramReady {
    /// A token.
    ///
    /// # Safety
    ///
    /// Call only after the PSRAM is initialised and verified (init, then a canary write and
    /// read-back), and only on core 0; a ring bound without it reads and writes memory that is
    /// not there.
    pub const unsafe fn new() -> Self {
        Self(())
    }
}

/// Why [`ExtLogRing::bind`] refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BindError {
    /// The ring is already bound (to this buffer or another); a ring is bound once.
    AlreadyBound,
}

/// The producer's running statistics of one window; `epoch` is the consumer's request counter
/// at the time the window began (the same as `xcore`'s).
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
/// up for this call.
const STAT_READ_ATTEMPTS: u32 = 32;

/// Lock-free single-producer single-consumer byte ring for whole log records over external
/// memory: the API and semantics of [`LogRing`](crate::xcore::LogRing) (`N` a power of two, at
/// most 2^31; whole records or a counted drop; the consumer reads in place as at most two
/// slices), with the bytes in a [`PsramBuf`] bound once by [`ExtLogRing::bind`]. See the module
/// documentation for the rules P1 to P4.
///
/// ```
/// use fw_core::extring::{ExtLogRing, PsramBuf, PsramReady};
///
/// static RING: ExtLogRing<64> = ExtLogRing::new();
/// static mut BUF: PsramBuf<64> = PsramBuf::new();
///
/// assert!(RING.take_producer().is_none(), "not bound yet");
/// // The board does this after the PSRAM is initialised and verified.
/// let ready = unsafe { PsramReady::new() };
/// let buf = unsafe { &mut *core::ptr::addr_of_mut!(BUF) };
/// RING.bind(buf, &ready).expect("first bind");
/// let mut producer = RING.take_producer().expect("first take");
/// let mut consumer = RING.take_consumer().expect("first take");
/// assert!(producer.push(b"hello"));
/// assert_eq!(consumer.peek(), (&b"hello"[..], &b""[..]));
/// assert_eq!(consumer.consume(5), 5);
/// ```
pub struct ExtLogRing<const N: usize> {
    /// The first byte of the bound buffer; null until bound. Stored once.
    buf: AtomicPtr<u8>,
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

impl<const N: usize> ExtLogRing<N> {
    /// An unbound ring with both handles not yet taken. A compile error unless `N` is a power of
    /// two and at most 2^31.
    pub const fn new() -> Self {
        const {
            assert!(
                N.is_power_of_two() && N <= 1 << 31,
                "ExtLogRing: N must be a power of two, at most 2^31"
            );
        }
        Self {
            buf: AtomicPtr::new(ptr::null_mut()),
            head: AtomicU32::new(0),
            tail: AtomicU32::new(0),
            dropped: AtomicU32::new(0),
            stat_req: AtomicU32::new(0),
            stats: Seqlock::new(Window::empty(0)),
            consumer_taken: AtomicBool::new(false),
        }
    }

    /// Binds the ring to its external buffer, once: after `Ok` the handles can be taken. A second
    /// call, with any buffer, is refused and changes nothing (the second buffer is not written).
    /// `ready` is the proof that the memory is usable (see [`PsramReady`]).
    ///
    /// The `N` bytes are zeroed before the ring publishes the buffer, so that the `&mut [u8]` the
    /// grants hand out never cover uninitialised memory: this writes the whole buffer once (64 KiB
    /// of PSRAM is about 3 ms), in the caller's thread, before any handle exists.
    pub fn bind(&self, buf: &'static mut PsramBuf<N>, ready: &PsramReady) -> Result<(), BindError> {
        let _ = ready;
        if self.is_bound() {
            return Err(BindError::AlreadyBound);
        }
        let base = buf.0.get().cast::<u8>();
        // SAFETY: `buf` is a `&'static mut`, so nothing else accesses its `N` bytes; `base` is
        // valid for `N` writes of `u8`.
        unsafe { base.write_bytes(0, N) };
        match self
            .buf
            .compare_exchange(ptr::null_mut(), base, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => Ok(()),
            Err(_) => Err(BindError::AlreadyBound),
        }
    }

    /// `true` once [`bind`](Self::bind) has succeeded.
    pub fn is_bound(&self) -> bool {
        !self.buf.load(Ordering::Acquire).is_null()
    }

    /// The ring's one producer: `Some` on the first call after the bind, `None` before it and on
    /// every later call.
    pub fn take_producer(&self) -> Option<LogProducer<'_, N>> {
        if !self.is_bound() {
            return None;
        }
        let stats = self.stats.take_writer()?;
        Some(LogProducer {
            ring: self,
            stats,
            window: Window::empty(0),
        })
    }

    /// The ring's one consumer: `Some` on the first call after the bind, `None` before it and on
    /// every later call.
    pub fn take_consumer(&self) -> Option<LogConsumer<'_, N>> {
        if !self.is_bound() {
            return None;
        }
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

    /// Committed bytes not yet consumed. Readable from any core; a snapshot (`tail` is read
    /// first, so the difference cannot wrap; see `LogRing::readable`).
    pub fn readable(&self) -> usize {
        let tail = self.tail.load(Ordering::Acquire);
        let head = self.head.load(Ordering::Acquire);
        head.wrapping_sub(tail) as usize
    }

    /// Free bytes (`capacity - readable`). Readable from any core, same caveat as `readable`.
    pub fn free(&self) -> usize {
        N - self.readable().min(N)
    }

    /// The first byte of the buffer. Only called through a handle, which exists only after the
    /// bind, so it is not null.
    fn base(&self) -> *mut u8 {
        self.buf.load(Ordering::Acquire)
    }
}

impl<const N: usize> Default for ExtLogRing<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// The one producer of an [`ExtLogRing`], from [`ExtLogRing::take_producer`]: `Send`, not
/// `Clone`. Every method is wait-free.
pub struct LogProducer<'a, const N: usize> {
    ring: &'a ExtLogRing<N>,
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

    /// As `LogProducer::push_with` of `LogRing`: `encode` gets a CONTIGUOUS buffer of exactly
    /// `len` bytes (write-only: its contents are not defined) and returns how many it wrote;
    /// those bytes, at most `len`, are committed. `false`, with the drop counted and `encode` not
    /// called, if `len` bytes are not free or `len` is over [`PUSH_WITH_MAX`]; otherwise `true`,
    /// also when `encode` returns 0. When the free area wraps around the end of the buffer,
    /// `encode` writes a stack buffer that is copied in.
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
    /// free. A grant dropped without `commit` publishes nothing. One grant at a time.
    ///
    /// Every call samples the free space for the statistics, whether it succeeds or not.
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

/// Space reserved in an [`ExtLogRing`] for one record, from [`LogProducer::reserve`]: one or two
/// mutable slices (two when the area wraps around the end of the buffer), `len` bytes in all.
/// Nothing is visible to the consumer until [`Grant::commit`].
pub struct Grant<'a, const N: usize> {
    ring: &'a ExtLogRing<N>,
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
    /// wraps around the end of the buffer. Their contents are not defined until written: treat
    /// them as write-only.
    pub fn slices(&mut self) -> (&mut [u8], &mut [u8]) {
        let start = self.head as usize & (N - 1);
        let len = self.len as usize;
        let first = len.min(N - start);
        let base = self.ring.base();
        // SAFETY: the ring is bound (a producer exists only after the bind), so `base` points at
        // the `N` bytes of the `PsramBuf` that `bind` took by `&'static mut`, which nothing else
        // accesses. `start < N` and `first <= N - start`, so the first slice lies inside the
        // buffer; the second starts at its beginning and has `len - first <= start` bytes, so the
        // two do not overlap each other, and both lie in the free area, which only this grant (it
        // borrows the unique producer) may access until `commit` publishes it.
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

/// The one consumer of an [`ExtLogRing`], from [`ExtLogRing::take_consumer`]: `Send`, not
/// `Clone`. Every method is wait-free except [`LogConsumer::take_stats`], which retries a bounded
/// number of times.
pub struct LogConsumer<'a, const N: usize> {
    ring: &'a ExtLogRing<N>,
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
    /// they wrap around the end of the buffer. They stay valid until `consume`; bytes committed
    /// after the call are not in them.
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
        // SAFETY: the ring is bound (a consumer exists only after the bind), so `base` points at
        // the `N` bytes of the bound `PsramBuf`. `n` is at most the committed bytes
        // `[tail, head)`, which the `Acquire` load of `head` made visible and which the producer
        // does not write until `consume` releases them (and `consume` needs `&mut self`, so not
        // while these slices live): only bytes the producer wrote are read, never the buffer's
        // undefined contents. `start < N` and `first <= N - start`; the second slice starts at
        // the beginning of the buffer and has `n - first <= start` bytes: the two slices lie
        // inside the buffer and do not overlap.
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
    /// window begins; exactly as `LogConsumer::take_stats` of `LogRing`.
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
    use crate::ring_contract::Rng;
    use std::boxed::Box;
    use std::thread;
    use std::time::{Duration, Instant};
    use std::vec::Vec;

    fn ready() -> PsramReady {
        // SAFETY: a host test, there is no PSRAM to wait for.
        unsafe { PsramReady::new() }
    }

    fn leaked<const N: usize>() -> &'static mut PsramBuf<N> {
        Box::leak(Box::new(PsramBuf::<N>::new()))
    }

    /// A leaked buffer with every byte `fill`, and a raw pointer to inspect it with after the bind.
    fn filled<const N: usize>(fill: u8) -> (&'static mut PsramBuf<N>, *const u8) {
        let buf = leaked::<N>();
        let p = buf.0.get().cast::<u8>();
        // SAFETY: `p` points at the `N` bytes of `buf`, which nothing else uses yet.
        unsafe { p.write_bytes(fill, N) };
        (buf, p.cast_const())
    }

    #[test]
    fn unbound_take_returns_none_and_does_not_use_up_the_handles() {
        let ring = ExtLogRing::<16>::new();
        assert!(!ring.is_bound());
        assert!(ring.take_producer().is_none());
        assert!(ring.take_consumer().is_none());
        assert!(ring.take_producer().is_none(), "still none");
        assert_eq!((ring.readable(), ring.free(), ring.dropped()), (0, 16, 0));
        ring.bind(leaked::<16>(), &ready()).unwrap();
        assert!(ring.is_bound());
        assert!(
            ring.take_producer().is_some(),
            "the failed takes took nothing"
        );
        assert!(ring.take_consumer().is_some());
        assert!(ring.take_producer().is_none());
        assert!(ring.take_consumer().is_none());
    }

    #[test]
    fn second_bind_is_refused_and_changes_nothing() {
        let ring = ExtLogRing::<16>::new();
        let (first, first_ptr) = filled::<16>(0xA5);
        assert_eq!(ring.bind(first, &ready()), Ok(()));
        let mut producer = ring.take_producer().unwrap();
        let consumer = ring.take_consumer().unwrap();
        let (second, second_ptr) = filled::<16>(0x5A);
        assert_eq!(ring.bind(second, &ready()), Err(BindError::AlreadyBound));
        assert!(producer.push(&[1, 2, 3]));
        assert_eq!(consumer.peek(), (&[1, 2, 3][..], &[][..]));
        // SAFETY: both pointers are to leaked buffers; reading while no handle writes.
        unsafe {
            assert_eq!(*first_ptr, 1, "the first buffer is the ring's");
            assert_eq!(*second_ptr, 0x5A, "the second was never written");
        }
    }

    #[test]
    fn indices_wrap_around_u32() {
        let ring = ExtLogRing::<8>::new();
        ring.bind(leaked::<8>(), &ready()).unwrap();
        ring.head.store(u32::MAX - 2, Ordering::Relaxed);
        ring.tail.store(u32::MAX - 2, Ordering::Relaxed);
        let mut producer = ring.take_producer().unwrap();
        let mut consumer = ring.take_consumer().unwrap();
        for round in 0..8u8 {
            assert!(producer.push(&[round; 5]));
            let (a, b) = consumer.peek();
            assert_eq!([a, b].concat(), [round; 5]);
            assert_eq!(consumer.consume(5), 5);
        }
        assert_eq!(ring.dropped(), 0);
    }

    #[test]
    fn dropped_counter_saturates() {
        let ring = ExtLogRing::<4>::new();
        ring.bind(leaked::<4>(), &ready()).unwrap();
        ring.dropped.store(u32::MAX - 1, Ordering::Relaxed);
        let mut producer = ring.take_producer().unwrap();
        for _ in 0..4 {
            assert!(!producer.push(&[0; 5]));
        }
        assert_eq!(ring.dropped(), u32::MAX);
    }

    /// `bind` writes zeroes over all `N` bytes of a buffer that held anything, before the ring
    /// publishes it (so no `&mut [u8]` of a grant covers uninitialised bytes), and a refused
    /// second bind writes nothing.
    #[test]
    fn bind_zeroes_the_whole_buffer_and_a_refused_bind_writes_nothing() {
        const N: usize = 4096;
        let ring = ExtLogRing::<N>::new();
        let (first, first_ptr) = filled::<N>(0xA5);
        let (second, second_ptr) = filled::<N>(0x5A);
        assert_eq!(ring.bind(first, &ready()), Ok(()));
        assert_eq!(ring.bind(second, &ready()), Err(BindError::AlreadyBound));
        // SAFETY: both pointers are to leaked buffers of `N` bytes; no handle exists to write.
        unsafe {
            assert!(
                core::slice::from_raw_parts(first_ptr, N)
                    .iter()
                    .all(|&b| b == 0),
                "the bound buffer is all zero"
            );
            assert!(
                core::slice::from_raw_parts(second_ptr, N)
                    .iter()
                    .all(|&b| b == 0x5A),
                "the refused buffer is untouched"
            );
        }
    }

    /// The buffer holds no initial value the ring may rely on: after the bind (which zeroes it) the
    /// test fills it with a sentinel again, as stale bytes of an earlier lap would be, and whatever
    /// the consumer is given, over many laps and wraps, is only bytes the producer wrote (a
    /// sentinel byte never appears in the output, the output equals the input), and the producer
    /// writes nothing outside its grants (the untouched tail of a fresh buffer stays sentinel).
    /// Before the zeroing in `bind` the sentinel was the buffer's whole initial content; it now
    /// stays a regression for the read side.
    #[test]
    fn consumer_reads_only_what_the_producer_wrote_never_the_sentinel() {
        const SENTINEL: u8 = 0xA5;
        const N: usize = 64;
        let ring = ExtLogRing::<N>::new();
        let (buf, base) = filled::<N>(SENTINEL);
        ring.bind(buf, &ready()).unwrap();
        // SAFETY: `base` is the bound buffer's first byte, `N` bytes; no handle exists yet.
        unsafe { base.cast_mut().write_bytes(SENTINEL, N) };
        let mut producer = ring.take_producer().unwrap();
        let mut consumer = ring.take_consumer().unwrap();
        assert_eq!(
            consumer.peek(),
            (&[][..], &[][..]),
            "empty: no bytes at all"
        );

        // Fresh buffer: 10 bytes in, only those 10 are written.
        let first: Vec<u8> = (1..=10).collect();
        assert!(producer.push(&first));
        // SAFETY: `base` is the bound buffer; only a read while no handle is writing.
        let outside = unsafe { core::slice::from_raw_parts(base.add(10), N - 10) };
        assert!(
            outside.iter().all(|&b| b == SENTINEL),
            "wrote outside the grant"
        );
        assert_eq!(consumer.peek(), (&first[..], &[][..]));
        assert_eq!(consumer.consume(first.len()), first.len());

        // Many laps with random record and read sizes; no byte of the output is a sentinel.
        let mut rng = Rng(0xFEED);
        let (mut sent, mut got): (Vec<u8>, Vec<u8>) = (Vec::new(), Vec::new());
        let mut next = 0u8;
        for _ in 0..200_000 {
            if rng.below(2) == 0 {
                let len = rng.below(40);
                let record: Vec<u8> = (0..len)
                    .map(|_| {
                        next = (next + 1) % 0x7F;
                        next
                    })
                    .collect();
                if producer.push(&record) {
                    sent.extend(&record);
                }
            } else {
                let (a, b) = consumer.peek_up_to(rng.below(N + 8));
                assert!(
                    !a.contains(&SENTINEL) && !b.contains(&SENTINEL),
                    "sentinel read"
                );
                got.extend_from_slice(a);
                got.extend_from_slice(b);
                let n = a.len() + b.len();
                consumer.consume(n);
            }
            assert_eq!(got.len() + consumer.readable(), sent.len());
        }
        let (a, b) = consumer.peek();
        got.extend_from_slice(a);
        got.extend_from_slice(b);
        assert_eq!(got, sent);
        assert!(sent.len() > 100 * N, "many laps: {}", sent.len());
    }

    /// 10 million bytes in records of 1 to 200 bytes, one thread producing and one consuming, the
    /// output byte for byte the input, no drop.
    #[test]
    fn ten_million_bytes_two_threads_byte_identical() {
        const TOTAL: u64 = 10_000_000;
        const N: usize = 4096;
        const DEADLINE: Duration = Duration::from_secs(120);
        fn pattern(i: u64) -> u8 {
            (i.wrapping_mul(0x9E37_79B9_7F4A_7C15) >> 56) as u8
        }
        let ring = ExtLogRing::<N>::new();
        ring.bind(leaked::<N>(), &ready()).unwrap();
        let mut producer = ring.take_producer().unwrap();
        let mut consumer = ring.take_consumer().unwrap();
        let start = Instant::now();
        thread::scope(|s| {
            s.spawn(|| {
                let mut rng = Rng(7);
                let mut at = 0u64;
                while at < TOTAL {
                    let len = (1 + rng.below(200)).min((TOTAL - at) as usize);
                    while producer.free() < len {
                        assert!(start.elapsed() < DEADLINE, "producer: time limit");
                        core::hint::spin_loop();
                    }
                    if at.is_multiple_of(2) {
                        let record: Vec<u8> = (0..len).map(|i| pattern(at + i as u64)).collect();
                        assert!(producer.push(&record));
                    } else {
                        let mut grant = producer.reserve(len).unwrap();
                        let (a, b) = grant.slices();
                        for (i, slot) in a.iter_mut().chain(b.iter_mut()).enumerate() {
                            *slot = pattern(at + i as u64);
                        }
                        grant.commit();
                    }
                    at += len as u64;
                }
            });
            let mut rng = Rng(9);
            let mut at = 0u64;
            while at < TOTAL {
                assert!(start.elapsed() < DEADLINE, "consumer: time limit");
                let (a, b) = consumer.peek_up_to(1 + rng.below(700));
                let n = a.len() + b.len();
                for &byte in a.iter().chain(b) {
                    assert_eq!(byte, pattern(at), "byte {at}");
                    at += 1;
                }
                consumer.consume(n);
                if n == 0 {
                    core::hint::spin_loop();
                }
            }
        });
        assert_eq!(ring.dropped(), 0);
        assert_eq!(ring.readable(), 0);
        std::println!("10 MB two threads: {:?}", start.elapsed());
    }
}
