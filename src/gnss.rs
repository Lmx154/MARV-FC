//! The GNSS receiver: Quectel LC76G on UART1 (GPIO24 = the board's TX, GPIO25 = RX, 115200 8N1), a
//! core-0 device, a buffered UART read in thread mode without DMA. The `lc76g` driver is used as it
//! is; the firmware only reads: nothing is ever sent to the receiver (no `init`, no setting, no save),
//! so its stored configuration cannot change. The receiver's default stream, one epoch per second
//! (GGA, RMC, GSA, GSV, GLL, VTG), is what the row sees. PPS (GPIO5) is not read.
//!
//! "Ok" means valid sentences keep arriving: one sample is one fix of the driver (the GGA and RMC
//! of one epoch), WHETHER OR NOT it holds a position (indoors it does not). 2.5 s without a fix
//! (`DEADLINE_US`) is one error event (`ErrorKind::Timeout`); a checksum error, a malformed sentence
//! and a UART error (overrun, framing, break, parity) are counted and are not error events by
//! themselves: a broken stream shows as that silence.
//!
//! The driver's read is droppable (`lc76g` README, "Dropped reads"; its test
//! `dropping_a_read_future_loses_nothing`, `src/tests.rs:1013`; `Lc76g::next_sentence`, `src/lib.rs`
//! keeps the bytes and the framer in the driver and awaits only one `UartRx::read`, which hands out
//! its bytes in the poll that completes it). So the data-ready wait IS the read of the next fix: it
//! may be dropped by the timeout or the wake signal at any await and nothing is lost. Nothing needs
//! `recover` after a drop.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::yield_now;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_24, PIN_25, UART1};
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Duration, Instant, with_deadline, with_timeout};
use embedded_io_async::Read;
use fw_core::health::{ErrorKind, Limits};
use lc76g::sentence::{FixType, Quality};
use lc76g::{Error, Fix, Lc76g, Message};
use mavlink_device::dialect::{GPS_RAW_INT_DATA, GpsFixType};
use static_cell::StaticCell;

use crate::device::{
    AdapterCounters, DeviceAdapter, GNSS, Pending, Reading, StartFailure, VerifyError, run_device,
};

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

/// The nominal sample period P: one fix per second (the receiver's default).
pub const PERIOD_US: u32 = 1_000_000;
/// The deadline T of the data-ready wait: 2.5 s of silence is one error event (Betaflight's figure),
/// instead of the 3 P the other rows use.
pub const DEADLINE_US: u32 = 2_500_000;
/// How long `start` waits for the first valid sentence. The README of the driver gives no start time
/// of the module (the bench had it running); 5 s covers a receiver that is still booting. The
/// health machine's start retries (100, 300, 1000 ms) are too short for that, so the wait is inside
/// `start`: three retries after it make the longest start 4 x 5 s + 1.4 s before the row is Absent.
const START_WAIT: Duration = Duration::from_secs(5);
/// The UART's receive buffer (the driver's README: 4096 bytes) and its transmit buffer (unused: nothing
/// is sent; the pin is driven so that the receiver's input does not float).
const RX_BUFFER: usize = 4096;
const TX_BUFFER: usize = 64;
/// `recover` empties the UART's buffer: reads until one finds it empty for this long ...
const DRAIN_IDLE: Duration = Duration::from_millis(5);
/// ... or at most this many reads.
const DRAIN_READS: u32 = 128;

/// What the GNSS row counts besides the health counters, summed over the driver instances (a
/// `recover` builds a new driver, which starts its own counts at 0). Read by the supervisor's `[STAT]`.
#[derive(Clone, Copy, Default)]
pub struct Totals {
    /// Valid sentences.
    pub sentences: u32,
    /// Checksum errors, sentences cut off by the next `$`.
    pub checksum_errors: u32,
    /// Sentences over the driver's length limit.
    pub overflows: u32,
    /// Valid checksum, fields that do not fit the layout.
    pub malformed: u32,
    /// Epochs whose GGA and RMC times did not match.
    pub epochs_dropped: u32,
    /// Fixes (GGA + RMC pairs).
    pub fixes: u32,
    /// UART receive errors seen by a read.
    pub overrun: u32,
    pub framing: u32,
    pub brk: u32,
    pub parity: u32,
    /// `recover` or `restart` calls.
    pub resyncs: u32,
    /// Bytes thrown away by them.
    pub drained: u32,
    /// Board time of the last valid sentence, us.
    pub last_sentence_us: Option<u64>,
    /// The sentences are swallowed by the `inject-gnss-silent` wrapper.
    pub silenced: bool,
}

static TOTALS: Mutex<CriticalSectionRawMutex, Cell<Totals>> = Mutex::new(Cell::new(Totals {
    sentences: 0,
    checksum_errors: 0,
    overflows: 0,
    malformed: 0,
    epochs_dropped: 0,
    fixes: 0,
    overrun: 0,
    framing: 0,
    brk: 0,
    parity: 0,
    resyncs: 0,
    drained: 0,
    last_sentence_us: None,
    silenced: false,
}));
static SENTENCES_AT_STAT: Mutex<CriticalSectionRawMutex, Cell<u32>> = Mutex::new(Cell::new(0));
static SILENCED: AtomicBool = AtomicBool::new(false);

pub fn totals() -> Totals {
    TOTALS.lock(Cell::get)
}

pub struct GnssAdapter {
    gps: Option<Lc76g<BufferedUart>>,
    /// The counts of the driver instances that were replaced.
    carried: Totals,
    totals: Totals,
    /// `Stats::fixes` of the driver when the last fix was taken from it.
    fixes_taken: u32,
    /// The fix `data_ready` ended with, for the `acquire` that follows.
    ready: Option<Fix>,
    /// The error `data_ready` ended with, for the `acquire` that follows.
    failed: Option<ErrorKind>,
    calls: u32,
}

impl GnssAdapter {
    pub fn new(
        uart1: Peri<'static, UART1>,
        tx: Peri<'static, PIN_24>,
        rx: Peri<'static, PIN_25>,
    ) -> Self {
        static RX_BUF: StaticCell<[u8; RX_BUFFER]> = StaticCell::new();
        static TX_BUF: StaticCell<[u8; TX_BUFFER]> = StaticCell::new();
        let uart = BufferedUart::new(
            uart1,
            tx,
            rx,
            Irqs,
            TX_BUF.init([0; TX_BUFFER]),
            RX_BUF.init([0; RX_BUFFER]),
            uart::Config::default(),
        );
        defmt::info!(
            "[INFO] GNSS: LC76G on UART1, TX GPIO24 RX GPIO25, 115200 8N1, receive only (nothing is sent), RX buffer {=usize} B, nominal period {=u32} us, deadline {=u32} us, PPS not read",
            RX_BUFFER,
            PERIOD_US,
            DEADLINE_US
        );
        Self {
            gps: Some(Lc76g::new(uart)),
            carried: Totals::default(),
            totals: Totals::default(),
            fixes_taken: 0,
            ready: None,
            failed: None,
            calls: 0,
        }
    }

    /// Publishes the counts: the replaced drivers' plus the live one's.
    fn publish(&mut self) {
        let mut t = self.totals;
        if let Some(gps) = &self.gps {
            let s = gps.stats();
            t.sentences = self.carried.sentences.saturating_add(s.sentences);
            t.checksum_errors = self
                .carried
                .checksum_errors
                .saturating_add(s.checksum_errors);
            t.overflows = self.carried.overflows.saturating_add(s.overflows);
            t.epochs_dropped = self.carried.epochs_dropped.saturating_add(s.epochs_dropped);
            t.fixes = self.carried.fixes.saturating_add(s.fixes);
        }
        t.silenced = SILENCED.load(Ordering::Relaxed);
        self.totals = t;
        TOTALS.lock(|cell| cell.set(t));
    }

    /// Counts a UART receive error (never an error event: a broken stream shows as silence).
    fn uart_error(&mut self, error: uart::Error) {
        let t = &mut self.totals;
        match error {
            uart::Error::Overrun => t.overrun = t.overrun.saturating_add(1),
            uart::Error::Framing => t.framing = t.framing.saturating_add(1),
            uart::Error::Break => t.brk = t.brk.saturating_add(1),
            uart::Error::Parity => t.parity = t.parity.saturating_add(1),
            _ => {}
        }
    }

    fn sentence(&mut self, message: &Message) {
        if matches!(message, Message::Malformed) {
            self.totals.malformed = self.totals.malformed.saturating_add(1);
        }
        self.totals.last_sentence_us = Some(Instant::now().as_micros());
        self.publish();
    }

    /// Flush and resynchronise: a new driver (empty framer, no half sentence, no held fix) over the
    /// UART, after the bytes waiting in its buffer were thrown away. A UART has no bus clear.
    async fn resync(&mut self) {
        let Some(gps) = self.gps.take() else {
            return;
        };
        let s = gps.stats();
        self.carried.sentences = self.carried.sentences.saturating_add(s.sentences);
        self.carried.checksum_errors = self
            .carried
            .checksum_errors
            .saturating_add(s.checksum_errors);
        self.carried.overflows = self.carried.overflows.saturating_add(s.overflows);
        self.carried.epochs_dropped = self.carried.epochs_dropped.saturating_add(s.epochs_dropped);
        self.carried.fixes = self.carried.fixes.saturating_add(s.fixes);
        let mut uart = gps.release();
        let mut buf = [0u8; 64];
        let mut drained = 0u32;
        for _ in 0..DRAIN_READS {
            match with_timeout(DRAIN_IDLE, uart.read(&mut buf)).await {
                Ok(Ok(n)) => drained = drained.saturating_add(n as u32),
                Ok(Err(e)) => self.uart_error(e),
                Err(_) => break,
            }
        }
        self.gps = Some(Lc76g::new(uart));
        self.fixes_taken = 0;
        self.ready = None;
        self.failed = None;
        self.totals.resyncs = self.totals.resyncs.saturating_add(1);
        self.totals.drained = self.totals.drained.saturating_add(drained);
        defmt::info!(
            "[INFO] GNSS: UART flushed ({=u32} bytes thrown away), driver rebuilt",
            drained
        );
        self.publish();
    }
}

/// MAVLink's `GPS_FIX_TYPE` of a fix: 1 NO_FIX unless the RMC status, the GGA quality and the GSA
/// all say there is a position; then 2 (2D), 3 (3D) or 4 (DGPS/SBAS aided 3D).
fn fix_type(fix: &Fix) -> u8 {
    if !fix.valid || fix.quality == Quality::NoFix {
        return 1;
    }
    match fix.fix_type {
        Some(FixType::Fix2d) => 2,
        Some(FixType::Fix3d) if fix.quality == Quality::Differential => 4,
        Some(FixType::Fix3d) => 3,
        Some(FixType::NoFix) | None => 1,
    }
}

fn reading(fix: &Fix) -> Reading {
    let fix_type = fix_type(fix);
    // Without a position the coordinates are not reported, whatever the receiver put in the sentences.
    let position = fix_type >= 2;
    Reading::Gnss {
        fix_type,
        lat_e7: fix.lat_e7.filter(|_| position),
        lon_e7: fix.lon_e7.filter(|_| position),
        alt_mm: fix.alt_mm.filter(|_| position),
        speed_mm_s: fix.speed_mm_s.filter(|_| position),
        course_cdeg: fix.course_cdeg.filter(|_| position),
        sats_used: fix.sats_used,
        hdop_x100: fix.hdop_x100,
        vdop_x100: fix.vdop_x100,
        time: fix.time,
        date: fix.date,
    }
}

/// `GPS_RAW_INT` of a sample (`Reading::Gnss`) taken at board time `t_us`. With no fix: `fix_type` 1
/// (NO_FIX), `lat`, `lon` and `alt` 0 (the dialect defines no other convention), `vel` and `cog`
/// UINT16_MAX unless known; `eph`/`epv` are HDOP/VDOP x 100 whenever the receiver says them
/// (UINT16_MAX when unknown); `satellites_visible` carries the satellites USED (the GGA's count; the
/// GSV's count of visible satellites is not read), 255 when unknown. `time_usec` is the board time.
pub fn raw_int(reading: &Reading, t_us: u64) -> Option<GPS_RAW_INT_DATA> {
    let Reading::Gnss {
        fix_type,
        lat_e7,
        lon_e7,
        alt_mm,
        speed_mm_s,
        course_cdeg,
        sats_used,
        hdop_x100,
        vdop_x100,
        ..
    } = *reading
    else {
        return None;
    };
    Some(GPS_RAW_INT_DATA {
        time_usec: t_us,
        lat: lat_e7.unwrap_or(0),
        lon: lon_e7.unwrap_or(0),
        alt: alt_mm.unwrap_or(0),
        eph: hdop_x100.unwrap_or(u16::MAX),
        epv: vdop_x100.unwrap_or(u16::MAX),
        // mm/s to cm/s
        vel: speed_mm_s.map_or(u16::MAX, |v| u16::try_from(v / 10).unwrap_or(u16::MAX)),
        cog: course_cdeg.unwrap_or(u16::MAX),
        fix_type: match fix_type {
            2 => GpsFixType::GPS_FIX_TYPE_2D_FIX,
            3 => GpsFixType::GPS_FIX_TYPE_3D_FIX,
            4 => GpsFixType::GPS_FIX_TYPE_DGPS,
            _ => GpsFixType::GPS_FIX_TYPE_NO_FIX,
        },
        satellites_visible: sats_used.unwrap_or(u8::MAX),
        ..GPS_RAW_INT_DATA::DEFAULT
    })
}

/// `[STAT]` line of the receiver (the generic line of every row precedes it).
pub fn stat(now: Instant, window_us: u64, latest: Option<crate::device::Sample>) {
    let t = totals();
    let previous = SENTENCES_AT_STAT.lock(|cell| cell.replace(t.sentences));
    let rate = t.sentences.wrapping_sub(previous) as f32 * 1e6 / window_us as f32;
    let age_ms = t
        .last_sentence_us
        .map_or(u64::MAX, |us| now.as_micros().saturating_sub(us) / 1000);
    let (fix_type, sats, hdop, utc) = match latest.map(|s| s.reading) {
        Some(Reading::Gnss {
            fix_type,
            sats_used,
            hdop_x100,
            time,
            ..
        }) => (fix_type, sats_used, hdop_x100, time),
        _ => (0, None, None, None),
    };
    defmt::info!(
        "[STAT] GNSS: sentences {=u32} ({=f32}/s over the last 10 s), fixes {=u32}, checksum errors {=u32}, malformed {=u32}, overflows {=u32}, epochs dropped {=u32}, UART errors overrun {=u32} framing {=u32} break {=u32} parity {=u32}, resyncs {=u32} ({=u32} B), fix type {=u8}, satellites {}, HDOP x100 {}, UTC {}, last sentence {=u64} ms ago{=str}",
        t.sentences,
        rate,
        t.fixes,
        t.checksum_errors,
        t.malformed,
        t.overflows,
        t.epochs_dropped,
        t.overrun,
        t.framing,
        t.brk,
        t.parity,
        t.resyncs,
        t.drained,
        fix_type,
        sats,
        hdop,
        utc,
        age_ms,
        if t.silenced {
            ", SENTENCES SWALLOWED (inject)"
        } else {
            ""
        }
    );
}

impl DeviceAdapter for GnssAdapter {
    /// The read of the next fix: droppable (see the module comment). Sentences on the way are
    /// decoded, counted and consumed; a UART error is counted and the read goes on. A fix the driver
    /// already holds (completed while `start` read the first sentence) is handed out at once.
    async fn data_ready(&mut self) {
        loop {
            let Some(gps) = self.gps.as_mut() else {
                return core::future::pending().await;
            };
            if gps.stats().fixes != self.fixes_taken
                && let Ok(fix) = gps.read_fix().await
            {
                self.fixes_taken = gps.stats().fixes;
                self.ready = Some(fix);
                self.publish();
                return;
            }
            match gps.read().await {
                Ok(message) => self.sentence(&message),
                Err(Error::Uart(e)) => {
                    self.uart_error(e);
                    self.publish();
                    yield_now().await;
                }
                Err(_) => {
                    // The UART returned zero bytes: the line is closed.
                    self.failed = Some(ErrorKind::Bus);
                    return;
                }
            }
        }
    }

    /// Waits for the first valid sentence (any type) for up to [`START_WAIT`]. No identity query: the
    /// firmware sends nothing, and a valid NMEA sentence is the receiver's answer.
    async fn start(&mut self) -> Result<(), StartFailure> {
        self.calls = self.calls.wrapping_add(1);
        let begun = Instant::now();
        let end = begun + START_WAIT;
        loop {
            let Some(gps) = self.gps.as_mut() else {
                return Err(StartFailure { identified: false });
            };
            match with_deadline(end, gps.read()).await {
                Ok(Ok(message)) => {
                    self.sentence(&message);
                    defmt::info!(
                        "[INFO] GNSS: started: the first valid sentence came {=u64} ms after the start, UART errors before it: overrun {=u32} framing {=u32} break {=u32} parity {=u32}",
                        begun.elapsed().as_millis(),
                        self.totals.overrun,
                        self.totals.framing,
                        self.totals.brk,
                        self.totals.parity
                    );
                    return Ok(());
                }
                Ok(Err(Error::Uart(e))) => {
                    self.uart_error(e);
                    self.publish();
                    yield_now().await;
                }
                Ok(Err(_)) => {
                    defmt::warn!("[INFO] GNSS: start failed: the UART is closed");
                    return Err(StartFailure { identified: false });
                }
                Err(_) => {
                    defmt::warn!(
                        "[INFO] GNSS: start failed: no valid sentence in {=u64} ms",
                        START_WAIT.as_millis()
                    );
                    self.publish();
                    return Err(StartFailure { identified: false });
                }
            }
        }
    }

    async fn acquire(&mut self) -> Result<Reading, ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        if let Some(kind) = self.failed.take() {
            return Err(kind);
        }
        self.ready
            .take()
            .map(|fix| reading(&fix))
            .ok_or(ErrorKind::Bus)
    }

    /// There is no status to read: after a deadline miss nothing is pending.
    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        Ok(Pending::Nothing)
    }

    /// Nothing is configured, so there is nothing to check.
    async fn verify(&mut self) -> Result<(), VerifyError> {
        Ok(())
    }

    /// No parameters.
    async fn apply(&mut self) -> Result<(), ErrorKind> {
        Ok(())
    }

    async fn recover(&mut self) -> Result<(), ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        self.resync().await;
        Ok(())
    }

    /// The same as `recover`: a UART has no bus clear and nothing is configured.
    async fn restart(&mut self) -> Result<(), ErrorKind> {
        self.recover().await
    }

    fn counters(&self) -> AdapterCounters {
        AdapterCounters { calls: self.calls }
    }
}

/// Feature `inject-gnss-silent`: 10 s after the row's first good sample the adapter stops seeing
/// sentences (the wrapper swallows the fixes) until a RESTART request has been raised.
#[cfg(feature = "inject-gnss-silent")]
pub struct Silent<A> {
    inner: A,
    first_good: Option<Instant>,
}

#[cfg(feature = "inject-gnss-silent")]
impl<A> Silent<A> {
    const AFTER: Duration = Duration::from_secs(10);

    fn new(inner: A) -> Self {
        defmt::info!(
            "[INFO] inject: GNSS silent armed (sentences swallowed 10 s after the first good sample until a RESTART request)"
        );
        Self {
            inner,
            first_good: None,
        }
    }

    fn silent(&mut self) -> bool {
        let on =
            GNSS.restarts() == 0 && self.first_good.is_some_and(|t| t.elapsed() >= Self::AFTER);
        if on != SILENCED.load(Ordering::Relaxed) {
            SILENCED.store(on, Ordering::Relaxed);
            defmt::info!(
                "[INFO] inject: GNSS sentences {=str} at {=u64} ms",
                if on {
                    "swallowed from"
                } else {
                    "pass again from"
                },
                Instant::now().as_millis()
            );
        }
        on
    }
}

#[cfg(feature = "inject-gnss-silent")]
impl<A: DeviceAdapter> DeviceAdapter for Silent<A> {
    async fn data_ready(&mut self) {
        loop {
            self.inner.data_ready().await;
            if !self.silent() {
                return;
            }
        }
    }

    async fn start(&mut self) -> Result<(), StartFailure> {
        self.inner.start().await
    }

    async fn acquire(&mut self) -> Result<Reading, ErrorKind> {
        let result = self.inner.acquire().await;
        if result.is_ok() && self.first_good.is_none() {
            self.first_good = Some(Instant::now());
        }
        result
    }

    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        self.inner.status().await
    }

    async fn verify(&mut self) -> Result<(), VerifyError> {
        self.inner.verify().await
    }

    async fn apply(&mut self) -> Result<(), ErrorKind> {
        self.inner.apply().await
    }

    async fn recover(&mut self) -> Result<(), ErrorKind> {
        self.inner.recover().await
    }

    async fn restart(&mut self) -> Result<(), ErrorKind> {
        self.inner.restart().await
    }

    fn counters(&self) -> AdapterCounters {
        self.inner.counters()
    }
}

#[cfg(not(feature = "inject-gnss-silent"))]
type Adapter = GnssAdapter;
#[cfg(feature = "inject-gnss-silent")]
type Adapter = Silent<GnssAdapter>;

/// The adapter for this build (with the fault-injection wrapper when `inject-gnss-silent` is on).
pub fn adapter(
    uart1: Peri<'static, UART1>,
    tx: Peri<'static, PIN_24>,
    rx: Peri<'static, PIN_25>,
) -> Adapter {
    wrap(GnssAdapter::new(uart1, tx, rx))
}

#[cfg(not(feature = "inject-gnss-silent"))]
fn wrap(adapter: GnssAdapter) -> Adapter {
    adapter
}
#[cfg(feature = "inject-gnss-silent")]
fn wrap(adapter: GnssAdapter) -> Adapter {
    Silent::new(adapter)
}

#[embassy_executor::task]
pub async fn task(mut adapter: Adapter) -> ! {
    fw_flight::tagged(
        &crate::watchdog::RUNNING0,
        fw_core::watchdog::task::GNSS,
        run_device(&GNSS, &mut adapter, Limits::DESIGN),
    )
    .await
}
