//! The GNSS receiver: Quectel LC76G on UART1 (GPIO24 = the board's TX, GPIO25 = RX, 115200 8N1), a
//! core-0 device, a buffered UART read in thread mode without DMA. The `lc76g` driver is used as it
//! is. At every start, restart and apply (a `GPS_RATE_MS` write) the firmware sends the receiver its
//! fix interval and message set, exactly the list of `fw_settings::gnss::commands` (version query,
//! interval 1000 ms, the ten sentence rates, EPE off, then the interval of `GPS_RATE_MS` if not 1000 ms, held
//! until `fw_settings::gnss::FIX_INTERVAL_SETTLE_MS` after the first interval's acknowledgement: sooner, a
//! receiver whose rate the first one changed answers "being processed" and restarts by itself):
//! never `init` (it writes the navigation mode), PPS, a restart or a save, so the settings are volatile
//! and the receiver's stored configuration does not change. The stream is then GGA, RMC and GST every
//! fix and GSA at most once per second. A configuration that fails (a refusal, no acknowledgement, the
//! whole list not done within T) is a WARNING `GNSS: rate not applied (<reason>)`; `start` and
//! `restart` still return Ok (the receiver keeps sending at the rate it has), `apply` returns a bus
//! error. After every failure the whole list is sent again `fw_settings::gnss::CONFIGURE_RETRY_MS` later,
//! until a try succeeds (a receiver just powered on does not complete a fix-interval change): the adapter
//! raises the row's APPLY request, so a try runs in the task's loop and is never cut by the data-ready
//! deadline. Only a first try's failure is the WARNING; a retry that succeeds is the INFO
//! `GNSS: rate applied on try <n>`. P stays 1 s and T 2.5 s whatever the rate. PPS (GPIO5) is not read.
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
//!
//! A fix takes the deviations of its epoch's GST (the same UTC time; `fw_core::sample::GstOrder`): a GST
//! that came before the GGA + RMC pair at once. A fix whose GST has not come goes out at once without
//! deviations, unless the stream showed that GST follows the pair (its own GST was the sentence right after
//! an earlier pair); then it waits for the one sentence after the pair and goes without deviations if that
//! is not its GST, which also forgets the order. So when GST comes after the pair, the first fix after a
//! start, a configuration or a flush has no deviations; the wait is one sentence (about 11 ms at 115200
//! baud for a 128-byte sentence) and never a fix interval, except a single epoch if GST stops arriving.
//! The order is forgotten at every configuration and flush; a fix waiting when the read is dropped is
//! cleared by `recover`, `restart` and `apply`.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::yield_now;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_24, PIN_25, UART1};
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Duration, Instant, Timer, with_deadline, with_timeout};
use embedded_io_async::Read;
use fw_core::health::{ErrorKind, Limits};
use fw_core::sample::{
    GnssReading, GnssSample, GstOrder, Publish, UtcDate, UtcTime, horizontal_std_mm,
};
use fw_core::xcore::TripleWriter;
use fw_settings::gnss::{CONFIGURE_RETRY_MS, Retry, RetryReport};
use lc76g::aux_sentences::Gst;
use lc76g::sentence::{Date, FixType, Quality, Time};
use lc76g::{Command, Error, Fix, Lc76g, Message};
use mavlink_device::dialect::{GPS_RAW_INT_DATA, GpsFixType, MavSeverity};
use static_cell::StaticCell;

use crate::device::{
    AdapterCounters, DeviceAdapter, GNSS, Pending, Reading, Sample, StartFailure, VerifyError,
    run_device,
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
/// The UART's receive buffer (the driver's README: 4096 bytes) and its transmit buffer, which holds the
/// longest command sent (34 B).
const RX_BUFFER: usize = 4096;
const TX_BUFFER: usize = 64;
/// `recover` empties the UART's buffer: reads until one finds it empty for this long ...
const DRAIN_IDLE: Duration = Duration::from_millis(5);
/// ... or at most this many reads.
const DRAIN_READS: u32 = 128;
/// The one deadline of the whole configuration list: the row's T.
const CONFIGURE_DEADLINE: Duration = Duration::from_micros(DEADLINE_US as u64);

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

static TOTALS: CriticalSectionMutex<Cell<Totals>> = CriticalSectionMutex::new(Cell::new(Totals {
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
static SENTENCES_AT_STAT: CriticalSectionMutex<Cell<u32>> = CriticalSectionMutex::new(Cell::new(0));
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
    /// The fix `data_ready` ended with and its GST deviations, for the `acquire` that follows.
    ready: Option<(Fix, [Option<u32>; 3])>,
    /// A fix whose GST did not come before it: it waits for the next sentence.
    waiting: Option<Fix>,
    /// The latest GST read.
    gst: Option<Gst>,
    /// Whether GST follows the GGA + RMC pair, as the stream showed it.
    order: GstOrder,
    /// The error `data_ready` ended with, for the `acquire` that follows.
    failed: Option<ErrorKind>,
    calls: u32,
    /// The writer of `crate::flight::FLIGHT_GNSS` (`None` if it was taken already: nothing is written).
    flight: Option<TripleWriter<'static, GnssSample>>,
    /// When a failed configuration is tried again.
    retry: Retry,
    /// The APPLY request of a due retry is raised and not yet carried out.
    retry_raised: bool,
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
            TX_BUF.init_with(|| [0; TX_BUFFER]),
            RX_BUF.init_with(|| [0; RX_BUFFER]),
            uart::Config::default(),
        );
        defmt::info!(
            "[INFO] GNSS: LC76G on UART1, TX GPIO24 RX GPIO25, 115200 8N1, fix interval and messages sent at every start (never saved), RX buffer {=usize} B, nominal period {=u32} us, deadline {=u32} us, PPS not read",
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
            waiting: None,
            gst: None,
            order: GstOrder::default(),
            failed: None,
            calls: 0,
            flight: crate::flight::FLIGHT_GNSS.take_writer(),
            retry: Retry::default(),
            retry_raised: false,
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
        self.waiting = None;
        self.gst = None;
        self.order.reset();
        self.failed = None;
        self.totals.resyncs = self.totals.resyncs.saturating_add(1);
        self.totals.drained = self.totals.drained.saturating_add(drained);
        defmt::info!(
            "[INFO] GNSS: UART flushed ({=u32} bytes thrown away), driver rebuilt",
            drained
        );
        self.publish();
    }

    /// Sends the configuration of the stored `GPS_RATE_MS`: `fw_settings::gnss::commands` in order, a
    /// fix interval held until `fw_settings::gnss::not_before_ms`, stopping at the first failure, the whole
    /// list under one deadline T. A failure is a WARNING
    /// (STATUSTEXT and defmt) and returned. A fix held or waiting from before is dropped: it is older
    /// than the fixes the driver discarded during the commands.
    /// `retry`: a try the retry schedule raised (its failure is not reported again).
    async fn configure(&mut self, retry: bool) -> Result<(), &'static str> {
        self.calls = self.calls.wrapping_add(1);
        self.retry_raised = false;
        let tries = self.retry.begin(retry);
        let begun = Instant::now();
        let result = self.send_configuration(begun, tries).await;
        self.ready = None;
        self.waiting = None;
        self.order.reset();
        if let Some(gps) = &self.gps {
            self.fixes_taken = gps.stats().fixes;
        }
        self.publish();
        let report = self.retry.end(result.is_ok(), Instant::now().as_millis());
        if let Err(reason) = result {
            defmt::warn!(
                "[INFO] GNSS: rate not applied ({=str}) after {=u64} ms, try {=u32}, the next in {=u64} ms",
                reason,
                begun.elapsed().as_millis(),
                tries,
                CONFIGURE_RETRY_MS
            );
        }
        match (report, result) {
            (RetryReport::Warn, Err(reason)) => crate::text_fmt(
                MavSeverity::MAV_SEVERITY_WARNING,
                format_args!("GNSS: rate not applied ({})", reason),
            ),
            (RetryReport::Applied { tries }, _) => crate::text_fmt(
                MavSeverity::MAV_SEVERITY_INFO,
                format_args!("GNSS: rate applied on try {}", tries),
            ),
            _ => {}
        }
        result
    }

    async fn send_configuration(&mut self, begun: Instant, tries: u32) -> Result<(), &'static str> {
        let ms = crate::store::gnss_rate_ms().map_err(|reason| {
            defmt::warn!("[INFO] GNSS: stored GPS_RATE_MS refused: {=str}", reason);
            "stored value refused"
        })?;
        let config = fw_settings::gnss::config(ms);
        if !config.is_valid() {
            return Err("configuration invalid");
        }
        let gps = self.gps.as_mut().ok_or("no driver")?;
        let list = fw_settings::gnss::commands(&config);
        let commands = list.as_slice();
        let mut done = 0usize;
        let mut version = None;
        let mut interval_acked: Option<Instant> = None;
        let outcome = with_deadline(begun + CONFIGURE_DEADLINE, async {
            for command in commands {
                if let Some(t) =
                    fw_settings::gnss::not_before_ms(command, interval_acked.map(|t| t.as_millis()))
                {
                    Timer::at(Instant::from_millis(t)).await;
                }
                match command {
                    Command::QueryVersion => version = Some(gps.version().await?),
                    _ => gps.command(command).await?,
                }
                if matches!(command, Command::SetFixInterval { .. }) {
                    interval_acked = Some(Instant::now());
                }
                done += 1;
            }
            Ok::<(), Error<uart::Error>>(())
        })
        .await;
        match outcome {
            Ok(Ok(())) => {
                defmt::info!(
                    "[INFO] GNSS: configured: {=u16} ms, GSA every {=u8}, GST every fix, other sentences off, EPE off, volatile (not saved), {=str}, {=u64} ms, try {=u32}",
                    ms,
                    config.rates.gsa,
                    version.as_ref().map_or("version unknown", |v| v.version()),
                    begun.elapsed().as_millis(),
                    tries
                );
                Ok(())
            }
            Ok(Err(error)) => {
                defmt::warn!(
                    "[INFO] GNSS: configuration stopped at command {=usize} of {=usize}: {}",
                    done + 1,
                    commands.len(),
                    error
                );
                Err(match error {
                    Error::NoAck => "no acknowledgement",
                    Error::Rejected { .. } => "refused by the receiver",
                    Error::Uart(_) => "UART error",
                    Error::Eof => "line closed",
                    Error::InvalidConfig | Error::InvalidCommand => "invalid command",
                })
            }
            Err(_) => {
                defmt::warn!(
                    "[INFO] GNSS: configuration not done in {=u64} ms, at command {=usize} of {=usize}",
                    CONFIGURE_DEADLINE.as_millis(),
                    done + 1,
                    commands.len()
                );
                Err("no answer in 2.5 s")
            }
        }
    }
}

fn utc(time: Option<Time>) -> Option<UtcTime> {
    time.map(|t| UtcTime {
        hour: t.hour,
        minute: t.minute,
        second: t.second,
        millis: t.millis,
    })
}

fn utc_date(date: Option<Date>) -> Option<UtcDate> {
    date.map(|d| UtcDate {
        year: d.year,
        month: d.month,
        day: d.day,
    })
}

/// A GST's UTC time and deviations `[lat, lon, alt]`, mm.
fn gst_std(gst: &Gst) -> (Option<UtcTime>, [Option<u32>; 3]) {
    (
        utc(gst.time),
        [gst.lat_std_mm, gst.lon_std_mm, gst.alt_std_mm],
    )
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

fn reading(fix: &Fix, [lat_std_mm, lon_std_mm, alt_std_mm]: [Option<u32>; 3]) -> Reading {
    let fix_type = fix_type(fix);
    // Without a position the coordinates are not reported, whatever the receiver put in the sentences.
    let position = fix_type >= 2;
    Reading::Gnss(GnssReading {
        fix_type,
        lat_e7: fix.lat_e7.filter(|_| position),
        lon_e7: fix.lon_e7.filter(|_| position),
        alt_mm: fix.alt_mm.filter(|_| position),
        speed_mm_s: fix.speed_mm_s.filter(|_| position),
        course_cdeg: fix.course_cdeg.filter(|_| position),
        sats_used: fix.sats_used,
        hdop_x100: fix.hdop_x100,
        vdop_x100: fix.vdop_x100,
        time: utc(fix.time),
        date: utc_date(fix.date),
        lat_std_mm,
        lon_std_mm,
        alt_std_mm,
        vel_ned_mm_s: None,
    })
}

/// `GPS_RAW_INT` of a sample (`Reading::Gnss`) taken at board time `t_us`. With no fix: `fix_type` 1
/// (NO_FIX), `lat`, `lon` and `alt` 0 (the dialect defines no other convention), `vel` and `cog`
/// UINT16_MAX unless known; `eph`/`epv` are HDOP/VDOP x 100 whenever the receiver says them
/// (UINT16_MAX when unknown); `satellites_visible` carries the satellites USED (the GGA's count; the
/// GSV's count of visible satellites is not read), 255 when unknown. `time_usec` is the board time.
/// `h_acc` is sqrt(lat^2 + lon^2) and `v_acc` the altitude of the GST's deviations, mm, 0 when not
/// available.
pub fn raw_int(reading: &Reading, t_us: u64) -> Option<GPS_RAW_INT_DATA> {
    let Reading::Gnss(GnssReading {
        fix_type,
        lat_e7,
        lon_e7,
        alt_mm,
        speed_mm_s,
        course_cdeg,
        sats_used,
        hdop_x100,
        vdop_x100,
        lat_std_mm,
        lon_std_mm,
        alt_std_mm,
        ..
    }) = *reading
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
        h_acc: horizontal_std_mm(lat_std_mm, lon_std_mm).unwrap_or(0),
        v_acc: alt_std_mm.unwrap_or(0),
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
        Some(Reading::Gnss(GnssReading {
            fix_type,
            sats_used,
            hdop_x100,
            time,
            ..
        })) => (fix_type, sats_used, hdop_x100, time),
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
        crate::device::Utc(utc),
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
    /// already holds (completed while `start` read the first sentence) is handed out at once. Whether
    /// the fix goes now (with or without its GST's deviations) or waits one sentence is
    /// `GstOrder`'s (see the module comment).
    async fn data_ready(&mut self) {
        loop {
            if !self.retry_raised && self.retry.due(Instant::now().as_millis()) {
                self.retry_raised = true;
                defmt::info!("[INFO] GNSS: configuration retry due: APPLY raised");
                GNSS.request_apply();
            }
            let Some(gps) = self.gps.as_mut() else {
                return core::future::pending().await;
            };
            if self.waiting.is_none()
                && gps.stats().fixes != self.fixes_taken
                && let Ok(fix) = gps.read_fix().await
            {
                self.fixes_taken = gps.stats().fixes;
                match self
                    .order
                    .pair(utc(fix.time), self.gst.as_ref().map(gst_std))
                {
                    Publish::Now(std_mm) => {
                        self.ready = Some((fix, std_mm));
                        self.publish();
                        return;
                    }
                    Publish::Wait => {
                        self.waiting = Some(fix);
                        continue;
                    }
                }
            }
            match gps.read().await {
                Ok(message) => {
                    self.sentence(&message);
                    let gst = match message {
                        Message::Gst(gst) => Some(gst),
                        _ => None,
                    };
                    if gst.is_some() {
                        self.gst = gst;
                    }
                    if let Some(std_mm) = self.order.sentence(gst.as_ref().map(gst_std))
                        && let Some(fix) = self.waiting.take()
                    {
                        self.ready = Some((fix, std_mm));
                        return;
                    }
                }
                Err(Error::Uart(e)) => {
                    self.uart_error(e);
                    self.publish();
                    yield_now().await;
                }
                Err(_) => {
                    // The UART returned zero bytes: the line is closed.
                    self.waiting = None;
                    self.failed = Some(ErrorKind::Bus);
                    return;
                }
            }
        }
    }

    /// Waits for the first valid sentence (any type) for up to [`START_WAIT`] (no identity query: a
    /// valid NMEA sentence is the receiver's answer), then sends the configuration. A configuration
    /// failure is reported and is not a start failure.
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
                    let _ = self.configure(false).await;
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
            .map(|(fix, sigmas)| reading(&fix, sigmas))
            .ok_or(ErrorKind::Bus)
    }

    /// There is no status to read: after a deadline miss nothing is pending.
    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        Ok(Pending::Nothing)
    }

    /// The configuration is not read back: a query every second would discard fixes.
    async fn verify(&mut self) -> Result<(), VerifyError> {
        Ok(())
    }

    /// Sends the configuration of the stored `GPS_RATE_MS`; a failure is one bus error, except for a retry
    /// the adapter raised itself (a failed retry is only scheduled again).
    async fn apply(&mut self) -> Result<(), ErrorKind> {
        let retry = core::mem::take(&mut self.retry_raised);
        let result = self.configure(retry).await;
        if retry {
            return Ok(());
        }
        result.map_err(|_| ErrorKind::Bus)
    }

    async fn recover(&mut self) -> Result<(), ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        self.resync().await;
        Ok(())
    }

    /// `recover` (a UART has no bus clear), then the configuration. A configuration failure is
    /// reported and is not a restart failure.
    async fn restart(&mut self) -> Result<(), ErrorKind> {
        self.recover().await?;
        let _ = self.configure(false).await;
        Ok(())
    }

    fn counters(&self) -> AdapterCounters {
        AdapterCounters { calls: self.calls }
    }

    fn to_flight(&mut self, sample: &Sample) {
        if let (Reading::Gnss(r), Some(w)) = (sample.reading, &mut self.flight) {
            w.write(r.sample(sample.t_us, sample.seq));
        }
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

    fn to_flight(&mut self, sample: &Sample) {
        self.inner.to_flight(sample);
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
