//! The GNSS receiver: u-blox SAM-M10Q on UART1 (GPIO24 = the board's TX, GPIO25 = RX, 115200 8N1 once
//! configured), a core-0 device, a buffered UART read in thread mode without DMA, UBX protocol only through the
//! `sam-m10q` driver (decision-gnss-m10-20261122).
//!
//! Every configuration (start, restart, recover after the deadline, and apply, which a `GPS_RATE_MS` write raises)
//! is the driver's whole `configure`: a baud scan (115200, 9600, 38400, a UBX-MON-VER poll at each), the switch to
//! 115200, one CFG-VALSET to RAM only (UBX only, NMEA off, the measurement period of `GPS_RATE_MS`, one NAV-PVT per
//! epoch, dynamic model 8, the constellation set of the driver) whose ACK is required, and 500 ms of settling:
//! nothing is saved, so the receiver's stored configuration does not change. It takes up to 5.6 s (three 1 s probes,
//! 100 ms, the 1 s acknowledgement, 500 ms); the scan starts at 115200, where the receiver is after an FC reset with it powered,
//! so it is found at once; a receiver at 9600 or 38400 is found after the silent 1 s probes before it. The MON-VER software and hardware
//! strings of the receiver are logged once per successful configure (an INFO text and the RTT line); nothing checks
//! them.
//!
//! A configure that fails (no receiver, a refused or unacknowledged VALSET, a baud switch the receiver did not
//! follow) is a device error: `start` returns `StartFailure` (the row's start retries), `recover`, `restart` and
//! `apply` return `ErrorKind::Bus`, the first try's failure is the WARNING `GNSS: configure failed (<reason>)`, the
//! reason and the counts are on the `[STAT]` line, and a failed apply is tried again
//! `fw_settings::gnss::CONFIGURE_RETRY_MS` later (the adapter raises the row's APPLY request from `data_ready`, so a
//! try runs in the task's loop). A retry that succeeds is the INFO `GNSS: configured on try <n>`.
//!
//! "Ok" means NAV-PVT frames keep arriving: one sample is one NAV-PVT, WHETHER OR NOT it holds a 3D fix (indoors it
//! does not). 2.5 s without one (`DEADLINE_US`) is one error event (`ErrorKind::Timeout`); a bad checksum, an
//! oversize or an unknown frame and a UART error (overrun, framing, break, parity) are counted and are not error
//! events by themselves: a broken stream shows as that silence. A UART error never reaches the driver (it would end
//! a `configure` in the middle of a baud probe, where framing errors are expected): `GnssUart::read` counts it and
//! reads on.
//!
//! The driver's `read_pvt` is droppable (its README: dropping any future loses nothing it already read; the UART's
//! read is cancel safe), so the data-ready wait IS the read of the next NAV-PVT: the timeout or the wake signal may
//! drop it at any await. It returns on the checksum byte of the frame, so the sample's `t_us`, taken by
//! `device::read` right after `data_ready` returns, is the time the frame arrived.

use core::cell::Cell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_futures::yield_now;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIN_24, PIN_25, UART1};
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_sync::blocking_mutex::CriticalSectionMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_io_async::{ErrorType, Read, Write};
use fw_core::health::{ErrorKind, Limits};
use fw_core::sample::{GnssReading, GnssSample, horizontal_std_mm};
use fw_core::xcore::TripleWriter;
use fw_settings::gnss::{CONFIGURE_RETRY_MS, Retry, RetryReport};
use mavlink_device::dialect::{GPS_RAW_INT_DATA, GpsFixType, MavSeverity};
use sam_m10q::{Pvt, Receiver, SetBaud};
use static_cell::StaticCell;

use crate::device::{
    AdapterCounters, DeviceAdapter, GNSS, Pending, Reading, Sample, StartFailure, VerifyError,
    run_device,
};

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
});

/// The nominal sample period P of the row, which sets its stuck threshold (20 P): 1 s, whatever `GPS_RATE_MS`.
pub const PERIOD_US: u32 = 1_000_000;
/// The deadline T of the data-ready wait: 2.5 s of silence is one error event (Betaflight's figure),
/// instead of the 3 P the other rows use.
pub const DEADLINE_US: u32 = 2_500_000;
/// The UART's receive buffer (4096 bytes: 8 s of NAV-PVT frames at 5 Hz) and its transmit buffer; the driver's
/// CFG-VALSET frame (109 bytes) goes through it in pieces.
const RX_BUFFER: usize = 4096;
const TX_BUFFER: usize = 64;
/// While the receiver is not configured (a failed apply), `data_ready` looks at the retry schedule this often.
const RETRY_POLL: Duration = Duration::from_millis(100);
/// `GnssUart::flush` looks at the UART's busy flag this often while the last byte is still leaving.
const BUSY_POLL: Duration = Duration::from_micros(200);

/// UART receive errors counted by `GnssUart::read` (the driver never sees them).
static OVERRUN: AtomicU32 = AtomicU32::new(0);
static FRAMING: AtomicU32 = AtomicU32::new(0);
static BREAK: AtomicU32 = AtomicU32::new(0);
static PARITY: AtomicU32 = AtomicU32::new(0);

fn count(counter: &AtomicU32) {
    let _ = counter.update(Ordering::Relaxed, Ordering::Relaxed, |n| n.saturating_add(1));
}

/// UART1 as the driver wants it: `BufferedUart` with the baud rate settable, receive errors counted and read past,
/// and a `flush` that returns when the last byte has left the UART (the driver changes the baud rate right after
/// it, and `BufferedUart::flush` returns when the software buffer is empty, up to 32 FIFO bytes earlier).
struct GnssUart(BufferedUart);

impl ErrorType for GnssUart {
    type Error = uart::Error;
}

impl Read for GnssUart {
    /// Reads, counting and skipping a receive error (a framing error, a break, ...); a yield between keeps a line
    /// that only errors from starving the executor and lets the driver's deadline be polled. Droppable at the
    /// yield and at the read.
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        loop {
            match self.0.read(buf).await {
                Ok(n) => return Ok(n),
                Err(uart::Error::Overrun) => count(&OVERRUN),
                Err(uart::Error::Framing) => count(&FRAMING),
                Err(uart::Error::Break) => count(&BREAK),
                Err(uart::Error::Parity) => count(&PARITY),
                Err(_) => {}
            }
            yield_now().await;
        }
    }
}

impl Write for GnssUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.0.write(buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.0.flush().await?;
        while self.0.busy() {
            Timer::after(BUSY_POLL).await;
        }
        Ok(())
    }
}

impl SetBaud for GnssUart {
    fn set_baud(&mut self, baud: u32) {
        self.0.set_baudrate(baud);
    }
}

/// What a failed configure was, for the WARNING, the `[STAT]` line and the start failure.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ConfigureError {
    /// The stored `GPS_RATE_MS` is not a period the receiver accepts.
    Stored,
    /// The driver refused the configuration (the period) before sending anything.
    Invalid,
    /// No UBX-MON-VER reply at any baud rate.
    NoReceiver,
    /// No reply after the switch to 115200.
    BaudSwitch,
    /// The CFG-VALSET got no ACK within 1 s.
    NoAck,
    /// The CFG-VALSET got a NAK.
    Rejected,
    /// A UART error.
    Uart,
    /// The UART returned zero bytes.
    Closed,
}

impl ConfigureError {
    /// At most 24 characters: `GNSS: configure failed (` + this + `)` is one 50-character STATUSTEXT.
    fn reason(self) -> &'static str {
        match self {
            Self::Stored => "stored value refused",
            Self::Invalid => "configuration invalid",
            Self::NoReceiver => "no receiver answered",
            Self::BaudSwitch => "baud switch failed",
            Self::NoAck => "no acknowledgement",
            Self::Rejected => "refused (NAK)",
            Self::Uart => "UART error",
            Self::Closed => "line closed",
        }
    }

    /// The receiver answered a MON-VER poll before the failure: the row's `present`.
    fn identified(self) -> bool {
        matches!(self, Self::BaudSwitch | Self::NoAck | Self::Rejected)
    }
}

impl From<sam_m10q::Error<uart::Error>> for ConfigureError {
    fn from(error: sam_m10q::Error<uart::Error>) -> Self {
        match error {
            sam_m10q::Error::Uart(_) => Self::Uart,
            sam_m10q::Error::Eof => Self::Closed,
            sam_m10q::Error::NoReceiver => Self::NoReceiver,
            sam_m10q::Error::BaudSwitchFailed => Self::BaudSwitch,
            sam_m10q::Error::NoAck => Self::NoAck,
            sam_m10q::Error::Rejected => Self::Rejected,
            sam_m10q::Error::NotConfigured | sam_m10q::Error::InvalidConfig => Self::Invalid,
        }
    }
}

/// What the GNSS row counts besides the health counters. Read by the supervisor's `[STAT]`.
#[derive(Clone, Copy)]
pub struct Totals {
    /// Frames with a valid checksum, decoded or not (the driver's).
    pub frames: u32,
    /// Frames with a bad checksum.
    pub checksum_errors: u32,
    /// Frames over the driver's size limit.
    pub oversize: u32,
    /// Valid frames the driver does not decode.
    pub ignored: u32,
    /// NAV-PVT frames read by `data_ready`.
    pub pvt: u32,
    /// Successful configures.
    pub configures: u32,
    /// Failed configures, and how many of them the receiver answered with a NAK.
    pub configure_failures: u32,
    pub naks: u32,
    /// The reason of the latest failed configure, "" before any.
    pub last_error: &'static str,
    /// UART receive errors (counted by `GnssUart::read`).
    pub overrun: u32,
    pub framing: u32,
    pub brk: u32,
    pub parity: u32,
    /// Board time of the latest NAV-PVT, us.
    pub last_pvt_us: Option<u64>,
    /// The frames are swallowed by the `inject-gnss-silent` wrapper.
    pub silenced: bool,
}

impl Totals {
    const ZERO: Self = Self {
        frames: 0,
        checksum_errors: 0,
        oversize: 0,
        ignored: 0,
        pvt: 0,
        configures: 0,
        configure_failures: 0,
        naks: 0,
        last_error: "",
        overrun: 0,
        framing: 0,
        brk: 0,
        parity: 0,
        last_pvt_us: None,
        silenced: false,
    };
}

static TOTALS: CriticalSectionMutex<Cell<Totals>> = CriticalSectionMutex::new(Cell::new(Totals::ZERO));
static PVT_AT_STAT: CriticalSectionMutex<Cell<u32>> = CriticalSectionMutex::new(Cell::new(0));
static SILENCED: AtomicBool = AtomicBool::new(false);

pub fn totals() -> Totals {
    TOTALS.lock(Cell::get)
}

/// The GPS time of week of the latest NAV-PVT (iTOW, ms) and its velocity, which keys it to the reading it belongs to.
#[derive(Clone, Copy)]
struct PvtTow {
    vel_ned_mm_s: Option<[i32; 3]>,
    tow_ms: u32,
}

static PVT_TOW: CriticalSectionMutex<Cell<Option<PvtTow>>> = CriticalSectionMutex::new(Cell::new(None));

/// The iTOW (GPS time of week, ms) of the latest NAV-PVT when its velocity is `vel_ned_mm_s`: what the log's GPV
/// record takes besides the reading (the reading has no time of week).
pub fn pvt_tow_ms(vel_ned_mm_s: [i32; 3]) -> Option<u32> {
    PVT_TOW
        .lock(Cell::get)
        .filter(|p| p.vel_ned_mm_s == Some(vel_ned_mm_s))
        .map(|p| p.tow_ms)
}

pub struct GnssAdapter {
    rx: Receiver<GnssUart>,
    totals: Totals,
    /// The NAV-PVT `data_ready` ended with, for the `acquire` that follows.
    ready: Option<Pvt>,
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
            "[INFO] GNSS: SAM-M10Q on UART1, TX GPIO24 RX GPIO25, 115200 8N1 after the configure (baud scan 115200, 9600, 38400), UBX only, configured at every start (never saved), RX buffer {=usize} B, nominal period {=u32} us, deadline {=u32} us, PPS not read",
            RX_BUFFER,
            PERIOD_US,
            DEADLINE_US
        );
        Self {
            rx: Receiver::new(GnssUart(uart)),
            totals: Totals::ZERO,
            ready: None,
            failed: None,
            calls: 0,
            flight: crate::flight::FLIGHT_GNSS.take_writer(),
            retry: Retry::default(),
            retry_raised: false,
        }
    }

    /// Publishes the counts: the driver's and the UART's.
    fn publish(&mut self) {
        let s = self.rx.stats();
        let mut t = self.totals;
        t.frames = s.frames;
        t.checksum_errors = s.bad_checksum;
        t.oversize = s.oversize;
        t.ignored = s.ignored;
        t.overrun = OVERRUN.load(Ordering::Relaxed);
        t.framing = FRAMING.load(Ordering::Relaxed);
        t.brk = BREAK.load(Ordering::Relaxed);
        t.parity = PARITY.load(Ordering::Relaxed);
        t.silenced = SILENCED.load(Ordering::Relaxed);
        self.totals = t;
        TOTALS.lock(|cell| cell.set(t));
    }

    /// The driver's whole `configure` with the stored `GPS_RATE_MS`, once. A failure is a WARNING (STATUSTEXT
    /// and defmt) for a first try, and returned; the retry schedule is armed. A NAV-PVT held from before is dropped.
    /// `retry`: a try the retry schedule raised (its failure is not reported again).
    async fn configure(&mut self, retry: bool) -> Result<(), ConfigureError> {
        self.calls = self.calls.wrapping_add(1);
        self.retry_raised = false;
        let tries = self.retry.begin(retry);
        let begun = Instant::now();
        let result = self.run_configure().await;
        self.ready = None;
        self.failed = None;
        match result {
            Ok(()) => self.totals.configures = self.totals.configures.saturating_add(1),
            Err(error) => {
                let t = &mut self.totals;
                t.configure_failures = t.configure_failures.saturating_add(1);
                if error == ConfigureError::Rejected {
                    t.naks = t.naks.saturating_add(1);
                }
                t.last_error = error.reason();
            }
        }
        self.publish();
        let report = self.retry.end(result.is_ok(), Instant::now().as_millis());
        if let Err(error) = result {
            defmt::warn!(
                "[INFO] GNSS: configure failed ({=str}) after {=u64} ms, try {=u32}, the next in {=u64} ms",
                error.reason(),
                begun.elapsed().as_millis(),
                tries,
                CONFIGURE_RETRY_MS
            );
        }
        match (report, result) {
            (RetryReport::Warn, Err(error)) => crate::text_fmt(
                MavSeverity::MAV_SEVERITY_WARNING,
                format_args!("GNSS: configure failed ({})", error.reason()),
            ),
            (RetryReport::Applied { tries }, _) => crate::text_fmt(
                MavSeverity::MAV_SEVERITY_INFO,
                format_args!("GNSS: configured on try {}", tries),
            ),
            _ => {}
        }
        result
    }

    async fn run_configure(&mut self) -> Result<(), ConfigureError> {
        let begun = Instant::now();
        let (ms, set) = crate::store::gnss_rate_and_set().map_err(|reason| {
            defmt::warn!("[INFO] GNSS: stored GPS_RATE_MS/GPS_GNSS refused: {=str}", reason);
            ConfigureError::Stored
        })?;
        let config = fw_settings::gnss::config(ms, set);
        let version = self.rx.configure(&config, &mut Delay).await?;
        defmt::info!(
            "[INFO] GNSS: configured: {=u16} ms, UBX only, NAV-PVT every epoch, volatile (not saved), software {=str}, hardware {=str}, {=u64} ms",
            ms,
            version.sw_str(),
            version.hw_str(),
            begun.elapsed().as_millis()
        );
        crate::text_fmt(
            MavSeverity::MAV_SEVERITY_INFO,
            format_args!("GNSS sw {} hw {}", version.sw_str(), version.hw_str()),
        );
        Ok(())
    }

    /// Reads the next NAV-PVT, or ends with `failed` set.
    async fn next_pvt(&mut self) {
        match self.rx.read_pvt().await {
            Ok(pvt) => {
                let t = &mut self.totals;
                t.pvt = t.pvt.saturating_add(1);
                t.last_pvt_us = Some(Instant::now().as_micros());
                self.ready = Some(pvt);
            }
            // The UART read never errs (`GnssUart::read`); a closed line or an unconfigured driver is a bus error.
            Err(_) => self.failed = Some(ErrorKind::Bus),
        }
        self.publish();
    }
}

/// MAVLink's `GPS_RAW_INT` of a sample (`Reading::Gnss`) taken at board time `t_us`. With no fix: `fix_type` 1
/// (NO_FIX), `lat`, `lon` and `alt` 0 (the dialect defines no other convention), `vel` and `cog`
/// UINT16_MAX unless known; `eph`/`epv` are HDOP/VDOP x 100, which the SAM-M10Q's NAV-PVT does not carry: always
/// UINT16_MAX (unknown); `satellites_visible` carries the satellites USED (numSV), 255 when unknown. `time_usec`
/// is the board time. `h_acc` is sqrt(lat^2 + lon^2) of the deviations, which are both the receiver's hAcc, so it
/// is sqrt(2) x hAcc, and `v_acc` the altitude deviation (vAcc), mm, 0 when not available.
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
    let previous = PVT_AT_STAT.lock(|cell| cell.replace(t.pvt));
    let rate = t.pvt.wrapping_sub(previous) as f32 * 1e6 / window_us as f32;
    let age_ms = t
        .last_pvt_us
        .map_or(u64::MAX, |us| now.as_micros().saturating_sub(us) / 1000);
    let (fix_type, sats, h_acc, utc) = match latest.map(|s| s.reading) {
        Some(Reading::Gnss(GnssReading {
            fix_type,
            sats_used,
            lat_std_mm,
            time,
            ..
        })) => (fix_type, sats_used, lat_std_mm, time),
        _ => (0, None, None, None),
    };
    defmt::info!(
        "[STAT] GNSS: frames ok {=u32}, NAV-PVT {=u32} ({=f32}/s over the last 10 s), checksum errors {=u32}, oversize {=u32}, ignored {=u32}, UART errors overrun {=u32} framing {=u32} break {=u32} parity {=u32}, configures done {=u32}, configure failures {=u32} (NAK {=u32}), last configure error \"{=str}\", fix type {=u8}, satellites {}, hAcc {} mm, UTC {}, last PVT {=u64} ms ago{=str}",
        t.frames,
        t.pvt,
        rate,
        t.checksum_errors,
        t.oversize,
        t.ignored,
        t.overrun,
        t.framing,
        t.brk,
        t.parity,
        t.configures,
        t.configure_failures,
        t.naks,
        t.last_error,
        fix_type,
        sats,
        h_acc,
        crate::device::Utc(utc),
        age_ms,
        if t.silenced {
            ", NAV-PVT SWALLOWED (inject)"
        } else {
            ""
        }
    );
}

impl DeviceAdapter for GnssAdapter {
    /// The read of the next NAV-PVT: droppable (see the module comment); it returns on the frame's checksum byte.
    /// Frames of other kinds on the way are consumed and counted by the driver, a UART error is counted by
    /// `GnssUart`. While the receiver is not configured (a failed apply) it waits for the retry schedule, which
    /// raises the APPLY request when a retry is due.
    async fn data_ready(&mut self) {
        loop {
            if !self.retry_raised && self.retry.due(Instant::now().as_millis()) {
                self.retry_raised = true;
                defmt::info!("[INFO] GNSS: configuration retry due: APPLY raised");
                GNSS.request_apply();
            }
            if self.rx.is_configured() {
                self.next_pvt().await;
                return;
            }
            Timer::after(RETRY_POLL).await;
        }
    }

    /// Configures the receiver (see the module comment). A failure is the start's failure: the row retries it.
    async fn start(&mut self) -> Result<(), StartFailure> {
        self.configure(false).await.map_err(|error| StartFailure {
            identified: error.identified(),
        })
    }

    async fn acquire(&mut self) -> Result<Reading, ErrorKind> {
        self.calls = self.calls.wrapping_add(1);
        if let Some(kind) = self.failed.take() {
            return Err(kind);
        }
        let pvt = self.ready.take().ok_or(ErrorKind::Bus)?;
        let reading = fw_settings::gnss::reading(&pvt);
        PVT_TOW.lock(|cell| {
            cell.set(Some(PvtTow {
                vel_ned_mm_s: reading.vel_ned_mm_s,
                tow_ms: pvt.itow,
            }))
        });
        Ok(Reading::Gnss(reading))
    }

    /// There is no status to read: after a deadline miss nothing is pending.
    async fn status(&mut self) -> Result<Pending, ErrorKind> {
        Ok(Pending::Nothing)
    }

    /// The configuration is not read back.
    async fn verify(&mut self) -> Result<(), VerifyError> {
        Ok(())
    }

    /// Configures again with the stored `GPS_RATE_MS`; a failure is one bus error, except for a retry the adapter
    /// raised itself (a failed retry is only scheduled again).
    async fn apply(&mut self) -> Result<(), ErrorKind> {
        let retry = core::mem::take(&mut self.retry_raised);
        let result = self.configure(retry).await;
        if retry {
            return Ok(());
        }
        result.map_err(|_| ErrorKind::Bus)
    }

    /// The driver's whole `configure` again, baud scan included (a UART has no bus clear).
    async fn recover(&mut self) -> Result<(), ErrorKind> {
        self.configure(false).await.map_err(|_| ErrorKind::Bus)
    }

    /// The same as `recover`.
    async fn restart(&mut self) -> Result<(), ErrorKind> {
        self.configure(false).await.map_err(|_| ErrorKind::Bus)
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
/// NAV-PVT frames (the wrapper swallows them) until a RESTART request has been raised.
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
            "[INFO] inject: GNSS silent armed (NAV-PVT frames swallowed 10 s after the first good sample until a RESTART request)"
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
                "[INFO] inject: GNSS NAV-PVT frames {=str} at {=u64} ms",
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
