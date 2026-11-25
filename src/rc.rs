//! The radio: the ExpressLRS receiver on UART0 through `rc-port`, which follows the receiver
//! between CRSF (420000 baud) and MAVLink (460800 baud) by itself. This module only REPORTS: the
//! RC row (no health machine, no recovery), RC_CHANNELS and the mapped sticks (MANUAL_CONTROL) for
//! USB, the `[STAT] RC:` line, the BATT value as CRSF telemetry, and the radio's MAVLink node (link
//! 1, active only while the port is locked on MAVLink). It also hands every CRSF stick report and every receiver report
//! (`fw_core::rc::receiver_report`, N19), mapped, to the flight core through [`FLIGHT_RC`]; arming and link loss are
//! decided there (`fw_flight::gate`).
//!
//! UART0 as the bench (`rc-port/hil/src/bin/rc_link.rs`) set it up: `BufferedUart` (interrupt
//! driven, no DMA), 256-byte rings, TX GPIO14, RX GPIO17 with the pull-up turned on after the UART
//! is built (embassy-rp 0.10 leaves the pad without a pull). The task runs alone on core 0's
//! `SWI_IRQ_0` interrupt executor (priority P3 = 0xC0, embassy-rp's lowest: below the hardware
//! interrupts at P0 and the watchdog monitor's `SWI_IRQ_1` at P1 = 0x40, above thread mode), as the bench did, so that no thread-mode task delays its reads (thread mode gave 5
//! UART overruns in 300 s). Everything it shares with thread mode is a critical-section mutex or an
//! atomic, none held across an await. The row's rule is `fw_core::rc`.
use core::cell::RefCell;
use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};
use embassy_executor::InterruptExecutor;
use embassy_rp::bind_interrupts;
use embassy_rp::interrupt::{self, InterruptExt, Priority};
use embassy_rp::peripherals::{PIN_14, PIN_17, UART0};
use embassy_rp::uart::{self, BufferedInterruptHandler, BufferedUart};
use embassy_rp::{Peri, pac};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_io_async::{ErrorType, Read, Write};
use fw_core::flight_cmd::SendersSeen;
use fw_core::health::Counters;
use fw_core::rc::{
    PortMode, RcInput, RcSample, RcSource, RcVerdict, crsf_gps, crsf_sticks, gps_frame_due, receiver_report,
    verdict,
};
use fw_core::sample::GnssReading;
use fw_core::sys_status::{DeviceRow, StatusInput, device_status};
use fw_core::watchdog::task;
use fw_core::xcore::{TripleBuffer, TripleWriter};
use fw_settings::rc::{MARK_HACC_MAX_CM, MarkText, MarkTracker, RcMap, arm_switch, mark_fix, mode_switch};
use mavlink_device::dialect::{MANUAL_CONTROL_DATA, MavState, RC_CHANNELS_DATA};
use mavlink_device::marv::{MARV_DEVICE_STATUS_DATA, MarvDevice};
use mavlink_device::{
    CommandOutcome, CommandRequest, Config as NodeConfig, LateAck, Outgoing, ParamStore,
    ParamValue, Stream, StreamSlot, Telemetry, Texts,
};
use rc_port::crsf_codec::{
    Battery, Gps, LinkStatistics, MAX_FRAME_LEN, encode_battery, encode_gps,
};
use rc_port::{
    Config, Hooks, MAVLINK_BAUD, Mode, RadioStatus, RcPort, Report, SetBaud, Source, Stats, TxBusy,
};
use static_cell::StaticCell;

use crate::device::{BATT, GNSS, Reading, Sample};

bind_interrupts!(struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

const RX_GPIO: usize = 17;
const RING_LEN: usize = 256;
static TX_BUF: StaticCell<[u8; RING_LEN]> = StaticCell::new();
static RX_BUF: StaticCell<[u8; RING_LEN]> = StaticCell::new();
static SLOTS: StaticCell<[StreamSlot; 7]> = StaticCell::new();

/// The radio node's streams (HEARTBEAT 1 Hz is the node's own). The fastest interval a station may
/// ask for is the default, so the radio's budget cannot be raised from the ground.
const SYS_STATUS_ID: u32 = 1;
const DEVICE_STATUS_ID: u32 = 42421;
const GPS_RAW_INT_ID: u32 = 24;
const HIGHRES_IMU_ID: u32 = 105;
const SCALED_IMU2_ID: u32 = 116;
const SCALED_PRESSURE_ID: u32 = 29;
const RADIO_STREAMS: [(u32, i32); 7] = [
    (SYS_STATUS_ID, 1_000_000),
    (DEVICE_STATUS_ID, 1_000_000),
    (GPS_RAW_INT_ID, 2_000_000),
    (HIGHRES_IMU_ID, 1_000_000),
    (SCALED_IMU2_ID, 1_000_000),
    (SCALED_PRESSURE_ID, 1_000_000),
    // The mission service's reply slot (`mission::sample`): idle, it is one load.
    (crate::mission::SLOT_ID, crate::mission::RADIO_INTERVAL_US),
];
/// Stick reports and link figures older than this are not shown.
const FRESH_US: u64 = 1_000_000;
/// Pause before `run` is called again after it returned.
const RESTART_MS: u64 = 100;
/// The shortest time between two CRSF GPS frames: keeps the frame mix (GPS and battery) of the 1 Hz
/// receiver on the radio link whatever `GPS_RATE_MS`.
const GPS_FRAME_MIN_US: u64 = 1_000_000;
/// The hook re-reads the stick map from the store at every this many CRSF stick reports (about 4/s).
const MAP_EVERY: u32 = 64;

/// Core 0 -> flight core: the latest CRSF or receiver stick report, mapped, with its switches and time (one
/// writer, the port's hooks; one reader, IMU1's [`crate::flight::Stepper`]).
pub static FLIGHT_RC: TripleBuffer<RcSample> = TripleBuffer::new(RcSample {
    sticks: fw_core::rc::Sticks {
        roll: 0.0,
        pitch: 0.0,
        throttle: 0.0,
        yaw: 0.0,
    },
    arm: false,
    switch6: fw_core::rc::Switch3::Low,
    source: RcSource::Crsf,
    mission: false,
    t_us: 0,
});

/// Core 0 -> flight core: the time of the latest RADIO_STATUS from the receiver, us (its `at_us`; one writer, the port's
/// hooks; one reader, IMU1's [`crate::flight::Stepper`]). Sets the hands-off latch M (decision-mavlink-mode-20261123, N12).
pub static FLIGHT_RADIO_STATUS: TripleBuffer<u64> = TripleBuffer::new(0);

/// Bytes the UART took at the CRSF baud and at 460800 (the port writes only while locked, in the
/// locked protocol), baud changes, and returns of `run`.
static TX_CRSF: AtomicU32 = AtomicU32::new(0);
static TX_MAV: AtomicU32 = AtomicU32::new(0);
static BAUD_CHANGES: AtomicU32 = AtomicU32::new(0);
static RESTARTS: AtomicU32 = AtomicU32::new(0);

/// What the port's hooks last reported, for the row, RC_CHANNELS and the stat line.
#[derive(Clone, Copy)]
struct Shared {
    stats: Option<Stats>,
    ever_locked: bool,
    lock_losses: u32,
    sticks: Option<([u16; 16], u64)>,
    /// The latest stick report came from CRSF (not from a MAVLink RC_CHANNELS_OVERRIDE).
    sticks_crsf: bool,
    crsf_link: Option<(LinkStatistics, u64)>,
    radio: Option<(RadioStatus, u64)>,
}

static SHARED: Mutex<CriticalSectionRawMutex, RefCell<Shared>> = Mutex::new(RefCell::new(Shared {
    stats: None,
    ever_locked: false,
    lock_losses: 0,
    sticks: None,
    sticks_crsf: false,
    crsf_link: None,
    radio: None,
}));

fn shared() -> Shared {
    SHARED.lock(|s| *s.borrow())
}

fn locked(mode: Option<Mode>) -> bool {
    matches!(mode, Some(Mode::Crsf | Mode::Mavlink))
}

impl Shared {
    fn mode(&self) -> Option<Mode> {
        self.stats.map(|s| s.mode)
    }

    /// The row's state, link up (the health bit) and last error: `fw_core::rc::verdict`.
    fn verdict(&self, now_us: u64) -> RcVerdict {
        verdict(&RcInput {
            mode: match self.mode() {
                Some(Mode::Crsf) => PortMode::Crsf,
                Some(Mode::Mavlink) => PortMode::Mavlink,
                _ => PortMode::Searching,
            },
            ever_locked: self.ever_locked,
            now_us,
            last_sticks_us: self.sticks.map(|(_, t)| t),
            radio_status: self.radio.map(|(r, t)| (r.rssi, t)),
        })
    }

    fn errors(&self) -> u32 {
        self.stats.map_or(0, |s| {
            s.crc_errors
                .saturating_add(s.uart_framing)
                .saturating_add(s.uart_parity)
                .saturating_add(s.uart_break)
                .saturating_add(s.uart_overrun)
                .saturating_add(s.uart_other)
        })
    }
}

/// The RC row for SYS_STATUS (health = the radio link is up) and the HEARTBEAT state.
pub fn row() -> DeviceRow {
    let s = shared();
    let v = s.verdict(crate::now_us());
    DeviceRow {
        device: MarvDevice::MARV_DEVICE_RC,
        present: s.ever_locked,
        state: v.state,
        stuck: false,
        locked: v.link_up,
    }
}

/// The RC row's `MARV_DEVICE_STATUS`: samples = stick reports + RADIO_STATUS reports, errors =
/// CRC + UART errors, timeouts = lock losses.
pub fn message(now_us: u64) -> MARV_DEVICE_STATUS_DATA {
    let s = shared();
    let v = s.verdict(now_us);
    let ms = |us: u64| u32::try_from(us / 1000).unwrap_or(u32::MAX);
    let (samples, last) = s.stats.map_or((0, None), |st| {
        (
            st.sticks.wrapping_add(st.radio_status),
            st.last_sticks_us.max(st.last_radio_status_us),
        )
    });
    device_status(&StatusInput {
        device: MarvDevice::MARV_DEVICE_RC,
        state: v.state,
        counters: Counters {
            errors: s.errors(),
            timeouts: s.lock_losses,
            recoveries: 0,
            attempts: 0,
            config_restored: 0,
            overflows: 0,
            last_error: v.last_error,
            present: s.ever_locked,
        },
        stuck: false,
        samples,
        last_sample_age_ms: last.map(|t| ms(now_us.saturating_sub(t))),
        hil: false,
        self_test: false,
        time_boot_ms: ms(now_us),
    })
}

/// RC_CHANNELS for USB while the latest stick report is younger than 1 s. `rssi`: CRSF uplink link
/// quality 0..100 % scaled to 0..254; in MAVLink mode RADIO_STATUS `rssi` (already 0..254); 255
/// (unknown) without a fresh link figure.
pub fn rc_channels(now_us: u64) -> Option<RC_CHANNELS_DATA> {
    let s = shared();
    let fresh = |t: u64| now_us.saturating_sub(t) < FRESH_US;
    let (ch, t_us) = s.sticks.filter(|&(_, t)| fresh(t))?;
    let rssi = match (s.mode(), s.crsf_link, s.radio) {
        (Some(Mode::Crsf), Some((l, t)), _) if fresh(t) => {
            (u16::from(l.uplink_link_quality.min(100)) * 254 / 100) as u8
        }
        (Some(Mode::Mavlink), _, Some((r, t))) if fresh(t) => r.rssi.min(254),
        _ => u8::MAX,
    };
    Some(RC_CHANNELS_DATA {
        time_boot_ms: u32::try_from(t_us / 1000).unwrap_or(u32::MAX),
        chan1_raw: ch[0],
        chan2_raw: ch[1],
        chan3_raw: ch[2],
        chan4_raw: ch[3],
        chan5_raw: ch[4],
        chan6_raw: ch[5],
        chan7_raw: ch[6],
        chan8_raw: ch[7],
        chan9_raw: ch[8],
        chan10_raw: ch[9],
        chan11_raw: ch[10],
        chan12_raw: ch[11],
        chan13_raw: ch[12],
        chan14_raw: ch[13],
        chan15_raw: ch[14],
        chan16_raw: ch[15],
        chan17_raw: u16::MAX,
        chan18_raw: u16::MAX,
        chancount: 16,
        rssi,
    })
}

/// MANUAL_CONTROL for USB, reported only: the four mapped sticks while the latest stick report is
/// from CRSF and younger than 1 s (`fw_core::rc::crsf_sticks`). One `SHARED` lock copies the sticks,
/// then (not nested) one store access copies the 44 RC values by index; the mapping is computed
/// outside both. The table's defaults if the store is not ready.
pub fn manual_control(now_us: u64) -> Option<MANUAL_CONTROL_DATA> {
    let (latest, from_crsf) = SHARED.lock(|s| {
        let s = s.borrow();
        (s.sticks, s.sticks_crsf)
    });
    let pwm = crsf_sticks(latest, from_crsf, now_us)?;
    let values = crate::store::with(|p| {
        core::array::from_fn(|i| {
            u16::try_from(fw_settings::rc::RC_OFFSET + i)
                .ok()
                .and_then(|n| ParamStore::get(p, n))
        })
    })
    .unwrap_or([None::<ParamValue>; fw_settings::rc::COUNT]);
    let axes = fw_settings::rc::RcMap::from_values(&values).axes(&pwm);
    Some(MANUAL_CONTROL_DATA {
        target: 1,
        x: axes.x,
        y: axes.y,
        z: axes.z,
        r: axes.r,
        buttons: 0,
        ..Default::default()
    })
}

/// The port's hooks: copy into `SHARED`, never block, never format.
struct RcHooks {
    /// The GNSS sample (its `t_us`) last sent as a GPS frame.
    last_gps_us: Option<u64>,
    /// Board time when the last GPS frame was sent.
    last_gps_sent_us: Option<u64>,
    /// The writer of [`FLIGHT_RC`] (`None` if it was taken already: nothing is written).
    flight: Option<TripleWriter<'static, RcSample>>,
    /// The writer of [`FLIGHT_RADIO_STATUS`] (`None` if it was taken already: nothing is written).
    flight_radio: Option<TripleWriter<'static, u64>>,
    /// The stick map last read from the store; `None` until the store gave one (no sample then).
    map: Option<RcMap>,
    /// Stick reports handed on (CRSF and receiver).
    crsf_reports: u32,
    /// The arm switch in the previous stick report handed on.
    arm_on: bool,
    /// Channel 9 edge tracking of the receiver reports (the waypoint mark).
    marks: MarkTracker,
    /// The ground-station HEARTBEAT senders already logged ("GCS HB"), at most `SENDERS_SEEN_MAX`.
    gcs_seen: SendersSeen,
}

impl Hooks for RcHooks {
    fn report(&mut self, report: Report) {
        let crsf = match &report {
            Report::Sticks(r) => match r.source {
                Source::Crsf => Some((r.channels_us, r.at_us, RcSource::Crsf)),
                Source::Mavlink { system_id, component_id }
                    if receiver_report(system_id, component_id, &r.channels_us) =>
                {
                    Some((r.channels_us, r.at_us, RcSource::Receiver))
                }
                _ => None,
            },
            _ => None,
        };
        SHARED.lock(|s| {
            let mut s = s.borrow_mut();
            match report {
                Report::Sticks(r) => {
                    s.sticks = Some((r.channels_us, r.at_us));
                    s.sticks_crsf = matches!(r.source, Source::Crsf);
                }
                Report::CrsfLink(r) => s.crsf_link = Some((r.stats, r.at_us)),
                Report::RadioStatus(r) => s.radio = Some((r.status, r.at_us)),
                _ => {}
            }
        });
        if let (Report::RadioStatus(r), Some(w)) = (&report, &mut self.flight_radio) {
            w.write(r.at_us);
        }
        let Some((ch, t_us, source)) = crsf else {
            return;
        };
        if self.crsf_reports.is_multiple_of(MAP_EVERY) {
            // One store access (not nested in `SHARED`) copies the 44 RC values; the map is built outside it.
            let fresh = crate::store::with(|p| {
                core::array::from_fn(|i| {
                    u16::try_from(fw_settings::rc::RC_OFFSET + i)
                        .ok()
                        .and_then(|n| ParamStore::get(p, n))
                })
            })
            .map(|values: [Option<ParamValue>; fw_settings::rc::COUNT]| {
                RcMap::from_values(&values)
            });
            // If the store did not answer this time, keep the map we have: dropping it would stop the
            // stick stream to the flight core until the next refresh.
            if fresh.is_some() {
                self.map = fresh;
            }
        }
        self.crsf_reports = self.crsf_reports.wrapping_add(1);
        if let (Some(map), Some(w)) = (&self.map, &mut self.flight) {
            w.write(RcSample {
                sticks: map.sticks(&ch),
                arm: arm_switch(ch[4]),
                switch6: mode_switch(ch[5], &map.channels[5]),
                source,
                mission: arm_switch(ch[7]),
                t_us,
            });
        }
        // A waypoint mark: channel 9 off-to-on on a MAVLink receiver report sends one MARK text with the position.
        if let Some(n) = self.marks.update(matches!(source, RcSource::Receiver), ch[8], t_us) {
            let pos = match GNSS.latest.try_get() {
                Some(Sample {
                    reading:
                        Reading::Gnss(GnssReading {
                            fix_type,
                            lat_e7: Some(lat),
                            lon_e7: Some(lon),
                            lat_std_mm,
                            ..
                        }),
                    t_us: gnss_us,
                    ..
                }) if mark_fix(fix_type) && crate::now_us().saturating_sub(gnss_us) < 2_000_000 => {
                    Some((lat, lon, (lat_std_mm.unwrap_or(0) / 10).min(MARK_HACC_MAX_CM)))
                }
                _ => None,
            };
            crate::text_fmt(
                mavlink_device::dialect::MavSeverity::MAV_SEVERITY_INFO,
                format_args!("{}", MarkText { n, pos }),
            );
        }
        // A refused arm (decision-prearm-20261119, section 4): on the switch's off-to-on edge, while not armed and
        // without core 0's permit, one WARNING text with the reason and the LED's three red flashes (the only
        // formatting and queueing here, and only on such an edge). The gate reads the permit on core 1 a few hundred
        // us apart from this read, so in that window a report without a refusal, or a refusal without a report, is
        // possible; both are harmless.
        let arm = arm_switch(ch[4]);
        let edge = arm && !self.arm_on;
        self.arm_on = arm;
        if edge && !crate::settings::HANDSHAKE.armed() && !crate::settings::HANDSHAKE.permitted() {
            crate::text(
                mavlink_device::dialect::MavSeverity::MAV_SEVERITY_WARNING,
                crate::actions::refusal(),
            );
            crate::led::event(false);
        }
    }

    /// The GPS frame from the GNSS row's latest sample when it is newer than the one last sent as
    /// GPS (fix or no fix: `fw_core::rc::crsf_gps`, zeros without a fix); otherwise the BATT row's
    /// latest sample while younger than 1 s (remaining percent and capacity are not estimated: 0).
    fn crsf_frame(&mut self, now_us: u64, out: &mut [u8; MAX_FRAME_LEN]) -> Option<usize> {
        if let Some(Sample {
            reading:
                Reading::Gnss(GnssReading {
                    lat_e7,
                    lon_e7,
                    alt_mm,
                    speed_mm_s,
                    course_cdeg,
                    sats_used,
                    ..
                }),
            t_us,
            ..
        }) = GNSS.latest.try_get()
            && gps_frame_due(Some(t_us), self.last_gps_us)
            && self
                .last_gps_sent_us
                .is_none_or(|sent| now_us.saturating_sub(sent) >= GPS_FRAME_MIN_US)
        {
            let g = crsf_gps(lat_e7, lon_e7, alt_mm, speed_mm_s, course_cdeg, sats_used);
            let gps = Gps {
                latitude_e7: g.latitude_e7,
                longitude_e7: g.longitude_e7,
                groundspeed_dkmh: g.groundspeed_dkmh,
                heading_cdeg: g.heading_cdeg,
                altitude_m: g.altitude_m,
                satellites: g.satellites,
            };
            if let Ok(n) = encode_gps(&gps, out) {
                self.last_gps_us = Some(t_us);
                self.last_gps_sent_us = Some(now_us);
                return Some(n);
            }
        }
        let Some(Sample {
            reading:
                Reading::Batt {
                    voltage_v,
                    current_a,
                },
            t_us,
            ..
        }) = BATT.latest.try_get()
        else {
            return None;
        };
        if now_us.saturating_sub(t_us) >= FRESH_US {
            return None;
        }
        let battery = Battery {
            voltage_dv: (voltage_v * 10.0 + 0.5).clamp(0.0, 65_535.0) as u16,
            current_da: (current_a * 10.0 + 0.5).clamp(0.0, 65_535.0) as u16,
            capacity_used_mah: 0,
            remaining_percent: 0,
        };
        encode_battery(&battery, out).ok()
    }

    fn stats(&mut self, stats: &Stats) {
        crate::station::station_heartbeat(stats.last_gcs_heartbeat_us, stats.last_gcs_heartbeat_sender);
        // The first time each sender of a ground-station HEARTBEAT is seen since boot: one text "GCS HB <sys>,<comp>"
        // (formatting and queueing only then, at most `SENDERS_SEEN_MAX` times). Log only: which one counts is unchanged.
        if let Some((sys, comp)) = stats.last_gcs_heartbeat_sender
            && self.gcs_seen.first((sys, comp))
        {
            crate::text_fmt(
                mavlink_device::dialect::MavSeverity::MAV_SEVERITY_INFO,
                format_args!("GCS HB {},{}", sys, comp),
            );
        }
        let before = SHARED.lock(|s| {
            let mut s = s.borrow_mut();
            let before = s.mode();
            if locked(Some(stats.mode)) {
                s.ever_locked = true;
            } else if locked(before) {
                s.lock_losses = s.lock_losses.saturating_add(1);
            }
            s.stats = Some(*stats);
            before
        });
        if before == Some(Mode::Mavlink) && stats.mode != Mode::Mavlink {
            crate::actions::link_lost(crate::LINK_RADIO as u8);
            crate::mission::link_down(fw_core::mission::Link::Radio);
        }
    }

    fn mavlink(&mut self, msg: mavlink_device::Incoming<'_>) {
        crate::mission::message(fw_core::mission::Link::Radio, msg);
    }
}

/// `BufferedUart` with the bytes each write took counted by baud rate, and the baud changes.
struct CountingUart {
    uart: BufferedUart,
    baud: u32,
}

impl ErrorType for CountingUart {
    type Error = uart::Error;
}

impl Read for CountingUart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, uart::Error> {
        self.uart.read(buf).await
    }
}

impl Write for CountingUart {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, uart::Error> {
        let r = self.uart.write(buf).await;
        if let Ok(n) = r {
            let counter = if self.baud == MAVLINK_BAUD {
                &TX_MAV
            } else {
                &TX_CRSF
            };
            counter.fetch_add(n as u32, Ordering::Relaxed);
        }
        r
    }

    async fn flush(&mut self) -> Result<(), uart::Error> {
        self.uart.flush().await
    }
}

impl SetBaud for CountingUart {
    fn set_baud(&mut self, baud: u32) {
        self.uart.set_baud(baud);
        self.baud = baud;
        BAUD_CHANGES.fetch_add(1, Ordering::Relaxed);
    }
}

impl TxBusy for CountingUart {
    fn tx_busy(&self) -> bool {
        self.uart.tx_busy()
    }
}

/// The radio node's hooks (link 1): HEARTBEAT state, SYS_STATUS, one MARV_DEVICE_STATUS row per
/// message (a row whose state changed since it was last sent on the radio goes out of turn, at most every
/// other message; the message after it is the cursor's),
/// GPS_RAW_INT, HIGHRES_IMU, SCALED_IMU2, SCALED_PRESSURE, the texts of link 1, commands through the one book.
struct RadioBoard {
    cursor: usize,
    last_out_of_turn: bool,
    sent_state: [Option<u8>; crate::ROWS],
}

impl Telemetry for RadioBoard {
    fn sample(&mut self, message_id: u32, now_us: u64, out: &mut Outgoing<'_>) -> bool {
        match message_id {
            SYS_STATUS_ID => crate::push_sys_status(now_us, out),
            GPS_RAW_INT_ID => crate::push_gps_raw_int(now_us, out),
            HIGHRES_IMU_ID => crate::push_highres_imu(now_us, out),
            SCALED_IMU2_ID => crate::push_scaled_imu2(now_us, out),
            SCALED_PRESSURE_ID => crate::push_scaled_pressure(now_us, out),
            crate::mission::SLOT_ID => crate::mission::sample(fw_core::mission::Link::Radio, now_us, out),
            DEVICE_STATUS_ID => {
                let messages: [Option<MARV_DEVICE_STATUS_DATA>; crate::ROWS] =
                    core::array::from_fn(|i| crate::device_message(i, now_us));
                let changed = messages.iter().zip(self.sent_state.iter()).position(
                    |(m, sent)| matches!((m, sent), (Some(m), Some(s)) if m.state as u8 != *s),
                );
                let (i, cursor, out_of_turn) =
                    fw_core::rc::pick_row(changed, self.cursor, crate::ROWS, self.last_out_of_turn);
                self.cursor = cursor;
                self.last_out_of_turn = out_of_turn;
                let Some(Some(message)) = messages.get(i) else {
                    return false;
                };
                if let Some(sent) = self.sent_state.get_mut(i) {
                    *sent = Some(message.state as u8);
                }
                out.push(message).is_ok()
            }
            _ => false,
        }
    }

    fn status(&mut self, now_us: u64) -> Option<MavState> {
        Some(crate::heartbeat_state(now_us))
    }

    fn texts(&mut self, _now_us: u64, out: &mut Texts<'_>) {
        if let Some((severity, text)) = crate::take_text(crate::LINK_RADIO) {
            defmt::info!("[INFO] text to the radio: {=str}", text.as_str());
            let _ = out.push(severity, text.as_str());
        }
    }

    fn command(&mut self, req: &CommandRequest, now_us: u64) -> CommandOutcome {
        crate::actions::command(req, crate::LINK_RADIO as u8, now_us / 1000)
    }

    fn command_deferrable(&mut self, req: &CommandRequest, now_us: u64) -> Option<CommandOutcome> {
        crate::actions::command_deferrable(req, crate::LINK_RADIO as u8, now_us / 1000)
    }

    fn custom_mode(&mut self, _now_us: u64) -> u32 {
        crate::station::custom_mode()
    }

    fn acks(&mut self, now_us: u64) -> Option<LateAck> {
        crate::actions::next_ack(crate::LINK_RADIO as u8, now_us / 1000)
    }
}

/// The radio node: system 1, component 1, the six mission messages handed over (the port adds the two it reports).
const RADIO_NODE: NodeConfig = match NodeConfig::new(1, 1).with_accept_list(&crate::mission::ACCEPT) {
    Ok(config) => config,
    Err(_) => panic!("accept list too long"),
};

static EXECUTOR_RC: InterruptExecutor = InterruptExecutor::new();

#[cortex_m_rt::interrupt]
unsafe fn SWI_IRQ_0() {
    // SAFETY: the one handler of the interrupt `EXECUTOR_RC` was started on.
    unsafe { EXECUTOR_RC.on_interrupt() }
}

/// Starts the port's task alone on `SWI_IRQ_0` at priority P3 (0xC0; below the watchdog monitor's
/// P1, above thread mode). Call on core 0.
pub fn start(uart0: Peri<'static, UART0>, tx: Peri<'static, PIN_14>, rx: Peri<'static, PIN_17>) {
    interrupt::SWI_IRQ_0.set_priority(Priority::P3);
    let spawner = EXECUTOR_RC.start(interrupt::SWI_IRQ_0);
    spawner.spawn(defmt::unwrap!(task(uart0, tx, rx)));
}

#[embassy_executor::task]
async fn task(
    uart0: Peri<'static, UART0>,
    tx: Peri<'static, PIN_14>,
    rx: Peri<'static, PIN_17>,
) -> ! {
    preempting(&crate::watchdog::RUNNING0, task::RC, rc_link(uart0, tx, rx)).await
}

/// `fw_flight::tagged` for a task that preempts thread mode: `id` is stored at the start of each
/// poll and the preempted task's id put back at its end, so `running` names RC exactly while RC
/// runs (a stall inside its poll is recorded as RC; the thread-mode task it interrupted keeps its
/// own id afterwards).
async fn preempting<F: Future>(running: &AtomicU8, id: u8, fut: F) -> F::Output {
    let mut fut = pin!(fut);
    poll_fn(|cx| {
        let before = running.swap(id, Ordering::Relaxed);
        let poll = fut.as_mut().poll(cx);
        running.store(before, Ordering::Relaxed);
        poll
    })
    .await
}

async fn rc_link(
    uart0: Peri<'static, UART0>,
    tx: Peri<'static, PIN_14>,
    rx: Peri<'static, PIN_17>,
) -> ! {
    // Built at 460800: the port's first actions set 460800 whatever the UART was built with.
    let mut cfg = uart::Config::default();
    cfg.baudrate = MAVLINK_BAUD;
    let uart = BufferedUart::new(
        uart0,
        tx,
        rx,
        Irqs,
        TX_BUF.init([0; RING_LEN]),
        RX_BUF.init([0; RING_LEN]),
        cfg,
    );
    // The HAL leaves GPIO17's pad without a pull; an idle UART line is high.
    pac::PADS_BANK0.gpio(RX_GPIO).modify(|w| w.set_pue(true));
    let pad = pac::PADS_BANK0.gpio(RX_GPIO).read();
    let level = (pac::SIO.gpio_in(0).read() >> RX_GPIO) & 1;
    defmt::info!(
        "[INFO] rc_link: UART0 BufferedUart (no DMA), TX GPIO14, RX GPIO17 pull-up {=bool} pull-down {=bool} level {=u32}; rings {=usize} B; SWI_IRQ_0 executor at priority P3 (0xC0); radio node system 1 component 1, link 1",
        pad.pue(),
        pad.pde(),
        level,
        RING_LEN
    );
    let mut uart = CountingUart {
        uart,
        baud: MAVLINK_BAUD,
    };
    let slots = SLOTS.init(RADIO_STREAMS.map(|(message_id, interval)| {
        StreamSlot::new(Stream {
            message_id,
            default_interval_us: interval,
            min_interval_us: interval.unsigned_abs(),
        })
    }));
    let Ok(mut port) = RcPort::new(Config::new(), RADIO_NODE, slots) else {
        defmt::panic!("[FAIL] rc_link: port not built");
    };
    let mut params = crate::store::Store;
    let mut radio = RadioBoard {
        cursor: 0,
        last_out_of_turn: false,
        sent_state: [None; crate::ROWS],
    };
    let mut hooks = RcHooks {
        last_gps_us: None,
        last_gps_sent_us: None,
        flight: FLIGHT_RC.take_writer(),
        flight_radio: FLIGHT_RADIO_STATUS.take_writer(),
        map: None,
        crsf_reports: 0,
        arm_on: false,
        marks: MarkTracker::new(),
        gcs_seen: SendersSeen::new(),
    };
    loop {
        let end = port
            .run(
                &mut uart,
                &mut Delay,
                crate::now_us,
                &mut params,
                &mut radio,
                &mut hooks,
            )
            .await;
        let n = RESTARTS.fetch_add(1, Ordering::Relaxed) + 1;
        defmt::warn!(
            "[INFO] rc_link: run returned ({=str}), restart {=u32}",
            match end {
                rc_port::RunError::Write(_) => "write error",
                rc_port::RunError::WriteZero => "write zero",
                rc_port::RunError::EndOfStream => "end of stream",
            },
            n
        );
        Timer::after(Duration::from_millis(RESTART_MS)).await;
    }
}

/// The `[STAT] RC:` line (every 10 s, from the supervisor): rates over the interval since the last.
pub fn stat(now: Instant) {
    static LAST: Mutex<CriticalSectionRawMutex, RefCell<(u32, u32, u64)>> =
        Mutex::new(RefCell::new((0, 0, 0)));
    let s = shared();
    let st = s.stats.unwrap_or_default();
    let now_us = now.as_micros();
    let v = s.verdict(now_us);
    let (sticks0, radio0, at0) = LAST.lock(|l| l.replace((st.sticks, st.radio_status, now_us)));
    let ms = (now_us.saturating_sub(at0) / 1000).max(1);
    let per_s = |n: u32| (u64::from(n) * 1000 / ms) as u32;
    let fresh = |t: u64| now_us.saturating_sub(t) < FRESH_US;
    let (lq, rssi_dbm) = match s.crsf_link {
        Some((l, t)) if fresh(t) => (
            i16::from(l.uplink_link_quality),
            -i16::from(l.uplink_rssi_1),
        ),
        _ => (-1, 0),
    };
    let (radio_rssi, txbuf) = match s.radio {
        Some((r, t)) if fresh(t) => (i16::from(r.rssi), i16::from(r.txbuf)),
        _ => (-1, -1),
    };
    defmt::info!(
        "[STAT] RC: {=str} for {=u64} ms, state {=str}, radio link {=str}, {=u32} stick/s, {=u32} radio-status/s, LQ {=i16} RSSI {=i16} dBm, radio rssi {=i16} txbuf {=i16}, slowdown {=u32} ms; CRC {=u32}, UART framing {=u32} parity {=u32} break {=u32} overrun {=u32} other {=u32}, garbage {=u32} B; locks lost {=u32}, mode changes {=u32}, baud changes {=u32}, run restarts {=u32}; sent CRSF {=u32} B MAVLink {=u32} B",
        match st.mode {
            Mode::Crsf => "Crsf",
            Mode::Mavlink => "Mavlink",
            _ => "Searching",
        },
        now_us.saturating_sub(st.mode_since_us) / 1000,
        crate::device::state_name(v.state),
        if v.link_up { "up" } else { "down" },
        per_s(st.sticks.wrapping_sub(sticks0)),
        per_s(st.radio_status.wrapping_sub(radio0)),
        lq,
        rssi_dbm,
        radio_rssi,
        txbuf,
        st.stream_slowdown_ms,
        st.crc_errors,
        st.uart_framing,
        st.uart_parity,
        st.uart_break,
        st.uart_overrun,
        st.uart_other,
        st.garbage_bytes,
        s.lock_losses,
        st.mode_changes,
        BAUD_CHANGES.load(Ordering::Relaxed),
        RESTARTS.load(Ordering::Relaxed),
        TX_CRSF.load(Ordering::Relaxed),
        TX_MAV.load(Ordering::Relaxed)
    );
}
