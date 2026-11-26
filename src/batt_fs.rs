//! The low-battery monitor on the board: core 0, from the supervisor's tick at 10 Hz. `fw_core::battery` decides; this module reads the BATT row's latest sample, the armed flag and the stored
//! `BATT_*` settings, raises the texts, publishes the event for the log and hands one byte to the flight core ([`CELL`]).
//! Only one relaxed load of [`CELL`] and one store of [`DESCENT`] per step run on core 1.

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use fw_core::battery::{CELL_KNOWN_OK, Config, Level, Monitor, Notices, Sample as Volts};
use mavlink_device::dialect::MavSeverity;

use crate::device::{BATT, Reading, Sample};

/// Core 0 -> flight core: bits 0 and 1 the level reached (`Level`), bit 2 (`CELL_DESCEND`) the descent request, both 0
/// until armed and while disarmed; bit 3 (`CELL_KNOWN_OK`) the battery known not to be critical.
pub static CELL: AtomicU8 = AtomicU8::new(0);

/// Flight core -> core 0: what the descent did at the latest step (`fw_flight::step::BattDescent` as a byte).
pub static DESCENT: AtomicU8 = AtomicU8::new(0);

pub fn level() -> u8 {
    CELL.load(Ordering::Relaxed) & 3
}

/// A level reached, for the log: the number of events so far and the latest one's fields. Written by [`Supervision`], read by
/// the log collector (both core 0); the fields are stored before the count.
static EVENT_SEQ: AtomicU32 = AtomicU32::new(0);
static EVENT_WORD: AtomicU32 = AtomicU32::new(0);
static EVENT_VOLTS: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy)]
pub struct Event {
    pub level: u8,
    pub act: u8,
    pub volts: f32,
    pub cells: u8,
}

/// The newest event if its number is not `seen` (which then becomes it).
pub fn event(seen: &mut u32) -> Option<Event> {
    let seq = EVENT_SEQ.load(Ordering::Acquire);
    if seq == *seen {
        return None;
    }
    *seen = seq;
    let word = EVENT_WORD.load(Ordering::Relaxed);
    Some(Event {
        level: word as u8,
        act: (word >> 8) as u8,
        cells: (word >> 16) as u8,
        volts: f32::from_bits(EVENT_VOLTS.load(Ordering::Relaxed)),
    })
}

pub struct Watch {
    monitor: Monitor,
    descent_seen: u8,
}

impl Watch {
    pub const fn new() -> Self {
        Self {
            monitor: Monitor::new(),
            descent_seen: 0,
        }
    }

    pub fn tick(&mut self, now_us: u64) {
        let cfg = crate::store::batt_monitor_config();
        let armed = crate::settings::HANDSHAKE.armed();
        let sample = match BATT.latest.try_get() {
            Some(Sample {
                reading: Reading::Batt { voltage_v, .. },
                t_us,
                ..
            }) => Some(Volts {
                volts: voltage_v,
                t_ms: t_us / 1000,
            }),
            _ => None,
        };
        let update = self.monitor.step(&cfg, now_us / 1000, armed, sample);
        let known_ok = if self.monitor.known_not_critical() { CELL_KNOWN_OK } else { 0 };
        CELL.store(update.cell | known_ok, Ordering::Relaxed);
        if let Some(level) = update.rose {
            self.rose(level, update.volts, &cfg);
        }
        let n = update.notices;
        if n.contains(Notices::NO_DATA) {
            defmt::warn!("[INFO] BATT: no voltage sample for 5 s while armed");
            crate::text(MavSeverity::MAV_SEVERITY_WARNING, "BATT no voltage data");
        }
        if n.contains(Notices::IMPLAUSIBLE) {
            defmt::warn!("[INFO] BATT: the voltage is above the cell count's full voltage");
            crate::text(MavSeverity::MAV_SEVERITY_WARNING, "BATT volts too high for BATT_CELLS");
        }
        if n.contains(Notices::INVERTED) {
            defmt::warn!("[INFO] BATT: BATT_CRT_CELL is not below BATT_LOW_CELL: the monitor is off");
            crate::text(MavSeverity::MAV_SEVERITY_WARNING, "BATT_CRT_CELL >= BATT_LOW_CELL: off");
        }
        let descent = DESCENT.load(Ordering::Relaxed);
        if descent != self.descent_seen {
            self.descent_seen = descent;
            match descent {
                1 => crate::text(MavSeverity::MAV_SEVERITY_CRITICAL, "BATT critical: descending"),
                2 => crate::text(MavSeverity::MAV_SEVERITY_WARNING, "BATT descent cancelled"),
                3 => crate::text(MavSeverity::MAV_SEVERITY_CRITICAL, "no height estimate: land now"),
                _ => {}
            }
        }
    }

    fn rose(&mut self, level: Level, volts: f32, cfg: &Config) {
        let centi = (volts * 100.0 + 0.5) as u32;
        let per_cell = (volts * 100.0 / cfg.cells.max(1) as f32 + 0.5) as u32;
        let (name, severity) = match level {
            Level::Critical => ("critical", MavSeverity::MAV_SEVERITY_CRITICAL),
            _ => ("low", MavSeverity::MAV_SEVERITY_WARNING),
        };
        crate::text_fmt(
            severity,
            format_args!(
                "BATT {} {}.{:02} V {}.{:02} V/cell",
                name,
                centi / 100,
                centi % 100,
                per_cell / 100,
                per_cell % 100
            ),
        );
        defmt::warn!(
            "[INFO] BATT {=str}: {=f32} V, {=i32} cells, action {=i32}",
            name,
            volts,
            cfg.cells,
            cfg.act
        );
        EVENT_WORD.store(
            level as u32 | (cfg.act as u32 & 0xFF) << 8 | (cfg.cells as u32 & 0xFF) << 16,
            Ordering::Relaxed,
        );
        EVENT_VOLTS.store(volts.to_bits(), Ordering::Relaxed);
        EVENT_SEQ.fetch_add(1, Ordering::Release);
    }
}
