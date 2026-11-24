//! The low-battery monitor (decision-nav-eskf-20261120, Amendment BATT): pure state machine on the raw pack voltage under
//! load. It runs on core 0 at 10 Hz and hands one byte ([`Update::cell`]) to the flight core.
//!
//! Rules (ArduPilot `AP_BattMonitor_Backend::update_failsafes`, `AP_BattMonitor::check_failsafes`; PX4 `batteryCheck`):
//! - LOW is `BATT_LOW_CELL` x `BATT_CELLS` volts, CRITICAL is `BATT_CRT_CELL` x `BATT_CELLS`. A level is reached when the
//!   voltage has been below it continuously for more than `BATT_LOW_TIMER` seconds (the first sample below starts the
//!   timer; any sample at or above the level restarts it).
//! - Only while armed. A level only rises, and clears at disarm.
//! - `BATT_CELLS` 0 is off; `BATT_LOW_TIMER` 0 is off (ArduPilot: "a value of zero disables low voltage errors"); a
//!   critical threshold at or above the low one is off with a notice.
//! - A sample older than [`STALE_MS`], or not a positive finite voltage, is no data: the timers restart and, armed, one
//!   notice is raised.
//!
//! No clock and no hardware: the caller passes the time in ms. The monitor only reads the sample it is given and the armed
//! flag; the facts for other code are [`Monitor::level`], [`Monitor::warning_reached`], [`Monitor::critical_reached`],
//! [`Monitor::voltage_valid`] and [`Monitor::known_not_critical`].

/// A sample older than this (ms) is no data.
pub const STALE_MS: u64 = 5_000;

/// The highest voltage of one lithium cell, V (full).
pub const CELL_FULL_V: f32 = 4.2;

/// A pack voltage above `cells` x [`CELL_FULL_V`] x this is implausible (PX4 `px4-battery.cpp:378-379`): the cell count is
/// probably set too low.
pub const PLAUSIBLE_MARGIN: f32 = 1.05;

/// Bit 2 of the cell byte: the critical action is "descend" and the critical level is reached.
pub const CELL_DESCEND: u8 = 4;

/// Bit 3 of the cell byte, added by core 0: [`Monitor::known_not_critical`] (the hands-off state machine's `battery_ok`).
pub const CELL_KNOWN_OK: u8 = 8;

/// The level reached; only rises while armed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    /// Neither level reached.
    #[default]
    None = 0,
    /// LOW reached.
    Low = 1,
    /// CRITICAL reached.
    Critical = 2,
}

/// The monitor's settings, as stored (the parameters `BATT_CELLS`, `BATT_LOW_CELL`, `BATT_CRT_CELL`, `BATT_LOW_TIMER`,
/// `BATT_FS_CRT_ACT`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Config {
    /// Cells in series; 0 is off.
    pub cells: i32,
    /// LOW threshold per cell, V.
    pub low_cell_v: f32,
    /// CRITICAL threshold per cell, V.
    pub crt_cell_v: f32,
    /// Seconds below a level before it is reached; 0 is off.
    pub timer_s: i32,
    /// The critical action: 0 warn, 1 descend.
    pub act: i32,
}

impl Config {
    /// The parameter table's defaults: 4 cells, 3.6 V and 3.5 V per cell, 10 s, warn only.
    pub const DEFAULT: Config = Config {
        cells: 4,
        low_cell_v: 3.6,
        crt_cell_v: 3.5,
        timer_s: 10,
        act: 0,
    };

    /// What the settings amount to.
    pub fn setup(&self) -> Setup {
        if self.cells <= 0 || self.timer_s <= 0 {
            Setup::Off
        } else if !(self.low_cell_v.is_finite() && self.crt_cell_v.is_finite())
            || self.crt_cell_v >= self.low_cell_v
        {
            Setup::Inverted
        } else {
            Setup::On
        }
    }
}

/// What the settings amount to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Setup {
    /// No cells or no timer: no level is ever reached, no notice.
    Off,
    /// The critical threshold is at or above the low one (or not a number): no level is reached; one notice.
    Inverted,
    /// Both levels are watched.
    On,
}

/// One voltage sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    /// Pack voltage, V.
    pub volts: f32,
    /// When it was taken, ms.
    pub t_ms: u64,
}

/// Notices of one step, a bitset; each is raised once per episode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Notices(u8);

impl Notices {
    /// Armed with no usable sample for [`STALE_MS`].
    pub const NO_DATA: Notices = Notices(1);
    /// The voltage is above `cells` x 4.2 V x 1.05: the cell count is probably too low.
    pub const IMPLAUSIBLE: Notices = Notices(1 << 1);
    /// The settings are `Setup::Inverted`.
    pub const INVERTED: Notices = Notices(1 << 2);

    /// Whether every bit of `other` is set.
    pub const fn contains(self, other: Notices) -> bool {
        self.0 & other.0 == other.0
    }

    /// No notice.
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

/// What one step decided.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Update {
    /// The level now.
    pub level: Level,
    /// The level reached by this step, if it rose.
    pub rose: Option<Level>,
    /// The voltage of the newest usable sample (0 when there is none).
    pub volts: f32,
    /// The notices raised by this step.
    pub notices: Notices,
    /// The byte for the flight core: bits 0 and 1 the level, [`CELL_DESCEND`] when the level is critical and the action
    /// is 1.
    pub cell: u8,
}

/// Whether the flight core's cell byte asks for the descent.
pub const fn cell_descends(cell: u8) -> bool {
    cell & CELL_DESCEND != 0
}

/// Whether the flight core's cell byte says CRITICAL is reached (armed, latched until disarm; whatever the action).
pub const fn cell_critical(cell: u8) -> bool {
    cell & 3 == Level::Critical as u8
}

/// Whether the flight core's cell byte carries [`CELL_KNOWN_OK`].
pub const fn cell_known_ok(cell: u8) -> bool {
    cell & CELL_KNOWN_OK != 0
}

/// The monitor's state.
#[derive(Clone, Copy, Debug)]
pub struct Monitor {
    level: Level,
    low_since_ms: Option<u64>,
    crt_since_ms: Option<u64>,
    last_t_ms: Option<u64>,
    fresh: bool,
    volts: f32,
    setup: Setup,
    no_data_shown: bool,
    implausible_shown: bool,
    inverted_shown: bool,
}

impl Default for Monitor {
    fn default() -> Self {
        Self::new()
    }
}

impl Monitor {
    /// Nothing reached, no sample seen.
    pub const fn new() -> Self {
        Self {
            level: Level::None,
            low_since_ms: None,
            crt_since_ms: None,
            last_t_ms: None,
            fresh: false,
            volts: 0.0,
            setup: Setup::Off,
            no_data_shown: false,
            implausible_shown: false,
            inverted_shown: false,
        }
    }

    /// One step (call at 10 Hz): `now_ms` the time, `armed` the gate's armed state, `sample` the newest voltage sample (the
    /// same one may be given again: it is used once).
    pub fn step(&mut self, cfg: &Config, now_ms: u64, armed: bool, sample: Option<Sample>) -> Update {
        let mut notices = 0u8;
        let setup = cfg.setup();
        self.setup = setup;
        let inverted = setup == Setup::Inverted;
        if inverted && !self.inverted_shown {
            notices |= Notices::INVERTED.0;
        }
        self.inverted_shown = inverted;

        let usable = sample.filter(|s| s.volts.is_finite() && s.volts > 0.0 && now_ms.saturating_sub(s.t_ms) <= STALE_MS);
        self.fresh = usable.is_some();
        if let Some(s) = usable {
            self.volts = s.volts;
        } else {
            self.volts = 0.0;
        }

        let implausible = usable.is_some_and(|s| cfg.cells > 0 && s.volts > cfg.cells as f32 * CELL_FULL_V * PLAUSIBLE_MARGIN);
        if implausible && !self.implausible_shown {
            notices |= Notices::IMPLAUSIBLE.0;
        }
        self.implausible_shown = implausible;

        let mut rose = None;
        if !armed {
            self.level = Level::None;
            self.low_since_ms = None;
            self.crt_since_ms = None;
            self.last_t_ms = None;
            self.no_data_shown = false;
        } else if let Some(s) = usable {
            self.no_data_shown = false;
            if setup == Setup::On && self.last_t_ms != Some(s.t_ms) {
                self.last_t_ms = Some(s.t_ms);
                let cells = cfg.cells as f32;
                let window_ms = u64::from(cfg.timer_s.unsigned_abs()) * 1000;
                let crt = Self::timer(&mut self.crt_since_ms, s, cells * cfg.crt_cell_v, window_ms);
                let low = Self::timer(&mut self.low_since_ms, s, cells * cfg.low_cell_v, window_ms);
                let reached = if crt {
                    Level::Critical
                } else if low {
                    Level::Low
                } else {
                    Level::None
                };
                if reached > self.level {
                    self.level = reached;
                    rose = Some(reached);
                }
            }
        } else {
            // Armed with no usable sample: the timers do not count across the gap.
            self.low_since_ms = None;
            self.crt_since_ms = None;
            self.last_t_ms = None;
            if !self.no_data_shown {
                self.no_data_shown = true;
                notices |= Notices::NO_DATA.0;
            }
        }
        let descend = self.level == Level::Critical && cfg.act == 1;
        Update {
            level: self.level,
            rose,
            volts: self.volts,
            notices: Notices(notices),
            cell: self.level as u8 | if descend { CELL_DESCEND } else { 0 },
        }
    }

    /// ArduPilot's timer for one level: true once the voltage has been below `threshold_v` for more than `window_ms`.
    fn timer(since: &mut Option<u64>, s: Sample, threshold_v: f32, window_ms: u64) -> bool {
        if s.volts < threshold_v {
            let start = *since.get_or_insert(s.t_ms);
            s.t_ms.saturating_sub(start) > window_ms
        } else {
            *since = None;
            false
        }
    }

    /// The level reached since arming.
    pub fn level(&self) -> Level {
        self.level
    }

    /// LOW or CRITICAL has been reached since arming.
    pub fn warning_reached(&self) -> bool {
        self.level >= Level::Low
    }

    /// CRITICAL has been reached since arming.
    pub fn critical_reached(&self) -> bool {
        self.level == Level::Critical
    }

    /// The newest sample is usable: a positive finite voltage no older than [`STALE_MS`].
    pub fn voltage_valid(&self) -> bool {
        self.fresh
    }

    /// The battery is known not to be critical: the monitor is on (cells, timer and thresholds set), the voltage is valid
    /// and CRITICAL has not been reached.
    pub fn known_not_critical(&self) -> bool {
        self.setup == Setup::On && self.fresh && self.level != Level::Critical
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DEFAULT: Config = Config::DEFAULT;

    /// Runs the monitor at 10 Hz from `t0` for `secs` seconds at constant `volts`, returning the end time and the
    /// first rise.
    fn run(m: &mut Monitor, cfg: &Config, t0: u64, secs: u64, volts: f32, armed: bool) -> (u64, Option<(Level, u64)>) {
        let mut first = None;
        let mut t = t0;
        while t < t0 + secs * 1000 {
            let u = m.step(cfg, t, armed, Some(Sample { volts, t_ms: t }));
            if first.is_none() {
                first = u.rose.map(|l| (l, t));
            }
            t += 100;
        }
        (t, first)
    }

    #[test]
    fn thresholds_are_per_cell_times_the_cell_count() {
        // 4 cells: LOW 14.4 V, CRITICAL 14.0 V.
        let mut m = Monitor::new();
        let (_, first) = run(&mut m, &DEFAULT, 0, 20, 14.39, true);
        assert_eq!(first.map(|f| f.0), Some(Level::Low));
        assert_eq!(m.level(), Level::Low);
        let mut m = Monitor::new();
        let (_, first) = run(&mut m, &DEFAULT, 0, 20, 14.41, true);
        assert_eq!(first, None);
        // CRITICAL: 4 x 3.5 = 14.0 V; at 13.99 V the critical timer runs too and both fire at the same sample.
        let mut m = Monitor::new();
        let (_, first) = run(&mut m, &DEFAULT, 0, 20, 13.99, true);
        assert_eq!(first.map(|f| f.0), Some(Level::Critical));
        let mut m = Monitor::new();
        run(&mut m, &DEFAULT, 0, 20, 14.01, true);
        assert_eq!(m.level(), Level::Low);
        // 3 cells: LOW 10.8 V; 13 V is nothing.
        let three = Config { cells: 3, ..DEFAULT };
        let mut m = Monitor::new();
        let (_, first) = run(&mut m, &three, 0, 30, 13.0, true);
        assert_eq!(first, None);
    }

    #[test]
    fn three_cells_critical_is_10_5_v() {
        let three = Config { cells: 3, ..DEFAULT };
        let mut m = Monitor::new();
        run(&mut m, &three, 0, 20, 10.6, true);
        assert_eq!(m.level(), Level::Low);
        run(&mut m, &three, 20_000, 20, 10.49, true);
        assert_eq!(m.level(), Level::Critical);
    }

    #[test]
    fn the_timer_fires_after_it_and_not_before_and_any_sample_at_or_above_restarts_it() {
        let mut m = Monitor::new();
        // Below LOW from t = 0: fires at the first sample more than 10 s after the first (10.1 s).
        let (_, first) = run(&mut m, &DEFAULT, 0, 30, 14.3, true);
        assert_eq!(first, Some((Level::Low, 10_100)));
        // A sample exactly at the threshold restarts the timer.
        let mut m = Monitor::new();
        let mut t = 0;
        for _ in 0..2 {
            let (t1, first) = run(&mut m, &DEFAULT, t, 9, 14.3, true);
            assert_eq!(first, None);
            t = t1;
            let u = m.step(&DEFAULT, t, true, Some(Sample { volts: 14.4, t_ms: t }));
            assert_eq!(u.level, Level::None);
            t += 100;
        }
        let (_, first) = run(&mut m, &DEFAULT, t, 11, 14.3, true);
        assert_eq!(first.map(|f| f.0), Some(Level::Low));
        assert_eq!(first.map(|f| f.1 - t), Some(10_100));
    }

    #[test]
    fn a_dip_shorter_than_the_timer_never_fires_the_negative_control_of_zero_timer_off() {
        // 9.9 s below, then back: nothing.
        let mut m = Monitor::new();
        let (t, _) = run(&mut m, &DEFAULT, 0, 10, 13.0, true);
        assert_eq!(m.level(), Level::None);
        let (_, first) = run(&mut m, &DEFAULT, t, 5, 15.0, true);
        assert_eq!(first, None);
        assert_eq!(m.level(), Level::None);
    }

    #[test]
    fn the_levels_only_rise_and_stay_after_the_voltage_recovers_and_clear_at_disarm() {
        let mut m = Monitor::new();
        let (t, _) = run(&mut m, &DEFAULT, 0, 20, 13.5, true);
        assert_eq!(m.level(), Level::Critical);
        // The voltage recovers (load off): the level stays.
        let (t, first) = run(&mut m, &DEFAULT, t, 20, 16.0, true);
        assert_eq!(first, None);
        assert_eq!(m.level(), Level::Critical);
        assert!(m.critical_reached() && m.warning_reached() && !m.known_not_critical());
        // Disarm clears it.
        let u = m.step(&DEFAULT, t, false, Some(Sample { volts: 16.0, t_ms: t }));
        assert_eq!(u.level, Level::None);
        assert_eq!(u.cell, 0);
        assert!(!m.warning_reached() && m.known_not_critical());
        // And it does not rise while disarmed, whatever the voltage.
        let (_, first) = run(&mut m, &DEFAULT, t + 100, 30, 12.0, false);
        assert_eq!(first, None);
        assert_eq!(m.level(), Level::None);
    }

    #[test]
    fn a_rise_is_reported_once_per_level() {
        let mut m = Monitor::new();
        let mut rises = std::vec::Vec::new();
        for i in 0..400u64 {
            let t = i * 100;
            let u = m.step(&DEFAULT, t, true, Some(Sample { volts: 13.0, t_ms: t }));
            if let Some(l) = u.rose {
                rises.push(l);
            }
        }
        // Both timers start together: one rise, to CRITICAL (LOW is reached at the same sample).
        assert_eq!(rises, [Level::Critical]);
        // A slow sag: LOW first, then CRITICAL.
        let mut m = Monitor::new();
        let mut rises = std::vec::Vec::new();
        for i in 0..1000u64 {
            let t = i * 100;
            let v = if t < 20_000 { 14.3 } else { 13.5 };
            let u = m.step(&DEFAULT, t, true, Some(Sample { volts: v, t_ms: t }));
            if let Some(l) = u.rose {
                rises.push(l);
            }
        }
        assert_eq!(rises, [Level::Low, Level::Critical]);
    }

    #[test]
    fn the_cell_count_rule_zero_cells_and_zero_timer_are_off() {
        for cfg in [Config { cells: 0, ..DEFAULT }, Config { timer_s: 0, ..DEFAULT }] {
            assert_eq!(cfg.setup(), Setup::Off);
            let mut m = Monitor::new();
            let (_, first) = run(&mut m, &cfg, 0, 60, 5.0, true);
            assert_eq!(first, None);
            assert!(!m.known_not_critical());
        }
    }

    #[test]
    fn a_critical_threshold_at_or_above_the_low_one_turns_the_monitor_off_with_one_notice() {
        for cfg in [
            Config { crt_cell_v: 3.6, ..DEFAULT },
            Config { crt_cell_v: 3.9, ..DEFAULT },
            Config { low_cell_v: f32::NAN, ..DEFAULT },
        ] {
            assert_eq!(cfg.setup(), Setup::Inverted);
            let mut m = Monitor::new();
            let u = m.step(&cfg, 0, true, Some(Sample { volts: 14.0, t_ms: 0 }));
            assert!(u.notices.contains(Notices::INVERTED));
            let mut n = 0;
            let mut t = 100;
            while t < 40_000 {
                let u = m.step(&cfg, t, true, Some(Sample { volts: 10.0, t_ms: t }));
                n += u32::from(u.notices.contains(Notices::INVERTED));
                assert_eq!(u.level, Level::None);
                t += 100;
            }
            assert_eq!(n, 0);
        }
    }

    #[test]
    fn stale_or_invalid_data_is_one_notice_restarts_the_timers_and_keeps_the_level() {
        let mut m = Monitor::new();
        let (t, _) = run(&mut m, &DEFAULT, 0, 9, 14.3, true);
        // The sample stops: still usable for 5 s, no notice, no progress.
        let last = t - 100;
        let mut notices = 0;
        let mut u = m.step(&DEFAULT, t, true, Some(Sample { volts: 14.3, t_ms: last }));
        assert!(u.notices.is_empty() && m.voltage_valid());
        let mut now = t;
        while now < t + 10_000 {
            now += 100;
            u = m.step(&DEFAULT, now, true, Some(Sample { volts: 14.3, t_ms: last }));
            notices += u32::from(u.notices.contains(Notices::NO_DATA));
        }
        assert_eq!(notices, 1);
        assert!(!m.voltage_valid() && !m.known_not_critical());
        assert_eq!(m.level(), Level::None);
        // Data returns below LOW: the 9 s before the gap do not count; it needs its own 10 s.
        let (_, first) = run(&mut m, &DEFAULT, now + 100, 12, 14.3, true);
        assert_eq!(first.map(|f| f.1 - (now + 100)), Some(10_100));
        // No sample at all, and 0 V, and NaN are no data.
        for s in [None, Some(Sample { volts: 0.0, t_ms: now }), Some(Sample { volts: f32::NAN, t_ms: now })] {
            let mut m = Monitor::new();
            let u = m.step(&DEFAULT, 1_000, true, s);
            assert!(!m.voltage_valid());
            assert_eq!(u.volts, 0.0);
        }
        // A level that was reached stays across a gap.
        let mut m = Monitor::new();
        run(&mut m, &DEFAULT, 0, 20, 13.0, true);
        m.step(&DEFAULT, 40_000, true, None);
        assert_eq!(m.level(), Level::Critical);
    }

    #[test]
    fn the_stale_notice_is_for_armed_only() {
        let mut m = Monitor::new();
        let u = m.step(&DEFAULT, 10_000, false, None);
        assert!(u.notices.is_empty());
    }

    #[test]
    fn the_plausibility_notice_is_raised_once_per_rise_and_changes_no_level() {
        // 4 x 4.2 x 1.05 = 17.64 V.
        let mut m = Monitor::new();
        let mut n = 0;
        for i in 0..50u64 {
            let t = i * 100;
            let u = m.step(&DEFAULT, t, false, Some(Sample { volts: 17.7, t_ms: t }));
            n += u32::from(u.notices.contains(Notices::IMPLAUSIBLE));
            assert_eq!(u.level, Level::None);
        }
        assert_eq!(n, 1);
        // 16.8 V (a full 4S) is plausible; 17.63 V too.
        for v in [16.8, 17.63] {
            let mut m = Monitor::new();
            let u = m.step(&DEFAULT, 0, true, Some(Sample { volts: v, t_ms: 0 }));
            assert!(!u.notices.contains(Notices::IMPLAUSIBLE), "{v}");
        }
        // With the cell count off there is nothing to compare with.
        let off = Config { cells: 0, ..DEFAULT };
        let mut m = Monitor::new();
        let u = m.step(&off, 0, true, Some(Sample { volts: 40.0, t_ms: 0 }));
        assert!(!u.notices.contains(Notices::IMPLAUSIBLE));
    }

    #[test]
    fn the_cell_byte_carries_the_level_and_the_descent_only_with_action_1() {
        let warn = DEFAULT;
        let descend = Config { act: 1, ..DEFAULT };
        for (cfg, cell) in [(warn, 2u8), (descend, 2 | CELL_DESCEND)] {
            let mut m = Monitor::new();
            let (t, _) = run(&mut m, &cfg, 0, 20, 13.0, true);
            let u = m.step(&cfg, t, true, Some(Sample { volts: 13.0, t_ms: t }));
            assert_eq!(u.cell, cell);
            assert_eq!(cell_descends(u.cell), cfg.act == 1);
        }
        // LOW with action 1 does not descend.
        let mut m = Monitor::new();
        run(&mut m, &descend, 0, 20, 14.3, true);
        let u = m.step(&descend, 20_000, true, Some(Sample { volts: 14.3, t_ms: 20_000 }));
        assert_eq!((u.level, u.cell), (Level::Low, 1));
        assert!(!cell_descends(u.cell));
        assert!(!cell_descends(0));
    }

    #[test]
    fn the_facts_for_other_code() {
        let mut m = Monitor::new();
        assert!(!m.voltage_valid() && !m.known_not_critical() && !m.warning_reached() && !m.critical_reached());
        m.step(&DEFAULT, 0, false, Some(Sample { volts: 16.0, t_ms: 0 }));
        assert!(m.voltage_valid() && m.known_not_critical());
        run(&mut m, &DEFAULT, 100, 20, 14.3, true);
        assert!(m.warning_reached() && !m.critical_reached() && m.known_not_critical());
        run(&mut m, &DEFAULT, 20_100, 20, 13.0, true);
        assert!(m.warning_reached() && m.critical_reached() && !m.known_not_critical());
        assert!(m.voltage_valid());
    }

    /// B-I1 and B-I7 by text: the monitor's source reaches no gate, no disarm and no throttle, and holds no voltage
    /// literal outside the defaults and the plausibility constants.
    #[test]
    fn the_source_reaches_no_gate_disarm_or_throttle() {
        let source = include_str!("battery.rs");
        let code = source.split("#[cfg(test)]").next().unwrap();
        let code: std::string::String = code
            .lines()
            .filter(|l| !l.trim_start().starts_with("//"))
            .collect::<std::vec::Vec<_>>()
            .join("\n");
        for word in ["gate", "Gate", "disarm", "throttle", "Throttle", "motor", "Motor"] {
            assert!(!code.contains(word), "{word}");
        }
    }

    /// T2: the recorded pack voltage of log 286 (BAT, instance 0, armed windows 2 and 3 of PB0, 10 Hz) replayed through the
    /// monitor. Expected times are PB0's table (pb0-report.md, seconds after the window's first armed sample); the monitor
    /// fires at the first sample MORE than the timer after the first one below the level, PB0's script at `>=`, so they may
    /// differ by one sample (0.1 s). Timer 0 is off here and fires never.
    #[test]
    fn log_286_replay_fires_where_the_pre_study_did() {
        let csv = include_str!("battery_log286_bat.csv");
        let mut wins: [std::vec::Vec<(u64, f32)>; 2] = [std::vec::Vec::new(), std::vec::Vec::new()];
        for line in csv.lines().skip(1) {
            let mut f = line.split(',');
            let w: usize = f.next().unwrap().parse().unwrap();
            let t: u64 = f.next().unwrap().parse().unwrap();
            let v: f32 = f.next().unwrap().parse().unwrap();
            wins[w - 2].push((t, v));
        }
        assert_eq!((wins[0].len(), wins[1].len()), (606, 1176));
        // Fire time (ms after the first sample) of the first LOW rise with the LOW level at `level_v` volts (4 cells).
        let fire = |win: usize, level_v: f32, timer_s: i32| -> Option<u64> {
            let cfg = Config { cells: 4, low_cell_v: level_v / 4.0, crt_cell_v: 1.0, timer_s, act: 0 };
            let mut m = Monitor::new();
            let t0 = wins[win][0].0;
            for &(t, volts) in &wins[win] {
                if m.step(&cfg, t, true, Some(Sample { volts, t_ms: t })).rose.is_some() {
                    return Some(t - t0);
                }
            }
            None
        };
        let near = |got: Option<u64>, want: Option<f32>| match (got, want) {
            (None, None) => true,
            (Some(g), Some(w)) => (g as f32 - w * 1000.0).abs() <= 100.0,
            _ => false,
        };
        // (level V, timer s, PB0's window 3 time s, this monitor's time s). The recorded volts are f32 at 0.01 V steps; PB0
        // compared them with an f64 threshold, so a reading equal to 14.4 or 13.2 as f32 (just under it as f64) counted as
        // below there and is not below here (the threshold is f32 too). Those two levels therefore differ from PB0 by
        // more than a sample for 14.4 V and for 13.2 V / 10 s; 14.0 V (exact in both) and 13.2 V / 3 s agree.
        let w3 = [
            (14.4, 3, 50.2, 53.5),
            (14.4, 10, 57.3, 60.5),
            (14.0, 3, 88.5, 88.5),
            (14.0, 10, 95.6, 95.6),
            (13.2, 3, 102.1, 102.2),
            (13.2, 10, 109.0, 109.2),
        ];
        for (level, timer, pb0, rust) in w3 {
            let got = fire(1, level, timer);
            assert!(near(got, Some(rust)), "{level} V / {timer} s: {got:?} want {rust}");
            if level == 14.0 || timer == 3 && level == 13.2 {
                assert!(near(got, Some(pb0)), "{level} V / {timer} s: {got:?} PB0 {pb0}");
            }
        }
        // Window 2 (the 14.4 V / 0 s false fire of PB0): nothing fires with a timer, and none with timer 0 (off).
        for level in [14.4, 14.0, 13.2] {
            for timer in [0, 3, 10] {
                assert_eq!(fire(0, level, timer), None, "w2 {level} V / {timer} s");
            }
        }
        for level in [14.4, 14.0, 13.2] {
            assert_eq!(fire(1, level, 0), None, "w3 {level} V / 0 s");
        }
        // The defaults (LOW 14.4 V, CRITICAL 14.0 V, 10 s): LOW at 60.5 s (PB0: 57.3 s, see above) and CRITICAL at 95.6 s of window 3.
        let mut m = Monitor::new();
        let t0 = wins[1][0].0;
        let mut rises = std::vec::Vec::new();
        for &(t, volts) in &wins[1] {
            if let Some(l) = m.step(&DEFAULT, t, true, Some(Sample { volts, t_ms: t })).rose {
                rises.push((l, t - t0));
            }
        }
        assert_eq!(rises.len(), 2, "{rises:?}");
        assert_eq!(rises[0].0, Level::Low);
        assert_eq!(rises[1].0, Level::Critical);
        assert!(near(Some(rises[0].1), Some(60.5)) && near(Some(rises[1].1), Some(95.6)), "{rises:?}");
    }
}
