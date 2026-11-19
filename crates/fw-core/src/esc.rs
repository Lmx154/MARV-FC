//! ESC telemetry on core 0's side: the plain data the flight core publishes about the four DShot
//! lines ([`EscStatus`]), the two MAVLink messages built from it ([`esc_status`] ESC_STATUS 291,
//! [`esc_info`] ESC_INFO 290) and the ESC device row ([`verdict`], [`device_status`]).
//! Design: `decision-esc-telemetry-20261118` Q2, Q3.8 and Q4.
//!
//! The data types live here, not in `fw_flight::esc` (which re-exports them), because `fw-flight`
//! depends on `fw-core` and the builders need the type. Nothing here knows the DShot codec: the
//! flight core stores decoded plain numbers (eRPM, degrees, millivolts, milliamps).
//!
//! Line numbers are the driver's: line 0 is GPIO36 (M4) ... line 3 is GPIO39 (M1). Message element
//! `e` is motor `e + 1`, which is line [`line_of_element`]`(e)`: that mapping is in one place.
//!
//! Ages: a value is *answering* when the line's latest good reply is younger than [`ANSWER_US`]
//! (strictly: 100 ms or more is a dropout); a value or a snapshot is *fresh* when it is not older
//! than [`FRESH_US`] (exactly 1 s still counts; "older than 1 s" is stale).

use crate::health::{Counters, ErrorKind, State};
use crate::sys_status::{StatusInput, device_status as row_message};
use mavlink_device::dialect::{ESC_INFO_DATA, ESC_STATUS_DATA, EscConnectionType};
use mavlink_device::marv::{MARV_DEVICE_STATUS_DATA, MarvDevice};

/// The number of DShot lines (ESCs).
pub const LINES: usize = 4;

/// A line is answering while its latest good reply is younger than this, in us.
pub const ANSWER_US: u64 = 100_000;

/// A value or a snapshot older than this is stale, in us.
pub const FRESH_US: u64 = 1_000_000;

/// After a start sequence ended, the row says STARTING for this long while not all lines answer, in us.
pub const STARTING_US: u64 = 5_000_000;

/// An eRPM value above this is AM32's "not started yet" report, not a speed: it is not stored.
pub const ERPM_MAX: u32 = 1_000_000;

/// The driver line of message element `element` (0 = M1 = line 3 ... 3 = M4 = line 0). The mapping
/// is its own inverse. `None` for an element above 3.
pub fn line_of_element(element: usize) -> Option<usize> {
    LINES
        .checked_sub(1)
        .and_then(|last| last.checked_sub(element))
}

/// What the flight core knows about one line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EscLine {
    /// Captures that were a good reply (eRPM or extended).
    pub good: u32,
    /// Captures that were not a reply (silent, invalid or foreign).
    pub non_replies: u32,
    /// Times the line went from answering to not answering.
    pub dropouts: u32,
    /// Command-13 bursts started on this line.
    pub edt_bursts: u32,
    /// When the line's current run of answering began, us (`None`: not answering).
    pub answering_since_us: Option<u64>,
    /// When the latest good reply was captured, us.
    pub last_good_us: Option<u64>,
    /// When the latest extended reply (including the acknowledge) was captured, us.
    pub last_extended_us: Option<u64>,
    /// The latest eRPM (0 for a stopped motor) and when it was captured, us.
    pub erpm: Option<(u32, u64)>,
    /// The latest temperature in degrees Celsius and its time, us.
    pub temperature_c: Option<(u8, u64)>,
    /// The latest voltage in millivolts and its time, us.
    pub voltage_mv: Option<(u32, u64)>,
    /// The latest current in milliamps and its time, us.
    pub current_ma: Option<(u32, u64)>,
}

impl EscLine {
    /// Nothing seen.
    pub const INITIAL: EscLine = EscLine {
        good: 0,
        non_replies: 0,
        dropouts: 0,
        edt_bursts: 0,
        answering_since_us: None,
        last_good_us: None,
        last_extended_us: None,
        erpm: None,
        temperature_c: None,
        voltage_mv: None,
        current_ma: None,
    };

    /// The line gave a good reply less than [`ANSWER_US`] before `now_us`.
    pub fn answering(&self, now_us: u64) -> bool {
        self.last_good_us
            .is_some_and(|t| now_us.saturating_sub(t) < ANSWER_US)
    }
}

/// Everything the ESC task publishes to core 0, after every exchange (and at the start sequence's
/// two ends). Plain `Copy` data for an `xcore::Seqlock`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EscStatus {
    /// Exchanges (one frame to each of the four lines) since boot.
    pub exchanges: u32,
    /// Board time of the latest exchange, us.
    pub last_exchange_us: u64,
    /// Start sequences begun since boot.
    pub starts: u32,
    /// A start sequence is running now.
    pub start_running: bool,
    /// When the latest start sequence ended, us.
    pub start_ended_us: Option<u64>,
    /// Longest spacing between two published exchanges since boot, us.
    pub max_spacing_us: u32,
    /// Longest `exchange()` duration since boot, us.
    pub max_exchange_us: u32,
    /// Shortest `exchange()` duration since boot, us; `u32::MAX` before the first exchange.
    pub min_exchange_us: u32,
    /// Per driver line.
    pub lines: [EscLine; LINES],
}

impl EscStatus {
    /// Before the first start.
    pub const INITIAL: EscStatus = EscStatus {
        exchanges: 0,
        last_exchange_us: 0,
        starts: 0,
        start_running: false,
        start_ended_us: None,
        max_spacing_us: 0,
        max_exchange_us: 0,
        min_exchange_us: u32::MAX,
        lines: [EscLine::INITIAL; LINES],
    };

    /// The newest event of the snapshot (an exchange or the end of a start), us.
    fn newest_us(&self) -> u64 {
        self.last_exchange_us.max(self.start_ended_us.unwrap_or(0))
    }

    /// The snapshot is not older than [`FRESH_US`] at `now_us`.
    pub fn is_fresh(&self, now_us: u64) -> bool {
        now_us.saturating_sub(self.newest_us()) <= FRESH_US
    }

    /// How many lines are answering at `now_us`.
    pub fn answering_count(&self, now_us: u64) -> usize {
        self.lines.iter().filter(|l| l.answering(now_us)).count()
    }
}

/// The line of message element `element`.
fn line(status: &EscStatus, element: usize) -> Option<&EscLine> {
    line_of_element(element).and_then(|l| status.lines.get(l))
}

/// ESC_STATUS (291): rpm, voltage and current per motor. `None` when the snapshot is older than
/// 1 s or no exchange has happened. `poles` is `MOT_POLES`.
///
/// - rpm = eRPM / (`poles` / 2), rounded down; 0 for a stopped motor (eRPM 0) and for a line that
///   is not answering.
/// - voltage in volts when received within the last 1 s, else NaN.
/// - current: always NaN (the ESCs' value is not a current).
pub fn esc_status(status: &EscStatus, now_us: u64, poles: i32) -> Option<ESC_STATUS_DATA> {
    if status.exchanges == 0 || !status.is_fresh(now_us) {
        return None;
    }
    let poles = u64::try_from(poles).unwrap_or(0).max(1);
    let mut rpm = [0i32; LINES];
    let mut voltage = [f32::NAN; LINES];
    for (element, (r, v)) in rpm.iter_mut().zip(voltage.iter_mut()).enumerate() {
        let Some(l) = line(status, element) else {
            continue;
        };
        if l.answering(now_us)
            && let Some((erpm, _)) = l.erpm
        {
            *r = i32::try_from(u64::from(erpm) * 2 / poles).unwrap_or(i32::MAX);
        }
        if let Some((mv, t)) = l.voltage_mv
            && now_us.saturating_sub(t) <= FRESH_US
        {
            *v = mv as f32 / 1000.0;
        }
    }
    Some(ESC_STATUS_DATA {
        time_usec: status.last_exchange_us,
        rpm,
        voltage,
        current: [f32::NAN; LINES],
        index: 0,
    })
}

/// ESC_INFO (290). `None` when the snapshot is older than 1 s or no exchange has happened.
///
/// `counter` = exchanges mod 65536; `error_count` = non-replies per motor; `info` bit `e` = motor
/// `e + 1` gave a good reply within 100 ms; `failure_flags` 0; `temperature` in cdegC when received
/// within the last 1 s, else `i16::MAX`; `count` 4, `index` 0, connection type DSHOT.
pub fn esc_info(status: &EscStatus, now_us: u64) -> Option<ESC_INFO_DATA> {
    if status.exchanges == 0 || !status.is_fresh(now_us) {
        return None;
    }
    let mut error_count = [0u32; LINES];
    let mut temperature = [i16::MAX; LINES];
    let mut info = 0u8;
    for (element, (e, t)) in error_count
        .iter_mut()
        .zip(temperature.iter_mut())
        .enumerate()
    {
        let Some(l) = line(status, element) else {
            continue;
        };
        *e = l.non_replies;
        if l.answering(now_us) {
            info |= 1 << element;
        }
        if let Some((celsius, at)) = l.temperature_c
            && now_us.saturating_sub(at) <= FRESH_US
        {
            *t = i16::from(celsius).saturating_mul(100);
        }
    }
    Some(ESC_INFO_DATA {
        time_usec: status.last_exchange_us,
        error_count,
        counter: (status.exchanges & 0xFFFF) as u16,
        failure_flags: [0; LINES],
        temperature,
        index: 0,
        count: LINES as u8,
        connection_type: EscConnectionType::ESC_CONNECTION_TYPE_DSHOT,
        info,
    })
}

/// The ESC row's state and last error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Verdict {
    /// The row's state.
    pub state: State,
    /// The row's last error.
    pub last_error: ErrorKind,
}

/// The ESC row (report only, recomputed for every message, no latching):
///
/// | state | when | error |
/// |---|---|---|
/// | STARTING | a start sequence is running; or none has begun yet (and the snapshot is not stale); or one ended less than 5 s ago and not all four answer | none |
/// | FAILED | the snapshot is older than 1 s | STUCK |
/// | ABSENT | no line has given a good reply since boot | none |
/// | OK | all four answer | none |
/// | DEGRADED | 1 to 3 answer | TIMEOUT |
/// | FAILED | none answers after some good reply | TIMEOUT |
///
/// The rows are tested in the order STARTING (running start), stale, STARTING (window), then the
/// rest. A running start is STARTING whatever the snapshot's age: the task publishes nothing
/// during the 3 s park.
pub fn verdict(status: &EscStatus, now_us: u64) -> Verdict {
    let v = |state, last_error| Verdict { state, last_error };
    if status.start_running {
        return v(State::Starting, ErrorKind::None);
    }
    if !status.is_fresh(now_us) {
        return v(State::Failed, ErrorKind::Stuck);
    }
    if status.starts == 0 {
        return v(State::Starting, ErrorKind::None);
    }
    let answering = status.answering_count(now_us);
    let in_window = status
        .start_ended_us
        .is_some_and(|t| now_us.saturating_sub(t) < STARTING_US);
    if answering < LINES && in_window {
        return v(State::Starting, ErrorKind::None);
    }
    if answering == LINES {
        return v(State::Ok, ErrorKind::None);
    }
    if answering > 0 {
        return v(State::Degraded, ErrorKind::Timeout);
    }
    if status.lines.iter().any(|l| l.good > 0) {
        v(State::Failed, ErrorKind::Timeout)
    } else {
        v(State::Absent, ErrorKind::None)
    }
}

/// The ESC row's `MARV_DEVICE_STATUS`: samples = good replies (sum of four), errors = non-replies
/// (sum), timeouts = dropouts (sum), attempts = starts, recoveries, overflows and config_restored
/// 0, `last_sample_age_ms` = age of the newest good reply on any line. The state and error are
/// [`verdict`]'s (a stale snapshot goes out as STUCK).
pub fn device_status(status: &EscStatus, now_us: u64) -> MARV_DEVICE_STATUS_DATA {
    let v = verdict(status, now_us);
    let sum = |f: fn(&EscLine) -> u32| {
        status
            .lines
            .iter()
            .fold(0u32, |acc, l| acc.saturating_add(f(l)))
    };
    let newest = status.lines.iter().filter_map(|l| l.last_good_us).max();
    let ms = |us: u64| u32::try_from(us / 1000).unwrap_or(u32::MAX);
    row_message(&StatusInput {
        device: MarvDevice::MARV_DEVICE_ESC,
        state: v.state,
        counters: Counters {
            errors: sum(|l| l.non_replies),
            timeouts: sum(|l| l.dropouts),
            recoveries: 0,
            attempts: u8::try_from(status.starts).unwrap_or(u8::MAX),
            config_restored: 0,
            overflows: 0,
            last_error: v.last_error,
            present: newest.is_some(),
        },
        stuck: v.last_error == ErrorKind::Stuck,
        samples: sum(|l| l.good),
        last_sample_age_ms: newest.map(|t| ms(now_us.saturating_sub(t))),
        hil: false,
        self_test: false,
        time_boot_ms: ms(now_us),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const NOW: u64 = 50_000_000;

    /// A status at NOW with every line answering, stopped motor, no extended data.
    fn live() -> EscStatus {
        let mut s = EscStatus::INITIAL;
        s.exchanges = 70_000;
        s.last_exchange_us = NOW;
        s.starts = 1;
        s.start_ended_us = Some(NOW - 40_000_000);
        for l in &mut s.lines {
            l.good = 100;
            l.last_good_us = Some(NOW);
            l.answering_since_us = Some(NOW - 10_000_000);
            l.erpm = Some((0, NOW));
        }
        s
    }

    #[test]
    fn motor_order_is_reversed_and_its_own_inverse() {
        for e in 0..LINES {
            let l = line_of_element(e).unwrap();
            assert_eq!(l, LINES - 1 - e);
            assert_eq!(line_of_element(l), Some(e));
        }
        assert_eq!(line_of_element(0), Some(3));
        assert_eq!(line_of_element(4), None);
    }

    #[test]
    fn nothing_is_built_when_the_snapshot_is_older_than_one_second() {
        let s = live();
        assert!(esc_status(&s, NOW + FRESH_US, 14).is_some());
        assert!(esc_info(&s, NOW + FRESH_US).is_some());
        assert!(esc_status(&s, NOW + FRESH_US + 1, 14).is_none());
        assert!(esc_info(&s, NOW + FRESH_US + 1).is_none());
        assert!(esc_status(&EscStatus::INITIAL, 0, 14).is_none());
        assert!(esc_info(&EscStatus::INITIAL, 0).is_none());
    }

    #[test]
    fn rpm_uses_pole_pairs_rounds_down_and_follows_the_motor_order() {
        let mut s = live();
        // line 3 = M1 = element 0; line 0 = M4 = element 3.
        s.lines[3].erpm = Some((70_000, NOW));
        s.lines[0].erpm = Some((70_013, NOW));
        let m = esc_status(&s, NOW, 14).unwrap();
        assert_eq!(m.rpm, [10_000, 0, 0, 10_001]);
        assert_eq!((m.index, m.time_usec), (0, NOW));
        // 7 pole pairs vs 8 pole pairs.
        assert_eq!(esc_status(&s, NOW, 16).unwrap().rpm[0], 8_750);
        // A stopped motor (0xFFF decodes to eRPM 0) is 0.
        s.lines[3].erpm = Some((0, NOW));
        assert_eq!(esc_status(&s, NOW, 14).unwrap().rpm[0], 0);
    }

    #[test]
    fn rpm_is_zero_for_a_line_that_is_not_answering() {
        let mut s = live();
        s.lines[2].erpm = Some((70_000, NOW));
        s.lines[2].last_good_us = Some(NOW - ANSWER_US + 1);
        assert_eq!(esc_status(&s, NOW, 14).unwrap().rpm[1], 10_000);
        s.lines[2].last_good_us = Some(NOW - ANSWER_US);
        assert_eq!(esc_status(&s, NOW, 14).unwrap().rpm[1], 0);
    }

    #[test]
    fn voltage_and_temperature_follow_the_one_second_rule() {
        let mut s = live();
        s.lines[3].voltage_mv = Some((16_250, NOW - FRESH_US));
        s.lines[3].temperature_c = Some((41, NOW - FRESH_US));
        s.lines[2].voltage_mv = Some((16_250, NOW - FRESH_US - 1));
        s.lines[2].temperature_c = Some((41, NOW - FRESH_US - 1));
        let m = esc_status(&s, NOW, 14).unwrap();
        assert_eq!(m.voltage[0], 16.25);
        assert!(m.voltage[1].is_nan() && m.voltage[2].is_nan() && m.voltage[3].is_nan());
        assert!(m.current.iter().all(|c| c.is_nan()));
        let i = esc_info(&s, NOW).unwrap();
        assert_eq!(i.temperature, [4100, i16::MAX, i16::MAX, i16::MAX]);
    }

    #[test]
    fn info_counter_wraps_and_bits_and_errors_follow_the_motor_order() {
        let mut s = live();
        s.exchanges = 65_536 + 7;
        s.lines[3].non_replies = 11;
        s.lines[0].non_replies = 22;
        s.lines[1].last_good_us = Some(NOW - ANSWER_US); // line 1 = M3: dropped out
        let i = esc_info(&s, NOW).unwrap();
        assert_eq!(i.counter, 7);
        assert_eq!(i.error_count, [11, 0, 0, 22]);
        assert_eq!(i.info, 0b1011);
        assert_eq!(i.failure_flags, [0; 4]);
        assert_eq!((i.index, i.count), (0, 4));
        assert_eq!(
            i.connection_type,
            EscConnectionType::ESC_CONNECTION_TYPE_DSHOT
        );
        assert_eq!(i.time_usec, NOW);
        s.exchanges = 65_535;
        assert_eq!(esc_info(&s, NOW).unwrap().counter, 65_535);
    }

    #[test]
    fn no_battery_reads_as_nan_int16_max_zero_rpm_and_no_bits() {
        let mut s = EscStatus::INITIAL;
        s.exchanges = 5_000;
        s.last_exchange_us = NOW;
        s.starts = 1;
        s.start_ended_us = Some(NOW - 5_000_000);
        for l in &mut s.lines {
            l.non_replies = 5_000;
        }
        let m = esc_status(&s, NOW, 14).unwrap();
        assert_eq!(m.rpm, [0; 4]);
        assert!(m.voltage.iter().all(|v| v.is_nan()));
        let i = esc_info(&s, NOW).unwrap();
        assert_eq!((i.info, i.temperature), (0, [i16::MAX; 4]));
        assert_eq!(i.error_count, [5_000; 4]);
    }

    /// (answering lines, ever good, starts, running, ended ago us) -> (state, error)
    #[test]
    fn row_state_table_at_its_boundaries() {
        use ErrorKind as E;
        use State as S;
        let status = |n: usize, ever: bool, starts: u32, running: bool, ended_ago: Option<u64>| {
            let mut s = EscStatus::INITIAL;
            s.exchanges = 1000;
            s.last_exchange_us = NOW;
            s.starts = starts;
            s.start_running = running;
            s.start_ended_us = ended_ago.map(|a| NOW - a);
            for (i, l) in s.lines.iter_mut().enumerate() {
                if ever {
                    l.good = 1;
                    l.last_good_us = Some(NOW - 5_000_000);
                }
                if i < n {
                    l.last_good_us = Some(NOW - 1);
                }
            }
            s
        };
        let old = Some(STARTING_US);
        let young = Some(STARTING_US - 1);
        let cases = [
            // running start
            (status(0, false, 1, true, None), S::Starting, E::None),
            (status(4, true, 1, true, None), S::Starting, E::None),
            // window: ended 5 s - 1 us ago, not all answering
            (status(3, true, 1, false, young), S::Starting, E::None),
            (status(0, false, 1, false, young), S::Starting, E::None),
            // window with all four answering is OK
            (status(4, true, 1, false, young), S::Ok, E::None),
            // window over
            (status(0, false, 1, false, old), S::Absent, E::None),
            (status(4, true, 1, false, old), S::Ok, E::None),
            (status(3, true, 1, false, old), S::Degraded, E::Timeout),
            (status(1, true, 1, false, old), S::Degraded, E::Timeout),
            (status(0, true, 1, false, old), S::Failed, E::Timeout),
        ];
        for (k, (s, state, err)) in cases.into_iter().enumerate() {
            assert_eq!(
                verdict(&s, NOW),
                Verdict {
                    state,
                    last_error: err
                },
                "case {k}"
            );
        }
        // Stale: exactly 1 s old is fresh, 1 s and 1 us is FAILED STUCK; a running start is not.
        let s = status(4, true, 1, false, old);
        assert_ne!(verdict(&s, NOW + FRESH_US).last_error, ErrorKind::Stuck);
        let stale = verdict(&s, NOW + FRESH_US + 1);
        assert_eq!(
            (stale.state, stale.last_error),
            (State::Failed, ErrorKind::Stuck)
        );
        let running = status(4, true, 1, true, None);
        assert_eq!(
            verdict(&running, NOW + 10 * FRESH_US).state,
            State::Starting
        );
        // No start yet: STARTING while young, STUCK once a second has passed.
        let none = EscStatus::INITIAL;
        assert_eq!(verdict(&none, FRESH_US).state, State::Starting);
        assert_eq!(verdict(&none, FRESH_US + 1).last_error, ErrorKind::Stuck);
    }

    #[test]
    fn the_device_row_counts_sums_and_ages() {
        let mut s = live();
        s.starts = 2;
        s.lines[0].good = 5;
        s.lines[1].good = 6;
        s.lines[2].good = 7;
        s.lines[3].good = 8;
        s.lines[0].non_replies = 1;
        s.lines[3].non_replies = 2;
        s.lines[1].dropouts = 4;
        s.lines[2].last_good_us = Some(NOW - 2_500);
        for l in [0usize, 1, 3] {
            s.lines[l].last_good_us = Some(NOW - 9_000);
        }
        let m = device_status(&s, NOW);
        assert_eq!((m.samples, m.errors, m.timeouts, m.attempts), (26, 3, 4, 2));
        assert_eq!((m.recoveries, m.overflows, m.config_restored), (0, 0, 0));
        assert_eq!(m.last_sample_age_ms, 2);
        assert_eq!(m.device_id, MarvDevice::MARV_DEVICE_ESC);
    }
}
