//! The watchdog's pure decisions (design: decision-foundation, two-core amendment, item 6): the
//! monitor's feed rule ([`Monitor`]), the task-id table ([`task`], [`task_name`]) and the boot
//! report ([`boot_report`]).
//!
//! The monitor checks every [`CHECK_MS`]: it feeds only while core 0's beat and the flight core's
//! `PONG` both changed less than [`STALL_MS`] ago. At [`STALL_MS`] of stall it asks for the
//! scratch record once and stops feeding, so the hardware resets the board about
//! [`PERIOD_MS`] after the last feed. A stall that ends before the reset clears the record; one
//! that reached [`TEXT_MS`] earns one text. The check after a flash save (`SAVE_EPOCH` changed)
//! never starts a stall.

use core::fmt;

use crate::scratch::{Reason, Record};

/// The hardware watchdog's period (ms).
pub const PERIOD_MS: u32 = 1600;
/// The monitor's check interval (ms).
pub const CHECK_MS: u32 = 100;
/// A heartbeat that has not changed for this long (ms) is a stall: record, stop feeding.
pub const STALL_MS: u32 = 200;
/// A stall this long (ms) earns one text if the core resumes before the reset.
pub const TEXT_MS: u32 = 500;
/// The earliest reset after the reboot's final ACK was handed to the link (ms).
pub const REBOOT_AFTER_ACK_MS: u32 = 100;
/// The reset after the reboot action ended when no ACK has been handed by then (ms).
pub const REBOOT_NO_ACK_MS: u32 = 1000;

/// The ids written to `RUNNING0` (core 0's thread-mode tasks) and `RUNNING1` (the flight core's
/// tasks) and kept in the scratch record. Names: [`task_name`].
pub mod task {
    /// Nothing has run yet.
    pub const NONE: u8 = 0;
    /// Core 0's `main` task (start-up checks).
    pub const MAIN: u8 = 1;
    /// The USB device task.
    pub const USB: u8 = 2;
    /// The MAVLink node on USB (`usb_link`).
    pub const LINK: u8 = 3;
    /// The supervisor (10 ms tick).
    pub const SUPERVISOR: u8 = 4;
    /// Core 0's beat (`beat0`).
    pub const BEAT: u8 = 5;
    /// A command's action (inside the link task).
    pub const ACTION: u8 = 6;
    /// The settings work of a check feature.
    pub const SETTINGS: u8 = 7;
    /// The barometer's device task.
    pub const BARO: u8 = 8;
    /// A check feature's own task.
    pub const TEST: u8 = 9;
    /// The flight core's liveness task (`fbeat`).
    pub const FBEAT: u8 = 16;
    /// IMU1's device task (flight core).
    pub const IMU1: u8 = 17;
}

/// The name of a task id ("?" for an unknown id).
pub const fn task_name(id: u8) -> &'static str {
    match id {
        task::NONE => "NONE",
        task::MAIN => "MAIN",
        task::USB => "USB",
        task::LINK => "LINK",
        task::SUPERVISOR => "SUPERVISOR",
        task::BEAT => "BEAT",
        task::ACTION => "ACTION",
        task::SETTINGS => "SETTINGS",
        task::BARO => "BARO",
        task::TEST => "TEST",
        task::FBEAT => "FBEAT",
        task::IMU1 => "IMU1",
        _ => "?",
    }
}

/// Which core stalled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Core {
    /// Core 0's thread-mode executor (its beat stopped).
    Core0,
    /// The flight core (its `PONG` stopped).
    Flight,
}

impl Core {
    /// The scratch record's reason for a stall of this core.
    pub const fn reason(self) -> Reason {
        match self {
            Self::Core0 => Reason::StallCore0,
            Self::Flight => Reason::StallFlight,
        }
    }

    const fn label(self) -> &'static str {
        match self {
            Self::Core0 => "core 0",
            Self::Flight => "flight core",
        }
    }
}

/// What the monitor reads at one check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Inputs {
    /// The monitor's clock (ms, wrapping).
    pub now_ms: u32,
    /// `BEAT0`.
    pub beat0: u32,
    /// `PONG`.
    pub pong: u32,
    /// `SAVE_EPOCH`.
    pub save_epoch: u32,
    /// The time (ms, same clock) a requested reset is due; `None` when none was requested.
    pub reboot_at_ms: Option<u32>,
}

/// A stall that ended before the reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ended {
    /// The core that stalled.
    pub core: Core,
    /// The largest age the monitor saw (ms): the stall lasted at least this long.
    pub age_ms: u32,
    /// It reached [`TEXT_MS`]: one text.
    pub text: bool,
}

/// What the monitor does after one check.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Step {
    /// Feed the watchdog.
    pub feed: bool,
    /// A stall began at this check: the core and its age (ms).
    pub began: Option<(Core, u32)>,
    /// Write the scratch record for the stall that began (not while a reboot is pending).
    pub record: bool,
    /// The stall is over: clear the record's magic.
    pub clear: bool,
    /// The stall that ended at this check.
    pub ended: Option<Ended>,
    /// Carry out the requested reset now.
    pub reset: bool,
    /// This check saw `SAVE_EPOCH` changed: it may not start a stall.
    pub save_check: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Stall {
    core: Core,
    age_ms: u32,
    recorded: bool,
    flagged: bool,
}

/// The monitor's state between checks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Monitor {
    beat0: u32,
    beat0_at: u32,
    pong: u32,
    pong_at: u32,
    epoch: u32,
    stall: Option<Stall>,
}

const fn since(now: u32, then: u32) -> u32 {
    now.wrapping_sub(then)
}

impl Monitor {
    /// A monitor started at `i.now_ms`: both heartbeats count as fresh from then.
    pub const fn new(i: Inputs) -> Self {
        Self {
            beat0: i.beat0,
            beat0_at: i.now_ms,
            pong: i.pong,
            pong_at: i.now_ms,
            epoch: i.save_epoch,
            stall: None,
        }
    }

    /// One check.
    pub fn step(&mut self, i: Inputs) -> Step {
        if i.beat0 != self.beat0 {
            self.beat0 = i.beat0;
            self.beat0_at = i.now_ms;
        }
        if i.pong != self.pong {
            self.pong = i.pong;
            self.pong_at = i.now_ms;
        }
        let age0 = since(i.now_ms, self.beat0_at);
        let age1 = since(i.now_ms, self.pong_at);
        let skip = i.save_epoch != self.epoch;
        self.epoch = i.save_epoch;
        let mut step = Step {
            save_check: skip,
            reset: i
                .reboot_at_ms
                .is_some_and(|at| since(i.now_ms, at) as i32 >= 0),
            ..Step::default()
        };
        let stalled = if age0 >= STALL_MS {
            Some((Core::Core0, age0))
        } else if age1 >= STALL_MS {
            Some((Core::Flight, age1))
        } else {
            None
        };
        match (stalled, &mut self.stall) {
            (None, _) => {
                step.feed = true;
                if let Some(s) = self.stall.take() {
                    step.clear = s.recorded;
                    step.ended = Some(Ended {
                        core: s.core,
                        age_ms: s.age_ms,
                        text: s.flagged,
                    });
                }
            }
            (Some(_), Some(s)) => {
                let age = match s.core {
                    Core::Core0 => age0,
                    Core::Flight => age1,
                };
                s.age_ms = s.age_ms.max(age);
                s.flagged |= s.age_ms >= TEXT_MS;
            }
            (Some(_), None) if skip => {}
            (Some((core, age)), None) => {
                let recorded = i.reboot_at_ms.is_none();
                self.stall = Some(Stall {
                    core,
                    age_ms: age,
                    recorded,
                    flagged: age >= TEXT_MS,
                });
                step.began = Some((core, age));
                step.record = recorded;
            }
        }
        step
    }
}

/// The HAL's reset reason, when the watchdog caused the reset.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResetKind {
    /// The watchdog was not fed in time.
    TimedOut,
    /// The reset was forced (`trigger_reset`).
    Forced,
}

/// How loud a boot report is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    /// MAV_SEVERITY_CRITICAL.
    Critical,
    /// MAV_SEVERITY_WARNING.
    Warning,
    /// MAV_SEVERITY_INFO.
    Info,
}

/// The boot report's case; its text is its `Display`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Report {
    /// Time-out with a stall record: "WDG reset: <TASK> stalled (core 0)" or "(flight core)".
    Stall {
        /// The core that stalled.
        core: Core,
        /// The task that ran last on it.
        task_id: u8,
    },
    /// Time-out without a stall record (interrupts off, a panic, a critical-section hang).
    NoRecord,
    /// Forced with the REQUESTED record.
    Requested,
    /// Forced without the REQUESTED record.
    ForcedNoRecord,
}

impl Report {
    /// The text's severity.
    pub const fn severity(self) -> Severity {
        match self {
            Self::Stall { .. } | Self::NoRecord => Severity::Critical,
            Self::Requested => Severity::Info,
            Self::ForcedNoRecord => Severity::Warning,
        }
    }
}

impl fmt::Display for Report {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Stall { core, task_id } => write!(
                f,
                "WDG reset: {} stalled ({})",
                task_name(*task_id),
                core.label()
            ),
            Self::NoRecord => f.write_str("WDG reset: no record"),
            Self::Requested => f.write_str("reboot: requested"),
            Self::ForcedNoRecord => f.write_str("reset: forced, no record"),
        }
    }
}

/// The boot report from the HAL's reset reason and the decoded scratch record. `None` (nothing to
/// report) for a reset the watchdog did not cause (power-on, a debugger reset). A record whose
/// reason does not fit the kind of reset counts as no record.
pub const fn boot_report(reset: Option<ResetKind>, record: Option<Record>) -> Option<Report> {
    let reason = match record {
        Some(r) => Some((r.reason, r.task_id)),
        None => None,
    };
    match (reset, reason) {
        (None, _) => None,
        (Some(ResetKind::TimedOut), Some((Reason::StallCore0, task_id))) => Some(Report::Stall {
            core: Core::Core0,
            task_id,
        }),
        (Some(ResetKind::TimedOut), Some((Reason::StallFlight, task_id))) => Some(Report::Stall {
            core: Core::Flight,
            task_id,
        }),
        (Some(ResetKind::TimedOut), _) => Some(Report::NoRecord),
        (Some(ResetKind::Forced), Some((Reason::Requested, _))) => Some(Report::Requested),
        (Some(ResetKind::Forced), _) => Some(Report::ForcedNoRecord),
    }
}

/// The text of a stall that ended before the reset: "core 0 stalled 0.6 s (BARO)".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StallText {
    /// The stall.
    pub ended: Ended,
    /// The task that ran last on the stalled core when the stall began.
    pub task_id: u8,
}

impl fmt::Display for StallText {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let ms = self.ended.age_ms;
        write!(
            f,
            "{} stalled {}.{} s ({})",
            self.ended.core.label(),
            ms / 1000,
            ms % 1000 / 100,
            task_name(self.task_id)
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scratch::{decode, encode};
    use std::string::ToString;
    use std::vec::Vec;

    struct Sim {
        m: Monitor,
        now: u32,
        beat0: u32,
        pong: u32,
        epoch: u32,
        reboot_at: Option<u32>,
    }

    impl Sim {
        fn new(now: u32) -> Self {
            let i = Inputs {
                now_ms: now,
                beat0: 0,
                pong: 0,
                save_epoch: 0,
                reboot_at_ms: None,
            };
            Self {
                m: Monitor::new(i),
                now,
                beat0: 0,
                pong: 0,
                epoch: 0,
                reboot_at: None,
            }
        }

        /// Advances 100 ms; each heartbeat moves when its core runs.
        fn check(&mut self, core0_runs: bool, flight_runs: bool) -> Step {
            self.now = self.now.wrapping_add(CHECK_MS);
            if core0_runs {
                self.beat0 += 2;
                if flight_runs {
                    self.pong += 1;
                }
            }
            self.m.step(Inputs {
                now_ms: self.now,
                beat0: self.beat0,
                pong: self.pong,
                save_epoch: self.epoch,
                reboot_at_ms: self.reboot_at,
            })
        }
    }

    #[test]
    fn healthy_feeds_every_check_and_never_records() {
        for start in [0, u32::MAX - 5_000] {
            let mut s = Sim::new(start);
            for _ in 0..10_000 {
                let st = s.check(true, true);
                assert_eq!(
                    st,
                    Step {
                        feed: true,
                        ..Step::default()
                    }
                );
            }
        }
    }

    #[test]
    fn core0_stall_records_once_at_200_ms_and_stops_feeding() {
        let mut s = Sim::new(1_000);
        s.check(true, true);
        assert!(s.check(false, true).feed, "100 ms: still fed");
        let st = s.check(false, true);
        assert!(!st.feed);
        assert_eq!(st.began, Some((Core::Core0, 200)));
        assert!(st.record);
        for _ in 0..30 {
            let st = s.check(false, true);
            assert!(!st.feed && st.began.is_none() && !st.record && st.ended.is_none());
        }
    }

    #[test]
    fn flight_stall_names_the_flight_core() {
        let mut s = Sim::new(0);
        s.check(true, true);
        s.check(true, false);
        let st = s.check(true, false);
        assert_eq!(st.began, Some((Core::Flight, 200)));
        assert!(st.record && !st.feed);
    }

    #[test]
    fn last_feed_comes_100_ms_before_the_record() {
        // The reset comes PERIOD_MS after the last feed: 1700 ms after the last beat seen.
        let mut s = Sim::new(0);
        let mut last_feed = 0;
        let mut last_beat = 0;
        for k in 0..40 {
            let runs = k < 10;
            let st = s.check(runs, true);
            if runs {
                last_beat = s.now;
            }
            if st.feed {
                last_feed = s.now;
            }
        }
        assert_eq!(last_feed + PERIOD_MS - last_beat, 1_700);
    }

    #[test]
    fn short_stall_clears_the_record_and_sends_no_text() {
        let mut s = Sim::new(0);
        s.check(true, true);
        s.check(false, true);
        assert!(s.check(false, true).record);
        let st = s.check(false, true);
        assert!(!st.feed);
        let st = s.check(true, true);
        assert!(st.feed && st.clear);
        assert_eq!(
            st.ended,
            Some(Ended {
                core: Core::Core0,
                age_ms: 300,
                text: false
            })
        );
    }

    #[test]
    fn stall_of_one_second_ends_with_one_text_and_no_reset() {
        let mut s = Sim::new(0);
        s.check(true, true);
        let mut steps = Vec::new();
        for _ in 0..10 {
            steps.push(s.check(false, true));
        }
        // Core 0 resumes; PONG follows at the next ping.
        steps.push(s.check(true, false));
        steps.push(s.check(true, true));
        steps.push(s.check(true, true));
        let records = steps.iter().filter(|st| st.record).count();
        let ended: Vec<_> = steps.iter().filter_map(|st| st.ended).collect();
        assert_eq!(records, 1);
        assert_eq!(
            ended,
            [Ended {
                core: Core::Core0,
                age_ms: 1_000,
                text: true
            }]
        );
        assert!(steps.iter().all(|st| !st.reset));
        // The longest run without a feed is shorter than the watchdog's period.
        let mut gap = 0;
        let mut worst = 0;
        for st in &steps {
            gap = if st.feed { 0 } else { gap + CHECK_MS };
            worst = worst.max(gap);
        }
        assert!(worst + CHECK_MS < PERIOD_MS, "{worst}");
    }

    #[test]
    fn text_flag_at_500_ms() {
        for (checks, text) in [(3, false), (4, false), (5, true), (6, true)] {
            let mut s = Sim::new(0);
            s.check(true, true);
            for _ in 0..checks {
                s.check(false, true);
            }
            let st = s.check(true, true);
            assert_eq!(st.ended.map(|e| e.text), Some(text), "{checks}");
        }
    }

    #[test]
    fn check_after_a_save_never_starts_a_stall() {
        let mut s = Sim::new(0);
        s.check(true, true);
        s.check(false, true);
        s.epoch += 1;
        let st = s.check(false, true);
        assert!(!st.feed && st.began.is_none() && !st.record && st.save_check);
        // Only one check is skipped.
        let st = s.check(false, true);
        assert_eq!(st.began, Some((Core::Core0, 300)));
        assert!(!st.save_check);
        // A healthy check after a save still feeds.
        let mut s = Sim::new(0);
        s.epoch += 1;
        let st = s.check(true, true);
        assert!(st.feed && st.save_check && st.began.is_none());
    }

    #[test]
    fn requested_reset_is_due_at_its_time_and_blocks_a_stall_record() {
        let mut s = Sim::new(u32::MAX - 150);
        s.reboot_at = Some(s.now.wrapping_add(250));
        assert!(!s.check(true, true).reset);
        assert!(!s.check(true, true).reset);
        assert!(s.check(true, true).reset);
        let mut s = Sim::new(0);
        s.reboot_at = Some(10_000);
        s.check(false, true);
        let st = s.check(false, true);
        assert_eq!(st.began, Some((Core::Core0, 200)));
        assert!(!st.record && !st.reset);
        let st = s.check(true, true);
        assert!(st.feed && !st.clear && st.ended.is_some());
    }

    #[test]
    fn boot_report_table() {
        let stall0 = decode(encode(Reason::StallCore0, task::BARO, 200, 21_000));
        let stall1 = decode(encode(Reason::StallFlight, task::IMU1, 200, 21_000));
        let req = decode(encode(Reason::Requested, 0, 0, 30_000));
        let t = Some(ResetKind::TimedOut);
        let f = Some(ResetKind::Forced);
        let cases = [
            (
                t,
                stall0,
                Some((Severity::Critical, "WDG reset: BARO stalled (core 0)")),
            ),
            (
                t,
                stall1,
                Some((Severity::Critical, "WDG reset: IMU1 stalled (flight core)")),
            ),
            (t, None, Some((Severity::Critical, "WDG reset: no record"))),
            (t, req, Some((Severity::Critical, "WDG reset: no record"))),
            (f, req, Some((Severity::Info, "reboot: requested"))),
            (
                f,
                None,
                Some((Severity::Warning, "reset: forced, no record")),
            ),
            (
                f,
                stall0,
                Some((Severity::Warning, "reset: forced, no record")),
            ),
            (None, None, None),
            (None, stall0, None),
            (None, req, None),
        ];
        for (reset, record, want) in cases {
            let got = boot_report(reset, record).map(|r| (r.severity(), r.to_string()));
            assert_eq!(
                got.as_ref().map(|(s, t)| (*s, t.as_str())),
                want,
                "{reset:?} {record:?}"
            );
            if let Some((_, text)) = got {
                assert!(text.len() <= 50);
            }
        }
    }

    #[test]
    fn stall_text_and_task_names() {
        let t = StallText {
            ended: Ended {
                core: Core::Core0,
                age_ms: 1_000,
                text: true,
            },
            task_id: task::BARO,
        };
        assert_eq!(t.to_string(), "core 0 stalled 1.0 s (BARO)");
        let t = StallText {
            ended: Ended {
                core: Core::Flight,
                age_ms: 650,
                text: true,
            },
            task_id: task::IMU1,
        };
        assert_eq!(t.to_string(), "flight core stalled 0.6 s (IMU1)");
        let mut names = Vec::new();
        for id in 0..=u8::MAX {
            let n = task_name(id);
            if n != "?" {
                assert!(!names.contains(&n), "{n}");
                assert!(n.len() <= 10);
                names.push(n);
            }
        }
        assert_eq!(names.len(), 12);
    }
}
