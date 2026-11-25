//! ESC telemetry logic for the flight core's `esc` task: pure functions and plain data. No HAL, no
//! clock of its own (every function is given `now_us`), no logging, no lock.
//!
//! The task, per exchange: [`Esc::frames`] chooses the four frames, the driver exchanges them,
//! [`Esc::record`] folds the four captures in and the task publishes [`Esc::status`]; the wait for
//! the next exchange uses [`next_deadline`]. The status type [`EscStatus`] is `fw_core::esc`'s (core
//! 0's builders live there); it is re-exported here.
//!
//! A frame is [`Value::Stop`]; or, for a command-13 burst on one line,
//! [`Command::ExtendedTelemetryOn`]; or, on the one line of a motor-test spin, a throttle built by
//! `capped` (the motor test's only `Throttle` constructor); or, on all four lines, the throttles of
//! the flight command while [`hold`] keeps it spinning (built only by `crate::gate`).

use crate::gate::{FLIGHT_GAP_DISARM_US, MotorCommand};
use dshot::{Command, LineCapture, LineFrame, Reply, Throttle, Value};
pub use fw_core::esc::{
    ANSWER_US, ERPM_MAX, EscLine, EscStatus, FRESH_US, LINES, TEST_BURST_MAX_US, TEST_CAP,
    TEST_PUSH_MARGIN_US, TestCause, TestCmd, TestEnding, TestSpin, line_of_element,
};

/// The exchange period: 1 kHz, in us.
pub const PERIOD_US: u64 = 1_000;

/// Consecutive command-13 frames of one burst (Betaflight's repeat count; the ESC needs 6).
pub const BURST_FRAMES: u8 = 10;

/// A line must have answered continuously this long, and given no extended reply this long, before
/// a burst starts; after a burst the line is held off this long, in us.
pub const BURST_GATE_US: u64 = 1_000_000;

/// Longest age of a flight command still sent, us: eight IMU periods (about 2 ms of normal age; the ESC's own
/// timeout is 510 ms).
pub const MOTOR_HOLD_MAX_US: u64 = 10_000;

/// The command to send at `now_us`: `cmd` while `now_us - t_us <= MOTOR_HOLD_MAX_US`, else STOP. The
/// initial `flight::OUT` (t_us 0, STOP) gives STOP, so "no command yet" needs no case of its own.
pub fn hold(t_us: u64, cmd: MotorCommand, now_us: u64) -> MotorCommand {
    if now_us.saturating_sub(t_us) > MOTOR_HOLD_MAX_US {
        MotorCommand::STOP
    } else {
        cmd
    }
}

/// Whether `cmd`, stamped `t_us`, spins and is older than `FLIGHT_GAP_DISARM_US` at `now_us`: the ESC task then
/// clears `ARMED`.
pub fn stale_spin(t_us: u64, cmd: MotorCommand, now_us: u64) -> bool {
    cmd.throttles().is_some() && now_us.saturating_sub(t_us) > FLIGHT_GAP_DISARM_US
}

/// The fields of one ESC log record (`fw_core::dflog::encode_esc`), by DShot line.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogFields {
    /// Raw eRPM of the latest capture; 0 if the line has not reported one.
    pub erpm: [u32; LINES],
    /// Volts in centivolts (mV / 10, saturating at `i16::MAX`); 0 if not reported.
    pub volts_cv: [i16; LINES],
    /// Degrees Celsius; 0 if not reported.
    pub temp_c: [u8; LINES],
    /// Bit i (i < 4): line i captured an eRPM after `since_us`; bit 4: this exchange sent the flight throttles.
    pub flags: u8,
}

/// The ESC record's fields from `status`; `since_us` is the time of the previous ESC record. Pure.
pub fn log_fields(status: &EscStatus, since_us: u64) -> LogFields {
    let mut f = LogFields {
        erpm: [0; LINES],
        volts_cv: [0; LINES],
        temp_c: [0; LINES],
        flags: if status.flight_spin { 1 << 4 } else { 0 },
    };
    for (i, line) in status.lines.iter().enumerate() {
        if let Some((erpm, t)) = line.erpm {
            f.erpm[i] = erpm;
            if t > since_us {
                f.flags |= 1 << i;
            }
        }
        if let Some((mv, _)) = line.voltage_mv {
            f.volts_cv[i] = i16::try_from(mv / 10).unwrap_or(i16::MAX);
        }
        if let Some((c, _)) = line.temperature_c {
            f.temp_c[i] = c;
        }
    }
    f
}

/// The deadline of the exchange after the one due at `deadline_us`, given `now_us` (the time the task looks at the
/// clock): `deadline + 1000 us`.
pub fn next_deadline(deadline_us: u64, now_us: u64) -> u64 {
    let next = deadline_us.saturating_add(PERIOD_US);
    if now_us.saturating_sub(next) > PERIOD_US {
        now_us
    } else {
        next
    }
}

#[derive(Clone, Copy)]
struct Burst {
    /// Command frames still to send.
    left: u8,
    /// No new burst before this time, us.
    holdoff_until_us: u64,
}

#[derive(Clone, Copy)]
struct Test {
    /// The `seq` of the latest command seen.
    seq: u32,
    /// That command gets no throttle frame again (it ended, or it is a stop).
    done: bool,
}

/// The ESC task's state: the counters and latest values, the burst state of each line and the
/// motor test's.
pub struct Esc {
    status: EscStatus,
    bursts: [Burst; LINES],
    test: Test,
    /// EDT bursts may start (see [`Esc::set_edt_bursts`]).
    edt_bursts: bool,
}

impl Default for Esc {
    fn default() -> Self {
        Self::new()
    }
}

impl Esc {
    /// Nothing seen, no burst running.
    pub const fn new() -> Self {
        Self {
            status: EscStatus::INITIAL,
            bursts: [Burst {
                left: 0,
                holdoff_until_us: 0,
            }; LINES],
            test: Test { seq: 0, done: true },
            edt_bursts: true,
        }
    }

    /// `false`: no EDT burst starts from now on (the binary's `esc-no-edt`); nothing else changes.
    pub fn set_edt_bursts(&mut self, enabled: bool) {
        self.edt_bursts = enabled;
    }

    /// The board's task could not read the flight command for this exchange (counts it, saturating).
    pub fn flight_read_failed(&mut self) {
        self.status.flight_read_failures = self.status.flight_read_failures.saturating_add(1);
    }

    /// The status to publish.
    pub fn status(&self) -> &EscStatus {
        &self.status
    }

    /// The start sequence begins (counts it).
    pub fn start_begin(&mut self) {
        self.status.starts = self.status.starts.saturating_add(1);
        self.status.start_running = true;
    }

    /// The start sequence ended at `now_us`.
    pub fn start_end(&mut self, now_us: u64) {
        self.status.start_running = false;
        self.status.start_ended_us = Some(now_us);
    }

    /// The four frames of the exchange at `now_us`, by driver line, given the latest motor-test
    /// command `test` and the latest flight command `flight` (its step time, us, and the command).
    pub fn frames(
        &mut self,
        now_us: u64,
        test: TestCmd,
        flight: (u64, MotorCommand),
    ) -> [LineFrame; LINES] {
        let (flight_us, flight_cmd) = flight;
        self.status.flight_spin = false;
        if let Some(throttles) = hold(flight_us, flight_cmd, now_us).throttles() {
            if self.status.test_spin.is_some() {
                self.end(test, TestCause::Replaced, now_us);
            }
            for burst in &mut self.bursts {
                if burst.left > 0 {
                    burst.left = 0;
                    burst.holdoff_until_us = now_us.saturating_add(BURST_GATE_US);
                }
            }
            self.status.flight_spin = true;
            self.status.flight_frames = self.status.flight_frames.saturating_add(1);
            return throttles.map(|t| LineFrame::new(Value::Throttle(t), false));
        }
        let mut out = [LineFrame::STOP; LINES];
        if let Some((line, throttle)) = self.spin(now_us, test) {
            for burst in &mut self.bursts {
                if burst.left > 0 {
                    burst.left = 0;
                    burst.holdoff_until_us = now_us.saturating_add(BURST_GATE_US);
                }
            }
            if let Some(frame) = out.get_mut(line) {
                *frame = LineFrame::new(Value::Throttle(throttle), false);
            }
            return out;
        }
        let edt_bursts = self.edt_bursts;
        for ((frame, burst), line) in out
            .iter_mut()
            .zip(self.bursts.iter_mut())
            .zip(self.status.lines.iter_mut())
        {
            if burst.left == 0 && edt_bursts && burst_due(line, burst, now_us) {
                burst.left = BURST_FRAMES;
                line.edt_bursts = line.edt_bursts.saturating_add(1);
            }
            if burst.left > 0 {
                burst.left -= 1;
                if burst.left == 0 {
                    burst.holdoff_until_us = now_us.saturating_add(BURST_GATE_US);
                }
                *frame = LineFrame::new(Value::Command(Command::ExtendedTelemetryOn), false);
            }
        }
        out
    }

    /// The throttle and its driver line for the exchange at `now_us`, if `test` keeps a spin active, and the spin status
    /// and ending.
    fn spin(&mut self, now_us: u64, test: TestCmd) -> Option<(usize, Throttle)> {
        if test.seq != self.test.seq {
            if self.status.test_spin.is_some() {
                let cause = if test.level == 0 {
                    TestCause::Stop
                } else {
                    TestCause::Replaced
                };
                self.end(test, cause, now_us);
            }
            self.test = Test {
                seq: test.seq,
                done: false,
            };
        }
        if self.test.done {
            return None;
        }
        if test.level == 0 {
            if self.status.test_spin.is_some() {
                self.end(test, TestCause::Stop, now_us);
            }
            self.test.done = true;
            return None;
        }
        let line = usize::from(test.line);
        let Some(throttle) = capped(test.level).filter(|_| line < LINES) else {
            self.end(test, TestCause::Cap, now_us);
            return None;
        };
        if now_us >= test.until_us {
            self.end(test, TestCause::Expiry, now_us);
            return None;
        }
        let first_us = self.status.test_spin.map_or(now_us, |s| s.first_us);
        if !within_limit(now_us.saturating_sub(first_us), TEST_BURST_MAX_US) {
            self.end(test, TestCause::Limit, now_us);
            return None;
        }
        self.status.throttle_frames = self.status.throttle_frames.wrapping_add(1);
        self.status.test_spin = Some(TestSpin {
            seq: test.seq,
            line: test.line,
            level: test.level,
            first_us,
            last_us: now_us,
        });
        Some((line, throttle))
    }

    /// Ends the running spin (or, when none runs, refuses `test`) for `cause` at `now_us`.
    fn end(&mut self, test: TestCmd, cause: TestCause, now_us: u64) {
        self.test.done = true;
        let (seq, line, level, first_us) = match self.status.test_spin.take() {
            Some(s) => (s.seq, s.line, s.level, Some(s.first_us)),
            None => (test.seq, test.line, test.level, None),
        };
        self.status.test_ended = Some(TestEnding {
            seq,
            line,
            level,
            cause,
            first_us,
            at_us: now_us,
        });
    }

    /// Folds one exchange (finished at `now_us`, `duration_us` long) into the counters and latest
    /// values.
    pub fn record(&mut self, now_us: u64, duration_us: u32, captures: &[LineCapture; LINES]) {
        let s = &mut self.status;
        if s.exchanges > 0 {
            let spacing = now_us.saturating_sub(s.last_exchange_us);
            s.max_spacing_us = s
                .max_spacing_us
                .max(u32::try_from(spacing).unwrap_or(u32::MAX));
        }
        s.max_exchange_us = s.max_exchange_us.max(duration_us);
        s.min_exchange_us = s.min_exchange_us.min(duration_us);
        s.exchanges = s.exchanges.wrapping_add(1);
        s.last_exchange_us = now_us;
        for (line, capture) in s.lines.iter_mut().zip(captures) {
            if line.answering_since_us.is_some() && !line.answering(now_us) {
                line.dropouts = line.dropouts.saturating_add(1);
                line.answering_since_us = None;
            }
            match capture {
                LineCapture::Reply(reply) => {
                    line.good = line.good.wrapping_add(1);
                    line.last_good_us = Some(now_us);
                    line.answering_since_us.get_or_insert(now_us);
                    fold_reply(line, reply, now_us);
                }
                LineCapture::Silent | LineCapture::Invalid(_) | LineCapture::Foreign => {
                    line.non_replies = line.non_replies.wrapping_add(1);
                }
            }
        }
    }
}

/// The motor test's only throttle constructor (the flight command's is the gate's). A level above [`TEST_CAP`] (or outside
/// 1..=2000) is refused.
fn capped(level: u16) -> Option<Throttle> {
    if !within_cap(level) {
        return None;
    }
    Throttle::new(level)
}

/// Whether a level is at or below the cap.
const fn within_cap(level: u16) -> bool {
    level <= TEST_CAP
}

/// Whether a frame pushed `elapsed_us` after the start of its spin stays within
/// `limit_us`, with the margin for the push.
const fn within_limit(elapsed_us: u64, limit_us: u64) -> bool {
    elapsed_us.saturating_add(TEST_PUSH_MARGIN_US) <= limit_us
}

fn burst_due(line: &EscLine, burst: &Burst, now_us: u64) -> bool {
    now_us >= burst.holdoff_until_us
        && line.answering(now_us)
        && line
            .answering_since_us
            .is_some_and(|t| now_us.saturating_sub(t) >= BURST_GATE_US)
        && line
            .last_extended_us
            .is_none_or(|t| now_us.saturating_sub(t) >= BURST_GATE_US)
}

fn fold_reply(line: &mut EscLine, reply: &Reply, now_us: u64) {
    use dshot::ExtendedTelemetry as X;
    match reply {
        Reply::Erpm(e) => {
            let erpm = e.erpm();
            if erpm <= ERPM_MAX {
                line.erpm = Some((erpm, now_us));
            }
        }
        Reply::Extended(x) => {
            line.last_extended_us = Some(now_us);
            match x {
                X::Temperature { celsius } => line.temperature_c = Some((*celsius, now_us)),
                X::Voltage { millivolts } => line.voltage_mv = Some((*millivolts, now_us)),
                X::Current { milliamps } => line.current_ma = Some((*milliamps, now_us)),
                _ => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use dshot::{CaptureError, ExtendedTelemetry, decode_reply};
    use std::vec::Vec;

    fn stopped() -> LineCapture {
        LineCapture::Reply(decode_reply(0x52951).unwrap())
    }

    fn voltage() -> LineCapture {
        LineCapture::Reply(Reply::Extended(ExtendedTelemetry::Voltage {
            millivolts: 16_000,
        }))
    }

    const EDT: Value = Value::Command(Command::ExtendedTelemetryOn);

    /// The initial flight command (`flight::OUT` before the first step).
    const NO_FLIGHT: (u64, MotorCommand) = (0, MotorCommand::STOP);

    #[test]
    fn log_fields_fresh_bits_and_units() {
        let mut st = EscStatus::INITIAL;
        let f = log_fields(&st, 0);
        assert_eq!(
            (f.erpm, f.volts_cv, f.temp_c, f.flags),
            ([0; 4], [0; 4], [0; 4], 0)
        );
        st.lines[0].erpm = Some((12_345, 5_000)); // after since: fresh
        st.lines[1].erpm = Some((777, 4_000)); // at since: not fresh
        st.lines[2].erpm = Some((0, 3_000)); // a stopped motor reported 0, older
        st.lines[3].erpm = None; // never reported
        st.lines[0].voltage_mv = Some((16_840, 1));
        st.lines[1].voltage_mv = Some((9, 1)); // 0.009 V rounds down to 0
        st.lines[2].voltage_mv = Some((4_000_000, 1)); // saturates
        st.lines[0].temperature_c = Some((42, 1));
        st.lines[3].temperature_c = Some((255, 1));
        let f = log_fields(&st, 4_000);
        assert_eq!(f.erpm, [12_345, 777, 0, 0]);
        assert_eq!(f.volts_cv, [1_684, 0, i16::MAX, 0]);
        assert_eq!(f.temp_c, [42, 0, 0, 255]);
        assert_eq!(f.flags, 0b0_0001);
        for line in &mut st.lines {
            line.erpm = Some((1, 9_000));
        }
        st.flight_spin = true;
        assert_eq!(log_fields(&st, 4_000).flags, 0b1_1111);
        st.flight_spin = false;
        assert_eq!(log_fields(&st, 9_000).flags, 0);
    }

    /// One exchange at `t_ms` milliseconds; returns the frames sent.
    fn step(esc: &mut Esc, t_ms: u64, caps: [LineCapture; LINES]) -> [LineFrame; LINES] {
        let now = t_ms * 1000;
        let frames = esc.frames(now, TestCmd::INITIAL, NO_FLIGHT);
        esc.record(now, 145, &caps);
        frames
    }

    fn only_stop_or_13(frames: &[LineFrame; LINES]) {
        for f in frames {
            assert!(!f.telemetry_request);
            match f.value {
                Value::Stop | Value::Command(Command::ExtendedTelemetryOn) => {}
                other => panic!("unexpected frame value {other:?}"),
            }
        }
    }

    type Caps<'a> = &'a dyn Fn(u64) -> [LineCapture; LINES];

    /// Runs exchanges `from..to` ms, recording the exchange times of every command-13 frame per line.
    fn run(esc: &mut Esc, bursts: &mut [Vec<u64>; LINES], from: u64, to: u64, caps: Caps) {
        for t in from..to {
            let frames = step(esc, t, caps(t));
            only_stop_or_13(&frames);
            for (l, f) in frames.iter().enumerate() {
                if f.value == EDT {
                    bursts[l].push(t);
                }
            }
        }
    }

    /// `next += 1000`; more than one period behind: `next = now`.
    #[test]
    fn deadline_advances_by_one_period_and_a_stall_gives_one_late_frame() {
        let mut next = 5_000;
        for k in 1..=100u64 {
            next = next_deadline(next, next + 145);
            assert_eq!(next, 5_000 + k * 1000);
        }
        assert_eq!(next_deadline(10_000, 12_000), 11_000);
        assert_eq!(next_deadline(10_000, 12_001), 12_001);
        let stalled_now = 10_000 + 50_000;
        let next = next_deadline(10_000, stalled_now);
        assert_eq!(next, stalled_now);
        assert_eq!(next_deadline(next, next + 145), stalled_now + 1000);
    }

    /// Nothing stops on errors; only Stop comes out.
    #[test]
    fn ten_thousand_non_replies_keep_the_frames_coming_and_stop_stays_stop() {
        let mut esc = Esc::new();
        esc.start_begin();
        esc.start_end(0);
        let junk = [
            LineCapture::Silent,
            LineCapture::Foreign,
            LineCapture::Invalid(CaptureError::CutOff),
            LineCapture::Invalid(CaptureError::Glitch),
        ];
        for k in 0..10_000u64 {
            let caps = [0, 1, 2, 3].map(|i| junk[((k + i) % 4) as usize]);
            let frames = step(&mut esc, k, caps);
            assert_eq!(frames, [LineFrame::STOP; LINES], "exchange {k}");
        }
        let s = esc.status();
        assert_eq!(s.exchanges, 10_000);
        for l in &s.lines {
            assert_eq!(
                (l.good, l.non_replies, l.dropouts, l.edt_bursts),
                (0, 10_000, 0, 0)
            );
        }
    }

    /// The burst gate, the hold-off, the restart after a dropout, no burst while extended replies arrive.
    #[test]
    fn the_edt_burst_follows_the_gate() {
        let mut esc = Esc::new();
        esc.start_begin();
        esc.start_end(0);
        let silent = LineCapture::Silent;
        let mut bursts: [Vec<u64>; LINES] = Default::default();
        let both = |_t: u64| [stopped(), stopped(), silent, silent];
        run(&mut esc, &mut bursts, 0, 1000, &both);
        assert!(bursts.iter().all(|b| b.is_empty()));
        run(&mut esc, &mut bursts, 1000, 1100, &both);
        let expect: Vec<u64> = (1000..1010).collect();
        assert_eq!(bursts[0], expect);
        assert_eq!(bursts[1], expect);
        assert!(bursts[2].is_empty() && bursts[3].is_empty());
        assert_eq!(esc.status().lines[0].edt_bursts, 1);
        assert_eq!(esc.status().lines[2].edt_bursts, 0);
        run(&mut esc, &mut bursts, 1100, 2009, &both);
        assert_eq!(bursts[0].len(), 10);
        run(&mut esc, &mut bursts, 2009, 2100, &both);
        assert_eq!(bursts[0].len(), 20);
        assert_eq!(bursts[0].get(10), Some(&2009));
        let mixed = |t: u64| {
            let l0 = if t % 100 == 0 { voltage() } else { stopped() };
            let l1 = if (2100..2250).contains(&t) {
                silent
            } else {
                stopped()
            };
            [l0, l1, silent, silent]
        };
        let before = (bursts[0].len(), bursts[1].len());
        run(&mut esc, &mut bursts, 2100, 3249, &mixed);
        assert_eq!(esc.status().lines[1].dropouts, 1);
        assert_eq!(
            bursts[1].len(),
            before.1,
            "no burst before 1 s of answering again"
        );
        assert_eq!(
            bursts[0].len(),
            before.0,
            "none while extended frames arrive"
        );
        run(&mut esc, &mut bursts, 3249, 3500, &mixed);
        assert_eq!(bursts[1].len(), before.1 + 10);
        assert_eq!(bursts[1].get(before.1), Some(&3250));
        assert_eq!(bursts[0].len(), before.0);
    }

    #[test]
    fn a_reply_folds_into_counters_and_latest_values() {
        let mut esc = Esc::new();
        let caps = [
            stopped(),
            voltage(),
            LineCapture::Reply(Reply::Extended(ExtendedTelemetry::Temperature {
                celsius: 38,
            })),
            LineCapture::Foreign,
        ];
        step(&mut esc, 5, caps);
        let s = esc.status();
        assert_eq!(s.lines[0].erpm, Some((0, 5000)));
        assert_eq!(s.lines[1].voltage_mv, Some((16_000, 5000)));
        assert_eq!(s.lines[1].last_extended_us, Some(5000));
        assert_eq!(s.lines[2].temperature_c, Some((38, 5000)));
        assert_eq!((s.lines[3].good, s.lines[3].non_replies), (0, 1));
        assert_eq!(
            (s.exchanges, s.last_exchange_us, s.max_exchange_us),
            (1, 5000, 145)
        );
        let none = [LineCapture::Foreign; LINES];
        for (t_ms, us) in [(6, 900), (7, 146)] {
            let now = t_ms * 1000;
            esc.frames(now, TestCmd::INITIAL, NO_FLIGHT);
            esc.record(now, us, &none);
        }
        let s = esc.status();
        assert_eq!((s.min_exchange_us, s.max_exchange_us), (145, 900));
    }

    fn spin_cmd(seq: u32, line: u8, level: u16, until_us: u64) -> TestCmd {
        TestCmd {
            seq,
            line,
            level,
            until_us,
        }
    }

    /// The line and level of the throttle frame, checking the motor-test frame rules: at most one throttle frame, never
    /// above the cap, and only Stop beside it.
    fn throttle_of(frames: &[LineFrame; LINES]) -> Option<(usize, u16)> {
        let mut found = None;
        for (l, f) in frames.iter().enumerate() {
            assert!(!f.telemetry_request);
            if let Value::Throttle(t) = f.value {
                assert!(found.is_none(), "two throttle frames");
                assert!(t.level() <= TEST_CAP);
                found = Some((l, t.level()));
            }
        }
        if found.is_some() {
            for f in frames {
                assert!(matches!(f.value, Value::Throttle(_) | Value::Stop));
            }
        }
        found
    }

    /// One exchange at `now_us` with the command `test`, every line answering stopped.
    fn exchange(esc: &mut Esc, now_us: u64, test: TestCmd) -> Option<(usize, u16)> {
        let frames = esc.frames(now_us, test, NO_FLIGHT);
        esc.record(now_us, 145, &[stopped(); LINES]);
        throttle_of(&frames)
    }

    fn ending(
        seq: u32,
        line: u8,
        level: u16,
        cause: TestCause,
        first: Option<u64>,
        at: u64,
    ) -> Option<TestEnding> {
        Some(TestEnding {
            seq,
            line,
            level,
            cause,
            first_us: first,
            at_us: at,
        })
    }

    /// A level above the cap gives Stop and a Cap ending; `capped` never builds above the cap.
    #[test]
    fn a_level_above_the_cap_gives_stop_and_a_cap_ending() {
        for level in 0..=u16::MAX {
            match capped(level) {
                Some(t) => assert!((1..=TEST_CAP).contains(&level) && t.level() == level),
                None => assert!(level == 0 || level > TEST_CAP),
            }
        }
        let mut esc = Esc::new();
        let over = spin_cmd(1, 2, TEST_CAP + 1, 10_000_000);
        for t in 0..5 {
            assert_eq!(
                esc.frames(t * 1000, over, NO_FLIGHT),
                [LineFrame::STOP; LINES]
            );
        }
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(1, 2, TEST_CAP + 1, TestCause::Cap, None, 0)
        );
        assert_eq!((s.test_spin, s.throttle_frames), (None, 0));
        let at_cap = spin_cmd(2, 2, TEST_CAP, 1_000_000);
        assert_eq!(exchange(&mut esc, 10_000, at_cap), Some((2, TEST_CAP)));
        let over = spin_cmd(3, 2, TEST_CAP + 1, 1_000_000);
        assert_eq!(
            esc.frames(11_000, over, NO_FLIGHT),
            [LineFrame::STOP; LINES]
        );
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(3, 2, TEST_CAP + 1, TestCause::Cap, None, 11_000)
        );
        assert_eq!(
            esc.frames(12_000, spin_cmd(4, 4, 50, 1_000_000), NO_FLIGHT),
            [LineFrame::STOP; LINES]
        );
        let s = esc.status();
        assert_eq!(s.test_ended, ending(4, 4, 50, TestCause::Cap, None, 12_000));
        assert_eq!((s.test_spin, s.throttle_frames), (None, 1));
    }

    /// Runs exchanges `from..to` ms with `test`, recording the times of command-13 frames per line and of throttle frames
    /// with their line.
    fn run_cmd(
        esc: &mut Esc,
        from: u64,
        to: u64,
        test: TestCmd,
        caps: [LineCapture; LINES],
        edt: &mut [Vec<u64>; LINES],
        thr: &mut Vec<(u64, usize)>,
    ) {
        for t in from..to {
            let frames = esc.frames(t * 1000, test, NO_FLIGHT);
            esc.record(t * 1000, 145, &caps);
            if let Some((l, _)) = throttle_of(&frames) {
                thr.push((t, l));
            }
            for (l, f) in frames.iter().enumerate() {
                if f.value == EDT {
                    edt[l].push(t);
                }
            }
        }
    }

    /// One throttle frame, Stop on the other three, no command 13 during a spin; a running EDT burst is cut, held off 1 s
    /// from the cut, and the burst gate resumes after the spin.
    #[test]
    fn a_spin_sends_one_throttle_frame_and_cuts_the_edt_burst() {
        let mut esc = Esc::new();
        esc.start_begin();
        esc.start_end(0);
        let silent = LineCapture::Silent;
        let caps = [stopped(), stopped(), silent, silent];
        let mut edt: [Vec<u64>; LINES] = Default::default();
        let mut thr = Vec::new();
        run_cmd(
            &mut esc,
            0,
            1003,
            TestCmd::INITIAL,
            caps,
            &mut edt,
            &mut thr,
        );
        assert_eq!(edt[0], [1000, 1001, 1002]);
        let spin = spin_cmd(1, 1, TEST_CAP, 10_000_000);
        run_cmd(&mut esc, 1003, 1500, spin, caps, &mut edt, &mut thr);
        assert_eq!(thr, (1003..1500).map(|t| (t, 1)).collect::<Vec<_>>());
        assert_eq!(edt[0], [1000, 1001, 1002]);
        assert_eq!(edt[1], [1000, 1001, 1002]);
        let stop = TestCmd {
            seq: 2,
            ..TestCmd::INITIAL
        };
        run_cmd(&mut esc, 1500, 2100, stop, caps, &mut edt, &mut thr);
        assert_eq!(thr.len(), 497);
        let expect: Vec<u64> = [1000, 1001, 1002].into_iter().chain(2003..2013).collect();
        assert_eq!(edt[0], expect);
        assert_eq!(edt[1], expect);
        assert!(edt[2].is_empty() && edt[3].is_empty());
        let s = esc.status();
        assert_eq!(s.lines[0].edt_bursts, 2);
        assert_eq!(s.throttle_frames, 497);
        assert_eq!(
            s.test_ended,
            ending(1, 1, TEST_CAP, TestCause::Stop, Some(1_003_000), 1_500_000)
        );
    }

    /// A spin ends on the task's clock at min(until, first + 3 s) with no further command.
    #[test]
    fn a_spin_ends_at_its_until_or_three_seconds_after_its_first_frame() {
        let mut esc = Esc::new();
        let cmd = spin_cmd(7, 3, 60, 7_000_000);
        let mut n = 0u32;
        for t in (5_000_000..9_000_000).step_by(1000) {
            let thr = exchange(&mut esc, t, cmd);
            assert_eq!(thr, (t < 7_000_000).then_some((3, 60)), "t {t}");
            n += u32::from(thr.is_some());
        }
        assert_eq!((n, esc.status().throttle_frames), (2000, 2000));
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(7, 3, 60, TestCause::Expiry, Some(5_000_000), 7_000_000)
        );
        assert_eq!(s.test_spin, None);
        let mut esc = Esc::new();
        let cmd = spin_cmd(8, 0, TEST_CAP, u64::MAX);
        assert_eq!(exchange(&mut esc, 1_000_000, cmd), Some((0, TEST_CAP)));
        assert_eq!(exchange(&mut esc, 3_999_000, cmd), Some((0, TEST_CAP)));
        let active = TestSpin {
            seq: 8,
            line: 0,
            level: TEST_CAP,
            first_us: 1_000_000,
            last_us: 3_999_000,
        };
        assert_eq!(esc.status().test_spin, Some(active));
        assert_eq!(exchange(&mut esc, 3_999_001, cmd), None);
        assert_eq!(exchange(&mut esc, 4_500_000, cmd), None);
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(8, 0, TEST_CAP, TestCause::Limit, Some(1_000_000), 3_999_001)
        );
        assert_eq!((s.test_spin, s.throttle_frames), (None, 2));
    }

    /// A stop ends a spin at the next exchange; a new seq on another line replaces it and gets its own 3 s count.
    #[test]
    fn a_stop_ends_a_spin_and_a_new_seq_replaces_it() {
        let mut esc = Esc::new();
        let a = spin_cmd(1, 3, 100, 10_000_000);
        assert_eq!(exchange(&mut esc, 1_000_000, a), Some((3, 100)));
        assert_eq!(exchange(&mut esc, 1_001_000, a), Some((3, 100)));
        let stop = TestCmd {
            seq: 2,
            ..TestCmd::INITIAL
        };
        assert_eq!(exchange(&mut esc, 1_002_000, stop), None);
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(1, 3, 100, TestCause::Stop, Some(1_000_000), 1_002_000)
        );
        assert_eq!(s.test_spin, None);
        let c = spin_cmd(3, 1, 50, 10_000_000);
        assert_eq!(exchange(&mut esc, 2_000_000, c), Some((1, 50)));
        assert_eq!(
            exchange(&mut esc, 2_001_000, TestCmd { level: 0, ..c }),
            None
        );
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(3, 1, 50, TestCause::Stop, Some(2_000_000), 2_001_000)
        );
        let d = spin_cmd(4, 3, 100, 20_000_000);
        let b = spin_cmd(5, 0, 80, 20_000_000);
        assert_eq!(exchange(&mut esc, 3_000_000, d), Some((3, 100)));
        assert_eq!(exchange(&mut esc, 3_500_000, d), Some((3, 100)));
        assert_eq!(exchange(&mut esc, 3_501_000, b), Some((0, 80)));
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(4, 3, 100, TestCause::Replaced, Some(3_000_000), 3_501_000)
        );
        assert_eq!(
            s.test_spin.map(|t| (t.seq, t.first_us)),
            Some((5, 3_501_000))
        );
        assert_eq!(exchange(&mut esc, 6_500_000, b), Some((0, 80)));
        assert_eq!(exchange(&mut esc, 6_500_001, b), None);
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(5, 0, 80, TestCause::Limit, Some(3_501_000), 6_500_001)
        );
    }

    /// A fresh command passes, one older than the limit is Stop, the initial cell is Stop.
    #[test]
    fn hold_passes_a_fresh_command_and_stops_a_stale_one() {
        let spin = MotorCommand::spinning([120, 250, 400, 1000]);
        assert_eq!(hold(5_000, spin, 5_000), spin);
        assert_eq!(hold(5_000, spin, 5_000 + MOTOR_HOLD_MAX_US), spin);
        assert_eq!(
            hold(5_000, spin, 5_001 + MOTOR_HOLD_MAX_US),
            MotorCommand::STOP
        );
        assert_eq!(hold(0, MotorCommand::STOP, 0), MotorCommand::STOP);
        assert_eq!(hold(0, MotorCommand::STOP, 50_000_000), MotorCommand::STOP);
        assert!(!stale_spin(5_000, spin, 5_000));
        assert!(!stale_spin(5_000, spin, 5_001 + MOTOR_HOLD_MAX_US));
        assert!(!stale_spin(5_000, spin, 5_000 + FLIGHT_GAP_DISARM_US));
        assert!(stale_spin(5_000, spin, 5_001 + FLIGHT_GAP_DISARM_US));
        assert!(!stale_spin(0, MotorCommand::STOP, 50_000_000));
    }

    /// A spinning flight command gives its throttles on all four lines, cuts the EDT burst and ends a running motor test as `Replaced`.
    #[test]
    fn a_spinning_flight_command_takes_all_four_lines() {
        let mut esc = Esc::new();
        esc.start_begin();
        esc.start_end(0);
        let silent = LineCapture::Silent;
        let caps = [stopped(), stopped(), silent, silent];
        let mut edt: [Vec<u64>; LINES] = Default::default();
        let mut thr = Vec::new();
        run_cmd(
            &mut esc,
            0,
            1003,
            TestCmd::INITIAL,
            caps,
            &mut edt,
            &mut thr,
        );
        assert_eq!(edt[0], [1000, 1001, 1002]);
        let levels = [120, 250, 400, 1000];
        let spin = MotorCommand::spinning(levels);
        for t in 1003..1100u64 {
            let frames = esc.frames(t * 1000, TestCmd::INITIAL, (t * 1000 - 1000, spin));
            esc.record(t * 1000, 145, &caps);
            for (f, level) in frames.iter().zip(levels) {
                assert!(!f.telemetry_request);
                assert!(
                    matches!(f.value, Value::Throttle(x) if x.level() == level),
                    "t {t}"
                );
            }
            assert!(esc.status().flight_spin);
        }
        assert_eq!(esc.status().flight_frames, 97);
        assert_eq!(esc.status().lines[0].edt_bursts, 1);
        let stale = (1_099_500, spin);
        for t in 1100..2100u64 {
            let frames = esc.frames(t * 1000, TestCmd::INITIAL, stale);
            esc.record(t * 1000, 145, &caps);
            if t >= 1110 {
                only_stop_or_13(&frames);
                assert!(!esc.status().flight_spin);
            }
            for (l, f) in frames.iter().enumerate() {
                if f.value == EDT {
                    edt[l].push(t);
                }
            }
        }
        let expect: Vec<u64> = [1000, 1001, 1002].into_iter().chain(2003..2013).collect();
        assert_eq!(edt[0], expect);
        assert_eq!(esc.status().flight_frames, 107);
        let test = spin_cmd(1, 2, 50, u64::MAX);
        let frames = esc.frames(3_000_000, test, stale);
        assert_eq!(throttle_of(&frames), Some((2, 50)));
        let frames = esc.frames(3_001_000, test, (3_000_500, spin));
        assert!(frames.iter().all(|f| matches!(f.value, Value::Throttle(_))));
        assert_eq!(
            esc.status().test_ended,
            ending(1, 2, 50, TestCause::Replaced, Some(3_000_000), 3_001_000)
        );
        assert_eq!(esc.status().test_spin, None);
        assert_eq!(throttle_of(&esc.frames(3_002_000, test, stale)), None);
    }

    /// A 50 ms clock jump: the next frame is throttle if the spin still holds, Stop if it expired meanwhile; one frame
    /// per exchange, no catch-up.
    #[test]
    fn a_clock_jump_gives_stop_if_expired_and_no_catch_up() {
        let mut esc = Esc::new();
        let cmd = spin_cmd(1, 2, 100, 1_000_000);
        for t in (0..=900_000).step_by(1000) {
            assert_eq!(exchange(&mut esc, t, cmd), Some((2, 100)));
        }
        assert_eq!(exchange(&mut esc, 950_000, cmd), Some((2, 100)));
        assert_eq!(exchange(&mut esc, 989_000, cmd), Some((2, 100)));
        assert_eq!(exchange(&mut esc, 1_039_000, cmd), None);
        assert_eq!(exchange(&mut esc, 1_040_000, cmd), None);
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(1, 2, 100, TestCause::Expiry, Some(0), 1_039_000)
        );
        assert_eq!(s.throttle_frames, 903);
        let cmd = spin_cmd(2, 2, 100, u64::MAX);
        assert_eq!(exchange(&mut esc, 10_000_000, cmd), Some((2, 100)));
        assert_eq!(exchange(&mut esc, 12_980_000, cmd), Some((2, 100)));
        assert_eq!(exchange(&mut esc, 13_030_000, cmd), None);
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(2, 2, 100, TestCause::Limit, Some(10_000_000), 13_030_000)
        );
        assert_eq!(s.throttle_frames, 905);
    }
}
