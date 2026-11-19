//! ESC telemetry logic for the flight core's `esc` task: pure functions and plain data. No HAL, no
//! clock of its own (every function is given `now_us`), no logging, no lock.
//! Design: `decision-esc-telemetry-20261118` Q1, Q3 and Q4.
//!
//! The task, per exchange: [`Esc::frames`] chooses the four frames, the driver exchanges them,
//! [`Esc::record`] folds the four captures in and the task publishes [`Esc::status`]; the wait for
//! the next exchange uses [`next_deadline`]. The status type [`EscStatus`] is `fw_core::esc`'s (core
//! 0's builders live there); it is re-exported here.
//!
//! A frame is [`Value::Stop`]; or, for a command-13 burst on one line,
//! [`Command::ExtendedTelemetryOn`]; or, on the one line of a motor-test spin, a throttle built by
//! `capped` (the firmware's only `Throttle` constructor). Motor test: `decision-motor-test-20261118`
//! amendment A, ported from `dshot_spin` (DS = `dshot/hil/src/bin/dshot_spin.rs` at dshot 3557eb1).

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

/// The deadline of the exchange after the one due at `deadline_us`, given `now_us` (the time the
/// task looks at the clock): `deadline + 1000 us`; but when `now` is already more than one period
/// past that, `now`, so a stall costs one late frame and never a burst of catch-up frames.
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
        }
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
    /// command `test`.
    ///
    /// While a spin is active (see `spin`): its throttle on its line, Stop on the other
    /// three, and no command 13 on any line: a running EDT burst is cut and that line held off
    /// 1 s from the cut. These are the frames DS:863-866 sent.
    ///
    /// Otherwise: Stop, except on a line in an EDT burst. A line starts a burst when it has
    /// answered continuously for 1 s or more, has given no extended reply for 1 s (or ever), and
    /// is not held off; the burst is [`BURST_FRAMES`] consecutive `ExtendedTelemetryOn` frames on
    /// that line only, then a 1 s hold-off. Nothing here depends on errors: the choice never stops
    /// the stream.
    pub fn frames(&mut self, now_us: u64, test: TestCmd) -> [LineFrame; LINES] {
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
        for ((frame, burst), line) in out
            .iter_mut()
            .zip(self.bursts.iter_mut())
            .zip(self.status.lines.iter_mut())
        {
            if burst.left == 0 && burst_due(line, burst, now_us) {
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

    /// The throttle and its driver line for the exchange at `now_us`, if `test` keeps a spin
    /// active, and the spin status and ending. A spin is active while all of these hold, checked
    /// on every frame in this order (the cap before the time limit, as DS:401-440):
    ///
    /// 1. the level is above 0 and at or below [`TEST_CAP`], the throttle built by `capped`
    ///    (DS:175-185, checked again per frame as DS:404-417 does) and the line is 0..=3;
    /// 2. `now < until_us`;
    /// 3. the frame is within [`TEST_BURST_MAX_US`] of the spin's first throttle frame, with the
    ///    [`TEST_PUSH_MARGIN_US`] margin (DS:418-440: the burst starts at its first throttle frame).
    ///
    /// A new `seq` ends a running spin (cause `Replaced`, or `Stop` for a level 0) and starts its
    /// own count. A command ends once; its `seq` gets no throttle again. Nothing catches up: every
    /// frame is judged at its own `now_us`.
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
    ///
    /// - A good reply is [`LineCapture::Reply`]; every other capture counts as a non-reply.
    /// - A line is answering while its latest good reply is younger than 100 ms; when that stops
    ///   being true (found at the next exchange, whatever came with it) it counts one dropout, and
    ///   its answering run starts again at its next good reply.
    /// - An eRPM above 1 000 000 does not replace the stored eRPM.
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

/// G3 (DS:175-185): the one place a throttle is built. A level above [`TEST_CAP`] (or outside
/// 1..=2000) is refused.
fn capped(level: u16) -> Option<Throttle> {
    if !within_cap(level) {
        return None;
    }
    Throttle::new(level)
}

/// G3 (DS:183-185): whether a level is at or below the cap.
const fn within_cap(level: u16) -> bool {
    level <= TEST_CAP
}

/// G4 (DS:187-191): whether a frame pushed `elapsed_us` after the start of its spin stays within
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

    /// One exchange at `t_ms` milliseconds; returns the frames sent.
    fn step(esc: &mut Esc, t_ms: u64, caps: [LineCapture; LINES]) -> [LineFrame; LINES] {
        let now = t_ms * 1000;
        let frames = esc.frames(now, TestCmd::INITIAL);
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

    /// I2: `next += 1000`; more than one period behind: `next = now`.
    #[test]
    fn deadline_advances_by_one_period_and_a_stall_gives_one_late_frame() {
        let mut next = 5_000;
        for k in 1..=100u64 {
            next = next_deadline(next, next + 145);
            assert_eq!(next, 5_000 + k * 1000);
        }
        // One period behind is still caught up by the normal step, more than one is not.
        assert_eq!(next_deadline(10_000, 12_000), 11_000);
        assert_eq!(next_deadline(10_000, 12_001), 12_001);
        // A 50 ms stall: the next deadline is `now`, then the period is normal again.
        let stalled_now = 10_000 + 50_000;
        let next = next_deadline(10_000, stalled_now);
        assert_eq!(next, stalled_now);
        assert_eq!(next_deadline(next, next + 145), stalled_now + 1000);
    }

    /// I3: nothing stops on errors; I1: only Stop comes out.
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

    /// I6: the gate, the burst, the hold-off, the restart after a dropout, no burst while extended
    /// replies arrive. Lines 0 and 1 answer; 2 and 3 are silent.
    #[test]
    fn the_edt_burst_follows_the_gate() {
        let mut esc = Esc::new();
        esc.start_begin();
        esc.start_end(0);
        let silent = LineCapture::Silent;
        let mut bursts: [Vec<u64>; LINES] = Default::default();
        // 1. Answering from t = 0 (the first reply at the first exchange): no frame before 1 s.
        let both = |_t: u64| [stopped(), stopped(), silent, silent];
        run(&mut esc, &mut bursts, 0, 1000, &both);
        assert!(bursts.iter().all(|b| b.is_empty()));
        // 2. At t = 1000 ms exactly 10 consecutive frames on lines 0 and 1 only.
        run(&mut esc, &mut bursts, 1000, 1100, &both);
        let expect: Vec<u64> = (1000..1010).collect();
        assert_eq!(bursts[0], expect);
        assert_eq!(bursts[1], expect);
        assert!(bursts[2].is_empty() && bursts[3].is_empty());
        assert_eq!(esc.status().lines[0].edt_bursts, 1);
        assert_eq!(esc.status().lines[2].edt_bursts, 0);
        // 3. Hold-off of 1 s after the 10th frame (t = 1009): none before t = 2009.
        run(&mut esc, &mut bursts, 1100, 2009, &both);
        assert_eq!(bursts[0].len(), 10);
        // 4. No extended reply came: a second burst at 2009.
        run(&mut esc, &mut bursts, 2009, 2100, &both);
        assert_eq!(bursts[0].len(), 20);
        assert_eq!(bursts[0].get(10), Some(&2009));
        // 5. A dropout (line 1 silent 150 ms) and its return: the 1 s count restarts, so the burst
        // comes 1 s after the first good reply back. Line 0 keeps answering and sees extended
        // replies every 100 ms: no burst on it.
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
            esc.frames(now, TestCmd::INITIAL);
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

    /// The line and level of the throttle frame, checking the motor-test frame rules: at most one
    /// throttle frame, never above the cap, and only Stop beside it.
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
        let frames = esc.frames(now_us, test);
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

    /// T1: a level above the cap gives Stop and a Cap ending; `capped` never builds above the cap.
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
            assert_eq!(esc.frames(t * 1000, over), [LineFrame::STOP; LINES]);
        }
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(1, 2, TEST_CAP + 1, TestCause::Cap, None, 0)
        );
        assert_eq!((s.test_spin, s.throttle_frames), (None, 0));
        // A spin at the cap, then a new seq above it: Stop on all four from that exchange.
        let at_cap = spin_cmd(2, 2, TEST_CAP, 1_000_000);
        assert_eq!(exchange(&mut esc, 10_000, at_cap), Some((2, TEST_CAP)));
        let over = spin_cmd(3, 2, TEST_CAP + 1, 1_000_000);
        assert_eq!(esc.frames(11_000, over), [LineFrame::STOP; LINES]);
        let s = esc.status();
        assert_eq!(
            s.test_ended,
            ending(3, 2, TEST_CAP + 1, TestCause::Cap, None, 11_000)
        );
        // A line above 3 is refused the same way.
        assert_eq!(
            esc.frames(12_000, spin_cmd(4, 4, 50, 1_000_000)),
            [LineFrame::STOP; LINES]
        );
        let s = esc.status();
        assert_eq!(s.test_ended, ending(4, 4, 50, TestCause::Cap, None, 12_000));
        assert_eq!((s.test_spin, s.throttle_frames), (None, 1));
    }

    /// Runs exchanges `from..to` ms with `test`, recording the times of command-13 frames per line
    /// and of throttle frames with their line.
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
            let frames = esc.frames(t * 1000, test);
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

    /// T2: one throttle frame, Stop on the other three, no command 13 during a spin; a running EDT
    /// burst is cut, held off 1 s from the cut, and the M1 gate resumes after the spin.
    #[test]
    fn a_spin_sends_one_throttle_frame_and_cuts_the_edt_burst() {
        let mut esc = Esc::new();
        esc.start_begin();
        esc.start_end(0);
        let silent = LineCapture::Silent;
        let caps = [stopped(), stopped(), silent, silent];
        let mut edt: [Vec<u64>; LINES] = Default::default();
        let mut thr = Vec::new();
        // Lines 0 and 1 are 3 frames into their burst (1000..1010) when the spin starts.
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
        // Stop at 1500; the cut lines' next burst is 1 s after the cut (1003), not after the spin.
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

    /// T3: a spin ends on the task's clock at min(until, first + 3 s) with no further command.
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
        // A far-future until: the last throttle frame 3 s - 1 ms after the first, then Stop.
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

    /// T4: a stop ends a spin at the next exchange; a new seq on another line replaces it and its
    /// 3 s count from its own first throttle frame.
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
        // A stop with the spin's own seq ends it as well.
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
        // Replace: the new line from the next exchange, the earlier line Stop in it.
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

    /// T7: a 50 ms clock jump: the next frame is throttle if the spin still holds, Stop if it
    /// expired meanwhile; one frame per exchange, no catch-up.
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
        // Across the 3 s limit.
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
