//! ESC telemetry logic for the flight core's `esc` task: pure functions and plain data. No HAL, no
//! clock of its own (every function is given `now_us`), no logging, no lock.
//! Design: `decision-esc-telemetry-20261118` Q1, Q3 and Q4.
//!
//! The task, per exchange: [`Esc::frames`] chooses the four frames, the driver exchanges them,
//! [`Esc::record`] folds the four captures in and the task publishes [`Esc::status`]; the wait for
//! the next exchange uses [`next_deadline`]. The status type [`EscStatus`] is `fw_core::esc`'s (core
//! 0's builders live there); it is re-exported here.
//!
//! In this version a frame is only ever [`Value::Stop`] or, for a command-13 burst on one line,
//! [`Command::ExtendedTelemetryOn`]. No `Throttle` is constructed anywhere.

use dshot::{Command, LineCapture, LineFrame, Reply, Value};
pub use fw_core::esc::{ANSWER_US, ERPM_MAX, EscLine, EscStatus, FRESH_US, LINES, line_of_element};

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

/// The ESC task's state: the counters and latest values, and the burst state of each line.
pub struct Esc {
    status: EscStatus,
    bursts: [Burst; LINES],
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

    /// The four frames of the exchange at `now_us`, by driver line: Stop, except on a line in an
    /// EDT burst. A line starts a burst when it has answered continuously for 1 s or more, has
    /// given no extended reply for 1 s (or ever), and is not held off; the burst is
    /// [`BURST_FRAMES`] consecutive `ExtendedTelemetryOn` frames on that line only, then a 1 s
    /// hold-off. Nothing here depends on errors: the choice never stops the stream.
    pub fn frames(&mut self, now_us: u64) -> [LineFrame; LINES] {
        let mut out = [LineFrame::STOP; LINES];
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
        let frames = esc.frames(now);
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
    }
}
