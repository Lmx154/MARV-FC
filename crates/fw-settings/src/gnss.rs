//! The GNSS receiver's parameter, its message set and the commands that apply it
//! (`decision-sensor-presets-20261117` Amendment C and C-b).
//!
//! The receiver is told its fix rate and its sentence rates, nothing else: never the navigation mode,
//! PPS, baud rate, a restart or a save. Nothing here touches hardware; [`commands`] is the whole list the
//! adapter may send, in order.

use fw_core::params::{Apply, Candidate, ParamDef};
use lc76g::{Command, Config, SentenceRates, SentenceType};
use mavlink_device::marv::MarvDevice;

const DEVICE: u8 = MarvDevice::MARV_DEVICE_GNSS as u8;

/// The position fix interval in milliseconds: one of 100, 200, 500 or 1000 (divisors of 1000).
pub const RATE_MS: &str = "GPS_RATE_MS";

/// The number of GNSS parameters.
pub const COUNT: usize = 1;

/// `GPS_RATE_MS`, appended to the firmware's table at index 80: `I32`, default 200, range 100 to 1000,
/// `Configure`. Device GNSS (5). The range is wider than the values [`validate`] accepts.
pub static TABLE: [ParamDef; COUNT] = [ParamDef::i32(
    RATE_MS,
    DEVICE,
    200,
    100,
    1000,
    Apply::Configure,
)];

const BAD_RATE: &str = "not 100, 200, 500 or 1000";

/// The most commands [`commands`] returns: version query, interval 1000, ten sentence rates, EPE, PVT rate,
/// interval.
pub const MAX_COMMANDS: usize = 15;

/// The store's validator for the GNSS receiver: 100, 200, 500 or 1000 ms.
pub fn validate(candidate: &Candidate<'_>) -> Result<(), &'static str> {
    match candidate.i32(RATE_MS) {
        Some(100 | 200 | 500 | 1000) => Ok(()),
        _ => Err(BAD_RATE),
    }
}

/// The stored `GPS_RATE_MS` as the fix interval to configure: the value when [`validate`] accepts it,
/// otherwise its refusal. A boot load runs no validator, so the adapter asks this before it configures.
pub fn rate_ms(value: Option<i32>) -> Result<u16, &'static str> {
    match value {
        Some(v @ (100 | 200 | 500 | 1000)) => u16::try_from(v).map_err(|_| BAD_RATE),
        _ => Err(BAD_RATE),
    }
}

/// The receiver's configuration for a fix interval: the driver's `Config::with_fix_interval(ms)` with GGA,
/// RMC and GST every fix, GSA every `ceil(1000 / ms)` fixes, GLL, VTG, GSV, ZDA, GRS and GNS off, EPE off,
/// `$PQTMPVT` every fix (`pvt_rate` 1). The navigation mode and PPS of the result are never sent by [`commands`].
pub fn config(ms: u16) -> Config {
    let base = Config::with_fix_interval(ms);
    Config {
        rates: SentenceRates {
            gga: 1,
            gll: 0,
            gsa: base.rates.gsa,
            gsv: 0,
            rmc: 1,
            vtg: 0,
            zda: 0,
            grs: 0,
            gst: 1,
            gns: 0,
        },
        epe: false,
        pvt_rate: 1,
        ..base
    }
}

/// The least time from the acknowledgement of a fix interval to the sending of the next fix interval of the
/// same list (the target, when it is not 1000 ms), ms. Measured on the bench (LC76GABNR12A05S, firmware
/// 2025/09/25, 2026-11-20): a `$PAIR050` sent 18 to 170 ms after the acknowledgement of one that changed the
/// rate is answered `$PAIR001,050,1` (being processed), then the receiver restarts by itself about 1.8 s
/// later, at its stored settings (8 of 8); sent 207 ms or more after it, it is accepted (7 of 7; once also at
/// 97 ms). After an applied change the receiver sends an unsolicited `$PAIR001,124,0` 114 to 241 ms after the
/// acknowledgement. 500 ms is twice the latest of those (241 ms) and three times the latest restart (170 ms);
/// with the driver's worst measured acknowledgement (93 ms) the 15 commands take at most 15 x 93 + 500 = 1895 ms
/// (1.9 s), inside T.
pub const FIX_INTERVAL_SETTLE_MS: u64 = 500;

/// When `command` may be sent, in ms of the caller's clock: not before `Some(t)`, for a fix interval that
/// follows a fix interval of the same list acknowledged at `interval_acked_ms`; `None`: at once.
pub fn not_before_ms(command: &Command<'_>, interval_acked_ms: Option<u64>) -> Option<u64> {
    match command {
        Command::SetFixInterval { .. } => {
            interval_acked_ms.map(|t| t.saturating_add(FIX_INTERVAL_SETTLE_MS))
        }
        _ => None,
    }
}

/// The wait from a failed configuration to the next try of the whole list, ms. Measured on the bench
/// (LC76GABNR12A05S, firmware 2025/09/25, 2026-11-20): after a cold power-on the receiver starts at the fix
/// interval it had before (kept through 47 s without power) with its default sentence set, and a fix-interval
/// change sent 0.1 s after its first sentence is answered `$PAIR001,050,1` (being processed) and never
/// completed (3 of 3); the same list passes once the receiver has been up for a while (warm resets). 5000 is the
/// figure of the start wait for the first sentence (`START_WAIT`, 5 s); the tries repeat until one succeeds.
pub const CONFIGURE_RETRY_MS: u64 = 5000;

/// What a finished configuration reports to the link.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RetryReport {
    /// Nothing: a first try that succeeded, or a retry that failed again.
    Quiet,
    /// The failure of a first try (start, restart or a parameter write): the WARNING.
    Warn,
    /// A retry succeeded.
    Applied {
        /// The number of the try that succeeded (2 or more).
        tries: u32,
    },
}

/// The retry schedule of the configuration: a failed list is sent again [`CONFIGURE_RETRY_MS`] after the
/// failure, until a try succeeds. Only a first try's failure is reported; a retry's success is reported once.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Retry {
    tries: u32,
    due_ms: Option<u64>,
}

impl Retry {
    /// A configuration begins: a retry this schedule called for (`retry`), or a first try (start, restart,
    /// parameter write), which starts a new count. Returns the try's number (1 for a first try).
    pub fn begin(&mut self, retry: bool) -> u32 {
        if !retry {
            self.tries = 0;
        }
        self.tries = self.tries.saturating_add(1);
        self.due_ms = None;
        self.tries
    }

    /// The configuration that [`Retry::begin`] began ended at `now_ms`; `ok` when the whole list was answered.
    pub fn end(&mut self, ok: bool, now_ms: u64) -> RetryReport {
        if ok {
            self.due_ms = None;
            return match self.tries {
                0 | 1 => RetryReport::Quiet,
                tries => RetryReport::Applied { tries },
            };
        }
        self.due_ms = Some(now_ms.saturating_add(CONFIGURE_RETRY_MS));
        if self.tries <= 1 {
            RetryReport::Warn
        } else {
            RetryReport::Quiet
        }
    }

    /// Whether a retry is due at `now_ms`.
    pub fn due(&self, now_ms: u64) -> bool {
        self.due_ms.is_some_and(|t| now_ms >= t)
    }
}

/// A fixed-capacity, ordered list of commands (no allocator).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandList {
    items: [Command<'static>; MAX_COMMANDS],
    len: usize,
}

impl CommandList {
    /// The commands, in the order to send them.
    pub fn as_slice(&self) -> &[Command<'static>] {
        self.items.get(..self.len).unwrap_or(&[])
    }

    fn push(&mut self, command: Command<'static>) {
        if let Some(slot) = self.items.get_mut(self.len) {
            *slot = command;
            self.len += 1;
        }
    }
}

/// The commands that apply `config`, in the driver's `init` order without the navigation mode and PPS:
/// the version query, fix interval 1000 ms, the rate of every sentence type, EPE, the `$PQTMPVT` rate
/// (`$PQTMCFGMSGRATE,W,PQTMPVT,<rate>,1`, acknowledged `OK` like EPE), then the fix interval
/// of `config` last and only if it is not 1000 ms (the receiver is at 1 Hz while the rates change). The
/// sender holds that last command until [`not_before_ms`].
/// Only these five kinds of command: never a save, a restart, a baud rate, a mode, PPS or a raw sentence.
pub fn commands(config: &Config) -> CommandList {
    let mut list = CommandList {
        items: [Command::QueryVersion; MAX_COMMANDS],
        len: 0,
    };
    list.push(Command::QueryVersion);
    list.push(Command::SetFixInterval { ms: 1000 });
    for sentence in SentenceType::ALL {
        list.push(Command::SetSentenceRate {
            sentence,
            rate: config.rates.get(sentence),
        });
    }
    list.push(Command::SetEpe(config.epe));
    list.push(Command::SetPvt {
        rate: config.pvt_rate,
    });
    if config.fix_interval_ms != 1000 {
        list.push(Command::SetFixInterval {
            ms: config.fix_interval_ms,
        });
    }
    list
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamStore, ParamValue};
    use std::vec::Vec;

    const ALLOWED: [u16; 4] = [100, 200, 500, 1000];

    #[test]
    fn config_is_the_decided_message_set_for_every_allowed_rate() {
        for ms in ALLOWED {
            let c = config(ms);
            assert!(c.is_valid(), "{ms}");
            assert_eq!(c.fix_interval_ms, ms);
            let r = c.rates;
            assert_eq!((r.gga, r.rmc, r.gst), (1, 1, 1), "{ms}");
            assert_eq!(u32::from(r.gsa), 1000u32.div_ceil(u32::from(ms)), "{ms}");
            assert!(u32::from(r.gsa) * u32::from(ms) >= 1000, "{ms}");
            assert_eq!(
                (r.gll, r.vtg, r.gsv, r.zda, r.grs, r.gns),
                (0, 0, 0, 0, 0, 0),
                "{ms}"
            );
            assert!(!c.epe);
            assert_eq!(c.pvt_rate, 1, "{ms}");
        }
        assert_eq!(config(200).rates.gsa, 5);
        assert_eq!(config(100).rates.gsa, 10);
    }

    /// The order: version, 1000 ms, the ten rates, EPE, the PVT rate, then the target interval last.
    #[test]
    fn commands_are_in_the_decided_order() {
        for ms in ALLOWED {
            let list = commands(&config(ms));
            let cmds = list.as_slice();
            assert_eq!(cmds[0], Command::QueryVersion);
            assert_eq!(cmds[1], Command::SetFixInterval { ms: 1000 });
            for (i, s) in SentenceType::ALL.iter().enumerate() {
                assert_eq!(
                    cmds[2 + i],
                    Command::SetSentenceRate {
                        sentence: *s,
                        rate: config(ms).rates.get(*s)
                    },
                    "{ms}"
                );
            }
            assert_eq!(cmds[12], Command::SetEpe(false));
            assert_eq!(cmds[13], Command::SetPvt { rate: 1 }, "{ms}");
            let intervals = cmds
                .iter()
                .filter(|c| matches!(c, Command::SetFixInterval { .. }))
                .count();
            if ms == 1000 {
                assert_eq!(cmds.len(), 14);
                assert_eq!(intervals, 1);
            } else {
                assert_eq!(cmds.len(), 15);
                assert_eq!(intervals, 2);
                assert_eq!(cmds[14], Command::SetFixInterval { ms });
            }
        }
    }

    /// Only the five allowed kinds: no navigation mode, PPS, save, restart or raw command.
    #[test]
    fn commands_are_only_version_interval_rate_epe_and_pvt() {
        for ms in ALLOWED {
            for c in commands(&config(ms)).as_slice() {
                assert!(
                    matches!(
                        c,
                        Command::QueryVersion
                            | Command::SetFixInterval { .. }
                            | Command::SetSentenceRate { .. }
                            | Command::SetEpe(_)
                            | Command::SetPvt { rate: 1 }
                    ),
                    "{c:?}"
                );
            }
        }
    }

    /// The whole list for 200 ms: version, 1000 ms, the ten rates, EPE off, PVT every fix, the target interval.
    #[test]
    fn commands_for_200_ms_are_exactly_these_fifteen() {
        let rate = |sentence, rate| Command::SetSentenceRate { sentence, rate };
        let list = commands(&config(200));
        assert_eq!(
            list.as_slice(),
            [
                Command::QueryVersion,
                Command::SetFixInterval { ms: 1000 },
                rate(SentenceType::Gga, 1),
                rate(SentenceType::Gll, 0),
                rate(SentenceType::Gsa, 5),
                rate(SentenceType::Gsv, 0),
                rate(SentenceType::Rmc, 1),
                rate(SentenceType::Vtg, 0),
                rate(SentenceType::Zda, 0),
                rate(SentenceType::Grs, 0),
                rate(SentenceType::Gst, 1),
                rate(SentenceType::Gns, 0),
                Command::SetEpe(false),
                Command::SetPvt { rate: 1 },
                Command::SetFixInterval { ms: 200 },
            ]
        );
    }

    #[test]
    fn the_validator_accepts_the_four_rates_and_refuses_the_rest() {
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        assert!(p.set_validator(DEVICE, validate));
        let def = &TABLE[0];
        assert_eq!(def.name, "GPS_RATE_MS");
        assert_eq!(def.device, 5);
        assert_eq!(def.default, ParamValue::I32(200));
        assert_eq!(
            (def.min, def.max),
            (ParamValue::I32(100), ParamValue::I32(1000))
        );
        assert_eq!(def.apply, Apply::Configure);
        for ok in [100, 200, 500, 1000] {
            assert_eq!(p.set(80, ParamValue::I32(ok)), Ok(ParamValue::I32(ok)));
        }
        assert_eq!(p.set(80, ParamValue::I32(300)), Err(ParamError::Rejected));
        assert_eq!(
            p.take_notice().unwrap().kind.text(),
            "not 100, 200, 500 or 1000"
        );
        // The STATUSTEXT puts the name before the text: the whole sentence fits its 50 characters once.
        assert!("GPS_RATE_MS: ".len() + BAD_RATE.len() <= 50);
        // 50 is below the range: refused before the validator is asked.
        assert_eq!(p.set(80, ParamValue::I32(50)), Err(ParamError::OutOfRange));
    }

    /// `rate_ms` accepts exactly what the validator accepts, over the whole range and beyond it.
    #[test]
    fn rate_ms_agrees_with_the_validator() {
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        assert!(p.set_validator(DEVICE, validate));
        for v in 100..=1000 {
            let accepted = p.set(80, ParamValue::I32(v)).is_ok();
            assert_eq!(rate_ms(Some(v)).is_ok(), accepted, "{v}");
            if accepted {
                assert_eq!(rate_ms(Some(v)), Ok(v as u16));
            }
        }
        assert_eq!(rate_ms(Some(300)), Err(BAD_RATE));
        for v in [None, Some(0), Some(-200), Some(65_736), Some(i32::MAX)] {
            assert_eq!(rate_ms(v), Err(BAD_RATE), "{v:?}");
        }
    }

    /// The receiver as measured on the bench (see [`FIX_INTERVAL_SETTLE_MS`]): a fix interval that arrives
    /// within `UNSAFE_MS` of the acknowledgement of a fix interval that changed the rate gets no final
    /// answer and the receiver restarts at its stored 1000 ms; anything else is acknowledged `ack_ms` after it
    /// arrives.
    struct Receiver {
        rate: u16,
        ack_ms: u64,
        unsafe_until: Option<u64>,
        /// After a cold power-on (measured, see [`CONFIGURE_RETRY_MS`]): a fix interval that would change the
        /// rate before this time is answered "being processed" only, never completed; the rate stays.
        cold_until: u64,
    }

    /// Above the latest restart (170 ms) and the latest `$PAIR001,124,0` (241 ms) measured.
    const UNSAFE_MS: u64 = 250;

    impl Receiver {
        fn receive(&mut self, command: &Command<'_>, t: u64) -> Option<u64> {
            let ack = t + self.ack_ms;
            if let Command::SetFixInterval { ms } = *command {
                if ms != self.rate && t < self.cold_until {
                    return None;
                }
                if self.unsafe_until.is_some_and(|u| t < u) {
                    self.rate = 1000;
                    self.unsafe_until = None;
                    return None;
                }
                if ms != self.rate {
                    self.rate = ms;
                    self.unsafe_until = Some(ack + UNSAFE_MS);
                }
            }
            Some(ack)
        }
    }

    /// Sends the list of `to` to a receiver at `from`, as the adapter does (each command after the
    /// acknowledgement of the one before, held until `not_before_ms` when `hold`). The receiver's rate at
    /// the end, whether every command was answered, and the time taken, ms.
    fn send(from: u16, to: u16, ack_ms: u64, hold: bool) -> (u16, bool, u64) {
        let mut receiver = Receiver {
            rate: from,
            ack_ms,
            unsafe_until: None,
            cold_until: 0,
        };
        let mut t = 0;
        let mut interval_acked = None;
        for command in commands(&config(to)).as_slice() {
            if hold && let Some(not_before) = not_before_ms(command, interval_acked) {
                t = t.max(not_before);
            }
            let Some(ack) = receiver.receive(command, t) else {
                return (receiver.rate, false, t);
            };
            t = ack;
            if matches!(command, Command::SetFixInterval { .. }) {
                interval_acked = Some(t);
            }
        }
        (receiver.rate, true, t)
    }

    /// Every transition among the allowed rates, at the fastest and the slowest acknowledgement measured
    /// (4.4 and 93 ms, lc76g README): the receiver ends at the target, every command answered, inside T.
    #[test]
    fn every_transition_reaches_the_target_inside_the_deadline() {
        for from in ALLOWED {
            for to in ALLOWED {
                for ack_ms in [5, 93] {
                    let (rate, answered, took) = send(from, to, ack_ms, true);
                    assert!(answered, "{from} -> {to}, ack {ack_ms} ms");
                    assert_eq!(rate, to, "{from} -> {to}, ack {ack_ms} ms");
                    assert!(took <= 2500, "{from} -> {to}, ack {ack_ms} ms: {took} ms");
                }
            }
        }
    }

    /// The scripted receiver shows the bench failure when the target follows at once: from a rate other
    /// than 1000 ms to another one other than 1000 ms, the list ends unanswered with the receiver at 1000 ms.
    #[test]
    fn without_the_hold_the_scripted_receiver_restarts_as_on_the_bench() {
        for from in ALLOWED {
            for to in ALLOWED {
                let (rate, answered, _) = send(from, to, 5, false);
                let fails = from != 1000 && to != 1000;
                assert_eq!(answered, !fails, "{from} -> {to}");
                assert_eq!(rate, if fails { 1000 } else { to }, "{from} -> {to}");
            }
        }
    }

    /// A command without its final answer ends the try after the driver's 100 sentences: 1.48 s in the
    /// receiver's default set at 5 Hz (measured at three cold power-ons).
    const NO_ACK_MS: u64 = 1480;

    /// One try of the list for `to` from `t0` as the adapter sends it (with the hold): whether every command
    /// was answered, and when the try ended.
    fn try_list(receiver: &mut Receiver, to: u16, t0: u64) -> (bool, u64) {
        let mut t = t0;
        let mut interval_acked = None;
        for command in commands(&config(to)).as_slice() {
            if let Some(not_before) = not_before_ms(command, interval_acked) {
                t = t.max(not_before);
            }
            let Some(ack) = receiver.receive(command, t) else {
                return (false, t + NO_ACK_MS);
            };
            t = ack;
            if matches!(command, Command::SetFixInterval { .. }) {
                interval_acked = Some(t);
            }
        }
        (true, t)
    }

    /// A cold power-on as measured: the receiver at 200 ms, its first sentence 110 ms after the board's
    /// start, fix-interval changes not completed until `cold_ms`. The adapter's configuration with the retry
    /// (`retry`) or without (the code before it): the receiver's rate at the end, whether a try succeeded,
    /// the warnings, the "applied" reports and the tries made.
    fn cold_boot(cold_ms: u64, retry: bool) -> (u16, bool, u32, Vec<u32>, u32) {
        let mut receiver = Receiver {
            rate: 200,
            ack_ms: 5,
            unsafe_until: None,
            cold_until: cold_ms,
        };
        let mut schedule = Retry::default();
        let (mut t, mut warnings, mut applied, mut is_retry) = (110, 0, Vec::new(), false);
        for _ in 0..20 {
            let tries = schedule.begin(is_retry);
            let (ok, end) = try_list(&mut receiver, 200, t);
            match schedule.end(ok, end) {
                RetryReport::Warn => warnings += 1,
                RetryReport::Applied { tries } => applied.push(tries),
                RetryReport::Quiet => {}
            }
            if ok || !retry {
                return (receiver.rate, ok, warnings, applied, tries);
            }
            // The adapter looks at the schedule at every sentence; one fix every 200 ms is enough here.
            t = end;
            while !schedule.due(t) {
                t += 200;
            }
            assert!(t - end < CONFIGURE_RETRY_MS + 200);
            is_retry = true;
        }
        panic!("no try succeeded");
    }

    /// The cold power-on: the first try fails (one warning); the retry 5 s later succeeds and is reported
    /// once with its try number. A receiver that stays unready longer takes more tries, still one warning.
    /// The code without the retry stays failed.
    #[test]
    fn a_cold_receiver_is_configured_by_the_retry() {
        let (rate, ok, warnings, applied, tries) = cold_boot(3000, true);
        assert!(ok);
        assert_eq!((rate, warnings, applied, tries), (200, 1, std::vec![2], 2));

        let (_, ok, warnings, applied, tries) = cold_boot(9000, true);
        assert!(ok);
        assert_eq!((warnings, applied, tries), (1, std::vec![3], 3));

        let (_, ok, warnings, applied, tries) = cold_boot(3000, false);
        assert!(!ok);
        assert_eq!((warnings, applied, tries), (1, std::vec![], 1));

        // A warm start (the receiver up for a while): the first try succeeds, nothing reported.
        let (rate, ok, warnings, applied, tries) = cold_boot(0, true);
        assert!(ok);
        assert_eq!((rate, warnings, applied, tries), (200, 0, std::vec![], 1));
    }

    #[test]
    fn the_retry_schedule() {
        let mut r = Retry::default();
        assert!(!r.due(u64::MAX));
        assert_eq!(r.begin(false), 1);
        assert_eq!(r.end(true, 100), RetryReport::Quiet);
        assert!(!r.due(u64::MAX));
        // A first try fails: warned, due CONFIGURE_RETRY_MS later.
        assert_eq!(r.begin(false), 1);
        assert_eq!(r.end(false, 1000), RetryReport::Warn);
        assert!(!r.due(1000 + CONFIGURE_RETRY_MS - 1));
        assert!(r.due(1000 + CONFIGURE_RETRY_MS));
        // Retries: quiet when they fail, not due while one runs.
        assert_eq!(r.begin(true), 2);
        assert!(!r.due(u64::MAX));
        assert_eq!(r.end(false, 7000), RetryReport::Quiet);
        assert!(r.due(7000 + CONFIGURE_RETRY_MS));
        assert_eq!(r.begin(true), 3);
        assert_eq!(r.end(true, 13000), RetryReport::Applied { tries: 3 });
        assert!(!r.due(u64::MAX));
        // A later parameter write that fails starts a new count and warns again.
        assert_eq!(r.begin(false), 1);
        assert_eq!(r.end(false, 20000), RetryReport::Warn);
        assert_eq!(r.begin(false), 1);
    }

    #[test]
    fn only_a_fix_interval_after_a_fix_interval_is_held() {
        assert_eq!(
            not_before_ms(&Command::SetFixInterval { ms: 200 }, None),
            None
        );
        assert_eq!(
            not_before_ms(&Command::SetFixInterval { ms: 200 }, Some(40)),
            Some(40 + FIX_INTERVAL_SETTLE_MS)
        );
        assert_eq!(not_before_ms(&Command::SetEpe(false), Some(40)), None);
        assert_eq!(not_before_ms(&Command::QueryVersion, Some(40)), None);
    }
}
