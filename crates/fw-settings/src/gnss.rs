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

const BAD_RATE: &str = "GPS_RATE_MS: not 100, 200, 500 or 1000";

/// The most commands [`commands`] returns: version query, interval 1000, ten sentence rates, EPE, interval.
pub const MAX_COMMANDS: usize = 14;

/// The store's validator for the GNSS receiver: 100, 200, 500 or 1000 ms.
pub fn validate(candidate: &Candidate<'_>) -> Result<(), &'static str> {
    match candidate.i32(RATE_MS) {
        Some(100 | 200 | 500 | 1000) => Ok(()),
        _ => Err(BAD_RATE),
    }
}

/// The receiver's configuration for a fix interval: the driver's `Config::with_fix_interval(ms)` with GGA,
/// RMC and GST every fix, GSA every `ceil(1000 / ms)` fixes, GLL, VTG, GSV, ZDA, GRS and GNS off, EPE off.
/// The navigation mode and PPS of the result are never sent by [`commands`].
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
        ..base
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
/// the version query, fix interval 1000 ms, the rate of every sentence type, EPE, then the fix interval
/// of `config` last and only if it is not 1000 ms (the receiver is at 1 Hz while the rates change).
/// Only these four kinds of command: never a save, a restart, a baud rate, a mode, PPS or a raw sentence.
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
        }
        assert_eq!(config(200).rates.gsa, 5);
        assert_eq!(config(100).rates.gsa, 10);
    }

    /// The order: version, 1000 ms, the ten rates, EPE, then the target interval last.
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
            let intervals = cmds
                .iter()
                .filter(|c| matches!(c, Command::SetFixInterval { .. }))
                .count();
            if ms == 1000 {
                assert_eq!(cmds.len(), 13);
                assert_eq!(intervals, 1);
            } else {
                assert_eq!(cmds.len(), 14);
                assert_eq!(intervals, 2);
                assert_eq!(cmds[13], Command::SetFixInterval { ms });
            }
        }
    }

    /// Only the four allowed kinds: no navigation mode, PPS, save, restart or raw command.
    #[test]
    fn commands_are_only_version_interval_rate_and_epe() {
        for ms in ALLOWED {
            for c in commands(&config(ms)).as_slice() {
                assert!(
                    matches!(
                        c,
                        Command::QueryVersion
                            | Command::SetFixInterval { .. }
                            | Command::SetSentenceRate { .. }
                            | Command::SetEpe(_)
                    ),
                    "{c:?}"
                );
            }
        }
    }

    /// The whole list for 200 ms: version, 1000 ms, the ten rates, EPE off, the target interval.
    #[test]
    fn commands_for_200_ms_are_exactly_these_fourteen() {
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
            "GPS_RATE_MS: not 100, 200, 500 or 1000"
        );
        // 50 is below the range: refused before the validator is asked.
        assert_eq!(p.set(80, ParamValue::I32(50)), Err(ParamError::OutOfRange));
    }
}
