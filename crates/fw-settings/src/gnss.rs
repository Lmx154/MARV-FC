//! The GNSS receiver's parameter, its configuration and the mapping of its NAV-PVT to a `GnssReading`
//! (decision-gnss-m10-20261122, M1 architect ruling D2 and D3).
//!
//! The receiver is the u-blox SAM-M10Q, UBX only. The `sam-m10q` driver sends it one RAM-only configuration
//! (nothing is saved): this module only picks the measurement period from `GPS_RATE_MS`. Nothing here
//! touches hardware.

use fw_core::params::{Apply, Candidate, ParamDef};
use fw_core::sample::{GnssReading, UtcDate, UtcTime};
use mavlink_device::marv::MarvDevice;
use sam_m10q::{Config, Constellations, Pvt};

const DEVICE: u8 = MarvDevice::MARV_DEVICE_GNSS as u8;

/// The position fix interval in milliseconds: one of 100, 200, 500 or 1000 (`sam_m10q::config::MEAS_RATES_MS`), and
/// not faster than the chosen constellation set allows ([`SET`], `Constellations::min_meas_rate_ms`).
pub const RATE_MS: &str = "GPS_RATE_MS";

/// The constellation set: 0 = GPS + Galileo + BeiDou B1C + QZSS, 1 = GPS + Galileo, 2 = GPS + Galileo + GLONASS.
pub const SET: &str = "GPS_GNSS";

/// The number of GNSS parameters in the GNSS block of the table (`GPS_RATE_MS`).
pub const COUNT: usize = 1;

/// The number of GNSS parameters appended at the end of the table (`GPS_GNSS`).
pub const SET_COUNT: usize = 1;

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

/// `GPS_GNSS`, the last parameter of the table: `I32`, default 0, range 0 to 2, `Configure`. Device GNSS (5).
pub static SET_TABLE: [ParamDef; SET_COUNT] = [ParamDef::i32(SET, DEVICE, 0, 0, 2, Apply::Configure)];

const BAD_RATE: &str = "not 100, 200, 500 or 1000";
const BAD_SET: &str = "not 0, 1 or 2";

/// The constellation set of a `GPS_GNSS` value; an unknown or missing value is the default, 0. A boot load runs no
/// validator, so the adapter asks this before it configures.
pub fn constellations(value: Option<i32>) -> Constellations {
    match value {
        Some(1) => Constellations::GpsGal,
        Some(2) => Constellations::GpsGalGlo,
        _ => Constellations::GpsGalBdsQzss,
    }
}

/// Refuses a fix interval faster than `set` allows, the text naming the period and the `GPS_GNSS` value.
fn fits(ms: u16, set: Constellations) -> Result<(), &'static str> {
    if ms >= set.min_meas_rate_ms() {
        return Ok(());
    }
    // Only 100 ms is faster than any set's limit among the allowed periods.
    Err(match set {
        Constellations::GpsGalBdsQzss => "100 ms too fast for GPS_GNSS 0",
        Constellations::GpsGal => "100 ms too fast for GPS_GNSS 1",
        Constellations::GpsGalGlo => "100 ms too fast for GPS_GNSS 2",
    })
}

/// The store's validator for the GNSS receiver: the whole candidate. `GPS_RATE_MS` 100, 200, 500 or 1000 ms, `GPS_GNSS`
/// 0 to 2, and the period not faster than the set's limit (100 ms only with `GPS_GNSS` 1).
pub fn validate(candidate: &Candidate<'_>) -> Result<(), &'static str> {
    let ms = match candidate.i32(RATE_MS) {
        Some(v @ (100 | 200 | 500 | 1000)) => u16::try_from(v).map_err(|_| BAD_RATE)?,
        _ => return Err(BAD_RATE),
    };
    let set = match candidate.i32(SET) {
        Some(0) | None => Constellations::GpsGalBdsQzss,
        Some(1) => Constellations::GpsGal,
        Some(2) => Constellations::GpsGalGlo,
        Some(_) => return Err(BAD_SET),
    };
    fits(ms, set)
}

/// The stored `GPS_RATE_MS` as the fix interval to configure: the value when [`validate`] accepts it for a rate,
/// otherwise its refusal. A boot load runs no validator, so the adapter asks this before it configures.
pub fn rate_ms(value: Option<i32>) -> Result<u16, &'static str> {
    match value {
        Some(v @ (100 | 200 | 500 | 1000)) => u16::try_from(v).map_err(|_| BAD_RATE),
        _ => Err(BAD_RATE),
    }
}

/// The stored pair as the fix interval and set to configure: [`rate_ms`] and [`constellations`], and the period must fit
/// the set (a boot load runs no validator).
pub fn rate_and_set(rate: Option<i32>, set: Option<i32>) -> Result<(u16, Constellations), &'static str> {
    let ms = rate_ms(rate)?;
    let set = constellations(set);
    fits(ms, set)?;
    Ok((ms, set))
}

/// The driver's configuration for a fix interval and constellation set: its defaults (UBX only, NMEA off, one NAV-PVT
/// per epoch, dynamic model 8, static hold off) with the measurement period `ms`, the set `set` and the baud scan
/// starting at 115200, the target: after an FC reset with the receiver powered it already runs
/// there, and a scan that starts at 9600 or 38400 would read its 115200 stream at the wrong rate (a framing-error
/// `warn!` per byte in embassy-rp's UART interrupt). Then 9600 (the u-blox default) and 38400.
pub fn config(ms: u16, set: Constellations) -> Config {
    Config {
        meas_rate_ms: ms,
        constellations: set,
        candidate_bauds: [115_200, 9_600, 38_400],
        ..Config::default()
    }
}

/// UTC time of day of a NAV-PVT: hour, minute and second as sent, `millis` the fraction of the second (`nano`) in
/// whole milliseconds, a negative fraction (a few ns to us at the receiver's epoch alignment) taken as 0.
fn utc_time(pvt: &Pvt) -> UtcTime {
    UtcTime {
        hour: pvt.hour,
        minute: pvt.min,
        second: pvt.sec,
        millis: u16::try_from((pvt.nano.max(0) / 1_000_000).min(999)).unwrap_or(0),
    }
}

/// The `GnssReading` of one NAV-PVT, every field and its unit (decision-gnss-m10-20261122, D3):
///
/// | field | value |
/// |---|---|
/// | `fix_type` (MAVLink numbering) | 3 if [`Pvt::is_3d_fix_ok`] (`fixType` 3, `gnssFixOK`, not `invalidLlh`), else 1 |
/// | `lat_e7`, `lon_e7` (1e-7 deg) | `lat`, `lon`; `None` unless fix_type 3 |
/// | `alt_mm` (mm, above mean sea level) | `hMSL` (not `height`, the ellipsoid height); `None` unless fix_type 3 |
/// | `speed_mm_s` (mm/s) | `gSpeed` clamped to 0 or more; `None` unless fix_type 3 |
/// | `course_cdeg` (0.01 deg) | `headMot` (1e-5 deg) / 1000, normalised to 0..35999; `None` unless fix_type 3 |
/// | `sats_used` | `numSV`, whatever the fix (0 to 255) |
/// | `hdop_x100`, `vdop_x100` | `None`: NAV-PVT carries `pDOP` only, which is not either of them |
/// | `time` | UTC time of day when `valid` has validTime, `millis` from `nano` (`nano` below 0 is 0); else `None` |
/// | `date` | UTC date when `valid` has validDate; else `None` |
/// | `lat_std_mm`, `lon_std_mm` (mm, 1 sigma) | both `hAcc`, factor 1 (not hAcc / sqrt 2: as PX4 and ArduPilot); `None` unless fix_type 3 |
/// | `alt_std_mm` (mm, 1 sigma) | `vAcc`; `None` unless fix_type 3 |
/// | `vel_ned_mm_s` (mm/s) | `[velN, velE, velD]` (down positive); `None` unless fix_type 3 |
///
/// Without a 3D fix every position, velocity and accuracy field is `None`, whatever the receiver put in the message
/// (fixType 2, 4, gnssFixOK 0 and invalidLlh 1 all give fix_type 1).
pub fn reading(pvt: &Pvt) -> GnssReading {
    let fix = pvt.is_3d_fix_ok();
    GnssReading {
        fix_type: if fix { 3 } else { 1 },
        lat_e7: fix.then_some(pvt.lat),
        lon_e7: fix.then_some(pvt.lon),
        alt_mm: fix.then_some(pvt.h_msl),
        speed_mm_s: fix.then(|| u32::try_from(pvt.g_speed.max(0)).unwrap_or(0)),
        course_cdeg: fix.then(|| {
            u16::try_from((pvt.head_mot / 1000).rem_euclid(36_000)).unwrap_or(0)
        }),
        sats_used: Some(pvt.num_sv),
        hdop_x100: None,
        vdop_x100: None,
        time: (pvt.valid & Pvt::VALID_TIME != 0).then_some(utc_time(pvt)),
        date: (pvt.valid & Pvt::VALID_DATE != 0).then_some(UtcDate {
            year: pvt.year,
            month: pvt.month,
            day: pvt.day,
        }),
        lat_std_mm: fix.then_some(pvt.h_acc),
        lon_std_mm: fix.then_some(pvt.h_acc),
        alt_std_mm: fix.then_some(pvt.v_acc),
        vel_ned_mm_s: fix.then_some([pvt.vel_n, pvt.vel_e, pvt.vel_d]),
    }
}

/// The wait from a failed configuration to the next try, ms. Kept from the LC76G design (its bench figure: a receiver
/// just powered on did not complete a change for several seconds; 5000 is the figure of the start wait for the first
/// sentence). Not measured on the SAM-M10Q: the driver's configuration is itself up to 5.6 s (three 1 s baud probes,
/// 100 ms, the acknowledgement's 1 s, 500 ms), so the retry follows a failed one by 5 s more.
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

/// The retry schedule of the configuration: a failed configure is run again [`CONFIGURE_RETRY_MS`] after the
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

    /// The configuration that [`Retry::begin`] began ended at `now_ms`; `ok` when the driver's configure succeeded.
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

/// An edge older than this is not the one of a fix's second: 1 s, us.
pub const PPS_MAX_AGE_US: u64 = 1_000_000;

/// One fix paired with its PPS edge.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PpsPair {
    /// Board time of the PPS edge, us.
    pub edge_us: u64,
    /// Board time of the fix (the sample's `t_us`) minus `edge_us`, us.
    pub latency_us: i32,
    /// PPS edges seen so far.
    pub pulses: u32,
    /// Edges that met no whole-second fix plus whole-second fixes that met no fresh edge, so far.
    pub unpaired: u32,
}

/// Pairs the PPS edge of each second with the whole-second NAV-PVT fix (iTOW a multiple of 1000 ms, whose time of
/// validity is the edge) and keeps the latency figures. Measure only: nothing reads it back into the estimator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pps {
    /// The latest edge and whether a fix has used it.
    edge: Option<(u64, bool)>,
    /// Edges counted.
    pub pulses: u32,
    /// Fixes paired.
    pub paired: u32,
    /// Unpaired edges plus whole-second fixes without a fresh edge.
    pub unpaired: u32,
    /// The latest latency, us (0 before the first pairing).
    pub last_us: i32,
    /// The smallest latency, us.
    pub min_us: i32,
    /// The largest latency, us.
    pub max_us: i32,
}

impl Pps {
    /// No edge, nothing counted.
    pub const ZERO: Self = Self {
        edge: None,
        pulses: 0,
        paired: 0,
        unpaired: 0,
        last_us: 0,
        min_us: 0,
        max_us: 0,
    };

    /// A PPS edge at board time `t_us`; the previous edge, if no fix used it, is unpaired.
    pub fn edge(&mut self, t_us: u64) {
        if matches!(self.edge, Some((_, false))) {
            self.unpaired = self.unpaired.saturating_add(1);
        }
        self.edge = Some((t_us, false));
        self.pulses = self.pulses.saturating_add(1);
    }

    /// A fix with GPS time of week `tow_ms`, published at board time `t_us`. Nothing for a fix that is not a whole
    /// second; a whole-second fix pairs with the latest unused edge when it is less than 1 s old (and not later
    /// than the fix), else it counts as unpaired.
    pub fn fix(&mut self, tow_ms: u32, t_us: u64) -> Option<PpsPair> {
        if tow_ms % 1000 != 0 {
            return None;
        }
        let Some((edge_us, false)) = self.edge.filter(|&(e, _)| e <= t_us && t_us - e < PPS_MAX_AGE_US) else {
            self.unpaired = self.unpaired.saturating_add(1);
            return None;
        };
        self.edge = Some((edge_us, true));
        let latency_us = i32::try_from(t_us - edge_us).unwrap_or(i32::MAX);
        self.last_us = latency_us;
        self.min_us = if self.paired == 0 { latency_us } else { self.min_us.min(latency_us) };
        self.max_us = if self.paired == 0 { latency_us } else { self.max_us.max(latency_us) };
        self.paired = self.paired.saturating_add(1);
        Some(PpsPair {
            edge_us,
            latency_us,
            pulses: self.pulses,
            unpaired: self.unpaired,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamStore, ParamValue};

    const ALLOWED: [u16; 4] = [100, 200, 500, 1000];
    const SETS: [Constellations; 3] = [
        Constellations::GpsGalBdsQzss,
        Constellations::GpsGal,
        Constellations::GpsGalGlo,
    ];
    /// Index of `GPS_RATE_MS` and `GPS_GNSS` in the firmware's table.
    const RATE_AT: usize = 80;
    const SET_AT: usize = crate::COUNT - 1 - crate::flight::COUNT - crate::batt::FS_COUNT;

    fn store() -> Params<{ crate::COUNT }> {
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        assert!(p.set_validator(DEVICE, validate));
        p
    }

    #[test]
    fn config_is_the_driver_default_with_the_stored_period_set_and_the_scan_from_115200() {
        for set in SETS {
            for ms in ALLOWED {
                let c = config(ms, set);
                assert!(c.is_valid(), "{ms}");
                assert_eq!(c.meas_rate_ms, ms);
                assert_eq!(c.constellations, set);
                assert_eq!(c.candidate_bauds, [115_200, 9_600, 38_400]);
                assert_eq!(
                    c,
                    Config {
                        meas_rate_ms: ms,
                        constellations: set,
                        candidate_bauds: c.candidate_bauds,
                        ..Config::default()
                    }
                );
                assert!(c.valset().is_some(), "{ms}");
            }
        }
        assert!(!config(300, Constellations::GpsGal).is_valid());
    }

    #[test]
    fn the_parameters_and_their_default() {
        let def = &TABLE[0];
        assert_eq!((def.name, def.device, def.default), ("GPS_RATE_MS", 5, ParamValue::I32(200)));
        assert_eq!((def.min, def.max), (ParamValue::I32(100), ParamValue::I32(1000)));
        assert_eq!(def.apply, Apply::Configure);
        let def = &SET_TABLE[0];
        assert_eq!((def.name, def.device, def.default), ("GPS_GNSS", 5, ParamValue::I32(0)));
        assert_eq!((def.min, def.max), (ParamValue::I32(0), ParamValue::I32(2)));
        assert_eq!(def.apply, Apply::Configure);
        assert_eq!(crate::TABLE[SET_AT].name, "GPS_GNSS");
        assert_eq!(crate::TABLE[RATE_AT].name, "GPS_RATE_MS");
        // The defaults configure the driver's default: 200 ms, GPS + Galileo + BeiDou B1C + QZSS.
        assert_eq!(rate_and_set(Some(200), Some(0)), Ok((200, Constellations::GpsGalBdsQzss)));
        assert_eq!(config(200, Constellations::GpsGalBdsQzss).constellations, Config::default().constellations);
    }

    /// Every set against every period: only 100 ms with GPS_GNSS 1 is allowed among the periods the limits cut.
    #[test]
    fn the_validator_matrix_set_by_rate() {
        const NAMES: [&str; 3] = ["GPS_GNSS 0", "GPS_GNSS 1", "GPS_GNSS 2"];
        for (k, set) in SETS.iter().enumerate() {
            for ms in ALLOWED {
                let mut p = store();
                // The set first (every set is valid with the default 200 ms), then the period.
                assert_eq!(p.set(SET_AT as u16, ParamValue::I32(k as i32)), Ok(ParamValue::I32(k as i32)));
                let r = p.set(RATE_AT as u16, ParamValue::I32(ms as i32));
                if ms >= set.min_meas_rate_ms() {
                    assert_eq!(r, Ok(ParamValue::I32(ms as i32)), "{k} {ms}");
                } else {
                    assert_eq!((ms, r), (100, Err(ParamError::Rejected)), "{k} {ms}");
                    // The refusal names both values.
                    let text = p.take_notice().unwrap().kind.text();
                    assert!(text.contains("100") && text.contains(NAMES[k]), "{text}");
                    assert!("GPS_RATE_MS: ".len() + text.len() <= 50);
                }
            }
        }
        // 100 ms with GPS_GNSS 1 is accepted, and GPS_GNSS cannot then go to 0 or 2 (the whole candidate).
        let mut p = store();
        assert_eq!(p.set(SET_AT as u16, ParamValue::I32(1)), Ok(ParamValue::I32(1)));
        assert_eq!(p.set(RATE_AT as u16, ParamValue::I32(100)), Ok(ParamValue::I32(100)));
        for g in [0, 2] {
            assert_eq!(p.set(SET_AT as u16, ParamValue::I32(g)), Err(ParamError::Rejected));
            assert!(p.take_notice().unwrap().kind.text().ends_with(["GPS_GNSS 0", "GPS_GNSS 1", "GPS_GNSS 2"][g as usize]));
        }
        assert_eq!(p.set(SET_AT as u16, ParamValue::I32(1)), Ok(ParamValue::I32(1)));
        // Other periods and out-of-range values.
        assert_eq!(p.set(RATE_AT as u16, ParamValue::I32(300)), Err(ParamError::Rejected));
        assert_eq!(p.take_notice().unwrap().kind.text(), "not 100, 200, 500 or 1000");
        assert!("GPS_RATE_MS: ".len() + BAD_RATE.len() <= 50);
        assert_eq!(p.set(RATE_AT as u16, ParamValue::I32(50)), Err(ParamError::OutOfRange));
        assert_eq!(p.set(SET_AT as u16, ParamValue::I32(3)), Err(ParamError::OutOfRange));
        assert_eq!(p.set(SET_AT as u16, ParamValue::I32(-1)), Err(ParamError::OutOfRange));
    }

    /// `rate_ms` accepts exactly the periods the validator accepts with some set, over the whole range and beyond it.
    #[test]
    fn rate_ms_agrees_with_the_validator() {
        for v in 100..=1000 {
            let mut p = store();
            // GPS_GNSS 1 allows every period.
            assert!(p.set(SET_AT as u16, ParamValue::I32(1)).is_ok());
            let accepted = p.set(RATE_AT as u16, ParamValue::I32(v)).is_ok();
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

    /// The boot sanitiser: an unknown or missing `GPS_GNSS` is the default; a stored pair that breaks a limit is refused.
    #[test]
    fn the_boot_sanitiser() {
        assert_eq!(constellations(Some(0)), Constellations::GpsGalBdsQzss);
        assert_eq!(constellations(Some(1)), Constellations::GpsGal);
        assert_eq!(constellations(Some(2)), Constellations::GpsGalGlo);
        for v in [None, Some(3), Some(-1), Some(i32::MAX)] {
            assert_eq!(constellations(v), Constellations::GpsGalBdsQzss, "{v:?}");
        }
        assert_eq!(rate_and_set(Some(100), Some(1)), Ok((100, Constellations::GpsGal)));
        assert!(rate_and_set(Some(100), Some(0)).is_err());
        assert!(rate_and_set(Some(100), Some(2)).is_err());
        assert!(rate_and_set(Some(100), None).is_err());
        assert_eq!(rate_and_set(Some(200), Some(7)), Ok((200, Constellations::GpsGalBdsQzss)));
        assert_eq!(rate_and_set(Some(300), Some(1)), Err(BAD_RATE));
        assert_eq!(rate_and_set(None, Some(1)), Err(BAD_RATE));
    }

    /// A record saved before `GPS_GNSS` existed (every parameter but the last) loads with `GPS_GNSS` at its default.
    #[test]
    fn an_old_record_without_gps_gnss_loads_with_the_default() {
        use fw_core::record::{self, Boot, Entry, SECTOR_LEN};
        use mavlink_device::ParamName;
        let old = &crate::TABLE[..SET_AT];
        let entries = old.iter().map(|d| Entry {
            name: ParamName::new(d.name).unwrap(),
            value: d.default,
        });
        let mut a = [0u8; SECTOR_LEN];
        let b = [0xFFu8; SECTOR_LEN];
        record::encode(7, entries, &mut a).unwrap();
        let Boot::Current { record, .. } = record::choose(&a, &b).boot else {
            panic!("the old record is not current");
        };
        assert_eq!(record.declared_count(), SET_AT);
        let mut p = store();
        assert_eq!(p.set(SET_AT as u16, ParamValue::I32(2)), Ok(ParamValue::I32(2)));
        p.reset_all();
        for entry in record.entries() {
            assert_eq!(p.load(&entry.name, entry.value), Ok(true));
        }
        assert_eq!(p.get_named(SET), Some(ParamValue::I32(0)));
        let get = |n| match p.get_named(n) {
            Some(ParamValue::I32(v)) => Some(v),
            _ => None,
        };
        assert_eq!(
            rate_and_set(get(RATE_MS), get(SET)),
            Ok((200, Constellations::GpsGalBdsQzss))
        );
    }

    /// A NAV-PVT of a good 3D fix with distinct values in every field the mapping reads.
    fn pvt() -> Pvt {
        Pvt {
            itow: 123_456_789,
            year: 2026,
            month: 11,
            day: 22,
            hour: 14,
            min: 5,
            sec: 9,
            valid: Pvt::VALID_DATE | Pvt::VALID_TIME | Pvt::FULLY_RESOLVED,
            t_acc: 30,
            nano: 200_999_999,
            fix_type: 3,
            flags: Pvt::GNSS_FIX_OK,
            flags2: 0,
            num_sv: 17,
            lon: -987_654_321,
            lat: 301_234_567,
            height: 250_000,
            h_msl: 212_345,
            h_acc: 1_800,
            v_acc: 2_700,
            vel_n: 1_500,
            vel_e: -2_500,
            vel_d: 350,
            g_speed: 2_914,
            head_mot: 31_500_000,
            s_acc: 80,
            head_acc: 100_000,
            p_dop: 120,
            flags3: 0,
        }
    }

    #[test]
    fn a_valid_3d_fix_maps_every_field() {
        assert_eq!(
            reading(&pvt()),
            GnssReading {
                fix_type: 3,
                lat_e7: Some(301_234_567),
                lon_e7: Some(-987_654_321),
                // hMSL, not the ellipsoid height 250_000.
                alt_mm: Some(212_345),
                speed_mm_s: Some(2_914),
                // 31_500_000 x 1e-5 deg = 315.00 deg.
                course_cdeg: Some(31_500),
                sats_used: Some(17),
                hdop_x100: None,
                vdop_x100: None,
                time: Some(UtcTime { hour: 14, minute: 5, second: 9, millis: 200 }),
                date: Some(UtcDate { year: 2026, month: 11, day: 22 }),
                lat_std_mm: Some(1_800),
                lon_std_mm: Some(1_800),
                alt_std_mm: Some(2_700),
                vel_ned_mm_s: Some([1_500, -2_500, 350]),
            }
        );
    }

    /// Every way a NAV-PVT can fail to be a usable 3D fix gives fix_type 1 and nothing of position, velocity or accuracy.
    #[test]
    fn a_pvt_that_is_not_a_usable_3d_fix_has_no_position_velocity_or_accuracy() {
        type Edit = fn(&mut Pvt);
        let controls: [(&str, Edit); 6] = [
            ("fixType 2", |p| p.fix_type = 2),
            ("fixType 4", |p| p.fix_type = 4),
            ("fixType 0", |p| p.fix_type = 0),
            ("gnssFixOK 0", |p| p.flags &= !Pvt::GNSS_FIX_OK),
            ("invalidLlh 1", |p| p.flags3 |= Pvt::INVALID_LLH),
            ("fixType 5 (time only)", |p| p.fix_type = 5),
        ];
        for (name, edit) in controls {
            let mut p = pvt();
            edit(&mut p);
            let r = reading(&p);
            assert_eq!(r.fix_type, 1, "{name}");
            assert_eq!(
                (r.lat_e7, r.lon_e7, r.alt_mm, r.speed_mm_s, r.course_cdeg),
                (None, None, None, None, None),
                "{name}"
            );
            assert_eq!(
                (r.lat_std_mm, r.lon_std_mm, r.alt_std_mm, r.vel_ned_mm_s),
                (None, None, None, None),
                "{name}"
            );
            // What does not depend on the fix is still reported.
            assert_eq!(r.sats_used, Some(17), "{name}");
            assert!(r.time.is_some() && r.date.is_some(), "{name}");
            assert_eq!((r.hdop_x100, r.vdop_x100), (None, None), "{name}");
        }
    }

    #[test]
    fn time_and_date_follow_their_own_valid_bits() {
        let mut p = pvt();
        p.valid = Pvt::VALID_DATE;
        let r = reading(&p);
        assert_eq!((r.time, r.date.is_some()), (None, true));
        p.valid = Pvt::VALID_TIME;
        let r = reading(&p);
        assert_eq!((r.time.is_some(), r.date), (true, None));
        p.valid = 0;
        let r = reading(&p);
        assert_eq!((r.time, r.date), (None, None));
        // A negative fraction of a second is 0 ms, one just under a second 999 ms.
        p.valid = Pvt::VALID_TIME;
        p.nano = -1_234;
        assert_eq!(reading(&p).time.map(|t| t.millis), Some(0));
        p.nano = 999_999_999;
        assert_eq!(reading(&p).time.map(|t| t.millis), Some(999));
    }

    #[test]
    fn speed_is_clamped_and_the_course_is_normalised() {
        let mut p = pvt();
        p.g_speed = -5;
        assert_eq!(reading(&p).speed_mm_s, Some(0));
        p.g_speed = i32::MAX;
        assert_eq!(reading(&p).speed_mm_s, Some(i32::MAX as u32));
        for (head_mot, cdeg) in [
            (0, 0),
            (1_000, 1),
            (999, 0),
            (35_999_000, 35_999),
            (36_000_000, 0),
            (-1_000, 35_999),
            (-36_000_000, 0),
            (72_000_000 + 5_000, 5),
            (i32::MIN, (i32::MIN / 1000).rem_euclid(36_000) as u16),
            (i32::MAX, (i32::MAX / 1000).rem_euclid(36_000) as u16),
        ] {
            p.head_mot = head_mot;
            assert_eq!(reading(&p).course_cdeg, Some(cdeg), "{head_mot}");
            assert!(cdeg < 36_000);
        }
    }


    #[test]
    fn pps_pairs_the_whole_second_fix_with_its_fresh_edge() {
        let mut p = Pps::ZERO;
        // Fixes before any edge: whole second is unpaired, others are ignored.
        assert_eq!(p.fix(200, 10), None);
        assert_eq!(p.unpaired, 0);
        assert_eq!(p.fix(1000, 50_000), None);
        assert_eq!(p.unpaired, 1);
        p.edge(1_000_000);
        // A 200 ms epoch is not paired and does not use the edge.
        assert_eq!(p.fix(1200, 1_250_000), None);
        assert_eq!(p.unpaired, 1);
        let pair = p.fix(2000, 1_060_000).unwrap();
        assert_eq!((pair.edge_us, pair.latency_us, pair.pulses, pair.unpaired), (1_000_000, 60_000, 1, 1));
        // The edge is used once.
        assert_eq!(p.fix(3000, 1_070_000), None);
        assert_eq!(p.unpaired, 2);
        // An edge 1 s old or older is not paired (the fix is whole-second but the edge was missed).
        p.edge(2_000_000);
        assert_eq!(p.fix(4000, 3_000_000), None);
        assert_eq!(p.unpaired, 3);
        // An edge later than the fix is not paired.
        p.edge(5_000_000);
        // (the unused edge at 2 s is replaced: unpaired too)
        assert_eq!(p.fix(5000, 4_999_999), None);
        assert_eq!(p.unpaired, 5);
        // Figures: 40 ms and 90 ms after 60 ms.
        p.edge(6_000_000);
        assert_eq!(p.fix(6000, 6_040_000).unwrap().latency_us, 40_000);
        p.edge(7_000_000);
        assert_eq!(p.fix(7000, 7_090_000).unwrap().latency_us, 90_000);
        assert_eq!((p.paired, p.last_us, p.min_us, p.max_us), (3, 90_000, 40_000, 90_000));
        // An edge replaced before a fix used it is unpaired.
        let before = p.unpaired;
        p.edge(8_000_000);
        p.edge(9_000_000);
        assert_eq!((p.unpaired, p.pulses), (before + 1, 7));
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
}
