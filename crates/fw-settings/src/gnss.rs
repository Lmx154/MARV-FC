//! The GNSS receiver's parameter, its configuration and the mapping of its NAV-PVT to a `GnssReading`
//! (decision-gnss-m10-20261122, M1 architect ruling D2 and D3).
//!
//! The receiver is the u-blox SAM-M10Q, UBX only. The `sam-m10q` driver sends it one RAM-only configuration
//! (nothing is saved): this module only picks the measurement period from `GPS_RATE_MS`. Nothing here
//! touches hardware.

use fw_core::params::{Apply, Candidate, ParamDef};
use fw_core::sample::{GnssReading, UtcDate, UtcTime};
use mavlink_device::marv::MarvDevice;
use sam_m10q::{Config, Pvt};

const DEVICE: u8 = MarvDevice::MARV_DEVICE_GNSS as u8;

/// The position fix interval in milliseconds: one of 200, 500 or 1000 (the periods the SAM-M10Q accepts with the
/// firmware's constellation set, `sam_m10q::config::MEAS_RATES_MS`).
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

const BAD_RATE: &str = "not 200, 500 or 1000";

/// The store's validator for the GNSS receiver: 200, 500 or 1000 ms.
pub fn validate(candidate: &Candidate<'_>) -> Result<(), &'static str> {
    match candidate.i32(RATE_MS) {
        Some(200 | 500 | 1000) => Ok(()),
        _ => Err(BAD_RATE),
    }
}

/// The stored `GPS_RATE_MS` as the fix interval to configure: the value when [`validate`] accepts it,
/// otherwise its refusal. A boot load runs no validator, so the adapter asks this before it configures.
pub fn rate_ms(value: Option<i32>) -> Result<u16, &'static str> {
    match value {
        Some(v @ (200 | 500 | 1000)) => u16::try_from(v).map_err(|_| BAD_RATE),
        _ => Err(BAD_RATE),
    }
}

/// The driver's configuration for a fix interval: its defaults (UBX only, NMEA off, one NAV-PVT per epoch, dynamic
/// model 8, static hold off, the constellation set of `sam_m10q::config::SIGNAL_KEYS`) with the measurement period
/// `ms` and the baud scan starting at 115200, the target: after an FC reset with the receiver powered it already runs
/// there, and a scan that starts at 9600 or 38400 would read its 115200 stream at the wrong rate (a framing-error
/// `warn!` per byte in embassy-rp's UART interrupt). Then 9600 (the u-blox default) and 38400.
pub fn config(ms: u16) -> Config {
    Config {
        meas_rate_ms: ms,
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

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamStore, ParamValue};

    const ALLOWED: [u16; 3] = [200, 500, 1000];

    #[test]
    fn config_is_the_driver_default_with_the_stored_period_and_the_scan_from_115200_for_every_allowed_rate() {
        for ms in ALLOWED {
            let c = config(ms);
            assert!(c.is_valid(), "{ms}");
            assert_eq!(c.meas_rate_ms, ms);
            assert_eq!(c.candidate_bauds, [115_200, 9_600, 38_400]);
            assert_eq!(c, Config { meas_rate_ms: ms, candidate_bauds: c.candidate_bauds, ..Config::default() });
            assert!(c.valset().is_some(), "{ms}");
        }
        // 100 ms is not a period the receiver accepts with this constellation set.
        assert!(!config(100).is_valid());
    }

    #[test]
    fn the_validator_accepts_the_three_rates_and_refuses_the_rest() {
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
        for ok in [200, 500, 1000] {
            assert_eq!(p.set(80, ParamValue::I32(ok)), Ok(ParamValue::I32(ok)));
        }
        // 100 was allowed with the LC76G; the SAM-M10Q refuses it.
        assert_eq!(p.set(80, ParamValue::I32(100)), Err(ParamError::Rejected));
        assert_eq!(p.set(80, ParamValue::I32(300)), Err(ParamError::Rejected));
        assert_eq!(
            p.take_notice().unwrap().kind.text(),
            "not 200, 500 or 1000"
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
        assert_eq!(rate_ms(Some(100)), Err(BAD_RATE));
        assert_eq!(rate_ms(Some(300)), Err(BAD_RATE));
        for v in [None, Some(0), Some(-200), Some(65_736), Some(i32::MAX)] {
            assert_eq!(rate_ms(v), Err(BAD_RATE), "{v:?}");
        }
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
