//! The slow sensors' samples as the flight code takes them: barometer, compass, GNSS
//! (decision-sim-seams-20261119, section 2). Units are the firmware's (conventions 3c).

/// Barometer: pressure, Pa; die temperature, deg C.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroReading {
    /// Pa.
    pub pressure_pa: f32,
    /// deg C.
    pub temperature_c: f32,
}

/// Compass: field, microtesla, body FRD, `MAG_ORIENT` applied and `MAG_OFF*` removed; die temperature, deg C.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagReading {
    /// Microtesla, body FRD.
    pub field_ut: [f32; 3],
    /// deg C.
    pub temperature_c: f32,
}

/// One receiver epoch in its integer units (today's `Reading::Gnss`, field for field). Position, speed and course
/// are `None` without a fix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssReading {
    /// MAVLink GPS_FIX_TYPE: 1 no fix, 2 2D, 3 3D, 4 DGPS/SBAS.
    pub fix_type: u8,
    /// 1e-7 degree, south and west negative.
    pub lat_e7: Option<i32>,
    /// 1e-7 degree, south and west negative.
    pub lon_e7: Option<i32>,
    /// Above mean sea level, mm.
    pub alt_mm: Option<i32>,
    /// Ground speed, mm/s; course over ground, 0.01 degree clockwise from true north.
    pub speed_mm_s: Option<u32>,
    /// Course over ground, 0.01 degree clockwise from true north.
    pub course_cdeg: Option<u16>,
    /// Satellites used.
    pub sats_used: Option<u8>,
    /// Dilution of precision x 100.
    pub hdop_x100: Option<u16>,
    /// Vertical dilution of precision x 100.
    pub vdop_x100: Option<u16>,
    /// UTC of the epoch when the receiver has it (the log's GPS week); flight code ignores it.
    pub time: Option<UtcTime>,
    /// UTC date of the epoch when the receiver has it.
    pub date: Option<UtcDate>,
    /// Standard deviation of the latitude error, mm, 1 sigma; `None` when not available for this epoch.
    pub lat_std_mm: Option<u32>,
    /// Standard deviation of the longitude error, mm, 1 sigma; `None` when not available for this epoch.
    pub lon_std_mm: Option<u32>,
    /// Standard deviation of the altitude error, mm, 1 sigma; `None` when not available for this epoch.
    pub alt_std_mm: Option<u32>,
}

/// The deviations to attach to a fix: the accuracy sentence's `[lat, lon, alt]` (mm) when its UTC time
/// equals the fix's, otherwise all `None`. A deviation is never carried to another epoch (G-I7): a
/// missing time on either side, or a different time, gives no deviation at all.
pub fn same_epoch_sigmas(
    fix_time: Option<UtcTime>,
    gst_time: Option<UtcTime>,
    gst_std_mm: [Option<u32>; 3],
) -> [Option<u32>; 3] {
    match (fix_time, gst_time) {
        (Some(fix), Some(gst)) if fix == gst => gst_std_mm,
        _ => [None; 3],
    }
}

/// What to do with a fix whose GGA + RMC pair just completed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Publish {
    /// Publish now with these deviations `[lat, lon, alt]` (all `None`: without).
    Now([Option<u32>; 3]),
    /// Wait for the next sentence ([`GstOrder::sentence`]).
    Wait,
}

/// The order of GST and the GGA + RMC pair, learnt from the stream (decision Amendment C-c, ruling 1). A fix
/// waits at most one sentence, and only once a GST of a fix's own epoch was seen right after the pair.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GstOrder {
    /// "GST follows the pair".
    follows: bool,
    /// A fix waits for the next sentence: its UTC time.
    waiting: Option<Option<UtcTime>>,
    /// A fix just went out without deviations and without waiting: its UTC time.
    unwaited: Option<UtcTime>,
}

impl GstOrder {
    /// Forget the order: at start, after every configuration (success or failure) and after a flush.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Whether the adapter waits the one sentence after the pair.
    pub fn follows(&self) -> bool {
        self.follows
    }

    /// The pair of a fix at `fix_time` completed; `kept` is the latest GST (its UTC time and deviations).
    /// Its own epoch's GST: now with it; otherwise now without unless GST follows the pair, then wait.
    pub fn pair(
        &mut self,
        fix_time: Option<UtcTime>,
        kept: Option<(Option<UtcTime>, [Option<u32>; 3])>,
    ) -> Publish {
        self.unwaited = None;
        if let Some((gst_time, std_mm)) = kept
            && fix_time.is_some()
            && gst_time == fix_time
        {
            return Publish::Now(same_epoch_sigmas(fix_time, gst_time, std_mm));
        }
        if self.follows {
            self.waiting = Some(fix_time);
            Publish::Wait
        } else {
            self.unwaited = fix_time;
            Publish::Now([None; 3])
        }
    }

    /// The next sentence read (`gst`: its UTC time and deviations when it is a GST). While a fix waits,
    /// returns its deviations (`None` if this is not its GST, which also clears "GST follows the pair").
    /// Right after a fix went out without waiting, its own GST here sets "GST follows the pair"; those
    /// deviations go to nothing (G-I7).
    pub fn sentence(
        &mut self,
        gst: Option<(Option<UtcTime>, [Option<u32>; 3])>,
    ) -> Option<[Option<u32>; 3]> {
        let unwaited = self.unwaited.take();
        if let Some(fix_time) = self.waiting.take() {
            let std_mm = match gst {
                Some((gst_time, std_mm)) if fix_time.is_some() && gst_time == fix_time => {
                    same_epoch_sigmas(fix_time, gst_time, std_mm)
                }
                _ => {
                    self.follows = false;
                    [None; 3]
                }
            };
            return Some(std_mm);
        }
        if let (Some(fix_time), Some((Some(gst_time), _))) = (unwaited, gst)
            && gst_time == fix_time
        {
            self.follows = true;
        }
        None
    }
}

/// The horizontal deviation sqrt(lat^2 + lon^2), mm (rounded down; `u32::MAX` beyond), when both are
/// known: GPS_RAW_INT's `h_acc`.
pub fn horizontal_std_mm(lat_std_mm: Option<u32>, lon_std_mm: Option<u32>) -> Option<u32> {
    let (lat, lon) = (u64::from(lat_std_mm?), u64::from(lon_std_mm?));
    Some(u32::try_from((lat * lat).saturating_add(lon * lon).isqrt()).unwrap_or(u32::MAX))
}

/// Time of day, UTC: the fields of `lc76g::sentence::Time`, copied one for one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UtcTime {
    /// 0 to 23.
    pub hour: u8,
    /// 0 to 59.
    pub minute: u8,
    /// 0 to 60 (leap second).
    pub second: u8,
    /// 0 to 999. Finer fractions are truncated.
    pub millis: u16,
}

/// Calendar date, UTC: the fields of `lc76g::sentence::Date`, copied one for one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UtcDate {
    /// Full year, 2000 + the two-digit year of the sentence.
    pub year: u16,
    /// 1 to 12.
    pub month: u8,
    /// 1 to 31.
    pub day: u8,
}

/// A reading, the board time its read began (us), +1 per publish: the shape of `fw_flight::imu::ImuSample`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroSample {
    /// The values.
    pub reading: BaroReading,
    /// Board time when the read began, us.
    pub t_us: u64,
    /// +1 per publish.
    pub seq: u32,
}

/// A compass reading, the board time its read began (us), +1 per publish.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagSample {
    /// The values.
    pub reading: MagReading,
    /// Board time when the read began, us.
    pub t_us: u64,
    /// +1 per publish.
    pub seq: u32,
}

/// A GNSS epoch, the board time its read began (us), +1 per publish.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssSample {
    /// The values.
    pub reading: GnssReading,
    /// Board time when the read began, us.
    pub t_us: u64,
    /// +1 per publish.
    pub seq: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(second: u8, millis: u16) -> Option<UtcTime> {
        Some(UtcTime {
            hour: 12,
            minute: 34,
            second,
            millis,
        })
    }

    const SIGMAS: [Option<u32>; 3] = [Some(1200), Some(900), Some(2500)];

    #[test]
    fn a_sigma_is_attached_only_in_its_own_epoch() {
        assert_eq!(same_epoch_sigmas(t(5, 200), t(5, 200), SIGMAS), SIGMAS);
        // Another epoch, even the very next one or a different millisecond: nothing.
        assert_eq!(same_epoch_sigmas(t(5, 400), t(5, 200), SIGMAS), [None; 3]);
        assert_eq!(same_epoch_sigmas(t(6, 200), t(5, 200), SIGMAS), [None; 3]);
        // A missing time on either side: nothing.
        assert_eq!(same_epoch_sigmas(None, t(5, 200), SIGMAS), [None; 3]);
        assert_eq!(same_epoch_sigmas(t(5, 200), None, SIGMAS), [None; 3]);
        assert_eq!(same_epoch_sigmas(None, None, SIGMAS), [None; 3]);
    }

    #[test]
    fn partial_sigmas_pass_through_when_the_times_match() {
        let part = [Some(1200), Some(900), None];
        assert_eq!(same_epoch_sigmas(t(5, 200), t(5, 200), part), part);
        assert_eq!(same_epoch_sigmas(t(5, 200), t(5, 200), [None; 3]), [None; 3]);
    }

    #[test]
    fn gst_order_publishes_with_its_own_epochs_gst_and_never_waits_unlearnt() {
        let mut o = GstOrder::default();
        // A kept GST of the fix's epoch: now, with it.
        assert_eq!(
            o.pair(t(5, 200), Some((t(5, 200), SIGMAS))),
            Publish::Now(SIGMAS)
        );
        // Flag false (start): now, without, whatever was kept.
        assert_eq!(
            o.pair(t(5, 400), Some((t(5, 200), SIGMAS))),
            Publish::Now([None; 3])
        );
        assert_eq!(o.pair(t(5, 600), None), Publish::Now([None; 3]));
        // A non-GST or another epoch's GST right after does not teach the order.
        assert_eq!(o.sentence(None), None);
        o.pair(t(5, 800), None);
        assert_eq!(o.sentence(Some((t(5, 600), SIGMAS))), None);
        assert!(!o.follows());
        // A fix without a time teaches nothing either.
        o.pair(None, None);
        assert_eq!(o.sentence(Some((None, SIGMAS))), None);
        assert!(!o.follows());
    }

    #[test]
    fn gst_order_learns_from_a_late_gst_then_waits_one_sentence() {
        let mut o = GstOrder::default();
        assert_eq!(o.pair(t(5, 200), None), Publish::Now([None; 3]));
        // Its own GST right after: learnt; its deviations go to nothing (returned None).
        assert_eq!(o.sentence(Some((t(5, 200), SIGMAS))), None);
        assert!(o.follows());
        // Next epoch: wait, then its GST.
        assert_eq!(o.pair(t(5, 400), Some((t(5, 200), SIGMAS))), Publish::Wait);
        assert_eq!(o.sentence(Some((t(5, 400), SIGMAS))), Some(SIGMAS));
        assert!(o.follows());
        // A kept GST of the epoch still goes at once while the flag is set.
        assert_eq!(
            o.pair(t(5, 600), Some((t(5, 600), SIGMAS))),
            Publish::Now(SIGMAS)
        );
        // Only the next sentence counts: a learn needs it to come at once.
        assert_eq!(o.sentence(None), None);
        assert!(o.follows());
    }

    #[test]
    fn gst_order_flag_is_cleared_by_a_wrong_sentence_and_by_reset() {
        let mut learnt = GstOrder::default();
        learnt.pair(t(5, 200), None);
        learnt.sentence(Some((t(5, 200), SIGMAS)));
        assert!(learnt.follows());
        // A non-GST during a wait: without, flag cleared, the next fix goes at once.
        let mut o = learnt;
        assert_eq!(o.pair(t(5, 400), None), Publish::Wait);
        assert_eq!(o.sentence(None), Some([None; 3]));
        assert!(!o.follows());
        assert_eq!(o.pair(t(5, 600), None), Publish::Now([None; 3]));
        // Another epoch's GST during a wait: the same.
        let mut o = learnt;
        assert_eq!(o.pair(t(5, 400), None), Publish::Wait);
        assert_eq!(o.sentence(Some((t(5, 200), SIGMAS))), Some([None; 3]));
        assert!(!o.follows());
        // Configuration or flush: forgotten, nothing waits.
        let mut o = learnt;
        assert_eq!(o.pair(t(5, 400), None), Publish::Wait);
        o.reset();
        assert_eq!(o, GstOrder::default());
        assert_eq!(o.sentence(Some((t(5, 400), SIGMAS))), None);
        assert_eq!(o.pair(t(5, 600), None), Publish::Now([None; 3]));
    }

    #[test]
    fn horizontal_std_needs_both_and_does_not_overflow() {
        assert_eq!(horizontal_std_mm(Some(3000), Some(4000)), Some(5000));
        assert_eq!(horizontal_std_mm(Some(1), Some(1)), Some(1));
        assert_eq!(horizontal_std_mm(Some(0), Some(0)), Some(0));
        assert_eq!(horizontal_std_mm(Some(3000), None), None);
        assert_eq!(horizontal_std_mm(None, Some(4000)), None);
        assert_eq!(horizontal_std_mm(None, None), None);
        assert_eq!(horizontal_std_mm(Some(u32::MAX), Some(0)), Some(u32::MAX));
        assert_eq!(
            horizontal_std_mm(Some(u32::MAX), Some(u32::MAX)),
            Some(u32::MAX)
        );
    }
}
