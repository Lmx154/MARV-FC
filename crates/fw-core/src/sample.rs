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

/// Compass: the calibrated field m = S (r - o), microtesla, body FRD (`MAG_ORIENT` applied, then the offsets and the
/// soft-iron matrix, `fw_settings::mag::apply`); die temperature, deg C.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagReading {
    /// Microtesla, body FRD, calibrated.
    pub field_ut: [f32; 3],
    /// deg C.
    pub temperature_c: f32,
}

/// What the compass should read where this vehicle was calibrated, for the flight core's health test (decision-nav-eskf-20261120,
/// Amendment C4). Units are converted once, where it is filled: the strength in microtesla, the angles in radians, the
/// tolerance of the strength as a fraction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagExpected {
    /// Expected field strength, microtesla (`MAG_FLD_UT`, > 0).
    pub strength_ut: f32,
    /// Expected inclination, radians, positive down (`MAG_FLD_INC`, degrees stored).
    pub inclination_rad: f32,
    /// Strength tolerance as a fraction of `strength_ut` (`MAG_HLT_STR`, percent stored).
    pub tol_strength_frac: f32,
    /// Inclination tolerance, radians (`MAG_HLT_INC`, degrees stored).
    pub tol_inclination_rad: f32,
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
    /// Velocity north, east and down in mm/s (NED) from the receiver's `$PQTMPVT` of this epoch ([`EpochAssembly`]);
    /// `None` when no PVT of the epoch came in its burst, a velocity field was empty, or there is no position. Logged
    /// (GPV); nothing in the flight step reads it yet. The sign of "down" from the receiver is not yet verified
    /// (decision-nav-eskf-20261120, C2g).
    pub vel_ned_mm_s: Option<[i32; 3]>,
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

/// The order of the receiver's `$PQTMPVT` and the GGA + RMC pair, learnt from the stream as [`GstOrder`] learns
/// GST's (decision-nav-eskf-20261120, C2f). The burst of an epoch is the sentences from its pair up to the first
/// one that is neither its GST nor its PVT. A fix waits for its PVT only once a PVT of a fix's own epoch was seen
/// in the burst of an earlier pair after that fix went out; it waits through its own GST, never past the end of
/// the burst, and a burst that ends without its PVT forgets the order. A PVT of another time is ignored.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PvtOrder {
    /// "PVT follows the pair".
    follows: bool,
    /// The fix of [`PvtOrder::epoch`] waits for its PVT.
    waiting: bool,
    /// The UTC time of the latest pair while its burst runs.
    epoch: Option<UtcTime>,
}

/// What [`PvtOrder::sentence`] made of one sentence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PvtStep {
    /// Nothing for the PVT (a fix that waits goes on waiting).
    Nothing,
    /// The PVT of the waiting fix's epoch: the fix goes now with it.
    Paired,
    /// The burst ended without the PVT: the waiting fix goes now without; `ignored` when this sentence is a PVT of
    /// another time.
    Without {
        /// The sentence is a PVT of another time.
        ignored: bool,
    },
    /// The PVT of the epoch whose fix already went out without waiting: teaches "PVT follows the pair".
    Late,
    /// A PVT of another time, no fix waiting.
    Ignored,
}

impl PvtOrder {
    /// Whether the fix waits for its PVT.
    pub fn follows(&self) -> bool {
        self.follows
    }

    /// Whether `pvt_time` (the time of a PVT) is the time of the epoch whose burst runs.
    pub fn of_epoch(&self, pvt_time: Option<Option<UtcTime>>) -> bool {
        matches!(pvt_time, Some(Some(t)) if self.epoch == Some(t))
    }

    /// The pair of a fix at `fix_time` completed: whether the fix waits for its PVT (never without a time).
    pub fn pair(&mut self, fix_time: Option<UtcTime>) -> bool {
        self.epoch = fix_time;
        self.waiting = fix_time.is_some() && self.follows;
        self.waiting
    }

    /// The next sentence read: `gst_time` is the UTC time of a GST, `pvt_time` that of a PVT.
    pub fn sentence(
        &mut self,
        gst_time: Option<Option<UtcTime>>,
        pvt_time: Option<Option<UtcTime>>,
    ) -> PvtStep {
        let Some(epoch) = self.epoch else {
            return if pvt_time.is_some() {
                PvtStep::Ignored
            } else {
                PvtStep::Nothing
            };
        };
        if pvt_time == Some(Some(epoch)) {
            self.epoch = None;
            if core::mem::take(&mut self.waiting) {
                return PvtStep::Paired;
            }
            self.follows = true;
            return PvtStep::Late;
        }
        if gst_time == Some(Some(epoch)) {
            return PvtStep::Nothing;
        }
        self.epoch = None;
        if core::mem::take(&mut self.waiting) {
            self.follows = false;
            return PvtStep::Without {
                ignored: pvt_time.is_some(),
            };
        }
        if pvt_time.is_some() {
            PvtStep::Ignored
        } else {
            PvtStep::Nothing
        }
    }
}

/// One sentence read after a pair, as [`EpochAssembly`] sees it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum After<P> {
    /// A GST: its UTC time and deviations `[lat, lon, alt]`, mm.
    Gst(Option<UtcTime>, [Option<u32>; 3]),
    /// A PVT: its UTC time and its contents.
    Pvt(Option<UtcTime>, P),
    /// Any other sentence.
    Other,
}

/// A fix ready to publish: the fix, its GST's deviations (all `None`: without) and its epoch's PVT (`None`: without).
pub type Ready<F, P> = (F, [Option<u32>; 3], Option<P>);

/// The assembly of one epoch's reading: [`GstOrder`] and [`PvtOrder`] together. A fix goes out once, when both have
/// decided: with its GST and its PVT when they came in its burst, otherwise without them. A PVT of the epoch is not
/// the sentence [`GstOrder`] waits for (it waits through it), and the GST of the epoch is not the end of the burst
/// for [`PvtOrder`]; with no PVT in the stream the assembly is [`GstOrder`] alone.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EpochAssembly<F, P> {
    gst: GstOrder,
    pvt: PvtOrder,
    /// The fix that waits, its deviations once decided, its PVT once decided.
    waiting: Option<Waiting<F, P>>,
}

type Waiting<F, P> = (F, Option<[Option<u32>; 3]>, Option<Option<P>>);

impl<F, P> Default for EpochAssembly<F, P> {
    fn default() -> Self {
        Self {
            gst: GstOrder::default(),
            pvt: PvtOrder::default(),
            waiting: None,
        }
    }
}

impl<F, P> EpochAssembly<F, P> {
    /// Forget both orders and drop a waiting fix: at start, after every configuration and after a flush.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Whether a fix waits.
    pub fn waiting(&self) -> bool {
        self.waiting.is_some()
    }

    /// Whether GST, and whether PVT, follows the pair, as learnt.
    pub fn follows(&self) -> (bool, bool) {
        (self.gst.follows(), self.pvt.follows())
    }

    /// The pair of `fix` (at `fix_time`) completed; `kept_gst` is the latest GST. The fix if it goes now.
    pub fn pair(
        &mut self,
        fix: F,
        fix_time: Option<UtcTime>,
        kept_gst: Option<(Option<UtcTime>, [Option<u32>; 3])>,
    ) -> Option<Ready<F, P>> {
        let std_mm = match self.gst.pair(fix_time, kept_gst) {
            Publish::Now(std_mm) => Some(std_mm),
            Publish::Wait => None,
        };
        let pvt = if self.pvt.pair(fix_time) {
            None
        } else {
            Some(None)
        };
        self.settle(fix, std_mm, pvt)
    }

    /// The next sentence read: the waiting fix if it goes now, and what the PVT rule made of the sentence.
    pub fn sentence(&mut self, sentence: After<P>) -> (Option<Ready<F, P>>, PvtStep) {
        let (gst, pvt_time) = match &sentence {
            After::Gst(time, std_mm) => (Some((*time, *std_mm)), None),
            After::Pvt(time, _) => (None, Some(*time)),
            After::Other => (None, None),
        };
        let std_now = if self.pvt.of_epoch(pvt_time) {
            None
        } else {
            self.gst.sentence(gst)
        };
        let step = self.pvt.sentence(gst.map(|(time, _)| time), pvt_time);
        let Some((fix, std_mm, pvt)) = self.waiting.take() else {
            return (None, step);
        };
        let pvt = match (step, sentence) {
            (PvtStep::Paired, After::Pvt(_, p)) => Some(Some(p)),
            (PvtStep::Without { .. }, _) => Some(None),
            _ => pvt,
        };
        (self.settle(fix, std_mm.or(std_now), pvt), step)
    }

    fn settle(
        &mut self,
        fix: F,
        std_mm: Option<[Option<u32>; 3]>,
        pvt: Option<Option<P>>,
    ) -> Option<Ready<F, P>> {
        match (std_mm, pvt) {
            (Some(std_mm), Some(pvt)) => Some((fix, std_mm, pvt)),
            (std_mm, pvt) => {
                self.waiting = Some((fix, std_mm, pvt));
                None
            }
        }
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

impl BaroReading {
    /// Core 0's hook to the flight core: this reading as published at board time `t_us` with `seq`.
    pub const fn sample(self, t_us: u64, seq: u32) -> BaroSample {
        BaroSample {
            reading: self,
            t_us,
            seq,
        }
    }
}

impl MagReading {
    /// Core 0's hook to the flight core: this reading as published at board time `t_us` with `seq`.
    pub const fn sample(self, t_us: u64, seq: u32) -> MagSample {
        MagSample {
            reading: self,
            t_us,
            seq,
            expected: None,
        }
    }
}

impl GnssReading {
    /// Core 0's hook to the flight core: this reading as published at board time `t_us` with `seq`.
    pub const fn sample(self, t_us: u64, seq: u32) -> GnssSample {
        GnssSample {
            reading: self,
            t_us,
            seq,
        }
    }
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
    /// The field the health test expects, filled by core 0's adapter on every sample from the stored parameters; `None`
    /// unless `MAG_CAL_TYPE` is 2 and `MAG_FLD_UT` > 0 (`MagReading::sample` leaves it `None`).
    pub expected: Option<MagExpected>,
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

    /// A scripted stream for [`EpochAssembly`]: the pair of epoch `n`, its GST, a PVT carrying `n` (with the time of
    /// epoch `n`), any other sentence.
    #[derive(Clone, Copy, Debug)]
    enum Ev {
        Pair(u32),
        Gst(u32),
        Pvt(u32),
        Other,
    }

    /// Epoch `n` at 100 ms steps.
    fn et(n: u32) -> Option<UtcTime> {
        t((n / 10) as u8, (n % 10 * 100) as u16)
    }

    /// Feeds a stream; returns every publication (event index, epoch, deviations, PVT) and the PVT steps.
    #[allow(clippy::type_complexity)]
    fn run(
        a: &mut EpochAssembly<u32, u32>,
        events: &[Ev],
    ) -> (Vec<(usize, u32, [Option<u32>; 3], Option<u32>)>, Vec<PvtStep>) {
        let (mut out, mut steps) = (Vec::new(), Vec::new());
        let mut kept = None;
        for (i, e) in events.iter().enumerate() {
            let ready = match *e {
                Ev::Pair(n) => a.pair(n, et(n), kept),
                Ev::Gst(n) => {
                    kept = Some((et(n), SIGMAS));
                    let (r, s) = a.sentence(After::Gst(et(n), SIGMAS));
                    steps.push(s);
                    r
                }
                Ev::Pvt(n) => {
                    let (r, s) = a.sentence(After::Pvt(et(n), n));
                    steps.push(s);
                    r
                }
                Ev::Other => {
                    let (r, s) = a.sentence(After::Other);
                    steps.push(s);
                    r
                }
            };
            if let Some((n, std_mm, pvt)) = ready {
                out.push((i, n, std_mm, pvt));
            }
        }
        (out, steps)
    }

    /// The receiver's burst with PVT on: GGA (other), the pair at the RMC, GST, PVT.
    fn burst(n: u32) -> [Ev; 4] {
        [Ev::Other, Ev::Pair(n), Ev::Gst(n), Ev::Pvt(n)]
    }

    use std::vec::Vec;

    #[test]
    fn pvt_after_the_pair_is_learnt_then_each_epoch_goes_once_with_its_velocity() {
        let mut a = EpochAssembly::<u32, u32>::default();
        let stream: Vec<Ev> = (1..=4).flat_map(burst).collect();
        let (out, steps) = run(&mut a, &stream);
        // Epoch 1 at its pair, without anything (nothing learnt); its GST and PVT teach the orders.
        assert_eq!(out[0], (1, 1, [None; 3], None));
        assert_eq!(steps[2], PvtStep::Late);
        assert_eq!(a.follows(), (true, true));
        // Epochs 2 to 4: once each, at the PVT (the last sentence of the burst), with the GST and the velocity.
        for (k, n) in (2..=4).enumerate() {
            assert_eq!(out[k + 1], (4 * n as usize - 1, n, SIGMAS, Some(n)));
        }
        assert_eq!(out.len(), 4);
        assert_eq!(steps.iter().filter(|s| **s == PvtStep::Paired).count(), 3);
        assert!(!a.waiting());
    }

    #[test]
    fn an_epoch_without_its_pvt_goes_at_the_end_of_its_burst_then_the_order_is_learnt_again() {
        let mut a = EpochAssembly::<u32, u32>::default();
        let mut stream: Vec<Ev> = (1..=2).flat_map(burst).collect();
        // Epoch 3 has no PVT; epochs 4 and 5 have it again.
        stream.extend([Ev::Other, Ev::Pair(3), Ev::Gst(3)]);
        stream.extend((4..=5).flat_map(burst));
        let (out, steps) = run(&mut a, &stream);
        // Epoch 3 waits through its GST and goes at the next sentence (the GGA of epoch 4), without a velocity.
        assert_eq!(out[2], (11, 3, SIGMAS, None));
        assert_eq!(steps[8], PvtStep::Without { ignored: false });
        // Epoch 4 goes at its GST (PVT forgotten), its PVT teaches again, epoch 5 is paired.
        assert_eq!(out[3], (13, 4, SIGMAS, None));
        assert_eq!(steps[10], PvtStep::Late);
        assert_eq!(out[4], (18, 5, SIGMAS, Some(5)));
        assert_eq!(out.len(), 5);
    }

    /// Without PVT in the stream the assembly publishes exactly as [`GstOrder`] alone.
    #[test]
    fn without_pvt_the_assembly_is_the_gst_rule() {
        let stream: Vec<Ev> = (1..=6)
            .flat_map(|n| [Ev::Other, Ev::Pair(n), Ev::Gst(n)])
            .chain([Ev::Other, Ev::Pair(7), Ev::Other, Ev::Pair(8), Ev::Gst(8)])
            .collect();
        let mut a = EpochAssembly::<u32, u32>::default();
        let (out, steps) = run(&mut a, &stream);
        let mut o = GstOrder::default();
        let mut kept = None;
        let mut want = Vec::new();
        let mut waiting = None;
        for (i, e) in stream.iter().enumerate() {
            match *e {
                Ev::Pair(n) => match o.pair(et(n), kept) {
                    Publish::Now(std_mm) => want.push((i, n, std_mm, None)),
                    Publish::Wait => waiting = Some(n),
                },
                Ev::Gst(n) => {
                    kept = Some((et(n), SIGMAS));
                    if let Some(std_mm) = o.sentence(kept)
                        && let Some(w) = waiting.take()
                    {
                        want.push((i, w, std_mm, None));
                    }
                }
                _ => {
                    if let Some(std_mm) = o.sentence(None)
                        && let Some(w) = waiting.take()
                    {
                        want.push((i, w, std_mm, None));
                    }
                }
            }
        }
        assert_eq!(out, want);
        assert_eq!(out.len(), 8);
        assert!(steps.iter().all(|s| *s == PvtStep::Nothing));
        assert!(!a.follows().1);
    }

    #[test]
    fn a_pvt_of_another_epoch_is_ignored() {
        let mut a = EpochAssembly::<u32, u32>::default();
        let mut stream: Vec<Ev> = (1..=2).flat_map(burst).collect();
        // Epoch 3 gets the PVT of epoch 2 (indices 8 to 11), then a stray PVT with no fix waiting.
        stream.extend([Ev::Other, Ev::Pair(3), Ev::Gst(3), Ev::Pvt(2), Ev::Pvt(9)]);
        let (out, steps) = run(&mut a, &stream);
        assert_eq!(out[2], (11, 3, SIGMAS, None));
        assert_eq!(steps[8], PvtStep::Without { ignored: true });
        assert_eq!(steps[9], PvtStep::Ignored);
        assert!(!a.follows().1);
        // A PVT that comes before its pair is another epoch's for the running burst: ignored, not used.
        let mut a = EpochAssembly::<u32, u32>::default();
        let (out, steps) = run(&mut a, &[Ev::Pvt(1), Ev::Pair(1), Ev::Gst(1)]);
        assert_eq!(steps[0], PvtStep::Ignored);
        assert_eq!(out, [(1, 1, [None; 3], None)]);
    }

    /// The PVT ahead of the GST would also work: the PVT of the epoch is not the sentence GST's rule waits for.
    #[test]
    fn pvt_before_gst_also_pairs_both() {
        let mut a = EpochAssembly::<u32, u32>::default();
        let stream: Vec<Ev> = (1..=3)
            .flat_map(|n| [Ev::Other, Ev::Pair(n), Ev::Pvt(n), Ev::Gst(n)])
            .collect();
        let (out, _) = run(&mut a, &stream);
        assert_eq!(a.follows(), (true, true));
        assert_eq!(out[1..], [(7, 2, SIGMAS, Some(2)), (11, 3, SIGMAS, Some(3))]);
    }

    /// A configuration or a flush forgets both orders and drops the waiting fix.
    #[test]
    fn reset_forgets_both_orders() {
        let mut a = EpochAssembly::<u32, u32>::default();
        let mut stream: Vec<Ev> = burst(1).to_vec();
        stream.extend([Ev::Other, Ev::Pair(2)]);
        let _ = run(&mut a, &stream);
        assert!(a.waiting());
        a.reset();
        assert!(!a.waiting());
        assert_eq!(a.follows(), (false, false));
        let (out, _) = run(&mut a, &[Ev::Gst(2), Ev::Pvt(2), Ev::Other, Ev::Pair(3)]);
        assert_eq!(out, [(3, 3, [None; 3], None)]);
    }

    /// 10 Hz on the 115200 baud line (11.52 bytes/ms): GGA 75 B, RMC 70 B, GST 50 B, PVT 137 B, GSA 66 B once a
    /// second, sizes with CR LF (PVT as measured, 134 to 139 B; the others typical). Every epoch after the first goes
    /// once, inside its own burst, 16.2 ms after its RMC (the GST and the PVT on the line), and never in the next
    /// interval.
    #[test]
    fn at_10_hz_every_epoch_goes_inside_its_burst_about_16_ms_after_its_rmc() {
        let mut a = EpochAssembly::<u32, u32>::default();
        let mut kept = None;
        let mut published = Vec::new();
        for n in 1..=50u32 {
            let start_us = u64::from(n) * 100_000;
            let mut at_us = start_us;
            let mut on_line = |bytes: u64| {
                at_us += bytes * 1_000_000 / 11_520;
                at_us
            };
            on_line(75);
            if n % 10 == 0 {
                on_line(66);
                let _ = a.sentence(After::Other);
            }
            let _ = a.sentence(After::Other);
            let rmc_us = on_line(70);
            let mut ready = a.pair(n, et(n), kept).map(|r| (r, rmc_us));
            let gst_us = on_line(50);
            kept = Some((et(n), SIGMAS));
            if let (Some(r), _) = a.sentence(After::Gst(et(n), SIGMAS)) {
                ready = Some((r, gst_us));
            }
            let pvt_us = on_line(137);
            if let (Some(r), _) = a.sentence(After::Pvt(et(n), n)) {
                ready = Some((r, pvt_us));
            }
            let ((epoch, std_mm, pvt), went_us) = ready.unwrap();
            assert_eq!(epoch, n);
            assert!(went_us < start_us + 100_000, "{n}");
            published.push((n, std_mm, pvt, went_us - rmc_us));
        }
        assert_eq!(published[0], (1, [None; 3], None, 0));
        for &(n, std_mm, pvt, delay_us) in &published[1..] {
            assert_eq!((std_mm, pvt), (SIGMAS, Some(n)), "{n}");
            assert!((16_000..16_500).contains(&delay_us), "{n}: {delay_us} us");
        }
    }

    const BARO: BaroReading = BaroReading {
        pressure_pa: 101_325.5,
        temperature_c: 24.25,
    };
    const MAG: MagReading = MagReading {
        field_ut: [21.5, -3.25, 40.125],
        temperature_c: 30.5,
    };
    const GNSS: GnssReading = GnssReading {
        fix_type: 3,
        lat_e7: Some(254_000_001),
        lon_e7: Some(-801_000_002),
        alt_mm: Some(12_345),
        speed_mm_s: Some(1_234),
        course_cdeg: Some(35_999),
        sats_used: Some(11),
        hdop_x100: Some(87),
        vdop_x100: Some(132),
        time: Some(UtcTime {
            hour: 12,
            minute: 34,
            second: 5,
            millis: 200,
        }),
        date: Some(UtcDate {
            year: 2026,
            month: 11,
            day: 20,
        }),
        lat_std_mm: Some(1200),
        lon_std_mm: Some(900),
        alt_std_mm: Some(2500),
        vel_ned_mm_s: Some([1_000, -2_000, 3_000]),
    };

    #[test]
    fn each_reading_becomes_its_sample_with_its_time_and_seq() {
        let b = BARO.sample(1_000_123, 7);
        assert_eq!((b.reading, b.t_us, b.seq), (BARO, 1_000_123, 7));
        let m = MAG.sample(2_000_456, u32::MAX);
        assert_eq!((m.reading, m.t_us, m.seq), (MAG, 2_000_456, u32::MAX));
        assert_eq!(m.expected, None);
        let g = GNSS.sample(u64::MAX, 0);
        assert_eq!((g.reading, g.t_us, g.seq), (GNSS, u64::MAX, 0));
    }

    /// The flight core's view through the stick path's cell: a sample is fresh once, then stale until the next
    /// write; only the newest of several writes is seen, and the gap shows in `seq`.
    fn fresh_once<T: Copy + PartialEq + core::fmt::Debug>(initial: T, a: T, b: T, c: T) {
        let cell = crate::xcore::TripleBuffer::new(initial);
        let (mut w, mut r) = (cell.take_writer().unwrap(), cell.take_reader().unwrap());
        assert_eq!(r.read(), (initial, false));
        w.write(a);
        assert_eq!(r.read(), (a, true));
        assert_eq!(r.read(), (a, false));
        assert_eq!(r.read(), (a, false));
        w.write(b);
        w.write(c);
        assert_eq!(r.read(), (c, true));
        assert_eq!(r.read(), (c, false));
    }

    #[test]
    fn a_sample_is_fresh_once_on_the_flight_side() {
        fresh_once(
            BARO.sample(0, 0),
            BARO.sample(10, 1),
            BARO.sample(20, 2),
            BARO.sample(30, 3),
        );
        fresh_once(
            MAG.sample(0, 0),
            MAG.sample(10, 1),
            MAG.sample(20, 2),
            MAG.sample(30, 3),
        );
        fresh_once(
            GNSS.sample(0, 0),
            GNSS.sample(10, 1),
            GNSS.sample(20, 2),
            GNSS.sample(30, 3),
        );
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
