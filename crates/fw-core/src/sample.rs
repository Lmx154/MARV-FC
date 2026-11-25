//! The slow sensors' samples as the flight code takes them: barometer, compass, GNSS.
//! Units are the firmware's.

/// Barometer: pressure, Pa; die temperature, deg C.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroReading {
    /// Pa.
    pub pressure_pa: f32,
    /// deg C.
    pub temperature_c: f32,
}

/// Compass: the calibrated field m = S (r - o), microtesla, body FRD (`MAG_ORIENT` applied, then the offsets and the soft-iron matrix, `fw_settings::mag::apply`); die temperature, deg C.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagReading {
    /// Microtesla, body FRD, calibrated.
    pub field_ut: [f32; 3],
    /// deg C.
    pub temperature_c: f32,
}

/// What the compass should read where this vehicle was calibrated, for the flight core's health test.
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

/// One receiver epoch in its integer units. Position, speed and course
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
    /// Velocity north, east and down in mm/s (NED) from the receiver's NAV-PVT of this epoch (velN, velE, velD; down positive); `None` when there is no 3D fix.
    pub vel_ned_mm_s: Option<[i32; 3]>,
}

/// The horizontal deviation sqrt(lat^2 + lon^2), mm (rounded down; `u32::MAX` beyond), when both are
/// known: GPS_RAW_INT's `h_acc`.
pub fn horizontal_std_mm(lat_std_mm: Option<u32>, lon_std_mm: Option<u32>) -> Option<u32> {
    let (lat, lon) = (u64::from(lat_std_mm?), u64::from(lon_std_mm?));
    Some(u32::try_from((lat * lat).saturating_add(lon * lon).isqrt()).unwrap_or(u32::MAX))
}

/// Time of day, UTC.
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

/// Calendar date, UTC.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UtcDate {
    /// Full year.
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
    /// The field the health test expects, filled by core 0's adapter on every sample from the stored parameters; `None` unless `MAG_CAL_TYPE` is 2 and `MAG_FLD_UT` > 0 (`MagReading::sample` leaves it `None`).
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

    /// The flight core's view through the stick path's cell: a sample is fresh once, then stale until the next write; only the newest of several writes is seen, and the gap shows in `seq`.
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
