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
