//! The RC stick mapping: the 44 parameters (ArduPilot's `RCn_MIN`, `RCn_TRIM`, `RCn_MAX`,
//! `RCn_REVERSED`, `RCn_DZ` for channels 1 to 8, and `RCMAP_ROLL`, `RCMAP_PITCH`, `RCMAP_THROTTLE`,
//! `RCMAP_YAW`) and the pure rule that turns the microseconds of a stick report into the four
//! normalised functions and the MANUAL_CONTROL axes (decision-rc-mapping, 1.1, 1.2, 1.4).
//!
//! All are `Live`: nothing is configured, the next computation reads them. The functions are total:
//! no cross-parameter check exists (MIN below TRIM below MAX, a unique map), so every value in the
//! ranges gives a result.

use fw_core::params::{Apply, ParamDef};
use mavlink_device::ParamValue;
use mavlink_device::marv::MarvDevice;

const DEVICE: u8 = MarvDevice::MARV_DEVICE_RC as u8;

/// The number of calibrated channels.
pub const CHANNELS: usize = 8;
/// The parameters of one channel: MIN, TRIM, MAX, REVERSED, DZ.
const PER_CHANNEL: usize = 5;
/// The number of function-map parameters: roll, pitch, throttle, yaw.
const FUNCTIONS: usize = 4;

/// The number of RC parameters.
pub const COUNT: usize = CHANNELS * PER_CHANNEL + FUNCTIONS;

/// Where the RC table starts in `fw_settings::TABLE` (it is appended last).
pub const RC_OFFSET: usize = crate::baro::COUNT
    + crate::mag::COUNT
    + crate::batt::COUNT
    + crate::imu::COUNT
    + crate::sd::COUNT;

/// Roll's channel (ArduPilot's name).
pub const MAP_ROLL: &str = "RCMAP_ROLL";
/// Pitch's channel.
pub const MAP_PITCH: &str = "RCMAP_PITCH";
/// Throttle's channel.
pub const MAP_THROTTLE: &str = "RCMAP_THROTTLE";
/// Yaw's channel.
pub const MAP_YAW: &str = "RCMAP_YAW";

/// Lowest and highest value of MIN, TRIM and MAX, in microseconds.
const US_MIN: i32 = 800;
const US_MAX: i32 = 2200;

const fn channel(
    min: &'static str,
    trim: &'static str,
    max: &'static str,
    rev: &'static str,
    dz: &'static str,
) -> [ParamDef; PER_CHANNEL] {
    [
        ParamDef::i32(min, DEVICE, 1100, US_MIN, US_MAX, Apply::Live),
        ParamDef::i32(trim, DEVICE, 1500, US_MIN, US_MAX, Apply::Live),
        ParamDef::i32(max, DEVICE, 1900, US_MIN, US_MAX, Apply::Live),
        ParamDef::i32(rev, DEVICE, 0, 0, 1, Apply::Live),
        ParamDef::i32(dz, DEVICE, 0, 0, 200, Apply::Live),
    ]
}

const CH: [[ParamDef; PER_CHANNEL]; CHANNELS] = [
    channel("RC1_MIN", "RC1_TRIM", "RC1_MAX", "RC1_REVERSED", "RC1_DZ"),
    channel("RC2_MIN", "RC2_TRIM", "RC2_MAX", "RC2_REVERSED", "RC2_DZ"),
    channel("RC3_MIN", "RC3_TRIM", "RC3_MAX", "RC3_REVERSED", "RC3_DZ"),
    channel("RC4_MIN", "RC4_TRIM", "RC4_MAX", "RC4_REVERSED", "RC4_DZ"),
    channel("RC5_MIN", "RC5_TRIM", "RC5_MAX", "RC5_REVERSED", "RC5_DZ"),
    channel("RC6_MIN", "RC6_TRIM", "RC6_MAX", "RC6_REVERSED", "RC6_DZ"),
    channel("RC7_MIN", "RC7_TRIM", "RC7_MAX", "RC7_REVERSED", "RC7_DZ"),
    channel("RC8_MIN", "RC8_TRIM", "RC8_MAX", "RC8_REVERSED", "RC8_DZ"),
];

/// The RC parameter table: for channels 1 to 8 MIN, TRIM, MAX, REVERSED, DZ, then the four function
/// maps. None is a `calibration` parameter: MIN, TRIM and MAX are calibration results, but "reset
/// calibration" (`PREFLIGHT_STORAGE` param1 = 3) resets sensor calibrations only; the end points come
/// back through "reset all" (param1 = 2) or a new radio calibration.
pub static TABLE: [ParamDef; COUNT] = build();

#[allow(clippy::indexing_slicing)] // the loops run exactly over the sources' lengths
const fn build() -> [ParamDef; COUNT] {
    let mut out = [CH[0][0]; COUNT];
    let mut n = 0;
    let mut c = 0;
    while c < CHANNELS {
        let mut k = 0;
        while k < PER_CHANNEL {
            out[n] = CH[c][k];
            n += 1;
            k += 1;
        }
        c += 1;
    }
    out[n] = ParamDef::i32(MAP_ROLL, DEVICE, 1, 1, CHANNELS as i32, Apply::Live);
    out[n + 1] = ParamDef::i32(MAP_PITCH, DEVICE, 2, 1, CHANNELS as i32, Apply::Live);
    out[n + 2] = ParamDef::i32(MAP_THROTTLE, DEVICE, 3, 1, CHANNELS as i32, Apply::Live);
    out[n + 3] = ParamDef::i32(MAP_YAW, DEVICE, 4, 1, CHANNELS as i32, Apply::Live);
    out
}

/// One channel's calibration, in microseconds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Channel {
    /// `RCn_MIN`.
    pub min: i32,
    /// `RCn_TRIM`.
    pub trim: i32,
    /// `RCn_MAX`.
    pub max: i32,
    /// `RCn_REVERSED` is not 0.
    pub reversed: bool,
    /// `RCn_DZ`.
    pub dz: i32,
}

const DEFAULT_CHANNEL: Channel = Channel {
    min: 1100,
    trim: 1500,
    max: 1900,
    reversed: false,
    dz: 0,
};

/// The mapping: eight channels' calibration and the channel (1 to 8) of each function.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RcMap {
    /// Channels 1 to 8.
    pub channels: [Channel; CHANNELS],
    /// `RCMAP_ROLL`.
    pub roll: i32,
    /// `RCMAP_PITCH`.
    pub pitch: i32,
    /// `RCMAP_THROTTLE`.
    pub throttle: i32,
    /// `RCMAP_YAW`.
    pub yaw: i32,
}

/// The four functions as MANUAL_CONTROL axes: `x` pitch and `y` roll and `r` yaw in -1000 to 1000,
/// `z` throttle in 0 to 1000.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Axes {
    /// Pitch, positive for stick forward.
    pub x: i16,
    /// Roll, positive for stick right.
    pub y: i16,
    /// Throttle, 0 at the bottom.
    pub z: i16,
    /// Yaw, positive clockwise.
    pub r: i16,
}

/// The four functions normalised: roll, pitch, yaw in -1 to 1, throttle in 0 to 1 (moved to `fw_core::rc`).
pub use fw_core::rc::Sticks;

fn int(v: Option<ParamValue>, def: &ParamDef) -> i32 {
    match v {
        Some(ParamValue::I32(i)) => i,
        _ => match def.default {
            ParamValue::I32(i) => i,
            ParamValue::F32(_) => 0,
        },
    }
}

impl RcMap {
    /// The mapping from the 44 values in table order (a value that is missing or not an integer
    /// reads as its default); a channel number outside 1 to 8 reads as the function's default.
    pub fn from_values(values: &[Option<ParamValue>; COUNT]) -> Self {
        let mut channels = [DEFAULT_CHANNEL; CHANNELS];
        let mut get = values.iter().zip(TABLE.iter());
        for ch in &mut channels {
            let mut next = || get.next().map_or(0, |(v, d)| int(*v, d));
            ch.min = next();
            ch.trim = next();
            ch.max = next();
            ch.reversed = next() != 0;
            ch.dz = next();
        }
        let mut next = |default: i32| {
            let c = get.next().map_or(default, |(v, d)| int(*v, d));
            if (1..=CHANNELS as i32).contains(&c) {
                c
            } else {
                default
            }
        };
        let roll = next(1);
        let pitch = next(2);
        let throttle = next(3);
        let yaw = next(4);
        Self {
            channels,
            roll,
            pitch,
            throttle,
            yaw,
        }
    }

    /// The mapping with every parameter at its default.
    pub fn defaults() -> Self {
        Self::from_values(&[None; COUNT])
    }

    fn channel(&self, number: i32) -> Channel {
        usize::try_from(number - 1)
            .ok()
            .and_then(|i| self.channels.get(i).copied())
            .unwrap_or(DEFAULT_CHANNEL)
    }

    /// The four functions from the 16 microsecond values of a stick report (channel 1 is index 0).
    pub fn sticks(&self, pwm: &[u16; 16]) -> Sticks {
        let at = |number: i32| {
            usize::try_from(number - 1)
                .ok()
                .and_then(|i| pwm.get(i).copied())
                .unwrap_or(0)
        };
        Sticks {
            roll: angle(at(self.roll), &self.channel(self.roll)),
            pitch: angle(at(self.pitch), &self.channel(self.pitch)),
            throttle: range(at(self.throttle), &self.channel(self.throttle)),
            yaw: angle(at(self.yaw), &self.channel(self.yaw)),
        }
    }

    /// The MANUAL_CONTROL axes from the 16 microsecond values of a stick report.
    pub fn axes(&self, pwm: &[u16; 16]) -> Axes {
        let s = self.sticks(pwm);
        Axes {
            x: scale(s.pitch),
            y: scale(s.roll),
            z: scale(s.throttle),
            r: scale(s.yaw),
        }
    }
}

/// `x` times 1000 rounded to the nearest integer, halves away from zero (`core` has no
/// `f32::round`, as in `batt`).
fn scale(x: f32) -> i16 {
    let y = x * 1000.0;
    let r = if y >= 0.0 {
        (y + 0.5) as i32
    } else {
        -((-y + 0.5) as i32)
    };
    r.clamp(-1000, 1000) as i16
}

/// Roll, pitch, yaw: ArduPilot's `pwm_to_angle_dz_trim` divided by its `high_in`, -1 to 1.
pub fn angle(pwm: u16, ch: &Channel) -> f32 {
    if ch.max <= ch.min {
        return 0.0;
    }
    // Not `clamp`: it panics when MIN is above MAX, and a user can write such values.
    #[allow(clippy::manual_clamp)]
    let r = i32::from(pwm).max(ch.min).min(ch.max);
    let hi = ch.trim + ch.dz;
    let lo = ch.trim - ch.dz;
    let s = if ch.reversed { -1.0 } else { 1.0 };
    let v = if r > hi && ch.max != hi {
        s * (r - hi) as f32 / (ch.max - hi) as f32
    } else if r < lo && lo != ch.min {
        s * (r - lo) as f32 / (lo - ch.min) as f32
    } else {
        0.0
    };
    v.clamp(-1.0, 1.0)
}

/// Throttle: ArduPilot's `pwm_to_range_dz` divided by its `high_in` (TRIM is ignored), 0 to 1.
pub fn range(pwm: u16, ch: &Channel) -> f32 {
    if ch.max <= ch.min {
        return 0.0;
    }
    #[allow(clippy::manual_clamp)] // not `clamp`: it panics when MIN is above MAX
    let mut r = i32::from(pwm).max(ch.min).min(ch.max);
    if ch.reversed {
        r = ch.max - (r - ch.min);
    }
    let lo = ch.min + ch.dz;
    let v = if r > lo {
        (r - lo) as f32 / (ch.max - lo) as f32
    } else {
        0.0
    };
    v.clamp(0.0, 1.0)
}

/// The arm switch's on threshold, us: ArduPilot's `AUX_SWITCH_PWM_TRIGGER_HIGH`. Applied to the raw
/// channel 5 microseconds; no calibration or reversal.
pub const ARM_SWITCH_HIGH_US: u16 = 1800;

/// Channel 5 (raw us) in the arm position: strictly above [`ARM_SWITCH_HIGH_US`]. 0 (a missing
/// channel) is off.
pub fn arm_switch(ch5_us: u16) -> bool {
    ch5_us > ARM_SWITCH_HIGH_US
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;

    const fn ch(min: i32, trim: i32, max: i32, rev: i32, dz: i32) -> Channel {
        Channel {
            min,
            trim,
            max,
            reversed: rev != 0,
            dz,
        }
    }

    fn out(v: f32) -> i32 {
        i32::from(scale(v))
    }

    #[test]
    fn the_table_is_the_44_of_the_decision() {
        assert_eq!(COUNT, 44);
        assert_eq!(TABLE.len(), 44);
        let mut it = TABLE.iter();
        for n in 1..=8 {
            for (suffix, default, min, max, calibration) in [
                ("MIN", 1100, 800, 2200, false),
                ("TRIM", 1500, 800, 2200, false),
                ("MAX", 1900, 800, 2200, false),
                ("REVERSED", 0, 0, 1, false),
                ("DZ", 0, 0, 200, false),
            ] {
                let d = it.next().unwrap();
                assert_eq!(d.name, std::format!("RC{n}_{suffix}"));
                assert_eq!(d.device, 7);
                assert_eq!(d.default, ParamValue::I32(default), "{}", d.name);
                assert_eq!(d.min, ParamValue::I32(min), "{}", d.name);
                assert_eq!(d.max, ParamValue::I32(max), "{}", d.name);
                assert_eq!(d.apply, Apply::Live, "{}", d.name);
                assert_eq!(d.calibration, calibration, "{}", d.name);
                assert!(!d.read_only);
            }
        }
        for (name, default) in [
            ("RCMAP_ROLL", 1),
            ("RCMAP_PITCH", 2),
            ("RCMAP_THROTTLE", 3),
            ("RCMAP_YAW", 4),
        ] {
            let d = it.next().unwrap();
            assert_eq!(d.name, name);
            assert_eq!(d.device, 7);
            assert_eq!(d.default, ParamValue::I32(default));
            assert_eq!(d.min, ParamValue::I32(1));
            assert_eq!(d.max, ParamValue::I32(8));
            assert_eq!(d.apply, Apply::Live);
            assert!(!d.calibration && !d.read_only);
        }
        assert!(it.next().is_none());
        assert!("RCMAP_THROTTLE".len() <= 16);
        Params::<COUNT>::new(&TABLE).unwrap();
    }

    #[test]
    fn reset_calibration_leaves_the_end_points_and_reset_all_resets_them() {
        use mavlink_device::{ParamName, ParamStore};
        let mut p = Params::<{ crate::COUNT }>::new(&crate::TABLE).unwrap();
        let min = p.find(&ParamName::new("RC1_MIN").unwrap()).unwrap();
        let off = p.find(&ParamName::new("MAG_OFFX").unwrap()).unwrap();
        p.set(min, ParamValue::I32(1000)).unwrap();
        p.set(off, ParamValue::F32(10.0)).unwrap();
        assert_eq!(p.reset_calibration(), 1);
        assert_eq!(p.get(off), Some(ParamValue::F32(0.0)));
        assert_eq!(p.get(min), Some(ParamValue::I32(1000)));
        p.set(off, ParamValue::F32(10.0)).unwrap();
        assert_eq!(p.reset_all(), 2);
        assert_eq!(p.get(off), Some(ParamValue::F32(0.0)));
        assert_eq!(p.get(min), Some(ParamValue::I32(1100)));
    }

    #[test]
    fn the_rc_table_keeps_its_place_in_the_firmware_table() {
        assert_eq!(crate::COUNT, 81);
        assert_eq!(RC_OFFSET, 26);
        for (i, d) in TABLE.iter().enumerate() {
            assert_eq!(crate::TABLE[RC_OFFSET + i].name, d.name);
            assert_eq!(crate::TABLE[RC_OFFSET + i].device, d.device);
            assert_eq!(crate::TABLE[RC_OFFSET + i].default, d.default);
        }
        assert_eq!(crate::TABLE[0].name, "BARO_ODR_HZ");
        assert_eq!(crate::TABLE[RC_OFFSET - 1].name, "LOG_BACKEND_TYPE");
    }

    #[test]
    fn angle_worked_examples() {
        let rows: &[(Channel, &[(u16, i32)])] = &[
            (
                ch(1100, 1500, 1900, 0, 0),
                &[
                    (1500, 0),
                    (1700, 500),
                    (1300, -500),
                    (1900, 1000),
                    (2012, 1000),
                    (1100, -1000),
                    (988, -1000),
                ],
            ),
            (
                ch(988, 1500, 2012, 0, 0),
                &[
                    (2012, 1000),
                    (1756, 500),
                    (1244, -500),
                    (1600, 195),
                    (1400, -195),
                    (988, -1000),
                ],
            ),
            (
                ch(988, 1500, 2012, 0, 20),
                &[
                    (1510, 0),
                    (1520, 0),
                    (1521, 2),
                    (1766, 500),
                    (1234, -500),
                    (2012, 1000),
                ],
            ),
            (
                ch(988, 1500, 2012, 1, 0),
                &[(1756, -500), (1244, 500), (2012, -1000)],
            ),
            (
                ch(1000, 1600, 2000, 0, 0),
                &[(1800, 500), (1300, -500), (1000, -1000), (2000, 1000)],
            ),
            (
                ch(1100, 1000, 1900, 0, 0),
                &[(1100, 111), (1550, 611), (1900, 1000)],
            ),
            (
                ch(1400, 1500, 1600, 0, 200),
                &[(1400, 0), (1500, 0), (1600, 0)],
            ),
            (
                ch(1900, 1500, 1100, 0, 0),
                &[(988, 0), (1500, 0), (2012, 0)],
            ),
        ];
        for (c, cases) in rows {
            for &(pwm, want) in *cases {
                assert_eq!(out(angle(pwm, c)), want, "{c:?} pwm {pwm}");
            }
        }
    }

    #[test]
    fn range_worked_examples() {
        let rows: &[(Channel, &[(u16, i32)])] = &[
            (
                ch(1100, 1500, 1900, 0, 0),
                &[(988, 0), (1100, 0), (1500, 500), (1900, 1000), (2012, 1000)],
            ),
            (
                ch(988, 1500, 2012, 0, 0),
                &[(988, 0), (1244, 250), (1500, 500), (2012, 1000)],
            ),
            (
                ch(988, 1500, 2012, 0, 30),
                &[(1018, 0), (1515, 500), (2012, 1000)],
            ),
            (
                ch(988, 1500, 2012, 1, 0),
                &[(988, 1000), (1244, 750), (2012, 0)],
            ),
            (
                ch(1400, 1500, 1600, 0, 200),
                &[(1400, 0), (1500, 0), (1600, 0)],
            ),
            (
                ch(1900, 1500, 1100, 0, 0),
                &[(988, 0), (1500, 0), (2012, 0)],
            ),
        ];
        for (c, cases) in rows {
            for &(pwm, want) in *cases {
                assert_eq!(out(range(pwm, c)), want, "{c:?} pwm {pwm}");
            }
        }
    }

    fn report(first4: [u16; 4]) -> [u16; 16] {
        let mut p = [1500u16; 16];
        p[..4].copy_from_slice(&first4);
        p
    }

    #[test]
    fn the_map_example() {
        let mut map = RcMap::defaults();
        let pwm = report([1700, 1300, 1500, 1900]);
        let s = map.sticks(&pwm);
        assert_eq!(
            (out(s.roll), out(s.pitch), out(s.throttle), out(s.yaw)),
            (500, -500, 500, 1000)
        );
        assert_eq!(
            map.axes(&pwm),
            Axes {
                x: -500,
                y: 500,
                z: 500,
                r: 1000
            }
        );
        map.roll = 2;
        map.pitch = 1;
        let s = map.sticks(&pwm);
        assert_eq!((out(s.roll), out(s.pitch)), (-500, 500));
        // A duplicate map is accepted: both read channel 1.
        map.roll = 1;
        map.yaw = 1;
        let s = map.sticks(&pwm);
        assert_eq!((out(s.roll), out(s.yaw)), (500, 500));
    }

    #[test]
    fn the_map_is_built_from_the_values_in_table_order() {
        let mut values = [None; COUNT];
        assert_eq!(RcMap::from_values(&values), RcMap::defaults());
        // RC2_MIN, RC2_REVERSED, RCMAP_YAW.
        values[5] = Some(ParamValue::I32(900));
        values[8] = Some(ParamValue::I32(1));
        values[43] = Some(ParamValue::I32(8));
        let m = RcMap::from_values(&values);
        assert_eq!(m.channels[1].min, 900);
        assert!(m.channels[1].reversed);
        assert!(!m.channels[0].reversed);
        assert_eq!(m.yaw, 8);
        assert_eq!((m.roll, m.pitch, m.throttle), (1, 2, 3));
    }

    #[test]
    fn every_corner_of_the_ranges_and_every_input_is_well_defined() {
        for min in [800, 2200] {
            for trim in [800, 2200] {
                for max in [800, 2200] {
                    for rev in [0, 1] {
                        for dz in [0, 200] {
                            let c = ch(min, trim, max, rev, dz);
                            let mut prev_a = f32::MIN;
                            let mut prev_r = f32::MIN;
                            for pwm in 0..=u16::MAX {
                                let a = angle(pwm, &c);
                                let r = range(pwm, &c);
                                assert!(a.is_finite() && (-1.0..=1.0).contains(&a), "{c:?} {pwm}");
                                assert!(r.is_finite() && (0.0..=1.0).contains(&r), "{c:?} {pwm}");
                                if rev == 0 {
                                    assert!(a >= prev_a && r >= prev_r, "{c:?} {pwm}");
                                }
                                prev_a = a;
                                prev_r = r;
                                let n = ch(c.min, c.trim, c.max, 1 - rev, c.dz);
                                assert_eq!(angle(pwm, &n), -a, "{c:?} {pwm}");
                                if c.max > c.min {
                                    let r = i32::from(pwm).max(c.min).min(c.max);
                                    let mirrored = u16::try_from(c.max - (r - c.min)).unwrap();
                                    assert_eq!(range(pwm, &n), range(mirrored, &c), "{c:?} {pwm}");
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn the_dead_zone_gives_zero() {
        for (min, trim, max, dz) in [(1100, 1500, 1900, 50), (988, 1500, 2012, 200)] {
            let c = ch(min, trim, max, 0, dz);
            for pwm in (trim - dz)..=(trim + dz) {
                assert_eq!(angle(u16::try_from(pwm).unwrap(), &c), 0.0);
            }
            for pwm in 0..=(min + dz) {
                assert_eq!(range(u16::try_from(pwm).unwrap(), &c), 0.0);
            }
        }
    }

    #[test]
    fn the_axes_are_scaled_and_rounded_away_from_zero() {
        assert_eq!(scale(0.0), 0);
        assert_eq!(scale(0.0005), 1);
        assert_eq!(scale(-0.0005), -1);
        assert_eq!(scale(1.0), 1000);
        assert_eq!(scale(-1.0), -1000);
    }

    #[test]
    fn the_arm_switch_is_on_strictly_above_1800() {
        assert!(!arm_switch(1800));
        assert!(arm_switch(1801));
        assert!(!arm_switch(0));
        assert!(arm_switch(2000));
    }
}
