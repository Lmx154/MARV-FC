//! The on-board log format: ArduPilot DataFlash `.BIN` records.
//!
//! This module only turns plain values into bytes in a caller's buffer. It has no state, no file,
//! no ring and no clock: every function writes exactly one record (or the whole header) at the
//! start of `out` and returns the number of bytes written, or 0 when `out` is too small (nothing
//! is written then; every record is longer than 0 bytes, so 0 is unambiguous).
//!
//! # Record layout
//!
//! `A3 95 <type> <payload>`, little-endian, fixed length per type. The payload is the columns of
//! the type in order, each with the size of its format character:
//!
//! | Char | Size | Meaning                                                       |
//! |------|------|---------------------------------------------------------------|
//! | `B`  | 1    | `u8`                                                          |
//! | `H`  | 2    | `u16`                                                         |
//! | `h`  | 2    | `i16`                                                         |
//! | `c`  | 2    | `i16`, value x 100 (readers divide by 100)                    |
//! | `i`  | 4    | `i32`                                                         |
//! | `I`  | 4    | `u32`                                                         |
//! | `e`  | 4    | `i32`, value x 100 (readers divide by 100)                    |
//! | `f`  | 4    | `f32`                                                         |
//! | `L`  | 4    | `i32`, latitude or longitude in 1e-7 degree                   |
//! | `Q`  | 8    | `u64`                                                         |
//! | `n`  | 4    | `char[4]`, NUL padded                                         |
//! | `N`  | 16   | `char[16]`, NUL padded                                        |
//! | `Z`  | 64   | `char[64]`, NUL padded                                        |
//!
//! Text is copied up to its first NUL and truncated to the field size (a full field has no
//! terminator), as ArduPilot's `strncpy` does. There is no `d` (the firmware has no 64-bit float).
//!
//! # The file
//!
//! [`write_header`] writes the FMT of FMT (type 128, length 89), then one FMT for every other
//! type of [`TYPES`]; the start records and the data follow, in any order. Every data type has
//! `TimeUS` (board microseconds, `Q`) as its first column. There is no FMTU, UNIT or MULT.
//!
//! The table [`TYPES`] is the single description of all types: the length of a record is computed
//! from its format string when the table is built (a wrong format character is a compile error),
//! so the length that goes into a FMT record and the length of the encoded record cannot differ
//! (invariant Q3-I1).

pub mod walk;

/// First byte of every record.
pub const HEAD1: u8 = 0xA3;
/// Second byte of every record.
pub const HEAD2: u8 = 0x95;
/// Bytes of the record header: `HEAD1`, `HEAD2`, the type.
pub const HEADER_LEN: usize = 3;

/// Description of one record type: the data of its FMT record.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RecordDef {
    /// Type number (the third byte of every record of this type).
    pub type_id: u8,
    /// Name, at most 4 bytes.
    pub name: &'static str,
    /// One format character per column, at most 16.
    pub format: &'static str,
    /// Column names separated by commas, at most 64 bytes.
    pub columns: &'static str,
    /// Length of a whole record in bytes, header included.
    pub length: u8,
}

/// Size in bytes of the payload column that format character `c` describes; 0 for a character
/// this module does not know.
const fn char_size(c: u8) -> usize {
    match c {
        b'B' => 1,
        b'H' | b'h' | b'c' => 2,
        b'i' | b'I' | b'e' | b'f' | b'L' | b'n' => 4,
        b'Q' => 8,
        b'N' => 16,
        b'Z' => 64,
        _ => 0,
    }
}

/// Length of a record with format string `format`: the header plus its columns, or 0 if a
/// character is unknown.
const fn record_len(format: &str) -> usize {
    let mut rest = format.as_bytes();
    let mut len = HEADER_LEN;
    while let [c, tail @ ..] = rest {
        let size = char_size(*c);
        if size == 0 {
            return 0;
        }
        len += size;
        rest = tail;
    }
    len
}

const fn def(
    type_id: u8,
    name: &'static str,
    format: &'static str,
    columns: &'static str,
) -> RecordDef {
    let len = record_len(format);
    assert!(len > HEADER_LEN && len <= 255, "bad format string");
    assert!(name.len() <= 4, "name over 4 bytes");
    assert!(format.len() <= 16, "format over 16 bytes");
    assert!(columns.len() <= 64, "columns over 64 bytes");
    RecordDef {
        type_id,
        name,
        format,
        columns,
        length: len as u8,
    }
}

/// Type number of the FMT record, as in ArduPilot.
pub const ID_FMT: u8 = 128;
/// Type number of PARM.
pub const ID_PARM: u8 = 129;
/// Type number of MSG.
pub const ID_MSG: u8 = 130;
/// Type number of IMU.
pub const ID_IMU: u8 = 131;
/// Type number of BARO.
pub const ID_BARO: u8 = 132;
/// Type number of MAG.
pub const ID_MAG: u8 = 133;
/// Type number of GPS.
pub const ID_GPS: u8 = 134;
/// Type number of BAT.
pub const ID_BAT: u8 = 135;
/// Type number of RCIN.
pub const ID_RCIN: u8 = 136;
/// Type number of DEV.
pub const ID_DEV: u8 = 137;
/// Type number of DSF.
pub const ID_DSF: u8 = 138;
/// Type number of NAV (flight step).
pub const ID_NAV: u8 = 139;
/// Type number of SETP (flight step).
pub const ID_SETP: u8 = 140;
/// Type number of DMD (flight step).
pub const ID_DMD: u8 = 141;
/// Type number of MOT (flight step).
pub const ID_MOT: u8 = 142;
/// Type number of ESC (ESC telemetry, by DShot line).
pub const ID_ESC: u8 = 143;
/// Type number of RCS (one stick report the flight step consumed).
pub const ID_RCS: u8 = 144;
/// Type number of FDS (the flight ring's statistics).
pub const ID_FDS: u8 = 145;
/// Type number of GPA (the accuracy of a GNSS sample).
pub const ID_GPA: u8 = 146;
/// Type number of NKF1 (an estimator's state; decision-nav-eskf-20261120, 8c).
pub const ID_NKF1: u8 = 147;
/// Type number of NKF2 (an estimator's biases and sigmas).
pub const ID_NKF2: u8 = 148;
/// Type number of NKI (one scalar fusion or reset of an estimator).
pub const ID_NKI: u8 = 149;
/// Type number of MODE (the selected and the effective flight mode; decision-nav-eskf-20261120, 8c, F6).
pub const ID_MODE: u8 = 150;
/// Type number of SPN (the altitude setpoint; decision-nav-eskf-20261120, 8c, F1).
pub const ID_SPN: u8 = 151;
/// Type number of GPV (the velocity of a GNSS sample).
pub const ID_GPV: u8 = 152;
/// Type number of PPS (a whole-second fix paired with its PPS edge: latency measurement).
pub const ID_PPS: u8 = 153;
/// Type number of PIDT (the rate loop's target and P, I, D terms per axis; decision-rpm-notch-20261123, 1B).
pub const ID_PIDT: u8 = 154;
/// Type number of NTCH (the rate the loop used, the four rotor frequencies and the notch switch).
pub const ID_NTCH: u8 = 155;
/// Type number of GDN (the shadow guidance's target, setpoint and range; decision-guidance-shadow-20261123, section 1 item 5).
pub const ID_GDN: u8 = 156;
/// Type number of BFS (a battery level reached; Amendment BATT).
pub const ID_BFS: u8 = 157;

/// FMT: describes a type (89 bytes).
pub const FMT: RecordDef = def(ID_FMT, "FMT", "BBnNZ", "Type,Length,Name,Format,Columns");
/// PARM: one parameter, its value and its default.
pub const PARM: RecordDef = def(ID_PARM, "PARM", "QNff", "TimeUS,Name,Value,Default");
/// MSG: one status text.
pub const MSG: RecordDef = def(ID_MSG, "MSG", "QZ", "TimeUS,Message");
/// IMU: one sample of one IMU (rad/s, m/s^2, degC).
pub const IMU: RecordDef = def(
    ID_IMU,
    "IMU",
    "QBfffffff",
    "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,T",
);
/// BARO: pressure (Pa) and temperature (centi-degC, shown as degC).
pub const BARO: RecordDef = def(ID_BARO, "BARO", "QBfc", "TimeUS,I,Press,Temp");
/// MAG: field in milligauss (microtesla x 10).
pub const MAG: RecordDef = def(ID_MAG, "MAG", "QBhhh", "TimeUS,I,MagX,MagY,MagZ");
/// GPS: one fix.
pub const GPS: RecordDef = def(
    ID_GPS,
    "GPS",
    "QBBIHBcLLeff",
    "TimeUS,I,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs",
);
/// BAT: battery voltage (V) and current (A).
pub const BAT: RecordDef = def(ID_BAT, "BAT", "QBff", "TimeUS,Inst,Volt,Curr");
/// RCIN: 14 receiver channels (us).
pub const RCIN: RecordDef = def(
    ID_RCIN,
    "RCIN",
    "QHHHHHHHHHHHHHH",
    "TimeUS,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14",
);
/// DEV: one row of the device status table.
pub const DEV: RecordDef = def(
    ID_DEV,
    "DEV",
    "QBBBIIIHHH",
    "TimeUS,Id,State,Err,Smp,Errs,TOs,Rec,Att,Ovf",
);
/// DSF: the logger's own statistics.
pub const DSF: RecordDef = def(ID_DSF, "DSF", "QIHIIII", "TimeUS,Dp,Blk,Bytes,FMn,FMx,FAv");
/// NAV: the estimate of one flight step: attitude quaternion (w, x, y, z), body rates (rad/s), valid bits.
pub const NAV: RecordDef = def(
    ID_NAV,
    "NAV",
    "QfffffffB",
    "TimeUS,Q1,Q2,Q3,Q4,GX,GY,GZ,Vld",
);
/// SETP: the setpoint of one flight step: kind, attitude quaternion, rates (rad/s), thrust (N).
pub const SETP: RecordDef = def(
    ID_SETP,
    "SETP",
    "QBffffffff",
    "TimeUS,K,Q1,Q2,Q3,Q4,RX,RY,RZ,Thr",
);
/// DMD: the controller's demand of one flight step: accepted, thrust (N), torques (N.m), saturation bits.
pub const DMD: RecordDef = def(ID_DMD, "DMD", "QBffffB", "TimeUS,Ok,Thr,TX,TY,TZ,Sat");
/// MOT: armed, mode, the allocator's levels and the gate's command (0 = Stop), by DShot line.
pub const MOT: RecordDef = def(
    ID_MOT,
    "MOT",
    "QBBHHHHHHHH",
    "TimeUS,Arm,Mode,A0,A1,A2,A3,C0,C1,C2,C3",
);
/// ESC: raw eRPM, volts (centivolts) and temperature (degC) by DShot line, and flag bits (bit i: line i captured an eRPM since the previous ESC record; bit 4: this exchange sent the flight throttles).
pub const ESC: RecordDef = def(
    ID_ESC,
    "ESC",
    "QIIIIccccBBBBB",
    "TimeUS,E0,E1,E2,E3,V0,V1,V2,V3,T0,T1,T2,T3,Fl",
);
/// RCS: one stick report (roll, pitch, throttle, yaw) and the arm switch; `TimeUS` is the report's own time.
pub const RCS: RecordDef = def(ID_RCS, "RCS", "QffffB", "TimeUS,R,P,T,Y,Arm");
/// FDS: the flight ring's statistics: records dropped since the log opened, smallest free space.
pub const FDS: RecordDef = def(ID_FDS, "FDS", "QII", "TimeUS,Dp,FMn");
/// GPA: the accuracy of one GNSS sample (`TimeUS` is the sample's, as in its GPS record): vertical DOP, horizontal and vertical accuracy in metres, milliseconds since the previous sample.
pub const GPA: RecordDef = def(ID_GPA, "GPA", "QBcffH", "TimeUS,I,VDop,HAcc,VAcc,Delta");
/// GPV: the velocity of one GNSS sample from the receiver's NAV-PVT (`TimeUS` is the sample's, as in its GPS record): velocity north, east and down (mm/s, down positive), the NAV-PVT's iTOW (GPS time of week, ms), ground speed gSpeed (mm/s) and heading headMot (0.01 degree).
pub const GPV: RecordDef = def(
    ID_GPV,
    "GPV",
    "QBiiiIIH",
    "TimeUS,I,VelN,VelE,VelD,GMS,Spd,Hdg",
);
/// PPS: one whole-second GNSS fix paired with the PPS edge of its second (`TimeUS` is the fix's sample time, as in GPV): the edge's board time (us), the latency from the edge to `TimeUS` (us), the PPS edges counted and the unpaired ones counted since boot.
pub const PPS: RecordDef = def(ID_PPS, "PPS", "QQiII", "TimeUS,Edge,Dly,N,Unp");

/// PIDT: the rate loop of one flight step: per axis X, Y, Z the rate target (rad/s) and the P, I and D torques (N.m).
pub const PIDT: RecordDef = def(
    ID_PIDT,
    "PIDT",
    "Qffffffffffff",
    "TimeUS,TarX,PX,IX,DX,TarY,PY,IY,DY,TarZ,PZ,IZ,DZ",
);
/// NTCH: the rate the loop used (rad/s, after the notch) per axis, the rotor frequency f1 (Hz) by DShot line (0: no valid eRPM) and whether the notch is on.
pub const NTCH: RecordDef = def(
    ID_NTCH,
    "NTCH",
    "QfffffffB",
    "TimeUS,GX,GY,GZ,F0,F1,F2,F3,On",
);

/// GDN: the shadow guidance of one step (not a flight-step record: written on its own, every 8th step): the active target's index, the status bits (1 running, 2 arrived, 4 done, 8 invalid), the target and the setpoint position in NE (m), the setpoint velocity in NE (m/s), the distance (m) and the bearing (rad, clockwise from north) from the vehicle to the target.
pub const GDN: RecordDef = def(
    ID_GDN,
    "GDN",
    "QBBffffffff",
    "TimeUS,Idx,St,TN,TE,PN,PE,VN,VE,Dist,Brg",
);

/// BFS: the low-battery monitor reached a level (Amendment BATT): the level (1 LOW, 2 CRITICAL), the critical action (`BATT_FS_CRT_ACT`: 0 warn, 1 descend), the pack voltage (V) and `BATT_CELLS`.
pub const BFS: RecordDef = def(ID_BFS, "BFS", "QBBfB", "TimeUS,Level,Act,Volt,Cells");

/// NKF1: an estimator's state (decision-nav-eskf-20261120, 8c, B5): instance (0 the steering estimator, 1 the shadow), attitude quaternion, velocity and position NED (m/s, m), valid bits as NAV's.
pub const NKF1: RecordDef = def(
    ID_NKF1,
    "NKF1",
    "QBffffffffffB",
    "TimeUS,I,Q1,Q2,Q3,Q4,VN,VE,VD,PN,PE,PD,Vld",
);
/// NKF2: an estimator's gyro bias (rad/s) and accelerometer bias (m/s^2), and the sigmas of tilt, heading (rad), velocity NE and D (m/s), position NE and D (m); `TimeUS` as NKF1's.
pub const NKF2: RecordDef = def(
    ID_NKF2,
    "NKF2",
    "QBffffffffffff",
    "TimeUS,I,GBX,GBY,GBZ,ABX,ABY,ABZ,STlt,SHdg,SVne,SVd,SPne,SPd",
);
/// NKI: one scalar fusion or reset of an estimator: source, innovation (or the applied change), its variance (or the new variance), outcome (codes: decision-nav-eskf-20261120, 8c, B5, C8); `TimeUS` as NKF1's.
pub const NKI: RecordDef = def(ID_NKI, "NKI", "QBBffB", "TimeUS,I,Src,Inn,Var,Ok");

/// MODE: the flight mode the pilot selected and the one in effect, written at the first step and at every change of either.
pub const MODE: RecordDef = def(ID_MODE, "MODE", "QBB", "TimeUS,Sel,Eff");
/// SPN: the altitude-hold setpoint of one flight step: position NED (m), velocity NED (m/s) and heading (rad).
pub const SPN: RecordDef = def(
    ID_SPN,
    "SPN",
    "Qfffffff",
    "TimeUS,PN,PE,PD,VN,VE,VD,Hdg",
);

/// Every record type of the file, FMT first, in the order of the header.
pub const TYPES: [RecordDef; 30] = [
    FMT, PARM, MSG, IMU, BARO, MAG, GPS, BAT, RCIN, DEV, DSF, NAV, SETP, DMD, MOT, ESC, RCS, FDS,
    GPA, GPV, NKF1, NKF2, NKI, MODE, SPN, PPS, PIDT, NTCH, GDN, BFS,
];

/// Size of the header sequence: the FMT of FMT and one FMT for each other type.
pub const HEADER_SEQUENCE_LEN: usize = TYPES.len() * FMT.length as usize;

/// The flight-step types (decision-first-pair-wiring-20261119, A6), the ones `encode_step` writes.
pub const FLIGHT_TYPES: [RecordDef; 6] = [NAV, SETP, DMD, MOT, PIDT, NTCH];

/// Writes one record into the start of a buffer.
struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    /// Starts a record of type `def`: `None` (nothing written) if `buf` is shorter than the record.
    fn start(buf: &'a mut [u8], def: &RecordDef) -> Option<Self> {
        if buf.len() < usize::from(def.length) {
            return None;
        }
        let mut w = Writer { buf, pos: 0 };
        w.bytes(&[HEAD1, HEAD2, def.type_id]);
        Some(w)
    }

    fn bytes(&mut self, src: &[u8]) {
        if let Some(dst) = self.buf.get_mut(self.pos..self.pos + src.len()) {
            dst.copy_from_slice(src);
        }
        self.pos += src.len();
    }

    fn u8(&mut self, v: u8) {
        self.bytes(&[v]);
    }

    fn u16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }

    fn i16(&mut self, v: i16) {
        self.bytes(&v.to_le_bytes());
    }

    fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }

    fn i32(&mut self, v: i32) {
        self.bytes(&v.to_le_bytes());
    }

    fn u64(&mut self, v: u64) {
        self.bytes(&v.to_le_bytes());
    }

    fn f32(&mut self, v: f32) {
        self.bytes(&v.to_le_bytes());
    }

    /// A text field of `size` bytes: `text` up to its first NUL, truncated, then NUL padding.
    fn text(&mut self, text: &[u8], size: usize) {
        let mut ended = false;
        for i in 0..size {
            let c = if ended {
                0
            } else {
                text.get(i).copied().unwrap_or(0)
            };
            ended = c == 0;
            self.u8(c);
        }
    }
}

/// Writes the FMT record that describes `def` (89 bytes). Returns 89, or 0 if `out` is shorter.
pub fn fmt_record(def: &RecordDef, out: &mut [u8]) -> usize {
    let Some(mut w) = Writer::start(out, &FMT) else {
        return 0;
    };
    w.u8(def.type_id);
    w.u8(def.length);
    w.text(def.name.as_bytes(), 4);
    w.text(def.format.as_bytes(), 16);
    w.text(def.columns.as_bytes(), 64);
    w.pos
}

/// Writes the header sequence: the FMT of FMT, then one FMT for each other type of [`TYPES`].
pub fn write_header(out: &mut [u8]) -> usize {
    if out.len() < HEADER_SEQUENCE_LEN {
        return 0;
    }
    let mut at = 0;
    for def in &TYPES {
        let Some(rest) = out.get_mut(at..) else {
            return 0;
        };
        at += fmt_record(def, rest);
    }
    at
}

/// PARM: parameter `name` (at most 16 bytes) with its `value` (an I32 parameter as `f32`) and
/// `default`. 35 bytes.
pub fn encode_parm(out: &mut [u8], time_us: u64, name: &[u8], value: f32, default: f32) -> usize {
    let Some(mut w) = Writer::start(out, &PARM) else {
        return 0;
    };
    w.u64(time_us);
    w.text(name, 16);
    w.f32(value);
    w.f32(default);
    w.pos
}

/// MSG: one status text, truncated to 64 bytes. 75 bytes.
pub fn encode_msg(out: &mut [u8], time_us: u64, text: &[u8]) -> usize {
    let Some(mut w) = Writer::start(out, &MSG) else {
        return 0;
    };
    w.u64(time_us);
    w.text(text, 64);
    w.pos
}

/// IMU: instance `inst` (0 or 1), angular rate in rad/s, acceleration in m/s^2, temperature in degC.
pub fn encode_imu(
    out: &mut [u8],
    time_us: u64,
    inst: u8,
    gyro_rad_s: [f32; 3],
    accel_m_s2: [f32; 3],
    temp_c: f32,
) -> usize {
    let Some(mut w) = Writer::start(out, &IMU) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(inst);
    for v in gyro_rad_s.into_iter().chain(accel_m_s2) {
        w.f32(v);
    }
    w.f32(temp_c);
    w.pos
}

/// `v` x 100 rounded to the nearest integer (halves away from zero), saturating; NaN gives 0.
fn centi(v: f32) -> i32 {
    let scaled = v * 100.0;
    (if scaled < 0.0 {
        scaled - 0.5
    } else {
        scaled + 0.5
    }) as i32
}

/// BARO: instance `inst`, pressure in Pa, temperature in degC (stored as an `i16` of centi-degC,
/// saturating). 18 bytes.
pub fn encode_baro(out: &mut [u8], time_us: u64, inst: u8, press_pa: f32, temp_c: f32) -> usize {
    let Some(mut w) = Writer::start(out, &BARO) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(inst);
    w.f32(press_pa);
    let c = centi(temp_c).clamp(i32::from(i16::MIN), i32::from(i16::MAX));
    w.i16(c as i16);
    w.pos
}

/// MAG: instance `inst`, field in milligauss (microtesla x 10). 18 bytes.
pub fn encode_mag(out: &mut [u8], time_us: u64, inst: u8, field_mgauss: [i16; 3]) -> usize {
    let Some(mut w) = Writer::start(out, &MAG) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(inst);
    for v in field_mgauss {
        w.i16(v);
    }
    w.pos
}

/// The values of one GPS record.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Gps {
    /// Receiver instance.
    pub inst: u8,
    /// 0 none, 1 no fix, 2 2D, 3 3D.
    pub status: u8,
    /// Milliseconds of the GPS week (see [`gps_week_ms`]); 0 before a time is known.
    pub gms: u32,
    /// GPS week; 0 before a time is known.
    pub gwk: u16,
    /// Satellites used.
    pub nsats: u8,
    /// Horizontal DOP x 100 (the column shows DOP; values over 327.67 saturate).
    pub hdop_x100: u16,
    /// Latitude in 1e-7 degree.
    pub lat_e7: i32,
    /// Longitude in 1e-7 degree.
    pub lng_e7: i32,
    /// Altitude above mean sea level in centimetres (the column shows metres).
    pub alt_cm: i32,
    /// Ground speed in m/s.
    pub speed_m_s: f32,
    /// Course over ground in degrees.
    pub course_deg: f32,
}

/// GPS: one fix. 42 bytes.
pub fn encode_gps(out: &mut [u8], time_us: u64, g: &Gps) -> usize {
    let Some(mut w) = Writer::start(out, &GPS) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(g.inst);
    w.u8(g.status);
    w.u32(g.gms);
    w.u16(g.gwk);
    w.u8(g.nsats);
    w.i16(i16::try_from(g.hdop_x100).unwrap_or(i16::MAX));
    w.i32(g.lat_e7);
    w.i32(g.lng_e7);
    w.i32(g.alt_cm);
    w.f32(g.speed_m_s);
    w.f32(g.course_deg);
    w.pos
}

/// BAT: instance `inst`, voltage in V, current in A. 20 bytes.
pub fn encode_bat(out: &mut [u8], time_us: u64, inst: u8, volt: f32, curr: f32) -> usize {
    let Some(mut w) = Writer::start(out, &BAT) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(inst);
    w.f32(volt);
    w.f32(curr);
    w.pos
}

/// RCIN: the 14 channels in microseconds. 39 bytes.
pub fn encode_rcin(out: &mut [u8], time_us: u64, channels_us: &[u16; 14]) -> usize {
    let Some(mut w) = Writer::start(out, &RCIN) else {
        return 0;
    };
    w.u64(time_us);
    for &c in channels_us {
        w.u16(c);
    }
    w.pos
}

/// The values of one DEV record (one row of the device status table).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Dev {
    /// Row id.
    pub id: u8,
    /// Health state.
    pub state: u8,
    /// Error code.
    pub err: u8,
    /// Samples.
    pub samples: u32,
    /// Errors.
    pub errors: u32,
    /// Time-outs.
    pub timeouts: u32,
    /// Recoveries.
    pub recoveries: u16,
    /// Attempts.
    pub attempts: u16,
    /// Overflows.
    pub overflows: u16,
}

/// DEV: one device status row. 32 bytes.
pub fn encode_dev(out: &mut [u8], time_us: u64, d: &Dev) -> usize {
    let Some(mut w) = Writer::start(out, &DEV) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(d.id);
    w.u8(d.state);
    w.u8(d.err);
    w.u32(d.samples);
    w.u32(d.errors);
    w.u32(d.timeouts);
    w.u16(d.recoveries);
    w.u16(d.attempts);
    w.u16(d.overflows);
    w.pos
}

/// The values of one DSF record (the logger's statistics).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Dsf {
    /// Records dropped since the log began.
    pub dropped: u32,
    /// 512-byte blocks written; only the low 16 bits are stored (ArduPilot's `uint16_t`).
    pub blocks: u32,
    /// Bytes written.
    pub bytes: u32,
    /// Smallest free space of the ring over the last second.
    pub free_min: u32,
    /// Largest free space of the ring over the last second.
    pub free_max: u32,
    /// Mean free space of the ring over the last second.
    pub free_avg: u32,
}

/// DSF: the logger's statistics. 33 bytes.
pub fn encode_dsf(out: &mut [u8], time_us: u64, d: &Dsf) -> usize {
    let Some(mut w) = Writer::start(out, &DSF) else {
        return 0;
    };
    w.u64(time_us);
    w.u32(d.dropped);
    w.u16((d.blocks & 0xFFFF) as u16);
    w.u32(d.bytes);
    w.u32(d.free_min);
    w.u32(d.free_max);
    w.u32(d.free_avg);
    w.pos
}

/// NAV: attitude quaternion (w, x, y, z), body rates in rad/s, valid bits. 40 bytes.
pub fn encode_nav(
    out: &mut [u8],
    time_us: u64,
    attitude: [f32; 4],
    rates_rads: [f32; 3],
    valid: u8,
) -> usize {
    let Some(mut w) = Writer::start(out, &NAV) else {
        return 0;
    };
    w.u64(time_us);
    for v in attitude.into_iter().chain(rates_rads) {
        w.f32(v);
    }
    w.u8(valid);
    w.pos
}

/// SETP: setpoint kind, attitude quaternion, rates in rad/s, thrust in N. 44 bytes.
pub fn encode_setp(
    out: &mut [u8],
    time_us: u64,
    kind: u8,
    attitude: [f32; 4],
    rates_rads: [f32; 3],
    thrust_n: f32,
) -> usize {
    let Some(mut w) = Writer::start(out, &SETP) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(kind);
    for v in attitude.into_iter().chain(rates_rads) {
        w.f32(v);
    }
    w.f32(thrust_n);
    w.pos
}

/// DMD: demand accepted, thrust in N, torques in N.m, saturation bits. 29 bytes.
pub fn encode_dmd(
    out: &mut [u8],
    time_us: u64,
    ok: bool,
    thrust_n: f32,
    torque_nm: [f32; 3],
    saturation: u8,
) -> usize {
    let Some(mut w) = Writer::start(out, &DMD) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(u8::from(ok));
    w.f32(thrust_n);
    for v in torque_nm {
        w.f32(v);
    }
    w.u8(saturation);
    w.pos
}

/// MOT: armed, mode number, the allocator's levels and the gate's command levels (0 = Stop), by
/// DShot line. 29 bytes.
pub fn encode_mot(
    out: &mut [u8],
    time_us: u64,
    armed: bool,
    mode: u8,
    levels: [u16; 4],
    command: [u16; 4],
) -> usize {
    let Some(mut w) = Writer::start(out, &MOT) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(u8::from(armed));
    w.u8(mode);
    for v in levels.into_iter().chain(command) {
        w.u16(v);
    }
    w.pos
}

/// PIDT: per axis X, Y, Z the rate target (rad/s) and the P, I and D torques (N.m). 59 bytes.
pub fn encode_pidt(
    out: &mut [u8],
    time_us: u64,
    target_rads: [f32; 3],
    p_nm: [f32; 3],
    i_nm: [f32; 3],
    d_nm: [f32; 3],
) -> usize {
    let Some(mut w) = Writer::start(out, &PIDT) else {
        return 0;
    };
    w.u64(time_us);
    for ((t, p), (i, d)) in target_rads.into_iter().zip(p_nm).zip(i_nm.into_iter().zip(d_nm)) {
        for v in [t, p, i, d] {
            w.f32(v);
        }
    }
    w.pos
}

/// NTCH: the rate the loop used (rad/s) per axis, the rotor frequency f1 (Hz) by DShot line (0: no valid eRPM) and the notch switch.
pub fn encode_ntch(
    out: &mut [u8],
    time_us: u64,
    rate_rads: [f32; 3],
    f1_hz: [f32; 4],
    on: bool,
) -> usize {
    let Some(mut w) = Writer::start(out, &NTCH) else {
        return 0;
    };
    w.u64(time_us);
    for v in rate_rads.into_iter().chain(f1_hz) {
        w.f32(v);
    }
    w.u8(u8::from(on));
    w.pos
}

/// GDN: the active target's index, the status bits, the target (TN, TE) and the setpoint position (PN, PE) in NE (m), the setpoint velocity (VN, VE, m/s), the distance (m) and the bearing (rad) to the target.
#[allow(clippy::too_many_arguments)]
pub fn encode_gdn(
    out: &mut [u8],
    time_us: u64,
    index: u8,
    status: u8,
    target_ne_m: [f32; 2],
    position_ne_m: [f32; 2],
    velocity_ne_ms: [f32; 2],
    dist_m: f32,
    bearing_rad: f32,
) -> usize {
    let Some(mut w) = Writer::start(out, &GDN) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(index);
    w.u8(status);
    for v in target_ne_m.into_iter().chain(position_ne_m).chain(velocity_ne_ms) {
        w.f32(v);
    }
    w.f32(dist_m);
    w.f32(bearing_rad);
    w.pos
}

/// ESC: raw eRPM, volts in centivolts, temperature in degC, by DShot line, and the flag bits (bit i: line i captured an eRPM since the previous ESC record; bit 4: this exchange sent the flight throttles).
pub fn encode_esc(
    out: &mut [u8],
    time_us: u64,
    erpm: [u32; 4],
    volts_cv: [i16; 4],
    temp_c: [u8; 4],
    flags: u8,
) -> usize {
    let Some(mut w) = Writer::start(out, &ESC) else {
        return 0;
    };
    w.u64(time_us);
    for v in erpm {
        w.u32(v);
    }
    for v in volts_cv {
        w.i16(v);
    }
    for v in temp_c {
        w.u8(v);
    }
    w.u8(flags);
    w.pos
}

/// RCS: one stick report, roll, pitch, throttle, yaw, and the arm switch. `time_us` is the
/// report's own time. 28 bytes.
pub fn encode_rcs(out: &mut [u8], time_us: u64, sticks: [f32; 4], armed: bool) -> usize {
    let Some(mut w) = Writer::start(out, &RCS) else {
        return 0;
    };
    w.u64(time_us);
    for v in sticks {
        w.f32(v);
    }
    w.u8(u8::from(armed));
    w.pos
}

/// FDS: flight-ring records dropped since the log opened, and the ring's smallest free space. 19 bytes.
pub fn encode_fds(out: &mut [u8], time_us: u64, dropped: u32, free_min: u32) -> usize {
    let Some(mut w) = Writer::start(out, &FDS) else {
        return 0;
    };
    w.u64(time_us);
    w.u32(dropped);
    w.u32(free_min);
    w.pos
}

/// GPA: the accuracy of one GNSS sample, instance 0.
pub fn encode_gpa(
    out: &mut [u8],
    time_us: u64,
    vdop_x100: u16,
    lat_std_mm: Option<u32>,
    lon_std_mm: Option<u32>,
    alt_std_mm: Option<u32>,
    delta_ms: u32,
) -> usize {
    let Some(mut w) = Writer::start(out, &GPA) else {
        return 0;
    };
    let hacc_m = match (lat_std_mm, lon_std_mm) {
        (Some(lat), Some(lon)) => {
            let (lat, lon) = (lat as f32, lon as f32);
            crate::calib::sqrt(lat * lat + lon * lon) / 1000.0
        }
        _ => f32::NAN,
    };
    let vacc_m = alt_std_mm.map_or(f32::NAN, |a| a as f32 / 1000.0);
    w.u64(time_us);
    w.u8(0);
    w.i16(i16::try_from(vdop_x100).unwrap_or(i16::MAX));
    w.f32(hacc_m);
    w.f32(vacc_m);
    w.u16(u16::try_from(delta_ms).unwrap_or(u16::MAX));
    w.pos
}

/// GPV: the velocity of one GNSS sample, instance 0: north, east, down in mm/s, the GPS time of week in ms (0 when not known), ground speed in mm/s (`u32::MAX` when unknown) and heading in 0.01 degree (`u16::MAX` when unknown), all from the epoch's NAV-PVT.
pub fn encode_gpv(
    out: &mut [u8],
    time_us: u64,
    vel_ned_mm_s: [i32; 3],
    tow_ms: u32,
    speed_mm_s: u32,
    heading_cdeg: u16,
) -> usize {
    let Some(mut w) = Writer::start(out, &GPV) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(0);
    for v in vel_ned_mm_s {
        w.i32(v);
    }
    w.u32(tow_ms);
    w.u32(speed_mm_s);
    w.u16(heading_cdeg);
    w.pos
}

/// PPS: the edge's board time, the latency in us, the edge count and the unpaired count. 31 bytes.
pub fn encode_pps(out: &mut [u8], time_us: u64, edge_us: u64, latency_us: i32, pulses: u32, unpaired: u32) -> usize {
    let Some(mut w) = Writer::start(out, &PPS) else {
        return 0;
    };
    w.u64(time_us);
    w.u64(edge_us);
    w.i32(latency_us);
    w.u32(pulses);
    w.u32(unpaired);
    w.pos
}

/// NKF1: instance, attitude quaternion (w, x, y, z), velocity and position NED, valid bits. 53 bytes.
pub fn encode_nkf1(
    out: &mut [u8],
    time_us: u64,
    inst: u8,
    attitude: [f32; 4],
    velocity_ms: [f32; 3],
    position_m: [f32; 3],
    valid: u8,
) -> usize {
    let Some(mut w) = Writer::start(out, &NKF1) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(inst);
    for v in attitude.into_iter().chain(velocity_ms).chain(position_m) {
        w.f32(v);
    }
    w.u8(valid);
    w.pos
}

/// NKF2: instance, gyro bias, accelerometer bias, then the sigmas tilt, heading, velocity NE, velocity D, position NE, position D.
pub fn encode_nkf2(
    out: &mut [u8],
    time_us: u64,
    inst: u8,
    gyro_bias_rads: [f32; 3],
    accel_bias_ms2: [f32; 3],
    sigmas: [f32; 6],
) -> usize {
    let Some(mut w) = Writer::start(out, &NKF2) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(inst);
    for v in gyro_bias_rads.into_iter().chain(accel_bias_ms2).chain(sigmas) {
        w.f32(v);
    }
    w.pos
}

/// NKI: instance, source code, innovation, variance, outcome code. 22 bytes.
pub fn encode_nki(
    out: &mut [u8],
    time_us: u64,
    inst: u8,
    src: u8,
    inn: f32,
    var: f32,
    ok: u8,
) -> usize {
    let Some(mut w) = Writer::start(out, &NKI) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(inst);
    w.u8(src);
    w.f32(inn);
    w.f32(var);
    w.u8(ok);
    w.pos
}

/// MODE: the selected and the effective mode, codes 0 to 3. 13 bytes.
pub fn encode_mode(out: &mut [u8], time_us: u64, selected: u8, effective: u8) -> usize {
    let Some(mut w) = Writer::start(out, &MODE) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(selected);
    w.u8(effective);
    w.pos
}

/// BFS: the level reached, the critical action, the pack voltage and the cell count. 18 bytes.
pub fn encode_bfs(out: &mut [u8], time_us: u64, level: u8, action: u8, volts: f32, cells: u8) -> usize {
    let Some(mut w) = Writer::start(out, &BFS) else {
        return 0;
    };
    w.u64(time_us);
    w.u8(level);
    w.u8(action);
    w.f32(volts);
    w.u8(cells);
    w.pos
}

/// SPN: position NED, velocity NED, heading. 39 bytes.
pub fn encode_spn(
    out: &mut [u8],
    time_us: u64,
    position_m: [f32; 3],
    velocity_ms: [f32; 3],
    heading_rad: f32,
) -> usize {
    let Some(mut w) = Writer::start(out, &SPN) else {
        return 0;
    };
    w.u64(time_us);
    for v in position_m.into_iter().chain(velocity_ms) {
        w.f32(v);
    }
    w.f32(heading_rad);
    w.pos
}

/// GPA's `delta_ms`: milliseconds from the previous GNSS sample's `t_us` to this one's, 0 for the first (`previous_us` `None`) and for a time that went backwards.
pub fn gpa_delta_ms(previous_us: Option<u64>, t_us: u64) -> u32 {
    previous_us.map_or(0, |p| {
        u32::try_from(t_us.saturating_sub(p) / 1000).unwrap_or(u32::MAX)
    })
}

/// Days from 1970-01-01 to `y-m-d` (proleptic Gregorian, `m` 1 to 12, `d` 1 to 31), after
/// Howard Hinnant's `days_from_civil`.
const fn days_from_civil(y: i64, m: i64, d: i64) -> i64 {
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let mp = (m + 9) % 12;
    let doy = (153 * mp + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}

/// Days from 1970-01-01 to the GPS epoch, 1980-01-06.
const GPS_EPOCH_DAYS: i64 = 3_657;
/// Leap seconds between GPS time and UTC since 2017-01-01 (no leap second is scheduled).
const LEAP_SECONDS: i64 = 18;
const SECONDS_PER_WEEK: i64 = 604_800;

/// GPS week and milliseconds of the week for a UTC date and time, GPS time being UTC plus 18 leap seconds.
#[must_use]
pub fn gps_week_ms(
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: u8,
    millis: u16,
) -> (u16, u32) {
    if !(1..=12).contains(&month)
        || !(1..=31).contains(&day)
        || hour > 23
        || minute > 59
        || second > 60
        || millis > 999
    {
        return (0, 0);
    }
    let days = days_from_civil(i64::from(year), i64::from(month), i64::from(day)) - GPS_EPOCH_DAYS;
    if days < 0 {
        return (0, 0);
    }
    let secs = days * 86_400
        + i64::from(hour) * 3_600
        + i64::from(minute) * 60
        + i64::from(second)
        + LEAP_SECONDS;
    let (Ok(week), Ok(ms)) = (
        u16::try_from(secs / SECONDS_PER_WEEK),
        u32::try_from((secs % SECONDS_PER_WEEK) * 1_000 + i64::from(millis)),
    ) else {
        return (0, 0);
    };
    (week, ms)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn cat(parts: &[&[u8]]) -> Vec<u8> {
        parts.concat()
    }

    /// One encoded sample of every type (type id, bytes), with values that are not special.
    fn one_of_each() -> Vec<(u8, Vec<u8>)> {
        let mut out = Vec::new();
        let mut b = [0u8; 256];
        let g = Gps {
            status: 3,
            ..Gps::default()
        };
        let n = fmt_record(&PARM, &mut b);
        out.push((ID_FMT, b[..n].to_vec()));
        let n = encode_parm(&mut b, 1, b"X", 1.0, 2.0);
        out.push((ID_PARM, b[..n].to_vec()));
        let n = encode_msg(&mut b, 1, b"text");
        out.push((ID_MSG, b[..n].to_vec()));
        let n = encode_imu(&mut b, 1, 1, [1.0; 3], [2.0; 3], 3.0);
        out.push((ID_IMU, b[..n].to_vec()));
        let n = encode_baro(&mut b, 1, 0, 1.0, 2.0);
        out.push((ID_BARO, b[..n].to_vec()));
        let n = encode_mag(&mut b, 1, 0, [1, 2, 3]);
        out.push((ID_MAG, b[..n].to_vec()));
        let n = encode_gps(&mut b, 1, &g);
        out.push((ID_GPS, b[..n].to_vec()));
        let n = encode_bat(&mut b, 1, 0, 1.0, 2.0);
        out.push((ID_BAT, b[..n].to_vec()));
        let n = encode_rcin(&mut b, 1, &[1500; 14]);
        out.push((ID_RCIN, b[..n].to_vec()));
        let n = encode_dev(&mut b, 1, &Dev::default());
        out.push((ID_DEV, b[..n].to_vec()));
        let n = encode_dsf(&mut b, 1, &Dsf::default());
        out.push((ID_DSF, b[..n].to_vec()));
        let n = encode_nav(&mut b, 1, [1.0; 4], [2.0; 3], 3);
        out.push((ID_NAV, b[..n].to_vec()));
        let n = encode_setp(&mut b, 1, 1, [1.0; 4], [2.0; 3], 3.0);
        out.push((ID_SETP, b[..n].to_vec()));
        let n = encode_dmd(&mut b, 1, true, 1.0, [2.0; 3], 3);
        out.push((ID_DMD, b[..n].to_vec()));
        let n = encode_mot(&mut b, 1, true, 1, [1; 4], [2; 4]);
        out.push((ID_MOT, b[..n].to_vec()));
        let n = encode_esc(&mut b, 1, [1; 4], [2; 4], [3; 4], 5);
        out.push((ID_ESC, b[..n].to_vec()));
        let n = encode_rcs(&mut b, 1, [1.0; 4], true);
        out.push((ID_RCS, b[..n].to_vec()));
        let n = encode_fds(&mut b, 1, 1, 2);
        out.push((ID_FDS, b[..n].to_vec()));
        let n = encode_gpa(&mut b, 1, 150, Some(1), Some(2), Some(3), 4);
        out.push((ID_GPA, b[..n].to_vec()));
        let n = encode_gpv(&mut b, 1, [1, -2, 3], 4, 5, 6);
        out.push((ID_GPV, b[..n].to_vec()));
        let n = encode_nkf1(&mut b, 1, 1, [1.0; 4], [2.0; 3], [3.0; 3], 4);
        out.push((ID_NKF1, b[..n].to_vec()));
        let n = encode_nkf2(&mut b, 1, 1, [1.0; 3], [2.0; 3], [3.0; 6]);
        out.push((ID_NKF2, b[..n].to_vec()));
        let n = encode_nki(&mut b, 1, 1, 4, 1.0, 2.0, 1);
        out.push((ID_NKI, b[..n].to_vec()));
        let n = encode_mode(&mut b, 1, 2, 1);
        out.push((ID_MODE, b[..n].to_vec()));
        let n = encode_spn(&mut b, 1, [1.0; 3], [2.0; 3], 3.0);
        out.push((ID_SPN, b[..n].to_vec()));
        let n = encode_pps(&mut b, 1, 2, -3, 4, 5);
        out.push((ID_PPS, b[..n].to_vec()));
        let n = encode_pidt(&mut b, 1, [1.0; 3], [2.0; 3], [3.0; 3], [4.0; 3]);
        out.push((ID_PIDT, b[..n].to_vec()));
        let n = encode_ntch(&mut b, 1, [1.0; 3], [2.0; 4], true);
        out.push((ID_NTCH, b[..n].to_vec()));
        let n = encode_gdn(&mut b, 1, 2, 5, [1.0; 2], [2.0; 2], [3.0; 2], 4.0, 5.0);
        out.push((ID_GDN, b[..n].to_vec()));
        let n = encode_bfs(&mut b, 1, 2, 1, 13.5, 4);
        out.push((ID_BFS, b[..n].to_vec()));
        out
    }

    /// Q3-I1: the length of every encoded record equals the length its FMT announces, and the table is consistent (one column per format character, unique types, distinct names).
    #[test]
    fn q3_i1_record_length_equals_fmt_length() {
        let samples = one_of_each();
        assert_eq!(samples.len(), TYPES.len());
        let mut header = [0u8; HEADER_SEQUENCE_LEN];
        assert_eq!(write_header(&mut header), HEADER_SEQUENCE_LEN);
        for (i, def) in TYPES.iter().enumerate() {
            assert_eq!(samples[i].0, def.type_id);
            let fmt = &header[i * 89..(i + 1) * 89];
            assert_eq!(fmt[3], def.type_id);
            assert_eq!(usize::from(fmt[4]), samples[i].1.len(), "type {}", def.name);
            assert_eq!(usize::from(def.length), samples[i].1.len());
            assert_eq!(samples[i].1[..3], [HEAD1, HEAD2, def.type_id]);
            assert_eq!(
                def.columns.split(',').count(),
                def.format.len(),
                "{}",
                def.name
            );
            if def.type_id != ID_FMT {
                assert!(def.columns.starts_with("TimeUS,"));
                assert!(def.format.starts_with('Q'));
            }
            for other in &TYPES[..i] {
                assert_ne!(other.type_id, def.type_id);
                assert_ne!(other.name, def.name);
            }
        }
        let all: Vec<u8> = samples.iter().flat_map(|(_, b)| b.clone()).collect();
        let mut at = 0;
        for _ in 0..TYPES.len() {
            assert_eq!(all[at..at + 2], [HEAD1, HEAD2]);
            let t = all[at + 2];
            at += usize::from(TYPES.iter().find(|d| d.type_id == t).unwrap().length);
        }
        assert_eq!(at, all.len());
    }

    /// Q3-I2: the FMT of FMT, byte for byte.
    #[test]
    fn q3_i2_fmt_of_fmt_is_byte_exact() {
        let mut want = [0u8; 89];
        want[..5].copy_from_slice(&[0xA3, 0x95, 0x80, 0x80, 0x59]);
        want[5..9].copy_from_slice(&[0x46, 0x4D, 0x54, 0x00]);
        want[9..14].copy_from_slice(&[0x42, 0x42, 0x6E, 0x4E, 0x5A]);
        want[25..56].copy_from_slice(b"Type,Length,Name,Format,Columns");
        let mut got = [0xEEu8; 100];
        assert_eq!(fmt_record(&FMT, &mut got), 89);
        assert_eq!(got[..89], want);
        assert_eq!(
            got[89..],
            [0xEE; 11],
            "nothing beyond the record is touched"
        );
        assert_eq!(fmt_record(&FMT, &mut [0u8; 88]), 0);
    }

    #[test]
    fn header_sequence_is_30_fmt_records() {
        assert_eq!(TYPES.len(), 30);
        assert_eq!(HEADER_SEQUENCE_LEN, 2670);
        let mut buf = [0u8; 2680];
        assert_eq!(write_header(&mut buf), 2670);
        assert_eq!(write_header(&mut [0u8; 2669]), 0);
        for (i, def) in TYPES.iter().enumerate() {
            let rec = &buf[i * 89..(i + 1) * 89];
            assert_eq!(rec[..3], [0xA3, 0x95, 0x80]);
            assert_eq!(rec[3], def.type_id);
            assert_eq!(rec[4], def.length);
        }
        let gps = &buf[6 * 89..7 * 89];
        assert_eq!(gps[3..9], [0x86, 0x2A, b'G', b'P', b'S', 0]);
        assert_eq!(gps[9..25], *b"QBBIHBcLLeff\0\0\0\0");
        let gpv = &buf[19 * 89..20 * 89];
        assert_eq!(gpv[3..9], [0x98, 0x22, b'G', b'P', b'V', 0]);
        assert_eq!(gpv[9..25], *b"QBiiiIIH\0\0\0\0\0\0\0\0");
        assert_eq!(gpv[25..60], *b"TimeUS,I,VelN,VelE,VelD,GMS,Spd,Hdg");
        assert!(gpv[60..].iter().all(|&c| c == 0));
        let nkf1 = &buf[20 * 89..21 * 89];
        assert_eq!(nkf1[3..9], [0x93, 53, b'N', b'K', b'F', b'1']);
        assert_eq!(nkf1[9..25], *b"QBffffffffffB\0\0\0");
        assert_eq!(nkf1[25..67], *b"TimeUS,I,Q1,Q2,Q3,Q4,VN,VE,VD,PN,PE,PD,Vld");
        assert!(nkf1[67..].iter().all(|&c| c == 0));
        let nkf2 = &buf[21 * 89..22 * 89];
        assert_eq!(nkf2[3..9], [0x94, 60, b'N', b'K', b'F', b'2']);
        assert_eq!(nkf2[9..25], *b"QBffffffffffff\0\0");
        assert_eq!(nkf2[25..85], *b"TimeUS,I,GBX,GBY,GBZ,ABX,ABY,ABZ,STlt,SHdg,SVne,SVd,SPne,SPd");
        assert!(nkf2[85..].iter().all(|&c| c == 0));
        let nki = &buf[22 * 89..23 * 89];
        assert_eq!(nki[3..9], [0x95, 22, b'N', b'K', b'I', 0]);
        assert_eq!(nki[9..25], *b"QBBffB\0\0\0\0\0\0\0\0\0\0");
        assert_eq!(nki[25..48], *b"TimeUS,I,Src,Inn,Var,Ok");
        assert!(nki[48..].iter().all(|&c| c == 0));
        let mode = &buf[23 * 89..24 * 89];
        assert_eq!(mode[3..9], [0x96, 13, b'M', b'O', b'D', b'E']);
        assert_eq!(mode[9..25], *b"QBB\0\0\0\0\0\0\0\0\0\0\0\0\0");
        assert_eq!(mode[25..39], *b"TimeUS,Sel,Eff");
        assert!(mode[39..].iter().all(|&c| c == 0));
        let spn = &buf[24 * 89..25 * 89];
        assert_eq!(spn[3..9], [0x97, 39, b'S', b'P', b'N', 0]);
        assert_eq!(spn[9..25], *b"Qfffffff\0\0\0\0\0\0\0\0");
        assert_eq!(spn[25..53], *b"TimeUS,PN,PE,PD,VN,VE,VD,Hdg");
        assert!(spn[53..].iter().all(|&c| c == 0));
        let pidt = &buf[26 * 89..27 * 89];
        assert_eq!(pidt[3..9], [0x9A, 59, b'P', b'I', b'D', b'T']);
        assert_eq!(pidt[9..25], *b"Qffffffffffff   ");
        assert_eq!(pidt[25..73], *b"TimeUS,TarX,PX,IX,DX,TarY,PY,IY,DY,TarZ,PZ,IZ,DZ");
        assert!(pidt[73..].iter().all(|&c| c == 0));
        let ntch = &buf[27 * 89..28 * 89];
        assert_eq!(ntch[3..9], [0x9B, 40, b'N', b'T', b'C', b'H']);
        assert_eq!(ntch[9..25], *b"QfffffffB       ");
        assert_eq!(ntch[25..55], *b"TimeUS,GX,GY,GZ,F0,F1,F2,F3,On");
        assert!(ntch[55..].iter().all(|&c| c == 0));
        let gdn = &buf[28 * 89..29 * 89];
        assert_eq!(gdn[3..9], [0x9C, 45, b'G', b'D', b'N', 0]);
        assert_eq!(gdn[9..25], *b"QBBffffffff\0\0\0\0\0");
        assert_eq!(gdn[25..65], *b"TimeUS,Idx,St,TN,TE,PN,PE,VN,VE,Dist,Brg");
        assert!(gdn[65..].iter().all(|&c| c == 0));
    }

    /// The table, as the file announces it (also printed into the report by hand).
    #[test]
    fn table_lengths() {
        let lens: Vec<(&str, u8)> = TYPES.iter().map(|d| (d.name, d.length)).collect();
        assert_eq!(
            lens,
            [
                ("FMT", 89),
                ("PARM", 35),
                ("MSG", 75),
                ("IMU", 40),
                ("BARO", 18),
                ("MAG", 18),
                ("GPS", 42),
                ("BAT", 20),
                ("RCIN", 39),
                ("DEV", 32),
                ("DSF", 33),
                ("NAV", 40),
                ("SETP", 44),
                ("DMD", 29),
                ("MOT", 29),
                ("ESC", 40),
                ("RCS", 28),
                ("FDS", 19),
                ("GPA", 24),
                ("GPV", 34),
                ("NKF1", 53),
                ("NKF2", 60),
                ("NKI", 22),
                ("MODE", 13),
                ("SPN", 39),
                ("PPS", 31),
                ("PIDT", 59),
                ("NTCH", 40),
                ("GDN", 45),
                ("BFS", 18)
            ]
        );
    }

    #[test]
    fn golden_parm() {
        let mut b = [0u8; 35];
        assert_eq!(encode_parm(&mut b, 1, b"A", 1.0, 2.0), 35);
        let want = cat(&[
            &[0xA3, 0x95, 0x81],
            &[1, 0, 0, 0, 0, 0, 0, 0],
            &[0x41],
            &[0; 15],
            &[0x00, 0x00, 0x80, 0x3F],
            &[0x00, 0x00, 0x00, 0x40],
        ]);
        assert_eq!(b.as_slice(), want);
    }

    #[test]
    fn golden_msg_and_text_rules() {
        let mut b = [0u8; 75];
        assert_eq!(encode_msg(&mut b, 2, b"Hi"), 75);
        assert_eq!(
            b.as_slice(),
            cat(&[
                &[0xA3, 0x95, 0x82, 2, 0, 0, 0, 0, 0, 0, 0, 0x48, 0x69],
                &[0; 62]
            ])
        );
        assert_eq!(encode_msg(&mut b, 2, &[b'x'; 70]), 75);
        assert_eq!(b[11..], [b'x'; 64]);
        assert_eq!(encode_msg(&mut b, 2, b"a\0b"), 75);
        assert_eq!(b[11..14], [b'a', 0, 0]);
        let mut p = [0u8; 35];
        encode_parm(&mut p, 0, &[b'N'; 20], 0.0, 0.0);
        assert_eq!(p[11..27], [b'N'; 16]);
        assert_eq!(p[27..31], [0; 4]);
    }

    #[test]
    fn golden_imu() {
        let mut b = [0u8; 40];
        assert_eq!(
            encode_imu(&mut b, 3, 1, [1.0, 2.0, -1.0], [0.5, 3.0, 4.0], 2.0),
            40
        );
        let want = cat(&[
            &[0xA3, 0x95, 0x83, 3, 0, 0, 0, 0, 0, 0, 0, 1],
            &[0, 0, 0x80, 0x3F, 0, 0, 0, 0x40, 0, 0, 0x80, 0xBF],
            &[0, 0, 0, 0x3F, 0, 0, 0x40, 0x40, 0, 0, 0x80, 0x40],
            &[0, 0, 0, 0x40],
        ]);
        assert_eq!(b.as_slice(), want);
    }

    #[test]
    fn golden_baro() {
        let mut b = [0u8; 18];
        assert_eq!(encode_baro(&mut b, 4, 0, 100_000.0, 25.5), 18);
        assert_eq!(
            b,
            [
                0xA3, 0x95, 0x84, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x50, 0xC3, 0x47, 0xF6, 0x09
            ]
        );
        encode_baro(&mut b, 4, 0, 0.0, -0.005);
        assert_eq!(b[16..], [0xFF, 0xFF]);
        encode_baro(&mut b, 4, 0, 0.0, 1000.0);
        assert_eq!(b[16..], [0xFF, 0x7F]);
    }

    #[test]
    fn golden_mag() {
        let mut b = [0u8; 18];
        assert_eq!(encode_mag(&mut b, 5, 1, [1000, -1, -1000]), 18);
        assert_eq!(
            b,
            [
                0xA3, 0x95, 0x85, 5, 0, 0, 0, 0, 0, 0, 0, 1, 0xE8, 0x03, 0xFF, 0xFF, 0x18, 0xFC
            ]
        );
    }

    #[test]
    fn golden_gps() {
        let g = Gps {
            inst: 0,
            status: 3,
            gms: 18_000,
            gwk: 2445,
            nsats: 12,
            hdop_x100: 95,
            lat_e7: 0x0102_0304,
            lng_e7: -1,
            alt_cm: 123_456,
            speed_m_s: 1.0,
            course_deg: 2.0,
        };
        let mut b = [0u8; 42];
        assert_eq!(encode_gps(&mut b, 6, &g), 42);
        let want = cat(&[
            &[0xA3, 0x95, 0x86, 6, 0, 0, 0, 0, 0, 0, 0, 0, 3],
            &[0x50, 0x46, 0, 0, 0x8D, 0x09, 12, 0x5F, 0],
            &[4, 3, 2, 1, 0xFF, 0xFF, 0xFF, 0xFF, 0x40, 0xE2, 1, 0],
            &[0, 0, 0x80, 0x3F, 0, 0, 0, 0x40],
        ]);
        assert_eq!(b.as_slice(), want);
        let mut g2 = g;
        g2.hdop_x100 = 60_000;
        encode_gps(&mut b, 6, &g2);
        assert_eq!(b[20..22], [0xFF, 0x7F]);
    }

    #[test]
    fn golden_bat() {
        let mut b = [0u8; 20];
        assert_eq!(encode_bat(&mut b, 7, 0, 3.0, 0.5), 20);
        assert_eq!(
            b,
            [
                0xA3, 0x95, 0x87, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x40, 0x40, 0, 0, 0, 0x3F
            ]
        );
    }

    #[test]
    fn golden_rcin() {
        let mut ch = [0u16; 14];
        ch[0] = 1500;
        ch[1] = 1000;
        let mut b = [0u8; 39];
        assert_eq!(encode_rcin(&mut b, 8, &ch), 39);
        let want = cat(&[
            &[
                0xA3, 0x95, 0x88, 8, 0, 0, 0, 0, 0, 0, 0, 0xDC, 0x05, 0xE8, 0x03,
            ],
            &[0; 24],
        ]);
        assert_eq!(b.as_slice(), want);
    }

    #[test]
    fn golden_dev() {
        let d = Dev {
            id: 7,
            state: 2,
            err: 1,
            samples: 0x0102_0304,
            errors: 5,
            timeouts: 6,
            recoveries: 7,
            attempts: 8,
            overflows: 9,
        };
        let mut b = [0u8; 32];
        assert_eq!(encode_dev(&mut b, 9, &d), 32);
        assert_eq!(
            b,
            [
                0xA3, 0x95, 0x89, 9, 0, 0, 0, 0, 0, 0, 0, 7, 2, 1, 4, 3, 2, 1, 5, 0, 0, 0, 6, 0, 0,
                0, 7, 0, 8, 0, 9, 0
            ]
        );
    }

    #[test]
    fn golden_dsf() {
        let d = Dsf {
            dropped: 1,
            blocks: 0x1_0001,
            bytes: 3,
            free_min: 4,
            free_max: 5,
            free_avg: 6,
        };
        let mut b = [0u8; 33];
        assert_eq!(encode_dsf(&mut b, 10, &d), 33);
        assert_eq!(
            b,
            [
                0xA3, 0x95, 0x8A, 10, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 3, 0, 0, 0, 4, 0, 0,
                0, 5, 0, 0, 0, 6, 0, 0, 0
            ]
        );
    }

    #[test]
    fn golden_gpa() {
        let mut b = [0u8; 24];
        assert_eq!(
            encode_gpa(&mut b, 7, 150, Some(3000), Some(4000), Some(2500), 200),
            24
        );
        assert_eq!(
            b,
            [
                0xA3, 0x95, 0x92, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0x96, 0x00, 0x00, 0x00, 0xA0, 0x40,
                0x00, 0x00, 0x20, 0x40, 0xC8, 0x00
            ]
        );
        assert_eq!(GPA.format, "QBcffH");
        assert_eq!(GPA.columns, "TimeUS,I,VDop,HAcc,VAcc,Delta");
        assert_eq!(ID_GPA, 146);
    }

    #[test]
    fn golden_gpv() {
        let mut b = [0xEEu8; 40];
        assert_eq!(encode_gpv(&mut b, 7, [-12, 34, -1], 542_618_800, 450, 25_118), 34);
        let want = cat(&[
            &[0xA3, 0x95, 0x98],
            &7u64.to_le_bytes(),
            &[0],
            &(-12i32).to_le_bytes(),
            &34i32.to_le_bytes(),
            &(-1i32).to_le_bytes(),
            &542_618_800u32.to_le_bytes(),
            &450u32.to_le_bytes(),
            &25_118u16.to_le_bytes(),
        ]);
        assert_eq!(b[..34], want[..]);
        assert_eq!(b[34..], [0xEE; 6], "nothing beyond the record is touched");
        assert_eq!(b[12..16], [0xF4, 0xFF, 0xFF, 0xFF]);
        assert_eq!(GPV.format, "QBiiiIIH");
        assert_eq!(GPV.columns, "TimeUS,I,VelN,VelE,VelD,GMS,Spd,Hdg");
        assert_eq!(ID_GPV, 152);
        assert_eq!(encode_gpv(&mut [0u8; 33], 7, [0; 3], 0, 0, 0), 0);
    }

    #[test]
    fn golden_nkf1() {
        let mut b = [0xEEu8; 60];
        let n = encode_nkf1(&mut b, 7, 1, [1.0, 0.0, 0.0, -0.5], [0.25, -2.0, 0.0], [3.0, -4.0, 1.5], 0x55);
        assert_eq!(n, 53);
        let floats: [f32; 10] = [1.0, 0.0, 0.0, -0.5, 0.25, -2.0, 0.0, 3.0, -4.0, 1.5];
        let mut want = cat(&[&[0xA3, 0x95, 0x93], &7u64.to_le_bytes(), &[1]]);
        for f in floats {
            want.extend_from_slice(&f.to_le_bytes());
        }
        want.push(0x55);
        assert_eq!(b[..53], want[..]);
        assert_eq!(b[53..], [0xEE; 7], "nothing beyond the record is touched");
        assert_eq!(b[24..28], [0x00, 0x00, 0x00, 0xBF]);
        assert_eq!(NKF1.format, "QBffffffffffB");
        assert_eq!(NKF1.columns, "TimeUS,I,Q1,Q2,Q3,Q4,VN,VE,VD,PN,PE,PD,Vld");
        assert_eq!(ID_NKF1, 147);
        assert_eq!(encode_nkf1(&mut [0u8; 52], 7, 1, [0.0; 4], [0.0; 3], [0.0; 3], 0), 0);
    }

    #[test]
    fn golden_nkf2() {
        let mut b = [0xEEu8; 64];
        let inf = f32::INFINITY;
        let n = encode_nkf2(&mut b, 7, 0, [0.0; 3], [0.0, 0.0, 0.125], [inf, inf, inf, 0.5, inf, 2.0]);
        assert_eq!(n, 60);
        let floats: [f32; 12] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.125, inf, inf, inf, 0.5, inf, 2.0];
        let mut want = cat(&[&[0xA3, 0x95, 0x94], &7u64.to_le_bytes(), &[0]]);
        for f in floats {
            want.extend_from_slice(&f.to_le_bytes());
        }
        assert_eq!(b[..60], want[..]);
        assert_eq!(b[60..], [0xEE; 4], "nothing beyond the record is touched");
        assert_eq!(b[36..40], [0x00, 0x00, 0x80, 0x7F]);
        assert_eq!(NKF2.format, "QBffffffffffff");
        assert_eq!(NKF2.columns, "TimeUS,I,GBX,GBY,GBZ,ABX,ABY,ABZ,STlt,SHdg,SVne,SVd,SPne,SPd");
        assert_eq!(ID_NKF2, 148);
        assert_eq!(encode_nkf2(&mut [0u8; 59], 7, 0, [0.0; 3], [0.0; 3], [0.0; 6]), 0);
    }

    #[test]
    fn golden_mode_and_spn() {
        let mut b = [0u8; 13];
        assert_eq!(encode_mode(&mut b, 7, 2, 1), 13);
        assert_eq!(b, [0xA3, 0x95, 0x96, 7, 0, 0, 0, 0, 0, 0, 0, 2, 1]);
        assert_eq!(encode_mode(&mut [0u8; 12], 7, 2, 1), 0);
        let mut b = [0u8; 39];
        assert_eq!(
            encode_spn(&mut b, 7, [1.0, 2.0, -1.5], [0.0, 0.0, 0.5], -2.0),
            39
        );
        let want = cat(&[
            &[0xA3, 0x95, 0x97],
            &[7, 0, 0, 0, 0, 0, 0, 0],
            &[0x00, 0x00, 0x80, 0x3F],
            &[0x00, 0x00, 0x00, 0x40],
            &[0x00, 0x00, 0xC0, 0xBF],
            &[0x00, 0x00, 0x00, 0x00],
            &[0x00, 0x00, 0x00, 0x00],
            &[0x00, 0x00, 0x00, 0x3F],
            &[0x00, 0x00, 0x00, 0xC0],
        ]);
        assert_eq!(b.as_slice(), want);
        assert_eq!(encode_spn(&mut [0u8; 38], 7, [0.0; 3], [0.0; 3], 0.0), 0);
        assert_eq!((ID_MODE, ID_SPN), (150, 151));
        assert_eq!((MODE.format, MODE.columns), ("QBB", "TimeUS,Sel,Eff"));
        assert_eq!(
            (SPN.format, SPN.columns),
            ("Qfffffff", "TimeUS,PN,PE,PD,VN,VE,VD,Hdg")
        );
    }

    #[test]
    fn golden_pidt_and_ntch() {
        let mut b = [0xEEu8; 70];
        assert_eq!(
            encode_pidt(
                &mut b,
                7,
                [1.0, 5.0, 9.0],
                [2.0, 6.0, 10.0],
                [3.0, 7.0, 11.0],
                [4.0, 8.0, 12.0]
            ),
            59
        );
        let mut want = cat(&[&[0xA3, 0x95, 0x9A], &7u64.to_le_bytes()]);
        for v in 1..=12 {
            want.extend_from_slice(&(v as f32).to_le_bytes());
        }
        assert_eq!(b[..59], want[..]);
        assert_eq!(b[59..], [0xEE; 11], "nothing beyond the record is touched");
        assert_eq!(encode_pidt(&mut [0u8; 58], 7, [0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]), 0);
        let mut b = [0xEEu8; 50];
        assert_eq!(encode_ntch(&mut b, 7, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0, 7.0], true), 40);
        let mut want = cat(&[&[0xA3, 0x95, 0x9B], &7u64.to_le_bytes()]);
        for v in 1..=7 {
            want.extend_from_slice(&(v as f32).to_le_bytes());
        }
        want.push(1);
        assert_eq!(b[..40], want[..]);
        assert_eq!(b[40..], [0xEE; 10], "nothing beyond the record is touched");
        assert_eq!(encode_ntch(&mut [0u8; 39], 7, [0.0; 3], [0.0; 4], false), 0);
        assert_eq!((ID_PIDT, ID_NTCH), (154, 155));
        assert_eq!(PIDT.format, "Qffffffffffff");
        assert_eq!(NTCH.format, "QfffffffB");
    }

    #[test]
    fn golden_gdn() {
        let mut b = [0xEEu8; 60];
        assert_eq!(
            encode_gdn(&mut b, 7, 2, 5, [1.0, 2.0], [3.0, 4.0], [5.0, 6.0], 7.0, 8.0),
            45
        );
        let mut want = cat(&[&[0xA3, 0x95, 0x9C], &7u64.to_le_bytes(), &[2, 5]]);
        for v in 1..=8 {
            want.extend_from_slice(&(v as f32).to_le_bytes());
        }
        assert_eq!(b[..45], want[..]);
        assert_eq!(b[45..], [0xEE; 15], "nothing beyond the record is touched");
        assert_eq!(encode_gdn(&mut [0u8; 44], 7, 0, 0, [0.0; 2], [0.0; 2], [0.0; 2], 0.0, 0.0), 0);
        assert_eq!(ID_GDN, 156);
        assert_eq!(GDN.format, "QBBffffffff");
        assert!(!FLIGHT_TYPES.iter().any(|d| d.type_id == ID_GDN), "not part of the per-step block");
    }

    /// BFS by hand: type 157 = 0x9D, length 18; Level 2, Act 1, Volt 13.5 V, Cells 4. Not part of the per-step block.
    #[test]
    fn golden_bfs() {
        let mut b = [0xEEu8; 20];
        assert_eq!(encode_bfs(&mut b, 7, 2, 1, 13.5, 4), 18);
        let want = cat(&[&[0xA3, 0x95, 0x9D], &7u64.to_le_bytes(), &[2, 1], &13.5f32.to_le_bytes(), &[4]]);
        assert_eq!(b[..18], want[..]);
        assert_eq!(b[18..], [0xEE; 2], "nothing beyond the record is touched");
        assert_eq!(encode_bfs(&mut [0u8; 17], 7, 2, 1, 13.5, 4), 0);
        assert_eq!((ID_BFS, BFS.format, BFS.length), (157, "QBBfB", 18));
        assert!(!FLIGHT_TYPES.iter().any(|d| d.type_id == ID_BFS), "not part of the per-step block");
    }

    #[test]
    fn golden_nki() {
        let mut b = [0xEEu8; 30];
        assert_eq!(encode_nki(&mut b, 7, 1, 6, -1.0, 0.25, 1), 22);
        let want = cat(&[
            &[0xA3, 0x95, 0x95],
            &7u64.to_le_bytes(),
            &[1, 6],
            &[0x00, 0x00, 0x80, 0xBF],
            &[0x00, 0x00, 0x80, 0x3E],
            &[1],
        ]);
        assert_eq!(b[..22], want[..]);
        assert_eq!(b[22..], [0xEE; 8], "nothing beyond the record is touched");
        assert_eq!(NKI.format, "QBBffB");
        assert_eq!(NKI.columns, "TimeUS,I,Src,Inn,Var,Ok");
        assert_eq!(ID_NKI, 149);
        assert_eq!(encode_nki(&mut [0u8; 21], 7, 1, 6, -1.0, 0.25, 1), 0);
    }

    #[test]
    fn gpa_unknown_values_and_saturation() {
        let f = |b: &[u8], at: usize| f32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]]);
        let mut b = [0u8; 24];
        assert_eq!(encode_gpa(&mut b, 1, u16::MAX, None, None, None, 0), 24);
        assert_eq!(b[12..14], 32_767i16.to_le_bytes());
        assert!(f(&b, 14).is_nan() && f(&b, 18).is_nan());
        encode_gpa(&mut b, 1, 0, Some(3000), None, Some(1500), 0);
        assert!(f(&b, 14).is_nan());
        assert_eq!(f(&b, 18), 1.5);
        encode_gpa(&mut b, 1, 0, None, Some(3000), None, 0);
        assert!(f(&b, 14).is_nan() && f(&b, 18).is_nan());
        encode_gpa(&mut b, 1, 0, Some(3000), Some(4000), None, 0);
        assert!((f(&b, 14) - 5.0).abs() < 1e-6);
        encode_gpa(&mut b, 1, 0, None, None, None, 65_535);
        assert_eq!(b[22..24], [0xFF, 0xFF]);
        encode_gpa(&mut b, 1, 0, None, None, None, 70_000);
        assert_eq!(b[22..24], [0xFF, 0xFF]);
        assert_eq!(encode_gpa(&mut [0x55u8; 23], 1, 0, None, None, None, 0), 0);
    }

    #[test]
    fn gpa_delta_is_ms_since_the_previous_sample_and_0_for_the_first() {
        assert_eq!(gpa_delta_ms(None, 5_000_000), 0);
        assert_eq!(gpa_delta_ms(Some(5_000_000), 5_200_000), 200);
        assert_eq!(gpa_delta_ms(Some(5_000_000), 5_100_999), 100);
        assert_eq!(gpa_delta_ms(Some(5_000_000), 6_000_000), 1000);
        assert_eq!(gpa_delta_ms(Some(5_000_000), 4_000_000), 0);
        assert_eq!(gpa_delta_ms(Some(0), u64::MAX), u32::MAX);
        let mut b = [0u8; 24];
        encode_gpa(
            &mut b,
            1,
            0,
            None,
            None,
            None,
            gpa_delta_ms(Some(0), 70_000_000),
        );
        assert_eq!(b[22..24], [0xFF, 0xFF]);
    }

    #[test]
    fn new_encoders_write_their_table_length_and_nothing_into_a_short_buffer() {
        let mut b = [0u8; 64];
        assert_eq!(encode_esc(&mut b, 1, [1; 4], [2; 4], [3; 4], 5), 40);
        assert_eq!(encode_rcs(&mut b, 1, [1.0; 4], true), 28);
        assert_eq!(encode_fds(&mut b, 1, 1, 2), 19);
        let mut e = [0x55u8; 39];
        assert_eq!(encode_esc(&mut e, 1, [1; 4], [2; 4], [3; 4], 5), 0);
        assert_eq!(e, [0x55; 39]);
        let mut r = [0x55u8; 27];
        assert_eq!(encode_rcs(&mut r, 1, [1.0; 4], true), 0);
        assert_eq!(r, [0x55; 27]);
        let mut f = [0x55u8; 18];
        assert_eq!(encode_fds(&mut f, 1, 1, 2), 0);
        assert_eq!(f, [0x55; 18]);
        assert_eq!(encode_esc(&mut [], 1, [0; 4], [0; 4], [0; 4], 0), 0);
    }

    #[test]
    fn too_small_buffer_writes_nothing() {
        let mut b = [0x55u8; 17];
        assert_eq!(encode_baro(&mut b, 1, 0, 1.0, 1.0), 0);
        assert_eq!(b, [0x55; 17]);
        assert_eq!(encode_rcin(&mut [0u8; 38], 1, &[0; 14]), 0);
        assert_eq!(encode_dsf(&mut [], 1, &Dsf::default()), 0);
    }

    #[test]
    fn gps_week_known_dates() {
        assert_eq!(gps_week_ms(1980, 1, 6, 0, 0, 0, 0), (0, 18_000));
        assert_eq!(gps_week_ms(2026, 11, 15, 0, 0, 0, 0), (2445, 18_000));
        assert_eq!(
            gps_week_ms(2026, 11, 15, 12, 34, 56, 789),
            (2445, 45_314_789)
        );
        assert_eq!(gps_week_ms(2026, 11, 14, 23, 59, 42, 0), (2445, 0));
        assert_eq!(
            gps_week_ms(2026, 11, 14, 23, 59, 41, 999),
            (2444, 604_799_999)
        );
        let (w1, m1) = gps_week_ms(2024, 2, 29, 0, 0, 0, 0);
        let (w2, m2) = gps_week_ms(2024, 3, 1, 0, 0, 0, 0);
        assert_eq!(
            (i64::from(w2) - i64::from(w1)) * 604_800_000 + i64::from(m2) - i64::from(m1),
            86_400_000
        );
    }

    #[test]
    fn gps_week_no_time() {
        assert_eq!(gps_week_ms(1980, 1, 5, 23, 59, 59, 999), (0, 0));
        assert_eq!(gps_week_ms(2026, 0, 15, 0, 0, 0, 0), (0, 0));
        assert_eq!(gps_week_ms(2026, 13, 15, 0, 0, 0, 0), (0, 0));
        assert_eq!(gps_week_ms(2026, 11, 0, 0, 0, 0, 0), (0, 0));
        assert_eq!(gps_week_ms(2026, 11, 15, 24, 0, 0, 0), (0, 0));
        assert_eq!(gps_week_ms(2026, 11, 15, 0, 60, 0, 0), (0, 0));
        assert_eq!(gps_week_ms(2026, 11, 15, 0, 0, 0, 1000), (0, 0));
        assert_eq!(gps_week_ms(9999, 12, 31, 0, 0, 0, 0), (0, 0));
    }
}
