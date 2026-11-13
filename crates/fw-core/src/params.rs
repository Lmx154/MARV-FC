//! The parameter table, its metadata and the `ParamStore` implementation.
//!
//! One static [`ParamDef`] per parameter; [`Params`] holds the values and decides whether a write
//! is accepted. Everything is polled: after writes the firmware takes [`Params::dirty`], the two
//! per-device masks and the last [`Notice`]; nothing is called back. The table itself (the real
//! parameters) belongs to the device milestones; this module only checks and stores it.

use mavlink_device::{ParamError, ParamName, ParamStore, ParamValue};

/// Most entries a table can have (the capacity of the flash record).
pub const MAX_ENTRIES: usize = 169;

/// Highest device id a definition may carry; the per-device masks are `u32` (bit = device id).
pub const MAX_DEVICE: u8 = 31;

/// How an accepted write takes effect.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Apply {
    /// Through the device's `configure`: the device's whole candidate configuration is validated
    /// before the value is stored, and the device's APPLY request is raised.
    Configure,
    /// Only after a reboot.
    Reboot,
    /// Read by the running code on its next use (calibration offsets, for example).
    Live,
}

/// One parameter's static definition.
#[derive(Clone, Copy, Debug)]
pub struct ParamDef {
    /// `<PREFIX>_<NAME>`, at most 16 characters of `A-Z`, `0-9`, `_`.
    pub name: &'static str,
    /// The `MARV_DEVICE` id the parameter belongs to (1 IMU1, 2 IMU2, 3 BARO, 4 MAG, 5 GNSS,
    /// 6 BATT, 7 RC); 0 is the system, no device. At most [`MAX_DEVICE`].
    pub device: u8,
    /// The value after "all to defaults" and the value the flash record omits.
    pub default: ParamValue,
    /// Smallest accepted value; the same type as `default`.
    pub min: ParamValue,
    /// Largest accepted value; the same type as `default`.
    pub max: ParamValue,
    /// Writes are refused.
    pub read_only: bool,
    /// A calibration result: "calibration to defaults" resets it.
    pub calibration: bool,
    /// How an accepted write takes effect.
    pub apply: Apply,
}

impl ParamDef {
    /// An `I32` definition, writable, not a calibration.
    pub const fn i32(
        name: &'static str,
        device: u8,
        default: i32,
        min: i32,
        max: i32,
        apply: Apply,
    ) -> Self {
        Self {
            name,
            device,
            default: ParamValue::I32(default),
            min: ParamValue::I32(min),
            max: ParamValue::I32(max),
            read_only: false,
            calibration: false,
            apply,
        }
    }

    /// An `F32` definition, writable, not a calibration.
    pub const fn f32(
        name: &'static str,
        device: u8,
        default: f32,
        min: f32,
        max: f32,
        apply: Apply,
    ) -> Self {
        Self {
            name,
            device,
            default: ParamValue::F32(default),
            min: ParamValue::F32(min),
            max: ParamValue::F32(max),
            read_only: false,
            calibration: false,
            apply,
        }
    }

    /// The same definition, read only.
    pub const fn with_read_only(mut self) -> Self {
        self.read_only = true;
        self
    }

    /// The same definition, marked as a calibration result.
    pub const fn with_calibration(mut self) -> Self {
        self.calibration = true;
        self
    }
}

/// What is wrong with a table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TableErrorKind {
    /// The name is not a valid [`ParamName`].
    InvalidName,
    /// An earlier entry has the same name.
    DuplicateName,
    /// `default`, `min` and `max` are not all of one type.
    MixedTypes,
    /// An `F32` bound (or default) is NaN or infinite.
    NonFiniteBound,
    /// `min` is above `max`.
    MinAboveMax,
    /// `default` is outside `[min, max]`.
    DefaultOutOfRange,
    /// The device id is above [`MAX_DEVICE`].
    DeviceTooHigh,
    /// More than [`MAX_ENTRIES`] entries; the error's index is the first one over.
    TooManyEntries,
    /// More entries than the store's const-generic capacity; the index is the first one over.
    StorageTooSmall,
}

/// A table that [`Params::new`] refused, and the entry at fault.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TableError {
    /// Position of the entry in the table.
    pub index: u16,
    /// What is wrong with it.
    pub kind: TableErrorKind,
}

/// Why a write was refused, as a notice for the ground station.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoticeKind {
    /// The parameter is read only.
    ReadOnly,
    /// The value has the other type.
    WrongType,
    /// Outside `[min, max]`, or not finite.
    OutOfRange,
    /// The device's validator refused the candidate configuration, with its text.
    Rejected(&'static str),
    /// An accepted `Reboot`-class change.
    AppliesAfterReboot,
}

impl NoticeKind {
    /// The text for a STATUSTEXT.
    pub const fn text(self) -> &'static str {
        match self {
            Self::ReadOnly => "read only",
            Self::WrongType => "wrong type",
            Self::OutOfRange => "out of range",
            Self::Rejected(reason) => reason,
            Self::AppliesAfterReboot => "applies after reboot",
        }
    }
}

/// The last thing worth a STATUSTEXT: which parameter and why.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Notice {
    /// Index of the parameter.
    pub index: u16,
    /// Name of the parameter.
    pub name: ParamName,
    /// What happened.
    pub kind: NoticeKind,
}

/// Why [`Params::load`] did not apply a value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadError {
    /// No parameter has this name.
    UnknownName,
    /// The value has the other type.
    WrongType,
    /// The value is outside `[min, max]` or not finite.
    OutOfRange,
}

/// A device's whole configuration as it would be after a write: every parameter reads as stored,
/// except the one being written, which reads as the proposed value.
#[derive(Clone, Copy, Debug)]
pub struct Candidate<'a> {
    defs: &'a [ParamDef],
    values: &'a [ParamValue],
    index: usize,
    proposed: ParamValue,
}

impl Candidate<'_> {
    /// The value of parameter `name` in the candidate; `None` if there is no such parameter.
    pub fn get(&self, name: &str) -> Option<ParamValue> {
        let i = self
            .defs
            .iter()
            .position(|d| d.name.as_bytes() == name.as_bytes())?;
        if i == self.index {
            Some(self.proposed)
        } else {
            self.values.get(i).copied()
        }
    }

    /// [`Self::get`] when the parameter exists and is an `I32`.
    pub fn i32(&self, name: &str) -> Option<i32> {
        match self.get(name)? {
            ParamValue::I32(v) => Some(v),
            ParamValue::F32(_) => None,
        }
    }

    /// [`Self::get`] when the parameter exists and is an `F32`.
    pub fn f32(&self, name: &str) -> Option<f32> {
        match self.get(name)? {
            ParamValue::F32(v) => Some(v),
            ParamValue::I32(_) => None,
        }
    }
}

/// A device's pure validator: `Ok` to accept the candidate configuration, or the reason to refuse.
pub type Validator = fn(&Candidate<'_>) -> Result<(), &'static str>;

/// Bit-for-bit equality (`-0.0` differs from `0.0`; the same NaN equals itself).
fn bits_eq(a: ParamValue, b: ParamValue) -> bool {
    match (a, b) {
        (ParamValue::I32(x), ParamValue::I32(y)) => x == y,
        (ParamValue::F32(x), ParamValue::F32(y)) => x.to_bits() == y.to_bits(),
        _ => false,
    }
}

/// `None` if `v` has another type than the definition; otherwise whether it is finite and in range.
fn in_range(def: &ParamDef, v: ParamValue) -> Option<bool> {
    match (v, def.min, def.max) {
        (ParamValue::I32(x), ParamValue::I32(lo), ParamValue::I32(hi)) => Some(x >= lo && x <= hi),
        (ParamValue::F32(x), ParamValue::F32(lo), ParamValue::F32(hi)) => {
            Some(x.is_finite() && x >= lo && x <= hi)
        }
        _ => None,
    }
}

fn device_bit(device: u8) -> u32 {
    1u32.checked_shl(u32::from(device)).unwrap_or(0)
}

fn check_entry(defs: &[ParamDef], i: usize, def: &ParamDef) -> Result<(), TableErrorKind> {
    use TableErrorKind as K;
    if ParamName::new(def.name).is_err() {
        return Err(K::InvalidName);
    }
    let earlier = defs.get(..i).unwrap_or(&[]);
    if earlier
        .iter()
        .any(|d| d.name.as_bytes() == def.name.as_bytes())
    {
        return Err(K::DuplicateName);
    }
    if def.device > MAX_DEVICE {
        return Err(K::DeviceTooHigh);
    }
    match (def.default, def.min, def.max) {
        (ParamValue::I32(d), ParamValue::I32(lo), ParamValue::I32(hi)) => {
            if lo > hi {
                return Err(K::MinAboveMax);
            }
            if d < lo || d > hi {
                return Err(K::DefaultOutOfRange);
            }
        }
        (ParamValue::F32(d), ParamValue::F32(lo), ParamValue::F32(hi)) => {
            if !lo.is_finite() || !hi.is_finite() || !d.is_finite() {
                return Err(K::NonFiniteBound);
            }
            if lo > hi {
                return Err(K::MinAboveMax);
            }
            if d < lo || d > hi {
                return Err(K::DefaultOutOfRange);
            }
        }
        _ => return Err(K::MixedTypes),
    }
    Ok(())
}

/// The parameter store: a validated static table, `N` value slots, and the polled state
/// (dirty flag, per-device masks, last notice). `N` must be at least the table's length.
pub struct Params<const N: usize> {
    defs: &'static [ParamDef],
    values: [ParamValue; N],
    validators: [Option<Validator>; 32],
    dirty: bool,
    configure_mask: u32,
    live_mask: u32,
    notice: Option<Notice>,
}

impl<const N: usize> Params<N> {
    /// Validates `defs` and builds a store holding every default.
    pub fn new(defs: &'static [ParamDef]) -> Result<Self, TableError> {
        let fail = |i: usize, kind| TableError {
            index: u16::try_from(i).unwrap_or(u16::MAX),
            kind,
        };
        if defs.len() > MAX_ENTRIES {
            return Err(fail(MAX_ENTRIES, TableErrorKind::TooManyEntries));
        }
        if defs.len() > N {
            return Err(fail(N, TableErrorKind::StorageTooSmall));
        }
        for (i, def) in defs.iter().enumerate() {
            check_entry(defs, i, def).map_err(|kind| fail(i, kind))?;
        }
        let mut values = [ParamValue::I32(0); N];
        for (slot, def) in values.iter_mut().zip(defs) {
            *slot = def.default;
        }
        Ok(Self {
            defs,
            values,
            validators: [None; 32],
            dirty: false,
            configure_mask: 0,
            live_mask: 0,
            notice: None,
        })
    }

    /// Installs the validator run before a `Configure`-class write of any parameter of `device`.
    /// Returns `false` (and installs nothing) if `device` is above [`MAX_DEVICE`].
    pub fn set_validator(&mut self, device: u8, validator: Validator) -> bool {
        match self.validators.get_mut(usize::from(device)) {
            Some(slot) => {
                *slot = Some(validator);
                true
            }
            None => false,
        }
    }

    /// The stored value of the parameter called `name`, for a device's start.
    pub fn get_named(&self, name: &str) -> Option<ParamValue> {
        let i = self
            .defs
            .iter()
            .position(|d| d.name.as_bytes() == name.as_bytes())?;
        self.values.get(i).copied()
    }

    /// Whether an accepted change (or a reset that changed a value) is not yet saved.
    pub fn dirty(&self) -> bool {
        self.dirty
    }

    /// Marks the store saved.
    pub fn clear_dirty(&mut self) {
        self.dirty = false;
    }

    /// Devices (bit = device id) with an accepted `Configure`-class change since the last call;
    /// clears the mask. The firmware raises each device's APPLY request.
    pub fn take_configure_mask(&mut self) -> u32 {
        core::mem::take(&mut self.configure_mask)
    }

    /// Devices (bit = device id) with an accepted `Live`-class change since the last call;
    /// clears the mask.
    pub fn take_live_mask(&mut self) -> u32 {
        core::mem::take(&mut self.live_mask)
    }

    /// The last notice (one slot: a newer one replaces an older one); clears it.
    pub fn take_notice(&mut self) -> Option<Notice> {
        self.notice.take()
    }

    /// The values that differ bit-for-bit from their default, in table order: what the flash
    /// record stores.
    pub fn non_default(&self) -> impl Iterator<Item = (ParamName, ParamValue)> + '_ {
        self.defs
            .iter()
            .zip(self.values.iter())
            .filter(|(def, value)| !bits_eq(**value, def.default))
            .filter_map(|(def, value)| Some((ParamName::new(def.name).ok()?, *value)))
    }

    /// Boot-time load of one stored value: applied if the name exists, the type matches and the
    /// value is in range. `Ok(true)` applied, `Ok(false)` skipped because the parameter is read
    /// only. Marks nothing dirty, requests no apply, runs no validator.
    pub fn load(&mut self, name: &ParamName, value: ParamValue) -> Result<bool, LoadError> {
        let i = self
            .defs
            .iter()
            .position(|d| d.name.as_bytes() == name.as_bytes())
            .ok_or(LoadError::UnknownName)?;
        let (Some(def), Some(slot)) = (self.defs.get(i), self.values.get_mut(i)) else {
            return Err(LoadError::UnknownName);
        };
        match in_range(def, value) {
            None => Err(LoadError::WrongType),
            Some(false) => Err(LoadError::OutOfRange),
            Some(true) if def.read_only => Ok(false),
            Some(true) => {
                *slot = value;
                Ok(true)
            }
        }
    }

    /// MAV_CMD_PREFLIGHT_STORAGE "all to defaults". Returns the number of values that changed.
    /// A change marks the store dirty and sets the masks of the devices whose values changed;
    /// the validators are not run (defaults are valid by construction of the table).
    pub fn reset_all(&mut self) -> u16 {
        self.reset(false)
    }

    /// "Calibration to defaults": as [`Self::reset_all`], for the `calibration` parameters only.
    pub fn reset_calibration(&mut self) -> u16 {
        self.reset(true)
    }

    fn reset(&mut self, calibration_only: bool) -> u16 {
        let mut changed = 0u16;
        for (i, (def, value)) in self.defs.iter().zip(self.values.iter_mut()).enumerate() {
            if (calibration_only && !def.calibration) || bits_eq(*value, def.default) {
                continue;
            }
            *value = def.default;
            changed = changed.saturating_add(1);
            match def.apply {
                Apply::Configure => self.configure_mask |= device_bit(def.device),
                Apply::Live => self.live_mask |= device_bit(def.device),
                Apply::Reboot => {
                    self.notice =
                        Self::notice_for(def, i, NoticeKind::AppliesAfterReboot).or(self.notice);
                }
            }
        }
        if changed > 0 {
            self.dirty = true;
        }
        changed
    }

    fn notice_for(def: &ParamDef, index: usize, kind: NoticeKind) -> Option<Notice> {
        Some(Notice {
            index: u16::try_from(index).ok()?,
            name: ParamName::new(def.name).ok()?,
            kind,
        })
    }

    fn refuse(&mut self, index: usize, kind: NoticeKind, error: ParamError) -> ParamError {
        if let Some(def) = self.defs.get(index) {
            self.notice = Self::notice_for(def, index, kind).or(self.notice);
        }
        error
    }
}

impl<const N: usize> ParamStore for Params<N> {
    fn count(&self) -> u16 {
        u16::try_from(self.defs.len()).unwrap_or(u16::MAX)
    }

    fn name(&self, i: u16) -> Option<ParamName> {
        ParamName::new(self.defs.get(usize::from(i))?.name).ok()
    }

    fn get(&self, i: u16) -> Option<ParamValue> {
        self.defs.get(usize::from(i))?;
        self.values.get(usize::from(i)).copied()
    }

    fn find(&self, n: &ParamName) -> Option<u16> {
        let i = self
            .defs
            .iter()
            .position(|d| d.name.as_bytes() == n.as_bytes())?;
        u16::try_from(i).ok()
    }

    fn set(&mut self, i: u16, v: ParamValue) -> Result<ParamValue, ParamError> {
        let index = usize::from(i);
        let defs = self.defs;
        let (Some(def), Some(current)) = (defs.get(index), self.values.get(index).copied()) else {
            return Err(ParamError::NotFound);
        };
        if def.read_only {
            return Err(self.refuse(index, NoticeKind::ReadOnly, ParamError::Rejected));
        }
        match in_range(def, v) {
            None => return Err(self.refuse(index, NoticeKind::WrongType, ParamError::WrongType)),
            Some(false) => {
                return Err(self.refuse(index, NoticeKind::OutOfRange, ParamError::OutOfRange));
            }
            Some(true) => {}
        }
        if bits_eq(current, v) {
            return Ok(v);
        }
        if def.apply == Apply::Configure {
            let validator = self
                .validators
                .get(usize::from(def.device))
                .copied()
                .flatten();
            if let Some(validate) = validator {
                let candidate = Candidate {
                    defs,
                    values: &self.values,
                    index,
                    proposed: v,
                };
                if let Err(reason) = validate(&candidate) {
                    return Err(self.refuse(
                        index,
                        NoticeKind::Rejected(reason),
                        ParamError::Rejected,
                    ));
                }
            }
        }
        if let Some(slot) = self.values.get_mut(index) {
            *slot = v;
        }
        self.dirty = true;
        match def.apply {
            Apply::Configure => self.configure_mask |= device_bit(def.device),
            Apply::Live => self.live_mask |= device_bit(def.device),
            Apply::Reboot => {
                self.notice =
                    Self::notice_for(def, index, NoticeKind::AppliesAfterReboot).or(self.notice);
            }
        }
        Ok(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::boxed::Box;
    use std::format;
    use std::vec::Vec;

    const TABLE: &[ParamDef] = &[
        ParamDef::i32("SYS_BAUD", 0, 115_200, 9_600, 921_600, Apply::Reboot), // 0
        ParamDef::i32("SYS_ID", 0, 1, 1, 250, Apply::Live),                   // 1
        ParamDef::i32("SYS_VER", 0, 7, 0, 100, Apply::Live).with_read_only(), // 2
        ParamDef::i32("IMU1_ODR", 1, 1000, 100, 8000, Apply::Configure),      // 3
        ParamDef::i32("IMU1_LPF", 1, 200, 0, 4000, Apply::Configure),         // 4
        ParamDef::f32("IMU1_GYR_OFFX", 1, 0.0, -1.0, 1.0, Apply::Live).with_calibration(), // 5
        ParamDef::f32("IMU1_GYR_OFFY", 1, 0.0, -1.0, 1.0, Apply::Live).with_calibration(), // 6
        ParamDef::i32("IMU2_ODR", 2, 1000, 100, 8000, Apply::Configure),      // 7
        ParamDef::f32("IMU2_RANGE", 2, 16.0, 2.0, 32.0, Apply::Configure),    // 8
        ParamDef::f32("IMU2_ACC_OFFX", 2, 0.0, -2.0, 2.0, Apply::Live).with_calibration(), // 9
        ParamDef::i32("IMU1_MODE", 1, 0, 0, 3, Apply::Reboot),                // 10
    ];

    const BAUD: u16 = 0;
    const ID: u16 = 1;
    const VER: u16 = 2;
    const ODR1: u16 = 3;
    const LPF1: u16 = 4;
    const OFFX: u16 = 5;
    const OFFY: u16 = 6;
    const ODR2: u16 = 7;
    const RANGE2: u16 = 8;
    const ACC2: u16 = 9;
    const MODE1: u16 = 10;

    fn v_imu1(c: &Candidate<'_>) -> Result<(), &'static str> {
        match (c.i32("IMU1_ODR"), c.i32("IMU1_LPF")) {
            (Some(odr), Some(lpf)) if lpf > odr / 2 => Err("IMU1 LPF above half the ODR"),
            _ => Ok(()),
        }
    }

    fn store() -> Params<16> {
        let mut p = Params::<16>::new(TABLE).unwrap();
        assert!(p.set_validator(1, v_imu1));
        p
    }

    fn name(s: &str) -> ParamName {
        ParamName::new(s).unwrap()
    }

    fn assert_quiet(p: &mut Params<16>) {
        assert!(!p.dirty());
        assert_eq!(p.take_configure_mask(), 0);
        assert_eq!(p.take_live_mask(), 0);
    }

    // ---- table validation ----

    fn table_err<const N: usize>(defs: &'static [ParamDef]) -> TableError {
        Params::<N>::new(defs).err().unwrap()
    }

    #[test]
    fn valid_table_constructs() {
        let p = store();
        assert_eq!(p.count(), 11);
        assert_eq!(p.name(3), Some(name("IMU1_ODR")));
        assert_eq!(p.get(0), Some(ParamValue::I32(115_200)));
        assert_eq!(p.find(&name("IMU2_RANGE")), Some(RANGE2));
        assert_eq!(p.find(&name("NOPE")), None);
        assert_eq!(p.name(11), None);
        assert_eq!(p.get(11), None);
        assert!(!p.dirty());
    }

    #[test]
    fn table_invalid_name() {
        const T: &[ParamDef] = &[
            ParamDef::i32("A", 0, 0, 0, 1, Apply::Live),
            ParamDef::i32("bad", 0, 0, 0, 1, Apply::Live),
        ];
        assert_eq!(
            table_err::<4>(T),
            TableError {
                index: 1,
                kind: TableErrorKind::InvalidName
            }
        );
        const LONG: &[ParamDef] = &[ParamDef::i32("ABCDEFGHIJKLMNOPQ", 0, 0, 0, 1, Apply::Live)];
        assert_eq!(table_err::<4>(LONG).kind, TableErrorKind::InvalidName);
    }

    #[test]
    fn table_duplicate_name() {
        const T: &[ParamDef] = &[
            ParamDef::i32("A", 0, 0, 0, 1, Apply::Live),
            ParamDef::i32("B", 0, 0, 0, 1, Apply::Live),
            ParamDef::f32("A", 0, 0.0, 0.0, 1.0, Apply::Live),
        ];
        assert_eq!(
            table_err::<4>(T),
            TableError {
                index: 2,
                kind: TableErrorKind::DuplicateName
            }
        );
    }

    #[test]
    fn table_mixed_types() {
        const T: &[ParamDef] = &[ParamDef {
            name: "A",
            device: 0,
            default: ParamValue::I32(0),
            min: ParamValue::F32(0.0),
            max: ParamValue::I32(1),
            read_only: false,
            calibration: false,
            apply: Apply::Live,
        }];
        assert_eq!(
            table_err::<4>(T),
            TableError {
                index: 0,
                kind: TableErrorKind::MixedTypes
            }
        );
    }

    #[test]
    fn table_min_above_max() {
        const T: &[ParamDef] = &[ParamDef::i32("A", 0, 5, 9, 1, Apply::Live)];
        assert_eq!(table_err::<4>(T).kind, TableErrorKind::MinAboveMax);
        const F: &[ParamDef] = &[ParamDef::f32("A", 0, 0.0, 2.0, 1.0, Apply::Live)];
        assert_eq!(table_err::<4>(F).kind, TableErrorKind::MinAboveMax);
    }

    #[test]
    fn table_default_out_of_range() {
        const T: &[ParamDef] = &[ParamDef::i32("A", 0, 10, 0, 5, Apply::Live)];
        assert_eq!(table_err::<4>(T).kind, TableErrorKind::DefaultOutOfRange);
        const F: &[ParamDef] = &[ParamDef::f32("A", 0, -0.5, 0.0, 1.0, Apply::Live)];
        assert_eq!(table_err::<4>(F).kind, TableErrorKind::DefaultOutOfRange);
    }

    #[test]
    fn table_non_finite_bound() {
        const A: &[ParamDef] = &[ParamDef::f32(
            "A",
            0,
            0.0,
            f32::NEG_INFINITY,
            1.0,
            Apply::Live,
        )];
        const B: &[ParamDef] = &[ParamDef::f32("A", 0, 0.0, 0.0, f32::NAN, Apply::Live)];
        const C: &[ParamDef] = &[ParamDef::f32("A", 0, f32::NAN, 0.0, 1.0, Apply::Live)];
        for t in [A, B, C] {
            assert_eq!(table_err::<4>(t).kind, TableErrorKind::NonFiniteBound);
        }
    }

    #[test]
    fn table_device_too_high() {
        const T: &[ParamDef] = &[
            ParamDef::i32("A", 31, 0, 0, 1, Apply::Live),
            ParamDef::i32("B", 32, 0, 0, 1, Apply::Live),
        ];
        assert_eq!(
            table_err::<4>(T),
            TableError {
                index: 1,
                kind: TableErrorKind::DeviceTooHigh
            }
        );
    }

    fn big_table(n: usize) -> &'static [ParamDef] {
        let defs: Vec<ParamDef> = (0..n)
            .map(|i| {
                let name: &'static str = Box::leak(format!("P{i}").into_boxed_str());
                ParamDef::i32(name, 0, 0, 0, 10, Apply::Live)
            })
            .collect();
        Box::leak(defs.into_boxed_slice())
    }

    #[test]
    fn table_too_many_entries() {
        let ok = Params::<169>::new(big_table(169)).unwrap();
        assert_eq!(ok.count(), 169);
        assert_eq!(
            table_err::<200>(big_table(170)),
            TableError {
                index: 169,
                kind: TableErrorKind::TooManyEntries
            }
        );
    }

    #[test]
    fn table_storage_too_small() {
        assert_eq!(
            table_err::<3>(TABLE),
            TableError {
                index: 3,
                kind: TableErrorKind::StorageTooSmall
            }
        );
    }

    #[test]
    fn set_validator_rejects_device_above_31() {
        let mut p = store();
        assert!(!p.set_validator(32, v_imu1));
    }

    // ---- set: refusals ----

    fn refused(p: &mut Params<16>, i: u16, v: ParamValue, err: ParamError, kind: NoticeKind) {
        let before = p.get(i);
        let _ = p.take_notice();
        assert_eq!(p.set(i, v), Err(err), "{i} {v:?}");
        assert_eq!(p.get(i), before);
        assert_quiet(p);
        let n = p.take_notice().unwrap();
        assert_eq!(n.index, i);
        assert_eq!(n.name, p.name(i).unwrap());
        assert_eq!(n.kind, kind);
        assert_eq!(p.take_notice(), None);
    }

    #[test]
    fn set_refusals_change_nothing() {
        let mut p = store();
        use NoticeKind as K;
        refused(
            &mut p,
            VER,
            ParamValue::I32(8),
            ParamError::Rejected,
            K::ReadOnly,
        );
        refused(
            &mut p,
            ODR1,
            ParamValue::F32(1000.0),
            ParamError::WrongType,
            K::WrongType,
        );
        refused(
            &mut p,
            OFFX,
            ParamValue::I32(0),
            ParamError::WrongType,
            K::WrongType,
        );
        refused(
            &mut p,
            ID,
            ParamValue::I32(0),
            ParamError::OutOfRange,
            K::OutOfRange,
        );
        refused(
            &mut p,
            ID,
            ParamValue::I32(251),
            ParamError::OutOfRange,
            K::OutOfRange,
        );
        refused(
            &mut p,
            OFFX,
            ParamValue::F32(-1.01),
            ParamError::OutOfRange,
            K::OutOfRange,
        );
        refused(
            &mut p,
            OFFX,
            ParamValue::F32(1.01),
            ParamError::OutOfRange,
            K::OutOfRange,
        );
        refused(
            &mut p,
            OFFX,
            ParamValue::F32(f32::NAN),
            ParamError::OutOfRange,
            K::OutOfRange,
        );
        refused(
            &mut p,
            OFFX,
            ParamValue::F32(f32::INFINITY),
            ParamError::OutOfRange,
            K::OutOfRange,
        );
        refused(
            &mut p,
            OFFX,
            ParamValue::F32(f32::NEG_INFINITY),
            ParamError::OutOfRange,
            K::OutOfRange,
        );
        refused(
            &mut p,
            LPF1,
            ParamValue::I32(600),
            ParamError::Rejected,
            K::Rejected("IMU1 LPF above half the ODR"),
        );
    }

    #[test]
    fn refusals_leave_other_state_alone() {
        let mut p = store();
        p.set(ODR1, ParamValue::I32(2000)).unwrap();
        // dirty and mask of an earlier accepted write survive a later refusal
        assert_eq!(p.set(ID, ParamValue::I32(0)), Err(ParamError::OutOfRange));
        assert!(p.dirty());
        assert_eq!(p.take_configure_mask(), 1 << 1);
        assert_eq!(p.take_live_mask(), 0);
    }

    #[test]
    fn set_unknown_index() {
        let mut p = store();
        assert_eq!(p.set(11, ParamValue::I32(1)), Err(ParamError::NotFound));
        assert_eq!(
            p.set(u16::MAX, ParamValue::I32(1)),
            Err(ParamError::NotFound)
        );
        assert_quiet(&mut p);
        assert_eq!(p.take_notice(), None);
    }

    // ---- set: accepted (I3.1) ----

    #[test]
    fn echo_equals_stored_on_every_accepted_path() {
        let mut p = store();
        let cases: &[(u16, ParamValue)] = &[
            (BAUD, ParamValue::I32(9_600)),
            (BAUD, ParamValue::I32(921_600)),
            (ID, ParamValue::I32(1_i32 + 1)),
            (LPF1, ParamValue::I32(0)),
            (ODR1, ParamValue::I32(100)),
            (ODR1, ParamValue::I32(8000)),
            (OFFX, ParamValue::F32(-0.0)),
            (OFFX, ParamValue::F32(-1.0)),
            (OFFX, ParamValue::F32(1.0)),
            (OFFX, ParamValue::F32(0.1)),
            (RANGE2, ParamValue::F32(2.0)),
            (RANGE2, ParamValue::F32(32.0)),
            (ACC2, ParamValue::F32(f32::MIN_POSITIVE / 2.0)),
            (MODE1, ParamValue::I32(3)),
        ];
        for &(i, v) in cases {
            let echoed = p.set(i, v).unwrap();
            let stored = p.get(i).unwrap();
            match (echoed, stored, v) {
                (ParamValue::F32(a), ParamValue::F32(b), ParamValue::F32(c)) => {
                    assert_eq!(a.to_bits(), b.to_bits());
                    assert_eq!(a.to_bits(), c.to_bits());
                }
                (a, b, c) => assert!(a == b && a == c),
            }
        }
        assert!(p.dirty());
    }

    #[test]
    fn negative_zero_over_zero_is_a_change() {
        let mut p = store();
        p.set(OFFX, ParamValue::F32(-0.0)).unwrap();
        assert!(p.dirty());
        assert_eq!(p.take_live_mask(), 1 << 1);
        let ParamValue::F32(x) = p.get(OFFX).unwrap() else {
            panic!()
        };
        assert_eq!(x.to_bits(), (-0.0f32).to_bits());
    }

    #[test]
    fn same_value_write_is_a_noop() {
        let mut p = store();
        for (i, v) in [
            (BAUD, ParamValue::I32(115_200)),
            (ID, ParamValue::I32(1)),
            (ODR1, ParamValue::I32(1000)),
            (OFFX, ParamValue::F32(0.0)),
            (MODE1, ParamValue::I32(0)),
        ] {
            assert_eq!(p.set(i, v), Ok(v));
            assert_quiet(&mut p);
            assert_eq!(p.take_notice(), None);
        }
        // a no-op after a real change is a no-op too
        p.set(ODR1, ParamValue::I32(2000)).unwrap();
        p.clear_dirty();
        let _ = p.take_configure_mask();
        assert_eq!(
            p.set(ODR1, ParamValue::I32(2000)),
            Ok(ParamValue::I32(2000))
        );
        assert_quiet(&mut p);
    }

    // ---- APPLY exactly once per accepted CONFIGURE set ----

    #[test]
    fn configure_set_raises_exactly_that_device() {
        let mut p = store();
        assert_eq!(
            p.set(ODR1, ParamValue::I32(2000)),
            Ok(ParamValue::I32(2000))
        );
        assert!(p.dirty());
        assert_eq!(p.take_live_mask(), 0);
        assert_eq!(p.take_configure_mask(), 1 << 1);
        assert_eq!(p.take_configure_mask(), 0);
        assert_eq!(p.take_notice(), None);
    }

    #[test]
    fn configure_bit_stays_until_taken() {
        let mut p = store();
        p.set(ODR1, ParamValue::I32(2000)).unwrap();
        p.set(LPF1, ParamValue::I32(100)).unwrap();
        assert_eq!(p.take_configure_mask(), 1 << 1);
        assert_eq!(p.take_configure_mask(), 0);
    }

    #[test]
    fn refused_and_noop_configure_set_nothing() {
        let mut p = store();
        assert!(p.set(ODR1, ParamValue::I32(99)).is_err());
        assert!(p.set(ODR1, ParamValue::F32(500.0)).is_err());
        assert!(p.set(LPF1, ParamValue::I32(600)).is_err());
        assert!(p.set(ODR1, ParamValue::I32(1000)).is_ok());
        assert_quiet(&mut p);
    }

    #[test]
    fn two_devices_two_bits() {
        let mut p = store();
        p.set(ODR1, ParamValue::I32(2000)).unwrap();
        p.set(RANGE2, ParamValue::F32(8.0)).unwrap();
        assert_eq!(p.take_configure_mask(), (1 << 1) | (1 << 2));
        assert_eq!(p.take_live_mask(), 0);
    }

    #[test]
    fn live_set_raises_only_the_live_mask() {
        let mut p = store();
        p.set(OFFY, ParamValue::F32(0.25)).unwrap();
        p.set(ID, ParamValue::I32(7)).unwrap();
        assert!(p.dirty());
        assert_eq!(p.take_configure_mask(), 0);
        assert_eq!(p.take_live_mask(), 1 | (1 << 1));
        assert_eq!(p.take_notice(), None);
    }

    #[test]
    fn reboot_set_raises_no_mask_and_leaves_the_notice() {
        let mut p = store();
        p.set(BAUD, ParamValue::I32(57_600)).unwrap();
        assert!(p.dirty());
        assert_eq!(p.take_configure_mask(), 0);
        assert_eq!(p.take_live_mask(), 0);
        let n = p.take_notice().unwrap();
        assert_eq!(n.kind, NoticeKind::AppliesAfterReboot);
        assert_eq!(n.kind.text(), "applies after reboot");
        assert_eq!(n.name.as_str(), "SYS_BAUD");
        assert_eq!(n.index, BAUD);
        assert_eq!(p.take_notice(), None);
    }

    #[test]
    fn notice_texts() {
        assert_eq!(NoticeKind::ReadOnly.text(), "read only");
        assert_eq!(NoticeKind::WrongType.text(), "wrong type");
        assert_eq!(NoticeKind::OutOfRange.text(), "out of range");
        assert_eq!(NoticeKind::Rejected("why").text(), "why");
    }

    // ---- the validator sees the candidate ----

    #[test]
    fn validator_sees_the_whole_candidate() {
        let mut p = store();
        p.set(LPF1, ParamValue::I32(400)).unwrap(); // 400 <= 1000 / 2
        let _ = p.take_configure_mask();
        p.clear_dirty();
        let r = p.set(ODR1, ParamValue::I32(700)); // 400 > 350
        assert_eq!(r, Err(ParamError::Rejected));
        assert_eq!(p.get(ODR1), Some(ParamValue::I32(1000)));
        assert_eq!(
            p.take_notice().unwrap().kind,
            NoticeKind::Rejected("IMU1 LPF above half the ODR")
        );
        assert_quiet(&mut p);
        p.set(LPF1, ParamValue::I32(200)).unwrap(); // changed back
        let _ = p.take_configure_mask();
        assert_eq!(p.set(ODR1, ParamValue::I32(700)), Ok(ParamValue::I32(700)));
        assert_eq!(p.take_configure_mask(), 1 << 1);
    }

    #[test]
    fn device_without_validator_accepts() {
        let mut p = store();
        assert!(p.set(ODR2, ParamValue::I32(100)).is_ok());
        assert_eq!(p.take_configure_mask(), 1 << 2);
    }

    #[test]
    fn validator_not_run_for_live_or_reboot() {
        fn never(_: &Candidate<'_>) -> Result<(), &'static str> {
            Err("never")
        }
        let mut p = Params::<16>::new(TABLE).unwrap();
        p.set_validator(1, never);
        p.set_validator(0, never);
        assert!(p.set(OFFX, ParamValue::F32(0.5)).is_ok());
        assert!(p.set(MODE1, ParamValue::I32(2)).is_ok());
        assert!(p.set(ID, ParamValue::I32(2)).is_ok());
        assert_eq!(
            p.set(ODR1, ParamValue::I32(2000)),
            Err(ParamError::Rejected)
        );
    }

    #[test]
    fn candidate_reads_by_name() {
        fn probe(c: &Candidate<'_>) -> Result<(), &'static str> {
            assert_eq!(c.i32("IMU1_ODR"), Some(4000));
            assert_eq!(c.i32("IMU1_LPF"), Some(200));
            assert_eq!(c.f32("IMU1_ODR"), None);
            assert_eq!(c.i32("IMU1_GYR_OFFX"), None);
            assert_eq!(c.f32("IMU1_GYR_OFFX"), Some(0.0));
            assert_eq!(c.get("NOPE"), None);
            Err("probed")
        }
        let mut p = Params::<16>::new(TABLE).unwrap();
        p.set_validator(1, probe);
        assert_eq!(
            p.set(ODR1, ParamValue::I32(4000)),
            Err(ParamError::Rejected)
        );
        assert_eq!(p.get(ODR1), Some(ParamValue::I32(1000)));
    }

    // ---- persistence ----

    #[test]
    fn non_default_yields_exactly_the_changed_values() {
        let mut p = store();
        assert_eq!(p.non_default().count(), 0);
        p.set(BAUD, ParamValue::I32(57_600)).unwrap();
        p.set(OFFX, ParamValue::F32(-0.0)).unwrap(); // bit-for-bit differs from 0.0
        p.set(ODR2, ParamValue::I32(1000)).unwrap(); // no-op
        p.set(RANGE2, ParamValue::F32(8.0)).unwrap();
        p.set(RANGE2, ParamValue::F32(16.0)).unwrap(); // back to default
        let got: Vec<_> = p.non_default().collect();
        assert_eq!(got.len(), 2);
        assert_eq!(got[0], (name("SYS_BAUD"), ParamValue::I32(57_600)));
        assert_eq!(got[1].0, name("IMU1_GYR_OFFX"));
        let ParamValue::F32(x) = got[1].1 else {
            panic!()
        };
        assert_eq!(x.to_bits(), (-0.0f32).to_bits());
    }

    #[test]
    fn load_failure_kinds() {
        let mut p = store();
        assert_eq!(
            p.load(&name("NOPE"), ParamValue::I32(1)),
            Err(LoadError::UnknownName)
        );
        assert_eq!(
            p.load(&name("SYS_ID"), ParamValue::F32(1.0)),
            Err(LoadError::WrongType)
        );
        assert_eq!(
            p.load(&name("SYS_ID"), ParamValue::I32(300)),
            Err(LoadError::OutOfRange)
        );
        assert_eq!(
            p.load(&name("IMU1_GYR_OFFX"), ParamValue::F32(f32::NAN)),
            Err(LoadError::OutOfRange)
        );
        assert_eq!(p.load(&name("SYS_VER"), ParamValue::I32(9)), Ok(false));
        assert_eq!(p.get(VER), Some(ParamValue::I32(7)));
        assert_eq!(p.non_default().count(), 0);
        assert_quiet(&mut p);
        assert_eq!(p.take_notice(), None);
    }

    #[test]
    fn load_round_trips_and_does_not_dirty() {
        let mut a = store();
        a.set(BAUD, ParamValue::I32(57_600)).unwrap();
        a.set(ODR1, ParamValue::I32(2000)).unwrap();
        a.set(OFFY, ParamValue::F32(-0.0)).unwrap();
        a.set(RANGE2, ParamValue::F32(32.0)).unwrap();
        let saved: Vec<_> = a.non_default().collect();
        let mut b = store();
        for (n, v) in &saved {
            assert_eq!(b.load(n, *v), Ok(true));
        }
        assert_quiet(&mut b);
        assert_eq!(b.take_notice(), None);
        let again: Vec<_> = b.non_default().collect();
        assert_eq!(again.len(), saved.len());
        for ((n1, v1), (n2, v2)) in saved.iter().zip(&again) {
            assert_eq!(n1, n2);
            match (v1, v2) {
                (ParamValue::F32(x), ParamValue::F32(y)) => assert_eq!(x.to_bits(), y.to_bits()),
                _ => assert_eq!(v1, v2),
            }
        }
    }

    #[test]
    fn load_runs_no_validator() {
        let mut p = store();
        assert_eq!(p.load(&name("IMU1_LPF"), ParamValue::I32(3000)), Ok(true));
        assert_eq!(p.get(LPF1), Some(ParamValue::I32(3000)));
        assert_quiet(&mut p);
    }

    // ---- resets ----

    fn changed_store() -> Params<16> {
        let mut p = store();
        p.set(BAUD, ParamValue::I32(57_600)).unwrap();
        p.set(ID, ParamValue::I32(9)).unwrap();
        p.set(ODR1, ParamValue::I32(2000)).unwrap();
        p.set(OFFX, ParamValue::F32(0.5)).unwrap();
        p.set(ACC2, ParamValue::F32(-1.0)).unwrap();
        p.clear_dirty();
        let _ = p.take_configure_mask();
        let _ = p.take_live_mask();
        let _ = p.take_notice();
        p
    }

    #[test]
    fn reset_all_restores_defaults() {
        let mut p = changed_store();
        assert_eq!(p.reset_all(), 5);
        assert_eq!(p.non_default().count(), 0);
        assert!(p.dirty());
        assert_eq!(p.take_configure_mask(), 1 << 1);
        assert_eq!(p.take_live_mask(), 1 | (1 << 1) | (1 << 2));
        assert_eq!(
            p.take_notice().unwrap().kind,
            NoticeKind::AppliesAfterReboot
        );
        assert_eq!(p.reset_all(), 0);
    }

    #[test]
    fn reset_calibration_touches_only_calibration() {
        let mut p = changed_store();
        assert_eq!(p.reset_calibration(), 2);
        let left: Vec<_> = p.non_default().map(|(n, _)| n).collect();
        assert_eq!(left, [name("SYS_BAUD"), name("SYS_ID"), name("IMU1_ODR")]);
        assert!(p.dirty());
        assert_eq!(p.take_configure_mask(), 0);
        assert_eq!(p.take_live_mask(), (1 << 1) | (1 << 2));
        assert_eq!(p.take_notice(), None);
    }

    #[test]
    fn reset_when_default_changes_nothing() {
        let mut p = store();
        assert_eq!(p.reset_all(), 0);
        assert_quiet(&mut p);
        assert_eq!(p.reset_calibration(), 0);
        assert_quiet(&mut p);
        assert_eq!(p.take_notice(), None);
        // a pending dirty flag is not cleared by a no-change reset
        p.set(ID, ParamValue::I32(2)).unwrap();
        p.reset_calibration();
        assert!(p.dirty());
    }

    // ---- property ----

    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> u64 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            self.0 = x;
            x
        }
        fn below(&mut self, n: u64) -> u64 {
            self.next() % n
        }
    }

    fn random_value(r: &mut Rng, def: Option<&ParamDef>) -> ParamValue {
        let float_special = [
            f32::NAN,
            f32::INFINITY,
            f32::NEG_INFINITY,
            -0.0,
            0.0,
            f32::MAX,
            f32::MIN,
            f32::MIN_POSITIVE,
            1.0,
            -1.0,
            2.0,
            32.0,
        ];
        let int_special = [
            0,
            1,
            -1,
            3,
            100,
            250,
            251,
            8000,
            8001,
            i32::MAX,
            i32::MIN,
            921_600,
        ];
        let pick_float = r.below(2) == 0;
        match r.below(4) {
            0 if pick_float => ParamValue::F32(float_special[r.below(12) as usize]),
            0 => ParamValue::I32(int_special[r.below(12) as usize]),
            1 => ParamValue::F32(f32::from_bits(r.next() as u32)),
            2 => ParamValue::I32(r.next() as i32),
            _ => match def {
                Some(d) => match (d.min, d.max) {
                    (ParamValue::I32(lo), ParamValue::I32(hi)) => {
                        let span = (i64::from(hi) - i64::from(lo) + 1) as u64;
                        ParamValue::I32((i64::from(lo) + r.below(span) as i64) as i32)
                    }
                    (ParamValue::F32(lo), ParamValue::F32(hi)) => {
                        ParamValue::F32(lo + (hi - lo) * (r.below(1001) as f32 / 1000.0))
                    }
                    _ => ParamValue::I32(0),
                },
                None => ParamValue::I32(0),
            },
        }
    }

    fn check_invariants(p: &Params<16>) {
        for (i, def) in TABLE.iter().enumerate() {
            let v = p.get(i as u16).unwrap();
            assert_eq!(in_range(def, v), Some(true), "{} = {v:?}", def.name);
        }
    }

    #[test]
    fn property_values_keep_type_and_range() {
        for seed in [1u64, 0x9E37_79B9_7F4A_7C15, 0xDEAD_BEEF_1234_5678, 42] {
            let mut r = Rng(seed);
            let mut p = store();
            for _ in 0..10_000 {
                match r.below(10) {
                    0..=5 => {
                        let i = r.below(13) as u16;
                        let v = random_value(&mut r, TABLE.get(usize::from(i)));
                        let _ = p.set(i, v);
                    }
                    6 | 7 => {
                        let i = r.below(12) as usize;
                        let v = random_value(&mut r, TABLE.get(i));
                        let n = name(TABLE.get(i).map_or("NOPE", |d| d.name));
                        let _ = p.load(&n, v);
                    }
                    8 => match r.below(3) {
                        0 => {
                            p.reset_all();
                        }
                        1 => {
                            p.reset_calibration();
                        }
                        _ => {
                            let _ = (p.take_configure_mask(), p.take_live_mask());
                            let _ = p.take_notice();
                            p.clear_dirty();
                        }
                    },
                    _ => {
                        let _ = p.non_default().count();
                    }
                }
                check_invariants(&p);
                assert_eq!(p.get(VER), Some(ParamValue::I32(7)));
            }
        }
    }
}
