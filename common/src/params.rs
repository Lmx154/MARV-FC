//! Single source of truth for firmware parameters.
//!
//! Goals:
//! - One authoritative registry in firmware (GCS discovers via MAVLink).
//! - Same registry backs both MAVLink PARAM_* and USB-CDC CLI.
//! - No metadata required for correctness (UI helpers can be added later).
//!
//! Constraints:
//! - MAVLink PARAM_VALUE uses param_id[16]. Keep names <= 16 chars.
//!
//! ## MAVLink Integration
//!
//! The `protocol::mavlink::encode` module provides helpers for the parameter protocol:
//! - `build_param_value_frame()` - Build PARAM_VALUE responses
//! - `handle_param_request_read()` - Process PARAM_REQUEST_READ
//! - `handle_param_set()` - Process PARAM_SET and update registry
//!
//! See `protocol::mavlink` for MAVLink parameter protocol helpers.
//!
//! ## Quick Start
//!
//! ```ignore
//! // 1. Create registry in your firmware main()
//! let mut params = ParamRegistry::new();
//!
//! // 2. Read params in tasks
//! if params.bool(ParamId::Bmi088En) {
//!     let rate = params.u32(ParamId::Bmi088RateHz);
//!     configure_sensor(rate);
//! }
//!
//! // 3. Wire into MAVLink handler (see `protocol::mavlink`)
//! match msg {
//!     Common::ParamRequestList(_) => { /* send all params */ }
//!     Common::ParamRequestRead(_) => { /* send one param */ }
//!     Common::ParamSet(_) => { /* update param, send ACK */ }
//!     _ => {}
//! }
//! ```

#![allow(dead_code)]

use core::fmt;

/// MAVLink param_id is 16 bytes (null-terminated / padded).
pub const PARAM_NAME_MAX: usize = 16;

/// Minimal param type set for bring-up.
#[derive(Copy, Clone, Debug, Eq, PartialEq, defmt::Format)]
pub enum ParamType {
    Bool,
    I32,
    U32,
    F32,
}

#[derive(Copy, Clone, Debug, PartialEq, defmt::Format)]
pub enum ParamValue {
    Bool(bool),
    I32(i32),
    U32(u32),
    F32(f32),
}

impl ParamValue {
    pub fn ty(&self) -> ParamType {
        match self {
            ParamValue::Bool(_) => ParamType::Bool,
            ParamValue::I32(_) => ParamType::I32,
            ParamValue::U32(_) => ParamType::U32,
            ParamValue::F32(_) => ParamType::F32,
        }
    }

    /// MAVLink PARAM_VALUE.param_value is f32, but for integer types we must
    /// reinterpret the bits (transmute), not convert the value.
    /// This ensures Mission Planner displays the correct values.
    pub fn as_mavlink_f32(self) -> f32 {
        match self {
            ParamValue::Bool(v) => if v { 1.0 } else { 0.0 },
            ParamValue::I32(v) => f32::from_bits(v as u32),
            ParamValue::U32(v) => f32::from_bits(v),
            ParamValue::F32(v) => v,
        }
    }

    /// Convenience: parse “CLI-ish” inputs without pulling in alloc.
    /// (You can ignore this if you already have parsing elsewhere.)
    pub fn parse_as(ty: ParamType, s: &str) -> Option<Self> {
        match ty {
            ParamType::Bool => {
                let v = match s {
                    "1" | "true" | "True" | "TRUE" | "on" | "ON" => true,
                    "0" | "false" | "False" | "FALSE" | "off" | "OFF" => false,
                    _ => return None,
                };
                Some(ParamValue::Bool(v))
            }
            ParamType::I32 => s.parse::<i32>().ok().map(ParamValue::I32),
            ParamType::U32 => s.parse::<u32>().ok().map(ParamValue::U32),
            ParamType::F32 => s.parse::<f32>().ok().map(ParamValue::F32),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct ParamDef {
    /// Must be <= 16 chars for MAVLink.
    pub name: &'static str,
    pub ty: ParamType,
    pub default: ParamValue,
}

impl ParamDef {
    pub const fn new(name: &'static str, ty: ParamType, default: ParamValue) -> Self {
        // NOTE: Can't assert in const without extra tricks; validated at runtime by `validate()`.
        Self { name, ty, default }
    }
}

/// Stable indices are the protocol.
/// - MAVLink uses param_index 0..N-1
/// - PARAM_REQUEST_READ by index expects stable ordering
#[repr(u16)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ParamId {
    // --- System / Comms ---
    SysId = 0,
    CompId,
    TelemRateHz,
    HeartbeatEn,
    StatustextEn,

    // --- Logging ---
    LogEn,
    LogRateHz,

    // --- Sensor enable flags ---
    Bmi088En,
    Icm45686En,
    Bmm350En,
    Bmp390En,
    GpsEn,

    // --- Sensor rates ---
    Bmi088RateHz,
    Icm45686RateHz,
    Bmm350RateHz,
    Bmp390RateHz,
    GpsRateHz,

    // --- Simple calibration offsets (applied in your fusion / preprocessing) ---
    AccBiasX,
    AccBiasY,
    AccBiasZ,
    GyrBiasX,
    GyrBiasY,
    GyrBiasZ,
    MagBiasX,
    MagBiasY,
    MagBiasZ,

    // --- Simple filtering knobs (0 = “driver default / off”) ---
    ImuAccLpfHz,
    ImuGyrLpfHz,
    BaroLpfHz,

    // --- Orientation hook (you can interpret later) ---
    ImuRot, // 0=identity (you define enum later)
    
    // --- LED Configuration ---
    LedColorR,
    LedColorG,
    LedColorB,

    // --- Radio / LoRa link ---
    RadioTelemRateHz,

    // --- FC state / safety ---
    FcModeUsbForcesConfig,
    FcArmAllowedInConfig,
    FcStateHeartbeatHz,

    // --- Telemetry policy (fast/normal lanes) ---
    TelFastEn,
    TelFastHz,
    TelFastMaxB,
    TelNormEn,
    TelNormMaxHz,
    TelNormQmax,
    TelAckTmoMs,
    TelAckRtryMax,
    TelQos,
    TelAdaptEn,
    TelSnrBad,
    TelSnrGood,
    TelAdaptHold,
    // --- LoRa tick MAC configuration (applied to GS/Radio link MAC) ---
    LinkTickHz,
    LinkSlotMode,
    LinkFastMaxB,
    LasMaxAgeMs,

    // --- LoRa RF (demodulation-affecting) configuration ---
    // Stored on FC; applied by GS + Radio via staged RF reconfiguration.
    LoraFreqKhz,
    LoraSf,
    LoraBwCode,
    LoraCrCode,
    LoraSyncWord,
}

pub const PARAM_COUNT: usize = (ParamId::LoraSyncWord as usize) + 1;

/// The single parameter definition table (authoritative).
pub const PARAM_DEFS: [ParamDef; PARAM_COUNT] = [
    // --- System / Comms ---
    ParamDef::new("SYS_ID",        ParamType::U32,  ParamValue::U32(1)),
    ParamDef::new("COMP_ID",       ParamType::U32,  ParamValue::U32(1)),
    ParamDef::new("TEL_RATE_HZ",   ParamType::U32,  ParamValue::U32(50)),
    ParamDef::new("HB_EN",         ParamType::Bool, ParamValue::Bool(true)),
    ParamDef::new("STATUSTXT_EN",  ParamType::Bool, ParamValue::Bool(true)),

    // --- Logging ---
    ParamDef::new("LOG_EN",        ParamType::Bool, ParamValue::Bool(false)),
    ParamDef::new("LOG_RATE_HZ",   ParamType::U32,  ParamValue::U32(50)),

    // --- Sensor enable flags ---
    ParamDef::new("BMI088_EN",     ParamType::Bool, ParamValue::Bool(true)),
    ParamDef::new("ICM45686_EN",   ParamType::Bool, ParamValue::Bool(true)),
    ParamDef::new("BMM350_EN",     ParamType::Bool, ParamValue::Bool(true)),
    ParamDef::new("BMP390_EN",     ParamType::Bool, ParamValue::Bool(true)),
    ParamDef::new("GPS_EN",        ParamType::Bool, ParamValue::Bool(false)),

    // --- Sensor rates ---
    ParamDef::new("BMI088_HZ",     ParamType::U32,  ParamValue::U32(200)),
    ParamDef::new("ICM45686_HZ",   ParamType::U32,  ParamValue::U32(200)),
    ParamDef::new("BMM350_HZ",     ParamType::U32,  ParamValue::U32(50)),
    ParamDef::new("BMP390_HZ",     ParamType::U32,  ParamValue::U32(50)),
    ParamDef::new("GPS_HZ",        ParamType::U32,  ParamValue::U32(10)),

    // --- Calibration offsets (SI units in your codebase; define interpretation later) ---
    ParamDef::new("ACC_BIAS_X",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("ACC_BIAS_Y",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("ACC_BIAS_Z",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("GYR_BIAS_X",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("GYR_BIAS_Y",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("GYR_BIAS_Z",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("MAG_BIAS_X",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("MAG_BIAS_Y",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("MAG_BIAS_Z",    ParamType::F32,  ParamValue::F32(0.0)),

    // --- Filters (Hz). 0 = default/off.
    ParamDef::new("ACC_LPF_HZ",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("GYR_LPF_HZ",    ParamType::F32,  ParamValue::F32(0.0)),
    ParamDef::new("BARO_LPF_HZ",   ParamType::F32,  ParamValue::F32(0.0)),

    // --- Orientation hook ---
    ParamDef::new("IMU_ROT",       ParamType::U32,  ParamValue::U32(0)),
    
    // --- LED Configuration ---
    ParamDef::new("LED_COLOR_R",   ParamType::U32,  ParamValue::U32(0)),
    ParamDef::new("LED_COLOR_G",   ParamType::U32,  ParamValue::U32(50)),
    ParamDef::new("LED_COLOR_B",   ParamType::U32,  ParamValue::U32(0)),

    // --- Radio / LoRa link ---
    ParamDef::new("RAD_TEL_HZ",    ParamType::U32,  ParamValue::U32(10)),

    // --- FC state / safety ---
    ParamDef::new("FC_USB_FORCE_CFG", ParamType::Bool, ParamValue::Bool(true)),
    ParamDef::new("FC_ARM_IN_CFG",    ParamType::Bool, ParamValue::Bool(false)),
    ParamDef::new("FC_HB_HZ",         ParamType::U32,  ParamValue::U32(1)),

    // --- Telemetry policy (fast/normal lanes) ---
    ParamDef::new("TEL_FAST_EN",      ParamType::Bool, ParamValue::Bool(false)),
    ParamDef::new("TEL_FAST_HZ",      ParamType::U32,  ParamValue::U32(100)),
    ParamDef::new("TEL_FAST_MAXB",    ParamType::U32,  ParamValue::U32(120)),
    ParamDef::new("TEL_NORM_EN",      ParamType::Bool, ParamValue::Bool(true)),
    ParamDef::new("TEL_NORM_MAXHZ",   ParamType::U32,  ParamValue::U32(10)),
    ParamDef::new("TEL_NORM_QMAX",    ParamType::U32,  ParamValue::U32(5)),
    ParamDef::new("TEL_ACK_TMO_MS",   ParamType::U32,  ParamValue::U32(500)),
    ParamDef::new("TEL_ACK_RTRY_MAX", ParamType::U32,  ParamValue::U32(3)),
    ParamDef::new("TEL_QOS",          ParamType::U32,  ParamValue::U32(0)),
    ParamDef::new("TEL_ADAPT_EN",     ParamType::Bool, ParamValue::Bool(false)),
    ParamDef::new("TEL_SNR_BAD",      ParamType::I32,  ParamValue::I32(0)),
    ParamDef::new("TEL_SNR_GOOD",     ParamType::I32,  ParamValue::I32(10)),
    ParamDef::new("TEL_ADAPT_HOLD",   ParamType::U32,  ParamValue::U32(1000)),
    // --- LoRa tick MAC configuration ---
    // These values are stored durably on the FC (single source of truth).
    // The GS pulls them via MAVLink PARAM_* and then applies them locally and
    // distributes runtime config to the Radio via COMMAND_LONG (see protocol::mavlink::link_mac_config).
    ParamDef::new("LINK_TICK_HZ",   ParamType::U32,  ParamValue::U32(50)),
    ParamDef::new("LINK_SLOT_MD",   ParamType::U32,  ParamValue::U32(1)),
    ParamDef::new("LINK_FAST_MB",   ParamType::U32,  ParamValue::U32(236)),
    ParamDef::new("LAS_MAX_AGE_MS", ParamType::U32,  ParamValue::U32(1000)),

    // --- LoRa RF (demodulation-affecting) configuration ---
    // Encoding matches `protocol::mavlink::rf_reconfig::RfReconfigSettings`.
    ParamDef::new("LORA_F_KHZ",      ParamType::U32,  ParamValue::U32(915_000)),
    ParamDef::new("LORA_SF",         ParamType::U32,  ParamValue::U32(7)),
    ParamDef::new("LORA_BW",         ParamType::U32,  ParamValue::U32(0)),
    ParamDef::new("LORA_CR",         ParamType::U32,  ParamValue::U32(0)),
    ParamDef::new("LORA_SW",         ParamType::U32,  ParamValue::U32(0x0047)),
];

/// Runtime storage for parameter values.
///
/// This is the thing you actually mutate via:
/// - MAVLink PARAM_SET
/// - USB-CDC `param set ...`
pub struct ParamRegistry {
    values: [ParamValue; PARAM_COUNT],
}

impl fmt::Debug for ParamRegistry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ParamRegistry")
            .field("values_len", &PARAM_COUNT)
            .finish()
    }
}

impl ParamRegistry {
    /// Initialize from defaults in PARAM_DEFS.
    pub fn new() -> Self {
        let mut values = [ParamValue::U32(0); PARAM_COUNT];
        for (i, def) in PARAM_DEFS.iter().enumerate() {
            values[i] = def.default;
        }
        let mut s = Self { values };
        s.validate(); // debug-time safety
        s
    }

    /// Validate name lengths/types at boot (cheap).
    pub fn validate(&mut self) {
        for def in PARAM_DEFS.iter() {
            debug_assert!(
                def.name.len() <= PARAM_NAME_MAX,
                "Param name too long for MAVLink (max 16): {}",
                def.name
            );
            debug_assert_eq!(def.ty, def.default.ty(), "Default type mismatch for {}", def.name);
        }
    }

    #[inline]
    pub fn count(&self) -> u16 {
        PARAM_COUNT as u16
    }

    #[inline]
    pub fn def_by_index(&self, idx: u16) -> Option<&'static ParamDef> {
        PARAM_DEFS.get(idx as usize)
    }

    #[inline]
    pub fn get_by_index(&self, idx: u16) -> Option<ParamValue> {
        self.values.get(idx as usize).copied()
    }

    #[inline]
    pub fn set_by_index(&mut self, idx: u16, v: ParamValue) -> Result<(), ()> {
        let def = self.def_by_index(idx).ok_or(())?;
        if v.ty() != def.ty {
            return Err(());
        }
        self.values[idx as usize] = v;
        Ok(())
    }

    pub fn find_index_by_name(&self, name: &str) -> Option<u16> {
        PARAM_DEFS
            .iter()
            .position(|d| d.name == name)
            .map(|i| i as u16)
    }

    pub fn get_by_name(&self, name: &str) -> Option<(u16, ParamValue, &'static ParamDef)> {
        let idx = self.find_index_by_name(name)?;
        let def = self.def_by_index(idx)?;
        let val = self.get_by_index(idx)?;
        Some((idx, val, def))
    }

    pub fn set_by_name(&mut self, name: &str, v: ParamValue) -> Result<u16, ()> {
        let idx = self.find_index_by_name(name).ok_or(())?;
        self.set_by_index(idx, v)?;
        Ok(idx)
    }

    pub fn reset_all_to_defaults(&mut self) {
        for (i, def) in PARAM_DEFS.iter().enumerate() {
            self.values[i] = def.default;
        }
    }

    /// Handy “typed getters” you’ll use in tasks.
    #[inline] pub fn u32(&self, id: ParamId) -> u32 { match self.values[id as usize] { ParamValue::U32(v) => v, _ => 0 } }
    #[inline] pub fn f32(&self, id: ParamId) -> f32 { match self.values[id as usize] { ParamValue::F32(v) => v, _ => 0.0 } }
    #[inline] pub fn bool(&self, id: ParamId) -> bool { match self.values[id as usize] { ParamValue::Bool(v) => v, _ => false } }

    /// Serialize all parameters to a text buffer (NAME=VALUE format, one per line).
    /// Returns the number of bytes written.
    pub fn serialize_to_text(&self, buf: &mut [u8]) -> Result<usize, ()> {
        use core::fmt::Write;
        let mut writer = SliceWriter::new(buf);
        
        for (idx, def) in PARAM_DEFS.iter().enumerate() {
            if let Some(val) = self.get_by_index(idx as u16) {
                // Format: "PARAM_NAME=value\n"
                match val {
                    ParamValue::Bool(b) => {
                        if b {
                            write!(writer, "{}=1\n", def.name).map_err(|_| ())?;
                        } else {
                            write!(writer, "{}=0\n", def.name).map_err(|_| ())?;
                        }
                    }
                    ParamValue::I32(v) => {
                        write!(writer, "{}={}\n", def.name, v).map_err(|_| ())?;
                    }
                    ParamValue::U32(v) => {
                        write!(writer, "{}={}\n", def.name, v).map_err(|_| ())?;
                    }
                    ParamValue::F32(v) => {
                        write!(writer, "{}={}\n", def.name, v).map_err(|_| ())?;
                    }
                }
            }
        }
        
        Ok(writer.written())
    }

    /// Deserialize parameters from text buffer (NAME=VALUE format).
    /// Only updates parameters that are found in the text.
    pub fn deserialize_from_text(&mut self, text: &[u8]) -> Result<usize, ()> {
        let text_str = core::str::from_utf8(text).map_err(|_| ())?;
        let mut loaded_count = 0;
        
        for line in text_str.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue; // Skip empty lines and comments
            }
            
            if let Some((name, value_str)) = line.split_once('=') {
                let name = name.trim();
                let value_str = value_str.trim();
                
                if let Some(idx) = self.find_index_by_name(name) {
                    if let Some(def) = self.def_by_index(idx) {
                        let parsed_val = match def.ty {
                            ParamType::Bool => {
                                if value_str == "1" || value_str.eq_ignore_ascii_case("true") {
                                    Some(ParamValue::Bool(true))
                                } else {
                                    Some(ParamValue::Bool(false))
                                }
                            }
                            ParamType::I32 => {
                                value_str.parse::<i32>().ok().map(ParamValue::I32)
                            }
                            ParamType::U32 => {
                                value_str.parse::<u32>().ok().map(ParamValue::U32)
                            }
                            ParamType::F32 => {
                                value_str.parse::<f32>().ok().map(ParamValue::F32)
                            }
                        };
                        
                        if let Some(val) = parsed_val {
                            if self.set_by_index(idx, val).is_ok() {
                                loaded_count += 1;
                            }
                        }
                    }
                }
            }
        }
        
        Ok(loaded_count)
    }
}

/// Helper for writing formatted text to a slice
struct SliceWriter<'a> {
    buf: &'a mut [u8],
    written: usize,
}

impl<'a> SliceWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, written: 0 }
    }
    
    fn written(&self) -> usize {
        self.written
    }
}

impl<'a> core::fmt::Write for SliceWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        if self.written + bytes.len() > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.written..self.written + bytes.len()].copy_from_slice(bytes);
        self.written += bytes.len();
        Ok(())
    }
}
