//! The ESC's parameter: the motors' pole count, for the rpm of ESC_STATUS.

use fw_core::params::{Apply, ParamDef};
use mavlink_device::marv::MarvDevice;

const DEVICE: u8 = MarvDevice::MARV_DEVICE_ESC as u8;

/// Magnets per motor (poles, not pole pairs): mechanical rpm = eRPM / (`MOT_POLES` / 2). Betaflight's
/// `motor_poles` and ArduPilot's `SERVO_BLH_POLES` both default to 14, as do the motors.
pub const POLES: &str = "MOT_POLES";

/// The number of ESC parameters.
pub const COUNT: usize = 1;

/// `MOT_POLES`, appended to the firmware's table at index 78: `I32`, default 14, range 2 to 64,
/// `Live` (core 0 reads it for every message; it never reaches the flight core). Device ESC (12).
pub static TABLE: [ParamDef; COUNT] = [ParamDef::i32(POLES, DEVICE, 14, 2, 64, Apply::Live)];
