//! The status LED's parameter: its brightness level (`decision-status-led-20261118`).

use fw_core::params::{Apply, ParamDef};

/// The level of the status LED: 0 off, 1 dim, 2 medium, 3 bright (`fw_core::led::level_brightness`).
pub const BRIGHT: &str = "LED_BRIGHT";

/// The number of LED parameters.
pub const COUNT: usize = 1;

/// `LED_BRIGHT`, appended to the firmware's table at index 79: `I32`, default 1 (dim), range 0 to 3,
/// `Live` (the LED task on core 0 reads it every 20 ms; it never reaches the flight core). Device 0,
/// the system, like `AHRS_ORIENTATION`: there is no LED device row.
pub static TABLE: [ParamDef; COUNT] = [ParamDef::i32(BRIGHT, 0, 1, 0, 3, Apply::Live)];
