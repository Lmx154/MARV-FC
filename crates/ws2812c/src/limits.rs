//! The timing limits of the WS2812C-2020 data line.
//!
//! Source: Worldsemi WS2812C-2020-V1 datasheet, page 4 of 9, "Data Transfer Time". A data bit is a
//! high time followed by a low time; the code is told apart by the high time. Data is sent in the
//! order green, red, blue, most significant bit first. A low time of more than 280 us ends the
//! frame and latches the data.

/// "0" code, high time, minimum.
pub const T0H_MIN_NS: u32 = 220;
/// "0" code, high time, maximum.
pub const T0H_MAX_NS: u32 = 380;
/// "1" code, high time, minimum.
pub const T1H_MIN_NS: u32 = 580;
/// "1" code, high time, maximum.
pub const T1H_MAX_NS: u32 = 1_000;
/// "0" code, low time, minimum.
pub const T0L_MIN_NS: u32 = 580;
/// "0" code, low time, maximum.
pub const T0L_MAX_NS: u32 = 1_000;
/// "1" code, low time, minimum.
pub const T1L_MIN_NS: u32 = 580;
/// "1" code, low time, maximum.
pub const T1L_MAX_NS: u32 = 1_000;
/// Low time that ends a frame (the datasheet says "over 280 us").
pub const RESET_MIN_US: u32 = 280;

/// Low time the transports leave between two frames: the limit plus a margin.
pub const RESET_US: u32 = 300;
