//! Portable drivers and adapters.

pub mod sensors;
pub mod storage;
pub mod radio;
pub mod leds;

// Flat re-exports keep legacy call sites usable while modules migrate.
pub use radio::sx1262;
pub use sensors::{bmi088, bmm350, bmp390, bmp581, lsm6dsv32x, neom9n};
