//! Portable drivers and adapters.

pub mod leds;
pub mod radio;
pub mod sensors;
pub mod servo;
pub mod storage;

// Flat re-exports keep legacy call sites usable while modules migrate.
pub use radio::sx1262;
pub use sensors::{
    bmi088, bmm350, bmp388, bmp390, bmp581, lsm6dsv32x, neom9n, pressure_transducer,
};
