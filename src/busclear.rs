//! The I2C bus clear, shared by the core-0 devices on an I2C bus (the barometer on I2C1, the
//! magnetometer on I2C0): neither the drivers nor embassy-rp 0.10 offer one.

use embassy_rp::Peri;
use embassy_rp::gpio::{Flex, Pin, Pull};
use embassy_time::{Duration, block_for};

/// Up to 9 SCL pulses while SDA is held low, then a START and a STOP (as `bmp581_cancel`'s
/// `bus_clear`). Called only while no bus object exists on these pins. Returns (SDA low, SCL
/// low, pulses).
pub fn bus_clear<SCL: Pin, SDA: Pin>(
    scl: &mut Peri<'static, SCL>,
    sda: &mut Peri<'static, SDA>,
) -> (bool, bool, u8) {
    let (sda_low, scl_low, pulses, _, _) = clear(scl, sda);
    (sda_low, scl_low, pulses)
}

/// The bus clear at boot, before the first transaction on the bus: one `[INFO]` line with the
/// levels of SDA and SCL before and after it and the pulses given, a warning when the bus is
/// still not idle (the device then reports its own failure).
pub fn clear_at_boot<SCL: Pin, SDA: Pin>(
    name: &'static str,
    scl: &mut Peri<'static, SCL>,
    sda: &mut Peri<'static, SDA>,
) {
    let (sda_low, scl_low, pulses, sda_low_after, scl_low_after) = clear(scl, sda);
    defmt::info!(
        "[INFO] {=str} bus at boot: SDA {=u8} SCL {=u8} before, {=u8} pulses, SDA {=u8} SCL {=u8} after",
        name,
        u8::from(!sda_low),
        u8::from(!scl_low),
        pulses,
        u8::from(!sda_low_after),
        u8::from(!scl_low_after)
    );
    if sda_low_after || scl_low_after {
        defmt::warn!("[INFO] {=str} bus not idle after the clear at boot", name);
    }
}

/// The clear: (SDA low, SCL low, pulses) as found, then (SDA low, SCL low) after it.
fn clear<SCL: Pin, SDA: Pin>(
    scl: &mut Peri<'static, SCL>,
    sda: &mut Peri<'static, SDA>,
) -> (bool, bool, u8, bool, bool) {
    // SAFETY: the bus was dropped before this call and these pins are released again when this
    // function returns, before the next bus is built (the caller's contract).
    let (scl, sda) = unsafe { (scl.clone_unchecked(), sda.clone_unchecked()) };
    let mut scl = Flex::new(scl);
    let mut sda = Flex::new(sda);
    let half = Duration::from_micros(5);
    scl.set_pull(Pull::None);
    sda.set_pull(Pull::None);
    scl.set_low();
    sda.set_low();
    scl.set_as_input();
    sda.set_as_input();
    block_for(half);
    let (sda_low, scl_low) = (sda.is_low(), scl.is_low());
    let mut pulses = 0u8;
    while sda.is_low() && pulses < 9 {
        scl.set_as_output();
        block_for(half);
        scl.set_as_input();
        block_for(half);
        pulses += 1;
    }
    if sda.is_high() && scl.is_high() {
        sda.set_as_output();
        block_for(half);
        sda.set_as_input();
        block_for(half);
    }
    (sda_low, scl_low, pulses, sda.is_low(), scl.is_low())
}
