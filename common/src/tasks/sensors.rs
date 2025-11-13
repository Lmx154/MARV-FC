//! Generic async sensor tasks (HAL-agnostic)
//!
//! Provides reusable async loops for reading sensors independently and
//! forwarding data to a user-supplied sink. These functions are pure async
//! functions and can be wrapped by an executor task in device crates.

#![allow(async_fn_in_trait)]

use defmt::warn;

use crate::drivers::bmi088::{Bmi088, Bmi088Raw};
use crate::drivers::bmm350::{Bmm350, RawMag};
use crate::drivers::bmp390::Bmp390;
use crate::utils::delay::DelayMs;

/// Generic async sink for delivering sensor data to the application layer.
/// Implement this in the device crate to push into shared state, channels, etc.
pub trait DataSink<T> {
    /// Publish a new sample. Implementations should be lightweight and non-blocking.
    async fn publish(&mut self, data: T);
}

/// Run an async BMI088 read loop.
/// - `imu` is an initialized BMI088 driver (or it will be initialized here if not).
/// - `delay` is an async ms delay adapter
/// - `sink` receives raw accel/gyro frames
/// - `interval_ms` pacing; 0 to run as fast as possible
pub async fn run_bmi088_task<SPI, CSACC, CSGYR, D, S>(
    imu: &mut Bmi088<SPI, CSACC, CSGYR>,
    delay: &mut D,
    sink: &mut S,
    interval_ms: u32,
)
where
    SPI: embedded_hal_async::spi::SpiBus<u8>,
    CSACC: embedded_hal::digital::OutputPin,
    CSGYR: embedded_hal::digital::OutputPin,
    D: DelayMs,
    S: DataSink<Bmi088Raw>,
{
    // Try to init; ignore error to keep attempting reads (caller may have init'd already)
    if let Err(e) = imu.init(delay).await {
        warn!("BMI088 init error: {:?}", e);
    }

    loop {
        match imu.read_raw().await {
            Ok(raw) => {
                sink.publish(raw).await;
            }
            Err(e) => {
                warn!("BMI088 read error: {:?}", e);
            }
        }
        if interval_ms > 0 { delay.delay_ms(interval_ms).await; }
    }
}

/// Run an async BMM350 read loop delivering RawMag.
pub async fn run_bmm350_task<I2C, D, S>(
    mag: &mut Bmm350<I2C>,
    delay: &mut D,
    sink: &mut S,
    interval_ms: u32,
)
where
    I2C: embedded_hal_async::i2c::I2c,
    D: DelayMs,
    S: DataSink<RawMag>,
{
    if let Err(e) = mag.init(delay).await { warn!("BMM350 init error: {:?}", e); }
    loop {
        match mag.read_raw(delay).await {
            Ok(m) => sink.publish(m).await,
            Err(e) => warn!("BMM350 read error: {:?}", e),
        }
        if interval_ms > 0 { delay.delay_ms(interval_ms).await; }
    }
}

/// Run an async BMP390 read loop delivering (temp_c_x100, pressure_pa)
pub async fn run_bmp390_task<I2C, D, S>(
    baro: &mut Bmp390<I2C>,
    delay: &mut D,
    sink: &mut S,
    interval_ms: u32,
)
where
    I2C: embedded_hal_async::i2c::I2c,
    D: DelayMs,
    S: DataSink<(i32, i32)>,
{
    if let Err(e) = baro.init(delay).await { warn!("BMP390 init error: {:?}", e); }
    loop {
        match baro.read_compensated(delay).await {
            Ok(sample) => sink.publish(sample).await,
            Err(e) => warn!("BMP390 read error: {:?}", e),
        }
        if interval_ms > 0 { delay.delay_ms(interval_ms).await; }
    }
}
