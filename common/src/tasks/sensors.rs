//! Generic async sensor tasks (HAL-agnostic)
//!
//! Provides reusable async loops for reading sensors independently and
//! forwarding data to a user-supplied sink. These functions are pure async
//! functions and can be wrapped by an executor task in device crates.

#![allow(async_fn_in_trait)]

use defmt::warn;

use crate::drivers::adxl375::{Adxl375, Adxl375Raw};
use crate::drivers::bmi088::{Bmi088, Bmi088Raw};
use crate::drivers::bmm350::{Bmm350, RawMag};
use crate::drivers::bmp390::Bmp390;
use crate::drivers::neom9n::{NeoM9n, GpsData};
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
        if interval_ms > 0 {
            delay.delay_ms(interval_ms).await;
        }
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
    if let Err(e) = mag.init(delay).await {
        warn!("BMM350 init error: {:?}", e);
    }
    loop {
        match mag.read_raw(delay).await {
            Ok(m) => sink.publish(m).await,
            Err(e) => warn!("BMM350 read error: {:?}", e),
        }
        if interval_ms > 0 {
            delay.delay_ms(interval_ms).await;
        }
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
    if let Err(e) = baro.init(delay).await {
        warn!("BMP390 init error: {:?}", e);
    }
    loop {
        match baro.read_compensated(delay).await {
            Ok(sample) => sink.publish(sample).await,
            Err(e) => warn!("BMP390 read error: {:?}", e),
        }
        if interval_ms > 0 {
            delay.delay_ms(interval_ms).await;
        }
    }
}

/// Run an async NEO-M9N read loop delivering parsed `GpsData` when available.
pub async fn run_neom9n_task<I2C, D, S>(
    gps: &mut NeoM9n<I2C>,
    delay: &mut D,
    sink: &mut S,
    interval_ms: u32,
)
where
    I2C: embedded_hal_async::i2c::I2c,
    D: DelayMs,
    S: DataSink<GpsData>,
{
    if let Err(e) = gps.init(delay).await {
        warn!("NEO-M9N init error: {:?}", e);
    }
    // Nudge a NAV-PVT reply initially
    let _ = gps.poll_nav_pvt().await;
    let mut buf = [0u8; 128];
    let mut loops: u32 = 0;
    loop {
        match gps.read_and_parse(delay, &mut buf).await {
            Ok(Some(fix)) => {
                sink.publish(fix).await;
            }
            Ok(None) => {
                // Publish the last known (stale) fix so downstream can see activity
                if let Some(last) = gps.last() {
                    sink.publish(last).await;
                }
                // If nothing parsed for a while, send another NAV-PVT poll to verify link.
                if loops % 20 == 0 {
                    let _ = gps.poll_nav_pvt().await;
                }
            }
            Err(e) => warn!("NEO-M9N read error: {:?}", e),
        }
        loops = loops.wrapping_add(1);
        if interval_ms > 0 {
            delay.delay_ms(interval_ms).await;
        }
    }
}

/// Run an async ADXL375 read loop delivering `Adxl375Raw`.
///
/// - `accel` is an ADXL375 driver
/// - `delay` is an async ms delay adapter
/// - `sink` receives raw accel frames
/// - `interval_ms` pacing; 0 to run as fast as possible
pub async fn run_adxl375_task<I2C, INT, D, S>(
    accel: &mut Adxl375<I2C, INT>,
    delay: &mut D,
    sink: &mut S,
    interval_ms: u32,
)
where
    I2C: embedded_hal_async::i2c::I2c,
    INT: embedded_hal::digital::InputPin,
    D: DelayMs,
    S: DataSink<Adxl375Raw>,
{
    if let Err(_e) = accel.init(delay).await {
        // Can't format generic error type with defmt safely without extra bounds,
        // so just log a static message (or match on variants higher up if needed).
        warn!("ADXL375 init error");
    }

    loop {
        match accel.read_accel_raw().await {
            Ok(raw_vec) => {
                let frame = Adxl375Raw { accel: raw_vec };
                sink.publish(frame).await;
            }
            Err(_e) => {
                warn!("ADXL375 read error");
            }
        }
        if interval_ms > 0 {
            delay.delay_ms(interval_ms).await;
        }
    }
}
