//! Async BMM350 magnetometer driver (I2C, embedded-hal-async)
//!
//! Device and pinout agnostic. Targeted to Embassy systems but independent
//! of any specific HAL. Uses embedded-hal-async I2C traits and a simple
//! async millisecond delay trait from `common::utils::delay`.
//!
//! Notes on BMM350 bus behavior (from working sync driver):
//! - Reads require discarding 2 dummy bytes when using repeated-start
//!   read sequences (effective payload begins at byte index 2).
//! - Some data fields (OTP, burst XYZ) are read via multi-byte transfers
//!   where the first two bytes are dummy and remaining are valid.
//! - Busy flag is bit0 of PMU_STATUS (0x07). We poll this with a small
//!   async delay between attempts up to a timeout window.

#![allow(dead_code)]

use core::convert::TryInto;
use defmt::{debug, error, Format};
use embedded_hal_async::i2c::I2c;

use crate::utils::delay::DelayMs;

pub const BMM350_ADDR: u8 = 0x14; // Default I2C address (SDO low)

/// Register addresses (per datasheet)
pub mod registers {
	pub const CHIP_ID: u8 = 0x00; // Expected: 0x33
	pub const CMD: u8 = 0x7E; // Soft reset command (write 0xB6)
	pub const PMU_CMD: u8 = 0x06; // Power mode (suspend: 0x00, normal: 0x01)
	pub const PMU_CMD_AXIS_EN: u8 = 0x05; // Axis enable (XYZ: 0x07)
	pub const PMU_CMD_AGGR_SET: u8 = 0x04; // ODR and averaging (regular 25 Hz: 0x16)
	pub const PMU_STATUS: u8 = 0x07; // Busy flag (bit 0)
	pub const MAG_X_XLSB: u8 = 0x31; // Burst read start for X/Y/Z (9 bytes after 2 dummies)
	pub const OTP_CMD: u8 = 0x50; // OTP command for boot load
	pub const OTP_BASE: u8 = 0x40; // Start of OTP trim coefficients (21 bytes after 2 dummies)
}

/// OTP coefficients for compensation (placeholder for future parsing)
#[derive(Debug, Default, Format, Clone, Copy)]
pub struct OtpCoeffs {
	pub data: [u8; 21],
}

#[derive(Debug, Format)]
pub enum Error {
	I2c,
	ChipId(u8),
	BusyTimeout,
}

/// Raw magnetometer reading (signed 20-bit expanded to i32)
#[derive(Debug, Default, Clone, Copy, Format)]
pub struct RawMag {
	pub xyz: [i32; 3],
}

/// Async BMM350 driver holding I2C bus instance
pub struct Bmm350<I2C>
where
	I2C: I2c,
{
	i2c: I2C,
	addr: u8,
	otp: OtpCoeffs,
	verbose: bool,
}

impl<I2C> Bmm350<I2C>
where
	I2C: I2c,
{
	/// Create a new driver with given I2C bus and optional address
	pub fn new(i2c: I2C, address: u8) -> Self {
		Self { i2c, addr: address, otp: OtpCoeffs::default(), verbose: false }
	}

	/// Enable or disable verbose debug logging at runtime.
	pub fn set_debug(&mut self, enable: bool) { self.verbose = enable; }

	/// Release the owned I2C bus
	pub fn release(self) -> I2C { self.i2c }

	/// Initialize sensor (verify ID, reset, load OTP, enable axes, set ODR, enter normal mode)
	pub async fn init<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
		// Verify chip ID with a few retries (device powers up in suspend)
		let mut retries = 5u8;
		let mut chip_id = 0u8;
		while retries > 0 {
			match self.read_register(registers::CHIP_ID).await {
				Ok(id) => {
					chip_id = id;
					if self.verbose { debug!("BMM350: CHIP_ID=0x{:02X}", id); }
					if id == 0x33 { break; }
				}
				Err(_) => {
					if self.verbose { debug!("BMM350: CHIP_ID read failed, retry"); }
				}
			}
			delay.delay_ms(2).await;
			retries -= 1;
		}
		if chip_id != 0x33 {
			error!("BMM350: Invalid chip ID 0x{:02X}", chip_id);
			return Err(Error::ChipId(chip_id));
		}

		// Soft reset
		self.write_register(registers::CMD, 0xB6).await?;
		delay.delay_ms(3).await;
		self.wait_not_busy(delay, 100).await?;

		// Suspend
		self.write_register(registers::PMU_CMD, 0x00).await?;
		delay.delay_ms(1).await;
		self.wait_not_busy(delay, 20).await?;

		// Load OTP trims
		self.load_otp(delay).await?;

		// Enable XYZ
		self.write_register(registers::PMU_CMD_AXIS_EN, 0x07).await?;
		delay.delay_ms(1).await;
		self.wait_not_busy(delay, 20).await?;

		// Set regular preset 25 Hz (example 0x16)
		self.write_register(registers::PMU_CMD_AGGR_SET, 0x16).await?;
		delay.delay_ms(1).await;
		self.wait_not_busy(delay, 20).await?;

		// Normal mode
		self.write_register(registers::PMU_CMD, 0x01).await?;
		delay.delay_ms(70).await;
		self.wait_not_busy(delay, 100).await?;

		Ok(())
	}

	/// Read raw signed values (scaled from 20-bit format to i32)
	pub async fn read_raw<D: DelayMs>(&mut self, delay: &mut D) -> Result<RawMag, Error> {
		self.wait_not_busy(delay, 100).await?;

		let mut buffer = [0u8; 11];
		self.i2c
			.write_read(self.addr, &[registers::MAG_X_XLSB], &mut buffer)
			.await
			.map_err(|_| Error::I2c)?;

		// Discard the first 2 dummy bytes
		let data_slice = &buffer[2..11];
		if self.verbose {
			let mut first_six = [0u8; 6];
			first_six.copy_from_slice(&buffer[0..6]);
			debug!("BMM350 raw buf (first6) {=[u8]} valid {=[u8]}", first_six, data_slice);
		}

		let mut out = [0i32; 3];
		for axis in 0..3 {
			let o = axis * 3;
			let low = data_slice[o] as u32;
			let mid = data_slice[o + 1] as u32;
			let high = data_slice[o + 2] as u32; // lower nibble has bits 16..19
			let mut val = (high << 16) | (mid << 8) | low; // 20-bit value in bits 0..19
			// Sign-extend 20-bit signed value (bit 19 is sign bit => high nibble bit 4)
			if (high & 0x10) != 0 {
				val |= 0xFFF0_0000; // set upper 12 bits to 1
			}
			out[axis] = val as i32;
		}
		Ok(RawMag { xyz: out })
	}

	/// Read values converted to microtesla (approximate scale; full trim not applied yet)
	pub async fn read_ut<D: DelayMs>(&mut self, delay: &mut D) -> Result<(f32, f32, f32), Error> {
		let raw = self.read_raw(delay).await?;
		let scale = 0.001_907_f32; // Approximate from datasheet
		Ok((raw.xyz[0] as f32 * scale, raw.xyz[1] as f32 * scale, raw.xyz[2] as f32 * scale))
	}

	/// Compute a compass heading in degrees [0, 360)
	pub async fn heading_deg<D: DelayMs>(&mut self, delay: &mut D) -> Result<f32, Error> {
		let (x, y, _z) = self.read_ut(delay).await?;
		let h = atan2_approx(y, x);
		Ok(if h < 0.0 { h + 360.0 } else { h })
	}

	// ----- Low-level helpers -----

	async fn read_register(&mut self, reg: u8) -> Result<u8, Error> {
		// Read 3 bytes via repeated-start and discard first 2 dummy bytes
		let mut buf = [0u8; 3];
		self.i2c
			.write_read(self.addr, &[reg], &mut buf)
			.await
			.map_err(|_| Error::I2c)?;
		if self.verbose {
			debug!(
				"BMM350 reg 0x{:02X} -> raw {=[u8]} value 0x{:02X}",
				reg,
				buf,
				buf[2]
			);
		}
		Ok(buf[2])
	}

	async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), Error> {
		self.i2c
			.write(self.addr, &[reg, val])
			.await
			.map_err(|_| Error::I2c)
	}

	async fn load_otp<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
		self.write_register(registers::OTP_CMD, 0x80).await?;
		// Datasheet: up to ~8ms for OTP load
		delay.delay_ms(8).await;
		self.wait_not_busy(delay, 20).await?;

		let mut buf = [0u8; 23];
		self.i2c
			.write_read(self.addr, &[registers::OTP_BASE], &mut buf)
			.await
			.map_err(|_| Error::I2c)?;
		let data: [u8; 21] = buf[2..23].try_into().map_err(|_| Error::I2c)?;
		self.otp.data = data;
		if self.verbose {
			let mut first_six = [0u8; 6];
			first_six.copy_from_slice(&buf[0..6]);
			debug!(
				"BMM350 OTP first6 {=[u8]} last 0x{:02X} first_valid 0x{:02X}",
				first_six,
				buf[22],
				self.otp.data[0]
			);
		}
		Ok(())
	}

	/// Wait until PMU_STATUS is not busy. `timeout_ms` caps total wait.
	async fn wait_not_busy<D: DelayMs>(&mut self, delay: &mut D, timeout_ms: u32) -> Result<(), Error> {
		for _ in 0..timeout_ms {
			if let Ok(s) = self.read_register(registers::PMU_STATUS).await {
				if (s & 0x01) == 0 { return Ok(()); }
			}
			delay.delay_ms(1).await;
		}
		Err(Error::BusyTimeout)
	}

	// (no wait_not_busy_no_delay to keep this crate HAL-agnostic)
}

// ---- Math helpers (atan approximation) ----

fn atan2_approx(y: f32, x: f32) -> f32 {
	if x == 0.0 && y == 0.0 { return 0.0; }
	let abs_y = y.abs();
	let base_angle = match (x >= 0.0, y >= 0.0) {
		(true, _) => atan_approx(y / x),
		(false, true) => core::f32::consts::FRAC_PI_2 - atan_approx(x / y),
		(false, false) => core::f32::consts::FRAC_PI_2 - atan_approx(x / abs_y),
	};
	let mut angle = base_angle;
	if x < 0.0 { angle = core::f32::consts::PI - angle; }
	if y < 0.0 { angle = -angle; }
	angle.to_degrees()
}

fn atan_approx(z: f32) -> f32 {
	let abs_z = z.abs();
	if abs_z > 1.0 { return core::f32::consts::FRAC_PI_2 - atan_approx(1.0 / abs_z); }
	let zz = abs_z * abs_z;
	let mut res = abs_z;
	let mut term = abs_z * zz;
	res -= term / 3.0; term *= zz; res += term / 5.0; term *= zz; res -= term / 7.0;
	term *= zz; res += term / 9.0; term *= zz; res -= term / 11.0;
	if z < 0.0 { -res } else { res }
}
