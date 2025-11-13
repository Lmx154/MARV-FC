//! Async BMP390/BMP388 I2C driver (embedded-hal-async)
//!
//! Hardware configuration agnostic. Designed for use with Embassy but does not
//! depend on any particular HAL. Uses `embedded-hal-async` I2C traits and a
//! simple async delay trait from `common::utils::delay`.

#![allow(dead_code)]

use defmt::{debug, error, warn, Format};
use embedded_hal_async::i2c::I2c;

use crate::utils::delay::DelayMs;

// I2C addresses
pub const BMP388_ADDR_SDO_LOW: u8 = 0x76;
pub const BMP388_ADDR_SDO_HIGH: u8 = 0x77;
pub const BMP3X_ADDR_SDO_LOW: u8 = BMP388_ADDR_SDO_LOW;
pub const BMP3X_ADDR_SDO_HIGH: u8 = BMP388_ADDR_SDO_HIGH;

// Registers / constants
const REG_CHIP_ID: u8 = 0x00;
const CHIP_ID_388: u8 = 0x50;
const CHIP_ID_390: u8 = 0x60; // BMP390/BMP390L
const REG_ERR: u8 = 0x02; // ERR_REG
const REG_STATUS: u8 = 0x03; // STATUS (drdy and cmd_rdy)
const REG_PRESS_DATA: u8 = 0x04; // P[23:0], T[23:0]
const REG_PWR_CTRL: u8 = 0x1B;
const REG_OSR: u8 = 0x1C;
const REG_ODR: u8 = 0x1D;
const REG_CONFIG: u8 = 0x1F;
const REG_CALIB_START: u8 = 0x31;
const REG_CMD: u8 = 0x7E; // Command register (soft reset)
const CMD_SOFTRESET: u8 = 0xB6;

// Power control op mode bits are in REG_PWR_CTRL[5:4]:
// 00 = sleep, 01 = forced, 11 = normal. Use 0b11 (0x30) for normal stream.
const PWR_CTRL_NORMAL_MODE: u8 = 0x30;
const PWR_CTRL_TEMP_PRESS_EN: u8 = 0x03; // bits 1:0 enable temp+press

// Oversampling register encodings (BMP388):
const OSR_TEMP_1X: u8 = 0x00; // TEMP_OS pos=3 mask=0x38; 0x00 => 1x
const OSR_PRESS_1X: u8 = 0x00; // PRESS_OS pos=0 mask=0x07; 0x00 => 1x

// ODR codes (BMP388/BMP390):
// 0x00 => 200 Hz, 0x01 => 100 Hz, 0x02 => 50 Hz, 0x03 => 25 Hz, ...
const ODR_200HZ: u8 = 0x00;

// IIR filter coefficient: bits 2:1; 0x00 => off, 0x01 => 1, 0x02 => 2, 0x03 => 4, ...
const IIR_FILTER_OFF: u8 = 0x00;

const CALIB_DATA_LEN: usize = 21;
const DATA_LEN: usize = 6;

macro_rules! bmp_dbg {
	($s:expr, $($arg:tt)*) => {
		if $s.verbose { debug!($($arg)*); }
	};
}

#[derive(Clone, Copy, Default, Format)]
pub struct CalibData {
	pub par_t1: f64,
	pub par_t2: f64,
	pub par_t3: f64,
	pub par_p1: f64,
	pub par_p2: f64,
	pub par_p3: f64,
	pub par_p4: f64,
	pub par_p5: f64,
	pub par_p6: f64,
	pub par_p7: f64,
	pub par_p8: f64,
	pub par_p9: f64,
	pub par_p10: f64,
	pub par_p11: f64,
	pub t_lin: f64,
}

#[derive(Debug, Format)]
pub enum Error {
	I2c,
	InvalidChipId(u8),
	Timeout,
}

/// Async BMP3x driver owning the I2C bus
pub struct Bmp390<I2C>
where
	I2C: I2c,
{
	i2c: I2C,
	address: u8,
	pub calib: CalibData,
	verbose: bool,
}

impl<I2C> Bmp390<I2C>
where
	I2C: I2c,
{
	/// Create new driver with I2C bus and device address.
	pub fn new(i2c: I2C, address: u8) -> Self {
		Self { i2c, address, calib: CalibData::default(), verbose: false }
	}

	/// Enable/disable verbose debugging.
	pub fn set_debug(&mut self, enable: bool) { self.verbose = enable; }

	/// Release the I2C bus back to the caller.
	pub fn release(self) -> I2C { self.i2c }

	// ----- Low-level helpers -----
	async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error> {
		self.i2c.write(self.address, &[reg, val]).await.map_err(|_| Error::I2c)
	}
	async fn read_reg(&mut self, reg: u8) -> Result<u8, Error> {
		let mut b = [0u8; 1];
		self.i2c
			.write_read(self.address, &[reg], &mut b)
			.await
			.map_err(|_| Error::I2c)?;
		Ok(b[0])
	}
	async fn read_many(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error> {
		self.i2c
			.write_read(self.address, &[reg], buf)
			.await
			.map_err(|_| Error::I2c)
	}

	#[inline]
	async fn wait_cmd_ready<D: DelayMs>(&mut self, delay: &mut D, mut tries: u8) -> Result<(), Error> {
		// STATUS bit4 (cmd_rdy) indicates command interface ready
		while tries > 0 {
			let s = self.read_reg(REG_STATUS).await?;
			if (s & 0x10) != 0 { return Ok(()); }
			delay.delay_ms(1).await;
			tries -= 1;
		}
		warn!("BMP390: cmd_rdy wait timed out");
		Err(Error::Timeout)
	}

	#[inline]
	async fn ensure_normal_mode<D: DelayMs>(&mut self, delay: &mut D, retries: u8) -> Result<(), Error> {
		let mut left = retries;
		while left > 0 {
			let p = self.read_reg(REG_PWR_CTRL).await?;
			if (p & PWR_CTRL_NORMAL_MODE) == PWR_CTRL_NORMAL_MODE { return Ok(()); }
			let _ = self.wait_cmd_ready(delay, 50).await;
			self.write_reg(REG_PWR_CTRL, PWR_CTRL_TEMP_PRESS_EN | PWR_CTRL_NORMAL_MODE).await?;
			delay.delay_ms(10).await;
			left -= 1;
		}
		let p = self.read_reg(REG_PWR_CTRL).await.unwrap_or(0xFF);
		error!("BMP390: failed to latch normal mode, PWR=0x{:02X}", p);
		Err(Error::Timeout)
	}

	/// Initialize the device: probe, reset, read calibration, configure ODR/OSR/IIR, normal mode.
	pub async fn init<D: DelayMs>(&mut self, delay: &mut D) -> Result<(), Error> {
		// Probe with retries
		let mut retries = 5u8;
		let mut id = 0u8;
		while retries > 0 {
			match self.read_reg(REG_CHIP_ID).await {
				Ok(v) => {
					id = v;
					if id == CHIP_ID_388 || id == CHIP_ID_390 { break; }
				}
				Err(_) => {}
			}
			delay.delay_ms(1).await;
			retries -= 1;
		}
		if !(id == CHIP_ID_388 || id == CHIP_ID_390) { return Err(Error::InvalidChipId(id)); }

		// Optional: soft reset for clean start
		self.write_reg(REG_CMD, CMD_SOFTRESET).await?;
		delay.delay_ms(10).await;

		// Read back basic regs after reset
		let pwr0 = self.read_reg(REG_PWR_CTRL).await.unwrap_or(0xFF);
		let stat0 = self.read_reg(REG_STATUS).await.unwrap_or(0xFF);
		let model = if id == CHIP_ID_390 { "BMP390" } else { "BMP388" };
		bmp_dbg!(self, "After reset: {} CHIP_ID=0x{:02X} PWR=0x{:02X} STATUS=0x{:02X}", model, id, pwr0, stat0);

		// Clear EVENT register (0x10) to handle POR detection (ignore result)
		let _ = self.read_reg(0x10).await;
		bmp_dbg!(self, "Cleared EVENT register after POR");

		// Read calib (21 bytes from 0x31, no CRC)
		let mut calib = [0u8; CALIB_DATA_LEN];
		self.read_many(REG_CALIB_START, &mut calib).await?;
		bmp_dbg!(self, "Full calib: {=[u8]:X}", calib);

		let t1 = u16::from_le_bytes([calib[0], calib[1]]) as f64;
		let t2 = i16::from_le_bytes([calib[2], calib[3]]) as f64; // s16
		let t3 = (calib[4] as i8) as f64;
		let p1 = i16::from_le_bytes([calib[5], calib[6]]) as f64;
		let p2 = i16::from_le_bytes([calib[7], calib[8]]) as f64;
		let p3 = (calib[9] as i8) as f64;
		let p4 = (calib[10] as i8) as f64;
		let p5 = u16::from_le_bytes([calib[11], calib[12]]) as f64;
		let p6 = u16::from_le_bytes([calib[13], calib[14]]) as f64;
		let p7 = (calib[15] as i8) as f64;
		let p8 = (calib[16] as i8) as f64;
		let p9 = i16::from_le_bytes([calib[17], calib[18]]) as f64;
		let p10 = (calib[19] as i8) as f64;
		let p11 = (calib[20] as i8) as f64;

		// Scale per BMP388 datasheet (exact formulas)
		self.calib.par_t1 = t1 * 256.0;
		self.calib.par_t2 = t2 / 1073741824.0; // 2^30
		self.calib.par_t3 = t3 / 281474976710656.0; // 2^48
		self.calib.par_p1 = (p1 - 16384.0) / 1048576.0; // (2^14)/2^20
		self.calib.par_p2 = (p2 - 16384.0) / 536870912.0; // (2^14)/2^29
		self.calib.par_p3 = p3 / 4294967296.0; // 2^32
		self.calib.par_p4 = p4 / 137438953472.0; // 2^37
		self.calib.par_p5 = p5 * 8.0; // per Bosch API float
		self.calib.par_p6 = p6 / 64.0; // 2^6
		self.calib.par_p7 = p7 / 256.0; // 2^8
		self.calib.par_p8 = p8 / 32768.0; // 2^15
		self.calib.par_p9 = p9 / 281474976710656.0; // 2^48
		self.calib.par_p10 = p10 / 281474976710656.0; // 2^48
		self.calib.par_p11 = p11 / 36893488147419103232.0; // 2^65
		bmp_dbg!(self, "calib t1={:?} t2={:?} t3={:?} p1={:?} p2={:?} p5={:?}", self.calib.par_t1 as f32, self.calib.par_t2 as f32, self.calib.par_t3 as f32, self.calib.par_p1 as f32, self.calib.par_p2 as f32, self.calib.par_p5 as f32);

		// Configure OSR/ODR/IIR while in sleep
		self.write_reg(REG_OSR, OSR_TEMP_1X | OSR_PRESS_1X).await?;
		delay.delay_ms(2).await;
		let _ = self.read_reg(REG_OSR).await.unwrap_or(0xFF);
		let _ = self.read_reg(REG_STATUS).await.unwrap_or(0xFF);

		// Target 200 Hz
		self.write_reg(REG_ODR, ODR_200HZ).await?;
		delay.delay_ms(2).await;
		let _ = self.read_reg(REG_ODR).await.unwrap_or(0xFF);

		// IIR off initially
		self.write_reg(REG_CONFIG, IIR_FILTER_OFF).await?;
		delay.delay_ms(2).await;
		let _ = self.read_reg(REG_CONFIG).await.unwrap_or(0xFF);

		// Enable sensors in sleep, then enter normal mode
		self.wait_cmd_ready(delay, 50).await?;
		self.write_reg(REG_PWR_CTRL, PWR_CTRL_TEMP_PRESS_EN).await?;
		delay.delay_ms(2).await;
		let _ = self.read_reg(REG_STATUS).await;

		self.wait_cmd_ready(delay, 50).await?;
		self
			.write_reg(REG_PWR_CTRL, PWR_CTRL_TEMP_PRESS_EN | PWR_CTRL_NORMAL_MODE)
			.await?;
		delay.delay_ms(10).await;
		let pwr = self.read_reg(REG_PWR_CTRL).await?;
		if (pwr & PWR_CTRL_NORMAL_MODE) != PWR_CTRL_NORMAL_MODE {
			let _ = self.read_reg(REG_ERR).await.unwrap_or(0xFF);
			self.ensure_normal_mode(delay, 3).await?;
		}

		// Optional: Read EVENT 0x10 to clear POR if needed
		let _ = self.read_reg(0x10).await?;

		Ok(())
	}

	/// Read compensated values.
	/// Returns (temperature_c_x100, pressure_pa), both rounded to nearest i32.
	pub async fn read_compensated<D: DelayMs>(&mut self, delay: &mut D) -> Result<(i32, i32), Error> {
		// Poll STATUS for both temp and press data ready: bits 6 and 5
		let mut tries: u16 = 0;
		loop {
			let s = self.read_reg(REG_STATUS).await?;
			if (s & 0x60) == 0x60 { break; }
			delay.delay_ms(2).await;
			tries += 1;
			if tries > 50 { return Err(Error::Timeout); }
		}

		// Burst read 6 bytes: 0x04..0x09 => P[23:0], T[23:0]
		let mut buf = [0u8; DATA_LEN];
		self.read_many(REG_PRESS_DATA, &mut buf).await?;

		// Assemble little-endian 24-bit values
		let up = ((buf[2] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[0] as u32);
		let ut = ((buf[5] as u32) << 16) | ((buf[4] as u32) << 8) | (buf[3] as u32);

		// Compensation
		let ut_f = ut as f64;
		let up_f = up as f64;
		let dt = ut_f - self.calib.par_t1;
		let t_lin = dt * self.calib.par_t2 + dt * dt * self.calib.par_t3;
		self.calib.t_lin = t_lin;

		let pd1 = self.calib.par_p6 * t_lin;
		let pd2 = self.calib.par_p7 * t_lin * t_lin;
		let pd3 = self.calib.par_p8 * t_lin * t_lin * t_lin;
		let out1 = self.calib.par_p5 + pd1 + pd2 + pd3;

		let pd4 = self.calib.par_p2 * t_lin;
		let pd5 = self.calib.par_p3 * t_lin * t_lin;
		let pd6 = self.calib.par_p4 * t_lin * t_lin * t_lin;
		let out2 = up_f * (self.calib.par_p1 + pd4 + pd5 + pd6);

		let pd7 = up_f * up_f;
		let pd8 = self.calib.par_p9 + self.calib.par_p10 * t_lin;
		let pd9 = pd7 * (self.calib.par_p11 + pd8);
		let p_pa = out1 + out2 + pd9;

		// no_std-friendly rounding
		let t_c_x100 = t_lin * 100.0;
		let t_i32 = if t_c_x100.is_finite() { (t_c_x100 + if t_c_x100 >= 0.0 { 0.5 } else { -0.5 }) as i32 } else { 0 };
		let p_i32f = p_pa;
		let p_i32 = if p_i32f.is_finite() { (p_i32f + if p_i32f >= 0.0 { 0.5 } else { -0.5 }) as i32 } else { 0 };

		Ok((t_i32, p_i32))
	}

	/// Debug helper to dump key registers for diagnosing issues
	pub async fn dump_regs(&mut self) {
		let _chip = self.read_reg(REG_CHIP_ID).await.unwrap_or(0xFF);
		let _err = self.read_reg(REG_ERR).await.unwrap_or(0xFF);
		let _stat = self.read_reg(REG_STATUS).await.unwrap_or(0xFF);
		let _pwr = self.read_reg(REG_PWR_CTRL).await.unwrap_or(0xFF);
		let _osr = self.read_reg(REG_OSR).await.unwrap_or(0xFF);
		let _odr = self.read_reg(REG_ODR).await.unwrap_or(0xFF);
		let _cfg = self.read_reg(REG_CONFIG).await.unwrap_or(0xFF);
		let _event = self.read_reg(0x10).await.unwrap_or(0xFF);
		let _ = (_chip, _err, _stat, _pwr, _osr, _odr, _cfg, _event);
	}
}

// BMP3x alias type for external naming if preferred
pub type Bmp3x<I2C> = Bmp390<I2C>;

