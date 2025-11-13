//! Async u-blox NEO-M9N GPS driver (I2C/DDC) with minimal UBX NAV-PVT parser
//!
//! Hardware configuration agnostic and compatible with Embassy via
//! `embedded-hal-async` I2C traits. This driver owns the I2C bus instance
//! (like the other drivers) and exposes simple init/poll/read helpers.
//!
//! Notes
//! - The u-blox I2C (aka DDC) interface exposes a FIFO stream at 7-bit addr
//!   0x42. A host I2C read consumes bytes from this FIFO; I2C write pushes
//!   frames (UBX or NMEA) into the module.
//! - This lightweight driver focuses on NAV-PVT (class=0x01, id=0x07) only.
//! - Higher layers should periodically call `read_and_parse()` to feed bytes
//!   into the internal UBX parser and pick up parsed `GpsData`.

#![allow(dead_code)]

use defmt::Format;
use embedded_hal_async::i2c::I2c;

use crate::utils::delay::DelayMs;

/// Default 7-bit I2C address for u-blox DDC
pub const UBLOX_I2C_ADDR: u8 = 0x42;

#[derive(Debug, Format)]
pub enum Error {
	I2c,
}

/// Parsed subset of UBX NAV-PVT payload
#[derive(Copy, Clone, Default, Debug, Format)]
pub struct GpsData {
	pub year: u16,
	pub month: u8,
	pub day: u8,
	pub hour: u8,
	pub minute: u8,
	pub second: u8,
	/// Latitude in 1e-7 degrees
	pub latitude: i32,
	/// Longitude in 1e-7 degrees
	pub longitude: i32,
	/// Mean sea level altitude in millimeters
	pub altitude: i32,
	pub satellites: u8,
	/// Fix type (0=no fix, 3=3D fix, etc.)
	pub fix_type: u8,
}

/// Async NEO-M9N driver holding I2C bus instance
pub struct NeoM9n<I2C>
where
	I2C: I2c,
{
	i2c: I2C,
	addr: u8,
	parser: UbxParser,
	last: Option<GpsData>,
	verbose: bool,
}

impl<I2C> NeoM9n<I2C>
where
	I2C: I2c,
{
	/// Create a new driver with given I2C bus and optional address
	pub fn new(i2c: I2C, address: u8) -> Self {
		Self { i2c, addr: address, parser: UbxParser::new(), last: None, verbose: false }
	}

	/// Enable/disable verbose logging at runtime
	pub fn set_debug(&mut self, enable: bool) { self.verbose = enable; }

	/// Release the owned I2C bus
	pub fn release(self) -> I2C { self.i2c }

	/// Initialize the device by sending a NAV-PVT poll.
	/// This verifies the I2C write path without assuming a response timing.
	pub async fn init<D: DelayMs>(&mut self, _delay: &mut D) -> Result<(), Error> {
		self.poll_nav_pvt().await?;
		Ok(())
	}

	/// Send a UBX NAV-PVT poll frame (B5 62 01 07 00 00 08 19)
	pub async fn poll_nav_pvt(&mut self) -> Result<(), Error> {
		// DDC requires writing to subaddress 0xFF to push stream bytes
		const PAYLOAD: [u8; 8] = [0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19];
		let mut buf = [0u8; 1 + 8];
		buf[0] = 0xFF; // DDC data port
		buf[1..].copy_from_slice(&PAYLOAD);
		self.i2c.write(self.addr, &buf).await.map_err(|_| Error::I2c)
	}

	/// Attempt to read available bytes from the DDC FIFO and feed the parser.
	/// This checks the 0xFD length register first, then reads up to `buf.len()`
	/// bytes from 0xFF. Returns Ok(Some(GpsData)) if a NAV-PVT frame was decoded.
	pub async fn read_and_parse<D: DelayMs>(
		&mut self,
		_delay: &mut D,
		buf: &mut [u8],
	) -> Result<Option<GpsData>, Error> {
		if buf.is_empty() { return Ok(None); }
		// Query number of bytes available in the RX buffer (0xFD returns u16 LE)
		let mut len_bytes = [0u8; 2];
		self.i2c
			.write_read(self.addr, &[0xFD], &mut len_bytes)
			.await
			.map_err(|_| Error::I2c)?;
		let available = u16::from_le_bytes(len_bytes) as usize;
		if available == 0 { return Ok(None); }

		let to_read = core::cmp::min(buf.len(), available);
		// Read stream bytes from 0xFF
		self.i2c
			.write_read(self.addr, &[0xFF], &mut buf[..to_read])
			.await
			.map_err(|_| Error::I2c)?;
		let mut new_fix: Option<GpsData> = None;
		for &b in buf[..to_read].iter() {
			if let Some(d) = self.parser.parse_byte(b) {
				self.last = Some(d);
				new_fix = Some(d);
				// keep parsing remaining bytes to flush the stream
			}
		}
		Ok(new_fix)
	}

	/// Return the most recently parsed NAV-PVT sample, if any
	pub fn last(&self) -> Option<GpsData> { self.last }
}

// -------------------- Minimal UBX parser (NAV-PVT only) --------------------

#[derive(PartialEq)]
enum ParserState { Sync1, Sync2, Header, Payload, Checksum }

struct UbxParser {
	state: ParserState,
	header: [u8; 4],
	payload: [u8; 92],
	h_idx: usize,
	p_idx: usize,
	p_len: u16,
	ck_a: u8,
	ck_b: u8,
	calc_a: u8,
	calc_b: u8,
	cidx: u8,
}

const SYNC1: u8 = 0xB5;
const SYNC2: u8 = 0x62;

impl UbxParser {
	fn new() -> Self {
		Self {
			state: ParserState::Sync1,
			header: [0; 4],
			payload: [0; 92],
			h_idx: 0,
			p_idx: 0,
			p_len: 0,
			ck_a: 0,
			ck_b: 0,
			calc_a: 0,
			calc_b: 0,
			cidx: 0,
		}
	}

	fn reset(&mut self) {
		self.state = ParserState::Sync1;
		self.h_idx = 0;
		self.p_idx = 0;
		self.p_len = 0;
		self.calc_a = 0;
		self.calc_b = 0;
		self.cidx = 0;
	}

	#[inline]
	fn add_ck(&mut self, b: u8) {
		self.calc_a = self.calc_a.wrapping_add(b);
		self.calc_b = self.calc_b.wrapping_add(self.calc_a);
	}

	/// Feed one byte; return Some(GpsData) when a valid NAV-PVT completes
	fn parse_byte(&mut self, b: u8) -> Option<GpsData> {
		match self.state {
			ParserState::Sync1 => {
				if b == SYNC1 { self.state = ParserState::Sync2; }
			}
			ParserState::Sync2 => {
				if b == SYNC2 {
					self.state = ParserState::Header;
					self.h_idx = 0;
					self.calc_a = 0;
					self.calc_b = 0;
				} else {
					self.reset();
					if b == SYNC1 { self.state = ParserState::Sync2; }
				}
			}
			ParserState::Header => {
				self.header[self.h_idx] = b;
				self.add_ck(b);
				self.h_idx += 1;
				if self.h_idx == 4 {
					self.p_len = ((self.header[3] as u16) << 8) | (self.header[2] as u16);
					let class = self.header[0];
					let id = self.header[1];
					if class == 0x01 && id == 0x07 && self.p_len == 92 {
						self.state = ParserState::Payload;
						self.p_idx = 0;
					} else {
						self.reset();
					}
				}
			}
			ParserState::Payload => {
				if (self.p_idx as u16) < self.p_len && self.p_idx < self.payload.len() {
					self.payload[self.p_idx] = b;
				}
				self.add_ck(b);
				self.p_idx += 1;
				if (self.p_idx as u16) == self.p_len { self.state = ParserState::Checksum; self.cidx = 0; }
			}
			ParserState::Checksum => {
				if self.cidx == 0 { self.ck_a = b; self.cidx = 1; }
				else { self.ck_b = b; if self.calc_a == self.ck_a && self.calc_b == self.ck_b { let r = self.parse_nav_pvt(); self.reset(); return r; } else { self.reset(); } }
			}
		}
		None
	}

	fn parse_nav_pvt(&self) -> Option<GpsData> {
		let p = &self.payload;
		let mut g = GpsData::default();
		g.year = u16::from_le_bytes([p[4], p[5]]);
		g.month = p[6];
		g.day = p[7];
		g.hour = p[8];
		g.minute = p[9];
		g.second = p[10];
		g.fix_type = p[20];
		g.satellites = p[23];
		g.longitude = i32::from_le_bytes([p[24], p[25], p[26], p[27]]);
		g.latitude = i32::from_le_bytes([p[28], p[29], p[30], p[31]]);
		g.altitude = i32::from_le_bytes([p[36], p[37], p[38], p[39]]);
		Some(g)
	}
}

