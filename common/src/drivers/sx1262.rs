//! Async, MCU-agnostic SX1262 LoRa transceiver driver
//!
//! Notes:
//! - Uses embedded-hal-async SPI and embedded-hal digital pins
//! - No board-specific wiring; caller passes generic pins and an RF switch abstraction
//! - For modules with internal RF switch (DIO2 control), set `RfSwitchMode::Dio2`
//! - For modules with external TXEN/RXEN (e.g., Waveshare Core1262-HF), set `RfSwitchMode::External`
//!   and implement `RfSwitch` to toggle GPIOs appropriately.

#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::{info, warn, Format};
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::spi::SpiBus;

use crate::utils::delay::DelayMs;

// ------------------ Public types ------------------

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum Sx1262Mode {
	Unknown,
	Sleep,
	StandbyRc,
	StandbyXosc,
	Fs,
	Rx,
	Tx,
	Cad,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum Sx1262Error {
	BusyTimeout,
	Spi,
	Cs,
	InvalidParam,
	InvalidTransition { from: Sx1262Mode, to: Sx1262Mode },
}

pub type Result<T> = core::result::Result<T, Sx1262Error>;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum LoRaSpreadingFactor { Sf5=5, Sf6=6, Sf7=7, Sf8=8, Sf9=9, Sf10=10, Sf11=11, Sf12=12 }
impl LoRaSpreadingFactor { fn param(self)->u8 { self as u8 } }

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum LoRaBandwidth { Bw7_81, Bw10_42, Bw15_63, Bw20_83, Bw31_25, Bw41_67, Bw62_5, Bw125, Bw250, Bw500 }
impl LoRaBandwidth { fn param(self)->u8 { match self { Self::Bw7_81=>0x00, Self::Bw10_42=>0x01, Self::Bw15_63=>0x02, Self::Bw20_83=>0x03, Self::Bw31_25=>0x04, Self::Bw41_67=>0x05, Self::Bw62_5=>0x06, Self::Bw125=>0x07, Self::Bw250=>0x08, Self::Bw500=>0x09 } } }

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum LoRaCodingRate { Cr45, Cr46, Cr47, Cr48 }
impl LoRaCodingRate { fn param(self)->u8 { match self { Self::Cr45=>0x01, Self::Cr46=>0x02, Self::Cr47=>0x03, Self::Cr48=>0x04 } } }

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum TxRamp { Us10, Us20, Us40, Us80, Us160, Us200, Us240, Us300 }
impl TxRamp { fn param(self)->u8 { match self { Self::Us10=>0x00, Self::Us20=>0x01, Self::Us40=>0x02, Self::Us80=>0x03, Self::Us160=>0x04, Self::Us200=>0x05, Self::Us240=>0x06, Self::Us300=>0x07 } } }

#[derive(Copy, Clone, Debug)]
pub struct Sx1262Config {
	pub use_dcdc: bool,
	pub xtal_freq_hz: u32,
	pub lora_sf: LoRaSpreadingFactor,
	pub lora_bw: LoRaBandwidth,
	pub lora_cr: LoRaCodingRate,
	pub lora_ldro: bool,
	pub preamble_len: u16,
	pub explicit_header: bool,
	pub crc_on: bool,
	pub invert_iq: bool,
	pub pa_duty_cycle: u8,
	pub pa_hp_max: u8,
	pub tx_power: i8,
	pub tx_ramp: TxRamp,
}
impl Default for Sx1262Config { fn default()->Self { Self {
	use_dcdc: true,
	xtal_freq_hz: 32_000_000,
	lora_sf: LoRaSpreadingFactor::Sf7,
	lora_bw: LoRaBandwidth::Bw125,
	lora_cr: LoRaCodingRate::Cr45,
	lora_ldro: false,
	preamble_len: 12,
	explicit_header: true,
	crc_on: true,
	invert_iq: false,
	pa_duty_cycle: 0x04,
	pa_hp_max: 0x07,
	tx_power: 14,
	tx_ramp: TxRamp::Us200,
} } }

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum RfState { Off, Rx, Tx }

/// Abstraction for modules without DIO2 RF switch. Implement using external TXEN/RXEN GPIOs.
pub trait RfSwitch { fn set(&mut self, _state: RfState) {} }

/// Choose how RF switch is handled.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum RfSwitchMode { Dio2, External }

#[derive(Copy, Clone, Debug, Format)]
pub struct TxReport { pub irq: u16, pub done: bool, pub timeout: bool }

#[derive(Copy, Clone, Debug, Format)]
pub struct RxReport { pub irq: u16, pub done: bool, pub timeout: bool, pub len: u8, pub rssi: i16, pub snr_x4: i16 }

// ------------------ Driver ------------------

pub struct Sx1262<SPI, NSS, RESET, BUSY, DIO1, DIO2, SW>
where
	SPI: SpiBus<u8>,
	NSS: OutputPin,
	RESET: OutputPin,
	BUSY: InputPin,
	DIO1: InputPin,
	DIO2: InputPin,
	SW: RfSwitch,
{
	spi: SPI,
	nss: NSS,
	reset: RESET,
	busy: BUSY,
	dio1: DIO1,
	dio2: DIO2,
	rf_switch: SW,
	mode: Sx1262Mode,
	cfg: Sx1262Config,
	rf_mode: RfSwitchMode,
}

impl<SPI, NSS, RESET, BUSY, DIO1, DIO2, SW> Sx1262<SPI, NSS, RESET, BUSY, DIO1, DIO2, SW>
where
	SPI: SpiBus<u8>,
	NSS: OutputPin,
	RESET: OutputPin,
	BUSY: InputPin,
	DIO1: InputPin,
	DIO2: InputPin,
	SW: RfSwitch,
{
	pub fn new(spi: SPI, nss: NSS, reset: RESET, busy: BUSY, dio1: DIO1, dio2: DIO2, rf_switch: SW, rf_mode: RfSwitchMode, cfg: Sx1262Config) -> Self {
		Self { spi, nss, reset, busy, dio1, dio2, rf_switch, mode: Sx1262Mode::Unknown, cfg, rf_mode }
	}

	pub fn mode(&self) -> Sx1262Mode { self.mode }
	pub fn cfg(&self) -> &Sx1262Config { &self.cfg }
	pub fn cfg_mut(&mut self) -> &mut Sx1262Config { &mut self.cfg }

	// ---------- Core helpers ----------
	async fn wait_not_busy<D: DelayMs>(&mut self, delay: &mut D, timeout_ms: u32) -> Result<()> {
		let mut waited = 0u32;
		while self.busy.is_high().unwrap_or(true) {
			if waited >= timeout_ms { return Err(Sx1262Error::BusyTimeout); }
			delay.delay_ms(1).await;
			waited += 1;
		}
		Ok(())
	}

	async fn write_cmd<D: DelayMs>(&mut self, delay: &mut D, opcode: u8, params: &[u8]) -> Result<()> {
		self.wait_not_busy(delay, 50).await?;
		self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
		let mut frame = [0u8; 1];
		frame[0] = opcode;
		self.spi.write(&frame).await.map_err(|_| Sx1262Error::Spi)?;
		if !params.is_empty() { self.spi.write(params).await.map_err(|_| Sx1262Error::Spi)?; }
		self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
		Ok(())
	}

	async fn read_cmd<const N: usize, D: DelayMs>(&mut self, delay: &mut D, opcode: u8) -> Result<[u8; N]> {
		self.wait_not_busy(delay, 50).await?;
		assert!(N <= 16, "read_cmd N too large");
		let mut tx = [0u8; 18]; // max: opcode + status + 16 bytes
		let mut rx = [0u8; 18];
		tx[0] = opcode;
		let count = N + 2;
		self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
		self.spi.transfer(&mut rx[..count], &tx[..count]).await.map_err(|_| Sx1262Error::Spi)?;
		self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
		let mut out = [0u8; N];
		out.copy_from_slice(&rx[2..count]);
		Ok(out)
	}

	async fn get_status<D: DelayMs>(&mut self, delay: &mut D) -> Result<u8> {
		self.wait_not_busy(delay, 20).await?;
	let tx = [0xC0u8, 0x00];
		let mut rx = [0u8; 2];
		self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
		self.spi.transfer(&mut rx, &tx).await.map_err(|_| Sx1262Error::Spi)?;
		self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
		let st = rx[1];
		// Rough mode decode from bits6:4
		let mode_bits = (st >> 4) & 0x07;
		self.mode = match mode_bits { 0x2 => Sx1262Mode::StandbyRc, 0x3 => Sx1262Mode::StandbyXosc, 0x4 => Sx1262Mode::Fs, 0x5 => Sx1262Mode::Rx, 0x6 => Sx1262Mode::Tx, _ => Sx1262Mode::Unknown };
		Ok(st)
	}

	// ---------- Public init ----------
	pub async fn init_lora<D: DelayMs>(&mut self, delay: &mut D, freq_hz: u32) -> Result<()> {
		info!("SX1262: init");
		// Reset pulse >100us
		self.reset.set_low().map_err(|_| Sx1262Error::Cs)?;
		delay.delay_ms(1).await;
		self.reset.set_high().map_err(|_| Sx1262Error::Cs)?;
		self.wait_not_busy(delay, 20).await?;
		let _ = self.get_status(delay).await;

		// Regulator
		let mode = if self.cfg.use_dcdc { 0x02 } else { 0x01 };
		self.write_cmd(delay, 0x96, &[mode]).await?; // SetRegulatorMode

		// Packet type LoRa
		self.write_cmd(delay, 0x8A, &[0x01]).await?; // SetPacketType LoRa

		// Frequency
		self.set_rf_frequency(delay, freq_hz).await?;

		// Optionally enable DIO2 auto RF switch
		match self.rf_mode {
			RfSwitchMode::Dio2 => {
				self.write_cmd(delay, 0x9D, &[0x01]).await?; // SetDio2AsRfSwitchCtrl enable
			}
			RfSwitchMode::External => {
				// leave DIO2 untouched; external handled via trait
			}
		}

		// Apply LoRa params
		self.set_lora_modulation_params(delay).await?;
		self.set_lora_packet_params(delay, 255).await?; // allow up to 255 for RX
		self.set_pa_config(delay).await?;
		self.set_tx_params(delay).await?;

		// Buffer bases: TX=0x00 RX=0x80
		self.write_cmd(delay, 0x8F, &[0x00, 0x80]).await?;

		Ok(())
	}

	async fn set_rf_frequency<D: DelayMs>(&mut self, delay: &mut D, freq_hz: u32) -> Result<()> {
		let raw: u32 = (((freq_hz as u64) << 25) / 32_000_000u64) as u32;
		self.write_cmd(delay, 0x86, &raw.to_be_bytes()).await
	}

	async fn set_lora_modulation_params<D: DelayMs>(&mut self, delay: &mut D) -> Result<()> {
		let sf = self.cfg.lora_sf.param();
		let bw = self.cfg.lora_bw.param();
		let cr = self.cfg.lora_cr.param();
		let ldro = if self.cfg.lora_ldro { 0x01 } else { 0x00 };
		self.write_cmd(delay, 0x8B, &[sf, bw, cr, ldro]).await
	}

	async fn set_lora_packet_params<D: DelayMs>(&mut self, delay: &mut D, payload_len: u8) -> Result<()> {
		let pre = self.cfg.preamble_len.to_be_bytes();
		let header = if self.cfg.explicit_header { 0x00 } else { 0x01 };
		let crc = if self.cfg.crc_on { 0x01 } else { 0x00 };
		let iq = if self.cfg.invert_iq { 0x01 } else { 0x00 };
		let bytes = [pre[0], pre[1], header, payload_len, crc, iq];
		self.write_cmd(delay, 0x8C, &bytes).await
	}

	async fn set_pa_config<D: DelayMs>(&mut self, delay: &mut D) -> Result<()> {
		let duty = self.cfg.pa_duty_cycle.min(0x07);
		let hp = self.cfg.pa_hp_max.min(0x07);
		self.write_cmd(delay, 0x95, &[duty, hp, 0x00, 0x01]).await
	}

	async fn set_tx_params<D: DelayMs>(&mut self, delay: &mut D) -> Result<()> {
		let p = self.cfg.tx_power.clamp(-3, 22);
		let pbyte = if p < 0 { (256 + p as i16) as u8 } else { p as u8 };
		let ramp = self.cfg.tx_ramp.param();
		self.write_cmd(delay, 0x8E, &[pbyte, ramp]).await
	}

	async fn clear_irq_status<D: DelayMs>(&mut self, delay: &mut D, mask: u16) -> Result<()> {
		self.write_cmd(delay, 0x02, &mask.to_be_bytes()).await
	}

	async fn get_irq_status<D: DelayMs>(&mut self, delay: &mut D) -> Result<u16> {
		let bytes = self.read_cmd::<2, _>(delay, 0x12).await?;
		Ok(u16::from_be_bytes(bytes))
	}

	async fn set_dio_irq_params<D: DelayMs>(&mut self, delay: &mut D, irq_mask: u16, dio1_mask: u16) -> Result<()> {
		let frame = [
			(irq_mask >> 8) as u8, irq_mask as u8,
			(dio1_mask >> 8) as u8, dio1_mask as u8,
			0, 0, // DIO2
			0, 0, // DIO3
		];
		self.write_cmd(delay, 0x08, &frame).await
	}

	async fn write_buffer<D: DelayMs>(&mut self, delay: &mut D, offset: u8, data: &[u8]) -> Result<()> {
		if data.len() > 255 { return Err(Sx1262Error::InvalidParam); }
		self.wait_not_busy(delay, 20).await?;
		self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
		self.spi.write(&[0x0E, offset]).await.map_err(|_| Sx1262Error::Spi)?;
		if !data.is_empty() { self.spi.write(data).await.map_err(|_| Sx1262Error::Spi)?; }
		self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
		Ok(())
	}

	async fn get_rx_buffer_status<D: DelayMs>(&mut self, delay: &mut D) -> Result<(u8,u8)> {
		let b = self.read_cmd::<2, _>(delay, 0x13).await?;
		Ok((b[0], b[1]))
	}

	async fn read_buffer<D: DelayMs>(&mut self, delay: &mut D, offset: u8, out: &mut [u8]) -> Result<usize> {
		if out.is_empty() { return Ok(0); }
		let len = out.len().min(255);
		self.wait_not_busy(delay, 20).await?;
		self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
		self.spi.write(&[0x1E, offset, 0x00]).await.map_err(|_| Sx1262Error::Spi)?;
		let zeros = [0u8; 255];
		self.spi.transfer(&mut out[..len], &zeros[..len]).await.map_err(|_| Sx1262Error::Spi)?;
		self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
		Ok(len)
	}

	async fn get_packet_status<D: DelayMs>(&mut self, delay: &mut D) -> Result<[u8;3]> {
		self.read_cmd::<3, _>(delay, 0x14).await
	}

	async fn set_rx<D: DelayMs>(&mut self, delay: &mut D, timeout_syms: u32) -> Result<()> {
		let t = timeout_syms.min(0x00FF_FFFF);
		let b = [(t >> 16) as u8, (t >> 8) as u8, t as u8];
		self.write_cmd(delay, 0x82, &b).await
	}

	async fn set_tx<D: DelayMs>(&mut self, delay: &mut D, timeout_syms: u32) -> Result<()> {
		let t = timeout_syms.min(0x00FF_FFFF);
		let b = [(t >> 16) as u8, (t >> 8) as u8, t as u8];
		self.write_cmd(delay, 0x83, &b).await
	}

	// ------------------- TX/RX public API -------------------

	pub async fn start_rx_continuous<D: DelayMs>(&mut self, delay: &mut D) -> Result<()> {
		// Configure IRQ: RxDone | Timeout | HeaderValid | CrcErr
		const IRQ_RX_DONE: u16 = 0x0002;
		const IRQ_TIMEOUT: u16 = 0x0100;
		const IRQ_HEADER_VALID: u16 = 0x0010;
		const IRQ_CRC_ERR: u16 = 0x0040;
		let mask = IRQ_RX_DONE | IRQ_TIMEOUT | IRQ_HEADER_VALID | IRQ_CRC_ERR;
		self.clear_irq_status(delay, 0xFFFF).await?;
		self.set_dio_irq_params(delay, mask, mask).await?;
		if matches!(self.rf_mode, RfSwitchMode::External) { self.rf_switch.set(RfState::Rx); }
		self.set_rx(delay, 0x00FF_FFFF).await?; // continuous
		Ok(())
	}

	pub async fn poll_rx<D: DelayMs>(&mut self, delay: &mut D, buf: &mut [u8]) -> Result<Option<RxReport>> {
		let irq = self.get_irq_status(delay).await?;
		if irq == 0 { return Ok(None); }
		let done = (irq & 0x0002) != 0; // RxDone
		let timeout = (irq & 0x0100) != 0; // Timeout
		let mut len = 0u8;
		if done {
			let (l, start) = self.get_rx_buffer_status(delay).await?;
			len = l;
			let copy_len = core::cmp::min(len as usize, buf.len());
			let _ = self.read_buffer(delay, start, &mut buf[..copy_len]).await?;
		}
		let pkt = self.get_packet_status(delay).await.unwrap_or([0;3]);
		self.clear_irq_status(delay, irq).await?;
		let rssi = -(pkt[0] as i16) / 2; // approx dBm
		let snr_x4 = pkt[1] as i8 as i16; // 1/4 dB units
		Ok(Some(RxReport { irq, done, timeout, len, rssi, snr_x4 }))
	}

	pub async fn tx_send_blocking<D: DelayMs>(&mut self, delay: &mut D, payload: &[u8], timeout_syms: u32) -> Result<TxReport> {
		if payload.len() > 255 { return Err(Sx1262Error::InvalidParam); }
		// Prepare buffer and IRQs
		self.clear_irq_status(delay, 0xFFFF).await?;
		const IRQ_TX_DONE: u16 = 0x0001; const IRQ_TIMEOUT: u16 = 0x0100;
		let mask = IRQ_TX_DONE | IRQ_TIMEOUT;
		self.set_dio_irq_params(delay, mask, mask).await?;
		self.set_lora_packet_params(delay, payload.len() as u8).await?;
		self.write_buffer(delay, 0x00, payload).await?;
		if matches!(self.rf_mode, RfSwitchMode::External) { self.rf_switch.set(RfState::Tx); }
		self.set_tx(delay, timeout_syms).await?;

		// Poll for completion
		let mut waited = 0u32;
		loop {
			let irq = self.get_irq_status(delay).await?;
			if irq != 0 {
				let done = (irq & IRQ_TX_DONE) != 0;
				let timeout = (irq & IRQ_TIMEOUT) != 0;
				self.clear_irq_status(delay, irq).await?;
				if matches!(self.rf_mode, RfSwitchMode::External) { self.rf_switch.set(RfState::Off); }
				return Ok(TxReport { irq, done, timeout });
			}
			if waited > 2000 { // ~2s guard (depends on caller pacing)
				warn!("SX1262: TX wait exceeded 2s; forcing timeout report");
				if matches!(self.rf_mode, RfSwitchMode::External) { self.rf_switch.set(RfState::Off); }
				return Ok(TxReport { irq: 0, done: false, timeout: true });
			}
			delay.delay_ms(1).await;
			waited += 1;
		}
	}
}