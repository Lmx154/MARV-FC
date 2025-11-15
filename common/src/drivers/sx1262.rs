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
pub enum LoRaSpreadingFactor {
    Sf5 = 5,
    Sf6 = 6,
    Sf7 = 7,
    Sf8 = 8,
    Sf9 = 9,
    Sf10 = 10,
    Sf11 = 11,
    Sf12 = 12,
}
impl LoRaSpreadingFactor {
    fn param(self) -> u8 {
        self as u8
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum LoRaBandwidth {
    Bw7_81,
    Bw10_42,
    Bw15_63,
    Bw20_83,
    Bw31_25,
    Bw41_67,
    Bw62_5,
    Bw125,
    Bw250,
    Bw500,
}
impl LoRaBandwidth {
    fn param(self) -> u8 {
        // Semtech encoding (datasheet / sx126x.h): fine BWs + normal 125/250/500 values
        // NOTE: Only Bw125/Bw250/Bw500 are used in our current app; they must be 0x04/0x05/0x06.
        match self {
            Self::Bw7_81 => 0x00,
            Self::Bw10_42 => 0x08,
            Self::Bw15_63 => 0x01,
            Self::Bw20_83 => 0x09,
            Self::Bw31_25 => 0x02,
            Self::Bw41_67 => 0x0A,
            Self::Bw62_5 => 0x03,
            Self::Bw125 => 0x04,
            Self::Bw250 => 0x05,
            Self::Bw500 => 0x06,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum LoRaCodingRate {
    Cr45,
    Cr46,
    Cr47,
    Cr48,
}
impl LoRaCodingRate {
    fn param(self) -> u8 {
        match self {
            Self::Cr45 => 0x01,
            Self::Cr46 => 0x02,
            Self::Cr47 => 0x03,
            Self::Cr48 => 0x04,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum TxRamp {
    Us10,
    Us20,
    Us40,
    Us80,
    Us160,
    Us200,
    Us240,
    Us300,
}
impl TxRamp {
    fn param(self) -> u8 {
        match self {
            Self::Us10 => 0x00,
            Self::Us20 => 0x01,
            Self::Us40 => 0x02,
            Self::Us80 => 0x03,
            Self::Us160 => 0x04,
            Self::Us200 => 0x05,
            Self::Us240 => 0x06,
            Self::Us300 => 0x07,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Sx1262Config {
    pub use_dcdc: bool,
    // TCXO control (Waveshare Core1262-HF requires DIO3 to power TCXO)
    pub tcxo_enable: bool,
    pub tcxo_voltage_code: u8, // datasheet voltage enum: 0=1.6V .. 7=3.3V
    pub tcxo_delay_ms: u16,    // startup delay in ms (converted to 15.625us units)
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
    pub lora_sync_word: u16, // public 0x3444 default (private 0x1424)
}

impl Default for Sx1262Config {
    fn default() -> Self {
        Self {
            use_dcdc: true,
            tcxo_enable: true,
            tcxo_voltage_code: 0x02, // 1.8V (legacy known-good)
            tcxo_delay_ms: 20,       // longer warmup to avoid XOSC start issues
            xtal_freq_hz: 32_000_000,
            lora_sf: LoRaSpreadingFactor::Sf7,
            lora_bw: LoRaBandwidth::Bw125,
            lora_cr: LoRaCodingRate::Cr45,
            lora_ldro: false,
            preamble_len: 12,        // legacy preamble length
            explicit_header: true,
            crc_on: true,
            invert_iq: false,
            pa_duty_cycle: 0x04,
            pa_hp_max: 0x07,
            tx_power: 0,             // conservative starting TX power
            tx_ramp: TxRamp::Us40,   // faster ramp per legacy
            lora_sync_word: 0x3444,  // public sync by default (override to private if desired)
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum RfState {
    Off,
    Rx,
    Tx,
}

/// Abstraction for modules without DIO2 RF switch. Implement using external TXEN/RXEN GPIOs.
pub trait RfSwitch {
    fn set(&mut self, _state: RfState) {}
}

/// Choose how RF switch is handled.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Format)]
pub enum RfSwitchMode {
    Dio2,
    External,
}

#[derive(Copy, Clone, Debug, Format)]
pub struct TxReport {
    pub irq: u16,
    pub done: bool,
    pub timeout: bool,
}

#[derive(Copy, Clone, Debug, Format)]
pub struct RxReport {
    pub irq: u16,
    pub done: bool,
    pub timeout: bool,
    pub len: u8,
    pub rssi: i16,
    pub snr_x4: i16,
}

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
    pub fn new(
        spi: SPI,
        nss: NSS,
        reset: RESET,
        busy: BUSY,
        dio1: DIO1,
        dio2: DIO2,
        rf_switch: SW,
        rf_mode: RfSwitchMode,
        cfg: Sx1262Config,
    ) -> Self {
        Self {
            spi,
            nss,
            reset,
            busy,
            dio1,
            dio2,
            rf_switch,
            mode: Sx1262Mode::Unknown,
            cfg,
            rf_mode,
        }
    }

    pub fn mode(&self) -> Sx1262Mode {
        self.mode
    }
    pub fn cfg(&self) -> &Sx1262Config {
        &self.cfg
    }
    pub fn cfg_mut(&mut self) -> &mut Sx1262Config {
        &mut self.cfg
    }

    // ---------- Core helpers ----------
    async fn wait_not_busy<D: DelayMs>(&mut self, delay: &mut D, timeout_ms: u32) -> Result<()> {
        let mut waited = 0u32;
        while self.busy.is_high().unwrap_or(true) {
            if waited >= timeout_ms {
                return Err(Sx1262Error::BusyTimeout);
            }
            delay.delay_ms(1).await;
            waited += 1;
        }
        Ok(())
    }

    async fn write_cmd<D: DelayMs>(&mut self, delay: &mut D, opcode: u8, params: &[u8]) -> Result<()> {
        self.wait_not_busy(delay, 50).await?;
        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        let frame = [opcode];
        self.spi.write(&frame).await.map_err(|_| Sx1262Error::Spi)?;
        if !params.is_empty() {
            self.spi.write(params).await.map_err(|_| Sx1262Error::Spi)?;
        }
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
        Ok(())
    }

    async fn read_cmd<const N: usize, D: DelayMs>(
        &mut self,
        delay: &mut D,
        opcode: u8,
    ) -> Result<[u8; N]> {
        self.wait_not_busy(delay, 50).await?;
        assert!(N <= 16, "read_cmd N too large");

        let mut tx = [0u8; 18]; // opcode + status + up to 16 bytes
        let mut rx = [0u8; 18];
        tx[0] = opcode;
        let count = N + 2;

        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi
            .transfer(&mut rx[..count], &tx[..count])
            .await
            .map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;

        let mut out = [0u8; N];
        out.copy_from_slice(&rx[2..count]);
        Ok(out)
    }

    async fn write_registers<D: DelayMs>(
        &mut self,
        delay: &mut D,
        addr: u16,
        data: &[u8],
    ) -> Result<()> {
        // SX126x WriteRegister:
        // 0x0D, ADDR_MSB, ADDR_LSB, DATA...
        // NOTE: No dummy byte (dummy is only for ReadRegister)
        self.wait_not_busy(delay, 50).await?;

        if data.len() > 255 {
            return Err(Sx1262Error::InvalidParam);
        }

        let mut frame = [0u8; 3 + 255];
        frame[0] = 0x0D;
        frame[1] = (addr >> 8) as u8;
        frame[2] = addr as u8;
        frame[3..3 + data.len()].copy_from_slice(data);

        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi
            .write(&frame[..3 + data.len()])
            .await
            .map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
        Ok(())
    }

    pub async fn read_registers<const N: usize, D: DelayMs>(
        &mut self,
        delay: &mut D,
        addr: u16,
    ) -> Result<[u8; N]> {
        // SX126x ReadRegister:
        // 0x1D, ADDR_MSB, ADDR_LSB, DUMMY(0x00), N*DATA
        self.wait_not_busy(delay, 50).await?;
        assert!(N > 0 && N <= 255, "read_registers N out of range");

        let mut tx = [0u8; 4 + 255];
        let mut rx = [0u8; 4 + 255];
        let frame_len = 4 + N;

        tx[0] = 0x1D;
        tx[1] = (addr >> 8) as u8;
        tx[2] = addr as u8;
        tx[3] = 0x00; // dummy

        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi
            .transfer(&mut rx[..frame_len], &tx[..frame_len])
            .await
            .map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;

        let mut out = [0u8; N];
        out.copy_from_slice(&rx[4..4 + N]);
        Ok(out)
    }

    async fn get_status<D: DelayMs>(&mut self, delay: &mut D) -> Result<u8> {
        self.wait_not_busy(delay, 20).await?;
        let tx = [0xC0u8, 0x00];
        let mut rx = [0u8; 2];

        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi
            .transfer(&mut rx, &tx)
            .await
            .map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;

        let st = rx[1];
        let mode_bits = (st >> 4) & 0x07;
        self.mode = match mode_bits {
            0x2 => Sx1262Mode::StandbyRc,
            0x3 => Sx1262Mode::StandbyXosc,
            0x4 => Sx1262Mode::Fs,
            0x5 => Sx1262Mode::Rx,
            0x6 => Sx1262Mode::Tx,
            _ => Sx1262Mode::Unknown,
        };
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

    // SetRegulatorMode: 0 = LDO, 1 = DC-DC+LDO (datasheet)
    let reg_mode = if self.cfg.use_dcdc { 0x01 } else { 0x00 };
    self.write_cmd(delay, 0x96, &[reg_mode]).await?; // SetRegulatorMode

        // Clear device errors
        self.write_cmd(delay, 0x07, &[0x00, 0x00]).await?; // ClearDeviceErrors

        // TCXO power control using DIO3
        if self.cfg.tcxo_enable {
            info!(
                "SX1262: enabling TCXO via DIO3 ({} ms)",
                self.cfg.tcxo_delay_ms
            );
            self.enable_tcxo(delay).await?;
            // Extra warmup to ensure TCXO is fully stable before calibration
            delay.delay_ms(20).await;
        }

        // Ensure StandbyXosc before calibration when TCXO/XOSC is used
        self.write_cmd(delay, 0x80, &[0x01]).await?; // SetStandby( XOSC )

        // Calibrate (0x89) all blocks mask 0x7F
        self.write_cmd(delay, 0x89, &[0x7F]).await?;
        delay.delay_ms(10).await;

        // CalibrateImage for frequency band
        let (cal1, cal2) = if freq_hz > 900_000_000 && freq_hz < 930_000_000 {
            (0xE1u8, 0xE9u8) // 902-928 MHz
        } else if freq_hz > 850_000_000 && freq_hz < 880_000_000 {
            (0xD7u8, 0xDBu8) // 863-870 MHz
        } else {
            warn!("SX1262: unknown band {}Hz, using 915MHz cal", freq_hz);
            (0xE1u8, 0xE9u8)
        };
        self.write_cmd(delay, 0x98, &[cal1, cal2]).await?; // CalibrateImage

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
                // external handled via trait
            }
        }

        // Apply LoRa params
        self.set_lora_modulation_params(delay).await?;
        self.set_lora_packet_params(delay, 255).await?; // allow up to 255 for RX
        self.set_pa_config(delay).await?;
        self.set_tx_params(delay).await?;

        // MUST be done AFTER modulation/packet params as some commands reset SyncWord
        self.set_lora_sync_word(delay, self.cfg.lora_sync_word).await?;
        if let Ok(sw) = self.read_registers::<2, _>(delay, 0x0740).await {
            info!("SX1262: SyncWord final=0x{:02X}{:02X}", sw[0], sw[1]);
        }

        // Buffer bases: TX=0x00 RX=0x80
        self.write_cmd(delay, 0x8F, &[0x00, 0x80]).await?;

        // Default RF path to RX when external switch is used
        if matches!(self.rf_mode, RfSwitchMode::External) {
            self.rf_switch.set(RfState::Rx);
            info!("SX1262: RF switch -> RX (post-init)");
        }

        Ok(())
    }

    async fn enable_tcxo<D: DelayMs>(&mut self, delay: &mut D) -> Result<()> {
        // delay units: 15.625 us, datasheet: delay * 15.625us
        let d = (self.cfg.tcxo_delay_ms as u32 * 64).min(0x00FF_FFFF);
        let frame = [
            self.cfg.tcxo_voltage_code,
            (d >> 16) as u8,
            (d >> 8) as u8,
            d as u8,
        ];

        // SetDio3AsTcxoCtrl
        self.write_cmd(delay, 0x97, &frame).await?;
    // Give TCXO explicit warm-up time before forcing XOSC
    delay.delay_ms(self.cfg.tcxo_delay_ms as u32).await;

        // Move to StandbyXosc so TCXO drives the 32 MHz source
        self.write_cmd(delay, 0x80, &[0x01]).await?; // StandbyXosc
        // Allow a generous busy timeout here; some boards take longer than the nominal delay
        let tmo = (self.cfg.tcxo_delay_ms as u32).saturating_add(50);
        self.wait_not_busy(delay, tmo).await?;
        Ok(())
    }

    async fn set_rf_frequency<D: DelayMs>(
        &mut self,
        delay: &mut D,
        freq_hz: u32,
    ) -> Result<()> {
        // Freq = freq_hz * 2^25 / 32e6
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

    async fn set_lora_packet_params<D: DelayMs>(
        &mut self,
        delay: &mut D,
        payload_len: u8,
    ) -> Result<()> {
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
        let pbyte = if p < 0 {
            (256 + p as i16) as u8
        } else {
            p as u8
        };
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

    async fn set_dio_irq_params<D: DelayMs>(
        &mut self,
        delay: &mut D,
        irq_mask: u16,
        dio1_mask: u16,
    ) -> Result<()> {
        let frame = [
            (irq_mask >> 8) as u8,
            irq_mask as u8,
            (dio1_mask >> 8) as u8,
            dio1_mask as u8,
            0,
            0, // DIO2
            0,
            0, // DIO3
        ];
        self.write_cmd(delay, 0x08, &frame).await
    }

    async fn write_buffer<D: DelayMs>(
        &mut self,
        delay: &mut D,
        offset: u8,
        data: &[u8],
    ) -> Result<()> {
        if data.len() > 255 {
            return Err(Sx1262Error::InvalidParam);
        }
        self.wait_not_busy(delay, 20).await?;
        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi
            .write(&[0x0E, offset])
            .await
            .map_err(|_| Sx1262Error::Spi)?;
        if !data.is_empty() {
            self.spi.write(data).await.map_err(|_| Sx1262Error::Spi)?;
        }
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
        Ok(())
    }

    async fn get_rx_buffer_status<D: DelayMs>(
        &mut self,
        delay: &mut D,
    ) -> Result<(u8, u8)> {
        let b = self.read_cmd::<2, _>(delay, 0x13).await?;
        Ok((b[0], b[1]))
    }

    async fn read_buffer<D: DelayMs>(
        &mut self,
        delay: &mut D,
        offset: u8,
        out: &mut [u8],
    ) -> Result<usize> {
        if out.is_empty() {
            return Ok(0);
        }
        let len = out.len().min(255);
        self.wait_not_busy(delay, 20).await?;
        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi
            .write(&[0x1E, offset, 0x00])
            .await
            .map_err(|_| Sx1262Error::Spi)?;
        let zeros = [0u8; 255];
        self.spi
            .transfer(&mut out[..len], &zeros[..len])
            .await
            .map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
        Ok(len)
    }

    async fn get_packet_status<D: DelayMs>(&mut self, delay: &mut D) -> Result<[u8; 3]> {
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

    /// Set LoRa sync word (typical values: 0x1424 private, 0x3444 public).
    pub async fn set_lora_sync_word<D: DelayMs>(
        &mut self,
        delay: &mut D,
        sw: u16,
    ) -> Result<()> {
        // SX126x LoRa SyncWord registers: 0x0740 (MSB), 0x0741 (LSB)
        let bytes = [(sw >> 8) as u8, sw as u8];
        self.write_registers(delay, 0x0740, &bytes).await
    }

    // ------------------- TX/RX public API -------------------

    pub async fn start_rx_continuous<D: DelayMs>(&mut self, delay: &mut D) -> Result<()> {
        info!("SX1262: start_rx_continuous begin");

        // Force StandbyXosc before entering RX for robustness
        self.write_cmd(delay, 0x80, &[0x01]).await?; // SetStandby(XOSC)

        // Re-apply packet params for RX with max payload
        self.set_lora_packet_params(delay, 255).await?;

        // Configure IRQ: RxDone | Timeout | HeaderValid | CrcErr | PreambleDetected | SyncWordValid
    const IRQ_RX_DONE: u16 = 0x0002;
    const IRQ_TIMEOUT: u16 = 0x0200; // correct Rx/Tx timeout bit
        const IRQ_HEADER_VALID: u16 = 0x0010;
        const IRQ_CRC_ERR: u16 = 0x0040;
        const IRQ_PREAMBLE_DET: u16 = 0x0004;
        const IRQ_SYNCWORD_VAL: u16 = 0x0008;

        let mask = IRQ_RX_DONE
            | IRQ_TIMEOUT
            | IRQ_HEADER_VALID
            | IRQ_CRC_ERR
            | IRQ_PREAMBLE_DET
            | IRQ_SYNCWORD_VAL;

        self.clear_irq_status(delay, 0xFFFF).await?;
        self.set_dio_irq_params(delay, mask, mask).await?;
        info!("SX1262: IRQ configured mask=0x{:04X}", mask);

        if matches!(self.rf_mode, RfSwitchMode::External) {
            self.rf_switch.set(RfState::Rx);
            info!("SX1262: RF switch -> RX");
        }

        self.set_rx(delay, 0x00FF_FFFF).await?; // continuous

        // Verify mode transition
        let st = self.get_status(delay).await.unwrap_or(0);
        let dio1_high = self.dio1.is_high().unwrap_or(false);
        info!(
            "SX1262: post-SetRx status=0x{:02X} mode={:?} DIO1_high={}",
            st, self.mode, dio1_high
        );
        if !matches!(self.mode, Sx1262Mode::Rx) {
            warn!(
                "SX1262: NOT IN RX MODE! status=0x{:02X} mode={:?}",
                st, self.mode
            );
        }
        delay.delay_ms(2).await; // Small delay for RX to stabilize
        Ok(())
    }

    pub async fn poll_rx<D: DelayMs>(
        &mut self,
        delay: &mut D,
        buf: &mut [u8],
    ) -> Result<Option<RxReport>> {
        let irq = self.get_irq_status(delay).await?;
        if irq == 0 {
            return Ok(None);
        }

        info!(
            "SX1262: RX IRQ=0x{:04X} (RxDone={} Timeout={} PreambleDet={} SyncValid={} HdrValid={} CrcErr={})",
            irq,
            (irq & 0x0002) != 0,
            (irq & 0x0200) != 0,
            (irq & 0x0004) != 0,
            (irq & 0x0008) != 0,
            (irq & 0x0010) != 0,
            (irq & 0x0040) != 0
        );

        let done = (irq & 0x0002) != 0; // RxDone
        let timeout = (irq & 0x0200) != 0; // Timeout
        let mut len = 0u8;

        if done {
            let (l, start) = self.get_rx_buffer_status(delay).await?;
            len = l;
            let copy_len = core::cmp::min(len as usize, buf.len());
            let _ = self
                .read_buffer(delay, start, &mut buf[..copy_len])
                .await?;
        }

        let pkt = self.get_packet_status(delay).await.unwrap_or([0; 3]);
        self.clear_irq_status(delay, irq).await?;

        let rssi = -(pkt[0] as i16) / 2; // approx dBm
        let snr_x4 = pkt[1] as i8 as i16; // 1/4 dB units

        Ok(Some(RxReport {
            irq,
            done,
            timeout,
            len,
            rssi,
            snr_x4,
        }))
    }

    pub async fn tx_send_blocking<D: DelayMs>(
        &mut self,
        delay: &mut D,
        payload: &[u8],
        timeout_syms: u32,
    ) -> Result<TxReport> {
        if payload.len() > 255 {
            return Err(Sx1262Error::InvalidParam);
        }
        info!(
            "SX1262: tx_send_blocking len={} timeout_syms={}",
            payload.len(),
            timeout_syms
        );

        // Enter Standby mode before TX
        self.write_cmd(delay, 0x80, &[0x00]).await?; // SetStandby(RC)
        let st_pre = self.get_status(delay).await.unwrap_or(0);
        info!(
            "SX1262: post-SetStandby status=0x{:02X} mode={:?}",
            st_pre, self.mode
        );

        // Prepare buffer and IRQs
        self.clear_irq_status(delay, 0xFFFF).await?;
    const IRQ_TX_DONE: u16 = 0x0001;
    const IRQ_TIMEOUT: u16 = 0x0200; // correct Rx/Tx timeout bit
        let mask = IRQ_TX_DONE | IRQ_TIMEOUT;
        self.set_dio_irq_params(delay, mask, mask).await?;
        info!("SX1262: TX IRQ configured mask=0x{:04X}", mask);

        self.set_lora_packet_params(delay, payload.len() as u8)
            .await?;
        self.write_buffer(delay, 0x00, payload).await?;

        if matches!(self.rf_mode, RfSwitchMode::External) {
            self.rf_switch.set(RfState::Tx);
            info!("SX1262: RF switch -> TX");
        }

        self.set_tx(delay, timeout_syms).await?;

        let st = self.get_status(delay).await.unwrap_or(0);
        info!(
            "SX1262: post-SetTx status=0x{:02X} mode={:?}",
            st, self.mode
        );
        if !matches!(self.mode, Sx1262Mode::Tx | Sx1262Mode::Fs) {
            warn!(
                "SX1262: NOT IN TX MODE! status=0x{:02X} mode={:?}",
                st, self.mode
            );
        }

        // Poll for completion
        let mut waited = 0u32;
        loop {
            let irq = self.get_irq_status(delay).await?;
            if irq != 0 {
                info!(
                    "SX1262: TX IRQ=0x{:04X} (TxDone={} Timeout={})",
                    irq,
                    (irq & IRQ_TX_DONE) != 0,
                    (irq & IRQ_TIMEOUT) != 0
                );
                let done = (irq & IRQ_TX_DONE) != 0;
                let timeout = (irq & IRQ_TIMEOUT) != 0;
                self.clear_irq_status(delay, irq).await?;

                if matches!(self.rf_mode, RfSwitchMode::External) {
                    self.rf_switch.set(RfState::Rx);
                }

                return Ok(TxReport { irq, done, timeout });
            }

            if waited > 2000 {
                warn!("SX1262: TX wait exceeded 2s; forcing timeout report");
                if matches!(self.rf_mode, RfSwitchMode::External) {
                    self.rf_switch.set(RfState::Rx);
                }
                return Ok(TxReport {
                    irq: 0,
                    done: false,
                    timeout: true,
                });
            }

            delay.delay_ms(1).await;
            waited += 1;
        }
    }

    pub async fn tx_async<D: DelayMs>(
        &mut self,
        delay: &mut D,
        payload: &[u8],
    ) -> Result<()> {
        let _ = self
            .tx_send_blocking(delay, payload, 0x00FF_FFFF)
            .await?;
        Ok(())
    }

    pub async fn rx_async_continuous<D: DelayMs>(&mut self, delay: &mut D) -> Result<()> {
        self.start_rx_continuous(delay).await
    }
}
