// src/drivers/sx1262.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::info;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::spi::SpiBus;

use crate::lora::lora_config::LoRaConfig;
use crate::utils::delay::DelayMs;

// ========================================================================
//  Public Types
// ========================================================================

#[derive(Debug, Clone, Copy)]
pub struct RawRx {
    pub len: u8,
    pub rssi: i16,
    pub snr_x4: i16,
}

#[derive(Debug)]
pub enum Sx1262Error {
    BusyTimeout,
    Cs,
    Spi,
    InvalidParam,
}

pub type Result<T> = core::result::Result<T, Sx1262Error>;

#[derive(Clone, Copy, Debug)]
pub enum RfState {
    Rx,
    Tx,
    Off,
}

pub trait RfSwitch {
    fn set(&mut self, _state: RfState) {}
}

// ========================================================================
//  SX1262 Driver (Layer 0)
// ========================================================================

pub struct Sx1262<SPI, NSS, RESET, BUSY, DIO1, SW>
where
    SPI: SpiBus<u8>,
    NSS: OutputPin,
    RESET: OutputPin,
    BUSY: InputPin,
    DIO1: InputPin,
    SW: RfSwitch,
{
    spi: SPI,
    nss: NSS,
    reset: RESET,
    busy: BUSY,
    dio1: DIO1,
    rf_sw: SW,
}

impl<SPI, NSS, RESET, BUSY, DIO1, SW> Sx1262<SPI, NSS, RESET, BUSY, DIO1, SW>
where
    SPI: SpiBus<u8>,
    NSS: OutputPin,
    RESET: OutputPin,
    BUSY: InputPin,
    DIO1: InputPin,
    SW: RfSwitch,
{
    pub fn new(spi: SPI, nss: NSS, reset: RESET, busy: BUSY, dio1: DIO1, rf_sw: SW) -> Self {
        Self { spi, nss, reset, busy, dio1, rf_sw }
    }

    // -----------------------------------------------------------
    // Low-level helpers
    // -----------------------------------------------------------

    async fn wait_not_busy(&mut self, delay: &mut impl DelayMs, timeout_ms: u32) -> Result<()> {
        let mut waited = 0;
        while self.busy.is_high().unwrap_or(true) {
            if waited >= timeout_ms {
                return Err(Sx1262Error::BusyTimeout);
            }
            delay.delay_ms(1).await;
            waited += 1;
        }
        Ok(())
    }

    async fn write_cmd(
        &mut self,
        delay: &mut impl DelayMs,
        opcode: u8,
        params: &[u8],
    ) -> Result<()> {
        self.wait_not_busy(delay, 50).await?;

        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi.write(&[opcode]).await.map_err(|_| Sx1262Error::Spi)?;
        if !params.is_empty() {
            self.spi.write(params).await.map_err(|_| Sx1262Error::Spi)?;
        }
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
        Ok(())
    }

    async fn read_cmd<const N: usize>(
        &mut self,
        delay: &mut impl DelayMs,
        opcode: u8,
    ) -> Result<[u8; N]> {
        assert!(N <= 16);

        self.wait_not_busy(delay, 50).await?;
        let mut tx = [0u8; 18];
        let mut rx = [0u8; 18];
        tx[0] = opcode;

        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi.transfer(&mut rx[..N + 2], &tx[..N + 2]).await.map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;

        let mut out = [0u8; N];
        out.copy_from_slice(&rx[2..N + 2]);
        Ok(out)
    }

    // ========================================================================
    //  Layer-0 Public API
    // ========================================================================

    /// FULL known-good init with TCXO, calibration, RF switch.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayMs,
        cfg: &LoRaConfig,
    ) -> Result<()> {
        info!("SX1262 Init…");

        // Reset pulse
        self.reset.set_low().map_err(|_| Sx1262Error::Cs)?;
        delay.delay_ms(1).await;
        self.reset.set_high().map_err(|_| Sx1262Error::Cs)?;
        self.wait_not_busy(delay, 20).await?;

        // Regulator mode (DC/DC recommended)
        self.write_cmd(delay, 0x96, &[if cfg.use_dcdc { 0x01 } else { 0x00 }]).await?;

        // Clear device errors
        self.write_cmd(delay, 0x07, &[0, 0]).await?;

        // TCXO power on DIO3
        if cfg.tcxo_enable {
            let delay_units = cfg.tcxo_delay_ms as u32 * 64;
            let frame = [
                cfg.tcxo_voltage,
                (delay_units >> 16) as u8,
                (delay_units >> 8) as u8,
                delay_units as u8,
            ];
            self.write_cmd(delay, 0x97, &frame).await?; // SetDio3AsTcxoCtrl
            delay.delay_ms(cfg.tcxo_delay_ms).await;

            // Force XOSC
            self.write_cmd(delay, 0x80, &[0x01]).await?;
            self.wait_not_busy(delay, cfg.tcxo_delay_ms + 50).await?;
        }

        // Calibration
        self.write_cmd(delay, 0x80, &[0x01]).await?; // Standby XOSC
        self.write_cmd(delay, 0x89, &[0x7F]).await?;
        delay.delay_ms(10).await;

        // CalibrateImage for 915MHz
        self.write_cmd(delay, 0x98, &[0xE1, 0xE9]).await?;

        // LoRa mode
        self.write_cmd(delay, 0x8A, &[0x01]).await?;

        // RF Frequency
        self.set_rf_freq(delay, cfg.freq_hz).await?;

        // No DIO2 RF switch (Waveshare)
        // Buffer bases
        self.write_cmd(delay, 0x8F, &[0x00, 0x80]).await?;

        // LoRa modulation params
        self.write_cmd(delay, 0x8B, &cfg.mod_params()).await?;
        self.write_cmd(delay, 0x8C, &cfg.pkt_params()).await?;

        // PA
        self.write_cmd(delay, 0x95, &cfg.pa_config()).await?;
        self.write_cmd(delay, 0x8E, &cfg.tx_params()).await?;

        // Sync word
        self.write_registers(delay, 0x0740, &cfg.sync_word.to_be_bytes()).await?;

        info!("SX1262 Init OK");
        Ok(())
    }

    async fn set_rf_freq(
        &mut self,
        delay: &mut impl DelayMs,
        freq: u32,
    ) -> Result<()> {
        let raw = (((freq as u64) << 25) / 32_000_000) as u32;
        self.write_cmd(delay, 0x86, &raw.to_be_bytes()).await
    }

    async fn write_registers(
        &mut self,
        delay: &mut impl DelayMs,
        addr: u16,
        data: &[u8],
    ) -> Result<()> {
        self.wait_not_busy(delay, 50).await?;
        let mut frame = [0u8; 3 + 16];
        frame[0] = 0x0D;
        frame[1] = (addr >> 8) as u8;
        frame[2] = addr as u8;
        frame[3..3 + data.len()].copy_from_slice(data);

        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi.write(&frame[..3 + data.len()]).await.map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
        Ok(())
    }

    // -----------------------------------------------------------
    // RAW TX
    // -----------------------------------------------------------
    pub async fn tx_raw(
        &mut self,
        delay: &mut impl DelayMs,
        payload: &[u8],
    ) -> Result<()> {
        if payload.len() > 255 {
            return Err(Sx1262Error::InvalidParam);
        }

        // Standby RC
        self.write_cmd(delay, 0x80, &[0x00]).await?;

        // Clear IRQs
        self.write_cmd(delay, 0x02, &[0xFF, 0xFF]).await?;

        // IRQ mask: TxDone
        self.write_cmd(delay, 0x08, &[0x00, 0x01, 0x00, 0x01, 0, 0, 0, 0]).await?;

        // Buffer
        self.write_buffer(delay, 0x00, payload).await?;

        self.rf_sw.set(RfState::Tx);
        self.write_cmd(delay, 0x83, &[0xFF, 0xFF, 0xFF]).await?;

        // Poll IRQ
        loop {
            let irq = self.get_irq(delay).await?;
            if irq & 0x0001 != 0 {
                self.write_cmd(delay, 0x02, &[0x00, 0x01]).await?;
                self.rf_sw.set(RfState::Rx);
                info!("TX done -> entering RX");
                return Ok(());
            }
            delay.delay_ms(1).await;
        }
    }

    async fn write_buffer(
        &mut self,
        delay: &mut impl DelayMs,
        offset: u8,
        data: &[u8],
    ) -> Result<()> {
        self.wait_not_busy(delay, 20).await?;
        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi.write(&[0x0E, offset]).await.map_err(|_| Sx1262Error::Spi)?;
        if !data.is_empty() {
            self.spi.write(data).await.map_err(|_| Sx1262Error::Spi)?;
        }
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
        Ok(())
    }

    // -----------------------------------------------------------
    // RAW RX
    // -----------------------------------------------------------

    pub async fn start_rx_continuous(
        &mut self,
        delay: &mut impl DelayMs,
    ) -> Result<()> {
        // Standby XOSC
        self.write_cmd(delay, 0x80, &[0x01]).await?;

        // Full IRQ mask: RxDone | Timeout | HeaderValid | CrcErr
        self.write_cmd(
            delay,
            0x08,
            &[
                0x02, 0x12, // mask
                0x02, 0x12, // DIO1
                0, 0, 0, 0,
            ],
        )
        .await?;

        self.write_cmd(delay, 0x02, &[0xFF, 0xFF]).await?;

        self.rf_sw.set(RfState::Rx);

        // Continuous RX
        self.write_cmd(delay, 0x82, &[0xFF, 0xFF, 0xFF]).await?;
        info!("Started RX (continuous)");
        Ok(())
    }

    pub async fn poll_raw(
        &mut self,
        delay: &mut impl DelayMs,
        buf: &mut [u8],
    ) -> Result<Option<RawRx>> {
        let irq = self.get_irq(delay).await?;
        if irq == 0 {
            return Ok(None);
        }

        // RxDone?
        let done = irq & 0x0002 != 0;

        let mut len = 0;
        if done {
            let (l, start) = self.get_rx_buffer_status(delay).await?;
            len = l as usize;
            let max = buf.len().min(len);
            self.read_buffer(delay, start, &mut buf[..max]).await?;
        }

        let pkt = self.read_cmd::<3>(delay, 0x14).await.unwrap_or([0; 3]);
        let rssi = -(pkt[0] as i16) / 2;
        let snr_x4 = pkt[1] as i8 as i16;

        self.write_cmd(delay, 0x02, &irq.to_be_bytes()).await?;

        Ok(Some(RawRx {
            len: len as u8,
            rssi,
            snr_x4,
        }))
    }

    // -----------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------

    async fn get_irq(&mut self, delay: &mut impl DelayMs) -> Result<u16> {
        let b = self.read_cmd::<2>(delay, 0x12).await?;
        Ok(u16::from_be_bytes(b))
    }

    async fn get_rx_buffer_status(
        &mut self,
        delay: &mut impl DelayMs,
    ) -> Result<(u8, u8)> {
        let b = self.read_cmd::<2>(delay, 0x13).await?;
        Ok((b[0], b[1]))
    }

    async fn read_buffer(
        &mut self,
        delay: &mut impl DelayMs,
        offset: u8,
        out: &mut [u8],
    ) -> Result<()> {
        self.wait_not_busy(delay, 20).await?;
        self.nss.set_low().map_err(|_| Sx1262Error::Cs)?;
        self.spi.write(&[0x1E, offset, 0]).await.map_err(|_| Sx1262Error::Spi)?;

        let zeros = [0u8; 255];
        let max = out.len().min(255);

        self.spi.transfer(&mut out[..max], &zeros[..max]).await.map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
        Ok(())
    }
}
