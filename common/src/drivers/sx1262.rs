// common/src/drivers/sx1262.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::{info, warn, debug};
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

#[derive(Debug, defmt::Format)]
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
//  SX1262 Driver (Layer 0) – LoRa RAW PHY
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
    cfg: LoRaConfig,
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
    // IRQ bit definitions (SX1262 datasheet)
    const IRQ_TX_DONE: u16 = 0x0001;
    const IRQ_RX_DONE: u16 = 0x0002;
    const IRQ_PREAMBLE_DETECTED: u16 = 0x0004;
    const IRQ_SYNCWORD_VALID: u16 = 0x0008;
    const IRQ_HEADER_VALID: u16 = 0x0010;
    const IRQ_HEADER_ERR: u16 = 0x0020;
    const IRQ_CRC_ERR: u16 = 0x0040;
    const IRQ_CAD_DONE: u16 = 0x0080;
    const IRQ_CAD_DETECTED: u16 = 0x0100;
    const IRQ_TIMEOUT: u16 = 0x0200;

    #[inline]
    fn log_irq(label: &str, irq: u16) {
        if irq == 0 {
            return;
        }
        info!(
            "{} IRQ={:#05x} tx_done={} rx_done={} hdr_valid={} hdr_err={} crc_err={} timeout={} preamble={} sync={} cad={} cad_det={}",
            label,
            irq,
            (irq & Self::IRQ_TX_DONE) != 0,
            (irq & Self::IRQ_RX_DONE) != 0,
            (irq & Self::IRQ_HEADER_VALID) != 0,
            (irq & Self::IRQ_HEADER_ERR) != 0,
            (irq & Self::IRQ_CRC_ERR) != 0,
            (irq & Self::IRQ_TIMEOUT) != 0,
            (irq & Self::IRQ_PREAMBLE_DETECTED) != 0,
            (irq & Self::IRQ_SYNCWORD_VALID) != 0,
            (irq & Self::IRQ_CAD_DONE) != 0,
            (irq & Self::IRQ_CAD_DETECTED) != 0,
        );
    }

    // --------------------------------------------------------------------
    // Constructor / config access
    // --------------------------------------------------------------------

    pub fn new(
        spi: SPI,
        nss: NSS,
        reset: RESET,
        busy: BUSY,
        dio1: DIO1,
        rf_sw: SW,
        cfg: LoRaConfig,
    ) -> Self {
        Self {
            spi,
            nss,
            reset,
            busy,
            dio1,
            rf_sw,
            cfg,
        }
    }

    pub fn cfg(&self) -> &LoRaConfig {
        &self.cfg
    }

    pub fn cfg_mut(&mut self) -> &mut LoRaConfig {
        &mut self.cfg
    }

    // --------------------------------------------------------------------
    // Low-level helpers
    // --------------------------------------------------------------------

    async fn wait_not_busy(
        &mut self,
        delay: &mut impl DelayMs,
        timeout_ms: u32,
    ) -> Result<()> {
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
        self.spi
            .transfer(&mut rx[..N + 2], &tx[..N + 2])
            .await
            .map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;

        let mut out = [0u8; N];
        out.copy_from_slice(&rx[2..N + 2]);
        Ok(out)
    }

    async fn write_registers(
        &mut self,
        delay: &mut impl DelayMs,
        addr: u16,
        data: &[u8],
    ) -> Result<()> {
        self.wait_not_busy(delay, 50).await?;
        if data.len() > 16 {
            return Err(Sx1262Error::InvalidParam);
        }

        let mut frame = [0u8; 3 + 16];
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

    async fn set_rf_freq(
        &mut self,
        delay: &mut impl DelayMs,
        freq: u32,
    ) -> Result<()> {
        // Freq = freq_hz * 2^25 / 32e6
        let raw = (((freq as u64) << 25) / 32_000_000) as u32;
        self.write_cmd(delay, 0x86, &raw.to_be_bytes()).await
    }

    async fn write_buffer(
        &mut self,
        delay: &mut impl DelayMs,
        offset: u8,
        data: &[u8],
    ) -> Result<()> {
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
        self.spi
            .write(&[0x1E, offset, 0])
            .await
            .map_err(|_| Sx1262Error::Spi)?;

        let zeros = [0u8; 255];
        let max = out.len().min(255);

        self.spi
            .transfer(&mut out[..max], &zeros[..max])
            .await
            .map_err(|_| Sx1262Error::Spi)?;
        self.nss.set_high().map_err(|_| Sx1262Error::Cs)?;
        Ok(())
    }

    // ====================================================================
    //  Init
    // ====================================================================

    /// FULL known-good init with TCXO, calibration, RF switch, LoRa params.
    pub async fn init(
        &mut self,
        delay: &mut impl DelayMs,
    ) -> Result<()> {
        let cfg = self.cfg;

        info!("SX1262 Init…");

        // Reset pulse
        self.reset.set_low().map_err(|_| Sx1262Error::Cs)?;
        delay.delay_ms(1).await;
        self.reset.set_high().map_err(|_| Sx1262Error::Cs)?;
        self.wait_not_busy(delay, 20).await?;

        // Regulator mode (DC/DC recommended)
        self.write_cmd(
            delay,
            0x96,
            &[if cfg.use_dcdc { 0x01 } else { 0x00 }],
        )
        .await?;

        // Clear device errors
        self.write_cmd(delay, 0x07, &[0, 0]).await?;

        // TCXO power on DIO3 if needed
        if cfg.tcxo_enable {
            let delay_units = cfg.tcxo_delay_ms.saturating_mul(64);
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
            self.wait_not_busy(delay, cfg.tcxo_delay_ms + 50)
                .await?;
        }

        // Calibration
        self.write_cmd(delay, 0x80, &[0x01]).await?; // Standby XOSC
        self.write_cmd(delay, 0x89, &[0x7F]).await?;
        delay.delay_ms(10).await;

        // CalibrateImage for 902–928 MHz
        self.write_cmd(delay, 0x98, &[0xE1, 0xE9]).await?;

        // LoRa mode
        self.write_cmd(delay, 0x8A, &[0x01]).await?;

        // RF Frequency
        self.set_rf_freq(delay, cfg.freq_hz).await?;

        // Buffer bases: TX=0x00, RX=0x80
        self.write_cmd(delay, 0x8F, &[0x00, 0x80]).await?;

        // LoRa modulation params
        self.write_cmd(delay, 0x8B, &cfg.mod_params())
            .await?;

        // Packet params for RX (max payload)
        self.write_cmd(delay, 0x8C, &cfg.pkt_params_rx())
            .await?;

        // PA
        self.write_cmd(delay, 0x95, &cfg.pa_config())
            .await?;
        self.write_cmd(delay, 0x8E, &cfg.tx_params()).await?;

        // Sync word (AFTER modulation + packet params)
        self.write_registers(delay, 0x0740, &cfg.sync_word.to_be_bytes())
            .await?;

        // Default RF path to RX
        self.rf_sw.set(RfState::Rx);

        info!("SX1262 Init OK");
        Ok(())
    }

    // ====================================================================
    //  RAW TX
    // ====================================================================

    pub async fn tx_raw(
        &mut self,
        delay: &mut impl DelayMs,
        payload: &[u8],
    ) -> Result<()> {
        if payload.len() > 255 {
            return Err(Sx1262Error::InvalidParam);
        }

        let len = payload.len() as u8;
        info!("TX: len={} (LoRa)", len);

        // Standby RC before TX
        self.write_cmd(delay, 0x80, &[0x00]).await?;

        // Clear all IRQs
        self.write_cmd(delay, 0x02, &[0xFF, 0xFF]).await?;

        // Set packet params for TX with REAL payload length
        let pkt_tx = self.cfg.pkt_params_tx(len);
        self.write_cmd(delay, 0x8C, &pkt_tx).await?;

        // IRQ mask: TxDone routed to DIO1
        self.write_cmd(
            delay,
            0x08,
            &[
                0x00, 0x01, // irq mask
                0x00, 0x01, // DIO1 mask
                0, 0, 0, 0, // DIO2/DIO3 (unused)
            ],
        )
        .await?;

        // Buffer: write payload at offset 0x00
        self.write_buffer(delay, 0x00, payload).await?;

        // RF switch to TX
        self.rf_sw.set(RfState::Tx);

        // SetTx with max timeout (0xFFFFFF)
        self.write_cmd(delay, 0x83, &[0xFF, 0xFF, 0xFF])
            .await?;

        // Poll IRQ for TxDone
        let mut waited = 0u32;
        loop {
            let irq = self.get_irq(delay).await?;
            if irq != 0 {
                Self::log_irq("TX poll", irq);
            }

            if irq & Self::IRQ_TX_DONE != 0 {
                // Clear all bits we saw (including TxDone)
                self.write_cmd(delay, 0x02, &irq.to_be_bytes())
                    .await?;

                // Restore RF switch + RX packet params
                self.rf_sw.set(RfState::Rx);

                // Back to RX-style packet params (0xFF len)
                let pkt_rx = self.cfg.pkt_params_rx();
                self.write_cmd(delay, 0x8C, &pkt_rx).await?;

                // Re-write sync word just to be safe (cheap op)
                self.write_registers(
                    delay,
                    0x0740,
                    &self.cfg.sync_word.to_be_bytes(),
                )
                .await?;

                info!("TX done -> entering RX");
                return Ok(());
            }

            // Spurious IRQs during TX → clear them and continue
            let spurious = irq & !Self::IRQ_TX_DONE;
            if spurious != 0 {
                warn!("Clearing spurious IRQs during TX: {:#05x}", spurious);
                self.write_cmd(delay, 0x02, &spurious.to_be_bytes())
                    .await?;
            }

            if waited > 3000 {
                warn!("TX: timeout waiting for TxDone");
                // Attempt to clear everything and go back to RX
                self.write_cmd(delay, 0x02, &[0xFF, 0xFF])
                    .await?;
                self.rf_sw.set(RfState::Rx);
                let pkt_rx = self.cfg.pkt_params_rx();
                let _ = self.write_cmd(delay, 0x8C, &pkt_rx).await;
                return Err(Sx1262Error::BusyTimeout);
            }

            delay.delay_ms(1).await;
            waited += 1;
        }
    }

    // ====================================================================
    //  RAW RX
    // ====================================================================

    pub async fn start_rx_continuous(
        &mut self,
        delay: &mut impl DelayMs,
    ) -> Result<()> {
        // Standby XOSC
        self.write_cmd(delay, 0x80, &[0x01]).await?;

        // Ensure RX-style packet params (max payload)
        self.write_cmd(delay, 0x8C, &self.cfg.pkt_params_rx())
            .await?;

        // IRQ mask: RxDone | Timeout | HeaderValid | CrcErr
        // 0x0002 | 0x0010 | 0x0040 | 0x0200 = 0x0252
        self.write_cmd(
            delay,
            0x08,
            &[
                0x02, 0x52, // irq mask
                0x02, 0x52, // DIO1 routing
                0, 0, 0, 0,
            ],
        )
        .await?;

        // Clear all IRQs
        self.write_cmd(delay, 0x02, &[0xFF, 0xFF]).await?;

        // RF switch to RX
        self.rf_sw.set(RfState::Rx);

        // Continuous RX
        self.write_cmd(delay, 0x82, &[0xFF, 0xFF, 0xFF])
            .await?;
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
        Self::log_irq("RX poll", irq);

        let done         = irq & Self::IRQ_RX_DONE        != 0;
        let header_valid = irq & Self::IRQ_HEADER_VALID   != 0;
        let header_err   = irq & Self::IRQ_HEADER_ERR     != 0;
        let crc_err      = irq & Self::IRQ_CRC_ERR        != 0;
        let timeout      = irq & Self::IRQ_TIMEOUT        != 0;

        if timeout {
            debug!("RX timeout IRQ observed");
        }
        if header_err {
            warn!("Header error IRQ observed");
        }
        if crc_err {
            warn!("CRC error IRQ observed");
        }
        if header_valid && !done {
            debug!("Header valid before RxDone (partial reception)");
        }

        // Any of these means the frame is unusable at the PHY level.
        // Do NOT forward it upward – just clear IRQs and report "no frame".
        if header_err || crc_err || timeout {
            self.write_cmd(delay, 0x02, &irq.to_be_bytes()).await?;
            return Ok(None);
        }

        let mut len = 0usize;
        if done {
            let (l, start) = self.get_rx_buffer_status(delay).await?;
            len = l as usize;
            let max = buf.len().min(len);
            self.read_buffer(delay, start, &mut buf[..max]).await?;
        }

        // Packet status (RSSI / SNR)
        let pkt = self.read_cmd::<3>(delay, 0x14).await.unwrap_or([0; 3]);
        let rssi = -(pkt[0] as i16) / 2;
        let snr_x4 = pkt[1] as i8 as i16;

        // Clear consumed IRQ bits
        self.write_cmd(delay, 0x02, &irq.to_be_bytes()).await?;

        Ok(Some(RawRx {
            len: len as u8,
            rssi,
            snr_x4,
        }))
    }

}
