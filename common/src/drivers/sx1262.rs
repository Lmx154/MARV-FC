// common/src/drivers/sx1262.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::{
    Bandwidth, CodingRate, ModulationParams, PacketParams, RadioError, RxMode,
    SpreadingFactor,
};
use lora_phy::mod_traits::RadioKind;
use lora_phy::sx126x::{
    Config as Sx126xConfig, DeviceSel, Sx126x, Sx126xVariant, TcxoCtrlVoltage,
};
use lora_phy::LoRa;

use crate::coms::transport::lora::rf_config::LoRaConfig;

#[derive(Debug, Clone, Copy)]
pub struct RawRx {
    pub len: u8,
    pub rssi: i16,
    pub snr_x4: i16,
    pub irq_instant_us: u64,
}

#[derive(Debug)]
pub enum Sx1262Error {
    Radio(RadioError),
    InvalidParam,
}

pub type Result<T> = core::result::Result<T, Sx1262Error>;

impl From<RadioError> for Sx1262Error {
    fn from(err: RadioError) -> Self {
        Self::Radio(err)
    }
}

struct ExternalRfSwitch;

impl Sx126xVariant for ExternalRfSwitch {
    fn get_device_sel(&self) -> DeviceSel {
        DeviceSel::HighPowerPA
    }

    fn use_dio2_as_rfswitch(&self) -> bool {
        false
    }
}

type Sx1262Radio<SPI, CTRL, WAIT> =
    Sx126x<SPI, GenericSx126xInterfaceVariant<CTRL, WAIT>, ExternalRfSwitch>;

pub struct Sx1262<SPI, CTRL, WAIT, DLY>
where
    SPI: SpiDevice<u8>,
    CTRL: OutputPin,
    WAIT: Wait,
    DLY: DelayNs + Clone,
{
    lora: LoRa<Sx1262Radio<SPI, CTRL, WAIT>, DLY>,
    cfg: LoRaConfig,
    mod_params: ModulationParams,
    rx_pkt_params: PacketParams,
    tx_pkt_params: PacketParams,
}

pub fn set_irq_timestamp_fn(f: fn() -> u64) {
    lora_phy::set_irq_timestamp_fn(f);
}

impl<SPI, CTRL, WAIT, DLY> Sx1262<SPI, CTRL, WAIT, DLY>
where
    SPI: SpiDevice<u8>,
    CTRL: OutputPin,
    WAIT: Wait,
    DLY: DelayNs + Clone,
{
    pub async fn new(
        spi: SPI,
        reset: CTRL,
        busy: WAIT,
        dio1: WAIT,
        rf_switch_tx: CTRL,
        rf_switch_rx: CTRL,
        cfg: LoRaConfig,
        delay: DLY,
    ) -> Result<Self> {
        let tcxo = if cfg.tcxo_enable {
            Some(map_tcxo_voltage(cfg.tcxo_voltage)?)
        } else {
            None
        };

        let sx_cfg = Sx126xConfig {
            chip: ExternalRfSwitch,
            tcxo_ctrl: tcxo,
            use_dcdc: cfg.use_dcdc,
            rx_boost: false,
        };

        // Some modules wire TX/RX enable lines swapped vs the driver defaults.
        let (rf_rx, rf_tx) = if cfg.rf_switch_swap {
            (rf_switch_tx, rf_switch_rx)
        } else {
            (rf_switch_rx, rf_switch_tx)
        };

        let iv = GenericSx126xInterfaceVariant::new(
            reset,
            dio1,
            busy,
            Some(rf_rx),
            Some(rf_tx),
        )
        .map_err(Sx1262Error::Radio)?;

        let radio = Sx126x::new(spi, iv, sx_cfg);

        let enable_public = match cfg.sync_word {
            0x3444 => true,
            0x1424 => false,
            _ => return Err(Sx1262Error::InvalidParam),
        };

        let mut delay_extra = delay.clone();
        let mut lora = LoRa::new(radio, enable_public, delay).await?;
        if cfg.tcxo_enable {
            // lora-phy already waits 10ms for TCXO; top up to the configured delay.
            let extra_ms = cfg.tcxo_delay_ms.saturating_sub(10);
            if extra_ms > 0 {
                delay_extra.delay_ms(extra_ms).await;
            }
        }

        let (mod_params, rx_pkt_params, tx_pkt_params) =
            build_params(&mut lora, &cfg)?;

        Ok(Self {
            lora,
            cfg,
            mod_params,
            rx_pkt_params,
            tx_pkt_params,
        })
    }

    pub fn cfg(&self) -> &LoRaConfig {
        &self.cfg
    }

    pub fn cfg_mut(&mut self) -> &mut LoRaConfig {
        &mut self.cfg
    }

    pub async fn apply_lora_config(&mut self, cfg: LoRaConfig) -> Result<()> {
        if cfg.sync_word != self.cfg.sync_word {
            return Err(Sx1262Error::InvalidParam);
        }

        self.cfg = cfg;
        let (mod_params, rx_pkt_params, tx_pkt_params) =
            build_params(&mut self.lora, &self.cfg)?;
        self.mod_params = mod_params;
        self.rx_pkt_params = rx_pkt_params;
        self.tx_pkt_params = tx_pkt_params;

        self.start_rx_continuous().await?;
        Ok(())
    }

    pub async fn tx_raw(&mut self, payload: &[u8]) -> Result<()> {
        if payload.len() > 255 {
            return Err(Sx1262Error::InvalidParam);
        }

        let power = self.cfg.tx_power as i32;
        self.lora
            .prepare_for_tx(
                &self.mod_params,
                &mut self.tx_pkt_params,
                power,
                payload,
            )
            .await?;
        self.lora.tx().await?;
        self.start_rx_continuous().await?;
        Ok(())
    }

    pub async fn start_rx_continuous(&mut self) -> Result<()> {
        self.lora
            .prepare_for_rx(RxMode::Continuous, &self.mod_params, &self.rx_pkt_params)
            .await?;
        Ok(())
    }

    pub async fn start_rx_single(&mut self, symbol_timeout: u16) -> Result<()> {
        self.lora
            .prepare_for_rx(
                RxMode::Single(symbol_timeout),
                &self.mod_params,
                &self.rx_pkt_params,
            )
            .await?;
        Ok(())
    }

    pub async fn wait_raw(&mut self, buf: &mut [u8]) -> Result<Option<RawRx>> {
        let (len, status) = self.lora.rx(&self.rx_pkt_params, buf).await?;
        if len == 0 {
            return Ok(None);
        }
        let irq_instant_us = lora_phy::last_irq_timestamp_us();
        Ok(Some(RawRx {
            len,
            rssi: status.rssi,
            snr_x4: status.snr.saturating_mul(4),
            irq_instant_us,
        }))
    }
}

fn build_params<RK, DLY>(
    lora: &mut LoRa<RK, DLY>,
    cfg: &LoRaConfig,
) -> Result<(ModulationParams, PacketParams, PacketParams)>
where
    RK: RadioKind,
    DLY: DelayNs,
{
    let sf = map_spreading_factor(cfg.sf)?;
    let bw = map_bandwidth(cfg.bw)?;
    let cr = map_coding_rate(cfg.cr)?;
    let mod_params = lora.create_modulation_params(sf, bw, cr, cfg.freq_hz)?;

    let implicit_header = !cfg.explicit_header;
    let rx_params = lora.create_rx_packet_params(
        cfg.preamble_len,
        implicit_header,
        0xFF,
        cfg.crc_on,
        cfg.invert_iq,
        &mod_params,
    )?;
    let tx_params = lora.create_tx_packet_params(
        cfg.preamble_len,
        implicit_header,
        cfg.crc_on,
        cfg.invert_iq,
        &mod_params,
    )?;

    Ok((mod_params, rx_params, tx_params))
}

fn map_spreading_factor(sf: u8) -> Result<SpreadingFactor> {
    match sf {
        5 => Ok(SpreadingFactor::_5),
        6 => Ok(SpreadingFactor::_6),
        7 => Ok(SpreadingFactor::_7),
        8 => Ok(SpreadingFactor::_8),
        9 => Ok(SpreadingFactor::_9),
        10 => Ok(SpreadingFactor::_10),
        11 => Ok(SpreadingFactor::_11),
        12 => Ok(SpreadingFactor::_12),
        _ => Err(Sx1262Error::InvalidParam),
    }
}

fn map_bandwidth(bw: u8) -> Result<Bandwidth> {
    match bw {
        0x00 => Ok(Bandwidth::_7KHz),
        0x08 => Ok(Bandwidth::_10KHz),
        0x01 => Ok(Bandwidth::_15KHz),
        0x09 => Ok(Bandwidth::_20KHz),
        0x02 => Ok(Bandwidth::_31KHz),
        0x0A => Ok(Bandwidth::_41KHz),
        0x03 => Ok(Bandwidth::_62KHz),
        0x04 => Ok(Bandwidth::_125KHz),
        0x05 => Ok(Bandwidth::_250KHz),
        0x06 => Ok(Bandwidth::_500KHz),
        _ => Err(Sx1262Error::InvalidParam),
    }
}

fn map_coding_rate(cr: u8) -> Result<CodingRate> {
    match cr {
        0x01 => Ok(CodingRate::_4_5),
        0x02 => Ok(CodingRate::_4_6),
        0x03 => Ok(CodingRate::_4_7),
        0x04 => Ok(CodingRate::_4_8),
        _ => Err(Sx1262Error::InvalidParam),
    }
}

fn map_tcxo_voltage(voltage: u8) -> Result<TcxoCtrlVoltage> {
    match voltage {
        0x00 => Ok(TcxoCtrlVoltage::Ctrl1V6),
        0x01 => Ok(TcxoCtrlVoltage::Ctrl1V7),
        0x02 => Ok(TcxoCtrlVoltage::Ctrl1V8),
        0x03 => Ok(TcxoCtrlVoltage::Ctrl2V2),
        0x04 => Ok(TcxoCtrlVoltage::Ctrl2V4),
        0x05 => Ok(TcxoCtrlVoltage::Ctrl2V7),
        0x06 => Ok(TcxoCtrlVoltage::Ctrl3V0),
        0x07 => Ok(TcxoCtrlVoltage::Ctrl3V3),
        _ => Err(Sx1262Error::InvalidParam),
    }
}
