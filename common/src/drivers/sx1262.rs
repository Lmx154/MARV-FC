#![allow(async_fn_in_trait)]

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use lora_phy::LoRa;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::{ModulationParams, PacketParams, RadioError, RxMode};
use lora_phy::mod_traits::RadioKind;
use lora_phy::sx126x::{Config as Sx126xConfig, DeviceSel, Sx126x, Sx126xVariant, TcxoCtrlVoltage};

use crate::comms::links::lora::{LoraProfile, TcxoVoltage};

#[derive(Debug, Clone, Copy)]
pub struct RawRx {
    pub len: u8,
    pub rssi: i16,
    pub snr_x4: i16,
}

#[derive(Debug)]
pub enum Sx1262Error {
    Radio(RadioError),
    InvalidParam,
}

impl Sx1262Error {
    pub const fn is_receive_timeout(&self) -> bool {
        matches!(self, Self::Radio(RadioError::ReceiveTimeout))
    }
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

pub struct WaveshareSx1262<SPI, CTRL, WAIT, DLY>
where
    SPI: SpiDevice<u8>,
    CTRL: OutputPin,
    WAIT: Wait,
    DLY: DelayNs + Clone,
{
    lora: LoRa<Sx1262Radio<SPI, CTRL, WAIT>, DLY>,
    profile: LoraProfile,
    mod_params: ModulationParams,
    rx_pkt_params: PacketParams,
    tx_pkt_params: PacketParams,
}

pub type Sx1262<SPI, CTRL, WAIT, DLY> = WaveshareSx1262<SPI, CTRL, WAIT, DLY>;

impl<SPI, CTRL, WAIT, DLY> WaveshareSx1262<SPI, CTRL, WAIT, DLY>
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
        profile: LoraProfile,
        delay: DLY,
    ) -> Result<Self> {
        let sx_cfg = Sx126xConfig {
            chip: ExternalRfSwitch,
            tcxo_ctrl: profile.tcxo.map(|tcxo| tcxo.voltage.into()),
            use_dcdc: profile.use_dcdc,
            rx_boost: false,
        };

        // Waveshare's SX1262 node labels the two external switch controls opposite
        // of lora-phy's rx/tx switch roles, so pass the pins swapped here.
        let (rf_rx, rf_tx) = (rf_switch_tx, rf_switch_rx);
        let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, Some(rf_rx), Some(rf_tx))
            .map_err(Sx1262Error::Radio)?;

        let radio = Sx126x::new(spi, iv, sx_cfg);

        let mut delay_extra = delay.clone();
        let enable_public_network = profile.sync_word == 0x34;
        let mut lora = LoRa::new(radio, enable_public_network, delay).await?;
        if let Some(tcxo) = profile.tcxo {
            let extra_ms = tcxo.startup_delay_ms.saturating_sub(10);
            if extra_ms > 0 {
                delay_extra.delay_ms(extra_ms).await;
            }
        }

        let (mod_params, rx_pkt_params, tx_pkt_params) = build_params(&mut lora, &profile)?;

        Ok(Self {
            lora,
            profile,
            mod_params,
            rx_pkt_params,
            tx_pkt_params,
        })
    }

    pub fn profile(&self) -> &LoraProfile {
        &self.profile
    }

    pub async fn apply_profile(&mut self, profile: LoraProfile) -> Result<()> {
        if profile.sync_word != self.profile.sync_word {
            return Err(Sx1262Error::InvalidParam);
        }

        self.profile = profile;
        let (mod_params, rx_pkt_params, tx_pkt_params) =
            build_params(&mut self.lora, &self.profile)?;
        self.mod_params = mod_params;
        self.rx_pkt_params = rx_pkt_params;
        self.tx_pkt_params = tx_pkt_params;

        self.start_rx_continuous().await?;
        Ok(())
    }

    pub async fn reinitialize(&mut self) -> Result<()> {
        self.lora.init().await?;
        let (mod_params, rx_pkt_params, tx_pkt_params) =
            build_params(&mut self.lora, &self.profile)?;
        self.mod_params = mod_params;
        self.rx_pkt_params = rx_pkt_params;
        self.tx_pkt_params = tx_pkt_params;
        Ok(())
    }

    pub async fn recover_rx_continuous(&mut self) -> Result<()> {
        self.reinitialize().await?;
        self.start_rx_continuous().await
    }

    pub async fn transmit(&mut self, payload: &[u8]) -> Result<()> {
        if payload.len() > 255 {
            return Err(Sx1262Error::InvalidParam);
        }

        let power = self.profile.tx_power_dbm as i32;
        self.lora
            .prepare_for_tx(&self.mod_params, &mut self.tx_pkt_params, power, payload)
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

    pub async fn receive(&mut self, buf: &mut [u8]) -> Result<Option<RawRx>> {
        let (len, status) = self.lora.rx(&self.rx_pkt_params, buf).await?;
        if len == 0 {
            return Ok(None);
        }
        Ok(Some(RawRx {
            len,
            rssi: status.rssi,
            snr_x4: status.snr.saturating_mul(4),
        }))
    }
}

fn build_params<RK, DLY>(
    lora: &mut LoRa<RK, DLY>,
    profile: &LoraProfile,
) -> Result<(ModulationParams, PacketParams, PacketParams)>
where
    RK: RadioKind,
    DLY: DelayNs,
{
    let mod_params = lora.create_modulation_params(
        profile.modulation.sf,
        profile.modulation.bw,
        profile.modulation.cr,
        profile.frequency_hz,
    )?;

    let implicit_header = !profile.explicit_header;
    let rx_params = lora.create_rx_packet_params(
        profile.preamble_len,
        implicit_header,
        0xFF,
        profile.crc_on,
        profile.invert_iq,
        &mod_params,
    )?;
    let tx_params = lora.create_tx_packet_params(
        profile.preamble_len,
        implicit_header,
        profile.crc_on,
        profile.invert_iq,
        &mod_params,
    )?;

    Ok((mod_params, rx_params, tx_params))
}

impl From<TcxoVoltage> for TcxoCtrlVoltage {
    fn from(value: TcxoVoltage) -> Self {
        match value {
            TcxoVoltage::V1_6 => TcxoCtrlVoltage::Ctrl1V6,
            TcxoVoltage::V1_7 => TcxoCtrlVoltage::Ctrl1V7,
            TcxoVoltage::V1_8 => TcxoCtrlVoltage::Ctrl1V8,
            TcxoVoltage::V2_2 => TcxoCtrlVoltage::Ctrl2V2,
            TcxoVoltage::V2_4 => TcxoCtrlVoltage::Ctrl2V4,
            TcxoVoltage::V2_7 => TcxoCtrlVoltage::Ctrl2V7,
            TcxoVoltage::V3_0 => TcxoCtrlVoltage::Ctrl3V0,
            TcxoVoltage::V3_3 => TcxoCtrlVoltage::Ctrl3V3,
        }
    }
}
