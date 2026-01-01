#![allow(dead_code)]

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use heapless::Vec;
use lora_phy::mod_params::RadioError;

use super::lora_config::LoRaConfig;
use crate::drivers::sx1262::{RawRx, Sx1262, Sx1262Error};

pub const MAX_PHY_PAYLOAD: usize = 255;
pub const DEFAULT_TX_QUEUE_LEN: usize = 4;
pub const DEFAULT_RX_QUEUE_LEN: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProfileId {
    Default,
    Fast,
    LongRange,
}

impl ProfileId {
    pub fn config(self) -> LoRaConfig {
        match self {
            ProfileId::Default => LoRaConfig::preset_default(),
            ProfileId::Fast => LoRaConfig::preset_fast(),
            ProfileId::LongRange => LoRaConfig::preset_long_range(),
        }
    }

    pub fn from_config(cfg: &LoRaConfig) -> Option<Self> {
        if *cfg == LoRaConfig::preset_default() {
            Some(ProfileId::Default)
        } else if *cfg == LoRaConfig::preset_fast() {
            Some(ProfileId::Fast)
        } else if *cfg == LoRaConfig::preset_long_range() {
            Some(ProfileId::LongRange)
        } else {
            None
        }
    }
}

#[derive(Clone, Debug)]
pub struct TxRequest {
    pub bytes: Vec<u8, MAX_PHY_PAYLOAD>,
}

#[derive(Clone, Debug)]
pub struct RxIndication {
    pub bytes: Vec<u8, MAX_PHY_PAYLOAD>,
    pub rssi: i16,
    pub snr: i16,
    pub timestamp_ms: u64,
    pub rx_done_instant_us: u64,
}

#[derive(Debug)]
pub enum PhyError {
    PayloadTooLarge,
}

pub trait TimeSource {
    fn now_us(&self) -> u64;

    fn now_ms(&self) -> u64 {
        self.now_us() / 1000
    }
}

impl TimeSource for () {
    fn now_us(&self) -> u64 {
        0
    }
}

pub fn toa_us(profile: ProfileId, payload_len: usize) -> u64 {
    profile.config().toa_us(payload_len)
}

pub struct PhyChannels<const TXQ: usize, const RXQ: usize> {
    pub tx: Channel<RawMutex, TxRequest, TXQ>,
    pub rx: Channel<RawMutex, RxIndication, RXQ>,
    pub profile: Channel<RawMutex, ProfileId, 1>,
}

impl<const TXQ: usize, const RXQ: usize> PhyChannels<TXQ, RXQ> {
    pub const fn new() -> Self {
        Self {
            tx: Channel::new(),
            rx: Channel::new(),
            profile: Channel::new(),
        }
    }

    pub fn phy(&self) -> Phy<'_, TXQ, RXQ> {
        Phy {
            tx: self.tx.sender(),
            rx: self.rx.receiver(),
            profile: self.profile.sender(),
        }
    }

    pub fn service_queues(&self) -> PhyServiceQueues<'_, TXQ, RXQ> {
        PhyServiceQueues {
            tx: self.tx.receiver(),
            rx: self.rx.sender(),
            profile: self.profile.receiver(),
        }
    }
}

#[derive(Clone, Copy)]
pub struct Phy<'a, const TXQ: usize, const RXQ: usize> {
    tx: Sender<'a, RawMutex, TxRequest, TXQ>,
    rx: Receiver<'a, RawMutex, RxIndication, RXQ>,
    profile: Sender<'a, RawMutex, ProfileId, 1>,
}

impl<'a, const TXQ: usize, const RXQ: usize> Phy<'a, TXQ, RXQ> {
    pub async fn tx(&self, bytes: &[u8]) -> Result<(), PhyError> {
        let mut buf = Vec::<u8, MAX_PHY_PAYLOAD>::new();
        buf.extend_from_slice(bytes)
            .map_err(|_| PhyError::PayloadTooLarge)?;
        self.tx.send(TxRequest { bytes: buf }).await;
        Ok(())
    }

    pub async fn rx(&self) -> RxIndication {
        self.rx.receive().await
    }

    pub async fn apply_profile(&self, profile: ProfileId) {
        self.profile.send(profile).await;
    }
}

pub struct PhyServiceQueues<'a, const TXQ: usize, const RXQ: usize> {
    tx: Receiver<'a, RawMutex, TxRequest, TXQ>,
    rx: Sender<'a, RawMutex, RxIndication, RXQ>,
    profile: Receiver<'a, RawMutex, ProfileId, 1>,
}

#[derive(Clone, Copy)]
pub struct PhyServiceConfig {
    pub rx_timeout_symbols: u16,
}

impl Default for PhyServiceConfig {
    fn default() -> Self {
        Self {
            rx_timeout_symbols: 16,
        }
    }
}

pub struct PhyService<'a, SPI, CTRL, WAIT, DLY, TS, const TXQ: usize, const RXQ: usize>
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
    DLY: embedded_hal_async::delay::DelayNs + Clone,
    TS: TimeSource,
{
    radio: Sx1262<SPI, CTRL, WAIT, DLY>,
    tx: Receiver<'a, RawMutex, TxRequest, TXQ>,
    rx: Sender<'a, RawMutex, RxIndication, RXQ>,
    profile: Receiver<'a, RawMutex, ProfileId, 1>,
    time: TS,
    cfg: PhyServiceConfig,
    active_profile: ProfileId,
}

impl<'a, SPI, CTRL, WAIT, DLY, TS, const TXQ: usize, const RXQ: usize>
    PhyService<'a, SPI, CTRL, WAIT, DLY, TS, TXQ, RXQ>
where
    SPI: embedded_hal_async::spi::SpiDevice<u8>,
    CTRL: embedded_hal::digital::OutputPin,
    WAIT: embedded_hal_async::digital::Wait,
    DLY: embedded_hal_async::delay::DelayNs + Clone,
    TS: TimeSource,
{
    pub fn new(
        radio: Sx1262<SPI, CTRL, WAIT, DLY>,
        time: TS,
        queues: PhyServiceQueues<'a, TXQ, RXQ>,
        cfg: PhyServiceConfig,
    ) -> Self {
        let active_profile =
            ProfileId::from_config(radio.cfg()).unwrap_or(ProfileId::Default);
        Self {
            radio,
            tx: queues.tx,
            rx: queues.rx,
            profile: queues.profile,
            time,
            cfg,
            active_profile,
        }
    }

    pub async fn run(&mut self) -> ! {
        let mut rx_buf = [0u8; MAX_PHY_PAYLOAD];

        loop {
            if let Ok(profile) = self.profile.try_receive() {
                self.handle_profile(profile).await;
                continue;
            }

            if let Ok(req) = self.tx.try_receive() {
                self.handle_tx(req).await;
                continue;
            }

            if let Err(err) = self
                .radio
                .start_rx_single(self.cfg.rx_timeout_symbols)
                .await
            {
                defmt::warn!("phy rx start error: {:?}", defmt::Debug2Format(&err));
                continue;
            }

            match self.radio.wait_raw(&mut rx_buf).await {
                Ok(Some(pkt)) => {
                    self.handle_rx(pkt, &rx_buf).await;
                }
                Ok(None) => {}
                Err(Sx1262Error::Radio(RadioError::CRCError)) => {}
                Err(Sx1262Error::Radio(RadioError::HeaderError)) => {}
                Err(Sx1262Error::Radio(RadioError::ReceiveTimeout)) => {}
                Err(err) => {
                    defmt::warn!("phy rx error: {:?}", defmt::Debug2Format(&err));
                }
            }
        }
    }

    async fn handle_profile(&mut self, profile: ProfileId) {
        let cfg = profile.config();
        if let Err(err) = self.radio.apply_lora_config(cfg).await {
            defmt::warn!(
                "phy profile apply error: {:?}",
                defmt::Debug2Format(&err)
            );
        }
        self.active_profile = profile;
    }

    async fn handle_tx(&mut self, req: TxRequest) {
        if let Err(err) = self.radio.tx_raw(&req.bytes).await {
            defmt::warn!("phy tx error: {:?}", defmt::Debug2Format(&err));
        }
    }

    async fn handle_rx(&mut self, pkt: RawRx, buf: &[u8; MAX_PHY_PAYLOAD]) {
        let mut bytes = Vec::<u8, MAX_PHY_PAYLOAD>::new();
        let payload = &buf[..pkt.len as usize];
        let _ = bytes.extend_from_slice(payload);
        let rx_done_instant_us = if pkt.irq_instant_us != 0 {
            pkt.irq_instant_us
        } else {
            self.time.now_us()
        };

        let indication = RxIndication {
            bytes,
            rssi: pkt.rssi,
            snr: pkt.snr_x4 / 4,
            timestamp_ms: rx_done_instant_us / 1000,
            rx_done_instant_us,
        };

        if self.rx.try_send(indication).is_err() {
            defmt::warn!("phy rx drop: queue full");
        }
    }
}
