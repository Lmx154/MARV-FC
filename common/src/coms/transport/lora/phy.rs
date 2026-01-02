#![allow(dead_code)]

use embassy_futures::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use heapless::Vec;
use lora_phy::mod_params::RadioError;

use super::lora_config::LoRaConfig;
use crate::drivers::sx1262::{RawRx, Sx1262, Sx1262Error};

pub const MAX_PHY_PAYLOAD: usize = 255;
pub const DEFAULT_TX_QUEUE_LEN: usize = 4;
pub const DEFAULT_RX_QUEUE_LEN: usize = 8;
pub const DEFAULT_RX_CTRL_QUEUE_LEN: usize = 2;

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

#[derive(Clone, Copy, Debug)]
pub struct RxControl {
    pub timeout_symbols: u16,
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
    pub rx_ctrl: Channel<RawMutex, RxControl, DEFAULT_RX_CTRL_QUEUE_LEN>,
}

impl<const TXQ: usize, const RXQ: usize> PhyChannels<TXQ, RXQ> {
    pub const fn new() -> Self {
        Self {
            tx: Channel::new(),
            rx: Channel::new(),
            profile: Channel::new(),
            rx_ctrl: Channel::new(),
        }
    }

    pub fn phy(&self) -> Phy<'_, TXQ, RXQ> {
        Phy {
            tx: self.tx.sender(),
            rx: self.rx.receiver(),
            profile: self.profile.sender(),
            rx_ctrl: self.rx_ctrl.sender(),
        }
    }

    pub fn service_queues(&self) -> PhyServiceQueues<'_, TXQ, RXQ> {
        PhyServiceQueues {
            tx: self.tx.receiver(),
            rx: self.rx.sender(),
            profile: self.profile.receiver(),
            rx_ctrl: self.rx_ctrl.receiver(),
        }
    }
}

#[derive(Clone, Copy)]
pub struct Phy<'a, const TXQ: usize, const RXQ: usize> {
    tx: Sender<'a, RawMutex, TxRequest, TXQ>,
    rx: Receiver<'a, RawMutex, RxIndication, RXQ>,
    profile: Sender<'a, RawMutex, ProfileId, 1>,
    rx_ctrl: Sender<'a, RawMutex, RxControl, DEFAULT_RX_CTRL_QUEUE_LEN>,
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

    pub fn arm_rx(&self, timeout_symbols: u16) {
        let _ = self
            .rx_ctrl
            .try_send(RxControl { timeout_symbols });
    }
}

pub struct PhyServiceQueues<'a, const TXQ: usize, const RXQ: usize> {
    tx: Receiver<'a, RawMutex, TxRequest, TXQ>,
    rx: Sender<'a, RawMutex, RxIndication, RXQ>,
    profile: Receiver<'a, RawMutex, ProfileId, 1>,
    rx_ctrl: Receiver<'a, RawMutex, RxControl, DEFAULT_RX_CTRL_QUEUE_LEN>,
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
    rx_ctrl: Receiver<'a, RawMutex, RxControl, DEFAULT_RX_CTRL_QUEUE_LEN>,
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
            rx_ctrl: queues.rx_ctrl,
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

            if let Ok(ctrl) = self.rx_ctrl.try_receive() {
                self.rx_once(ctrl.timeout_symbols, &mut rx_buf).await;
                continue;
            }

            if self.cfg.rx_timeout_symbols == 0 {
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

            let wait_fut = self.radio.wait_raw(&mut rx_buf);
            let tx_fut = self.tx.receive();
            let ctrl_fut = self.rx_ctrl.receive();
            match select::select3(tx_fut, ctrl_fut, wait_fut).await {
                select::Either3::First(req) => {
                    self.handle_tx(req).await;
                }
                select::Either3::Second(ctrl) => {
                    self.rx_once(ctrl.timeout_symbols, &mut rx_buf).await;
                }
                select::Either3::Third(result) => {
                    self.handle_rx_result(result, &rx_buf).await;
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

    async fn rx_once(&mut self, timeout_symbols: u16, rx_buf: &mut [u8; MAX_PHY_PAYLOAD]) {
        if timeout_symbols == 0 {
            return;
        }

        if let Err(err) = self.radio.start_rx_single(timeout_symbols).await {
            defmt::warn!("phy rx start error: {:?}", defmt::Debug2Format(&err));
            return;
        }

        let result = self.radio.wait_raw(rx_buf).await;
        self.handle_rx_result(result, rx_buf).await;
    }

    async fn handle_rx_result(
        &mut self,
        result: Result<Option<RawRx>, Sx1262Error>,
        rx_buf: &[u8; MAX_PHY_PAYLOAD],
    ) {
        match result {
            Ok(Some(pkt)) => {
                self.handle_rx(pkt, rx_buf).await;
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
