#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use static_cell::StaticCell;

use common::coms::transport::lora::lora_config::LoRaConfig;
use common::drivers::sx1262::Sx1262;

#[derive(Clone, Copy)]
enum Role {
    Tx,
    Rx,
}

const ROLE: Role = Role::Tx; // flip to Role::Rx to swap roles
const TX_INTERVAL_MS: u64 = 1_000;

type SpiBus = Mutex<RawMutex, Spi<'static, SPI0, Async>>;
static SPI_BUS: StaticCell<SpiBus> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("GS L1 ping-pong (SX1262)");

    let p = embassy_rp::init(Default::default());

    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 10_000_000;

    let spi = Spi::new(
        p.SPI0,
        p.PIN_2,  // SCK
        p.PIN_3,  // MOSI
        p.PIN_4,  // MISO
        p.DMA_CH0,
        p.DMA_CH1,
        spi_cfg,
    );

    let spi_bus = SPI_BUS.init(Mutex::new(spi));
    let nss = Output::new(p.PIN_5, Level::High);
    let spi_dev = SpiDevice::new(spi_bus, nss);

    let reset = Output::new(p.PIN_1, Level::High);
    let busy = Input::new(p.PIN_0, Pull::None);
    let dio1 = Input::new(p.PIN_6, Pull::None);

    let rf_tx = Output::new(p.PIN_8, Level::Low);
    let rf_rx = Output::new(p.PIN_9, Level::Low);

    let cfg = LoRaConfig::preset_default();
    info!(
        "LoRa cfg: f={} Hz sf={} bw_code={} cr_code={} sw=0x{:04X}",
        cfg.freq_hz, cfg.sf, cfg.bw, cfg.cr, cfg.sync_word
    );

    let mut radio = Sx1262::new(
        spi_dev,
        reset,
        busy,
        dio1,
        rf_tx,
        rf_rx,
        cfg,
        Delay,
    )
    .await
    .unwrap();

    match ROLE {
        Role::Tx => {
            let mut seq: u32 = 0;
            loop {
                seq = seq.wrapping_add(1);
                match radio.tx_raw(b"PING").await {
                    Ok(()) => info!("TX seq={} len=4", seq),
                    Err(e) => warn!("TX error: {:?}", defmt::Debug2Format(&e)),
                }
                Timer::after(Duration::from_millis(TX_INTERVAL_MS)).await;
            }
        }
        Role::Rx => {
            if let Err(e) = radio.start_rx_continuous().await {
                warn!("RX start error: {:?}", defmt::Debug2Format(&e));
            }

            let mut rx_buf = [0u8; 255];
            let mut rx_ok: u32 = 0;
            let mut rx_err: u32 = 0;
            let mut ping_ok: u32 = 0;

            loop {
                match radio.wait_raw(&mut rx_buf).await {
                    Ok(Some(pkt)) => {
                        rx_ok = rx_ok.wrapping_add(1);
                        let payload = &rx_buf[..pkt.len as usize];
                        if payload == b"PING" {
                            ping_ok = ping_ok.wrapping_add(1);
                        } else {
                            warn!("RX payload mismatch len={}", pkt.len);
                        }

                        info!(
                            "RX ok={} ping_ok={} rssi={} snr_x4={}",
                            rx_ok, ping_ok, pkt.rssi, pkt.snr_x4
                        );
                    }
                    Ok(None) => {}
                    Err(e) => {
                        rx_err = rx_err.wrapping_add(1);
                        warn!(
                            "RX error count={} err={:?}",
                            rx_err,
                            defmt::Debug2Format(&e)
                        );
                        let _ = radio.start_rx_continuous().await;
                    }
                }
            }
        }
    }
}
