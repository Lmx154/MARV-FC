#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::uart::{Config as UartConfig, Uart, InterruptHandler as UartInterruptHandler, Async as UartAsync};
use embassy_time::{Duration, Timer};

use common::drivers::sx1262::*;
use common::lora::lora_config::LoRaConfig;
use common::lora::link::{LoRaLink, Sx1262Interface};
use common::mavlink2;
use common::utils::delay::DelayMs;
use common::tasks::coms::{TelemetrySample, statustext_to_str};
use common::mavlink2::prelude::dialects::common::Common;
use common::coms::uart_coms::{AsyncUartBus, MAVLINK_MAX_FRAME};

// Simple embassy-based delay
struct EmbassyDelay;
impl DelayMs for EmbassyDelay {
    async fn delay_ms(&mut self, ms: u32) {
        Timer::after(Duration::from_millis(ms.into())).await;
    }
}

embassy_rp::bind_interrupts!(struct UartIrqs {
    UART0_IRQ => UartInterruptHandler<embassy_rp::peripherals::UART0>;
});

struct RpUart<'d>(Uart<'d, UartAsync>);

impl<'d> AsyncUartBus for RpUart<'d> {
    type Error = embassy_rp::uart::Error;

    async fn write(&mut self, bytes: &[u8]) -> core::result::Result<(), Self::Error> {
        Uart::write(&mut self.0, bytes).await
    }

    async fn read_exact(&mut self, buf: &mut [u8]) -> core::result::Result<(), Self::Error> {
        Uart::read(&mut self.0, buf).await
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Radio Boot");

    let p = embassy_rp::init(Default::default());

    // SPI
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

    let nss   = Output::new(p.PIN_5, Level::High);
    let reset = Output::new(p.PIN_1, Level::High);
    let busy  = Input::new(p.PIN_0, Pull::None);
    let dio1  = Input::new(p.PIN_6, Pull::None);

    // RF Switch (external TXEN/RXEN)
    struct ExtSw {
        tx: Output<'static>,
        rx: Output<'static>,
    }
    impl RfSwitch for ExtSw {
        fn set(&mut self, s: RfState) {
            match s {
                RfState::Rx => {
                    self.rx.set_low();
                    self.tx.set_high();
                }
                RfState::Tx => {
                    self.rx.set_high();
                    self.tx.set_low();
                }
                RfState::Off => {
                    self.rx.set_low();
                    self.tx.set_low();
                }
            }
        }
    }

    let rf_sw = ExtSw {
        tx: Output::new(p.PIN_8, Level::Low),
        rx: Output::new(p.PIN_9, Level::Low),
    };

    let cfg = LoRaConfig::preset_default();
    info!(
        "Radio LoRa cfg: f={} Hz sf={} bw_code={} cr_code={} sw=0x{:04X}",
        cfg.freq_hz, cfg.sf, cfg.bw, cfg.cr, cfg.sync_word
    );

    let mut radio = Sx1262::new(spi, nss, reset, busy, dio1, rf_sw, cfg);
    let mut delay = EmbassyDelay;

    radio.init(&mut delay).await.unwrap();

    let mut link = LoRaLink::new(&mut radio);
    link.ack_timeout_ms = cfg.link_ack_timeout_ms;
    link.retries = cfg.link_retries;
    info!(
        "Radio: LoRaLink bridge (ack_timeout_ms={} retries={})",
        link.ack_timeout_ms,
        link.retries
    );

    // UART0 (Radio <-> FC) on GP12 (TX) / GP13 (RX)
    let mut uart_cfg = UartConfig::default();
    uart_cfg.baudrate = 115_200;
    let mut uart = RpUart(Uart::new(
        p.UART0,
        p.PIN_12,
        p.PIN_13,
        UartIrqs,
        p.DMA_CH2,
        p.DMA_CH3,
        uart_cfg,
    ));

    let mut uart_buf = [0u8; MAVLINK_MAX_FRAME];
    uart_bridge_loop(&mut uart, &mut uart_buf, &mut link, &mut delay).await;
}

async fn uart_bridge_loop<'a, RADIO>(
    uart: &mut RpUart<'static>,
    scratch: &mut [u8; MAVLINK_MAX_FRAME],
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut EmbassyDelay,
) where
    RADIO: Sx1262Interface,
{
    loop {
        let frame = match mavlink2::recv_frame_over_uart(uart, scratch).await {
            Ok(f) => f,
            Err(e) => {
                warn!("UART RX error: {:?}", e);
                continue;
            }
        };

        match frame.decode::<Common>() {
            Ok(Common::Statustext(msg)) => {
                let text = statustext_to_str(&msg);
                info!(
                    "UART RX sys={} comp={} seq={}: {}",
                    frame.system_id(),
                    frame.component_id(),
                    frame.sequence(),
                    text
                );
            }
            Ok(Common::EncapsulatedData(pkt)) => {
                if let Some(sample) = TelemetrySample::decode(&pkt.data) {
                    info!(
                        "UART RX tel seq={} acc[{},{},{}] gyro[{},{},{}] mag[{},{},{}] P:{} Pa T:{}x10C GPS lat {} lon {} alt {}mm sat {} fix {}",
                        pkt.seqnr,
                        sample.accel[0], sample.accel[1], sample.accel[2],
                        sample.gyro[0], sample.gyro[1], sample.gyro[2],
                        sample.mag[0], sample.mag[1], sample.mag[2],
                        sample.baro_p_pa,
                        sample.baro_t_cx10,
                        sample.gps_lat_e7,
                        sample.gps_lon_e7,
                        sample.gps_alt_mm,
                        sample.gps_sat,
                        sample.gps_fix,
                    );
                } else {
                    warn!("UART RX: telemetry decode error");
                }
            }
            Ok(_) => warn!("UART RX: unsupported MAVLink Common message"),
            Err(_) => warn!("UART RX: MAV decode error"),
        }

        if let Err(e) = mavlink2::send_frame_over_lora(link, delay, &frame).await {
            warn!("LoRa forward error: {:?}", e);
        }
    }
}
