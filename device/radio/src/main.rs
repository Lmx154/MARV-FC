#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use embassy_executor::Spawner;
use embassy_rp::gpio::{Input, Output, Level, Pull};
use embassy_rp::spi::{Config as SpiConfig, Spi};
use embassy_rp::uart::{Config as UartConfig, Uart, InterruptHandler as UartInterruptHandler, Async as UartAsync};
use embassy_time::{Duration, Instant, Timer};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use heapless::Vec;

use common::drivers::sx1262::*;
use common::coms::scheduler::TxGate;
use common::coms::transport::lora::lora_config::LoRaConfig;
use common::coms::transport::lora::link::{LoRaLink, Sx1262Interface};
use common::mavlink2;
use common::utils::delay::DelayMs;
use common::tasks::coms::{TelemetrySample, statustext_to_str};
use common::mavlink2::prelude::dialects::common::Common;
use common::coms::transport::uart::{AsyncUartBus, MAVLINK_MAX_FRAME};

type FrameMsg = Vec<u8, MAVLINK_MAX_FRAME>;

// UART -> LoRa (high priority: commands/params/etc)
static UART_TO_LORA_HI: Channel<RawMutex, FrameMsg, 8> = Channel::new();
// LoRa -> UART (all inbound frames)
static LORA_TO_UART: Channel<RawMutex, FrameMsg, 8> = Channel::new();

// Latest telemetry frame from UART -> LoRa (overwrite semantics, Betaflight-style).
static UART_TO_LORA_TELEM_LATEST: Mutex<RawMutex, Option<FrameMsg>> = Mutex::new(None);
static UART_TO_LORA_TELEM_SIG: Signal<RawMutex, ()> = Signal::new();

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
async fn main(spawner: Spawner) {
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
    let uart = RpUart(Uart::new(
        p.UART0,
        p.PIN_12,
        p.PIN_13,
        UartIrqs,
        p.DMA_CH2,
        p.DMA_CH3,
        uart_cfg,
    ));

    // Spawn UART task (bidirectional) so UART RX is not blocked by LoRa TX pacing/ACK waits.
    spawner.spawn(uart_task(uart)).unwrap();

    // Main task owns the LoRa link and services both RX and TX.
    lora_loop(&mut link, &mut delay).await;
}

#[embassy_executor::task]
async fn uart_task(mut uart: RpUart<'static>) {
    let mut scratch = [0u8; MAVLINK_MAX_FRAME];
    let mut ser_buf = [0u8; MAVLINK_MAX_FRAME];

    let lora_to_uart_rx = LORA_TO_UART.receiver();

    loop {
        let rx_fut = mavlink2::recv_frame_over_uart(&mut uart, &mut scratch);
        let tx_fut = lora_to_uart_rx.receive();

        match embassy_futures::select::select(rx_fut, tx_fut).await {
            embassy_futures::select::Either::First(res) => {
                let frame = match res {
                    Ok(f) => f,
                    Err(e) => {
                        warn!("UART RX error: {:?}", e);
                        continue;
                    }
                };

                // Optional logs (keep lightweight; do not decode everything).
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
                        }
                    }
                    _ => {}
                }

                // Serialize once and forward as raw bytes.
                let written = match frame.serialize(&mut ser_buf) {
                    Ok(n) => n,
                    Err(_) => {
                        warn!("UART RX: serialize failed");
                        continue;
                    }
                };
                let mut msg: FrameMsg = FrameMsg::new();
                if msg.extend_from_slice(&ser_buf[..written]).is_err() {
                    warn!("UART RX: frame too big for queue");
                    continue;
                }

                // Treat EncapsulatedData (131) as telemetry: keep latest, drop older.
                if mavlink2_msg_id(&msg) == Some(131) {
                    *UART_TO_LORA_TELEM_LATEST.lock().await = Some(msg);
                    UART_TO_LORA_TELEM_SIG.signal(());
                } else if UART_TO_LORA_HI.sender().try_send(msg).is_err() {
                    warn!("UART->LoRa queue full (dropping)");
                }
            }
            embassy_futures::select::Either::Second(msg) => {
                if uart.write(&msg).await.is_err() {
                    warn!("UART TX error");
                }
            }
        }
    }
}

async fn lora_loop<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut EmbassyDelay,
) where
    RADIO: Sx1262Interface,
{
    if let Err(e) = link.start_rx(delay).await {
        warn!("Radio: start_rx failed: {:?}", e);
    }

    let mut tx_gate = TxGate::new();
    let mut rx_buf = [0u8; 255];

    let hi_rx = UART_TO_LORA_HI.receiver();

    loop {
        // Service LoRa RX frequently (low latency), but always prioritize high-priority TX.
        let hi_fut = hi_rx.receive();
        let tel_fut = UART_TO_LORA_TELEM_SIG.wait();
        let tick_fut = Timer::after(Duration::from_millis(2));

        match embassy_futures::select::select3(hi_fut, tel_fut, tick_fut).await {
            embassy_futures::select::Either3::First(msg) => {
                lora_send_bytes(link, delay, &mut tx_gate, &msg).await;
            }
            embassy_futures::select::Either3::Second(()) => {
                let maybe = UART_TO_LORA_TELEM_LATEST.lock().await.take();
                if let Some(msg) = maybe {
                    lora_send_bytes(link, delay, &mut tx_gate, &msg).await;
                }
            }
            embassy_futures::select::Either3::Third(()) => {
                match link.try_recv(delay, &mut rx_buf).await {
                    Ok(Some(len)) => {
                        let mut msg: FrameMsg = FrameMsg::new();
                        if msg.extend_from_slice(&rx_buf[..len]).is_ok() {
                            if LORA_TO_UART.sender().try_send(msg).is_err() {
                                warn!("LoRa->UART queue full (dropping)");
                            }
                        }
                    }
                    Ok(None) => {}
                    Err(e) => warn!("LoRa RX error: {:?}", e),
                }
            }
        }
    }
}

async fn lora_send_bytes<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut EmbassyDelay,
    tx_gate: &mut TxGate,
    bytes: &[u8],
) where
    RADIO: Sx1262Interface,
{
    let now_ms = Instant::now().as_millis() as u32;
    let min_gap_ms = link.recommended_tx_gap_ms();
    let wait_ms = tx_gate.time_until_allowed_ms(now_ms, min_gap_ms);
    if wait_ms != 0 {
        delay.delay_ms(wait_ms).await;
    }

    if let Err(e) = link.send(delay, bytes).await {
        warn!("LoRa TX error: {:?}", e);
    } else {
        let now_ms = Instant::now().as_millis() as u32;
        tx_gate.on_tx(now_ms);
    }
}

fn mavlink2_msg_id(frame_bytes: &[u8]) -> Option<u32> {
    // MAVLink2 header: magic(0), len(1), incompat(2), compat(3), seq(4), sys(5), comp(6), msgid(7..9)
    if frame_bytes.len() < 10 || frame_bytes[0] != 0xFD {
        return None;
    }
    let id0 = frame_bytes[7] as u32;
    let id1 = frame_bytes[8] as u32;
    let id2 = frame_bytes[9] as u32;
    Some(id0 | (id1 << 8) | (id2 << 16))
}
