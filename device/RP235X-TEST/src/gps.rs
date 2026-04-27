use common::drivers::gnss::ublox_m10::{Event, UbloxM10};
use common::protocol::ubx::{MAX_CFG_VALSET_FRAME_LEN, SamM10qConfig};
use common::utilities::time::MeasurementTimestamp;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Pin, Pull};
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartTx};
use embassy_time::{Duration, Instant, Ticker, Timer};
use rp235x_base::pio_uart::new_duplex_pio_uart;

use crate::buses::GpsPioUartBus;
use crate::channels::GPS_CHANNEL;
use crate::pinmap;
use crate::resources::GpsPioUartPins;

const GPS_NAV_RATE_HZ: u8 = 5;
const GPS_BOOT_DELAY: Duration = Duration::from_millis(750);
const GPS_CONFIG_RETRY_PERIOD: Duration = Duration::from_secs(1);
const GPS_STALE_RECONFIGURE_MS: u64 = 2_500;
const GPS_RX_LOG_INTERVAL_BYTES: u32 = 2_048;
const GPS_CONFIG_INITIAL_RETRIES: u32 = 3;
const GPS_CONFIG_RESEND_INTERVAL_TICKS: u32 = 5;
const GPS_NO_RX_LOG_INTERVAL_TICKS: u32 = 5;
const GPS_RX_EDGE_PROBE_TIMEOUT: Duration = Duration::from_secs(2);
const GPS_PIO_UART_DIAGNOSTIC_PING: bool = true;
const GPS_DIAGNOSTIC_PING_PERIOD: Duration = Duration::from_secs(1);

bind_interrupts!(struct GpsPioIrqs {
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
});

pub fn spawn(spawner: &Spawner, bus: GpsPioUartBus, pins: GpsPioUartPins) {
    spawner.spawn(gps_pio_uart_task(bus.pio, pins)).unwrap();
}

#[embassy_executor::task]
async fn gps_pio_uart_task(pio_peripheral: Peri<'static, PIO1>, pins: GpsPioUartPins) -> ! {
    Timer::after(GPS_BOOT_DELAY).await;
    // Diagnostic only: clone the pin before the PIO driver owns it so we can tell
    // whether the GPS TX line is electrically active at all.
    probe_rx_pin_activity(unsafe { pins.rx.clone_unchecked() }).await;

    let mut pio = Pio::new(pio_peripheral, GpsPioIrqs);
    let (mut gps_tx, mut gps_rx) = new_duplex_pio_uart(
        &mut pio.common,
        pio.sm0,
        pio.sm1,
        pins,
        pinmap::GPS_PIO_UART_BAUD,
    );
    info!(
        "sam-m10q gps pio uart ready: tx=GP{=u8} rx=GP{=u8} baud={=u32} nav_hz={=u8}",
        pinmap::GPS_PIO_UART_TX,
        pinmap::GPS_PIO_UART_RX,
        pinmap::GPS_PIO_UART_BAUD,
        GPS_NAV_RATE_HZ,
    );

    if GPS_PIO_UART_DIAGNOSTIC_PING {
        gps_uart_diagnostic_ping(&mut gps_tx, &mut gps_rx).await;
    }

    let mut gps = UbloxM10::new();
    let config = SamM10qConfig {
        baud_rate: pinmap::GPS_PIO_UART_BAUD,
        nav_rate_hz: GPS_NAV_RATE_HZ,
        ..SamM10qConfig::DEFAULT_PIO_UART
    };
    let mut payload_scratch = [0u8; 1];
    let mut tx_frame = [0u8; MAX_CFG_VALSET_FRAME_LEN];

    let mut poll_frame = [0u8; 8];
    let mut config_acknowledged = false;
    let mut rx_byte_count = 0u32;
    let mut nav_pvt_count = 0u32;
    let mut last_nav_pvt_ms = 0u64;
    let mut retry_ticks = 0u32;
    let mut previous_rx_byte = 0u8;
    let mut first_rx_logged = false;
    let mut retry = Ticker::every(GPS_CONFIG_RETRY_PERIOD);

    send_startup_config(&mut gps_tx, config, &mut payload_scratch, &mut tx_frame, 0).await;
    send_nav_pvt_poll(&mut gps_tx, &mut poll_frame).await;

    loop {
        let byte = match select(gps_rx.read_u8(), retry.next()).await {
            Either::First(byte) => byte,
            Either::Second(()) => {
                retry_ticks = retry_ticks.wrapping_add(1);
                let stale = last_nav_pvt_ms == 0
                    || Instant::now().as_millis().saturating_sub(last_nav_pvt_ms)
                        > GPS_STALE_RECONFIGURE_MS;
                let should_resend_config = !config_acknowledged
                    && (retry_ticks <= GPS_CONFIG_INITIAL_RETRIES
                        || retry_ticks % GPS_CONFIG_RESEND_INTERVAL_TICKS == 0);

                if rx_byte_count == 0 && retry_ticks % GPS_NO_RX_LOG_INTERVAL_TICKS == 0 {
                    warn!(
                        "sam-m10q no rx bytes yet after {=u32}s; check gps tx->GP{=u8}, ground, power, and baud",
                        retry_ticks,
                        pinmap::GPS_PIO_UART_RX,
                    );
                }

                if should_resend_config
                    || (stale && retry_ticks % GPS_CONFIG_RESEND_INTERVAL_TICKS == 0)
                {
                    send_startup_config(
                        &mut gps_tx,
                        config,
                        &mut payload_scratch,
                        &mut tx_frame,
                        retry_ticks,
                    )
                    .await;
                }
                if stale {
                    send_nav_pvt_poll(&mut gps_tx, &mut poll_frame).await;
                }
                continue;
            }
        };

        rx_byte_count = rx_byte_count.wrapping_add(1);
        if !first_rx_logged {
            first_rx_logged = true;
            info!("sam-m10q first rx byte=0x{=u8:02x}", byte);
        }
        if byte == b'$' {
            info!("sam-m10q nmea start byte observed");
        }
        if previous_rx_byte == 0xb5 && byte == 0x62 {
            info!("sam-m10q ubx sync observed");
        }
        previous_rx_byte = byte;

        if rx_byte_count % GPS_RX_LOG_INTERVAL_BYTES == 0 {
            info!(
                "sam-m10q rx bytes={=u32} nav_pvt={=u32}",
                rx_byte_count, nav_pvt_count,
            );
        }

        let event = match gps.push_byte(byte) {
            Ok(event) => event,
            Err(_) => {
                warn!("sam-m10q ubx parser error; resetting parser");
                gps.reset_parser();
                continue;
            }
        };

        match event {
            Some(Event::Ack(ack)) if ack.acknowledged => {
                info!(
                    "sam-m10q ack class=0x{=u8:02x} id=0x{=u8:02x}",
                    ack.class_id, ack.message_id,
                );
                if ack.class_id == 0x06 && ack.message_id == 0x8a {
                    config_acknowledged = true;
                }
            }
            Some(Event::Ack(ack)) => {
                warn!(
                    "sam-m10q nack class=0x{=u8:02x} id=0x{=u8:02x}",
                    ack.class_id, ack.message_id,
                );
            }
            Some(Event::NavPvt(nav_pvt)) => {
                let now = Instant::now();
                let timestamp = MeasurementTimestamp::from_micros(now.as_micros());
                nav_pvt_count = nav_pvt_count.wrapping_add(1);
                last_nav_pvt_ms = now.as_millis();
                config_acknowledged = true;
                GPS_CHANNEL
                    .immediate_publisher()
                    .publish_immediate(nav_pvt.gps_fix_sample_stamped(timestamp));
            }
            Some(Event::OtherPacket) | None => {}
        }
    }
}

async fn gps_uart_diagnostic_ping(
    gps_tx: &mut PioUartTx<'_, PIO1, 0>,
    gps_rx: &mut PioUartRx<'_, PIO1, 1>,
) -> ! {
    let mut ticker = Ticker::every(GPS_DIAGNOSTIC_PING_PERIOD);
    let mut tx_count = 0u32;
    let mut rx_count = 0u32;

    warn!(
        "gps pio uart diagnostic ping enabled: tx=GP{=u8} rx=GP{=u8} baud={=u32}",
        pinmap::GPS_PIO_UART_TX,
        pinmap::GPS_PIO_UART_RX,
        pinmap::GPS_PIO_UART_BAUD,
    );

    loop {
        match select(gps_rx.read_u8(), ticker.next()).await {
            Either::First(byte) => {
                rx_count = rx_count.wrapping_add(1);
                info!(
                    "gps pio diag rx count={=u32} byte=0x{=u8:02x} ascii={=u8}",
                    rx_count, byte, byte,
                );
            }
            Either::Second(()) => {
                tx_count = tx_count.wrapping_add(1);
                write_gps_bytes(gps_tx, b"MARV-PIO-PING\n").await;
                info!("gps pio diag tx ping count={=u32}", tx_count);
            }
        }
    }
}

async fn probe_rx_pin_activity<RxPin>(rx_pin: Peri<'static, RxPin>)
where
    RxPin: Pin + 'static,
{
    let mut input = Input::new(rx_pin, Pull::None);
    info!(
        "sam-m10q rx pin GP{=u8} idle_high={=bool}",
        pinmap::GPS_PIO_UART_RX,
        input.is_high(),
    );

    if embassy_time::with_timeout(GPS_RX_EDGE_PROBE_TIMEOUT, input.wait_for_any_edge())
        .await
        .is_ok()
    {
        info!(
            "sam-m10q rx pin GP{=u8} edge observed before pio uart",
            pinmap::GPS_PIO_UART_RX,
        );
    } else {
        warn!(
            "sam-m10q rx pin GP{=u8} had no edges during pre-pio probe",
            pinmap::GPS_PIO_UART_RX,
        );
    }
}

async fn send_startup_config(
    gps_tx: &mut PioUartTx<'_, PIO1, 0>,
    config: SamM10qConfig,
    payload_scratch: &mut [u8],
    tx_frame: &mut [u8; MAX_CFG_VALSET_FRAME_LEN],
    attempt: u32,
) {
    match UbloxM10::encode_startup_config(config, payload_scratch, tx_frame) {
        Ok(len) => {
            write_gps_bytes(gps_tx, &tx_frame[..len]).await;
            info!(
                "sam-m10q startup config sent bytes={=usize} attempt={=u32}",
                len, attempt,
            );
        }
        Err(_) => warn!("sam-m10q startup config encode failed"),
    }
}

async fn send_nav_pvt_poll(gps_tx: &mut PioUartTx<'_, PIO1, 0>, poll_frame: &mut [u8; 8]) {
    if let Ok(len) = UbloxM10::encode_poll_nav_pvt(poll_frame) {
        write_gps_bytes(gps_tx, &poll_frame[..len]).await;
    }
}

async fn write_gps_bytes(gps_tx: &mut PioUartTx<'_, PIO1, 0>, bytes: &[u8]) {
    for byte in bytes {
        gps_tx.write_u8(*byte).await;
    }
}
