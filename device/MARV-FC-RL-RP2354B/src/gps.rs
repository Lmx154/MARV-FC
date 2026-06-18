use common::drivers::gnss::ublox_m10::{Event, UbloxM10};
use common::protocol::ubx::{MAX_CFG_VALSET_FRAME_LEN, SamM10qConfig};
use common::utilities::time::MeasurementTimestamp;
use defmt::{info, warn};
use embassy_executor::SendSpawner;
use embassy_rp::uart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::buses::GpsUart;
use crate::channels::GPS_CHANNEL;
use crate::interrupts::GpsIrqs;
use crate::resources::GpsPins;

const GPS_BOOT_DELAY: Duration = Duration::from_millis(750);
const GPS_RX_ERROR_BACKOFF: Duration = Duration::from_millis(20);
const GPS_CONFIG: SamM10qConfig = SamM10qConfig {
    baud_rate: crate::pinmap::GPS_UART_BAUD,
    nav_rate_hz: 10,
    enable_nmea_input: false,
    enable_nmea_output: false,
};

static GPS_UART_TX_BUFFER: StaticCell<[u8; crate::config::GPS_UART_BUFFER_BYTES]> =
    StaticCell::new();
static GPS_UART_RX_BUFFER: StaticCell<[u8; crate::config::GPS_UART_BUFFER_BYTES]> =
    StaticCell::new();

pub fn spawn(spawner: SendSpawner, bus: GpsUart, pins: GpsPins) {
    spawner
        .spawn(gps_uart_task(bus, pins))
        .expect("gps uart task spawn failed");
}

#[embassy_executor::task]
async fn gps_uart_task(bus: GpsUart, pins: GpsPins) -> ! {
    Timer::after(GPS_BOOT_DELAY).await;

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = crate::pinmap::GPS_UART_BAUD;

    let tx_buffer = &mut GPS_UART_TX_BUFFER.init([0; crate::config::GPS_UART_BUFFER_BYTES])[..];
    let rx_buffer = &mut GPS_UART_RX_BUFFER.init([0; crate::config::GPS_UART_BUFFER_BYTES])[..];
    let uart = BufferedUart::new(
        bus.uart,
        pins.tx,
        pins.rx,
        GpsIrqs,
        tx_buffer,
        rx_buffer,
        uart_config,
    );
    let (mut tx, rx) = uart.split();

    info!(
        "sam-m10q gps uart ready: mcu_tx=GP{=u8} mcu_rx=GP{=u8} baud={=u32}",
        crate::pinmap::GPS_UART_TX,
        crate::pinmap::GPS_UART_RX,
        crate::pinmap::GPS_UART_BAUD,
    );

    configure_ubx_only(&mut tx).await;
    gps_rx_loop(rx).await
}

async fn configure_ubx_only(tx: &mut BufferedUartTx) {
    let mut payload_scratch = [0u8; MAX_CFG_VALSET_FRAME_LEN];
    let mut frame = [0u8; MAX_CFG_VALSET_FRAME_LEN];

    let len = match UbloxM10::encode_startup_config(GPS_CONFIG, &mut payload_scratch, &mut frame) {
        Ok(len) => len,
        Err(_) => {
            warn!("sam-m10q gps failed to encode UBX-only startup config");
            return;
        }
    };

    if tx.write_all(&frame[..len]).await.is_err() {
        warn!("sam-m10q gps failed to write UBX-only startup config");
        return;
    }

    info!(
        "sam-m10q gps configured for UBX NAV-PVT only: rate={=u8}Hz nmea_in={} nmea_out={}",
        GPS_CONFIG.nav_rate_hz, GPS_CONFIG.enable_nmea_input, GPS_CONFIG.enable_nmea_output
    );
}

async fn gps_rx_loop(mut rx: BufferedUartRx) -> ! {
    let mut read_buf = [0u8; 64];
    let mut gps = UbloxM10::new();

    loop {
        match rx.read(&mut read_buf).await {
            Ok(0) => {}
            Ok(count) => {
                for &byte in &read_buf[..count] {
                    handle_ubx_byte(byte, &mut gps);
                }
            }
            Err(error) => {
                warn!("sam-m10q gps uart rx failed: {:?}", error);
                Timer::after(GPS_RX_ERROR_BACKOFF).await;
            }
        }
    }
}

fn handle_ubx_byte(byte: u8, gps: &mut UbloxM10) {
    let event = match gps.push_byte(byte) {
        Ok(event) => event,
        Err(_) => {
            warn!("sam-m10q gps uart ubx parser error; resetting parser");
            gps.reset_parser();
            return;
        }
    };

    match event {
        Some(Event::Ack(ack)) if ack.acknowledged => {
            info!(
                "sam-m10q gps uart ack class=0x{=u8:02x} id=0x{=u8:02x}",
                ack.class_id, ack.message_id,
            );
        }
        Some(Event::Ack(ack)) => {
            warn!(
                "sam-m10q gps uart nack class=0x{=u8:02x} id=0x{=u8:02x}",
                ack.class_id, ack.message_id,
            );
        }
        Some(Event::NavPvt(nav_pvt)) => {
            let timestamp = MeasurementTimestamp::from_micros(Instant::now().as_micros());
            GPS_CHANNEL
                .immediate_publisher()
                .publish_immediate(nav_pvt.gps_fix_sample_stamped(timestamp));
        }
        Some(Event::OtherPacket) | None => {}
    }
}
