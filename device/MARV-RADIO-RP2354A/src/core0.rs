use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::uart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig};
use embassy_time::{Duration, Ticker, Timer};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::buses::HostUartBus;
use crate::config::{self, DeviceConfig};
use crate::interrupts::HostUartIrqs;
use crate::lora_demo;
use crate::resources::{DeviceResources, HostUartPins};
use crate::status_indicator;
use crate::status_led;

static HOST_UART_TX_BUFFER: StaticCell<[u8; config::HOST_UART_BUFFER_BYTES]> = StaticCell::new();
static HOST_UART_RX_BUFFER: StaticCell<[u8; config::HOST_UART_BUFFER_BYTES]> = StaticCell::new();

#[embassy_executor::task]
async fn host_uart_tx_task(mut tx: BufferedUartTx, heartbeat: &'static str, period_ms: u64) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(period_ms));

    loop {
        if tx.write_all(heartbeat.as_bytes()).await.is_err() {
            warn!("host uart heartbeat write failed");
        }
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn host_uart_rx_task(mut rx: BufferedUartRx) -> ! {
    let mut buf = [0u8; 64];

    loop {
        match rx.read(&mut buf).await {
            Ok(0) => {}
            Ok(count) => info!("host uart rx bytes={=usize}", count),
            Err(_) => warn!("host uart rx failed"),
        }
    }
}

async fn spawn_host_uart(
    spawner: &Spawner,
    bus: HostUartBus,
    pins: HostUartPins,
    config: config::HostUartConfig,
) {
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = config.baud;

    let tx_buffer = &mut HOST_UART_TX_BUFFER.init([0; config::HOST_UART_BUFFER_BYTES])[..];
    let rx_buffer = &mut HOST_UART_RX_BUFFER.init([0; config::HOST_UART_BUFFER_BYTES])[..];
    let uart = BufferedUart::new(
        bus.uart,
        pins.tx,
        pins.rx,
        HostUartIrqs,
        tx_buffer,
        rx_buffer,
        uart_config,
    );
    let (mut tx, rx) = uart.split();

    info!(
        "host uart ready: peer={=str} tx=GP{=u8} rx=GP{=u8} baud={=u32}",
        config.peer,
        crate::pinmap::HOST_UART_TX,
        crate::pinmap::HOST_UART_RX,
        config.baud
    );

    if tx.write_all(config.boot_message.as_bytes()).await.is_err() {
        warn!("host uart boot message write failed");
    }

    spawner
        .spawn(host_uart_rx_task(rx))
        .expect("host uart rx task spawn failed");
    spawner
        .spawn(host_uart_tx_task(
            tx,
            config.heartbeat_message,
            config.heartbeat_period_ms,
        ))
        .expect("host uart tx task spawn failed");
}

pub async fn run(spawner: Spawner, resources: DeviceResources) -> ! {
    let config = DeviceConfig::default();

    info!("{=str} booted", config.name);
    info!("firmware role: {:?}", config.role);
    status_led::spawn(
        &spawner,
        resources.pins.status.data,
        resources.buses.status_led,
    );
    status_indicator::spawn(&spawner);

    spawn_host_uart(
        &spawner,
        resources.buses.host_uart,
        resources.pins.host_uart,
        config.host_uart,
    )
    .await;

    lora_demo::spawn(
        &spawner,
        resources.buses.lora,
        resources.pins.lora,
        resources.watchdog,
        config.role,
    );

    loop {
        Timer::after_secs(60).await;
    }
}
