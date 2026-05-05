use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::uart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig};
use embassy_time::Timer;
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use common::protocol::hilink;

use crate::buses::HostUartBus;
use crate::channels::{
    HilinkBridgeFrame, HilinkBridgeReceiver, LORA_TO_HOST_CHANNEL, host_to_lora_sender,
};
use crate::config::{self, DeviceConfig};
use crate::interrupts::HostUartIrqs;
use crate::lora_bridge;
use crate::radio_dialect::scheduler::classify_host_frame;
use crate::resources::{DeviceResources, HostUartPins};
use crate::status_indicator;
use crate::status_led;

static HOST_UART_TX_BUFFER: StaticCell<[u8; config::HOST_UART_BUFFER_BYTES]> = StaticCell::new();
static HOST_UART_RX_BUFFER: StaticCell<[u8; config::HOST_UART_BUFFER_BYTES]> = StaticCell::new();

#[embassy_executor::task]
async fn host_uart_tx_task(mut tx: BufferedUartTx, receiver: HilinkBridgeReceiver) -> ! {
    loop {
        let frame = receiver.receive().await;
        if tx.write_all(frame.as_slice()).await.is_err() {
            warn!("host uart protocol write failed");
        }
    }
}

#[embassy_executor::task]
async fn host_uart_rx_task(mut rx: BufferedUartRx) -> ! {
    let mut buf = [0u8; 64];
    let mut frame = HilinkBridgeFrame::new();
    let mut dropping_oversized = false;

    loop {
        match rx.read(&mut buf).await {
            Ok(0) => {}
            Ok(count) => {
                for &byte in &buf[..count] {
                    if dropping_oversized {
                        if byte == hilink::FRAME_DELIMITER {
                            dropping_oversized = false;
                            frame.clear();
                        }
                        continue;
                    }

                    if !frame.push(byte) {
                        warn!("host uart hilink frame exceeded lora payload capacity");
                        frame.clear();
                        dropping_oversized = true;
                        continue;
                    }

                    if byte == hilink::FRAME_DELIMITER {
                        if frame.len > 1 {
                            let priority = classify_host_frame(&frame).into_bridge_priority();
                            let sender = host_to_lora_sender(priority);
                            if sender.try_send(frame).is_err() {
                                warn!(
                                    "host-to-lora priority queue full priority={:?}; dropping frame",
                                    priority
                                );
                            } else {
                                info!(
                                    "host uart hilink frame queued priority={:?} bytes={=usize}",
                                    priority, frame.len
                                );
                            }
                        }
                        frame.clear();
                    }
                }
            }
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
    let (tx, rx) = uart.split();

    info!(
        "host uart bridge ready: peer={=str} tx=GP{=u8} rx=GP{=u8} baud={=u32}",
        config.peer,
        crate::pinmap::HOST_UART_TX,
        crate::pinmap::HOST_UART_RX,
        config.baud
    );

    spawner
        .spawn(host_uart_rx_task(rx))
        .expect("host uart rx task spawn failed");
    spawner
        .spawn(host_uart_tx_task(tx, LORA_TO_HOST_CHANNEL.receiver()))
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

    lora_bridge::spawn(
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
