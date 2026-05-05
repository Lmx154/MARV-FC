use common::comms::links::lora::frame::MAX_FRAME_PAYLOAD_LEN;
use common::messages::control::RgbLedCommand;
use common::protocol::hilink;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::uart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::buses::RadioLinkUart;
use crate::channels::RGB_LED_COMMAND_CHANNEL;
use crate::interrupts::RadioLinkIrqs;
use crate::resources::RadioLinkPins;

const FRAME_BYTES: usize = MAX_FRAME_PAYLOAD_LEN;
const RX_LED_PULSE_COLOR: RgbLedCommand = RgbLedCommand::new(0, 24, 0);

static RADIO_UART_TX_BUFFER: StaticCell<[u8; crate::config::RADIO_LINK_UART_BUFFER_BYTES]> =
    StaticCell::new();
static RADIO_UART_RX_BUFFER: StaticCell<[u8; crate::config::RADIO_LINK_UART_BUFFER_BYTES]> =
    StaticCell::new();
static RADIO_RX_LED_PULSE_CHANNEL: Channel<CriticalSectionRawMutex, (), 4> = Channel::new();

struct RadioHilinkFrame {
    len: usize,
    bytes: [u8; FRAME_BYTES],
}

impl RadioHilinkFrame {
    const fn new() -> Self {
        Self {
            len: 0,
            bytes: [0; FRAME_BYTES],
        }
    }

    fn clear(&mut self) {
        self.len = 0;
    }

    fn push(&mut self, byte: u8) -> bool {
        if self.len >= self.bytes.len() {
            return false;
        }

        self.bytes[self.len] = byte;
        self.len += 1;
        true
    }

    fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

fn now_ms() -> u32 {
    Instant::now().as_millis().min(u32::MAX as u64) as u32
}

fn note_radio_rx_for_led() {
    let _ = RADIO_RX_LED_PULSE_CHANNEL.try_send(());
}

async fn send_pong(tx: &mut BufferedUartTx, seq: &mut u16, peer_seq: u16) {
    let mut raw = [0u8; FRAME_BYTES];
    let mut encoded = [0u8; FRAME_BYTES];
    let reply_seq = *seq;
    let len = match hilink::encode_packet(
        &hilink::PongPayload,
        reply_seq,
        now_ms(),
        &mut raw,
        &mut encoded,
    ) {
        Ok(len) => len,
        Err(_) => {
            warn!("fc radio link failed to encode hilink pong");
            return;
        }
    };

    if tx.write_all(&encoded[..len]).await.is_err() {
        warn!("fc radio link failed to write hilink pong");
        return;
    }

    *seq = seq.wrapping_add(1);
    info!(
        "fc radio link hilink pong tx peer_seq={=u16} seq={=u16} bytes={=usize}",
        peer_seq, reply_seq, len
    );
}

async fn handle_hilink_frame(frame: &[u8], tx: &mut BufferedUartTx, seq: &mut u16) {
    let mut raw = [0u8; FRAME_BYTES];
    let packet = match hilink::decode_packet(frame, &mut raw) {
        Ok(packet) => packet,
        Err(_) => {
            warn!("fc radio link rejected malformed hilink frame");
            return;
        }
    };

    match packet.header.message_type() {
        Ok(hilink::MsgType::Ping) => {
            if hilink::decode_payload::<hilink::PingPayload>(&packet).is_err() {
                warn!("fc radio link rejected invalid hilink ping payload");
                return;
            }

            info!(
                "fc radio link hilink ping rx seq={=u16} bytes={=usize}",
                packet.header.seq,
                frame.len()
            );
            note_radio_rx_for_led();
            send_pong(tx, seq, packet.header.seq).await;
        }
        Ok(message_type) => {
            info!(
                "fc radio link ignoring hilink message type={=u8} seq={=u16} bytes={=usize}",
                message_type as u8,
                packet.header.seq,
                frame.len()
            );
        }
        Err(_) => warn!("fc radio link ignoring unknown hilink message"),
    }
}

#[embassy_executor::task]
async fn radio_rx_led_pulse_task() -> ! {
    let receiver = RADIO_RX_LED_PULSE_CHANNEL.receiver();
    let led = RGB_LED_COMMAND_CHANNEL.sender();

    loop {
        receiver.receive().await;
        let _ = led.try_send(RX_LED_PULSE_COLOR);
        Timer::after(Duration::from_millis(
            crate::config::RADIO_LINK_RX_LED_PULSE_MS,
        ))
        .await;
        let _ = led.try_send(RgbLedCommand::OFF);

        while receiver.try_receive().is_ok() {}
    }
}

#[embassy_executor::task]
async fn radio_link_task(mut tx: BufferedUartTx, mut rx: BufferedUartRx) -> ! {
    let mut read_buf = [0u8; 64];
    let mut frame = RadioHilinkFrame::new();
    let mut dropping_oversized = false;
    let mut seq = 0u16;

    loop {
        match rx.read(&mut read_buf).await {
            Ok(0) => {}
            Ok(count) => {
                for &byte in &read_buf[..count] {
                    if dropping_oversized {
                        if byte == hilink::FRAME_DELIMITER {
                            dropping_oversized = false;
                            frame.clear();
                        }
                        continue;
                    }

                    if !frame.push(byte) {
                        warn!("fc radio link hilink frame exceeded buffer");
                        frame.clear();
                        dropping_oversized = true;
                        continue;
                    }

                    if byte == hilink::FRAME_DELIMITER {
                        if frame.len > 1 {
                            handle_hilink_frame(frame.as_slice(), &mut tx, &mut seq).await;
                        }
                        frame.clear();
                    }
                }
            }
            Err(_) => warn!("fc radio link uart rx failed"),
        }
    }
}

pub fn spawn(spawner: &Spawner, bus: RadioLinkUart, pins: RadioLinkPins) {
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = crate::config::RADIO_LINK_UART_BAUD;

    let tx_buffer =
        &mut RADIO_UART_TX_BUFFER.init([0; crate::config::RADIO_LINK_UART_BUFFER_BYTES])[..];
    let rx_buffer =
        &mut RADIO_UART_RX_BUFFER.init([0; crate::config::RADIO_LINK_UART_BUFFER_BYTES])[..];
    let uart = BufferedUart::new(
        bus.uart,
        pins.tx,
        pins.rx,
        RadioLinkIrqs,
        tx_buffer,
        rx_buffer,
        uart_config,
    );
    let (tx, rx) = uart.split();

    info!(
        "fc radio link hilink endpoint ready: uart=1 tx=GP{=u8} rx=GP{=u8} baud={=u32}",
        crate::pinmap::FC_RADIO_TX,
        crate::pinmap::FC_RADIO_RX,
        crate::config::RADIO_LINK_UART_BAUD
    );

    spawner
        .spawn(radio_rx_led_pulse_task())
        .expect("fc radio rx led pulse task spawn failed");
    spawner
        .spawn(radio_link_task(tx, rx))
        .expect("fc radio link task spawn failed");
}
