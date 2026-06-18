use common::messages::control::RgbLedCommand;
use common::protocol::hilink::{self, WirePayload};
use common::services::telemetry::{RadioReply, RadioTelemetryConfig, run_radio_telemetry_emitter};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::uart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Read;
use static_cell::StaticCell;

use crate::buses::RadioLinkUart;
use crate::channels::{
    AUX_IMU_CHANNEL, BAROMETER_CHANNEL, FLIGHT_PHASE_CHANNEL, FcBarometerSubscriber,
    FcFlightPhaseSubscriber, FcGpsSubscriber, FcImuSubscriber, FcMagnetometerSubscriber,
    GPS_CHANNEL, IMU_CHANNEL, MAGNETOMETER_CHANNEL, RGB_LED_COMMAND_CHANNEL,
};
use crate::interrupts::RadioLinkIrqs;
use crate::resources::RadioLinkPins;

const FRAME_BYTES: usize = hilink::encoded_frame_len(hilink::HilSensorFrame::WIRE_LEN);
const RX_LED_PULSE_COLOR: RgbLedCommand = RgbLedCommand::new(0, 24, 0);
const RADIO_REPLY_DEPTH: usize = 8;
const ACK_STATUS_ACCEPTED: u8 = 0;
const NACK_REASON_INVALID_PAYLOAD: u8 = 1;
const NACK_REASON_UNSUPPORTED: u8 = 4;

static RADIO_UART_TX_BUFFER: StaticCell<[u8; crate::config::RADIO_LINK_UART_BUFFER_BYTES]> =
    StaticCell::new();
static RADIO_UART_RX_BUFFER: StaticCell<[u8; crate::config::RADIO_LINK_UART_BUFFER_BYTES]> =
    StaticCell::new();
static RADIO_RX_LED_PULSE_CHANNEL: Channel<CriticalSectionRawMutex, (), 4> = Channel::new();
static RADIO_REPLY_CHANNEL: Channel<CriticalSectionRawMutex, RadioReply, RADIO_REPLY_DEPTH> =
    Channel::new();

type RadioReplySender = Sender<'static, CriticalSectionRawMutex, RadioReply, RADIO_REPLY_DEPTH>;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
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

fn try_queue_reply(sender: RadioReplySender, reply: RadioReply) {
    if sender.try_send(reply).is_err() {
        warn!("fc radio link reply queue full; dropping response");
    }
}

fn nack_reply(packet: &hilink::DecodedPacket<'_>, reason: u8) -> RadioReply {
    RadioReply::Nack(hilink::NackPayload {
        rejected_seq: packet.header.seq,
        rejected_msg_type: packet.header.msg_type,
        reason,
    })
}

fn validate_empty_payload<P: WirePayload>(
    packet: &hilink::DecodedPacket<'_>,
    sender: RadioReplySender,
) -> bool {
    if hilink::decode_payload::<P>(packet).is_ok() {
        true
    } else {
        try_queue_reply(sender, nack_reply(packet, NACK_REASON_INVALID_PAYLOAD));
        false
    }
}

fn nack_unsupported_command(packet: &hilink::DecodedPacket<'_>, sender: RadioReplySender) {
    try_queue_reply(sender, nack_reply(packet, NACK_REASON_UNSUPPORTED));
}

fn actuator_status_reply() -> RadioReply {
    RadioReply::ActuatorStatus(hilink::ActuatorStatusPayload {
        armed: 0,
        bench_enabled: 0,
        active_motor_mask: 0,
        mode: 0,
        commanded_dshot: [0; 4],
        last_command_age_ms: u16::MAX,
        bench_timeout_ms: 0,
        mixer_motor_order: hilink::mixer_motor_order::IDENTITY,
        flags: 0,
    })
}

fn handle_hilink_frame(frame: &[u8], sender: RadioReplySender) {
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
            if !validate_empty_payload::<hilink::PingPayload>(&packet, sender) {
                return;
            }

            info!(
                "fc radio link hilink ping rx seq={=u16} bytes={=usize}",
                packet.header.seq,
                frame.len()
            );
            note_radio_rx_for_led();
            try_queue_reply(
                sender,
                RadioReply::Pong {
                    peer_seq: packet.header.seq,
                },
            );
        }
        Ok(hilink::MsgType::Arm) => {
            if validate_empty_payload::<hilink::ArmPayload>(&packet, sender) {
                nack_unsupported_command(&packet, sender);
            }
        }
        Ok(hilink::MsgType::Disarm) => {
            if validate_empty_payload::<hilink::DisarmPayload>(&packet, sender) {
                nack_unsupported_command(&packet, sender);
            }
        }
        Ok(hilink::MsgType::Rtl) => {
            if validate_empty_payload::<hilink::RtlPayload>(&packet, sender) {
                nack_unsupported_command(&packet, sender);
            }
        }
        Ok(hilink::MsgType::MotorStop) => {
            if validate_empty_payload::<hilink::MotorStopPayload>(&packet, sender) {
                nack_unsupported_command(&packet, sender);
            }
        }
        Ok(hilink::MsgType::ActuatorStatusRequest) => {
            if !validate_empty_payload::<hilink::ActuatorStatusRequestPayload>(&packet, sender) {
                return;
            }
            try_queue_reply(sender, actuator_status_reply());
            try_queue_reply(
                sender,
                RadioReply::Ack(hilink::AckPayload {
                    acked_seq: packet.header.seq,
                    acked_msg_type: packet.header.msg_type,
                    status: ACK_STATUS_ACCEPTED,
                }),
            );
        }
        Ok(hilink::MsgType::HilSensorFrame) => {
            if hilink::decode_payload::<hilink::HilSensorFrame>(&packet).is_err() {
                try_queue_reply(sender, nack_reply(&packet, NACK_REASON_INVALID_PAYLOAD));
                return;
            }
            nack_unsupported_command(&packet, sender);
        }
        Ok(hilink::MsgType::Ack)
        | Ok(hilink::MsgType::Nack)
        | Ok(hilink::MsgType::Pong)
        | Ok(hilink::MsgType::Heartbeat)
        | Ok(hilink::MsgType::SystemState)
        | Ok(hilink::MsgType::TelemetrySnapshot)
        | Ok(hilink::MsgType::Gps)
        | Ok(hilink::MsgType::ActuatorStatus)
        | Ok(hilink::MsgType::HilResponseFrame) => {
            info!(
                "fc radio link ignoring inbound normal hilink message type={=u8} seq={=u16} bytes={=usize}",
                packet.header.msg_type,
                packet.header.seq,
                frame.len()
            );
        }
        Ok(message_type) => {
            warn!(
                "fc radio link rejecting unsupported normal hilink message type={=u8} seq={=u16} bytes={=usize}",
                message_type as u8,
                packet.header.seq,
                frame.len()
            );
            nack_unsupported_command(&packet, sender);
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
async fn radio_link_emitter_task(
    mut tx: BufferedUartTx,
    mut phase: FcFlightPhaseSubscriber,
    mut barometer: FcBarometerSubscriber,
    mut gps: FcGpsSubscriber,
    mut imu: FcImuSubscriber,
    mut aux_imu: FcImuSubscriber,
    mut magnetometer: FcMagnetometerSubscriber,
) -> ! {
    let replies = RADIO_REPLY_CHANNEL.receiver();
    run_radio_telemetry_emitter(
        &mut tx,
        &mut phase,
        &mut barometer,
        &mut gps,
        &mut imu,
        &mut aux_imu,
        &mut magnetometer,
        &replies,
        RadioTelemetryConfig::new(
            crate::config::RADIO_LINK_TELEMETRY_SNAPSHOT_PERIOD_MS,
            crate::config::RADIO_LINK_GPS_PERIOD_MS,
            crate::config::RADIO_LINK_STATUS_PERIOD_MS,
        )
        .with_raw_sensor_periods(
            crate::config::RADIO_LINK_IMU_PERIOD_MS,
            crate::config::RADIO_LINK_MAG_PERIOD_MS,
            crate::config::RADIO_LINK_BARO_PERIOD_MS,
        ),
        now_ms,
        // The rocket runs no active flight-phase estimator yet, so the wire `system_state` byte is
        // stubbed to the rocket `flight_state::PAD` code regardless of the (unused) `FlightPhase`.
        |_| hilink::flight_state::PAD,
        |error| warn!("fc radio link emit failed: {:?}", error),
    )
    .await
}

#[embassy_executor::task]
async fn radio_link_rx_task(mut rx: BufferedUartRx) -> ! {
    let mut read_buf = [0u8; 64];
    let mut frame = RadioHilinkFrame::new();
    let mut dropping_oversized = false;
    let sender = RADIO_REPLY_CHANNEL.sender();

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
                        warn!("fc radio link hilink frame exceeded normal endpoint buffer");
                        frame.clear();
                        dropping_oversized = true;
                        continue;
                    }

                    if byte == hilink::FRAME_DELIMITER {
                        if frame.len > 1 {
                            handle_hilink_frame(frame.as_slice(), sender);
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
        "fc radio link normal hilink endpoint ready: uart=1 tx=GP{=u8} rx=GP{=u8} baud={=u32}",
        crate::pinmap::FC_RADIO_TX,
        crate::pinmap::FC_RADIO_RX,
        crate::config::RADIO_LINK_UART_BAUD
    );

    spawner
        .spawn(radio_rx_led_pulse_task())
        .expect("fc radio rx led pulse task spawn failed");
    spawner
        .spawn(radio_link_emitter_task(
            tx,
            FLIGHT_PHASE_CHANNEL.subscriber().unwrap(),
            BAROMETER_CHANNEL.subscriber().unwrap(),
            GPS_CHANNEL.subscriber().unwrap(),
            IMU_CHANNEL.subscriber().unwrap(),
            AUX_IMU_CHANNEL.subscriber().unwrap(),
            MAGNETOMETER_CHANNEL.subscriber().unwrap(),
        ))
        .expect("fc radio link emitter task spawn failed");
    spawner
        .spawn(radio_link_rx_task(rx))
        .expect("fc radio link rx task spawn failed");
}
