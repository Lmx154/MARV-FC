use common::messages::sensor::{BarometerSampleStamped, GpsFixSampleStamped};
use common::protocol::hilink::{self, WirePayload};
use common::utilities::time::MeasurementTimestamp;
use common::utilities::units::{STANDARD_SEA_LEVEL_PRESSURE_PA, pressure_altitude_m};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, BufferedUartRx, BufferedUartTx};
use embassy_rp::{peripherals, uart::Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Ticker};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::buses::RadioLinkUart;
use crate::channels::{BAROMETER_CHANNEL, GPS_CHANNEL, TestBarometerSubscriber, TestGpsSubscriber};
use crate::resources::RadioLinkPins;

const RADIO_LINK_UART_BAUD: u32 = 115_200;
const RADIO_LINK_UART_BUFFER_BYTES: usize = 256;
const RADIO_LINK_STATUS_PERIOD_MS: u64 = 1_000;
const RADIO_LINK_TELEMETRY_SNAPSHOT_PERIOD_MS: u64 = 200;
const RADIO_LINK_GPS_PERIOD_MS: u64 = 1_000;
const FRAME_BYTES: usize = hilink::encoded_frame_len(hilink::HilSensorFrame::WIRE_LEN);
const RADIO_OUTBOUND_DEPTH: usize = 8;
const ACK_STATUS_ACCEPTED: u8 = 0;
const NACK_REASON_INVALID_PAYLOAD: u8 = 1;
const NACK_REASON_UNSUPPORTED: u8 = 4;

bind_interrupts!(struct RadioLinkIrqs {
    UART1_IRQ => BufferedInterruptHandler<peripherals::UART1>;
});

static RADIO_UART_TX_BUFFER: StaticCell<[u8; RADIO_LINK_UART_BUFFER_BYTES]> = StaticCell::new();
static RADIO_UART_RX_BUFFER: StaticCell<[u8; RADIO_LINK_UART_BUFFER_BYTES]> = StaticCell::new();
static RADIO_OUTBOUND_CHANNEL: Channel<
    CriticalSectionRawMutex,
    RadioOutbound,
    RADIO_OUTBOUND_DEPTH,
> = Channel::new();

type RadioOutboundSender =
    Sender<'static, CriticalSectionRawMutex, RadioOutbound, RADIO_OUTBOUND_DEPTH>;

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

#[derive(Clone, Copy, Debug)]
enum RadioOutbound {
    Pong {
        peer_seq: u16,
    },
    Ack {
        acked_seq: u16,
        acked_msg_type: u8,
    },
    Nack {
        rejected_seq: u16,
        rejected_msg_type: u8,
        reason: u8,
    },
    ActuatorStatus,
    Heartbeat,
    SystemState,
    TelemetrySnapshot(hilink::TelemetrySnapshotPayload),
    Gps(hilink::GpsPayload),
}

fn now_ms() -> u32 {
    Instant::now().as_millis().min(u32::MAX as u64) as u32
}

fn sim_stamp_now() -> hilink::SimStamp {
    hilink::SimStamp {
        sim_tick: 0,
        sim_time_us: u64::from(now_ms()) * 1_000,
    }
}

fn stamp_from_measurement(timestamp: MeasurementTimestamp) -> hilink::SimStamp {
    let micros = timestamp.as_micros();
    hilink::SimStamp {
        sim_tick: micros / 1_000,
        sim_time_us: micros,
    }
}

fn try_queue_outbound(sender: RadioOutboundSender, outbound: RadioOutbound) {
    if sender.try_send(outbound).is_err() {
        warn!("rp235x-test radio link outbound queue full; dropping response");
    }
}

async fn send_payload<P: WirePayload>(
    tx: &mut BufferedUartTx,
    seq: &mut u16,
    payload: &P,
) -> Option<usize> {
    let mut raw = [0u8; FRAME_BYTES];
    let mut encoded = [0u8; FRAME_BYTES];
    let len = match hilink::encode_packet(payload, *seq, now_ms(), &mut raw, &mut encoded) {
        Ok(len) => len,
        Err(_) => {
            warn!("rp235x-test radio link failed to encode hilink payload");
            return None;
        }
    };

    if tx.write_all(&encoded[..len]).await.is_err() {
        warn!("rp235x-test radio link failed to write hilink payload");
        return None;
    }

    *seq = seq.wrapping_add(1);
    Some(len)
}

async fn send_outbound(tx: &mut BufferedUartTx, seq: &mut u16, outbound: RadioOutbound) {
    match outbound {
        RadioOutbound::Pong { peer_seq } => {
            let reply_seq = *seq;
            if let Some(len) = send_payload(tx, seq, &hilink::PongPayload).await {
                info!(
                    "rp235x-test radio link pong tx peer_seq={=u16} seq={=u16} bytes={=usize}",
                    peer_seq, reply_seq, len
                );
            }
        }
        RadioOutbound::Ack {
            acked_seq,
            acked_msg_type,
        } => {
            let payload = hilink::AckPayload {
                acked_seq,
                acked_msg_type,
                status: ACK_STATUS_ACCEPTED,
            };
            let _ = send_payload(tx, seq, &payload).await;
        }
        RadioOutbound::Nack {
            rejected_seq,
            rejected_msg_type,
            reason,
        } => {
            let payload = hilink::NackPayload {
                rejected_seq,
                rejected_msg_type,
                reason,
            };
            let _ = send_payload(tx, seq, &payload).await;
        }
        RadioOutbound::ActuatorStatus => {
            let payload = crate::dshot::actuator_status();
            let _ = send_payload(tx, seq, &payload).await;
        }
        RadioOutbound::Heartbeat => {
            let payload = hilink::HeartbeatPayload {
                stamp: sim_stamp_now(),
                system_state: crate::protocol::system_state_code(),
                reserved0: [0; 3],
                flags: crate::protocol::response_flags(),
            };
            let _ = send_payload(tx, seq, &payload).await;
        }
        RadioOutbound::SystemState => {
            let payload = hilink::SystemStatePayload {
                stamp: sim_stamp_now(),
                system_state: crate::protocol::system_state_code(),
                reserved0: [0; 3],
                flags: crate::protocol::response_flags(),
                battery_voltage_v: 0.0,
            };
            let _ = send_payload(tx, seq, &payload).await;
        }
        RadioOutbound::TelemetrySnapshot(payload) => {
            let _ = send_payload(tx, seq, &payload).await;
        }
        RadioOutbound::Gps(payload) => {
            let _ = send_payload(tx, seq, &payload).await;
        }
    }
}

fn telemetry_snapshot_payload(altitude_m: f32) -> hilink::TelemetrySnapshotPayload {
    hilink::TelemetrySnapshotPayload {
        stamp: sim_stamp_now(),
        system_state: crate::protocol::system_state_code(),
        reserved0: [0; 3],
        flags: crate::protocol::response_flags(),
        position_ned_m: [0.0, 0.0, -altitude_m],
        velocity_ned_mps: [0.0, 0.0, 0.0],
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        battery_voltage_v: 0.0,
        rssi_dbm: 0,
        snr_db_x100: 0,
        loss_pct_x100: 0,
    }
}

fn gps_payload(sample: Option<GpsFixSampleStamped>) -> hilink::GpsPayload {
    let Some(sample) = sample else {
        return hilink::GpsPayload {
            stamp: sim_stamp_now(),
            lat_deg: 0.0,
            lon_deg: 0.0,
            alt_msl_m: 0.0,
            vel_ned_mps: [0.0, 0.0, 0.0],
            sats: 0,
            fix_type: 0,
            reserved0: [0; 2],
        };
    };

    hilink::GpsPayload {
        stamp: stamp_from_measurement(sample.timestamp),
        lat_deg: sample.sample.lat_deg,
        lon_deg: sample.sample.lon_deg,
        alt_msl_m: sample.sample.alt_m,
        vel_ned_mps: sample.sample.vel_ned_mps,
        sats: sample.sample.sats,
        fix_type: sample.sample.fix_type,
        reserved0: [0; 2],
    }
}

fn validate_empty_payload<P: WirePayload>(
    packet: &hilink::DecodedPacket<'_>,
    sender: RadioOutboundSender,
) -> bool {
    if hilink::decode_payload::<P>(packet).is_ok() {
        true
    } else {
        try_queue_outbound(
            sender,
            RadioOutbound::Nack {
                rejected_seq: packet.header.seq,
                rejected_msg_type: packet.header.msg_type,
                reason: NACK_REASON_INVALID_PAYLOAD,
            },
        );
        false
    }
}

fn nack_unsupported_command(packet: &hilink::DecodedPacket<'_>, sender: RadioOutboundSender) {
    try_queue_outbound(
        sender,
        RadioOutbound::Nack {
            rejected_seq: packet.header.seq,
            rejected_msg_type: packet.header.msg_type,
            reason: NACK_REASON_UNSUPPORTED,
        },
    );
}

fn handle_hilink_frame(frame: &[u8], sender: RadioOutboundSender) {
    let mut raw = [0u8; FRAME_BYTES];
    let packet = match hilink::decode_packet(frame, &mut raw) {
        Ok(packet) => packet,
        Err(_) => {
            warn!("rp235x-test radio link rejected malformed hilink frame");
            return;
        }
    };

    match packet.header.message_type() {
        Ok(hilink::MsgType::Ping) => {
            if !validate_empty_payload::<hilink::PingPayload>(&packet, sender) {
                return;
            }

            info!(
                "rp235x-test radio link ping rx seq={=u16} bytes={=usize}",
                packet.header.seq,
                frame.len()
            );
            try_queue_outbound(
                sender,
                RadioOutbound::Pong {
                    peer_seq: packet.header.seq,
                },
            );
        }
        Ok(hilink::MsgType::ActuatorStatusRequest) => {
            if !validate_empty_payload::<hilink::ActuatorStatusRequestPayload>(&packet, sender) {
                return;
            }
            try_queue_outbound(sender, RadioOutbound::ActuatorStatus);
            try_queue_outbound(
                sender,
                RadioOutbound::Ack {
                    acked_seq: packet.header.seq,
                    acked_msg_type: packet.header.msg_type,
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
        Ok(hilink::MsgType::HilSensorFrame) => {
            if hilink::decode_payload::<hilink::HilSensorFrame>(&packet).is_err() {
                try_queue_outbound(
                    sender,
                    RadioOutbound::Nack {
                        rejected_seq: packet.header.seq,
                        rejected_msg_type: packet.header.msg_type,
                        reason: NACK_REASON_INVALID_PAYLOAD,
                    },
                );
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
                "rp235x-test radio link ignoring inbound hilink message type={=u8} seq={=u16} bytes={=usize}",
                packet.header.msg_type,
                packet.header.seq,
                frame.len()
            );
        }
        Ok(message_type) => {
            warn!(
                "rp235x-test radio link rejecting unsupported hilink message type={=u8} seq={=u16} bytes={=usize}",
                message_type as u8,
                packet.header.seq,
                frame.len()
            );
            nack_unsupported_command(&packet, sender);
        }
        Err(_) => warn!("rp235x-test radio link ignoring unknown hilink message"),
    }
}

fn drain_latest_baro(subscriber: &mut TestBarometerSubscriber) -> Option<BarometerSampleStamped> {
    let mut latest = None;
    while let Some(message) = subscriber.try_next_message() {
        if let WaitResult::Message(sample) = message {
            latest = Some(sample);
        }
    }
    latest
}

fn drain_latest_gps(subscriber: &mut TestGpsSubscriber) -> Option<GpsFixSampleStamped> {
    let mut latest = None;
    while let Some(message) = subscriber.try_next_message() {
        if let WaitResult::Message(sample) = message {
            latest = Some(sample);
        }
    }
    latest
}

#[embassy_executor::task]
async fn radio_link_tx_task(mut tx: BufferedUartTx) -> ! {
    let receiver = RADIO_OUTBOUND_CHANNEL.receiver();
    let mut seq = 0u16;

    loop {
        let outbound = receiver.receive().await;
        send_outbound(&mut tx, &mut seq, outbound).await;
    }
}

#[embassy_executor::task]
async fn radio_link_status_task(
    mut barometer_subscriber: TestBarometerSubscriber,
    mut gps_subscriber: TestGpsSubscriber,
) -> ! {
    let sender = RADIO_OUTBOUND_CHANNEL.sender();
    let mut altitude_m = 0.0f32;
    let mut gps = None;
    let mut last_status_ms = now_ms().wrapping_sub(RADIO_LINK_STATUS_PERIOD_MS as u32);
    let mut last_telemetry_ms =
        now_ms().wrapping_sub(RADIO_LINK_TELEMETRY_SNAPSHOT_PERIOD_MS as u32);
    let mut last_gps_ms = now_ms().wrapping_sub(RADIO_LINK_GPS_PERIOD_MS as u32);
    let mut ticker = Ticker::every(Duration::from_millis(
        RADIO_LINK_TELEMETRY_SNAPSHOT_PERIOD_MS,
    ));

    loop {
        if let Some(sample) = drain_latest_baro(&mut barometer_subscriber) {
            if let Some(next_altitude_m) =
                pressure_altitude_m(sample.sample.pressure_pa, STANDARD_SEA_LEVEL_PRESSURE_PA)
            {
                altitude_m = next_altitude_m;
            }
        }
        if let Some(sample) = drain_latest_gps(&mut gps_subscriber) {
            gps = Some(sample);
        }

        let now = now_ms();
        if now.wrapping_sub(last_status_ms) >= RADIO_LINK_STATUS_PERIOD_MS as u32 {
            last_status_ms = now;
            try_queue_outbound(sender, RadioOutbound::Heartbeat);
            try_queue_outbound(sender, RadioOutbound::SystemState);
        }
        if now.wrapping_sub(last_telemetry_ms) >= RADIO_LINK_TELEMETRY_SNAPSHOT_PERIOD_MS as u32 {
            last_telemetry_ms = now;
            try_queue_outbound(
                sender,
                RadioOutbound::TelemetrySnapshot(telemetry_snapshot_payload(altitude_m)),
            );
        }
        if now.wrapping_sub(last_gps_ms) >= RADIO_LINK_GPS_PERIOD_MS as u32 {
            last_gps_ms = now;
            try_queue_outbound(sender, RadioOutbound::Gps(gps_payload(gps)));
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn radio_link_rx_task(mut rx: BufferedUartRx) -> ! {
    let mut read_buf = [0u8; 64];
    let mut frame = RadioHilinkFrame::new();
    let mut dropping_oversized = false;
    let sender = RADIO_OUTBOUND_CHANNEL.sender();

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
                        warn!("rp235x-test radio link hilink frame exceeded buffer");
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
            Err(_) => warn!("rp235x-test radio link uart rx failed"),
        }
    }
}

pub fn spawn(spawner: &Spawner, bus: RadioLinkUart, pins: RadioLinkPins) {
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = RADIO_LINK_UART_BAUD;

    let tx_buffer = &mut RADIO_UART_TX_BUFFER.init([0; RADIO_LINK_UART_BUFFER_BYTES])[..];
    let rx_buffer = &mut RADIO_UART_RX_BUFFER.init([0; RADIO_LINK_UART_BUFFER_BYTES])[..];
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
        "rp235x-test radio link ready: uart=1 tx=GP{=u8} rx=GP{=u8} baud={=u32}",
        crate::pinmap::FC_RADIO_TX,
        crate::pinmap::FC_RADIO_RX,
        RADIO_LINK_UART_BAUD
    );

    spawner
        .spawn(radio_link_tx_task(tx))
        .expect("rp235x-test radio link tx task spawn failed");
    spawner
        .spawn(radio_link_status_task(
            BAROMETER_CHANNEL.subscriber().unwrap(),
            GPS_CHANNEL.subscriber().unwrap(),
        ))
        .expect("rp235x-test radio link status task spawn failed");
    spawner
        .spawn(radio_link_rx_task(rx))
        .expect("rp235x-test radio link rx task spawn failed");
}
