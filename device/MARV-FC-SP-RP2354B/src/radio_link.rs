use common::messages::control::RgbLedCommand;
use common::messages::runtime::FlightPhase;
use common::messages::sensor::GpsFixSample;
use common::protocol::hilink::{self, WirePayload};
use common::utilities::units::{STANDARD_SEA_LEVEL_PRESSURE_PA, pressure_altitude_m};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::uart::{BufferedUart, BufferedUartRx, BufferedUartTx, Config as UartConfig};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;

use crate::buses::RadioLinkUart;
use crate::channels::{
    BAROMETER_CHANNEL, FLIGHT_PHASE_CHANNEL, GPS_CHANNEL, RGB_LED_COMMAND_CHANNEL,
};
use crate::interrupts::RadioLinkIrqs;
use crate::resources::RadioLinkPins;

const FRAME_BYTES: usize = hilink::encoded_frame_len(hilink::HilSensorFrame::WIRE_LEN);
const RX_LED_PULSE_COLOR: RgbLedCommand = RgbLedCommand::new(0, 24, 0);
const RADIO_OUTBOUND_DEPTH: usize = 8;
const ACK_STATUS_ACCEPTED: u8 = 0;
const NACK_REASON_INVALID_PAYLOAD: u8 = 1;
const NACK_REASON_UNSUPPORTED: u8 = 4;

static RADIO_UART_TX_BUFFER: StaticCell<[u8; crate::config::RADIO_LINK_UART_BUFFER_BYTES]> =
    StaticCell::new();
static RADIO_UART_RX_BUFFER: StaticCell<[u8; crate::config::RADIO_LINK_UART_BUFFER_BYTES]> =
    StaticCell::new();
static RADIO_RX_LED_PULSE_CHANNEL: Channel<CriticalSectionRawMutex, (), 4> = Channel::new();
static RADIO_OUTBOUND_CHANNEL: Channel<
    CriticalSectionRawMutex,
    RadioOutbound,
    RADIO_OUTBOUND_DEPTH,
> = Channel::new();

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
    Heartbeat {
        phase: FlightPhase,
    },
    SystemState {
        phase: FlightPhase,
    },
    TelemetrySnapshot(hilink::TelemetrySnapshotPayload),
    Gps(hilink::GpsPayload),
}

fn now_ms() -> u32 {
    Instant::now().as_millis().min(u32::MAX as u64) as u32
}

fn note_radio_rx_for_led() {
    let _ = RADIO_RX_LED_PULSE_CHANNEL.try_send(());
}

fn try_queue_outbound(sender: RadioOutboundSender, outbound: RadioOutbound) {
    if sender.try_send(outbound).is_err() {
        warn!("fc radio link outbound queue full; dropping response");
    }
}

type RadioOutboundSender =
    Sender<'static, CriticalSectionRawMutex, RadioOutbound, RADIO_OUTBOUND_DEPTH>;

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
            warn!("fc radio link failed to encode normal hilink payload");
            return None;
        }
    };

    if tx.write_all(&encoded[..len]).await.is_err() {
        warn!("fc radio link failed to write normal hilink payload");
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
                    "fc radio link hilink pong tx peer_seq={=u16} seq={=u16} bytes={=usize}",
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
            if let Some(len) = send_payload(tx, seq, &payload).await {
                info!(
                    "fc radio link hilink ack tx msg_type={=u8} peer_seq={=u16} bytes={=usize}",
                    acked_msg_type, acked_seq, len
                );
            }
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
            if let Some(len) = send_payload(tx, seq, &payload).await {
                info!(
                    "fc radio link hilink nack tx msg_type={=u8} peer_seq={=u16} reason={=u8} bytes={=usize}",
                    rejected_msg_type, rejected_seq, reason, len
                );
            }
        }
        RadioOutbound::ActuatorStatus => {
            let payload = hilink::ActuatorStatusPayload {
                armed: 0,
                bench_enabled: 0,
                active_motor_mask: 0,
                mode: 0,
                commanded_dshot: [0; 4],
                last_command_age_ms: u16::MAX,
                bench_timeout_ms: 0,
                mixer_motor_order: hilink::mixer_motor_order::IDENTITY,
                flags: 0,
            };
            let _ = send_payload(tx, seq, &payload).await;
        }
        RadioOutbound::Heartbeat { phase } => {
            let payload = hilink::HeartbeatPayload {
                stamp: hilink::SimStamp {
                    sim_tick: 0,
                    sim_time_us: u64::from(now_ms()) * 1_000,
                },
                system_state: phase.wire_code(),
                reserved0: [0; 3],
                flags: hilink::response_flags::ESTIMATOR_VALID,
            };
            let _ = send_payload(tx, seq, &payload).await;
        }
        RadioOutbound::SystemState { phase } => {
            let payload = hilink::SystemStatePayload {
                stamp: hilink::SimStamp {
                    sim_tick: 0,
                    sim_time_us: u64::from(now_ms()) * 1_000,
                },
                system_state: phase.wire_code(),
                reserved0: [0; 3],
                flags: hilink::response_flags::ESTIMATOR_VALID,
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

fn sim_stamp_now() -> hilink::SimStamp {
    hilink::SimStamp {
        sim_tick: 0,
        sim_time_us: u64::from(now_ms()) * 1_000,
    }
}

fn telemetry_snapshot_payload(
    phase: FlightPhase,
    altitude_m: f32,
) -> hilink::TelemetrySnapshotPayload {
    hilink::TelemetrySnapshotPayload {
        stamp: sim_stamp_now(),
        system_state: phase.wire_code(),
        reserved0: [0; 3],
        flags: hilink::response_flags::ESTIMATOR_VALID,
        position_ned_m: [0.0, 0.0, -altitude_m],
        velocity_ned_mps: [0.0, 0.0, 0.0],
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        battery_voltage_v: 0.0,
        rssi_dbm: 0,
        snr_db_x100: 0,
        loss_pct_x100: 0,
    }
}

fn gps_payload(sample: Option<GpsFixSample>) -> hilink::GpsPayload {
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
        stamp: sim_stamp_now(),
        lat_deg: sample.lat_deg,
        lon_deg: sample.lon_deg,
        alt_msl_m: sample.alt_m,
        vel_ned_mps: sample.vel_ned_mps,
        sats: sample.sats,
        fix_type: sample.fix_type,
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
            try_queue_outbound(
                sender,
                RadioOutbound::Pong {
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
            try_queue_outbound(sender, RadioOutbound::ActuatorStatus);
            try_queue_outbound(
                sender,
                RadioOutbound::Ack {
                    acked_seq: packet.header.seq,
                    acked_msg_type: packet.header.msg_type,
                },
            );
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
    mut phase_subscriber: crate::channels::FcFlightPhaseSubscriber,
    mut barometer_subscriber: crate::channels::FcBarometerSubscriber,
    mut gps_subscriber: crate::channels::FcGpsSubscriber,
) -> ! {
    let sender = RADIO_OUTBOUND_CHANNEL.sender();
    let mut phase = FlightPhase::Init;
    let mut altitude_m = 0.0f32;
    let mut gps = None;
    let mut last_status_ms =
        now_ms().wrapping_sub(crate::config::RADIO_LINK_STATUS_PERIOD_MS as u32);
    let mut last_telemetry_ms =
        now_ms().wrapping_sub(crate::config::RADIO_LINK_TELEMETRY_SNAPSHOT_PERIOD_MS as u32);
    let mut last_gps_ms = now_ms().wrapping_sub(crate::config::RADIO_LINK_GPS_PERIOD_MS as u32);
    let mut ticker = Ticker::every(Duration::from_millis(
        crate::config::RADIO_LINK_TELEMETRY_SNAPSHOT_PERIOD_MS,
    ));

    loop {
        while let Some(next_phase) = phase_subscriber.try_next_message_pure() {
            phase = next_phase;
        }
        while let Some(sample) = barometer_subscriber.try_next_message_pure() {
            if let Some(next_altitude_m) =
                pressure_altitude_m(sample.sample.pressure_pa, STANDARD_SEA_LEVEL_PRESSURE_PA)
            {
                altitude_m = next_altitude_m;
            }
        }
        while let Some(sample) = gps_subscriber.try_next_message_pure() {
            gps = Some(sample.sample);
        }

        let now = now_ms();
        if now.wrapping_sub(last_status_ms) >= crate::config::RADIO_LINK_STATUS_PERIOD_MS as u32 {
            last_status_ms = now;
            try_queue_outbound(sender, RadioOutbound::Heartbeat { phase });
            try_queue_outbound(sender, RadioOutbound::SystemState { phase });
        }
        if now.wrapping_sub(last_telemetry_ms)
            >= crate::config::RADIO_LINK_TELEMETRY_SNAPSHOT_PERIOD_MS as u32
        {
            last_telemetry_ms = now;
            try_queue_outbound(
                sender,
                RadioOutbound::TelemetrySnapshot(telemetry_snapshot_payload(phase, altitude_m)),
            );
        }
        if now.wrapping_sub(last_gps_ms) >= crate::config::RADIO_LINK_GPS_PERIOD_MS as u32 {
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
        .spawn(radio_link_tx_task(tx))
        .expect("fc radio link tx task spawn failed");
    spawner
        .spawn(radio_link_status_task(
            FLIGHT_PHASE_CHANNEL.subscriber().unwrap(),
            BAROMETER_CHANNEL.subscriber().unwrap(),
            GPS_CHANNEL.subscriber().unwrap(),
        ))
        .expect("fc radio link status task spawn failed");
    spawner
        .spawn(radio_link_rx_task(rx))
        .expect("fc radio link rx task spawn failed");
}
