use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use common::messages::sensor::{
    BarometerSampleStamped, GpsFixSampleStamped, ImuSampleStamped, MagnetometerSampleStamped,
};
use common::protocol::hilink::{
    self, AckPayload, ActuatorStatusPayload, ActuatorStatusRequestPayload, ArmPayload, BaroPayload,
    BenchDisablePayload, BenchEnablePayload, CvWaypointPayload, DisarmPayload, DshotCommandPayload,
    GlobalWaypointPayload, GpsPayload, HeartbeatPayload, HilReadyPayload, HilResponseFrame,
    HilSensorFrame, ImuPayload, MagPayload, MissionWaypointPayload, MotorStopPayload,
    MotorSweepPayload, MotorTestPayload, MsgType, NackPayload, PingPayload, PongPayload,
    RtlPayload, SimStamp, TofWaypointPayload, WirePayload, response_flags,
};
use common::utilities::time::MeasurementTimestamp;
use common::utilities::units::{STANDARD_SEA_LEVEL_PRESSURE_PA, pressure_altitude_m};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::peripherals::USB;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Ticker};
use heapless::Vec;
use rp235x_base::usb_cdc::{
    EndpointError, RpUsbReceiver, RpUsbSender, USB_PACKET_SIZE, UsbCdcConfig, spawn_cdc_acm,
    write_packet_chunks,
};

use crate::channels::{
    BAROMETER_CHANNEL, GPS_CHANNEL, IMU_CHANNEL, MAGNETOMETER_CHANNEL, TestBarometerSubscriber,
    TestGpsSubscriber, TestImuSubscriber, TestMagnetometerSubscriber,
};
use crate::dshot;

const RX_FRAME_CAPACITY: usize = hilink::encoded_frame_len(HilSensorFrameMaxPayload::WIRE_LEN);
const RAW_FRAME_CAPACITY: usize = hilink::raw_frame_len(HilSensorFrameMaxPayload::WIRE_LEN);
const TX_FRAME_CAPACITY: usize = hilink::encoded_frame_len(HilResponseFrameMaxPayload::WIRE_LEN);
const TX_RAW_CAPACITY: usize = hilink::raw_frame_len(HilResponseFrameMaxPayload::WIRE_LEN);
const SENSOR_TELEMETRY_PERIOD: Duration = Duration::from_millis(20);
const STATE_IDLE: u8 = 1;
const STATE_ARMED: u8 = 2;
const ACK_STATUS_ACCEPTED: u8 = 0;
const NACK_REASON_INVALID_PAYLOAD: u8 = 1;
const NACK_REASON_REJECTED: u8 = 2;

type RxFrame = Vec<u8, RX_FRAME_CAPACITY>;
type TxFrame = Vec<u8, TX_FRAME_CAPACITY>;

struct HilSensorFrameMaxPayload;
struct HilResponseFrameMaxPayload;

impl HilSensorFrameMaxPayload {
    const WIRE_LEN: usize = hilink::HilSensorFrame::WIRE_LEN;
}

impl HilResponseFrameMaxPayload {
    const WIRE_LEN: usize = hilink::HilResponseFrame::WIRE_LEN;
}

#[derive(Clone, Copy, Debug)]
enum OutboundMessage {
    HilReady,
    Heartbeat,
    Pong,
    Ack(AckPayload),
    Nack(NackPayload),
    HilResponse(HilResponseFrame),
    Imu(ImuPayload),
    Mag(MagPayload),
    Baro(BaroPayload),
    Gps(GpsPayload),
    ActuatorStatus(ActuatorStatusPayload),
}

static TX_CHANNEL: Channel<CriticalSectionRawMutex, OutboundMessage, 16> = Channel::new();
static SYSTEM_STATE: AtomicU8 = AtomicU8::new(STATE_IDLE);
static RESPONSE_FLAGS: AtomicU32 = AtomicU32::new(response_flags::MOTORS_VALID);
static BENCH_REQUESTED: AtomicU8 = AtomicU8::new(0);

pub fn spawn(spawner: &Spawner, usb: Peri<'static, USB>) {
    let usb = spawn_cdc_acm(
        spawner,
        usb,
        UsbCdcConfig::new(
            0x1209,
            0x0002,
            "MARV",
            "RP235X Protocol Test",
            "MARV-RP235X-TEST",
        ),
    );

    spawner.spawn(usb_rx_task(usb.receiver)).unwrap();
    spawner.spawn(usb_tx_task(usb.sender)).unwrap();
    spawner
        .spawn(sensor_telemetry_task(
            IMU_CHANNEL.subscriber().unwrap(),
            BAROMETER_CHANNEL.subscriber().unwrap(),
            MAGNETOMETER_CHANNEL.subscriber().unwrap(),
            GPS_CHANNEL.subscriber().unwrap(),
        ))
        .unwrap();
}

pub fn is_armed() -> bool {
    SYSTEM_STATE.load(Ordering::Relaxed) == STATE_ARMED
}

#[embassy_executor::task]
async fn usb_rx_task(mut receiver: RpUsbReceiver) -> ! {
    let mut packet = [0u8; USB_PACKET_SIZE as usize];
    let mut frame = RxFrame::new();
    let mut raw_scratch = [0u8; RAW_FRAME_CAPACITY];

    loop {
        receiver.wait_connection().await;
        info!("rp235x-test usb cdc rx connected");
        frame.clear();

        loop {
            match receiver.read_packet(&mut packet).await {
                Ok(0) => {}
                Ok(len) => {
                    ingest_usb_bytes(&packet[..len], &mut frame, &mut raw_scratch);
                }
                Err(_) => {
                    warn!("rp235x-test usb cdc rx disconnected");
                    break;
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn usb_tx_task(mut sender: RpUsbSender) -> ! {
    let mut seq = 0u16;
    let mut raw_scratch = [0u8; TX_RAW_CAPACITY];
    let mut encoded = TxFrame::new();

    loop {
        sender.wait_connection().await;
        info!("rp235x-test usb cdc tx connected");

        if send_outbound(
            &mut sender,
            OutboundMessage::HilReady,
            &mut seq,
            &mut raw_scratch,
            &mut encoded,
        )
        .await
        .is_err()
        {
            warn!("rp235x-test failed to send hil ready");
            continue;
        }

        let mut heartbeat = Ticker::every(Duration::from_secs(1));

        loop {
            let outbound = match select(TX_CHANNEL.receive(), heartbeat.next()).await {
                Either::First(message) => message,
                Either::Second(()) => OutboundMessage::Heartbeat,
            };

            if send_outbound(
                &mut sender,
                outbound,
                &mut seq,
                &mut raw_scratch,
                &mut encoded,
            )
            .await
            .is_err()
            {
                warn!("rp235x-test protocol write failed");
                break;
            }

            let mut write_failed = false;
            while let Ok(outbound) = TX_CHANNEL.try_receive() {
                if send_outbound(
                    &mut sender,
                    outbound,
                    &mut seq,
                    &mut raw_scratch,
                    &mut encoded,
                )
                .await
                .is_err()
                {
                    warn!("rp235x-test protocol write failed");
                    write_failed = true;
                    break;
                }
            }

            if write_failed {
                break;
            }
        }
    }
}

fn ingest_usb_bytes(bytes: &[u8], frame: &mut RxFrame, raw_scratch: &mut [u8; RAW_FRAME_CAPACITY]) {
    for &byte in bytes {
        if frame.push(byte).is_err() {
            warn!("rp235x-test rx frame buffer full");
            frame.clear();
            continue;
        }

        if byte == hilink::FRAME_DELIMITER {
            handle_protocol_frame(frame.as_slice(), raw_scratch);
            frame.clear();
        }
    }
}

fn handle_protocol_frame(frame: &[u8], raw_scratch: &mut [u8; RAW_FRAME_CAPACITY]) {
    let packet = match hilink::decode_packet(frame, raw_scratch) {
        Ok(packet) => packet,
        Err(_) => {
            warn!("rp235x-test rejected malformed hilink frame");
            return;
        }
    };

    match packet.header.message_type() {
        Ok(MsgType::Ping) => {
            if hilink::decode_payload::<PingPayload>(&packet).is_ok() {
                enqueue_outbound(OutboundMessage::Pong);
            } else {
                warn!("rp235x-test rejected invalid ping payload");
                enqueue_outbound(invalid_payload_nack(&packet));
            }
        }
        Ok(MsgType::Arm) => handle_arm_packet(&packet),
        Ok(MsgType::Disarm) => handle_disarm_packet(&packet),
        Ok(MsgType::ControlWaypoint) => handle_control_waypoint_packet(&packet),
        Ok(MsgType::CvWaypoint) => handle_cv_waypoint_packet(&packet),
        Ok(MsgType::TofWaypoint) => handle_tof_waypoint_packet(&packet),
        Ok(MsgType::MissionWaypoint) => handle_mission_waypoint_packet(&packet),
        Ok(MsgType::Rtl) => handle_rtl_packet(&packet),
        Ok(MsgType::BenchEnable) => handle_bench_enable_packet(&packet),
        Ok(MsgType::BenchDisable) => handle_bench_disable_packet(&packet),
        Ok(MsgType::MotorTest) => handle_motor_test_packet(&packet),
        Ok(MsgType::MotorSweep) => handle_motor_sweep_packet(&packet),
        Ok(MsgType::MotorStop) => handle_motor_stop_packet(&packet),
        Ok(MsgType::DshotCommand) => handle_dshot_command_packet(&packet),
        Ok(MsgType::ActuatorStatusRequest) => handle_actuator_status_request_packet(&packet),
        Ok(MsgType::HilSensorFrame) => handle_hil_sensor_frame(&packet),
        Ok(_) => {
            warn!("rp235x-test ignoring unsupported hilink message");
        }
        Err(_) => {
            warn!("rp235x-test ignoring unknown hilink message");
        }
    }
}

fn handle_arm_packet(packet: &hilink::DecodedPacket<'_>) {
    if hilink::decode_payload::<ArmPayload>(packet).is_err() {
        warn!("rp235x-test rejected invalid arm payload");
        enqueue_outbound(invalid_payload_nack(packet));
        return;
    }

    SYSTEM_STATE.store(STATE_ARMED, Ordering::Relaxed);
    RESPONSE_FLAGS.store(
        response_flags::ARMED | response_flags::MOTORS_VALID,
        Ordering::Relaxed,
    );
    enqueue_outbound(ack_for(packet));
}

fn handle_disarm_packet(packet: &hilink::DecodedPacket<'_>) {
    if hilink::decode_payload::<DisarmPayload>(packet).is_err() {
        warn!("rp235x-test rejected invalid disarm payload");
        enqueue_outbound(invalid_payload_nack(packet));
        return;
    }

    SYSTEM_STATE.store(STATE_IDLE, Ordering::Relaxed);
    RESPONSE_FLAGS.store(response_flags::MOTORS_VALID, Ordering::Relaxed);
    BENCH_REQUESTED.store(0, Ordering::Relaxed);
    let _ = dshot::submit_motor_stop();
    let _ = dshot::submit_bench_disable();
    enqueue_outbound(ack_for(packet));
}

fn handle_control_waypoint_packet(packet: &hilink::DecodedPacket<'_>) {
    match hilink::decode_payload::<GlobalWaypointPayload>(packet) {
        Ok(_) => {}
        Err(_) => {
            warn!("rp235x-test rejected invalid control waypoint payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    }

    info!("rp235x-test accepted control waypoint");
    enqueue_outbound(ack_for(packet));
}

fn handle_cv_waypoint_packet(packet: &hilink::DecodedPacket<'_>) {
    match hilink::decode_payload::<CvWaypointPayload>(packet) {
        Ok(_) => {}
        Err(_) => {
            warn!("rp235x-test rejected invalid cv waypoint payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    }

    info!("rp235x-test accepted cv waypoint");
    enqueue_outbound(ack_for(packet));
}

fn handle_tof_waypoint_packet(packet: &hilink::DecodedPacket<'_>) {
    match hilink::decode_payload::<TofWaypointPayload>(packet) {
        Ok(_) => {}
        Err(_) => {
            warn!("rp235x-test rejected invalid tof waypoint payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    }

    info!("rp235x-test accepted tof waypoint");
    enqueue_outbound(ack_for(packet));
}

fn handle_mission_waypoint_packet(packet: &hilink::DecodedPacket<'_>) {
    match hilink::decode_payload::<MissionWaypointPayload>(packet) {
        Ok(_) => {}
        Err(_) => {
            warn!("rp235x-test rejected invalid mission waypoint payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    }

    info!("rp235x-test accepted mission waypoint");
    enqueue_outbound(ack_for(packet));
}

fn handle_rtl_packet(packet: &hilink::DecodedPacket<'_>) {
    if hilink::decode_payload::<RtlPayload>(packet).is_err() {
        warn!("rp235x-test rejected invalid rtl payload");
        enqueue_outbound(invalid_payload_nack(packet));
        return;
    }

    info!("rp235x-test accepted rtl command");
    enqueue_outbound(ack_for(packet));
}

fn handle_bench_enable_packet(packet: &hilink::DecodedPacket<'_>) {
    let payload = match hilink::decode_payload::<BenchEnablePayload>(packet) {
        Ok(payload) => payload,
        Err(_) => {
            warn!("rp235x-test rejected invalid bench enable payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    };

    if !is_armed() || !dshot::submit_bench_enable(payload) {
        warn!("rp235x-test rejected bench enable");
        enqueue_outbound(rejected_nack(packet));
        return;
    }

    BENCH_REQUESTED.store(1, Ordering::Relaxed);
    enqueue_outbound(ack_for(packet));
}

fn handle_bench_disable_packet(packet: &hilink::DecodedPacket<'_>) {
    if hilink::decode_payload::<BenchDisablePayload>(packet).is_err() {
        warn!("rp235x-test rejected invalid bench disable payload");
        enqueue_outbound(invalid_payload_nack(packet));
        return;
    }

    if dshot::submit_bench_disable() {
        BENCH_REQUESTED.store(0, Ordering::Relaxed);
        enqueue_outbound(ack_for(packet));
    } else {
        enqueue_outbound(rejected_nack(packet));
    }
}

fn handle_motor_test_packet(packet: &hilink::DecodedPacket<'_>) {
    let payload = match hilink::decode_payload::<MotorTestPayload>(packet) {
        Ok(payload) => payload,
        Err(_) => {
            warn!("rp235x-test rejected invalid motor test payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    };

    if !is_armed()
        || BENCH_REQUESTED.load(Ordering::Relaxed) == 0
        || !dshot::submit_motor_test(payload)
    {
        warn!("rp235x-test rejected motor test");
        enqueue_outbound(rejected_nack(packet));
        return;
    }

    enqueue_outbound(ack_for(packet));
}

fn handle_motor_sweep_packet(packet: &hilink::DecodedPacket<'_>) {
    let payload = match hilink::decode_payload::<MotorSweepPayload>(packet) {
        Ok(payload) => payload,
        Err(_) => {
            warn!("rp235x-test rejected invalid motor sweep payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    };

    if !is_armed()
        || BENCH_REQUESTED.load(Ordering::Relaxed) == 0
        || !dshot::submit_motor_sweep(payload)
    {
        warn!("rp235x-test rejected motor sweep");
        enqueue_outbound(rejected_nack(packet));
        return;
    }

    enqueue_outbound(ack_for(packet));
}

fn handle_motor_stop_packet(packet: &hilink::DecodedPacket<'_>) {
    if hilink::decode_payload::<MotorStopPayload>(packet).is_err() {
        warn!("rp235x-test rejected invalid motor stop payload");
        enqueue_outbound(invalid_payload_nack(packet));
        return;
    }

    if dshot::submit_motor_stop() {
        enqueue_outbound(ack_for(packet));
    } else {
        enqueue_outbound(rejected_nack(packet));
    }
}

fn handle_dshot_command_packet(packet: &hilink::DecodedPacket<'_>) {
    let payload = match hilink::decode_payload::<DshotCommandPayload>(packet) {
        Ok(payload) => payload,
        Err(_) => {
            warn!("rp235x-test rejected invalid dshot command payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    };

    if !is_armed()
        || BENCH_REQUESTED.load(Ordering::Relaxed) == 0
        || !dshot::submit_dshot_command(payload)
    {
        warn!("rp235x-test rejected dshot special command");
        enqueue_outbound(rejected_nack(packet));
        return;
    }

    enqueue_outbound(ack_for(packet));
}

fn handle_actuator_status_request_packet(packet: &hilink::DecodedPacket<'_>) {
    if hilink::decode_payload::<ActuatorStatusRequestPayload>(packet).is_err() {
        warn!("rp235x-test rejected invalid actuator status request payload");
        enqueue_outbound(invalid_payload_nack(packet));
        return;
    }

    enqueue_outbound(OutboundMessage::ActuatorStatus(dshot::actuator_status()));
}

fn handle_hil_sensor_frame(packet: &hilink::DecodedPacket<'_>) {
    let sensors = match hilink::decode_payload::<HilSensorFrame>(packet) {
        Ok(sensors) => sensors,
        Err(_) => {
            warn!("rp235x-test rejected invalid hil sensor frame payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    };

    let system_state = SYSTEM_STATE.load(Ordering::Relaxed);
    let armed = system_state == STATE_ARMED;
    let flags = if armed {
        response_flags::ARMED | response_flags::MOTORS_VALID
    } else {
        response_flags::MOTORS_VALID
    };

    let response = HilResponseFrame {
        stamp: sensors.stamp,
        system_state,
        reserved0: [0; 3],
        flags,
        position_ned_m: [0.0, 0.0, 0.0],
        velocity_ned_mps: [0.0, 0.0, 0.0],
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        motor_cmd: if armed {
            [12_000, 12_000, 12_000, 12_000]
        } else {
            [0, 0, 0, 0]
        },
    };

    enqueue_outbound(OutboundMessage::HilResponse(response));
}

fn ack_for(packet: &hilink::DecodedPacket<'_>) -> OutboundMessage {
    OutboundMessage::Ack(AckPayload {
        acked_seq: packet.header.seq,
        acked_msg_type: packet.header.msg_type,
        status: ACK_STATUS_ACCEPTED,
    })
}

fn invalid_payload_nack(packet: &hilink::DecodedPacket<'_>) -> OutboundMessage {
    OutboundMessage::Nack(NackPayload {
        rejected_seq: packet.header.seq,
        rejected_msg_type: packet.header.msg_type,
        reason: NACK_REASON_INVALID_PAYLOAD,
    })
}

fn rejected_nack(packet: &hilink::DecodedPacket<'_>) -> OutboundMessage {
    OutboundMessage::Nack(NackPayload {
        rejected_seq: packet.header.seq,
        rejected_msg_type: packet.header.msg_type,
        reason: NACK_REASON_REJECTED,
    })
}

fn enqueue_outbound(message: OutboundMessage) {
    if TX_CHANNEL.try_send(message).is_err() {
        warn!("rp235x-test tx queue full");
    }
}

fn enqueue_telemetry_outbound(message: OutboundMessage) {
    let _ = TX_CHANNEL.try_send(message);
}

fn stamp_from_measurement(timestamp: MeasurementTimestamp) -> SimStamp {
    let micros = timestamp.as_micros();
    SimStamp {
        sim_tick: micros / 1_000,
        sim_time_us: micros,
    }
}

fn imu_payload(sample: ImuSampleStamped) -> ImuPayload {
    ImuPayload {
        stamp: stamp_from_measurement(sample.timestamp),
        accel_mps2: sample.sample.accel_mps2,
        gyro_rps: sample.sample.gyro_rad_s,
    }
}

fn mag_payload(sample: MagnetometerSampleStamped) -> MagPayload {
    MagPayload {
        stamp: stamp_from_measurement(sample.timestamp),
        field_ut: sample.sample.field_ut,
    }
}

fn baro_payload(sample: BarometerSampleStamped) -> BaroPayload {
    BaroPayload {
        stamp: stamp_from_measurement(sample.timestamp),
        pressure_pa: sample.sample.pressure_pa,
        altitude_m: pressure_altitude_m(sample.sample.pressure_pa, STANDARD_SEA_LEVEL_PRESSURE_PA)
            .unwrap_or(0.0),
        temperature_c: sample.sample.temperature_c,
    }
}

fn gps_payload(sample: GpsFixSampleStamped) -> GpsPayload {
    GpsPayload {
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

fn drain_latest_imu(subscriber: &mut TestImuSubscriber) -> Option<ImuSampleStamped> {
    let mut latest = None;
    while let Some(message) = subscriber.try_next_message() {
        if let WaitResult::Message(sample) = message {
            latest = Some(sample);
        }
    }
    latest
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

fn drain_latest_mag(
    subscriber: &mut TestMagnetometerSubscriber,
) -> Option<MagnetometerSampleStamped> {
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
async fn sensor_telemetry_task(
    mut imu_subscriber: TestImuSubscriber,
    mut barometer_subscriber: TestBarometerSubscriber,
    mut magnetometer_subscriber: TestMagnetometerSubscriber,
    mut gps_subscriber: TestGpsSubscriber,
) -> ! {
    let mut ticker = Ticker::every(SENSOR_TELEMETRY_PERIOD);

    loop {
        ticker.next().await;

        if let Some(sample) = drain_latest_imu(&mut imu_subscriber) {
            enqueue_telemetry_outbound(OutboundMessage::Imu(imu_payload(sample)));
        }

        if let Some(sample) = drain_latest_baro(&mut barometer_subscriber) {
            enqueue_telemetry_outbound(OutboundMessage::Baro(baro_payload(sample)));
        }

        if let Some(sample) = drain_latest_mag(&mut magnetometer_subscriber) {
            enqueue_telemetry_outbound(OutboundMessage::Mag(mag_payload(sample)));
        }

        if let Some(sample) = drain_latest_gps(&mut gps_subscriber) {
            enqueue_telemetry_outbound(OutboundMessage::Gps(gps_payload(sample)));
        }
    }
}

async fn send_outbound(
    sender: &mut RpUsbSender,
    message: OutboundMessage,
    seq: &mut u16,
    raw_scratch: &mut [u8; TX_RAW_CAPACITY],
    encoded: &mut TxFrame,
) -> Result<(), EndpointError> {
    encoded.clear();
    let mut encoded_scratch = [0u8; TX_FRAME_CAPACITY];
    let len = match encode_outbound(
        message,
        *seq,
        send_time_ms(),
        raw_scratch,
        &mut encoded_scratch,
    ) {
        Ok(len) => len,
        Err(_) => {
            warn!("rp235x-test failed to encode outbound hilink frame");
            return Ok(());
        }
    };

    if encoded.extend_from_slice(&encoded_scratch[..len]).is_err() {
        warn!("rp235x-test encoded tx frame exceeded buffer");
        return Ok(());
    }

    *seq = seq.wrapping_add(1);
    write_packet_chunks(sender, encoded.as_slice()).await
}

fn encode_outbound(
    message: OutboundMessage,
    seq: u16,
    send_time_ms: u32,
    raw_scratch: &mut [u8; TX_RAW_CAPACITY],
    encoded: &mut [u8; TX_FRAME_CAPACITY],
) -> hilink::Result<usize> {
    match message {
        OutboundMessage::HilReady => {
            hilink::encode_packet(&HilReadyPayload, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::Heartbeat => {
            let heartbeat = HeartbeatPayload {
                stamp: SimStamp::default(),
                system_state: SYSTEM_STATE.load(Ordering::Relaxed),
                reserved0: [0; 3],
                flags: RESPONSE_FLAGS.load(Ordering::Relaxed),
            };
            hilink::encode_packet(&heartbeat, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::Pong => {
            hilink::encode_packet(&PongPayload, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::Ack(payload) => {
            hilink::encode_packet(&payload, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::Nack(payload) => {
            hilink::encode_packet(&payload, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::HilResponse(payload) => {
            hilink::encode_packet(&payload, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::Imu(payload) => {
            hilink::encode_packet(&payload, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::Mag(payload) => {
            hilink::encode_packet(&payload, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::Baro(payload) => {
            hilink::encode_packet(&payload, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::Gps(payload) => {
            hilink::encode_packet(&payload, seq, send_time_ms, raw_scratch, encoded)
        }
        OutboundMessage::ActuatorStatus(payload) => {
            hilink::encode_packet(&payload, seq, send_time_ms, raw_scratch, encoded)
        }
    }
}

fn send_time_ms() -> u32 {
    Instant::now().as_millis() as u32
}
