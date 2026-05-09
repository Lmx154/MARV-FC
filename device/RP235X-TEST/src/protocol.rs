use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use common::messages::control::ActuatorOutputStamped;
use common::messages::estimate::StateEstimateStamped;
use common::messages::sensor::{
    BarometerSampleStamped, GpsFixSampleStamped, ImuSampleStamped, MagnetometerSampleStamped,
};
use common::policies::failsafe::build_hil_response_frame;
use common::protocol::hilink::{
    self, AckPayload, ActuatorStatusPayload, ActuatorStatusRequestPayload, ArmPayload, BaroPayload,
    BenchDisablePayload, BenchEnablePayload, CvWaypointPayload, DisarmPayload, DshotCommandPayload,
    GlobalWaypointPayload, GpsPayload, HeartbeatPayload, HilReadyPayload, HilResponseFrame,
    HilSensorFrame, ImuPayload, MagPayload, MissionWaypointPayload, MixerMotorOrderPayload,
    MotorStopPayload, MotorSweepPayload, MotorTestPayload, MsgType, NackPayload, PingPayload,
    PongPayload, RtlPayload, SimStamp, TofWaypointPayload, WirePayload, response_flags,
};
use common::services::hil::{
    HilIngressRoutes, HilSensorFrameAdapter, HilSensorFrameRejection, SensorBackend,
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
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::Vec;
use rp235x_base::usb_cdc::{
    EndpointError, RpUsbReceiver, RpUsbSender, USB_PACKET_SIZE, UsbCdcConfig, spawn_cdc_acm,
    write_packet_chunks,
};

use crate::channels::{
    ACTUATOR_OUTPUT_CHANNEL, BAROMETER_CHANNEL, GPS_CHANNEL, IMU_CHANNEL, MAGNETOMETER_CHANNEL,
    STATE_ESTIMATE_CHANNEL, TIME_CHANNEL, TestActuatorOutputSubscriber, TestBarometerSubscriber,
    TestGpsSubscriber, TestImuSubscriber, TestMagnetometerSubscriber, TestStateEstimateSubscriber,
};
use crate::dshot;
use crate::flight_control;

const RX_FRAME_CAPACITY: usize = hilink::encoded_frame_len(HilSensorFrameMaxPayload::WIRE_LEN);
const RAW_FRAME_CAPACITY: usize = hilink::raw_frame_len(HilSensorFrameMaxPayload::WIRE_LEN);
const TX_FRAME_CAPACITY: usize = hilink::encoded_frame_len(HilResponseFrameMaxPayload::WIRE_LEN);
const TX_RAW_CAPACITY: usize = hilink::raw_frame_len(HilResponseFrameMaxPayload::WIRE_LEN);
const SENSOR_TELEMETRY_PERIOD: Duration = Duration::from_millis(20);
const HIL_RESPONSE_SOURCE_WAIT: Duration = Duration::from_millis(2);
const HIL_STREAM_RESET_BACKWARD_US: u64 = 1_000_000;
const STATE_IDLE: u8 = 1;
const STATE_ARMED: u8 = 2;
const ACK_STATUS_ACCEPTED: u8 = 0;
const NACK_REASON_INVALID_PAYLOAD: u8 = 1;
const NACK_REASON_REJECTED: u8 = 2;
const NACK_REASON_UNSUPPORTED: u8 = 3;

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

struct HilResponseSources {
    estimate_subscriber: TestStateEstimateSubscriber,
    actuator_subscriber: TestActuatorOutputSubscriber,
    latest_estimate: Option<StateEstimateStamped>,
    latest_actuator: Option<ActuatorOutputStamped>,
}

impl HilResponseSources {
    fn new(
        estimate_subscriber: TestStateEstimateSubscriber,
        actuator_subscriber: TestActuatorOutputSubscriber,
    ) -> Self {
        Self {
            estimate_subscriber,
            actuator_subscriber,
            latest_estimate: None,
            latest_actuator: None,
        }
    }

    fn refresh(&mut self) {
        self.refresh_estimate();
        self.refresh_actuator();
    }

    async fn refresh_for_hil_stamp(&mut self, timestamp: MeasurementTimestamp) {
        self.refresh();
        if !self.latest_estimate_covers(timestamp) {
            match select(
                self.wait_for_estimate_at_or_after(timestamp),
                Timer::after(HIL_RESPONSE_SOURCE_WAIT),
            )
            .await
            {
                Either::First(()) => {}
                Either::Second(()) => {
                    warn!("rp235x-test HIL response estimate wait timed out");
                }
            }
        }

        self.refresh_actuator();
        if !self.latest_actuator_covers(timestamp) {
            match select(
                self.wait_for_actuator_at_or_after(timestamp),
                Timer::after(HIL_RESPONSE_SOURCE_WAIT),
            )
            .await
            {
                Either::First(()) => {}
                Either::Second(()) => {
                    warn!("rp235x-test HIL response actuator wait timed out");
                }
            }
        }
    }

    fn refresh_estimate(&mut self) {
        while let Some(message) = self.estimate_subscriber.try_next_message() {
            if let WaitResult::Message(sample) = message {
                self.latest_estimate = Some(sample);
            }
        }
    }

    fn refresh_actuator(&mut self) {
        while let Some(message) = self.actuator_subscriber.try_next_message() {
            if let WaitResult::Message(sample) = message {
                self.latest_actuator = Some(sample);
            }
        }
    }

    async fn wait_for_estimate_at_or_after(&mut self, timestamp: MeasurementTimestamp) {
        loop {
            match self.estimate_subscriber.next_message().await {
                WaitResult::Message(sample) => {
                    self.latest_estimate = Some(sample);
                    if self.latest_estimate_covers(timestamp) {
                        return;
                    }
                }
                WaitResult::Lagged(skipped) => {
                    warn!(
                        "rp235x-test HIL response skipped {=u64} estimate samples",
                        skipped
                    );
                }
            }
        }
    }

    async fn wait_for_actuator_at_or_after(&mut self, timestamp: MeasurementTimestamp) {
        loop {
            match self.actuator_subscriber.next_message().await {
                WaitResult::Message(sample) => {
                    self.latest_actuator = Some(sample);
                    if self.latest_actuator_covers(timestamp) {
                        return;
                    }
                }
                WaitResult::Lagged(skipped) => {
                    warn!(
                        "rp235x-test HIL response skipped {=u64} actuator samples",
                        skipped
                    );
                }
            }
        }
    }

    fn latest_estimate_covers(&self, timestamp: MeasurementTimestamp) -> bool {
        self.latest_estimate
            .is_some_and(|estimate| estimate.timestamp >= timestamp)
    }

    fn latest_actuator_covers(&self, timestamp: MeasurementTimestamp) -> bool {
        self.latest_actuator
            .is_some_and(|actuator| actuator.timestamp >= timestamp)
    }
}

pub fn spawn(spawner: &Spawner, usb: Peri<'static, USB>, sensor_backend: SensorBackend) {
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

    spawner
        .spawn(usb_rx_task(usb.receiver, sensor_backend))
        .unwrap();
    spawner
        .spawn(usb_tx_task(usb.sender, sensor_backend))
        .unwrap();
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

pub fn system_state_code() -> u8 {
    SYSTEM_STATE.load(Ordering::Relaxed)
}

pub fn response_flags() -> u32 {
    RESPONSE_FLAGS.load(Ordering::Relaxed)
}

#[embassy_executor::task]
async fn usb_rx_task(mut receiver: RpUsbReceiver, sensor_backend: SensorBackend) -> ! {
    let mut packet = [0u8; USB_PACKET_SIZE as usize];
    let mut frame = RxFrame::new();
    let mut raw_scratch = [0u8; RAW_FRAME_CAPACITY];
    let mut hil_adapter = HilSensorFrameAdapter::new();
    let mut response_sources = HilResponseSources::new(
        STATE_ESTIMATE_CHANNEL.subscriber().unwrap(),
        ACTUATOR_OUTPUT_CHANNEL.subscriber().unwrap(),
    );
    hil_adapter.set_input_active(matches!(sensor_backend, SensorBackend::Hil));

    loop {
        receiver.wait_connection().await;
        info!("rp235x-test usb cdc rx connected");
        frame.clear();

        loop {
            match receiver.read_packet(&mut packet).await {
                Ok(0) => {}
                Ok(len) => {
                    ingest_usb_bytes(
                        &packet[..len],
                        &mut frame,
                        &mut raw_scratch,
                        &mut hil_adapter,
                        &mut response_sources,
                    )
                    .await;
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
async fn usb_tx_task(mut sender: RpUsbSender, sensor_backend: SensorBackend) -> ! {
    let mut seq = 0u16;
    let mut raw_scratch = [0u8; TX_RAW_CAPACITY];
    let mut encoded = TxFrame::new();

    loop {
        sender.wait_connection().await;
        info!("rp235x-test usb cdc tx connected");

        if matches!(sensor_backend, SensorBackend::Hil) {
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

async fn ingest_usb_bytes(
    bytes: &[u8],
    frame: &mut RxFrame,
    raw_scratch: &mut [u8; RAW_FRAME_CAPACITY],
    hil_adapter: &mut HilSensorFrameAdapter,
    response_sources: &mut HilResponseSources,
) {
    for &byte in bytes {
        if frame.push(byte).is_err() {
            warn!("rp235x-test rx frame buffer full");
            frame.clear();
            continue;
        }

        if byte == hilink::FRAME_DELIMITER {
            handle_protocol_frame(frame.as_slice(), raw_scratch, hil_adapter, response_sources)
                .await;
            frame.clear();
        }
    }
}

async fn handle_protocol_frame(
    frame: &[u8],
    raw_scratch: &mut [u8; RAW_FRAME_CAPACITY],
    hil_adapter: &mut HilSensorFrameAdapter,
    response_sources: &mut HilResponseSources,
) {
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
        Ok(MsgType::MixerMotorOrder) => handle_mixer_motor_order_packet(&packet),
        Ok(MsgType::HilSensorFrame) => {
            handle_hil_sensor_frame(&packet, hil_adapter, response_sources).await
        }
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
        response_flags::ARMED | response_flags::FAILSAFE | response_flags::MOTORS_VALID,
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
        Ok(payload) => {
            if !flight_control::submit_control_waypoint(payload) {
                warn!("rp235x-test rejected control waypoint");
                enqueue_outbound(rejected_nack(packet));
                return;
            }
        }
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
        Ok(_) => {
            warn!("rp235x-test cv waypoint not implemented");
            enqueue_outbound(unsupported_nack(packet));
        }
        Err(_) => {
            warn!("rp235x-test rejected invalid cv waypoint payload");
            enqueue_outbound(invalid_payload_nack(packet));
        }
    }
}

fn handle_tof_waypoint_packet(packet: &hilink::DecodedPacket<'_>) {
    match hilink::decode_payload::<TofWaypointPayload>(packet) {
        Ok(_) => {
            warn!("rp235x-test tof waypoint not implemented");
            enqueue_outbound(unsupported_nack(packet));
        }
        Err(_) => {
            warn!("rp235x-test rejected invalid tof waypoint payload");
            enqueue_outbound(invalid_payload_nack(packet));
        }
    }
}

fn handle_mission_waypoint_packet(packet: &hilink::DecodedPacket<'_>) {
    match hilink::decode_payload::<MissionWaypointPayload>(packet) {
        Ok(payload) => {
            if !flight_control::submit_control_waypoint(payload.0) {
                warn!("rp235x-test rejected mission waypoint");
                enqueue_outbound(rejected_nack(packet));
                return;
            }
            info!("rp235x-test accepted mission waypoint");
            enqueue_outbound(ack_for(packet));
        }
        Err(_) => {
            warn!("rp235x-test rejected invalid mission waypoint payload");
            enqueue_outbound(invalid_payload_nack(packet));
        }
    }
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

    if !dshot::submit_bench_enable(payload) {
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

    if BENCH_REQUESTED.load(Ordering::Relaxed) == 0 || !dshot::submit_motor_test(payload) {
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

    if BENCH_REQUESTED.load(Ordering::Relaxed) == 0 || !dshot::submit_motor_sweep(payload) {
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

    if BENCH_REQUESTED.load(Ordering::Relaxed) == 0 || !dshot::submit_dshot_command(payload) {
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

fn handle_mixer_motor_order_packet(packet: &hilink::DecodedPacket<'_>) {
    let payload = match hilink::decode_payload::<MixerMotorOrderPayload>(packet) {
        Ok(payload) => payload,
        Err(_) => {
            warn!("rp235x-test rejected invalid mixer motor order payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    };

    if !flight_control::submit_mixer_motor_order(payload) {
        warn!("rp235x-test rejected mixer motor order");
        enqueue_outbound(rejected_nack(packet));
        return;
    }

    enqueue_outbound(ack_for(packet));
    enqueue_outbound(OutboundMessage::ActuatorStatus(dshot::actuator_status()));
}

async fn handle_hil_sensor_frame(
    packet: &hilink::DecodedPacket<'_>,
    hil_adapter: &mut HilSensorFrameAdapter,
    response_sources: &mut HilResponseSources,
) {
    let sensors = match hilink::decode_payload::<HilSensorFrame>(packet) {
        Ok(sensors) => sensors,
        Err(_) => {
            warn!("rp235x-test rejected invalid hil sensor frame payload");
            enqueue_outbound(invalid_payload_nack(packet));
            return;
        }
    };

    let routes = HilIngressRoutes::new(
        &TIME_CHANNEL,
        &IMU_CHANNEL,
        &BAROMETER_CHANNEL,
        &GPS_CHANNEL,
        &MAGNETOMETER_CHANNEL,
        &(),
    );
    reset_hil_adapter_for_stream_reset(sensors, hil_adapter);
    let dispatch = hil_adapter.accept_frame(sensors, &routes);
    if !dispatch.accepted {
        warn!(
            "rp235x-test HIL sensor frame rejected reason={=u8} tick={=u64} time_us={=u64} flags=0x{=u32:08x} duplicate={=u32} out_of_order={=u32} invalid={=u32}",
            hil_sensor_rejection_code(dispatch.rejection),
            sensors.stamp.sim_tick,
            sensors.stamp.sim_time_us,
            sensors.valid_flags,
            dispatch.counters.duplicate_tick,
            dispatch.counters.out_of_order_tick,
            dispatch.counters.invalid_non_finite_sample
        );
    }

    if dshot::hil_motor_override().is_some() {
        response_sources.refresh();
    } else {
        response_sources
            .refresh_for_hil_stamp(MeasurementTimestamp::from_micros(sensors.stamp.sim_time_us))
            .await;
    }
    let sensor_input_valid = dispatch.accepted && dispatch.dispatch.imu_published;
    let response =
        hil_response_from_current_outputs(sensors.stamp, sensor_input_valid, response_sources);

    if TX_CHANNEL
        .try_send(OutboundMessage::HilResponse(response))
        .is_ok()
    {
        if dispatch.accepted {
            hil_adapter.mark_response_sent(sensors.stamp);
        }
    } else {
        warn!("rp235x-test HIL response dropped (tx full)");
        if dispatch.accepted {
            hil_adapter.mark_missed_response();
        }
    }
}

fn reset_hil_adapter_for_stream_reset(
    sensors: HilSensorFrame,
    hil_adapter: &mut HilSensorFrameAdapter,
) {
    let Some(latest_stamp) = hil_adapter.correlation().latest_accepted_stamp else {
        return;
    };

    if sensors.stamp.sim_tick >= latest_stamp.sim_tick
        && sensors.stamp.sim_time_us >= latest_stamp.sim_time_us
    {
        return;
    }

    let backward_us = latest_stamp
        .sim_time_us
        .saturating_sub(sensors.stamp.sim_time_us);
    if backward_us < HIL_STREAM_RESET_BACKWARD_US {
        return;
    }

    warn!(
        "rp235x-test HIL stream clock reset detected previous_tick={=u64} previous_time_us={=u64} new_tick={=u64} new_time_us={=u64}",
        latest_stamp.sim_tick,
        latest_stamp.sim_time_us,
        sensors.stamp.sim_tick,
        sensors.stamp.sim_time_us
    );
    hil_adapter.reset_stream_correlation();
}

fn hil_response_from_current_outputs(
    stamp: SimStamp,
    sensor_input_valid: bool,
    response_sources: &HilResponseSources,
) -> HilResponseFrame {
    let system_state = SYSTEM_STATE.load(Ordering::Relaxed);
    let armed = system_state == STATE_ARMED;
    let mut response = build_hil_response_frame(
        stamp,
        system_state,
        armed,
        sensor_input_valid,
        response_sources.latest_estimate,
        response_sources.latest_actuator,
    );

    if let Some(motor_cmd) = dshot::hil_motor_override() {
        response.motor_cmd = motor_cmd;
        response.flags |= response_flags::MOTORS_VALID
            | response_flags::CONTROL_VALID
            | response_flags::BENCH_OVERRIDE;
        response.flags &= !response_flags::FAILSAFE;
    }

    RESPONSE_FLAGS.store(response.flags, Ordering::Relaxed);

    response
}

fn hil_sensor_rejection_code(rejection: Option<HilSensorFrameRejection>) -> u8 {
    match rejection {
        Some(HilSensorFrameRejection::Inactive) => 1,
        Some(HilSensorFrameRejection::DuplicateTick) => 2,
        Some(HilSensorFrameRejection::OutOfOrderTick) => 3,
        Some(HilSensorFrameRejection::InvalidSample) => 4,
        None => 0,
    }
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

fn unsupported_nack(packet: &hilink::DecodedPacket<'_>) -> OutboundMessage {
    OutboundMessage::Nack(NackPayload {
        rejected_seq: packet.header.seq,
        rejected_msg_type: packet.header.msg_type,
        reason: NACK_REASON_UNSUPPORTED,
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
