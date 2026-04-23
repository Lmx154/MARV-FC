#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use common::protocol::hilink::{
    self, AckPayload, ArmPayload, DisarmPayload, HeartbeatPayload, HilReadyPayload,
    HilResponseFrame, HilSensorFrame, MsgType, NackPayload, PingPayload, PongPayload, SimStamp,
    WirePayload, response_flags,
};
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Drive, SlewRate};
use embassy_rp::peripherals::{PIN_35, PIN_36, PIN_38, PIN_39, PIO0, USB};
use embassy_rp::pio::program as pio_program;
use embassy_rp::pio::{
    Common, Config as PioConfig, Direction, FifoJoin, Instance,
    InterruptHandler as PioInterruptHandler, LoadedProgram, Pio, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Receiver, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const USB_PACKET_SIZE: u16 = 64;
const RX_FRAME_CAPACITY: usize = hilink::encoded_frame_len(HilSensorFrameMaxPayload::WIRE_LEN);
const RAW_FRAME_CAPACITY: usize = hilink::raw_frame_len(HilSensorFrameMaxPayload::WIRE_LEN);
const TX_FRAME_CAPACITY: usize = hilink::encoded_frame_len(HilResponseFrameMaxPayload::WIRE_LEN);
const TX_RAW_CAPACITY: usize = hilink::raw_frame_len(HilResponseFrameMaxPayload::WIRE_LEN);
const STATE_IDLE: u8 = 1;
const STATE_ARMED: u8 = 2;
const ACK_STATUS_ACCEPTED: u8 = 0;
const NACK_REASON_INVALID_PAYLOAD: u8 = 1;
const DSHOT_BITS: u8 = 16;
const DSHOT_BIT_RATE_HZ: u32 = 300_000;
const DSHOT_CYCLES_PER_BIT: u32 = 20;
const DSHOT_PIO_CLOCK_HZ: u32 = DSHOT_BIT_RATE_HZ * DSHOT_CYCLES_PER_BIT;
const MOTOR_TEST_BOOT_DELAY_FRAMES: u16 = 5_000;
const MOTOR_TEST_FRAME_PERIOD: Duration = Duration::from_millis(1);
const MOTOR_TEST_BURST_FRAMES: u8 = 4;
const MOTOR_TEST_SEQUENCE: [(u16, u16); 6] = [
    (0, 500),
    (220, 250),
    (320, 250),
    (450, 350),
    (600, 750),
    (0, 500),
];

type RpUsbDriver = Driver<'static, USB>;
type RpUsbDevice = UsbDevice<'static, RpUsbDriver>;
type RpUsbSender = Sender<'static, RpUsbDriver>;
type RpUsbReceiver = Receiver<'static, RpUsbDriver>;
type RxFrame = Vec<u8, RX_FRAME_CAPACITY>;
type TxFrame = Vec<u8, TX_FRAME_CAPACITY>;

// Named wrappers keep the capacity constants easy to read as the test app grows.
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
}

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
});

static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESCRIPTOR: StaticCell<[u8; 0]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_ACM_STATE: StaticCell<State<'static>> = StaticCell::new();
static TX_CHANNEL: Channel<CriticalSectionRawMutex, OutboundMessage, 8> = Channel::new();
static SYSTEM_STATE: AtomicU8 = AtomicU8::new(STATE_IDLE);
static RESPONSE_FLAGS: AtomicU32 = AtomicU32::new(response_flags::MOTORS_VALID);

// RP235x uses an IMAGE_DEF block in flash instead of an RP2040-style boot2 blob.
#[unsafe(link_section = ".start_block")]
#[used]
static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

struct DshotProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> DshotProgram<'d, PIO> {
    fn new(common: &mut Common<'d, PIO>) -> Self {
        let side_set = pio_program::SideSet::new(false, 1, false);
        let mut assembler: pio_program::Assembler<32> =
            pio_program::Assembler::new_with_side_set(side_set);

        let mut wrap_target = assembler.label();
        let mut wrap_source = assembler.label();
        let mut do_zero = assembler.label();

        assembler.set_with_side_set(pio_program::SetDestination::PINDIRS, 1, 0);
        assembler.bind(&mut wrap_target);
        assembler.out_with_delay_and_side_set(
            pio_program::OutDestination::X,
            1,
            6,
            1,
        );
        assembler.jmp_with_delay_and_side_set(
            pio_program::JmpCondition::XIsZero,
            &mut do_zero,
            0,
            1,
        );
        assembler.nop_with_delay_and_side_set(6, 1);
        assembler.nop_with_delay_and_side_set(3, 0);
        assembler.jmp_with_delay_and_side_set(pio_program::JmpCondition::Always, &mut wrap_target, 0, 0);
        assembler.bind(&mut do_zero);
        assembler.nop_with_delay_and_side_set(10, 0);
        assembler.jmp_with_delay_and_side_set(pio_program::JmpCondition::Always, &mut wrap_target, 0, 0);
        assembler.bind(&mut wrap_source);

        let prg = assembler.assemble_with_wrap(wrap_source, wrap_target);
        let prg = common.load_program(&prg);

        Self { prg }
    }
}

fn dshot_frame(command: u16, request_telemetry: bool) -> u16 {
    let payload = ((command & 0x07ff) << 1) | u16::from(request_telemetry);
    let checksum = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0f;

    (payload << 4) | checksum
}

fn dshot_word(command: u16) -> u32 {
    u32::from(dshot_frame(command, false)) << (32 - DSHOT_BITS)
}

fn configure_dshot_sm<'d, PIO, PIN, const SM: usize>(
    common: &mut Common<'d, PIO>,
    mut sm: StateMachine<'d, PIO, SM>,
    pin: Peri<'d, PIN>,
    program: &DshotProgram<'d, PIO>,
) -> StateMachine<'d, PIO, SM>
where
    PIO: Instance + 'd,
    PIN: PioPin + 'd,
{
    let mut pin = common.make_pio_pin(pin);
    pin.set_drive_strength(Drive::_4mA);
    pin.set_slew_rate(SlewRate::Fast);

    sm.set_pin_dirs(Direction::Out, &[&pin]);

    let mut config = PioConfig::default();
    config.set_set_pins(&[&pin]);
    config.use_program(&program.prg, &[&pin]);
    config.clock_divider = calculate_pio_clock_divider(DSHOT_PIO_CLOCK_HZ);
    config.fifo_join = FifoJoin::TxOnly;
    config.shift_out = ShiftConfig {
        auto_fill: true,
        threshold: DSHOT_BITS,
        direction: ShiftDirection::Left,
    };

    sm.set_config(&config);
    sm.set_enable(true);
    sm
}

async fn push_motor_frames(
    motor1: &mut StateMachine<'static, PIO0, 0>,
    motor2: &mut StateMachine<'static, PIO0, 1>,
    motor3: &mut StateMachine<'static, PIO0, 2>,
    motor4: &mut StateMachine<'static, PIO0, 3>,
    word: u32,
) {
    motor1.tx().wait_push(word).await;
    motor2.tx().wait_push(word).await;
    motor3.tx().wait_push(word).await;
    motor4.tx().wait_push(word).await;
}

async fn push_motor_burst(
    motor1: &mut StateMachine<'static, PIO0, 0>,
    motor2: &mut StateMachine<'static, PIO0, 1>,
    motor3: &mut StateMachine<'static, PIO0, 2>,
    motor4: &mut StateMachine<'static, PIO0, 3>,
    word: u32,
) {
    for _ in 0..MOTOR_TEST_BURST_FRAMES {
        push_motor_frames(motor1, motor2, motor3, motor4, word).await;
    }
}

#[embassy_executor::task]
async fn dshot_motor_test_task(
    pio_peripheral: Peri<'static, PIO0>,
    motor1_pin: Peri<'static, PIN_39>,
    motor2_pin: Peri<'static, PIN_38>,
    motor3_pin: Peri<'static, PIN_35>,
    motor4_pin: Peri<'static, PIN_36>,
) -> ! {
    let mut pio = Pio::new(pio_peripheral, Irqs);
    let program = DshotProgram::new(&mut pio.common);
    let mut motor1 = configure_dshot_sm(&mut pio.common, pio.sm0, motor1_pin, &program);
    let mut motor2 = configure_dshot_sm(&mut pio.common, pio.sm1, motor2_pin, &program);
    let mut motor3 = configure_dshot_sm(&mut pio.common, pio.sm2, motor3_pin, &program);
    let mut motor4 = configure_dshot_sm(&mut pio.common, pio.sm3, motor4_pin, &program);
    let stop_word = dshot_word(0);
    let mut ticker = Ticker::every(MOTOR_TEST_FRAME_PERIOD);

    info!(
        "dshot motor test armed: GP39/GP38/GP35/GP36 stop for 5s, then stepped retry ramp with {=u8}x burst resend",
        MOTOR_TEST_BURST_FRAMES,
    );

    for _ in 0..MOTOR_TEST_BOOT_DELAY_FRAMES {
        push_motor_burst(
            &mut motor1,
            &mut motor2,
            &mut motor3,
            &mut motor4,
            stop_word,
        )
        .await;
        ticker.next().await;
    }

    info!("dshot motor test starting retry ramp on all four motors");

    loop {
        for (throttle, frames) in MOTOR_TEST_SEQUENCE {
            let word = dshot_word(throttle);

            if throttle == 0 {
                info!("dshot motor test neutral reset for {=u16} ms", frames);
            } else {
                info!(
                    "dshot motor test throttle step {=u16} for {=u16} ms",
                    throttle,
                    frames
                );
            }

            for _ in 0..frames {
                push_motor_burst(
                    &mut motor1,
                    &mut motor2,
                    &mut motor3,
                    &mut motor4,
                    word,
                )
                .await;
                ticker.next().await;
            }
        }
    }
}

#[embassy_executor::task]
async fn usb_device_task(mut usb: RpUsbDevice) -> ! {
    usb.run().await
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
    enqueue_outbound(ack_for(packet));
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

fn enqueue_outbound(message: OutboundMessage) {
    if TX_CHANNEL.try_send(message).is_err() {
        warn!("rp235x-test tx queue full");
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
    write_packet(sender, encoded.as_slice()).await
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
    }
}

fn send_time_ms() -> u32 {
    Instant::now().as_millis() as u32
}

async fn write_packet(sender: &mut RpUsbSender, bytes: &[u8]) -> Result<(), EndpointError> {
    for chunk in bytes.chunks(USB_PACKET_SIZE as usize) {
        sender.write_packet(chunk).await?;
    }

    Ok(())
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = embassy_rp::init(Default::default());
    let driver = Driver::new(peripherals.USB, Irqs);

    let mut config = Config::new(0x1209, 0x0002);
    config.manufacturer = Some("MARV");
    config.product = Some("RP235X Protocol Test");
    config.serial_number = Some("MARV-RP235X-TEST");
    config.max_packet_size_0 = 64;

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        MSOS_DESCRIPTOR.init([0; 0]),
        CONTROL_BUF.init([0; 64]),
    );

    let state = CDC_ACM_STATE.init(State::new());
    let class = CdcAcmClass::new(&mut builder, state, USB_PACKET_SIZE);
    let (sender, receiver) = class.split();
    let usb = builder.build();

    info!("rp235x-test usb cdc bring-up firmware booted");

    spawner.spawn(usb_device_task(usb)).unwrap();
    spawner.spawn(usb_rx_task(receiver)).unwrap();
    spawner.spawn(usb_tx_task(sender)).unwrap();
    spawner
        .spawn(dshot_motor_test_task(
            peripherals.PIO0,
            peripherals.PIN_39,
            peripherals.PIN_38,
            peripherals.PIN_35,
            peripherals.PIN_36,
        ))
        .unwrap();
}
