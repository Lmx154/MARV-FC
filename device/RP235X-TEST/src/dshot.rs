use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Drive, SlewRate};
use embassy_rp::peripherals::{PIN_35, PIN_36, PIN_38, PIN_39, PIO2};
use embassy_rp::pio::program::pio_asm;
use embassy_rp::pio::{
    Common, Config as PioConfig, Direction, FifoJoin, Instance,
    InterruptHandler as PioInterruptHandler, LoadedProgram, Pio, PioPin, ShiftConfig,
    ShiftDirection, StateMachine,
};
use embassy_rp::pio_programs::clock_divider::calculate_pio_clock_divider;
use embassy_time::{Duration, Ticker};

use crate::buses::DshotPioBus;
use crate::pinmap;
use crate::protocol;
use crate::resources::ActuatorPins;

const DSHOT_MAX_COMMAND: u16 = 2047;
const DSHOT_BIT_RATE_HZ: u32 = 300_000;
const DSHOT_CYCLES_PER_BIT: u32 = 8;
const DSHOT_PIO_CLOCK_HZ: u32 = DSHOT_BIT_RATE_HZ * DSHOT_CYCLES_PER_BIT;
const DSHOT_OUTPUT_PERIOD: Duration = Duration::from_millis(1);
const DSHOT_PREARM_ZERO_FRAMES: u16 = 500;
const MOTOR_COUNT: usize = 4;
const MOTOR_TEST_SEQUENCE: [(u16, u16); 6] = [
    (150, 750),
    (200, 750),
    (250, 750),
    (300, 1_000),
    (400, 1_000),
    (0, 500),
];

bind_interrupts!(struct PioIrqs {
    PIO2_IRQ_0 => PioInterruptHandler<PIO2>;
});

struct DshotProgram<'d, PIO: Instance> {
    prg: LoadedProgram<'d, PIO>,
}

impl<'d, PIO: Instance> DshotProgram<'d, PIO> {
    fn new(common: &mut Common<'d, PIO>) -> Self {
        let prg = pio_asm!(
            "set pindirs, 1",
            "entry:",
            "   pull",
            "   out null, 16",
            "   set x, 15",
            "loop:",
            "   set pins, 1",
            "   out y, 1",
            "   jmp !y zero",
            "   nop [2]",
            "one:",
            "   set pins, 0",
            "   jmp x-- loop",
            "   jmp reset",
            "zero:",
            "   set pins, 0 [3]",
            "   jmp x-- loop",
            "   jmp reset",
            "reset:",
            "   nop [31]",
            "   nop [31]",
            "   nop [31]",
            "   jmp entry [31]",
        );
        let prg = common.load_program(&prg.program);

        Self { prg }
    }
}

pub fn spawn(spawner: &Spawner, bus: DshotPioBus, pins: ActuatorPins) {
    spawner
        .spawn(dshot_motor_test_task(
            bus.pio, pins.pwm1, pins.pwm2, pins.pwm3, pins.pwm4,
        ))
        .unwrap();
}

fn dshot_frame(command: u16, request_telemetry: bool) -> u16 {
    let command = command.min(DSHOT_MAX_COMMAND);
    let payload = (command << 1) | u16::from(request_telemetry);
    let checksum = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0f;

    (payload << 4) | checksum
}

fn dshot_word(command: u16) -> u32 {
    u32::from(dshot_frame(command, false))
}

fn motor_test_words(stop_words: [u32; MOTOR_COUNT], motor_index: usize, throttle: u16) -> [u32; MOTOR_COUNT] {
    let mut words = stop_words;
    if motor_index < MOTOR_COUNT {
        words[motor_index] = dshot_word(throttle);
    }
    words
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
    config.use_program(&program.prg, &[]);
    config.clock_divider = calculate_pio_clock_divider(DSHOT_PIO_CLOCK_HZ);
    config.fifo_join = FifoJoin::TxOnly;
    config.shift_out = ShiftConfig {
        auto_fill: false,
        threshold: 32,
        direction: ShiftDirection::Left,
    };

    sm.set_config(&config);
    sm.set_enable(true);
    sm
}

async fn push_motor_frames(
    motor1: &mut StateMachine<'static, PIO2, 0>,
    motor2: &mut StateMachine<'static, PIO2, 1>,
    motor3: &mut StateMachine<'static, PIO2, 2>,
    motor4: &mut StateMachine<'static, PIO2, 3>,
    words: [u32; MOTOR_COUNT],
) {
    motor1.tx().wait_push(words[0]).await;
    motor2.tx().wait_push(words[1]).await;
    motor3.tx().wait_push(words[2]).await;
    motor4.tx().wait_push(words[3]).await;
}

#[embassy_executor::task]
async fn dshot_motor_test_task(
    pio_peripheral: Peri<'static, PIO2>,
    motor1_pin: Peri<'static, PIN_39>,
    motor2_pin: Peri<'static, PIN_38>,
    motor3_pin: Peri<'static, PIN_35>,
    motor4_pin: Peri<'static, PIN_36>,
) -> ! {
    let mut pio = Pio::new(pio_peripheral, PioIrqs);
    let program = DshotProgram::new(&mut pio.common);
    let mut motor1 = configure_dshot_sm(&mut pio.common, pio.sm0, motor1_pin, &program);
    let mut motor2 = configure_dshot_sm(&mut pio.common, pio.sm1, motor2_pin, &program);
    let mut motor3 = configure_dshot_sm(&mut pio.common, pio.sm2, motor3_pin, &program);
    let mut motor4 = configure_dshot_sm(&mut pio.common, pio.sm3, motor4_pin, &program);
    let stop_word = dshot_word(0);
    let stop_words = [stop_word; 4];
    let mut ticker = Ticker::every(DSHOT_OUTPUT_PERIOD);
    let mut last_armed = false;
    let mut prearm_zero_frames = DSHOT_PREARM_ZERO_FRAMES;
    let mut test_motor_index = 0usize;
    let mut test_step_index = 0usize;
    let mut test_step_frames_remaining = 0u16;

    info!(
        "dshot300 output ready: GP{=u8}/GP{=u8}/GP{=u8}/GP{=u8}, idle zero until explicit arm",
        pinmap::ESC_PWM_1,
        pinmap::ESC_PWM_2,
        pinmap::ESC_PWM_3,
        pinmap::ESC_PWM_4,
    );

    loop {
        let armed = protocol::is_armed();

        if armed && !last_armed {
            prearm_zero_frames = DSHOT_PREARM_ZERO_FRAMES;
            test_motor_index = 0;
            test_step_index = 0;
            test_step_frames_remaining = 0;
            warn!(
                "dshot motor test armed: holding zero for {=u16} ms, then sweeping all motors one at a time",
                DSHOT_PREARM_ZERO_FRAMES,
            );
        } else if !armed && last_armed {
            info!("dshot motor test disarmed: forcing all motors to zero");
        }

        let words = if armed && prearm_zero_frames == 0 {
            if test_step_frames_remaining == 0 {
                let (throttle, frames) = MOTOR_TEST_SEQUENCE[test_step_index];
                test_step_frames_remaining = frames;
                test_step_index = (test_step_index + 1) % MOTOR_TEST_SEQUENCE.len();

                if throttle == 0 {
                    info!(
                        "dshot motor test motor {=usize} zero reset for {=u16} ms",
                        test_motor_index + 1,
                        frames,
                    );
                    test_motor_index = (test_motor_index + 1) % MOTOR_COUNT;
                } else {
                    warn!(
                        "dshot motor test motor {=usize} throttle {=u16} for {=u16} ms",
                        test_motor_index + 1,
                        throttle,
                        frames,
                    );
                }
            }

            test_step_frames_remaining -= 1;
            let throttle =
                MOTOR_TEST_SEQUENCE[(test_step_index + MOTOR_TEST_SEQUENCE.len() - 1)
                    % MOTOR_TEST_SEQUENCE.len()]
                .0;
            motor_test_words(stop_words, test_motor_index, throttle)
        } else {
            if armed {
                prearm_zero_frames -= 1;
            }
            stop_words
        };

        push_motor_frames(&mut motor1, &mut motor2, &mut motor3, &mut motor4, words).await;
        last_armed = armed;
        ticker.next().await;
    }
}
