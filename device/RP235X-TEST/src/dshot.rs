use core::sync::atomic::{AtomicU32, Ordering};

use common::drivers::dshot::{
    MOTOR_COUNT, command_value_to_dshot, encode_output_words, motor_mask_words,
    valid_dshot_command, valid_motor_sweep, valid_motor_test,
};
use common::protocol::hilink::{
    ActuatorStatusPayload, BenchEnablePayload, DshotCommandPayload, MotorSweepPayload,
    MotorTestPayload, actuator_flags, bench, motor_test_mode,
};
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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Ticker};

use crate::buses::DshotPioBus;
use crate::pinmap;
use crate::protocol;
use crate::resources::ActuatorPins;

const DSHOT_BIT_RATE_HZ: u32 = 300_000;
const DSHOT_CYCLES_PER_BIT: u32 = 8;
const DSHOT_PIO_CLOCK_HZ: u32 = DSHOT_BIT_RATE_HZ * DSHOT_CYCLES_PER_BIT;
const DSHOT_OUTPUT_PERIOD: Duration = Duration::from_millis(1);
const DSHOT_PREARM_ZERO_FRAMES: u16 = 500;

#[derive(Clone, Copy)]
enum BenchCommand {
    Enable(BenchEnablePayload),
    Disable,
    MotorTest(MotorTestPayload),
    MotorSweep(MotorSweepPayload),
    DshotCommand(DshotCommandPayload),
    Stop,
}

#[derive(Clone, Copy)]
struct SweepState {
    motor_mask: u8,
    mode: u8,
    start_value: u16,
    current_value: u16,
    end_value: u16,
    step_value: u16,
    step_duration_ms: u16,
    zero_between_ms: u16,
    repeat_count: u8,
    ascending: bool,
    step_remaining_ms: u16,
    zero_remaining_ms: u16,
}

bind_interrupts!(struct PioIrqs {
    PIO2_IRQ_0 => PioInterruptHandler<PIO2>;
});

static BENCH_COMMANDS: Channel<CriticalSectionRawMutex, BenchCommand, 8> = Channel::new();
static STATUS_WORDS_01: AtomicU32 = AtomicU32::new(0);
static STATUS_WORDS_23: AtomicU32 = AtomicU32::new(0);
static STATUS_STATE: AtomicU32 = AtomicU32::new(0);
static STATUS_TIMERS: AtomicU32 = AtomicU32::new(0);
static STATUS_FLAGS: AtomicU32 = AtomicU32::new(0);

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

pub fn actuator_status() -> ActuatorStatusPayload {
    let words_01 = STATUS_WORDS_01.load(Ordering::Relaxed);
    let words_23 = STATUS_WORDS_23.load(Ordering::Relaxed);
    let state = STATUS_STATE.load(Ordering::Relaxed);
    let timers = STATUS_TIMERS.load(Ordering::Relaxed);

    ActuatorStatusPayload {
        armed: (state & 0xff) as u8,
        bench_enabled: ((state >> 8) & 0xff) as u8,
        active_motor_mask: ((state >> 16) & 0xff) as u8,
        mode: ((state >> 24) & 0xff) as u8,
        commanded_dshot: [
            (words_01 & 0xffff) as u16,
            ((words_01 >> 16) & 0xffff) as u16,
            (words_23 & 0xffff) as u16,
            ((words_23 >> 16) & 0xffff) as u16,
        ],
        last_command_age_ms: (timers & 0xffff) as u16,
        bench_timeout_ms: ((timers >> 16) & 0xffff) as u16,
        flags: STATUS_FLAGS.load(Ordering::Relaxed),
    }
}

pub fn submit_bench_enable(payload: BenchEnablePayload) -> bool {
    payload.magic == bench::ENABLE_MAGIC
        && payload.timeout_ms > 0
        && BENCH_COMMANDS
            .try_send(BenchCommand::Enable(payload))
            .is_ok()
}

pub fn submit_bench_disable() -> bool {
    BENCH_COMMANDS.try_send(BenchCommand::Disable).is_ok()
}

pub fn submit_motor_stop() -> bool {
    BENCH_COMMANDS.try_send(BenchCommand::Stop).is_ok()
}

pub fn submit_motor_test(payload: MotorTestPayload) -> bool {
    valid_motor_test(payload)
        && BENCH_COMMANDS
            .try_send(BenchCommand::MotorTest(payload))
            .is_ok()
}

pub fn submit_motor_sweep(payload: MotorSweepPayload) -> bool {
    valid_motor_sweep(payload)
        && BENCH_COMMANDS
            .try_send(BenchCommand::MotorSweep(payload))
            .is_ok()
}

pub fn submit_dshot_command(payload: DshotCommandPayload) -> bool {
    valid_dshot_command(payload)
        && BENCH_COMMANDS
            .try_send(BenchCommand::DshotCommand(payload))
            .is_ok()
}

fn next_sweep_value(state: &SweepState) -> Option<u16> {
    if state.current_value == state.end_value {
        return None;
    }

    let next = if state.ascending {
        state.current_value.saturating_add(state.step_value)
    } else {
        state.current_value.saturating_sub(state.step_value)
    };

    if state.ascending {
        Some(next.min(state.end_value))
    } else {
        Some(next.max(state.end_value))
    }
}

fn advance_sweep_step(state: &mut SweepState) -> bool {
    if let Some(next_value) = next_sweep_value(state) {
        state.current_value = next_value;
        state.step_remaining_ms = state.step_duration_ms;
        return true;
    }

    if state.repeat_count <= 1 {
        return false;
    }

    state.repeat_count -= 1;
    state.current_value = state.start_value;
    state.step_remaining_ms = state.step_duration_ms;
    true
}

fn store_status(
    armed: bool,
    bench_enabled: bool,
    active_motor_mask: u8,
    mode: u8,
    commanded_dshot: [u16; MOTOR_COUNT],
    last_command_age_ms: u16,
    bench_timeout_ms: u16,
    flags: u32,
) {
    STATUS_WORDS_01.store(
        u32::from(commanded_dshot[0]) | (u32::from(commanded_dshot[1]) << 16),
        Ordering::Relaxed,
    );
    STATUS_WORDS_23.store(
        u32::from(commanded_dshot[2]) | (u32::from(commanded_dshot[3]) << 16),
        Ordering::Relaxed,
    );
    STATUS_STATE.store(
        u32::from(armed as u8)
            | (u32::from(bench_enabled as u8) << 8)
            | (u32::from(active_motor_mask) << 16)
            | (u32::from(mode) << 24),
        Ordering::Relaxed,
    );
    STATUS_TIMERS.store(
        u32::from(last_command_age_ms) | (u32::from(bench_timeout_ms) << 16),
        Ordering::Relaxed,
    );
    STATUS_FLAGS.store(flags, Ordering::Relaxed);
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
    let stop_commands = [0u16; MOTOR_COUNT];
    let stop_words = encode_output_words(stop_commands);
    let mut ticker = Ticker::every(DSHOT_OUTPUT_PERIOD);
    let mut last_armed = false;
    let mut prearm_zero_frames = DSHOT_PREARM_ZERO_FRAMES;
    let mut bench_enabled = false;
    let mut bench_timeout_ms = 0u16;
    let mut active_motor_mask = 0u8;
    let mut command_mode = motor_test_mode::STOP;
    let mut commanded_dshot = stop_commands;
    let mut command_remaining_ms = 0u16;
    let mut sweep_state: Option<SweepState> = None;
    let mut dshot_special_remaining_frames = 0u8;
    let mut last_command_age_ms = u16::MAX;
    let mut status_flags = 0u32;

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
            warn!(
                "dshot output armed: holding zero for {=u16} ms before bench output",
                DSHOT_PREARM_ZERO_FRAMES,
            );
        } else if !armed && last_armed {
            info!("dshot output disarmed: forcing all motors to zero");
            bench_enabled = false;
            bench_timeout_ms = 0;
            active_motor_mask = 0;
            command_mode = motor_test_mode::STOP;
            commanded_dshot = stop_commands;
            command_remaining_ms = 0;
            sweep_state = None;
            dshot_special_remaining_frames = 0;
        }

        while let Ok(command) = BENCH_COMMANDS.try_receive() {
            match command {
                BenchCommand::Enable(payload) => {
                    bench_enabled = true;
                    bench_timeout_ms = payload.timeout_ms;
                    status_flags &= !actuator_flags::COMMAND_TIMEOUT;
                    info!("dshot bench enabled for {=u16} ms", payload.timeout_ms);
                }
                BenchCommand::Disable => {
                    bench_enabled = false;
                    bench_timeout_ms = 0;
                    active_motor_mask = 0;
                    command_mode = motor_test_mode::STOP;
                    commanded_dshot = stop_commands;
                    command_remaining_ms = 0;
                    sweep_state = None;
                    dshot_special_remaining_frames = 0;
                    info!("dshot bench disabled");
                }
                BenchCommand::Stop => {
                    active_motor_mask = 0;
                    command_mode = motor_test_mode::STOP;
                    commanded_dshot = stop_commands;
                    command_remaining_ms = 0;
                    sweep_state = None;
                    dshot_special_remaining_frames = 0;
                    last_command_age_ms = 0;
                    info!("dshot bench motor stop");
                }
                BenchCommand::MotorTest(payload) => {
                    if armed && bench_enabled && valid_motor_test(payload) {
                        let dshot = command_value_to_dshot(payload.mode, payload.value);
                        commanded_dshot = motor_mask_words(payload.motor_mask, dshot);
                        active_motor_mask = if dshot == 0 { 0 } else { payload.motor_mask };
                        command_mode = payload.mode;
                        command_remaining_ms = if payload.mode == motor_test_mode::STOP {
                            0
                        } else {
                            payload.duration_ms
                        };
                        sweep_state = None;
                        dshot_special_remaining_frames = 0;
                        last_command_age_ms = 0;
                        status_flags &= !actuator_flags::COMMAND_TIMEOUT;
                        warn!(
                            "dshot bench motor test mask=0x{=u8:02x} mode={=u8} dshot={=u16} duration={=u16} ms",
                            payload.motor_mask, payload.mode, dshot, payload.duration_ms,
                        );
                    } else {
                        active_motor_mask = 0;
                        command_mode = motor_test_mode::STOP;
                        commanded_dshot = stop_commands;
                        command_remaining_ms = 0;
                        sweep_state = None;
                        dshot_special_remaining_frames = 0;
                        status_flags |= actuator_flags::REJECTED_WHILE_DISARMED;
                        warn!("dshot rejected motor test while disarmed or bench-disabled");
                    }
                }
                BenchCommand::MotorSweep(payload) => {
                    if armed && bench_enabled && valid_motor_sweep(payload) {
                        let ascending = payload.start_value <= payload.end_value;
                        let current_dshot =
                            command_value_to_dshot(payload.mode, payload.start_value);
                        commanded_dshot = motor_mask_words(payload.motor_mask, current_dshot);
                        active_motor_mask = if current_dshot == 0 {
                            0
                        } else {
                            payload.motor_mask
                        };
                        command_mode = payload.mode;
                        command_remaining_ms = 0;
                        sweep_state = Some(SweepState {
                            motor_mask: payload.motor_mask,
                            mode: payload.mode,
                            start_value: payload.start_value,
                            current_value: payload.start_value,
                            end_value: payload.end_value,
                            step_value: payload.step_value,
                            step_duration_ms: payload.step_duration_ms,
                            zero_between_ms: payload.zero_between_ms,
                            repeat_count: payload.repeat_count,
                            ascending,
                            step_remaining_ms: payload.step_duration_ms,
                            zero_remaining_ms: 0,
                        });
                        dshot_special_remaining_frames = 0;
                        last_command_age_ms = 0;
                        status_flags &= !actuator_flags::COMMAND_TIMEOUT;
                        warn!(
                            "dshot bench motor sweep mask=0x{=u8:02x} mode={=u8} start={=u16} end={=u16} step={=u16}",
                            payload.motor_mask,
                            payload.mode,
                            payload.start_value,
                            payload.end_value,
                            payload.step_value,
                        );
                    } else {
                        active_motor_mask = 0;
                        command_mode = motor_test_mode::STOP;
                        commanded_dshot = stop_commands;
                        command_remaining_ms = 0;
                        sweep_state = None;
                        dshot_special_remaining_frames = 0;
                        status_flags |= actuator_flags::REJECTED_WHILE_DISARMED;
                        warn!("dshot rejected motor sweep while disarmed or bench-disabled");
                    }
                }
                BenchCommand::DshotCommand(payload) => {
                    if armed && bench_enabled && valid_dshot_command(payload) {
                        commanded_dshot =
                            motor_mask_words(payload.motor_mask, u16::from(payload.command));
                        active_motor_mask = payload.motor_mask;
                        command_mode = motor_test_mode::RAW_DSHOT;
                        command_remaining_ms = 0;
                        sweep_state = None;
                        dshot_special_remaining_frames = payload.repeat_count;
                        last_command_age_ms = 0;
                        status_flags &= !actuator_flags::COMMAND_TIMEOUT;
                        warn!(
                            "dshot special command mask=0x{=u8:02x} command={=u8} repeat={=u8}",
                            payload.motor_mask, payload.command, payload.repeat_count,
                        );
                    } else {
                        active_motor_mask = 0;
                        command_mode = motor_test_mode::STOP;
                        commanded_dshot = stop_commands;
                        command_remaining_ms = 0;
                        sweep_state = None;
                        dshot_special_remaining_frames = 0;
                        status_flags |= actuator_flags::REJECTED_WHILE_DISARMED;
                        warn!("dshot rejected special command while disarmed or bench-disabled");
                    }
                }
            }
        }

        if bench_enabled && bench_timeout_ms > 0 {
            bench_timeout_ms -= 1;
            if bench_timeout_ms == 0 {
                bench_enabled = false;
                active_motor_mask = 0;
                command_mode = motor_test_mode::STOP;
                commanded_dshot = stop_commands;
                command_remaining_ms = 0;
                sweep_state = None;
                dshot_special_remaining_frames = 0;
                status_flags |= actuator_flags::COMMAND_TIMEOUT;
                warn!("dshot bench mode timed out");
            }
        }

        let mut clear_special_after_output = false;
        let mut clear_sweep = false;
        let mut sweep_completed = false;
        if let Some(sweep) = sweep_state.as_mut() {
            last_command_age_ms = last_command_age_ms.saturating_add(1);
            if !armed || !bench_enabled {
                active_motor_mask = 0;
                command_mode = motor_test_mode::STOP;
                commanded_dshot = stop_commands;
                clear_sweep = true;
            } else if sweep.zero_remaining_ms > 0 {
                sweep.zero_remaining_ms -= 1;
                active_motor_mask = 0;
                commanded_dshot = stop_commands;
                if sweep.zero_remaining_ms == 0 && !advance_sweep_step(sweep) {
                    sweep_completed = true;
                }
            } else if sweep.step_remaining_ms > 0 {
                sweep.step_remaining_ms -= 1;
                let dshot = command_value_to_dshot(sweep.mode, sweep.current_value);
                commanded_dshot = motor_mask_words(sweep.motor_mask, dshot);
                active_motor_mask = if dshot == 0 { 0 } else { sweep.motor_mask };
                command_mode = sweep.mode;
                if sweep.step_remaining_ms == 0 {
                    if sweep.zero_between_ms > 0 {
                        sweep.zero_remaining_ms = sweep.zero_between_ms;
                        active_motor_mask = 0;
                        commanded_dshot = stop_commands;
                    } else if !advance_sweep_step(sweep) {
                        sweep_completed = true;
                    }
                }
            }

            if clear_sweep || sweep_completed {
                active_motor_mask = 0;
                command_mode = motor_test_mode::STOP;
                commanded_dshot = stop_commands;
                sweep_state = None;
                if sweep_completed {
                    info!("dshot bench motor sweep completed");
                }
            }
        } else if dshot_special_remaining_frames > 0 {
            dshot_special_remaining_frames -= 1;
            last_command_age_ms = last_command_age_ms.saturating_add(1);
            if dshot_special_remaining_frames == 0 {
                clear_special_after_output = true;
            }
        } else if command_remaining_ms > 0 {
            command_remaining_ms -= 1;
            last_command_age_ms = last_command_age_ms.saturating_add(1);
            if command_remaining_ms == 0 {
                active_motor_mask = 0;
                command_mode = motor_test_mode::STOP;
                commanded_dshot = stop_commands;
                status_flags |= actuator_flags::COMMAND_TIMEOUT;
                info!("dshot bench motor command timed out");
            }
        } else {
            last_command_age_ms = last_command_age_ms.saturating_add(1);
        }

        let output_active =
            armed && bench_enabled && prearm_zero_frames == 0 && active_motor_mask != 0;
        let mut flags = status_flags;
        if bench_enabled {
            flags |= actuator_flags::BENCH_MODE_ENABLED;
        }
        if output_active {
            flags |= actuator_flags::OUTPUT_ACTIVE;
        }

        let output_commands = if output_active {
            commanded_dshot
        } else {
            if armed && prearm_zero_frames > 0 {
                prearm_zero_frames -= 1;
            }
            stop_commands
        };
        let output_words = if output_active {
            encode_output_words(commanded_dshot)
        } else {
            stop_words
        };

        store_status(
            armed,
            bench_enabled,
            active_motor_mask,
            command_mode,
            output_commands,
            last_command_age_ms,
            bench_timeout_ms,
            flags,
        );

        push_motor_frames(
            &mut motor1,
            &mut motor2,
            &mut motor3,
            &mut motor4,
            output_words,
        )
        .await;

        if clear_special_after_output {
            active_motor_mask = 0;
            command_mode = motor_test_mode::STOP;
            commanded_dshot = stop_commands;
        }
        last_armed = armed;
        ticker.next().await;
    }
}
