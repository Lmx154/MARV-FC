//! Hardware-independent DSHOT and AM32-compatible command helpers.

use crate::protocol::hilink::{
    BenchEnablePayload, DshotCommandPayload, MotorSweepPayload, MotorTestPayload, bench,
    motor_test_mode,
};

pub const MOTOR_COUNT: usize = 4;
pub const DSHOT_MAX_COMMAND: u16 = 2047;
pub const DSHOT_MIN_THROTTLE: u16 = 48;
pub const DSHOT_MAX_SPECIAL_COMMAND: u8 = 47;

pub mod am32 {
    //! AM32 accepts the standard DSHOT special-command range.

    pub const MAX_SPECIAL_COMMAND: u8 = super::DSHOT_MAX_SPECIAL_COMMAND;

    pub const fn is_special_command(command: u8) -> bool {
        command <= MAX_SPECIAL_COMMAND
    }

    pub const fn valid_repeat_count(repeat_count: u8) -> bool {
        repeat_count > 0
    }
}

pub fn frame(command: u16, request_telemetry: bool) -> u16 {
    let command = if command > DSHOT_MAX_COMMAND {
        DSHOT_MAX_COMMAND
    } else {
        command
    };
    let payload = (command << 1) | u16::from(request_telemetry);
    let checksum = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0f;

    (payload << 4) | checksum
}

pub fn pio_word(command: u16) -> u32 {
    u32::from(frame(command, false))
}

pub fn normalized_throttle_to_command(value: u16) -> u16 {
    if value == 0 {
        0
    } else {
        let span = u32::from(DSHOT_MAX_COMMAND - DSHOT_MIN_THROTTLE);
        (u32::from(DSHOT_MIN_THROTTLE) + (u32::from(value) * span / u32::from(u16::MAX))) as u16
    }
}

pub fn unit_throttle_to_command(value: f32) -> u16 {
    if value <= 0.0 {
        return 0;
    }

    let value = value.min(1.0);
    let span = f32::from(DSHOT_MAX_COMMAND - DSHOT_MIN_THROTTLE);
    (f32::from(DSHOT_MIN_THROTTLE) + value * span) as u16
}

pub fn command_value_to_dshot(mode: u8, value: u16) -> u16 {
    match mode {
        motor_test_mode::RAW_DSHOT => value.min(DSHOT_MAX_COMMAND),
        motor_test_mode::NORMALIZED => normalized_throttle_to_command(value),
        _ => 0,
    }
}

pub fn motor_mask_commands(mask: u8, command: u16) -> [u16; MOTOR_COUNT] {
    let mut commands = [0u16; MOTOR_COUNT];
    for (index, word) in commands.iter_mut().enumerate() {
        if (mask & (1 << index)) != 0 {
            *word = command;
        }
    }
    commands
}

pub fn encode_commands(commands: [u16; MOTOR_COUNT]) -> [u32; MOTOR_COUNT] {
    [
        pio_word(commands[0]),
        pio_word(commands[1]),
        pio_word(commands[2]),
        pio_word(commands[3]),
    ]
}

pub fn valid_motor_mask(mask: u8) -> bool {
    mask != 0 && (mask & !bench::MOTOR_MASK_ALL) == 0
}

pub fn valid_bench_enable(payload: BenchEnablePayload) -> bool {
    payload.magic == bench::ENABLE_MAGIC && payload.timeout_ms > 0
}

pub fn valid_motor_test(payload: MotorTestPayload) -> bool {
    let valid_mode = matches!(
        payload.mode,
        motor_test_mode::STOP | motor_test_mode::RAW_DSHOT | motor_test_mode::NORMALIZED
    );
    let valid_duration = payload.mode == motor_test_mode::STOP
        || (payload.duration_ms > 0 && payload.duration_ms <= bench::MAX_TEST_DURATION_MS);
    let valid_value = match payload.mode {
        motor_test_mode::STOP => true,
        motor_test_mode::RAW_DSHOT => {
            payload.value == 0 || (DSHOT_MIN_THROTTLE..=DSHOT_MAX_COMMAND).contains(&payload.value)
        }
        motor_test_mode::NORMALIZED => true,
        _ => false,
    };

    valid_motor_mask(payload.motor_mask) && valid_mode && valid_duration && valid_value
}

pub fn valid_motor_sweep(payload: MotorSweepPayload) -> bool {
    let valid_mode = matches!(
        payload.mode,
        motor_test_mode::RAW_DSHOT | motor_test_mode::NORMALIZED
    );
    let valid_step = payload.step_value > 0;
    let valid_duration =
        payload.step_duration_ms > 0 && payload.step_duration_ms <= bench::MAX_TEST_DURATION_MS;
    let valid_repeat = payload.repeat_count > 0;
    let valid_values = match payload.mode {
        motor_test_mode::RAW_DSHOT => {
            let start = payload.start_value;
            let end = payload.end_value;
            (start == 0 || (DSHOT_MIN_THROTTLE..=DSHOT_MAX_COMMAND).contains(&start))
                && (end == 0 || (DSHOT_MIN_THROTTLE..=DSHOT_MAX_COMMAND).contains(&end))
        }
        motor_test_mode::NORMALIZED => true,
        _ => false,
    };

    valid_motor_mask(payload.motor_mask)
        && valid_mode
        && valid_step
        && valid_duration
        && valid_repeat
        && valid_values
}

pub fn valid_dshot_command(payload: DshotCommandPayload) -> bool {
    valid_motor_mask(payload.motor_mask)
        && am32::is_special_command(payload.command)
        && am32::valid_repeat_count(payload.repeat_count)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_matches_known_zero_command() {
        assert_eq!(frame(0, false), 0);
    }

    #[test]
    fn frame_clamps_to_dshot_range() {
        assert_eq!(frame(4095, false), frame(DSHOT_MAX_COMMAND, false));
    }

    #[test]
    fn normalized_throttle_preserves_zero_and_minimum_nonzero() {
        assert_eq!(normalized_throttle_to_command(0), 0);
        assert_eq!(normalized_throttle_to_command(1), DSHOT_MIN_THROTTLE);
        assert_eq!(normalized_throttle_to_command(u16::MAX), DSHOT_MAX_COMMAND);
    }

    #[test]
    fn rejects_out_of_range_special_command() {
        assert!(!valid_dshot_command(DshotCommandPayload {
            motor_mask: bench::MOTOR_MASK_M1,
            command: DSHOT_MAX_SPECIAL_COMMAND + 1,
            repeat_count: 1,
            reserved0: 0,
        }));
    }
}
