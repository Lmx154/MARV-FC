//! Portable DSHOT command encoding and bench-command validation.

use crate::protocol::hilink::{
    DshotCommandPayload, MotorSweepPayload, MotorTestPayload, bench, motor_test_mode,
};

pub const MOTOR_COUNT: usize = 4;
pub const DSHOT_MAX_COMMAND: u16 = 2047;
pub const DSHOT_MIN_THROTTLE: u16 = 48;
pub const DSHOT_MAX_SPECIAL_COMMAND: u8 = 47;

pub fn dshot_frame(command: u16, request_telemetry: bool) -> u16 {
    let command = command.min(DSHOT_MAX_COMMAND);
    let payload = (command << 1) | u16::from(request_telemetry);
    let checksum = (payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0f;

    (payload << 4) | checksum
}

pub fn dshot_word(command: u16) -> u32 {
    u32::from(dshot_frame(command, false))
}

pub fn normalized_to_dshot(value: u16) -> u16 {
    if value == 0 {
        0
    } else {
        let span = u32::from(DSHOT_MAX_COMMAND - DSHOT_MIN_THROTTLE);
        (u32::from(DSHOT_MIN_THROTTLE) + (u32::from(value) * span / u32::from(u16::MAX))) as u16
    }
}

pub fn command_value_to_dshot(mode: u8, value: u16) -> u16 {
    match mode {
        motor_test_mode::RAW_DSHOT => value,
        motor_test_mode::NORMALIZED => normalized_to_dshot(value),
        _ => 0,
    }
}

pub fn valid_motor_test(payload: MotorTestPayload) -> bool {
    let valid_mask = payload.motor_mask != 0 && (payload.motor_mask & !bench::MOTOR_MASK_ALL) == 0;
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

    valid_mask && valid_mode && valid_duration && valid_value
}

pub fn valid_motor_sweep(payload: MotorSweepPayload) -> bool {
    let valid_mask = payload.motor_mask != 0 && (payload.motor_mask & !bench::MOTOR_MASK_ALL) == 0;
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

    valid_mask && valid_mode && valid_step && valid_duration && valid_repeat && valid_values
}

pub fn valid_dshot_command(payload: DshotCommandPayload) -> bool {
    let valid_mask = payload.motor_mask != 0 && (payload.motor_mask & !bench::MOTOR_MASK_ALL) == 0;
    let valid_command = payload.command <= DSHOT_MAX_SPECIAL_COMMAND;
    let valid_repeat = payload.repeat_count > 0;

    valid_mask && valid_command && valid_repeat
}

pub fn motor_mask_words(mask: u8, dshot: u16) -> [u16; MOTOR_COUNT] {
    let mut words = [0u16; MOTOR_COUNT];
    for (index, word) in words.iter_mut().enumerate() {
        if (mask & (1 << index)) != 0 {
            *word = dshot;
        }
    }
    words
}

pub fn encode_output_words(commands: [u16; MOTOR_COUNT]) -> [u32; MOTOR_COUNT] {
    [
        dshot_word(commands[0]),
        dshot_word(commands[1]),
        dshot_word(commands[2]),
        dshot_word(commands[3]),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encodes_dshot_checksum() {
        assert_eq!(dshot_frame(0, false), 0);
        assert_eq!(dshot_frame(48, false), 0x0606);
        assert_eq!(dshot_frame(2047, false), 0xffee);
    }

    #[test]
    fn maps_normalized_commands_to_dshot_range() {
        assert_eq!(normalized_to_dshot(0), 0);
        assert_eq!(normalized_to_dshot(u16::MAX), DSHOT_MAX_COMMAND);
        assert_eq!(normalized_to_dshot(1) >= DSHOT_MIN_THROTTLE, true);
    }

    #[test]
    fn validates_raw_motor_test_payloads() {
        let payload = MotorTestPayload {
            motor_mask: bench::MOTOR_MASK_M1,
            mode: motor_test_mode::RAW_DSHOT,
            value: DSHOT_MIN_THROTTLE,
            duration_ms: 100,
            ..MotorTestPayload::default()
        };
        assert!(valid_motor_test(payload));

        assert!(!valid_motor_test(MotorTestPayload {
            value: DSHOT_MIN_THROTTLE - 1,
            ..payload
        }));
    }

    #[test]
    fn builds_masked_motor_words() {
        assert_eq!(
            motor_mask_words(bench::MOTOR_MASK_M1 | bench::MOTOR_MASK_M3, 100),
            [100, 0, 100, 0,]
        );
    }
}
