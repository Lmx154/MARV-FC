//! The mapping from firmware state to the SYS_STATUS bits.
//!
//! Pure functions over plain values, from the per-device health rows to what the ground station
//! is sent: the three SYS_STATUS sensor masks ([`sensor_masks`]), the HEARTBEAT `system_status`
//! ([`system_status`]) and the dialect's `MARV_DEVICE_STATUS` row ([`device_status`]). Invariant: the health
//! bit of a device is set if and only if it is Ok and not stuck; for the receiver, if and only if the port is locked.
//!
//! # Bit table
//!
//! | device | bits |
//! |---|---|
//! | IMU1 | 3D_GYRO + 3D_ACCEL |
//! | IMU2 | 3D_GYRO2 + 3D_ACCEL2 |
//! | MAG | 3D_MAG |
//! | BARO | ABSOLUTE_PRESSURE |
//! | GNSS | GPS |
//! | RC (receiver) | RC_RECEIVER |
//! | BATT | BATTERY |
//! | SD | LOGGING |
//! | PSRAM, LED, ESC | none |
//!
//! A device with no row in the slice sets no bit; two rows of one device are ORed.

use crate::health::{Counters, ErrorKind, State};
use mavlink_device::dialect::MavState;
use mavlink_device::marv::{
    MARV_DEVICE_STATUS_DATA, MarvDevice, MarvDeviceError, MarvDeviceState, MavSysStatusSensor,
};

/// Time since boot after which the HEARTBEAT no longer says BOOT, whatever the rows say.
pub const BOOT_WINDOW_MS: u64 = 3_000;

/// Value of `last_sample_age_ms` when the device has never delivered a sample.
pub const AGE_NEVER: u16 = u16::MAX;

/// Largest age reported for a device that has delivered a sample (one below [`AGE_NEVER`]).
pub const AGE_MAX: u16 = u16::MAX - 1;

/// Bit 0 of the `flags` of a `MARV_DEVICE_STATUS` row: the sample source is HIL.
pub const FLAG_HIL: u8 = 1;

/// Bit 1 of the `flags` of a `MARV_DEVICE_STATUS` row: a self-test is running.
pub const FLAG_SELF_TEST: u8 = 2;

/// One device as the status mapping sees it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeviceRow {
    /// The device the row is about.
    pub device: MarvDevice,
    /// The device answered its identity this boot.
    pub present: bool,
    /// The state of its health machine.
    pub state: State,
    /// The supervisor has flagged its task as stuck.
    pub stuck: bool,
    /// The receiver's port is locked. Read for the receiver (RC) only, whose health is this fact.
    pub locked: bool,
}

/// The three SYS_STATUS sensor masks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SensorMasks {
    /// `onboard_control_sensors_present`.
    pub present: u32,
    /// `onboard_control_sensors_enabled`.
    pub enabled: u32,
    /// `onboard_control_sensors_health`.
    pub health: u32,
}

/// The SYS_STATUS bits of a device (the SD card is LOGGING, as in ArduPilot); 0 for the devices that
/// have none (PSRAM, LED, ESC).
pub fn sensor_bits(device: MarvDevice) -> u32 {
    match device {
        MarvDevice::MARV_DEVICE_IMU1 => {
            MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO.bits()
                | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL.bits()
        }
        MarvDevice::MARV_DEVICE_IMU2 => {
            MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO2.bits()
                | MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL2.bits()
        }
        MarvDevice::MARV_DEVICE_MAG => MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG.bits(),
        MarvDevice::MARV_DEVICE_BARO => {
            MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE.bits()
        }
        MarvDevice::MARV_DEVICE_GNSS => MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS.bits(),
        MarvDevice::MARV_DEVICE_RC => MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_RC_RECEIVER.bits(),
        MarvDevice::MARV_DEVICE_BATT => MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY.bits(),
        MarvDevice::MARV_DEVICE_SD => MavSysStatusSensor::MAV_SYS_STATUS_LOGGING.bits(),
        MarvDevice::MARV_DEVICE_PSRAM
        | MarvDevice::MARV_DEVICE_LED
        | MarvDevice::MARV_DEVICE_ESC => 0,
    }
}

fn is_enabled(row: &DeviceRow) -> bool {
    matches!(
        row.state,
        State::Starting | State::Ok | State::Degraded | State::Recovering
    )
}

/// Ok and not stuck; for the receiver, locked.
fn is_healthy(row: &DeviceRow) -> bool {
    match row.device {
        MarvDevice::MARV_DEVICE_RC => row.locked,
        _ => row.state == State::Ok && !row.stuck,
    }
}

fn masks_with(rows: &[DeviceRow], healthy: fn(&DeviceRow) -> bool) -> SensorMasks {
    let mut masks = SensorMasks::default();
    for row in rows {
        let bits = sensor_bits(row.device);
        if row.present {
            masks.present |= bits;
        }
        if is_enabled(row) {
            masks.enabled |= bits;
        }
        if healthy(row) {
            masks.health |= bits;
        }
    }
    masks
}

/// The SYS_STATUS masks for the given rows: `present` = answered its identity this boot, `enabled` = Starting, Ok, Degraded or Recovering, `health` = Ok and not stuck (the receiver: locked).
pub fn sensor_masks(rows: &[DeviceRow]) -> SensorMasks {
    masks_with(rows, is_healthy)
}

/// The HEARTBEAT `system_status`.
pub fn system_status(rows: &[DeviceRow], now_ms: u64, calibrating: bool) -> MavState {
    let starting = rows.iter().any(|row| row.state == State::Starting);
    if starting && now_ms < BOOT_WINDOW_MS {
        MavState::MAV_STATE_BOOT
    } else if calibrating {
        MavState::MAV_STATE_CALIBRATING
    } else {
        MavState::MAV_STATE_STANDBY
    }
}

/// What one `MARV_DEVICE_STATUS` row is built from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StatusInput {
    /// The device the row is about.
    pub device: MarvDevice,
    /// The state of its health machine.
    pub state: State,
    /// Its counters and last error.
    pub counters: Counters,
    /// The supervisor has flagged its task as stuck (reported as Failed with error STUCK).
    pub stuck: bool,
    /// Samples delivered since boot.
    pub samples: u32,
    /// Age of the last sample in ms; `None` = never.
    pub last_sample_age_ms: Option<u32>,
    /// The sample source is HIL (flag bit 0).
    pub hil: bool,
    /// A self-test is running (flag bit 1).
    pub self_test: bool,
    /// Time since boot in ms.
    pub time_boot_ms: u32,
}

fn dialect_state(state: State) -> MarvDeviceState {
    match state {
        State::Absent => MarvDeviceState::MARV_DEVICE_STATE_ABSENT,
        State::Starting => MarvDeviceState::MARV_DEVICE_STATE_STARTING,
        State::Ok => MarvDeviceState::MARV_DEVICE_STATE_OK,
        State::Degraded => MarvDeviceState::MARV_DEVICE_STATE_DEGRADED,
        State::Recovering => MarvDeviceState::MARV_DEVICE_STATE_RECOVERING,
        State::Failed => MarvDeviceState::MARV_DEVICE_STATE_FAILED,
    }
}

fn dialect_error(error: ErrorKind) -> MarvDeviceError {
    match error {
        ErrorKind::None => MarvDeviceError::MARV_DEVICE_ERROR_NONE,
        ErrorKind::Bus => MarvDeviceError::MARV_DEVICE_ERROR_BUS,
        ErrorKind::Timeout => MarvDeviceError::MARV_DEVICE_ERROR_TIMEOUT,
        ErrorKind::IdMismatch => MarvDeviceError::MARV_DEVICE_ERROR_ID_MISMATCH,
        ErrorKind::ConfigMismatch => MarvDeviceError::MARV_DEVICE_ERROR_CONFIG_MISMATCH,
        ErrorKind::Por => MarvDeviceError::MARV_DEVICE_ERROR_POR,
        ErrorKind::Stuck => MarvDeviceError::MARV_DEVICE_ERROR_STUCK,
    }
}

fn saturate16(value: u32) -> u16 {
    u16::try_from(value).unwrap_or(u16::MAX)
}

fn age_field(age: Option<u32>) -> u16 {
    match age {
        None => AGE_NEVER,
        Some(ms) => u16::try_from(ms).unwrap_or(AGE_MAX).min(AGE_MAX),
    }
}

/// Builds the `MARV_DEVICE_STATUS` row.
pub fn device_status(input: &StatusInput) -> MARV_DEVICE_STATUS_DATA {
    let (state, last_error) = if input.stuck {
        (State::Failed, ErrorKind::Stuck)
    } else {
        (input.state, input.counters.last_error)
    };
    let mut flags = 0;
    if input.hil {
        flags |= FLAG_HIL;
    }
    if input.self_test {
        flags |= FLAG_SELF_TEST;
    }
    MARV_DEVICE_STATUS_DATA {
        time_boot_ms: input.time_boot_ms,
        samples: input.samples,
        errors: input.counters.errors,
        timeouts: input.counters.timeouts,
        last_sample_age_ms: age_field(input.last_sample_age_ms),
        recoveries: saturate16(input.counters.recoveries),
        attempts: u16::from(input.counters.attempts),
        overflows: saturate16(input.counters.overflows),
        config_restored: saturate16(input.counters.config_restored),
        device_id: input.device,
        state: dialect_state(state),
        last_error: dialect_error(last_error),
        flags,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL_STATES: [State; 6] = [
        State::Starting,
        State::Ok,
        State::Degraded,
        State::Recovering,
        State::Failed,
        State::Absent,
    ];

    const ALL_DEVICES: [MarvDevice; 11] = [
        MarvDevice::MARV_DEVICE_IMU1,
        MarvDevice::MARV_DEVICE_IMU2,
        MarvDevice::MARV_DEVICE_BARO,
        MarvDevice::MARV_DEVICE_MAG,
        MarvDevice::MARV_DEVICE_GNSS,
        MarvDevice::MARV_DEVICE_BATT,
        MarvDevice::MARV_DEVICE_RC,
        MarvDevice::MARV_DEVICE_SD,
        MarvDevice::MARV_DEVICE_PSRAM,
        MarvDevice::MARV_DEVICE_LED,
        MarvDevice::MARV_DEVICE_ESC,
    ];

    const ALL_ERRORS: [ErrorKind; 7] = [
        ErrorKind::None,
        ErrorKind::Bus,
        ErrorKind::Timeout,
        ErrorKind::IdMismatch,
        ErrorKind::ConfigMismatch,
        ErrorKind::Por,
        ErrorKind::Stuck,
    ];

    /// The bits of the MAVLink standard, as literals (not read from the dialect).
    fn literal_bits(device: MarvDevice) -> u32 {
        match device {
            MarvDevice::MARV_DEVICE_IMU1 => 0x01 | 0x02,
            MarvDevice::MARV_DEVICE_IMU2 => 0x2_0000 | 0x4_0000,
            MarvDevice::MARV_DEVICE_MAG => 0x04,
            MarvDevice::MARV_DEVICE_BARO => 0x08,
            MarvDevice::MARV_DEVICE_GNSS => 0x20,
            MarvDevice::MARV_DEVICE_RC => 0x1_0000,
            MarvDevice::MARV_DEVICE_BATT => 0x200_0000,
            MarvDevice::MARV_DEVICE_SD => 0x100_0000,
            _ => 0,
        }
    }

    fn row(
        device: MarvDevice,
        present: bool,
        state: State,
        stuck: bool,
        locked: bool,
    ) -> DeviceRow {
        DeviceRow {
            device,
            present,
            state,
            stuck,
            locked,
        }
    }

    #[test]
    fn bit_mapping_matches_the_standard_literals() {
        for device in ALL_DEVICES {
            assert_eq!(sensor_bits(device), literal_bits(device), "{device:?}");
        }
        assert_eq!(sensor_bits(MarvDevice::MARV_DEVICE_IMU1), 0x03);
        assert_eq!(sensor_bits(MarvDevice::MARV_DEVICE_IMU2), 0x6_0000);
        assert_eq!(sensor_bits(MarvDevice::MARV_DEVICE_SD), 0x100_0000);
        assert_eq!(sensor_bits(MarvDevice::MARV_DEVICE_ESC), 0);
        for device in ALL_DEVICES {
            assert_eq!(sensor_bits(device) & 0x8_0000, 0);
        }
    }

    #[test]
    fn standard_literals_equal_the_dialect_constants() {
        let pairs = [
            (MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO, 0x01),
            (MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL, 0x02),
            (MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG, 0x04),
            (
                MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
                0x08,
            ),
            (MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_GPS, 0x20),
            (
                MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
                0x1_0000,
            ),
            (MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_GYRO2, 0x2_0000),
            (
                MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_ACCEL2,
                0x4_0000,
            ),
            (MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_3D_MAG2, 0x8_0000),
            (
                MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_BATTERY,
                0x200_0000,
            ),
            (MavSysStatusSensor::MAV_SYS_STATUS_LOGGING, 0x100_0000),
        ];
        for (flag, literal) in pairs {
            assert_eq!(flag.bits(), literal, "{flag:?}");
        }
    }

    /// A table of every device in a fixed mixed baseline.
    fn baseline() -> [DeviceRow; 11] {
        let mut rows = ALL_DEVICES.map(|d| row(d, true, State::Ok, false, true));
        for (i, r) in rows.iter_mut().enumerate() {
            r.state = ALL_STATES[i % 6];
            r.present = i % 2 == 0;
            r.stuck = i % 3 == 0;
            r.locked = i % 4 != 0;
        }
        rows
    }

    /// Checks I4.1 and the present/enabled rules exhaustively against `f`; returns the number of violations and the number of cases checked.
    fn check_exhaustive(f: fn(&[DeviceRow]) -> SensorMasks) -> (usize, usize) {
        let base = baseline();
        let mut violations = 0;
        let mut cases = 0;
        for (idx, device) in ALL_DEVICES.iter().enumerate() {
            let bits = literal_bits(*device);
            let mut rest = base;
            rest[idx].present = false;
            rest[idx].state = State::Absent;
            rest[idx].stuck = false;
            rest[idx].locked = false;
            let rest_masks = f(&rest);
            for state in ALL_STATES {
                for stuck in [false, true] {
                    for present in [false, true] {
                        for locked in [false, true] {
                            cases += 1;
                            let mut rows = rest;
                            rows[idx] = row(*device, present, state, stuck, locked);
                            let m = f(&rows);
                            let want_health = if *device == MarvDevice::MARV_DEVICE_RC {
                                locked
                            } else {
                                state == State::Ok && !stuck
                            };
                            let want_enabled = matches!(
                                state,
                                State::Starting | State::Ok | State::Degraded | State::Recovering
                            );
                            let own = |mask: u32| mask & bits;
                            let other = |mask: u32| mask & !bits;
                            let ok = own(m.health) == if want_health { bits } else { 0 }
                                && own(m.enabled) == if want_enabled { bits } else { 0 }
                                && own(m.present) == if present { bits } else { 0 }
                                && other(m.health) == other(rest_masks.health)
                                && other(m.enabled) == other(rest_masks.enabled)
                                && other(m.present) == other(rest_masks.present);
                            if !ok {
                                violations += 1;
                            }
                        }
                    }
                }
            }
        }
        (violations, cases)
    }

    #[test]
    fn i4_1_exhaustive() {
        let (violations, cases) = check_exhaustive(sensor_masks);
        assert_eq!(cases, 11 * 6 * 2 * 2 * 2);
        assert_eq!(violations, 0);
    }

    #[test]
    fn i4_1_negative_control_ignoring_stuck_is_caught() {
        fn ignores_stuck(row: &DeviceRow) -> bool {
            match row.device {
                MarvDevice::MARV_DEVICE_RC => row.locked,
                _ => row.state == State::Ok,
            }
        }
        fn buggy(rows: &[DeviceRow]) -> SensorMasks {
            masks_with(rows, ignores_stuck)
        }
        let (violations, _) = check_exhaustive(buggy);
        assert!(violations > 0);
        assert_eq!(violations, 7 * 4);
    }

    #[test]
    fn full_table_example() {
        let rows = [
            row(MarvDevice::MARV_DEVICE_IMU1, true, State::Ok, false, false),
            row(
                MarvDevice::MARV_DEVICE_IMU2,
                true,
                State::Degraded,
                false,
                false,
            ),
            row(
                MarvDevice::MARV_DEVICE_MAG,
                true,
                State::Recovering,
                false,
                false,
            ),
            row(MarvDevice::MARV_DEVICE_BARO, true, State::Ok, true, false),
            row(
                MarvDevice::MARV_DEVICE_GNSS,
                false,
                State::Starting,
                false,
                false,
            ),
            row(MarvDevice::MARV_DEVICE_RC, true, State::Ok, false, true),
            row(
                MarvDevice::MARV_DEVICE_BATT,
                true,
                State::Failed,
                false,
                false,
            ),
            row(MarvDevice::MARV_DEVICE_SD, true, State::Ok, false, false),
            row(
                MarvDevice::MARV_DEVICE_PSRAM,
                true,
                State::Failed,
                false,
                false,
            ),
            row(MarvDevice::MARV_DEVICE_LED, true, State::Ok, false, false),
            row(
                MarvDevice::MARV_DEVICE_ESC,
                false,
                State::Absent,
                false,
                false,
            ),
        ];
        let m = sensor_masks(&rows);
        assert_eq!(m.present, 0x0307_000F);
        assert_eq!(m.enabled, 0x0107_002F);
        assert_eq!(m.health, 0x0101_0003);
        assert_eq!(sensor_masks(&[]), SensorMasks::default());
    }

    #[test]
    fn system_status_rules() {
        let starting = [
            row(MarvDevice::MARV_DEVICE_IMU1, true, State::Ok, false, false),
            row(
                MarvDevice::MARV_DEVICE_GNSS,
                false,
                State::Starting,
                false,
                false,
            ),
        ];
        let left = [
            row(MarvDevice::MARV_DEVICE_IMU1, true, State::Ok, false, false),
            row(
                MarvDevice::MARV_DEVICE_GNSS,
                false,
                State::Absent,
                false,
                false,
            ),
        ];
        let boot = MavState::MAV_STATE_BOOT;
        let standby = MavState::MAV_STATE_STANDBY;
        let calib = MavState::MAV_STATE_CALIBRATING;
        assert_eq!(system_status(&starting, 0, false), boot);
        assert_eq!(system_status(&starting, 2_999, false), boot);
        assert_eq!(system_status(&starting, 3_000, false), standby);
        assert_eq!(system_status(&starting, 10_000, false), standby);
        assert_eq!(system_status(&left, 100, false), standby);
        assert_eq!(system_status(&left, 0, false), standby);
        assert_eq!(system_status(&left, 5_000, true), calib);
        assert_eq!(system_status(&starting, 100, true), boot);
        assert_eq!(system_status(&starting, 3_000, true), calib);
        for state in ALL_STATES {
            let rows = [row(MarvDevice::MARV_DEVICE_IMU1, true, state, true, true)];
            for now in [0, 2_999, 3_000, 1_000_000] {
                for cal in [false, true] {
                    let s = system_status(&rows, now, cal);
                    assert!(s == boot || s == standby || s == calib);
                }
            }
        }
    }

    fn input(state: State, counters: Counters, stuck: bool) -> StatusInput {
        StatusInput {
            device: MarvDevice::MARV_DEVICE_MAG,
            state,
            counters,
            stuck,
            samples: 0,
            last_sample_age_ms: None,
            hil: false,
            self_test: false,
            time_boot_ms: 0,
        }
    }

    #[test]
    fn state_mapping_numbers() {
        let want = [
            (State::Absent, 0),
            (State::Starting, 1),
            (State::Ok, 2),
            (State::Degraded, 3),
            (State::Recovering, 4),
            (State::Failed, 5),
        ];
        for (state, number) in want {
            let r = device_status(&input(state, Counters::default(), false));
            assert_eq!(r.state as u8, number, "{state:?}");
        }
    }

    #[test]
    fn error_mapping_numbers() {
        let want = [0u8, 1, 2, 3, 4, 5, 6];
        for (kind, number) in ALL_ERRORS.into_iter().zip(want) {
            let counters = Counters {
                last_error: kind,
                ..Counters::default()
            };
            let r = device_status(&input(State::Degraded, counters, false));
            assert_eq!(r.last_error as u8, number, "{kind:?}");
        }
    }

    #[test]
    fn device_id_numbers() {
        let want = [1u8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12];
        for (device, number) in ALL_DEVICES.into_iter().zip(want) {
            let mut i = input(State::Ok, Counters::default(), false);
            i.device = device;
            assert_eq!(device_status(&i).device_id as u8, number);
        }
    }

    #[test]
    fn stuck_is_failed_with_error_stuck() {
        for state in ALL_STATES {
            for kind in ALL_ERRORS {
                let counters = Counters {
                    last_error: kind,
                    ..Counters::default()
                };
                let r = device_status(&input(state, counters, true));
                assert_eq!(r.state as u8, 5, "{state:?}");
                assert_eq!(r.last_error as u8, 6, "{state:?}");
            }
        }
    }

    #[test]
    fn counters_saturate_and_copy() {
        let counters = Counters {
            errors: 70_000,
            timeouts: 0xFFFF_FFFF,
            recoveries: 65_535,
            attempts: 255,
            config_restored: 65_536,
            overflows: 1_000_000,
            ..Counters::default()
        };
        let mut i = input(State::Ok, counters, false);
        i.samples = 0xDEAD_BEEF;
        i.time_boot_ms = 123_456;
        let r = device_status(&i);
        assert_eq!(r.errors, 70_000);
        assert_eq!(r.timeouts, 0xFFFF_FFFF);
        assert_eq!(r.samples, 0xDEAD_BEEF);
        assert_eq!(r.time_boot_ms, 123_456);
        assert_eq!(r.recoveries, 65_535);
        assert_eq!(r.attempts, 255);
        assert_eq!(r.config_restored, 65_535);
        assert_eq!(r.overflows, 65_535);
        let small = Counters {
            recoveries: 65_534,
            config_restored: 7,
            overflows: 0,
            ..Counters::default()
        };
        let r = device_status(&input(State::Ok, small, false));
        assert_eq!(r.recoveries, 65_534);
        assert_eq!(r.config_restored, 7);
        assert_eq!(r.overflows, 0);
    }

    #[test]
    fn each_sixteen_bit_field_saturates() {
        let big = 0x1_0000_u32;
        let c = Counters {
            recoveries: big,
            ..Counters::default()
        };
        assert_eq!(
            device_status(&input(State::Ok, c, false)).recoveries,
            65_535
        );
        let c = Counters {
            overflows: big,
            ..Counters::default()
        };
        assert_eq!(device_status(&input(State::Ok, c, false)).overflows, 65_535);
        let c = Counters {
            config_restored: big,
            ..Counters::default()
        };
        assert_eq!(
            device_status(&input(State::Ok, c, false)).config_restored,
            65_535
        );
    }

    #[test]
    fn age_mapping() {
        let age = |a: Option<u32>| {
            let mut i = input(State::Ok, Counters::default(), false);
            i.last_sample_age_ms = a;
            device_status(&i).last_sample_age_ms
        };
        assert_eq!(age(None), 65_535);
        assert_eq!(age(Some(0)), 0);
        assert_eq!(age(Some(65_533)), 65_533);
        assert_eq!(age(Some(65_534)), 65_534);
        assert_eq!(age(Some(65_535)), 65_534);
        assert_eq!(age(Some(70_000)), 65_534);
        assert_eq!(age(Some(u32::MAX)), 65_534);
    }

    #[test]
    fn flags_bits() {
        let flags = |hil: bool, self_test: bool| {
            let mut i = input(State::Ok, Counters::default(), false);
            i.hil = hil;
            i.self_test = self_test;
            device_status(&i).flags
        };
        assert_eq!(flags(false, false), 0);
        assert_eq!(flags(true, false), 1);
        assert_eq!(flags(false, true), 2);
        assert_eq!(flags(true, true), 3);
    }
}
