//! A device self-test (`MAV_CMD_MARV_DEVICE` action 2) as its result crosses from the device's task
//! to the action: the [`Verdict`] and the failed axes as a bit mask, never figures; the final result
//! of the action; and its one STATUSTEXT.
//!
//! Failed-axis mask: IMU1 bits 0..=2 accelerometer X, Y, Z and 3..=5 gyroscope X, Y, Z; IMU2 bits
//! 0..=2 low-g X, Y, Z, 3..=5 gyroscope, 6..=8 high-g; MAG bits 0..=3 X+, X-, Y+, Y- (the order the
//! BMM350 runs them). [`axes`] builds the bits of one three-axis channel.

use core::fmt;

use mavlink_device::marv::{MarvDevice, MavResult};

use crate::health::ErrorKind;

/// What one self-test found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    /// Every tested axis passed.
    Pass,
    /// The test completed and did not pass; the failed axes (module documentation), 0 when the
    /// driver names none.
    Fail(u16),
    /// The driver returned an error of this kind; nothing was judged.
    Error(ErrorKind),
    /// The device has no self-test.
    Unsupported,
}

/// A finished self-test as the device's task publishes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Outcome {
    /// The request's number (the device's restart numbering: the restart that follows the test
    /// carries the same number).
    pub number: u32,
    /// What the test found.
    pub verdict: Verdict,
    /// How long the driver's test took, us.
    pub took_us: u32,
}

impl Outcome {
    /// The action's final result: ACCEPTED only when the test passed and the device came back Ok (`device_ok`, the outcome of the restart with the same number), FAILED otherwise.
    pub fn result(&self, device_ok: bool) -> MavResult {
        if self.verdict == Verdict::Pass && device_ok {
            MavResult::MAV_RESULT_ACCEPTED
        } else {
            MavResult::MAV_RESULT_FAILED
        }
    }
}

/// The mask bits of three-axis channel `channel` (0 based, in the device's order) for the axes
/// that failed.
pub const fn axes(channel: u8, x_failed: bool, y_failed: bool, z_failed: bool) -> u16 {
    let bits = (x_failed as u16) | ((y_failed as u16) << 1) | ((z_failed as u16) << 2);
    bits << (3 * channel)
}

/// The channel names of a three-axis device, in mask order.
fn channels(device: MarvDevice) -> &'static [&'static str] {
    match device {
        MarvDevice::MARV_DEVICE_IMU1 => &["accel", "gyro"],
        MarvDevice::MARV_DEVICE_IMU2 => &["lowg", "gyro", "highg"],
        _ => &[],
    }
}

/// The magnetometer's entries, in mask order.
const MAG_ENTRIES: [&str; 4] = ["X+", "X-", "Y+", "Y-"];

/// The self-test's STATUSTEXT (at most 50 bytes for the names in use): `IMU1 self-test: PASS`, `IMU2 self-test FAIL: lowg X gyro YZ`, `MAG self-test FAIL: X+ Y-`, `IMU1 self-test: error bus`.
pub struct Text<'a> {
    /// The device, for its mask layout.
    pub device: MarvDevice,
    /// Its name in the text.
    pub name: &'a str,
    /// What the test found.
    pub verdict: Verdict,
    /// The name of an error kind (the firmware's names).
    pub error_name: fn(ErrorKind) -> &'static str,
}

impl fmt::Display for Text<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = self.name;
        let mask = match self.verdict {
            Verdict::Pass => return write!(f, "{name} self-test: PASS"),
            Verdict::Error(kind) => {
                return write!(f, "{name} self-test: error {}", (self.error_name)(kind));
            }
            Verdict::Unsupported => return write!(f, "{name} self-test: not supported"),
            Verdict::Fail(mask) => mask,
        };
        write!(f, "{name} self-test FAIL")?;
        let mut first = true;
        let mut sep = |f: &mut fmt::Formatter<'_>| {
            let s = if first { ": " } else { " " };
            first = false;
            f.write_str(s)
        };
        if self.device == MarvDevice::MARV_DEVICE_MAG {
            for (i, entry) in MAG_ENTRIES.iter().enumerate() {
                if mask & (1 << i) != 0 {
                    sep(f)?;
                    f.write_str(entry)?;
                }
            }
            return Ok(());
        }
        for (i, channel) in channels(self.device).iter().enumerate() {
            let bits = (mask >> (3 * i)) & 0b111;
            if bits == 0 {
                continue;
            }
            sep(f)?;
            f.write_str(channel)?;
            f.write_str(" ")?;
            for (b, axis) in ["X", "Y", "Z"].iter().enumerate() {
                if bits & (1 << b) != 0 {
                    f.write_str(axis)?;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::format;
    use std::string::String;

    fn names(kind: ErrorKind) -> &'static str {
        match kind {
            ErrorKind::Bus => "bus",
            ErrorKind::ConfigMismatch => "config mismatch",
            _ => "other",
        }
    }

    fn text(device: MarvDevice, name: &str, verdict: Verdict) -> String {
        format!(
            "{}",
            Text {
                device,
                name,
                verdict,
                error_name: names
            }
        )
    }

    const IMU1: MarvDevice = MarvDevice::MARV_DEVICE_IMU1;
    const IMU2: MarvDevice = MarvDevice::MARV_DEVICE_IMU2;
    const MAG: MarvDevice = MarvDevice::MARV_DEVICE_MAG;

    #[test]
    fn axes_put_three_bits_per_channel() {
        assert_eq!(axes(0, true, false, false), 0b1);
        assert_eq!(axes(0, false, true, true), 0b110);
        assert_eq!(axes(1, false, true, true), 0b110_000);
        assert_eq!(axes(2, true, true, true), 0b111_000_000);
        assert_eq!(axes(1, false, false, false), 0);
    }

    #[test]
    fn pass_and_error_texts() {
        assert_eq!(text(MAG, "MAG", Verdict::Pass), "MAG self-test: PASS");
        assert_eq!(text(IMU1, "IMU1", Verdict::Pass), "IMU1 self-test: PASS");
        assert_eq!(
            text(IMU2, "IMU2", Verdict::Error(ErrorKind::Bus)),
            "IMU2 self-test: error bus"
        );
        assert_eq!(
            text(IMU1, "IMU1", Verdict::Error(ErrorKind::ConfigMismatch)),
            "IMU1 self-test: error config mismatch"
        );
        assert_eq!(
            text(MarvDevice::MARV_DEVICE_BARO, "BARO", Verdict::Unsupported),
            "BARO self-test: not supported"
        );
    }

    #[test]
    fn a_failed_test_names_its_channels_and_axes() {
        let mask = axes(0, true, false, false) | axes(1, false, true, true);
        assert_eq!(
            text(IMU2, "IMU2", Verdict::Fail(mask)),
            "IMU2 self-test FAIL: lowg X gyro YZ"
        );
        assert_eq!(
            text(IMU1, "IMU1", Verdict::Fail(mask)),
            "IMU1 self-test FAIL: accel X gyro YZ"
        );
        assert_eq!(
            text(MAG, "MAG", Verdict::Fail(0b1001)),
            "MAG self-test FAIL: X+ Y-"
        );
        assert_eq!(
            text(IMU2, "IMU2", Verdict::Fail(axes(2, false, false, true))),
            "IMU2 self-test FAIL: highg Z"
        );
        assert_eq!(
            text(IMU1, "IMU1", Verdict::Fail(axes(2, true, true, true))),
            "IMU1 self-test FAIL"
        );
        assert_eq!(text(MAG, "MAG", Verdict::Fail(0)), "MAG self-test FAIL");
    }

    #[test]
    fn every_text_fits_a_statustext() {
        let all = text(IMU2, "IMU2", Verdict::Fail(0x1FF));
        assert_eq!(all, "IMU2 self-test FAIL: lowg XYZ gyro XYZ highg XYZ");
        assert!(all.len() <= 50);
        assert!(text(IMU1, "IMU1", Verdict::Fail(0x3F)).len() <= 50);
        assert!(text(MAG, "MAG", Verdict::Fail(0xF)).len() <= 50);
        assert!(text(IMU1, "IMU1", Verdict::Error(ErrorKind::ConfigMismatch)).len() <= 50);
    }

    #[test]
    fn accepted_only_on_pass_with_the_device_back_ok() {
        let outcome = |verdict| Outcome {
            number: 3,
            verdict,
            took_us: 1,
        };
        let ok = MavResult::MAV_RESULT_ACCEPTED;
        let failed = MavResult::MAV_RESULT_FAILED;
        assert_eq!(outcome(Verdict::Pass).result(true), ok);
        assert_eq!(outcome(Verdict::Pass).result(false), failed);
        assert_eq!(outcome(Verdict::Fail(1)).result(true), failed);
        assert_eq!(outcome(Verdict::Error(ErrorKind::Bus)).result(true), failed);
        assert_eq!(outcome(Verdict::Unsupported).result(true), failed);
    }
}
