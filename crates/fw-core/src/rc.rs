//! The radio receiver's row (RC): whether the radio link is up, from what the receiver port
//! reports. Report only: no health machine, no recovery (the port recovers by itself).
//!
//! - CRSF lock with a stick report younger than [`FRESH_US`]: Ok, link up.
//! - MAVLink lock with a RADIO_STATUS younger than [`FRESH_US`] whose `rssi` is above 0: Ok, link up.
//!   The receiver keeps sending RADIO_STATUS with `rssi` 0 while the transmitter is off, so the
//!   lock alone does not mean the link is up.
//! - Either lock otherwise: Degraded, last error Timeout, link down.
//! - Searching: Starting until the first lock since boot, then Degraded; link down.
//!
//! "Link up" is the row's health (SYS_STATUS RC_RECEIVER): it is what the firmware puts into
//! [`DeviceRow::locked`](crate::sys_status::DeviceRow::locked).

use crate::health::{ErrorKind, State};

/// A stick report (CRSF) or RADIO_STATUS (MAVLink) older than this does not count, in us.
pub const FRESH_US: u64 = 1_000_000;

/// The port's state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PortMode {
    /// Not locked, alternating between the two bauds.
    Searching,
    /// Locked on CRSF.
    Crsf,
    /// Locked on MAVLink.
    Mavlink,
}

/// What the port reported, as the row needs it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RcInput {
    /// The port's state now.
    pub mode: PortMode,
    /// The port has locked at least once since boot.
    pub ever_locked: bool,
    /// Now, on the clock of the timestamps below, in us.
    pub now_us: u64,
    /// When the latest stick report was read, in us.
    pub last_sticks_us: Option<u64>,
    /// The latest RADIO_STATUS: its `rssi` and when it was read, in us.
    pub radio_status: Option<(u8, u64)>,
}

/// The row's state, whether the radio link is up (the health bit), and its last error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RcVerdict {
    /// The row's state.
    pub state: State,
    /// The radio link is up.
    pub link_up: bool,
    /// The row's last error.
    pub last_error: ErrorKind,
}

/// The rule of the module documentation.
pub fn verdict(input: &RcInput) -> RcVerdict {
    let fresh = |t_us: u64| input.now_us.saturating_sub(t_us) < FRESH_US;
    let locked = |up: bool| RcVerdict {
        state: if up { State::Ok } else { State::Degraded },
        link_up: up,
        last_error: if up {
            ErrorKind::None
        } else {
            ErrorKind::Timeout
        },
    };
    match input.mode {
        PortMode::Crsf => locked(input.last_sticks_us.is_some_and(fresh)),
        PortMode::Mavlink => locked(
            input
                .radio_status
                .is_some_and(|(rssi, t_us)| rssi > 0 && fresh(t_us)),
        ),
        PortMode::Searching => RcVerdict {
            state: if input.ever_locked {
                State::Degraded
            } else {
                State::Starting
            },
            link_up: false,
            last_error: ErrorKind::None,
        },
    }
}

/// The next MARV_DEVICE_STATUS row on the radio: `changed` (a row whose state differs from the
/// state last sent) goes out of turn, but never twice in a row; otherwise the row at `cursor` goes
/// in turn. Returns (row, new cursor, whether this message was out of turn).
pub fn pick_row(
    changed: Option<usize>,
    cursor: usize,
    rows: usize,
    last_out_of_turn: bool,
) -> (usize, usize, bool) {
    match changed {
        Some(row) if !last_out_of_turn => (row, cursor, true),
        _ => {
            let row = cursor.checked_rem(rows).unwrap_or(0);
            (row, row.saturating_add(1), false)
        }
    }
}

/// The sticks that may feed MANUAL_CONTROL: the latest stick report (its channels in us and when it
/// was read) only when it came from CRSF and is younger than [`FRESH_US`] (strictly). A report from
/// a MAVLink RC_CHANNELS_OVERRIDE never does: "CRSF is for sticks".
pub fn crsf_sticks(
    latest: Option<([u16; 16], u64)>,
    from_crsf: bool,
    now_us: u64,
) -> Option<[u16; 16]> {
    let (channels, at_us) = latest?;
    (from_crsf && now_us.saturating_sub(at_us) < FRESH_US).then_some(channels)
}

/// The values of a CRSF GPS frame (type 0x02), in the units of its `Gps` struct in `crsf-codec`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CrsfGps {
    /// Degrees times 1e7.
    pub latitude_e7: i32,
    /// Degrees times 1e7.
    pub longitude_e7: i32,
    /// 0.1 km/h.
    pub groundspeed_dkmh: u16,
    /// 0.01 degree.
    pub heading_cdeg: u16,
    /// Metres above mean sea level, within [`CRSF_MIN_ALTITUDE_M`] to [`CRSF_MAX_ALTITUDE_M`].
    pub altitude_m: i32,
    /// Satellites used.
    pub satellites: u8,
}

/// Lowest altitude sent, in m: Betaflight's wire value 0 (`betaflight-telemetry-crsf.c`, `crsfFrameGps`:
/// `constrain(altCm / 100 + 1000, 0, 5000)`).
pub const CRSF_MIN_ALTITUDE_M: i32 = -1000;
/// Highest altitude sent, in m: Betaflight's wire value 5000.
pub const CRSF_MAX_ALTITUDE_M: i32 = 4000;

/// The GNSS sample's values as a CRSF GPS frame's: speed mm/s to 0.1 km/h rounded half up
/// (`(mm_s * 36 + 500) / 1000`, Betaflight's `(cm_s * 36 + 50) / 100` from a finer unit, clamped to
/// `u16`), altitude mm to whole metres (floor) clamped to -1000..=4000 m as Betaflight does.
///
/// Before a fix, and after a loss, the frame carries zeros (as Betaflight and ArduPilot send their
/// zeroed GPS state before a first fix); unlike them it does not hold the last position after a loss.
/// Every `None` is sent as 0 (an altitude of 0 m, satellites 0).
pub fn crsf_gps(
    lat_e7: Option<i32>,
    lon_e7: Option<i32>,
    alt_mm: Option<i32>,
    speed_mm_s: Option<u32>,
    course_cdeg: Option<u16>,
    sats_used: Option<u8>,
) -> CrsfGps {
    let speed_dkmh = (u64::from(speed_mm_s.unwrap_or(0)) * 36 + 500) / 1000;
    CrsfGps {
        latitude_e7: lat_e7.unwrap_or(0),
        longitude_e7: lon_e7.unwrap_or(0),
        groundspeed_dkmh: u16::try_from(speed_dkmh).unwrap_or(u16::MAX),
        heading_cdeg: course_cdeg.unwrap_or(0),
        altitude_m: alt_mm
            .unwrap_or(0)
            .div_euclid(1000)
            .clamp(CRSF_MIN_ALTITUDE_M, CRSF_MAX_ALTITUDE_M),
        satellites: sats_used.unwrap_or(0),
    }
}

/// Whether the GNSS row's latest sample (read at `sample_t_us`) is newer than the one last sent as a
/// GPS frame (`last_sent_us`; `None` before the first): then this call of the telemetry hook sends
/// the GPS frame (fix or no fix), otherwise the battery frame. Betaflight's "on each new solution".
pub fn gps_frame_due(sample_t_us: Option<u64>, last_sent_us: Option<u64>) -> bool {
    match (sample_t_us, last_sent_us) {
        (Some(t), Some(sent)) => t > sent,
        (Some(_), None) => true,
        (None, _) => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sys_status::{DeviceRow, sensor_masks};
    use mavlink_device::dialect::MavSysStatusSensor;
    use mavlink_device::marv::MarvDevice;

    const NOW: u64 = 100_000_000;
    const FRESH: u64 = NOW - (FRESH_US - 1);
    const STALE: u64 = NOW - FRESH_US;

    /// (mode, ever locked, last stick report, latest RADIO_STATUS) -> (state, link up, last error)
    type Row = (
        PortMode,
        bool,
        Option<u64>,
        Option<(u8, u64)>,
        State,
        bool,
        ErrorKind,
    );

    #[test]
    fn the_rule_table() {
        use ErrorKind::{None as NoError, Timeout};
        use PortMode::{Crsf, Mavlink, Searching};
        use State::{Degraded, Ok, Starting};
        // (mode, ever locked, last stick report, latest RADIO_STATUS) -> (state, link up, last error)
        #[rustfmt::skip]
        let table: [Row; 20] = [
            (Searching, false, None, None, Starting, false, NoError),
            (Searching, false, Some(FRESH), Some((200, FRESH)), Starting, false, NoError),
            (Searching, true, None, None, Degraded, false, NoError),
            (Searching, true, Some(FRESH), Some((200, FRESH)), Degraded, false, NoError),
            (Crsf, true, Some(FRESH), None, Ok, true, NoError),
            (Crsf, true, Some(NOW), None, Ok, true, NoError),
            (Crsf, true, Some(NOW + 5), None, Ok, true, NoError),
            (Crsf, true, Some(STALE), None, Degraded, false, Timeout),
            (Crsf, true, None, None, Degraded, false, Timeout),
            // RADIO_STATUS does not make a CRSF link up.
            (Crsf, true, None, Some((200, FRESH)), Degraded, false, Timeout),
            (Mavlink, true, None, Some((1, FRESH)), Ok, true, NoError),
            (Mavlink, true, None, Some((254, NOW)), Ok, true, NoError),
            (Mavlink, true, Some(FRESH), Some((254, FRESH)), Ok, true, NoError),
            // The transmitter off: RADIO_STATUS goes on with rssi 0.
            (Mavlink, true, None, Some((0, NOW)), Degraded, false, Timeout),
            (Mavlink, true, Some(FRESH), Some((0, FRESH)), Degraded, false, Timeout),
            (Mavlink, true, None, Some((254, STALE)), Degraded, false, Timeout),
            (Mavlink, true, None, None, Degraded, false, Timeout),
            // Stick reports do not make a MAVLink link up.
            (Mavlink, true, Some(FRESH), None, Degraded, false, Timeout),
            (Mavlink, true, Some(FRESH), Some((254, STALE)), Degraded, false, Timeout),
            (Crsf, true, Some(FRESH), Some((0, FRESH)), Ok, true, NoError),
        ];
        for (i, &(mode, ever_locked, sticks, radio, state, link_up, last_error)) in
            table.iter().enumerate()
        {
            let input = RcInput {
                mode,
                ever_locked,
                now_us: NOW,
                last_sticks_us: sticks,
                radio_status: radio,
            };
            assert_eq!(
                verdict(&input),
                RcVerdict {
                    state,
                    link_up,
                    last_error
                },
                "row {i}"
            );
        }
    }

    #[test]
    fn the_health_bit_follows_the_link_not_the_lock() {
        let bit = MavSysStatusSensor::MAV_SYS_STATUS_SENSOR_RC_RECEIVER.bits();
        for (rssi, up) in [(0u8, false), (90, true)] {
            let v = verdict(&RcInput {
                mode: PortMode::Mavlink,
                ever_locked: true,
                now_us: NOW,
                last_sticks_us: None,
                radio_status: Some((rssi, NOW)),
            });
            let masks = sensor_masks(&[DeviceRow {
                device: MarvDevice::MARV_DEVICE_RC,
                present: true,
                state: v.state,
                stuck: false,
                locked: v.link_up,
            }]);
            assert_eq!(masks.health & bit != 0, up, "rssi {rssi}");
            assert_ne!(masks.present & bit, 0);
            assert_ne!(masks.enabled & bit, 0, "Degraded is still enabled");
        }
    }

    #[test]
    fn a_changed_row_takes_at_most_every_other_message() {
        const ROWS: usize = 7;
        let (mut cursor, mut out_of_turn) = (0, false);
        let mut seen = [false; ROWS];
        for _ in 0..2 * ROWS {
            let (row, next, oot) = pick_row(Some(3), cursor, ROWS, out_of_turn);
            seen[row] = true;
            (cursor, out_of_turn) = (next, oot);
        }
        assert_eq!(seen, [true; ROWS]);
        let (mut cursor, mut out_of_turn) = (0, false);
        for n in 0..3 * ROWS {
            let (row, next, oot) = pick_row(None, cursor, ROWS, out_of_turn);
            assert_eq!((row, oot), (n % ROWS, false));
            (cursor, out_of_turn) = (next, oot);
        }
    }

    #[test]
    fn only_fresh_crsf_sticks_feed_manual_control() {
        let ch = [1500u16; 16];
        // (latest report, from CRSF, now) -> sticks
        assert_eq!(
            crsf_sticks(Some((ch, FRESH)), true, NOW),
            Some(ch),
            "fresh CRSF"
        );
        assert_eq!(
            crsf_sticks(Some((ch, NOW)), true, NOW),
            Some(ch),
            "just read"
        );
        assert_eq!(
            crsf_sticks(Some((ch, FRESH)), false, NOW),
            None,
            "fresh override"
        );
        assert_eq!(
            crsf_sticks(Some((ch, NOW)), false, NOW),
            None,
            "override just read"
        );
        assert_eq!(
            crsf_sticks(Some((ch, STALE)), true, NOW),
            None,
            "stale CRSF, exactly 1 s"
        );
        assert_eq!(crsf_sticks(Some((ch, 0)), true, NOW), None, "old CRSF");
        assert_eq!(
            crsf_sticks(Some((ch, STALE)), false, NOW),
            None,
            "stale override"
        );
        assert_eq!(crsf_sticks(None, true, NOW), None, "no report");
        assert_eq!(
            crsf_sticks(Some((ch, NOW + 5)), true, NOW),
            Some(ch),
            "clock behind"
        );
    }

    #[test]
    fn crsf_gps_converts_a_made_up_sample() {
        // 45.0 N, 9.0 E (made up), 123.456 m, 5.5 m/s = 19.8 km/h, course 270.25, 9 satellites.
        let g = crsf_gps(
            Some(450_000_000),
            Some(90_000_000),
            Some(123_456),
            Some(5_500),
            Some(27_025),
            Some(9),
        );
        assert_eq!(
            g,
            CrsfGps {
                latitude_e7: 450_000_000,
                longitude_e7: 90_000_000,
                groundspeed_dkmh: 198,
                heading_cdeg: 27_025,
                altitude_m: 123,
                satellites: 9,
            }
        );
        let south_west = crsf_gps(
            Some(-450_000_000),
            Some(-90_000_000),
            None,
            None,
            None,
            None,
        );
        assert_eq!(
            (south_west.latitude_e7, south_west.longitude_e7),
            (-450_000_000, -90_000_000)
        );
    }

    #[test]
    fn crsf_gps_clamps_altitude_like_betaflight() {
        let alt = |mm| crsf_gps(None, None, Some(mm), None, None, None).altitude_m;
        assert_eq!(alt(-1_001_000), -1000);
        assert_eq!(alt(-1_000_000), -1000);
        assert_eq!(alt(4_000_000), 4000);
        assert_eq!(alt(4_001_000), 4000);
        assert_eq!(alt(i32::MAX), 4000);
        assert_eq!(alt(i32::MIN), -1000);
        assert_eq!(alt(-1), -1, "floor, not truncation");
        assert_eq!(alt(999), 0);
    }

    #[test]
    fn crsf_gps_rounds_speed_half_up_and_clamps() {
        let sp = |mm_s| crsf_gps(None, None, None, Some(mm_s), None, None).groundspeed_dkmh;
        assert_eq!(sp(0), 0);
        assert_eq!(sp(138), 5, "138 mm/s = 4.968 dkmh");
        assert_eq!(sp(13), 0, "0.468 dkmh rounds down");
        assert_eq!(sp(14), 1, "0.504 dkmh rounds up");
        assert_eq!(sp(5_500), 198);
        assert_eq!(sp(u32::MAX), u16::MAX);
    }

    #[test]
    fn crsf_gps_without_a_fix_carries_zeros() {
        let g = crsf_gps(None, None, None, None, None, Some(3));
        assert_eq!(
            g,
            CrsfGps {
                latitude_e7: 0,
                longitude_e7: 0,
                groundspeed_dkmh: 0,
                heading_cdeg: 0,
                altitude_m: 0,
                satellites: 3,
            }
        );
        assert_eq!(crsf_gps(None, None, None, None, None, None).satellites, 0);
    }

    #[test]
    fn gps_frame_is_due_once_per_new_sample() {
        assert!(gps_frame_due(Some(5), None), "first sample");
        assert!(
            gps_frame_due(Some(2_000_000), Some(1_000_000)),
            "newer sample"
        );
        assert!(
            !gps_frame_due(Some(1_000_000), Some(1_000_000)),
            "already sent"
        );
        assert!(
            !gps_frame_due(Some(500_000), Some(1_000_000)),
            "older than the one sent"
        );
        assert!(!gps_frame_due(None, None), "no sample");
        assert!(!gps_frame_due(None, Some(1)), "no sample");
    }
}
