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
}
