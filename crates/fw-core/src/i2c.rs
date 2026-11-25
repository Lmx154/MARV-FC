//! The I2C transfer deadline and the rule for a bus after a timed-out transfer, as ArduPilot's
//! `AP_HAL_ChibiOS/I2CDevice.cpp` (`I2CDevice::_transfer`): every transfer runs under a deadline
//! of twice its time on the wire, at least 4 ms; a transfer that misses it leaves the bus
//! refused until the bus is cleared and rebuilt.

use core::sync::atomic::{AtomicBool, Ordering};

/// ArduPilot's floor: the `timeout_ms = 4` default of `I2CDeviceManager::get_device_ptr`.
pub const FLOOR_MS: u32 = 4;

/// The deadline in ms of one transfer of `bytes` data bytes (sent plus received) at `bus_hz`: ArduPilot's `1 + 2 * (((8 * 1000000 / busclock) * (send_len + recv_len)) / 1000)`, at least [`FLOOR_MS`].
pub fn timeout_ms(bytes: usize, bus_hz: u32) -> u32 {
    let us_per_byte = 8_000_000u32.checked_div(bus_hz).unwrap_or(u32::MAX);
    let bytes = u32::try_from(bytes).unwrap_or(u32::MAX);
    let wire_ms = us_per_byte.saturating_mul(bytes) / 1000;
    wire_ms.saturating_mul(2).saturating_add(1).max(FLOOR_MS)
}

/// Whether a bus may be used: yes from its (re)build until one of its transfers timed out, then no until it is rebuilt (the timed-out transfer was abandoned, so the controller's state is unknown).
#[derive(Debug, Default)]
pub struct Gate {
    dirty: AtomicBool,
}

impl Gate {
    /// A gate for a bus not yet built: clean.
    pub const fn new() -> Self {
        Self {
            dirty: AtomicBool::new(false),
        }
    }

    /// True from a timed-out transfer until [`rebuilt`](Self::rebuilt).
    pub fn dirty(&self) -> bool {
        self.dirty.load(Ordering::Relaxed)
    }

    /// A transfer on the bus timed out and was abandoned.
    pub fn timed_out(&self) {
        self.dirty.store(true, Ordering::Relaxed);
    }

    /// The bus was cleared and rebuilt.
    pub fn rebuilt(&self) {
        self.dirty.store(false, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn short_transfers_at_400_khz_get_the_floor() {
        for bytes in [0, 1, 2, 7, 15, 34, 49] {
            assert_eq!(timeout_ms(bytes, 400_000), FLOOR_MS, "{bytes} bytes");
        }
    }

    #[test]
    fn long_transfers_follow_ardupilots_formula() {
        assert_eq!(timeout_ms(100, 400_000), 5);
        assert_eq!(timeout_ms(250, 400_000), 11);
        assert_eq!(timeout_ms(25, 100_000), 5);
        assert_eq!(timeout_ms(12, 100_000), FLOOR_MS);
        assert_eq!(timeout_ms(1000, 1_000_000), 17);
    }

    #[test]
    fn extremes_saturate() {
        let most = 2 * (u32::MAX / 1000) + 1;
        assert_eq!(timeout_ms(1, 0), most);
        assert_eq!(timeout_ms(usize::MAX, 400_000), most);
        assert_eq!(timeout_ms(0, 0), FLOOR_MS);
    }

    #[test]
    fn a_timed_out_bus_is_refused_until_rebuilt() {
        let gate = Gate::new();
        assert!(!gate.dirty());
        gate.timed_out();
        for _ in 0..3 {
            assert!(gate.dirty());
        }
        gate.timed_out();
        assert!(gate.dirty());
        gate.rebuilt();
        assert!(!gate.dirty());
        assert!(!Gate::default().dirty());
    }
}
