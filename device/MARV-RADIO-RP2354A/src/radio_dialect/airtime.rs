//! Airtime-aware downlink budget for the LoRa air link.
//!
//! The vehicle radio can only put a few frames per second on the air (one max-size frame is
//! ~100 ms at the slower profiles). This module turns the *live* LoRa profile into a concrete
//! send period per telemetry class, so each sensor gets airtime in proportion to its update
//! rate, bounded by what the RF settings can actually carry.
//!
//! Every period is derived from [`LoraProfile::time_on_air_us`], so changing the RF profile
//! (spreading factor / bandwidth / coding rate) automatically rescales all periods with no
//! manual retuning: a slower profile inflates every per-frame airtime, which inflates the total
//! demand, which shrinks the scale factor, which lengthens every class period proportionally.

use common::comms::links::lora::LoraProfile;
use common::comms::links::lora::frame::FRAME_HEADER_LEN;
use common::protocol::hilink;

use crate::radio_dialect::rf::{RfWirePayload, encoded_rf_len};

/// Budget-gated periodic downlink classes. Command-acks / events / faults are intentionally
/// *not* here — they are rare, critical, and preempt the budget (see `scheduler.rs`).
#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub enum DownlinkClass {
    Flight,
    Imu1,
    Imu2,
    Mag,
    Baro,
    Gps,
    LinkStatus,
}

pub const BUDGETED_CLASS_COUNT: usize = 7;

/// Per-class tuning input: ideal send rate in milli-Hz (the only manual knob — reflects each
/// sensor's natural cadence) and the RF dialect wire length used to price its airtime.
struct ClassSpec {
    ideal_rate_mhz: u32,
    wire_len: usize,
}

/// Order MUST match the `DownlinkClass` discriminants — entries are indexed by `class as usize`.
fn class_specs() -> [ClassSpec; BUDGETED_CLASS_COUNT] {
    [
        ClassSpec { ideal_rate_mhz: 5_000, wire_len: rf_wire_len::<hilink::LoRaFlightSnapshotPayload>() }, // Flight  5 Hz
        ClassSpec { ideal_rate_mhz: 10_000, wire_len: rf_wire_len::<hilink::LoRaImu1SnapshotPayload>() },   // Imu1   10 Hz
        ClassSpec { ideal_rate_mhz: 10_000, wire_len: rf_wire_len::<hilink::LoRaImu2SnapshotPayload>() },   // Imu2   10 Hz
        ClassSpec { ideal_rate_mhz: 5_000, wire_len: rf_wire_len::<hilink::LoRaMagSnapshotPayload>() },     // Mag     5 Hz
        ClassSpec { ideal_rate_mhz: 2_000, wire_len: rf_wire_len::<hilink::LoRaBaroSnapshotPayload>() },    // Baro    2 Hz
        ClassSpec { ideal_rate_mhz: 1_000, wire_len: rf_wire_len::<hilink::LoRaGpsSnapshotPayload>() },     // Gps     1 Hz
        ClassSpec { ideal_rate_mhz: 500, wire_len: rf_wire_len::<hilink::LoRaLinkStatusPayload>() },        // Link  0.5 Hz
    ]
}

const fn rf_wire_len<P: RfWirePayload>() -> usize {
    P::WIRE_LEN
}

/// Fraction of wall-clock time the scheduled downlink stream may occupy. The link is
/// half-duplex (one TX then one RX window per loop), so scheduled telemetry must leave room
/// for RX windows, keepalives, and ungated critical acks/events/faults. 30% is a conservative
/// reserve that keeps the RX side responsive and stays well inside ISM duty-cycle ceilings.
const DUTY_PERCENT: u64 = 30;
const MIN_PERIOD_MS: u32 = 50;
const MAX_PERIOD_MS: u32 = 60_000;

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub struct AirtimeBudget {
    periods_ms: [u32; BUDGETED_CLASS_COUNT],
}

impl AirtimeBudget {
    /// Compute per-class send periods from the live LoRa profile. Run once at link bring-up.
    pub fn from_profile(profile: &LoraProfile) -> Self {
        let specs = class_specs();
        let mut air_us = [0u32; BUDGETED_CLASS_COUNT];
        // Ideal airtime demand per second, in us: Σ_c rate_hz_c · airtime_c.
        let mut demand_us_per_s: u64 = 0;
        let mut i = 0;
        while i < BUDGETED_CLASS_COUNT {
            // On-air PHY length = LoRa frame header + RF packet (overhead + wire payload).
            let on_air_len = (FRAME_HEADER_LEN + encoded_rf_len(specs[i].wire_len)) as u8;
            air_us[i] = profile.time_on_air_us(on_air_len);
            demand_us_per_s += (specs[i].ideal_rate_mhz as u64) * (air_us[i] as u64) / 1_000;
            i += 1;
        }

        let budget_us_per_s: u64 = DUTY_PERCENT * 1_000_000 / 100;

        // s = min(1, B/D); period_c = 1 / (s · f_c). When demand fits the budget every class
        // runs at its ideal rate; otherwise all rates scale down by the same factor, preserving
        // their ratios (airtime in proportion to update rate).
        let mut periods_ms = [MAX_PERIOD_MS; BUDGETED_CLASS_COUNT];
        let mut i = 0;
        while i < BUDGETED_CLASS_COUNT {
            let rate_mhz = specs[i].ideal_rate_mhz as u64;
            if rate_mhz != 0 {
                let period = if demand_us_per_s <= budget_us_per_s {
                    1_000_000 / rate_mhz
                } else {
                    (1_000_000u128 * demand_us_per_s as u128
                        / (rate_mhz as u128 * budget_us_per_s as u128)) as u64
                };
                periods_ms[i] = (period as u32).clamp(MIN_PERIOD_MS, MAX_PERIOD_MS);
            }
            i += 1;
        }

        Self { periods_ms }
    }

    #[inline]
    pub fn period_ms(&self, class: DownlinkClass) -> u32 {
        self.periods_ms[class as usize]
    }
}
