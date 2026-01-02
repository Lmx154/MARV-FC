//! Comms/link scheduling utilities.
//!
//! Rationale (inspired by ArduPilot / PX4 / Betaflight):
//! - Scheduling is fundamentally **per-link** (each transport has different bandwidth/latency).
//! - The telemetry layer decides *what* to send; the comms layer enforces *when* we can send.
//! - Keep this module time-source agnostic: callers pass a monotonic `now_ms: u32`.
//!
//! This is intentionally tiny and allocation-free.

/// Output of a scheduler poll.
#[derive(Clone, Copy, Debug, Default)]
pub struct LinkScheduleDecision {
    pub send_heartbeat: bool,
    pub send_telemetry: bool,
    pub send_fast: bool,
}

#[derive(Clone, Copy, Debug)]
struct Periodic {
    interval_ms: u32,
    next_due_ms: u32,
}

impl Periodic {
    fn new(now_ms: u32, interval_ms: u32) -> Self {
        let interval_ms = interval_ms.max(1);
        Self {
            interval_ms,
            next_due_ms: now_ms.wrapping_add(interval_ms),
        }
    }

    fn set_interval_ms(&mut self, now_ms: u32, interval_ms: u32) {
        let interval_ms = interval_ms.max(1);
        if self.interval_ms != interval_ms {
            self.interval_ms = interval_ms;
            // Reset schedule relative to now to avoid bursts.
            self.next_due_ms = now_ms.wrapping_add(interval_ms);
        }
    }

    fn due(&mut self, now_ms: u32) -> bool {
        // Wrap-safe reached check.
        if now_ms.wrapping_sub(self.next_due_ms) < 0x8000_0000 {
            self.next_due_ms = now_ms.wrapping_add(self.interval_ms);
            true
        } else {
            false
        }
    }
}

/// Simple TX pacing gate.
///
/// Used to enforce a minimum gap between transmissions on a constrained link
/// (e.g., LoRa). This is complementary to periodic scheduling.
#[derive(Clone, Copy, Debug, Default)]
pub struct TxGate {
    last_tx_ms: Option<u32>,
}

impl TxGate {
    pub fn new() -> Self {
        Self { last_tx_ms: None }
    }

    /// Returns how many ms remain until the next TX is allowed.
    pub fn time_until_allowed_ms(&self, now_ms: u32, min_gap_ms: u32) -> u32 {
        let Some(last) = self.last_tx_ms else {
            return 0;
        };

        let elapsed = now_ms.wrapping_sub(last);
        if elapsed >= min_gap_ms {
            0
        } else {
            min_gap_ms - elapsed
        }
    }

    /// Record that a transmission happened at `now_ms`.
    pub fn on_tx(&mut self, now_ms: u32) {
        self.last_tx_ms = Some(now_ms);
    }
}

/// Per-link scheduler for heartbeat + telemetry.
///
/// This is deliberately simple (Step 3). Higher-level stream selection/priorities
/// can build on top of this.
pub struct LinkScheduler {
    heartbeat: Periodic,
    telemetry: Periodic,
    fast: Periodic,
    heartbeat_enabled: bool,
    telemetry_enabled: bool,
    fast_enabled: bool,
    heartbeat_hz: u32,
    telemetry_hz: u32,
    fast_hz: u32,
}

impl LinkScheduler {
    /// - `heartbeat_hz`: typically 1 for GCS compatibility.
    /// - `telemetry_hz`: may be 0 to disable telemetry.
    pub fn new(now_ms: u32, heartbeat_hz: u32, telemetry_hz: u32) -> Self {
        let heartbeat_interval_ms = hz_to_interval_ms(heartbeat_hz.max(1));
        let telemetry_interval_ms = hz_to_interval_ms(telemetry_hz.max(1));
        let fast_interval_ms = hz_to_interval_ms(1);

        Self {
            heartbeat: Periodic::new(now_ms, heartbeat_interval_ms),
            telemetry: Periodic::new(now_ms, telemetry_interval_ms),
            fast: Periodic::new(now_ms, fast_interval_ms),
            heartbeat_enabled: true,
            telemetry_enabled: telemetry_hz != 0,
            fast_enabled: false,
            heartbeat_hz: heartbeat_hz.max(1),
            telemetry_hz,
            fast_hz: 0,
        }
    }

    pub fn set_heartbeat_enabled(&mut self, enabled: bool) {
        self.heartbeat_enabled = enabled;
    }

    pub fn set_heartbeat_hz(&mut self, now_ms: u32, heartbeat_hz: u32) {
        let heartbeat_hz = heartbeat_hz.max(1);
        if self.heartbeat_hz == heartbeat_hz {
            return;
        }

        self.heartbeat_hz = heartbeat_hz;
        self.heartbeat
            .set_interval_ms(now_ms, hz_to_interval_ms(heartbeat_hz));
    }

    pub fn set_telemetry_hz(&mut self, now_ms: u32, telemetry_hz: u32) {
        if self.telemetry_hz == telemetry_hz {
            return;
        }

        self.telemetry_hz = telemetry_hz;
        self.telemetry_enabled = telemetry_hz != 0;

        let effective_hz = telemetry_hz.max(1);
        self.telemetry
            .set_interval_ms(now_ms, hz_to_interval_ms(effective_hz));
    }

    pub fn set_fast_enabled(&mut self, enabled: bool) {
        self.fast_enabled = enabled;
    }

    pub fn set_fast_hz(&mut self, now_ms: u32, fast_hz: u32) {
        if self.fast_hz == fast_hz {
            return;
        }

        self.fast_hz = fast_hz;

        let effective_hz = fast_hz.max(1);
        self.fast
            .set_interval_ms(now_ms, hz_to_interval_ms(effective_hz));
    }

    pub fn poll(&mut self, now_ms: u32) -> LinkScheduleDecision {
        let mut out = LinkScheduleDecision::default();

        if self.heartbeat_enabled && self.heartbeat.due(now_ms) {
            out.send_heartbeat = true;
        }

        if self.telemetry_enabled && self.telemetry.due(now_ms) {
            out.send_telemetry = true;
        }

        if self.fast_enabled && self.fast_hz != 0 && self.fast.due(now_ms) {
            out.send_fast = true;
        }

        out
    }
}

fn hz_to_interval_ms(hz: u32) -> u32 {
    (1000 / hz.max(1)).max(1)
}
