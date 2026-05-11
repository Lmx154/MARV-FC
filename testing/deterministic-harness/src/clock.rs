use crate::{HarnessFailure, HarnessResult};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ClockSnapshot {
    pub tick: u64,
    pub sim_time_us: u64,
    pub dt_s: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockStep {
    pub previous: ClockSnapshot,
    pub current: ClockSnapshot,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LockstepClock {
    tick: u64,
    sim_time_us: u64,
    dt_s: f32,
}

impl LockstepClock {
    pub const fn new() -> Self {
        Self {
            tick: 0,
            sim_time_us: 0,
            dt_s: 0.0,
        }
    }

    pub const fn snapshot(self) -> ClockSnapshot {
        ClockSnapshot {
            tick: self.tick,
            sim_time_us: self.sim_time_us,
            dt_s: self.dt_s,
        }
    }

    pub fn advance(&mut self, dt_s: f32) -> HarnessResult<ClockStep> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(HarnessFailure::new(
                self.tick,
                self.sim_time_us,
                "lockstep dt_s must be finite and positive",
            ));
        }

        let dt_us = dt_s_to_us(dt_s).ok_or_else(|| {
            HarnessFailure::new(
                self.tick,
                self.sim_time_us,
                "lockstep dt_s is too small or too large to represent in microseconds",
            )
        })?;
        let previous = self.snapshot();
        self.tick = self.tick.checked_add(1).ok_or_else(|| {
            HarnessFailure::new(self.tick, self.sim_time_us, "lockstep tick overflow")
        })?;
        self.sim_time_us = self.sim_time_us.checked_add(dt_us).ok_or_else(|| {
            HarnessFailure::new(self.tick, self.sim_time_us, "lockstep sim_time_us overflow")
        })?;
        self.dt_s = dt_s;

        Ok(ClockStep {
            previous,
            current: self.snapshot(),
        })
    }
}

fn dt_s_to_us(dt_s: f32) -> Option<u64> {
    let dt_us = (dt_s as f64 * 1_000_000.0).round();
    if dt_us < 1.0 || dt_us > u64::MAX as f64 {
        None
    } else {
        Some(dt_us as u64)
    }
}
