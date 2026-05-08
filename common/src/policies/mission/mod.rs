//! Mission policy rules belong here.

use crate::messages::control::RgbLedCommand;
use crate::messages::sensor::BarometerSampleStamped;
use crate::utilities::units::{STANDARD_SEA_LEVEL_PRESSURE_PA, pressure_altitude_ft};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MissionUpdate<T> {
    NoChange,
    Command(T),
}

pub trait Mission<Input> {
    type Output: Copy;

    fn initial_output(&self) -> Option<Self::Output> {
        None
    }

    fn update(&mut self, input: Input) -> MissionUpdate<Self::Output>;
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BarometerRgbLedMissionConfig {
    pub trigger_altitude_ft: f32,
    pub sea_level_pressure_pa: f32,
    pub latched_command: RgbLedCommand,
}

impl BarometerRgbLedMissionConfig {
    pub const fn new(
        trigger_altitude_ft: f32,
        sea_level_pressure_pa: f32,
        latched_command: RgbLedCommand,
    ) -> Self {
        Self {
            trigger_altitude_ft,
            sea_level_pressure_pa,
            latched_command,
        }
    }
}

impl Default for BarometerRgbLedMissionConfig {
    fn default() -> Self {
        Self {
            trigger_altitude_ft: 5_000.0,
            sea_level_pressure_pa: STANDARD_SEA_LEVEL_PRESSURE_PA,
            latched_command: RgbLedCommand::new(24, 24, 24),
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BarometerRgbLedMission {
    config: BarometerRgbLedMissionConfig,
    latched: bool,
}

impl BarometerRgbLedMission {
    pub const fn new(config: BarometerRgbLedMissionConfig) -> Self {
        Self {
            config,
            latched: false,
        }
    }

    pub const fn is_latched(&self) -> bool {
        self.latched
    }

    pub const fn config(&self) -> BarometerRgbLedMissionConfig {
        self.config
    }
}

impl Mission<BarometerSampleStamped> for BarometerRgbLedMission {
    type Output = RgbLedCommand;

    fn initial_output(&self) -> Option<Self::Output> {
        Some(RgbLedCommand::OFF)
    }

    fn update(&mut self, input: BarometerSampleStamped) -> MissionUpdate<Self::Output> {
        if self.latched {
            return MissionUpdate::NoChange;
        }

        let Some(altitude_ft) =
            pressure_altitude_ft(input.sample.pressure_pa, self.config.sea_level_pressure_pa)
        else {
            return MissionUpdate::NoChange;
        };

        if altitude_ft >= self.config.trigger_altitude_ft {
            self.latched = true;
            MissionUpdate::Command(self.config.latched_command)
        } else {
            MissionUpdate::NoChange
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{BarometerRgbLedMission, BarometerRgbLedMissionConfig, Mission, MissionUpdate};
    use crate::messages::control::RgbLedCommand;
    use crate::messages::sensor::{BarometerSample, BarometerSampleStamped};
    use crate::utilities::time::MeasurementTimestamp;
    use crate::utilities::units::STANDARD_SEA_LEVEL_PRESSURE_PA;

    fn sample(pressure_pa: f32) -> BarometerSampleStamped {
        BarometerSampleStamped {
            timestamp: MeasurementTimestamp::from_micros(1),
            sample: BarometerSample {
                pressure_pa,
                temperature_c: 20.0,
            },
        }
    }

    #[test]
    fn mission_starts_with_led_off() {
        let mission = BarometerRgbLedMission::new(BarometerRgbLedMissionConfig::default());
        assert_eq!(mission.initial_output(), Some(RgbLedCommand::OFF));
    }

    #[test]
    fn mission_latches_on_when_altitude_threshold_is_crossed() {
        let mut mission = BarometerRgbLedMission::new(BarometerRgbLedMissionConfig::new(
            5_000.0,
            STANDARD_SEA_LEVEL_PRESSURE_PA,
            RgbLedCommand::new(1, 2, 3),
        ));

        assert_eq!(mission.update(sample(95_000.0)), MissionUpdate::NoChange);
        assert_eq!(
            mission.update(sample(84_307.0)),
            MissionUpdate::Command(RgbLedCommand::new(1, 2, 3))
        );
        assert!(mission.is_latched());
    }

    #[test]
    fn mission_stays_latched_after_altitude_drops_back_down() {
        let mut mission = BarometerRgbLedMission::new(BarometerRgbLedMissionConfig::default());

        let _ = mission.update(sample(84_307.0));
        assert_eq!(mission.update(sample(101_325.0)), MissionUpdate::NoChange);
        assert!(mission.is_latched());
    }

    #[test]
    fn invalid_pressure_does_not_change_mission_state() {
        let mut mission = BarometerRgbLedMission::new(BarometerRgbLedMissionConfig::default());
        assert_eq!(mission.update(sample(0.0)), MissionUpdate::NoChange);
        assert!(!mission.is_latched());
    }
}
