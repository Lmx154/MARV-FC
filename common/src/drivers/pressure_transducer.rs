//! Analog pressure transducer conversion helpers.

use crate::messages::sensor::PressureTransducerSample;

const PSI_TO_KPA: f32 = 6.894_757_3;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PressureTransducerConfig {
    pub adc_reference_volts: f32,
    pub adc_max_counts: u16,
    pub divider_r_top_ohms: f32,
    pub divider_r_bottom_ohms: f32,
    pub sensor_v_out_min: f32,
    pub sensor_v_out_max: f32,
    pub pressure_min_psi: f32,
    pub pressure_max_psi: f32,
}

impl Default for PressureTransducerConfig {
    fn default() -> Self {
        Self {
            adc_reference_volts: 3.3,
            adc_max_counts: 4095,
            divider_r_top_ohms: 10_000.0,
            divider_r_bottom_ohms: 27_000.0,
            sensor_v_out_min: 0.5,
            sensor_v_out_max: 4.5,
            pressure_min_psi: 0.0,
            pressure_max_psi: 150.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum PressureTransducerConfigError {
    InvalidAdcMaxCounts,
    InvalidDividerRatio,
    InvalidVoltageSpan,
    InvalidPressureSpan,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PressureTransducer {
    config: PressureTransducerConfig,
}

impl PressureTransducer {
    pub fn try_new(
        config: PressureTransducerConfig,
    ) -> Result<Self, PressureTransducerConfigError> {
        if config.adc_max_counts == 0 {
            return Err(PressureTransducerConfigError::InvalidAdcMaxCounts);
        }

        if config.divider_r_top_ohms <= 0.0 || config.divider_r_bottom_ohms <= 0.0 {
            return Err(PressureTransducerConfigError::InvalidDividerRatio);
        }

        if config.sensor_v_out_max <= config.sensor_v_out_min {
            return Err(PressureTransducerConfigError::InvalidVoltageSpan);
        }

        if config.pressure_max_psi <= config.pressure_min_psi {
            return Err(PressureTransducerConfigError::InvalidPressureSpan);
        }

        Ok(Self { config })
    }

    pub const fn config(&self) -> PressureTransducerConfig {
        self.config
    }

    pub fn divider_ratio(&self) -> f32 {
        self.config.divider_r_bottom_ohms
            / (self.config.divider_r_top_ohms + self.config.divider_r_bottom_ohms)
    }

    pub fn adc_counts_to_voltage(&self, adc_counts: u16) -> f32 {
        (adc_counts as f32) * self.config.adc_reference_volts / (self.config.adc_max_counts as f32)
    }

    pub fn adc_voltage_to_sensor_voltage(&self, adc_voltage_v: f32) -> f32 {
        adc_voltage_v / self.divider_ratio()
    }

    pub fn sensor_voltage_to_pressure_psi(&self, sensor_voltage_v: f32) -> f32 {
        let fraction = (sensor_voltage_v - self.config.sensor_v_out_min)
            / (self.config.sensor_v_out_max - self.config.sensor_v_out_min);

        self.config.pressure_min_psi
            + fraction * (self.config.pressure_max_psi - self.config.pressure_min_psi)
    }

    pub fn adc_counts_to_pressure_psi(&self, adc_counts: u16) -> f32 {
        let adc_voltage_v = self.adc_counts_to_voltage(adc_counts);
        let sensor_voltage_v = self.adc_voltage_to_sensor_voltage(adc_voltage_v);
        self.sensor_voltage_to_pressure_psi(sensor_voltage_v)
    }

    pub fn sample_from_adc_counts(&self, adc_counts: u16) -> PressureTransducerSample {
        PressureTransducerSample {
            pressure_psi: self.adc_counts_to_pressure_psi(adc_counts),
        }
    }
}

pub fn psi_to_kpa(pressure_psi: f32) -> f32 {
    pressure_psi * PSI_TO_KPA
}

pub fn psi_to_pa(pressure_psi: f32) -> f32 {
    psi_to_kpa(pressure_psi) * 1_000.0
}

#[cfg(test)]
mod tests {
    use super::{PressureTransducer, PressureTransducerConfig, psi_to_pa};

    #[test]
    fn default_config_maps_endpoints() {
        let sensor = PressureTransducer::try_new(PressureTransducerConfig::default()).unwrap();

        let zero = sensor.sample_from_adc_counts(453);
        let full_scale = sensor.sample_from_adc_counts(4075);

        assert!((zero.pressure_psi - 0.0).abs() < 0.25);
        assert!((full_scale.pressure_psi - 150.0).abs() < 0.25);
    }

    #[test]
    fn psi_to_pa_matches_expected_scale() {
        assert!((psi_to_pa(100.0) - 689_475.75).abs() < 1.0);
    }
}
