//! Barometric altitude measurement model.

use crate::localization::estimation::core::{
    EstimatorScalar, MatMN, MatN, MeasurementLabel, MeasurementModel, Vec1, scalar,
};
use crate::localization::estimation::models::{NAVIGATION_STATE_DIM, NavigationState};

#[derive(Clone, Copy, Debug)]
pub struct BarometricAltitudeConfig<T: EstimatorScalar> {
    pub measurement_std_m: T,
    pub axis: usize,
}

impl<T: EstimatorScalar> Default for BarometricAltitudeConfig<T> {
    fn default() -> Self {
        Self {
            measurement_std_m: scalar(2.0),
            axis: 2,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BarometricAltitudeMeasurementModel<T: EstimatorScalar> {
    pub config: BarometricAltitudeConfig<T>,
}

impl<T: EstimatorScalar> Default for BarometricAltitudeMeasurementModel<T> {
    fn default() -> Self {
        Self {
            config: BarometricAltitudeConfig::default(),
        }
    }
}

impl<T: EstimatorScalar> MeasurementModel<NavigationState<T>, T, NAVIGATION_STATE_DIM, 1>
    for BarometricAltitudeMeasurementModel<T>
{
    type Measurement = T;

    fn label(&self) -> MeasurementLabel {
        MeasurementLabel::BarometricAltitude
    }

    fn predict_measurement(&self, nominal_state: &NavigationState<T>) -> Vec1<T> {
        Vec1::<T>::new(nominal_state.position_m[self.config.axis])
    }

    fn innovation(
        &self,
        measurement: &Self::Measurement,
        predicted_measurement: &Vec1<T>,
    ) -> Vec1<T> {
        Vec1::<T>::new(*measurement - predicted_measurement[0])
    }

    fn measurement_jacobian(
        &self,
        _measurement: &Self::Measurement,
        _nominal_state: &NavigationState<T>,
    ) -> MatMN<T, 1, NAVIGATION_STATE_DIM> {
        let mut jacobian = MatMN::<T, 1, NAVIGATION_STATE_DIM>::zeros();
        jacobian[(0, self.config.axis)] = T::one();
        jacobian
    }

    fn measurement_covariance(
        &self,
        _measurement: &Self::Measurement,
        _nominal_state: &NavigationState<T>,
    ) -> MatN<T, 1> {
        MatN::<T, 1>::new(self.config.measurement_std_m * self.config.measurement_std_m)
    }
}
