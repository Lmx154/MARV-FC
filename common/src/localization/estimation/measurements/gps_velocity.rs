//! GPS velocity measurement model.

use crate::localization::estimation::core::{
    EstimatorScalar, MatMN, MatN, MeasurementLabel, MeasurementModel, Vec3, scalar,
};
use crate::localization::estimation::models::{NAVIGATION_STATE_DIM, NavigationState};

#[derive(Clone, Debug)]
pub struct GpsVelocityConfig<T: EstimatorScalar> {
    pub measurement_std_mps: Vec3<T>,
}

impl<T: EstimatorScalar> Default for GpsVelocityConfig<T> {
    fn default() -> Self {
        Self {
            measurement_std_mps: Vec3::<T>::new(scalar(3.0), scalar(3.0), scalar(4.0)),
        }
    }
}

#[derive(Clone, Debug)]
pub struct GpsVelocityMeasurementModel<T: EstimatorScalar> {
    pub config: GpsVelocityConfig<T>,
}

impl<T: EstimatorScalar> Default for GpsVelocityMeasurementModel<T> {
    fn default() -> Self {
        Self {
            config: GpsVelocityConfig::default(),
        }
    }
}

impl<T: EstimatorScalar> MeasurementModel<NavigationState<T>, T, NAVIGATION_STATE_DIM, 3>
    for GpsVelocityMeasurementModel<T>
{
    type Measurement = Vec3<T>;

    fn label(&self) -> MeasurementLabel {
        MeasurementLabel::GpsVelocity
    }

    fn predict_measurement(&self, nominal_state: &NavigationState<T>) -> Vec3<T> {
        nominal_state.velocity_mps
    }

    fn innovation(
        &self,
        measurement: &Self::Measurement,
        predicted_measurement: &Vec3<T>,
    ) -> Vec3<T> {
        measurement - predicted_measurement
    }

    fn measurement_jacobian(
        &self,
        _measurement: &Self::Measurement,
        _nominal_state: &NavigationState<T>,
    ) -> MatMN<T, 3, NAVIGATION_STATE_DIM> {
        let mut jacobian = MatMN::<T, 3, NAVIGATION_STATE_DIM>::zeros();
        jacobian
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&crate::localization::estimation::core::Mat3::<T>::identity());
        jacobian
    }

    fn measurement_covariance(
        &self,
        _measurement: &Self::Measurement,
        _nominal_state: &NavigationState<T>,
    ) -> MatN<T, 3> {
        let mut covariance = MatN::<T, 3>::zeros();
        for axis in 0..3 {
            let std = self.config.measurement_std_mps[axis];
            covariance[(axis, axis)] = std * std;
        }
        covariance
    }
}
