//! Body-frame gravity alignment measurement model.

use crate::localization::estimation::core::{
    EstimatorScalar, MatN, MeasurementLabel, MeasurementModel, Vec3, scalar,
};
use crate::localization::estimation::math::{quaternion_to_rotation_matrix, skew_symmetric};
use crate::localization::estimation::models::{ATTITUDE_STATE_DIM, AttitudeState};

#[derive(Clone, Debug)]
pub struct GravityAlignmentConfig<T: EstimatorScalar> {
    pub gravity_vector: Vec3<T>,
    pub measurement_std_mps2: T,
}

impl<T: EstimatorScalar> Default for GravityAlignmentConfig<T> {
    fn default() -> Self {
        Self {
            gravity_vector: Vec3::<T>::new(T::zero(), T::zero(), scalar(-9.80665)),
            measurement_std_mps2: scalar(0.5),
        }
    }
}

#[derive(Clone, Debug)]
pub struct GravityAlignmentMeasurementModel<T: EstimatorScalar> {
    pub config: GravityAlignmentConfig<T>,
}

impl<T: EstimatorScalar> Default for GravityAlignmentMeasurementModel<T> {
    fn default() -> Self {
        Self {
            config: GravityAlignmentConfig::default(),
        }
    }
}

impl<T: EstimatorScalar> MeasurementModel<AttitudeState<T>, T, ATTITUDE_STATE_DIM, 3>
    for GravityAlignmentMeasurementModel<T>
{
    type Measurement = Vec3<T>;

    fn label(&self) -> MeasurementLabel {
        MeasurementLabel::GravityAlignment
    }

    fn predict_measurement(&self, nominal_state: &AttitudeState<T>) -> Vec3<T> {
        let rotation_body_to_inertial = quaternion_to_rotation_matrix(&nominal_state.quaternion);
        rotation_body_to_inertial.transpose() * self.config.gravity_vector
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
        nominal_state: &AttitudeState<T>,
    ) -> crate::localization::estimation::core::MatMN<T, 3, ATTITUDE_STATE_DIM> {
        let predicted_measurement = self.predict_measurement(nominal_state);
        let mut jacobian =
            crate::localization::estimation::core::MatMN::<T, 3, ATTITUDE_STATE_DIM>::zeros();
        jacobian
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&skew_symmetric(&predicted_measurement));
        jacobian
    }

    fn measurement_covariance(
        &self,
        _measurement: &Self::Measurement,
        _nominal_state: &AttitudeState<T>,
    ) -> MatN<T, 3> {
        MatN::<T, 3>::identity()
            * (self.config.measurement_std_mps2 * self.config.measurement_std_mps2)
    }
}
