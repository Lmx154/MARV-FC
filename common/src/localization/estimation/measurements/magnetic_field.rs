//! Body-frame magnetic-field direction measurement model.

use crate::localization::estimation::core::{
    EstimatorScalar, MatN, MeasurementLabel, MeasurementModel, Vec3, scalar,
};
use crate::localization::estimation::math::{quaternion_to_rotation_matrix, skew_symmetric};
use crate::localization::estimation::models::{ATTITUDE_STATE_DIM, AttitudeState};

#[derive(Clone, Debug)]
pub struct MagneticFieldConfig<T: EstimatorScalar> {
    pub magnetic_field_inertial_ut: Vec3<T>,
    pub measurement_std_unit: T,
}

impl<T: EstimatorScalar> MagneticFieldConfig<T> {
    pub fn new(magnetic_field_inertial_ut: Vec3<T>) -> Self {
        Self {
            magnetic_field_inertial_ut,
            measurement_std_unit: scalar(0.2),
        }
    }
}

#[derive(Clone, Debug)]
pub struct MagneticFieldMeasurementModel<T: EstimatorScalar> {
    pub config: MagneticFieldConfig<T>,
}

impl<T: EstimatorScalar> MagneticFieldMeasurementModel<T> {
    pub fn new(magnetic_field_inertial_ut: Vec3<T>) -> Self {
        Self {
            config: MagneticFieldConfig::new(magnetic_field_inertial_ut),
        }
    }
}

impl<T: EstimatorScalar> MeasurementModel<AttitudeState<T>, T, ATTITUDE_STATE_DIM, 3>
    for MagneticFieldMeasurementModel<T>
{
    type Measurement = Vec3<T>;

    fn label(&self) -> MeasurementLabel {
        MeasurementLabel::MagneticField
    }

    fn predict_measurement(&self, nominal_state: &AttitudeState<T>) -> Vec3<T> {
        let rotation_body_to_inertial = quaternion_to_rotation_matrix(&nominal_state.quaternion);
        rotation_body_to_inertial.transpose()
            * normalized_or_x_axis(self.config.magnetic_field_inertial_ut)
    }

    fn innovation(
        &self,
        measurement: &Self::Measurement,
        predicted_measurement: &Vec3<T>,
    ) -> Vec3<T> {
        normalized_or_zero(*measurement) - predicted_measurement
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
            * (self.config.measurement_std_unit * self.config.measurement_std_unit)
    }
}

fn normalized_or_x_axis<T: EstimatorScalar>(vector: Vec3<T>) -> Vec3<T> {
    let norm = vector.norm();
    if norm <= T::default_epsilon() {
        Vec3::<T>::new(T::one(), T::zero(), T::zero())
    } else {
        vector / norm
    }
}

fn normalized_or_zero<T: EstimatorScalar>(vector: Vec3<T>) -> Vec3<T> {
    let norm = vector.norm();
    if norm <= T::default_epsilon() {
        Vec3::<T>::zeros()
    } else {
        vector / norm
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_attitude_predicts_inertial_field_in_body_frame() {
        let model = MagneticFieldMeasurementModel::new(Vec3::<f64>::new(20.0, 0.0, 40.0));

        let predicted = model.predict_measurement(&AttitudeState::<f64>::default());

        assert!((predicted.norm() - 1.0).abs() < 0.000_001);
        assert!(predicted[0] > 0.4);
        assert!(predicted[2] > 0.8);
    }

    #[test]
    fn innovation_uses_field_direction_not_magnitude() {
        let model = MagneticFieldMeasurementModel::new(Vec3::<f64>::new(10.0, 0.0, 0.0));
        let predicted = model.predict_measurement(&AttitudeState::<f64>::default());

        let innovation = model.innovation(&Vec3::<f64>::new(100.0, 0.0, 0.0), &predicted);

        assert!(innovation.norm() < 0.000_001);
    }
}
