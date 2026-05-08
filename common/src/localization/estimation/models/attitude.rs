//! Quaternion attitude process model with gyro-bias error states.

use crate::localization::estimation::core::{
    ErrorStateProcessModel, EstimatorScalar, MatN, MatNW, NominalState, Quaternion, Vec3, VecN,
    scalar,
};
use crate::localization::estimation::math::{
    identity_quaternion, normalize_quaternion, quaternion_multiply, rotation_vector_to_quaternion,
    skew_symmetric,
};

pub const ATTITUDE_STATE_DIM: usize = 6;
pub const ATTITUDE_NOISE_DIM: usize = 6;

#[derive(Clone, Copy, Debug)]
pub struct AttitudeProcessNoise<T: EstimatorScalar> {
    pub gyroscope_noise_std: T,
    pub gyro_bias_random_walk_std: T,
}

impl<T: EstimatorScalar> Default for AttitudeProcessNoise<T> {
    fn default() -> Self {
        Self {
            gyroscope_noise_std: scalar(2.0e-3),
            gyro_bias_random_walk_std: scalar(2.0e-4),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AttitudeProcessModelConfig<T: EstimatorScalar> {
    pub process_noise: AttitudeProcessNoise<T>,
}

impl<T: EstimatorScalar> Default for AttitudeProcessModelConfig<T> {
    fn default() -> Self {
        Self {
            process_noise: AttitudeProcessNoise::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct AttitudeInput<T: EstimatorScalar> {
    pub gyroscope_rps: Vec3<T>,
}

#[derive(Clone, Debug)]
pub struct AttitudeState<T: EstimatorScalar> {
    pub quaternion: Quaternion<T>,
    pub gyro_bias_rps: Vec3<T>,
}

impl<T: EstimatorScalar> Default for AttitudeState<T> {
    fn default() -> Self {
        Self {
            quaternion: identity_quaternion(),
            gyro_bias_rps: Vec3::<T>::zeros(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AttitudeErrorStateProcessModel<T: EstimatorScalar> {
    pub config: AttitudeProcessModelConfig<T>,
}

impl<T: EstimatorScalar> Default for AttitudeErrorStateProcessModel<T> {
    fn default() -> Self {
        Self {
            config: AttitudeProcessModelConfig::default(),
        }
    }
}

impl<T: EstimatorScalar> AttitudeErrorStateProcessModel<T> {
    pub fn corrected_angular_rate(
        &self,
        nominal_state: &AttitudeState<T>,
        control: &AttitudeInput<T>,
    ) -> Vec3<T> {
        control.gyroscope_rps - nominal_state.gyro_bias_rps
    }

    pub fn continuous_error_dynamics_jacobian(
        &self,
        nominal_state: &AttitudeState<T>,
        control: &AttitudeInput<T>,
    ) -> MatN<T, ATTITUDE_STATE_DIM> {
        let corrected_rate = self.corrected_angular_rate(nominal_state, control);
        let mut system_matrix = MatN::<T, ATTITUDE_STATE_DIM>::zeros();
        system_matrix
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(-skew_symmetric(&corrected_rate)));
        system_matrix
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(-crate::localization::estimation::core::Mat3::<T>::identity()));
        system_matrix
    }
}

impl<T: EstimatorScalar>
    crate::localization::estimation::core::ProcessModel<
        AttitudeState<T>,
        AttitudeInput<T>,
        T,
        ATTITUDE_STATE_DIM,
        ATTITUDE_NOISE_DIM,
    > for AttitudeErrorStateProcessModel<T>
{
    fn predict(
        &self,
        nominal_state: &AttitudeState<T>,
        control: &AttitudeInput<T>,
        dt: T,
    ) -> AttitudeState<T> {
        if dt <= T::zero() {
            return nominal_state.copy();
        }

        let corrected_rate = self.corrected_angular_rate(nominal_state, control);
        let delta_quaternion = rotation_vector_to_quaternion(&(corrected_rate * dt));
        let mut predicted_state = nominal_state.copy();
        predicted_state.quaternion = normalize_quaternion(&quaternion_multiply(
            &predicted_state.quaternion,
            &delta_quaternion,
        ));
        predicted_state
    }

    fn process_noise_jacobian(
        &self,
        _nominal_state: &AttitudeState<T>,
        _control: &AttitudeInput<T>,
        _dt: T,
    ) -> MatNW<T, ATTITUDE_STATE_DIM, ATTITUDE_NOISE_DIM> {
        let mut noise_jacobian = MatNW::<T, ATTITUDE_STATE_DIM, ATTITUDE_NOISE_DIM>::zeros();
        noise_jacobian
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(-crate::localization::estimation::core::Mat3::<T>::identity()));
        noise_jacobian
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&crate::localization::estimation::core::Mat3::<T>::identity());
        noise_jacobian
    }

    fn process_noise_covariance(
        &self,
        _nominal_state: &AttitudeState<T>,
        _control: &AttitudeInput<T>,
        dt: T,
    ) -> MatN<T, ATTITUDE_NOISE_DIM> {
        let dt = if dt < T::zero() { T::zero() } else { dt };
        let mut covariance = MatN::<T, ATTITUDE_NOISE_DIM>::zeros();
        let gyro_variance = self.config.process_noise.gyroscope_noise_std
            * self.config.process_noise.gyroscope_noise_std
            * dt;
        let bias_variance = self.config.process_noise.gyro_bias_random_walk_std
            * self.config.process_noise.gyro_bias_random_walk_std
            * dt;
        for axis in 0..3 {
            covariance[(axis, axis)] = gyro_variance;
            covariance[(axis + 3, axis + 3)] = bias_variance;
        }
        covariance
    }
}

impl<T: EstimatorScalar>
    ErrorStateProcessModel<
        AttitudeState<T>,
        AttitudeInput<T>,
        T,
        ATTITUDE_STATE_DIM,
        ATTITUDE_NOISE_DIM,
    > for AttitudeErrorStateProcessModel<T>
{
    fn error_state_jacobian(
        &self,
        nominal_state: &AttitudeState<T>,
        control: &AttitudeInput<T>,
        dt: T,
    ) -> MatN<T, ATTITUDE_STATE_DIM> {
        MatN::<T, ATTITUDE_STATE_DIM>::identity()
            + self.continuous_error_dynamics_jacobian(nominal_state, control) * dt
    }

    fn inject(
        &self,
        nominal_state: &AttitudeState<T>,
        error_state: &VecN<T, ATTITUDE_STATE_DIM>,
    ) -> AttitudeState<T> {
        let mut injected_state = nominal_state.copy();
        let attitude_error = error_state.fixed_rows::<3>(0).into_owned();
        let bias_error = error_state.fixed_rows::<3>(3).into_owned();
        injected_state.quaternion = normalize_quaternion(&quaternion_multiply(
            &injected_state.quaternion,
            &rotation_vector_to_quaternion(&attitude_error),
        ));
        injected_state.gyro_bias_rps += bias_error;
        injected_state
    }

    fn reset_jacobian(
        &self,
        injected_error_state: &VecN<T, ATTITUDE_STATE_DIM>,
    ) -> MatN<T, ATTITUDE_STATE_DIM> {
        let mut reset = MatN::<T, ATTITUDE_STATE_DIM>::identity();
        let half = scalar::<T>(0.5);
        let attitude_error = injected_error_state.fixed_rows::<3>(0).into_owned();
        reset.fixed_view_mut::<3, 3>(0, 0).copy_from(
            &(crate::localization::estimation::core::Mat3::<T>::identity()
                - skew_symmetric(&attitude_error) * half),
        );
        reset
    }
}
