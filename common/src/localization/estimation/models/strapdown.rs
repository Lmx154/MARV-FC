//! Generic strapdown inertial propagation model.

use crate::localization::estimation::core::{
    ErrorStateProcessModel, EstimatorScalar, MatN, MatNW, NominalState, Quaternion, Vec3, VecN,
    scalar,
};
use crate::localization::estimation::math::{
    identity_quaternion, normalize_quaternion, quaternion_multiply, quaternion_to_rotation_matrix,
    rotation_vector_to_quaternion, skew_symmetric,
};

pub const STRAPDOWN_STATE_DIM: usize = 9;
pub const STRAPDOWN_NOISE_DIM: usize = 6;

#[derive(Clone, Copy, Debug)]
pub struct StrapdownProcessNoise<T: EstimatorScalar> {
    pub accelerometer_noise_std: T,
    pub gyroscope_noise_std: T,
}

impl<T: EstimatorScalar> Default for StrapdownProcessNoise<T> {
    fn default() -> Self {
        Self {
            accelerometer_noise_std: scalar(8.0e-2),
            gyroscope_noise_std: scalar(2.0e-3),
        }
    }
}

#[derive(Clone, Debug)]
pub struct StrapdownConfig<T: EstimatorScalar> {
    pub gravity_vector: Vec3<T>,
    pub accelerometer_includes_gravity: bool,
    pub process_noise: StrapdownProcessNoise<T>,
}

impl<T: EstimatorScalar> Default for StrapdownConfig<T> {
    fn default() -> Self {
        Self {
            gravity_vector: Vec3::<T>::new(T::zero(), T::zero(), scalar(-9.80665)),
            accelerometer_includes_gravity: true,
            process_noise: StrapdownProcessNoise::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct StrapdownInput<T: EstimatorScalar> {
    pub accelerometer_mps2: Vec3<T>,
    pub gyroscope_rps: Vec3<T>,
    pub gyro_bias_rps: Vec3<T>,
    pub accel_bias_mps2: Vec3<T>,
}

impl<T: EstimatorScalar> Default for StrapdownInput<T> {
    fn default() -> Self {
        Self {
            accelerometer_mps2: Vec3::<T>::zeros(),
            gyroscope_rps: Vec3::<T>::zeros(),
            gyro_bias_rps: Vec3::<T>::zeros(),
            accel_bias_mps2: Vec3::<T>::zeros(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct StrapdownState<T: EstimatorScalar> {
    pub position_m: Vec3<T>,
    pub velocity_mps: Vec3<T>,
    pub quaternion: Quaternion<T>,
}

impl<T: EstimatorScalar> Default for StrapdownState<T> {
    fn default() -> Self {
        Self {
            position_m: Vec3::<T>::zeros(),
            velocity_mps: Vec3::<T>::zeros(),
            quaternion: identity_quaternion(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct StrapdownInertialProcessModel<T: EstimatorScalar> {
    pub config: StrapdownConfig<T>,
}

impl<T: EstimatorScalar> Default for StrapdownInertialProcessModel<T> {
    fn default() -> Self {
        Self {
            config: StrapdownConfig::default(),
        }
    }
}

impl<T: EstimatorScalar> StrapdownInertialProcessModel<T> {
    pub fn corrected_angular_rate(&self, control: &StrapdownInput<T>) -> Vec3<T> {
        control.gyroscope_rps - control.gyro_bias_rps
    }

    pub fn corrected_acceleration_body(&self, control: &StrapdownInput<T>) -> Vec3<T> {
        control.accelerometer_mps2 - control.accel_bias_mps2
    }

    pub fn rotation_body_to_inertial(
        &self,
        nominal_state: &StrapdownState<T>,
    ) -> crate::localization::estimation::core::Mat3<T> {
        quaternion_to_rotation_matrix(&nominal_state.quaternion)
    }

    pub fn inertial_acceleration(
        &self,
        nominal_state: &StrapdownState<T>,
        control: &StrapdownInput<T>,
    ) -> Vec3<T> {
        let corrected_acceleration = self.corrected_acceleration_body(control);
        let inertial_acceleration =
            self.rotation_body_to_inertial(nominal_state) * corrected_acceleration;
        if self.config.accelerometer_includes_gravity {
            inertial_acceleration
        } else {
            inertial_acceleration + self.config.gravity_vector
        }
    }

    pub fn continuous_error_dynamics_jacobian(
        &self,
        nominal_state: &StrapdownState<T>,
        control: &StrapdownInput<T>,
    ) -> MatN<T, STRAPDOWN_STATE_DIM> {
        let corrected_rate = self.corrected_angular_rate(control);
        let corrected_acceleration = self.corrected_acceleration_body(control);
        let rotation_body_to_inertial = self.rotation_body_to_inertial(nominal_state);

        let mut system_matrix = MatN::<T, STRAPDOWN_STATE_DIM>::zeros();
        system_matrix
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&crate::localization::estimation::core::Mat3::<T>::identity());
        system_matrix
            .fixed_view_mut::<3, 3>(3, 6)
            .copy_from(&(-rotation_body_to_inertial * skew_symmetric(&corrected_acceleration)));
        system_matrix
            .fixed_view_mut::<3, 3>(6, 6)
            .copy_from(&(-skew_symmetric(&corrected_rate)));
        system_matrix
    }
}

impl<T: EstimatorScalar>
    crate::localization::estimation::core::ProcessModel<
        StrapdownState<T>,
        StrapdownInput<T>,
        T,
        STRAPDOWN_STATE_DIM,
        STRAPDOWN_NOISE_DIM,
    > for StrapdownInertialProcessModel<T>
{
    fn predict(
        &self,
        nominal_state: &StrapdownState<T>,
        control: &StrapdownInput<T>,
        dt: T,
    ) -> StrapdownState<T> {
        if dt <= T::zero() {
            return nominal_state.copy();
        }

        let inertial_acceleration = self.inertial_acceleration(nominal_state, control);
        let corrected_rate = self.corrected_angular_rate(control);
        let delta_quaternion = rotation_vector_to_quaternion(&(corrected_rate * dt));
        let half = scalar::<T>(0.5);

        let mut predicted_state = nominal_state.copy();
        predicted_state.position_m = predicted_state.position_m
            + predicted_state.velocity_mps * dt
            + inertial_acceleration * (half * dt * dt);
        predicted_state.velocity_mps = predicted_state.velocity_mps + inertial_acceleration * dt;
        predicted_state.quaternion = normalize_quaternion(&quaternion_multiply(
            &predicted_state.quaternion,
            &delta_quaternion,
        ));
        predicted_state
    }

    fn process_noise_jacobian(
        &self,
        nominal_state: &StrapdownState<T>,
        _control: &StrapdownInput<T>,
        _dt: T,
    ) -> MatNW<T, STRAPDOWN_STATE_DIM, STRAPDOWN_NOISE_DIM> {
        let rotation_body_to_inertial = self.rotation_body_to_inertial(nominal_state);
        let mut noise_jacobian = MatNW::<T, STRAPDOWN_STATE_DIM, STRAPDOWN_NOISE_DIM>::zeros();
        noise_jacobian
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&rotation_body_to_inertial);
        noise_jacobian
            .fixed_view_mut::<3, 3>(6, 3)
            .copy_from(&(-crate::localization::estimation::core::Mat3::<T>::identity()));
        noise_jacobian
    }

    fn process_noise_covariance(
        &self,
        _nominal_state: &StrapdownState<T>,
        _control: &StrapdownInput<T>,
        dt: T,
    ) -> MatN<T, STRAPDOWN_NOISE_DIM> {
        let dt = if dt < T::zero() { T::zero() } else { dt };
        let mut covariance = MatN::<T, STRAPDOWN_NOISE_DIM>::zeros();
        let accelerometer_variance = self.config.process_noise.accelerometer_noise_std
            * self.config.process_noise.accelerometer_noise_std
            * dt;
        let gyroscope_variance = self.config.process_noise.gyroscope_noise_std
            * self.config.process_noise.gyroscope_noise_std
            * dt;
        for axis in 0..3 {
            covariance[(axis, axis)] = accelerometer_variance;
            covariance[(axis + 3, axis + 3)] = gyroscope_variance;
        }
        covariance
    }
}

impl<T: EstimatorScalar>
    ErrorStateProcessModel<
        StrapdownState<T>,
        StrapdownInput<T>,
        T,
        STRAPDOWN_STATE_DIM,
        STRAPDOWN_NOISE_DIM,
    > for StrapdownInertialProcessModel<T>
{
    fn error_state_jacobian(
        &self,
        nominal_state: &StrapdownState<T>,
        control: &StrapdownInput<T>,
        dt: T,
    ) -> MatN<T, STRAPDOWN_STATE_DIM> {
        MatN::<T, STRAPDOWN_STATE_DIM>::identity()
            + self.continuous_error_dynamics_jacobian(nominal_state, control) * dt
    }

    fn inject(
        &self,
        nominal_state: &StrapdownState<T>,
        error_state: &VecN<T, STRAPDOWN_STATE_DIM>,
    ) -> StrapdownState<T> {
        let mut injected_state = nominal_state.copy();
        injected_state.position_m =
            injected_state.position_m + error_state.fixed_rows::<3>(0).into_owned();
        injected_state.velocity_mps =
            injected_state.velocity_mps + error_state.fixed_rows::<3>(3).into_owned();
        let attitude_error = error_state.fixed_rows::<3>(6).into_owned();
        injected_state.quaternion = normalize_quaternion(&quaternion_multiply(
            &injected_state.quaternion,
            &rotation_vector_to_quaternion(&attitude_error),
        ));
        injected_state
    }

    fn reset_jacobian(
        &self,
        injected_error_state: &VecN<T, STRAPDOWN_STATE_DIM>,
    ) -> MatN<T, STRAPDOWN_STATE_DIM> {
        let mut reset = MatN::<T, STRAPDOWN_STATE_DIM>::identity();
        let half = scalar::<T>(0.5);
        let attitude_error = injected_error_state.fixed_rows::<3>(6).into_owned();
        reset.fixed_view_mut::<3, 3>(6, 6).copy_from(
            &(crate::localization::estimation::core::Mat3::<T>::identity()
                - skew_symmetric(&attitude_error) * half),
        );
        reset
    }
}
