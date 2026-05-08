//! Strapdown-driven Euclidean navigation process model.

use crate::localization::estimation::core::{
    EstimatorScalar, EuclideanProcessModel, EuclideanState, Mat3, MatN, MatNW, NominalState, Vec3,
    VecN, scalar,
};

pub const NAVIGATION_STATE_DIM: usize = 9;
pub const NAVIGATION_NOISE_DIM: usize = 9;

#[derive(Clone, Copy, Debug)]
pub struct NavigationProcessNoise<T: EstimatorScalar> {
    pub accelerometer_noise_std: T,
    pub accel_bias_random_walk_std: T,
}

impl<T: EstimatorScalar> Default for NavigationProcessNoise<T> {
    fn default() -> Self {
        Self {
            accelerometer_noise_std: scalar(8.0e-2),
            accel_bias_random_walk_std: scalar(2.0e-2),
        }
    }
}

#[derive(Clone, Debug)]
pub struct NavigationProcessModelConfig<T: EstimatorScalar> {
    pub gravity_vector: Vec3<T>,
    pub accelerometer_includes_gravity: bool,
    pub process_noise: NavigationProcessNoise<T>,
}

impl<T: EstimatorScalar> Default for NavigationProcessModelConfig<T> {
    fn default() -> Self {
        Self {
            gravity_vector: Vec3::<T>::new(T::zero(), T::zero(), scalar(-9.80665)),
            accelerometer_includes_gravity: true,
            process_noise: NavigationProcessNoise::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct NavigationInput<T: EstimatorScalar> {
    pub accelerometer_mps2: Vec3<T>,
    pub rotation_body_to_inertial: Mat3<T>,
}

#[derive(Clone, Debug)]
pub struct NavigationState<T: EstimatorScalar> {
    pub position_m: Vec3<T>,
    pub velocity_mps: Vec3<T>,
    pub accel_bias_mps2: Vec3<T>,
}

impl<T: EstimatorScalar> Default for NavigationState<T> {
    fn default() -> Self {
        Self {
            position_m: Vec3::<T>::zeros(),
            velocity_mps: Vec3::<T>::zeros(),
            accel_bias_mps2: Vec3::<T>::zeros(),
        }
    }
}

impl<T: EstimatorScalar> EuclideanState<T, NAVIGATION_STATE_DIM> for NavigationState<T> {
    fn plus(&self, delta: &VecN<T, NAVIGATION_STATE_DIM>) -> Self {
        Self {
            position_m: self.position_m + delta.fixed_rows::<3>(0).into_owned(),
            velocity_mps: self.velocity_mps + delta.fixed_rows::<3>(3).into_owned(),
            accel_bias_mps2: self.accel_bias_mps2 + delta.fixed_rows::<3>(6).into_owned(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct NavigationProcessModel<T: EstimatorScalar> {
    pub config: NavigationProcessModelConfig<T>,
}

impl<T: EstimatorScalar> Default for NavigationProcessModel<T> {
    fn default() -> Self {
        Self {
            config: NavigationProcessModelConfig::default(),
        }
    }
}

impl<T: EstimatorScalar> NavigationProcessModel<T> {
    pub fn corrected_acceleration_body(
        &self,
        nominal_state: &NavigationState<T>,
        control: &NavigationInput<T>,
    ) -> Vec3<T> {
        control.accelerometer_mps2 - nominal_state.accel_bias_mps2
    }

    pub fn inertial_acceleration(
        &self,
        nominal_state: &NavigationState<T>,
        control: &NavigationInput<T>,
    ) -> Vec3<T> {
        let corrected_acceleration = self.corrected_acceleration_body(nominal_state, control);
        let inertial_acceleration = control.rotation_body_to_inertial * corrected_acceleration;
        if self.config.accelerometer_includes_gravity {
            inertial_acceleration
        } else {
            inertial_acceleration + self.config.gravity_vector
        }
    }
}

impl<T: EstimatorScalar>
    crate::localization::estimation::core::ProcessModel<
        NavigationState<T>,
        NavigationInput<T>,
        T,
        NAVIGATION_STATE_DIM,
        NAVIGATION_NOISE_DIM,
    > for NavigationProcessModel<T>
{
    fn predict(
        &self,
        nominal_state: &NavigationState<T>,
        control: &NavigationInput<T>,
        dt: T,
    ) -> NavigationState<T> {
        if dt <= T::zero() {
            return nominal_state.copy();
        }

        let inertial_acceleration = self.inertial_acceleration(nominal_state, control);
        let half = scalar::<T>(0.5);
        let mut predicted_state = nominal_state.copy();
        predicted_state.position_m = predicted_state.position_m
            + predicted_state.velocity_mps * dt
            + inertial_acceleration * (half * dt * dt);
        predicted_state.velocity_mps = predicted_state.velocity_mps + inertial_acceleration * dt;
        predicted_state
    }

    fn process_noise_jacobian(
        &self,
        _nominal_state: &NavigationState<T>,
        control: &NavigationInput<T>,
        _dt: T,
    ) -> MatNW<T, NAVIGATION_STATE_DIM, NAVIGATION_NOISE_DIM> {
        let mut noise_jacobian = MatNW::<T, NAVIGATION_STATE_DIM, NAVIGATION_NOISE_DIM>::zeros();
        noise_jacobian
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&control.rotation_body_to_inertial);
        noise_jacobian
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&control.rotation_body_to_inertial);
        noise_jacobian
            .fixed_view_mut::<3, 3>(6, 6)
            .copy_from(&Mat3::<T>::identity());
        noise_jacobian
    }

    fn process_noise_covariance(
        &self,
        _nominal_state: &NavigationState<T>,
        _control: &NavigationInput<T>,
        dt: T,
    ) -> MatN<T, NAVIGATION_NOISE_DIM> {
        let dt = if dt < T::zero() { T::zero() } else { dt };
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let third = scalar::<T>(1.0 / 3.0);
        let half = scalar::<T>(0.5);
        let accelerometer_variance = self.config.process_noise.accelerometer_noise_std
            * self.config.process_noise.accelerometer_noise_std;
        let bias_variance = self.config.process_noise.accel_bias_random_walk_std
            * self.config.process_noise.accel_bias_random_walk_std;

        let mut covariance = MatN::<T, NAVIGATION_NOISE_DIM>::zeros();
        for axis in 0..3 {
            covariance[(axis, axis)] = accelerometer_variance * dt3 * third;
            covariance[(axis, axis + 3)] = accelerometer_variance * dt2 * half;
            covariance[(axis + 3, axis)] = accelerometer_variance * dt2 * half;
            covariance[(axis + 3, axis + 3)] = accelerometer_variance * dt;
            covariance[(axis + 6, axis + 6)] = bias_variance * dt;
        }
        covariance
    }
}

impl<T: EstimatorScalar>
    EuclideanProcessModel<
        NavigationState<T>,
        NavigationInput<T>,
        T,
        NAVIGATION_STATE_DIM,
        NAVIGATION_NOISE_DIM,
    > for NavigationProcessModel<T>
{
    fn state_jacobian(
        &self,
        _nominal_state: &NavigationState<T>,
        control: &NavigationInput<T>,
        dt: T,
    ) -> MatN<T, NAVIGATION_STATE_DIM> {
        let mut transition = MatN::<T, NAVIGATION_STATE_DIM>::identity();
        let half = scalar::<T>(0.5);
        transition
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(Mat3::<T>::identity() * dt));
        transition
            .fixed_view_mut::<3, 3>(0, 6)
            .copy_from(&(-control.rotation_body_to_inertial * (half * dt * dt)));
        transition
            .fixed_view_mut::<3, 3>(3, 6)
            .copy_from(&(-control.rotation_body_to_inertial * dt));
        transition
    }
}
