//! Core estimator contracts mirrored from the Python reference design.

use super::types::{EstimatorScalar, MatMN, MatN, MatNW, VecN};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeasurementLabel {
    GravityAlignment,
    GpsPosition,
    GpsVelocity,
    BarometricAltitude,
    Custom(u16),
}

pub trait NominalState: Clone {
    fn copy(&self) -> Self {
        self.clone()
    }
}

impl<T> NominalState for T where T: Clone {}

pub trait EuclideanState<T: EstimatorScalar, const N: usize>: NominalState {
    fn plus(&self, delta: &VecN<T, N>) -> Self;
}

pub trait ProcessModel<S, C, T: EstimatorScalar, const N: usize, const W: usize> {
    fn predict(&self, nominal_state: &S, control: &C, dt: T) -> S;
    fn process_noise_jacobian(&self, nominal_state: &S, control: &C, dt: T) -> MatNW<T, N, W>;
    fn process_noise_covariance(&self, nominal_state: &S, control: &C, dt: T) -> MatN<T, W>;
}

pub trait EuclideanProcessModel<S, C, T: EstimatorScalar, const N: usize, const W: usize>:
    ProcessModel<S, C, T, N, W>
{
    fn state_jacobian(&self, nominal_state: &S, control: &C, dt: T) -> MatN<T, N>;
}

pub trait ErrorStateProcessModel<S, C, T: EstimatorScalar, const N: usize, const W: usize>:
    ProcessModel<S, C, T, N, W>
{
    fn error_state_jacobian(&self, nominal_state: &S, control: &C, dt: T) -> MatN<T, N>;
    fn inject(&self, nominal_state: &S, error_state: &VecN<T, N>) -> S;
    fn reset_jacobian(&self, injected_error_state: &VecN<T, N>) -> MatN<T, N>;
}

pub trait MeasurementModel<S, T: EstimatorScalar, const N: usize, const M: usize> {
    type Measurement;

    fn label(&self) -> MeasurementLabel;
    fn predict_measurement(&self, nominal_state: &S) -> VecN<T, M>;
    fn innovation(
        &self,
        measurement: &Self::Measurement,
        predicted_measurement: &VecN<T, M>,
    ) -> VecN<T, M>;
    fn measurement_jacobian(
        &self,
        measurement: &Self::Measurement,
        nominal_state: &S,
    ) -> MatMN<T, M, N>;
    fn measurement_covariance(
        &self,
        measurement: &Self::Measurement,
        nominal_state: &S,
    ) -> MatN<T, M>;
}
