//! Gaussian filtering utilities shared by EKF and ESKF engines.

use super::base::MeasurementLabel;
use super::types::{EstimatorScalar, MatMN, MatN, VecN};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CovarianceUpdateForm {
    Joseph,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MeasurementUpdateStatus {
    Accepted,
    Rejected,
    Skipped,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EstimationError {
    SingularInnovationCovariance,
}

#[derive(Clone, Copy, Debug)]
pub struct CovarianceConvention<T> {
    pub min_variance: T,
    pub symmetrize_after_predict: bool,
    pub symmetrize_after_update: bool,
    pub update_form: CovarianceUpdateForm,
}

impl<T: EstimatorScalar> Default for CovarianceConvention<T> {
    fn default() -> Self {
        Self {
            min_variance: super::types::scalar(1.0e-9),
            symmetrize_after_predict: true,
            symmetrize_after_update: true,
            update_form: CovarianceUpdateForm::Joseph,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PredictionDiagnostics<T: EstimatorScalar, const N: usize, const W: usize> {
    pub transition_jacobian: MatN<T, N>,
    pub process_noise_jacobian: MatMN<T, N, W>,
    pub process_noise_covariance: MatN<T, W>,
}

#[derive(Clone, Debug)]
pub struct MeasurementDiagnostics<T: EstimatorScalar, const N: usize, const M: usize> {
    pub predicted_measurement: VecN<T, M>,
    pub innovation: VecN<T, M>,
    pub measurement_jacobian: MatMN<T, M, N>,
    pub measurement_covariance: MatN<T, M>,
    pub innovation_covariance: MatN<T, M>,
}

#[derive(Clone, Debug)]
pub struct MeasurementUpdateSummary<T: EstimatorScalar, const N: usize, const M: usize> {
    pub status: MeasurementUpdateStatus,
    pub innovation: VecN<T, M>,
    pub label: MeasurementLabel,
    pub mahalanobis_distance: Option<T>,
    pub innovation_covariance: Option<MatN<T, M>>,
    pub state_correction: Option<VecN<T, N>>,
    pub gate_reason: Option<&'static str>,
    pub diagnostics: Option<MeasurementDiagnostics<T, N, M>>,
}

pub fn symmetrize_covariance<T: EstimatorScalar, const N: usize>(
    covariance: &MatN<T, N>,
) -> MatN<T, N> {
    let half = super::types::scalar::<T>(0.5);
    (covariance + covariance.transpose()) * half
}

pub fn apply_minimum_variance<T: EstimatorScalar, const N: usize>(
    covariance: &MatN<T, N>,
    min_variance: T,
) -> MatN<T, N> {
    let mut regularized = covariance.clone();
    for index in 0..N {
        let diagonal = regularized[(index, index)];
        regularized[(index, index)] = if diagonal < min_variance {
            min_variance
        } else {
            diagonal
        };
    }
    regularized
}

pub fn regularize_covariance<T: EstimatorScalar, const N: usize>(
    covariance: &MatN<T, N>,
    min_variance: T,
) -> MatN<T, N> {
    apply_minimum_variance(&symmetrize_covariance(covariance), min_variance)
}

pub fn compute_innovation_covariance<T: EstimatorScalar, const N: usize, const M: usize>(
    prior_covariance: &MatN<T, N>,
    measurement_jacobian: &MatMN<T, M, N>,
    measurement_covariance: &MatN<T, M>,
    min_variance: T,
) -> MatN<T, M> {
    let innovation_covariance =
        measurement_jacobian * prior_covariance * measurement_jacobian.transpose()
            + measurement_covariance;
    regularize_covariance(&innovation_covariance, min_variance)
}

fn solve_spd<T: EstimatorScalar, const M: usize, const C: usize>(
    system: &MatN<T, M>,
    rhs: &MatMN<T, M, C>,
) -> Option<MatMN<T, M, C>> {
    system
        .clone()
        .cholesky()
        .map(|cholesky| cholesky.solve(rhs))
}

pub fn mahalanobis_distance<T: EstimatorScalar, const M: usize>(
    innovation: &VecN<T, M>,
    innovation_covariance: &MatN<T, M>,
) -> Result<T, EstimationError> {
    let solved = solve_spd::<T, M, 1>(innovation_covariance, innovation)
        .ok_or(EstimationError::SingularInnovationCovariance)?;
    Ok(innovation.dot(&solved))
}

pub fn compute_kalman_gain<T: EstimatorScalar, const N: usize, const M: usize>(
    prior_covariance: &MatN<T, N>,
    measurement_jacobian: &MatMN<T, M, N>,
    innovation_covariance: &MatN<T, M>,
) -> Result<MatMN<T, N, M>, EstimationError> {
    let ph_t = prior_covariance * measurement_jacobian.transpose();
    let solved = solve_spd::<T, M, N>(innovation_covariance, &ph_t.transpose())
        .ok_or(EstimationError::SingularInnovationCovariance)?;
    Ok(solved.transpose())
}

pub fn joseph_covariance_update<T: EstimatorScalar, const N: usize, const M: usize>(
    prior_covariance: &MatN<T, N>,
    measurement_jacobian: &MatMN<T, M, N>,
    measurement_covariance: &MatN<T, M>,
    kalman_gain: &MatMN<T, N, M>,
) -> MatN<T, N> {
    let identity = MatN::<T, N>::identity();
    let joseph_left = identity - kalman_gain * measurement_jacobian;
    joseph_left * prior_covariance * joseph_left.transpose()
        + kalman_gain * measurement_covariance * kalman_gain.transpose()
}
