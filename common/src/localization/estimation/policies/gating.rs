//! External measurement gating contracts.

use crate::localization::estimation::core::{EstimatorScalar, MatN, MeasurementLabel, VecN};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GateStatus {
    Accept,
    Reject,
    Skip,
}

#[derive(Clone, Copy, Debug)]
pub struct GateDecision<T> {
    pub status: GateStatus,
    pub reason: Option<&'static str>,
    pub mahalanobis_distance: Option<T>,
}

impl<T> GateDecision<T> {
    pub const fn accept(mahalanobis_distance: Option<T>) -> Self {
        Self {
            status: GateStatus::Accept,
            reason: None,
            mahalanobis_distance,
        }
    }
}

pub struct GateContext<'a, S, T: EstimatorScalar, const N: usize, const M: usize, Measurement> {
    pub label: MeasurementLabel,
    pub measurement: &'a Measurement,
    pub nominal_state: &'a S,
    pub innovation: &'a VecN<T, M>,
    pub innovation_covariance: &'a MatN<T, M>,
}

pub trait MeasurementGate<S, T: EstimatorScalar, const N: usize, const M: usize, Measurement> {
    fn evaluate(&self, context: &GateContext<'_, S, T, N, M, Measurement>) -> GateDecision<T>;
}
