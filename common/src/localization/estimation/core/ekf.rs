//! Generic EKF engine over Euclidean nominal states.

use core::marker::PhantomData;

use super::base::{EuclideanProcessModel, EuclideanState, MeasurementModel};
use super::gaussian::{
    CovarianceConvention, EstimationError, MeasurementDiagnostics, MeasurementUpdateStatus,
    MeasurementUpdateSummary, PredictionDiagnostics, apply_minimum_variance,
    compute_innovation_covariance, compute_kalman_gain, joseph_covariance_update,
    mahalanobis_distance, symmetrize_covariance,
};
use super::types::{EstimatorScalar, MatN};
use crate::localization::estimation::policies::{
    GateContext, GateDecision, GateStatus, MeasurementGate,
};

pub struct GenericEkf<S, C, P, T: EstimatorScalar, const N: usize, const W: usize>
where
    S: EuclideanState<T, N>,
    P: EuclideanProcessModel<S, C, T, N, W>,
{
    pub process_model: P,
    pub state: S,
    pub covariance: MatN<T, N>,
    pub covariance_convention: CovarianceConvention<T>,
    _control: PhantomData<C>,
    #[cfg(feature = "estimation-diagnostics")]
    pub last_prediction: Option<PredictionDiagnostics<T, N, W>>,
}

impl<S, C, P, T: EstimatorScalar, const N: usize, const W: usize> GenericEkf<S, C, P, T, N, W>
where
    S: EuclideanState<T, N>,
    P: EuclideanProcessModel<S, C, T, N, W>,
{
    pub fn new(
        process_model: P,
        initial_state: S,
        initial_covariance: MatN<T, N>,
        covariance_convention: CovarianceConvention<T>,
    ) -> Self {
        Self {
            process_model,
            state: initial_state.copy(),
            covariance: regularize_covariance::<T, N>(
                &initial_covariance,
                &covariance_convention,
                false,
            ),
            covariance_convention,
            _control: PhantomData,
            #[cfg(feature = "estimation-diagnostics")]
            last_prediction: None,
        }
    }

    pub fn predict(
        &mut self,
        control: &C,
        dt: T,
    ) -> Result<PredictionDiagnostics<T, N, W>, EstimationError> {
        if dt <= T::zero() {
            let diagnostics = PredictionDiagnostics {
                transition_jacobian: MatN::<T, N>::identity(),
                process_noise_jacobian: nalgebra::SMatrix::<T, N, W>::zeros(),
                process_noise_covariance: MatN::<T, W>::zeros(),
            };
            #[cfg(feature = "estimation-diagnostics")]
            {
                self.last_prediction = Some(diagnostics.clone());
            }
            return Ok(diagnostics);
        }

        let prior_state = self.state.copy();
        let transition_jacobian = self.process_model.state_jacobian(&prior_state, control, dt);
        let process_noise_jacobian =
            self.process_model
                .process_noise_jacobian(&prior_state, control, dt);
        let process_noise_covariance =
            self.process_model
                .process_noise_covariance(&prior_state, control, dt);

        let predicted_covariance =
            transition_jacobian * self.covariance * transition_jacobian.transpose()
                + process_noise_jacobian
                    * process_noise_covariance
                    * process_noise_jacobian.transpose();
        self.state = self.process_model.predict(&prior_state, control, dt).copy();
        self.covariance = regularize_covariance::<T, N>(
            &predicted_covariance,
            &self.covariance_convention,
            false,
        );

        let diagnostics = PredictionDiagnostics {
            transition_jacobian,
            process_noise_jacobian,
            process_noise_covariance,
        };
        #[cfg(feature = "estimation-diagnostics")]
        {
            self.last_prediction = Some(diagnostics.clone());
        }
        Ok(diagnostics)
    }

    pub fn update<Model, const M: usize>(
        &mut self,
        measurement_model: &Model,
        measurement: &Model::Measurement,
        measurement_gate: Option<&dyn MeasurementGate<S, T, N, M, Model::Measurement>>,
    ) -> Result<MeasurementUpdateSummary<T, N, M>, EstimationError>
    where
        Model: MeasurementModel<S, T, N, M>,
    {
        let predicted_measurement = measurement_model.predict_measurement(&self.state);
        let innovation = measurement_model.innovation(measurement, &predicted_measurement);
        let measurement_jacobian = measurement_model.measurement_jacobian(measurement, &self.state);
        let measurement_covariance =
            measurement_model.measurement_covariance(measurement, &self.state);
        let innovation_covariance = compute_innovation_covariance(
            &self.covariance,
            &measurement_jacobian,
            &measurement_covariance,
            self.covariance_convention.min_variance,
        );
        let distance = mahalanobis_distance(&innovation, &innovation_covariance)?;
        let diagnostics = MeasurementDiagnostics {
            predicted_measurement,
            innovation,
            measurement_jacobian,
            measurement_covariance,
            innovation_covariance,
        };

        let gate_decision = measurement_gate
            .map(|gate| {
                gate.evaluate(&GateContext {
                    label: measurement_model.label(),
                    measurement,
                    nominal_state: &self.state,
                    innovation: &diagnostics.innovation,
                    innovation_covariance: &diagnostics.innovation_covariance,
                })
            })
            .unwrap_or_else(|| GateDecision::accept(Some(distance)));

        if gate_decision.status != GateStatus::Accept {
            return Ok(MeasurementUpdateSummary {
                status: map_gate_status(gate_decision.status),
                innovation: diagnostics.innovation,
                label: measurement_model.label(),
                mahalanobis_distance: gate_decision.mahalanobis_distance.or(Some(distance)),
                innovation_covariance: Some(diagnostics.innovation_covariance),
                state_correction: None,
                gate_reason: gate_decision.reason,
                diagnostics: Some(diagnostics),
            });
        }

        let kalman_gain = compute_kalman_gain(
            &self.covariance,
            &diagnostics.measurement_jacobian,
            &diagnostics.innovation_covariance,
        )?;
        let state_correction = kalman_gain * diagnostics.innovation;
        let updated_covariance = joseph_covariance_update(
            &self.covariance,
            &diagnostics.measurement_jacobian,
            &diagnostics.measurement_covariance,
            &kalman_gain,
        );
        self.state = self.state.plus(&state_correction);
        self.covariance =
            regularize_covariance::<T, N>(&updated_covariance, &self.covariance_convention, true);

        Ok(MeasurementUpdateSummary {
            status: MeasurementUpdateStatus::Accepted,
            innovation: diagnostics.innovation,
            label: measurement_model.label(),
            mahalanobis_distance: Some(distance),
            innovation_covariance: Some(diagnostics.innovation_covariance),
            state_correction: Some(state_correction),
            gate_reason: None,
            diagnostics: Some(diagnostics),
        })
    }
}

fn regularize_covariance<T: EstimatorScalar, const N: usize>(
    covariance: &MatN<T, N>,
    convention: &CovarianceConvention<T>,
    after_update: bool,
) -> MatN<T, N> {
    let should_symmetrize = if after_update {
        convention.symmetrize_after_update
    } else {
        convention.symmetrize_after_predict
    };
    let covariance = if should_symmetrize {
        symmetrize_covariance(covariance)
    } else {
        covariance.clone()
    };
    if convention.min_variance > T::zero() {
        apply_minimum_variance(&covariance, convention.min_variance)
    } else {
        covariance
    }
}

fn map_gate_status(status: GateStatus) -> MeasurementUpdateStatus {
    match status {
        GateStatus::Accept => MeasurementUpdateStatus::Accepted,
        GateStatus::Reject => MeasurementUpdateStatus::Rejected,
        GateStatus::Skip => MeasurementUpdateStatus::Skipped,
    }
}
