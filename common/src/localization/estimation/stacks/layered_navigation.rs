//! Layered navigation composition preserving the Python estimator structure.

use crate::localization::estimation::core::{
    CovarianceConvention, EstimationError, GenericEkf, GenericEskf, MatN, Quaternion, Vec3, scalar,
};
use crate::localization::estimation::math::{identity_quaternion, quaternion_to_rotation_matrix};
use crate::localization::estimation::measurements::{
    BarometricAltitudeMeasurementModel, GpsPositionMeasurementModel, GpsVelocityMeasurementModel,
    GravityAlignmentMeasurementModel,
};
use crate::localization::estimation::models::{
    ATTITUDE_STATE_DIM, AttitudeErrorStateProcessModel, AttitudeInput, AttitudeState,
    NAVIGATION_STATE_DIM, NavigationInput, NavigationProcessModel, NavigationState,
};
use crate::localization::estimation::policies::MeasurementGate;

#[derive(Clone, Debug)]
pub struct LayeredNavigationState<T: crate::localization::estimation::core::EstimatorScalar> {
    pub quaternion: Quaternion<T>,
    pub gyro_bias_rps: Vec3<T>,
    pub position_m: Vec3<T>,
    pub velocity_mps: Vec3<T>,
    pub accel_bias_mps2: Vec3<T>,
}

impl<T: crate::localization::estimation::core::EstimatorScalar> Default
    for LayeredNavigationState<T>
{
    fn default() -> Self {
        Self {
            quaternion: identity_quaternion(),
            gyro_bias_rps: Vec3::<T>::zeros(),
            position_m: Vec3::<T>::zeros(),
            velocity_mps: Vec3::<T>::zeros(),
            accel_bias_mps2: Vec3::<T>::zeros(),
        }
    }
}

impl<T: crate::localization::estimation::core::EstimatorScalar> LayeredNavigationState<T> {
    pub fn to_attitude_state(&self) -> AttitudeState<T> {
        AttitudeState {
            quaternion: self.quaternion,
            gyro_bias_rps: self.gyro_bias_rps,
        }
    }

    pub fn to_navigation_state(&self) -> NavigationState<T> {
        NavigationState {
            position_m: self.position_m,
            velocity_mps: self.velocity_mps,
            accel_bias_mps2: self.accel_bias_mps2,
        }
    }
}

#[derive(Clone, Debug)]
pub struct LayeredNavigationCovariance<T: crate::localization::estimation::core::EstimatorScalar> {
    pub attitude: MatN<T, ATTITUDE_STATE_DIM>,
    pub navigation: MatN<T, NAVIGATION_STATE_DIM>,
}

impl<T: crate::localization::estimation::core::EstimatorScalar> Default
    for LayeredNavigationCovariance<T>
{
    fn default() -> Self {
        Self {
            attitude: default_attitude_initial_covariance(),
            navigation: default_navigation_initial_covariance(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct LayeredNavigationStackConfig<T: crate::localization::estimation::core::EstimatorScalar> {
    pub initial_state: LayeredNavigationState<T>,
    pub initial_covariance: LayeredNavigationCovariance<T>,
    pub attitude_process_model: AttitudeErrorStateProcessModel<T>,
    pub navigation_process_model: NavigationProcessModel<T>,
    pub gravity_alignment_model: GravityAlignmentMeasurementModel<T>,
    pub position_measurement_model: GpsPositionMeasurementModel<T>,
    pub velocity_measurement_model: GpsVelocityMeasurementModel<T>,
    pub barometric_altitude_model: BarometricAltitudeMeasurementModel<T>,
    pub attitude_covariance_convention: CovarianceConvention<T>,
    pub navigation_covariance_convention: CovarianceConvention<T>,
}

impl<T: crate::localization::estimation::core::EstimatorScalar> Default
    for LayeredNavigationStackConfig<T>
{
    fn default() -> Self {
        Self {
            initial_state: LayeredNavigationState::default(),
            initial_covariance: LayeredNavigationCovariance::default(),
            attitude_process_model: AttitudeErrorStateProcessModel::default(),
            navigation_process_model: NavigationProcessModel::default(),
            gravity_alignment_model: GravityAlignmentMeasurementModel::default(),
            position_measurement_model: GpsPositionMeasurementModel::default(),
            velocity_measurement_model: GpsVelocityMeasurementModel::default(),
            barometric_altitude_model: BarometricAltitudeMeasurementModel::default(),
            attitude_covariance_convention: CovarianceConvention::default(),
            navigation_covariance_convention: CovarianceConvention::default(),
        }
    }
}

pub struct LayeredNavigationStack<T: crate::localization::estimation::core::EstimatorScalar> {
    pub attitude_process_model: AttitudeErrorStateProcessModel<T>,
    pub navigation_process_model: NavigationProcessModel<T>,
    pub gravity_alignment_model: GravityAlignmentMeasurementModel<T>,
    pub position_measurement_model: GpsPositionMeasurementModel<T>,
    pub velocity_measurement_model: GpsVelocityMeasurementModel<T>,
    pub barometric_altitude_model: BarometricAltitudeMeasurementModel<T>,
    pub attitude: GenericEskf<
        AttitudeState<T>,
        AttitudeInput<T>,
        AttitudeErrorStateProcessModel<T>,
        T,
        ATTITUDE_STATE_DIM,
        ATTITUDE_STATE_DIM,
    >,
    pub navigation: GenericEkf<
        NavigationState<T>,
        NavigationInput<T>,
        NavigationProcessModel<T>,
        T,
        NAVIGATION_STATE_DIM,
        NAVIGATION_STATE_DIM,
    >,
    pub last_timestamp_s: Option<T>,
    pub last_prediction_dt_s: Option<T>,
    pub last_inertial_acceleration_mps2: Vec3<T>,
}

impl<T: crate::localization::estimation::core::EstimatorScalar> Default
    for LayeredNavigationStack<T>
{
    fn default() -> Self {
        Self::from_config(LayeredNavigationStackConfig::default())
    }
}

impl<T: crate::localization::estimation::core::EstimatorScalar> LayeredNavigationStack<T> {
    pub fn from_config(config: LayeredNavigationStackConfig<T>) -> Self {
        let attitude_process_model = config.attitude_process_model;
        let navigation_process_model = config.navigation_process_model.clone();

        Self {
            attitude_process_model,
            navigation_process_model: navigation_process_model.clone(),
            gravity_alignment_model: config.gravity_alignment_model,
            position_measurement_model: config.position_measurement_model,
            velocity_measurement_model: config.velocity_measurement_model,
            barometric_altitude_model: config.barometric_altitude_model,
            attitude: GenericEskf::new(
                attitude_process_model,
                config.initial_state.to_attitude_state(),
                config.initial_covariance.attitude,
                config.attitude_covariance_convention,
            ),
            navigation: GenericEkf::new(
                navigation_process_model,
                config.initial_state.to_navigation_state(),
                config.initial_covariance.navigation,
                config.navigation_covariance_convention,
            ),
            last_timestamp_s: None,
            last_prediction_dt_s: None,
            last_inertial_acceleration_mps2: Vec3::<T>::zeros(),
        }
    }

    pub fn state(&self) -> LayeredNavigationState<T> {
        LayeredNavigationState {
            quaternion: self.attitude.state.quaternion,
            gyro_bias_rps: self.attitude.state.gyro_bias_rps,
            position_m: self.navigation.state.position_m,
            velocity_mps: self.navigation.state.velocity_mps,
            accel_bias_mps2: self.navigation.state.accel_bias_mps2,
        }
    }

    pub fn covariance(&self) -> LayeredNavigationCovariance<T> {
        LayeredNavigationCovariance {
            attitude: self.attitude.covariance,
            navigation: self.navigation.covariance,
        }
    }

    pub fn predict(
        &mut self,
        accelerometer_mps2: Vec3<T>,
        gyroscope_rps: Vec3<T>,
        dt: T,
        timestamp_s: Option<T>,
    ) -> Result<(), EstimationError> {
        self.attitude
            .predict(&AttitudeInput { gyroscope_rps }, dt)?;
        let rotation_body_to_inertial =
            quaternion_to_rotation_matrix(&self.attitude.state.quaternion);
        let navigation_control = NavigationInput {
            accelerometer_mps2,
            rotation_body_to_inertial,
        };
        self.last_inertial_acceleration_mps2 = self
            .navigation_process_model
            .inertial_acceleration(&self.navigation.state, &navigation_control);
        self.navigation.predict(&navigation_control, dt)?;

        self.last_prediction_dt_s = Some(dt);
        self.last_timestamp_s = timestamp_s;
        Ok(())
    }

    pub fn update_gravity_alignment(
        &mut self,
        accelerometer_mps2: Vec3<T>,
        measurement_gate: Option<
            &dyn MeasurementGate<AttitudeState<T>, T, ATTITUDE_STATE_DIM, 3, Vec3<T>>,
        >,
    ) -> Result<
        crate::localization::estimation::core::MeasurementUpdateSummary<T, ATTITUDE_STATE_DIM, 3>,
        EstimationError,
    > {
        self.attitude.update(
            &self.gravity_alignment_model,
            &accelerometer_mps2,
            measurement_gate,
        )
    }

    pub fn update_position(
        &mut self,
        position_m: Vec3<T>,
        measurement_gate: Option<
            &dyn MeasurementGate<NavigationState<T>, T, NAVIGATION_STATE_DIM, 3, Vec3<T>>,
        >,
    ) -> Result<
        crate::localization::estimation::core::MeasurementUpdateSummary<T, NAVIGATION_STATE_DIM, 3>,
        EstimationError,
    > {
        self.navigation.update(
            &self.position_measurement_model,
            &position_m,
            measurement_gate,
        )
    }

    pub fn update_velocity(
        &mut self,
        velocity_mps: Vec3<T>,
        measurement_gate: Option<
            &dyn MeasurementGate<NavigationState<T>, T, NAVIGATION_STATE_DIM, 3, Vec3<T>>,
        >,
    ) -> Result<
        crate::localization::estimation::core::MeasurementUpdateSummary<T, NAVIGATION_STATE_DIM, 3>,
        EstimationError,
    > {
        self.navigation.update(
            &self.velocity_measurement_model,
            &velocity_mps,
            measurement_gate,
        )
    }

    pub fn update_barometric_altitude(
        &mut self,
        altitude_m: T,
        measurement_gate: Option<
            &dyn MeasurementGate<NavigationState<T>, T, NAVIGATION_STATE_DIM, 1, T>,
        >,
    ) -> Result<
        crate::localization::estimation::core::MeasurementUpdateSummary<T, NAVIGATION_STATE_DIM, 1>,
        EstimationError,
    > {
        self.navigation.update(
            &self.barometric_altitude_model,
            &altitude_m,
            measurement_gate,
        )
    }
}

fn default_attitude_initial_covariance<
    T: crate::localization::estimation::core::EstimatorScalar,
>() -> MatN<T, ATTITUDE_STATE_DIM> {
    let deg_to_rad = scalar::<T>(core::f64::consts::PI / 180.0);
    let attitude_variance = (scalar::<T>(10.0) * deg_to_rad).powi(2);
    let bias_variance = ((scalar::<T>(1.0) / scalar::<T>(60.0)) * deg_to_rad).powi(2);
    let mut covariance = MatN::<T, ATTITUDE_STATE_DIM>::zeros();
    for axis in 0..3 {
        covariance[(axis, axis)] = attitude_variance;
        covariance[(axis + 3, axis + 3)] = bias_variance;
    }
    covariance
}

fn default_navigation_initial_covariance<
    T: crate::localization::estimation::core::EstimatorScalar,
>() -> MatN<T, NAVIGATION_STATE_DIM> {
    let mut covariance = MatN::<T, NAVIGATION_STATE_DIM>::zeros();
    for axis in 0..3 {
        covariance[(axis, axis)] = scalar(100.0);
        covariance[(axis + 3, axis + 3)] = scalar(25.0);
        covariance[(axis + 6, axis + 6)] = scalar::<T>(0.5).powi(2);
    }
    covariance
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layered_stack_predict_and_measurement_updates_smoke() {
        let mut stack = LayeredNavigationStack::<f64>::default();
        stack
            .predict(
                Vec3::<f64>::new(0.0, 0.0, -9.80665),
                Vec3::<f64>::zeros(),
                0.01,
                Some(0.01),
            )
            .unwrap();
        stack
            .update_gravity_alignment(Vec3::<f64>::new(0.0, 0.0, -9.80665), None)
            .unwrap();
        stack
            .update_position(Vec3::<f64>::new(1.0, 2.0, 3.0), None)
            .unwrap();
        stack.update_velocity(Vec3::<f64>::zeros(), None).unwrap();
        stack.update_barometric_altitude(3.0, None).unwrap();
    }
}
