//! Layered navigation composition preserving the Python estimator structure.

use crate::localization::estimation::core::{
    CovarianceConvention, EstimationError, GenericEkf, GenericEskf, MatN, Quaternion, Vec3, scalar,
};
use crate::localization::estimation::math::{identity_quaternion, quaternion_to_rotation_matrix};
use crate::localization::estimation::measurements::{
    BarometricAltitudeMeasurementModel, GpsPositionMeasurementModel, GpsVelocityMeasurementModel,
    GravityAlignmentMeasurementModel, MagneticFieldMeasurementModel,
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

    pub fn update_magnetic_field(
        &mut self,
        magnetic_field_body_ut: Vec3<T>,
        magnetic_field_inertial_ut: Vec3<T>,
        measurement_gate: Option<
            &dyn MeasurementGate<AttitudeState<T>, T, ATTITUDE_STATE_DIM, 3, Vec3<T>>,
        >,
    ) -> Result<
        crate::localization::estimation::core::MeasurementUpdateSummary<T, ATTITUDE_STATE_DIM, 3>,
        EstimationError,
    > {
        let model = MagneticFieldMeasurementModel::new(magnetic_field_inertial_ut);
        self.attitude
            .update(&model, &magnetic_field_body_ut, measurement_gate)
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
    use crate::localization::estimation::core::{MeasurementUpdateStatus, Vec4};
    use crate::localization::estimation::models::NavigationState;
    use crate::localization::estimation::policies::{GateContext, GateDecision, GateStatus};
    use crate::test_helpers::{
        assert_quaternion_near_f64, assert_quaternion_normalized_f64, assert_scalar_near_f64,
        assert_vec3_near_f64,
    };

    const GRAVITY_MPS2: f64 = 9.80665;
    const TOLERANCE: f64 = 1.0e-9;

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
            .update_magnetic_field(
                Vec3::<f64>::new(20.0, 0.0, 40.0),
                Vec3::<f64>::new(20.0, 0.0, 40.0),
                None,
            )
            .unwrap();
        stack
            .update_position(Vec3::<f64>::new(1.0, 2.0, 3.0), None)
            .unwrap();
        stack.update_velocity(Vec3::<f64>::zeros(), None).unwrap();
        stack.update_barometric_altitude(3.0, None).unwrap();
    }

    #[test]
    fn default_state_is_finite_and_quaternion_normalized() {
        let stack = LayeredNavigationStack::<f64>::default();
        let state = stack.state();

        assert_quaternion_normalized_f64(state.quaternion, TOLERANCE);
        assert_vec3_finite(state.gyro_bias_rps);
        assert_vec3_finite(state.position_m);
        assert_vec3_finite(state.velocity_mps);
        assert_vec3_finite(state.accel_bias_mps2);
    }

    #[test]
    fn predict_stores_dt_and_timestamp() {
        let mut stack = LayeredNavigationStack::<f64>::default();

        stack
            .predict(Vec3::<f64>::zeros(), Vec3::<f64>::zeros(), 0.02, Some(1.25))
            .unwrap();

        assert_eq!(stack.last_prediction_dt_s, Some(0.02));
        assert_eq!(stack.last_timestamp_s, Some(1.25));
    }

    #[test]
    fn static_level_predict_loop_remains_bounded() {
        let mut stack = LayeredNavigationStack::<f64>::default();

        for step in 0..200 {
            stack
                .predict(
                    Vec3::<f64>::zeros(),
                    Vec3::<f64>::zeros(),
                    0.01,
                    Some((step + 1) as f64 * 0.01),
                )
                .unwrap();
        }

        let state = stack.state();
        assert_quaternion_near_f64(state.quaternion, identity_quaternion(), TOLERANCE);
        assert_vec3_near_f64(vec3_to_array(state.position_m), [0.0, 0.0, 0.0], TOLERANCE);
        assert_vec3_near_f64(
            vec3_to_array(state.velocity_mps),
            [0.0, 0.0, 0.0],
            TOLERANCE,
        );
    }

    #[test]
    fn constant_yaw_rate_integrates_quaternion() {
        let mut stack = LayeredNavigationStack::<f64>::default();

        stack
            .predict(
                Vec3::<f64>::zeros(),
                Vec3::<f64>::new(0.0, 0.0, core::f64::consts::FRAC_PI_2),
                1.0,
                Some(1.0),
            )
            .unwrap();

        assert_quaternion_near_f64(
            stack.state().quaternion,
            Vec4::<f64>::new(
                core::f64::consts::FRAC_1_SQRT_2,
                0.0,
                0.0,
                core::f64::consts::FRAC_1_SQRT_2,
            ),
            1.0e-9,
        );
    }

    #[test]
    fn gravity_update_keeps_level_state_stable() {
        let mut stack = LayeredNavigationStack::<f64>::default();

        let summary = stack
            .update_gravity_alignment(Vec3::<f64>::new(0.0, 0.0, -GRAVITY_MPS2), None)
            .unwrap();

        assert_eq!(summary.status, MeasurementUpdateStatus::Accepted);
        assert_quaternion_near_f64(stack.state().quaternion, identity_quaternion(), 1.0e-9);
        assert_quaternion_normalized_f64(stack.state().quaternion, 1.0e-9);
    }

    #[test]
    fn magnetic_update_keeps_aligned_field_stable() {
        let mut stack = LayeredNavigationStack::<f64>::default();

        let summary = stack
            .update_magnetic_field(
                Vec3::<f64>::new(20.0, 0.0, 40.0),
                Vec3::<f64>::new(20.0, 0.0, 40.0),
                None,
            )
            .unwrap();

        assert_eq!(summary.status, MeasurementUpdateStatus::Accepted);
        assert_quaternion_near_f64(stack.state().quaternion, identity_quaternion(), 1.0e-9);
        assert_quaternion_normalized_f64(stack.state().quaternion, 1.0e-9);
    }

    #[test]
    fn gps_position_update_moves_toward_measurement() {
        let mut stack = LayeredNavigationStack::<f64>::default();

        stack
            .update_position(Vec3::<f64>::new(10.0, -4.0, 2.0), None)
            .unwrap();

        let position = stack.state().position_m;
        assert!(position[0] > 0.0 && position[0] < 10.0);
        assert!(position[1] < 0.0 && position[1] > -4.0);
        assert!(position[2] > 0.0 && position[2] < 2.0);
    }

    #[test]
    fn gps_velocity_update_moves_toward_measurement() {
        let mut stack = LayeredNavigationStack::<f64>::default();

        stack
            .update_velocity(Vec3::<f64>::new(5.0, -2.0, 1.0), None)
            .unwrap();

        let velocity = stack.state().velocity_mps;
        assert!(velocity[0] > 0.0 && velocity[0] < 5.0);
        assert!(velocity[1] < 0.0 && velocity[1] > -2.0);
        assert!(velocity[2] > 0.0 && velocity[2] < 1.0);
    }

    #[test]
    fn barometric_altitude_update_moves_toward_measurement() {
        let mut stack = LayeredNavigationStack::<f64>::default();

        stack.update_barometric_altitude(12.0, None).unwrap();

        let down = stack.state().position_m[2];
        assert!(down > 0.0 && down < 12.0);
    }

    #[test]
    fn rejected_gps_position_measurement_leaves_state_unchanged() {
        let mut stack = LayeredNavigationStack::<f64>::default();
        let before = stack.state();
        let gate = RejectAllGate;

        let summary = stack
            .update_position(Vec3::<f64>::new(10.0, 0.0, 0.0), Some(&gate))
            .unwrap();
        let after = stack.state();

        assert_eq!(summary.status, MeasurementUpdateStatus::Rejected);
        assert_eq!(summary.gate_reason, Some("forced rejection"));
        assert!(summary.state_correction.is_none());
        assert_vec3_near_f64(
            vec3_to_array(after.position_m),
            vec3_to_array(before.position_m),
            TOLERANCE,
        );
        assert_vec3_near_f64(
            vec3_to_array(after.velocity_mps),
            vec3_to_array(before.velocity_mps),
            TOLERANCE,
        );
        assert_quaternion_near_f64(after.quaternion, before.quaternion, TOLERANCE);
    }

    #[test]
    fn deterministic_estimator_replay_from_fixed_sensor_arrays_is_bounded() {
        let mut stack = LayeredNavigationStack::<f64>::default();
        let replay = [
            ReplaySample {
                timestamp_s: 0.01,
                accel_mps2: [0.0, 0.0, 0.0],
                gyro_rps: [0.0, 0.0, 0.02],
                gps_position_m: [0.0, 0.0, 0.0],
                gps_velocity_mps: [0.0, 0.0, 0.0],
                baro_down_m: 0.0,
                mag_body_ut: [20.0, 0.0, 40.0],
            },
            ReplaySample {
                timestamp_s: 0.02,
                accel_mps2: [0.02, -0.01, 0.0],
                gyro_rps: [0.0, 0.0, 0.02],
                gps_position_m: [0.15, -0.04, 0.02],
                gps_velocity_mps: [0.05, -0.02, 0.0],
                baro_down_m: 0.02,
                mag_body_ut: [20.0, -0.004, 40.0],
            },
            ReplaySample {
                timestamp_s: 0.03,
                accel_mps2: [0.01, -0.02, 0.0],
                gyro_rps: [0.0, 0.0, 0.02],
                gps_position_m: [0.28, -0.08, 0.03],
                gps_velocity_mps: [0.08, -0.02, 0.01],
                baro_down_m: 0.03,
                mag_body_ut: [20.0, -0.008, 40.0],
            },
            ReplaySample {
                timestamp_s: 0.04,
                accel_mps2: [0.0, -0.01, 0.0],
                gyro_rps: [0.0, 0.0, 0.02],
                gps_position_m: [0.4, -0.1, 0.04],
                gps_velocity_mps: [0.1, -0.03, 0.01],
                baro_down_m: 0.04,
                mag_body_ut: [20.0, -0.012, 40.0],
            },
        ];

        let mut previous_timestamp_s = 0.0;
        for sample in replay {
            let dt = sample.timestamp_s - previous_timestamp_s;
            previous_timestamp_s = sample.timestamp_s;

            stack
                .predict(
                    Vec3::<f64>::new(
                        sample.accel_mps2[0],
                        sample.accel_mps2[1],
                        sample.accel_mps2[2],
                    ),
                    Vec3::<f64>::new(sample.gyro_rps[0], sample.gyro_rps[1], sample.gyro_rps[2]),
                    dt,
                    Some(sample.timestamp_s),
                )
                .unwrap();
            stack
                .update_gravity_alignment(Vec3::<f64>::new(0.0, 0.0, -GRAVITY_MPS2), None)
                .unwrap();
            stack
                .update_magnetic_field(
                    Vec3::<f64>::new(
                        sample.mag_body_ut[0],
                        sample.mag_body_ut[1],
                        sample.mag_body_ut[2],
                    ),
                    Vec3::<f64>::new(20.0, 0.0, 40.0),
                    None,
                )
                .unwrap();
            stack
                .update_position(
                    Vec3::<f64>::new(
                        sample.gps_position_m[0],
                        sample.gps_position_m[1],
                        sample.gps_position_m[2],
                    ),
                    None,
                )
                .unwrap();
            stack
                .update_velocity(
                    Vec3::<f64>::new(
                        sample.gps_velocity_mps[0],
                        sample.gps_velocity_mps[1],
                        sample.gps_velocity_mps[2],
                    ),
                    None,
                )
                .unwrap();
            stack
                .update_barometric_altitude(sample.baro_down_m, None)
                .unwrap();
        }

        let state = stack.state();
        assert_quaternion_normalized_f64(state.quaternion, 1.0e-9);
        assert_vec3_finite(state.position_m);
        assert_vec3_finite(state.velocity_mps);
        assert_scalar_near_f64(stack.last_prediction_dt_s.unwrap(), 0.01, 1.0e-12);
        assert_scalar_near_f64(stack.last_timestamp_s.unwrap(), 0.04, 1.0e-12);
        assert!(state.position_m[0] > 0.0 && state.position_m[0] < 0.4);
        assert!(state.position_m[1] < 0.0 && state.position_m[1] > -0.1);
        assert!(state.position_m[2] > 0.0 && state.position_m[2] < 0.04);
        assert!(state.velocity_mps[0] > 0.0 && state.velocity_mps[0] < 0.1);
        assert!(state.velocity_mps[1] < 0.0 && state.velocity_mps[1] > -0.03);
    }

    struct RejectAllGate;

    #[derive(Clone, Copy)]
    struct ReplaySample {
        timestamp_s: f64,
        accel_mps2: [f64; 3],
        gyro_rps: [f64; 3],
        gps_position_m: [f64; 3],
        gps_velocity_mps: [f64; 3],
        baro_down_m: f64,
        mag_body_ut: [f64; 3],
    }

    impl
        crate::localization::estimation::policies::MeasurementGate<
            NavigationState<f64>,
            f64,
            NAVIGATION_STATE_DIM,
            3,
            Vec3<f64>,
        > for RejectAllGate
    {
        fn evaluate(
            &self,
            _context: &GateContext<
                '_,
                NavigationState<f64>,
                f64,
                NAVIGATION_STATE_DIM,
                3,
                Vec3<f64>,
            >,
        ) -> GateDecision<f64> {
            GateDecision {
                status: GateStatus::Reject,
                reason: Some("forced rejection"),
                mahalanobis_distance: None,
            }
        }
    }

    fn assert_vec3_finite(values: Vec3<f64>) {
        assert!(values.iter().all(|value| value.is_finite()));
    }

    fn vec3_to_array(values: Vec3<f64>) -> [f64; 3] {
        [values[0], values[1], values[2]]
    }
}
