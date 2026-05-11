//! Host-side deterministic test harness for MARV.
//!
//! The first harness increments establish host-runnable contracts, lockstep
//! time, deterministic fixture loading, and failure reports before later phases
//! add plant models and MARV pipeline adapters.

pub const PHASE_0_CONTRACTS_FROZEN: bool = true;

pub mod cases;
pub mod clock;
pub mod closed_loop;
pub mod closed_loop_estimator;
pub mod control_pipeline;
pub mod estimator_replay;
pub mod fixtures;
pub mod gazebo_contract;
pub mod hil_semantics;
pub mod plant;

pub use cases::{
    HarnessFailure, HarnessReport, HarnessResult, HarnessTrace, StepTrace, TestCase,
    assert_all_finite, assert_unit_interval,
};
pub use clock::{ClockSnapshot, ClockStep, LockstepClock};
pub use closed_loop::{
    ClosedLoopConfig, ClosedLoopRunner, ClosedLoopTrace, ClosedLoopTruthState, TruthPlant,
};
pub use closed_loop_estimator::{
    BaroSpike, ClosedLoopEstimatorConfig, ClosedLoopEstimatorReport, ClosedLoopEstimatorRunner,
    ClosedLoopEstimatorTrace, SimulatedSensorConfig, simulated_sensors,
};
pub use control_pipeline::{
    ControlPipeline, ControlPipelineTrace, ControlSetpoint, EstimateSnapshot, ImuControlInput,
    PureControlConfig,
};
pub use estimator_replay::{
    EstimatorReplayConfig, EstimatorReplayDriver, EstimatorReplayReport, EstimatorReplayTrace,
    SensorFrame, replay_estimator_frames,
};
pub use fixtures::{Fixture, FixtureSample};
pub use gazebo_contract::{GazeboBridgeConfig, GazeboSensorLine};
pub use hil_semantics::{
    HilPublishedGroups, HilRouteRecorder, HilSemanticAdapter, HilSemanticArchive,
    HilSemanticFrameConfig, HilSemanticTrace, hil_response_from_pipeline, hil_valid_flags,
    response_has_flag, response_is_failsafe_zero_output, sensor_frame_to_hil_frame,
    stale_hil_response_from_pipeline,
};
pub use plant::{
    MotorGeometry, MotorSpec, OpenLoopPlant, OpenLoopResponse, SpinDirection, StaticPlantState,
};

#[cfg(test)]
mod tests {
    use common::control::mixing::{QUAD_MOTOR_COUNT, QuadXMixer, TorqueCommand};

    #[test]
    fn skeleton_links_against_common_control_contracts() {
        let outputs = QuadXMixer::default().mix(TorqueCommand::new(0.0, 0.0, 0.0, 0.25));

        assert_eq!(outputs.commands.len(), QUAD_MOTOR_COUNT);
        assert_eq!(outputs.commands, [0.25; QUAD_MOTOR_COUNT]);
        assert!(!outputs.clamped);
    }
}
