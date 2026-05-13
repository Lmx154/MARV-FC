//! Harness-facing aliases for the canonical portable control step.

pub use common::control::pipeline::{
    ClampSource, ControlDebug, ControlFrame, ControlInput, ControlNavPhase,
    ControlOutput as ControlPipelineTrace, ControlSetpoint, ControlSetpointSource,
    EstimateSnapshot, EstimatorLocalState, EstimatorResetDelta,
    FlightControlConfig as PureControlConfig, FlightControlPipeline as ControlPipeline,
    ImuControlInput, LocalTrajectorySetpoint, MissionWaypoint, TruthEvidence,
};

pub use common::control::guidance::{
    GuidanceCommand, GuidanceOutput, GuidancePhase, GuidanceStateMachine, TrajectoryGenerator,
    TrajectoryLimits,
};
pub use common::control::mixing::MixerLimitFlags;
pub use common::control::position::{
    ThrustVectorConfig, ThrustVectorController, ThrustVectorSetpoint,
};
pub use common::control::rate::{RateControllerOutput, RateLimitFlags};
