//! Harness-facing aliases for the canonical portable control step.

pub use common::control::pipeline::{
    ControlDebug, ControlInput, ControlOutput as ControlPipelineTrace, ControlSetpoint,
    EstimateSnapshot, FlightControlConfig as PureControlConfig,
    FlightControlPipeline as ControlPipeline, ImuControlInput,
};
