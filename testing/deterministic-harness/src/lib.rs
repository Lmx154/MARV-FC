//! Host-side validation support for MARV GNC tests.
//!
//! The cleanup-first reset keeps only production-seam helpers here. Historical
//! phase-numbered tests and synthetic plant/replay scaffolding have been
//! removed from the active harness surface.

pub const PHASE_0_CONTRACTS_FROZEN: bool = true;

pub mod gazebo_contract;
pub mod hil_semantics;

pub use gazebo_contract::{
    GAZEBO_G0_DEFAULT_ENDPOINT, GazeboActuatorAxis, GazeboActuatorExpectation,
    GazeboActuatorTruthCase, GazeboAirframeConfig, GazeboAirframeMotorConfig, GazeboBodyFrame,
    GazeboBridgeConfig, GazeboRotorSpin, GazeboSensorLine, GazeboSimControlAction,
    GazeboTruthOrigin, SensorFrame, format_gazebo_actuator_line, format_gazebo_sim_control_line,
    gazebo_g0_actuator_truth_cases,
};
pub use hil_semantics::{
    HilPublishedGroups, HilRouteRecorder, HilSemanticAdapter, HilSemanticArchive,
    HilSemanticFrameConfig, HilSemanticTrace, hil_response_from_samples, hil_valid_flags,
    response_has_flag, response_is_failsafe_zero_output, sensor_frame_to_hil_frame,
    stale_hil_response_from_samples,
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
