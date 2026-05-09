//! Reusable hardware-in-the-loop framework services.

pub mod backend;
pub mod boot;
pub mod egress;
pub mod events;
pub mod frame_adapter;
pub mod ingress;
pub mod model;
pub mod routing;
pub mod runtime;
pub mod tasks;

pub use backend::SensorBackend;
pub use boot::{
    HilBootDecision, HilBootSelector, InitControlCommandOutcome, UsbHilMode,
    evaluate_usb_init_control_command, should_report_init_probe_liveness, usb_hil_mode_for_phase,
};
pub use egress::{HilByteWriter, HilEgressProtocol};
pub use events::{MAV_CMD_MARV_RUNTIME_PHASE_EVENT, runtime_phase_event};
pub use frame_adapter::{
    HilCorrelationState, HilSensorFrameAdapter, HilSensorFrameDispatch, HilSensorFrameRejection,
    HilValidationCounters,
};
pub use ingress::{HilByteReader, HilIngressProtocol};
pub use model::{
    HilActuatorCommand, HilBarometerSample, HilCommandAck, HilCommandAckResult, HilControlAction,
    HilControlCommand, HilEgressMessage, HilGpsSample, HilImuSample, HilIngressMessage,
    HilMagSample, HilMissionEvent, HilSessionState, HilSubmode, HilTick,
};
pub use routing::{
    HilBarometerRoute, HilControlCommandRoute, HilGpsRoute, HilImuRoute, HilIngressRoutes,
    HilMagnetometerRoute, HilTimeRoute,
};
pub use runtime::{HilControlCommandOutcome, HilControlRuntime, HilDispatch, HilRuntime};
pub use tasks::{
    HilControlCommandReceiver, HilEgressSender, HilEgressTrySender, HilIngressLoopError,
    HilSessionStateSender, run_hil_control_command_loop, run_hil_ingress_loop,
    send_hil_egress_message,
};
