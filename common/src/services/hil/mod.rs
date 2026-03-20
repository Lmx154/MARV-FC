//! Reusable hardware-in-the-loop framework services.

pub mod backend;
pub mod egress;
pub mod ingress;
pub mod model;
pub mod routing;
pub mod runtime;
pub mod tasks;

pub use backend::SensorBackend;
pub use egress::{HilByteWriter, HilEgressProtocol};
pub use ingress::{HilByteReader, HilIngressProtocol};
pub use model::{
    HilActuatorCommand, HilBarometerSample, HilEgressMessage, HilGpsSample, HilImuSample,
    HilIngressMessage, HilMagSample, HilMissionEvent, HilTick,
};
pub use routing::{
    HilBarometerRoute, HilGpsRoute, HilImuRoute, HilIngressRoutes, HilMagnetometerRoute,
    HilTimeRoute,
};
pub use runtime::{HilDispatch, HilRuntime};
pub use tasks::{HilIngressLoopError, run_hil_ingress_loop, send_hil_egress_message};
