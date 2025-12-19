// Compatibility shim: LoRa transport moved under `crate::coms::transport::lora`.
// Keep `crate::lora::*` working to minimize churn in device crates.

pub mod lora_config {
	pub use crate::coms::transport::lora::lora_config::*;
}

pub mod link {
	pub use crate::coms::transport::lora::link::*;
}
