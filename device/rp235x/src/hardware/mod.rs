//! Hardware-specific modules for RP2350
//! 
//! Separates hardware I/O concerns from application logic

pub mod sd_params;

pub use sd_params::{load_config_and_params_from_sd, save_params_to_sd};
