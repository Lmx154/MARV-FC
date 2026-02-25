//! Hardware-specific modules for RP2350
//!
//! Separates hardware I/O concerns from application logic

pub mod pinout;
pub mod sd_params;
pub mod usbcdc;

pub use sd_params::{load_config_and_params_from_sd, save_params_to_sd};
pub use usbcdc::{RpUsbDevice, UsbCdc, build_usb_cdc};
