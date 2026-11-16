//! The SD card's one parameter (decision-sd-logging Q6): the log backend, read once at boot.

use fw_core::params::{Apply, ParamDef};
use mavlink_device::marv::MarvDevice;

const DEVICE: u8 = MarvDevice::MARV_DEVICE_SD as u8;

/// The log backend (ArduPilot's name): 0 none (the card is not touched), 1 file on the SD card.
/// Takes effect at the next boot.
pub const BACKEND_TYPE: &str = "LOG_BACKEND_TYPE";

/// The number of SD parameters.
pub const COUNT: usize = 1;

/// The SD card's parameter table.
pub static TABLE: [ParamDef; COUNT] = [ParamDef::i32(BACKEND_TYPE, DEVICE, 1, 0, 1, Apply::Reboot)];

/// Whether the SD card is to be used, from the stored `LOG_BACKEND_TYPE` (`None`: the default, 1).
pub fn file_backend(backend_type: Option<i32>) -> bool {
    backend_type.unwrap_or(1) != 0
}

#[cfg(test)]
mod tests {
    use super::*;
    use fw_core::params::Params;
    use mavlink_device::{ParamError, ParamStore, ParamValue};

    #[test]
    fn the_parameter_is_reboot_only_and_0_or_1() {
        let mut params = Params::<COUNT>::new(&TABLE).unwrap();
        assert_eq!(params.count() as usize, COUNT);
        let def = &TABLE[0];
        assert_eq!(def.name, "LOG_BACKEND_TYPE");
        assert!(def.name.len() <= 16);
        assert_eq!(def.device, DEVICE);
        assert_eq!(def.apply, Apply::Reboot);
        assert!(!def.calibration);
        assert_eq!(def.default, ParamValue::I32(1));
        assert_eq!(params.get(0), Some(ParamValue::I32(1)));
        assert_eq!(params.set(0, ParamValue::I32(0)), Ok(ParamValue::I32(0)));
        assert_eq!(params.set(0, ParamValue::I32(2)), Err(ParamError::OutOfRange));
        assert_eq!(params.set(0, ParamValue::I32(-1)), Err(ParamError::OutOfRange));
        assert_eq!(params.get(0), Some(ParamValue::I32(0)));
        // Reboot: nothing to configure and nothing live.
        assert_eq!(params.take_configure_mask(), 0);
        assert_eq!(params.take_live_mask(), 0);
    }

    #[test]
    fn only_0_turns_the_card_off() {
        assert!(file_backend(None));
        assert!(file_backend(Some(1)));
        assert!(!file_backend(Some(0)));
    }
}
