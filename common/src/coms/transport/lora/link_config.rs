//! Link-level pairing of RF + MAC configs with derived helpers.
use super::link_profile::LinkProfile;
use super::mac_config::{self, MacConfig};
use super::mac_presets;
use super::rf_config::RfConfig;
use super::rf_presets;

#[derive(Clone, Copy, Debug)]
pub struct LinkConfig {
    pub rf: RfConfig,
    pub mac: MacConfig,
}

impl LinkConfig {
    pub const fn new(rf: RfConfig, mac: MacConfig) -> Self {
        Self { rf, mac }
    }

    pub fn rx_timeout_symbols(self) -> u16 {
        mac_config::rx_timeout_symbols(self.rf, self.mac)
    }

    pub fn slot_rx_symbols(self) -> u16 {
        mac_config::slot_rx_symbols(self.rf, self.mac)
    }

    pub fn profile(self) -> LinkProfile {
        LinkProfile::from_configs(self.rf, self.mac)
    }
}

pub const LORA_CMD_TELEM_LONG_RANGE_915: LinkConfig = LinkConfig::new(
    rf_presets::LORA_CMD_TELEM_LONG_RANGE_915,
    mac_presets::LORA_CMD_TELEM_LONG_RANGE_915,
);

pub const ACTIVE: LinkConfig = LORA_CMD_TELEM_LONG_RANGE_915;
