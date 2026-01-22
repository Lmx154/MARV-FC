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

// Drone link configs.
pub const DRONE_200: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::DRONE_200);
pub const DRONE_150: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::DRONE_150);
pub const DRONE_100: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF5_BW500_CR45, mac_presets::DRONE_100);
pub const DRONE_50: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::DRONE_50);
pub const DRONE_25: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::DRONE_25);
pub const DRONE_10: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::DRONE_10);
pub const DRONE_5: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::DRONE_5);
pub const DRONE_1: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::DRONE_1);

// Rocket link configs.
pub const ROCKET_200: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::ROCKET_200);
pub const ROCKET_150: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::ROCKET_150);
pub const ROCKET_100: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF5_BW500_CR45, mac_presets::ROCKET_100);
pub const ROCKET_50: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::ROCKET_50);
pub const ROCKET_25: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::ROCKET_25);
pub const ROCKET_10: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::ROCKET_10);
pub const ROCKET_5: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::ROCKET_5);
pub const ROCKET_1: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF6_BW500_CR45, mac_presets::ROCKET_1);

// SX1262 LoRa telemetry-first presets.
pub const LORA_TELEM_200: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF5_BW500_CR45, mac_presets::LORA_TELEM_200);
pub const LORA_TELEM_100: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF5_BW500_CR45, mac_presets::LORA_TELEM_100);
pub const LORA_TELEM_50: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF7_BW500_CR45, mac_presets::LORA_TELEM_50);
pub const LORA_TELEM_20: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF8_BW500_CR45, mac_presets::LORA_TELEM_20);
pub const LORA_TELEM_5: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF9_BW250_CR46, mac_presets::LORA_TELEM_5);
pub const LORA_TELEM_1: LinkConfig =
    LinkConfig::new(rf_presets::LORA_SF11_BW125_CR48, mac_presets::LORA_TELEM_1);

// Toggle this to switch link configs at build time.
pub const ACTIVE: LinkConfig = LORA_TELEM_50;
