//! Link-level pairing of RF + MAC configs with derived helpers.
use super::link_profile::LinkProfile;
use super::mac_config::{self, MacConfig};
use super::rf_config::RfConfig;

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

// Shared RF preset for both vehicles; adjust MAC per vehicle as needed.
pub const SHARED_RF: RfConfig = super::rf_presets::ACTIVE;

// Drone vs rocket differences live in MAC schedule/payload sizing for now.
// These presets are derived from vehicle packetization, so keep them distinct.
pub const DRONE_MAC: MacConfig = super::mac_config::DRONE_MAC;
pub const ROCKET_MAC: MacConfig = super::mac_config::ROCKET_MAC;

pub const DRONE: LinkConfig = LinkConfig::new(SHARED_RF, DRONE_MAC);
pub const ROCKET: LinkConfig = LinkConfig::new(SHARED_RF, ROCKET_MAC);

// Toggle this to switch link configs at build time.
pub const ACTIVE: LinkConfig = DRONE;
