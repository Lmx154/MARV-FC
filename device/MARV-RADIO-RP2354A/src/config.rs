#![allow(dead_code)]

use common::comms::links::lora::LoraProfile;
use common::protocol::hilink;

#[cfg(all(feature = "radio", feature = "ground-station"))]
compile_error!("features `radio` and `ground-station` are mutually exclusive");

#[cfg(not(any(feature = "radio", feature = "ground-station")))]
compile_error!("select exactly one firmware role: `radio` or `ground-station`");

mod build_config {
    include!(concat!(env!("OUT_DIR"), "/radio_build_config.rs"));
}

pub const XOSC_HZ: u32 = 12_000_000;
pub const WATCHDOG_TIMEOUT_MS: u32 = 250;
// Raised to 460_800 so the FC↔radio link carries 100 Hz HILink telemetry with headroom.
// Must stay in lockstep with RADIO_LINK_UART_BAUD in both flight-controller firmwares.
pub const HOST_UART_BAUD: u32 = 460_800;
pub const HOST_UART_BUFFER_BYTES: usize = 256;
pub const HOST_UART_HEARTBEAT_PERIOD_MS: u64 = 1_000;
pub const LORA_SPI_FREQUENCY_HZ: u32 = 4_000_000;
pub const LORA_HEARTBEAT_PERIOD_MS: u64 = 2_000;
pub const LORA_PING_PERIOD_MS: u64 = LORA_HEARTBEAT_PERIOD_MS;
pub const LORA_LINK_STATUS_PERIOD_MS: u64 = 2_000;
pub const LORA_STATION_ID_PERIOD_MS: u64 = 9 * 60 * 1_000;
pub const LORA_LED_EVENT_HOLD_MS: u64 = 1_000;
pub const LORA_PACKET_LOG_EVERY: u32 = 10;
pub const LORA_MISS_LOG_EVERY: u8 = 6;
pub const LORA_FREQUENCY_HZ: u32 = build_config::LORA_FREQUENCY_HZ;
pub const AMATEUR_CALLSIGN: &str = build_config::AMATEUR_CALLSIGN;

/// Boot RF profile, expressed as the `(preset, frequency, power)` triple the runtime switch uses.
/// `DEFAULT_PRESET` is the fastest Mode C narrowband preset; the default power matches the
/// dense-RF ceiling. The bridge builds the live profile from these via `lora_profile_for_preset`.
pub const DEFAULT_PRESET: u8 = hilink::lora_profile::FAST;
pub const DEFAULT_TX_POWER_DBM: i8 = hilink::band_plan::DEFAULT_MAX_TX_POWER_DBM;

/// Boot value for the idle-fallback window: how long this radio tolerates hearing nothing from its
/// peer before unilaterally returning to the boot/"setup" profile ([`DEFAULT_PRESET`] @
/// [`LORA_FREQUENCY_HZ`] @ [`DEFAULT_TX_POWER_DBM`]). Operator-tunable at runtime via the cache;
/// both ends re-home to the same absolute channel so a dead link re-rendezvous there. See
/// `hilink::idle_fallback`.
pub const DEFAULT_IDLE_FALLBACK_MS: u32 = hilink::idle_fallback::DEFAULT_MS;

/// Const fallback if `lora_profile_for_preset(DEFAULT_PRESET, ..)` ever fails to map (it will not
/// for `FAST`, which is byte-identical to this). Kept as a compile-time safety net.
pub const LORA_PROFILE: LoraProfile = LoraProfile::irec_33cm_mode_c(LORA_FREQUENCY_HZ);

#[derive(Clone, Copy, Debug, defmt::Format, PartialEq, Eq)]
pub enum FirmwareRole {
    Radio,
    GroundStation,
}

#[cfg(feature = "radio")]
pub const FIRMWARE_ROLE: FirmwareRole = FirmwareRole::Radio;

#[cfg(feature = "ground-station")]
pub const FIRMWARE_ROLE: FirmwareRole = FirmwareRole::GroundStation;

#[cfg(feature = "radio")]
pub const DEVICE_NAME: &str = "MARV-RADIO-RP2354A";

#[cfg(feature = "ground-station")]
pub const DEVICE_NAME: &str = "MARV-GS-RP2354A";

#[cfg(feature = "radio")]
pub const HOST_UART_PEER: &str = "MARV-FC";

#[cfg(feature = "ground-station")]
pub const HOST_UART_PEER: &str = "CP2102";

#[cfg(feature = "radio")]
pub const HOST_UART_BOOT_MESSAGE: &str = "MARV-RADIO-RP2354A radio UART online\r\n";

#[cfg(feature = "ground-station")]
pub const HOST_UART_BOOT_MESSAGE: &str = "MARV-GS-RP2354A ground-station UART online\r\n";

#[cfg(feature = "radio")]
pub const HOST_UART_HEARTBEAT_MESSAGE: &str = "MARV radio heartbeat\r\n";

#[cfg(feature = "ground-station")]
pub const HOST_UART_HEARTBEAT_MESSAGE: &str = "MARV ground-station heartbeat\r\n";

#[derive(Clone, Copy, Debug)]
pub struct HostUartConfig {
    pub baud: u32,
    pub peer: &'static str,
    pub boot_message: &'static str,
    pub heartbeat_message: &'static str,
    pub heartbeat_period_ms: u64,
}

impl Default for HostUartConfig {
    fn default() -> Self {
        Self {
            baud: HOST_UART_BAUD,
            peer: HOST_UART_PEER,
            boot_message: HOST_UART_BOOT_MESSAGE,
            heartbeat_message: HOST_UART_HEARTBEAT_MESSAGE,
            heartbeat_period_ms: HOST_UART_HEARTBEAT_PERIOD_MS,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DeviceConfig {
    pub role: FirmwareRole,
    pub name: &'static str,
    pub host_uart: HostUartConfig,
}

impl Default for DeviceConfig {
    fn default() -> Self {
        Self {
            role: FIRMWARE_ROLE,
            name: DEVICE_NAME,
            host_uart: HostUartConfig::default(),
        }
    }
}
