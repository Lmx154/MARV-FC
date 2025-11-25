//! Simple runtime-tweakable logging switches for defmt output.
//! Flip these booleans to reduce RTT spam without touching call sites.

/// Low-level SX1262 IRQ / status chatter.
pub const LOG_PHY_IRQS: bool = false;
/// TX/RX state transitions at the PHY.
pub const LOG_PHY_TRAFFIC: bool = true;
/// Header/CRC/timeout warnings at the PHY.
pub const LOG_PHY_ERRORS: bool = true;
/// Link-layer traffic info (TX attempts, RX start, etc).
pub const LOG_LINK_TRAFFIC: bool = true;
/// Link-layer ACK send/receive info.
pub const LOG_LINK_ACKS: bool = true;
/// Out-of-order sequence warnings.
pub const LOG_LINK_ORDER_WARN: bool = true;
/// MAV decode warnings.
pub const LOG_MAV_DECODE_WARN: bool = true;
