//! Packet formats, parsers, framing, and checksums.

pub mod mavlink;
pub mod ubx;
pub mod framing;
pub mod crc;
pub mod packet_types;

// Legacy alias retained while old radio code still imports `protocol::packet`.
pub use packet_types as packet;
