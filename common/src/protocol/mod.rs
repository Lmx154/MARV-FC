//! Packet formats, parsers, framing, and checksums.

pub mod crc;
pub mod framing;
pub mod hilink;
pub mod mavlink;
pub mod packet_types;
pub mod ubx;

// Legacy alias retained while old radio code still imports `protocol::packet`.
pub use packet_types as packet;
