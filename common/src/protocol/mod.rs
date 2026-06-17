//! Packet formats, parsers, framing, and checksums.

pub mod crc;
pub mod framing;
pub mod mavlink;
pub mod packet_types;
pub mod ubx;

// HILink lives in its own `no_std` crate so host tools can share the exact same
// wire encode/decode source. Re-exported here so `protocol::hilink::...` keeps working.
pub use marv_hilink as hilink;

// Legacy alias retained while old radio code still imports `protocol::packet`.
pub use packet_types as packet;
