use common::protocol::hilink::{self, DecodedPacket};

pub fn decode_frame<'a>(
    encoded: &[u8],
    scratch: &'a mut [u8],
) -> hilink::Result<DecodedPacket<'a>> {
    hilink::decode_packet(encoded, scratch)
}
