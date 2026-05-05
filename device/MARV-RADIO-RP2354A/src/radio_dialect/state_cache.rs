#![allow(dead_code)]

use common::protocol::hilink;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RadioStateCache {
    pub latest_normal_seq: Option<u16>,
    pub latest_normal_msg_type: Option<u8>,
}

impl RadioStateCache {
    pub const fn new() -> Self {
        Self {
            latest_normal_seq: None,
            latest_normal_msg_type: None,
        }
    }

    pub fn note_normal_packet(&mut self, header: hilink::Header) {
        self.latest_normal_seq = Some(header.seq);
        self.latest_normal_msg_type = Some(header.msg_type);
    }
}
