use crate::channels::HilinkBridgeFrame;
use crate::radio_dialect::rf;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HostToRfDecision<'a> {
    LegacyPassThrough(&'a [u8]),
}

pub fn host_to_rf_legacy_pass_through(frame: &HilinkBridgeFrame) -> HostToRfDecision<'_> {
    HostToRfDecision::LegacyPassThrough(rf::legacy_payload_from_hilink_frame(frame))
}
