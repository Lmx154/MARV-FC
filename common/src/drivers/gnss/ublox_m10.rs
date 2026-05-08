//! Transport-independent u-blox M10 GNSS driver core.

use crate::protocol::ubx::{self, Ack, CfgItem, CfgLayerSet, Error, NavPvt, Result, SamM10qConfig};

pub const MAX_M10_PAYLOAD_LEN: usize = 256;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Event {
    Ack(Ack),
    NavPvt(NavPvt),
    OtherPacket,
}

type InnerParser = ublox::Parser<ublox::FixedBuffer<MAX_M10_PAYLOAD_LEN>, ublox::proto33::Proto33>;

pub struct UbloxM10 {
    parser: InnerParser,
    last_nav_pvt: Option<NavPvt>,
}

impl Default for UbloxM10 {
    fn default() -> Self {
        Self::new()
    }
}

impl UbloxM10 {
    pub fn new() -> Self {
        Self {
            parser: new_parser(),
            last_nav_pvt: None,
        }
    }

    pub fn reset_parser(&mut self) {
        self.parser = new_parser();
    }

    pub fn last_nav_pvt(&self) -> Option<NavPvt> {
        self.last_nav_pvt
    }

    pub fn push_byte(&mut self, byte: u8) -> Result<Option<Event>> {
        let mut latest_nav_pvt = None;
        let latest = {
            let mut latest = None;
            let mut iter = self.parser.consume_ubx(core::slice::from_ref(&byte));

            while let Some(packet) = iter.next() {
                let event = Self::packet_event(packet.map_err(Error::Parser)?);
                if let Event::NavPvt(nav_pvt) = event {
                    latest_nav_pvt = Some(nav_pvt);
                }
                latest = Some(event);
            }

            latest
        };

        if let Some(nav_pvt) = latest_nav_pvt {
            self.last_nav_pvt = Some(nav_pvt);
        }

        Ok(latest)
    }

    pub fn encode_startup_config(
        config: SamM10qConfig,
        payload_scratch: &mut [u8],
        out: &mut [u8],
    ) -> Result<usize> {
        ubx::encode_sam_m10q_startup_cfg(config, payload_scratch, out)
    }

    pub fn encode_custom_config(
        layers: CfgLayerSet,
        items: &[CfgItem],
        payload_scratch: &mut [u8],
        out: &mut [u8],
    ) -> Result<usize> {
        ubx::encode_cfg_valset(layers, items, payload_scratch, out)
    }

    pub fn encode_poll_nav_pvt(out: &mut [u8]) -> Result<usize> {
        ubx::encode_poll_nav_pvt(out)
    }

    pub fn encode_poll_mon_ver(out: &mut [u8]) -> Result<usize> {
        ubx::encode_poll_mon_ver(out)
    }

    fn packet_event(packet: ublox::UbxPacket<'_>) -> Event {
        match packet {
            ublox::UbxPacket::Proto33(ublox::proto33::PacketRef::AckAck(ack)) => Event::Ack(Ack {
                acknowledged: true,
                class_id: ack.class(),
                message_id: ack.msg_id(),
            }),
            ublox::UbxPacket::Proto33(ublox::proto33::PacketRef::AckNak(ack)) => Event::Ack(Ack {
                acknowledged: false,
                class_id: ack.class(),
                message_id: ack.msg_id(),
            }),
            ublox::UbxPacket::Proto33(ublox::proto33::PacketRef::NavPvt(packet)) => {
                let nav_pvt = NavPvt::from_ublox_proto33(packet);
                Event::NavPvt(nav_pvt)
            }
            _ => Event::OtherPacket,
        }
    }
}

fn new_parser() -> InnerParser {
    ublox::ParserBuilder::new()
        .with_protocol::<ublox::proto33::Proto33>()
        .with_fixed_buffer::<MAX_M10_PAYLOAD_LEN>()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::ubx::{CLASS_NAV, ID_NAV_PVT, NAV_PVT_PAYLOAD_LEN, UBX_FRAME_OVERHEAD};

    const CLASS_CFG: u8 = 0x06;
    const ID_CFG_VALSET: u8 = 0x8a;

    #[test]
    fn driver_core_emits_ack_events() {
        let frame = [
            0xb5,
            0x62,
            0x05,
            0x01,
            0x02,
            0x00,
            CLASS_CFG,
            ID_CFG_VALSET,
            0x98,
            0xc1,
        ];

        let mut driver = UbloxM10::new();
        let mut event = None;
        for byte in &frame {
            event = driver.push_byte(*byte).unwrap().or(event);
        }

        assert_eq!(
            event,
            Some(Event::Ack(Ack {
                acknowledged: true,
                class_id: CLASS_CFG,
                message_id: ID_CFG_VALSET,
            }))
        );
    }

    #[test]
    fn driver_core_emits_nav_pvt_events_and_remembers_last_fix() {
        let mut payload = [0u8; NAV_PVT_PAYLOAD_LEN];
        payload[20] = 3;
        payload[23] = 11;
        payload[24..28].copy_from_slice(&(-971_234_567i32).to_le_bytes());
        payload[28..32].copy_from_slice(&301_234_567i32.to_le_bytes());
        payload[36..40].copy_from_slice(&123_000i32.to_le_bytes());

        let mut frame = [0u8; NAV_PVT_PAYLOAD_LEN + UBX_FRAME_OVERHEAD];
        let len = ubx::encode_test_frame(CLASS_NAV, ID_NAV_PVT, &payload, &mut frame);

        let mut driver = UbloxM10::new();
        let mut event = None;
        for byte in &frame[..len] {
            event = driver.push_byte(*byte).unwrap().or(event);
        }

        match event {
            Some(Event::NavPvt(nav_pvt)) => {
                assert_eq!(nav_pvt.fix_type, 3);
                assert_eq!(nav_pvt.num_sv, 11);
                assert_eq!(driver.last_nav_pvt(), Some(nav_pvt));
            }
            other => panic!("unexpected event: {other:?}"),
        }
    }
}
