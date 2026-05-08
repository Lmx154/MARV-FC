//! Project-specific u-blox M10 helpers built on top of the `ublox` crate.
//!
//! The upstream crate owns UBX framing, checksums, CFG-VALSET serialization,
//! streaming parsing, ACK handling, and NAV-PVT decoding. This module keeps the
//! MARV-facing preset/config and telemetry conversion layer transport agnostic.

use heapless::Vec;

use crate::messages::sensor::{GpsFixSample, GpsFixSampleStamped};
use crate::protocol::hilink::{GpsPayload, SimStamp};
use crate::protocol::packet_types::{MAX_PACKET_PAYLOAD, TelemetryGps};
use crate::utilities::time::MeasurementTimestamp;
use num_traits::float::FloatCore;
use ublox::packets::cfg_val::{CfgLayerSet as UbloxCfgLayerSet, CfgValSetBuilder};

pub const CLASS_NAV: u8 = 0x01;
pub const CLASS_MON: u8 = 0x0a;

pub const ID_NAV_PVT: u8 = 0x07;
pub const ID_MON_VER: u8 = 0x04;

pub const NAV_PVT_PAYLOAD_LEN: usize = 92;
pub const UBX_FRAME_OVERHEAD: usize = 8;
pub const MAX_CFG_VALSET_PAIRS: usize = 64;
pub const MAX_CFG_VALSET_FRAME_LEN: usize = UBX_FRAME_OVERHEAD + 4 + (4 + 8) * MAX_CFG_VALSET_PAIRS;

pub type CfgItem = ublox::cfg_val::CfgVal;
pub type CfgLayerSet = UbloxCfgLayerSet;

#[derive(Debug)]
pub enum Error {
    BufferTooSmall,
    Parser(ublox::ParserError),
}

pub type Result<T> = core::result::Result<T, Error>;

fn copy_to_out(packet: &[u8], out: &mut [u8]) -> Result<usize> {
    if out.len() < packet.len() {
        return Err(Error::BufferTooSmall);
    }

    out[..packet.len()].copy_from_slice(packet);
    Ok(packet.len())
}

pub fn encode_poll(class_id: u8, message_id: u8, out: &mut [u8]) -> Result<usize> {
    let packet =
        ublox::UbxPacketRequest::request_for_unknown(class_id, message_id).into_packet_bytes();
    copy_to_out(&packet, out)
}

pub fn encode_poll_nav_pvt(out: &mut [u8]) -> Result<usize> {
    encode_poll(CLASS_NAV, ID_NAV_PVT, out)
}

pub fn encode_poll_mon_ver(out: &mut [u8]) -> Result<usize> {
    encode_poll(CLASS_MON, ID_MON_VER, out)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ack {
    pub acknowledged: bool,
    pub class_id: u8,
    pub message_id: u8,
}

pub fn encode_cfg_valset(
    layers: CfgLayerSet,
    items: &[CfgItem],
    _payload_scratch: &mut [u8],
    out: &mut [u8],
) -> Result<usize> {
    let mut packet = Vec::<u8, MAX_CFG_VALSET_FRAME_LEN>::new();

    CfgValSetBuilder {
        version: 0,
        layers,
        reserved1: 0,
        cfg_data: items,
    }
    .extend_to(&mut packet);

    copy_to_out(packet.as_slice(), out)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SamM10qConfig {
    pub baud_rate: u32,
    pub nav_rate_hz: u8,
    pub enable_nmea_input: bool,
    pub enable_nmea_output: bool,
}

impl SamM10qConfig {
    pub const DEFAULT_PIO_UART: Self = Self {
        baud_rate: 9_600,
        nav_rate_hz: 10,
        enable_nmea_input: true,
        enable_nmea_output: false,
    };

    pub fn measurement_period_ms(self) -> u16 {
        let hz = if self.nav_rate_hz == 0 {
            1
        } else {
            self.nav_rate_hz
        };
        (1000u16 / hz as u16).max(25)
    }
}

pub fn sam_m10q_startup_items(config: SamM10qConfig) -> [CfgItem; 9] {
    use ublox::cfg_val::CfgVal::*;

    [
        Uart1Enabled(true),
        Uart1Baudrate(config.baud_rate),
        Uart1InProtUbx(true),
        Uart1InProtNmea(config.enable_nmea_input),
        Uart1OutProtUbx(true),
        Uart1OutProtNmea(config.enable_nmea_output),
        RateMeas(config.measurement_period_ms()),
        RateNav(1),
        MsgOutUbxNavPvtUart1(1),
    ]
}

pub fn encode_sam_m10q_startup_cfg(
    config: SamM10qConfig,
    payload_scratch: &mut [u8],
    out: &mut [u8],
) -> Result<usize> {
    let items = sam_m10q_startup_items(config);
    encode_cfg_valset(CfgLayerSet::RAM, &items, payload_scratch, out)
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NavPvt {
    pub i_tow_ms: u32,
    pub year: u16,
    pub month: u8,
    pub day: u8,
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub fix_type: u8,
    pub num_sv: u8,
    pub lon_deg_e7: i32,
    pub lat_deg_e7: i32,
    pub h_msl_mm: i32,
    pub vel_n_mm_s: i32,
    pub vel_e_mm_s: i32,
    pub vel_d_mm_s: i32,
}

impl NavPvt {
    pub fn from_ublox_proto33(packet: ublox::nav_pvt::proto33::NavPvtRef<'_>) -> Self {
        Self {
            i_tow_ms: packet.itow(),
            year: packet.year(),
            month: packet.month(),
            day: packet.day(),
            hour: packet.hour(),
            minute: packet.min(),
            second: packet.sec(),
            fix_type: gnss_fix_type_code(packet.fix_type()),
            num_sv: packet.num_satellites(),
            lon_deg_e7: deg_to_e7(packet.longitude()),
            lat_deg_e7: deg_to_e7(packet.latitude()),
            h_msl_mm: meters_to_mm_i32(packet.height_msl()),
            vel_n_mm_s: meters_to_mm_i32(packet.vel_north()),
            vel_e_mm_s: meters_to_mm_i32(packet.vel_east()),
            vel_d_mm_s: meters_to_mm_i32(packet.vel_down()),
        }
    }

    pub fn lat_deg(self) -> f64 {
        self.lat_deg_e7 as f64 * 1.0e-7
    }

    pub fn lon_deg(self) -> f64 {
        self.lon_deg_e7 as f64 * 1.0e-7
    }

    pub fn alt_msl_m(self) -> f32 {
        self.h_msl_mm as f32 * 1.0e-3
    }

    pub fn vel_ned_mps(self) -> [f32; 3] {
        [
            self.vel_n_mm_s as f32 * 1.0e-3,
            self.vel_e_mm_s as f32 * 1.0e-3,
            self.vel_d_mm_s as f32 * 1.0e-3,
        ]
    }

    pub fn gps_fix_sample(self) -> GpsFixSample {
        GpsFixSample {
            lat_deg: self.lat_deg(),
            lon_deg: self.lon_deg(),
            alt_m: self.alt_msl_m(),
            vel_ned_mps: self.vel_ned_mps(),
            sats: self.num_sv,
            fix_type: self.fix_type,
        }
    }

    pub fn gps_fix_sample_stamped(self, timestamp: MeasurementTimestamp) -> GpsFixSampleStamped {
        GpsFixSampleStamped {
            timestamp,
            sample: self.gps_fix_sample(),
        }
    }

    pub fn hilink_gps_payload(self, stamp: SimStamp) -> GpsPayload {
        GpsPayload {
            stamp,
            lat_deg: self.lat_deg(),
            lon_deg: self.lon_deg(),
            alt_msl_m: self.alt_msl_m(),
            vel_ned_mps: self.vel_ned_mps(),
            sats: self.num_sv,
            fix_type: self.fix_type,
            reserved0: [0; 2],
        }
    }

    pub fn telemetry_gps(self) -> TelemetryGps {
        TelemetryGps {
            lat: self.lat_deg_e7,
            lon: self.lon_deg_e7,
            alt_mm: self.h_msl_mm,
            sats: self.num_sv,
            fix: self.fix_type,
        }
    }

    pub fn telemetry_gps_payload(self) -> Vec<u8, MAX_PACKET_PAYLOAD> {
        self.telemetry_gps().encode()
    }
}

fn gnss_fix_type_code(fix_type: ublox::GnssFixType) -> u8 {
    match fix_type {
        ublox::GnssFixType::NoFix => 0,
        ublox::GnssFixType::DeadReckoningOnly => 1,
        ublox::GnssFixType::Fix2D => 2,
        ublox::GnssFixType::Fix3D => 3,
        ublox::GnssFixType::GPSPlusDeadReckoning => 4,
        ublox::GnssFixType::TimeOnlyFix => 5,
        _ => 0,
    }
}

fn deg_to_e7(deg: f64) -> i32 {
    <f64 as FloatCore>::round(deg * 1.0e7) as i32
}

fn meters_to_mm_i32(meters: f64) -> i32 {
    <f64 as FloatCore>::round(meters * 1.0e3) as i32
}

#[cfg(test)]
pub(crate) fn encode_test_frame(
    class_id: u8,
    message_id: u8,
    payload: &[u8],
    out: &mut [u8],
) -> usize {
    let payload_len = payload.len() as u16;
    let len = UBX_FRAME_OVERHEAD + payload.len();
    out[0] = 0xb5;
    out[1] = 0x62;
    out[2] = class_id;
    out[3] = message_id;
    out[4..6].copy_from_slice(&payload_len.to_le_bytes());
    out[6..6 + payload.len()].copy_from_slice(payload);

    let mut ck_a = 0u8;
    let mut ck_b = 0u8;
    for byte in &out[2..6 + payload.len()] {
        ck_a = ck_a.wrapping_add(*byte);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    out[6 + payload.len()] = ck_a;
    out[7 + payload.len()] = ck_b;
    len
}

#[cfg(test)]
mod tests {
    use super::*;
    use ublox::UbxParserIter;

    #[test]
    fn nav_pvt_poll_matches_known_ubx_frame() {
        let mut out = [0u8; 8];
        let len = encode_poll_nav_pvt(&mut out).unwrap();

        assert_eq!(len, 8);
        assert_eq!(out, [0xb5, 0x62, 0x01, 0x07, 0, 0, 0x08, 0x19]);
    }

    #[test]
    fn ublox_parser_recovers_complete_ack_frames() {
        let encoded = [0xb5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x8a, 0x98, 0xc1];
        let mut parser = ublox::ParserBuilder::new()
            .with_protocol::<ublox::proto33::Proto33>()
            .with_fixed_buffer::<64>();

        let mut iter: UbxParserIter<_, ublox::proto33::Proto33> = parser.consume_ubx(&encoded);
        match iter.next() {
            Some(Ok(ublox::UbxPacket::Proto33(ublox::proto33::PacketRef::AckAck(ack)))) => {
                assert_eq!(ack.class(), 0x06);
                assert_eq!(ack.msg_id(), 0x8a);
            }
            other => panic!("unexpected parser event: {other:?}"),
        }
    }

    #[test]
    fn cfg_valset_encodes_m10_startup_items_with_ublox_builder() {
        let mut payload_scratch = [0u8; 1];
        let mut out = [0u8; MAX_CFG_VALSET_FRAME_LEN];
        let len = encode_sam_m10q_startup_cfg(
            SamM10qConfig::DEFAULT_PIO_UART,
            &mut payload_scratch,
            &mut out,
        )
        .unwrap();

        assert_eq!(&out[..6], &[0xb5, 0x62, 0x06, 0x8a, 0x36, 0x00]);
        assert_eq!(&out[6..10], &[0, 1, 0, 0]);
        assert!(
            out[..len]
                .windows(4)
                .any(|window| window == 9_600u32.to_le_bytes())
        );
        assert!(
            out[..len]
                .windows(4)
                .any(|window| window == 0x2091_0007u32.to_le_bytes())
        );
    }

    #[test]
    fn nav_pvt_maps_to_project_gps_shapes() {
        let mut payload = [0u8; NAV_PVT_PAYLOAD_LEN];
        payload[0..4].copy_from_slice(&123_456u32.to_le_bytes());
        payload[4..6].copy_from_slice(&2026u16.to_le_bytes());
        payload[6] = 4;
        payload[7] = 26;
        payload[8] = 17;
        payload[9] = 33;
        payload[10] = 12;
        payload[11] = 0b11;
        payload[20] = 3;
        payload[23] = 14;
        payload[24..28].copy_from_slice(&(-972_000_000i32).to_le_bytes());
        payload[28..32].copy_from_slice(&321_000_000i32.to_le_bytes());
        payload[32..36].copy_from_slice(&153_000i32.to_le_bytes());
        payload[36..40].copy_from_slice(&151_250i32.to_le_bytes());
        payload[48..52].copy_from_slice(&1_500i32.to_le_bytes());
        payload[52..56].copy_from_slice(&(-250i32).to_le_bytes());
        payload[56..60].copy_from_slice(&50i32.to_le_bytes());

        let mut encoded = [0u8; NAV_PVT_PAYLOAD_LEN + UBX_FRAME_OVERHEAD];
        let encoded_len = encode_test_frame(CLASS_NAV, ID_NAV_PVT, &payload, &mut encoded);
        let mut parser = ublox::ParserBuilder::new()
            .with_protocol::<ublox::proto33::Proto33>()
            .with_fixed_buffer::<128>();
        let mut iter = parser.consume_ubx(&encoded[..encoded_len]);

        let nav = match iter.next() {
            Some(Ok(ublox::UbxPacket::Proto33(ublox::proto33::PacketRef::NavPvt(packet)))) => {
                NavPvt::from_ublox_proto33(packet)
            }
            other => panic!("unexpected parser event: {other:?}"),
        };

        let sample = nav.gps_fix_sample();
        let hilink = nav.hilink_gps_payload(SimStamp {
            sim_tick: 7,
            sim_time_us: 42,
        });
        let telemetry = nav.telemetry_gps();

        assert_eq!(nav.year, 2026);
        assert!((sample.lat_deg - 32.1).abs() < 1.0e-12);
        assert!((sample.lon_deg - -97.2).abs() < 1.0e-12);
        assert_eq!(sample.alt_m, 151.25);
        assert!((sample.vel_ned_mps[0] - 1.5).abs() < 1.0e-6);
        assert!((sample.vel_ned_mps[1] - -0.25).abs() < 1.0e-6);
        assert!((sample.vel_ned_mps[2] - 0.05).abs() < 1.0e-6);
        assert_eq!(sample.sats, 14);
        assert_eq!(hilink.fix_type, 3);
        assert_eq!(hilink.stamp.sim_tick, 7);
        assert_eq!(telemetry.lat, 321_000_000);
        assert_eq!(telemetry.lon, -972_000_000);
        assert_eq!(telemetry.alt_mm, 151_250);
    }
}
