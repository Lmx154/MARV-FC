//! End-to-end data-path test (no hardware): real HILink bytes → frame accumulator → decode →
//! aggregator → UI packet, including the camelCase JSON contract the frontend depends on.

use std::time::{Duration, Instant};

use marv_hilink::{
    encode_packet, flight_state, GpsPayload, RadioStatusPayload, SimStamp, TelemetrySnapshotPayload,
    WirePayload,
};

use crate::aggregator::TelemetryAggregator;
use crate::decode::decode_frame;
use crate::framing::FrameAccumulator;

/// Encode one payload into a complete COBS frame (ending in the 0x00 delimiter).
fn frame<P: WirePayload>(payload: &P, seq: u16) -> Vec<u8> {
    let mut raw = [0u8; 256];
    let mut out = [0u8; 320];
    let len = encode_packet(payload, seq, 0, &mut raw, &mut out).unwrap();
    out[..len].to_vec()
}

#[test]
fn full_pipeline_bytes_to_ui_packet() {
    let telem = TelemetrySnapshotPayload {
        stamp: SimStamp { sim_tick: 0, sim_time_us: 12_000_000 },
        system_state: flight_state::BOOST,
        reserved0: [0; 3],
        flags: 0,
        position_ned_m: [0.0, 0.0, -742.5], // altitude 742.5 m up
        velocity_ned_mps: [0.0, 0.0, -128.0], // climbing 128 m/s
        attitude_quat: [1.0, 0.0, 0.0, 0.0],
        battery_voltage_v: 15.6,
        rssi_dbm: -40,
        snr_db_x100: 1100,
        loss_pct_x100: 0,
        accel_mag_cms2: 5_886, // ~6 g
    };
    let gps = GpsPayload {
        stamp: SimStamp { sim_tick: 0, sim_time_us: 12_000_000 },
        lat_deg: 32.9903,
        lon_deg: -106.975,
        alt_msl_m: 2100.0,
        vel_ned_mps: [0.0, 0.0, -128.0],
        sats: 11,
        fix_type: 3,
        reserved0: [0; 2],
    };
    let radio = RadioStatusPayload {
        rssi_dbm: -72,
        snr_db_x100: 825,
        loss_pct_x100: 140, // 1.4 %
        packet_rate_hz: 10,
        reserved0: 0,
    };

    // Concatenate the three frames, then split mid-stream to prove reassembly across chunks.
    let mut stream = Vec::new();
    stream.extend_from_slice(&frame(&telem, 100));
    stream.extend_from_slice(&frame(&gps, 101));
    stream.extend_from_slice(&frame(&radio, 102));
    let split = stream.len() / 2;

    let mut acc = FrameAccumulator::new();
    let mut agg = TelemetryAggregator::new();
    let mut frames = acc.push(&stream[..split]);
    frames.extend(acc.push(&stream[split..]));
    assert_eq!(frames.len(), 3, "all three frames reassembled");

    let t0 = Instant::now();
    for (i, raw) in frames.iter().enumerate() {
        let decoded = decode_frame(raw).expect("frame decodes");
        agg.ingest(&decoded, t0 + Duration::from_millis(i as u64 * 100));
    }

    let packet = agg.snapshot(t0 + Duration::from_millis(200));

    // Values derived from each message type are all present and correct.
    assert_eq!(packet.stage, "BOOST");
    assert!((packet.altitude_m - 742.5).abs() < 0.1);
    assert!((packet.vertical_speed_mps - 128.0).abs() < 0.1);
    assert!((packet.accel_g - 6.0).abs() < 0.02, "accel_g={}", packet.accel_g);
    assert!((packet.battery_v - 15.6).abs() < 0.01);
    assert!(packet.gps_valid);
    assert_eq!(packet.gps_sats, 11);
    assert!((packet.gps_lat - 32.9903).abs() < 1e-6);
    // RadioStatus is authoritative for link metrics, overriding the snapshot copies.
    assert_eq!(packet.rssi_dbm, -72);
    assert!((packet.snr_db - 8.25).abs() < 0.01);
    assert!((packet.packet_loss_pct - 1.4).abs() < 0.01);
    assert!((packet.packet_rate_hz - 10.0).abs() < 0.01);
    assert_eq!(packet.mission_time_ms, 12_000);

    // The serialized JSON must use the exact camelCase keys src/types.ts expects.
    let json = serde_json::to_value(&packet).unwrap();
    for key in [
        "seq",
        "missionTimeMs",
        "stage",
        "altitudeM",
        "maxAltitudeM",
        "verticalSpeedMps",
        "accelG",
        "batteryV",
        "gpsLat",
        "gpsLon",
        "gpsAltM",
        "gpsValid",
        "gpsSats",
        "rssiDbm",
        "snrDb",
        "packetAgeMs",
        "packetRateHz",
        "packetLossPct",
    ] {
        assert!(json.get(key).is_some(), "missing UI contract key: {key}");
    }
}
