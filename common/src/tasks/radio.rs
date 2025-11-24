// common/src/tasks/radio.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::{info, warn};
use crate::lora::link::{LoRaLink, Sx1262Interface, LinkError};
use crate::utils::delay::DelayMs;
use crate::mavlink2::{self, MavError};
use crate::tasks::coms::{
    build_statustext, build_statustext_frame, statustext_to_str, MavEndpointConfig,
    TelemetrySample,
};

use mavio::Frame;
use mavio::protocol::V2;
use mavio::dialects::common::Common;

/// Role for this node in the simple demo.
#[derive(Clone, Copy, Debug)]
pub enum Role {
    /// Radio node - periodically sends text messages.
    Radio,
    /// Ground node - receives and logs text messages.
    Ground,
}

/// Entry point for a **small, human-readable link test**.
///
/// Instead of ping/pong torture, this:
///   - RADIO: sends STATUSTEXT-style messages like "hello from RADIO #n".
///   - GS:    receives frames, decodes as `common::Common` and logs text.
///
/// All traffic still flows Radio <-> GS over:
///   SX1262 (Layer 0) → LoRaLink (Layer 1) → MAVLink2 (Layer 2).
pub async fn run_mavlink_text_demo<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
    role: Role,
    cfg: MavEndpointConfig,
) where
    RADIO: Sx1262Interface,
{
    match role {
        Role::Radio => run_radio_talker(link, delay, cfg).await,
        Role::Ground => run_gs_listener(link, delay).await,
    }
}

// -------------------------------------------------------------------------
// Radio side: send small "word packets" as MAVLink2 frames
// -------------------------------------------------------------------------

async fn run_radio_talker<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
    cfg: MavEndpointConfig,
) where
    RADIO: Sx1262Interface,
{
    info!("RadioTask: starting MAVLink text demo (talker)");

    if let Err(e) = link.start_rx(delay).await {
        warn!("RadioTask: initial start_rx failed: {:?}", e);
    }

    let mut seq: u8 = 0;

    loop {
        seq = seq.wrapping_add(1);

        let msg_str = match seq % 3 {
            0 => "RADIO says hello",
            1 => "MARV-FC link test",
            _ => "Rocket telemetry path",
        };

        let status_msg = build_statustext(msg_str);
        let frame = build_statustext_frame(cfg, seq, status_msg);

        match mavlink2::send_frame_over_lora(link, delay, &frame).await {
            Ok(()) => {
                info!(
                    "RadioTask: sent STATUSTEXT seq={} text=\"{}\"",
                    seq,
                    msg_str
                );
            }
            Err(MavError::Link(LinkError::Timeout)) => {
                warn!("RadioTask: LoRaLink timeout when sending text");
            }
            Err(e) => {
                warn!("RadioTask: MAV send error: {:?}", e);
            }
        }

        // Small pause (not a torture-test anymore).
        delay.delay_ms(500).await;
    }
}

// -------------------------------------------------------------------------
// GS side: receive and log text messages
// -------------------------------------------------------------------------

async fn run_gs_listener<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
) where
    RADIO: Sx1262Interface,
{
    info!("GsTask: starting MAVLink text demo (listener)");

    if let Err(e) = link.start_rx(delay).await {
        warn!("GsTask: initial start_rx failed: {:?}", e);
    }

    let mut rx_buf = [0u8; 255];

    loop {
        let frame: Frame<V2> = match mavlink2::recv_frame_over_lora(link, delay, &mut rx_buf).await
        {
            Ok(f) => f,
            Err(MavError::Link(LinkError::Timeout)) => {
                warn!("GsTask: LoRaLink timeout while waiting for MAV frame");
                continue;
            }
            Err(e) => {
                warn!("GsTask: MAV recv error: {:?}", e);
                continue;
            }
        };

        match frame.decode::<Common>() {
            Ok(Common::Statustext(msg)) => {
                let text = statustext_to_str(&msg);
                info!(
                    "GsTask: STATUSTEXT from sys={} comp={} seq={}: \"{}\"",
                    frame.system_id(),
                    frame.component_id(),
                    frame.sequence(),
                    text,
                );
            }
            Ok(Common::EncapsulatedData(pkt)) => {
                let raw_head = &pkt.data[..TelemetrySample::WIRE_LEN];
                match TelemetrySample::decode(raw_head) {
                    Some(s) => info!(
                        "GsTask: Telemetry seq={} acc[{},{},{}] gyro[{},{},{}] mag[{},{},{}] P:{}Pa T:{}x10C GPS lat {} lon {} alt {}mm sat {} fix {}",
                        pkt.seqnr,
                        s.accel[0], s.accel[1], s.accel[2],
                        s.gyro[0], s.gyro[1], s.gyro[2],
                        s.mag[0], s.mag[1], s.mag[2],
                        s.baro_p_pa,
                        s.baro_t_cx10,
                        s.gps_lat_e7,
                        s.gps_lon_e7,
                        s.gps_alt_mm,
                        s.gps_sat,
                        s.gps_fix,
                    ),
                    None => warn!(
                        "GsTask: Telemetry decode failed seq={} head={:?}",
                        pkt.seqnr,
                        raw_head
                    ),
                }
            }
            Ok(_other) => {
                warn!("GsTask: received Common MAVLink message we don't handle yet");
            }
            Err(_e) => {
                warn!("GsTask: MAV decode error");
            }
        }
    }
}

// -------------------------------------------------------------------------
// Small helpers: build / interpret STATUSTEXT
// -------------------------------------------------------------------------
