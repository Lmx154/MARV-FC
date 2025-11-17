// common/src/tasks/radio.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::{info, warn};
use crate::lora::link::{LoRaLink, Sx1262Interface, LinkError};
use crate::utils::delay::DelayMs;
use crate::mavlink2::{self, MavError};

use mavio::Frame;
use mavio::protocol::V2;
use mavio::dialects::common::{Common, messages};

/// Role for this node in the simple demo.
#[derive(Clone, Copy, Debug)]
pub enum Role {
    /// Radio node – periodically sends text messages.
    Radio,
    /// Ground node – receives and logs text messages.
    Ground,
}

/// Config for MAVLink endpoint (MAV system + component IDs).
#[derive(Clone, Copy)]
pub struct MavEndpointConfig {
    pub sys_id: u8,
    pub comp_id: u8,
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
        let frame = build_common_frame(cfg, seq, Common::Statustext(status_msg));

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
            Ok(_other) => {
                warn!(
                    "GsTask: received Common MAVLink message we don't handle yet"
                );
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

fn build_statustext(s: &str) -> messages::Statustext {
    use mavio::dialects::common::enums::MavSeverity;

    let mut text_bytes = [0u8; 50];
    let raw = s.as_bytes();
    let copy_len = core::cmp::min(raw.len(), text_bytes.len());
    text_bytes[..copy_len].copy_from_slice(&raw[..copy_len]);

    messages::Statustext {
        severity: MavSeverity::Info,
        text: text_bytes,
        id: 0,
        chunk_seq: 0,
    }
}

/// Build a MAVLink2 Frame<V2> with a `common::Common` message.
///
/// For now we only support `Common::Statustext`.
fn build_common_frame(
    cfg: MavEndpointConfig,
    seq: u8,
    msg: Common,
) -> Frame<V2> {
    match msg {
        Common::Statustext(st) => {
            Frame::builder()
                .version(V2)
                .system_id(cfg.sys_id.into())
                .component_id(cfg.comp_id.into())
                .sequence(seq.into())
                .message(&st)
                .expect("MAV: build Statustext frame")
                .build()
        }
        _ => {
            panic!("build_common_frame: only Statustext is supported for now");
        }
    }
}

/// Convert STATUSTEXT payload back into &str-like view for logging.
fn statustext_to_str(msg: &messages::Statustext) -> &str {
    let end = msg
        .text
        .iter()
        .position(|&b| b == 0)
        .unwrap_or(msg.text.len());
    core::str::from_utf8(&msg.text[..end]).unwrap_or("<non-utf8>")
}
