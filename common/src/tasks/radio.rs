// common/src/tasks/radio.rs
#![allow(dead_code)]
#![allow(async_fn_in_trait)]

use defmt::*;
use crate::lora::link::{LoRaLink, LinkError, Sx1262Interface};
use crate::utils::delay::DelayMs;

/// Application-level frame kinds (on top of LoRaLink)
const KIND_APP_PING: u8 = 0x10;
const KIND_APP_PONG: u8 = 0x11;

/// Stress-test tuning
const STRESS_MODE: bool = true;

/// Max payload size at LoRaLink level (must match LoRaLink::MTU)
const LINK_MAX_PAYLOAD: usize = 240;

/// Simple stats for debugging (optional)
#[derive(Default)]
pub struct LinkStats {
    pub tx_ok: u32,
    pub tx_timeout: u32,
    pub tx_radio_err: u32,
    pub rx_ok: u32,
    pub rx_corrupt: u32,
}

/// Role for this node in the bidirectional test.
#[derive(Clone, Copy, Debug)]
pub enum Role {
    /// "Client" – starts the ping-pong cycles.
    Radio,
    /// "Server" – responds to each ping with a pong.
    Ground,
}

/// In stress mode, choose a payload length in [8, 220] pseudo-randomly.
/// In normal mode, fixed 16 bytes for readability.
fn choose_payload_len(app_seq: u32) -> usize {
    if !STRESS_MODE {
        return 16;
    }
    let min_len = 8usize;
    let max_len = 220usize;
    let span = max_len - min_len;
    min_len + ((app_seq as usize * 17) % span)
}

/// Gap between full ping-pong cycles.
/// In stress mode we keep it very small; in normal mode we make logs readable.
fn cycle_gap_ms() -> u32 {
    if STRESS_MODE {
        5
    } else {
        500
    }
}

/// Entry point for a **bidirectional link-layer test**.
///
/// - `Role::Radio`: sends PING, waits for PONG.
/// - `Role::Ground`: waits for PING, sends PONG.
///
/// Both sides use `LoRaLink::send` + `LoRaLink::recv`, so we’re
/// fully exercising Layer 1 (ACK, retries, header parsing, RX/TX
/// transitions) over the known-good Layer 0 driver.
pub async fn run_bidir_test<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
    role: Role,
) where
    RADIO: Sx1262Interface,
{
    match role {
        Role::Radio => run_radio_client(link, delay).await,
        Role::Ground => run_gs_server(link, delay).await,
    }
}

// -------------------------------------------------------------------------
// Radio side: initiator (PING -> PONG)
// -------------------------------------------------------------------------

async fn run_radio_client<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
) where
    RADIO: Sx1262Interface,
{
    info!("RadioTask: starting bidirectional LoRaLink test (client)");

    // Make sure we're in RX initially.
    if let Err(e) = link.start_rx(delay).await {
        warn!("RadioTask: initial start_rx failed: {:?}", e);
    }

    let mut stats = LinkStats::default();
    let mut app_seq_tx: u32 = 0;

    // TX buffer sized to the link MTU so we can hit edge cases
    let mut tx_buf = [0u8; LINK_MAX_PAYLOAD];
    let mut rx_buf = [0u8; 255];

    let gap = cycle_gap_ms();

    loop {
        app_seq_tx = app_seq_tx.wrapping_add(1);

        // -------------------------------------------------------------
        // Build PING payload:
        //   [0]    = KIND_APP_PING
        //   [1..4] = app_seq_tx (big-endian)
        //   [5..]  = deterministic pattern based on app_seq_tx
        // -------------------------------------------------------------
        let mut tx_len = choose_payload_len(app_seq_tx);
        if tx_len < 16 {
            tx_len = 16;
        }
        if tx_len > LINK_MAX_PAYLOAD {
            tx_len = LINK_MAX_PAYLOAD;
        }

        tx_buf[0] = KIND_APP_PING;
        tx_buf[1] = (app_seq_tx >> 24) as u8;
        tx_buf[2] = (app_seq_tx >> 16) as u8;
        tx_buf[3] = (app_seq_tx >> 8) as u8;
        tx_buf[4] = app_seq_tx as u8;

        for i in 5..tx_len {
            tx_buf[i] = (app_seq_tx as u8).wrapping_add(i as u8);
        }

        info!(
            "RadioTask: SEND PING app_seq_tx={} len={}",
            app_seq_tx,
            tx_len
        );

        match link.send(delay, &tx_buf[..tx_len]).await {
            Ok(()) => {
                stats.tx_ok += 1;
                info!(
                    "RadioTask: LoRaLink send OK (PING) app_seq_tx={} tx_ok={}",
                    app_seq_tx,
                    stats.tx_ok
                );
            }
            Err(LinkError::Timeout) => {
                stats.tx_timeout += 1;
                warn!(
                    "RadioTask: LoRaLink send TIMEOUT (PING) app_seq_tx={} timeouts={}",
                    app_seq_tx,
                    stats.tx_timeout
                );
                // On timeout, skip waiting for PONG and move to next cycle.
                delay.delay_ms(gap).await;
                continue;
            }
            Err(LinkError::Radio) => {
                stats.tx_radio_err += 1;
                warn!(
                    "RadioTask: LoRaLink RADIO error on send (PING) app_seq_tx={} radio_errs={}",
                    app_seq_tx,
                    stats.tx_radio_err
                );
                delay.delay_ms(gap).await;
                continue;
            }
            Err(e) => {
                warn!(
                    "RadioTask: unexpected LinkError on send (PING) app_seq_tx={}: {:?}",
                    app_seq_tx,
                    e
                );
                delay.delay_ms(gap).await;
                continue;
            }
        }

        // -------------------------------------------------------------
        // Now wait for PONG from GS.
        // This is where the link becomes truly bidirectional:
        // - Radio: PING → send
        // - GS:    receives PING, replies with PONG
        // - Radio: receives PONG here via recv()
        // -------------------------------------------------------------
        info!(
            "RadioTask: waiting for PONG in response to app_seq_tx={}",
            app_seq_tx
        );

        let len = match link.recv(delay, &mut rx_buf).await {
            Ok(len) => len,
            Err(LinkError::Radio) => {
                warn!("RadioTask: RADIO error in recv while waiting for PONG");
                continue;
            }
            Err(LinkError::Timeout) => {
                // Normally recv() is a blocking loop; hitting Timeout here means
                // something unusual at the radio layer.
                warn!("RadioTask: TIMEOUT in recv while waiting for PONG");
                continue;
            }
            Err(LinkError::CorruptFrame) => {
                stats.rx_corrupt += 1;
                warn!(
                    "RadioTask: CORRUPT frame while waiting for PONG (count={})",
                    stats.rx_corrupt
                );
                continue;
            }
            Err(LinkError::TooLarge) => {
                warn!("RadioTask: frame too large while waiting for PONG");
                continue;
            }
        };

        if len < 5 {
            warn!(
                "RadioTask: short frame while waiting for PONG len={}",
                len
            );
            continue;
        }

        let kind = rx_buf[0];
        let pong_seq: u32 = ((rx_buf[1] as u32) << 24)
            | ((rx_buf[2] as u32) << 16)
            | ((rx_buf[3] as u32) << 8)
            | (rx_buf[4] as u32);

        if kind != KIND_APP_PONG {
            warn!(
                "RadioTask: unexpected kind=0x{:02X} while waiting for PONG (len={})",
                kind, len
            );
            continue;
        }

        // Validate payload pattern end-to-end
        for i in 5..len {
            let expected = (pong_seq as u8).wrapping_add(i as u8);
            if rx_buf[i] != expected {
                warn!(
                    "RadioTask: PONG payload mismatch at i={} got=0x{:02X} exp=0x{:02X}",
                    i, rx_buf[i], expected
                );
                stats.rx_corrupt += 1;
                break;
            }
        }

        stats.rx_ok += 1;
        info!(
            "RadioTask: RECV PONG pong_seq={} for ping_seq={} rx_ok={}",
            pong_seq,
            app_seq_tx,
            stats.rx_ok
        );

        delay.delay_ms(gap).await;
    }
}

// -------------------------------------------------------------------------
// GS side: responder (PING -> PONG)
// -------------------------------------------------------------------------

async fn run_gs_server<'a, RADIO>(
    link: &mut LoRaLink<'a, RADIO>,
    delay: &mut impl DelayMs,
) where
    RADIO: Sx1262Interface,
{
    info!("GsTask: starting bidirectional LoRaLink test (server)");

    if let Err(e) = link.start_rx(delay).await {
        warn!("GsTask: initial start_rx failed: {:?}", e);
    }

    let mut stats = LinkStats::default();
    let mut app_seq_tx: u32 = 0;

    let mut rx_buf = [0u8; 255];
    let mut tx_buf = [0u8; LINK_MAX_PAYLOAD];

    loop {
        // -------------------------------------------------------------
        // Wait for a PING from the Radio node.
        // recv():
        //   - Blocks until a valid LoRaLink frame
        //   - Validates header
        //   - Sends ACK automatically
        //   - Returns payload bytes len
        // -------------------------------------------------------------
        let len = match link.recv(delay, &mut rx_buf).await {
            Ok(len) => len,
            Err(LinkError::Radio) => {
                warn!("GsTask: RADIO error in recv (waiting for PING)");
                continue;
            }
            Err(LinkError::Timeout) => {
                warn!("GsTask: TIMEOUT in recv (waiting for PING)");
                continue;
            }
            Err(LinkError::CorruptFrame) => {
                stats.rx_corrupt += 1;
                warn!(
                    "GsTask: CORRUPT frame while waiting for PING (count={})",
                    stats.rx_corrupt
                );
                continue;
            }
            Err(LinkError::TooLarge) => {
                warn!("GsTask: frame too large while waiting for PING");
                continue;
            }
        };

        if len < 5 {
            warn!("GsTask: short frame while waiting for PING len={}", len);
            continue;
        }

        let kind = rx_buf[0];
        let ping_seq: u32 = ((rx_buf[1] as u32) << 24)
            | ((rx_buf[2] as u32) << 16)
            | ((rx_buf[3] as u32) << 8)
            | (rx_buf[4] as u32);

        if kind != KIND_APP_PING {
            warn!(
                "GsTask: unexpected kind=0x{:02X} (expected PING) len={}",
                kind, len
            );
            continue;
        }

        // Validate PING payload pattern
        for i in 5..len {
            let expected = (ping_seq as u8).wrapping_add(i as u8);
            if rx_buf[i] != expected {
                warn!(
                    "GsTask: PING payload mismatch at i={} got=0x{:02X} exp=0x{:02X}",
                    i, rx_buf[i], expected
                );
                stats.rx_corrupt += 1;
                break;
            }
        }

        stats.rx_ok += 1;
        info!(
            "GsTask: RECV PING ping_seq={} len={} rx_ok={}",
            ping_seq,
            len,
            stats.rx_ok
        );

        // -------------------------------------------------------------
        // Build PONG response:
        //   [0]    = KIND_APP_PONG
        //   [1..4] = app_seq_tx (server-side sequence)
        //   [5..]  = deterministic pattern based on app_seq_tx
        // -------------------------------------------------------------
        app_seq_tx = app_seq_tx.wrapping_add(1);

        let mut tx_len = choose_payload_len(app_seq_tx);
        if tx_len < 16 {
            tx_len = 16;
        }
        if tx_len > LINK_MAX_PAYLOAD {
            tx_len = LINK_MAX_PAYLOAD;
        }

        tx_buf[0] = KIND_APP_PONG;
        tx_buf[1] = (app_seq_tx >> 24) as u8;
        tx_buf[2] = (app_seq_tx >> 16) as u8;
        tx_buf[3] = (app_seq_tx >> 8) as u8;
        tx_buf[4] = app_seq_tx as u8;

        for i in 5..tx_len {
            tx_buf[i] = (app_seq_tx as u8).wrapping_add(i as u8);
        }

        info!(
            "GsTask: SEND PONG app_seq_tx={} in response to ping_seq={}",
            app_seq_tx,
            ping_seq
        );

        match link.send(delay, &tx_buf[..tx_len]).await {
            Ok(()) => {
                stats.tx_ok += 1;
                info!(
                    "GsTask: LoRaLink send OK (PONG) app_seq_tx={} tx_ok={}",
                    app_seq_tx,
                    stats.tx_ok
                );
            }
            Err(LinkError::Timeout) => {
                stats.tx_timeout += 1;
                warn!(
                    "GsTask: LoRaLink send TIMEOUT (PONG) app_seq_tx={} timeouts={}",
                    app_seq_tx,
                    stats.tx_timeout
                );
            }
            Err(LinkError::Radio) => {
                stats.tx_radio_err += 1;
                warn!(
                    "GsTask: LoRaLink RADIO error on send (PONG) app_seq_tx={} radio_errs={}",
                    app_seq_tx,
                    stats.tx_radio_err
                );
            }
            Err(e) => {
                warn!(
                    "GsTask: unexpected LinkError on send (PONG) app_seq_tx={}: {:?}",
                    app_seq_tx,
                    e
                );
            }
        }
    }
}
