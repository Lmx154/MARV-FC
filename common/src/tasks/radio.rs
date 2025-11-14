//! Generic async SX1262 transceiver task
//!
//! This task runs a simple TX/RX loop using the generic, HAL-agnostic SX1262 driver.
//! It does not configure any device-specific pins or clocks.
//!
//! Provide a TX source and an RX sink via small async traits so higher layers can
//! connect channels, ring-buffers, etc., without coupling common to a specific executor.

#![allow(async_fn_in_trait)]

use defmt::{info, warn};

use crate::drivers::sx1262::{Sx1262, Sx1262Error, RxReport};
use crate::utils::delay::DelayMs;

/// App-provided async source of outbound frames.
pub trait RadioTxSource {
    /// Try to obtain the next frame to send, writing into `out` and returning the length.
    /// Return None if there's nothing to send right now (non-blocking).
    async fn try_next(&mut self, out: &mut [u8]) -> Option<usize>;
}

/// App-provided async sink for inbound frames.
pub trait RadioRxSink {
    /// Deliver a received frame plus basic metadata (RSSI dBm approx, SNR x 1/4 dB)
    async fn on_frame(&mut self, data: &[u8], rssi_dbm: i16, snr_x4: i16);
}

/// Run a combined TX/RX loop on an initialized SX1262 radio.
/// - `radio` should have been initialized via `init_lora()`
/// - `delay` provides pacing for polls; choose a small interval like 5-10 ms
/// - `tx` supplies outbound payloads opportunistically
/// - `rx` receives inbound payloads when available
pub async fn run_sx1262_lora_transceiver<SPI, NSS, RESET, BUSY, DIO1, DIO2, SW, D, TX, RX>(
    radio: &mut Sx1262<SPI, NSS, RESET, BUSY, DIO1, DIO2, SW>,
    delay: &mut D,
    tx: &mut TX,
    rx: &mut RX,
    poll_interval_ms: u32,
) -> Result<(), Sx1262Error>
where
    SPI: embedded_hal_async::spi::SpiBus<u8>,
    NSS: embedded_hal::digital::OutputPin,
    RESET: embedded_hal::digital::OutputPin,
    BUSY: embedded_hal::digital::InputPin,
    DIO1: embedded_hal::digital::InputPin,
    DIO2: embedded_hal::digital::InputPin,
    SW: crate::drivers::sx1262::RfSwitch,
    D: DelayMs,
    TX: RadioTxSource,
    RX: RadioRxSink,
{
    // Arm continuous RX once; TX will temporarily switch modes then we re-arm RX on success
    let mut rx_buf = [0u8; 255];
    radio.start_rx_continuous(delay).await?;
    info!("SX1262 transceiver task: armed continuous RX");

    loop {
        // 1) Opportunistic TX if a frame is ready
        if let Some(len) = tx.try_next(&mut rx_buf).await { // reuse buffer as TX scratch
            let to_send = &rx_buf[..len.min(255)];
            match radio.tx_send_blocking(delay, to_send, 0x0010_0000).await { // large timeout
                Ok(rep) if rep.done => {
                    // Re-arm RX after TX
                    let _ = radio.start_rx_continuous(delay).await;
                }
                Ok(rep) if rep.timeout => {
                    warn!("SX1262: TX timeout (irq=0x{:x})", rep.irq);
                    let _ = radio.start_rx_continuous(delay).await;
                }
                Ok(_) => {
                    let _ = radio.start_rx_continuous(delay).await;
                }
                Err(e) => {
                    warn!("SX1262: TX error: {:?}", e);
                    let _ = radio.start_rx_continuous(delay).await;
                }
            }
        }

        // 2) Poll RX events
        match radio.poll_rx(delay, &mut rx_buf).await {
            Ok(Some(RxReport { done: true, len, rssi, snr_x4, .. })) if len > 0 => {
                let data = &rx_buf[..len as usize];
                rx.on_frame(data, rssi, snr_x4).await;
            }
            Ok(Some(_evt)) => { /* ignore non-done events */ }
            Ok(None) => { /* nothing */ }
            Err(e) => warn!("SX1262: RX poll error: {:?}", e),
        }

        if poll_interval_ms > 0 { delay.delay_ms(poll_interval_ms).await; }
    }
}
