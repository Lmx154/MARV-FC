mod config;
mod fs_logger;
mod mavlink;
mod tx_actuator;

use std::error::Error;
use std::time::Duration;
use std::time::Instant;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

use common::services::acquisition::{
    BarometerSampleChannel, GpsFixSampleChannel, ImuSampleChannel, MavlinkHilSensorBridge,
    TimeSampleChannel,
};
use config::Config;
use fs_logger::FsLogger;
use mavlink::{
    DecodeError, MAVLINK_V1_STX, MAVLINK_V2_STX, MIN_MAVLINK_FRAME_LEN, try_consume_frame,
};
use tokio::net::UdpSocket;
use tokio::time::timeout;

const CHANNEL_DEPTH: usize = 16;
const CHANNEL_SUBSCRIBERS: usize = 4;
const CHANNEL_PUBLISHERS: usize = 1;
const RX_BUFFER_LIMIT: usize = 4096;

static TIME_CHANNEL: TimeSampleChannel<
    ThreadModeRawMutex,
    CHANNEL_DEPTH,
    CHANNEL_SUBSCRIBERS,
    CHANNEL_PUBLISHERS,
> = TimeSampleChannel::new();
static IMU_CHANNEL: ImuSampleChannel<
    ThreadModeRawMutex,
    CHANNEL_DEPTH,
    CHANNEL_SUBSCRIBERS,
    CHANNEL_PUBLISHERS,
> = ImuSampleChannel::new();
static BAROMETER_CHANNEL: BarometerSampleChannel<
    ThreadModeRawMutex,
    CHANNEL_DEPTH,
    CHANNEL_SUBSCRIBERS,
    CHANNEL_PUBLISHERS,
> = BarometerSampleChannel::new();
static GPS_CHANNEL: GpsFixSampleChannel<
    ThreadModeRawMutex,
    CHANNEL_DEPTH,
    CHANNEL_SUBSCRIBERS,
    CHANNEL_PUBLISHERS,
> = GpsFixSampleChannel::new();

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn Error>> {
    let config = Config::from_env().map_err(std::io::Error::other)?;
    let rx_socket = UdpSocket::bind(config.bind_addr).await?;
    let mut logger = FsLogger::new(
        &config.log_dir,
        &IMU_CHANNEL,
        &BAROMETER_CHANNEL,
        &GPS_CHANNEL,
    )?;
    let mut bridge = MavlinkHilSensorBridge::default();
    let tick_timeout = Duration::from_millis(config.tick_timeout_ms);
    let mut receive_buffer = [0u8; 512];
    let mut frame_buffer = Vec::<u8>::with_capacity(RX_BUFFER_LIMIT);
    let mut last_tick_at = Instant::now();
    let mut timeout_reported = false;

    println!("FC SITL RX listening on {}", config.bind_addr);

    loop {
        match timeout(tick_timeout, rx_socket.recv_from(&mut receive_buffer)).await {
            Ok(Ok((len, _peer))) => {
                extend_frame_buffer(&mut frame_buffer, &receive_buffer[..len]);

                while let Some(frame) = try_next_frame(&mut frame_buffer) {
                    let frame = match frame {
                        Ok(frame) => frame,
                        Err(DecodeError::UnsupportedMessage { .. }) => continue,
                        Err(error) => {
                            eprintln!("rx decode error: {error}");
                            continue;
                        }
                    };

                    let dispatch = bridge.handle_frame(
                        frame,
                        &TIME_CHANNEL,
                        &IMU_CHANNEL,
                        &BAROMETER_CHANNEL,
                        &GPS_CHANNEL,
                    );

                    if let Some(tick) = dispatch.tick {
                        logger.log_tick(tick)?;
                        println!("tick={}", tick.time_boot_ms);
                        last_tick_at = Instant::now();
                        timeout_reported = false;
                    }
                }

                if !timeout_reported && last_tick_at.elapsed() >= tick_timeout {
                    eprintln!(
                        "rx timeout: no SYSTEM_TIME within {} ms",
                        config.tick_timeout_ms
                    );
                    timeout_reported = true;
                }
            }
            Ok(Err(error)) => return Err(Box::new(error) as Box<dyn Error>),
            Err(_) if !timeout_reported => {
                eprintln!(
                    "rx timeout: no SYSTEM_TIME within {} ms",
                    config.tick_timeout_ms
                );
                timeout_reported = true;
            }
            Err(_) => {}
        }
    }

    #[allow(unreachable_code)]
    Ok(())
}

fn extend_frame_buffer(frame_buffer: &mut Vec<u8>, bytes: &[u8]) {
    if bytes.len() >= RX_BUFFER_LIMIT {
        frame_buffer.clear();
        frame_buffer.extend_from_slice(&bytes[bytes.len() - RX_BUFFER_LIMIT..]);
        return;
    }

    let required = frame_buffer.len().saturating_add(bytes.len());
    if required > RX_BUFFER_LIMIT {
        let overflow = required - RX_BUFFER_LIMIT;
        frame_buffer.drain(..overflow.min(frame_buffer.len()));
    }

    frame_buffer.extend_from_slice(bytes);
}

fn try_next_frame(
    frame_buffer: &mut Vec<u8>,
) -> Option<Result<common::protocol::mavlink::MavlinkFrame, DecodeError>> {
    loop {
        let sync_offset = frame_buffer
            .iter()
            .position(|byte| *byte == MAVLINK_V2_STX || *byte == MAVLINK_V1_STX)?;
        if sync_offset > 0 {
            frame_buffer.drain(..sync_offset);
        }

        if frame_buffer.len() < MIN_MAVLINK_FRAME_LEN {
            return None;
        }

        match try_consume_frame(frame_buffer.as_slice()) {
            Ok(Some((frame, consumed))) => {
                frame_buffer.drain(..consumed);
                return Some(Ok(frame));
            }
            Ok(None) => return None,
            Err(DecodeError::BadMagic(_)) => {
                frame_buffer.drain(..1);
            }
            Err(error) => {
                frame_buffer.drain(..1);
                return Some(Err(error));
            }
        }
    }
}
