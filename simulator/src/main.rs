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
use common::services::telemetry::MavlinkStreamPump;
use config::Config;
use fs_logger::FsLogger;
use mavlink::DecodeError;
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
    let mut stream = MavlinkStreamPump::<RX_BUFFER_LIMIT>::new();
    let tick_timeout = Duration::from_millis(config.tick_timeout_ms);
    let mut receive_buffer = [0u8; 512];
    let mut last_tick_at = Instant::now();
    let mut timeout_reported = false;

    println!("FC SITL RX listening on {}", config.bind_addr);

    loop {
        match timeout(tick_timeout, rx_socket.recv_from(&mut receive_buffer)).await {
            Ok(Ok((len, _peer))) => {
                stream.ingest_bytes(&receive_buffer[..len]);

                while let Some(frame) = stream.try_next_frame() {
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
