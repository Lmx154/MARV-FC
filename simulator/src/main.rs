mod config;
mod fs_logger;
mod mavlink;
mod tx_actuator;

use std::error::Error;
use std::time::Duration;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;

use common::protocol::mavlink::{DecodeError, MavlinkHilMessagePump};
use common::services::acquisition::{
    BarometerSampleChannel, GpsFixSampleChannel, ImuSampleChannel, TimeSampleChannel,
};
use common::services::hil::{
    HilByteReader, HilIngressLoopError, HilIngressRoutes, HilRuntime, run_hil_ingress_loop,
};
use config::Config;
use fs_logger::FsLogger;
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

enum UdpReadError {
    Io(std::io::Error),
    Timeout,
}

struct UdpHilReader {
    socket: UdpSocket,
    timeout: Duration,
}

impl HilByteReader for UdpHilReader {
    type Error = UdpReadError;

    async fn read(&mut self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        match timeout(self.timeout, self.socket.recv_from(buffer)).await {
            Ok(Ok((len, _peer))) => Ok(len),
            Ok(Err(error)) => Err(UdpReadError::Io(error)),
            Err(_) => Err(UdpReadError::Timeout),
        }
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<(), Box<dyn Error>> {
    let config = Config::from_env().map_err(std::io::Error::other)?;
    let reader_timeout = Duration::from_millis(config.tick_timeout_ms);
    let mut reader = UdpHilReader {
        socket: UdpSocket::bind(config.bind_addr).await?,
        timeout: reader_timeout,
    };
    let mut logger = FsLogger::new(
        &config.log_dir,
        &IMU_CHANNEL,
        &BAROMETER_CHANNEL,
        &GPS_CHANNEL,
    )?;
    let routes = HilIngressRoutes::new(
        &TIME_CHANNEL,
        &IMU_CHANNEL,
        &BAROMETER_CHANNEL,
        &GPS_CHANNEL,
        &(),
        &(),
    );
    let mut protocol = MavlinkHilMessagePump::<RX_BUFFER_LIMIT>::new();
    let mut runtime = HilRuntime::new();
    runtime.enable_virtual_sensor_streaming();
    let mut receive_buffer = [0u8; 512];
    let mut timeout_reported = false;

    println!("FC SITL RX listening on {}", config.bind_addr);

    loop {
        match run_hil_ingress_loop(
            &mut reader,
            &mut protocol,
            &mut runtime,
            &routes,
            &mut receive_buffer,
            |dispatch| {
                if let Some(tick) = dispatch.tick {
                    logger.log_tick(tick)?;
                    println!("tick={}", tick.time_boot_ms);
                    timeout_reported = false;
                }
                Ok::<(), fs_logger::FsLoggerError>(())
            },
            |error| match error {
                DecodeError::UnsupportedMessage { .. } => {}
                error => eprintln!("rx decode error: {error}"),
            },
        )
        .await
        {
            Err(HilIngressLoopError::Transport(UdpReadError::Io(error))) => {
                return Err(Box::new(error) as Box<dyn Error>);
            }
            Err(HilIngressLoopError::Transport(UdpReadError::Timeout)) if !timeout_reported => {
                eprintln!(
                    "rx timeout: no SYSTEM_TIME within {} ms",
                    config.tick_timeout_ms
                );
                timeout_reported = true;
            }
            Err(HilIngressLoopError::Transport(UdpReadError::Timeout)) => {}
            Err(HilIngressLoopError::Dispatch(error)) => return Err(Box::new(error)),
            Ok(()) => unreachable!(),
        }
    }

    #[allow(unreachable_code)]
    Ok(())
}
