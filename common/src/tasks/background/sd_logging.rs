//! Portable log-sink task body for a buffered logging channel.

use embassy_sync::blocking_mutex::raw::RawMutex;

use crate::interfaces::storage::{LogError, LoggerEngine};
use crate::services::logging::{LogChannel, handle_log_command};

pub async fn run_sd_logging_task<M, E, F, const DEPTH: usize>(
    channel: &'static LogChannel<M, DEPTH>,
    mut engine: E,
    mut on_error: F,
) -> !
where
    M: RawMutex,
    E: LoggerEngine,
    F: FnMut(LogError),
{
    let receiver = channel.receiver();

    loop {
        let command = receiver.receive().await;
        if let Err(error) = handle_log_command(&mut engine, command) {
            on_error(error);
        }
    }
}
