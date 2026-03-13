//! Channel-facing SD logging helpers.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::{Channel, TrySendError};

use crate::interfaces::storage::{LogError, LoggerEngine};
use crate::messages::logging::LogCommand;

pub type LogChannel<M, const DEPTH: usize> = Channel<M, LogCommand, DEPTH>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum TryEnqueueLogError {
    Build(LogError),
    ChannelFull,
}

pub fn try_enqueue_line<M, const DEPTH: usize>(
    channel: &LogChannel<M, DEPTH>,
    path: &str,
    line: &str,
) -> Result<(), TryEnqueueLogError>
where
    M: RawMutex,
{
    let command = LogCommand::append_line(path, line).map_err(TryEnqueueLogError::Build)?;
    channel
        .try_send(command)
        .map_err(|TrySendError::Full(_)| TryEnqueueLogError::ChannelFull)
}

pub async fn enqueue_line<M, const DEPTH: usize>(
    channel: &LogChannel<M, DEPTH>,
    path: &str,
    line: &str,
) -> Result<(), LogError>
where
    M: RawMutex,
{
    channel
        .send(LogCommand::append_line(path, line)?)
        .await;
    Ok(())
}

pub fn handle_log_command<E>(engine: &mut E, command: LogCommand) -> Result<(), LogError>
where
    E: LoggerEngine,
{
    match command {
        LogCommand::AppendLine { path, line } => engine.append_line(path.as_str(), line.as_str()),
    }
}
