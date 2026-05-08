//! Portable log-sink task body for a buffered logging channel.

use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::channel::TrySendError;

use crate::interfaces::storage::{LogError, LoggerEngine};
use crate::messages::logging::LogSinkState;
use crate::services::logging::{LogChannel, LogSinkStateChannel, handle_log_command};

pub async fn run_sd_logging_task<M, S, E, F, const DEPTH: usize, const STATUS_DEPTH: usize>(
    channel: &'static LogChannel<M, DEPTH>,
    sink_state_channel: Option<&'static LogSinkStateChannel<S, STATUS_DEPTH>>,
    mut engine: E,
    mut on_error: F,
) -> !
where
    M: RawMutex,
    S: RawMutex,
    E: LoggerEngine,
    F: FnMut(LogError),
{
    let receiver = channel.receiver();
    let sink_state_sender = sink_state_channel.map(|channel| channel.sender());
    let mut reported_state = LogSinkState::Healthy;

    loop {
        let command = receiver.receive().await;
        match handle_log_command(&mut engine, command) {
            Ok(()) => {
                if reported_state != LogSinkState::Healthy
                    && try_report_sink_state(sink_state_sender, LogSinkState::Healthy)
                {
                    reported_state = LogSinkState::Healthy;
                }
            }
            Err(error) => {
                let state: LogSinkState = error.into();
                if state != reported_state && try_report_sink_state(sink_state_sender, state) {
                    reported_state = state;
                }
                on_error(error);
            }
        }
    }
}

fn try_report_sink_state<S, const STATUS_DEPTH: usize>(
    sender: Option<embassy_sync::channel::Sender<'_, S, LogSinkState, STATUS_DEPTH>>,
    state: LogSinkState,
) -> bool
where
    S: RawMutex,
{
    match sender {
        Some(sender) => match sender.try_send(state) {
            Ok(()) => true,
            Err(TrySendError::Full(_)) => false,
        },
        None => true,
    }
}
