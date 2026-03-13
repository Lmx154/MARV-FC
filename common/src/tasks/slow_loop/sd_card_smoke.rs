//! Minimal SD-card smoke test task body.

use crate::interfaces::storage::{LogError, LogPath, LoggerEngine};

pub fn run_sd_card_smoke_test<E>(engine: &mut E) -> Result<LogPath, LogError>
where
    E: LoggerEngine,
{
    let path = engine.create_new_csv("TEST")?;
    engine.append_line(path.as_str(), "hello world")?;
    Ok(path)
}
