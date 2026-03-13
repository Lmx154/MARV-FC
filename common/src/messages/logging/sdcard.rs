//! Typed queue messages for the SD logging pipeline.

use crate::interfaces::storage::{LogError, LogLine, LogPath};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LogCommand {
    AppendLine { path: LogPath, line: LogLine },
}

impl LogCommand {
    pub fn append_line(path: &str, line: &str) -> Result<Self, LogError> {
        let mut owned_path = LogPath::new();
        owned_path.push_str(path).map_err(|_| LogError::PathTooLong)?;

        let mut owned_line = LogLine::new();
        owned_line.push_str(line).map_err(|_| LogError::LineTooLong)?;

        Ok(Self::AppendLine {
            path: owned_path,
            line: owned_line,
        })
    }
}
