//! Hardware-agnostic SD logging abstraction.

use heapless::{String, Vec};

pub const MAX_LOG_PREFIX_LEN: usize = 4;
pub const MAX_LOG_PATH_LEN: usize = 64;
pub const MAX_LOG_LINE_LEN: usize = 1024;
pub const MAX_LOG_ENTRIES: usize = 64;
pub const MAX_LOG_LINES: usize = 32;

pub type LogPath = String<MAX_LOG_PATH_LEN>;
pub type LogLine = String<MAX_LOG_LINE_LEN>;
pub type LogEntryList = Vec<LogPath, MAX_LOG_ENTRIES>;
pub type LogLineList = Vec<LogLine, MAX_LOG_LINES>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum LogError {
    Device,
    Filesystem,
    NotFound,
    Busy,
    AlreadyExists,
    ReadOnly,
    InvalidName,
    InvalidPath,
    PrefixTooLong,
    PathTooLong,
    LineTooLong,
    TooManyEntries,
    TooManyLinesRequested,
    SequenceOverflow,
    Utf8,
}

pub trait LoggerEngine {
    fn create_new_csv(&mut self, prefix: &str) -> Result<LogPath, LogError>;
    fn list_files(&mut self, path: &str) -> Result<LogEntryList, LogError>;
    fn list_directories(&mut self, path: &str) -> Result<LogEntryList, LogError>;
    fn read_lines(
        &mut self,
        path: &str,
        start_line: usize,
        count: usize,
    ) -> Result<LogLineList, LogError>;
    fn append_line(&mut self, filename: &str, data: &str) -> Result<(), LogError>;
}
