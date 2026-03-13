//! Storage and logging sink capability traits belong here.

pub mod logging;

pub use logging::{
    LogEntryList, LogError, LogLine, LogLineList, LogPath, LoggerEngine, MAX_LOG_ENTRIES,
    MAX_LOG_LINE_LEN, MAX_LOG_LINES, MAX_LOG_PATH_LEN, MAX_LOG_PREFIX_LEN,
};
