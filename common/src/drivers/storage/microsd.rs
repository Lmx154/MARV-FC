//! `embedded_sdmmc`-backed logger engine.
//!
//! This stays portable by depending only on `embedded_sdmmc` traits and
//! generic block devices/time sources. The blocking nature of the filesystem is
//! isolated behind the `LoggerEngine` trait so flight-critical code only sees
//! a queue-facing producer API.

use core::fmt::Write;
use core::str;

use embedded_sdmmc::{
    BlockDevice, Error as SdError, FilenameError, Mode, RawDirectory, RawFile, RawVolume,
    ShortFileName, TimeSource, VolumeIdx, VolumeManager,
};
use heapless::{String, Vec};

use crate::interfaces::storage::{
    LogEntryList, LogError, LogLine, LogLineList, LogPath, LoggerEngine, MAX_LOG_ENTRIES,
    MAX_LOG_LINE_LEN, MAX_LOG_LINES, MAX_LOG_PREFIX_LEN,
};

pub const DEFAULT_FLUSH_EVERY_LINES: usize = 16;

const READ_BUFFER_BYTES: usize = 512;
const CSV_SEQUENCE_DIGITS: usize = 4;
const CSV_EXTENSION: &str = "CSV";

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MicrosdLoggerConfig {
    pub volume_idx: VolumeIdx,
    pub log_root: LogPath,
    pub flush_every_lines: usize,
}

impl MicrosdLoggerConfig {
    pub fn with_log_root(log_root: &str) -> Result<Self, LogError> {
        Ok(Self {
            volume_idx: VolumeIdx(0),
            log_root: normalize_user_path("/", log_root)?,
            flush_every_lines: DEFAULT_FLUSH_EVERY_LINES,
        })
    }
}

impl Default for MicrosdLoggerConfig {
    fn default() -> Self {
        let mut log_root = LogPath::new();
        let _ = log_root.push('/');
        Self {
            volume_idx: VolumeIdx(0),
            log_root,
            flush_every_lines: DEFAULT_FLUSH_EVERY_LINES,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct ActiveLogFile {
    path: LogPath,
    handle: RawFile,
}

pub struct MicrosdLogger<
    D,
    TS,
    const MAX_DIRS: usize = 8,
    const MAX_FILES: usize = 4,
    const MAX_VOLUMES: usize = 1,
> where
    D: BlockDevice,
    TS: TimeSource,
{
    volume_mgr: VolumeManager<D, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    volume: RawVolume,
    config: MicrosdLoggerConfig,
    active_file: Option<ActiveLogFile>,
    pending_lines: usize,
}

impl<D, TS, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_VOLUMES: usize>
    MicrosdLogger<D, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
where
    D: BlockDevice,
    TS: TimeSource,
{
    pub fn new(
        block_device: D,
        time_source: TS,
        config: MicrosdLoggerConfig,
    ) -> Result<Self, LogError> {
        let mut volume_mgr =
            VolumeManager::<D, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>::new_with_limits(
                block_device,
                time_source,
                0,
            );
        let volume = volume_mgr
            .open_raw_volume(config.volume_idx)
            .map_err(map_sd_error)?;

        Ok(Self {
            volume_mgr,
            volume,
            config,
            active_file: None,
            pending_lines: 0,
        })
    }

    fn close_active_file(&mut self) -> Result<(), LogError> {
        if let Some(active) = self.active_file.as_ref() {
            self.volume_mgr
                .close_file(active.handle)
                .map_err(map_sd_error)?;
            self.active_file = None;
            self.pending_lines = 0;
        }
        Ok(())
    }

    fn open_directory(&mut self, path: &str, create: bool) -> Result<RawDirectory, LogError> {
        let normalized = normalize_user_path("/", path)?;
        let mut current = self
            .volume_mgr
            .open_root_dir(self.volume)
            .map_err(map_sd_error)?;

        for component in normalized.as_str().split('/').filter(|part| !part.is_empty()) {
            let next = match self.volume_mgr.open_dir(current, component) {
                Ok(dir) => dir,
                Err(SdError::NotFound) if create => {
                    self.volume_mgr
                        .make_dir_in_dir(current, component)
                        .map_err(map_sd_error)?;
                    self.volume_mgr
                        .open_dir(current, component)
                        .map_err(map_sd_error)?
                }
                Err(err) => {
                    let _ = self.volume_mgr.close_dir(current);
                    return Err(map_sd_error(err));
                }
            };

            self.volume_mgr.close_dir(current).map_err(map_sd_error)?;
            current = next;
        }

        Ok(current)
    }

    fn ensure_active_file(&mut self, filename: &str) -> Result<RawFile, LogError> {
        let path = normalize_user_path(self.config.log_root.as_str(), filename)?;

        if let Some(active) = self.active_file.as_ref() {
            if active.path == path {
                return Ok(active.handle);
            }
        }

        self.close_active_file()?;

        let (parent, file_name) = split_parent_and_name(&path)?;
        let dir = self.open_directory(parent.as_str(), true)?;
        let file = self
            .volume_mgr
            .open_file_in_dir(dir, &file_name, Mode::ReadWriteCreateOrAppend)
            .map_err(map_sd_error)?;
        self.volume_mgr.close_dir(dir).map_err(map_sd_error)?;

        self.active_file = Some(ActiveLogFile {
            path,
            handle: file,
        });

        Ok(file)
    }

    fn create_empty_file(&mut self, directory: RawDirectory, name: &ShortFileName) -> Result<(), LogError> {
        let file = self
            .volume_mgr
            .open_file_in_dir(directory, name, Mode::ReadWriteCreate)
            .map_err(map_sd_error)?;
        self.volume_mgr.close_file(file).map_err(map_sd_error)
    }

    fn collect_entries(
        &mut self,
        directory: RawDirectory,
        current_path: &LogPath,
        kind: EntryKind,
        out: &mut LogEntryList,
    ) -> Result<(), LogError> {
        let mut child_dirs: Vec<PendingDir, MAX_LOG_ENTRIES> = Vec::new();
        let mut closure_error: Option<LogError> = None;

        self.volume_mgr
            .iterate_dir(directory, |entry| {
                if closure_error.is_some() || is_special_directory(&entry.name) || entry.attributes.is_lfn() {
                    return;
                }

                let entry_name = match short_name_to_string(&entry.name) {
                    Ok(name) => name,
                    Err(err) => {
                        closure_error = Some(err);
                        return;
                    }
                };

                let full_path = match join_path(current_path.as_str(), entry_name.as_str()) {
                    Ok(path) => path,
                    Err(err) => {
                        closure_error = Some(err);
                        return;
                    }
                };

                if entry.attributes.is_directory() {
                    if matches!(kind, EntryKind::Directories) && out.push(full_path.clone()).is_err() {
                        closure_error = Some(LogError::TooManyEntries);
                        return;
                    }

                    if child_dirs
                        .push(PendingDir {
                            name: entry.name.clone(),
                            path: full_path,
                        })
                        .is_err()
                    {
                        closure_error = Some(LogError::TooManyEntries);
                    }
                } else if matches!(kind, EntryKind::Files) && out.push(full_path).is_err() {
                    closure_error = Some(LogError::TooManyEntries);
                }
            })
            .map_err(map_sd_error)?;

        if let Some(err) = closure_error {
            return Err(err);
        }

        for child in child_dirs {
            let subdir = self
                .volume_mgr
                .open_dir(directory, &child.name)
                .map_err(map_sd_error)?;
            self.collect_entries(subdir, &child.path, kind, out)?;
            self.volume_mgr.close_dir(subdir).map_err(map_sd_error)?;
        }

        Ok(())
    }

    fn read_lines_inner(
        &mut self,
        file: RawFile,
        start_line: usize,
        count: usize,
    ) -> Result<LogLineList, LogError> {
        let mut output = LogLineList::new();
        let mut buffer = [0u8; READ_BUFFER_BYTES];
        let mut line_bytes: Vec<u8, MAX_LOG_LINE_LEN> = Vec::new();
        let mut current_line = 0usize;
        let mut done = false;

        while !done {
            let read = self
                .volume_mgr
                .read(file, &mut buffer)
                .map_err(map_sd_error)?;
            if read == 0 {
                break;
            }

            for &byte in &buffer[..read] {
                if byte == b'\n' {
                    if current_line >= start_line && output.len() < count {
                        push_line(&mut output, &line_bytes)?;
                    }
                    line_bytes.clear();
                    current_line += 1;

                    if output.len() == count {
                        done = true;
                        break;
                    }

                    continue;
                }

                if byte != b'\r' && current_line >= start_line && output.len() < count {
                    line_bytes
                        .push(byte)
                        .map_err(|_| LogError::LineTooLong)?;
                }
            }
        }

        if !done && !line_bytes.is_empty() && current_line >= start_line && output.len() < count {
            push_line(&mut output, &line_bytes)?;
        }

        Ok(output)
    }
}

impl<D, TS, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_VOLUMES: usize> Drop
    for MicrosdLogger<D, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
where
    D: BlockDevice,
    TS: TimeSource,
{
    fn drop(&mut self) {
        let _ = self.close_active_file();
        let _ = self.volume_mgr.close_volume(self.volume);
    }
}

impl<D, TS, const MAX_DIRS: usize, const MAX_FILES: usize, const MAX_VOLUMES: usize> LoggerEngine
    for MicrosdLogger<D, TS, MAX_DIRS, MAX_FILES, MAX_VOLUMES>
where
    D: BlockDevice,
    TS: TimeSource,
{
    fn create_new_csv(&mut self, prefix: &str) -> Result<LogPath, LogError> {
        let log_root = self.config.log_root.clone();
        let directory = self.open_directory(log_root.as_str(), true)?;
        let mut highest = None;

        self.volume_mgr
            .iterate_dir(directory, |entry| {
                if entry.attributes.is_directory() || entry.attributes.is_lfn() {
                    return;
                }

                if let Some(sequence) = parse_csv_sequence(&entry.name, prefix) {
                    highest = Some(highest.map_or(sequence, |current: u16| current.max(sequence)));
                }
            })
            .map_err(map_sd_error)?;

        let next_sequence = highest.unwrap_or(0).saturating_add(1);
        if next_sequence > 9999 {
            let _ = self.volume_mgr.close_dir(directory);
            return Err(LogError::SequenceOverflow);
        }

        let file_name = format_csv_name(prefix, next_sequence)?;
        let file_path = join_path(
            log_root.as_str(),
            short_name_to_string(&file_name)?.as_str(),
        )?;

        self.create_empty_file(directory, &file_name)?;
        self.volume_mgr.close_dir(directory).map_err(map_sd_error)?;

        Ok(file_path)
    }

    fn list_files(&mut self, path: &str) -> Result<LogEntryList, LogError> {
        let normalized = normalize_user_path(self.config.log_root.as_str(), path)?;
        let dir = self.open_directory(normalized.as_str(), false)?;
        let mut out = LogEntryList::new();
        let result = self.collect_entries(dir, &normalized, EntryKind::Files, &mut out);
        let close_result = self.volume_mgr.close_dir(dir).map_err(map_sd_error);
        result?;
        close_result?;
        Ok(out)
    }

    fn list_directories(&mut self, path: &str) -> Result<LogEntryList, LogError> {
        let normalized = normalize_user_path(self.config.log_root.as_str(), path)?;
        let dir = self.open_directory(normalized.as_str(), false)?;
        let mut out = LogEntryList::new();
        let result = self.collect_entries(dir, &normalized, EntryKind::Directories, &mut out);
        let close_result = self.volume_mgr.close_dir(dir).map_err(map_sd_error);
        result?;
        close_result?;
        Ok(out)
    }

    fn read_lines(
        &mut self,
        path: &str,
        start_line: usize,
        count: usize,
    ) -> Result<LogLineList, LogError> {
        if count > MAX_LOG_LINES {
            return Err(LogError::TooManyLinesRequested);
        }

        let normalized = normalize_user_path(self.config.log_root.as_str(), path)?;
        if self
            .active_file
            .as_ref()
            .is_some_and(|active| active.path == normalized)
        {
            self.close_active_file()?;
        }

        let (parent, file_name) = split_parent_and_name(&normalized)?;
        let dir = self.open_directory(parent.as_str(), false)?;
        let file = self
            .volume_mgr
            .open_file_in_dir(dir, &file_name, Mode::ReadOnly)
            .map_err(map_sd_error)?;
        self.volume_mgr.close_dir(dir).map_err(map_sd_error)?;

        let result = self.read_lines_inner(file, start_line, count);
        let close_result = self.volume_mgr.close_file(file).map_err(map_sd_error);
        let lines = result?;
        close_result?;
        Ok(lines)
    }

    fn append_line(&mut self, filename: &str, data: &str) -> Result<(), LogError> {
        let file = self.ensure_active_file(filename)?;
        self.volume_mgr
            .write(file, data.as_bytes())
            .map_err(map_sd_error)?;
        self.volume_mgr.write(file, b"\n").map_err(map_sd_error)?;

        self.pending_lines = self.pending_lines.saturating_add(1);
        if self.pending_lines >= self.config.flush_every_lines.max(1) {
            self.close_active_file()?;
        }

        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EntryKind {
    Files,
    Directories,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct PendingDir {
    name: ShortFileName,
    path: LogPath,
}

fn normalize_user_path(base: &str, path: &str) -> Result<LogPath, LogError> {
    let mut normalized = LogPath::new();
    normalized.push('/').map_err(|_| LogError::PathTooLong)?;

    let mut push_component = |component: &str| -> Result<(), LogError> {
        if component.is_empty() || component == "." {
            return Ok(());
        }
        if component == ".." {
            return Err(LogError::InvalidPath);
        }

        if normalized.len() > 1 {
            normalized.push('/').map_err(|_| LogError::PathTooLong)?;
        }
        normalized
            .push_str(component)
            .map_err(|_| LogError::PathTooLong)
    };

    let source = if path.starts_with('/') {
        path
    } else if path.is_empty() {
        base
    } else {
        for component in base.split('/') {
            push_component(component)?;
        }
        path
    };

    for component in source.split('/') {
        push_component(component)?;
    }

    Ok(normalized)
}

fn join_path(base: &str, child: &str) -> Result<LogPath, LogError> {
    let mut path = normalize_user_path("/", base)?;
    if path.len() > 1 {
        path.push('/').map_err(|_| LogError::PathTooLong)?;
    }
    path.push_str(child).map_err(|_| LogError::PathTooLong)?;
    Ok(path)
}

fn split_parent_and_name(path: &LogPath) -> Result<(LogPath, ShortFileName), LogError> {
    let slash = path.as_str().rfind('/').ok_or(LogError::InvalidPath)?;
    let parent = if slash == 0 {
        normalize_user_path("/", "/")?
    } else {
        normalize_user_path("/", &path.as_str()[..slash])?
    };
    let name = &path.as_str()[slash + 1..];
    if name.is_empty() {
        return Err(LogError::InvalidPath);
    }
    let short_name = ShortFileName::create_from_str(name).map_err(map_filename_error)?;
    Ok((parent, short_name))
}

fn short_name_to_string(name: &ShortFileName) -> Result<LogPath, LogError> {
    let mut out = LogPath::new();
    write!(&mut out, "{name}").map_err(|_| LogError::PathTooLong)?;
    Ok(out)
}

fn is_special_directory(name: &ShortFileName) -> bool {
    *name == ShortFileName::this_dir() || *name == ShortFileName::parent_dir()
}

fn parse_csv_sequence(name: &ShortFileName, prefix: &str) -> Option<u16> {
    let mut prefix_upper: String<MAX_LOG_PREFIX_LEN> = String::new();
    if prefix.is_empty() || prefix.len() > MAX_LOG_PREFIX_LEN {
        return None;
    }

    for byte in prefix.bytes() {
        if prefix_upper.push(byte.to_ascii_uppercase() as char).is_err() {
            return None;
        }
    }

    let base = str::from_utf8(name.base_name()).ok()?;
    let extension = str::from_utf8(name.extension()).ok()?;

    if extension != CSV_EXTENSION || !base.starts_with(prefix_upper.as_str()) {
        return None;
    }

    let digits = &base[prefix_upper.len()..];
    if digits.len() != CSV_SEQUENCE_DIGITS || !digits.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }

    digits.parse::<u16>().ok()
}

fn format_csv_name(prefix: &str, sequence: u16) -> Result<ShortFileName, LogError> {
    if prefix.is_empty() {
        return Err(LogError::InvalidName);
    }
    if prefix.len() > MAX_LOG_PREFIX_LEN {
        return Err(LogError::PrefixTooLong);
    }

    let mut name: String<12> = String::new();
    for byte in prefix.bytes() {
        name.push(byte.to_ascii_uppercase() as char)
            .map_err(|_| LogError::InvalidName)?;
    }
    write!(&mut name, "{sequence:04}.{CSV_EXTENSION}").map_err(|_| LogError::InvalidName)?;

    ShortFileName::create_from_str(name.as_str()).map_err(map_filename_error)
}

fn push_line(lines: &mut LogLineList, bytes: &[u8]) -> Result<(), LogError> {
    let text = str::from_utf8(bytes).map_err(|_| LogError::Utf8)?;
    let mut line = LogLine::new();
    line.push_str(text).map_err(|_| LogError::LineTooLong)?;
    lines.push(line).map_err(|_| LogError::TooManyLinesRequested)
}

fn map_filename_error(error: FilenameError) -> LogError {
    match error {
        FilenameError::NameTooLong => LogError::InvalidName,
        FilenameError::FilenameEmpty => LogError::InvalidName,
        FilenameError::MisplacedPeriod => LogError::InvalidName,
        FilenameError::InvalidCharacter => LogError::InvalidName,
        FilenameError::Utf8Error => LogError::Utf8,
    }
}

fn map_sd_error<E>(error: SdError<E>) -> LogError
where
    E: core::fmt::Debug,
{
    match error {
        SdError::DeviceError(_) => LogError::Device,
        SdError::FilenameError(err) => map_filename_error(err),
        SdError::NotFound => LogError::NotFound,
        SdError::FileAlreadyOpen | SdError::DirAlreadyOpen | SdError::VolumeAlreadyOpen => {
            LogError::Busy
        }
        SdError::FileAlreadyExists | SdError::DirAlreadyExists => LogError::AlreadyExists,
        SdError::ReadOnly => LogError::ReadOnly,
        _ => LogError::Filesystem,
    }
}

#[cfg(test)]
mod tests {
    use super::{format_csv_name, normalize_user_path, parse_csv_sequence};
    use embedded_sdmmc::ShortFileName;

    #[test]
    fn normalizes_relative_path_against_root() {
        assert_eq!(
            normalize_user_path("/LOGS", "SESSION/LOG_0001.CSV")
                .unwrap()
                .as_str(),
            "/LOGS/SESSION/LOG_0001.CSV"
        );
    }

    #[test]
    fn formats_csv_name_with_8_3_limit() {
        let name = format_csv_name("LOG_", 12).unwrap();
        assert_eq!(name.to_string(), "LOG_0012.CSV");
    }

    #[test]
    fn parses_existing_sequence_from_short_name() {
        let name = ShortFileName::create_from_str("LOG_0042.CSV").unwrap();
        assert_eq!(parse_csv_sequence(&name, "LOG_"), Some(42));
    }
}
