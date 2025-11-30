//! Async SPI SD blackbox storage helpers for logs/configs.
//! This keeps a tiny in-memory view of files so we can exercise the
//! API (create -> write -> edit -> read) before wiring up a full FAT/LFS layer.

use core::str::FromStr;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use heapless::{String, Vec};

/// Max supported file name bytes (ASCII only for now).
const MAX_NAME_LEN: usize = 24;
/// Max bytes we keep per test file. This is deliberately small to keep RAM bounded.
const MAX_FILE_BYTES: usize = 512;
/// Max count of files tracked in-memory until a real filesystem is used.
const MAX_FILES: usize = 4;

type FileName = String<MAX_NAME_LEN>;
type FileBuf = Vec<u8, MAX_FILE_BYTES>;

/// Simple error bucket for the SD helper.
#[derive(Debug, defmt::Format)]
pub enum SdError<SpiE, PinE>
where
    SpiE: defmt::Format,
    PinE: defmt::Format,
{
    Spi(SpiE),
    Cs(PinE),
    NameTooLong,
    TooManyFiles,
    FileTooLarge,
    MissingFile,
    EditTargetMissing,
}

pub type SdResult<T, SpiE, PinE> = Result<T, SdError<SpiE, PinE>>;

#[derive(Clone, Debug, defmt::Format)]
struct FileEntry {
    name: FileName,
    data: FileBuf,
}

/// Minimal async SD helper that uses SPI + CS. The card operations are
/// placeholders; file operations are kept in-memory to prove the interface.
pub struct SdBlackBox<SPI, CS> {
    spi: SPI,
    cs: CS,
    files: Vec<FileEntry, MAX_FILES>,
}

impl<SPI, CS> SdBlackBox<SPI, CS>
where
    SPI: SpiBus<u8>,
    CS: OutputPin,
    SPI::Error: defmt::Format,
    CS::Error: defmt::Format,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self { spi, cs, files: Vec::new() }
    }

    /// Wake the card by clocking a few dummy bytes with CS high.
    /// This is just enough for bring-up plumbing; a full init sequence
    /// (CMD0/CMD8/etc) will be added when the filesystem is wired.
    pub async fn init(&mut self) -> SdResult<(), SPI::Error, CS::Error> {
        self.cs.set_high().map_err(SdError::Cs)?;
        let idle_bytes = [0xFFu8; 10];
        self.spi.write(&idle_bytes).await.map_err(SdError::Spi)?;
        Ok(())
    }

    /// Demo flow: create a file, write initial content, edit it, then
    /// read back the post-edit contents.
    pub async fn run_smoke_test(&mut self) -> SdResult<SmokeTestResult, SPI::Error, CS::Error> {
        const TEST_FILE: &str = "blackbox.txt";

        self.init().await?;
        self.create_or_truncate(TEST_FILE, b"boot=ok\n").await?;
        self.append(TEST_FILE, b"mode=bringup\n").await?;
        self.replace_first(TEST_FILE, b"boot=ok", b"boot=pico").await?;
        let contents = self.read_all(TEST_FILE).await?;

        Ok(SmokeTestResult {
            file_name: to_name(TEST_FILE)?,
            contents,
        })
    }

    /// Create or truncate a file, then write the provided contents.
    pub async fn create_or_truncate(
        &mut self,
        name: &str,
        bytes: &[u8],
    ) -> SdResult<(), SPI::Error, CS::Error> {
        let name = to_name(name)?;
        if bytes.len() > MAX_FILE_BYTES {
            return Err(SdError::FileTooLarge);
        }

        if let Some(entry) = self.find_file_mut(&name) {
            entry.data.clear();
            entry.data.extend_from_slice(bytes).map_err(|_| SdError::FileTooLarge)?;
            return Ok(());
        }

        if self.files.len() == MAX_FILES {
            return Err(SdError::TooManyFiles);
        }

        let mut data = FileBuf::new();
        data.extend_from_slice(bytes).map_err(|_| SdError::FileTooLarge)?;
        let entry = FileEntry { name, data };
        self.files.push(entry).map_err(|_| SdError::TooManyFiles)?;
        Ok(())
    }

    /// Append data to an existing file.
    pub async fn append(
        &mut self,
        name: &str,
        bytes: &[u8],
    ) -> SdResult<(), SPI::Error, CS::Error> {
        let name = to_name(name)?;
        let entry = self.find_file_mut(&name).ok_or(SdError::MissingFile)?;
        entry.data.extend_from_slice(bytes).map_err(|_| SdError::FileTooLarge)?;
        Ok(())
    }

    /// Replace the first occurrence of `needle` with `replacement`.
    pub async fn replace_first(
        &mut self,
        name: &str,
        needle: &[u8],
        replacement: &[u8],
    ) -> SdResult<(), SPI::Error, CS::Error> {
        let name = to_name(name)?;
        let entry = self.find_file_mut(&name).ok_or(SdError::MissingFile)?;
        let Some(idx) = find_subslice(&entry.data, needle) else {
            return Err(SdError::EditTargetMissing);
        };

        let mut new_buf = FileBuf::new();
        new_buf.extend_from_slice(&entry.data[..idx]).map_err(|_| SdError::FileTooLarge)?;
        new_buf.extend_from_slice(replacement).map_err(|_| SdError::FileTooLarge)?;
        new_buf.extend_from_slice(&entry.data[idx + needle.len()..]).map_err(|_| SdError::FileTooLarge)?;
        entry.data = new_buf;
        Ok(())
    }

    /// Read the full contents of a file.
    pub async fn read_all(&self, name: &str) -> SdResult<FileBuf, SPI::Error, CS::Error> {
        let name = to_name(name)?;
        let entry = self.find_file(&name).ok_or(SdError::MissingFile)?;
        let mut out = FileBuf::new();
        out.extend_from_slice(&entry.data).map_err(|_| SdError::FileTooLarge)?;
        Ok(out)
    }

    fn find_file(&self, name: &FileName) -> Option<&FileEntry> {
        self.files.iter().find(|f| &f.name == name)
    }

    fn find_file_mut(&mut self, name: &FileName) -> Option<&mut FileEntry> {
        self.files.iter_mut().find(|f| &f.name == name)
    }
}

/// Result payload for `run_smoke_test`.
#[derive(Debug, defmt::Format)]
pub struct SmokeTestResult {
    pub file_name: FileName,
    pub contents: FileBuf,
}

fn to_name<SpiE, PinE>(raw: &str) -> SdResult<FileName, SpiE, PinE>
where
    SpiE: defmt::Format,
    PinE: defmt::Format,
{
    String::from_str(raw).map_err(|_| SdError::NameTooLong)
}

fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    if needle.is_empty() || needle.len() > haystack.len() {
        return None;
    }
    for i in 0..=haystack.len() - needle.len() {
        if haystack[i..i + needle.len()] == *needle {
            return Some(i);
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use embedded_hal::digital::ErrorType as DigitalErrorType;
    use embedded_hal_async::spi::ErrorType as SpiErrorType;
    use futures::executor::block_on;

    #[derive(Clone, Copy, Default)]
    struct NoopSpi;

    impl SpiErrorType for NoopSpi {
        type Error = Infallible;
    }

    impl SpiBus<u8> for NoopSpi {
        async fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            for w in words.iter_mut() {
                *w = 0xFF;
            }
            Ok(())
        }

        async fn write(&mut self, _words: &[u8]) -> Result<(), Self::Error> {
            Ok(())
        }

        async fn transfer(&mut self, read: &mut [u8], _write: &[u8]) -> Result<(), Self::Error> {
            self.read(read).await
        }

        async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
            self.read(words).await
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[derive(Clone, Copy, Default)]
    struct NoopPin;

    impl DigitalErrorType for NoopPin {
        type Error = Infallible;
    }

    impl OutputPin for NoopPin {
        fn set_low(&mut self) -> Result<(), Self::Error> { Ok(()) }
        fn set_high(&mut self) -> Result<(), Self::Error> { Ok(()) }
    }

    #[test]
    fn smoke_test_flow_edits_and_reads() {
        let spi = NoopSpi::default();
        let cs = NoopPin::default();
        let mut sd = SdBlackBox::new(spi, cs);

        let result = block_on(sd.run_smoke_test()).expect("smoke test should succeed");
        assert_eq!(result.file_name.as_str(), "blackbox.txt");
        assert_eq!(result.contents.as_slice(), b"boot=pico\nmode=bringup\n");
    }

    #[test]
    fn rejects_overlong_names() {
        let spi = NoopSpi::default();
        let cs = NoopPin::default();
        let mut sd = SdBlackBox::new(spi, cs);
        let long_name = "this_is_a_super_long_filename_that_will_fail";
        let err = block_on(sd.create_or_truncate(long_name, b"x")).unwrap_err();
        matches!(err, SdError::NameTooLong);
    }
}
