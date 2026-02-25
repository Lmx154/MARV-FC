//! SD CRUD facade.
//!
//! Until full filesystem support lands, CRUD operations are backed by a
//! bounded in-memory table while still touching the SPI abstraction for each
//! operation. This keeps call sites stable and hardware wiring exercised.

use core::str::FromStr;
use heapless::{String, Vec};

use super::spi::SdSpiDevice;

pub const MAX_FILE_NAME_LEN: usize = 24;
pub const MAX_FILE_BYTES: usize = 512;
pub const MAX_FILES: usize = 8;

pub type FileName = String<MAX_FILE_NAME_LEN>;
pub type FileData = Vec<u8, MAX_FILE_BYTES>;

#[derive(Clone, Debug, defmt::Format)]
struct FileEntry {
    name: FileName,
    data: FileData,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, defmt::Format)]
pub enum SdError {
    Spi,
    Cs,
    NameTooLong,
    TooManyFiles,
    FileTooLarge,
    MissingFile,
    FileAlreadyExists,
}

pub trait SdCrud {
    fn create(&mut self, name: &str, data: &[u8]) -> Result<(), SdError>;
    fn read(&mut self, name: &str) -> Result<FileData, SdError>;
    fn update(&mut self, name: &str, data: &[u8]) -> Result<(), SdError>;
    fn delete(&mut self, name: &str) -> Result<(), SdError>;
}

pub struct SdStore<DEV> {
    dev: DEV,
    files: Vec<FileEntry, MAX_FILES>,
}

impl<DEV> SdStore<DEV>
where
    DEV: SdSpiDevice,
{
    pub fn new(dev: DEV) -> Self {
        Self {
            dev,
            files: Vec::new(),
        }
    }

    /// SD wakeup clocks with CS released.
    pub fn init(&mut self) -> Result<(), SdError> {
        self.dev.release().map_err(|_| SdError::Cs)?;
        self.dev.write(&[0xFF; 10]).map_err(|_| SdError::Spi)?;
        Ok(())
    }

    pub fn device_mut(&mut self) -> &mut DEV {
        &mut self.dev
    }

    fn touch_bus(&mut self) -> Result<(), SdError> {
        self.dev.select().map_err(|_| SdError::Cs)?;
        let mut probe = [0xFF];
        let transfer_result = self.dev.transfer_in_place(&mut probe);
        let release_result = self.dev.release();

        if transfer_result.is_err() {
            return Err(SdError::Spi);
        }
        if release_result.is_err() {
            return Err(SdError::Cs);
        }
        Ok(())
    }

    fn find_file(&self, name: &FileName) -> Option<&FileEntry> {
        self.files.iter().find(|entry| &entry.name == name)
    }

    fn find_file_mut(&mut self, name: &FileName) -> Option<&mut FileEntry> {
        self.files.iter_mut().find(|entry| &entry.name == name)
    }
}

impl<DEV> SdCrud for SdStore<DEV>
where
    DEV: SdSpiDevice,
{
    fn create(&mut self, name: &str, data: &[u8]) -> Result<(), SdError> {
        self.touch_bus()?;
        let name = FileName::from_str(name).map_err(|_| SdError::NameTooLong)?;
        if data.len() > MAX_FILE_BYTES {
            return Err(SdError::FileTooLarge);
        }
        if self.find_file(&name).is_some() {
            return Err(SdError::FileAlreadyExists);
        }
        if self.files.len() == MAX_FILES {
            return Err(SdError::TooManyFiles);
        }

        let mut file_data = FileData::new();
        file_data
            .extend_from_slice(data)
            .map_err(|_| SdError::FileTooLarge)?;

        self.files
            .push(FileEntry {
                name,
                data: file_data,
            })
            .map_err(|_| SdError::TooManyFiles)?;

        Ok(())
    }

    fn read(&mut self, name: &str) -> Result<FileData, SdError> {
        self.touch_bus()?;
        let name = FileName::from_str(name).map_err(|_| SdError::NameTooLong)?;
        let entry = self.find_file(&name).ok_or(SdError::MissingFile)?;
        let mut out = FileData::new();
        out.extend_from_slice(&entry.data)
            .map_err(|_| SdError::FileTooLarge)?;
        Ok(out)
    }

    fn update(&mut self, name: &str, data: &[u8]) -> Result<(), SdError> {
        self.touch_bus()?;
        let name = FileName::from_str(name).map_err(|_| SdError::NameTooLong)?;
        if data.len() > MAX_FILE_BYTES {
            return Err(SdError::FileTooLarge);
        }

        let entry = self.find_file_mut(&name).ok_or(SdError::MissingFile)?;
        entry.data.clear();
        entry
            .data
            .extend_from_slice(data)
            .map_err(|_| SdError::FileTooLarge)?;
        Ok(())
    }

    fn delete(&mut self, name: &str) -> Result<(), SdError> {
        self.touch_bus()?;
        let name = FileName::from_str(name).map_err(|_| SdError::NameTooLong)?;
        let index = self
            .files
            .iter()
            .position(|entry| entry.name == name)
            .ok_or(SdError::MissingFile)?;
        self.files.swap_remove(index);
        Ok(())
    }
}
