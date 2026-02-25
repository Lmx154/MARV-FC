//! Shared SD filesystem policy for config + parameter persistence.
//!
//! This module is platform-agnostic at the HAL layer. It operates on an
//! `embedded_sdmmc::VolumeManager` provided by the board binary.

use embedded_sdmmc::{BlockDevice, Error as FsError, Mode as FsMode, TimeSource, VolumeIdx, VolumeManager};

use crate::config::Config as AppConfig;
use crate::params::ParamRegistry;

pub const CONFIG_FILE_NAME: &str = "CONFIG.TXT";
pub const PARAMS_FILE_NAME: &str = "PARAMS.TXT";

/// Load `CONFIG.TXT` (creating defaults if missing) and `PARAMS.TXT`.
pub fn load_config_and_params_from_volume_mgr<
    D,
    T,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    volume_mgr: &mut VolumeManager<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    params: &mut ParamRegistry,
) -> Result<(AppConfig, usize), FsError<D::Error>>
where
    D: BlockDevice,
    D::Error: core::fmt::Debug,
    T: TimeSource,
{
    let config = load_or_create_config(volume_mgr)?;
    let param_count = load_params_from_volume_mgr(volume_mgr, params)?;
    Ok((config, param_count))
}

/// Save the current parameter registry into `PARAMS.TXT`.
pub fn save_params_to_volume_mgr<
    D,
    T,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    volume_mgr: &mut VolumeManager<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    params: &ParamRegistry,
) -> Result<(), FsError<D::Error>>
where
    D: BlockDevice,
    D::Error: core::fmt::Debug,
    T: TimeSource,
{
    let mut volume0 = volume_mgr.open_volume(VolumeIdx(0))?;
    let mut root_dir = volume0.open_root_dir()?;

    let mut buf = [0u8; 2048];
    let len = params
        .serialize_to_text(&mut buf)
        .map_err(|_| FsError::FormatError("Failed to serialize parameter registry"))?;

    let mut file = root_dir.open_file_in_dir(PARAMS_FILE_NAME, FsMode::ReadWriteCreateOrTruncate)?;
    file.write(&buf[..len])?;
    Ok(())
}

fn load_or_create_config<
    D,
    T,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    volume_mgr: &mut VolumeManager<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
) -> Result<AppConfig, FsError<D::Error>>
where
    D: BlockDevice,
    D::Error: core::fmt::Debug,
    T: TimeSource,
{
    let mut volume0 = volume_mgr.open_volume(VolumeIdx(0))?;
    let mut root_dir = volume0.open_root_dir()?;

    if let Ok(mut file) = root_dir.open_file_in_dir(CONFIG_FILE_NAME, FsMode::ReadOnly) {
        let mut buf = [0u8; 256];
        let n = file.read(&mut buf)?;
        return Ok(AppConfig::from_bytes(&buf[..n]));
    }

    let defaults = AppConfig::default();
    let mut file = root_dir.open_file_in_dir(CONFIG_FILE_NAME, FsMode::ReadWriteCreateOrTruncate)?;
    let bytes = defaults.to_bytes();
    file.write(bytes.as_bytes())?;
    Ok(defaults)
}

fn load_params_from_volume_mgr<
    D,
    T,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    volume_mgr: &mut VolumeManager<D, T, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    params: &mut ParamRegistry,
) -> Result<usize, FsError<D::Error>>
where
    D: BlockDevice,
    D::Error: core::fmt::Debug,
    T: TimeSource,
{
    let mut volume0 = volume_mgr.open_volume(VolumeIdx(0))?;
    let mut root_dir = volume0.open_root_dir()?;

    let res = root_dir.open_file_in_dir(PARAMS_FILE_NAME, FsMode::ReadOnly);
    if let Ok(mut file) = res {
        let mut buf = [0u8; 2048];
        let n = file.read(&mut buf)?;
        if let Ok(count) = params.deserialize_from_text(&buf[..n]) {
            return Ok(count);
        }
    }

    Ok(0)
}
