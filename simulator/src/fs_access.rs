use std::{
    env, fs,
    path::{Path, PathBuf},
    time::UNIX_EPOCH,
};

use crate::types::FsEntry;

pub fn load_fs_roots() -> Result<Vec<PathBuf>, std::io::Error> {
    let roots = match env::var_os("SIM_ALLOWED_ROOTS") {
        Some(raw) => env::split_paths(&raw).collect::<Vec<_>>(),
        None => vec![env::current_dir()?],
    };

    let mut normalized = Vec::new();
    for root in roots {
        let root = if root.is_absolute() {
            root
        } else {
            env::current_dir()?.join(root)
        };

        if let Ok(canonical) = root.canonicalize() {
            normalized.push(canonical);
        }
    }

    if normalized.is_empty() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "SIM_ALLOWED_ROOTS did not resolve to any valid directories",
        ));
    }

    Ok(normalized)
}

pub fn resolve_existing_path(input_path: &str, roots: &[PathBuf]) -> Result<PathBuf, String> {
    if input_path.trim().is_empty() {
        return Err("path cannot be empty".to_string());
    }

    let requested = PathBuf::from(input_path);
    let candidate = if requested.is_absolute() {
        requested
    } else {
        roots[0].join(requested)
    };

    let canonical = candidate
        .canonicalize()
        .map_err(|err| format!("unable to resolve path: {err}"))?;

    if roots.iter().any(|root| canonical.starts_with(root)) {
        return Ok(canonical);
    }

    Err("path is outside allowed simulator roots".to_string())
}

pub fn fs_entry(name: &str, path: &Path) -> FsEntry {
    let metadata = fs::metadata(path).ok();
    let is_dir = metadata.as_ref().is_some_and(|meta| meta.is_dir());
    let size_bytes = metadata.as_ref().and_then(|meta| {
        if meta.is_file() {
            Some(meta.len())
        } else {
            None
        }
    });
    let modified_unix_ms = metadata
        .and_then(|meta| meta.modified().ok())
        .and_then(|mtime| mtime.duration_since(UNIX_EPOCH).ok())
        .map(|duration| duration.as_millis());

    FsEntry {
        name: name.to_string(),
        path: path.display().to_string(),
        is_dir,
        size_bytes,
        modified_unix_ms,
    }
}
