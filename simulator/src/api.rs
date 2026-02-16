use std::{cmp::Ordering, fs, io::Read, sync::Arc};

use poem::{
    Error, Result, Route, get, handler,
    http::StatusCode,
    post,
    web::{Data, Json, Query},
};

use crate::{
    fs_access::{fs_entry, resolve_existing_path},
    state::AppState,
    types::{
        FsListQuery, FsListResponse, FsReadQuery, FsReadResponse, FsStatQuery, FsStatResponse,
        HealthResponse, RootsResponse, SelectDataFileRequest, SimSnapshot,
    },
};

const DEFAULT_FILE_PREVIEW_BYTES: usize = 64 * 1024;
const MAX_FILE_PREVIEW_BYTES: usize = 2 * 1024 * 1024;

pub fn routes() -> Route {
    Route::new().at("/health", get(health)).nest(
        "/api",
        Route::new()
            .nest(
                "/fs",
                Route::new()
                    .at("/roots", get(fs_roots_handler))
                    .at("/list", get(fs_list_handler))
                    .at("/stat", get(fs_stat_handler))
                    .at("/read", get(fs_read_handler)),
            )
            .nest(
                "/sim",
                Route::new()
                    .at("/state", get(sim_state_handler))
                    .at("/select_data_file", post(select_data_file_handler)),
            ),
    )
}

#[handler]
async fn health() -> Json<HealthResponse> {
    Json(HealthResponse { status: "ok" })
}

#[handler]
async fn fs_roots_handler(state: Data<&Arc<AppState>>) -> Json<RootsResponse> {
    Json(RootsResponse {
        roots: state
            .0
            .fs_roots
            .iter()
            .map(|root| root.display().to_string())
            .collect(),
    })
}

#[handler]
async fn fs_list_handler(
    state: Data<&Arc<AppState>>,
    query: Query<FsListQuery>,
) -> Result<Json<FsListResponse>> {
    let include_hidden = query.include_hidden.unwrap_or(false);
    let dir = resolve_existing_path(&query.path, &state.0.fs_roots)
        .map_err(|msg| api_error(StatusCode::BAD_REQUEST, msg))?;

    if !dir.is_dir() {
        return Err(api_error(
            StatusCode::BAD_REQUEST,
            "path is not a directory",
        ));
    }

    let mut entries = Vec::new();
    let iter = fs::read_dir(&dir)
        .map_err(|err| api_error(StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;

    for entry_res in iter {
        let entry = entry_res
            .map_err(|err| api_error(StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
        let entry_path = entry.path();
        let name = entry.file_name().to_string_lossy().to_string();

        if !include_hidden && name.starts_with('.') {
            continue;
        }

        entries.push(fs_entry(&name, &entry_path));
    }

    entries.sort_by(|a, b| match (a.is_dir, b.is_dir) {
        (true, false) => Ordering::Less,
        (false, true) => Ordering::Greater,
        _ => a.name.cmp(&b.name),
    });

    Ok(Json(FsListResponse {
        path: dir.display().to_string(),
        entries,
    }))
}

#[handler]
async fn fs_stat_handler(
    state: Data<&Arc<AppState>>,
    query: Query<FsStatQuery>,
) -> Result<Json<FsStatResponse>> {
    let path = resolve_existing_path(&query.path, &state.0.fs_roots)
        .map_err(|msg| api_error(StatusCode::BAD_REQUEST, msg))?;
    let name = path
        .file_name()
        .map(|name| name.to_string_lossy().to_string())
        .unwrap_or_else(|| path.display().to_string());

    Ok(Json(FsStatResponse {
        entry: fs_entry(&name, &path),
    }))
}

#[handler]
async fn fs_read_handler(
    state: Data<&Arc<AppState>>,
    query: Query<FsReadQuery>,
) -> Result<Json<FsReadResponse>> {
    let path = resolve_existing_path(&query.path, &state.0.fs_roots)
        .map_err(|msg| api_error(StatusCode::BAD_REQUEST, msg))?;

    if !path.is_file() {
        return Err(api_error(StatusCode::BAD_REQUEST, "path is not a file"));
    }

    let max_bytes = query
        .max_bytes
        .unwrap_or(DEFAULT_FILE_PREVIEW_BYTES)
        .clamp(1, MAX_FILE_PREVIEW_BYTES);

    let mut file = fs::File::open(&path)
        .map_err(|err| api_error(StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;
    let mut limited = (&mut file).take((max_bytes + 1) as u64);
    let mut buffer = Vec::with_capacity(max_bytes.saturating_add(1));
    limited
        .read_to_end(&mut buffer)
        .map_err(|err| api_error(StatusCode::INTERNAL_SERVER_ERROR, err.to_string()))?;

    let truncated = buffer.len() > max_bytes;
    if truncated {
        buffer.truncate(max_bytes);
    }

    let bytes_returned = buffer.len();
    let content = String::from_utf8_lossy(&buffer).into_owned();

    Ok(Json(FsReadResponse {
        path: path.display().to_string(),
        bytes_returned,
        truncated,
        content,
    }))
}

#[handler]
async fn sim_state_handler(state: Data<&Arc<AppState>>) -> Json<SimSnapshot> {
    let sim = state.0.sim.read().await;
    Json(sim.snapshot())
}

#[handler]
async fn select_data_file_handler(
    state: Data<&Arc<AppState>>,
    payload: Json<SelectDataFileRequest>,
) -> Result<Json<SimSnapshot>> {
    let file_path = resolve_existing_path(&payload.path, &state.0.fs_roots)
        .map_err(|msg| api_error(StatusCode::BAD_REQUEST, msg))?;

    if !file_path.is_file() {
        return Err(api_error(StatusCode::BAD_REQUEST, "path is not a file"));
    }

    let snapshot = {
        let mut sim = state.0.sim.write().await;
        sim.set_selected_data_file(file_path);
        sim.snapshot()
    };

    let _ = state.0.events_tx.send(crate::types::ServerEvent::Snapshot {
        snapshot: snapshot.clone(),
    });

    Ok(Json(snapshot))
}

fn api_error(status: StatusCode, message: impl Into<String>) -> Error {
    Error::from_string(message.into(), status)
}
