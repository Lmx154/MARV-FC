use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize)]
pub struct SimSnapshot {
    pub tick: u64,
    pub sim_time_s: f64,
    pub running: bool,
    pub rate_hz: f64,
    pub selected_data_file: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ServerEvent {
    Hello {
        message: String,
    },
    Snapshot {
        snapshot: SimSnapshot,
    },
    Ack {
        command: String,
        ok: bool,
        message: String,
    },
    Error {
        message: String,
    },
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum ClientCommand {
    Pause,
    Resume,
    SetRateHz { hz: f64 },
    Step { dt_ms: Option<u64> },
    SelectDataFile { path: String },
    Ping { id: Option<String> },
}

#[derive(Debug, Serialize)]
pub struct HealthResponse {
    pub status: &'static str,
}

#[derive(Debug, Serialize)]
pub struct RootsResponse {
    pub roots: Vec<String>,
}

#[derive(Debug, Deserialize)]
pub struct FsListQuery {
    pub path: String,
    pub include_hidden: Option<bool>,
}

#[derive(Debug, Deserialize)]
pub struct FsStatQuery {
    pub path: String,
}

#[derive(Debug, Deserialize)]
pub struct FsReadQuery {
    pub path: String,
    pub max_bytes: Option<usize>,
}

#[derive(Debug, Deserialize)]
pub struct SelectDataFileRequest {
    pub path: String,
}

#[derive(Debug, Serialize)]
pub struct FsListResponse {
    pub path: String,
    pub entries: Vec<FsEntry>,
}

#[derive(Debug, Serialize)]
pub struct FsStatResponse {
    pub entry: FsEntry,
}

#[derive(Debug, Serialize)]
pub struct FsReadResponse {
    pub path: String,
    pub bytes_returned: usize,
    pub truncated: bool,
    pub content: String,
}

#[derive(Debug, Serialize)]
pub struct FsEntry {
    pub name: String,
    pub path: String,
    pub is_dir: bool,
    pub size_bytes: Option<u64>,
    pub modified_unix_ms: Option<u128>,
}
