use std::path::PathBuf;

use tokio::sync::{RwLock, broadcast};

use crate::types::{ServerEvent, SimSnapshot};

pub const MIN_SIM_RATE_HZ: f64 = 0.2;
pub const MAX_SIM_RATE_HZ: f64 = 500.0;

#[derive(Debug)]
pub struct AppState {
    pub sim: RwLock<SimulationState>,
    pub events_tx: broadcast::Sender<ServerEvent>,
    pub fs_roots: Vec<PathBuf>,
}

#[derive(Debug, Clone)]
pub struct SimulationState {
    tick: u64,
    sim_time_s: f64,
    running: bool,
    rate_hz: f64,
    selected_data_file: Option<PathBuf>,
}

impl Default for SimulationState {
    fn default() -> Self {
        Self {
            tick: 0,
            sim_time_s: 0.0,
            running: true,
            rate_hz: 60.0,
            selected_data_file: None,
        }
    }
}

impl SimulationState {
    pub fn step(&mut self, dt_s: f64) {
        self.tick = self.tick.saturating_add(1);
        self.sim_time_s += dt_s;
    }

    pub fn snapshot(&self) -> SimSnapshot {
        SimSnapshot {
            tick: self.tick,
            sim_time_s: self.sim_time_s,
            running: self.running,
            rate_hz: self.rate_hz,
            selected_data_file: self
                .selected_data_file
                .as_ref()
                .map(|path| path.display().to_string()),
        }
    }

    pub fn set_running(&mut self, running: bool) {
        self.running = running;
    }

    pub fn running(&self) -> bool {
        self.running
    }

    pub fn set_rate_hz(&mut self, rate_hz: f64) {
        self.rate_hz = rate_hz;
    }

    pub fn rate_hz(&self) -> f64 {
        self.rate_hz
    }

    pub fn set_selected_data_file(&mut self, path: PathBuf) {
        self.selected_data_file = Some(path);
    }
}
