use std::{sync::Arc, time::Duration};

use crate::{
    state::{AppState, MAX_SIM_RATE_HZ, MIN_SIM_RATE_HZ},
    types::ServerEvent,
};

pub async fn simulation_loop(state: Arc<AppState>) {
    loop {
        let (running, rate_hz) = {
            let sim = state.sim.read().await;
            (sim.running(), sim.rate_hz())
        };

        if !running {
            tokio::time::sleep(Duration::from_millis(50)).await;
            continue;
        }

        let dt_s = 1.0 / rate_hz.clamp(MIN_SIM_RATE_HZ, MAX_SIM_RATE_HZ);
        let snapshot = {
            let mut sim = state.sim.write().await;
            sim.step(dt_s);
            sim.snapshot()
        };
        let _ = state.events_tx.send(ServerEvent::Snapshot { snapshot });
        tokio::time::sleep(Duration::from_secs_f64(dt_s)).await;
    }
}
