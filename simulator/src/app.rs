use std::{env, sync::Arc};

use poem::{EndpointExt, Server, get, listener::TcpListener};
use tokio::sync::{RwLock, broadcast};

use crate::{api, fs_access::load_fs_roots, sim_loop::simulation_loop, state::AppState, ui, ws};

const DEFAULT_BIND_ADDR: &str = "127.0.0.1:9000";

pub async fn run() -> Result<(), std::io::Error> {
    let bind_addr = env::var("SIM_BIND").unwrap_or_else(|_| DEFAULT_BIND_ADDR.to_string());
    let fs_roots = load_fs_roots()?;

    let (events_tx, _) = broadcast::channel(256);
    let app_state = Arc::new(AppState {
        sim: RwLock::new(Default::default()),
        events_tx,
        fs_roots,
    });

    tokio::spawn(simulation_loop(app_state.clone()));

    let app = api::routes()
        .at("/", get(ui::index_handler))
        .at("/ui/app.js", get(ui::app_js_handler))
        .at("/ui/styles.css", get(ui::styles_css_handler))
        .at("/ws", get(ws::ws_handler))
        .data(app_state.clone());

    println!("Simulator backend listening on http://{bind_addr}");
    println!("WebSocket endpoint: ws://{bind_addr}/ws");
    println!("Filesystem roots:");
    for root in &app_state.fs_roots {
        println!("  - {}", root.display());
    }

    Server::new(TcpListener::bind(bind_addr)).run(app).await
}
