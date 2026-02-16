use std::{sync::Arc, time::Duration};

use futures_util::{SinkExt, StreamExt};
use poem::{
    IntoResponse, handler,
    web::{
        Data,
        websocket::{Message, WebSocket},
    },
};

use crate::{
    fs_access::resolve_existing_path,
    state::{AppState, MAX_SIM_RATE_HZ, MIN_SIM_RATE_HZ},
    types::{ClientCommand, ServerEvent},
};

#[handler]
pub async fn ws_handler(ws: WebSocket, state: Data<&Arc<AppState>>) -> impl IntoResponse {
    let state = state.0.clone();
    ws.on_upgrade(move |socket| async move {
        let _ = handle_ws(socket, state).await;
    })
}

async fn handle_ws(
    socket: poem::web::websocket::WebSocketStream,
    state: Arc<AppState>,
) -> Result<(), ()> {
    let (mut sink, mut stream) = socket.split();
    let mut rx = state.events_tx.subscribe();

    let hello = ServerEvent::Hello {
        message: "MARV simulator websocket ready".to_string(),
    };
    if send_ws_event(&mut sink, &hello).await.is_err() {
        return Ok(());
    }

    let snapshot = {
        let sim = state.sim.read().await;
        sim.snapshot()
    };
    if send_ws_event(&mut sink, &ServerEvent::Snapshot { snapshot })
        .await
        .is_err()
    {
        return Ok(());
    }

    loop {
        tokio::select! {
            maybe_msg = stream.next() => {
                let Some(msg_res) = maybe_msg else {
                    break;
                };

                let Ok(msg) = msg_res else {
                    break;
                };

                match msg {
                    Message::Text(text) => {
                        let response = handle_ws_command(&state, &text).await;
                        if send_ws_event(&mut sink, &response).await.is_err() {
                            break;
                        }
                    }
                    Message::Ping(payload) => {
                        if sink.send(Message::Pong(payload)).await.is_err() {
                            break;
                        }
                    }
                    Message::Close(_) => {
                        break;
                    }
                    _ => {}
                }
            }
            evt = rx.recv() => {
                let event = match evt {
                    Ok(event) => event,
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(skipped)) => {
                        ServerEvent::Error {
                            message: format!("ws lagged and skipped {skipped} events"),
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
                };

                if send_ws_event(&mut sink, &event).await.is_err() {
                    break;
                }
            }
        }
    }

    Ok(())
}

async fn send_ws_event<S>(sink: &mut S, event: &ServerEvent) -> Result<(), ()>
where
    S: futures_util::Sink<Message> + Unpin,
{
    let Ok(payload) = serde_json::to_string(event) else {
        return Err(());
    };
    sink.send(Message::Text(payload)).await.map_err(|_| ())
}

async fn handle_ws_command(state: &Arc<AppState>, raw: &str) -> ServerEvent {
    let cmd = match serde_json::from_str::<ClientCommand>(raw) {
        Ok(cmd) => cmd,
        Err(err) => {
            return ServerEvent::Error {
                message: format!("invalid command payload: {err}"),
            };
        }
    };

    match cmd {
        ClientCommand::Pause => {
            let snapshot = {
                let mut sim = state.sim.write().await;
                sim.set_running(false);
                sim.snapshot()
            };
            let _ = state.events_tx.send(ServerEvent::Snapshot {
                snapshot: snapshot.clone(),
            });
            ServerEvent::Ack {
                command: "pause".to_string(),
                ok: true,
                message: "simulation paused".to_string(),
            }
        }
        ClientCommand::Resume => {
            let snapshot = {
                let mut sim = state.sim.write().await;
                sim.set_running(true);
                sim.snapshot()
            };
            let _ = state.events_tx.send(ServerEvent::Snapshot {
                snapshot: snapshot.clone(),
            });
            ServerEvent::Ack {
                command: "resume".to_string(),
                ok: true,
                message: "simulation running".to_string(),
            }
        }
        ClientCommand::SetRateHz { hz } => {
            if !(MIN_SIM_RATE_HZ..=MAX_SIM_RATE_HZ).contains(&hz) {
                return ServerEvent::Ack {
                    command: "set_rate_hz".to_string(),
                    ok: false,
                    message: format!("hz must be between {MIN_SIM_RATE_HZ} and {MAX_SIM_RATE_HZ}"),
                };
            }

            let snapshot = {
                let mut sim = state.sim.write().await;
                sim.set_rate_hz(hz);
                sim.snapshot()
            };
            let _ = state.events_tx.send(ServerEvent::Snapshot {
                snapshot: snapshot.clone(),
            });
            ServerEvent::Ack {
                command: "set_rate_hz".to_string(),
                ok: true,
                message: format!("rate set to {hz:.3} Hz"),
            }
        }
        ClientCommand::Step { dt_ms } => {
            let dt_s = dt_ms
                .map(|ms| Duration::from_millis(ms).as_secs_f64())
                .unwrap_or(0.01);

            let snapshot = {
                let mut sim = state.sim.write().await;
                sim.step(dt_s);
                sim.snapshot()
            };
            let _ = state.events_tx.send(ServerEvent::Snapshot {
                snapshot: snapshot.clone(),
            });
            ServerEvent::Ack {
                command: "step".to_string(),
                ok: true,
                message: format!("advanced simulation by {dt_s:.6} s"),
            }
        }
        ClientCommand::SelectDataFile { path } => {
            match resolve_existing_path(&path, &state.fs_roots) {
                Ok(file_path) => {
                    if !file_path.is_file() {
                        return ServerEvent::Ack {
                            command: "select_data_file".to_string(),
                            ok: false,
                            message: "path is not a file".to_string(),
                        };
                    }

                    let snapshot = {
                        let mut sim = state.sim.write().await;
                        sim.set_selected_data_file(file_path.clone());
                        sim.snapshot()
                    };
                    let _ = state.events_tx.send(ServerEvent::Snapshot {
                        snapshot: snapshot.clone(),
                    });
                    ServerEvent::Ack {
                        command: "select_data_file".to_string(),
                        ok: true,
                        message: format!("selected {}", file_path.display()),
                    }
                }
                Err(err) => ServerEvent::Ack {
                    command: "select_data_file".to_string(),
                    ok: false,
                    message: err,
                },
            }
        }
        ClientCommand::Ping { id } => ServerEvent::Ack {
            command: "ping".to_string(),
            ok: true,
            message: id.unwrap_or_else(|| "pong".to_string()),
        },
    }
}
