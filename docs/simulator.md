# Simulator (SITL) Backend + UI

This document explains how to run and use the `simulator` crate backend and the built-in web UI.

## What it provides

- HTTP API for simulation state and filesystem browsing
- WebSocket endpoint for simulation control + live events
- Built-in static UI served directly by the backend

No `npm` or frontend build step is required.

## Run

From repo root:

```bash
cargo run -p simulator
```

Default bind address is `127.0.0.1:9000`.

Open:

```text
http://127.0.0.1:9000/
```

## Environment variables

- `SIM_BIND`
  - HTTP/WebSocket bind address
  - Example: `SIM_BIND=0.0.0.0:8080`
- `SIM_ALLOWED_ROOTS`
  - Allowed filesystem roots exposed by `/api/fs/*`
  - If unset, defaults to current working directory
  - Uses OS path list separator (`:` on Linux/macOS, `;` on Windows)

Example:

```bash
SIM_BIND=0.0.0.0:8080 \
SIM_ALLOWED_ROOTS="/home/luis/data:/tmp" \
cargo run -p simulator
```

## UI overview

The UI is served at `/` and uses the same backend origin.

- Connect/disconnect WebSocket
- View live sim snapshot (`tick`, `sim_time_s`, `running`, `rate_hz`, selected data file)
- Pause/resume, set sim rate, and single-step
- Browse allowed roots, preview files, and select a data file for simulation

## HTTP API

Base URL: `http://<host>:<port>`

- `GET /health`
- `GET /api/sim/state`
- `POST /api/sim/select_data_file`
- `GET /api/fs/roots`
- `GET /api/fs/list?path=<path>&include_hidden=<bool>`
- `GET /api/fs/stat?path=<path>`
- `GET /api/fs/read?path=<path>&max_bytes=<n>`

### Example: select data file

```bash
curl -X POST http://127.0.0.1:9000/api/sim/select_data_file \
  -H "content-type: application/json" \
  -d '{"path":"/home/luis/data/flight1.csv"}'
```

## WebSocket API

Endpoint:

```text
ws://<host>:<port>/ws
```

Client sends JSON text messages:

- `{"type":"pause"}`
- `{"type":"resume"}`
- `{"type":"set_rate_hz","hz":120.0}`
- `{"type":"step","dt_ms":20}`
- `{"type":"select_data_file","path":"/abs/path/file.csv"}`
- `{"type":"ping","id":"abc"}`

Server sends JSON events with `type`:

- `hello`
- `snapshot`
- `ack`
- `error`

## Notes

- File access is constrained to `SIM_ALLOWED_ROOTS`.
- Relative paths are resolved from the first configured root.
- `cargo check -p simulator --offline` works if dependencies are already cached.
