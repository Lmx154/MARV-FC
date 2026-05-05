# Cerberus

Cerberus is a native Rust desktop app built with `eframe/egui` using a 3-layer architecture:

- Backend: system and domain logic.
- Bridge: maps backend data into UI state.
- Frontend: tabbed `egui` rendering.

## Current UI State

- Top-level tab system:
  - Settings
  - Debug
  - Mission
  - HIL
  - Data
- Landing tab on startup: **Mission**.
- Settings currently controls the global theme (Dark/Light) and a backend-wired UART port UI:
  - Left-side button opens or closes the selected UART port.
  - Right-side dropdown refreshes available UART ports when opened and selects a port while closed.
  - The dropdown is disabled while the port state is open.
- Debug currently shows a live local time feed from app state.
- Mission now includes a telemetry workspace:
  - Top-centered mission display area.
  - Left and right telemetry selectors (3 dropdowns per side).
  - Center frame with switchable content:
    - Long telemetry list (scrollable inside the frame).
    - 3D visual placeholder.
  - Collapsible telemetry fields panel (hidden/collapsed by default).
  - Telemetry fields grouped by Sensors and System, each with parameter/value/unit columns.
  - Values are currently frontend placeholders (`--`) pending live bridge wiring.
- HIL and Data remain placeholder panes.
- App-wide content overflow scrolling is enabled in the shared frontend container.

## Project Structure

- `src/main.rs`
  - Native entrypoint (`eframe::run_native`).

- `src/app.rs`
  - App lifecycle root.
  - Runs bridge updates and frontend render each frame.

- `src/backend/time_service.rs`
  - Time service (`chrono::Local`) in 12-hour AM/PM format.

- `src/backend/uart_service.rs`
  - UART service for opening, closing, and listing serial ports.

- `src/bridge/app_state.rs`
  - Shared UI state model.

- `src/bridge/app_bridge.rs`
  - Periodically refreshes backend outputs into `AppState`.

- `src/frontend/frontend_view.rs`
  - Top-level frontend container.
  - Owns active tab and global theme state.
  - Applies shared scroll behavior for tab content overflow.
  - Routes rendering to per-tab modules.

- `src/frontend/tabs/settings_tab.rs`
  - Settings tab UI, global theme controls, and UART open/close selector state.

- `src/frontend/tabs/debug_tab.rs`
  - Debug tab UI (currently shows live time feed).

- `src/frontend/tabs/mission_tab.rs`
  - Mission telemetry workspace UI.
  - Left/right selector columns for telemetry field selection.
  - Switchable center frame (long telemetry list or 3D placeholder).
  - Collapsible telemetry fields table grouped by Sensors/System.

- `src/frontend/tabs/hil_tab.rs`
  - HIL tab placeholder.

- `src/frontend/tabs/data_tab.rs`
  - Data tab placeholder.

## Run From The MARV-FC Repo Root

From `/home/luis/Projects/Rust/MARV-FC`:

```bash
cargo run -p cerberus
```

## Build The Rust UI From The Repo Root

```bash
cargo build -p cerberus --release
```

On Linux, the executable will be generated at:

```bash
target/release/cerberus
```

From the repo root, run:
```bash
./target/release/cerberus
```

If your shell is inside `cerberus/`, use:

```bash
cargo run
cargo build --release
../target/release/cerberus
```

## Build The Gazebo Bridge From The Repo Root

```bash
cmake -S cerberus/gazebo_bridge -B cerberus/gazebo_bridge/build
cmake --build cerberus/gazebo_bridge/build
```

From the repo root, run:

```bash
./cerberus/gazebo_bridge/build/cerberus_gazebo_bridge
```

If your shell is inside `cerberus/`, use:

```bash
cmake -S gazebo_bridge -B gazebo_bridge/build
cmake --build gazebo_bridge/build
./gazebo_bridge/build/cerberus_gazebo_bridge
```

## Notes

- `mod.rs` files are used only for module wiring and re-exports.
- Data flow is explicit: backend -> bridge -> frontend.
