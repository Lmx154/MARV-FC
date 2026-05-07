# Cerberus Telemetry App

Tauri desktop UI for the MARV telemetry backend.

## Development

From this directory:

```sh
pnpm dev
```

This launches the desktop app through Tauri. Tauri starts the Vite frontend through `pnpm web:dev`.

For the browser-only frontend server:

```sh
pnpm web:dev
```

For a production frontend build:

```sh
pnpm build
```

## Gazebo Bridge

The C++ Gazebo Transport bridge lives in `gazebo_bridge/`.

From the repository root:

```sh
cmake -S telemetry-app/gazebo_bridge -B telemetry-app/gazebo_bridge/build
cmake --build telemetry-app/gazebo_bridge/build
```

The Tauri UI can launch `telemetry-app/gazebo_bridge/build/cerberus_gazebo_bridge` from the Dashboard Gazebo panel.
