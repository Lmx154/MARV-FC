# MARV (Modular Avionics for Rockets and Vehicles) Flight Controller

MARV is an avionics system aimed at growing my understanding of avionics system and is my exploration into the lowest level components of avionics. This repository is the firmware designed to be flashed into the MARV flight controller hardware. 


- sensors available
- devices available
- tasks
- simulation
- Telemetry visualization UI: `telemetry-app/`

## Docs

- Simulator: `docs/simulator.md`
- Telemetry app UI: `telemetry-app/README.md`
- Protocol: `docs/protocol.md`
- Localization: `docs/localization.md`
- PCB hardware: `docs/pcbhardware.md`

## Telemetry App UI

The desktop visualization software now lives at `telemetry-app/` and uses Tauri.
Run this command from the MARV-FC repository root:

```bash
pnpm --dir telemetry-app dev
```

Build the Gazebo bridge from the repo root:

```bash
cmake -S telemetry-app/gazebo_bridge -B telemetry-app/gazebo_bridge/build
cmake --build telemetry-app/gazebo_bridge/build
```

The Tauri UI can launch the built bridge from the Dashboard Gazebo panel.

