# Telemetry App Gazebo Bridge

This directory contains a small C++ host-side bridge that sits between the Rust desktop app and Gazebo.

## What it does

- Listens on `localhost` for the Rust backend.
- Streams sensor frames to the backend stamped with Gazebo sim time.
- Accepts actuator commands from the backend.
- Publishes actuator commands into Gazebo Transport on `/marv_f450/command/motor_speed`.

## Build From The MARV-FC Repo Root

```bash
cmake -S telemetry-app/gazebo_bridge -B telemetry-app/gazebo_bridge/build
cmake --build telemetry-app/gazebo_bridge/build
```

## Run From The MARV-FC Repo Root

```bash
./telemetry-app/gazebo_bridge/build/cerberus_gazebo_bridge
```

The checked-in `config/bridge_config` is loaded automatically by the Tauri app launcher and matches the MARV `marv_field` world:

- clock: `/world/marv_field/clock`
- IMU: `/world/marv_field/model/marv_f450/link/base_link/sensor/imu_sensor/imu`
- GPS/NavSat: `/world/marv_field/model/marv_f450/link/base_link/sensor/navsat_sensor/navsat`
- barometer: `/world/marv_field/model/marv_f450/link/base_link/sensor/air_pressure_sensor/air_pressure`
- magnetometer: `/world/marv_field/model/marv_f450/link/base_link/sensor/magnetometer_sensor/magnetometer`
- actuator command: `/marv_f450/command/motor_speed`
- world control service: `/world/marv_field/control`

The checked-in config uses `actuators.motor_direction_mode=gazebo_model` with
positive bridge motor directions. That means the bridge scales normalized motor
commands to rotor speed magnitudes and delegates spin-direction/yaw sign to the
Gazebo model's motor configuration.

If your shell is inside `telemetry-app/`, use:

```bash
cmake -S gazebo_bridge -B gazebo_bridge/build
cmake --build gazebo_bridge/build
./gazebo_bridge/build/cerberus_gazebo_bridge
```

Optional arguments:

- `--port <port>`: TCP port for the Rust backend connection.
- `--tick-ms <ms>`: Bridge update interval in milliseconds.
- `--clock-topic <topic>`: Gazebo clock topic to use for sim time, default `/clock`.

## Wire protocol

The bridge uses simple newline-delimited text frames for now:

- `SENSOR ...` for bridge-to-backend sensor updates, including `sim_time_us` and `clock`.
- `ACTUATOR ...` for backend-to-bridge actuator commands, including `sim_time_us` when available.
- `SIM_CONTROL seq=<n> action=reset|pause|play` for backend-to-bridge world control.
- `SIM_CONTROL_ACK seq=<n> action=... ok=0|1 message=...` for bridge-to-backend world-control results.

This keeps the first implementation easy to debug before moving to a binary packet format.

The bridge publishes actuator commands to Gazebo on:

- `/marv_f450/command/motor_speed` as `gz.msgs.Actuators`

The bridge sends reset/pause/play requests to Gazebo on:

- `/world/marv_field/control` as `gz.msgs.WorldControl`
