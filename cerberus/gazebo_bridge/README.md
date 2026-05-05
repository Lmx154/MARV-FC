# Cerberus Gazebo Bridge

This directory contains a small C++ host-side bridge that sits between the Rust desktop app and Gazebo.

## What it does

- Listens on `localhost` for the Rust backend.
- Streams sensor frames to the backend stamped with Gazebo sim time.
- Accepts actuator commands from the backend.
- Publishes actuator commands into Gazebo Transport on `/X3/gazebo/command/motor_speed`.

## Build From The MARV-FC Repo Root

```bash
cmake -S cerberus/gazebo_bridge -B cerberus/gazebo_bridge/build
cmake --build cerberus/gazebo_bridge/build
```

## Run From The MARV-FC Repo Root

```bash
./cerberus/gazebo_bridge/build/cerberus_gazebo_bridge
```

If your shell is inside `cerberus/`, use:

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

This keeps the first implementation easy to debug before moving to a binary packet format.

The bridge publishes actuator commands to Gazebo on:

- `/X3/gazebo/command/motor_speed` as `gz.msgs.Actuators`
