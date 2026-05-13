# Gazebo Topic Map

This document records the Gazebo Transport topics produced by the MARV simulation environment.

Generated from:

```bash
./scripts/list_topics.sh
gz topic -i -t <topic>
```

## World

World name:

```text
marv_field
```

Model name:

```text
marv_f450
```

Primary vehicle link / sensor frame:

```text
base_link
```

World geographic origin:

```text
surface model: EARTH_WGS84
world frame orientation: ENU
latitude: 26.310942 deg
longitude: -98.174728 deg
elevation: 28.711437225 m
```

The geographic origin is configured in `worlds/marv_field.sdf`. Elevation was queried from the USGS Elevation Point Query Service and should be replaced with a surveyed launch-site value when available.

Reference map:

```text
https://www.google.com/maps?q=26.310942,-98.174728
```

## Clock

Topic:

```text
/clock
```

Message type:

```text
gz.msgs.Clock
```

Alternative world-scoped clock topic:

```text
/world/marv_field/clock
```

## Pose / Ground Truth

Topic:

```text
/world/marv_field/pose/info
```

Message type:

```text
gz.msgs.Pose_V
```

Dynamic pose topic:

```text
/world/marv_field/dynamic_pose/info
```

Dynamic pose message type:

```text
gz.msgs.Pose_V
```

Usage:

```text
Validation and debugging only. Do not treat ground truth pose as a real flight sensor.
```

## IMU

Topic:

```text
/world/marv_field/model/marv_f450/link/base_link/sensor/imu_sensor/imu
```

Message type:

```text
gz.msgs.IMU
```

Nominal update rate:

```text
250 Hz
```

## GPS / NavSat

Topic:

```text
/world/marv_field/model/marv_f450/link/base_link/sensor/navsat_sensor/navsat
```

Message type:

```text
gz.msgs.NavSat
```

Nominal update rate:

```text
30 Hz
```

## Barometer / Air Pressure

Topic:

```text
/world/marv_field/model/marv_f450/link/base_link/sensor/air_pressure_sensor/air_pressure
```

Message type:

```text
gz.msgs.FluidPressure
```

Nominal update rate:

```text
50 Hz
```

## Magnetometer

Topic:

```text
/world/marv_field/model/marv_f450/link/base_link/sensor/magnetometer_sensor/magnetometer
```

Message type:

```text
gz.msgs.Magnetometer
```

Nominal update rate:

```text
100 Hz
```

Frame:

```text
FRD relative to body FLU, represented in SDF by roll = pi.
```

## Actuator / Motor Command

Topic:

```text
/marv_f450/command/motor_speed
```

Message type:

```text
gz.msgs.Actuators
```

Direction:

```text
Bridge publishes to Gazebo.
```

Motor order:

```text
motor 0 -> ESC output 1 -> rotor_0_joint / rotor_0 / rear_right  / CW
motor 1 -> ESC output 2 -> rotor_1_joint / rotor_1 / front_right / CCW
motor 2 -> ESC output 3 -> rotor_2_joint / rotor_2 / rear_left   / CCW
motor 3 -> ESC output 4 -> rotor_3_joint / rotor_3 / front_left  / CW
```

Gazebo motor command field:

```text
velocity
```

## Relevant Services

Spawn vehicle:

```text
/world/marv_field/create
```

Blocking spawn vehicle:

```text
/world/marv_field/create/blocking
```

Remove entity:

```text
/world/marv_field/remove
```

Set pose:

```text
/world/marv_field/set_pose
```

Set physics:

```text
/world/marv_field/set_physics
```

Sensor rate services:

```text
/world/marv_field/model/marv_f450/link/base_link/sensor/air_pressure_sensor/air_pressure/set_rate
/world/marv_field/model/marv_f450/link/base_link/sensor/imu_sensor/imu/set_rate
/world/marv_field/model/marv_f450/link/base_link/sensor/magnetometer_sensor/magnetometer/set_rate
/world/marv_field/model/marv_f450/link/base_link/sensor/navsat_sensor/navsat/set_rate
```

## Notes

This repository documents the topics only. The external bridge repository is responsible for subscribing, decoding, and forwarding data.

The bridge should subscribe to the sensor topics, publish actuator commands, and treat pose topics as validation-only ground truth.

The default sim world includes the vehicle. Launch the world before the bridge starts:

```bash
./scripts/launch_world.sh
```

Use `scripts/spawn_f450.sh` only for alternate worlds that do not already include `marv_f450`.
