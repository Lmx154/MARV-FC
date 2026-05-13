# MARV F450 Physical Parameters

This document stores the physical parameters used by the `models/marv_f450` Gazebo model.

## Vehicle Identity

- Frame: F450 quadcopter
- Layout: X quad
- Sim model name: `marv_f450`
- Airframe profile: `config/airframes/f450_xing2_2809_1045_4s_v0.toml`
- Canonical body frame: FLU (`X+` forward, `Y+` left, `Z+` up)

## Mass Properties

- Total vehicle mass with battery: 1.338 kg
- Battery mass: 0.504 kg
- Battery cells/capacity: 4S, 5000 mAh, 74 Wh nominal
- Battery voltage: 14.8 V nominal, 16.8 V full
- Battery dimensions: 0.150 x 0.030 x 0.050 m
- Battery center offset: [0.0, 0.0, -0.035] m
- Estimated center of mass: [0.0, 0.0, -0.013] m

## Geometry

- Motor-to-motor diagonal distance: 0.455 m
- Motor-to-center arm length: 0.2275 m
- Motor XY coordinate magnitude: 0.160867 m
- Central plate: 0.180 x 0.110 x 0.006 m
- Propeller diameter/radius: 0.254 m / 0.127 m
- Propeller pitch: 0.1143 m

## Propulsion

- Motor model: provisional Gazebo quadratic thrust model
- Propeller size: 10 x 4.5 in
- Battery cell count: 4S
- Battery nominal voltage: 14.8 V
- Estimated max thrust per motor: 10.0 N
- Hover thrust: 13.121 N total, 3.280 N per motor
- Max rotor velocity: 1100 rad/s
- Motor constant: 8.26e-6 N/(rad/s)^2
- Moment constant / yaw gamma: 0.016 m
- Time constants: 0.030 s up, 0.050 s down
- Motor spin directions:
  - ESC output 1 / command index 0: rear_right, [-0.160867, -0.160867, 0.0], CW
  - ESC output 2 / command index 1: front_right, [0.160867, -0.160867, 0.0], CCW
  - ESC output 3 / command index 2: rear_left, [-0.160867, 0.160867, 0.0], CCW
  - ESC output 4 / command index 3: front_left, [0.160867, 0.160867, 0.0], CW

## Allocation

The active mixer/allocation convention is FLU:

```text
[ Fz ]   [  1    1    1    1  ] [ T1 ]
[ Mx ] = [ -a   -a    a    a  ] [ T2 ]
[ My ]   [  a   -a    a   -a  ] [ T3 ]
[ Mz ]   [  g   -g   -g    g  ] [ T4 ]
```

Where `a = 0.160867 m` and `g = 0.016 m`.

## Sensors

- IMU location relative to body center: [0.0, 0.0, 0.0], no rotation, FLU
- GPS location relative to body center: [0.0, 0.0, 0.0]
- Magnetometer location relative to body center: [0.0, 0.0, 0.0], roll 180 deg from body, FRD
- Barometer location relative to body center: [0.0, 0.0, 0.0]

## Inertia Tensor

Initial estimate:

- Ixx: 0.010 kg*m^2
- Iyy: 0.011 kg*m^2
- Izz: 0.019 kg*m^2
- Ixy: 0.0 kg*m^2
- Ixz: 0.0 kg*m^2
- Iyz: 0.0 kg*m^2

## Notes

The model uses one main inertial body. Rotor links carry negligible 1e-6 kg inertials so the Gazebo motor joints remain explicit while the total model mass remains 1.338 kg.

The model should be updated whenever the real drone configuration changes.
