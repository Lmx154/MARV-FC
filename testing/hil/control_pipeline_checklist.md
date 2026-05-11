# Control Pipeline HIL Checklist

## Preconditions

- Confirm HIL mode is active and sensor frames are accepted.
- Confirm arming state is explicit in the test script.
- Confirm mixer motor order is the expected one-based permutation.
- Confirm all motors are bench-safe.

## Scenarios

- Disarmed estimate produces valid zero motor output.
- Armed missing IMU produces invalid zero motor output.
- Armed invalid estimate produces invalid zero motor output.
- Level hold with zero rate error keeps commands near hover throttle.
- Positive roll, pitch, and yaw rate errors produce the expected mixer basis signs.
- NED-down altitude error below the setpoint increases throttle.
- NED-down altitude error above the setpoint decreases throttle.
- Position north/east errors produce the expected pitch/roll signs.
- Saturated controller output sets clamped actuator output and response flags.

## Evidence To Save

- Control setpoint commands.
- State estimates.
- IMU samples used by the control step.
- Actuator output samples.
- HIL response frames with flags.
