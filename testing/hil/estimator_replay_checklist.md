# Estimator Replay HIL Checklist

## Fixed Inputs

- Use a short recorded or generated sequence with IMU, GPS, barometer, and magnetometer samples.
- Keep sample timestamps monotonic and record the expected `dt`.
- Include at least one partial sensor frame where GPS or magnetometer is absent.
- Include one rejected measurement case with a documented gate reason.

## Expected Firmware Behavior

- First IMU sample publishes an invalid neutral estimate until a valid `dt` exists.
- Valid IMU `dt` enters predict.
- Gravity and magnetic updates keep the quaternion finite and normalized.
- GPS position update moves the estimate toward the measurement.
- GPS velocity update moves the estimate toward the measurement.
- Barometric altitude update moves down-position toward the measurement.
- Invalid or stale samples publish an invalid neutral estimate or leave state unchanged according to the estimator policy.

## Evidence To Save

- Input frame log.
- State estimate log.
- Rejection summaries and counters.
- Final bounded-state snapshot.
