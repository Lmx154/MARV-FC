# HIL Frame And Response Truth Table

## Sensor Frame Ingress

- Inactive input rejects sensor frames and publishes no canonical samples.
- Duplicate ticks are rejected and increment `duplicate_tick`.
- Older ticks or non-increasing simulation time are rejected and increment `out_of_order_tick`.
- Partial valid flags publish only asserted sensor groups.
- Non-finite values in unasserted fields are ignored.
- Non-finite values in asserted fields reject the frame and increment `invalid_non_finite_sample`.
- Stream reset clears tick correlation so a new epoch can start from a lower tick.

## Response Correlation

- Accepted frame sets `current_in_flight_tick`.
- Matching response clears `current_in_flight_tick`.
- Mismatched response increments `response_mismatch`.
- Missing response increments `missed_response` once per in-flight frame.
- Reset stream correlation clears any in-flight tick.

## Response Flags

- `SENSOR_INPUT_VALID` is set only when the accepted frame published the required sensor inputs.
- `ESTIMATOR_VALID` requires a fresh valid estimate at or after the sensor stamp.
- `CONTROL_VALID` requires a fresh valid actuator sample at or after the sensor stamp.
- `CONTROL_CLAMPED` requires valid control plus a clamped actuator sample.
- Armed response with stale estimate or stale actuator enters failsafe with zero motor output.
- Disarmed response may report valid zero motor output without failsafe.
