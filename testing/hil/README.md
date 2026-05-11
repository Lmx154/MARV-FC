# HIL Scenario Checklists

These checklists preserve the HIL-first validation path. They are not a software simulator plan; use them to drive bench and hardware-in-loop runs after host-side deterministic tests pass.

## Before Each Run

- Confirm firmware build hash and config.
- Confirm simulator, bridge, and firmware protocol versions.
- Confirm motor outputs are physically safe or routed to a bench load.
- Confirm HIL sensor input is inactive before the host declares readiness.
- Capture raw HILink traffic and firmware logs.
- Record the expected arming state and failsafe policy for the run.

## Acceptance Evidence

- Raw HIL sensor frames and response frames are archived.
- Firmware logs include HIL activation, accepted frames, rejected frames, and response correlation counters.
- Any rejected frame has an expected reason.
- Any failsafe output has an expected trigger.
- Host-side tests for `common` pass before the run.
