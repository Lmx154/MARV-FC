# AI Change Spec

Use this before non-trivial implementation work.

## Task

- What is changing?
- What user-visible or system-visible outcome must change?

## Repo Area

- Which area is touched: `common/`, `device/`, simulator/HIL, docs, or multiple?
- Exact allowed files:

## Architectural Role

- Is the change ISR, Tier 0, Tier 1, or Tier 2?
- Does it add or alter a task boundary, bus owner, core boundary, or watchdog contract?

## Forbidden Violations

- No concrete `embassy-rp` types in `common/`
- No implicit cross-core subscriptions
- No transport/protocol collapse
- No arbitrary watchdog feeding
- No variable-latency work inserted into the deterministic path without explicit justification

## Tests Required

- Unit tests:
- Integration tests:
- Host/SITL/HIL checks:
- Target checks:

## Measurements Required

- Timing or jitter:
- Freshness or loop age:
- RAM/flash delta:
- Bus occupancy or rate impact:
- Log volume or sink impact:

## Definition of Done

- Code change lands only in allowed files
- Validation commands are recorded and pass
- Canonical docs are updated if an architecture rule or contract changed
- `docs/05-current-deviations-and-migration.md` is updated if repo reality still diverges from the target architecture
