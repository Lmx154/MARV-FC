# 03. Verification Strategy

Every non-trivial change must carry validation evidence that matches its architectural impact.

## Validation Ladder

1. Unit tests
   Tier 1 math, transforms, parsers, filters, freshness helpers, and deadline arithmetic
2. Boundary integration tests
   Interfaces, messages, drivers, services, protocols, channel topology, and ownership boundaries
3. Host validation
   `common` host tests plus simulator or replay checks for timestamp semantics and step-driven progression
4. SITL and HIL validation
   Canonical sample publication, output egress, degraded-mode handling, and watchdog parity
5. Embedded target validation
   Bring-up of the affected target path, including reset-cause behavior and liveness evidence when applicable
6. Performance and resource evidence
   Jitter, freshness, end-to-end age, RAM/flash deltas, bus occupancy, log volume, and execution-time impact

## Required Evidence by Change Type

- Pure Tier 1 logic changes require unit tests.
- Interface, message, protocol, service, or task-boundary changes require integration tests and an updated review checklist.
- Fast-path, bus-owner, core-placement, or watchdog changes require timing and liveness evidence.
- Simulation, replay, or HIL changes require proof that portable consumers still run from typed timestamps rather than host wall-clock cadence.

## Observability Requirements

When a change touches runtime behavior, capture the metrics that make the behavior reviewable:

- input freshness and output freshness
- loop age or end-to-end latency where timing matters
- jitter or deadline miss evidence for deterministic paths
- RAM and flash deltas when code size or buffers change
- bus occupancy or transaction-rate impact when a shared bus changes
- log volume and sink-pressure impact when observability changes
- reset cause, watchdog status, or fault evidence when liveness changes

## Definition of Sufficient Validation

Validation is sufficient only when:

- the tests or runs directly exercise the changed boundary
- the commands are recorded in the change spec or review notes
- timing-sensitive changes include measurements, not just reasoning
- any unresolved architectural mismatch is recorded in `05-current-deviations-and-migration.md`

## Preferred Progression

Use the same ladder for new architecture work:

1. prove the pure logic
2. prove the boundary
3. prove host parity
4. prove target behavior
5. prove timing and liveness impact
