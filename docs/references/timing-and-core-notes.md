# Timing and Core Notes

This reference holds the longer discussion around execution classes, task creation, timing islands, and core placement.

## Focus Areas

- ISR, Tier 0, Tier 1, and Tier 2 role separation
- why tasks are scheduling boundaries rather than architecture layers
- why sample timestamps drive portable `dt`
- why Core 0 owns deterministic work and Core 1 owns slower or more variable-latency work

## Primary Source Material Retained During Migration

- `docs/05-function-classification-and-task-policy.md`
- `docs/06-timing-isolation-and-core-partitioning.md`

## Use With

- `docs/01-architecture-rules.md`
- `docs/03-verification-strategy.md`
