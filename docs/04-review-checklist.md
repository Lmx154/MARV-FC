# 04. Review Checklist

Use this checklist for every non-trivial change.

## Classification

- Which domain does the change belong to: `common/`, `device/`, or simulation/HIL?
- Is the changed logic ISR, Tier 0, Tier 1, or Tier 2?
- Does the change introduce or remove a task boundary?
- Does the change alter a stable interface or message contract?

## Boundaries

- Are dependency directions still legal?
- Does any board-specific knowledge leak into portable logic?
- Does protocol remain separate from transport and session behavior?
- Are telemetry and logging still subscribers rather than fast-path owners?

## Timing and Core Ownership

- Can this affect Core 0 determinism?
- Is any new variable-latency behavior entering a deterministic island?
- If cross-core data is added, is it an explicit bridge in `channels.rs`?
- Does portable logic still compute `dt` from sample timestamps?

## Bus and Dataflow

- Which bus or transport domain does this touch?
- Who owns that bus or transport after the change?
- Does the producer publish typed stamped samples?
- Could any consumer backpressure or stall a feed-critical producer?

## Watchdog and Faults

- Is the change feed-critical, degrade-critical, or non-critical?
- Does it change what counts as valid forward progress?
- Does it change watchdog authority, feed cadence, or reset reporting?
- Are reset cause and fault evidence still surfaced correctly?

## Validation

- What tests are required?
- What runtime or target commands prove the change?
- What measurements are required for timing, freshness, RAM/flash, bus occupancy, or log volume?
- If the repo still diverges from the target architecture, was `05-current-deviations-and-migration.md` updated?
