# MARV Firmware Architecture Docs

This repo now splits architecture material into:

- canonical docs: small, always-loadable rules and contracts
- reference docs: rationale, examples, diagrams, target notes, and migration source material
- templates: change-steering and deviation logging helpers

If a canonical doc conflicts with a reference or an older doc, the canonical doc wins.

## Canonical Docs

1. [`00-system-definition-spine.md`](./00-system-definition-spine.md)
2. [`01-architecture-rules.md`](./01-architecture-rules.md)
3. [`02-interface-contracts.md`](./02-interface-contracts.md)
4. [`03-verification-strategy.md`](./03-verification-strategy.md)
5. [`04-review-checklist.md`](./04-review-checklist.md)
6. [`05-current-deviations-and-migration.md`](./05-current-deviations-and-migration.md)

## Reference Docs

1. [`references/overview-and-rationale.md`](./references/overview-and-rationale.md)
2. [`references/layer-reference.md`](./references/layer-reference.md)
3. [`references/timing-and-core-notes.md`](./references/timing-and-core-notes.md)
4. [`references/bus-and-dataflow-notes.md`](./references/bus-and-dataflow-notes.md)
5. [`references/watchdog-notes.md`](./references/watchdog-notes.md)
6. [`references/target-guidance.md`](./references/target-guidance.md)
7. [`references/simulator-notes.md`](./references/simulator-notes.md)
8. [`references/localization-notes.md`](./references/localization-notes.md)
9. [`references/diagrams.md`](./references/diagrams.md)

## Templates

1. [`templates/ai-change-spec.md`](./templates/ai-change-spec.md)
2. [`templates/architecture-deviation-entry.md`](./templates/architecture-deviation-entry.md)

## Repo AI Guidance

1. [`.github/copilot-instructions.md`](../.github/copilot-instructions.md)

## Legacy Source Docs Retained During Migration

These remain useful as longer-form source material while the repo converges on the new structure:

1. [`01-overview-and-scope.md`](./01-overview-and-scope.md)
2. [`02-core-principles-and-guard-rails.md`](./02-core-principles-and-guard-rails.md)
3. [`03-repository-structure.md`](./03-repository-structure.md)
4. [`04-layer-responsibilities.md`](./04-layer-responsibilities.md)
5. [`05-function-classification-and-task-policy.md`](./05-function-classification-and-task-policy.md)
6. [`06-timing-isolation-and-core-partitioning.md`](./06-timing-isolation-and-core-partitioning.md)
7. [`07-communication-bus-and-dataflow.md`](./07-communication-bus-and-dataflow.md)
8. [`08-watchdog-architecture-and-liveness-supervision.md`](./08-watchdog-architecture-and-liveness-supervision.md)
9. [`09-target-guidance-and-milestones.md`](./09-target-guidance-and-milestones.md)
10. [`10-architecture-and-abstractions-visualization.md`](./10-architecture-and-abstractions-visualization.md)
11. [`11-hil-framework-plan.md`](./11-hil-framework-plan.md)
12. [`FC_SITL_ICD.md`](./FC_SITL_ICD.md)
13. [`simulator.md`](./simulator.md)
14. [`localization.md`](./localization.md)
15. [`pcbhardware.md`](./pcbhardware.md)

## Reading Order

For most work:

1. `docs/00-system-definition-spine.md`
2. `docs/01-architecture-rules.md`
3. `docs/02-interface-contracts.md`
4. `.github/copilot-instructions.md`
5. `docs/templates/ai-change-spec.md`
6. one targeted file under `docs/references/`

## Current Migration Notes

- `common/` and `device/` already exist in the intended roles.
- Host-side simulation is currently implemented in the `simulator/` crate rather than the long-term `sim/` directory layout.
- LoRa physical link profiles live under `common/src/comms/links/lora/`; the previous `coms` compatibility path has been removed.
- Repo-reality mismatches belong in [`05-current-deviations-and-migration.md`](./05-current-deviations-and-migration.md).
