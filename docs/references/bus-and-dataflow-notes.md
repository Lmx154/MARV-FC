# Bus and Dataflow Notes

This reference expands the ownership and message-flow guidance for shared buses, pub-sub, telemetry, logging, and cross-core exchange.

## Focus Areas

- same-core pub-sub as the default fan-out model
- explicit cross-core bridges owned in `channels.rs`
- subscriber-only telemetry and logging posture
- shared-bus ownership models and FC hardware domains

## Primary Source Material Retained During Migration

- `docs/07-communication-bus-and-dataflow.md`
- `docs/FC_SITL_ICD.md`

## Use With

- `docs/01-architecture-rules.md`
- `docs/02-interface-contracts.md`
