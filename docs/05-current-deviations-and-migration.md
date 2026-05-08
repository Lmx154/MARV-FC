# 05. Current Deviations and Migration

This document records where the current repo matches the target architecture and where it still diverges.

Status meanings:

- `compliant`: matches the target architectural shape
- `transitional`: intentionally moving toward the target shape
- `violation`: conflicts with the target shape and needs correction
- `deferred`: intentionally out of scope for the current migration window

## Current State

| Area | Status | Current State | Target State | Next Step |
| --- | --- | --- | --- | --- |
| `common/interfaces/` seam | `compliant` | `common/src/interfaces/` exists and is already the primary portable capability seam. | Keep interfaces stable and minimal. | Update only when a real capability boundary changes. |
| `common/messages/` seam | `compliant` | `common/src/messages/` exists and is already the primary typed boundary seam. | Keep message categories explicit and portable. | Extend only through typed contracts. |
| Device ownership files | `compliant` | Primary device crates such as `MARV-FC-RL-RP2354B`, `PAYLOAD-CONTROLLER-RP2350`, and `RECOVERY-ALTIMETER-RP2350` already expose `resources.rs`, `channels.rs`, `watchdog.rs`, `core0.rs`, and `core1.rs`. | Keep per-target ownership explicit. | Maintain this file posture as targets evolve. |
| Cross-core test target structure | `deferred` | `device/RP2354B-CROSSCORE-TESTS` currently exposes only `src/main.rs`. | Full target-owned resource, channel, and watchdog structure if it grows beyond a narrow experiment. | Expand only if the test target becomes long-lived. |
| `common/` / `device/` / `sim/` split | `transitional` | `common/` and `device/` exist, but host-side simulation currently lives in the `simulator/` crate instead of a mirrored `sim/` tree. | Contract-preserving `sim/` layout that mirrors embedded assembly more directly. | Keep `simulator/` aligned with canonical contracts; migrate layout deliberately later. |
| Target naming discipline | `transitional` | `MARV-FC-RL-RP2354B` matches the naming pattern, while `PAYLOAD-CONTROLLER-RP2350` and `RECOVERY-ALTIMETER-RP2350` do not. | Consistent `MARV-[ROLE]-[PROGRAM]-[TARGET]` naming where practical. | Rename or alias target crates only as part of a deliberate migration. |
| Protocol/comms split | `compliant` | `common/src/protocol/` and `common/src/comms/` exist, and the LoRa physical profile surface now lives under `common/src/comms/links/lora/`. | Keep protocol codecs separate from link ownership and radio adapters. | Add new link behavior under `common/src/comms/links/` only when the protocol is intentionally designed. |
| HIL framework home | `transitional` | `common/src/services/hil/` exists, but `common/src/services/acquisition/hil/` remains as a compatibility shim during migration. | HIL lives entirely under the shared `common/src/services/hil/` framework. | Continue converging new HIL work on the shared framework path. |
| HIL control-plane rollout | `transitional` | The HIL session model now supports `Inactive -> Limbo -> Selected(submode)` through MAVLink control commands, and invalid payloads for HIL commands are acknowledged deterministically instead of being dropped. | Full control-plane behavior is enforced uniformly across embedded and host runtimes. | Keep expanding host-side path convergence and remove remaining transitional exceptions. |
| Host runtime HIL entry parity | `transitional` | The host `simulator/` path currently uses a shared `common/services/hil` transitional helper to start in state-estimation mode before full command-plane parity is complete. | Host runtime enters and selects HIL modes only through the same explicit enter/limbo/select command flow used by embedded targets. | Replace the transitional helper with full host-side control-plane wiring and remove the exception. |
| Full-run and typed device event boundaries | `deferred` | Full-run mode objectives and per-device event contracts are not fully split into typed device-specific HIL event messages yet; generic mission-event payloads remain in use. | Device-specific full-run event contracts are explicit and typed while shared semantics stay in `common/services/hil`. | Land a dedicated follow-up that introduces typed per-device event contracts and updates simulator consumers. |
| SITL symmetry | `transitional` | The repo has a working host-side SITL runtime and frozen FC SITL ICD, but not the full target-mirrored `sim/<target>/` layout described in the architecture. | Embedded and simulated targets share mirrored assembly posture and contracts. | Preserve contract parity now; pursue directory symmetry when the host runtime grows. |
| Radio and ground-station target coverage | `deferred` | The current workspace focuses on FC and other embedded device crates; radio and GS targets from the architecture docs are not present as workspace members today. | Broader target-family coverage under the same rules. | Add only when those targets are actively brought up. |

## Migration Rules

- Update this document whenever architecture intent and repo reality diverge in a way a reviewer should know.
- Prefer `transitional` over pretending a mismatch is already solved.
- Do not mark an item `compliant` unless the code and file layout actually match the rule today.
- When a migration lands, update the canonical doc first, then update this deviation log, then refresh any supporting references.
