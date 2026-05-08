# HIL Control Routing Note

## Summary

The shared HIL framework still exposes a lossy synchronous control-command route, but the current embedded USB ingress paths already bypass it. This is real framework debt, not a confirmed live bug in the current device ingress implementations.

## What Is True

- `HilControlCommandRoute` is synchronous, and its `Channel` implementation uses `try_send`, which can silently drop a control command if the queue is full.
- `HilRuntime::accept(...)` is also synchronous, so it cannot await a guaranteed control-command handoff.
- `run_hil_ingress_loop(...)` only accepts a synchronous `on_dispatch` callback, so the reusable ingress loop cannot await control delivery either.
- `HilRuntime::accept(...)` reports `control_command_published = true` after calling the route, even though the `Channel` route does not confirm enqueue success.

## What Is Not Quite True

It is not accurate to say the current device-side HIL ingress path is still relying on that lossy shared route.

The embedded USB ingress tasks already intercept `HilIngressMessage::ControlCommand` directly and forward with `send(...).await` instead of pushing control commands through `HilRuntime::accept(...)`:

- `device/RECOVERY-ALTIMETER-RP2350/src/usb_cdc.rs`
- `device/MARV-FC-RL-RP2354B/src/usb_cdc.rs`
- `device/PAYLOAD-CONTROLLER-RP2350/src/usb_cdc.rs`

Also, the generic in-tree callers I found do not pass a live control route into `HilIngressRoutes`; they pass `&()` for control:

- `simulator/src/main.rs`
- `common/src/services/acquisition/hil/mod.rs`

So the issue is best described as an architectural problem in `common::services::hil`, not as evidence that the current embedded device path is still dropping control commands through the generic route.

## Evidence

### Shared lossy route shape

- `common/src/services/hil/routing.rs`: `HilControlCommandRoute` is synchronous.
- `common/src/services/hil/routing.rs`: `impl HilControlCommandRoute for Channel<...>` uses `try_send`.
- `common/src/services/hil/runtime.rs`: `HilRuntime::accept(...)` is synchronous and routes `HilIngressMessage::ControlCommand` through `routes.control.publish_control_command(...)`.
- `common/src/services/hil/tasks.rs`: `run_hil_ingress_loop(...)` only provides a synchronous dispatch callback.

### Current embedded bypass

- `device/RECOVERY-ALTIMETER-RP2350/src/usb_cdc.rs`: matches `HilIngressMessage::ControlCommand(command)` and uses `control_sender.send(command).await`.
- `device/MARV-FC-RL-RP2354B/src/usb_cdc.rs`: same pattern.
- `device/PAYLOAD-CONTROLLER-RP2350/src/usb_cdc.rs`: same pattern.

### Current generic callers

- `simulator/src/main.rs`: constructs `HilIngressRoutes::new(..., &(), &())`.
- `common/src/services/acquisition/hil/mod.rs`: constructs `HilIngressRoutes::new(..., &(), &())`.

## Recommended Issue Framing

Suggested framing:

> Shared HIL ingress still exposes a lossy synchronous control-command route.
>
> `common::services::hil` models control-command delivery as synchronous fire-and-forget. The `Channel`-backed control route uses `try_send`, which can silently drop commands on a full queue. `HilRuntime::accept(...)` and `run_hil_ingress_loop(...)` are both synchronous, so the reusable ingress path has no way to await control delivery or surface delivery failure explicitly.
>
> Current embedded USB ingress tasks already bypass this path by intercepting control commands and forwarding them with `send(...).await`, so this is not the mechanism used by current device ingress. The real problem is that the shared framework still exposes a lossy control-routing contract for generic callers.

## Proper Fix

1. Stop routing control commands through the current synchronous `HilControlCommandRoute`.
2. Either return control commands as explicit dispatch output or split control ingress from sample ingress entirely.
3. Make `run_hil_ingress_loop(...)` own an async control sink, or accept an async dispatch callback.
4. Remove or replace the lossy `HilControlCommandRoute for Channel<...>` implementation.
5. Make control-delivery success or failure explicit in runtime dispatch results.

## Bottom Line

The architectural diagnosis is correct: the shared HIL ingress contract is still shaped around lossy synchronous control delivery. The overstated part is the impact. The current embedded USB device paths already route around that problem, so this should be tracked as framework debt and API cleanup, not as proof of an active device-side regression.
