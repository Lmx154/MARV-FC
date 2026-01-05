# Link Protocol Progress

This is a concise summary of the decisions and implementation work from the
recent protocol/link iteration.

## Changes Implemented

- LoRa MAC header trimmed to 3 bytes (tick_seq + frame_type); magic/version/net_id/flags removed.
- Manual packet layout adopted; per-sensor telemetry packet types with encode/decode added.
- `docs/protocol.md` updated to per-sensor telemetry, manual packet bytes, and LoRa/UART framing notes.
- Telemetry scheduling added in `link_transport` with mock data and per-rate selection.
- GS logs packet type/len on RX; MAC presets downlink payload increased to 16 bytes.

## Current Behavior

- Telemetry packet types show on GS logs (mock data flowing).
- Occasional UL slot gaps observed on radio; LQ stays ~90-100%.
- Downlink rate is limited by MAC schedule (tick_hz/slot_ratio).

## Open Work

- Test all RF profiles and derive compatible MAC parameters (tick_hz, payload sizes, guards).
- Implement GS/radio state machine for mode switching (drone/rocket), acquisition, and RF profile changes.
- Build FC<->radio UART and GS<->GCS USB-CDC links; decide framing + CRC and protocol mapping.
- Replace mock telemetry with real sensor/FC sources; define any quantization/packing rules.
- Reconcile telemetry target rates with MAC downlink budget or adjust schedule/payload sizes.
