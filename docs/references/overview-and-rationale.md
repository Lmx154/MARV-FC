# Overview and Rationale

This reference collects the longer-form rationale behind the canonical docs.

Use it when you need the background for why the repo is strict about portability, timing, ownership, and watchdog behavior.

## Main Themes

- portability across embedded, SITL, HIL, and replay
- timing-aware design rather than convenience-first structure
- explicit ownership of buses, channels, cores, and watchdog authority
- simulation parity as an architectural requirement rather than a side project

## Primary Source Material Retained During Migration

- `docs/01-overview-and-scope.md`
- `docs/02-core-principles-and-guard-rails.md`
- `docs/03-repository-structure.md`

## Use With

- `docs/00-system-definition-spine.md`
- `docs/01-architecture-rules.md`
