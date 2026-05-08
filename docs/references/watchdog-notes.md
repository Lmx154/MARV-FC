# Watchdog Notes

This reference keeps the longer watchdog rationale, classification, and file-placement discussion.

## Focus Areas

- watchdog as a liveness contract rather than a timer peripheral detail
- Tier 0 hardware ownership versus Tier 2 supervisory logic
- feed-critical, degrade-critical, and non-critical classification
- boot-time reset-cause and fault reporting

## Primary Source Material Retained During Migration

- `docs/08-watchdog-architecture-and-liveness-supervision.md`

## Use With

- `docs/01-architecture-rules.md`
- `docs/03-verification-strategy.md`
