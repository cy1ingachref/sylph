# Sylph safety policy

## What "safe" means

`PreflightReport.safe` is the single boolean for "may this system proceed". It
is True only when ALL of the following hold:

1. `checks_passed` — firmware, GPS fix, failsafes tested, battery OK, geofence set.
2. `mode_authorized` for the explicit operating mode.
3. Telemetry is present and fresh.

It is deliberately NOT enough for simulation checks to pass. A report can have
`checks_passed == True` and `safe == False` (e.g. SITL without authorization,
hardware without permission, stale telemetry).

## Operating modes

- `simulation` — default; synthetic telemetry; no connection; authorized when
  checks pass.
- `sitl` — requires `sitl_authorized=True`; a software-in-the-loop flight
  controller. The MAVLink client is protocol-tested but not live-SITL-tested.
- `hardware` — disabled by default and in code. Requires an explicit reviewed
  `hardware_arm_permitted=True` AND independent `interlocks_ok=True`. No
  hardware adapter exists in the repository; `hardware_arm_permitted_now` is
  the only property that may authorize physical arming, and no caller grants it.

## Fail-closed behavior

- Missing or stale telemetry blocks `safe`.
- A conflicting `simulation` flag and `mode` raises `ValueError`.
- `PreflightReport.invalidated(reason)` returns a copy that can never pass again.
- `require_arm_authorization()` raises `PreflightError` unless `safe` is True.

## Command confirmation

`CommandInterpreter` never executes a recognized utterance on recognition
alone. Every new utterance clears prior pending state (so invalid input cannot
leave a hidden command), pending commands expire, confirmation can be bound to
a unique token or exact normalized utterance, and an optional `safety_check`
is re-run immediately before execution. Confirmation is single-use.

## No real flight

There is no physical arming or takeoff path in this repository. The 3D demo
will be simulation-only and must display `SIMULATION — NO REAL FLIGHT CONTROL`.
