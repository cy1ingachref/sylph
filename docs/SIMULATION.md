# Simulation assumptions and replay

The simulator is a deterministic decision/energy model, not a flight dynamics
model. See `docs/SITL.md` for battery, wind and sensor assumptions.

## Clock and state

- One mission clock: `TwinState.time_s`, incremented per flight segment.
- Simulation state (`TwinState`) is separate from rendering state
  (`TelemetryFrame` in `sylph/replay.py`). The renderer only reads frames.
- All randomness is seeded (wind turbulence, sensor noise, scenario fixtures).
  Same seed -> same frames and events.

## Scenario fixtures

`python -m sylph.cli scenario --name <normal|low_battery|wind_gust> --seed N`
emits a schema-v1 payload with explicit units, stable IDs, events and
detections. `low_battery` deliberately aborts before the survey and logs
`battery_low` / `rtl` / `failed` events; `wind_gust` adds headwind and gusts.

## Known limitations

- No takeoff/landing/turn dynamics, no 6-DOF attitude, no obstacle avoidance.
- Detection is a deterministic demo stub, not image inference.
- Energy is momentum-theory-based and uncalibrated against real flight data.
- Monte-Carlo statistics are simulated robustness checks, not flight data.

Replay logs can be written as JSONL via `ReplayLog.write_jsonl()` for later
inspection. Scenario-level assertions live in `tests/test_replay.py`.
