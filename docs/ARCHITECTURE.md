# Sylph architecture (current, verified 2026-11-18)

## Layers

1. **Planner** (`sylph/mission.py`, `sylph/optimizer.py`, `sylph/navigation.py`)
   `parse_command` / `CommandInterpreter` -> `lawnmower_pattern` ->
   `plan_mission` (explicit RTL reserve) -> `Waypoint` list.
2. **Safety** (`sylph/safety.py`) `preflight()` produces a `PreflightReport`.
   NOTE: currently being revised to distinguish simulation/SITL/hardware and
   to fail closed on stale telemetry (see P0.1 in `docs/BASELINE.md`).
3. **Execution** (`sylph/simulator.py`, `sylph/twin.py`, `sylph/mavlink_sitl.py`)
   `DigitalTwin` is the default; `MavlinkSitlController` is an opt-in real
   MAVLink protocol client behind the `FlightController` seam. No hardware
   adapter exists.
4. **Perception** (`sylph/vision.py`, `sylph/camera.py`, `sylph/voi.py`)
   `StubDetector` is demo-only (filename-based). `Camera` models FOV
   footprint and detection probability; VoI decides revisits.
5. **Reporting** (`sylph/reporting.py`, `sylph/monte_carlo.py`, `sylph/cli.py`)
   GeoJSON detection store, mission reports, Monte-Carlo robustness runs,
   JSON export.

## Data flow

```
utterance -> Command (confirm-gated) -> MissionPlan -> waypoint list
   -> DigitalTwin / MavlinkSitlController -> telemetry + events
   -> DetectionStore / LiveDetectionMap -> report / Monte-Carlo
```

The planned 3D demo must be a visualization of the simulation state, never a
second source of movement or truth.

## Operation modes

- `simulation`: synthetic telemetry, no connection, default.
- `sitl`: explicit `--sitl-host` opt-in; protocol client, not live-tested.
- `hardware`: absent and disabled; no arming path exists anywhere in the code.

## External dependencies

Python standard library only at runtime; `pytest` for tests. No frontend, no
model weights, no pymavlink import at import time (lazy).
