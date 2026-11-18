# Sylph

An autonomous AI scout drone: a small aerial platform that accepts high-level
orders, scouts autonomously, analyzes terrain, detects plant/tree infections,
and returns home safely on low battery.

This repository is the project home for **Sylph**. It starts from the complete
build guide in [`docs/Autonomous_AI_Scout_Drone_Guide.md`](docs/Autonomous_AI_Scout_Drone_Guide.md).

## Architecture

```
Ground Station / App        (orders, mission planning, live monitoring)
        | MAVLink / MQTT / 4G
Companion Computer          (mission logic, onboard AI, camera, battery decisions)
        | MAVLink (UART / USB)
Flight Controller           (stabilization, GPS, RTH, failsafes)
```

- **Flight controller** handles low-level safety and flight (ArduPilot / PX4).
- **Companion computer** handles AI, mission logic, and high-level decisions
  (Jetson Orin Nano recommended, or Pi 5 + Coral TPU).
- Always keep a safety margin for return-to-home.

## Roadmap

Each phase has a Python module under `sylph/` with a matching test file under
`tests/`, gated by GitHub Actions CI on Python 3.10–3.12.

- **Phase 0 — Done:** safety/preflight gate (`sylph/safety.py`), CI scaffolding.
- **Phase 1 — Done:** telemetry link abstraction (`sylph/link.py`), smart RTL
  energy logic and waypoint runner (`sylph/navigation.py`, `sylph/waypoints.py`).
- **Phase 2 — Done:** pluggable detection model, geotagging, GeoJSON store and
  offline leaf diagnosis (`sylph/vision.py`).
- **Phase 3 — Done:** voice/high-level commands with confirm gate, lawnmower
  grid scouting, live detection map, battery-aware planner (`sylph/mission.py`).
- **Phase 4 — Done:** reporting, sensor fusion, field profiles
  (`sylph/reporting.py`) plus an end-to-end test (`tests/test_integration.py`).
- **Phase 6 — Done:** digital twin, physics-based power model, optimizer and
  mission simulator CLI (`sylph/design.py`, `sylph/twin.py`,
  `sylph/optimizer.py`, `sylph/simulator.py`, `sylph/cli.py`).

See the [full guide](docs/Autonomous_AI_Scout_Drone_Guide.md) for hardware,
software stack, model training, smart RTL, and safety/legal details, and
[docs/TESTING.md](docs/TESTING.md) for the simulation + Benchtwin HIL workflow.

## Running

```bash
python -m pip install -e ".[test]"
python -m pytest tests/ -v
```

### Design, optimize and test the airframe in simulation

```bash
python -m sylph.cli design    # BOM mass/cost, hover power, endurance
python -m sylph.cli optimize  # energy-optimal cruise speed + camera FOV-based grid spacing
python -m sylph.cli fly       # fly an optimized 300x300 m survey in the digital twin
python -m sylph.cli compare   # optimized survey vs naive (slow, dense) survey
```

`fly --json` emits machine-readable results. `fly --wind N`, `--speed`,
`--spacing`, `--width`, `--height` and `--altitude` let you test other
configurations. The tool uses momentum-theory hover power, a forward-flight
drag term, wind-adjusted ground speed and 80% depth of discharge; reported
numbers are simulated estimates, not flight measurements.

## Status

All six phases are implemented as pure, hardware-independent decision logic
with a passing test suite. **Simulation first:** the modules are ready for SITL
/ Gazebo or AirSim integration and Benchtwin hardware-in-the-loop testing; no
real flight has been performed. Hardware adapters (pymavlink/MAVSDK, camera,
YOLO/TFLite weights) plug into the `DetectionModel` and `TelemetryLink`
interfaces and are deliberately not committed (see `.gitignore`).
