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
- **Phase 8 — Done:** opt-in battery realism (OCV sag + Peukert), gust/turbulence
  and seeded sensor noise (`sylph/battery.py`, `sylph/sensors.py`), value-region
  revisits, and SITL mission export + swappable FC interface (`sylph/sitl.py`).
- **Phase 9 — Done:** MAVLink SITL client with injectable transport
  (`sylph/mavlink_sitl.py`), camera FOV + probabilistic detection
  (`sylph/camera.py`), value-of-information revisit decisions (`sylph/voi.py`),
  Monte-Carlo evaluation (`sylph/monte_carlo.py`), and explicit RTL reserve in
  the mission planner.
- **Roadmap PRs 1–5 — Done:** baseline audit, P0 safety fixes, latitude-aware
  geometry + mission states, deterministic replay/scenarios, and a **3D
  simulation-only ground-station demo** under `web/` (`docs/3D_DEMO.md`).

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

### Realism (all opt-in; defaults reproduce the baseline)

```bash
python -m sylph.cli fly --battery-realism --gust 2 --turbulence 0.5 --seed 42 \
    --sensor-noise --revisit-lat 36.8005 --revisit-lon 10.2005
```

- `--battery-realism`: 4S LiPo OCV curve, I·R voltage sag, Peukert derating.
- `--gust` / `--turbulence`: sinusoidal gust + seeded gaussian wind noise.
- `--seed`: makes turbulence and sensor streams reproducible.
- `--sensor-noise`: seeded GPS (position + altitude), barometer and voltage noise.
- `--revisit-lat/lon/value`: adds a dense sub-grid over a high-value (infected)
  area **in addition to** the full lawnmower survey — never instead of it.

### SITL mission export (software-only, no flight controller required)

```bash
python -m sylph.cli export --format wpl      --output survey.waypoints
python -m sylph.cli export --format mavlink-json --output survey.json
```

Exports QGC WPL 110 (QGroundControl / Mission Planner) and a MAVLink
`MISSION_ITEM_INT`-shaped JSON. `FlightController` in `sylph/sitl.py` is the seam
for swapping in a real ArduPilot/PX4 SITL link. `SitlFlightController` is an
explicit stub that raises `NotImplementedError` until such a link is built —
no SITL flight is claimed. The export command runs a digital-twin dry run only.

**Assumptions:** OCV/Peukert/internal-resistance values are typical 4S LiPo
approximations, not measured cell data; sensor sigmas are consumer-grade
defaults; revisit value is supplied by the caller (e.g. from a prior flight).
See `docs/SITL.md` for details.

### Monte-Carlo robustness evaluation

```bash
python -m sylph.cli montecarlo --runs 50 --seed-start 0
python -m sylph.cli montecarlo --runs 20 --json
```

Runs the same mission with different wind/turbulence/sensor seeds and reports
success rate, energy mean/median/stdev/range, and worst-case RTL margin.
Results are deterministic for a given seed range.

### MAVLink SITL (explicit opt-in only)

```bash
python -m sylph.cli export --format wpl --output survey.waypoints \
    --sitl-host 127.0.0.1 --sitl-port 5762
```

`MavlinkSitlController` uploads the mission via the standard MISSION_COUNT /
MISSION_REQUEST_INT / MISSION_ITEM_INT / MISSION_ACK handshake, sets AUTO,
arms, and receives telemetry through an injectable transport. Protocol logic is
tested with a fake transport; **a live SITL connection has NOT been run or
verified in this repository** — validate on a bench before relying on it.
Without `--sitl-host` the CLI never connects to anything.

### 3D ground-station demo (simulation only)

```bash
cd web && npm ci && npm run dev    # http://127.0.0.1:8080
```

The demo renders the deterministic scenario payloads from
`python -m sylph.cli scenario`. See `docs/3D_DEMO.md`. It never controls real
hardware and always displays `SIMULATION — NO REAL FLIGHT CONTROL`.

## Status

All nine phases plus roadmap PRs 1–5 are implemented as pure,
hardware-independent logic with a passing test suite. **Simulation first:**
the modules are ready for SITL / Gazebo or AirSim integration and Benchtwin
hardware-in-the-loop testing; no real flight has been performed. Hardware
adapters (pymavlink/MAVSDK, camera, YOLO/TFLite weights) plug into the
`DetectionModel` and `TelemetryLink` interfaces and are deliberately not
committed (see `.gitignore`).
