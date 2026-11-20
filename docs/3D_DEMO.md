# Sylph 3D demo ground station

The demo is a **visualization of simulation state only**. It renders telemetry
frames produced by `sylph.scenarios` / `sylph.simulator` and never invents
drone movement. Every screen displays `SIMULATION — NO REAL FLIGHT CONTROL`.
There is no takeoff, arming, or hardware-control button.

## Run locally

```bash
# 1. regenerate the sample scenarios from the simulator (single source of truth)
python -m sylph.cli scenario --name normal      --seed 1 --output web/public/scenarios/normal.json
python -m sylph.cli scenario --name low_battery --seed 1 --output web/public/scenarios/low_battery.json
python -m sylph.cli scenario --name wind_gust   --seed 1 --output web/public/scenarios/wind_gust.json

# 2. install and run
cd web
npm ci
npm run dev       # http://127.0.0.1:8080
npm run build     # typecheck + production bundle
npm test          # Playwright browser tests
```

CI workflow: `.github/workflows/web.yml` (Node 22, build + Playwright).

## Features

- 3D farm field with crop rows, boundary and HOME marker; low-poly quadcopter.
- Orange lawnmower route with numbered waypoint markers.
- Drone position/heading/altitude driven entirely by `TelemetryFrame`s.
- Camera presets: orbit (angled overview), top (plan view), follow (chase).
- Panels: mission/waypoints (left), vehicle telemetry + live events (right).
- Timeline: play/pause, reset, speed (0.5–4×), scrub to any frame.
- Scenario selector: normal, low battery (visible RTL response), wind gust.
- Detection markers color-coded by severity, tied to geotagged coordinates.
- Versioned data contract validation: the UI refuses non-schema-v1 or
  non-finite values rather than silently treating them as zero.

## Data contract

`web/src/types.ts` mirrors `sylph/replay.py` schema_version=1. Units are
explicit (`alt_m`, `lat_deg`, `speed_mps`, `t_s`). Geographic -> local scene
coordinates use ONE documented ENU transform (`web/src/sceneTransform.ts`)
shared by the route, field and drone. The demo consumes checked-in sample JSON
for deterministic frontend development; a later API/WebSocket can replace the
data source without changing the renderer.

## Safety

The demo is not proof of flight readiness. Hardware mode is disabled in code
(`docs/SAFETY.md`); SITL is protocol-tested only (`docs/SITL.md`); the detector
is a clearly named stub (`docs/BASELINE.md`). No visual effect here implies
real aircraft control.
