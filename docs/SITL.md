# SITL readiness and model assumptions

This document records what Sylph's SITL path does today, what it does not do,
and the assumptions behind the realism features. Nothing here claims a real
flight or a working MAVLink connection.

## Mission export

`sylph.cli export` writes a survey mission in two formats:

- **QGC WPL 110** (`.waypoints`) — plain text, loadable by QGroundControl and
  Mission Planner. Row 0 is HOME at (0,0,0); the flight controller fills it
  from its own GPS. Subsequent rows are `MAV_CMD_NAV_WAYPOINT` (16) in
  `MAV_FRAME_GLOBAL_RELATIVE_ALT` (3), with hold time, acceptance radius and
  latitude/longitude/altitude.
- **MAVLink mission JSON** — a readable representation whose fields map to
  `MISSION_ITEM_INT`: `x`/`y` are degrees × 1e7, `z` is relative altitude,
  `frame=3`, `command=16`.

Both are generated from the same lawnmower grid as the digital twin, so the
exported mission covers exactly the area the simulator surveys. Optional
high-value regions add a dense 3×3 revisit sub-grid *on top of* the base
lawnmower route — the base coverage is never removed or reordered.

## Flight-controller interface

`sylph.sitl.FlightController` is a small protocol (`upload`, `execute`) that
lets the mission runner be swapped later:

- `TwinFlightController` — runs the mission through the existing digital twin.
  This is what the CLI uses for its dry run.
- `MavlinkSitlController` (`sylph/mavlink_sitl.py`) — a real MAVLink client that
  uploads `MissionItem`s with MISSION_COUNT → MISSION_REQUEST_INT →
  MISSION_ITEM_INT → MISSION_ACK, sets a mode, arms, and samples GLOBAL_POSITION_INT
  / VFR_HUD / SYS_STATUS telemetry. It depends on a `MavlinkTransport` protocol;
  `PymavlinkTransport` implements it with lazy pymavlink import. Protocol flow is
  tested with a fake transport. **Not run against live SITL in this repo.**
- `SitlFlightController` — a legacy intentional placeholder that raises
  `NotImplementedError`. It remains to make the "stub vs real" distinction clear.

`export --sitl-host HOST` is the only path that constructs a live transport;
no endpoint is probed or connected to implicitly.

## Mission-level RTL reserve

`plan_mission(..., rtl_reserve_wh=Y)` or `rtl_reserve_pct=X` enforces an explicit
energy reserve *inside the planner*. If the full route would violate it, the
planner keeps only the longest feasible prefix and reports the trim in
`MissionPlan.reason`. With no reserve argument, the legacy behaviour is intact.

## Camera and value-of-information

`sylph/camera.py` models footprint (`2·h·tan(FOV/2)`) and a detection probability
that decreases with altitude and speed and increases with overlap.
`sylph/voi.py` uses that probability to decide whether a high-value (infected)
region's expected detections justify the extra energy of a revisit. Constants
are explicit `VoIConfig` assumptions, not measured economics.

## Monte-Carlo evaluation

`sylph/monte_carlo.py` runs the mission across seeds (wind gusts, turbulence,
sensor noise) and reports success rate, energy distribution and worst-case RTL
margin. The CLI's `montecarlo` command exposes this; JSON export is available.

## Realism assumptions (not measured cell data)

| Feature | Model | Assumptions |
|---------|-------|-------------|
| Battery OCV | piecewise-linear 4S curve, 14.0–16.8 V | typical LiPo, room temperature, no aging |
| Voltage sag | terminal = OCV − I·R | constant per-pack resistance (default 0.030 Ω) |
| Peukert | I_eff = I·(I/I_nom)^(k−1), k=1.05 | mild derating; at I=I_nom it equals the simple model |
| Gust | sinusoidal, configurable amplitude/period | deterministic function of mission time |
| Turbulence | seeded gaussian per leg | independent sample per segment |
| GPS noise | gaussian, sigma 1.5 m horizontal / 3 m altitude | consumer-grade, no bias or multipath |
| Barometer | gaussian, sigma 0.5 m | no temperature drift |
| Voltage sensor | gaussian, sigma 0.05 V | independent of OCV model |

All realism features are off by default. With defaults, the baseline survey
numbers (47.3 Wh, 3.8 min, successful RTL) are unchanged; enabling them changes
results honestly, with no claimed performance improvement.

## Verification

- `python -m pytest tests/` — includes tests for OCV endpoints, sag direction,
  Peukert cost, seeded reproducibility, gust shape, additive revisits, QGC/JSON
  format correctness, and the honest SITL stub.
- `python -m sylph.cli export ...` writes real files and runs a twin dry run;
  it does not connect to any flight controller.
