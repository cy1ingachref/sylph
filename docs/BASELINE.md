# Sylph baseline audit (2026-11-18)

This is a fresh verification, not a restatement of the static roadmap review.
The counts below are the state AT THE AUDIT (94 tests). The latest state is
recorded in `docs/ROADMAP.md` and measured in CI.

## Environment and commands

- Branch/commit at audit: `main` @ `9670023781d38b0bf17db8264f8cded61cf569ca`
- Python 3.11.15, pytest 9.1.1
- `python -m pytest tests/ -q` -> **94 passed at audit** (now 132+; see ROADMAP)
- CI workflow: `.github/workflows/ci.yml`, Python 3.10 / 3.11 / 3.12

## Confirmed architecture

Mission flow: `CommandInterpreter` (voice/high-level order with confirm gate)
-> `plan_mission` (route + explicit RTL reserve) -> simulation via
`DigitalTwin` (wind, battery, sensors) or MAVLink SITL transport ->
`DetectionStore` / `LiveDetectionMap` -> `build_report` / Monte-Carlo.

Modules at audit: `sylph/` had 17 Python modules and `tests/` 9 test files.
Runtime dependencies: none (pure stdlib); `pytest` test dependency.
Updated state: 22+ modules including `mission_state.py`, `replay.py`,
`scenarios.py`; a simulation-only 3D frontend exists under `web/`
(`docs/3D_DEMO.md`). Still no hardware adapter.

## Verified issue list

| ID | Status | Severity | File / line | Evidence |
|----|--------|----------|-------------|----------|
| P0.1 preflight gate | **FIXED** | High | `sylph/safety.py` | Was: `safe` ignored `simulation`. Now: explicit `OperatingMode`, `checks_passed` / `mode_authorized` / `hardware_arm_permitted_now`, stale-telemetry fail-closed, hardware disabled by default. 11 regression tests. |
| P0.2 stale pending command | **FIXED** | High | `sylph/mission.py` | Was: invalid utterance left prior command executable. Now: every submit invalidates pending, UUID token + normalized-text binding, expiry, pre-execution safety check, single-use confirm. 7 regression tests. |
| Survey geometry | **FIXED** | Medium | `sylph/mission.py` | Was: fixed 111,320 m/deg longitude. Now: `meters_per_degree_lon(latitude)` with documented local-ENU range, optional `heading_deg`, tests at lat 0/36.8/60. Honest consequence: baseline survey path 4374→5390 m, energy 47.3→57.8 Wh (old field was physically ~238 m wide, not 300 m). |
| Detector stub | **CONFIRMED** | Low (honest) | `sylph/vision.py` | `StubDetector` infers from filenames; it is clearly named and documented as a stub, not real inference. |
| SITL live validation | **NOT DONE** | High (for flight readiness) | `sylph/mavlink_sitl.py` | Protocol tested with a fake transport only; no live SITL run. |

## Incomplete / simulated-only features

- `StubDetector`: filename-based demo detector, not image inference (optional
  real detector added later; see `docs/PERCEPTION.md`).
- `MavlinkSitlController`: real protocol implementation, but **not live-tested**
  in this repo; bench script added later (see `docs/SITL.md`).
- Camera/VoI probabilities: documented assumptions, not calibrated from data.
- Monte-Carlo: deterministic digital-twin runs, not flight statistics.
- 3D demo: absent at audit; now implemented and simulation-only (`web/`).

## Mission lifecycle

At audit, there was no explicit mission state machine. A validated, logged
`MissionStateMachine` with abort priority was added in roadmap PR 3
(`sylph/mission_state.py`); states in the twin remain implicit in its success
flag and event log.

## Next smallest high-value task

Fix P0.1 and P0.2 with regression tests (next PR), then survey geometry.
