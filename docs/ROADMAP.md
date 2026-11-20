# Sylph roadmap (living document)

Re-evaluated against the code at commit `9670023` on 2026-11-18, updated after
the production-readiness work. See `docs/BASELINE.md` for the original evidence.
This file tracks what is actually done, not what is planned.

Note on numbering: phases 5 and 7 are intentionally absent. Perception work is
tracked as PR 8 (`docs/PERCEPTION.md`); hardware readiness (phase 7) is gated and
not implemented (`docs/SAFETY.md`). Numbers stay stable for documentation
references.

## Completed

- Phases 0–4: safety preflight, telemetry/waypoint logic, vision/store,
  voice commands + live map, reporting. `sylph/` modules + tests.
- Phase 6: BOM/power sizing, physics twin, optimizer, mission simulator CLI.
- Phase 8: opt-in battery realism (OCV/sag/Peukert), wind gusts/turbulence,
  seeded sensors, additive value revisits, QGC/MAVLink mission export.
- Phase 9: MAVLink SITL protocol client (fake-link tested only), camera FOV +
  probabilistic detection, value-of-information revisits, Monte-Carlo CLI,
  explicit RTL reserve in the planner.

## Now active (roadmap order)

- **PR 1 (this baseline):** audit docs + verified findings.
- **PR 2 (done):** safety mode policy (P0.1) and pending-command fixes (P0.2),
  18 new regression tests.
- **PR 3 (done):** latitude-aware survey geometry (local ENU, heading rotation,
  tests at multiple latitudes), route completeness, mission state machine with
  validated/logged transitions and abort priority. 12 new tests; 124 green.
- **PR 4 (done):** deterministic replay log, scenario fixtures (normal, low
  battery, wind gust), versioned 3D demo data contract (schema v1), and
  `scenario` CLI emitting the payload. 8 new tests; 132 green.
- **PR 5–7 (done):** 3D simulation-only ground station under `web/`
  (React 19 + Three.js/R3F/Drei), driven by schema-v1 replay frames; mission
  panels, waypoint selection, camera presets, timeline, scenarios, detections,
  6 Playwright browser tests, CI workflow. See `docs/3D_DEMO.md`.
- **PR 8:** perception (real pixel-based detector) evaluation plan.
- **PR 9:** SITL integration against a live process (blocked until available).

## Explicitly not done

- No physical hardware control or arming path exists. Hardware mode must stay
  disabled.
- No real image inference is active by default: `StubDetector` is filename-based
  and clearly named. An optional weights-backed detector exists but needs
  externally obtained weights (see `docs/PERCEPTION.md`).
- No live SITL validation has been run in this repository. The MAVLink client is
  protocol-tested with a fake transport; a bench script is provided but must be
  run where SITL is installed (see `docs/SITL.md`).
- No real-flight data calibration: energy, detection and noise models are
  documented assumptions, not measured values.

The 3D demo IS implemented and live under `web/` (see PR 5–7 above and
`docs/3D_DEMO.md`). It is simulation-only by design.

## Definition of done for each PR

Tests cover normal/edge/failure paths; CI green; interfaces documented;
demo-only behavior labeled; no new hardware capability; docs accurate.
