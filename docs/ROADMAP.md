# Sylph roadmap (living document)

Re-evaluated against the code at commit `9670023` on 2026-11-18. See
`docs/BASELINE.md` for the full evidence. This file tracks what is actually
done, not what is planned.

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
- **PR 4:** deterministic replay scenarios and event log.
- **PR 5–7:** 3D demo scene, ground-station UI, browser tests.
- **PR 8:** perception (real pixel-based detector) evaluation plan.
- **PR 9:** SITL integration against a live process (blocked until available).

## Explicitly not done

- No physical hardware control or arming path exists. Hardware mode must stay
  disabled.
- No real image inference: `StubDetector` is filename-based and clearly named.
- No live SITL validation; only protocol logic with a fake transport.
- No 3D demo / frontend exists yet.

## Definition of done for each PR

Tests cover normal/edge/failure paths; CI green; interfaces documented;
demo-only behavior labeled; no new hardware capability; docs accurate.
