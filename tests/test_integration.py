"""End-to-end integration test: voice order -> preflight -> plan -> fly ->
detect -> live map -> report. Pure logic, no hardware required."""

from sylph.link import SimulatedLink, VehicleState
from sylph.mission import CommandInterpreter, LiveDetectionMap, lawnmower_pattern, plan_mission
from sylph.navigation import should_return_to_launch
from sylph.reporting import DEFAULT_PROFILE, build_report
from sylph.safety import preflight
from sylph.vision import DetectionStore, StubDetector, diagnose_leaf_offline, geotag
from sylph.waypoints import Waypoint, WaypointRunner


def test_full_mission_flow_scout_north_field():
    # Phase 0: safety gate
    report = preflight(
        firmware_ok=True,
        gps_fix=True,
        failsafes_tested=True,
        geofence_set=True,
        battery_ok=True,
    )
    assert report.safe is True

    # Phase 3: high-level voice order with confirmation
    executed = []
    interpreter = CommandInterpreter(executed.append)
    cmd = interpreter.submit("scout the north field")
    assert (cmd.kind, cmd.target) == ("scout", "north")
    assert interpreter.confirm() is True

    # Phase 1: vehicle state and smart-RTL callback
    state = VehicleState(36.8, 10.2, DEFAULT_PROFILE.target_altitude_m, 95, 12)
    home = Waypoint(36.8, 10.2, 0)

    def needs_rtl():
        return should_return_to_launch(
            state, home.latitude, home.longitude,
            battery_capacity_wh=100, cruise_speed_mps=12, cruise_power_w=200,
        ).rtl

    # Phase 3: grid pattern + battery-aware plan
    grid = lawnmower_pattern(
        36.8005, 10.2005,
        width_m=100, height_m=50,
        spacing_m=DEFAULT_PROFILE.grid_spacing_m,
        altitude_m=DEFAULT_PROFILE.target_altitude_m,
    )
    grid = [Waypoint(w.latitude, w.longitude, w.altitude_m) for w in grid]
    plan = plan_mission(
        state, grid, home.latitude, home.longitude,
        battery_capacity_wh=100, cruise_speed_mps=12, cruise_power_w=200,
        safety_margin=DEFAULT_PROFILE.safety_margin,
    )
    assert plan.feasible is True

    # Phase 1: fly the mission
    link = SimulatedLink(state)
    result = WaypointRunner(link, needs_rtl).run(plan.waypoints)
    assert result.aborted is False
    assert len(result.completed) == len(grid)
    assert link.sent[-1] == "MISSION_COMPLETE"

    # Phase 2 + 3: detect, geotag, publish to live map
    store = DetectionStore()
    live = LiveDetectionMap(store)
    updates = []
    live.subscribe(updates.append)
    for img in ("field_north_blight_01.jpg", "field_north_leaf.jpg", "field_north_rust_02.jpg"):
        raw = StubDetector().detect(img)
        for d in geotag(raw, 36.8005, 10.2005, timestamp="2026-11-18T00:00:00+00:00"):
            live.publish(d)
    assert len(updates) == 3
    assert len(store) == 3

    # Phase 2: offline diagnosis of a single leaf photo
    diagnosis = diagnose_leaf_offline("field_north_blight_01.jpg")
    assert diagnosis.diagnosis == "late_blight"
    assert diagnosis.offline is True

    # Phase 4: final report
    summary = build_report(list(store.detections))
    assert summary.total_detections == 3
    assert "Sylph Mission Report" in summary.to_text()
