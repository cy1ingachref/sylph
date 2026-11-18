"""Phase 1 tests: link, smart RTL energy logic, waypoint runner."""

import pytest

from sylph.link import SimulatedLink, VehicleState, send_command
from sylph.navigation import (
    distance_m,
    estimate_return_energy_wh,
    should_return_to_launch,
)
from sylph.waypoints import Waypoint, WaypointRunner


def state(battery_pct=100.0, lat=0.0, lon=0.0):
    return VehicleState(
        latitude=lat,
        longitude=lon,
        altitude_m=20.0,
        battery_pct=battery_pct,
        ground_speed_mps=12.0,
    )


def test_send_command_records_command():
    link = SimulatedLink(state())
    send_command(link, "ARM")
    assert link.sent == ["ARM"]


def test_distance_known_equator_degree():
    # One degree of longitude at the equator is ~111.2 km.
    assert distance_m(0, 0, 0, 1) == pytest.approx(111195, rel=0.01)


def test_distance_same_point_is_zero():
    assert distance_m(48.85, 2.35, 48.85, 2.35) == 0.0


def test_return_energy_scales_with_distance():
    e_near = estimate_return_energy_wh(1000, 10, 100)
    e_far = estimate_return_energy_wh(2000, 10, 100)
    assert e_far == pytest.approx(2 * e_near)


def test_return_energy_rejects_bad_speed():
    with pytest.raises(ValueError):
        estimate_return_energy_wh(1000, 0, 100)


def test_full_battery_near_home_no_rtl():
    decision = should_return_to_launch(
        state(battery_pct=100, lat=0, lon=0.001),
        0,
        0,
        battery_capacity_wh=100,
        cruise_speed_mps=15,
        cruise_power_w=200,
    )
    assert decision.rtl is False


def test_low_battery_far_triggers_rtl():
    # ~1.1 km from home; return ~4 Wh; 25% reserve => ~5 Wh needed.
    # 5% of 100 Wh = 5 Wh -> at/below threshold.
    decision = should_return_to_launch(
        state(battery_pct=5, lat=0, lon=0.01),
        0,
        0,
        battery_capacity_wh=100,
        cruise_speed_mps=15,
        cruise_power_w=200,
    )
    assert decision.rtl is True
    assert "safety margin" in decision.reason


def test_rtl_rejects_bad_margin():
    with pytest.raises(ValueError):
        should_return_to_launch(
            state(), 0, 0, battery_capacity_wh=100,
            cruise_speed_mps=15, cruise_power_w=200, safety_margin=1.5,
        )


def test_waypoint_runner_completes_all():
    link = SimulatedLink(state())
    runner = WaypointRunner(link, should_rtl=lambda: False)
    wps = [Waypoint(1, 2, 20), Waypoint(3, 4, 25)]
    result = runner.run(wps)
    assert result.aborted is False
    assert result.completed == wps
    assert link.sent[-1] == "MISSION_COMPLETE"


def test_waypoint_runner_aborts_on_rtl():
    link = SimulatedLink(state())
    runner = WaypointRunner(link, should_rtl=lambda: True)
    result = runner.run([Waypoint(1, 2, 20)])
    assert result.aborted is True
    assert result.reason == "rtl"
    assert result.completed == []
    assert link.sent == ["RTL"]
