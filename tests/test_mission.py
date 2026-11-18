"""Phase 3 tests: voice commands, grid pattern, live map, battery planner."""

import pytest

from sylph.link import VehicleState
from sylph.mission import (
    CommandInterpreter,
    LiveDetectionMap,
    lawnmower_pattern,
    parse_command,
    plan_mission,
)
from sylph.vision import Detection, DetectionStore
from sylph.waypoints import Waypoint


def test_parse_scout_north_field():
    cmd = parse_command("scout the north field")
    assert cmd.kind == "scout"
    assert cmd.target == "north"


def test_parse_other_commands():
    assert parse_command("return to launch").kind == "rtl"
    assert parse_command("hold position").kind == "hold"
    assert parse_command("gibberish").kind == "unknown"
    assert parse_command("").kind == "unknown"


def test_command_requires_confirmation():
    executed = []
    interpreter = CommandInterpreter(executed.append)
    cmd = interpreter.submit("scout the north field")
    assert cmd.kind == "scout"
    assert executed == []
    assert interpreter.confirm() is True
    assert executed[0].kind == "scout"


def test_confirmation_can_be_denied():
    executed = []
    interpreter = CommandInterpreter(executed.append)
    interpreter.submit("scout the north field")
    assert interpreter.confirm(allowed=False) is False
    assert executed == []


def test_confirm_without_pending_returns_false():
    interpreter = CommandInterpreter(lambda cmd: None)
    assert interpreter.confirm() is False


def test_lawnmower_pattern_shape():
    wps = lawnmower_pattern(
        36.8, 10.2, width_m=200, height_m=100, spacing_m=50, altitude_m=20
    )
    assert len(wps) == 6  # 3 rows x 2 points
    assert all(wp.altitude_m == 20 for wp in wps)
    # rows alternate direction
    assert wps[0].longitude < wps[1].longitude
    assert wps[2].longitude > wps[3].longitude


def test_lawnmower_rejects_zero_spacing():
    with pytest.raises(ValueError):
        lawnmower_pattern(0, 0, width_m=100, height_m=100, spacing_m=0, altitude_m=20)


def test_live_map_notifies_incrementally():
    store = DetectionStore()
    live = LiveDetectionMap(store)
    received = []
    live.subscribe(received.append)
    live.publish(Detection("blight", 0.9, 1.0, 2.0, "t"))
    assert len(received) == 1
    assert len(store) == 1
    assert received[0]["geometry"]["coordinates"] == [2.0, 1.0]


def test_planner_accepts_feasible_short_route():
    state = VehicleState(36.8, 10.2, 20.0, battery_pct=100, ground_speed_mps=12)
    plan = plan_mission(
        state,
        [Waypoint(36.8005, 10.2005, 20.0)],
        36.8, 10.2,
        battery_capacity_wh=100, cruise_speed_mps=15, cruise_power_w=200,
    )
    assert plan.feasible is True
    assert len(plan.waypoints) == 1


def test_planner_rejects_exhausted_battery():
    state = VehicleState(36.8, 10.2, 20.0, battery_pct=1, ground_speed_mps=12)
    plan = plan_mission(
        state,
        [Waypoint(36.9, 10.3, 20.0)],
        36.8, 10.2,
        battery_capacity_wh=100, cruise_speed_mps=15, cruise_power_w=200,
    )
    assert plan.feasible is False
    assert plan.waypoints == []
    assert "insufficient energy" in plan.reason


def test_planner_rejects_empty_route():
    state = VehicleState(0, 0, 20.0, 100, 12)
    plan = plan_mission(state, [], 0, 0, battery_capacity_wh=100,
                        cruise_speed_mps=15, cruise_power_w=200)
    assert plan.feasible is False
