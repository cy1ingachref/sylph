"""PR 3 tests: latitude-aware geometry, route completeness, mission states."""

import pytest

from sylph.link import VehicleState
from sylph.mission import (
    lawnmower_pattern,
    meters_per_degree_lon,
    plan_mission,
)
from sylph.mission_state import MissionState, MissionStateError, MissionStateMachine
from sylph.navigation import distance_m
from sylph.waypoints import Waypoint

# --- geometry ----------------------------------------------------------------

def test_longitude_scale_varies_with_latitude():
    eq = meters_per_degree_lon(0)
    mid = meters_per_degree_lon(45)
    assert eq > mid > 0
    assert eq == pytest.approx(111_320, rel=0.01)


def test_same_metre_width_shrinks_longitude_span_at_high_latitude():
    eq = lawnmower_pattern(0, 10, width_m=1000, height_m=100, spacing_m=50, altitude_m=20)
    high = lawnmower_pattern(60, 10, width_m=1000, height_m=100, spacing_m=50, altitude_m=20)
    eq_span = max(w.longitude for w in eq) - min(w.longitude for w in eq)
    hi_span = max(w.longitude for w in high) - min(w.longitude for w in high)
    assert hi_span > eq_span  # fewer metres per degree -> larger degree span


def test_physical_width_matches_across_latitudes():
    for lat in (0, 36.8, 60):
        wps = lawnmower_pattern(lat, 10, width_m=300, height_m=200,
                                spacing_m=50, altitude_m=20)
        west = min(wps, key=lambda w: w.longitude)
        east = max(wps, key=lambda w: w.longitude)
        span = distance_m(lat, west.longitude, lat, east.longitude)
        assert span == pytest.approx(300, rel=0.02)


def test_rotated_grid_is_different_but_same_point_count():
    a = lawnmower_pattern(36.8, 10.2, width_m=200, height_m=100,
                          spacing_m=50, altitude_m=20)
    b = lawnmower_pattern(36.8, 10.2, width_m=200, height_m=100,
                          spacing_m=50, altitude_m=20, heading_deg=45)
    assert len(a) == len(b)
    assert [(w.latitude, w.longitude) for w in a] != [(w.latitude, w.longitude) for w in b]


def test_narrow_and_wide_fields_and_min_spacing():
    narrow = lawnmower_pattern(36.8, 10.2, width_m=10, height_m=10,
                               spacing_m=5, altitude_m=20)
    wide = lawnmower_pattern(36.8, 10.2, width_m=2000, height_m=2000,
                             spacing_m=500, altitude_m=20)
    assert len(narrow) >= 2 and len(wide) >= 2
    with pytest.raises(ValueError):
        lawnmower_pattern(0, 0, width_m=0, height_m=100, spacing_m=10, altitude_m=20)


# --- route completeness ------------------------------------------------------

def test_planner_counts_home_to_first_and_last_to_home():
    state = VehicleState(36.8, 10.2, 20, 100, 12)
    one = [Waypoint(36.81, 10.2, 20)]  # ~1.1 km north
    plan = plan_mission(
        state, one, 36.8, 10.2, battery_capacity_wh=100,
        cruise_speed_mps=15, cruise_power_w=200,
    )
    assert plan.feasible and plan.required_energy_wh > 0
    # ~1.113 km north, 2.226 km round trip at 15 m/s = 8.24 Wh * 1.25 = 10.3 Wh
    assert plan.required_energy_wh == pytest.approx(10.3, abs=0.5)


# --- mission state machine ---------------------------------------------------

def test_happy_path_transitions():
    sm = MissionStateMachine()
    sm.transition(MissionState.VALIDATING)
    sm.transition(MissionState.READY)
    sm.transition(MissionState.RUNNING)
    sm.transition(MissionState.COMPLETED)
    assert sm.is_terminal
    assert [t.to_state for t in sm.log] == [
        MissionState.VALIDATING, MissionState.READY,
        MissionState.RUNNING, MissionState.COMPLETED,
    ]


def test_invalid_transition_rejected_and_logged():
    sm = MissionStateMachine()
    with pytest.raises(MissionStateError):
        sm.transition(MissionState.RUNNING)
    assert sm.state == MissionState.DRAFT
    assert sm.log == []


def test_abort_from_running_and_paused():
    sm = MissionStateMachine()
    sm.transition(MissionState.VALIDATING)
    sm.transition(MissionState.READY)
    sm.transition(MissionState.RUNNING)
    sm.transition(MissionState.PAUSED)
    sm.request_abort("low battery")
    assert sm.state == MissionState.ABORTING


def test_abort_from_draft_fails_closed():
    sm = MissionStateMachine()
    sm.request_abort("early abort")
    assert sm.state == MissionState.FAILED


def test_abort_is_idempotent_in_terminal_state():
    sm = MissionStateMachine()
    sm.transition(MissionState.VALIDATING)
    sm.transition(MissionState.READY)
    sm.transition(MissionState.RUNNING)
    sm.transition(MissionState.COMPLETED)
    sm.request_abort()
    assert sm.state == MissionState.COMPLETED
    assert len(sm.log) == 4


def test_pause_resume_cycle():
    sm = MissionStateMachine()
    for s in (MissionState.VALIDATING, MissionState.READY, MissionState.RUNNING):
        sm.transition(s)
    sm.transition(MissionState.PAUSED)
    sm.transition(MissionState.RUNNING)
    assert sm.state == MissionState.RUNNING
