"""Phase 9 tests: MAVLink SITL protocol, camera model, VoI, Monte-Carlo, RTL reserve."""

import random

import pytest

from sylph.camera import Camera, FrameObservation, detect_frame, detection_probability
from sylph.link import VehicleState
from sylph.mavlink_sitl import MavlinkSitlController, VehicleTelemetry
from sylph.mission import plan_mission
from sylph.monte_carlo import monte_carlo
from sylph.optimizer import ValueRegion
from sylph.sitl import MissionItem
from sylph.voi import VoIConfig, evaluate_revisit
from sylph.waypoints import Waypoint

# --- MAVLink SITL protocol (fake transport, no live endpoint) ---------------

class FakeTransport:
    def __init__(self, samples=3):
        self.connected = False
        self.mission = None
        self.mode = None
        self.armed = False
        self.samples_left = samples
        self.closed = False

    def connect(self, timeout_s):
        self.connected = True

    def send_mission(self, items, timeout_s):
        self.mission = list(items)

    def set_mode(self, mode, timeout_s):
        self.mode = mode

    def arm(self, timeout_s):
        self.armed = True

    def poll_telemetry(self, timeout_s):
        if self.samples_left <= 0:
            return None
        self.samples_left -= 1
        return VehicleTelemetry(36.8, 10.2, 20.0, 12.0, 80.0, True, "AUTO", 1.0)

    def close(self):
        self.closed = True


def test_mavlink_controller_uploads_and_receives_telemetry():
    transport = FakeTransport()
    fc = MavlinkSitlController(transport)
    items = [MissionItem(36.8, 10.2, 20), MissionItem(36.801, 10.2, 20)]
    fc.upload(items)
    assert transport.connected and transport.mission == items
    result = fc.execute(max_samples=5)
    assert result["backend"] == "mavlink_sitl"
    assert result["success"] is True
    assert result["samples"] == 3
    assert result["claimed_mission_complete"] is False  # honest
    assert transport.armed and transport.mode == "auto"


def test_mavlink_controller_rejects_empty_mission():
    fc = MavlinkSitlController(FakeTransport())
    with pytest.raises(ValueError):
        fc.upload([])
    with pytest.raises(RuntimeError):
        fc.execute()


def test_mavlink_controller_no_telemetry_is_not_success():
    fc = MavlinkSitlController(FakeTransport(samples=0))
    fc.upload([MissionItem(36.8, 10.2, 20)])
    result = fc.execute()
    assert result["success"] is False
    assert result["last_state"] is None


# --- camera -----------------------------------------------------------------

def test_footprint_grows_with_altitude_and_fov():
    cam = Camera(horizontal_fov_deg=60, vertical_fov_deg=45)
    fp20 = cam.footprint(20)
    fp40 = cam.footprint(40)
    assert fp40.width_m == pytest.approx(2 * fp20.width_m)
    assert fp40.area_m2 == pytest.approx(4 * fp20.area_m2)


def test_detection_probability_falls_with_speed_and_altitude():
    cam = Camera()
    slow = FrameObservation(20, 10, 0.0)
    fast = FrameObservation(20, 25, 0.0)
    high = FrameObservation(40, 10, 0.0)
    assert detection_probability(cam, fast) < detection_probability(cam, slow)
    assert detection_probability(cam, high) < detection_probability(cam, slow)


def test_overlap_improves_detection_probability():
    cam = Camera()
    p0 = detection_probability(cam, FrameObservation(20, 12, 0.0))
    p2 = detection_probability(cam, FrameObservation(20, 12, 0.6))
    assert p2 > p0


def test_detect_frame_respects_ground_truth_and_seed():
    cam = Camera(base_detection_p=0.9)
    obs = FrameObservation(20, 12, 0.5, infected=False)
    assert detect_frame(cam, obs, rng=random.Random(1)) is False
    obs_infected = FrameObservation(20, 12, 0.5, infected=True)
    assert detect_frame(cam, obs_infected, rng=random.Random(1)) is True


def test_camera_validation():
    with pytest.raises(ValueError):
        Camera(horizontal_fov_deg=200)
    with pytest.raises(ValueError):
        detection_probability(Camera(), FrameObservation(20, 12, 1.5))


# --- value of information ---------------------------------------------------

def test_voi_high_value_region_is_worthwhile():
    base = [Waypoint(36.8, 10.2, 20), Waypoint(36.801, 10.2, 20)]
    region = ValueRegion(36.8005, 10.2005, 30, 0.9)
    d = evaluate_revisit(
        base, region, altitude_m=20, speed_mps=12, overlap_fraction=0.2,
        power_w=370,
    )
    assert d.worthwhile is True
    assert d.expected_value > d.energy_cost_utility
    assert d.extra_distance_m > 0


def test_voi_low_value_region_rejected():
    base = [Waypoint(36.8, 10.2, 20)]
    region = ValueRegion(36.8, 10.2, 30, 0.05)
    d = evaluate_revisit(
        base, region, altitude_m=20, speed_mps=12, overlap_fraction=0.2,
        power_w=370,
    )
    assert d.worthwhile is False


def test_voi_energy_cost_can_veto():
    base = [Waypoint(36.8, 10.2, 20)]
    region = ValueRegion(36.8005, 10.2005, 30, 0.9)
    cfg = VoIConfig(energy_cost_per_wh=10000.0)
    d = evaluate_revisit(
        base, region, altitude_m=20, speed_mps=12, overlap_fraction=0.2,
        power_w=370, config=cfg,
    )
    assert d.worthwhile is False
    assert "energy cost" in d.reason


# --- Monte-Carlo ------------------------------------------------------------

def test_monte_carlo_reproducible_and_reports_distribution():
    a = monte_carlo(runs=5, seed_start=10)
    b = monte_carlo(runs=5, seed_start=10)
    assert a == b
    assert a.runs == 5
    assert 0 <= a.success_rate <= 1
    assert a.energy_min_wh <= a.energy_max_wh
    assert a.worst_case_rtl_margin_wh >= 0
    assert "Success rate" in a.to_text()


def test_monte_carlo_rejects_zero_runs():
    with pytest.raises(ValueError):
        monte_carlo(runs=0)


# --- mission-level RTL reserve ----------------------------------------------

def _state(pct=50):
    return VehicleState(36.8, 10.2, 20.0, pct, 12.0)


def _wps(n=8):
    return [Waypoint(36.8 + 0.001 * i, 10.2005, 20.0) for i in range(n)]


def test_planner_explicit_wh_reserve_trims_route():
    plan = plan_mission(
        _state(), _wps(), 36.8, 10.2,
        battery_capacity_wh=100, cruise_speed_mps=15, cruise_power_w=200,
        rtl_reserve_wh=30,
    )
    assert plan.rtl_reserve_wh == 30
    assert plan.required_energy_wh <= plan.available_energy_wh - 30 + 1e-6
    assert "waypoints" in plan.reason or plan.reason == "feasible"


def test_planner_explicit_pct_reserve():
    plan = plan_mission(
        _state(), _wps(), 36.8, 10.2,
        battery_capacity_wh=100, cruise_speed_mps=15, cruise_power_w=200,
        rtl_reserve_pct=0.2,
    )
    assert plan.rtl_reserve_wh == pytest.approx(10.0)


def test_planner_rejects_both_reserve_forms():
    with pytest.raises(ValueError):
        plan_mission(
            _state(), _wps(), 36.8, 10.2,
            battery_capacity_wh=100, cruise_speed_mps=15, cruise_power_w=200,
            rtl_reserve_wh=10, rtl_reserve_pct=0.1,
        )


def test_planner_legacy_path_unchanged():
    old = plan_mission(
        _state(), _wps(2), 36.8, 10.2,
        battery_capacity_wh=100, cruise_speed_mps=15, cruise_power_w=200,
    )
    assert old.reason == "feasible"
    assert old.rtl_reserve_wh == 0.0
