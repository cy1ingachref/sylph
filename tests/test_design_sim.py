"""Phase 6 tests: design sizing, physics twin, optimizer, simulator, CLI."""

import json

import pytest

from sylph.cli import main
from sylph.design import DEFAULT_BOM, size_power_system
from sylph.optimizer import (
    optimal_grid_spacing,
    optimal_speed,
    path_length_m,
    route_length_m,
)
from sylph.simulator import simulate_mission
from sylph.twin import DigitalTwin, Environment, FlightSegment, TwinState


def test_bom_mass_and_cost_positive():
    assert DEFAULT_BOM.total_mass_g > 1500
    assert 600 <= DEFAULT_BOM.total_cost_usd <= 1500  # guide's $600-1500 budget


def test_endurance_in_guide_range():
    b = size_power_system()
    assert 10 <= b.estimated_hover_minutes <= 25  # guide expects 15-25+ min


def test_heavier_drone_needs_more_hover_power():
    light = size_power_system(mtow_kg=1.5)
    heavy = size_power_system(mtow_kg=3.0)
    assert heavy.hover_power_w > light.hover_power_w


def test_power_positive_and_increasing_with_speed():
    twin = DigitalTwin(size_power_system())
    assert twin.power_w(0) < twin.power_w(10) < twin.power_w(20)
    with pytest.raises(ValueError):
        twin.power_w(-1)


def test_headwind_slows_and_costs_more():
    calm = DigitalTwin(size_power_system())
    windy = DigitalTwin(size_power_system(), environment=Environment(wind_speed_mps=5))
    a = calm.simulate_segment(TwinState(100), FlightSegment(1000, 15))
    b = windy.simulate_segment(TwinState(100), FlightSegment(1000, 15), into_wind=1)
    assert b.ground_speed_mps < a.ground_speed_mps
    assert b.energy_wh > a.energy_wh
    assert b.time_s > a.time_s


def test_fly_depletes_battery_is_an_honest_failure():
    twin = DigitalTwin(size_power_system())
    state = TwinState(energy_remaining_wh=0.01)
    with pytest.raises(RuntimeError, match="battery depleted"):
        twin.fly(state, FlightSegment(1000, 15))


def test_optimal_grid_spacing_matches_fov_geometry():
    # 20 m altitude, 60 deg FOV => swath 23.09 m; 20% overlap => 18.48 m
    assert optimal_grid_spacing(20) == pytest.approx(18.48, abs=0.01)
    with pytest.raises(ValueError):
        optimal_grid_spacing(20, overlap_fraction=1.0)


def test_optimal_speed_in_plausible_range():
    twin = DigitalTwin(size_power_system())
    v = optimal_speed(twin)
    assert 4 <= v <= 22


def test_open_path_vs_closed_route():
    pts = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0)]
    assert route_length_m(pts) > path_length_m(pts)


def test_optimized_survey_beats_naive_on_energy():
    opt = simulate_mission()
    naive = simulate_mission(speed_mps=6.0, spacing_m=10.0)
    assert opt.success is True
    assert opt.energy_wh < naive.energy_wh  # honest optimization comparison
    assert opt.detections > 0


def test_naive_survey_can_fail_battery():
    naive = simulate_mission(speed_mps=6.0, spacing_m=10.0)
    assert naive.success is False  # 6 m/s + 10 m spacing exhausts the pack


def test_cli_design_prints_bom(capsys):
    assert main(["design"]) == 0
    out = capsys.readouterr().out
    assert "BOM mass:" in out and "endurance" in out.lower()


def test_cli_fly_json_is_parseable(capsys):
    assert main(["fly", "--json"]) == 0
    data = json.loads(capsys.readouterr().out)
    assert data["success"] is True
    assert data["distance_m"] > 0
    assert data["energy_wh"] > 0


def test_cli_compare_reports_both(capsys):
    assert main(["compare"]) == 0
    out = capsys.readouterr().out
    assert "optimized" in out and "naive" in out
