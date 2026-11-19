"""Phase 8 tests: battery realism, sensors, wind gusts, value revisits, SITL export."""

import json

import pytest

from sylph.battery import BatteryModel, BatteryState, interpolate_ocv
from sylph.cli import main
from sylph.design import size_power_system
from sylph.optimizer import (
    ValueRegion,
    plan_revisits,
    revisit_extra_distance_m,
    optimal_speed,
)
from sylph.sensors import SensorConfig, SensorSuite
from sylph.simulator import simulate_mission
from sylph.sitl import (
    MissionItem,
    to_mavlink_json,
    to_qgc_wpl,
    TwinFlightController,
    SitlFlightController,
)
from sylph.twin import DigitalTwin, Environment, FlightSegment, TwinState
from sylph.waypoints import Waypoint


# --- battery ---------------------------------------------------------------

def test_ocv_interpolation_endpoints():
    assert interpolate_ocv(0.0) == 14.0
    assert interpolate_ocv(1.0) == 16.8
    assert interpolate_ocv(0.5) == pytest.approx(15.2)


def test_ocv_rejects_bad_soc():
    with pytest.raises(ValueError):
        interpolate_ocv(1.5)


def test_battery_disabled_matches_linear_draw():
    model = BatteryModel(enabled=False)
    st, volt = model.draw(BatteryState(100, 1.0), 100, 3600)
    assert st.energy_remaining_wh == pytest.approx(0.0) or st.energy_remaining_wh == 0.0
    assert volt == 14.8


def test_battery_sag_reduces_terminal_voltage():
    model = BatteryModel(enabled=True)
    _, volt = model.draw(BatteryState(90, 0.8), 500, 1)
    assert volt < model.ocv_v(0.8)
    assert volt > 0


def test_battery_peukert_costs_more_at_high_current():
    low = BatteryModel(enabled=True, nominal_current_a=25)
    high = BatteryModel(enabled=True, nominal_current_a=5)  # 25A is 5x nominal
    st0 = BatteryState(100, 1.0)
    a, _ = low.draw(st0, 370, 60)    # ~25 A at 14.8 V
    b, _ = high.draw(st0, 370, 60)
    assert b.energy_remaining_wh < a.energy_remaining_wh


def test_battery_rejects_bad_peukert():
    with pytest.raises(ValueError):
        BatteryModel(peukert_k=2.5)


# --- sensors ----------------------------------------------------------------

def test_sensors_without_seed_are_perfect():
    s = SensorSuite(seed=None)
    m = s.measure(latitude=36.8, longitude=10.2, altitude_m=20, voltage_v=14.8)
    assert (m.latitude, m.longitude, m.baro_altitude_m, m.voltage_v) == (36.8, 10.2, 20, 14.8)


def test_sensors_with_seed_are_reproducible():
    a = SensorSuite(seed=7)
    b = SensorSuite(seed=7)
    ma = a.measure(latitude=36.8, longitude=10.2, altitude_m=20, voltage_v=14.8)
    mb = b.measure(latitude=36.8, longitude=10.2, altitude_m=20, voltage_v=14.8)
    assert ma == mb
    assert ma.latitude != 36.8  # noise actually applied


def test_sensor_noise_is_bounded():
    s = SensorSuite(SensorConfig(gps_position_sigma_m=1.5), seed=1)
    m = s.measure(latitude=36.8, longitude=10.2, altitude_m=20, voltage_v=14.8)
    assert abs((m.latitude - 36.8) * 111_320) < 10  # well within 1.5 m sigma ~ 5 m


# --- twin wind / battery ----------------------------------------------------

def test_gust_is_time_varying_and_zero_by_default():
    calm = Environment()
    assert calm.wind_at(3) == 0.0
    gusty = Environment(wind_speed_mps=2, gust_amplitude_mps=3, gust_period_s=10)
    assert gusty.wind_at(0) == pytest.approx(2.0)
    assert gusty.wind_at(2.5) == pytest.approx(5.0)


def test_turbulence_seeded_reproducible():
    env = Environment(turbulence_sigma_mps=1.0, turbulence_seed=11)
    a = DigitalTwin(size_power_system(), environment=env)
    b = DigitalTwin(size_power_system(), environment=env)
    sa, sb = TwinState(90), TwinState(90)
    ra = a.simulate_segment(sa, FlightSegment(500, 15))
    rb = b.simulate_segment(sb, FlightSegment(500, 15))
    assert ra.ground_speed_mps == rb.ground_speed_mps


def test_battery_model_increases_energy_draw():
    budget = size_power_system()
    baseline = DigitalTwin(budget)
    realistic = DigitalTwin(budget, battery_model=BatteryModel(
        nominal_v=budget.voltage_v, capacity_ah=budget.capacity_ah,
        nominal_current_a=1.0,  # force heavy Peukert derating
    ))
    a = baseline.simulate_segment(TwinState(90), FlightSegment(1000, 15))
    b = realistic.simulate_segment(TwinState(90), FlightSegment(1000, 15))
    assert b.energy_wh > a.energy_wh


# --- value revisits ---------------------------------------------------------

def test_revisit_adds_coverage_without_removing_base():
    base = [Waypoint(36.8, 10.2, 20), Waypoint(36.801, 10.2, 20)]
    region = ValueRegion(36.8005, 10.2005, 30, 0.9)
    combined, selected = plan_revisits(base, [region], spacing_m=15)
    assert len(combined) > len(base)
    assert combined[:2] == base  # base route preserved in order
    assert len(selected) == 1


def test_low_value_regions_ignored():
    base = [Waypoint(36.8, 10.2, 20)]
    combined, selected = plan_revisits(base, [ValueRegion(36.8, 10.2, 30, 0.1)], spacing_m=15)
    assert combined == base and selected == []


def test_revisit_extra_distance_positive():
    base = [Waypoint(36.8, 10.2, 20), Waypoint(36.801, 10.2, 20)]
    combined, _ = plan_revisits(base, [ValueRegion(36.8005, 10.2, 30, 0.9)], spacing_m=15)
    assert revisit_extra_distance_m(base, combined) > 0


def test_value_region_validation():
    with pytest.raises(ValueError):
        ValueRegion(0, 0, 0, 0.5)
    with pytest.raises(ValueError):
        ValueRegion(0, 0, 30, -1)


# --- SITL export ------------------------------------------------------------

def test_qgc_wpl_format():
    items = [MissionItem(36.8, 10.2, 20), MissionItem(36.801, 10.2, 20)]
    text = to_qgc_wpl(items)
    lines = text.strip().splitlines()
    assert lines[0] == "QGC WPL 110"
    assert len(lines) == 4  # header + HOME + 2 waypoints
    assert len(lines[2].split("\t")) == 12
    assert "36.8000000" in lines[2]


def test_mavlink_json_scaling():
    text = to_mavlink_json([MissionItem(36.8, 10.2, 20)])
    data = json.loads(text)
    assert data["items"][0]["x"] == 368000000
    assert data["items"][0]["y"] == 102000000
    assert data["items"][0]["z"] == 20


def test_twin_flight_controller_adapter():
    twin = DigitalTwin(size_power_system())
    fc = TwinFlightController(twin, 15)
    fc.upload([MissionItem(36.8, 10.2, 20), MissionItem(36.801, 10.2, 20)])
    result = fc.execute()
    assert result["backend"] == "digital_twin"
    assert result["distance_m"] > 0
    assert result["success"] is True


def test_sitl_stub_raises_honestly():
    fc = SitlFlightController()
    with pytest.raises(NotImplementedError):
        fc.upload([])
    with pytest.raises(NotImplementedError):
        fc.execute()


# --- simulator / CLI integration -------------------------------------------

def test_baseline_simulation_numbers_unchanged():
    r = simulate_mission()
    assert r.success is True
    # Latitude-aware geometry corrected field width: ~54 m longer path than
    # the old fixed m/deg approximation, so energy rose from 47.3 to ~57.8 Wh.
    assert 54 <= r.energy_wh <= 62
    assert r.terminal_voltage_v == 14.8  # nominal, no sag model
    assert r.sensor_sample == {}
    assert r.revisit_regions == 0


def test_simulator_realism_fields_populate():
    r = simulate_mission(
        sensor_seed=3, sensor_config=SensorConfig(),
        battery_model=BatteryModel(nominal_v=14.8, capacity_ah=8.0),
        value_regions=[ValueRegion(36.8, 10.2, 30, 0.9)],
    )
    assert r.terminal_voltage_v is not None
    assert r.sensor_sample != {}
    assert r.revisit_regions == 1
    assert r.revisit_extra_m > 0


def test_cli_fly_json_with_new_options():
    assert main([
        "fly", "--json", "--seed", "5", "--sensor-noise",
        "--gust", "1.0", "--revisit-lat", "36.8005",
        "--revisit-lon", "10.2005",
    ]) == 0


def test_cli_export_wpl_and_json(tmp_path, capsys):
    wpl = tmp_path / "m.waypoints"
    mj = tmp_path / "m.json"
    assert main(["export", "--format", "wpl", "--output", str(wpl), "--width", "100", "--height", "100"]) == 0
    assert main(["export", "--format", "mavlink-json", "--output", str(mj), "--width", "100", "--height", "100"]) == 0
    assert wpl.read_text(encoding="utf-8").startswith("QGC WPL 110")
    assert json.loads(mj.read_text(encoding="utf-8"))["items"]
    assert "Exported" in capsys.readouterr().out
