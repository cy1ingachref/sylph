"""PR 4 tests: replay log, scenario fixtures, 3D demo data contract."""

import json

import pytest

from sylph.replay import SCHEMA_VERSION, MissionEvent, ReplayLog, TelemetryFrame, to_demo_payload
from sylph.scenarios import run_scenario


def _log():
    log = ReplayLog("test")
    log.waypoints = [{"id": "WP1", "lat_deg": 36.8, "lon_deg": 10.2, "alt_m": 20}]
    log.add_frame(TelemetryFrame(0.0, 36.8, 10.2, 20.0, 0.0, 0.0, 90.0, 85.0, 0))
    log.add_event(MissionEvent(0.0, "takeoff", "start"))
    return log


def test_schema_version_is_one():
    assert SCHEMA_VERSION == 1


def test_demo_payload_contract():
    p = _log().to_demo_payload()
    assert p["schema_version"] == 1
    assert p["mode"] == "simulation"
    assert "NO REAL FLIGHT CONTROL" in p["simulation_notice"]
    assert p["vehicle"]["position"]["lat_deg"] == 36.8
    assert p["mission"]["waypoints"][0]["id"] == "WP1"
    assert p["frames"][0]["t_s"] == 0.0
    assert p["events"][0]["kind"] == "takeoff"


def test_empty_log_cannot_be_exported():
    with pytest.raises(ValueError):
        to_demo_payload(ReplayLog("empty"))


def test_replay_jsonl_roundtrip(tmp_path):
    p = _log().write_jsonl(tmp_path / "replay.jsonl")
    lines = p.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 3
    assert json.loads(lines[0])["schema_version"] == 1
    assert "frame" in json.loads(lines[1])
    assert "event" in json.loads(lines[2])


def test_normal_scenario_is_deterministic():
    a = run_scenario("normal", seed=3)
    b = run_scenario("normal", seed=3)
    assert [f.t_s for f in a.frames] == [f.t_s for f in b.frames]
    assert [e.kind for e in a.events] == [e.kind for e in b.events]
    assert a.detections == b.detections


def test_normal_scenario_reaches_end():
    log = run_scenario("normal")
    assert any(e.kind == "complete" for e in log.events)
    assert len(log.frames) >= 3
    assert log.frames[-1].t_s > log.frames[0].t_s


def test_low_battery_scenario_triggers_rtl():
    log = run_scenario("low_battery")
    kinds = [e.kind for e in log.events]
    assert "battery_low" in kinds and "rtl" in kinds


def test_wind_scenario_differs_from_calm():
    calm = run_scenario("normal", seed=1)
    windy = run_scenario("wind_gust", seed=1, wind_mps=4, gust_amplitude_mps=2)
    assert windy.frames[-1].t_s != calm.frames[-1].t_s
