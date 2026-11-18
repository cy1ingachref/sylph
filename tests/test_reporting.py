"""Phase 4 tests: reporting, sensor fusion, profiles."""

from sylph.reporting import (
    DEFAULT_PROFILE,
    altitude_hold_correction,
    build_report,
    fuse_altitude,
)
from sylph.vision import Detection


def test_report_empty():
    r = build_report([])
    assert r.total_detections == 0
    assert r.infection_rate == 0.0
    assert "No detections" in r.recommendations[0]


def test_report_high_infection():
    detections = [
        Detection("late_blight", 0.9, 1, 2, "t", severity=0.1),
        Detection("healthy", 0.95, 1, 2, "t", severity=0.05),
        Detection("leaf_rust", 0.8, 1, 2, "t", severity=0.2),
    ]
    r = build_report(detections)
    assert r.total_detections == 3
    assert r.infected_detections == 2
    assert r.infection_rate == 2 / 3
    assert any("High infection" in rec for rec in r.recommendations)


def test_report_text_contains_summary():
    r = build_report([Detection("healthy", 0.9, 1, 2, "t")])
    text = r.to_text()
    assert "Sylph Mission Report" in text
    assert "Infection rate:" in text


def test_fuse_altitude_with_and_without_tof():
    assert fuse_altitude(20, None) == 20
    assert fuse_altitude(20, 22) == 21


def test_altitude_hold_deadband():
    assert altitude_hold_correction(20.1, 20.0) == 0.0
    assert altitude_hold_correction(18.0, 20.0) == 2.0


def test_default_profile_values():
    assert DEFAULT_PROFILE.target_altitude_m == 20.0
    assert 0 < DEFAULT_PROFILE.safety_margin < 1
