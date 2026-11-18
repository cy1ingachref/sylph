"""Phase 0 tests: safety / preflight gate."""

import pytest

from sylph.safety import PreflightError, preflight


def test_all_checks_pass_when_conditions_met():
    report = preflight(
        firmware_ok=True,
        gps_fix=True,
        failsafes_tested=True,
        geofence_set=True,
        battery_ok=True,
    )
    assert report.safe is True
    assert report.failures == []


def test_unsafe_report_lists_failures():
    report = preflight(
        firmware_ok=True,
        gps_fix=False,
        failsafes_tested=False,
        geofence_set=True,
        battery_ok=True,
    )
    assert report.safe is False
    assert report.failures == ["failsafes_tested", "gps_fix"]


def test_missing_geofence_blocks_safety():
    report = preflight(
        firmware_ok=True,
        gps_fix=True,
        failsafes_tested=True,
        geofence_set=False,
        battery_ok=True,
    )
    assert report.safe is False
    assert "geofence_set" not in report.checks  # tracked as separate field
    assert report.geofence_set is False


def test_arm_refused_when_unsafe():
    with pytest.raises(PreflightError, match="refusing to arm"):
        preflight(
            firmware_ok=True,
            gps_fix=False,
            failsafes_tested=True,
            geofence_set=True,
            battery_ok=True,
            arm=True,
        )


def test_arm_allowed_when_safe():
    report = preflight(
        firmware_ok=True,
        gps_fix=True,
        failsafes_tested=True,
        geofence_set=True,
        battery_ok=True,
        arm=True,
    )
    assert report.safe is True
