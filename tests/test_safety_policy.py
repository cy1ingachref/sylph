"""P0 regression tests: explicit mode policy and pending-command safety."""

import pytest

from sylph.mission import Command, CommandInterpreter
from sylph.safety import (
    OperatingMode,
    PreflightError,
    preflight,
    require_arm_authorization,
)


def safe_base(**overrides):
    args = dict(
        firmware_ok=True, gps_fix=True, failsafes_tested=True,
        geofence_set=True, battery_ok=True,
    )
    args.update(overrides)
    return args


# --- P0.1 preflight mode policy ---------------------------------------------

def test_simulation_passes_checks():
    r = preflight(mode=OperatingMode.SIMULATION, **safe_base())
    assert r.checks_passed and r.safe and r.mode_authorized
    assert r.hardware_arm_permitted_now is False


def test_sitl_requires_explicit_authorization():
    denied = preflight(mode=OperatingMode.SITL, **safe_base())
    assert denied.checks_passed is True
    assert denied.safe is False
    assert "sitl_not_authorized" in denied.failures
    allowed = preflight(mode=OperatingMode.SITL, sitl_authorized=True, **safe_base())
    assert allowed.safe is True
    assert allowed.hardware_arm_permitted_now is False


def test_hardware_never_authorized_by_checks_alone():
    r = preflight(mode=OperatingMode.HARDWARE, **safe_base(telemetry_fresh=True))
    assert r.checks_passed is True
    assert r.safe is False
    assert r.hardware_arm_permitted_now is False
    with pytest.raises(PreflightError, match="hardware_arm_not_permitted"):
        require_arm_authorization(r)


def test_hardware_requires_permission_and_interlocks():
    r = preflight(
        mode=OperatingMode.HARDWARE, hardware_arm_permitted=True,
        interlocks_ok=True, **safe_base()
    )
    assert r.hardware_arm_permitted_now is True
    assert r.safe is True  # explicit hardware, not accidental simulation


def test_all_checks_pass_but_safety_check_fails():
    r = preflight(**safe_base(gps_fix=False))
    assert r.checks_passed is False
    assert "gps_fix" in r.failures


def test_missing_or_stale_telemetry_fails_closed():
    stale = preflight(**safe_base(telemetry_stale=True))
    assert stale.safe is False
    assert "telemetry_stale_or_missing" in stale.failures
    missing = preflight(**safe_base(telemetry_fresh=False))
    assert missing.safe is False


def test_invalidation_after_arm_fails_closed():
    r = preflight(mode=OperatingMode.SITL, sitl_authorized=True, **safe_base())
    assert r.safe is True
    after = r.invalidated("gps lost")
    assert after.safe is False
    assert any("invalidated_after_arm" in f for f in after.failures)


def test_legacy_simulation_flag_still_works():
    r = preflight(simulation=True, **safe_base())
    assert r.safe is True
    legacy_hw = preflight(simulation=False, **safe_base())
    assert legacy_hw.safe is False  # no implicit hardware authorization


def test_contradictory_mode_and_flag_rejected():
    with pytest.raises(ValueError):
        preflight(simulation=True, mode=OperatingMode.SITL, **safe_base())
    with pytest.raises(ValueError):
        preflight(mode="bogus", **safe_base())


def test_arm_requires_authorization():
    with pytest.raises(PreflightError):
        preflight(mode=OperatingMode.SITL, arm=True, **safe_base())


# --- P0.2 pending command state ---------------------------------------------

def test_valid_then_invalid_cannot_leave_hidden_command():
    executed = []
    interp = CommandInterpreter(executed.append)
    assert interp.submit("scout the north field").kind == "scout"
    assert interp.submit("%%%").kind == "unknown"
    assert interp.pending is None
    assert interp.confirm() is False
    assert executed == []


def test_pending_command_expires():
    now = [0.0]
    executed = []
    interp = CommandInterpreter(
        executed.append, expiry_s=5.0, clock=lambda: now[0]
    )
    interp.submit("scout the north field")
    now[0] = 6.0
    assert interp.confirm() is False
    assert executed == []


def test_replacement_command_invalidates_old_confirmation():
    executed = []
    interp = CommandInterpreter(executed.append)
    interp.submit("scout the north field")
    second = interp.submit("return to launch")
    assert second.kind == "rtl"
    assert interp.confirm() is True
    assert executed == [Command("rtl")]  # NOT the old scout command


def test_token_binding_prevents_wrong_confirmation():
    executed = []
    interp = CommandInterpreter(executed.append)
    interp.submit("scout the north field")
    token = interp.pending.token
    assert interp.confirm(token="wrong") is False
    assert executed == []
    assert interp.confirm(token=token) is True


def test_utterance_binding_prevents_different_command():
    executed = []
    interp = CommandInterpreter(executed.append)
    interp.submit("scout the north field")
    assert interp.confirm(utterance="return to launch") is False
    assert executed == []
    assert interp.confirm(utterance="  SCOUT   THE NORTH FIELD ") is True


def test_repeated_confirmation_is_single_use():
    executed = []
    interp = CommandInterpreter(executed.append)
    interp.submit("scout the north field")
    assert interp.confirm() is True
    assert interp.confirm() is False
    assert len(executed) == 1


def test_safety_check_rerun_before_execution():
    flags = {"safe": False}
    executed = []
    interp = CommandInterpreter(executed.append, safety_check=lambda: flags["safe"])
    interp.submit("scout the north field")
    assert interp.confirm() is False  # unsafe at confirmation time
    assert executed == []
    interp.submit("scout the north field")
    flags["safe"] = True
    assert interp.confirm() is True
    assert len(executed) == 1


def test_interpreter_rejects_bad_expiry():
    with pytest.raises(ValueError):
        CommandInterpreter(lambda c: None, expiry_s=0)
