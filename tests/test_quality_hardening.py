"""Production-hardening tests: MAVLink transport edges, CLI dispatch, validation.

All MAVLink tests use a fake pymavlink module installed in sys.modules -- no
network endpoint is contacted and no real autopilot is required.
"""

import sys
import types
from pathlib import Path

import pytest

from sylph.battery import BatteryModel, BatteryState, interpolate_ocv
from sylph.cli import configure_logging, main
from sylph.design import size_power_system
from sylph.mavlink_sitl import MavlinkSitlController, PymavlinkTransport
from sylph.navigation import distance_m
from sylph.optimizer import (
    optimal_grid_spacing,
    optimal_speed,
    optimize_route_2opt,
    path_length_m,
    route_length_m,
)
from sylph.sitl import MissionItem
from sylph.twin import DigitalTwin
from sylph.waypoints import Waypoint

# --- fake pymavlink ----------------------------------------------------------


class FakeConn:
    """Minimal connection double for PymavlinkTransport."""

    def __init__(self, responses=None, *, typeerror_once=False):
        self.target_system = 1
        self.target_component = 1
        self.responses = {k: list(v) for k, v in (responses or {}).items()}
        self.typeerror_once = typeerror_once
        self.sent = []
        self.closed = False
        self.waited = None

    def recv_match(self, type=None, blocking=False, timeout=None):
        keys = type if isinstance(type, (list, tuple)) else [type]
        for key in keys:
            seq = self.responses.get(key, [])
            if seq:
                return seq.pop(0)
        return None

    def wait_heartbeat(self, timeout=None):
        self.waited = timeout

    def close(self):
        self.closed = True


class FakeMav:
    def __init__(self, conn):
        self.conn = conn

    def mission_count_send(self, *args):
        self.conn.sent.append(("mission_count", args))

    def mission_item_int_send(self, *args):
        self.conn.sent.append(("mission_item", args))
        if self.conn.typeerror_once:
            self.conn.typeerror_once = False
            raise TypeError("older pymavlink: no mission_type parameter")

    def command_long_send(self, *args):
        self.conn.sent.append(("command_long", args))


def _install_fake_pymavlink(monkeypatch, responses=None, *, typeerror_once=False):
    conn = FakeConn(responses, typeerror_once=typeerror_once)
    mavutil = types.SimpleNamespace(
        mavlink=types.SimpleNamespace(
            MAV_MISSION_ACCEPTED=0,
            MAV_CMD_DO_SET_MODE=176,
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1,
            MAV_CMD_COMPONENT_ARM_DISARM=400,
            MAV_RESULT_ACCEPTED=0,
            MAV_MODE_FLAG_SAFETY_ARMED=128,
        ),
        mavlink_connection=lambda desc: conn,
    )
    monkeypatch.setitem(sys.modules, "pymavlink", types.SimpleNamespace(mavutil=mavutil))
    conn.mav = FakeMav(conn)
    t = PymavlinkTransport("127.0.0.1", 5762)
    t.connect(timeout_s=3.0)
    assert t._mav is not None  # noqa: SLF001 - test double introspection
    return t, conn


def _req(seq):
    return types.SimpleNamespace(seq=seq)


def _ack(**overrides):
    base = {
        "type": 0,
        "command": 400,
        "result": 0,
        "base_mode": 128,
        "get_type": lambda: "SYS_STATUS",
        "lat": 368000000,
        "lon": 102000000,
        "relative_alt": 20000,
        "groundspeed": 12.5,
        "battery_remaining": 55,
    }
    base.update(overrides)
    return types.SimpleNamespace(**base)


# --- PymavlinkTransport -----------------------------------------------------


def test_pymavlink_mission_upload_with_legacy_fallback(monkeypatch):
    items = [MissionItem(36.8, 10.2, 20), MissionItem(36.801, 10.2, 20)]
    responses = {
        "MISSION_REQUEST_INT": [_req(0), _req(1)],
        "MISSION_ACK": [_ack()],
    }
    t, conn = _install_fake_pymavlink(monkeypatch, responses, typeerror_once=True)
    t.send_mission(items)
    kinds = [k for k, _ in conn.sent]
    # First item's send raises TypeError, then succeeds via the legacy branch.
    assert kinds.count("mission_item") == 3
    assert conn.waited == 3.0
    assert conn.sent[0][1][2] == 2  # mission_count = len(items)


def test_pymavlink_mission_upload_timeout(monkeypatch):
    t, _ = _install_fake_pymavlink(monkeypatch)
    with pytest.raises(TimeoutError, match="MISSION_REQUEST_INT"):
        t.send_mission([MissionItem(36.8, 10.2, 20)])


def test_pymavlink_mission_upload_rejected(monkeypatch):
    responses = {
        "MISSION_REQUEST_INT": [_req(0)],
        "MISSION_ACK": [_ack(type=1)],
    }
    t, _ = _install_fake_pymavlink(monkeypatch, responses)
    with pytest.raises(RuntimeError, match="rejected"):
        t.send_mission([MissionItem(36.8, 10.2, 20)])


@pytest.mark.parametrize("mode,expected", [("auto", 3), ("guided", 4), ("7", 7)])
def test_pymavlink_set_mode(monkeypatch, mode, expected):
    t, conn = _install_fake_pymavlink(monkeypatch)
    t.set_mode(mode)
    assert conn.sent[-1][1][5] == expected


def test_pymavlink_arm_success_and_failures(monkeypatch):
    t, conn = _install_fake_pymavlink(monkeypatch, {"COMMAND_ACK": [_ack()]})
    t.arm()
    assert conn.sent[-1][0] == "command_long"

    t2, _ = _install_fake_pymavlink(monkeypatch)
    with pytest.raises(RuntimeError, match="no arm"):
        t2.arm()

    t3, _ = _install_fake_pymavlink(monkeypatch, {"COMMAND_ACK": [_ack(command=1)]})
    with pytest.raises(RuntimeError, match="no arm"):
        t3.arm()

    t4, _ = _install_fake_pymavlink(monkeypatch, {"COMMAND_ACK": [_ack(result=1)]})
    with pytest.raises(RuntimeError, match="refused arm"):
        t4.arm()


def test_pymavlink_poll_telemetry_all_message_types(monkeypatch):
    gpi = _ack(get_type=lambda: "GLOBAL_POSITION_INT")
    hud = _ack(get_type=lambda: "VFR_HUD")
    sys_status = _ack(get_type=lambda: "SYS_STATUS", battery_remaining=42)
    responses = {
        "GLOBAL_POSITION_INT": [gpi],
        "VFR_HUD": [hud],
        "SYS_STATUS": [sys_status],
        "HEARTBEAT": [_ack(base_mode=128) for _ in range(3)],
    }
    t, _ = _install_fake_pymavlink(monkeypatch, responses)
    a = t.poll_telemetry()
    b = t.poll_telemetry()
    c = t.poll_telemetry()
    assert (a.latitude, a.altitude_m, a.armed) == (36.8, 20.0, True)
    assert (b.ground_speed_mps, b.battery_pct) == (12.5, 0.0)
    assert c.battery_pct == 42 and c.armed is True
    assert t.poll_telemetry() is None


def test_pymavlink_transport_guard_and_close(monkeypatch):
    t, conn = _install_fake_pymavlink(monkeypatch)
    t.close()
    assert conn.closed and t._conn is None  # noqa: SLF001
    t.close()  # idempotent
    with pytest.raises(RuntimeError):
        t.send_mission([])
    with pytest.raises(RuntimeError):
        t.set_mode("auto")
    with pytest.raises(RuntimeError):
        t.arm()
    with pytest.raises(RuntimeError):
        t.poll_telemetry()


def test_mavlink_controller_execute_with_fake_transport(monkeypatch):
    t, conn = _install_fake_pymavlink(
        monkeypatch,
        {
            "MISSION_REQUEST_INT": [_req(0)],
            "MISSION_ACK": [_ack()],
            "COMMAND_ACK": [_ack()],
            "GLOBAL_POSITION_INT": [_ack(get_type=lambda: "GLOBAL_POSITION_INT")],
        },
    )
    fc = MavlinkSitlController(t)
    fc.upload([MissionItem(36.8, 10.2, 20)])
    result = fc.execute(max_samples=3)
    assert result["success"] is True and result["samples"] == 1
    assert result["last_state"]["latitude"] == 36.8
    assert conn.sent[-1][0] == "command_long"


# --- optimizer validation ---------------------------------------------------


def test_optimal_speed_validation():
    twin = DigitalTwin(size_power_system())
    with pytest.raises(ValueError, match="min_mps"):
        optimal_speed(twin, min_mps=0)
    with pytest.raises(ValueError, match="min_mps"):
        optimal_speed(twin, min_mps=12, max_mps=4)


def test_optimal_grid_spacing_validation_and_clamp():
    with pytest.raises(ValueError, match="camera_fov"):
        optimal_grid_spacing(20, camera_fov_deg=0)
    with pytest.raises(ValueError, match="camera_fov"):
        optimal_grid_spacing(20, camera_fov_deg=180)
    with pytest.raises(ValueError, match="overlap"):
        optimal_grid_spacing(20, overlap_fraction=1.0)
    with pytest.raises(ValueError, match="overlap"):
        optimal_grid_spacing(20, overlap_fraction=-0.1)
    assert optimal_grid_spacing(1) == 5.0  # min_spacing clamp


def test_route_length_and_2opt_edges():
    pts = [(0.0, 0.0), (0.0, 1.0), (1.0, 1.0), (1.0, 0.0)]
    assert route_length_m([]) == 0.0
    assert route_length_m(pts[:1]) == 0.0
    close_edge = distance_m(pts[-1][0], pts[-1][1], pts[0][0], pts[0][1])
    assert route_length_m(pts) == pytest.approx(path_length_m(pts) + close_edge)
    short = pts[:3]
    route, length = optimize_route_2opt(short)
    assert route == short and length == pytest.approx(path_length_m(short))
    crossing = [(0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)]
    route, length = optimize_route_2opt(crossing)
    assert length < path_length_m(crossing)
    assert route[0] == crossing[0] and route[-1] == crossing[-1]


def test_plan_revisits_rejects_zero_spacing():
    from sylph.optimizer import plan_revisits

    with pytest.raises(ValueError, match="spacing_m"):
        plan_revisits([Waypoint(36.8, 10.2, 20)], [], spacing_m=0)


# --- battery validation edges -----------------------------------------------


def test_battery_validation_edges():
    with pytest.raises(ValueError, match="empty OCV"):
        interpolate_ocv(0.5, [])
    with pytest.raises(ValueError, match="positive"):
        BatteryModel(nominal_v=0)
    with pytest.raises(ValueError, match="resistance"):
        BatteryModel(internal_resistance_ohm=-1)
    with pytest.raises(ValueError, match="peukert"):
        BatteryModel(peukert_k=0.5)
    with pytest.raises(ValueError, match="positive"):
        BatteryModel(nominal_current_a=0)


def test_battery_zero_resistance_and_overload():
    zero_r = BatteryModel(internal_resistance_ohm=0.0)
    st, volt = zero_r.draw(BatteryState(100, 0.5), 100, 3600)
    assert volt == pytest.approx(zero_r.ocv_v(0.5))
    assert st.energy_remaining_wh < 100

    overload = BatteryModel(internal_resistance_ohm=1e6)
    st, volt = overload.draw(BatteryState(100, 0.5), 500, 1)
    assert volt == pytest.approx(overload.ocv_v(0.5) / 2)


def test_battery_disabled_ocv_and_negative_draw():
    legacy = BatteryModel(enabled=False)
    assert legacy.ocv_v(0.3) == legacy.nominal_v
    with pytest.raises(ValueError, match="non-negative"):
        legacy.draw(BatteryState(100), -1, 1)
    with pytest.raises(ValueError, match="non-negative"):
        legacy.draw(BatteryState(100), 1, -1)


# --- CLI dispatch paths -----------------------------------------------------


def test_cli_design_optimize_compare_verbose(capsys):
    assert main(["design"]) == 0
    assert main(["optimize", "--altitude", "25"]) == 0
    assert main(["compare"]) == 0
    configure_logging(verbose=True)
    assert main(["design"]) == 0
    out = capsys.readouterr().out
    assert "BOM mass" in out and "Optimal" in out


def test_cli_montecarlo_json_and_scenario_output(tmp_path):
    assert main(["montecarlo", "--runs", "2", "--json"]) == 0
    out = tmp_path / "scenario.json"
    assert main(["scenario", "--name", "wind_gust", "--seed", "3", "--output", str(out)]) == 0
    assert Path(out).exists() and "frames" in out.read_text(encoding="utf-8")


def test_cli_fly_human_output_with_realism(tmp_path):
    assert main([
        "fly", "--seed", "3", "--battery-realism", "--sensor-noise",
        "--revisit-lat", "36.8005", "--revisit-lon", "10.2005",
    ]) == 0
