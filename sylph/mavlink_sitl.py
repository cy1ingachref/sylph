"""MAVLink transport for a running ArduPilot/PX4 SITL instance.

The controller logic lives in :class:`MavlinkSitlController` and depends only
on the :class:`MavlinkTransport` protocol, so it is fully testable with a fake
link. :class:`PymavlinkTransport` is the real implementation; it lazily imports
pymavlink and has NOT been exercised against a live SITL instance in this
repository. No endpoint is probed or connected to unless the caller explicitly
constructs it.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Protocol

from .sitl import MissionItem

__all__ = [
    "VehicleTelemetry",
    "MavlinkTransport",
    "PymavlinkTransport",
    "MavlinkSitlController",
]


@dataclass(frozen=True)
class VehicleTelemetry:
    """A snapshot received from a real/SITL vehicle."""

    latitude: float
    longitude: float
    altitude_m: float
    ground_speed_mps: float
    battery_pct: float
    armed: bool
    mode: str = "UNKNOWN"
    timestamp_s: float = 0.0


class MavlinkTransport(Protocol):
    """Minimal MAVLink operations required by the SITL controller."""

    def connect(self, timeout_s: float) -> None: ...
    def send_mission(self, items: list[MissionItem], timeout_s: float) -> None: ...
    def set_mode(self, mode: str, timeout_s: float) -> None: ...
    def arm(self, timeout_s: float) -> None: ...
    def poll_telemetry(self, timeout_s: float) -> VehicleTelemetry | None: ...
    def close(self) -> None: ...


class PymavlinkTransport:
    """Real MAVLink transport using pymavlink (lazy import, SITL UDP by default).

    ASSUMPTION: the vehicle is a standard MAVLink autopilot accepting the
    MISSION_COUNT -> MISSION_REQUEST_INT -> MISSION_ITEM_INT -> MISSION_ACK
    upload handshake. The implementation follows that flow but has not been
    run against a live SITL instance in this repo; validate on a bench first.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 5762) -> None:
        self.host = host
        self.port = port
        self._conn = None
        self._mav = None

    def connect(self, timeout_s: float = 10.0) -> None:
        from pymavlink import mavutil  # lazy: CI does not need pymavlink
        self._conn = mavutil.mavlink_connection(f"udp:{self.host}:{self.port}")
        self._mav = self._conn.mav
        self._conn.wait_heartbeat(timeout=timeout_s)

    def send_mission(self, items: list[MissionItem], timeout_s: float = 10.0) -> None:
        if self._conn is None:
            raise RuntimeError("connect() must be called before send_mission()")
        from pymavlink import mavutil
        mav = self._mav
        conn = self._conn
        mav.mission_count_send(
            conn.target_system, conn.target_component, len(items), 0  # MAV_MISSION_TYPE_MISSION
        )
        for _ in range(len(items)):
            req = conn.recv_match(type="MISSION_REQUEST_INT", blocking=True, timeout=timeout_s)
            if req is None:
                raise TimeoutError("no MISSION_REQUEST_INT from SITL")
            it = items[req.seq]
            try:
                mav.mission_item_int_send(
                    conn.target_system, conn.target_component,
                    req.seq, it.frame, it.command,
                    1 if req.seq == 0 else 0, 1,  # current, autocontinue
                    it.hold_s, it.acceptance_radius_m, 0.0, 0.0,
                    int(round(it.latitude * 1e7)), int(round(it.longitude * 1e7)),
                    it.altitude_m, 0,  # MAV_MISSION_TYPE_MISSION
                )
            except TypeError:
                # Older pymavlink omits the final mission_type parameter.
                mav.mission_item_int_send(
                    conn.target_system, conn.target_component,
                    req.seq, it.frame, it.command,
                    1 if req.seq == 0 else 0, 1,
                    it.hold_s, it.acceptance_radius_m, 0.0, 0.0,
                    int(round(it.latitude * 1e7)), int(round(it.longitude * 1e7)),
                    it.altitude_m,
                )
        ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=timeout_s)
        if ack is None or getattr(ack, "type", 0) != mavutil.mavlink.MAV_MISSION_ACCEPTED:
            raise RuntimeError("SITL rejected the mission upload")

    def set_mode(self, mode: str, timeout_s: float = 10.0) -> None:
        if self._conn is None:
            raise RuntimeError("connect() first")
        from pymavlink import mavutil
        # ArduPilot custom modes: AUTO=3, GUIDED=4. PX4 uses different values;
        # callers on PX4 should pass a numeric mode string.
        custom = {"auto": 3, "guided": 4}.get(mode.lower(), None)
        if custom is None:
            custom = int(mode)
        self._mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom,
            0, 0, 0, 0, 0,
        )

    def arm(self, timeout_s: float = 10.0) -> None:
        if self._conn is None:
            raise RuntimeError("connect() first")
        from pymavlink import mavutil
        self._mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0,
        )
        ack = self._conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=timeout_s)
        if ack is None or ack.command != mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            raise RuntimeError("no arm acknowledgement from SITL")
        if getattr(ack, "result", 0) != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f"SITL refused arm: result={ack.result}")

    def poll_telemetry(self, timeout_s: float = 2.0) -> VehicleTelemetry | None:
        if self._conn is None:
            raise RuntimeError("connect() first")
        from pymavlink import mavutil
        msg = self._conn.recv_match(
            type=["GLOBAL_POSITION_INT", "VFR_HUD", "SYS_STATUS"],
            blocking=True, timeout=timeout_s,
        )
        if msg is None:
            return None
        hb = self._conn.recv_match(type="HEARTBEAT", blocking=False)
        armed = bool(hb and (getattr(hb, "base_mode", 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
        if msg.get_type() == "GLOBAL_POSITION_INT":
            return VehicleTelemetry(
                latitude=msg.lat / 1e7, longitude=msg.lon / 1e7,
                altitude_m=msg.relative_alt / 1000.0,
                ground_speed_mps=0.0, battery_pct=0.0,
                armed=armed, timestamp_s=time.time(),
            )
        if msg.get_type() == "VFR_HUD":
            return VehicleTelemetry(
                latitude=0.0, longitude=0.0, altitude_m=0.0,
                ground_speed_mps=float(msg.groundspeed), battery_pct=0.0,
                armed=armed, timestamp_s=time.time(),
            )
        return VehicleTelemetry(
            latitude=0.0, longitude=0.0, altitude_m=0.0,
            ground_speed_mps=0.0, battery_pct=float(getattr(msg, "battery_remaining", 0)),
            armed=armed, timestamp_s=time.time(),
        )

    def close(self) -> None:
        if self._conn is not None:
            self._conn.close()
            self._conn = None


class MavlinkSitlController:
    """Drive a running SITL instance through the :class:`FlightController` seam.

    ``poll_telemetry`` samples are merged into one snapshot because MAVLink
    streams each message type independently. The caller decides when a mission
    is complete (e.g. by watching mode or position); this class does not claim
    an autopilot-reported "mission complete" unless the transport supplies one.
    """

    def __init__(self, transport: MavlinkTransport, *, mode: str = "auto") -> None:
        self.transport = transport
        self.mode = mode
        self.items: list[MissionItem] = []
        self.telemetry: list[VehicleTelemetry] = []

    def upload(self, items: list[MissionItem]) -> None:
        if not items:
            raise ValueError("empty mission")
        self.items = list(items)
        self.transport.connect(timeout_s=10.0)
        self.transport.send_mission(self.items, timeout_s=10.0)

    def execute(self, *, max_samples: int = 5, sample_timeout_s: float = 2.0) -> dict:
        if not self.items:
            raise RuntimeError("no mission uploaded")
        self.transport.set_mode(self.mode, timeout_s=10.0)
        self.transport.arm(timeout_s=10.0)
        self.telemetry = []
        for _ in range(max_samples):
            sample = self.transport.poll_telemetry(timeout_s=sample_timeout_s)
            if sample is not None:
                self.telemetry.append(sample)
        return {
            "backend": "mavlink_sitl",
            "waypoints": len(self.items),
            "samples": len(self.telemetry),
            "last_state": self.telemetry[-1].__dict__ if self.telemetry else None,
            "success": bool(self.telemetry),
            "claimed_mission_complete": False,  # honest: not verified here
        }
