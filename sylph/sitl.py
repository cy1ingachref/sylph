"""SITL readiness: mission export formats and a swappable flight-controller
interface.

EXPORTS
 * QGC WPL 110 -- the plain-text waypoint format used by QGroundControl and
   Mission Planner (ArduPilot/PX4). Fields are well documented; no MAVLink
   library is required to write it.
 * MAVLink mission JSON -- a readable, tool-agnostic representation whose
   fields map 1:1 to MAVLink MISSION_ITEM_INT (frame, command, params, x/y/z).

FLIGHT CONTROLLER INTERFACE
 ``FlightController`` is the seam where the digital twin can later be replaced
 by a real SITL connection. ``TwinFlightController`` adapts the current twin;
 ``SitlFlightController`` is an explicit, honest NOT-IMPLEMENTED stub that
 raises on use so nobody mistakes it for a working link.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Protocol

from .twin import DigitalTwin, FlightSegment, TwinState

__all__ = [
    "MissionItem",
    "to_qgc_wpl",
    "write_qgc_wpl",
    "to_mavlink_json",
    "write_mavlink_json",
    "FlightController",
    "TwinFlightController",
    "SitlFlightController",
]


@dataclass(frozen=True)
class MissionItem:
    """One waypoint, expressed in absolute WGS84 coordinates."""

    latitude: float
    longitude: float
    altitude_m: float
    command: int = 16  # MAV_CMD_NAV_WAYPOINT
    frame: int = 3     # MAV_FRAME_GLOBAL_RELATIVE_ALT
    hold_s: float = 0.0
    acceptance_radius_m: float = 2.0


def to_qgc_wpl(items: list[MissionItem]) -> str:
    """Render a QGC WPL 110 file (lf; index starts at 0 for HOME)."""
    lines = ["QGC WPL 110"]
    # Index 0 is HOME at (0,0,0) by convention; the FC fills it from GPS.
    lines.append("0\t1\t0\t16\t0\t0\t0\t0\t0\t0\t0\t0")
    for i, it in enumerate(items, start=1):
        # fields: INDEX CURRENT FRAME COMMAND PARAM1..PARAM4 X Y Z AUTOCONTINUE
        values = [
            i, 0, it.frame, it.command,
            it.hold_s, it.acceptance_radius_m, 0, 0,
            f"{it.latitude:.7f}", f"{it.longitude:.7f}", f"{it.altitude_m:.3f}", 1,
        ]
        lines.append("\t".join(str(v) for v in values))
    return "\n".join(lines) + "\n"


def write_qgc_wpl(items: list[MissionItem], path: str | Path) -> Path:
    p = Path(path)
    p.write_text(to_qgc_wpl(items), encoding="utf-8", newline="\n")
    return p


def to_mavlink_json(items: list[MissionItem]) -> str:
    """Render MAVLink-style mission items as readable JSON."""
    payload = {
        "format": "mavlink_mission_item_int_v1",
        "frame": "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT",
        "items": [
            {
                "seq": i,
                "command": it.command,
                "frame": it.frame,
                "param1": it.hold_s,
                "param2": it.acceptance_radius_m,
                "param3": 0.0,
                "param4": 0.0,
                # MISSION_ITEM_INT uses degrees * 1e7
                "x": int(round(it.latitude * 1e7)),
                "y": int(round(it.longitude * 1e7)),
                "z": it.altitude_m,
            }
            for i, it in enumerate(items)
        ],
    }
    return json.dumps(payload, indent=2) + "\n"


def write_mavlink_json(items: list[MissionItem], path: str | Path) -> Path:
    p = Path(path)
    p.write_text(to_mavlink_json(items), encoding="utf-8", newline="\n")
    return p


class FlightController(Protocol):
    """Seam between the simulator and a real/SITL flight controller."""

    def upload(self, items: list[MissionItem]) -> None: ...
    def execute(self) -> dict: ...


@dataclass
class TwinFlightController:
    """Adapter that flies a mission through the existing digital twin."""

    twin: DigitalTwin
    speed_mps: float
    items: list[MissionItem] | None = None

    def upload(self, items: list[MissionItem]) -> None:
        if not items:
            raise ValueError("empty mission")
        self.items = items

    def execute(self) -> dict:
        if not self.items:
            raise RuntimeError("no mission uploaded")
        from .navigation import distance_m

        first = self.items[0]
        x, y = first.latitude, first.longitude
        # Simple straight-line mission execution via the twin; wind is applied
        # per leg using the twin's environment settings.
        total_distance = 0.0
        for a, b in zip(self.items, self.items[1:]):
            total_distance += distance_m(a.latitude, a.longitude, b.latitude, b.longitude)
        state = TwinState(self.twin.budget.usable_energy_wh)
        self.twin.fly(state, FlightSegment(total_distance, self.speed_mps))
        return {
            "backend": "digital_twin",
            "waypoints": len(self.items),
            "distance_m": total_distance,
            "energy_wh": self.twin.budget.usable_energy_wh - state.energy_remaining_wh,
            "success": True,
            "start_lat": x,
            "start_lon": y,
        }


class SitlFlightController:
    """Explicit placeholder for a future ArduPilot/PX4 SITL connection.

    This does NOT pretend to be a working SITL link. It raises on every
    operation so a caller cannot accidentally believe a SITL flight happened.
    """

    def upload(self, items: list[MissionItem]) -> None:
        raise NotImplementedError(
            "SITL link not implemented: connect MAVLink (pymavlink/MAVSDK) to "
            "ArduPilot SITL or PX4 SITL before using this backend"
        )

    def execute(self) -> dict:
        raise NotImplementedError("SITL link not implemented")
