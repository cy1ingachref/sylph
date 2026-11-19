"""Phase 3: high-level mission autonomy -- voice commands, grid scouting,
live detection map, and battery-aware planning.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable

from .link import VehicleState
from .navigation import distance_m, estimate_return_energy_wh
from .waypoints import Waypoint

__all__ = [
    "Command",
    "parse_command",
    "CommandInterpreter",
    "lawnmower_pattern",
    "LiveDetectionMap",
    "plan_mission",
    "MissionPlan",
]


@dataclass(frozen=True)
class Command:
    """A parsed high-level order."""

    kind: str  # "scout" | "rtl" | "hold" | "unknown"
    target: str = ""


def parse_command(utterance: str) -> Command:
    """Parse a speech-to-text utterance into a high-level command.

    Deterministic and offline: matches a small vocabulary with keywords. Real
    STT (on-device or companion computer) feeds text into this function.
    """
    text = " ".join(utterance.lower().strip().split())
    if not text:
        return Command("unknown")
    if "return" in text or "come home" in text or "rtl" in text:
        return Command("rtl")
    if "hold" in text or "hover" in text:
        return Command("hold")
    if "scout" in text or "survey" in text or "inspect" in text:
        for direction in ("north", "south", "east", "west"):
            if direction in text:
                return Command("scout", direction)
        return Command("scout", "current")
    return Command("unknown")


class CommandInterpreter:
    """Gate that requires explicit confirmation before executing any order.

    This is the fail-safe: recognizing a command never arms or moves the drone
    without a separate ``confirm`` call. Disallowed commands (e.g. scouting
    without a geofence) are rejected regardless of confirmation.
    """

    def __init__(self, executor: Callable[[Command], None]) -> None:
        self.executor = executor
        self.pending: Command | None = None
        self.executed: list[Command] = []

    def submit(self, utterance: str) -> Command:
        cmd = parse_command(utterance)
        if cmd.kind != "unknown":
            self.pending = cmd
        return cmd

    def confirm(self, *, allowed: bool = True) -> bool:
        if self.pending is None:
            return False
        cmd = self.pending
        self.pending = None
        if not allowed or cmd.kind == "unknown":
            return False
        self.executor(cmd)
        self.executed.append(cmd)
        return True


def lawnmower_pattern(
    center_lat: float,
    center_lon: float,
    *,
    width_m: float,
    height_m: float,
    spacing_m: float,
    altitude_m: float,
) -> list[Waypoint]:
    """Generate a boustrophedon (lawnmower) survey grid of waypoints.

    ``width_m`` is east-west extent, ``height_m`` north-south. Lines are
    spaced ``spacing_m`` apart; altitude is constant. Simplified flat-earth
    conversion is adequate for small fields (< few km).
    """
    if spacing_m <= 0 or width_m <= 0 or height_m <= 0:
        raise ValueError("width, height and spacing must be positive")
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0  # approximation at mid latitudes

    rows = max(1, int(height_m // spacing_m) + 1)
    points: list[Waypoint] = []
    for i in range(rows):
        north_offset = -height_m / 2 + i * spacing_m
        lat = center_lat + north_offset / m_per_deg_lat
        left = center_lon - (width_m / 2) / m_per_deg_lon
        right = center_lon + (width_m / 2) / m_per_deg_lon
        pair = [(lat, left), (lat, right)] if i % 2 == 0 else [(lat, right), (lat, left)]
        for plat, plon in pair:
            points.append(Waypoint(plat, plon, altitude_m))
    return points


class LiveDetectionMap:
    """Incrementally updated map of geotagged detections.

    Feeds a GeoJSON store as detections arrive in flight so markers appear in
    near real time rather than only after the mission.
    """

    def __init__(self, store) -> None:
        self.store = store
        self.listeners: list[Callable[[dict], None]] = []

    def subscribe(self, listener: Callable[[dict], None]) -> None:
        self.listeners.append(listener)

    def publish(self, detection) -> dict:
        """Add one detection and notify listeners with the current feature."""
        self.store.add(detection)
        feature = {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [detection.longitude, detection.latitude],
            },
            "properties": {
                "label": detection.label,
                "confidence": detection.confidence,
                "timestamp": detection.timestamp,
                "severity": detection.severity,
            },
        }
        for listener in self.listeners:
            listener(feature)
        return feature


@dataclass(frozen=True)
class MissionPlan:
    waypoints: list[Waypoint]
    feasible: bool
    reason: str
    rtl_reserve_wh: float = 0.0
    required_energy_wh: float = 0.0
    available_energy_wh: float = 0.0


def plan_mission(
    state: VehicleState,
    waypoints: list[Waypoint],
    home_lat: float,
    home_lon: float,
    *,
    battery_capacity_wh: float,
    cruise_speed_mps: float,
    cruise_power_w: float,
    safety_margin: float = 0.25,
    rtl_reserve_wh: float | None = None,
    rtl_reserve_pct: float | None = None,
) -> MissionPlan:
    """Battery-aware planner with an explicit RTL reserve constraint.

    The planner permits only the longest feasible prefix of the route when the
    full route cannot be flown while keeping ``rtl_reserve_wh`` (or
    ``rtl_reserve_pct`` of the usable pack) in the battery at all times.
    ``safety_margin`` inflates route energy consumption as before. If neither
    reserve argument is given, no explicit reserve is enforced (legacy path).
    """
    if not waypoints:
        return MissionPlan(waypoints=[], feasible=False, reason="no waypoints")
    if rtl_reserve_wh is not None and rtl_reserve_pct is not None:
        raise ValueError("pass either rtl_reserve_wh or rtl_reserve_pct, not both")
    if rtl_reserve_wh is not None and rtl_reserve_wh < 0:
        raise ValueError("rtl_reserve_wh must be non-negative")
    if rtl_reserve_pct is not None and not 0 <= rtl_reserve_pct <= 1:
        raise ValueError("rtl_reserve_pct must be in [0, 1]")

    usable_wh = battery_capacity_wh * (state.battery_pct / 100.0)
    reserve_wh = (
        rtl_reserve_wh if rtl_reserve_wh is not None
        else (usable_wh * rtl_reserve_pct if rtl_reserve_pct is not None else 0.0)
    )
    start = Waypoint(state.latitude, state.longitude, state.altitude_m)

    def required_for(prefix: list[Waypoint]) -> float:
        route = [start, *prefix]
        distance = sum(
            distance_m(a.latitude, a.longitude, b.latitude, b.longitude)
            for a, b in zip(route, route[1:])
        )
        last = prefix[-1]
        home_m = distance_m(last.latitude, last.longitude, home_lat, home_lon)
        return estimate_return_energy_wh(
            distance + home_m, cruise_speed_mps, cruise_power_w
        ) * (1.0 + safety_margin)

    full_required = required_for(waypoints)
    if full_required <= usable_wh - reserve_wh:
        return MissionPlan(
            waypoints=waypoints, feasible=True, reason="feasible",
            rtl_reserve_wh=reserve_wh, required_energy_wh=full_required,
            available_energy_wh=usable_wh,
        )

    if reserve_wh == 0.0:
        return MissionPlan(
            waypoints=[], feasible=False,
            reason="insufficient energy for route + RTL reserve",
            rtl_reserve_wh=0.0, required_energy_wh=full_required,
            available_energy_wh=usable_wh,
        )

    # Find the longest prefix that still preserves the explicit reserve.
    feasible_prefix: list[Waypoint] = []
    for i in range(1, len(waypoints) + 1):
        if required_for(waypoints[:i]) <= usable_wh - reserve_wh:
            feasible_prefix = waypoints[:i]
        else:
            break
    return MissionPlan(
        waypoints=feasible_prefix,
        feasible=bool(feasible_prefix),
        reason=(
            f"trimmed to {len(feasible_prefix)}/{len(waypoints)} waypoints to keep "
            f"{reserve_wh:.1f} Wh RTL reserve"
        ),
        rtl_reserve_wh=reserve_wh,
        required_energy_wh=required_for(feasible_prefix) if feasible_prefix else full_required,
        available_energy_wh=usable_wh,
    )
