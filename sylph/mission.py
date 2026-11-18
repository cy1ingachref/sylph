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
) -> MissionPlan:
    """Battery-aware planner: abort early if the route cannot be completed
    with enough reserve to return home.
    """
    if not waypoints:
        return MissionPlan(waypoints=[], feasible=False, reason="no waypoints")

    route = [Waypoint(state.latitude, state.longitude, state.altitude_m), *waypoints]
    total_m = 0.0
    for a, b in zip(route, route[1:]):
        total_m += distance_m(a.latitude, a.longitude, b.latitude, b.longitude)
    home_m = distance_m(waypoints[-1].latitude, waypoints[-1].longitude, home_lat, home_lon)
    needed_wh = estimate_return_energy_wh(
        total_m + home_m, cruise_speed_mps, cruise_power_w
    ) * (1.0 + safety_margin)
    available_wh = battery_capacity_wh * (state.battery_pct / 100.0)
    feasible = available_wh >= needed_wh
    return MissionPlan(
        waypoints=waypoints if feasible else [],
        feasible=feasible,
        reason="feasible" if feasible else "insufficient energy for route + RTL reserve",
    )
