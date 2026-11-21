"""Phase 3: high-level mission autonomy -- voice commands, grid scouting,
live detection map, and battery-aware planning.
"""

from __future__ import annotations

import time
import uuid
from collections.abc import Callable
from dataclasses import dataclass

from .link import VehicleState
from .navigation import distance_m, estimate_return_energy_wh
from .waypoints import Waypoint

__all__ = [
    "Command",
    "parse_command",
    "CommandInterpreter",
    "PendingCommand",
    "lawnmower_pattern",
    "meters_per_degree_lon",
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


@dataclass(frozen=True)
class PendingCommand:
    """A confirmation candidate bound to one exact command instance."""

    command: Command
    token: str
    normalized: str
    created_at_s: float


class CommandInterpreter:
    """Gate requiring confirmation of the *currently displayed* command.

    Safety rules:
      * any new utterance replaces or clears pending state -- an invalid
        utterance can never leave an older command executable;
      * pending commands expire after ``expiry_s``;
      * confirmation must match the pending token (or exact normalized text),
        so a generic "yes" cannot execute a different command;
      * ``safety_check`` is re-run immediately before execution and can veto.
    """

    def __init__(
        self,
        executor: Callable[[Command], None],
        *,
        expiry_s: float = 30.0,
        safety_check: Callable[[], bool] | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        if expiry_s <= 0:
            raise ValueError("expiry_s must be positive")
        self.executor = executor
        self.expiry_s = expiry_s
        self.safety_check = safety_check
        self._clock = clock or time.monotonic
        self.pending: PendingCommand | None = None
        self.executed: list[Command] = []

    def submit(self, utterance: str) -> Command:
        """Parse an utterance; replace pending state in all cases."""
        cmd = parse_command(utterance)
        # Always invalidate the previous candidate before deciding the new one.
        self.pending = None
        if cmd.kind != "unknown":
            self.pending = PendingCommand(
                command=cmd,
                token=uuid.uuid4().hex,
                normalized=" ".join(utterance.lower().strip().split()),
                created_at_s=self._clock(),
            )
        return cmd

    def _live_pending(self) -> PendingCommand | None:
        p = self.pending
        if p is None:
            return None
        if self._clock() - p.created_at_s > self.expiry_s:
            self.pending = None
            return None
        return p

    def confirm(self, *, allowed: bool = True, token: str | None = None,
                utterance: str | None = None) -> bool:
        """Execute only the current, unexpired pending command.

        When ``token`` is supplied it must match. When ``utterance`` is
        supplied its normalized form must match. One of them is required for
        strict binding; ``confirm()`` alone remains for backward compatibility
        but still only ever executes the single current pending command.
        """
        p = self._live_pending()
        if p is None:
            return False
        if token is not None and token != p.token:
            return False
        if utterance is not None and " ".join(utterance.lower().strip().split()) != p.normalized:
            return False
        self.pending = None  # single-use: repeated confirm cannot re-execute
        if not allowed or p.command.kind == "unknown":
            return False
        if self.safety_check is not None and not self.safety_check():
            return False
        self.executor(p.command)
        self.executed.append(p.command)
        return True


def meters_per_degree_lon(latitude: float) -> float:
    """Metres per degree longitude at a given latitude (WGS84 approximation)."""
    from math import cos, radians

    lat = max(-89.9, min(89.9, latitude))
    return 111_320.0 * cos(radians(lat))


def lawnmower_pattern(
    center_lat: float,
    center_lon: float,
    *,
    width_m: float,
    height_m: float,
    spacing_m: float,
    altitude_m: float,
    heading_deg: float = 0.0,
) -> list[Waypoint]:
    """Generate a boustrophedon (lawnmower) survey grid of waypoints.

    ``width_m`` is east-west extent, ``height_m`` north-south. Lines are
    spaced ``spacing_m`` apart; altitude is constant. Uses a LOCAL TANGENT
    PLANE (ENU) approximation: metres north -> latitude via 111,320 m/deg,
    metres east -> longitude via 111,320*cos(latitude) m/deg. Valid for small
    fields (a few km) away from poles; for larger areas use a projected CRS.

    ``heading_deg`` rotates the grid in the local ENU plane (0 = north-up).
    """
    from math import cos, radians, sin

    if spacing_m <= 0 or width_m <= 0 or height_m <= 0:
        raise ValueError("width, height and spacing must be positive")
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = meters_per_degree_lon(center_lat)

    rows = max(1, int(height_m // spacing_m) + 1)
    points: list[Waypoint] = []
    h = radians(heading_deg)
    for i in range(rows):
        north = -height_m / 2 + i * spacing_m
        east_pair = (-width_m / 2, width_m / 2)
        if i % 2 == 1:  # boustrophedon: alternate travel direction each row
            east_pair = east_pair[::-1]
        for east in east_pair:
            # rotate (east, north) by heading
            e2 = east * cos(h) - north * sin(h)
            n2 = east * sin(h) + north * cos(h)
            lat = center_lat + n2 / m_per_deg_lat
            lon = center_lon + e2 / m_per_deg_lon
            points.append(Waypoint(lat, lon, altitude_m))
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
