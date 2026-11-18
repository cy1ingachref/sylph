"""Phase 1: deterministic waypoint mission runner."""

from __future__ import annotations

from dataclasses import dataclass

from .link import SimulatedLink

__all__ = ["Waypoint", "WaypointRunner", "MissionResult"]


@dataclass(frozen=True)
class Waypoint:
    latitude: float
    longitude: float
    altitude_m: float


@dataclass(frozen=True)
class MissionResult:
    completed: list[Waypoint]
    aborted: bool
    reason: str = ""


class WaypointRunner:
    """Executes a list of waypoints through a simulated link.

    The runner is deliberately transport-agnostic: it records the commands it
    would send to a flight controller so the mission logic can be tested
    without hardware. A ``should_rtl`` callback can abort early.
    """

    def __init__(self, link: SimulatedLink, should_rtl) -> None:
        self.link = link
        self.should_rtl = should_rtl

    def run(self, waypoints: list[Waypoint]) -> MissionResult:
        completed: list[Waypoint] = []
        for wp in waypoints:
            if self.should_rtl():
                self.link.send("RTL")
                return MissionResult(completed=completed, aborted=True, reason="rtl")
            self.link.send(f"GOTO {wp.latitude},{wp.longitude},{wp.altitude_m}")
            completed.append(wp)
        self.link.send("MISSION_COMPLETE")
        return MissionResult(completed=completed, aborted=False)
