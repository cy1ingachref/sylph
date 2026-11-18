"""Phase 1: smart return-to-launch energy logic.

Implements the guide's rule: trigger RTL when remaining energy is below the
energy needed to get home plus a safety margin. Also provides a great-circle
distance helper used by both RTL and the mission planner.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, cos, radians, sin, sqrt

from .link import VehicleState

__all__ = [
    "distance_m",
    "estimate_return_energy_wh",
    "should_return_to_launch",
    "RtlDecision",
]

EARTH_RADIUS_M = 6_371_000.0


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two WGS84 points, in metres."""
    p1, p2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)
    a = sin(dphi / 2) ** 2 + cos(p1) * cos(p2) * sin(dlambda / 2) ** 2
    return 2 * EARTH_RADIUS_M * asin(sqrt(a))


@dataclass(frozen=True)
class RtlDecision:
    """Result of the smart-RTL evaluation."""

    return_energy_wh: float
    reserve_wh: float
    battery_wh: float
    rtl: bool

    @property
    def reason(self) -> str:
        if self.rtl:
            return "remaining energy <= return energy + safety margin"
        return "energy sufficient"


def estimate_return_energy_wh(
    distance_metres: float, cruise_speed_mps: float, cruise_power_w: float
) -> float:
    """Energy (Wh) needed to fly ``distance_metres`` home at cruise settings."""
    if cruise_speed_mps <= 0:
        raise ValueError("cruise_speed_mps must be positive")
    if cruise_power_w < 0:
        raise ValueError("cruise_power_w must be non-negative")
    hours = (distance_metres / cruise_speed_mps) / 3600.0
    return hours * cruise_power_w


def should_return_to_launch(
    state: VehicleState,
    home_lat: float,
    home_lon: float,
    *,
    battery_capacity_wh: float,
    cruise_speed_mps: float,
    cruise_power_w: float,
    safety_margin: float = 0.25,
) -> RtlDecision:
    """Decide whether to trigger RTL.

    ``battery_pct`` of the state is converted to remaining Wh using
    ``battery_capacity_wh``. RTL triggers when remaining energy is at or below
    the return energy inflated by ``safety_margin`` (default 25%, per guide).
    """
    if not 0.0 <= safety_margin <= 1.0:
        raise ValueError("safety_margin must be between 0 and 1")
    if battery_capacity_wh <= 0:
        raise ValueError("battery_capacity_wh must be positive")

    d = distance_m(state.latitude, state.longitude, home_lat, home_lon)
    required = estimate_return_energy_wh(d, cruise_speed_mps, cruise_power_w)
    reserve = required * safety_margin
    remaining = battery_capacity_wh * (state.battery_pct / 100.0)
    return RtlDecision(
        return_energy_wh=required,
        reserve_wh=reserve,
        battery_wh=remaining,
        rtl=remaining <= required + reserve,
    )
