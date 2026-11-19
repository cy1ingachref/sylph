"""Phase 6: optimization -- speed, coverage spacing and survey route.

All functions are deterministic and depend only on the digital twin / geometry,
so the optimizer can be tested against measurable objectives.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, tan

from .navigation import distance_m
from .twin import DigitalTwin

__all__ = [
    "optimal_speed",
    "optimal_grid_spacing",
    "route_length_m",
    "optimize_route_2opt",
    "ValueRegion",
    "plan_revisits",
    "revisit_extra_distance_m",
]


@dataclass(frozen=True)
class ValueRegion:
    """A geographic area with an estimated disease-detection value.

    ``value`` is a non-negative weight: higher means more valuable to inspect.
    The planner adds *extra* coverage over high-value regions; it never
    removes or reorders the primary survey waypoints.
    """

    latitude: float
    longitude: float
    radius_m: float
    value: float  # e.g. disease severity from a previous flight

    def __post_init__(self) -> None:
        if self.radius_m <= 0:
            raise ValueError("radius_m must be positive")
        if self.value < 0:
            raise ValueError("value must be non-negative")


def plan_revisits(
    base_waypoints: list,
    regions: list[ValueRegion],
    *,
    spacing_m: float,
    value_threshold: float = 0.5,
) -> tuple[list, list[ValueRegion]]:
    """Add a dense sub-grid over high-value regions, preserving the base survey.

    Returns ``(combined_waypoints, selected_regions)``. The base lawnmower
    route stays intact and in order; revisit waypoints are appended as a
    second pass. This is deliberately *additive* -- no coverage is skipped.
    """
    from .waypoints import Waypoint

    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")
    selected = [r for r in regions if r.value >= value_threshold]
    revisits: list = []
    for r in selected:
        # 3x3 sub-grid across the region's bounding box.
        lat_deg_per_m = 1.0 / 111_320.0
        lon_deg_per_m = 1.0 / max(111_320.0 * cos(radians(r.latitude)), 1.0)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                revisits.append(
                    Waypoint(
                        r.latitude + di * r.radius_m * lat_deg_per_m,
                        r.longitude + dj * r.radius_m * lon_deg_per_m,
                        0.0,  # altitude set by caller/simulator
                    )
                )
    return list(base_waypoints) + revisits, selected


def revisit_extra_distance_m(base_waypoints: list, combined_waypoints: list) -> float:
    """Extra flown distance introduced by revisit waypoints (never negative)."""
    return max(
        0.0,
        path_length_m([(w.latitude, w.longitude) for w in combined_waypoints])
        - path_length_m([(w.latitude, w.longitude) for w in base_waypoints]),
    )


def optimal_speed(
    twin: DigitalTwin, *, min_mps: float = 4.0, max_mps: float = 22.0
) -> float:
    """Find the airspeed minimizing energy per metre (Wh/m) in still air.

    Energy per metre = P(v) / ground_speed(v); scan the allowed range.
    """
    if min_mps <= 0 or max_mps <= min_mps:
        raise ValueError("require 0 < min_mps < max_mps")
    best_v, best_cost = min_mps, float("inf")
    steps = 360
    for i in range(steps + 1):
        v = min_mps + (max_mps - min_mps) * i / steps
        cost = twin.power_w(v) / max(twin.ground_speed(v, 0.0), 0.1)
        if cost < best_cost:
            best_cost, best_v = cost, v
    return round(best_v, 2)


def optimal_grid_spacing(
    altitude_m: float,
    *,
    camera_fov_deg: float = 60.0,
    overlap_fraction: float = 0.20,
    min_spacing_m: float = 5.0,
) -> float:
    """Line spacing giving the requested overlap from camera cross-track FOV.

    swath = 2 * altitude * tan(FOV/2); spacing = swath * (1 - overlap).
    """
    if not 0 < camera_fov_deg < 180:
        raise ValueError("camera_fov_deg must be in (0, 180)")
    if not 0 <= overlap_fraction < 1:
        raise ValueError("overlap_fraction must be in [0, 1)")
    swath = 2 * altitude_m * tan(radians(camera_fov_deg) / 2)
    return max(min_spacing_m, round(swath * (1 - overlap_fraction), 2))


def path_length_m(points: list[tuple[float, float]]) -> float:
    """Length of the open path visiting points in order (no return edge).

    This is the distance actually flown between waypoints; RTL is accounted
    for separately by the simulator.
    """
    return sum(
        distance_m(a[0], a[1], b[0], b[1]) for a, b in zip(points, points[1:])
    )


def route_length_m(points: list[tuple[float, float]]) -> float:
    """Length of a closed tour (open path plus return to start)."""
    if len(points) < 2:
        return 0.0
    return path_length_m(points) + distance_m(
        points[-1][0], points[-1][1], points[0][0], points[0][1]
    )


def optimize_route_2opt(
    points: list[tuple[float, float]], *, passes: int = 4
) -> tuple[list[tuple[float, float]], float]:
    """Improve a survey visit order with 2-opt swaps on the *flown* path.

    Uses open-path length (no artificial return-to-start edge) so the reported
    improvement equals the distance actually saved in flight. Start and end
    waypoints are pinned to keep the open survey structure.
    """
    if len(points) < 4:
        return list(points), path_length_m(points)
    route = list(points)
    best = path_length_m(route)
    for _ in range(passes):
        improved = False
        for i in range(1, len(route) - 2):
            for j in range(i + 1, len(route) - 1):
                candidate = route[:i] + route[i : j + 1][::-1] + route[j + 1 :]
                length = path_length_m(candidate)
                if length < best:
                    route, best, improved = candidate, length, True
        if not improved:
            break
    return route, best
