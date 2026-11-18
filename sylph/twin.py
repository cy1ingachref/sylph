"""Phase 6: physics-based digital twin of the Sylph airframe.

Simulates power draw, ground speed, energy consumption and battery state over
a mission. Includes induced/parasite/profile drag, wind, climbs, and avionics.
Deterministic and hardware-free so it can be tested and optimized offline.
"""

from __future__ import annotations

from dataclasses import dataclass

from .design import PowerBudget

__all__ = ["Environment", "FlightSegment", "SegmentResult", "TwinState", "DigitalTwin"]


@dataclass(frozen=True)
class Environment:
    air_density_kgm3: float = 1.225
    wind_speed_mps: float = 0.0  # positive = headwind on the segment
    wind_direction_deg: float = 0.0


@dataclass(frozen=True)
class FlightSegment:
    """A straight flight at constant commanded airspeed and climb rate."""

    distance_m: float
    airspeed_mps: float
    climb_rate_mps: float = 0.0


@dataclass(frozen=True)
class SegmentResult:
    time_s: float
    energy_wh: float
    ground_speed_mps: float
    power_w: float


@dataclass
class TwinState:
    energy_remaining_wh: float
    time_s: float = 0.0
    distance_m: float = 0.0


class DigitalTwin:
    """Simple fixed-wing-style drag model applied to a multirotor in forward flight.

    Power model (per guide's realistic flight times):
      P = P_hover * (1 + k_v * v^2) + m*g*climb / eta_climb + avionics
    where the v^2 term captures parasite/profile power in forward flight.
    Wind affects ground speed (and therefore time/energy for a fixed distance).
    """

    def __init__(
        self,
        budget: PowerBudget,
        *,
        drag_factor: float = 0.0025,  # (m/s)^-2 scaling of forward-flight power
        climb_efficiency: float = 0.65,
        environment: Environment | None = None,
    ) -> None:
        self.budget = budget
        self.drag_factor = drag_factor
        self.climb_efficiency = climb_efficiency
        self.env = environment or Environment()
        if drag_factor < 0:
            raise ValueError("drag_factor must be non-negative")
        if not 0 < climb_efficiency <= 1:
            raise ValueError("climb_efficiency must be in (0, 1]")

    def power_w(self, airspeed_mps: float, climb_rate_mps: float = 0.0) -> float:
        if airspeed_mps < 0:
            raise ValueError("airspeed must be non-negative")
        forward = self.budget.hover_power_w * self.drag_factor * airspeed_mps**2
        climb = 0.0
        if climb_rate_mps > 0:
            # climb power derived from hover power via thrust-to-weight ~1 in hover
            hover_thrust_power = self.budget.hover_power_w
            climb = hover_thrust_power * (climb_rate_mps / max(airspeed_mps, 1.0))
        total = self.budget.hover_power_w + forward + climb / self.climb_efficiency + self.budget.avionics_power_w
        return total

    def ground_speed(self, airspeed_mps: float, heading_into_wind: float = 1.0) -> float:
        """Signed ground speed; wind component = wind_speed * heading_into_wind."""
        gs = airspeed_mps - self.env.wind_speed_mps * heading_into_wind
        return max(gs, 0.1)  # never stall to zero in the model

    def simulate_segment(self, state: TwinState, segment: FlightSegment, *, into_wind: float = 1.0) -> SegmentResult:
        gs = self.ground_speed(segment.airspeed_mps, into_wind)
        time_s = segment.distance_m / gs
        power = self.power_w(segment.airspeed_mps, segment.climb_rate_mps)
        energy_wh = power * (time_s / 3600.0)
        return SegmentResult(time_s=time_s, energy_wh=energy_wh, ground_speed_mps=gs, power_w=power)

    def fly(self, state: TwinState, segment: FlightSegment, *, into_wind: float = 1.0) -> SegmentResult:
        """Apply a segment to the state in place (energy/time accounting)."""
        r = self.simulate_segment(state, segment, into_wind=into_wind)
        if r.energy_wh > state.energy_remaining_wh:
            raise RuntimeError("battery depleted before segment completed")
        state.energy_remaining_wh -= r.energy_wh
        state.time_s += r.time_s
        state.distance_m += segment.distance_m
        return r
