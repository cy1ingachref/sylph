"""Phase 6: physics-based digital twin of the Sylph airframe.

Simulates power draw, ground speed, energy consumption and battery state over
a mission. Includes induced/parasite/profile drag, wind, climbs, and avionics.
Deterministic and hardware-free so it can be tested and optimized offline.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .battery import BatteryModel, BatteryState
from .design import PowerBudget
from .sensors import SensorSuite

__all__ = [
    "Environment", "FlightSegment", "SegmentResult", "TwinState", "DigitalTwin",
]


@dataclass(frozen=True)
class Environment:
    air_density_kgm3: float = 1.225
    wind_speed_mps: float = 0.0  # positive = headwind on the segment
    wind_direction_deg: float = 0.0
    # Opt-in gust/turbulence (all zero by default -> baseline behaviour).
    gust_amplitude_mps: float = 0.0   # sinusoidal gust peak amplitude
    gust_period_s: float = 10.0       # gust period in seconds
    turbulence_sigma_mps: float = 0.0  # seeded gaussian turbulence per segment
    turbulence_seed: int | None = None

    def wind_at(self, time_s: float) -> float:
        """Total headwind component at mission time ``time_s``.

        Sinusoidal gust is deterministic for a given time; gaussian turbulence
        is drawn once per segment by DigitalTwin from a seeded RNG.
        """
        base = self.wind_speed_mps
        if self.gust_amplitude_mps and self.gust_period_s > 0:
            base += self.gust_amplitude_mps * math.sin(
                2 * math.pi * time_s / self.gust_period_s
            )
        return base


@dataclass(frozen=True)
class FlightSegment:
    """A straight flight at constant commanded airspeed and climb rate."""

    distance_m: float
    airspeed_mps: float
    climb_rate_mps: float = 0.0


@dataclass
class TwinState:
    energy_remaining_wh: float
    time_s: float = 0.0
    distance_m: float = 0.0
    battery: BatteryState | None = None  # opt-in; mirrors energy when attached
    terminal_voltage_v: float | None = None  # last measured terminal voltage

    def __post_init__(self) -> None:
        if self.battery is None:
            self.battery = BatteryState(self.energy_remaining_wh, 1.0)


@dataclass(frozen=True)
class SegmentResult:
    time_s: float
    energy_wh: float
    ground_speed_mps: float
    power_w: float
    headwind_mps: float = 0.0
    terminal_voltage_v: float | None = None


class DigitalTwin:
    """Simple fixed-wing-style drag model applied to a multirotor in forward flight.

    Power model (per guide's realistic flight times):
      P = P_hover * (1 + k_v * v^2) + m*g*climb / eta_climb + avionics
    where the v^2 term captures parasite/profile power in forward flight.
    Wind affects ground speed (and therefore time/energy for a fixed distance).

    Opt-in realism (all off by default so baseline numbers do not move):
      * ``Environment.gust_amplitude_mps`` adds a sinusoidal gust.
      * ``Environment.turbulence_sigma_mps`` adds seeded gaussian wind noise.
      * ``battery_model`` applies OCV sag + Peukert derating to energy draw.
      * ``sensors`` samples seeded noisy measurements after each segment.
    """

    def __init__(
        self,
        budget: PowerBudget,
        *,
        drag_factor: float = 0.0025,  # (m/s)^-2 scaling of forward-flight power
        climb_efficiency: float = 0.65,
        environment: Environment | None = None,
        battery_model: BatteryModel | None = None,
        sensors: SensorSuite | None = None,
    ) -> None:
        self.budget = budget
        self.drag_factor = drag_factor
        self.climb_efficiency = climb_efficiency
        self.env = environment or Environment()
        self.battery_model = battery_model or BatteryModel(
            nominal_v=budget.voltage_v,
            capacity_ah=budget.capacity_ah,
            enabled=False,
        )
        self.sensors = sensors
        self._turbulence_rng = (
            random.Random(self.env.turbulence_seed)
            if self.env.turbulence_seed is not None
            else None
        )
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

    def ground_speed(self, airspeed_mps: float, heading_into_wind: float = 1.0, *, time_s: float = 0.0) -> float:
        """Signed ground speed; wind component = wind_at(t) * heading_into_wind."""
        gs = airspeed_mps - self.env.wind_at(time_s) * heading_into_wind
        return max(gs, 0.1)  # never stall to zero in the model

    def simulate_segment(
        self, state: TwinState, segment: FlightSegment, *, into_wind: float = 1.0
    ) -> SegmentResult:
        # Turbulence is a single seeded draw per segment, added to the wind at
        # the segment start time. With sigma=0 this is exactly the baseline.
        turbulence = (
            self._turbulence_rng.gauss(0.0, self.env.turbulence_sigma_mps)
            if self._turbulence_rng is not None
            else 0.0
        )
        wind = self.env.wind_at(state.time_s) + turbulence
        gs = max(segment.airspeed_mps - wind * into_wind, 0.1)
        time_s = segment.distance_m / gs
        power = self.power_w(segment.airspeed_mps, segment.climb_rate_mps)

        # The battery model is the source of truth for energy consumed: with
        # the default disabled model this is exactly power*dt; with Peukert
        # enabled high-current draws consume the reservoir faster.
        new_battery, terminal_v = self.battery_model.draw(
            state.battery or BatteryState(state.energy_remaining_wh),
            power,
            time_s,
        )
        # Physical power*time is reported separately for clarity; the draw
        # (which may be larger under Peukert) is what depletes the pack.
        physical_wh = power * time_s / 3600.0
        drawn_wh = max(
            0.0,
            (state.battery or BatteryState(state.energy_remaining_wh)).energy_remaining_wh
            - new_battery.energy_remaining_wh,
        )
        return SegmentResult(
            time_s=time_s,
            energy_wh=physical_wh if not self.battery_model.enabled else drawn_wh,
            ground_speed_mps=gs,
            power_w=power,
            headwind_mps=wind * into_wind,
            terminal_voltage_v=terminal_v,
        )

    def fly(self, state: TwinState, segment: FlightSegment, *, into_wind: float = 1.0) -> SegmentResult:
        """Apply a segment to the state in place (energy/time accounting)."""
        r = self.simulate_segment(state, segment, into_wind=into_wind)
        if r.energy_wh >= state.energy_remaining_wh:
            raise RuntimeError("battery depleted before segment completed")
        state.energy_remaining_wh -= r.energy_wh
        state.time_s += r.time_s
        state.distance_m += segment.distance_m
        state.terminal_voltage_v = r.terminal_voltage_v
        # Keep the battery object in lockstep with the authoritative energy.
        if state.battery is not None:
            state.battery.energy_remaining_wh = state.energy_remaining_wh
            state.battery.soc = max(
                0.0,
                min(1.0, state.energy_remaining_wh / max(self.budget.usable_energy_wh, 0.1)),
            )
        return r

