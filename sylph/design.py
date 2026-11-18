"""Phase 6: physical design -- BOM and power-system sizing.

Numbers follow the component ranges in the build guide (450-550 mm frame,
4S-6S LiPo, 920-1000 KV, Jetson Orin Nano companion computer).
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = ["Component", "BOM", "DEFAULT_BOM", "PowerBudget", "size_power_system"]


@dataclass(frozen=True)
class Component:
    name: str
    quantity: int
    mass_g: int  # mass of one unit
    cost_usd: int  # cost of one unit
    note: str = ""


@dataclass(frozen=True)
class BOM:
    components: tuple[Component, ...]

    @property
    def total_mass_g(self) -> int:
        return sum(c.quantity * c.mass_g for c in self.components)

    @property
    def total_cost_usd(self) -> int:
        return sum(c.quantity * c.cost_usd for c in self.components)

    def as_rows(self) -> list[tuple[str, int, int, int]]:
        return [(c.name, c.quantity, c.mass_g, c.cost_usd) for c in self.components]


DEFAULT_BOM = BOM(
    components=(
        Component("F550 carbon frame", 1, 560, 90),
        Component("920KV motors + 30A 4-in-1 ESC", 1, 320, 180),
        Component("Pixhawk 6C flight controller", 1, 40, 160),
        Component("Here3 GPS + compass", 1, 45, 120),
        Component("Jetson Orin Nano companion", 1, 170, 499, "AI + mission logic"),
        Component("Global shutter downward camera", 1, 60, 130),
        Component("4S 8000mAh LiPo (14.8V)", 1, 720, 90),
        Component("SiK telemetry radio", 1, 30, 60),
        Component("BEC, harness, mounts", 1, 150, 70),
    )
)


@dataclass(frozen=True)
class PowerBudget:
    voltage_v: float
    capacity_ah: float
    pack_energy_wh: float
    usable_energy_wh: float  # 80% DoD
    hover_power_w: float
    avionics_power_w: float
    total_hover_power_w: float
    estimated_hover_minutes: float


def size_power_system(
    *,
    voltage_v: float = 14.8,
    capacity_ah: float = 8.0,
    mtow_kg: float | None = None,
    propeller_diameter_m: float = 0.254,  # 10 inch
    propeller_efficiency: float = 0.55,
    figure_of_merit: float = 0.70,
    avionics_w: float = 30.0,
    depth_of_discharge: float = 0.80,
) -> PowerBudget:
    """Size endurance with momentum theory.

    Hover power: P = (W^1.5) / (FM * sqrt(2 * rho * A)), with W = m*g,
    A = 4 * pi * (r^2) total disk area for a quad, rho = 1.225 kg/m^3.
    """
    from math import pi, sqrt

    mass = mtow_kg if mtow_kg is not None else DEFAULT_BOM.total_mass_g / 1000.0
    rho = 1.225
    g = 9.81
    radius = propeller_diameter_m / 2.0
    area = 4 * pi * radius**2  # four rotors
    weight = mass * g
    ideal_power = weight**1.5 / sqrt(2 * rho * area)
    hover_power = ideal_power / (figure_of_merit * propeller_efficiency)
    pack = voltage_v * capacity_ah
    usable = pack * depth_of_discharge
    total = hover_power + avionics_w
    return PowerBudget(
        voltage_v=voltage_v,
        capacity_ah=capacity_ah,
        pack_energy_wh=pack,
        usable_energy_wh=usable,
        hover_power_w=hover_power,
        avionics_power_w=avionics_w,
        total_hover_power_w=total,
        estimated_hover_minutes=(usable / total) * 60.0,
    )
