"""Opt-in battery realism for the digital twin: OCV curve, voltage sag and
Peukert capacity derating.

ASSUMPTIONS (documented, not measured):
  * 4S LiPo, 14.8 V nominal, OCV table from 4.20 V/cell (full) to 3.50 V/cell
    (empty); linearly interpolated. Real cells vary with chemistry/temperature.
  * Internal resistance R is per-pack (default 0.030 ohm) and constant.
  * Peukert exponent k=1.05 (mild derating at high current). The model converts
    commanded power to an "effective" current draw: I_eff = I * (I/I_nom)^(k-1).
    At I == I_nom this equals the simple model exactly -- so enabling the model
    cannot silently inflate/deflate the baseline endurance result.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

__all__ = ["BatteryState", "BatteryModel", "OCV_4S", "interpolate_ocv"]

# (state of charge 0..1, open-circuit volts for a 4S pack)
OCV_4S: tuple[tuple[float, float], ...] = (
    (0.00, 14.00),
    (0.10, 14.60),
    (0.20, 14.96),
    (0.50, 15.20),
    (0.80, 15.60),
    (1.00, 16.80),
)


def interpolate_ocv(soc: float, table: Iterable[tuple[float, float]] = OCV_4S) -> float:
    """Linearly interpolate open-circuit voltage for a state of charge in [0,1]."""
    pts = list(table)
    if not pts:
        raise ValueError("empty OCV table")
    if not 0.0 <= soc <= 1.0:
        raise ValueError("soc must be in [0, 1]")
    for (s0, v0), (s1, v1) in zip(pts, pts[1:]):
        if s0 <= soc <= s1:
            f = (soc - s0) / (s1 - s0) if s1 > s0 else 0.0
            return v0 + f * (v1 - v0)
    return pts[-1][1]


@dataclass
class BatteryState:
    energy_remaining_wh: float
    soc: float = 1.0  # fraction of USABLE energy remaining, for OCV lookup


@dataclass(frozen=True)
class BatteryModel:
    """LiPo pack model with sag and Peukert derating.

    Construct with ``enabled=False`` to keep the legacy perfect-reservoir
    behaviour. All methods are pure functions of (state, power, dt).
    """

    nominal_v: float = 14.8
    capacity_ah: float = 8.0
    internal_resistance_ohm: float = 0.030
    peukert_k: float = 1.05
    nominal_current_a: float = 25.0
    ocv_table: tuple[tuple[float, float], ...] = OCV_4S
    enabled: bool = True

    def __post_init__(self) -> None:
        if self.nominal_v <= 0 or self.capacity_ah <= 0:
            raise ValueError("nominal_v and capacity_ah must be positive")
        if self.internal_resistance_ohm < 0:
            raise ValueError("internal resistance must be non-negative")
        if not 1.0 <= self.peukert_k <= 2.0:
            raise ValueError("peukert_k must be in [1, 2]")
        if self.nominal_current_a <= 0:
            raise ValueError("nominal_current_a must be positive")

    def ocv_v(self, soc: float) -> float:
        """Open-circuit voltage at a state of charge (legacy when disabled)."""
        if not self.enabled:
            return self.nominal_v
        return interpolate_ocv(soc, self.ocv_table)

    def draw(self, state: BatteryState, power_w: float, dt_s: float) -> tuple[BatteryState, float]:
        """Apply a power draw over ``dt_s``.

        Returns (new_state, terminal_voltage_v). When disabled, the energy
        accounting is exactly linear in power*dt (baseline behaviour).
        """
        if dt_s < 0 or power_w < 0:
            raise ValueError("power_w and dt_s must be non-negative")
        if not self.enabled:
            used_wh = power_w * dt_s / 3600.0
            remaining = max(0.0, state.energy_remaining_wh - used_wh)
            return BatteryState(remaining, state.soc), self.nominal_v

        ocv = self.ocv_v(state.soc)
        # Solve P = V_term * I with V_term = OCV - I*R  =>  R I^2 - OCV I + P = 0
        r = self.internal_resistance_ohm
        if r == 0:
            current = power_w / max(ocv, 0.1)
            terminal = ocv
        else:
            disc = ocv**2 - 4 * r * power_w
            if disc < 0:
                # Demand beyond pack capability: clamp to max deliverable power.
                current = ocv / (2 * r)
                terminal = ocv / 2
            else:
                current = (ocv - disc**0.5) / (2 * r)
                terminal = ocv - current * r
        # Peukert: high-current draw consumes capacity faster than nominal.
        peukert_factor = (max(current, 0.0) / self.nominal_current_a) ** (self.peukert_k - 1)
        used_wh = power_w * dt_s / 3600.0 * max(1.0, peukert_factor)
        remaining = max(0.0, state.energy_remaining_wh - used_wh)
        # OCV SOC is a normalized view of the usable reservoir.
        soc = min(1.0, max(0.0, state.soc - used_wh / max(self.capacity_ah * self.nominal_v, 0.1)))
        return BatteryState(remaining, soc), terminal
