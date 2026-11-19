"""Monte-Carlo evaluation of mission robustness under wind and sensor noise.

Runs the same mission with different seeds and reports success rate, energy
distribution and worst-case return-to-launch energy margin. Deterministic given
the same seed range; no randomness outside the seeded simulations.
"""

from __future__ import annotations

from dataclasses import dataclass
from statistics import mean, median, pstdev

from .design import size_power_system
from .simulator import simulate_mission

__all__ = ["MonteCarloResult", "monte_carlo"]


@dataclass(frozen=True)
class MonteCarloResult:
    runs: int
    successes: int
    success_rate: float
    energy_mean_wh: float
    energy_median_wh: float
    energy_pstdev_wh: float
    energy_min_wh: float
    energy_max_wh: float
    worst_case_rtl_margin_wh: float
    usable_energy_wh: float

    def to_text(self) -> str:
        return "\n".join([
            f"Monte-Carlo ({self.runs} seeds)",
            f"  Success rate:      {self.success_rate:.1%} ({self.successes}/{self.runs})",
            f"  Energy mean:       {self.energy_mean_wh:.1f} Wh",
            f"  Energy median:     {self.energy_median_wh:.1f} Wh",
            f"  Energy stdev:      {self.energy_pstdev_wh:.1f} Wh",
            f"  Energy range:      {self.energy_min_wh:.1f} - {self.energy_max_wh:.1f} Wh",
            f"  Worst RTL margin:  {self.worst_case_rtl_margin_wh:.1f} Wh",
        ])


def monte_carlo(
    *,
    runs: int = 50,
    seed_start: int = 0,
    gust_amplitude_mps: float = 2.0,
    turbulence_sigma_mps: float = 0.5,
    sensor_seed: bool = True,
    **mission_kwargs,
) -> MonteCarloResult:
    """Run ``runs`` missions with seeds seed_start..seed_start+runs-1."""
    if runs < 1:
        raise ValueError("runs must be >= 1")
    results = []
    for seed in range(seed_start, seed_start + runs):
        r = simulate_mission(
            gust_amplitude_mps=gust_amplitude_mps,
            turbulence_sigma_mps=turbulence_sigma_mps,
            turbulence_seed=seed,
            sensor_seed=seed if sensor_seed else None,
            **mission_kwargs,
        )
        results.append(r)

    energies = [r.energy_wh for r in results]
    successes = sum(1 for r in results if r.success)
    # RTL margin = energy remaining after landing attempt; failed runs have 0.
    margins = [r.energy_remaining_wh if r.success else 0.0 for r in results]
    usable = mission_kwargs.get("budget") or size_power_system()
    usable_wh = usable.usable_energy_wh if hasattr(usable, "usable_energy_wh") else results[0].energy_remaining_wh + results[0].energy_wh
    return MonteCarloResult(
        runs=runs,
        successes=successes,
        success_rate=successes / runs,
        energy_mean_wh=mean(energies),
        energy_median_wh=median(energies),
        energy_pstdev_wh=pstdev(energies) if runs > 1 else 0.0,
        energy_min_wh=min(energies),
        energy_max_wh=max(energies),
        worst_case_rtl_margin_wh=min(margins),
        usable_energy_wh=usable_wh,
    )
