"""Value-of-information (VoI) revisit decisions.

A revisit is only worth its energy if expected information gain exceeds the
energy cost, expressed in a common "utility per Wh" unit. All constants are
explicit assumptions, documented below, so the decision is auditable rather
than a hidden heuristic.
"""

from __future__ import annotations

from dataclasses import dataclass

from .camera import Camera, FrameObservation, detection_probability
from .navigation import distance_m
from .optimizer import ValueRegion, plan_revisits, revisit_extra_distance_m

__all__ = ["VoIConfig", "VoIDecision", "evaluate_revisit"]


@dataclass(frozen=True)
class VoIConfig:
    """Assumptions for the revisit trade-off.

    ``value_per_discovery`` is the utility assigned to one additional confirmed
    infected detection. ``energy_cost_per_wh`` is the utility cost of one Wh
    spent. A revisit is chosen when expected_value > energy * energy_cost_per_wh.
    """

    value_per_discovery: float = 10.0
    energy_cost_per_wh: float = 1.0
    min_expected_discoveries: float = 0.5  # require at least one likely result


@dataclass(frozen=True)
class VoIDecision:
    worthwhile: bool
    expected_discoveries: float
    expected_value: float
    energy_cost_wh: float
    energy_cost_utility: float
    extra_distance_m: float
    reason: str


def evaluate_revisit(
    base_waypoints: list,
    region: ValueRegion,
    *,
    altitude_m: float,
    speed_mps: float,
    overlap_fraction: float,
    power_w: float,
    camera: Camera | None = None,
    config: VoIConfig | None = None,
) -> VoIDecision:
    """Decide whether a high-value region is worth an extra survey pass."""
    cam = camera or Camera()
    cfg = config or VoIConfig()

    combined, selected = plan_revisits(
        base_waypoints, [region], spacing_m=max(5.0, altitude_m), value_threshold=0.0
    )
    extra_m = revisit_extra_distance_m(base_waypoints, combined) if selected else 0.0
    energy_wh = power_w * (extra_m / max(speed_mps, 0.1)) / 3600.0

    # Expected discoveries = p(detect per view) * region.value * number of views.
    p = detection_probability(
        cam,
        FrameObservation(
            altitude_m=altitude_m,
            ground_speed_mps=speed_mps,
            overlap_fraction=overlap_fraction,
        ),
    )
    expected = p * region.value * 3  # 3x3 revisit grid -> ~3 independent looks
    expected_value = expected * cfg.value_per_discovery
    cost_utility = energy_wh * cfg.energy_cost_per_wh
    worthwhile = (
        bool(selected)
        and expected >= cfg.min_expected_discoveries
        and expected_value > cost_utility
    )
    if not selected:
        reason = "region below value threshold"
    elif expected < cfg.min_expected_discoveries:
        reason = "too few expected discoveries"
    elif expected_value <= cost_utility:
        reason = "energy cost exceeds expected information value"
    else:
        reason = "expected value exceeds energy cost"
    return VoIDecision(
        worthwhile=worthwhile,
        expected_discoveries=expected,
        expected_value=expected_value,
        energy_cost_wh=energy_wh,
        energy_cost_utility=cost_utility,
        extra_distance_m=extra_m,
        reason=reason,
    )
