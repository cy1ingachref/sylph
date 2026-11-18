"""Phase 4: field-ready features -- reporting, sensor fusion, field profiles.
"""

from __future__ import annotations

from dataclasses import dataclass
from statistics import mean

from .vision import Detection

__all__ = [
    "MissionReport",
    "build_report",
    "altitude_hold_correction",
    "fuse_altitude",
    "FieldProfile",
    "DEFAULT_PROFILE",
]


@dataclass(frozen=True)
class MissionReport:
    total_detections: int
    infected_detections: int
    infection_rate: float
    average_confidence: float
    recommendations: tuple[str, ...]

    def to_text(self) -> str:
        lines = [
            "Sylph Mission Report",
            f"  Detections:      {self.total_detections}",
            f"  Infected:        {self.infected_detections}",
            f"  Infection rate:  {self.infection_rate:.1%}",
            f"  Avg confidence:  {self.average_confidence:.2f}",
            "  Recommendations:",
        ]
        lines.extend(f"    - {r}" for r in self.recommendations)
        return "\n".join(lines)


def build_report(detections: list[Detection]) -> MissionReport:
    """Summarize detections into an actionable field report."""
    if not detections:
        return MissionReport(0, 0, 0.0, 0.0, ("No detections - re-fly survey.",))
    infected = [d for d in detections if d.label.lower() not in ("healthy", "none")]
    rate = len(infected) / len(detections)
    avg_conf = mean(d.confidence for d in detections)
    recs: list[str] = []
    if rate > 0.2:
        recs.append("High infection rate: prioritize affected grid rows for treatment.")
    elif infected:
        recs.append("Sparse infections: monitor affected zones in next flight.")
    else:
        recs.append("No infection detected: maintain schedule.")
    high_severity = [d for d in infected if d.severity >= 0.2]
    if high_severity:
        recs.append(f"{len(high_severity)} low-confidence detections: re-image for verification.")
    return MissionReport(
        total_detections=len(detections),
        infected_detections=len(infected),
        infection_rate=rate,
        average_confidence=avg_conf,
        recommendations=tuple(recs),
    )


def fuse_altitude(barometer_m: float, tof_m: float | None) -> float:
    """Fuse barometric and optional time-of-flight altitude (simple average)."""
    if tof_m is None:
        return barometer_m
    return (barometer_m + tof_m) / 2.0


def altitude_hold_correction(current_m: float, target_m: float, deadband_m: float = 0.3) -> float:
    """Return vertical correction; zero inside the deadband (avoids hunting)."""
    error = target_m - current_m
    return 0.0 if abs(error) <= deadband_m else error


@dataclass(frozen=True)
class FieldProfile:
    """Named configuration for a field / mission type."""

    name: str
    target_altitude_m: float
    safety_margin: float
    grid_spacing_m: float
    cruise_speed_mps: float


DEFAULT_PROFILE = FieldProfile(
    name="default",
    target_altitude_m=20.0,
    safety_margin=0.25,
    grid_spacing_m=25.0,
    cruise_speed_mps=12.0,
)
