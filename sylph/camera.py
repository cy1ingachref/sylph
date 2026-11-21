"""Camera footprint and probabilistic disease detection.

ASSUMPTIONS (documented, not measured):
  * Nadir-facing camera with horizontal/vertical FOV in degrees.
  * Ground footprint at altitude h: width = 2*h*tan(FOV_x/2), height likewise.
  * A frame's ground-sample distance (GSD) improves with lower altitude.
  * Detection probability falls with faster ground speed (motion blur / fewer
    pixels on target) and rises with overlap (multiple looks at the same spot).
  * Probability is capped to [0, 0.99]; no detection is guaranteed.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

__all__ = ["Camera", "Footprint", "FrameObservation", "detection_probability", "detect_frame"]


@dataclass(frozen=True)
class Camera:
    horizontal_fov_deg: float = 60.0
    vertical_fov_deg: float = 45.0
    base_detection_p: float = 0.55  # p at reference altitude/speed, no overlap
    reference_altitude_m: float = 20.0
    reference_speed_mps: float = 12.0

    def __post_init__(self) -> None:
        if not 0 < self.horizontal_fov_deg < 180:
            raise ValueError("horizontal_fov_deg must be in (0, 180)")
        if not 0 < self.vertical_fov_deg < 180:
            raise ValueError("vertical_fov_deg must be in (0, 180)")
        if not 0 <= self.base_detection_p <= 1:
            raise ValueError("base_detection_p must be in [0, 1]")

    def footprint(self, altitude_m: float) -> Footprint:
        """Ground footprint of a nadir frame at ``altitude_m``."""
        if altitude_m <= 0:
            raise ValueError("altitude_m must be positive")
        w = 2 * altitude_m * math.tan(math.radians(self.horizontal_fov_deg) / 2)
        h = 2 * altitude_m * math.tan(math.radians(self.vertical_fov_deg) / 2)
        return Footprint(width_m=w, height_m=h, area_m2=w * h)


@dataclass(frozen=True)
class Footprint:
    width_m: float
    height_m: float
    area_m2: float


@dataclass(frozen=True)
class FrameObservation:
    altitude_m: float
    ground_speed_mps: float
    overlap_fraction: float
    infected: bool = True  # ground truth for this evaluation


def detection_probability(
    camera: Camera, obs: FrameObservation, *, max_views: int = 3
) -> float:
    """P(detect) from altitude, speed and overlap.

    - altitude: lower is better (quadratic GSD scaling), normalized to reference.
    - speed: faster than reference reduces probability, capped at 0.2x..1.2x.
    - overlap: each additional look multiplies miss probability by (1 - p_single).
    """
    if obs.altitude_m <= 0 or obs.ground_speed_mps < 0:
        raise ValueError("invalid altitude or speed")
    if not 0 <= obs.overlap_fraction < 1:
        raise ValueError("overlap_fraction must be in [0, 1)")

    alt_factor = (camera.reference_altitude_m / obs.altitude_m) ** 2
    speed_factor = min(1.2, max(0.2, camera.reference_speed_mps / max(obs.ground_speed_mps, 0.1)))
    p_single = min(0.99, camera.base_detection_p * alt_factor * speed_factor)

    views = 1 + min(max_views - 1, int(obs.overlap_fraction * max_views))
    p_detect = 1.0 - (1.0 - p_single) ** views
    return min(0.99, max(0.0, p_detect))


def detect_frame(
    camera: Camera,
    obs: FrameObservation,
    *,
    rng: random.Random | None = None,
) -> bool:
    """Sample one detection event from the probability model."""
    r = rng or random
    if not obs.infected:
        return False  # no false positives in this simple model
    return r.random() < detection_probability(camera, obs)
