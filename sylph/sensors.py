"""Opt-in, seeded sensor noise for SITL-style testing.

ASSUMPTIONS: Gaussian, zero-mean, independent per sample; sigma values are
typical consumer-grade hardware and are configurable. Same seed -> same stream,
so tests and simulated flights are reproducible. With ``seed=None`` the
sensors are perfect pass-through (the baseline behaviour).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

__all__ = ["SensorConfig", "SensorSuite", "Measurement"]


@dataclass(frozen=True)
class SensorConfig:
    gps_position_sigma_m: float = 1.5      # horizontal GPS noise
    gps_altitude_sigma_m: float = 3.0
    barometer_sigma_m: float = 0.5
    voltage_sigma_v: float = 0.05


@dataclass(frozen=True)
class Measurement:
    latitude: float
    longitude: float
    gps_altitude_m: float
    baro_altitude_m: float
    voltage_v: float


def _deg_per_m_lat() -> float:
    return 1.0 / 111_320.0


def _deg_per_m_lon(latitude: float) -> float:
    return 1.0 / max(111_320.0 * math.cos(math.radians(latitude)), 1.0)


class SensorSuite:
    """Sample noisy measurements from a true state; deterministic with a seed."""

    def __init__(self, config: SensorConfig | None = None, *, seed: int | None = None) -> None:
        self.config = config or SensorConfig()
        self._rng = random.Random(seed) if seed is not None else None

    def _noise(self, sigma: float) -> float:
        return 0.0 if self._rng is None else self._rng.gauss(0.0, sigma)

    def measure(
        self,
        *,
        latitude: float,
        longitude: float,
        altitude_m: float,
        voltage_v: float,
    ) -> Measurement:
        lat = latitude + self._noise(self.config.gps_position_sigma_m) * _deg_per_m_lat()
        lon = longitude + self._noise(self.config.gps_position_sigma_m) * _deg_per_m_lon(latitude)
        gps_alt = altitude_m + self._noise(self.config.gps_altitude_sigma_m)
        baro_alt = altitude_m + self._noise(self.config.barometer_sigma_m)
        volt = voltage_v + self._noise(self.config.voltage_sigma_v)
        return Measurement(lat, lon, gps_alt, baro_alt, volt)
