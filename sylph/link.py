"""Phase 1: telemetry link abstraction.

A minimal in-memory MAVLink-style link. Real hardware adapters (pymavlink /
MAVSDK) can implement the same ``TelemetryLink`` protocol; the decision logic
in this package only depends on the protocol, so it stays testable offline.
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = ["VehicleState", "TelemetryLink", "SimulatedLink", "send_command"]


@dataclass(frozen=True)
class VehicleState:
    """Snapshot of the vehicle at a point in time."""

    latitude: float
    longitude: float
    altitude_m: float
    battery_pct: float
    ground_speed_mps: float
    armed: bool = False


class TelemetryLink:
    """Protocol that any real or simulated MAVLink transport satisfies."""

    def state(self) -> VehicleState:  # pragma: no cover - interface
        raise NotImplementedError

    def send(self, command: str) -> None:  # pragma: no cover - interface
        raise NotImplementedError


class SimulatedLink(TelemetryLink):
    """Deterministic in-memory link for tests and SITL-style simulation."""

    def __init__(self, state: VehicleState) -> None:
        self._state = state
        self.sent: list[str] = []

    def state(self) -> VehicleState:
        return self._state

    def set_state(self, state: VehicleState) -> None:
        self._state = state

    def send(self, command: str) -> None:
        self.sent.append(command)


def send_command(link: TelemetryLink, command: str) -> None:
    """Send a high-level command string through the link."""
    link.send(command)
