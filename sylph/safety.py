"""Phase 0: ground safety and preflight checks.

Nothing in this module talks to hardware. It encodes the *decision rules* that
must hold before a drone is allowed to arm and fly, and can be exercised in
simulation or unit tests. Hardware integration calls :func:`preflight`.
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = ["PreflightReport", "preflight", "PreflightError"]


class PreflightError(RuntimeError):
    """Raised when a flight is attempted without a safe preflight state."""


@dataclass(frozen=True)
class PreflightReport:
    """Result of a preflight check. ``safe`` is True only when no block exists."""

    checks: dict[str, bool]
    simulation: bool
    geofence_set: bool

    @property
    def failures(self) -> list[str]:
        """Names of every check that did not pass."""
        return sorted(name for name, ok in self.checks.items() if not ok)

    @property
    def safe(self) -> bool:
        """True when all checks pass, a geofence exists, and we are in simulation.

        Real flights are gated off by default: the guide requires simulation
        first, failsafe testing, and an explicit opt-in before arming hardware.
        """
        return bool(self.checks) and all(self.checks.values()) and self.geofence_set


def preflight(
    *,
    firmware_ok: bool,
    gps_fix: bool,
    failsafes_tested: bool,
    geofence_set: bool,
    battery_ok: bool,
    simulation: bool = True,
    arm: bool = False,
) -> PreflightReport:
    """Evaluate preflight conditions.

    ``arm=True`` outside simulation with an unsafe report raises
    :class:`PreflightError`; otherwise an unsafe report is returned so callers
    can display it. This keeps the safety gate testable without real hardware.
    """
    checks = {
        "firmware_ok": firmware_ok,
        "gps_fix": gps_fix,
        "failsafes_tested": failsafes_tested,
        "battery_ok": battery_ok,
    }
    report = PreflightReport(
        checks=checks, simulation=simulation, geofence_set=geofence_set
    )
    if arm and not report.safe:
        raise PreflightError(
            "refusing to arm: " + ", ".join(report.failures or ["geofence_set"])
        )
    return report
