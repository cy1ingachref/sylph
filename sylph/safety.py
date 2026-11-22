"""Ground safety and preflight policy.

Nothing in this module talks to hardware. It encodes the decision rules that
must hold before flight is allowed, and is fully testable offline.

OPERATING MODES (explicit, never inferred):
  * ``simulation`` -- synthetic telemetry, no vehicle connection. Authorized
    by default when checks pass.
  * ``sitl``       -- software-in-the-loop flight controller. Authorized only
    with explicit ``sitl_authorized=True``.
  * ``hardware``   -- real aircraft. DISABLED BY DEFAULT AND IN CODE: requires
    ``hardware_arm_permitted=True`` plus independent interlocks, which this
    module never grants on its own. There is no hardware adapter in the repo.

The previous ``safe`` property aggregated checks only and ignored mode; it is
retained as ``checks_passed`` semantics for compatibility, but every arming
decision must use ``authorized`` / ``hardware_arm_permitted``.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

__all__ = [
    "OperatingMode",
    "PreflightReport",
    "PreflightError",
    "preflight",
    "require_arm_authorization",
]


class PreflightError(RuntimeError):
    """Raised when a flight is attempted without an authorized safe state."""


class OperatingMode:
    SIMULATION = "simulation"
    SITL = "sitl"
    HARDWARE = "hardware"
    ALL = (SIMULATION, SITL, HARDWARE)


@dataclass(frozen=True)
class PreflightReport:
    """Result of a preflight check, including explicit mode authorization."""

    checks: dict[str, bool]
    geofence_set: bool
    mode: str = OperatingMode.SIMULATION
    telemetry_fresh: bool = True
    telemetry_stale: bool = False
    sitl_authorized: bool = False
    hardware_arm_permitted: bool = False
    interlocks_ok: bool = False
    failure_log: tuple[str, ...] = ()

    @property
    def checks_passed(self) -> bool:
        """All vehicle checks pass and a geofence exists."""
        return bool(self.checks) and all(self.checks.values()) and self.geofence_set

    @property
    def failures(self) -> list[str]:
        """Human-readable list of every blocking condition."""
        out = [name for name, ok in sorted(self.checks.items()) if not ok]
        if not self.geofence_set:
            out.append("geofence_set")
        if self.telemetry_stale or not self.telemetry_fresh:
            out.append("telemetry_stale_or_missing")
        if self.mode not in OperatingMode.ALL:
            out.append(f"invalid_mode:{self.mode}")
        if self.mode == OperatingMode.SITL and not self.sitl_authorized:
            out.append("sitl_not_authorized")
        if self.mode == OperatingMode.HARDWARE:
            if not self.hardware_arm_permitted:
                out.append("hardware_arm_not_permitted")
            if not self.interlocks_ok:
                out.append("hardware_interlocks_not_ok")
        return out + list(self.failure_log)

    @property
    def mode_authorized(self) -> bool:
        """Whether this operating mode is authorized (never implies hardware)."""
        if self.mode == OperatingMode.SIMULATION:
            return True
        if self.mode == OperatingMode.SITL:
            return self.sitl_authorized
        if self.mode == OperatingMode.HARDWARE:
            return self.hardware_arm_permitted and self.interlocks_ok
        return False

    @property
    def safe(self) -> bool:
        """Backward-compatible aggregate: checks pass AND mode authorized AND
        telemetry is fresh. This is the only boolean callers should use for
        'may this system proceed', and it can never authorize hardware alone.
        """
        return (
            self.checks_passed
            and self.mode_authorized
            and self.telemetry_fresh
            and not self.telemetry_stale
        )

    @property
    def hardware_arm_permitted_now(self) -> bool:
        """Explicit hardware gate. Always False unless a reviewed policy and
        independent interlocks are present and every check still passes.
        """
        return (
            self.mode == OperatingMode.HARDWARE
            and self.hardware_arm_permitted
            and self.interlocks_ok
            and self.checks_passed
            and self.telemetry_fresh
            and not self.telemetry_stale
        )

    def invalidated(self, reason: str) -> PreflightReport:
        """Return a fail-closed copy after a safety condition changes post-arm."""
        return replace(
            self,
            telemetry_fresh=False,
            failure_log=self.failure_log + (f"invalidated_after_arm:{reason}",),
        )


def preflight(
    *,
    firmware_ok: bool,
    gps_fix: bool,
    failsafes_tested: bool,
    geofence_set: bool,
    battery_ok: bool,
    simulation: bool | None = None,  # legacy compatibility
    mode: str | None = None,
    telemetry_fresh: bool = True,
    telemetry_stale: bool = False,
    sitl_authorized: bool = False,
    hardware_arm_permitted: bool = False,
    interlocks_ok: bool = False,
    arm: bool = False,
) -> PreflightReport:
    """Evaluate preflight conditions for an explicit operating mode.

    ``simulation=True`` is accepted for old callers and maps to
    ``mode="simulation"``. New callers should pass ``mode`` directly.
    Hardware authorization is never granted implicitly.
    """
    if mode is None:
        mode = OperatingMode.SIMULATION if simulation is not False else OperatingMode.HARDWARE
    if mode not in OperatingMode.ALL:
        raise ValueError(f"unknown operating mode: {mode}")
    if simulation is not None and simulation != (mode == OperatingMode.SIMULATION):
        raise ValueError("simulation flag contradicts mode")

    checks = {
        "firmware_ok": bool(firmware_ok),
        "gps_fix": bool(gps_fix),
        "failsafes_tested": bool(failsafes_tested),
        "battery_ok": bool(battery_ok),
    }
    report = PreflightReport(
        checks=checks,
        geofence_set=bool(geofence_set),
        mode=mode,
        telemetry_fresh=bool(telemetry_fresh),
        telemetry_stale=bool(telemetry_stale),
        sitl_authorized=bool(sitl_authorized),
        hardware_arm_permitted=bool(hardware_arm_permitted),
        interlocks_ok=bool(interlocks_ok),
    )
    if arm:
        require_arm_authorization(report)
    return report


def require_arm_authorization(report: PreflightReport) -> None:
    """Fail closed unless the report authorizes arming in its explicit mode."""
    if not report.safe:
        raise PreflightError("refusing to arm: " + ", ".join(report.failures))
