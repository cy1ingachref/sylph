"""Explicit mission state machine with validated, logged transitions.

States: DRAFT, VALIDATING, READY, RUNNING, PAUSED, ABORTING, COMPLETED, FAILED.
Abort/RTL always outrank normal mission commands. Transitions are rejected if
not in the allowed map, and every attempted transition is recorded.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone

__all__ = ["MissionState", "MissionStateError", "MissionStateMachine"]


class MissionState:
    DRAFT = "DRAFT"
    VALIDATING = "VALIDATING"
    READY = "READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    ABORTING = "ABORTING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    ALL = (DRAFT, VALIDATING, READY, RUNNING, PAUSED, ABORTING, COMPLETED, FAILED)


class MissionStateError(RuntimeError):
    """Raised on an invalid state transition."""


_ALLOWED: dict[str, set[str]] = {
    MissionState.DRAFT: {MissionState.VALIDATING, MissionState.FAILED},
    MissionState.VALIDATING: {MissionState.READY, MissionState.FAILED},
    MissionState.READY: {MissionState.RUNNING, MissionState.DRAFT, MissionState.FAILED},
    MissionState.RUNNING: {
        MissionState.PAUSED, MissionState.ABORTING,
        MissionState.COMPLETED, MissionState.FAILED,
    },
    MissionState.PAUSED: {
        MissionState.RUNNING, MissionState.ABORTING, MissionState.FAILED,
    },
    MissionState.ABORTING: {MissionState.COMPLETED, MissionState.FAILED},
    MissionState.COMPLETED: set(),
    MissionState.FAILED: set(),
}


@dataclass(frozen=True)
class Transition:
    from_state: str
    to_state: str
    reason: str
    at: str


@dataclass
class MissionStateMachine:
    state: str = MissionState.DRAFT
    log: list[Transition] = field(default_factory=list)

    def transition(self, to_state: str, *, reason: str = "") -> None:
        if to_state not in MissionState.ALL:
            raise MissionStateError(f"unknown state: {to_state}")
        allowed = _ALLOWED.get(self.state, set())
        if to_state not in allowed:
            raise MissionStateError(
                f"invalid transition {self.state} -> {to_state}"
            )
        self.log.append(
            Transition(
                self.state, to_state, reason,
                datetime.now(timezone.utc).isoformat(),
            )
        )
        self.state = to_state

    def request_abort(self, reason: str = "abort requested") -> None:
        """Abort/RTL outranks everything: force a legal abort transition."""
        if self.state in (MissionState.COMPLETED, MissionState.FAILED,
                          MissionState.ABORTING):
            return  # already terminal or aborting
        if self.state in (MissionState.DRAFT, MissionState.VALIDATING):
            self.transition(MissionState.FAILED, reason=reason)
            return
        self.transition(MissionState.ABORTING, reason=reason)

    @property
    def is_terminal(self) -> bool:
        return self.state in (MissionState.COMPLETED, MissionState.FAILED)
