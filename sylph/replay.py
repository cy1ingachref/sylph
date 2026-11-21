"""Deterministic replay log and versioned 3D demo data contract.

The simulator emits telemetry frames and events; the renderer consumes them.
No movement is invented in the renderer. Units are explicit in field names.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

__all__ = [
    "SCHEMA_VERSION",
    "TelemetryFrame",
    "MissionEvent",
    "ReplayLog",
    "to_demo_payload",
]


SCHEMA_VERSION = 1


@dataclass(frozen=True)
class TelemetryFrame:
    """One simulation-clock sample, with explicit units in the names."""

    t_s: float
    lat_deg: float
    lon_deg: float
    alt_m: float
    ground_speed_mps: float
    heading_deg: float
    battery_pct: float
    battery_wh: float
    current_wp: int
    mode: str = "simulation"


@dataclass(frozen=True)
class MissionEvent:
    t_s: float
    kind: str  # waypoint_reached, rtl, abort, battery_low, detection, complete, failed
    detail: str = ""
    lat_deg: float | None = None
    lon_deg: float | None = None


@dataclass
class ReplayLog:
    scenario: str
    frames: list[TelemetryFrame] = field(default_factory=list)
    events: list[MissionEvent] = field(default_factory=list)
    waypoints: list[dict] = field(default_factory=list)
    detections: list[dict] = field(default_factory=list)

    def add_frame(self, frame: TelemetryFrame) -> None:
        self.frames.append(frame)

    def add_event(self, event: MissionEvent) -> None:
        self.events.append(event)

    def write_jsonl(self, path: str | Path) -> Path:
        """Write a replayable JSONL log (header + frames + events)."""
        p = Path(path)
        lines = [json.dumps({"schema_version": SCHEMA_VERSION, "scenario": self.scenario})]
        lines.extend(json.dumps({"frame": asdict(f)}) for f in self.frames)
        lines.extend(json.dumps({"event": asdict(e)}) for e in self.events)
        p.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
        return p

    def to_demo_payload(
        self, *, mode: str = "simulation", mission_id: str = "demo-field-001"
    ) -> dict:
        """Build the versioned payload consumed by the 3D demo renderer."""
        return to_demo_payload(self, mode=mode, mission_id=mission_id)


def to_demo_payload(
    log: ReplayLog, *, mode: str = "simulation", mission_id: str = "demo-field-001"
) -> dict:
    """Versioned state model. Missing values are never silently zeroed."""
    if not log.frames:
        raise ValueError("cannot export an empty replay log")
    last = log.frames[-1]
    return {
        "schema_version": SCHEMA_VERSION,
        "mode": mode,
        "mission_id": mission_id,
        "simulation_notice": "SIMULATION — NO REAL FLIGHT CONTROL",
        "vehicle": {
            "position": {"lat_deg": last.lat_deg, "lon_deg": last.lon_deg, "alt_m": last.alt_m},
            "heading_deg": last.heading_deg,
            "speed_mps": last.ground_speed_mps,
            "battery_pct": last.battery_pct,
            "battery_wh": last.battery_wh,
            "state": last.mode,
        },
        "mission": {
            "state": "running",
            "current_waypoint": last.current_wp,
            "waypoints": log.waypoints,
        },
        "events": [asdict(e) for e in log.events],
        "detections": log.detections,
        "frames": [asdict(f) for f in log.frames],
    }
