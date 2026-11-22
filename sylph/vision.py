"""Phase 2: vision pipeline -- disease detection, geotagging, GeoJSON store.

The detector is an interface: a real YOLOv8n/MobileNet export implements
``DetectionModel``; a deterministic stub is provided for offline tests. No
model weights are shipped in the repo (see .gitignore).
"""

from __future__ import annotations

import json
from collections.abc import Iterable
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Protocol

__all__ = [
    "Detection",
    "DetectionModel",
    "StubDetector",
    "geotag",
    "DetectionStore",
    "DiagnosisResult",
    "diagnose_leaf_offline",
]


@dataclass(frozen=True)
class Detection:
    """A single disease detection with location and confidence."""

    label: str
    confidence: float
    latitude: float
    longitude: float
    timestamp: str
    severity: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


class DetectionModel(Protocol):
    """A model that returns disease classifications for an image path."""

    def detect(self, image_path: str) -> list[tuple[str, float]]:
        """Return (label, confidence) pairs for an image."""


class StubDetector:
    """Deterministic detector used by tests and simulation.

    Encodes a *simple* rule for demonstrations: filenames containing "blight"
    yield a positive detection. Real deployments replace this with a quantized
    YOLO/TFLite model; the rest of the pipeline is unchanged.
    """

    def detect(self, image_path: str) -> list[tuple[str, float]]:
        name = Path(image_path).name.lower()
        if "blight" in name:
            return [("late_blight", 0.91)]
        if "rust" in name:
            return [("leaf_rust", 0.87)]
        return [("healthy", 0.95)]


def geotag(
    detections: Iterable[tuple[str, float]],
    latitude: float,
    longitude: float,
    *,
    timestamp: str | None = None,
) -> list[Detection]:
    """Attach a GPS position and timestamp to raw (label, confidence) pairs.

    With no camera-pose interpolation data, all detections in one frame share
    the vehicle position at capture time. Severity is derived from confidence.
    """
    stamp = timestamp or datetime.now(timezone.utc).isoformat()
    return [
        Detection(
            label=label,
            confidence=conf,
            latitude=latitude,
            longitude=longitude,
            timestamp=stamp,
            severity=round(1.0 - conf, 3),
        )
        for label, conf in detections
    ]


class DetectionStore:
    """Append-only GeoJSON FeatureCollection of detections."""

    def __init__(self) -> None:
        self._detections: list[Detection] = []

    def add(self, detection: Detection) -> None:
        self._detections.append(detection)

    def add_many(self, detections: Iterable[Detection]) -> None:
        for d in detections:
            self.add(d)

    def __len__(self) -> int:
        return len(self._detections)

    @property
    def detections(self) -> tuple[Detection, ...]:
        return tuple(self._detections)

    def to_geojson(self) -> dict:
        return {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": [d.longitude, d.latitude],
                    },
                    "properties": {
                        "label": d.label,
                        "confidence": d.confidence,
                        "timestamp": d.timestamp,
                        "severity": d.severity,
                    },
                }
                for d in self._detections
            ],
        }

    def write_geojson(self, path: str | Path) -> Path:
        """Persist the collection to disk (UTF-8)."""
        p = Path(path)
        p.write_text(json.dumps(self.to_geojson(), indent=2), encoding="utf-8")
        return p


@dataclass(frozen=True)
class DiagnosisResult:
    diagnosis: str
    confidence: float
    offline: bool = True


def diagnose_leaf_offline(
    image_path: str, model: DetectionModel | None = None
) -> DiagnosisResult:
    """Diagnose a leaf from a photo with no network access.

    Uses the supplied on-device model (or the stub) so the phone/Jetson app
    works in the field without connectivity.
    """
    engine = model or StubDetector()
    results = engine.detect(image_path)
    if not results:
        return DiagnosisResult(diagnosis="unknown", confidence=0.0)
    label, confidence = max(results, key=lambda r: r[1])
    return DiagnosisResult(diagnosis=label, confidence=confidence, offline=True)
