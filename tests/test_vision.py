"""Phase 2 tests: detection, geotagging, GeoJSON store, offline diagnosis."""

import json

from sylph.vision import (
    Detection,
    DetectionStore,
    StubDetector,
    diagnose_leaf_offline,
    geotag,
)


def test_stub_detector_classifies_blight_and_healthy():
    d = StubDetector()
    assert d.detect("/img/tomato_blight_04.jpg") == [("late_blight", 0.91)]
    assert d.detect("/img/leaf.jpg")[0][0] == "healthy"


def test_geotag_attaches_position_and_timestamp():
    tagged = geotag([("late_blight", 0.9)], 36.8, 10.2, timestamp="2026-11-18T00:00:00+00:00")
    assert len(tagged) == 1
    assert tagged[0].latitude == 36.8
    assert tagged[0].longitude == 10.2
    assert tagged[0].timestamp == "2026-11-18T00:00:00+00:00"
    assert tagged[0].severity == 0.1


def test_store_produces_valid_geojson():
    store = DetectionStore()
    store.add_many(
        geotag(
            [("late_blight", 0.9), ("healthy", 0.95)],
            36.8,
            10.2,
            timestamp="t",
        )
    )
    gj = store.to_geojson()
    assert gj["type"] == "FeatureCollection"
    assert len(gj["features"]) == 2
    lon, lat = gj["features"][0]["geometry"]["coordinates"]
    assert (lon, lat) == (10.2, 36.8)
    assert gj["features"][0]["properties"]["label"] == "late_blight"


def test_store_write_geojson(tmp_path):
    store = DetectionStore()
    store.add(Detection("rust", 0.8, 1.0, 2.0, "t"))
    p = store.write_geojson(tmp_path / "detections.geojson")
    data = json.loads(p.read_text(encoding="utf-8"))
    assert len(data["features"]) == 1
    assert data["features"][0]["properties"]["label"] == "rust"


def test_offline_diagnosis_no_network():
    result = diagnose_leaf_offline("/img/corn_rust.jpg")
    assert result.diagnosis == "leaf_rust"
    assert result.offline is True
    assert result.confidence == 0.87


def test_offline_diagnosis_with_custom_model():
    class Model:
        def detect(self, image_path):
            return [("custom_disease", 0.7)]

    result = diagnose_leaf_offline("/img/x.jpg", Model())
    assert result.diagnosis == "custom_disease"
