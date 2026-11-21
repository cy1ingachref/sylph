"""Scenario fixtures that produce deterministic replay logs.

Each scenario re-flies a short mission with a fixed seed and emits telemetry
frames + events through the same DigitalTwin used by the CLI. The renderer
consumes these frames; it never computes its own movement.
"""

from __future__ import annotations

from math import atan2, degrees

from .battery import BatteryModel
from .design import size_power_system
from .navigation import distance_m
from .replay import MissionEvent, ReplayLog, TelemetryFrame
from .twin import DigitalTwin, Environment, FlightSegment, TwinState


def _grid(lat=36.8, lon=10.2, rows=4, spacing_m=20.0, width_m=60.0, alt_m=20.0):
    from .mission import lawnmower_pattern

    return lawnmower_pattern(
        lat, lon, width_m=width_m, height_m=rows * spacing_m,
        spacing_m=spacing_m, altitude_m=alt_m,
    )


def run_scenario(
    name: str = "normal",
    *,
    seed: int = 0,
    low_battery: bool = False,
    wind_mps: float = 0.0,
    gust_amplitude_mps: float = 0.0,
    gps_degraded: bool = False,
) -> ReplayLog:
    """Run one of the demo scenarios and return a replayable log.

    Scenarios: ``normal``, ``low_battery`` (forces RTL), ``wind_gust``.
    """
    if name not in ("normal", "low_battery", "wind_gust"):
        raise ValueError(f"unknown scenario: {name}")
    low_battery = low_battery or name == "low_battery"
    budget = size_power_system()
    battery = BatteryModel(
        nominal_v=budget.voltage_v, capacity_ah=budget.capacity_ah,
        enabled=True,
    )
    env = Environment(
        wind_speed_mps=wind_mps,
        gust_amplitude_mps=gust_amplitude_mps,
        turbulence_seed=seed,
        turbulence_sigma_mps=0.3 if wind_mps or gust_amplitude_mps else 0.0,
    )
    twin = DigitalTwin(budget, environment=env, battery_model=battery)
    start_energy = budget.usable_energy_wh * (0.15 if low_battery else 0.9)
    state = TwinState(start_energy)
    wps = _grid()
    log = ReplayLog(scenario=name)
    log.waypoints = [
        {"id": f"WP{i+1}", "lat_deg": w.latitude, "lon_deg": w.longitude,
         "alt_m": w.altitude_m}
        for i, w in enumerate(wps)
    ]
    lat, lon = 36.8, 10.2
    speed = 10.0
    log.add_frame(TelemetryFrame(0.0, lat, lon, 20.0, 0.0, 0.0,
                                 100 * start_energy / budget.usable_energy_wh,
                                 start_energy, 0))
    log.add_event(MissionEvent(0.0, "takeoff", "scenario started"))
    if low_battery:
        # This scenario simulates a low-battery state discovered at takeoff:
        # no survey legs are flown; RTL is the only safe action.
        log.add_event(MissionEvent(
            0.0, "battery_low", f"battery {100 * start_energy / budget.usable_energy_wh:.0f}%"
        ))
        log.add_event(MissionEvent(0.0, "rtl", "return to launch before survey"))
        log.add_event(MissionEvent(0.0, "failed", "mission aborted for low battery"))
        return log

    for i, wp in enumerate(wps, start=1):
        d = distance_m(lat, lon, wp.latitude, wp.longitude)
        heading = degrees(atan2(wp.longitude - lon, wp.latitude - lat)) % 360
        try:
            twin.fly(state, FlightSegment(d, speed))
        except RuntimeError:
            log.add_event(MissionEvent(state.time_s, "battery_low",
                                       "RTL triggered: insufficient energy"))
            log.add_event(MissionEvent(state.time_s, "rtl", "returning to launch"))
            break
        lat, lon = wp.latitude, wp.longitude
        pct = 100 * state.energy_remaining_wh / budget.usable_energy_wh
        log.add_frame(TelemetryFrame(
            state.time_s, lat, lon, wp.altitude_m, speed, heading,
            pct, state.energy_remaining_wh, i,
        ))
        log.add_event(MissionEvent(state.time_s, "waypoint_reached",
                                   f"WP{i}", lat, lon))
        if low_battery and pct < 20:
            log.add_event(MissionEvent(state.time_s, "battery_low",
                                       f"battery {pct:.0f}%"))

        # deterministic pseudo-detection for the demo
        if (i * 7 + seed) % 5 == 0:
            det = {
                "id": f"DET{i}", "label": "late_blight",
                "confidence": 0.8, "severity": 0.2,
                "lat_deg": lat, "lon_deg": lon,
                "t_s": state.time_s,
            }
            log.detections.append(det)
            log.add_event(MissionEvent(state.time_s, "detection",
                                       "late_blight", lat, lon))
    else:
        log.add_event(MissionEvent(state.time_s, "complete", "survey finished"))
        return log

    log.add_event(MissionEvent(state.time_s, "failed", "mission ended early"))
    return log
