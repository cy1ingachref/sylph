"""Phase 6: full mission simulator -- ties design, twin, optimizer and detect/report."""

from __future__ import annotations

from dataclasses import dataclass, field

from .battery import BatteryModel
from .design import PowerBudget, size_power_system
from .mission import lawnmower_pattern
from .navigation import distance_m
from .optimizer import (
    optimal_grid_spacing,
    optimal_speed,
    path_length_m,
    plan_revisits,
    revisit_extra_distance_m,
    ValueRegion,
)
from .reporting import build_report
from .sensors import SensorConfig, SensorSuite
from .twin import DigitalTwin, Environment, FlightSegment, TwinState
from .vision import Detection, DetectionStore
from .waypoints import Waypoint


@dataclass
class SimulationResult:
    speed_mps: float
    spacing_m: float
    waypoints: int
    original_route_m: float
    optimized_route_m: float
    distance_m: float
    time_s: float
    energy_wh: float
    energy_remaining_wh: float
    energy_used_pct: float
    detections: int
    infected: int
    success: bool
    report_text: str
    log: list[str] = field(default_factory=list)
    # Opt-in realism / SITL fields (defaults keep baseline JSON compatible)
    terminal_voltage_v: float | None = None
    sensor_sample: dict = field(default_factory=dict)
    revisit_regions: int = 0
    revisit_extra_m: float = 0.0


def simulate_mission(
    *,
    center_lat: float = 36.8,
    center_lon: float = 10.2,
    width_m: float = 300.0,
    height_m: float = 300.0,
    altitude_m: float = 20.0,
    speed_mps: float | None = None,
    spacing_m: float | None = None,
    wind_mps: float = 0.0,
    budget: PowerBudget | None = None,
    detection_rate: float = 0.3,
    gust_amplitude_mps: float = 0.0,
    gust_period_s: float = 10.0,
    turbulence_sigma_mps: float = 0.0,
    turbulence_seed: int | None = None,
    battery_model: BatteryModel | None = None,
    sensor_seed: int | None = None,
    sensor_config: SensorConfig | None = None,
    value_regions: list[ValueRegion] | None = None,
    value_threshold: float = 0.5,
) -> SimulationResult:
    """Fly an optimized grid survey in the digital twin and report results.

    All realism features are opt-in; defaults reproduce the baseline exactly.
    """
    budget = budget or size_power_system()
    env = Environment(
        wind_speed_mps=wind_mps,
        gust_amplitude_mps=gust_amplitude_mps,
        gust_period_s=gust_period_s,
        turbulence_sigma_mps=turbulence_sigma_mps,
        turbulence_seed=turbulence_seed,
    )
    twin = DigitalTwin(budget, environment=env, battery_model=battery_model)
    sensors = (
        SensorSuite(sensor_config, seed=sensor_seed) if sensor_seed is not None else None
    )
    v = speed_mps if speed_mps is not None else optimal_speed(twin)
    sp = spacing_m if spacing_m is not None else optimal_grid_spacing(altitude_m)

    wps = lawnmower_pattern(
        center_lat, center_lon, width_m=width_m, height_m=height_m,
        spacing_m=sp, altitude_m=altitude_m,
    )
    points = [(w.latitude, w.longitude) for w in wps]
    # A boustrophedon lawnmower is already the coverage-optimal traversal for
    # a rectangular field: reordering its waypoints would "shorten" the route
    # by skipping rows, i.e. it would cheat on coverage. Keep the order.
    base_wps = wps
    selected_regions = []
    extra_m = 0.0
    if value_regions:
        combined, selected_regions = plan_revisits(
            base_wps, value_regions, spacing_m=sp, value_threshold=value_threshold
        )
        # Revisit sub-grid waypoints inherit the survey altitude.
        extra_m = revisit_extra_distance_m(base_wps, combined)
        ordered_wps = [
            Waypoint(w.latitude, w.longitude, altitude_m)
            if w.altitude_m == 0.0 else w
            for w in combined
        ]
        log_prefix = [
            f"Value revisits: {len(selected_regions)} region(s), +{extra_m:.0f} m"
        ]
    else:
        ordered_wps = base_wps
        log_prefix = []
    original = path_length_m(points)
    optimized = original

    state = TwinState(energy_remaining_wh=budget.usable_energy_wh)
    log: list[str] = list(log_prefix)
    last = (center_lat, center_lon)
    success = True
    store = DetectionStore()
    leg = 0
    for i, w in enumerate(ordered_wps):
        leg += 1
        d = distance_m(last[0], last[1], w.latitude, w.longitude)
        into_wind = 1.0 if ((leg % 2) == 1) else -1.0  # alternate legs into/with wind
        try:
            twin.fly(
                state, FlightSegment(distance_m=d, airspeed_mps=v), into_wind=into_wind
            )
        except RuntimeError:
            success = False
            log.append(f"Battery depleted at waypoint {i + 1}/{len(ordered_wps)}")
            break
        last = (w.latitude, w.longitude)
        # Deterministic pseudo-detection from position, no randomness.
        if ((i * 37 + leg * 11) % 100) / 100.0 < detection_rate:
            label = "late_blight" if (i % 3 == 0) else "healthy"
            store.add(
                Detection(label, 0.9, w.latitude, w.longitude, "sim", severity=0.1)
            )

    # Return to launch if energy allows.
    rtl_d = distance_m(last[0], last[1], center_lat, center_lon)
    if success:
        try:
            twin.fly(
                state,
                FlightSegment(distance_m=rtl_d, airspeed_mps=v),
                into_wind=1.0,
            )
            log.append("Returned to launch safely")
        except RuntimeError:
            success = False
            log.append("Battery depleted before RTL")

    used_pct = (
        (budget.usable_energy_wh - state.energy_remaining_wh)
        / budget.usable_energy_wh
        * 100.0
    )
    sensor_sample: dict = {}
    if sensors is not None:
        meas = sensors.measure(
            latitude=last[0], longitude=last[1],
            altitude_m=altitude_m,
            voltage_v=state.terminal_voltage_v or budget.voltage_v,
        )
        sensor_sample = {
            "latitude": meas.latitude, "longitude": meas.longitude,
            "gps_altitude_m": meas.gps_altitude_m,
            "baro_altitude_m": meas.baro_altitude_m,
            "voltage_v": meas.voltage_v,
        }

    report = build_report(list(store.detections))
    return SimulationResult(
        speed_mps=v,
        spacing_m=sp,
        waypoints=len(ordered_wps),
        original_route_m=original,
        optimized_route_m=optimized,
        distance_m=state.distance_m,
        time_s=state.time_s,
        energy_wh=budget.usable_energy_wh - state.energy_remaining_wh,
        energy_remaining_wh=state.energy_remaining_wh,
        energy_used_pct=used_pct,
        detections=report.total_detections,
        infected=report.infected_detections,
        success=success,
        report_text=report.to_text(),
        log=log,
        terminal_voltage_v=state.terminal_voltage_v,
        sensor_sample=sensor_sample,
        revisit_regions=len(selected_regions),
        revisit_extra_m=extra_m,
    )
