"""Sylph digital-twin CLI: design, optimize, fly and compare."""

from __future__ import annotations

import argparse
import json
import logging
import sys
from collections.abc import Callable

from .battery import BatteryModel
from .design import DEFAULT_BOM, size_power_system
from .mission import lawnmower_pattern
from .monte_carlo import monte_carlo
from .optimizer import ValueRegion, optimal_grid_spacing, optimal_speed
from .replay import SCHEMA_VERSION, to_demo_payload
from .scenarios import run_scenario
from .simulator import simulate_mission
from .sitl import MissionItem, TwinFlightController, write_mavlink_json, write_qgc_wpl
from .twin import DigitalTwin
from .waypoints import Waypoint

__all__ = ["main", "configure_logging"]

LOG = logging.getLogger("sylph.cli")


def configure_logging(*, verbose: bool = False) -> None:
    """Structured CLI logging to stdout (JSON-friendly format preserved).

    User-facing CLI output remains on stdout so existing scripts and tests
    that parse it keep working; diagnostics use the standard logging module.
    """
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter("%(message)s"))
    LOG.handlers.clear()
    LOG.addHandler(handler)
    LOG.setLevel(logging.DEBUG if verbose else logging.INFO)
    LOG.propagate = False


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="sylph", description="Sylph drone design and simulation tools")
    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("design", help="show BOM and power-system sizing")

    o = sub.add_parser("optimize", help="show optimal speed and grid spacing")
    o.add_argument("--altitude", type=float, default=20.0)
    o.add_argument("--fov", type=float, default=60.0)
    o.add_argument("--overlap", type=float, default=0.2)

    f = sub.add_parser("fly", help="fly an optimized survey in the digital twin")
    f.add_argument("--width", type=float, default=300.0)
    f.add_argument("--height", type=float, default=300.0)
    f.add_argument("--altitude", type=float, default=20.0)
    f.add_argument("--wind", type=float, default=0.0)
    f.add_argument("--speed", type=float, default=None)
    f.add_argument("--spacing", type=float, default=None)
    f.add_argument("--json", action="store_true")
    # Opt-in realism (defaults preserve baseline output)
    f.add_argument("--gust", type=float, default=0.0, help="sinusoidal gust amplitude m/s")
    f.add_argument("--turbulence", type=float, default=0.0, help="turbulence sigma m/s")
    f.add_argument(
        "--seed", type=int, default=None,
        help="seed for turbulence/sensors (reproducible)",
    )
    f.add_argument("--battery-realism", action="store_true", help="enable OCV sag + Peukert")
    f.add_argument("--sensor-noise", action="store_true", help="add seeded GPS/baro/voltage noise")
    f.add_argument(
        "--revisit-lat", type=float, default=None,
        help="infected-area latitude to revisit",
    )
    f.add_argument(
        "--revisit-lon", type=float, default=None,
        help="infected-area longitude to revisit",
    )
    f.add_argument("--revisit-value", type=float, default=0.9, help="severity/value of the region")

    e = sub.add_parser("export", help="export a survey mission for ArduPilot/PX4 SITL")
    e.add_argument("--format", choices=["wpl", "mavlink-json"], default="wpl")
    e.add_argument("--output", default="sylph_mission.waypoints")
    e.add_argument("--width", type=float, default=300.0)
    e.add_argument("--height", type=float, default=300.0)
    e.add_argument("--altitude", type=float, default=20.0)
    e.add_argument("--spacing", type=float, default=None)
    e.add_argument("--revisit-lat", type=float, default=None)
    e.add_argument("--revisit-lon", type=float, default=None)
    e.add_argument("--revisit-radius", type=float, default=30.0)
    e.add_argument("--revisit-value", type=float, default=0.9)
    e.add_argument("--sitl-host", default=None, help="explicit SITL host; no connection unless set")
    e.add_argument("--sitl-port", type=int, default=5762)
    e.add_argument("--sitl-mode", default="auto")

    m = sub.add_parser("montecarlo", help="evaluate success rate and RTL margin across seeds")
    m.add_argument("--runs", type=int, default=50)
    m.add_argument("--seed-start", type=int, default=0)
    m.add_argument("--gust", type=float, default=2.0)
    m.add_argument("--turbulence", type=float, default=0.5)
    m.add_argument("--json", action="store_true")

    s = sub.add_parser("scenario", help="emit a deterministic demo replay payload (JSON)")
    s.add_argument("--name", choices=["normal", "low_battery", "wind_gust"], default="normal")
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--output", default=None, help="write payload to file instead of stdout")

    sub.add_parser("compare", help="optimized vs naive (slow speed, wide spacing) mission")
    return p


def _design(_args: argparse.Namespace | None = None) -> int:
    b = size_power_system()
    LOG.info(f"BOM mass: {DEFAULT_BOM.total_mass_g} g, cost: ${DEFAULT_BOM.total_cost_usd}")
    for name, qty, mass, cost in DEFAULT_BOM.as_rows():
        LOG.info(f"  {name}: x{qty}, {mass} g, ${cost}")
    LOG.info(f"Pack: {b.voltage_v} V x {b.capacity_ah} Ah = {b.pack_energy_wh:.1f} Wh")
    LOG.info(f"Usable (80% DoD): {b.usable_energy_wh:.1f} Wh")
    LOG.info(f"Hover power: {b.hover_power_w:.1f} W (+ avionics {b.avionics_power_w:.1f} W)")
    LOG.info(f"Estimated hover endurance: {b.estimated_hover_minutes:.1f} min")
    return 0


def _optimize(args: argparse.Namespace) -> int:
    twin = DigitalTwin(size_power_system())
    v = optimal_speed(twin)
    sp = optimal_grid_spacing(args.altitude, camera_fov_deg=args.fov, overlap_fraction=args.overlap)
    LOG.info(f"Optimal cruise speed: {v:.2f} m/s ({v * 3.6:.1f} km/h)")
    LOG.info(
        f"Optimal grid spacing at {args.altitude:.0f} m "
        f"(FOV {args.fov:.0f}deg, {args.overlap:.0%} overlap): {sp:.2f} m"
    )
    return 0


def _fly(args: argparse.Namespace) -> int:
    budget = size_power_system()
    battery = (
        BatteryModel(nominal_v=budget.voltage_v, capacity_ah=budget.capacity_ah)
        if args.battery_realism else None
    )
    regions = None
    if args.revisit_lat is not None and args.revisit_lon is not None:
        regions = [ValueRegion(args.revisit_lat, args.revisit_lon, 30.0, args.revisit_value)]
    r = simulate_mission(
        width_m=args.width, height_m=args.height, altitude_m=args.altitude,
        wind_mps=args.wind, speed_mps=args.speed, spacing_m=args.spacing,
        gust_amplitude_mps=args.gust, turbulence_sigma_mps=args.turbulence,
        turbulence_seed=args.seed, battery_model=battery,
        sensor_seed=args.seed if args.sensor_noise else None,
        value_regions=regions,
    )
    if args.json:
        LOG.info(json.dumps(r.__dict__, indent=2))
        return 0 if r.success else 1
    LOG.info(f"Sylph survey simulation: {'SUCCESS' if r.success else 'FAILED'}")
    LOG.info(
        f"  Speed: {r.speed_mps:.2f} m/s, spacing: {r.spacing_m:.2f} m, "
        f"waypoints: {r.waypoints}"
    )
    LOG.info(
        f"  Survey path: {r.original_route_m:.0f} m "
        f"({r.waypoints} waypoints, lawnmower coverage)"
    )
    LOG.info(f"  Flew: {r.distance_m:.0f} m in {r.time_s / 60:.1f} min")
    LOG.info(f"  Energy used: {r.energy_wh:.1f} Wh ({r.energy_used_pct:.1f}% of usable)")
    LOG.info(f"  Remaining: {r.energy_remaining_wh:.1f} Wh")
    LOG.info(f"  Detections: {r.detections} ({r.infected} infected)")
    if r.terminal_voltage_v is not None:
        LOG.info(f"  Terminal voltage: {r.terminal_voltage_v:.2f} V")
    if r.sensor_sample:
        s = r.sensor_sample
        LOG.info(f"  Sensor sample: GPS {s['latitude']:.6f},{s['longitude']:.6f} "
              f"baro {s['baro_altitude_m']:.2f} m volt {s['voltage_v']:.2f} V")
    if r.revisit_regions:
        LOG.info(f"  Value revisits: {r.revisit_regions} region(s), +{r.revisit_extra_m:.0f} m")
    for line in r.log:
        LOG.info(f"  - {line}")
    LOG.info(r.report_text)
    return 0 if r.success else 1


def _export(args: argparse.Namespace) -> int:
    budget = size_power_system()
    twin = DigitalTwin(budget)
    sp = args.spacing or optimal_grid_spacing(args.altitude)
    wps = lawnmower_pattern(
        36.8, 10.2, width_m=args.width, height_m=args.height,
        spacing_m=sp, altitude_m=args.altitude,
    )
    if args.revisit_lat is not None and args.revisit_lon is not None:
        from .optimizer import plan_revisits
        region = ValueRegion(
            args.revisit_lat, args.revisit_lon, args.revisit_radius, args.revisit_value
        )
        combined, _ = plan_revisits(wps, [region], spacing_m=sp)
        wps = [
            Waypoint(w.latitude, w.longitude, args.altitude) if w.altitude_m == 0.0 else w
            for w in combined
        ]
    items = [MissionItem(w.latitude, w.longitude, w.altitude_m) for w in wps]
    out = args.output
    if args.format == "wpl":
        path = write_qgc_wpl(items, out)
    else:
        path = write_mavlink_json(items, out)
    LOG.info(f"Exported {len(items)} waypoints to {path} ({args.format})")
    # Sanity-check the adapter round trip without claiming a real SITL flight.
    fc = TwinFlightController(twin, optimal_speed(twin))
    fc.upload(items)
    result = fc.execute()
    LOG.info(f"Digital-twin dry run: {result['distance_m']:.0f} m, "
          f"{result['energy_wh']:.1f} Wh ({result['backend']})")
    if args.sitl_host:
        # Explicit opt-in only: never probe or connect implicitly.
        from .mavlink_sitl import MavlinkSitlController, PymavlinkTransport
        transport = PymavlinkTransport(args.sitl_host, args.sitl_port)
        sitl = MavlinkSitlController(transport, mode=args.sitl_mode)
        sitl.upload(items)
        try:
            sitl_result = sitl.execute()
            LOG.info(f"SITL run: {sitl_result['samples']} telemetry samples, "
                  f"success={sitl_result['success']}")
        finally:
            transport.close()
    return 0


def _montecarlo(args: argparse.Namespace) -> int:
    r = monte_carlo(
        runs=args.runs, seed_start=args.seed_start,
        gust_amplitude_mps=args.gust, turbulence_sigma_mps=args.turbulence,
    )
    if args.json:
        LOG.info(json.dumps(r.__dict__, indent=2))
    else:
        LOG.info(r.to_text())
    return 0


def _scenario(args: argparse.Namespace) -> int:
    log = run_scenario(
        args.name, seed=args.seed,
        wind_mps=4.0 if args.name == "wind_gust" else 0.0,
        gust_amplitude_mps=2.0 if args.name == "wind_gust" else 0.0,
    )
    payload = to_demo_payload(log, mission_id=f"demo-{args.name}-{args.seed}")
    text = json.dumps(payload, indent=2)
    if args.output:
        from pathlib import Path

        Path(args.output).write_text(text + "\n", encoding="utf-8")
        LOG.info(f"Wrote {args.output} (schema_version={SCHEMA_VERSION}, "
              f"{len(payload['frames'])} frames, {len(payload['events'])} events)")
    else:
        LOG.info(text)
    return 0


def _compare(_args: argparse.Namespace | None = None) -> int:
    opt = simulate_mission()
    naive = simulate_mission(speed_mps=6.0, spacing_m=10.0)
    LOG.info("Optimized vs naive survey (same 300x300 m field):")
    LOG.info(
        f"  {'':12} {'speed':>8} {'spacing':>9} "
        f"{'route m':>9} {'time min':>9} {'Wh':>7} {'ok':>5}"
    )
    for label, r in (("optimized", opt), ("naive", naive)):
        LOG.info(
            f"  {label:12} {r.speed_mps:8.2f} {r.spacing_m:9.2f} "
            f"{r.optimized_route_m:9.0f} {r.time_s / 60:9.1f} "
            f"{r.energy_wh:7.1f} {str(r.success):>5}"
        )
    if not opt.success:
        LOG.info("Optimized mission did not complete")
        return 1
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    parser.add_argument("--verbose", action="store_true", help="enable debug logging")
    args = parser.parse_args(argv)
    configure_logging(verbose=bool(getattr(args, "verbose", False)))
    dispatch: dict[str, Callable[[argparse.Namespace], int]] = {
        "design": _design,
        "optimize": _optimize,
        "fly": _fly,
        "export": _export,
        "montecarlo": _montecarlo,
        "scenario": _scenario,
        "compare": _compare,
    }
    return dispatch[args.command](args)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
