"""Sylph digital-twin CLI: design, optimize, fly and compare."""

from __future__ import annotations

import argparse
import json

from .battery import BatteryModel
from .camera import Camera, FrameObservation, detection_probability
from .design import DEFAULT_BOM, size_power_system
from .mission import lawnmower_pattern
from .monte_carlo import monte_carlo
from .optimizer import optimal_grid_spacing, optimal_speed, ValueRegion
from .simulator import simulate_mission
from .sitl import MissionItem, write_mavlink_json, write_qgc_wpl, TwinFlightController
from .twin import DigitalTwin
from .waypoints import Waypoint


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
    f.add_argument("--seed", type=int, default=None, help="seed for turbulence/sensors (reproducible)")
    f.add_argument("--battery-realism", action="store_true", help="enable OCV sag + Peukert")
    f.add_argument("--sensor-noise", action="store_true", help="add seeded GPS/baro/voltage noise")
    f.add_argument("--revisit-lat", type=float, default=None, help="infected-area latitude to revisit")
    f.add_argument("--revisit-lon", type=float, default=None, help="infected-area longitude to revisit")
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

    sub.add_parser("compare", help="optimized vs naive (slow speed, wide spacing) mission")
    return p


def _design(_args: argparse.Namespace | None = None) -> int:
    b = size_power_system()
    print(f"BOM mass: {DEFAULT_BOM.total_mass_g} g, cost: ${DEFAULT_BOM.total_cost_usd}")
    for name, qty, mass, cost in DEFAULT_BOM.as_rows():
        print(f"  {name}: x{qty}, {mass} g, ${cost}")
    print(f"Pack: {b.voltage_v} V x {b.capacity_ah} Ah = {b.pack_energy_wh:.1f} Wh")
    print(f"Usable (80% DoD): {b.usable_energy_wh:.1f} Wh")
    print(f"Hover power: {b.hover_power_w:.1f} W (+ avionics {b.avionics_power_w:.1f} W)")
    print(f"Estimated hover endurance: {b.estimated_hover_minutes:.1f} min")
    return 0


def _optimize(args: argparse.Namespace) -> int:
    twin = DigitalTwin(size_power_system())
    v = optimal_speed(twin)
    sp = optimal_grid_spacing(args.altitude, camera_fov_deg=args.fov, overlap_fraction=args.overlap)
    print(f"Optimal cruise speed: {v:.2f} m/s ({v * 3.6:.1f} km/h)")
    print(f"Optimal grid spacing at {args.altitude:.0f} m (FOV {args.fov:.0f}deg, {args.overlap:.0%} overlap): {sp:.2f} m")
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
        print(json.dumps(r.__dict__, indent=2))
        return 0 if r.success else 1
    print(f"Sylph survey simulation: {'SUCCESS' if r.success else 'FAILED'}")
    print(f"  Speed: {r.speed_mps:.2f} m/s, spacing: {r.spacing_m:.2f} m, waypoints: {r.waypoints}")
    print(f"  Survey path: {r.original_route_m:.0f} m ({r.waypoints} waypoints, lawnmower coverage)")
    print(f"  Flew: {r.distance_m:.0f} m in {r.time_s / 60:.1f} min")
    print(f"  Energy used: {r.energy_wh:.1f} Wh ({r.energy_used_pct:.1f}% of usable)")
    print(f"  Remaining: {r.energy_remaining_wh:.1f} Wh")
    print(f"  Detections: {r.detections} ({r.infected} infected)")
    if r.terminal_voltage_v is not None:
        print(f"  Terminal voltage: {r.terminal_voltage_v:.2f} V")
    if r.sensor_sample:
        s = r.sensor_sample
        print(f"  Sensor sample: GPS {s['latitude']:.6f},{s['longitude']:.6f} "
              f"baro {s['baro_altitude_m']:.2f} m volt {s['voltage_v']:.2f} V")
    if r.revisit_regions:
        print(f"  Value revisits: {r.revisit_regions} region(s), +{r.revisit_extra_m:.0f} m")
    for line in r.log:
        print(f"  - {line}")
    print(r.report_text)
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
    print(f"Exported {len(items)} waypoints to {path} ({args.format})")
    # Sanity-check the adapter round trip without claiming a real SITL flight.
    fc = TwinFlightController(twin, optimal_speed(twin))
    fc.upload(items)
    result = fc.execute()
    print(f"Digital-twin dry run: {result['distance_m']:.0f} m, "
          f"{result['energy_wh']:.1f} Wh ({result['backend']})")
    if args.sitl_host:
        # Explicit opt-in only: never probe or connect implicitly.
        from .mavlink_sitl import MavlinkSitlController, PymavlinkTransport
        transport = PymavlinkTransport(args.sitl_host, args.sitl_port)
        sitl = MavlinkSitlController(transport, mode=args.sitl_mode)
        sitl.upload(items)
        try:
            sitl_result = sitl.execute()
            print(f"SITL run: {sitl_result['samples']} telemetry samples, "
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
        print(json.dumps(r.__dict__, indent=2))
    else:
        print(r.to_text())
    return 0


def _compare(_args: argparse.Namespace | None = None) -> int:
    opt = simulate_mission()
    naive = simulate_mission(speed_mps=6.0, spacing_m=10.0)
    print("Optimized vs naive survey (same 300x300 m field):")
    print(f"  {'':12} {'speed':>8} {'spacing':>9} {'route m':>9} {'time min':>9} {'Wh':>7} {'ok':>5}")
    for label, r in (("optimized", opt), ("naive", naive)):
        print(
            f"  {label:12} {r.speed_mps:8.2f} {r.spacing_m:9.2f} "
            f"{r.optimized_route_m:9.0f} {r.time_s / 60:9.1f} "
            f"{r.energy_wh:7.1f} {str(r.success):>5}"
        )
    if not opt.success:
        print("Optimized mission did not complete")
        return 1
    return 0


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    return {
        "design": _design,
        "optimize": _optimize,
        "fly": _fly,
        "export": _export,
        "montecarlo": _montecarlo,
        "compare": _compare,
    }[args.command](args)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
