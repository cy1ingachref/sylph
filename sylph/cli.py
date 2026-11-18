"""Sylph digital-twin CLI: design, optimize, fly and compare."""

from __future__ import annotations

import argparse
import json

from .design import DEFAULT_BOM, size_power_system
from .optimizer import optimal_grid_spacing, optimal_speed
from .simulator import simulate_mission
from .twin import DigitalTwin


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
    r = simulate_mission(
        width_m=args.width, height_m=args.height, altitude_m=args.altitude,
        wind_mps=args.wind, speed_mps=args.speed, spacing_m=args.spacing,
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
    for line in r.log:
        print(f"  - {line}")
    print(r.report_text)
    return 0 if r.success else 1


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
        "compare": _compare,
    }[args.command](args)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
