#!/usr/bin/env python3
"""
Multi-drone circle mission (sim or real): every drone takes off, spreads out to evenly spaced
points on ONE shared circle, flies it together (same angular speed, fixed spacing around the
circle), then lands.

  Interactive (asks for each value):   python3 circle_swarm.py
  With options (no questions):         python3 circle_swarm.py --num-drones 2 --radius 6 --laps 1
  Options, but still review/confirm:   python3 circle_swarm.py --radius 6 -i
  Defaults, no questions at all:       python3 circle_swarm.py -y

Frames (same convention as drone_api / mission_utils):
  * Shared world frame = each drone's local ENU frame + its Swarm offset (0, i * spacing),
    so drone i must start `spacing` m North (+y) of drone i-1, as sim.launch.py spawns them.
  * The circle is centred between the takeoff points; heights are above each drone's own
    takeoff point.

Safety behaviour:
  * Any drone overridden (RC sticks / mode switch): mission stops, that drone is left with the
    pilot, every other drone lands.
  * Ctrl+C, a failed takeoff, a timeout, drones closer than min_sep, or any Python error:
    every drone lands and the script waits for them to disarm.
"""
import argparse
import math
import sys
import time
import traceback
from dataclasses import dataclass, field

from swarm.drone_api import Swarm, ManualOverride
from mission_utils import (
    MissionAbort, ask, check_all, confirm, dist_to_world, fly_to, land_all_safely,
    min_separation, send_world, takeoff_all,
)

TAG = "[CIRCLE]"
LOOP_HZ = 10.0           # Setpoint streaming rate (Hz)
RAMP_TIME = 4.0          # Speed ramp-up / ramp-down time on the circle (s)


# =============================================================================
# Configuration
# =============================================================================
@dataclass
class CircleConfig:
    num_drones: int = 2
    spacing: float = 1.0      # spawn spacing along +y / North (m), must match launch y_spacing
    alt: float = 5.0          # height above takeoff point (m)
    alt_step: float = 0.0     # extra height per drone index (m)
    radius: float = 5.0       # circle radius (m)
    speed: float = 1.5        # speed along the circle (m/s)
    laps: float = 2.0         # number of laps (fractional allowed)
    direction: str = "ccw"    # "ccw" or "cw" seen from above
    min_sep: float = 2.0      # abort if any two drones get closer than this (m)
    names: list[str] = field(default_factory=list)  # filled from num_drones if empty

    def heights(self) -> list[float]:
        return [self.alt + i * self.alt_step for i in range(len(self.names))]


# One entry per tunable value: (field, CLI flag, prompt label, type, valid?, hint)
PARAMS = [
    ("num_drones", "--num-drones", "Number of drones", int, lambda v: v >= 2, ">= 2"),
    ("spacing", "--spacing", "Spawn spacing along North / +y (m)", float, lambda v: v > 0,
     "must match launch y_spacing"),
    ("alt", "--alt", "Flight height above takeoff (m)", float, lambda v: v >= 1.5, ">= 1.5"),
    ("alt_step", "--alt-step", "Extra height per drone (m)", float, lambda v: v >= 0, ">= 0"),
    ("radius", "--radius", "Circle radius (m)", float, lambda v: v >= 1.5, ">= 1.5"),
    ("speed", "--speed", "Speed along the circle (m/s)", float, lambda v: 0 < v <= 5, "0 - 5"),
    ("laps", "--laps", "Number of laps", float, lambda v: v > 0, "> 0"),
    ("direction", "--direction", "Direction", str, lambda v: v in ("ccw", "cw"), "ccw / cw"),
    ("min_sep", "--min-sep", "Minimum separation (m)", float, lambda v: v > 0, "> 0"),
]


def parse_args():
    defaults = CircleConfig()
    p = argparse.ArgumentParser(
        description="Fly several drones around one shared circle. "
                    "With no options it asks for each value interactively.")
    for name, flag, label, typ, _, hint in PARAMS:
        kw = {"choices": ["ccw", "cw"]} if name == "direction" else {}
        p.add_argument(flag, dest=name, type=typ, default=None,
                       help=f"{label}; {hint} (default {getattr(defaults, name)})", **kw)
    p.add_argument("--names", nargs="+", default=None,
                   help="Explicit drone namespaces in spawn order (overrides --num-drones)")
    p.add_argument("-i", "--interactive", action="store_true",
                   help="Ask for every value even if options are given (options become the defaults)")
    p.add_argument("-y", "--yes", action="store_true",
                   help="Never ask: use options/defaults and skip the confirmation")
    return p.parse_args()


def validate(cfg: CircleConfig) -> list[str]:
    """Return a list of problems (empty if the config is OK)."""
    errors = [f"{label}: {getattr(cfg, name)!r} is out of range ({hint})"
              for name, _, label, _, check, hint in PARAMS if not check(getattr(cfg, name))]
    if len(cfg.names) != len(set(cfg.names)):
        errors.append("Drone names must be unique")
    n = len(cfg.names)
    if n >= 2:
        chord = 2.0 * cfg.radius * math.sin(math.pi / n)   # distance between neighbours on the circle
        if chord < 2.0 * cfg.min_sep:
            errors.append(f"Drones would be only {chord:.1f} m apart on the circle; "
                          f"need >= {2 * cfg.min_sep:.1f} m (bigger radius or smaller min separation)")
    return errors


def build_config(args) -> CircleConfig:
    """Options -> config; ask interactively when no options were given (or with -i)."""
    cfg = CircleConfig()
    given = {name: getattr(args, name) for name, *_ in PARAMS if getattr(args, name) is not None}
    for name, value in given.items():
        setattr(cfg, name, value)
    if args.names:
        cfg.names = list(args.names)
        cfg.num_drones = len(cfg.names)

    interactive = not args.yes and (args.interactive or (not given and not args.names))
    if interactive and not sys.stdin.isatty() and not args.interactive:
        interactive = False  # piped/non-terminal run without -i: just use defaults

    while True:
        if interactive:
            print(f"{TAG} Mission settings (press Enter to keep the value in brackets):")
            for name, _, label, typ, check, hint in PARAMS:
                if name == "num_drones" and args.names:
                    continue  # fixed by --names
                setattr(cfg, name, ask(label, getattr(cfg, name), typ, check, hint))
        if not args.names:
            cfg.names = [f"drone{i}" for i in range(cfg.num_drones)]

        errors = validate(cfg)
        if not errors:
            break
        for e in errors:
            print(f"{TAG} {e}")
        if not interactive:
            raise SystemExit(f"{TAG} Invalid settings, not flying.")
        print(f"{TAG} Please fix the values above.\n")

    if len(cfg.names) > 2 and cfg.alt_step == 0.0:
        print(f"{TAG} Warning: with more than 2 drones in a line, paths to the circle can cross. "
              f"Consider an extra height per drone (alt_step 1.0).")

    print_summary(cfg)
    if interactive and not confirm(f"{TAG} Start the mission?", default=False):
        raise SystemExit(f"{TAG} Cancelled.")
    return cfg


def print_summary(cfg: CircleConfig):
    hs = ", ".join(f"{h:g}" for h in cfg.heights())
    period = 2 * math.pi * cfg.radius / cfg.speed
    print(f"{TAG} Drones: {' '.join(cfg.names)} (spacing {cfg.spacing:g} m)")
    print(f"{TAG} Heights: {hs} m | circle r={cfg.radius:g} m, {cfg.speed:g} m/s, "
          f"{cfg.laps:g} lap(s) {cfg.direction} (~{period * cfg.laps:.0f} s on the circle)")


# =============================================================================
# Circle geometry and flight
# =============================================================================
def circle_geometry(drones):
    """Centre between the takeoff points; evenly spaced phases, first one on drone0's side."""
    homes = [(d.home[0] + d.offset_x, d.home[1] + d.offset_y) for d in drones]
    cx = sum(h[0] for h in homes) / len(homes)
    cy = sum(h[1] for h in homes) / len(homes)
    hx0, hy0 = homes[0]
    base = math.atan2(hy0 - cy, hx0 - cx) if math.hypot(hx0 - cx, hy0 - cy) > 0.05 else -math.pi / 2
    phases = [base + 2.0 * math.pi * i / len(drones) for i in range(len(drones))]
    return cx, cy, phases


def make_targets(cfg: CircleConfig, cx, cy, phases, z_tgt):
    """Return f(theta) -> per-drone (world_x, world_y, local_z, yaw facing the direction of travel)."""
    sign = 1.0 if cfg.direction == "ccw" else -1.0

    def targets(theta):
        return [(cx + cfg.radius * math.cos(a), cy + cfg.radius * math.sin(a), z, a + sign * math.pi / 2)
                for a, z in ((ph + theta, z) for ph, z in zip(phases, z_tgt))]
    return targets


def fly_circle(cfg: CircleConfig, drones, targets) -> float:
    """Advance a shared angle with a speed ramp at both ends. Returns the final angle."""
    sign = 1.0 if cfg.direction == "ccw" else -1.0
    omega = cfg.speed / cfg.radius            # rad/s at full speed
    total = cfg.laps * 2.0 * math.pi          # rad to travel
    ramp_angle = 0.5 * omega * RAMP_TIME      # angle covered while ramping
    dt = 1.0 / LOOP_HZ

    progress = 0.0
    t_start = t_prev = time.monotonic()
    while progress < total:
        now = time.monotonic()
        step_dt, t_prev = now - t_prev, now
        up = min(1.0, (now - t_start) / RAMP_TIME)
        down = max(0.1, min(1.0, (total - progress) / (2.0 * ramp_angle)))
        progress = min(total, progress + omega * min(up, down) * step_dt)

        check_all(drones)
        tg = targets(sign * progress)
        for d, (wx, wy, z, yaw) in zip(drones, tg):
            send_world(d, wx, wy, z, yaw)

        sep = min_separation(drones)
        if sep < cfg.min_sep:
            raise MissionAbort(f"Drones only {sep:.2f} m apart (< min separation {cfg.min_sep} m)")
        lag = max(dist_to_world(d, wx, wy, z) for d, (wx, wy, z, _) in zip(drones, tg))
        print(f"\r{TAG} Lap {progress / (2 * math.pi):4.2f}/{cfg.laps:g} | "
              f"max lag {lag:4.2f} m | min separation {sep:5.2f} m   ", end="", flush=True)
        time.sleep(dt)
    print()
    return sign * total


def run_mission(cfg: CircleConfig, drones):
    z_tgt = takeoff_all(drones, cfg.heights(), tag=TAG)

    cx, cy, phases = circle_geometry(drones)
    targets = make_targets(cfg, cx, cy, phases, z_tgt)
    print(f"{TAG} Centre ({cx:.2f}, {cy:.2f}) in world frame, radius {cfg.radius:g} m.")

    print(f"{TAG} Moving to start points...")
    fly_to(drones, targets(0.0), what="start points")

    print(f"{TAG} Flying {cfg.laps:g} lap(s) {cfg.direction} at {cfg.speed:g} m/s...")
    final_theta = fly_circle(cfg, drones, targets)

    fly_to(drones, targets(final_theta), timeout=15.0, required=False, what="final points")
    print(f"{TAG} Mission complete.")


# =============================================================================
# Main
# =============================================================================
def main():
    try:
        cfg = build_config(parse_args())
    except (KeyboardInterrupt, EOFError):
        raise SystemExit(f"\n{TAG} Cancelled.")

    swarm = Swarm(names=cfg.names, spacing=cfg.spacing, mapping=False)
    drones = [swarm[name] for name in cfg.names]

    try:
        swarm.connect_all()
        print(f"{TAG} Pilots: throttle sticks at MID so a LOITER handover holds altitude.")
        run_mission(cfg, drones)

    except ManualOverride as e:
        print(f"\n{TAG} MANUAL OVERRIDE: {e}. Landing the other drones.")
    except MissionAbort as e:
        print(f"\n{TAG} ABORT: {e}. Landing.")
    except KeyboardInterrupt:
        print(f"\n{TAG} Interrupted by user. Landing.")
    except Exception:
        print()
        traceback.print_exc()

    finally:
        land_all_safely(drones, tag=TAG)
        swarm.close()


if __name__ == "__main__":
    main()