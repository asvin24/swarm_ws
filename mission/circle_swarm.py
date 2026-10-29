#!/usr/bin/env python3
"""
Multi-drone circle mission (sim or real): every drone takes off, spreads out to evenly spaced
points on ONE shared circle, flies the circle together (same angular speed, fixed spacing
around the circle so they never get closer than the circle's chord), then lands.

  Sim, 2 drones 1 m apart (defaults):   python3 circle_swarm.py
  Bigger/slower circle:                 python3 circle_swarm.py --radius 8 --speed 1.0 --laps 1

Frames (same convention as drone_api):
  * Shared "world" frame = each drone's local ENU frame + its Swarm offset (0, i * spacing).
    So drone i must start `spacing` metres North (+y) of drone i-1, as sim.launch.py spawns them.
  * The circle is centred between the drones' takeoff points. Altitudes are relative to each
    drone's own takeoff point.

Safety behaviour:
  * If ANY drone is overridden (RC sticks / mode switch), the whole mission stops. That drone is
    left with the pilot; every other drone lands.
  * Ctrl+C, a failed takeoff, a timeout, drones closer than --min-sep, or any Python error:
    every drone lands and the script waits for them to disarm.
"""
import argparse
import itertools
import math
import time
import traceback

from swarm.drone_api import Swarm, Drone, ManualOverride, wrap_angle

LOOP_HZ = 10.0           # Setpoint streaming rate (Hz)
CLIMB_TIMEOUT = 40.0     # Max time for all drones to reach altitude after takeoff (s)
TRANSFER_TIMEOUT = 40.0  # Max time to reach the start points on the circle (s)
SETTLE_TIMEOUT = 15.0    # Max time to settle at the final points (s)
ARRIVE_TOL = 0.6         # "At the point" radius for transfer/settle (m)
RAMP_TIME = 4.0          # Speed ramp-up / ramp-down time on the circle (s)
LAND_TIMEOUT = 90.0      # Max wait for all drones to disarm after LAND (s)


class MissionAbort(Exception):
    """Mission stopped for a non-override reason (timeout, takeoff failure, separation)."""


def parse_args():
    p = argparse.ArgumentParser(description="Fly several drones around one shared circle")
    p.add_argument("--names", nargs="+", default=["drone0", "drone1"],
                   help="Drone namespaces, in spawn order (default: drone0 drone1)")
    p.add_argument("--spacing", type=float, default=1.0,
                   help="Spawn spacing along +y (North) in m; must match the launch y_spacing (default 1.0)")
    p.add_argument("--alt", type=float, default=5.0, help="Flight height above takeoff point (m)")
    p.add_argument("--alt-step", type=float, default=0.0,
                   help="Extra height per drone index, e.g. 1.0 -> 5, 6, 7 m (default 0)")
    p.add_argument("--radius", type=float, default=5.0, help="Circle radius (m)")
    p.add_argument("--speed", type=float, default=1.5, help="Speed along the circle (m/s)")
    p.add_argument("--laps", type=float, default=2.0, help="Number of laps (can be fractional)")
    p.add_argument("--direction", choices=["ccw", "cw"], default="ccw",
                   help="ccw = counter-clockwise seen from above (default)")
    p.add_argument("--min-sep", type=float, default=2.0,
                   help="Abort and land if any two drones get closer than this on the circle (m)")
    return p.parse_args()


def validate(args):
    n = len(args.names)
    if n < 2:
        raise SystemExit("Need at least 2 drones (--names)")
    if len(set(args.names)) != n:
        raise SystemExit("Drone names must be unique")
    if args.alt < 1.5 or args.alt_step < 0.0:
        raise SystemExit("Need --alt >= 1.5 and --alt-step >= 0")
    if args.radius < 1.5:
        raise SystemExit("Need --radius >= 1.5")
    if not (0.0 < args.speed <= 5.0):
        raise SystemExit("Need 0 < --speed <= 5")
    if args.laps <= 0.0:
        raise SystemExit("Need --laps > 0")
    chord = 2.0 * args.radius * math.sin(math.pi / n)  # distance between neighbours on the circle
    if chord < 2.0 * args.min_sep:
        raise SystemExit(
            f"Drones would be only {chord:.1f} m apart on the circle; "
            f"increase --radius (need >= {2.0 * args.min_sep:.1f} m between neighbours)"
        )
    if n > 2 and args.alt_step == 0.0:
        print("[CIRCLE] Warning: with more than 2 drones in a line, paths to the circle can cross. "
              "Consider --alt-step 1.0.")


# -----------------------------------------------------------------------------
# Frame helpers (shared world frame = local ENU + Swarm offset)
# -----------------------------------------------------------------------------
def world_pos(d: Drone):
    return d.get_pos(world=True)


def send_world(d: Drone, wx: float, wy: float, z_local: float, yaw: float):
    """Send a setpoint given in the shared world frame (x, y) and the drone's local z."""
    d.goto(wx - d.offset_x, wy - d.offset_y, z_local, yaw=wrap_angle(yaw))


def dist_to_world(d: Drone, wx: float, wy: float, z_local: float) -> float:
    x, y, z = world_pos(d)
    return math.dist((x, y, z), (wx, wy, z_local))


def min_separation(drones) -> float:
    return min(math.dist(world_pos(a), world_pos(b)) for a, b in itertools.combinations(drones, 2))


def check_all(drones):
    for d in drones:
        d.check_override()


# -----------------------------------------------------------------------------
# Mission
# -----------------------------------------------------------------------------
def run_mission(args, drones):
    n = len(drones)
    dt = 1.0 / LOOP_HZ
    sign = 1.0 if args.direction == "ccw" else -1.0

    # 1. TAKEOFF: arm and launch each drone in turn (wait=False), then wait for all to climb.
    z_tgt = {}
    for i, d in enumerate(drones):
        alt_i = args.alt + i * args.alt_step
        print(f"[CIRCLE] {d.name}: taking off to {alt_i:.1f} m...")
        if not d.takeoff(alt=alt_i, wait=False):
            raise MissionAbort(f"{d.name}: takeoff failed (see log)")
        z_tgt[d.name] = d.home[2] + alt_i

    t0 = time.monotonic()
    while not all(abs(d.get_pos()[2] - z_tgt[d.name]) <= 0.4 for d in drones):
        check_all(drones)
        if time.monotonic() - t0 > CLIMB_TIMEOUT:
            raise MissionAbort("Not all drones reached altitude in time")
        time.sleep(0.2)
    print("[CIRCLE] All drones at altitude.")

    # 2. CIRCLE GEOMETRY: centre between the takeoff points, evenly spaced phases.
    homes = [(d.home[0] + d.offset_x, d.home[1] + d.offset_y) for d in drones]
    cx = sum(h[0] for h in homes) / n
    cy = sum(h[1] for h in homes) / n
    hx0, hy0 = homes[0]
    base = math.atan2(hy0 - cy, hx0 - cx) if math.hypot(hx0 - cx, hy0 - cy) > 0.05 else -math.pi / 2
    phases = [base + 2.0 * math.pi * i / n for i in range(n)]
    R = args.radius
    print(f"[CIRCLE] Centre ({cx:.2f}, {cy:.2f}) in world frame, radius {R:.1f} m, {n} drones.")

    def targets(theta):
        """World (x, y), local z and yaw (facing the direction of travel) for each drone."""
        out = []
        for i, d in enumerate(drones):
            a = phases[i] + theta
            out.append((cx + R * math.cos(a), cy + R * math.sin(a), z_tgt[d.name], a + sign * math.pi / 2))
        return out

    # 3. TRANSFER: everyone flies outward to their start point on the circle.
    print("[CIRCLE] Moving to start points...")
    start = targets(0.0)
    t0 = time.monotonic()
    while True:
        check_all(drones)
        for d, (wx, wy, z, yaw) in zip(drones, start):
            send_world(d, wx, wy, z, yaw)
        if all(dist_to_world(d, wx, wy, z) <= ARRIVE_TOL for d, (wx, wy, z, _) in zip(drones, start)):
            break
        if time.monotonic() - t0 > TRANSFER_TIMEOUT:
            raise MissionAbort("Not all drones reached their start points in time")
        time.sleep(dt)

    # 4. CIRCLE: advance a shared angle with a speed ramp at both ends.
    omega = args.speed / R                    # rad/s at full speed
    total = args.laps * 2.0 * math.pi         # rad to travel
    ramp_angle = 0.5 * omega * RAMP_TIME      # angle covered while ramping
    print(f"[CIRCLE] Flying {args.laps:g} lap(s) {args.direction} at {args.speed:.1f} m/s...")
    progress = 0.0                            # rad travelled (always >= 0)
    t_start = t_prev = time.monotonic()
    while progress < total:
        now = time.monotonic()
        step_dt, t_prev = now - t_prev, now
        up = min(1.0, (now - t_start) / RAMP_TIME)
        down = max(0.1, min(1.0, (total - progress) / (2.0 * ramp_angle))) if ramp_angle > 0 else 1.0
        progress = min(total, progress + omega * min(up, down) * step_dt)

        check_all(drones)
        tg = targets(sign * progress)
        for d, (wx, wy, z, yaw) in zip(drones, tg):
            send_world(d, wx, wy, z, yaw)

        sep = min_separation(drones)
        if sep < args.min_sep:
            raise MissionAbort(f"Drones only {sep:.2f} m apart (< --min-sep {args.min_sep} m)")

        lag = max(dist_to_world(d, wx, wy, z) for d, (wx, wy, z, _) in zip(drones, tg))
        print(f"\r[CIRCLE] Lap {progress / (2 * math.pi):4.2f}/{args.laps:g} | "
              f"max lag {lag:4.2f} m | min separation {sep:5.2f} m   ", end="", flush=True)
        time.sleep(dt)
    print()

    # 5. SETTLE at the final points, then hand over to landing (in main's finally).
    final = targets(sign * total)
    t0 = time.monotonic()
    while time.monotonic() - t0 < SETTLE_TIMEOUT:
        check_all(drones)
        for d, (wx, wy, z, yaw) in zip(drones, final):
            send_world(d, wx, wy, z, yaw)
        if all(dist_to_world(d, wx, wy, z) <= ARRIVE_TOL for d, (wx, wy, z, _) in zip(drones, final)):
            break
        time.sleep(dt)
    print("[CIRCLE] Mission complete.")


# -----------------------------------------------------------------------------
# Landing (all drones at once, override-safe)
# -----------------------------------------------------------------------------
def land_all_safely(drones):
    """Land every airborne drone the pilot hasn't taken over, then wait for them to disarm."""
    landing = []
    for d in drones:
        if d.is_overridden():
            d.override_handled.wait(timeout=5.0)
            print(f"[CIRCLE] {d.name}: pilot has control (mode={d.state.mode}). Not sending commands.")
            continue
        if not d.state.armed:
            continue
        try:
            if not d.land(wait=False) and d.state.mode != "LAND":
                d.set_mode("LAND")  # fallback if the land command was rejected
            landing.append(d)
            print(f"[CIRCLE] {d.name}: landing...")
        except ManualOverride as e:
            print(f"[CIRCLE] {d.name}: pilot took over: {e}")
        except Exception as e:
            print(f"[CIRCLE] {d.name}: land request failed ({e!r}). PILOT: TAKE OVER.")

    t0 = time.monotonic()
    try:
        while landing and time.monotonic() - t0 < LAND_TIMEOUT:
            for d in list(landing):
                if not d.state.armed:
                    print(f"[CIRCLE] {d.name}: landed and disarmed.")
                    landing.remove(d)
                elif d.is_overridden():
                    print(f"[CIRCLE] {d.name}: pilot took over during landing.")
                    landing.remove(d)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[CIRCLE] Stopped waiting; the drones stay in LAND mode.")
        return
    for d in landing:
        print(f"[CIRCLE] {d.name}: still armed after {LAND_TIMEOUT:.0f} s. PILOT: be ready to take over.")


def main():
    args = parse_args()
    validate(args)

    swarm = Swarm(names=args.names, spacing=args.spacing, mapping=False)
    drones = [swarm[name] for name in args.names]

    try:
        swarm.connect_all()
        print("[CIRCLE] Pilots: throttle sticks at MID so a LOITER handover holds altitude.")
        run_mission(args, drones)

    except ManualOverride as e:
        print(f"\n[CIRCLE] MANUAL OVERRIDE: {e}. Landing the other drones.")
    except MissionAbort as e:
        print(f"\n[CIRCLE] ABORT: {e}. Landing.")
    except KeyboardInterrupt:
        print("\n[CIRCLE] Interrupted by user. Landing.")
    except Exception:
        print()
        traceback.print_exc()

    finally:
        land_all_safely(drones)
        swarm.close()


if __name__ == "__main__":
    main()