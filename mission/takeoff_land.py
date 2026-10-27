#!/usr/bin/env python3
"""
Simplest mission: take off, hover in place, land. Good first test on new hardware.

  Sim / maze (defaults):   python3 takeoff_land.py
  Outdoors with GPS:       python3 takeoff_land.py --alt 3

Safety behaviour:
  * RC stick input or an external flight-mode change aborts the mission (ManualOverride).
    After an override this script NEVER commands the drone again: the pilot is in control.
  * Ctrl+C, a failed takeoff, or any Python error (no override): LAND and wait for disarm.
"""
import argparse
import traceback

from swarm.drone_api import Swarm, Drone, ManualOverride


def parse_args():
    p = argparse.ArgumentParser(description="Take off, hover, land")
    p.add_argument("--name", default="drone0", help="Drone namespace (default: drone0)")
    p.add_argument("--alt", type=float, default=1.0, help="Hover height above takeoff point (m)")
    p.add_argument("--hover", type=float, default=10.0, help="Hover time (s)")
    return p.parse_args()


def land_safely(drone: Drone):
    """Land if airborne and the pilot hasn't taken over. Safe to call in any state."""
    if drone.is_overridden():
        drone.override_handled.wait(timeout=5.0)
        print(f"[TOL] Pilot has control (mode={drone.state.mode}). Not sending commands.")
        return
    if not drone.state.armed:
        return
    print("[TOL] Landing...")
    try:
        if drone.land(wait=True):
            print("[TOL] Landed and disarmed.")
            return
        if drone.state.armed and drone.state.mode != "LAND":
            drone.set_mode("LAND")  # fallback if the land command was rejected
        print("[TOL] Landing not confirmed. PILOT: be ready to take over.")
    except ManualOverride as e:
        print(f"[TOL] Pilot took over during landing: {e}")
    except KeyboardInterrupt:
        print("[TOL] Stopped waiting; the FCU stays in LAND mode.")


def main():
    args = parse_args()
    if args.alt <= 0.0 or args.hover < 0.0:
        raise SystemExit("Need --alt > 0 and --hover >= 0")

    swarm = Swarm(names=[args.name], mapping=False)
    drone = swarm[args.name]

    try:
        drone.connect()
        print("[TOL] Pilot: throttle stick at MID so a LOITER handover holds altitude.")
        print(f"[TOL] Taking off to {args.alt:.1f} m above takeoff point...")
        if not drone.takeoff(alt=args.alt, wait=True):
            print("[TOL] Takeoff failed (see log). Aborting.")
            return

        print(f"[TOL] Hovering for {args.hover:.0f} s...")
        drone.wait(args.hover)  # GUIDED holds the takeoff position; wait() still honours overrides
        x, y, z = drone.get_pos()
        print(f"[TOL] Hover done. Height above takeoff: {z - drone.home[2]:.2f} m")

    except ManualOverride as e:
        print(f"\n[TOL] MANUAL OVERRIDE: {e}")
    except KeyboardInterrupt:
        print("\n[TOL] Interrupted by user.")
    except Exception:
        traceback.print_exc()

    finally:
        land_safely(drone)
        swarm.close()


if __name__ == "__main__":
    main()