#!/usr/bin/env python3
"""
Keyboard teleop for one drone (sim or real). All altitudes are relative to the takeoff point.

  Maze / sim (defaults):  python3 teleop.py
  Outdoors with GPS:      python3 teleop.py --alt 3 --min-alt 2 --max-alt 8 --no-map

Safety behaviour:
  * RC stick input or an external flight-mode change ends teleop (ManualOverride) and the
    script sends nothing more: the pilot is in control.
  * L / ESC / Ctrl+C, or any Python error (no override): LAND and wait for disarm.
"""
import argparse
import math
import os
import select
import sys
import termios
import traceback
import tty

from swarm.drone_api import Swarm, Drone, ManualOverride, wrap_angle

XY_STEP = 0.5     # Metres per keypress
Z_STEP = 0.2      # Altitude step per keypress (m)
YAW_STEP = 0.08   # Radians (~5 deg) per keypress
LEASH = 2.5       # Max horizontal distance between target and drone (m)

HELP_MSG = """
=====================================================
   DRONE TELEOP
=====================================================
   W / S : Forward / Backward (Body Frame)
   A / D : Strafe Left / Right (Body Frame)
   Q / E : Rotate Yaw Left / Right
   R / F : Altitude Up / Down (clamped {min_alt:.1f} - {max_alt:.1f} m above takeoff)
   SPACE : Hold / Reset Target to Current Pos
   L / ESC / Ctrl+C : Land and Exit
=====================================================
"""


def parse_args():
    p = argparse.ArgumentParser(description="Keyboard teleop")
    p.add_argument("--name", default="drone0", help="Drone namespace (default: drone0)")
    p.add_argument("--alt", type=float, default=1.0, help="Takeoff height above takeoff point (m)")
    p.add_argument("--min-alt", type=float, default=0.5, help="Lowest allowed height (m)")
    p.add_argument("--max-alt", type=float, default=1.8, help="Highest allowed height (m)")
    p.add_argument("--no-map", action="store_true", help="Disable 3D mapping (no LiDAR)")
    return p.parse_args()


def get_key(settings, timeout=0.1):
    """Non-blocking single-key reader. Arrow/function keys are ignored."""
    fd = sys.stdin.fileno()
    tty.setraw(fd)
    try:
        rlist, _, _ = select.select([fd], [], [], timeout)
        if not rlist:
            return ''
        key = os.read(fd, 1).decode(errors='ignore')
        # Arrow/function keys send ESC followed by more bytes; don't treat them as ESC (= land)
        if key == '\x1b' and select.select([fd], [], [], 0.02)[0]:
            os.read(fd, 16)
            return ''
        return key
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)


def land_safely(drone: Drone):
    """Land if airborne and the pilot hasn't taken over. Safe to call in any state."""
    if drone.is_overridden():
        drone.override_handled.wait(timeout=5.0)
        print(f"Pilot has control (mode={drone.state.mode}). Not sending commands.")
        return
    if not drone.state.armed:
        return
    print("Landing...")
    try:
        if drone.land(wait=True):
            print("Landed and disarmed.")
            return
        if drone.state.armed and drone.state.mode != "LAND":
            drone.set_mode("LAND")  # fallback if the land command was rejected
        print("Landing not confirmed. PILOT: be ready to take over.")
    except ManualOverride as e:
        print(f"Pilot took over during landing: {e}")
    except KeyboardInterrupt:
        print("Stopped waiting; the FCU stays in LAND mode.")


def main():
    args = parse_args()
    if not (0.0 < args.min_alt <= args.alt <= args.max_alt):
        raise SystemExit("Need 0 < --min-alt <= --alt <= --max-alt")

    settings = termios.tcgetattr(sys.stdin)
    swarm = Swarm(names=[args.name], mapping=not args.no_map)
    drone = swarm[args.name]

    try:
        drone.connect()
        print(f"Taking off to {args.alt:.1f} m above takeoff point...")
        if not drone.takeoff(alt=args.alt, wait=True):
            print("Takeoff failed (see log).")
            return

        hz = drone.home[2]
        z_lo, z_hi = hz + args.min_alt, hz + args.max_alt

        # Initialize target setpoint from the drone's actual hover position
        tx, ty, tz = drone.get_pos()
        _, _, tyaw = drone.get_rpy()
        tz = min(max(tz, z_lo), z_hi)

        print(HELP_MSG.format(min_alt=args.min_alt, max_alt=args.max_alt))

        while True:
            key = get_key(settings, timeout=0.1).lower()

            if key == 'w':
                tx += XY_STEP * math.cos(tyaw)
                ty += XY_STEP * math.sin(tyaw)
            elif key == 's':
                tx -= XY_STEP * math.cos(tyaw)
                ty -= XY_STEP * math.sin(tyaw)
            elif key == 'a':
                tx += XY_STEP * math.cos(tyaw + math.pi / 2.0)
                ty += XY_STEP * math.sin(tyaw + math.pi / 2.0)
            elif key == 'd':
                tx += XY_STEP * math.cos(tyaw - math.pi / 2.0)
                ty += XY_STEP * math.sin(tyaw - math.pi / 2.0)
            elif key == 'q':
                tyaw += YAW_STEP
            elif key == 'e':
                tyaw -= YAW_STEP
            elif key == 'r':
                tz = min(z_hi, tz + Z_STEP)
            elif key == 'f':
                tz = max(z_lo, tz - Z_STEP)
            elif key == ' ':
                # Snap target immediately to current position to halt motion
                tx, ty, tz = drone.get_pos()
                tz = min(max(tz, z_lo), z_hi)
                _, _, tyaw = drone.get_rpy()
                print("\r[HOLD] Target snapped to current position.          ")
            elif key in ('l', '\x1b', '\x03'):  # 'l', ESC, or Ctrl+C (raw mode)
                print("\r")
                break  # landing happens in finally

            cx, cy, _ = drone.get_pos()
            d = math.hypot(tx - cx, ty - cy)
            if d > LEASH:  # leash keeps the target close enough for a sane velocity
                tx = cx + LEASH * (tx - cx) / d
                ty = cy + LEASH * (ty - cy) / d
            tyaw = wrap_angle(tyaw)

            drone.goto(tx, ty, tz, yaw=tyaw)

            # Live HUD on a single terminal line
            cx, cy, cz = drone.get_pos()
            front_dist = drone.min_dist(-30.0, 30.0)
            sys.stdout.write(
                f"\rPos: ({cx:5.2f}, {cy:5.2f}) Alt: {cz - hz:4.2f}m | "
                f"Target: ({tx:5.2f}, {ty:5.2f}) Alt: {tz - hz:4.2f}m | "
                f"Yaw: {math.degrees(tyaw):6.1f}° | "
                f"Front: {front_dist:4.2f}m   "
            )
            sys.stdout.flush()

    except ManualOverride as e:
        print(f"\r\nMANUAL OVERRIDE: {e}")
    except KeyboardInterrupt:
        print("\r\nInterrupted by user.")
    except Exception:
        print()
        traceback.print_exc()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        land_safely(drone)
        swarm.close()


if __name__ == "__main__":
    main()