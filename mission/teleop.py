#!/usr/bin/env python3
import sys
import select
import termios
import tty
import math
import time
import rclpy
from swarm.drone_api import Swarm

# Keep takeoff and ceiling strictly below the maze wall height (~2.0m - 2.5m)
TAKEOFF_ALT = 1.0
MIN_Z = 0.5
MAX_Z = 1.8       # Hard ceiling so the LiDAR never rises above the maze walls
XY_STEP = 0.5     # Meters per keypress
Z_STEP = 0.2      # Altitude step per keypress
YAW_STEP = 0.08   # Radians (~14 deg) per keypress

HELP_MSG = """
=====================================================
   DRONE MAZE TELEOP (Mapping Active on /drone0/map)
=====================================================
   W / S : Forward / Backward (Body Frame)
   A / D : Strafe Left / Right (Body Frame)
   Q / E : Rotate Yaw Left / Right
   R / F : Altitude Up / Down (Clamped: 0.5m - 1.8m)
   SPACE : Emergency Hold / Reset Target to Current Pos
   L     : Land and Exit
   ESC   : Land and Exit
=====================================================
"""


def get_key(settings, timeout=0.1):
    """Non-blocking terminal key reader."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def wrap(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    # Start Swarm API with 1 drone and live 2D OccupancyGrid mapping enabled
    swarm = Swarm(names=["drone0"], mapping=True)
    drone = swarm["drone0"]

    try:
        drone.connect()
        print(f"Taking off to {TAKEOFF_ALT}m (staying below maze wall height)...")
        drone.takeoff(alt=TAKEOFF_ALT, wait=True)

        # Initialize target setpoint from the drone's actual hover position
        tx, ty, tz = drone.get_pos()
        _, _, tyaw = drone.get_rpy()
        tz = min(max(tz, MIN_Z), MAX_Z)

        print(HELP_MSG)

        while rclpy.ok():
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
                tz = min(MAX_Z, tz + Z_STEP)
            elif key == 'f':
                tz = max(MIN_Z, tz - Z_STEP)
            elif key == ' ':
                # Snap target immediately to current position to halt motion
                tx, ty, tz = drone.get_pos()
                _, _, tyaw = drone.get_rpy()
                print("\r[HOLD] Target snapped to current position.          ")
            elif key in ('l', '\x1b', '\x03'):  # 'l', ESC, or Ctrl+C
                print("\r\nLanding drone...")
                drone.land(wait=True)
                break

            cx, cy, _ = drone.get_pos()
            d = math.hypot(tx - cx, ty - cy)
            if d > 2.5:  # Allow up to 2.5m leash so ArduPilot generates enough velocity
                tx = cx + 2.5 * (tx - cx) / d
                ty = cy + 2.5 * (ty - cy) / d
            tyaw = wrap(tyaw)

            drone.goto(tx, ty, tz, yaw=tyaw)

            # Live HUD on a single terminal line
            cx, cy, cz = drone.get_pos()
            front_dist = drone.min_dist(-30.0, 30.0)
            sys.stdout.write(
                f"\rPos: ({cx:5.2f}, {cy:5.2f}, {cz:4.2f}m) | "
                f"Target: ({tx:5.2f}, {ty:5.2f}, {tz:4.2f}m) | "
                f"Yaw: {math.degrees(tyaw):5.1f}° | "
                f"Front Wall: {front_dist:4.2f}m   "
            )
            sys.stdout.flush()

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        swarm.close()


if __name__ == "__main__":
    main()