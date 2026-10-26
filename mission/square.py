#!/usr/bin/env python3
"""
Simple mission: take off, fly a 10 m x 10 m square, land.
Manual override (RC sticks or an external mode change) aborts the mission and
leaves the drone with the pilot. Run: python3 square_mission.py
"""
import time

import rclpy
from swarm.drone_api import Swarm, ManualOverride

ALT = 1.2      # flight altitude (m)
SIDE = 10.0    # square side length (m)

# Local frame, starting from the takeoff point: +x forward, +y left
WAYPOINTS = [
    (SIDE, 0.0, ALT),
    (SIDE, SIDE, ALT),
    (0.0, SIDE, ALT),
    (0.0, 0.0, ALT),
]


def main():
    swarm = Swarm(names=["drone0"], mapping=False)
    drone = swarm["drone0"]

    try:
        drone.connect()

        # Wait for the first pose so altitude checks are meaningful
        while rclpy.ok() and drone.pose_msg.header.stamp.sec == 0:
            time.sleep(0.2)

        if not drone.takeoff(alt=ALT):
            print("Takeoff failed. Aborting.")
            return

        for i, (x, y, z) in enumerate(WAYPOINTS, start=1):
            print(f"Leg {i}/{len(WAYPOINTS)}: moving to ({x}, {y}, {z})")
            if not drone.move_to(x, y, z, tol=0.5, timeout=40.0):
                print(f"Could not reach waypoint {i}. Landing.")
                break
            drone.wait(1.0)

        print("Landing...")
        drone.land()

    except ManualOverride as e:
        # The pilot has control: do not brake, land, or send any more commands
        print(f"\nMANUAL OVERRIDE: {e}")
        drone.override_handled.wait(timeout=5.0)
        print(f"Mission aborted. Flight mode: {drone.state.mode}")

    except KeyboardInterrupt:
        if drone.is_overridden():
            print("\nInterrupted; pilot already has control, sending nothing.")
        else:
            print("\nInterrupted. Braking and landing...")
            try:
                drone.stop()
                time.sleep(0.5)
                drone.land()
            except ManualOverride as e:
                print(f"Pilot took over during landing: {e}")

    finally:
        swarm.close()


if __name__ == "__main__":
    main()