#!/usr/bin/env python3
"""
Boilerplate for PointCloud-driven autonomous navigation with manual-override safety.
Run directly: python3 boilerplate_auto.py

Safety behaviour:
  * RC stick input or an external flight-mode change aborts the mission (ManualOverride).
  * After an override this script NEVER commands the drone again (no brake, no land):
    the pilot is in control. Stick input hands the drone over in LOITER; a mode switch
    leaves whatever mode the pilot chose.
  * Ctrl+C, a failed takeoff, or any Python error (no override): LAND and wait for disarm.
"""
import sys
import math
import time
import traceback

import numpy as np
from swarm.drone_api import Swarm, Drone, ManualOverride

# ---------------- Mission Hyperparameters ----------------
FLIGHT_ALT = 1.0       # Flight height above the takeoff point inside the maze (m)
LOOP_HZ = 10.0         # Control loop frequency (Hz)
SAFE_FRONT_DIST = 1.1  # Obstacle clearance threshold ahead (m)
FORWARD_STEP = 0.8     # Forward leash when path is clear (m)
TURN_STEP_RAD = 0.35   # Yaw rotation step when avoiding obstacles (rad)


def compute_command(drone: Drone) -> tuple[float, float, float]:
    """
    Core Autonomous Decision Function.
    Reads the registered 3D PointCloud in the drone's body frame:
      +X = Forward (nose), +Y = Left, +Z = Up
    Returns:
      (forward_m, left_m, dyaw_rad)
    """
    # 1. Get registered 3D point cloud within 4.0m of the drone in body coordinates
    body_cloud = drone.get_body_cloud(radius=4.0, z_min=0.2, z_max=1.8)

    # 2. Check if a 3D corridor box ahead of the drone (0.2m to SAFE_FRONT_DIST forward, +/-0.45m wide) is clear
    front_clear = drone.is_box_clear(
        x_range=(0.2, SAFE_FRONT_DIST),
        y_range=(-0.45, 0.45),
        z_range=(-0.4, 0.4),
    )

    if front_clear:
        # Corridor ahead is clear -> fly straight forward
        return (FORWARD_STEP, 0.0, 0.0)

    # 3. Obstacle detected in the 3D box ahead!
    # Compare left (+Y) vs right (-Y) open space in the 3D point cloud to pick the best turn direction
    if body_cloud.shape[0] > 0:
        left_pts = body_cloud[(body_cloud[:, 0] > 0.0) & (body_cloud[:, 1] > 0.2)]
        right_pts = body_cloud[(body_cloud[:, 0] > 0.0) & (body_cloud[:, 1] < -0.2)]

        left_dist = float(np.min(np.linalg.norm(left_pts[:, :2], axis=1))) if len(left_pts) > 0 else 10.0
        right_dist = float(np.min(np.linalg.norm(right_pts[:, :2], axis=1))) if len(right_pts) > 0 else 10.0

        turn_dir = 1.0 if left_dist >= right_dist else -1.0
    else:
        turn_dir = 1.0

    # Hold position (forward=0.0) and rotate toward the more open side
    return (0.0, 0.0, turn_dir * TURN_STEP_RAD)


def land_safely(drone: Drone):
    """Land if airborne and the pilot hasn't taken over. Safe to call in any state."""
    if drone.is_overridden():
        drone.override_handled.wait(timeout=5.0)
        print(f"[AUTO] Pilot has control (mode={drone.state.mode}). Not sending commands.")
        return
    if not drone.state.armed:
        return
    print("\n[AUTO] Landing...")
    try:
        if drone.land(wait=True):
            return
        if drone.state.armed and drone.state.mode != "LAND":
            drone.set_mode("LAND")  # fallback if the land command was rejected
        print("[AUTO] Landing not confirmed. PILOT: be ready to take over.")
    except ManualOverride as e:
        print(f"[AUTO] Pilot took over during landing: {e}")
    except KeyboardInterrupt:
        print("[AUTO] Stopped waiting; the FCU stays in LAND mode.")


def main():
    swarm = Swarm(names=["drone0"], mapping=True)
    drone = swarm["drone0"]
    dt = 1.0 / LOOP_HZ

    try:
        drone.connect()
        if not drone.takeoff(alt=FLIGHT_ALT, wait=True):
            print("\n[AUTO] Takeoff failed (see log). Aborting.")
            return
        target_z = drone.home[2] + FLIGHT_ALT  # altitude lock, relative to the takeoff point

        print("\n[AUTO] Autonomous PointCloud Navigation started. Press Ctrl+C to stop & land.")
        print("[AUTO] Manual override armed: RC sticks or a mode switch will hand control to the pilot.\n")

        while True:
            t_start = time.time()

            # 0. SAFETY: abort immediately if the pilot has taken over
            drone.check_override()

            # 1. PERCEIVE & DECIDE
            fwd, left, dyaw = compute_command(drone)

            # 2. ACT (keep altitude locked to FLIGHT_ALT above takeoff)
            cx, cy, cz = drone.get_pos()
            dz = target_z - cz
            drone.step_body(forward=fwd, left=left, dz=dz, dyaw=dyaw)

            # 3. TELEMETRY HUD
            _, _, yaw_deg = drone.get_rpy(deg=True)
            num_voxels = len(drone.voxel_db)
            sys.stdout.write(
                f"\rPos: ({cx:5.2f}, {cy:5.2f}, {cz:4.2f}m) | "
                f"Yaw: {yaw_deg:6.1f}° | "
                f"3D Voxels: {num_voxels:5d} | "
                f"Cmd(fwd={fwd:4.2f}, dyaw={math.degrees(dyaw):5.1f}°)   "
            )
            sys.stdout.flush()

            # Pace the loop; drone.wait() keeps honouring the override while sleeping
            elapsed = time.time() - t_start
            if elapsed < dt:
                drone.wait(dt - elapsed)

    except ManualOverride as e:
        print(f"\n[AUTO] MANUAL OVERRIDE: {e}")
    except KeyboardInterrupt:
        print("\n[AUTO] Interrupted by user.")
    except Exception:
        traceback.print_exc()

    finally:
        land_safely(drone)
        swarm.close()


if __name__ == "__main__":
    main()