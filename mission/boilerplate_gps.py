#!/usr/bin/env python3
"""
Boilerplate for waypoint missions (real drone with GPS, or SITL) with manual-override safety.
Run directly: python3 boilerplate_gps.py

Coordinates:
  Waypoints are (forward, left) in metres from the takeoff point, where "forward"
  is the direction the nose pointed at takeoff. In sim the drone spawns facing +x,
  so this matches plain move_to(x, y, z) there.

Safety behaviour:
  * RC stick input or an external flight-mode change aborts the mission (ManualOverride).
    After an override this script NEVER commands the drone again: the pilot is in control.
  * Ctrl+C, a failed leg, or any Python error (no override): LAND and wait for disarm.
"""
import math
import time
import traceback

from swarm.drone_api import Swarm, Drone, ManualOverride

# ---------------- Mission Hyperparameters ----------------
DRONE_NAME = "drone0"
FLIGHT_ALT = 3.0        # Height above takeoff point (m)
SIDE = 5.0              # Square side (m)
WAYPOINTS = [           # (forward, left) from takeoff point, in takeoff heading
    (SIDE, 0.0),
    (SIDE, SIDE),
    (0.0, SIDE),
    (0.0, 0.0),
]
ARRIVE_TOL = 0.8        # Arrival radius (m), GPS-grade
LEG_TIMEOUT = 40.0      # Per-waypoint timeout (s)
HOLD_AT_WP = 2.0        # Hover at each waypoint (s)
MAX_RADIUS = 15.0       # Refuse any waypoint further than this from takeoff (m)
GPS_TIMEOUT = 120.0     # Max wait for GPS fix + local position (s)


def wait_for_gps(drone: Drone):
    """Block until MAVROS has a GPS fix and a local position (EKF origin set)."""
    print("[GPS] Waiting for GPS fix and local position...")
    t0 = time.monotonic()
    while True:
        has_pose = drone.pose_msg.header.stamp.sec != 0
        # NavSatFix's default status is 0 (= FIX), so also require that a message arrived
        has_fix = drone.gps.header.stamp.sec != 0 and drone.gps.status.status >= 0
        if has_pose and has_fix:
            break
        if time.monotonic() - t0 > GPS_TIMEOUT:
            raise RuntimeError(f"No GPS fix / local position after {GPS_TIMEOUT:.0f} s")
        time.sleep(0.5)
    if drone.state.armed:
        raise RuntimeError("Vehicle already armed; refusing to start")
    print(f"[GPS] OK: lat={drone.gps.latitude:.7f} lon={drone.gps.longitude:.7f}")


def to_local(home, yaw0: float, forward: float, left: float, up: float):
    """(forward, left, up) relative to takeoff point/heading -> local ENU (x, y, z) for move_to."""
    hx, hy, hz = home
    c, s = math.cos(yaw0), math.sin(yaw0)
    return hx + forward * c - left * s, hy + forward * s + left * c, hz + up


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
    for f, l in WAYPOINTS:
        if math.hypot(f, l) > MAX_RADIUS:
            raise SystemExit(f"Waypoint ({f}, {l}) is outside MAX_RADIUS={MAX_RADIUS} m")

    swarm = Swarm(names=[DRONE_NAME], mapping=False)
    drone = swarm[DRONE_NAME]

    try:
        drone.connect()
        wait_for_gps(drone)

        _, _, yaw0 = drone.get_rpy()
        print(f"[AUTO] Takeoff heading {math.degrees(yaw0):.0f}° (ENU, 0=East)")
        print("[AUTO] Pilot: throttle stick at MID so a LOITER handover holds altitude.")

        if not drone.takeoff(alt=FLIGHT_ALT, wait=True):
            print("[AUTO] Takeoff failed (see log). Aborting.")
            return

        for i, (fwd, left) in enumerate(WAYPOINTS, start=1):
            x, y, z = to_local(drone.home, yaw0, fwd, left, FLIGHT_ALT)
            print(f"[AUTO] Leg {i}/{len(WAYPOINTS)}: fwd={fwd:.1f} left={left:.1f}")
            if not drone.move_to(x, y, z, yaw=yaw0, tol=ARRIVE_TOL, timeout=LEG_TIMEOUT):
                print(f"[AUTO] Waypoint {i} not reached. Aborting.")
                break
            drone.wait(HOLD_AT_WP)
        else:
            print("[AUTO] Mission complete.")

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