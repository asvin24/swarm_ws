"""
Shared building blocks for multi-drone mission scripts (sim or real).

Import from any script in this folder:
    from mission_utils import MissionAbort, takeoff_all, fly_to, land_all_safely, ...

Frames (same convention as drone_api):
  * "World" frame = each drone's local ENU frame + its Swarm offset (0, i * spacing).
  * Targets are given as (world_x, world_y, local_z, yaw_rad_or_None). local_z is in the
    drone's own frame, so use drone.home[2] + height to fly relative to the takeoff point.
"""
import itertools
import math
import time

from swarm.drone_api import Drone, ManualOverride, wrap_angle


class MissionAbort(Exception):
    """Mission stopped for a non-override reason (timeout, takeoff failure, separation...)."""


# =============================================================================
# Frame helpers
# =============================================================================
def world_pos(d: Drone) -> tuple[float, float, float]:
    """Drone position in the shared world frame (x, y) with its local z."""
    return d.get_pos(world=True)


def send_world(d: Drone, wx: float, wy: float, z_local: float, yaw: float | None = None):
    """Send one setpoint given in the world frame (x, y) and the drone's local z."""
    d.goto(wx - d.offset_x, wy - d.offset_y, z_local, yaw=None if yaw is None else wrap_angle(yaw))


def dist_to_world(d: Drone, wx: float, wy: float, z_local: float) -> float:
    return math.dist(world_pos(d), (wx, wy, z_local))


def min_separation(drones) -> float:
    """Smallest 3D distance between any two drones (inf for fewer than 2)."""
    if len(drones) < 2:
        return float("inf")
    return min(math.dist(world_pos(a), world_pos(b)) for a, b in itertools.combinations(drones, 2))


def check_all(drones):
    """Raise ManualOverride if any drone has been overridden."""
    for d in drones:
        d.check_override()


# =============================================================================
# Flight phases
# =============================================================================
def takeoff_all(drones, heights, timeout: float = 40.0, tol: float = 0.4, tag: str = "[SWARM]"):
    """Arm and launch each drone in turn, then wait until all reach their height.

    heights: height above each drone's takeoff point (m), one per drone.
    Returns the local target z for each drone (home z + height).
    Raises MissionAbort if a takeoff fails or the climb times out.
    Pass any subset (e.g. [swarm["drone1"]], [5.0]); drones not listed stay on the ground.
    """
    drones, heights = list(drones), list(heights)
    if len(heights) != len(drones):
        raise ValueError(f"takeoff_all: {len(drones)} drone(s) but {len(heights)} height(s)")
    z_tgt = []
    for d, h in zip(drones, heights):
        print(f"{tag} {d.name}: taking off to {h:.1f} m...")
        if not d.takeoff(alt=h, wait=False):
            raise MissionAbort(f"{d.name}: takeoff failed (see log)")
        z_tgt.append(d.home[2] + h)

    t0 = time.monotonic()
    while not all(abs(d.get_pos()[2] - z) <= tol for d, z in zip(drones, z_tgt)):
        check_all(drones)
        if time.monotonic() - t0 > timeout:
            raise MissionAbort("Not all drones reached altitude in time")
        time.sleep(0.2)
    print(f"{tag} All drones at altitude.")
    return z_tgt


def fly_to(drones, targets, timeout: float = 40.0, tol: float = 0.6, rate_hz: float = 10.0,
           required: bool = True, min_sep: float | None = None, what: str = "targets",
           watch=()) -> bool:
    """Stream fixed world-frame targets to the given drones until every one of them arrives.

    drones:  any subset of your drones (e.g. [swarm["drone1"]]); only these are commanded.
    targets: one (world_x, world_y, local_z, yaw_or_None) per drone in `drones`.
    watch:   other drones (e.g. ones left hovering) that are NOT commanded but are still
             checked for pilot override and included in the min_sep check.
    Returns True when all arrived. On timeout: raises MissionAbort if required, else returns False.
    If min_sep is set, aborts when any two of drones+watch get closer than that.
    """
    drones, targets = list(drones), list(targets)
    if len(targets) != len(drones):
        raise ValueError(f"fly_to: {len(drones)} drone(s) but {len(targets)} target(s)")
    names = {d.name for d in drones}
    everyone = drones + [d for d in watch if d.name not in names]
    t0 = time.monotonic()
    while True:
        check_all(everyone)
        for d, (wx, wy, z, yaw) in zip(drones, targets):
            send_world(d, wx, wy, z, yaw)
        if min_sep is not None and (sep := min_separation(everyone)) < min_sep:
            raise MissionAbort(f"Drones only {sep:.2f} m apart (< {min_sep} m)")
        if all(dist_to_world(d, wx, wy, z) <= tol for d, (wx, wy, z, _) in zip(drones, targets)):
            return True
        if time.monotonic() - t0 > timeout:
            if required:
                raise MissionAbort(f"Not all drones reached their {what} in time")
            return False
        time.sleep(1.0 / rate_hz)


def land_all_safely(drones, timeout: float = 90.0, tag: str = "[SWARM]"):
    """Land every armed drone the pilot hasn't taken over, then wait for all to disarm.

    Safe to call in any state (use it in `finally:`). Overridden drones get no commands.
    """
    landing = []
    for d in drones:
        if d.is_overridden():
            d.override_handled.wait(timeout=5.0)
            print(f"{tag} {d.name}: pilot has control (mode={d.state.mode}). Not sending commands.")
            continue
        if not d.state.armed:
            continue
        try:
            if not d.land(wait=False) and d.state.mode != "LAND":
                d.set_mode("LAND")  # fallback if the land command was rejected
            landing.append(d)
            print(f"{tag} {d.name}: landing...")
        except ManualOverride as e:
            print(f"{tag} {d.name}: pilot took over: {e}")
        except Exception as e:
            print(f"{tag} {d.name}: land request failed ({e!r}). PILOT: TAKE OVER.")

    t0 = time.monotonic()
    try:
        while landing and time.monotonic() - t0 < timeout:
            for d in list(landing):
                if not d.state.armed:
                    print(f"{tag} {d.name}: landed and disarmed.")
                    landing.remove(d)
                elif d.is_overridden():
                    print(f"{tag} {d.name}: pilot took over during landing.")
                    landing.remove(d)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print(f"\n{tag} Stopped waiting; the drones stay in LAND mode.")
        return
    for d in landing:
        print(f"{tag} {d.name}: still armed after {timeout:.0f} s. PILOT: be ready to take over.")


# =============================================================================
# Interactive prompts
# =============================================================================
def ask(label: str, default, cast=float, check=None, hint: str = ""):
    """Prompt for one value. Enter keeps the default; invalid input is asked again."""
    suffix = f" ({hint})" if hint else ""
    while True:
        raw = input(f"  {label}{suffix} [{default}]: ").strip()
        if raw == "":
            return default
        try:
            value = cast(raw)
        except ValueError:
            print(f"    '{raw}' is not a valid {cast.__name__}.")
            continue
        if check is not None and not check(value):
            print(f"    {value!r} is out of range{suffix}.")
            continue
        return value


def confirm(question: str, default: bool = False) -> bool:
    """Yes/no prompt."""
    opts = "Y/n" if default else "y/N"
    raw = input(f"{question} [{opts}]: ").strip().lower()
    return default if raw == "" else raw in ("y", "yes")