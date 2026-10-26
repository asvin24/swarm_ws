# Drone API Guide

A simple, complete reference for controlling a drone with `swarm/drone_api.py` (ROS 2 + MAVROS + ArduPilot).

---

## 1. Overview

The API has two classes, one safety exception, and five small helper functions.

| Piece | What it is |
| --- | --- |
| `Swarm` | Starts ROS 2 for you, creates one `Drone` per name, and spins everything in a background thread. **This is the only thing a mission script needs to create.** |
| `Drone` | One vehicle. Holds sensor data, flight commands, a live 3D voxel point cloud, and an RC/mode safety override watchdog. |
| `ManualOverride` | Exception raised automatically when a safety pilot moves the RC sticks or flips the flight mode switch. |
| `quat_to_euler`, `quat_to_rot_matrix`, `yaw_to_quat`, `wrap_angle`, `numpy_to_pointcloud2` | Math and message helpers used internally (also usable by you). |

You never call `rclpy.init()`, create executors, or write callbacks. Write normal sequential Python.

### Prerequisites

1. **In Simulation:** The simulation is running (`ros2 launch swarm sim.launch.py world:=maze`) and MAVROS is connected to the FCU.
2. **On Real Hardware (Raspberry Pi):** Export `export SWARM_REAL_HW=1` and launch hardware MAVROS (`ros2 launch swarm real.launch.py`).
3. Your ROS 2 workspace is sourced in the terminal where you run your script (`source ~/ardupilot_ws/install/setup.bash`).
4. Drone names in your script match the launch file (`drone0`, `drone1`, ...).

---

## 2. Quick start

```python
from swarm.drone_api import Swarm, ManualOverride

swarm = Swarm(names=["drone0"])
drone = swarm["drone0"]

try:
    drone.connect()                          # wait for MAVROS services + FCU heartbeat
    drone.takeoff(alt=1.2)                   # GUIDED + arm + climb
    drone.move_to(3.0, 0.0, 1.2)             # fly 3 m forward (x)
    print("Position:", drone.get_pos())
    drone.land()                             # land and wait
except ManualOverride as e:
    print(f"Pilot took control: {e}")
    drone.override_handled.wait(5.0)         # let automatic LOITER switch finish
finally:
    swarm.close()                            # always clean up
```

Always wrap your mission in `try / except ManualOverride / finally` so pilot takeover is respected and `swarm.close()` runs even if you press Ctrl+C or hit an error.

---

## 3. Coordinate frames and units

| Quantity | Unit | Frame / convention |
| --- | --- | --- |
| Position `x, y, z` | metres | Local **ENU**: x = East, y = North, z = Up. Origin is where the drone's EKF started (its home point). |
| Body step `forward, left, dz` | metres | Yaw-aligned **Body Frame**: +forward = nose, +left = port side, +dz = up. |
| Angles `roll, pitch, yaw` | **radians** (unless `deg=True`) | Yaw 0 = East (+x), positive = counter-clockwise. |
| `goto` / `move_to` yaw argument | **radians** | Same convention as above. |
| LiDAR angles in `min_dist` | **degrees** | 0 = straight ahead of the drone, positive = to the left. |
| Altitude | metres | Relative to the takeoff point (local z). |

**Local vs world vs body position.**
* **Local frame:** Each drone's position is relative to its own start point. **Commands (`goto`, `move_to`) always use local coordinates.**
* **World frame (`map`):** `Swarm` gives drone *i* an offset of `(0, i * spacing)`. Pass `world=True` to `get_pos()` or use `get_cloud()` / `get_local_cloud()` to work in one shared frame across all drones.
* **Body frame:** `get_body_cloud()`, `is_box_clear()`, and `step_body()` operate relative to the drone's current position and yaw heading (`+X` straight ahead of the nose, `+Y` to the left, `+Z` relative to current altitude).

---

## 4. The Swarm class

```python
Swarm(names=("drone0",), spacing=2.0, mapping=True, sim_time=None, **drone_kwargs)
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `names` | `("drone0",)` | Tuple or list of drone names. Must match the namespaces used by MAVROS and the simulator. |
| `spacing` | `2.0` | Distance in metres between drones along the Y axis. Must match your launch `y_spacing`. Only used for the world-frame offset. |
| `mapping` | `True` | Turn live 3D voxel point cloud mapping on or off for all drones. |
| `sim_time` | `None` | `None` auto-detects (`True` unless `SWARM_REAL_HW=1` is set in the environment). |
| `**drone_kwargs` | `{}` | Any extra keyword arguments (such as `voxel_size`, `rc_deadband`, `monitor_rc`) are forwarded to each `Drone`. |

| Method | What it does |
| --- | --- |
| `swarm["drone0"]` | Returns that `Drone` object. |
| `swarm.drones` | Dict of `name -> Drone`. |
| `swarm.connect_all()` | Calls `connect()` on every drone, one after another. |
| `swarm.takeoff_all(alt=1.0)` | Takes off all drones **in parallel** and waits for all. Re-raises `ManualOverride` if any drone is overridden. |
| `swarm.land_all()` | Lands all drones in parallel and waits for all. Re-raises `ManualOverride` if any drone is overridden. |
| `swarm.check_overrides()` | Raises `ManualOverride` if any drone in the swarm has been overridden. |
| `swarm.close()` | Stops the executor, destroys nodes, shuts down ROS 2. Call it last. |

---

## 5. The Drone class

```python
Drone(
    name="drone0",
    offset=(0.0, 0.0),
    mapping=True,
    voxel_size=0.10,
    sim_time=None,
    monitor_rc=True,
    monitor_mode=True,
    rc_deadband=50,
    rc_confirm_samples=3,
    roll_idx=0,
    pitch_idx=1,
    allowed_modes=("GUIDED", "LAND", "BRAKE"),
    override_modes=("LOITER", "POSHOLD", "BRAKE"),
    only_when_armed=True,
    exit_on_override=False,
    on_override=None,
)
```

You normally get a `Drone` from `Swarm`, not by constructing it directly. Any keyword argument above can be passed through `Swarm(...)`.

### 5.1 Topics, services, and TF frames used

With the default prefix `/{name}/mavros`:

| Direction | Name | Type |
| --- | --- | --- |
| Service | `/{name}/mavros/cmd/arming` | `CommandBool` |
| Service | `/{name}/mavros/set_mode` | `SetMode` |
| Service | `/{name}/mavros/cmd/takeoff` | `CommandTOL` |
| Service | `/{name}/mavros/cmd/land` | `CommandTOL` |
| Service | `/{name}/mavros/set_stream_rate` | `StreamRate` |
| Subscribes | `/{name}/mavros/state` | `State` |
| Subscribes | `/{name}/mavros/rc/in` | `RCIn` |
| Subscribes | `/{name}/mavros/local_position/pose` | `PoseStamped` |
| Subscribes | `/{name}/mavros/imu/data` | `Imu` |
| Subscribes | `/{name}/mavros/global_position/global` | `NavSatFix` |
| Subscribes | `/{name}/scan` | `LaserScan` |
| Publishes | `/{name}/mavros/setpoint_position/local` | `PoseStamped` |
| Publishes | `/{name}/cloud` | `PointCloud2` (every 0.25 s, only if `mapping=True`) |
| Publishes | `/{name}/manual_override` | `Bool` (latched, publishes `True` on pilot takeover) |
| Broadcasts TF | `map` → `{name}/base_link` | `TransformStamped` (6-DoF pose on every scan) |

`connect()` automatically falls back to the prefix `/{name}` if the services are not found under `/{name}/mavros`.

---

## 6. Connecting

### `drone.connect()`

1. Waits until the arming, set-mode, takeoff, land, and stream-rate services exist.
2. Waits until `drone.state.connected` is `True` (FCU heartbeat received).
3. Requests a 20 Hz telemetry stream rate from the FCU so position and IMU updates arrive smoothly.

Call it once, before anything else.

---

## 7. Flight control

### 7.1 Mode, arm, disarm, wait

| Call | Returns | Notes |
| --- | --- | --- |
| `drone.set_mode("GUIDED")` | `True` if the mode request was sent | Automatically whitelists the mode with the override watchdog. Case-insensitive. |
| `drone.arm()` | `True` if armed | `False` while pre-arm checks fail (for example EKF or GPS not ready). |
| `drone.disarm()` | `True` if disarmed | Do not call in flight. |
| `drone.stop()` | `True` if mode sent | **Emergency stop**: whitelists and switches to `BRAKE`, halting the drone in place. |
| `drone.wait(seconds)` | `None` | **Override-safe sleep**: sleeps for `seconds` while checking `check_override()` every 0.1 s. Use instead of `time.sleep()`. |

Common ArduCopter mode names: `GUIDED` (required for API control), `LOITER`, `POSHOLD`, `ALT_HOLD`, `STABILIZE`, `BRAKE`, `RTL`, `LAND`, `AUTO`.

### 7.2 Takeoff

```python
drone.takeoff(alt=1.0, wait=True, tol=0.35) -> bool
```

What it does, in order:

1. Checks `check_override()` and sets stream rate to 20 Hz.
2. Retries `set_mode("GUIDED")` every second until it succeeds.
3. Arms the external flight-mode watchdog (`_FlightMode_ready = True`).
4. Retries `arm()` every 1.5 s until it succeeds (checking `check_override()` on each attempt).
5. Sends the takeoff command for `alt` metres.
6. If `wait=True`, polls the height until it is within `tol` metres of `alt`, for up to **25 seconds**.

| Argument | Default | Meaning |
| --- | --- | --- |
| `alt` | `1.0` | Target height in metres. |
| `wait` | `True` | Block until the target height is reached. |
| `tol` | `0.35` | Acceptable height error in metres. |

**Returns** `False` if the takeoff command is rejected. Otherwise `True`.

### 7.3 Landing

```python
drone.land(wait=True) -> bool
```

Checks `check_override()`, whitelists `LAND` mode, sends the land command, and if `wait=True`, blocks until the height drops to 0.2 m or less (aborting the wait immediately if a pilot triggers manual override mid-descent).

### 7.4 Moving: `goto`, `step_body`, and `move_to`

All three require the drone to be in `GUIDED` mode (takeoff does this for you) and automatically call `check_override()` before publishing.

#### `goto(x, y, z, yaw=None)` — non-blocking (local frame)

Publishes **one** position setpoint in the local EKF frame and returns immediately. It does not check arrival.

- `yaw` in radians. If `None`, the current orientation is kept.
- To keep the drone following a target, call it repeatedly (about 10 Hz).

```python
# Teleop-style loop: stream the target
while True:
    drone.goto(tx, ty, tz, yaw=tyaw)
    drone.wait(0.1)
```

#### `step_body(forward, left=0.0, dz=0.0, dyaw=0.0, max_leash=2.0)` — non-blocking (body frame)

Computes a target relative to the drone's **current position and yaw heading** and calls `goto()`.

| Argument | Default | Meaning |
| --- | --- | --- |
| `forward` | required | Metres ahead of the drone's nose (`-` for backward). |
| `left` | `0.0` | Metres to the drone's left (`-` for right). |
| `dz` | `0.0` | Altitude change in metres relative to current `z`. |
| `dyaw` | `0.0` | Yaw change in radians (`+` counter-clockwise) relative to current yaw. |
| `max_leash` | `2.0` | Maximum horizontal step distance in metres (clamps larger steps so the drone doesn't pitch too aggressively). |

```python
# Step 0.6 m straight ahead along current heading
drone.step_body(forward=0.6, left=0.0, dz=0.0, dyaw=0.0)
```

#### `move_to(x, y, z, yaw=None, tol=0.45, timeout=30.0, safe_dist=0.0)` — blocking

Streams the setpoint at 10 Hz until the drone arrives, a timeout occurs, or an obstacle is detected.

| Argument | Default | Meaning |
| --- | --- | --- |
| `x, y, z` | required | Target in local metres. |
| `yaw` | `None` | Target heading in radians (keeps current if `None`). |
| `tol` | `0.45` | Arrival radius in metres (3D distance). |
| `timeout` | `30.0` | Give up after this many seconds. |
| `safe_dist` | `0.0` | If greater than 0, stop when anything is closer than this distance within the forward cone of ±45°. |

**Returns** `True` if the target was reached. `False` on timeout or when the obstacle check triggered (the drone then holds its current position).

```python
if not drone.move_to(5.0, 0.0, 1.2, safe_dist=0.8):
    print("Blocked or timed out")
```

---

## 8. Reading sensor data & 3D point clouds

All getters are thread-safe and return values immediately from the latest received message.

### 8.1 Summary

| Sensor / Query | How to read it | Before data arrives |
| --- | --- | --- |
| Position | `drone.get_pos(world=False)` | `(0.0, 0.0, 0.0)` |
| Orientation | `drone.get_rpy(deg=False)` | `(0.0, 0.0, 0.0)` |
| IMU | `drone.get_imu()` | empty `Imu` message |
| LiDAR scan | `drone.get_scan()` | `None` |
| Nearest obstacle (2D scan) | `drone.min_dist(min_deg, max_deg)` | `inf` |
| Full 3D point cloud | `drone.get_cloud()` | empty `(0, 3)` float32 array |
| Local 3D point cloud | `drone.get_local_cloud(radius, z_min, z_max)` | empty `(0, 3)` float32 array |
| Body-frame 3D point cloud | `drone.get_body_cloud(radius, z_min, z_max)` | empty `(0, 3)` float32 array |
| 3D bounding-box check | `drone.is_box_clear(x_range, y_range, z_range)` | `True` |
| GPS | `drone.gps` | empty `NavSatFix` |
| Flight state | `drone.state` | empty `State` |

### 8.2 Position and Orientation

```python
x, y, z = drone.get_pos()              # local frame
wx, wy, wz = drone.get_pos(world=True) # adds the drone's offset (shared frame in a swarm)

roll, pitch, yaw = drone.get_rpy()          # radians
roll, pitch, yaw = drone.get_rpy(deg=True)  # degrees
```

Because `on_imu` updates `self.rpy` directly from `/mavros/imu/data`, attitude angles reflect instantaneous 6-DoF IMU orientation.

### 8.3 IMU, GPS, and Flight State

```python
imu = drone.get_imu()                  # thread-safe copy of sensor_msgs/Imu
g = drone.gps                          # sensor_msgs/NavSatFix (g.latitude, g.longitude, g.altitude)
s = drone.state                        # mavros_msgs/State (s.connected, s.armed, s.mode)
```

### 8.4 Raw 2D LiDAR & `min_dist`

```python
front = drone.min_dist(-30, 30)    # degrees, 0 = straight ahead
left  = drone.min_dist(60, 120)
right = drone.min_dist(-120, -60)
any_  = drone.min_dist()           # default: full circle (-180 to 180)
```

Returns `inf` if there is no scan or no valid beam in the window.

### 8.5 3D Voxel Point Cloud (`voxel_db`)

On every LiDAR scan, `drone_api.py` projects the 2D laser ranges into 3D using the drone's full 3D IMU rotation matrix (`quat_to_rot_matrix`) and position:
* When the drone pitches or rolls during movement (or climbs during takeoff), the tilted laser sweeps vertically across the walls, painting their true 3D height.
* Ground hits below `z = 0.05 m` are automatically filtered out once the drone is airborne (`pz > 0.35 m`).
* Points are quantized into a persistent 3D spatial hash (`voxel_db`) with `voxel_size = 0.10 m` (10 cm cubes) and published at 4 Hz on `/{name}/cloud` (`sensor_msgs/PointCloud2`, frame `map`).

#### Querying the 3D Cloud

1. **All accumulated world points (`map` frame):**
   ```python
   pts = drone.get_cloud()   # shape (N, 3), columns: [x_world, y_world, z_world]
   ```
2. **Nearby world points within a cylinder (`map` frame):**
   ```python
   pts = drone.get_local_cloud(radius=5.0, z_min=0.15, z_max=2.2)   # shape (N, 3)
   ```
3. **Points in the drone's yaw-aligned body frame (`+X` forward, `+Y` left, `+Z` relative to drone altitude):**
   ```python
   body_pts = drone.get_body_cloud(radius=5.0, z_min=0.15, z_max=2.2)  # shape (N, 3)
   # body_pts[:, 0] -> distance ahead (+) or behind (-) the nose
   # body_pts[:, 1] -> distance left (+) or right (-) of the drone
   # body_pts[:, 2] -> height above (+) or below (-) the drone
   ```
4. **Instant 3D Corridor / Bounding-Box Clearance Check:**
   ```python
   clear = drone.is_box_clear(
       x_range=(0.2, 1.4),     # 0.2 m to 1.4 m in front of the nose
       y_range=(-0.45, 0.45),  # 0.90 m wide corridor (±0.45 m left/right)
       z_range=(-0.4, 0.4),    # ±0.40 m above/below current altitude
   )
   ```
   Returns `True` if zero voxel points lie inside that 3D box, or `False` if any obstacle is inside.
5. **Resetting the 3D Map:**
   ```python
   drone.clear_cloud()
   ```

---

## 9. Manual Override Safety Watchdog

Every `Drone` runs a built-in watchdog in the background that monitors two triggers:

1. **RC Stick Deflection (`/{name}/mavros/rc/in`):**
   - Locks the initial `(roll, pitch)` PWM baseline on Channels 1 and 2 (`roll_idx=0, pitch_idx=1`).
   - While armed, if either stick moves more than `rc_deadband` (`50` PWM units) away from baseline for `rc_confirm_samples` (`3`) consecutive messages:
     1. Latches `drone.override_triggered` and publishes `True` on `/{name}/manual_override`.
     2. Automatically switches the FCU into **`LOITER`** (fallback `POSHOLD`, then `BRAKE`) so the drone stops and gives the pilot's sticks full control.
     3. Raises `ManualOverride` on the next `goto()`, `step_body()`, `move_to()`, `takeoff()`, `land()`, or `wait()` call to terminate your script.
2. **External Flight Mode Change (`/{name}/mavros/state`):**
   - Once `takeoff()` switches the drone into `GUIDED`, any mode change that your script did not authorize via `set_mode()` or `allow_mode()` (for example, the pilot flipping a transmitter switch to `POSHOLD`, `STABILIZE`, or `RTL`) immediately triggers `ManualOverride` **without** overwriting the pilot's chosen mode.

| Method / Attribute | Purpose |
| --- | --- |
| `drone.check_override()` | Raises `ManualOverride` if an override has occurred. |
| `drone.is_overridden()` | Returns `True` if overridden (`False` otherwise). |
| `drone.allow_mode("RTL")` | Whitelists a mode before your code intentionally switches to it. |
| `drone.reset_rc_baseline()` | Re-captures the neutral stick baseline on the next `RCIn` message. |
| `drone.override_handled.wait(5.0)` | Waits for the background `LOITER` mode switch to finish before closing the node. |

---

## 10. Ready-to-use recipes

### 10.1 Fly a square (override-safe)

```python
from swarm.drone_api import Swarm, ManualOverride

swarm = Swarm(["drone0"])
d = swarm["drone0"]
try:
    d.connect()
    d.takeoff(1.2)
    for x, y in [(3, 0), (3, 3), (0, 3), (0, 0)]:
        d.move_to(x, y, 1.2)
        d.wait(1.0)
    d.land()
except ManualOverride as e:
    print(f"Aborted by safety pilot: {e}")
    d.override_handled.wait(5.0)
finally:
    swarm.close()
```

### 10.2 Reactive 3D Corridor Explorer (using `is_box_clear` and `step_body`)

```python
d.takeoff(1.0)
while True:
    # Check 1.5 m ahead in a 0.9 m wide 3D box
    if d.is_box_clear(x_range=(0.2, 1.5), y_range=(-0.45, 0.45), z_range=(-0.4, 0.4)):
        d.step_body(forward=0.6)
    else:
        # Check which side is more open using 2D scan and rotate 35 deg toward it
        turn = 0.6 if d.min_dist(20, 100) > d.min_dist(-100, -20) else -0.6
        d.step_body(forward=0.0, dyaw=turn)
    d.wait(0.15)
```

### 10.3 Inspect 3D obstacle points in the drone's body frame

```python
pts = d.get_body_cloud(radius=4.0)   # (N, 3) array: [forward, left, up]
if len(pts) > 0:
    front_pts = pts[(pts[:, 0] > 0.0) & (abs(pts[:, 1]) < 0.5)]
    if len(front_pts) > 0:
        closest_x = front_pts[:, 0].min()
        print(f"Closest 3D voxel in front corridor: {closest_x:.2f} m")
```

### 10.4 Save the 3D point cloud to disk

```python
import numpy as np

cloud = d.get_cloud()   # shape (N, 3)
np.save("maze_cloud.npy", cloud)
np.savetxt("maze_cloud.csv", cloud, delimiter=",", header="x,y,z", comments="")
```

### 10.5 Several drones in parallel

```python
swarm = Swarm(["drone0", "drone1"], spacing=2.0)
try:
    swarm.connect_all()
    swarm.takeoff_all(alt=1.2)         # both climb at the same time
    swarm["drone0"].move_to(4, 0, 1.2)
    swarm["drone1"].move_to(4, 0, 1.2)
    swarm.land_all()
except ManualOverride:
    for dr in swarm.drones.values():
        dr.override_handled.wait(5.0)
finally:
    swarm.close()
```

---

## 11. Helper functions

| Function | Purpose |
| --- | --- |
| `quat_to_euler(q)` | `Quaternion` → `(roll, pitch, yaw)` in radians. |
| `quat_to_rot_matrix(q)` | `Quaternion` → `3×3` float32 rotation matrix (`np.ndarray`). |
| `yaw_to_quat(yaw)` | Yaw in radians → level `Quaternion`. |
| `wrap_angle(angle_rad)` | Wraps any angle in radians to `[-pi, pi]`. |
| `numpy_to_pointcloud2(pts, stamp, frame_id="map")` | `(N, 3)` float32 array → `sensor_msgs/PointCloud2`. |

---

## 12. Cheat sheet

| Goal | Code |
| --- | --- |
| Start | `swarm = Swarm(["drone0"]); d = swarm["drone0"]; d.connect()` |
| Take off | `d.takeoff(alt=1.2)` |
| Go to local point (wait) | `d.move_to(x, y, z)` |
| Go to local point (no wait) | `d.goto(x, y, z)` |
| Step relative to nose | `d.step_body(forward=0.5, left=0.0, dz=0.0, dyaw=0.0)` |
| Safe go-to | `d.move_to(x, y, z, safe_dist=0.8)` |
| Override-safe sleep | `d.wait(1.0)` |
| Hold position | `x, y, z = d.get_pos(); d.goto(x, y, z)` |
| Emergency stop | `d.stop()` |
| Land | `d.land()` |
| Position (local / world) | `d.get_pos()` / `d.get_pos(world=True)` |
| Roll, pitch, yaw (deg) | `d.get_rpy(deg=True)` |
| Distance ahead (2D scan) | `d.min_dist(-30, 30)` |
| Check 3D box ahead | `d.is_box_clear((0.2, 1.4), (-0.45, 0.45), (-0.4, 0.4))` |
| 3D cloud in body frame | `d.get_body_cloud(radius=5.0)` |
| Full 3D world cloud | `d.get_cloud()` |
| Shut down | `swarm.close()` |

---

## 13. Troubleshooting

| Symptom | Likely cause and fix |
| --- | --- |
| Script hangs at `connect()` | MAVROS is not running, or the namespace differs. Check `ros2 service list \| grep arming`. |
| `Waiting for EKF pre-arm checks...` forever | FCU not ready: GPS or EKF not initialised. Wait ~15 s after launching SITL or restart the sim if processes are stale. |
| RViz shows TF error or no `/drone0/cloud` | On real hardware, ensure `export SWARM_REAL_HW=1` is set so `use_sim_time` is `False`. In RViz, set Fixed Frame to `map`. |
| `min_dist` returns `inf` | No scan yet (check `ros2 topic hz /drone0/scan`), or no obstacle within range inside that angle window. |
| Drone sinks in SITL after testing `/rc/override` | When `/rc/override` stops publishing, SITL defaults Channel 3 (throttle) back to `1000` PWM after 3 s. Keep publishing `1500` on Channel 3 during SITL `LOITER` tests. |
| False manual override trigger on takeoff | Make sure your physical RC transmitter sticks are centered before starting the script, or call `d.reset_rc_baseline()` before arming. |

---

## 14. Good practice

- Always wrap missions in `try / except ManualOverride / finally: swarm.close()`.
- Use `drone.wait(sec)` instead of `time.sleep(sec)` so manual override triggers are caught immediately during pauses.
- Keep altitude below your maze wall height (`alt=1.0` to `1.2` m for 2 m walls) so the LiDAR sees the walls cleanly.
- Combine `drone.is_box_clear()` (persistent 3D memory) with `drone.min_dist()` (instantaneous 2D scan) for obstacle avoidance.
- Before relaunching the simulator, make sure no old `arducopter`, `gz`, or `mavros` processes are still running.