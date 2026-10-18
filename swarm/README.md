# Drone API Guide

A simple, complete reference for controlling a drone with `swarm/drone_api.py` (ROS 2 + MAVROS + ArduPilot).

---

## 1. Overview

The API has two classes and three small helper functions.

| Piece | What it is |
| --- | --- |
| `Swarm` | Starts ROS 2 for you, creates one `Drone` per name, and spins everything in a background thread. **This is the only thing a mission script needs to create.** |
| `Drone` | One vehicle. Holds sensor data, flight commands, and a live 2D map. |
| `quat_to_euler`, `yaw_to_quat`, `trace_ray` | Math helpers used internally (also usable by you). |

You never call `rclpy.init()`, create executors, or write callbacks. Write normal sequential Python.

### Prerequisites

1. The simulation is running (`ros2 launch swarm swarm.launch.py`) and MAVROS is connected to the FCU.
2. Your ROS 2 workspace is sourced in the terminal where you run your script.
3. Drone names in your script match the launch file (`drone0`, `drone1`, ...).

---

## 2. Quick start

```python
from swarm.drone_api import Swarm

swarm = Swarm(names=["drone0"])
drone = swarm["drone0"]

try:
    drone.connect()                          # wait for MAVROS services
    drone.takeoff(alt=1.5)                   # GUIDED + arm + climb
    drone.move_to(3.0, 0.0, 1.5)             # fly 3 m forward (x)
    print("Position:", drone.get_pos())
    drone.land()                             # land and wait
finally:
    swarm.close()                            # always clean up
```

Always wrap your mission in `try / finally` so `swarm.close()` runs even if you press Ctrl+C or hit an error.

---

## 3. Coordinate frames and units

| Quantity | Unit | Frame / convention |
| --- | --- | --- |
| Position `x, y, z` | metres | Local **ENU**: x = East, y = North, z = Up. Origin is where the drone's EKF started (its home point). |
| Angles `roll, pitch, yaw` | **radians** (unless `deg=True`) | Yaw 0 = East (+x), positive = counter-clockwise. |
| `goto` / `move_to` yaw argument | **radians** | Same convention as above. |
| LiDAR angles in `min_dist` | **degrees** | 0 = straight ahead of the drone, positive = to the left. |
| Altitude | metres | Relative to the takeoff point (local z). |

**Local vs world position.** Each drone's position is local to its own start point. `Swarm` gives drone *i* an offset of `(0, i * spacing)`. Pass `world=True` to a getter to add that offset, which puts all drones in one shared frame. **Commands (`goto`, `move_to`) always use local coordinates.**

---

## 4. The Swarm class

```python
Swarm(names=["drone0"], spacing=2.0, mapping=True)
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `names` | `["drone0"]` | List of drone names. Must match the namespaces used by MAVROS and the simulator. |
| `spacing` | `2.0` | Distance in metres between drones along the Y axis. Must match your launch `y_spacing`. Only used for the world-frame offset. |
| `mapping` | `True` | Turn live 2D mapping on or off for all drones. |

| Method | What it does |
| --- | --- |
| `swarm["drone0"]` | Returns that `Drone` object. |
| `swarm.drones` | Dict of `name -> Drone`. |
| `swarm.connect_all()` | Calls `connect()` on every drone, one after another. |
| `swarm.takeoff_all(alt=1.2)` | Takes off all drones **in parallel** and waits for all. |
| `swarm.land_all()` | Lands all drones in parallel and waits for all. |
| `swarm.close()` | Stops the executor, destroys nodes, shuts down ROS 2. Call it last. |

---

## 5. The Drone class

```python
Drone(name="drone0", offset=(0.0, 0.0), mapping=True, map_size=60.0, map_res=0.20)
```

You normally get a `Drone` from `Swarm`, not by constructing it. The arguments `map_size` (side length in metres) and `map_res` (cell size in metres) are only available if you build a `Drone` yourself.

### 5.1 Topics and services used

With the default prefix `/{name}/mavros`:

| Direction | Name | Type |
| --- | --- | --- |
| Service | `/{name}/mavros/cmd/arming` | `CommandBool` |
| Service | `/{name}/mavros/set_mode` | `SetMode` |
| Service | `/{name}/mavros/cmd/takeoff` | `CommandTOL` |
| Service | `/{name}/mavros/cmd/land` | `CommandTOL` |
| Subscribes | `/{name}/mavros/state` | `State` |
| Subscribes | `/{name}/mavros/local_position/pose` | `PoseStamped` |
| Subscribes | `/{name}/mavros/imu/data` | `Imu` |
| Subscribes | `/{name}/mavros/global_position/global` | `NavSatFix` |
| Subscribes | `/{name}/scan` | `LaserScan` |
| Publishes | `/{name}/mavros/setpoint_position/local` | `PoseStamped` |
| Publishes | `/{name}/map` | `OccupancyGrid` (every 0.5 s, only if `mapping=True`) |

`connect()` automatically falls back to the prefix `/{name}` if the services are not found under `/{name}/mavros`.

---

## 6. Connecting

### `drone.connect()`

Waits until the arming, set-mode, takeoff and land services exist. It loops (logging a message every second) until they appear, so a script started before MAVROS is ready simply waits.

Call it once, before anything else.

---

## 7. Flight control

### 7.1 Mode, arm, disarm

| Call | Returns | Notes |
| --- | --- | --- |
| `drone.set_mode("GUIDED")` | `True` if the mode request was sent | Case-insensitive. Returns `False` on service failure or timeout (5 s). |
| `drone.arm()` | `True` if armed | `False` while pre-arm checks fail (for example EKF or GPS not ready). |
| `drone.disarm()` | `True` if disarmed | Do not call in flight. |
| `drone.stop()` | `True` if mode sent | **Emergency stop**: switches to `BRAKE`, which halts the drone in place. |

Common ArduCopter mode names: `GUIDED` (required for API control), `LOITER`, `ALT_HOLD`, `STABILIZE`, `BRAKE`, `RTL`, `LAND`, `AUTO`.

### 7.2 Takeoff

```python
drone.takeoff(alt=1.2, wait=True, tol=0.4) -> bool
```

What it does, in order:

1. Retries `set_mode("GUIDED")` every second until it succeeds.
2. Retries `arm()` every 1.5 s until it succeeds (this is where "Waiting for EKF / pre-arm checks" appears).
3. Sends the takeoff command for `alt` metres.
4. If `wait=True`, polls the height until it is within `tol` metres of `alt`, for up to **25 seconds**.

| Argument | Default | Meaning |
| --- | --- | --- |
| `alt` | `1.2` | Target height in metres. |
| `wait` | `True` | Block until the target height is reached. |
| `tol` | `0.4` | Acceptable height error in metres. |

**Returns** `False` if the takeoff command is rejected. Otherwise `True`.

> **Known behaviour:** if the 25 s wait times out (the drone never climbed), the function still returns `True`. After takeoff, check the height yourself: `assert drone.get_pos()[2] > 0.5`.

> Steps 1 and 2 loop forever. If pre-arm checks never pass, the script waits indefinitely. Press Ctrl+C to abort.

### 7.3 Landing

```python
drone.land(wait=True) -> bool
```

Sends the land command and, if `wait=True`, blocks until the height drops to 0.2 m or less. Returns `False` if the command is rejected. The wait has no timeout.

### 7.4 Moving: `goto` vs `move_to`

These are the two ways to fly to a position. Both need the drone to be in GUIDED mode (takeoff does this for you).

#### `goto(x, y, z, yaw=None)` — non-blocking

Publishes **one** position setpoint and returns immediately. It does not check arrival.

- `yaw` in radians. If `None`, the current orientation is kept.
- To keep the drone following a target, call it repeatedly (about 10 Hz). This is what interactive teleop does.

```python
# Teleop-style loop: stream the target
while True:
    drone.goto(tx, ty, tz, yaw=tyaw)
    time.sleep(0.1)
```

#### `move_to(x, y, z, yaw=None, tol=0.5, timeout=30.0, safe_dist=0.0)` — blocking

Streams the setpoint at 10 Hz until the drone arrives, a timeout occurs, or an obstacle is detected.

| Argument | Default | Meaning |
| --- | --- | --- |
| `x, y, z` | required | Target in local metres. |
| `yaw` | `None` | Target heading in radians (keeps current if `None`). |
| `tol` | `0.5` | Arrival radius in metres (3D distance). |
| `timeout` | `30.0` | Give up after this many seconds. |
| `safe_dist` | `0.0` | If greater than 0, stop when anything is closer than this distance within the forward cone of ±60°. |

**Returns** `True` if the target was reached. `False` on timeout or when the obstacle check triggered (the drone then holds its current position).

```python
if not drone.move_to(5.0, 0.0, 1.5, safe_dist=1.0):
    print("Blocked or timed out")
```

---

## 8. Reading sensor data

All getters are thread-safe and return values immediately from the latest received message. Before the first message arrives they return defaults (see the table).

### 8.1 Summary

| Sensor | How to read it | Before data arrives |
| --- | --- | --- |
| Position | `drone.get_pos(world=False)` | `(0, 0, 0)` |
| Orientation | `drone.get_rpy(deg=False)` | `(0, 0, 0)` |
| IMU | `drone.get_imu()` | empty `Imu` message |
| LiDAR scan | `drone.get_scan()` | `None` |
| Nearest obstacle | `drone.min_dist(min_deg, max_deg)` | `inf` |
| Obstacle points | `drone.get_obstacles(world=True, max_tilt=12.0)` | empty `(0, 2)` array |
| Occupancy map | `drone.get_map()` | all `-1` |
| GPS | `drone.gps` | empty `NavSatFix` |
| Flight state | `drone.state` | empty `State` |

> If your position stays exactly `(0.00, 0.00, 0.00)`, no pose message has been received. Check `ros2 topic hz /drone0/mavros/local_position/pose`.

### 8.2 Position

```python
x, y, z = drone.get_pos()             # local frame
x, y, z = drone.get_pos(world=True)   # adds the drone's offset (shared frame in a swarm)
```

The raw list is also available as `drone.pos` (`[x, y, z]`).

### 8.3 Orientation

```python
roll, pitch, yaw = drone.get_rpy()          # radians
roll, pitch, yaw = drone.get_rpy(deg=True)  # degrees
```

The full pose message is in `drone.pose_msg` (`PoseStamped`).

### 8.4 IMU

```python
imu = drone.get_imu()   # sensor_msgs/Imu (a copy, safe to keep)

imu.linear_acceleration.x, .y, .z      # m/s^2
imu.angular_velocity.x, .y, .z         # rad/s
imu.orientation.x, .y, .z, .w          # quaternion
```

### 8.5 GPS

There is no getter. Read the attribute `drone.gps` (`sensor_msgs/NavSatFix`):

```python
g = drone.gps
print(g.latitude, g.longitude, g.altitude)   # degrees, degrees, metres
print(g.status.status)                       # -1 = no fix, 0 = fix, 1+ = augmented fix
```

> Direct attributes (`gps`, `state`, `scan`) are not lock-protected. For simple reading this is fine. Use `deepcopy` if you hold on to the value while the drone keeps flying.

### 8.6 Flight state

```python
s = drone.state            # mavros_msgs/State
s.connected                # bool: FCU heartbeat
s.armed                    # bool
s.mode                     # e.g. "GUIDED"
s.guided                   # bool
```

### 8.7 LiDAR

**Raw scan**

```python
scan = drone.get_scan()     # sensor_msgs/LaserScan or None
if scan:
    scan.ranges             # list of distances in metres (inf or NaN = no return)
    scan.angle_min          # radians
    scan.angle_increment    # radians between beams
    scan.range_min, scan.range_max
```

**Nearest obstacle in an angular window**

```python
front = drone.min_dist(-30, 30)    # degrees, 0 = straight ahead
left  = drone.min_dist(60, 120)
right = drone.min_dist(-120, -60)
any_  = drone.min_dist()           # default: full circle (-180 to 180)
```

Returns `inf` if there is no scan or no valid beam in the window. Invalid readings (inf, NaN, outside min/max range) are ignored.

**Obstacle points as coordinates**

```python
pts = drone.get_obstacles(world=True, max_tilt=12.0)   # numpy array, shape (N, 2)
# each row is [x, y] of a wall hit
```

- Beams at max range (no hit) are dropped.
- Returns an **empty array** if roll or pitch exceeds `max_tilt` degrees, because a tilted 2D scan is unreliable.
- `world=True` uses the shared frame; `False` uses the local frame.

### 8.8 Occupancy map

```python
grid = drone.get_map()   # numpy int8 array, shape (height, width)
```

| Value | Meaning |
| --- | --- |
| `-1` | Unknown |
| `0` | Free |
| `100` | Occupied (wall) |

Default grid: 60 m × 60 m, 0.2 m per cell, so 300 × 300 cells. The **centre of the grid is world (0, 0)**.

**Indexing is `grid[row=y, col=x]`.** Convert metres to cells:

```python
res, origin = drone.map_res, drone.origin_x        # 0.2, -30.0
col = int((x_world - origin) / res)
row = int((y_world - drone.origin_y) / res)
value = grid[row, col]
```

How it updates:

- It uses the LiDAR and the drone pose and updates on every scan.
- It **only updates when the drone is above 0.3 m and tilted less than 12°**. On the ground the map stays empty.
- Free space along each beam is cleared, and the end point is marked as a wall when the beam hit something.
- It is also published as `nav_msgs/OccupancyGrid` on `/{name}/map` every 0.5 s, for RViz.
- Pass `mapping=False` to `Swarm` to disable all of this (then `/{name}/map` does not exist).

---

## 9. Ready-to-use recipes

### 9.1 Fly a square

```python
from swarm.drone_api import Swarm

swarm = Swarm(["drone0"])
d = swarm["drone0"]
try:
    d.connect()
    d.takeoff(1.5)
    for x, y in [(3, 0), (3, 3), (0, 3), (0, 0)]:
        d.move_to(x, y, 1.5)
    d.land()
finally:
    swarm.close()
```

### 9.2 Fly forward until a wall is close

```python
d.takeoff(1.2)
while d.min_dist(-30, 30) > 1.0:
    x, y, z = d.get_pos()
    d.goto(x + 0.5, y, 1.2)
    time.sleep(0.1)
d.stop()                  # BRAKE
```

### 9.3 Print all sensors once a second

```python
import time
while True:
    x, y, z = d.get_pos()
    r, p, yaw = d.get_rpy(deg=True)
    imu = d.get_imu()
    g = d.gps
    print(f"pos=({x:.2f},{y:.2f},{z:.2f}) yaw={yaw:.0f} "
          f"acc_z={imu.linear_acceleration.z:.2f} "
          f"gps=({g.latitude:.6f},{g.longitude:.6f}) "
          f"front={d.min_dist(-30, 30):.2f}m mode={d.state.mode}")
    time.sleep(1.0)
```

### 9.4 Face a heading

```python
import math
x, y, z = d.get_pos()
d.move_to(x, y, z, yaw=math.radians(90), tol=0.3)   # turn to face +y (north)
```

### 9.5 Save the map as an image

```python
import numpy as np
import matplotlib.pyplot as plt

grid = d.get_map()
plt.imsave("map.png", grid, cmap="gray_r", origin="lower", vmin=-1, vmax=100)
np.save("map.npy", grid)
```

### 9.6 Several drones

```python
swarm = Swarm(["drone0", "drone1"], spacing=2.0)
try:
    swarm.connect_all()
    swarm.takeoff_all(alt=1.5)         # both climb at the same time
    swarm["drone0"].move_to(4, 0, 1.5)
    swarm["drone1"].move_to(4, 0, 1.5)
    swarm.land_all()
finally:
    swarm.close()
```

To move drones at the same time, give each one its own thread:

```python
import threading
ts = [threading.Thread(target=dr.move_to, args=(4, 0, 1.5)) for dr in swarm.drones.values()]
[t.start() for t in ts]; [t.join() for t in ts]
```

### 9.7 Manual (keyboard) control

Keep a target `(tx, ty, tz, tyaw)`, change it on each keypress, and call `drone.goto(tx, ty, tz, yaw=tyaw)` every \~0.1 s. Pressing hold/space snaps the target to `drone.get_pos()`.

---

## 10. Helper functions

| Function | Purpose |
| --- | --- |
| `quat_to_euler(q)` | `Quaternion` → `(roll, pitch, yaw)` in radians. |
| `yaw_to_quat(yaw)` | Yaw in radians → level `Quaternion`. |
| `trace_ray(x0, y0, x1, y1)` | Bresenham line: list of grid cells between two cells. |

---

## 11. Cheat sheet

| Goal | Code |
| --- | --- |
| Start | `swarm = Swarm(["drone0"]); d = swarm["drone0"]; d.connect()` |
| Take off | `d.takeoff(alt=1.5)` |
| Go to point (wait) | `d.move_to(x, y, z)` |
| Go to point (no wait) | `d.goto(x, y, z)` |
| Safe go-to | `d.move_to(x, y, z, safe_dist=1.0)` |
| Hold position | `x, y, z = d.get_pos(); d.goto(x, y, z)` |
| Emergency stop | `d.stop()` |
| Land | `d.land()` |
| Disarm | `d.disarm()` |
| Position | `d.get_pos()` |
| Heading (deg) | `d.get_rpy(deg=True)[2]` |
| IMU | `d.get_imu()` |
| GPS | `d.gps.latitude`, `d.gps.longitude` |
| Armed? Mode? | `d.state.armed`, `d.state.mode` |
| Distance ahead | `d.min_dist(-30, 30)` |
| Wall points | `d.get_obstacles()` |
| Map array | `d.get_map()` |
| Shut down | `swarm.close()` |

---

## 12. Troubleshooting

| Symptom | Likely cause and fix |
| --- | --- |
| Script hangs at `connect()` | MAVROS is not running, or the namespace differs. Check `ros2 service list \| grep arming`. |
| `Waiting for EKF / pre-arm checks...` forever | FCU not ready: GPS or EKF not initialised, or sensors inconsistent. Check the MAVROS `FCU:` log lines. Restart the sim if processes are stale. |
| Drone arms but never climbs | Simulator is not applying motor output (plugin not loaded, sim too slow, duplicate processes). Auto-disarm happens after 10 s. |
| `takeoff` returned `True` but the drone is on the ground | The 25 s wait timed out and the function still returns `True`. Check `d.get_pos()[2]`. |
| Position always `(0, 0, 0)` | No pose messages. Check the topic with `ros2 topic hz /drone0/mavros/local_position/pose`. |
| `Service ... timed out` warning | The FCU did not answer in 5 s. Retry. |
| `min_dist` returns `inf` | No scan yet (check `/drone0/scan` and the bridge), or no beam inside the angle window. |
| `get_obstacles()` is empty | The drone is tilted more than `max_tilt`, or there are no hits. |
| Map is all `-1` | The drone is on the ground (below 0.3 m), `mapping=False`, or no scans. |
| Drone ignores `goto` | It is not in GUIDED mode. Check `d.state.mode` and call `d.set_mode("GUIDED")`. |
| Drone drifts off or stops following | `goto` publishes once. Stream it at about 10 Hz, or use `move_to`. |

---

## 13. Good practice

- Always use `try / finally: swarm.close()`.
- Keep altitude below your maze wall height, otherwise the 2D LiDAR sees nothing.
- Use `safe_dist` on every `move_to` near walls.
- Check the result of `move_to` and the height after `takeoff`.
- Stream setpoints (`goto`) at about 10 Hz for manual control.
- Before relaunching the simulator, make sure no old `arducopter`, `gz` or `mavros` processes are still running.
