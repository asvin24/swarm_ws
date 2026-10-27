# Mission Scripts

Standalone flight scripts built on `swarm/drone_api.py`. They run unchanged in simulation (Gazebo + SITL) and on the real drone (Raspberry Pi + Cube Orange+). This folder is not built by colcon (`COLCON_IGNORE`), so you run the scripts directly with `python3`.

| Script | What it does | Needs | Use it for |
|---|---|---|---|
| [`takeoff_land.py`](#takeoff_landpy) | Takes off, hovers in place, lands | Position estimate (GPS or SITL) | The very first flight on new hardware |
| [`teleop.py`](#teleoppy) | Keyboard control of one drone | Position estimate (GPS or SITL) | Manual testing, first real flights |
| [`boilerplate_gps.py`](#boilerplate_gpspy) | Flies a fixed list of waypoints, then lands | Position estimate (GPS or SITL) | Pre-planned paths: squares, lines, patterns |
| [`boilerplate_auto.py`](#boilerplate_autopy) | Reactive obstacle avoidance from the LiDAR point cloud | Position estimate **and a LiDAR** | Maze exploration (sim only for now) |

---

## Running a script

1. Start the stack underneath:
   - **Sim:** `ros2 launch swarm sim.launch.py world:=maze` (or `world:=normal`)
   - **Real drone:** `export SWARM_REAL_HW=1`, then `ros2 launch swarm real.launch.py drone_id:=0 fcu_url:=/dev/ttyACM0:921600`
2. In another terminal with the workspace sourced (`source ~/ardupilot_ws/install/setup.bash`):
   ```bash
   cd ~/ardupilot_ws/src/swarm_ws/mission
   python3 <script>.py
   ```

On the Pi, run scripts inside `tmux`. If your SSH connection drops, the script keeps running instead of being killed mid-flight.

---

## Safety behaviour (all scripts)

All scripts end the same way, whatever happens:

| What happens | What the script does |
|---|---|
| Pilot moves the roll/pitch sticks | The API switches the drone to LOITER (fallback POSHOLD, then BRAKE). The script stops and **sends nothing more**. |
| Pilot flips the flight-mode switch | The script stops and **sends nothing more**. The pilot's chosen mode is kept. |
| Ctrl+C | Lands and waits for the drone to disarm. |
| Mission finishes or a waypoint/takeoff fails | Lands and waits for disarm. |
| Any Python error | Prints the traceback, then lands and waits for disarm. |

Landing is done by `land_safely()` in each script. It does nothing if the pilot has taken over or the drone is already disarmed. If the land command is rejected, it falls back to switching the mode to LAND. Pressing Ctrl+C again while it's landing only stops the script from waiting: the flight controller stays in LAND.

**Before every flight, the pilot should:**
- Keep the **throttle stick at mid**. A stick-triggered handover goes into LOITER, and in LOITER a low throttle stick means "descend".
- Have a flight-mode switch position set to LOITER (or STABILIZE), and know where it is. The transmitter always works, even if the Pi or the script dies.

**All altitudes are relative to the takeoff point.** `drone.takeoff()` records the ground position in `drone.home` before arming, and the scripts add their altitudes to that. This avoids errors from the barometer not reading exactly 0 on the ground.

---

## `takeoff_land.py`

The simplest mission: take off, hover in place, land. Use it to check a new drone or setup before anything else, since it exercises connecting, arming, takeoff, holding position, landing and the safety handling without moving the drone sideways.

### Arguments

| Argument | Default | Meaning |
|---|---|---|
| `--name` | `drone0` | Drone namespace to control. |
| `--alt` | `1.0` | Hover height above the takeoff point (m). Must be greater than 0. |
| `--hover` | `10.0` | Hover time (s). Must be 0 or more. |

```bash
# Sim / maze
python3 takeoff_land.py

# Outdoors with GPS
python3 takeoff_land.py --alt 3

# Longer hover, e.g. to check position hold
python3 takeoff_land.py --alt 3 --hover 30
```

### How it works

1. Connects, then takes off to `--alt`. If takeoff fails, it aborts and lands.
2. Hovers for `--hover` seconds. No setpoints are sent during the hover: in GUIDED, ArduCopter holds the takeoff position by itself. The wait still reacts to a pilot override straight away.
3. Prints the measured height above the takeoff point.
4. Lands and waits for disarm.

It doesn't wait for a GPS fix before starting the way `boilerplate_gps.py` does. On the real drone, `takeoff()` keeps retrying GUIDED and arming for up to 120 s until the flight controller is ready, and gives up after that.

### Suggested first-flight checks

While it hovers, this is a good moment to test, one per flight:
- **Ctrl+C:** the drone should land.
- **Small roll/pitch stick nudge:** the drone should switch to LOITER and the script should stop.
- **Flip the mode switch:** the script should stop and the drone stay in the mode you chose.

---

## `teleop.py`

Fly one drone with the keyboard. The script takes off, then streams a position target at about 10 Hz that you move around with keys.

### Arguments

| Argument | Default | Meaning |
|---|---|---|
| `--name` | `drone0` | Drone namespace to control. |
| `--alt` | `1.0` | Takeoff height above the takeoff point (m). |
| `--min-alt` | `0.5` | Lowest height the R/F keys allow, above the takeoff point (m). |
| `--max-alt` | `1.8` | Highest height the R/F keys allow, above the takeoff point (m). |
| `--no-map` | off | Disable the 3D point-cloud mapping. Use this when the drone has no LiDAR. |

The script refuses to start unless `0 < --min-alt <= --alt <= --max-alt`.

```bash
# Maze / sim (defaults keep the drone below the 2 m maze walls)
python3 teleop.py

# Outdoors with GPS, no LiDAR
python3 teleop.py --alt 3 --min-alt 2 --max-alt 8 --no-map
```

### Keys

| Key | Action |
|---|---|
| `W` / `S` | Move the target forward / backward along the current heading (0.5 m per press) |
| `A` / `D` | Move the target left / right of the current heading (0.5 m per press) |
| `Q` / `E` | Rotate left / right (~5° per press) |
| `R` / `F` | Up / down (0.2 m per press), clamped to `--min-alt` … `--max-alt` |
| `Space` | Hold: snap the target to the drone's current position and heading |
| `L`, `Esc`, or `Ctrl+C` | Land and exit |

Arrow keys are ignored.

### How it works

1. Connects, then takes off to `--alt`. If takeoff fails, it lands (or does nothing if still on the ground) and exits.
2. Sets the target to the drone's actual hover position.
3. Every loop (about 10 Hz): reads one key, updates the target, and sends it with `goto()`.
4. The target is kept within **2.5 m** horizontally of the drone. Holding `W` won't send the target far ahead of the drone.
5. The status line shows position, height above takeoff, target, heading and the nearest LiDAR return ahead (`inf` with no LiDAR).

Step sizes and the leash are constants at the top of the file: `XY_STEP`, `Z_STEP`, `YAW_STEP`, `LEASH`.

---

## `boilerplate_gps.py`

A waypoint mission template. It takes off, flies to each waypoint in order, hovers briefly at each, then lands. As shipped, it flies a 5 m square.

### Configuration

There are no command-line arguments. Edit the constants at the top of the file:

| Constant | Default | Meaning |
|---|---|---|
| `DRONE_NAME` | `"drone0"` | Drone namespace to control. |
| `FLIGHT_ALT` | `3.0` | Flight height above the takeoff point (m). |
| `SIDE` | `5.0` | Side length of the example square (m). |
| `WAYPOINTS` | 5 m square | List of `(forward, left)` points in metres. See below. |
| `ARRIVE_TOL` | `0.8` | A waypoint counts as reached within this distance (m). Kept loose for GPS. |
| `LEG_TIMEOUT` | `40.0` | Give up on a waypoint after this many seconds. |
| `HOLD_AT_WP` | `2.0` | Hover time at each waypoint (s). |
| `MAX_RADIUS` | `15.0` | The script refuses to start if any waypoint is further than this from the takeoff point (m). |
| `GPS_TIMEOUT` | `120.0` | Maximum wait for a GPS fix and local position before giving up (s). |

### Waypoint coordinates

Waypoints are `(forward, left)` in metres **from the takeoff point**, where "forward" is the direction the **nose pointed at takeoff**:

```python
WAYPOINTS = [
    (5.0, 0.0),   # 5 m ahead of where the nose pointed
    (5.0, 5.0),   # then 5 m to the left of that
    (0.0, 5.0),
    (0.0, 0.0),   # back above the takeoff point
]
```

So the shape is flown relative to how you set the drone down. In sim, the drone spawns facing +x (East), so these match plain `move_to(x, y, z)` coordinates there. On the real drone, point the nose where you want "forward" to be before arming.

The drone keeps its takeoff heading during the whole mission.

### How it works

1. Checks every waypoint against `MAX_RADIUS` before connecting.
2. Connects, then waits for a GPS fix and a local position. It refuses to start if the drone is already armed.
3. Records the takeoff heading.
4. Takes off to `FLIGHT_ALT`. If takeoff fails, it aborts and lands.
5. For each waypoint: converts `(forward, left)` to the local frame, flies there with `move_to()`, then hovers for `HOLD_AT_WP`. If a waypoint isn't reached within `LEG_TIMEOUT`, the mission stops early.
6. Lands and waits for disarm.

### Writing your own mission

Replace `WAYPOINTS` with your points. For anything that isn't a list of points (loops, conditions, reacting to telemetry), replace the `for` loop in `main()`. Use `to_local()` to convert `(forward, left)` points and `drone.wait()` instead of `time.sleep()`, so a pilot override is noticed during pauses.

---

## `boilerplate_auto.py`

Reactive navigation template for the maze. There are no waypoints: the drone steps forward while the path ahead is clear, and turns toward the more open side when it isn't. It runs until you press Ctrl+C or the pilot takes over.

> **LiDAR required.** This script reads the LiDAR point cloud. The real drone has no LiDAR yet, and with no scan data the map is empty, so the script would see a clear path and keep flying forward until stopped. Use it in the Gazebo maze only for now.

### Configuration

There are no command-line arguments. Edit the constants at the top of the file:

| Constant | Default | Meaning |
|---|---|---|
| `FLIGHT_ALT` | `1.0` | Flight height above the takeoff point (m). Keep it below the maze wall height. |
| `LOOP_HZ` | `10.0` | Control loop rate (Hz). |
| `SAFE_FRONT_DIST` | `1.1` | How far ahead the path must be clear before stepping forward (m). |
| `FORWARD_STEP` | `0.8` | How far ahead of the drone each forward target is placed (m). |
| `TURN_STEP_RAD` | `0.35` | Rotation per loop when the path is blocked (rad, ~20°). |

### How it works

1. Connects and takes off to `FLIGHT_ALT` with mapping on. If takeoff fails, it aborts and lands.
2. Every loop, `compute_command()` decides what to do:
   - **Path clear?** It checks a box in front of the nose: 0.2 m to `SAFE_FRONT_DIST` ahead, 0.9 m wide, ±0.4 m in height, using `drone.is_box_clear()`.
   - **Clear:** step forward by `FORWARD_STEP`.
   - **Blocked:** stay in place and rotate by `TURN_STEP_RAD` toward whichever side (left or right) has its nearest obstacle further away, based on `drone.get_body_cloud()`.
3. Sends the command with `step_body()`, correcting the height back to `FLIGHT_ALT` each loop.
4. The status line shows position, heading, the number of mapped voxels, and the current command.
5. On Ctrl+C or any error, lands. On override, sends nothing more.

### Writing your own navigation

Replace `compute_command()`. It receives the `Drone` and returns `(forward_m, left_m, dyaw_rad)` relative to the drone's current position and heading. Everything else (takeoff, loop timing, altitude hold, safety, landing) stays the same.

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Stuck at "waiting for …" during connect | MAVROS isn't running, or the drone name doesn't match the launch file. Check `ros2 service list \| grep arming`. |
| "Waiting for GUIDED" or "Waiting for pre-arm checks" repeating | EKF or GPS not ready yet. In SITL, wait 30–60 s after launch. On the real drone, check the GPS fix and pre-arm messages in Mission Planner/QGC. Gives up after 120 s. |
| `No GPS fix / local position` (`boilerplate_gps.py`) | No GPS lock within `GPS_TIMEOUT`. Go outside with clear sky and wait longer before starting. |
| "Takeoff failed" | Read the log line above it: GUIDED/arming timed out, the takeoff command was rejected, or the target height wasn't reached within 30 s. |
| Override triggers right after arming | RC sticks weren't centered when the script started (the baseline locks on the first RC message). Center them before starting the script. |
| Drone doesn't fly the shape you expected | `boilerplate_gps.py` waypoints are relative to the nose direction **at takeoff**. Check how the drone was placed. |
| TF or point cloud broken on the real drone | `SWARM_REAL_HW=1` isn't set, so the API uses simulation time. Check `echo $SWARM_REAL_HW`. |