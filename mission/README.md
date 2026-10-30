# Mission Scripts

Standalone flight scripts built on `swarm/drone_api.py`. They run unchanged in simulation (Gazebo + SITL) and on the real drone (Raspberry Pi + Cube Orange+). This folder is not built by colcon (`COLCON_IGNORE`), so you run the scripts directly with `python3`.

| Script | What it does | Needs | Use it for |
|---|---|---|---|
| [`takeoff_land.py`](#takeoff_landpy) | Takes off, hovers in place, lands | Position estimate (GPS or SITL) | The very first flight on new hardware |
| [`teleop.py`](#teleoppy) | Keyboard control of one drone | Position estimate (GPS or SITL) | Manual testing, first real flights |
| [`boilerplate_gps.py`](#boilerplate_gpspy) | Flies a fixed list of waypoints, then lands | Position estimate (GPS or SITL) | Pre-planned paths: squares, lines, patterns |
| [`circle_swarm.py`](#circle_swarmpy) | Several drones fly one shared circle together, then land. Asks for its settings interactively. | Position estimate (GPS or SITL), 2+ drones | First multi-drone flights |
| [`frontier_explore.py`](#frontier_explorepy) | Several drones explore the maze autonomously and build one shared map, shown live in RViz | 2D LiDAR + position estimate (sim) | Mapping demos, screen recordings |
| [`mission_utils.py`](#mission_utilspy) | Not a mission: shared helpers for multi-drone scripts | – | Building new multi-drone missions |
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

## `circle_swarm.py`

Multi-drone mission. All drones take off, spread out to evenly spaced points on **one shared circle**, fly around it together, then land. Because they keep the same spacing around the circle, they never get closer than the distance between neighbouring points (10 m for 2 drones on a 5 m radius).

### Launching in sim (2 drones, 1 m apart)

```bash
# Terminal 1: Gazebo + 2 SITL instances + 2 MAVROS, drones spawned 1 m apart along +y
ros2 launch swarm sim.launch.py world:=normal num_drones:=2 y_spacing:=1.0

# Terminal 2: the mission
cd ~/ardupilot_ws/src/swarm_ws/mission
python3 circle_swarm.py
```

Pass `num_drones` explicitly: the launch file's default is 1, and it doesn't read `num_drones` from `swarm.yaml`. Give SITL 30–60 s to get a GPS fix; `takeoff()` keeps retrying for up to 120 s anyway.

### Three ways to run it

| Command | What happens |
|---|---|
| `python3 circle_swarm.py` | **Interactive.** Asks for each value. Press Enter to keep the value in brackets. Invalid input is asked again. Shows a summary and asks `Start the mission? [y/N]` before flying. |
| `python3 circle_swarm.py --num-drones 3 --alt-step 1.0 --radius 6` | **Options only**, here for 3 drones at 5, 6 and 7 m. The launch must spawn the same number (`num_drones:=3`). |
| `python3 circle_swarm.py --radius 6 --laps 1` | **Options only.** Uses the given values plus defaults for the rest. No questions, no confirmation. |
| `python3 circle_swarm.py --num-drones 3 -i` | **Options as defaults, then interactive.** Asks for every value, starting from your options. |
| `python3 circle_swarm.py -y` | **Defaults, no questions** (2 drones, 5 m circle at 5 m height). |

Example interactive session:

```text
[CIRCLE] Mission settings (press Enter to keep the value in brackets):
  Number of drones (>= 2) [2]:
  Spawn spacing along North / +y (m) (must match launch y_spacing) [1.0]:
  Flight height above takeoff (m) (>= 1.5) [5.0]:
  Extra height per drone (m) (>= 0) [0.0]:
  Circle radius (m) (>= 1.5) [5.0]: 6
  Speed along the circle (m/s) (0 - 5) [1.5]:
  Number of laps (> 0) [2.0]: 1
  Direction (ccw / cw) [ccw]:
  Minimum separation (m) (> 0) [2.0]:
[CIRCLE] Drones: drone0 drone1 (spacing 1 m)
[CIRCLE] Heights: 5, 5 m | circle r=6 m, 1.5 m/s, 1 lap(s) ccw (~25 s on the circle)
[CIRCLE] Start the mission? [y/N]: y
```

### Settings

Each value can be given as an option or answered at the prompt.

| Option | Default | Meaning |
|---|---|---|
| `--num-drones` | `2` | Number of drones. They're named `drone0`, `drone1`, … in spawn order. Minimum 2. |
| `--spacing` | `1.0` | Spawn spacing along +y / North (m). **Must match the launch `y_spacing`.** |
| `--alt` | `5.0` | Flight height above each drone's own takeoff point (m). Minimum 1.5. |
| `--alt-step` | `0.0` | Extra height per drone index (m). For example, `1.0` gives 5, 6, 7 m. Recommended with 3+ drones. |
| `--radius` | `5.0` | Circle radius (m). Minimum 1.5. |
| `--speed` | `1.5` | Speed along the circle (m/s). Maximum 5. |
| `--laps` | `2.0` | Number of laps. Fractions allowed (`0.5` = half a lap). |
| `--direction` | `ccw` | `ccw` (counter-clockwise from above) or `cw`. |
| `--min-sep` | `2.0` | If any two drones get closer than this, abort and land (m). |

Option-only flags:

| Option | Meaning |
|---|---|
| `--names` | Explicit drone namespaces in spawn order, e.g. `--names drone0 drone2`. Replaces `--num-drones`. |
| `-i`, `--interactive` | Ask for every value, even when options are given. |
| `-y`, `--yes` | Never ask: use options and defaults, and skip the confirmation. |

The settings are checked before anything connects. Besides each value's range, the script refuses circles too small for the minimum separation: neighbouring drones on the circle must be at least `2 × min_sep` apart. In interactive mode it asks again; with options it exits without flying.

```bash
# 3 drones, staggered heights
ros2 launch swarm sim.launch.py world:=normal num_drones:=3 y_spacing:=1.0
python3 circle_swarm.py --num-drones 3 --alt-step 1.0 --radius 6
```

### How it works

1. **Settings:** read from options and/or prompts, validated, summarised.
2. **Connect** to every drone.
3. **Take off:** each drone is armed and launched in turn, then the script waits up to 40 s for all of them to reach their height.
4. **Circle geometry:** the centre is the midpoint of the takeoff points. The drones' start points are evenly spaced around the circle, with the first one on the side drone0 took off from. For 2 drones spawned along North, drone0 starts `radius` m South of the centre and drone1 `radius` m North, so they move apart and their paths never cross.
5. **Transfer:** all drones fly to their start points (timeout 40 s), already facing their direction of travel.
6. **Circle:** a shared angle advances at `speed / radius`, with a 4 s speed ramp at the start and end. Each drone's target is its own phase on the circle, sent at 10 Hz, with the nose facing the direction of travel. The status line shows lap progress, the worst tracking lag and the current minimum separation.
7. **Settle** at the final points (up to 15 s), then **land all drones together** and wait for every one to disarm.

### Code layout

The script is split so each part can be changed or reused on its own:

| Piece | Where | Role |
|---|---|---|
| `CircleConfig`, `PARAMS` | `circle_swarm.py` | All settings in one place. Adding a setting = one dataclass field + one `PARAMS` row; the option, prompt and range check come from that row. |
| `build_config()`, `validate()` | `circle_swarm.py` | Options/prompts → validated config. |
| `circle_geometry()`, `make_targets()`, `fly_circle()` | `circle_swarm.py` | The circle itself. Swap these out for a different formation or shape. |
| `run_mission()` | `circle_swarm.py` | The phase order: take off → move to start → circle → settle. |
| Takeoff, fly-to, landing, separation, frames, prompts | [`mission_utils.py`](#mission_utilspy) | Shared with any multi-drone script. |

### Frames

Targets are computed in the **shared world frame** that `drone_api` uses: each drone's local ENU position plus its `Swarm` offset `(0, i × spacing)`. They're converted back to each drone's local frame before `goto()`. This holds in sim as long as `--spacing` matches the launch file. On real drones it assumes they're set down exactly `spacing` m apart along North, and that each drone's local origin is its own takeoff point.

### Safety

- **Any drone overridden:** the whole mission stops. That drone is left with its pilot; all the others land.
- **Ctrl+C, a failed takeoff, a timeout, drones closer than `--min-sep`, or any error:** all drones land.
- **Ctrl+C at the prompts:** exits before anything connects.

---

## `mission_utils.py`

Shared helpers for multi-drone missions. It isn't a mission itself: scripts in this folder import it.

```python
from mission_utils import MissionAbort, takeoff_all, fly_to, land_all_safely
```

Targets are always `(world_x, world_y, local_z, yaw)`: x and y in the shared world frame, z in the drone's own frame (use `drone.home[2] + height` for a height above the takeoff point), yaw in radians or `None` to keep the current heading.

| Function | What it does |
|---|---|
| `takeoff_all(drones, heights)` | Arms and launches each drone in turn, waits until all reach their height (above their own takeoff point). Returns each drone's local target z. Raises `MissionAbort` on a failed takeoff or a 40 s timeout. |
| `fly_to(drones, targets, timeout=40, tol=0.6, required=True, min_sep=None, watch=())` | Streams fixed targets at 10 Hz until every drone in `drones` is within `tol`. On timeout raises `MissionAbort` (or returns `False` with `required=False`). Optional separation guard. `watch` = drones that aren't commanded but are still checked for override and separation. |
| `land_all_safely(drones)` | Lands every armed drone the pilot hasn't taken over, then waits for all to disarm. Safe in any state; meant for `finally:`. |
| `send_world(d, wx, wy, z, yaw)` | One setpoint in the world frame (converts to the drone's local frame). |
| `world_pos(d)`, `dist_to_world(...)`, `min_separation(drones)` | Positions and distances in the world frame. |
| `check_all(drones)` | Raises `ManualOverride` if any drone was overridden. |
| `ask(label, default, cast, check, hint)`, `confirm(question)` | Interactive prompts with defaults and re-asking on invalid input. |
| `MissionAbort` | Exception for "stop and land" reasons other than a pilot override. |

### Flying only some of the drones

Every function takes any list of drones, so you can command a subset, with one target or height per drone in that list:

```python
swarm = Swarm(names=["drone0", "drone1"], spacing=1.0)   # ALL spawned drones, in spawn order
d0, d1 = swarm["drone0"], swarm["drone1"]
swarm.connect_all()

takeoff_all([d0, d1], [5.0, 5.0])                          # both up

# Move only drone1; drone0 keeps hovering where it is
fly_to([d1], [(4.0, 1.0, d1.home[2] + 5.0, None)], watch=[d0], min_sep=2.0)

# Then each to its own point
fly_to([d0, d1], [(0.0, -3.0, d0.home[2] + 5.0, None),
                  (0.0,  4.0, d1.home[2] + 5.0, None)], min_sep=2.0)
```

Rules for subsets:

- **Create the `Swarm` with all spawned drones**, even if you only fly one. World-frame offsets come from each name's position in that list: `Swarm(["drone1"])` alone gives drone1 offset 0, the same as drone0, and its world targets come out `spacing` m off.
- **Put the drones you're not commanding in `watch`.** A hovering drone that isn't in the list still holds its position, but without `watch` the script won't notice if its pilot takes over, and the separation check won't see it.
- **Land everyone in `finally:`**: pass all drones to `land_all_safely()`, not just the ones you moved. It skips drones that are on the ground or overridden.
- Each target needs a **local z**. Use `drone.home[2] + height` for a height above that drone's takeoff point (`home` is set by `takeoff()`).

### Writing a new mission

A new multi-drone mission follows the same pattern as `circle_swarm.py`:

```python
swarm = Swarm(names=names, spacing=spacing, mapping=False)
drones = [swarm[n] for n in names]
try:
    swarm.connect_all()
    z = takeoff_all(drones, [5.0] * len(drones))
    fly_to(drones, [(wx, wy, zi, None) for (wx, wy), zi in zip(points, z)])
except (ManualOverride, MissionAbort, KeyboardInterrupt) as e:
    print(f"Stopping: {e}")
finally:
    land_all_safely(drones)
    swarm.close()
```

---

## `frontier_explore.py`

Multi-drone autonomous exploration. Every drone's LiDAR feeds **one shared 2D occupancy map**. The drones keep flying to the nearest unexplored boundary ("frontier") until the whole maze is mapped, then the map is saved and all drones land. The map, frontiers, goals and paths are published for RViz.

> **Sim only for now.** It needs the 2D LiDAR (`/droneN/scan`), and it builds the map from the drones' own position estimates. In Gazebo those are near-perfect, so the drones' scans line up into one map without SLAM. On real drones, GPS error would smear the map: that's what the SLAM work is for.

### Recommended setup: 2 drones

**Use 2 drones.** With 3, Gazebo (three GPU LiDARs) + three SITL + three MAVROS + RViz used ~10 GB of RAM on a 14 GB laptop. The third drone's MAVROS lost its heartbeat, kept restarting its parameter download and answered commands 3–6 s late, so it never got airborne. Two drones run reliably and map the maze about as well.

Use these flags unless you have a reason to change them:

| Setting | Use | Why |
|---|---|---|
| Launch `world` | `maze` | Run the **widened** maze. In the original, the narrowest corridor (0.91 m) has no room for the 0.6 m wall clearance on each side. |
| Launch `num_drones` / script `--num-drones` | `2` / `2` | **Must be the same number.** |
| Launch `y_spacing` / script `--spacing` | `1.0` / `1.0` (default) | **Must be the same value**, or the drones' scans won't line up in the map. |
| `--alt` / `--alt-step` | defaults (0.8 m, +0.2 m) | Keeps both LiDAR planes below the 1.48 m wall tops. |
| `--range` | `5.0` default, or `3.0` for recording | A shorter range makes the map grow more gradually, which looks better on video. |

### Running it, step by step

**One-time setup:** install SciPy with `sudo apt install python3-scipy`. Rebuild after updating `drone_api.py`: `colcon build --symlink-install --packages-select swarm`.

**1. Start from a clean slate.** Leftover processes from an earlier run cause frozen sims and duplicate drones:
```bash
cd ~/ardupilot_ws/src/swarm_ws
./swarm/stop.bash --sim --force
```

**2. Terminal 1: launch the sim with 2 drones.**
```bash
ros2 launch swarm sim.launch.py world:=maze num_drones:=2 y_spacing:=1.0
```

**3. Wait until both drones have a GPS position.** This takes about 1–1.5 min. In terminal 1, watch for this line from **both** drones:
```
[drone0.mavros.sys]: FCU: EKF3 IMU0 is using GPS
[drone1.mavros.sys]: FCU: EKF3 IMU0 is using GPS
```
Also check the version lines: drone0 should print `VER: 1.1` and drone1 `VER: 2.1`. If both print `1.1`, you're on the old launch file without unique system IDs.

**4. Terminal 2: start the exploration.**
```bash
cd ~/ardupilot_ws/src/swarm_ws/mission
python3 frontier_explore.py --num-drones 2 --spacing 1.0
```
For a recording, add `--range 3`.

**5. Terminal 3: start RViz once both drones are in the air.** Starting it later saves CPU during the slow start-up.
```bash
rviz2 --ros-args -p use_sim_time:=true
```
`use_sim_time:=true` is **required**: without it RViz drops the drones' TF and scans as too old. Set RViz up as described below, then start your screen recording.

**6. When it finishes,** the terminal prints `No reachable frontiers left: exploration complete.` The map is saved as `explore_map.pgm` / `explore_map.yaml` in the folder you ran it from (standard `map_server` format), and both drones land. Ctrl+C at any time also saves the map so far and lands the drones.

### Setting up RViz

No saved config is needed. Do this once per session, and optionally save it at the end with **File → Save Config As**.

1. **Global Options → Fixed Frame:** type `map`.
2. **Add → By topic**, then add these, repeating the per-drone ones for `drone1`:

| Topic | Display type | Shows | Settings to change |
|---|---|---|---|
| `/map` | Map | The shared explored map: white = free, black = wall, grey = unexplored | none |
| `/exploration/markers` | MarkerArray | Orange frontier cells + one coloured goal sphere per drone | none |
| `/drone0/trail` | Path | Where the drone has flown | Color: one per drone (e.g. red / green); Line Style: *Billboards*, Line Width `0.07` |
| `/drone0/plan` | Path | The path it's following right now | Same color as its trail, default thin line |
| `/drone0/scan` | LaserScan | Live LiDAR points | Size (m) `0.05`; Color Transformer *FlatColor*, same color; Topic → Reliability Policy *Best Effort* |
| (no topic) | Add → By display type → **TF** | Drone frames on the map | Optional; untick *Show Names* if cluttered |

3. **For a clean top-down view:** in the **Views** panel (right side), set **Type** to *TopDownOrtho*, then zoom and pan until the whole maze fits.

Alternatively, `explore.rviz` in this folder has all of this preset for 3 drones. Load it with `rviz2 -d explore.rviz --ros-args -p use_sim_time:=true` and untick the `drone2` displays.

### What you see while it runs

- **RViz:** the map fills in around the drones. The orange frontier cells move outwards, each goal sphere jumps to a new frontier as the old one gets explored, and the thin plan lines are recomputed every 1.5 s.
- **Terminal:** one status line with the time, the mapped area in m², the number of frontier cells, and each drone's state:
  - `go`: flying to a goal
  - `hold`: giving way to another drone
  - `idle`: nothing reachable to explore

### Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `Waiting for GUIDED` / `Waiting for pre-arm checks` for a long time | GPS not ready yet (step 3). The script waits up to 180 s per drone. Starting earlier is harmless, but slow. |
| Many `Service ... timed out` warnings, `RTT too high for timesync: 2000+ ms` | The PC is overloaded. Use 2 drones, start RViz only after takeoff, and close other programs. Check Gazebo's real-time factor (bottom right); it should be close to 1.0. |
| `CON: Lost connection, HEARTBEAT timed out` for one drone | That drone's SITL is being starved. Same fixes as above. |
| `ABORT: drone1: takeoff failed` | Takeoff didn't happen even after a retry. Check terminal 1 for `PreArm:` messages. |
| RViz: map empty / no scans / TF errors | RViz wasn't started with `use_sim_time:=true`, the Fixed Frame isn't `map`, or (for scans) the topic's Reliability isn't *Best Effort*. `/map` only exists while the script runs. |
| Drones' scans don't line up (walls appear twice) | `--spacing` doesn't match the launch `y_spacing`. |
| A drone stays `idle` while orange frontiers remain | Those frontiers aren't reachable with 0.6 m wall clearance (narrow gaps), or they're blocked by the other drone. Exploration ends 6 s after no drone has a reachable frontier. |

### Arguments

| Option | Default | Meaning |
|---|---|---|
| `--num-drones` | `3` | Drones `drone0` … `drone(N-1)`. Must match the launch `num_drones`. **Use `2` in Gazebo** (see above). One drone also works. |
| `--spacing` | `1.0` | Spawn spacing along +y (m). Must match the launch `y_spacing`. |
| `--alt` | `0.8` | Height of drone0 above its takeoff point (m). |
| `--alt-step` | `0.2` | Extra height per drone (m): 0.8, 1.0, 1.2 m by default. The script refuses heights whose LiDAR plane would reach the wall tops (`--wall-top`, 1.48 m). |
| `--range` | `5.0` | LiDAR range used for mapping (m). Shorter = the map grows more gradually (nicer to watch). |
| `--res` | `0.1` | Map resolution (m). |
| `--bounds` | `-6 16 -6 18` | Map area `XMIN XMAX YMIN YMAX` in the world frame (m). Fits the widened maze; enlarge for bigger worlds. |
| `--lookahead` | `0.9` | How far ahead along its path a drone aims (m). |
| `--time-limit` | `900` | Land everything after this many seconds. |
| `--save` | `explore_map` | File name (without extension) of the saved map. |

### How it works

1. **Take off** all drones to staggered heights.
2. **Map** (10 Hz): each new, level scan is ray-traced into the shared grid: cells along each beam become more "free", the cell where it hits becomes more "wall". Scans taken while a drone tilts more than ~7° are skipped (the beam would hit the floor or miss the walls), and hits within 0.6 m of another drone are ignored so drones aren't mapped as walls.
3. **Frontiers** (every 1.5 s): free cells next to unexplored cells, grouped into clusters; small clusters are ignored.
4. **Assign goals**: each drone runs a shortest-path search on the grid (keeping at least 0.6 m from walls, preferring 1 m, and treating other drones as 1.2 m obstacles) and picks the frontier with the best mix of short path and large frontier, avoiding frontiers within 3 m of another drone's goal. A drone keeps its goal until it's explored; a goal not reached in 45 s is skipped.
5. **Fly**: each drone aims at a point up to 0.9 m ahead on its path that it can reach in a straight line without cutting a corner or approaching another drone. drone0 has the highest priority: a lower-priority drone holds if its next move would bring it within 1.8 m of a higher-priority one.
6. **Finish** when no reachable frontier is left for 6 s: save the map, land all drones.

### Safety

Same as the other missions: a pilot override on any drone stops the mission and lands the rest; Ctrl+C, errors and the time limit land every drone. In addition, two drones closer than 0.8 m horizontally abort the mission. The map is saved even when the mission stops early.

### Offline test results

Tested with simulated drones (1 m/s, realistic acceleration and tilt) and a simulated LiDAR in the widened maze: 3 drones mapped 99% of the maze's free area in about 80 s of flight, staying at least 0.77 m from walls and never closer than their 1 m spawn spacing to each other; 2 and 1 drones also mapped 99%. Real Gazebo runs are slower (ArduCopter's own speed limits, settling, and the PC's load). Use 2 drones in Gazebo, as described above.

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