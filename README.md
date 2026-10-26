# Autonomous Drone Swarm

**ROS 2 Humble + Gazebo Harmonic + ArduPilot SITL + MAVROS**

Simulate one or more quadcopters in Gazebo, fly them with a simple Python API, and build a live 2D map from a LiDAR. Mission scripts run with plain `python3`, with no ROS boilerplate.

---



## 3. Repository structure

```text
swarm_ws/                        # this repository
├── mission/                     # standalone scripts (ignored by colcon)
│   ├── COLCON_IGNORE
│   └── teleop.py
└── swarm/                       # ROS 2 package (built once with --symlink-install)
    ├── config/swarm.yaml        # default parameters (y_spacing)
    ├── launch/swarm.launch.py   # Gazebo + SITL + MAVROS + LiDAR bridge
    ├── models/                  # iris_with_gimbal_custom (xacro), maze, ...
    ├── worlds/                  # artemis_world.sdf, world_with_maze.sdf
    ├── swarm/
    │   ├── drone_api.py         # Drone + Swarm API and 2D occupancy mapper
    │   └── drone.py
    ├── package.xml
    └── setup.py
```

`models/` and `worlds/` are plain data files. The compiled plugin is **not** in this repository. It is built separately (section 4).

Because the package is built with `--symlink-install`, you can edit files in `swarm/` and write new scripts in `mission/` and run them with `python3 <script>.py` without rebuilding.

---

## 4. First-time setup

All commands assume the workspace is `~/ardupilot_ws` and you are using the same user for every step (the examples use `root` inside the Docker container).

### 4.1 Install dependencies

```bash
apt update
apt install -y git cmake build-essential \
  libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
pip install numpy pyyaml "setuptools==58.2.0"
```

### 4.2 Build the ArduPilot Gazebo plugin

This step creates `libArduPilotPlugin.so` (plus the camera plugins). If it is missing, Gazebo prints `Failed to load system plugin [ArduPilotPlugin] : Could not find shared library`, and the drone arms but never lifts off.

```bash
cd ~/ardupilot_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
export GZ_VERSION=harmonic
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
ls *.so      # expect libArduPilotPlugin.so (and Gst/CameraZoom/Parachute plugins)
```

> The plugin must be **rebuilt** after upgrading Gazebo or its libraries. Keep the folder outside the cleanup commands you use on `swarm_ws`.

### 4.3 Clone this repository

```bash
cd ~/ardupilot_ws/src
git clone <YOUR_GITHUB_REPO_URL> swarm_ws
touch swarm_ws/mission/COLCON_IGNORE
```

If an older copy exists (`swarm`, `mission`, or `swarm_ws`) and you are **certain** it holds nothing you need, remove it first. Never run `rm -rf` on `ardupilot_gazebo` or `ardupilot`.

### 4.4 Set environment variables

Add to `~/.bashrc`, then `source ~/.bashrc`:

```bash
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_ws/src/swarm_ws/swarm/models:$HOME/ardupilot_ws/src/swarm_ws/swarm/worlds:$HOME/ardupilot_ws/src/ardupilot_gazebo/models:$HOME/ardupilot_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

If you moved from an older layout, fix stale paths with:

```bash
sed -i 's|/root/ardupilot_ws/src/swarm|/root/ardupilot_ws/src/swarm_ws/swarm|g' ~/.bashrc
```

On a laptop with an NVIDIA GPU (hybrid graphics), also add:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

If you run as `root`, add `export XDG_RUNTIME_DIR=/tmp/runtime-root` too.

### 4.5 Fix the repository path in the launch file

`swarm/launch/swarm.launch.py` builds `repo_root` from a fixed path. Make sure it matches where you cloned the repo:

```python
repo_root = os.path.join(home_dir, 'ardupilot_ws', 'src', 'swarm_ws', 'swarm')
```

The two places that need this are `launch_setup()` and `generate_launch_description()`. If this path is wrong, the xacro, world, and config file will not be found. The launch file also sets `GZ_SIM_RESOURCE_PATH` itself, but not the plugin path, so the plugin directory must come from your environment (section 4.4).

### 4.6 Build the ROS package once

```bash
cd ~/ardupilot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select swarm
source install/setup.bash
```

After that, `git pull` is enough to pick up changes (no rebuild), unless you add new files that need installing, such as new entries in `setup.py`.

---

## 5. Launching the simulation

```bash
cd ~/ardupilot_ws
source install/setup.bash
ros2 launch swarm swarm.launch.py
```

| Argument | Default | Description |
| --- | --- | --- |
| `world` | `normal` | `normal` (`artemis_world.sdf`), `maze` (`world_with_maze.sdf`), or any `.sdf` in `worlds/` |
| `num_drones` | `1` | Number of drones, spawned in a line along Y |
| `y_spacing` | from `swarm.yaml` (2.0) | Metres between drones along Y |

```bash
ros2 launch swarm swarm.launch.py world:=maze
ros2 launch swarm swarm.launch.py world:=maze num_drones:=2 y_spacing:=3.0
```

Drone *i* uses SITL instance `-I i`, MAVLink UDP port `14550 + 10i`, and physics port `9002 + 10i`.

**Wait for this line before running a mission script:**

```
[drone0.mavros.mavros]: CON: Got HEARTBEAT, connected. FCU: ArduPilot
```

The drone also needs a few more seconds for the EKF and GPS (`EKF3 IMU0 is using GPS`) before it can arm. The API waits for this automatically.

---

## 6. Running a mission script

In a **second terminal** (with ROS and the workspace sourced):

```bash
cd ~/ardupilot_ws/src/swarm_ws/mission
python3 teleop.py
```

Minimal example:

```python
from swarm.drone_api import Swarm

swarm = Swarm(["drone0"])
d = swarm["drone0"]
try:
    d.connect()
    d.takeoff(alt=1.5)
    d.move_to(3.0, 0.0, 1.5, safe_dist=1.0)
    print(d.get_pos(), d.min_dist(-30, 30))
    d.land()
finally:
    swarm.close()
```

### API at a glance

| Goal | Call |
| --- | --- |
| Connect | `d.connect()` |
| Takeoff / land | `d.takeoff(alt)`, `d.land()` |
| Fly to a point | `d.move_to(x, y, z)` (blocking), `d.goto(x, y, z)` (non-blocking, call at \~10 Hz) |
| Emergency stop | `d.stop()` (BRAKE mode) |
| Position / attitude | `d.get_pos()`, `d.get_rpy(deg=True)` |
| IMU / GPS / state | `d.get_imu()`, `d.gps`, `d.state` |
| LiDAR | `d.get_scan()`, `d.min_dist(min_deg, max_deg)`, `d.get_obstacles()` |
| Occupancy map | `d.get_map()` (`-1` unknown, `0` free, `100` wall) |
| Many drones | `Swarm(["drone0", "drone1"])`, `takeoff_all()`, `land_all()` |

See the full **Drone API Guide** for every method and recipe.

### Teleop keys (`mission/teleop.py`)

| Key | Action |
| --- | --- |
| `W` / `S` | Forward / backward (body frame) |
| `A` / `D` | Strafe left / right |
| `Q` / `E` | Yaw left / right |
| `R` / `F` | Altitude up / down (0.5 m to 1.8 m) |
| `Space` | Hold current position |
| `L` / `Esc` | Land and exit |

---

## 7. Viewing the map in RViz2

The API publishes `nav_msgs/OccupancyGrid` on `/drone0/map` (frame `map`, 2 Hz, 60 m × 60 m at 0.2 m/cell, centred on the start point). The map only updates while the drone is above 0.3 m and tilted less than 12°.

```bash
rviz2
```

1. Fixed Frame: `map`
2. Add → By topic → `/drone0/map` → **Map**
3. In the Map display, set Topic → Reliability **Reliable**, Durability **Volatile**
4. Set the view type to **TopDownOrthographic** if you cannot see the grid

To show the LiDAR scan, RViz needs transforms. The scan frame is `drone0/lidar_link`. Publish `map -> base_link` from the pose topic and a static transform `base_link -> drone0/lidar_link`:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link drone0/lidar_link
```

If RViz fails with a GLSL shader error, run it on the NVIDIA GPU (the environment variables in section 4.4) or use `LIBGL_ALWAYS_SOFTWARE=1 rviz2`.

---

## 8. Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `Failed to load system plugin [ArduPilotPlugin] : Could not find shared library` | Plugin not built, or `GZ_SIM_SYSTEM_PLUGIN_PATH` is wrong. Rebuild (section 4.2) and check `echo $GZ_SIM_SYSTEM_PLUGIN_PATH`. |
| MAVROS: `VER: your FCU don't support AUTOPILOT_VERSION` | SITL is not talking to Gazebo (usually the plugin problem above). |
| Drone arms, then disarms 10 s later, never lifts | Motor output is not reaching Gazebo: plugin missing, duplicate/stale processes, or the sim running far below real time. |
| `ArduPilot controller has reset`, `Time jump detected` | Stale `arducopter` or `gz` processes, or a restarted SITL. Kill everything (section 9) and relaunch. |
| `Arm: Accels/Gyros inconsistent`, `Need Position Estimate` | EKF still starting. Wait. If it persists, the sim is too slow. Use the NVIDIA GPU or run Gazebo headless. |
| Script stuck at `Waiting for pre-arm checks` | Same as above. Read the `FCU:` lines in the launch terminal. |
| Position stays `(0, 0, 0)` | No pose messages. Check `ros2 topic hz /drone0/mavros/local_position/pose`. |
| Map is blank in RViz | Reliability/Durability settings (section 7), drone on the ground, or no scans (`ros2 topic hz /drone0/scan`). |
| Launch cannot find the xacro or world | `repo_root` in `swarm.launch.py` is wrong (section 4.5). |
| Gazebo warnings about `material`, `gz_frame_id`, `mesh ... collision`, `amdgpu DRM` | Harmless. |

---

## 9. Cleaning up stuck processes

Run this before every relaunch if the last run was closed abruptly (Ctrl+C on `ros2 launch` does not always kill everything):

```bash
pkill -9 -f arducopter; pkill -9 -f sim_vehicle; pkill -9 -f mavros_node
pkill -9 -f "gz sim"; pkill -9 ruby; pkill -9 -f parameter_bridge
sleep 2
pgrep -a "arducopter|gz|mavros"      # should print nothing
```

SITL writes `eeprom.bin` into the directory it was started from. Always launch from `~/ardupilot_ws`. Delete `eeprom.bin` to reset stored parameters.

---

## 10. Updating

```bash
cd ~/ardupilot_ws/src/swarm_ws
git pull
```

Rebuild with `colcon build --symlink-install --packages-select swarm` only if `setup.py`, `package.xml`, or the list of installed files changed. Rebuild the Gazebo plugin (section 4.2) after a Gazebo upgrade.

---

## 11. Notes for contributors

- Put new scripts in `mission/` (no `colcon build` needed).
- Keep the drone below the maze wall height (about 1.8 m) so the 2D LiDAR still sees the walls.
- Record the `ardupilot_gazebo` commit you built against, so everyone uses the same one.
- Do not commit compiled `.so` files or `build/`, `install/`, `log/` folders.
